//##########################################################################
//#                                                                        #
//#                     CLOUDCOMPARE PLUGIN: qVoxFall                      #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 3 of the License.               #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#                 COPYRIGHT: THE UNIVERSITY OF NEWCASTLE                 #
//#                                                                        #
//##########################################################################

#include "qVoxFallProcess.h"
#include "qVoxFallCore.h"

//system
#include <atomic>
#include <unordered_set>

//local
#include "qVoxFallDialog.h"
#include "qVoxFallTools.h"

//CCCoreLib
#include "Grid3D.h"

//qCC_plugins
#include <ccMainAppInterface.h>
#include <ccQtHelpers.h>

//qCC_db
#include <ccMesh.h>
#include <ccPointCloud.h>
#include <ccGenericPointCloud.h>
#include <ccOctree.h>
#include <ccOctreeProxy.h>
#include <ccHObjectCaster.h>
#include <ccProgressDialog.h>
#include <ccScalarField.h>

//Qt
#include <QtGui>
#include <QtCore>
#include <QApplication>
#include <QElapsedTimer>
#include <QtConcurrentMap>
#include <QMessageBox>

#if defined(_OPENMP)
//OpenMP
#include <omp.h>
#endif
using namespace CCCoreLib;


//! Default name for VoxFall scalar fields
static const char CHANGE_TYPE_SF_NAME[] = "Loss/gain";
static const char VOLUME_SF_NAME[] = "Volume (m3)";
static const char UNCERTAINTY_SF_NAME[] = "Uncertainty (%)";
static const char CONNECTED_COMPONENTS_SF_NAME[] = "CC labels";



bool qVoxFallProcess::Compute(const qVoxFallDialog& dlg, QString& errorMessage, ccPointCloud*& outputCloud, ccHObject*& outputGroup, bool allowDialogs, QWidget* parentWidget/*=nullptr*/, ccMainAppInterface* app/*=nullptr*/)
{
	errorMessage.clear();
	ccPointCloud* patchCloud = nullptr;
	ccPointCloud* appCloud = nullptr;
	ccHObject* appGroup = nullptr;

	//get the input meshes in the right order
	ccMesh* mesh1 = dlg.getMesh1();
	ccMesh* mesh2 = dlg.getMesh2();

	if (!mesh1 || !mesh2)
	{
		assert(false);
		return false;
	}

	//get parameters from dialog
	double dip = dlg.getDip();
	double dipdir = dlg.getDipDir();
	double voxelSize = dlg.getVoxelSize();
	bool generateReport = dlg.getGenerateReportActivation();
	bool exportBlockMeshes = dlg.getExportMeshesActivation();

	//max thread count
	int maxThreadCount = dlg.getMaxThreadCount();

	if (app)
		app->dispToConsole(	QString("[VoxFall] Will use %1 threads").arg(maxThreadCount == 0 ? "the max number of" : QString::number(maxThreadCount)),
							ccMainAppInterface::STD_CONSOLE_MESSAGE	);

	//progress dialog
	ccProgressDialog pDlg(parentWidget);


	// 	   PREPROCESSING
	//=======================================================================================================================
	QElapsedTimer initTimer;
	initTimer.start();

	// Compute initial signed distances
	int bestOctreeLevel = 0;
	if (!qVoxFallTools::ComputeDistances(*mesh1, *mesh2, &pDlg, maxThreadCount, bestOctreeLevel))
	{
		errorMessage = "Failed to compute initial distances!";
		return false;
	}

	//create local patches around displaced areas for processing
	ccPointCloud* cloud = static_cast<ccPointCloud*>(mesh1->getAssociatedCloud());
	auto distancesSF = cloud->getCurrentInScalarField();
	ccPointCloud* lossCloud = cloud->filterPointsByScalarValue(static_cast<ScalarType>(voxelSize), distancesSF->getMax());
	ccPointCloud* gainCloud = cloud->filterPointsByScalarValue(distancesSF->getMin(), static_cast<ScalarType>(-voxelSize));

	//compute connected components on the loss and gain clouds
	if (!qVoxFallTools::ConnectedComponents(lossCloud, bestOctreeLevel - 2, &pDlg, errorMessage))
	{
		return false;
	}
	if (!qVoxFallTools::ConnectedComponents(gainCloud, bestOctreeLevel - 2, &pDlg, errorMessage))
	{
		return false;
	}

	qint64 initTime_ms = initTimer.elapsed();
	//we display block as mesh export timing only if no error occurred!
	if (app)
	{
		app->dispToConsole(QString("[VoxFall] Preprocessing: %1 s").arg(initTime_ms / 1000.0, 0, 'f', 3),
			ccMainAppInterface::STD_CONSOLE_MESSAGE);
	}

	// 	   VOXFALL COMPUTATION
	//=======================================================================================================================
	// Here starts the real VoxFall process
	QElapsedTimer processTimer;
	processTimer.start();

	auto mesh = mesh1->cloneMesh();
	mesh->merge(mesh2, false);
	mesh1->setEnabled(false);

	auto transform = qVoxFallTransform(dip, dipdir);
	mesh->applyGLTransformation_recursive(&transform.matrix);

	//we create output objects to store 'VoxFall clusters'
	QString sessionName = mesh1->getName() + "_to_" + mesh2->getName() + QString(" [VoxFall] (voxel %1m)").arg(voxelSize);
	appCloud = new ccPointCloud(sessionName);
	appGroup = new ccHObject(sessionName);

	//crop patches from mesh1 and mesh1 for each clusters bbox and pass iteratively to the core process
	int sfIdx = lossCloud->getScalarFieldIndexByName(CONNECTED_COMPONENTS_SF_NAME);
	auto ccSF = lossCloud->getScalarField(sfIdx);
	int patchFirstLabel = 0;
	patchCloud = new ccPointCloud;
	for (int cc = 1; cc < ccSF->getMax() + 1; ++cc)
	//for (int cc = 2; cc < 4; ++cc)
	{
		ccPointCloud* ccCloud = lossCloud->filterPointsByScalarValue(static_cast<ScalarType>(cc), static_cast<ScalarType>(cc));
		ccCloud->applyGLTransformation_recursive(&transform.matrix);
		ccBBox bb = ccCloud->getOwnBB();
		bb = qVoxFallTools::ScaleBBox(bb, 1.2);

		if (!qVoxFallCore::Run(	mesh,
								&bb,
								-1,
								transform,
								patchFirstLabel,
								maxThreadCount,
								errorMessage,
								patchCloud,
								appGroup,
								dlg,
								pDlg))
		{
			return false;
		}
		appCloud->append(patchCloud, appCloud->size());
		patchCloud->clear();
		app->dispToConsole(errorMessage);
	}
	//ccPointCloud* ccCloud = lossCloud->filterPointsByScalarValue(static_cast<ScalarType>(3), static_cast<ScalarType>(3));

	if (app)
	{
		app->addToDB(appCloud);
		if (exportBlockMeshes)
			appGroup->setVisible(true);
			app->addToDB(appGroup);
	}
	else
	{
		outputCloud = appCloud;
		if (exportBlockMeshes) outputGroup = appGroup;
	}

	qint64 processTime_ms = processTimer.elapsed();
	//we display block as mesh export timing only if no error occurred!
	if (app)
	{
		app->dispToConsole(QString("[VoxFall] VoxFall computation: %1 s").arg(processTime_ms / 1000.0, 0, 'f', 3),
			ccMainAppInterface::STD_CONSOLE_MESSAGE);
	}


	//if "generate report" is selected, open CSV file
	if (generateReport)
	{

		QString filename = dlg.destinationPathLineEdit->text();
		QFile outFile(filename);
		//open CSV file
		if (!outFile.open(QFile::WriteOnly | QFile::Text))
		{
			app->dispToConsole(QString("Failed to open file for writing! Check available space and access rights"), ccMainAppInterface::ERR_CONSOLE_MESSAGE);
			return false;
		}
		//write header
		QTextStream outStream(&outFile);
		outStream << sessionName << " \n\n";
		outStream << "Cluster ID,";
		outStream << " Center X,";
		outStream << " Center Y,";
		outStream << " Center Z,";
		outStream << " Extent X,";
		outStream << " Extent Y,";
		outStream << " Extent Z,";
		outStream << " Change type,";
		outStream << " Volume (m3),";
		outStream << " Uncertainty (m3),";
		outStream << " \n";

		//add info line for each cluster
		for (int label = 1; label < patchFirstLabel; ++label)
		{
			//get data from cluster cloud
			ccPointCloud* cluster = outputCloud->filterPointsByScalarValue(static_cast<ScalarType>(label), static_cast<ScalarType>(label));
			ccBBox bb = cluster->getOwnBB();
			CCVector3 centroid = bb.getCenter();
			CCVector3d extent = bb.maxCorner() - bb.minCorner();

			sfIdx = cluster->getScalarFieldIndexByName(VOLUME_SF_NAME);
			cluster->setCurrentDisplayedScalarField(sfIdx);;
			auto volume = cluster->getPointScalarValue(static_cast<unsigned int>(0));

			sfIdx = cluster->getScalarFieldIndexByName(UNCERTAINTY_SF_NAME);
			cluster->setCurrentDisplayedScalarField(sfIdx);;
			auto uncertainty = cluster->getPointScalarValue(static_cast<unsigned int>(0));

			auto loss_gain = "n/a";
			sfIdx = cluster->getScalarFieldIndexByName(CHANGE_TYPE_SF_NAME);
			cluster->setCurrentDisplayedScalarField(sfIdx);;
			auto changeType = cluster->getPointScalarValue(static_cast<unsigned int>(0));

			if (changeType == -1)
			{
				loss_gain = "loss";
			}
			else
			{
				loss_gain = "gain";
			}
			
			//add data to file
			outStream << label << ","; //cluster ID
			outStream << centroid.x << "," << centroid.y << "," << centroid.z << ","; //center XYZ
			if (extent.x > 0) { outStream << extent.x << ","; } else { outStream << voxelSize << ","; }; //extent X
			if (extent.y > 0) { outStream << extent.y << ","; } else { outStream << voxelSize << ","; }; //extent Y
			if (extent.z > 0) { outStream << extent.z << ","; } else { outStream << voxelSize << ","; }; //extent Z
			outStream << loss_gain << ","; //change type (loss/gain)
			outStream << volume << ","; //volume
			outStream << uncertainty << ","; //uncertainty
			outStream << " \n";
		}

		outFile.close();
		if (app)
		{
			app->dispToConsole(QString("[VoxFall] Report generated at: " + dlg.destinationPathLineEdit->text()),
				ccMainAppInterface::STD_CONSOLE_MESSAGE);
		}
	}
	
	if (app)
	{
		app->refreshAll();
	}

	return true;
}
