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

//system
#include <atomic>
#include <unordered_set>

//local
#include "qVoxFallDialog.h"
#include "qVoxFallTools.h"

//CCCoreLib
#include <CloudSamplingTools.h>
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
static const char OCCUPANCY_SF_NAME[] = "Occupancy";
static const char CLUSTER_SF_NAME[] = "Cluster ID";
static const char VOLUME_SF_NAME[] = "Volume (m3)";
static const char UNCERTAINTY_SF_NAME[] = "Uncertainty (%)";


// Structure for parallel call
struct VoxFallParams
{
	//main options
	float voxelSize = 0;
	int clusterLabel = 0;
	int currentLabel;
	int changeType;
	bool genarateReport = false;
	bool exportBlocksAsMeshes = false;
	CCVector3 minBound, maxBound, extent, steps;

	//helpers
	std::vector<std::vector<int>> nbs;
	std::vector<bool> isEmpty;
	std::vector<bool> isEmptyBefore;
	std::vector<bool> nonEmptyVoxelsVisited;
	std::vector<int> clusters;
	int emptyVoxelCount = 0;
	CCVector3 centroid;
	CCVector3 bbDims;
	std::vector<float> volumes;
	std::vector<unsigned int> clusterIndices;
	int clusterOutterVoxelCount;

	//export
	ccPointCloud* voxfall = nullptr;
	QString groupName;

	//scalar fields
	ccScalarField* clusterSF = nullptr;			//cluster ID
	ccScalarField* changeTypeSF = nullptr;		//loss or gain
	ccScalarField* volumeSF = nullptr;			//block volume
	ccScalarField* uncertaintySF = nullptr;		//volume uncertainty

	//progress notification
	CCCoreLib::NormalizedProgress* nProgress = nullptr;
	bool processCanceled = false;
	bool processFailed = false;
};
static VoxFallParams s_VoxFallParams;


bool InitializeOutputCloud(int voxelCount, GenericProgressCallback* progressCb = nullptr)
{
	//progress notification
	NormalizedProgress nProgress(progressCb, voxelCount);
	if (progressCb)
	{
		if (progressCb->textCanBeEdited())
		{
			progressCb->setInfo("Initialization");
			progressCb->setMethodTitle("VoxFall Detection");
		}
		progressCb->update(0);
		progressCb->start();
	}

	float voxelSize = s_VoxFallParams.voxelSize;
	CCVector3 minBound = s_VoxFallParams.minBound;

	for (int index = 0; index < voxelCount; ++index)
	{
		Tuple3i V = qVoxFallTools::Index2Grid(index, s_VoxFallParams.steps);
		CCVector3 P(static_cast<PointCoordinateType>(V.x * voxelSize + minBound.x),
					static_cast<PointCoordinateType>(V.y * voxelSize + minBound.y),
					static_cast<PointCoordinateType>(V.z * voxelSize + minBound.z));
		s_VoxFallParams.voxfall->addPoint(P);

		//progress bar
		if (progressCb && !nProgress.oneStep())
		{
			return false;
		}
	}
		
	return true;
}


void GetVoxelOccupancy(const Tuple3i& cellPos, unsigned n)
{
	int index = qVoxFallTools::Grid2Index(cellPos, s_VoxFallParams.steps);
	s_VoxFallParams.isEmpty[index] = false;
}


void GetVoxelOccupancyBefore(const Tuple3i& cellPos, unsigned n)
{
	int index = qVoxFallTools::Grid2Index(cellPos, s_VoxFallParams.steps);
	s_VoxFallParams.isEmptyBefore[index] = false;
}


bool ClusterEmptySpace(int maxThreads, int voxelCount, GenericProgressCallback* progressCb = nullptr)
{
	//progress notification
	NormalizedProgress nProgress(progressCb, voxelCount);
	if (progressCb)
	{
		if (progressCb->textCanBeEdited())
		{
			char buffer[64];
			snprintf(buffer, 64, "Clustering empty space \n Voxels: %u", voxelCount);
			progressCb->setInfo(buffer);
			progressCb->setMethodTitle("VoxFall Detection");
		}
		progressCb->update(0);
		progressCb->start();
	}

	auto steps = s_VoxFallParams.steps;
	s_VoxFallParams.nbs.resize(voxelCount);
#if defined(_OPENMP)
#pragma omp parallel for schedule(static) \
        num_threads(maxThreads)
#endif
	for (int index = 0; index < voxelCount; ++index) {
		auto V = qVoxFallTools::Index2Grid(index, steps);
		auto NN = qVoxFallTools::FindAdjacents(V, steps, false);
		for (auto const& n : NN)
		{
			int nIdx = qVoxFallTools::Grid2Index(n, steps);
			s_VoxFallParams.nbs[index].push_back(nIdx);
		}
#if defined(_OPENMP)
#pragma omp critical(ClusterEmptySpace)
		{ nProgress.oneStep(); }
#endif
	}
	for (int index = 0; index < voxelCount; ++index)
	{
		// Check if voxel is empty.
		if (!s_VoxFallParams.isEmpty[index])
			continue;

		// Label is not undefined.
		if (s_VoxFallParams.clusterSF->getValue(index) != -1)
		{
			continue;
		}

		// Check density.
		int nCount = 0;
		for (auto const& n : s_VoxFallParams.nbs[index])
		{
			if (s_VoxFallParams.isEmpty[n])
			{
				nCount++;
			}
		}

		std::unordered_set<unsigned int> nbs_next(s_VoxFallParams.nbs[index].begin(), s_VoxFallParams.nbs[index].end());
		std::unordered_set<unsigned int> visited;
		visited.insert(index);

		s_VoxFallParams.clusterSF->setValue(index, static_cast<ScalarType>(s_VoxFallParams.clusterLabel));
		if (s_VoxFallParams.clusterLabel > 0)	// keep track of the total voxels included in volumes
		{
			s_VoxFallParams.emptyVoxelCount++;
		}
		if (progressCb && !nProgress.oneStep())		//progress bar
		{
			return false;
		}
		while (!nbs_next.empty())
		{
			unsigned nb = *nbs_next.begin();
			nbs_next.erase(nbs_next.begin());
			// Check empty neighbor.
			if (!s_VoxFallParams.isEmpty[nb])
			{
				continue;
			}
			visited.insert(nb);

			// Not undefined label.
			if (s_VoxFallParams.clusterSF->getValue(nb) != -1)
			{
				continue;
			}
			s_VoxFallParams.clusterSF->setValue(nb, static_cast<ScalarType>(s_VoxFallParams.clusterLabel));
			if (s_VoxFallParams.clusterLabel > 0)	// keep track of the total voxels included in volumes
			{
				s_VoxFallParams.emptyVoxelCount++;
			}
			if (progressCb && !nProgress.oneStep())		//progress bar
			{
				return false;
			}

			// Get neighbor's density.
			int nCount = 0;
			for (auto const& n : s_VoxFallParams.nbs[nb])
			{
				if (s_VoxFallParams.isEmpty[n])
				{
					nCount++;
				}
			}
			if (nCount >= 1)
			{
				for (int qnb : s_VoxFallParams.nbs[nb])
				{
					if (s_VoxFallParams.isEmpty[qnb])
					{
						if (visited.count(qnb) == 0)
						{
							nbs_next.insert(qnb);
						}
					}
				}
			}
		}
		s_VoxFallParams.clusterLabel++;
	}
	return true;
}


bool ComputeClusterVolume(int maxThreads, int clusterCount, ccHObject* clusterGroup = nullptr)
{

	std::atomic<bool> error(false);
	CCVector3 minBound = s_VoxFallParams.maxBound;
	CCVector3 maxBound = s_VoxFallParams.minBound;
	int count = 0;

	if (s_VoxFallParams.processCanceled)
		return error;

#if defined(_OPENMP)
#pragma omp parallel for schedule(static) \
        num_threads(maxThreads)
#endif
	for (int i = 0; i < clusterCount; i++)
	{
		int index = s_VoxFallParams.clusterIndices[i];

		if(error) {
			continue;
		}

		std::unordered_set<unsigned int> nbs_next(s_VoxFallParams.nbs[index].begin(), s_VoxFallParams.nbs[index].end());
		while (!nbs_next.empty())
		{
			unsigned nb = *nbs_next.begin();
			nbs_next.erase(nbs_next.begin());

			// Check non empty neighbor.
			if (s_VoxFallParams.isEmpty[nb])
			{
				continue;
			}
			if (s_VoxFallParams.nonEmptyVoxelsVisited[nb] == false)
			{
				s_VoxFallParams.clusterOutterVoxelCount++;
				s_VoxFallParams.nonEmptyVoxelsVisited[nb] = true;

			}
			if (s_VoxFallParams.exportBlocksAsMeshes)
			{
				s_VoxFallParams.clusters[nb] = s_VoxFallParams.currentLabel;
			}
		}

		//progress bar
		if (!s_VoxFallParams.nProgress->oneStep())
		{
			error = true;
		}
	}

	if (error) return !error;
	return !error;
}


bool qVoxFallProcess::Compute(const qVoxFallDialog& dlg, QString& errorMessage, bool allowDialogs, QWidget* parentWidget/*=nullptr*/, ccMainAppInterface* app/*=nullptr*/)
{
	errorMessage.clear();

	//get the input meshes in the right order
	ccMesh* mesh = dlg.getMesh();

	if (!mesh)
	{
		assert(false);
		return false;
	}

	//get parameters from dialog
	double dip = dlg.getDip();
	double azimuth = dlg.getAzimuth();

	//max thread count
	int maxThreadCount = dlg.getMaxThreadCount();

	if (app)
		app->dispToConsole(	QString("[VoxFall] Will use %1 threads").arg(maxThreadCount == 0 ? "the max number of" : QString::number(maxThreadCount)),
							ccMainAppInterface::STD_CONSOLE_MESSAGE	);

	//progress dialog
	ccProgressDialog pDlg(parentWidget);

	//Duration: initialization
	QElapsedTimer initTimer;
	initTimer.start();

	auto transform = qVoxFallTransform(dip, azimuth);
	mesh->applyGLTransformation_recursive(&transform.matrix);

	//parameters are stored in 's_VoxFallParams' for parallel call
	s_VoxFallParams = VoxFallParams();
	s_VoxFallParams.voxelSize = dlg.getVoxelSize();
	s_VoxFallParams.minBound = mesh->getOwnBB().minCorner();
	s_VoxFallParams.maxBound = mesh->getOwnBB().maxCorner();
	s_VoxFallParams.extent = s_VoxFallParams.maxBound - s_VoxFallParams.minBound;
	s_VoxFallParams.steps = (s_VoxFallParams.extent / s_VoxFallParams.voxelSize) + Vector3Tpl<float>(1, 1, 1);
	s_VoxFallParams.genarateReport = dlg.getGenerateReportActivation();
	s_VoxFallParams.exportBlocksAsMeshes = dlg.getExportMeshActivation();
	s_VoxFallParams.groupName = mesh->getName() + QString(" [VoxFall clusters] (voxel %1 m)").arg(s_VoxFallParams.voxelSize);
	s_VoxFallParams.voxfall = new ccPointCloud(s_VoxFallParams.groupName);

	//Initialize voxel grid
	auto voxelGrid = CCCoreLib::Grid3D<int>();
	if (!voxelGrid.init(	int(s_VoxFallParams.steps.x),
							int(s_VoxFallParams.steps.y),
							int(s_VoxFallParams.steps.z),
							0	))  //margin
	{
		errorMessage = "Failed to initialize voxel grid!";
		return false;
	}

	// Initialize heplpers
	s_VoxFallParams.voxfall->reserve(voxelGrid.innerCellCount());
	s_VoxFallParams.nbs.resize(voxelGrid.innerCellCount());
	s_VoxFallParams.isEmpty.resize(voxelGrid.innerCellCount(), true);
	s_VoxFallParams.isEmptyBefore.resize(voxelGrid.innerCellCount(), true);
	if (s_VoxFallParams.exportBlocksAsMeshes)
	{
		s_VoxFallParams.clusters.resize(voxelGrid.innerCellCount(), 0);
	}

	//allocate cluster ID SF
	s_VoxFallParams.clusterSF = new ccScalarField(CLUSTER_SF_NAME);
	s_VoxFallParams.clusterSF->link();
	if (!s_VoxFallParams.clusterSF->resizeSafe(voxelGrid.innerCellCount(), true, static_cast<ScalarType>(-1.0)))
	{
		errorMessage = "Failed to allocate memory for cluster ID values!";
		return false;
	}

	//allocate volume SF
	s_VoxFallParams.volumeSF = new ccScalarField(VOLUME_SF_NAME);
	s_VoxFallParams.volumeSF->link();
	if (!s_VoxFallParams.volumeSF->resizeSafe(voxelGrid.innerCellCount(), true, CCCoreLib::NAN_VALUE))
	{
		errorMessage = "Failed to allocate memory for volume values!";
		return false;
	}
	//allocate volume uncertainty SF
	s_VoxFallParams.uncertaintySF = new ccScalarField(UNCERTAINTY_SF_NAME);
	s_VoxFallParams.uncertaintySF->link();
	if (!s_VoxFallParams.uncertaintySF->resizeSafe(voxelGrid.innerCellCount(), true, CCCoreLib::NAN_VALUE))
	{
		errorMessage = "Failed to allocate memory for volume uncertainty values!";
		return false;
	}

	// Initialize output cloud
	if (!InitializeOutputCloud(voxelGrid.innerCellCount(), &pDlg))
	{
		errorMessage = "Failed to initialize output data!";
		return false;
	}

	qint64 initTime_ms = initTimer.elapsed();
	//we display init. timing only if no error occurred!
	if (app)
		app->dispToConsole(	QString("[VoxFall] Initialization: %1 s").arg(initTime_ms / 1000.0, 0, 'f', 3),
							ccMainAppInterface::STD_CONSOLE_MESSAGE	);


// 	   BLOCK DETECTION
//=======================================================================================================================

	//Duration: Detection
	QElapsedTimer detectTimer;
	detectTimer.start();

	if (!voxelGrid.intersectWith(	mesh,
									s_VoxFallParams.voxelSize,
									s_VoxFallParams.minBound,
									GetVoxelOccupancy,
									&pDlg	))
	{
		errorMessage = "Failed to compute  grid occupancy!";
		return false;
	}

	//cluster DBSCAN
	if (!ClusterEmptySpace(	maxThreadCount,
							voxelGrid.innerCellCount(),
							&pDlg	))
	{
		errorMessage = "Failed to compute grid occupancy!";
		return false;
	}

	qint64 detectTime_ms = detectTimer.elapsed();
	//we display block extraction timing only if no error occurred!
	if (app)
		app->dispToConsole(QString("[VoxFall] Block detection: %1 s").arg(detectTime_ms / 1000.0, 0, 'f', 3),
			ccMainAppInterface::STD_CONSOLE_MESSAGE);
		app->dispToConsole(	QString("[VoxFall] Blocks found: %1").arg(s_VoxFallParams.clusterLabel - 1),
							ccMainAppInterface::STD_CONSOLE_MESSAGE	);

// 	   COMPUTE VOLUMES
//=======================================================================================================================

	//Duration: volume computation
	QElapsedTimer volumeTimer;
	volumeTimer.start();

	//progress notification
	pDlg.reset();
	NormalizedProgress nProgress(&pDlg, s_VoxFallParams.emptyVoxelCount);
	char buffer[64];
	snprintf(buffer, 64, "VoxFall clusters: %u \n Empty voxels: %u", s_VoxFallParams.clusterLabel - 1, s_VoxFallParams.emptyVoxelCount);
	pDlg.setInfo(buffer);
	pDlg.setMethodTitle(QObject::tr("Compute Volumes"));
	pDlg.update(0);
	pDlg.start();
	s_VoxFallParams.nProgress = &nProgress;


	s_VoxFallParams.volumes.resize(s_VoxFallParams.clusterLabel);
	s_VoxFallParams.nonEmptyVoxelsVisited.resize(voxelGrid.innerCellCount(), false);
	for (int label = 1; label < s_VoxFallParams.clusterLabel; ++label)
	{
		for (unsigned i = 0; i < static_cast<unsigned>(s_VoxFallParams.clusterSF->size()); ++i)
		{
			if (s_VoxFallParams.clusterSF->getValue(i) == static_cast<ScalarType>(label))
			{
				s_VoxFallParams.clusterIndices.push_back(i);
			}
		}

		s_VoxFallParams.currentLabel = label;
		s_VoxFallParams.clusterOutterVoxelCount = 0;

		if (!ComputeClusterVolume(	maxThreadCount, static_cast<int>(s_VoxFallParams.clusterIndices.size()) ))
		{
			errorMessage = "Failed to compute cluster volume!";
			return false;
		}

		ScalarType uncertainty = static_cast<ScalarType>(pow(s_VoxFallParams.voxelSize, 3) * s_VoxFallParams.clusterOutterVoxelCount / 2);
		ScalarType volume = static_cast<ScalarType>(pow(s_VoxFallParams.voxelSize, 3) * s_VoxFallParams.clusterIndices.size() + uncertainty);
		s_VoxFallParams.volumes[label - 1] = volume;

		for (unsigned i = 0; i < s_VoxFallParams.clusterIndices.size(); i++)
		{
			s_VoxFallParams.volumeSF->setValue(s_VoxFallParams.clusterIndices[i], volume);
			s_VoxFallParams.uncertaintySF->setValue(s_VoxFallParams.clusterIndices[i], volume/uncertainty/100);
		}
		s_VoxFallParams.clusterIndices.clear();
	}

	qint64 volumeTime_ms = volumeTimer.elapsed();
	//we display block volume computation timing only if no error occurred!
	if (app)
		app->dispToConsole(QString("[VoxFall] Volume computation: %1 s").arg(volumeTime_ms / 1000.0, 0, 'f', 3),
			ccMainAppInterface::STD_CONSOLE_MESSAGE);


// 	   EXPORT BLOCKS AS VOXEL MESH MODELS (IF SELECTED)
//=======================================================================================================================

	if (s_VoxFallParams.exportBlocksAsMeshes)
	{
		//Duration: block meshing
		QElapsedTimer meshTimer;
		meshTimer.start();

		//progress notification
		pDlg.reset();
		NormalizedProgress nProgress(&pDlg, s_VoxFallParams.emptyVoxelCount);
		char buffer[64];
		snprintf(buffer, 64, "Blocks: %u", s_VoxFallParams.clusterLabel - 1);
		pDlg.setInfo(buffer);
		pDlg.setMethodTitle(QObject::tr("Exporting blocks as meshes"));
		pDlg.update(0);
		pDlg.start();

		//we create a new group to store all output meshes as 'VoxFall clusters'
		ccHObject* ccGroup = new ccHObject(s_VoxFallParams.groupName);

		//we pair volumes with the labels vector and sort them by volume
		std::vector<std::pair<float, int>> pairVolumeLabel(s_VoxFallParams.volumes.size());
		for (int i = 0; i < s_VoxFallParams.volumes.size(); i++)
		{
			pairVolumeLabel[i] = { s_VoxFallParams.volumes[i], i + 1};
		}
		std::sort(pairVolumeLabel.begin(), pairVolumeLabel.end(), [](const std::pair<float, int>& a, const std::pair<float, int>& b) {
			return a.first > b.first;  // Compare by the first element (int) in descending order
		});

		for (int k = 1; k < s_VoxFallParams.clusterLabel; k++)
		{
			auto volume = pairVolumeLabel[k - 1].first;
			auto label = pairVolumeLabel[k - 1].second;

			std::vector<unsigned int> indices;
			auto it = std::find(s_VoxFallParams.clusters.begin(), s_VoxFallParams.clusters.end(), label);
			while (it != s_VoxFallParams.clusters.end())
			{
				indices.push_back(it - s_VoxFallParams.clusters.begin());
				it = std::find(it + 1, s_VoxFallParams.clusters.end(), label);
			}

			// we initiate the cluster cloud and mesh to add vertices and triangles of each voxel
			ccPointCloud* clusterCloud = new ccPointCloud("Vertices");
			ccMesh* clusterMesh = new ccMesh(clusterCloud);
			
			for (int i = 0; i < indices.size(); i++)
			{
				// we create the voxel box mesh
				CCVector3 V;
				s_VoxFallParams.voxfall->getPoint(indices[i], V);
				auto voxel = qVoxFallTransform::CreateVoxelMesh(V, s_VoxFallParams.voxelSize, indices[i]);
				ccPointCloud* voxelCloud = dynamic_cast<ccPointCloud*>(voxel->getAssociatedCloud());
				voxelCloud->applyGLTransformation_recursive(&transform.inverse);

				// we append voxel vertices in the cluster cloud;
				unsigned vertCount = clusterCloud->size();
				clusterCloud->append(voxelCloud, clusterCloud->size());

				// we add triangles from the voxel mesh to the cluster mesh
				for (unsigned i = 0; i < voxel->size(); ++i)
				{
					auto tri = voxel->getTriangleVertIndexes(i);
					clusterMesh->addTriangle(tri->i1+vertCount, tri->i2+vertCount, tri->i3+vertCount);
				}

				//progress bar
				if (!nProgress.oneStep())
				{
					return false;
				}
			}
			clusterMesh->setName(QString("Cluster#%1 - (v: %2 m3)").arg(label).arg(volume));
			clusterMesh->computePerVertexNormals();
			clusterCloud->resize(clusterCloud->size());
			clusterMesh->addChild(clusterCloud);
			ccGroup->addChild(clusterMesh);
			indices.clear();
		}
		ccGroup->setVisible(true);
		app->addToDB(ccGroup);

		qint64 meshTime_ms = meshTimer.elapsed();
		//we display block as mesh export timing only if no error occurred!
		if (app)
			app->dispToConsole(QString("[VoxFall] Block as mesh export: %1 s").arg(meshTime_ms / 1000.0, 0, 'f', 3),
				ccMainAppInterface::STD_CONSOLE_MESSAGE);
	}


// 	   OUTPUT FORMATION
//=======================================================================================================================
		
	//associate cluster ID scalar fields to the voxel grid
	int sfIdx = -1;
	if (s_VoxFallParams.clusterSF)
	{
		//add cluster ID SF to voxel grid
		s_VoxFallParams.clusterSF->computeMinAndMax();
		sfIdx = s_VoxFallParams.voxfall->addScalarField(s_VoxFallParams.clusterSF);
	}
	//associate volume scalar field to the voxel grid
	if (s_VoxFallParams.volumeSF)
	{
		//add volume SF to voxel grid
		s_VoxFallParams.volumeSF->computeMinAndMax();
		sfIdx = s_VoxFallParams.voxfall->addScalarField(s_VoxFallParams.volumeSF);
	}
	//associate volume uncertainty scalar field to the voxel grid
	if (s_VoxFallParams.uncertaintySF)
	{
		//add volume uncertainty SF to voxel grid
		s_VoxFallParams.uncertaintySF->computeMinAndMax();
		sfIdx = s_VoxFallParams.voxfall->addScalarField(s_VoxFallParams.uncertaintySF);
	}

	//prepare export cloud
	mesh->applyGLTransformation_recursive(&transform.inverse);
	s_VoxFallParams.voxfall->applyGLTransformation_recursive(&transform.inverse);
	sfIdx = s_VoxFallParams.voxfall->getScalarFieldIndexByName(CLUSTER_SF_NAME);
	s_VoxFallParams.voxfall->setCurrentDisplayedScalarField(sfIdx);;
	s_VoxFallParams.voxfall->showSF(true);
	if (s_VoxFallParams.exportBlocksAsMeshes)
	{
		s_VoxFallParams.voxfall->setEnabled(false);
	}
	app->addToDB(s_VoxFallParams.voxfall);
	
	//if "generate report" is selected, open CSV file
	if (s_VoxFallParams.genarateReport)
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
		outStream << s_VoxFallParams.groupName << " \n\n";
		outStream << "Cluster ID,";
		outStream << " Center X,";
		outStream << " Center Y,";
		outStream << " Center Z,";
		outStream << " Extent X,";
		outStream << " Extent Y,";
		outStream << " Extent Z,";
		outStream << " Volume (m3),";
		outStream << " Uncertainty (m3),";
		outStream << " \n";

		//add info line for each cluster
		for (int label = 1; label < s_VoxFallParams.clusterLabel; ++label)
		{
			//get data from cluster cloud
			ccPointCloud* cluster = s_VoxFallParams.voxfall->filterPointsByScalarValue(static_cast<ScalarType>(label), static_cast<ScalarType>(label));
			ccBBox bb = cluster->getOwnBB();
			CCVector3 centroid = bb.getCenter();
			CCVector3d extent = bb.maxCorner() - bb.minCorner();
			sfIdx = cluster->getScalarFieldIndexByName(VOLUME_SF_NAME);
			cluster->setCurrentDisplayedScalarField(sfIdx);;
			auto volume = cluster->getPointScalarValue(static_cast<unsigned int>(0));
			sfIdx = cluster->getScalarFieldIndexByName(UNCERTAINTY_SF_NAME);
			cluster->setCurrentDisplayedScalarField(sfIdx);;
			auto uncertainty = cluster->getPointScalarValue(static_cast<unsigned int>(0));
			
			//add data to file
			outStream << label << ","; //cluster ID
			outStream << centroid.x << "," << centroid.y << "," << centroid.z << ","; //center XYZ
			if (extent.x > 0) { outStream << extent.x << ","; } else { outStream << s_VoxFallParams.voxelSize << ","; }; //extent X
			if (extent.y > 0) { outStream << extent.y << ","; } else { outStream << s_VoxFallParams.voxelSize << ","; }; //extent Y
			if (extent.z > 0) { outStream << extent.z << ","; } else { outStream << s_VoxFallParams.voxelSize << ","; }; //extent Z
			outStream << volume << ","; //volume
			outStream << uncertainty << ","; //uncertainty
			outStream << " \n";
		}

		outFile.close();
		if (app)
			app->dispToConsole(QString("[VoxFall] Report generated at: " + dlg.destinationPathLineEdit->text()),
				ccMainAppInterface::STD_CONSOLE_MESSAGE);
	}
	
	if (app)
		app->refreshAll();

	return true;
}
