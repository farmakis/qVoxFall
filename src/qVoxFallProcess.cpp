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
#include "qVoxFallCluster.h"
#include "qVoxFallGraph.h"
#include "omp_num_threads.hpp"

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

using namespace CCCoreLib;


//! Default name for VoxFall scalar fields
static const char OCCUPANCY_SF_NAME[] = "Occupancy";
static const char CLUSTER_SF_NAME[] = "Cluster ID";
static const char CHANGE_TYPE_SF_NAME[] = "Loss/gain";
static const char VOLUME_SF_NAME[] = "Volume (m3)";
static const char UNCERTAINTY_SF_NAME[] = "Uncertainty (%)";


// Structure for parallel call
struct VoxFallParams
{
	//main options
	float voxelSize = 0;
	bool genarateReport = false;
	bool exportBlocksAsMeshes = false;
	bool exportLossGain = false;

	//helpers
	qVoxFallGraph* voxelGraph = nullptr;
	std::vector<qVoxFallCluster*> clusters;
	std::vector<int> relabel;
	std::vector<std::vector<int>> nbs;
	std::vector<bool> isEmpty;
	std::vector<bool> isEmptyBefore;
	std::vector<bool> nonEmptyVoxelsVisited;
	int emptyVoxelCount = 0;
	std::vector<float> volumes;
	CCVector3 minBound, maxBound, steps;

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
		Tuple3i V = s_VoxFallParams.voxelGraph->Index2Grid(index);
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
	int index = s_VoxFallParams.voxelGraph->Grid2Index(cellPos);
	s_VoxFallParams.isEmpty[index] = false;
}


void GetVoxelOccupancyBefore(const Tuple3i& cellPos, unsigned n)
{
	int index = s_VoxFallParams.voxelGraph->Grid2Index(cellPos);
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

	/* Flags grid graph edges based on node matching in the empty/non-empty space */
	s_VoxFallParams.voxelGraph->FlagEdgesNodeMatch(s_VoxFallParams.isEmpty);
	s_VoxFallParams.voxelGraph->compute_connected_components();

	auto rV = s_VoxFallParams.voxelGraph->GetNumReducedNodes(); // number of clusters (reduced vertices)
	auto labels = s_VoxFallParams.voxelGraph->GetLabels();;

	/* Need to remap the labels so the surrounding empty space cluster is always 0 and non-empty is -1
	 * Get the cluster label that correspond to the maximum corner of the grid
	 * It always belongs to the surrounding empty space cluster.
	 * The added 1-padding keeps the surrounding empty space cluster intact. */
	auto max_corner = Tuple3i(s_VoxFallParams.steps.x - 1, s_VoxFallParams.steps.y - 1, s_VoxFallParams.steps.z - 1);
	auto max_corner_index = s_VoxFallParams.voxelGraph->Grid2Index(max_corner);
	auto surr_space_cluster = labels[max_corner_index]; // surrounding empty space cluster id
	int non_empty_cluster;  // non-empty space cluster id
	for (int i = 0; i < voxelCount; ++i)
	{
		if (!s_VoxFallParams.isEmpty[i]) 
		{
			non_empty_cluster = labels[i];
			break;
		}
	}

	/* Initialize voxfall clusters */
	s_VoxFallParams.clusters.resize(rV - 2);
	s_VoxFallParams.relabel.resize(rV, 0);
	s_VoxFallParams.relabel[non_empty_cluster] = -1; // non-empty space cluster is relabeled -1
	int lbl = 1;
    for (int i = 0; i < rV; ++i)
	{
        if (i != surr_space_cluster && i != non_empty_cluster) 
		{
            auto cluster = new qVoxFallCluster(lbl, 
												i, 
												s_VoxFallParams.voxelGraph->GetFirstNode(i), 
												s_VoxFallParams.voxelGraph->GetLastNode(i),
												s_VoxFallParams.voxelSize); 
			s_VoxFallParams.clusters[lbl - 1] = cluster;
			s_VoxFallParams.relabel[i] = lbl++;
			s_VoxFallParams.emptyVoxelCount += cluster->GetSize();
        }
    }

	/* Update output scalar field with relabeled cluster indices */
	for (int i = 0; i < voxelCount; ++i)
	{
		s_VoxFallParams.clusterSF->setValue(i, static_cast<ScalarType>(s_VoxFallParams.relabel[labels[i]]));
	}
	free(labels);

	return true;
}


bool ComputeClusterVolume(int maxThreads, int clusterIndex, qVoxFallTransform* transform, ccMesh* mesh, ccHObject* clusterGroup = nullptr)
{

	std::atomic<bool> error(false);
	CCVector3 minBound = s_VoxFallParams.maxBound;
	CCVector3 maxBound = s_VoxFallParams.minBound;
	qVoxFallCluster* cluster = s_VoxFallParams.clusters[clusterIndex];

	if (s_VoxFallParams.processCanceled)
		return error;

	/* get cluster node (voxel) indices */
	#pragma omp parallel for schedule(static) NUM_THREADS(cluster->GetSize(), maxThreads)
	for (unsigned i = 0; i < cluster->GetSize(); i++)
	{
		auto voxel = s_VoxFallParams.voxelGraph->GetNodeIndex(cluster->GetVoxel(i));

		if (error) {
			continue;
		}

		if (s_VoxFallParams.exportBlocksAsMeshes)
		{
			CCVector3 V;
			s_VoxFallParams.voxfall->getPoint(voxel, V);
			   cluster->AddVoxelMesh(V, voxel, transform);
		}

		auto neighbors = s_VoxFallParams.voxelGraph->GetNeighbors(voxel);
		std::unordered_set<unsigned int> nbs_next(neighbors.begin(), neighbors.end());
		while (!nbs_next.empty())
		{
			unsigned nb = *nbs_next.begin();
			nbs_next.erase(nbs_next.begin());

			/* Check non empty neighbor. */
			if (s_VoxFallParams.isEmpty[nb]){ continue; }

			if (!s_VoxFallParams.nonEmptyVoxelsVisited[nb])
			{
				cluster->IncrementSurfaceVoxelCount();
				s_VoxFallParams.nonEmptyVoxelsVisited[nb] = true;

				if (s_VoxFallParams.exportBlocksAsMeshes)
				{
					CCVector3 V;
					s_VoxFallParams.voxfall->getPoint(nb, V);
					   cluster->AddVoxelMesh(V, nb, transform);
				}

				if (s_VoxFallParams.exportLossGain)
				{
					Tuple3i V = s_VoxFallParams.voxelGraph->Index2Grid(nb);
					CCVector3 pos(static_cast<PointCoordinateType>(V.x * s_VoxFallParams.voxelSize + s_VoxFallParams.minBound.x),
						static_cast<PointCoordinateType>(V.y * s_VoxFallParams.voxelSize + s_VoxFallParams.minBound.y),
						static_cast<PointCoordinateType>(V.z * s_VoxFallParams.voxelSize + s_VoxFallParams.minBound.z));

					if (pos.x > maxBound.x) maxBound.x = static_cast<PointCoordinateType>(pos.x);
					if (pos.y > maxBound.y) maxBound.y = static_cast<PointCoordinateType>(pos.y);
					if (pos.z > maxBound.z) maxBound.z = static_cast<PointCoordinateType>(pos.z);

					if (pos.x < minBound.x) minBound.x = static_cast<PointCoordinateType>(pos.x);
					if (pos.y < minBound.y) minBound.y = static_cast<PointCoordinateType>(pos.y);
					if (pos.z < minBound.z) minBound.z = static_cast<PointCoordinateType>(pos.z);
				}

			}
		}

		//progress bar
		if (!s_VoxFallParams.nProgress->oneStep())
		{
			error = true;
		}
	}


	""" THIS CRITERION NEEDS TO BE REVISED """;
	if (s_VoxFallParams.exportLossGain)
	{
		float ymin = minBound.y;
		float ymax = maxBound.y;
		CCVector3 extent = maxBound - minBound;
		CCVector3 center = minBound + extent / 2;
		minBound += extent / static_cast<PointCoordinateType>(2 * 0.9);
		maxBound -= extent / static_cast<PointCoordinateType>(2 * 0.9);
		maxBound.y = ymax + (ymax - ymin) / 2.0;

		auto centroid = minBound + (maxBound - minBound) / 1.5;
		auto bbDims = (maxBound - minBound) / 2;

		int count = 0;
		mesh->placeIteratorAtBeginning();
		for (unsigned n = 0; n < mesh->size(); n++)
		{
			//get the positions (in the grid) of each vertex
			const GenericTriangle* T = mesh->_getNextTriangle();

			//current triangle vertices
			const CCVector3* triPoints[3]{ T->_getA(), T->_getB(), T->_getC() };

			if (CCMiscTools::TriBoxOverlap(centroid, bbDims, triPoints))
			{
				count++;
			}
		}
		if (count > 0)
		{
			cluster->SetChangeType(-1);
		}
		else
		{
			cluster->SetChangeType(1);
		}
	}
	ScalarType changeType = static_cast<ScalarType>(cluster->GetChangeType());
	ScalarType uncertainty = static_cast<ScalarType>(cluster->GetUncertainty());
	ScalarType volume = static_cast<ScalarType>(cluster->GetVolume());

	/* Populate scalar fields for current cluster's voxels */
	for (unsigned i = 0; i < cluster->GetSize(); i++)
	{
		auto voxel = s_VoxFallParams.voxelGraph->GetNodeIndex(cluster->GetVoxel(i));
		if (s_VoxFallParams.exportLossGain)
		{
			s_VoxFallParams.changeTypeSF->setValue(voxel, changeType);
		}
		s_VoxFallParams.volumeSF->setValue(voxel, volume);
		s_VoxFallParams.uncertaintySF->setValue(voxel, volume/uncertainty/100);
	}

	if (error) return !error;
	return !error;
}


bool qVoxFallProcess::Compute(const qVoxFallDialog& dlg, QString& errorMessage, ccPointCloud*& outputCloud, ccHObject*& outputGroup, bool allowDialogs, QWidget* parentWidget/*=nullptr*/, ccMainAppInterface* app/*=nullptr*/)
{
	errorMessage.clear();
	outputCloud = nullptr;
	outputGroup = nullptr;

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

	auto mesh = mesh1->cloneMesh();
	mesh->merge(mesh2, false);

	auto transform = qVoxFallTransform(dip, dipdir);
	mesh->applyGLTransformation_recursive(&transform.matrix);
	mesh1->applyGLTransformation_recursive(&transform.matrix);

	mesh1->setEnabled(false);

	/* parameters are stored in 's_VoxFallParams' for parallel call */
	s_VoxFallParams = VoxFallParams();
	s_VoxFallParams.voxelSize = dlg.getVoxelSize();
	s_VoxFallParams.minBound = mesh->getOwnBB().minCorner();
	s_VoxFallParams.maxBound = mesh->getOwnBB().maxCorner();
	s_VoxFallParams.steps = ((s_VoxFallParams.maxBound - s_VoxFallParams.minBound) / s_VoxFallParams.voxelSize) + Vector3Tpl<float>(1, 1, 1);
	s_VoxFallParams.genarateReport = dlg.getGenerateReportActivation();
	s_VoxFallParams.exportBlocksAsMeshes = dlg.getExportMeshesActivation();
	s_VoxFallParams.exportLossGain = dlg.getLossGainActivation();
	s_VoxFallParams.groupName = mesh1->getName() + "_to_" + mesh2->getName() + QString(" [VoxFall] (voxel %1m)").arg(s_VoxFallParams.voxelSize);
	s_VoxFallParams.voxfall = new ccPointCloud(s_VoxFallParams.groupName);

	/** Initialize voxel grid **/
	auto voxelGrid = CCCoreLib::Grid3D<int>();
	if (!voxelGrid.init(	int(s_VoxFallParams.steps.x),
							int(s_VoxFallParams.steps.y),
							int(s_VoxFallParams.steps.z),
							0	))  //margin
	{
		errorMessage = "Failed to initialize voxel grid!";
		return false;
	}

	QElapsedTimer graphTimer;
	graphTimer.start();
	/** Computing voxel graph with 26 connectivity (1->6, 2->18, 3->26) **/
	s_VoxFallParams.voxelGraph = new qVoxFallGraph(voxelGrid.size(), 3);
	qint64 graphTime_ms = graphTimer.elapsed();
	if (app)
	{
		app->dispToConsole(QString("[VoxFall] Grid graph computation: %1 s").arg(graphTime_ms / 1000.0, 0, 'f', 3),
			ccMainAppInterface::STD_CONSOLE_MESSAGE);
	}

	/** Initialize helpers **/ 
	auto voxelCount = voxelGrid.innerCellCount();
	s_VoxFallParams.voxfall->reserve(voxelCount);
	s_VoxFallParams.nbs.resize(voxelCount);
	s_VoxFallParams.isEmpty.resize(voxelCount, true);
	s_VoxFallParams.isEmptyBefore.resize(voxelCount, true);

	/** Allocate cluster ID SF **/
	s_VoxFallParams.clusterSF = new ccScalarField(CLUSTER_SF_NAME);
	s_VoxFallParams.clusterSF->link();
	if (!s_VoxFallParams.clusterSF->resizeSafe(voxelCount, true, static_cast<ScalarType>(-1.0)))
	{
		errorMessage = "Failed to allocate memory for cluster ID values!";
		return false;
	}

	if (s_VoxFallParams.exportLossGain)
	{
		/** Allocate change type SF **/
		s_VoxFallParams.changeTypeSF = new ccScalarField(CHANGE_TYPE_SF_NAME);
		s_VoxFallParams.changeTypeSF->link();
		if (!s_VoxFallParams.changeTypeSF->resizeSafe(voxelCount, true, CCCoreLib::NAN_VALUE))
		{
			errorMessage = "Failed to allocate memory for change type values!";
			return false;
		}
	}
	/** Allocate volume SF **/
	s_VoxFallParams.volumeSF = new ccScalarField(VOLUME_SF_NAME);
	s_VoxFallParams.volumeSF->link();
	if (!s_VoxFallParams.volumeSF->resizeSafe(voxelCount, true, CCCoreLib::NAN_VALUE))
	{
		errorMessage = "Failed to allocate memory for volume values!";
		return false;
	}
	/** Allocate volume uncertainty SF **/
	s_VoxFallParams.uncertaintySF = new ccScalarField(UNCERTAINTY_SF_NAME);
	s_VoxFallParams.uncertaintySF->link();
	if (!s_VoxFallParams.uncertaintySF->resizeSafe(voxelCount, true, CCCoreLib::NAN_VALUE))
	{
		errorMessage = "Failed to allocate memory for volume uncertainty values!";
		return false;
	}

	/** Initialize output cloud **/
	if (!InitializeOutputCloud(voxelCount, &pDlg))
	{
		errorMessage = "Failed to initialize output data!";
		return false;
	}

	qint64 initTime_ms = initTimer.elapsed();
	/* we display init. timing only if no error occurred! */
	if (app)
	{
		app->dispToConsole(QString("[VoxFall] Initialization: %1 s").arg(initTime_ms / 1000.0, 0, 'f', 3),
			ccMainAppInterface::STD_CONSOLE_MESSAGE);
	}


/* 	   BLOCK DETECTION
 * ======================================================================================================================= */

	//Duration: Occupancy
	QElapsedTimer occupTimer;
	occupTimer.start();

	if (!voxelGrid.intersectWith(	mesh,
									s_VoxFallParams.voxelSize,
									s_VoxFallParams.minBound,
									GetVoxelOccupancy,
									&pDlg	))
	{
		errorMessage = "Failed to compute  grid occupancy!";
		return false;
	}

	if (s_VoxFallParams.exportLossGain)
	{
		if (!voxelGrid.intersectWith(mesh1,
			s_VoxFallParams.voxelSize,
			s_VoxFallParams.minBound,
			GetVoxelOccupancyBefore,
			&pDlg))
		{
			errorMessage = "Failed to compute  grid occupancy!";
			return false;
		}
	}

	qint64 occupTime_ms = occupTimer.elapsed();
	if (app)
	{
		app->dispToConsole(QString("[VoxFall] Occupancy computation: %1 s").arg(occupTime_ms / 1000.0, 0, 'f', 3),
			ccMainAppInterface::STD_CONSOLE_MESSAGE);
	}

	//Duration: Detection
	QElapsedTimer detectTimer;
	detectTimer.start();

	/** cluster empty space with connected components **/
	if (!ClusterEmptySpace(	maxThreadCount,
							voxelCount,
							&pDlg	))
	{
		errorMessage = "Failed to cluster empty space!";
		return false;
	}
	int numOfClusters = s_VoxFallParams.clusters.size();

	qint64 detectTime_ms = detectTimer.elapsed();
	/* we display block extraction timing only if no error occurred! */
	if (app)
	{
		app->dispToConsole(QString("[VoxFall] Block detection: %1 s").arg(detectTime_ms / 1000.0, 0, 'f', 3),
			ccMainAppInterface::STD_CONSOLE_MESSAGE);
		app->dispToConsole(QString("[VoxFall] Blocks found: %1").arg(numOfClusters),
			ccMainAppInterface::STD_CONSOLE_MESSAGE);
	}


/* 	   COMPUTE VOLUMES
 * ======================================================================================================================= */

	//Duration: volume computation
	QElapsedTimer volumeTimer;
	volumeTimer.start();

	//progress notification
	pDlg.reset();
	NormalizedProgress nProgress(&pDlg, s_VoxFallParams.emptyVoxelCount);
	char buffer[64];
	snprintf(buffer, 64, "VoxFall clusters: %u \n Empty voxels: %u", numOfClusters, s_VoxFallParams.emptyVoxelCount);
	pDlg.setInfo(buffer);
	pDlg.setMethodTitle(QObject::tr("Compute Volumes"));
	pDlg.update(0);
	pDlg.start();
	s_VoxFallParams.nProgress = &nProgress;

	auto rV = s_VoxFallParams.voxelGraph->GetNumReducedNodes();
	s_VoxFallParams.volumes.resize(numOfClusters);
	s_VoxFallParams.nonEmptyVoxelsVisited.resize(voxelCount, false);
	for (int cl = 0; cl < numOfClusters; ++cl)
	{
		/* volume computation */
		if (!ComputeClusterVolume(	maxThreadCount, 
									cl, 
									&transform,
									mesh1,
									nullptr	))
		{
			errorMessage = "Failed to compute cluster volume!";
			return false;
		}
	}

	qint64 volumeTime_ms = volumeTimer.elapsed();
	/* we display block volume computation timing only if no error occurred! */
	if (app)
	{
		app->dispToConsole(QString("[VoxFall] Volume computation: %1 s").arg(volumeTime_ms / 1000.0, 0, 'f', 3),
			ccMainAppInterface::STD_CONSOLE_MESSAGE);
	}


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
		snprintf(buffer, 64, "Blocks: %u", numOfClusters);
		pDlg.setInfo(buffer);
		pDlg.setMethodTitle(QObject::tr("Exporting blocks as meshes"));
		pDlg.update(0);
		pDlg.start();

		/* we create a new group to store all output meshes as 'VoxFall clusters' */
		ccHObject* ccGroup = new ccHObject(s_VoxFallParams.groupName);

		/* Reorder clusters by sort volume */
		std::vector<int> reorder(numOfClusters);
		std::vector<float> volumes(numOfClusters);
		for (unsigned cl = 0; cl < numOfClusters; cl++)
		{
			reorder[cl] = cl;
			volumes[cl] = s_VoxFallParams.clusters[cl]->GetVolume();
		}
		std::sort(reorder.begin(), reorder.end(), [&volumes](int a, int b) {
			return volumes[a] > volumes[b];
		});

		/* Add cluster to group in DB*/
		for (unsigned cl = 0; cl < numOfClusters; cl++)
		{
			auto cluster = s_VoxFallParams.clusters[reorder[cl]];
			ccGroup->addChild(cluster->GetClusterMesh());

		}
		ccGroup->setVisible(true);
		if (app)
		{
			app->addToDB(ccGroup);
		}
			
		{
			// command line mode
			outputGroup = ccGroup;
		}

		qint64 meshTime_ms = meshTimer.elapsed();
		//we display block as mesh export timing only if no error occurred!
		if (app)
		{
			app->dispToConsole(QString("[VoxFall] Block as mesh export: %1 s").arg(meshTime_ms / 1000.0, 0, 'f', 3),
				ccMainAppInterface::STD_CONSOLE_MESSAGE);
		}
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
	if (s_VoxFallParams.exportLossGain)
	{
		//associate change type scalar fields to the voxel grid
		if (s_VoxFallParams.changeTypeSF)
		{
			//add cluster ID SF to voxel grid
			s_VoxFallParams.changeTypeSF->computeMinAndMax();
			sfIdx = s_VoxFallParams.voxfall->addScalarField(s_VoxFallParams.changeTypeSF);
		}
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
	mesh1->applyGLTransformation_recursive(&transform.inverse);
	s_VoxFallParams.voxfall->applyGLTransformation_recursive(&transform.inverse);
	sfIdx = s_VoxFallParams.voxfall->getScalarFieldIndexByName(CLUSTER_SF_NAME);
	s_VoxFallParams.voxfall->setCurrentDisplayedScalarField(sfIdx);;
	s_VoxFallParams.voxfall->showSF(true);
	if (s_VoxFallParams.exportBlocksAsMeshes)
	{
		s_VoxFallParams.voxfall->setEnabled(false);
	}
	if (app)
	{
		app->addToDB(s_VoxFallParams.voxfall);
	}
	else
	{
		// command line mode
		outputCloud = s_VoxFallParams.voxfall;
	}
	
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
		outStream << " Change type,";
		outStream << " Volume (m3),";
		outStream << " Uncertainty (m3),";
		outStream << " \n";

		/* add info line for each cluster */
		for (unsigned cl = 0; cl < numOfClusters; cl++)
		{
			auto cluster = s_VoxFallParams.clusters[cl];

			/* get data from cluster cloud */
			ccBBox bb = cluster->GetBB();
			CCVector3 centroid = bb.getCenter();
			CCVector3d extent = bb.maxCorner() - bb.minCorner();

			auto loss_gain = "n/a";
			if (s_VoxFallParams.exportLossGain)
			{
				if (cluster->GetChangeType() == -1)
				{
					loss_gain = "loss";
				}
				else
				{
					loss_gain = "gain";
				}
			}
			
			//add data to file
			outStream << cluster->GetLabel()<< ","; //cluster ID
			outStream << centroid.x << "," << centroid.y << "," << centroid.z << ","; //center XYZ
			if (extent.x > 0) { outStream << extent.x << ","; } else { outStream << s_VoxFallParams.voxelSize << ","; }; //extent X
			if (extent.y > 0) { outStream << extent.y << ","; } else { outStream << s_VoxFallParams.voxelSize << ","; }; //extent Y
			if (extent.z > 0) { outStream << extent.z << ","; } else { outStream << s_VoxFallParams.voxelSize << ","; }; //extent Z
			outStream << loss_gain << ","; //change type (loss/gain)
			outStream << cluster->GetVolume() << ","; //volume
			outStream << cluster->GetUncertainty() << ","; //uncertainty
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
