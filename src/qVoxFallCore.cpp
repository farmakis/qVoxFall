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
	int clusterLabel = 0;
	int currentLabel;
	int changeType;
	bool genarateReport = false;
	bool exportBlocksAsMeshes = false;
	bool exportLossGain = false;
	CCVector3 minBound, maxBound, extent, steps;

	//helpers
	std::vector<std::vector<int>> nbs;
	std::vector<bool> isEmpty;
	std::vector<bool> nonEmptyVoxelsVisited;
	std::vector<int> clusters;
	std::vector<int> excludedClusterLabels;
	int tempCornerIdx = 0;
	int maxExcludedClusterLabel = 0;
	std::vector<unsigned int> clusterIndices;
	int clusterOutterVoxelCount;

	//export
	ccPointCloud* voxfall = nullptr;

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
	// this function is used as a callback within the Grid3D::intersectWith method
	// and is being activated when an intersection with the grid is found
	// to keep track of the non-empty voxels
	int index = qVoxFallTools::Grid2Index(cellPos, s_VoxFallParams.steps);
	s_VoxFallParams.isEmpty[index] = false;
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
		//TODO: check if the voxel is one of the 8 corners of the grid
		auto V = qVoxFallTools::Index2Grid(index, steps);
		auto NN = qVoxFallTools::FindAdjacents(V, steps, false);
		for (auto const& n : NN)
		{
			int nIdx = qVoxFallTools::Grid2Index(n, steps);
			s_VoxFallParams.nbs[index].push_back(nIdx);
		}
		//nProgress.oneStep();
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
			//If the voxel is a grid corner, we add its given label to the excluded cluster labels
			auto V = qVoxFallTools::Index2Grid(index, steps);
			if (qVoxFallTools::IsGridCorner(V, s_VoxFallParams.steps))
			{
				int excludedLabel = static_cast<int>(s_VoxFallParams.clusterSF->getValue(index));
				s_VoxFallParams.excludedClusterLabels[s_VoxFallParams.tempCornerIdx] = excludedLabel;
				s_VoxFallParams.tempCornerIdx++;
				if (excludedLabel > s_VoxFallParams.maxExcludedClusterLabel)
				{
					s_VoxFallParams.maxExcludedClusterLabel = excludedLabel;
				}
			}
			continue;
		}

		//If the voxel is yet unlabled and is a grid corner, we add the current label to the excluded cluster labels
		auto V = qVoxFallTools::Index2Grid(index, steps);
		if (qVoxFallTools::IsGridCorner(V, s_VoxFallParams.steps))
		{
			int excludedLabel = static_cast<int>(s_VoxFallParams.clusterSF->getValue(index));
			s_VoxFallParams.excludedClusterLabels[s_VoxFallParams.tempCornerIdx] = s_VoxFallParams.clusterLabel;
			s_VoxFallParams.tempCornerIdx++;
			if (excludedLabel > s_VoxFallParams.maxExcludedClusterLabel)
			{
				s_VoxFallParams.maxExcludedClusterLabel = excludedLabel;
			}
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
	//keep unique excluded cluster labels
	std::sort(s_VoxFallParams.excludedClusterLabels.begin(), s_VoxFallParams.excludedClusterLabels.end());
	auto newEnd = std::unique(s_VoxFallParams.excludedClusterLabels.begin(), s_VoxFallParams.excludedClusterLabels.end());
	s_VoxFallParams.excludedClusterLabels.erase(newEnd, s_VoxFallParams.excludedClusterLabels.end());

	return true;
}


bool ComputeClusterVolume(int maxThreads, int clusterCount, ccHObject* clusterGroup = nullptr)
{

	std::atomic<bool> error(false);

	if (s_VoxFallParams.processCanceled)
		return error;

#if defined(_OPENMP)
#pragma omp parallel for schedule(static) \
        num_threads(maxThreads)
#endif
	for (int i = 0; i < clusterCount; i++)
	{
		int index = s_VoxFallParams.clusterIndices[i];

		if (error) {
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

		////progress bar
		//if (!s_VoxFallParams.nProgress->oneStep())
		//{
		//	error = true;
		//}
	}

	if (error) return !error;
	return !error;
}

bool qVoxFallCore::Run(	ccMesh* mesh,
						ccBBox* bb,
						int sign,
						qVoxFallTransform transform,
						int& patchFirstLabel,
						int maxThreadCount,
						QString& errorMessage,
						ccPointCloud* outputCloud,
						ccHObject* outputGroup,
						const qVoxFallDialog& dlg,
						ccProgressDialog& pDlg)

{
	//parameters are stored in 's_VoxFallParams' for parallel call
	s_VoxFallParams = VoxFallParams();
	s_VoxFallParams.clusterLabel = patchFirstLabel;
	s_VoxFallParams.voxelSize = dlg.getVoxelSize();
	s_VoxFallParams.minBound = bb->minCorner();
	s_VoxFallParams.maxBound = bb->maxCorner();
	s_VoxFallParams.extent = s_VoxFallParams.maxBound - s_VoxFallParams.minBound;
	s_VoxFallParams.steps = (s_VoxFallParams.extent / s_VoxFallParams.voxelSize) + Vector3Tpl<float>(1, 1, 1);
	s_VoxFallParams.genarateReport = dlg.getGenerateReportActivation();
	s_VoxFallParams.exportBlocksAsMeshes = dlg.getExportMeshesActivation();
	s_VoxFallParams.excludedClusterLabels.resize(8);
	s_VoxFallParams.voxfall = outputCloud;

	//Initialize voxel grid
	auto voxelGrid = CCCoreLib::Grid3D<int>();
	if (!voxelGrid.init(int(s_VoxFallParams.steps.x),
						int(s_VoxFallParams.steps.y),
						int(s_VoxFallParams.steps.z),
						0))  //margin
	{
		errorMessage = "Failed to initialize voxel grid!";
		return false;
	}

	// Initialize heplpers
	s_VoxFallParams.voxfall->reserve(voxelGrid.innerCellCount());
	s_VoxFallParams.nbs.resize(voxelGrid.innerCellCount());
	s_VoxFallParams.isEmpty.resize(voxelGrid.innerCellCount(), true);
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

	if (s_VoxFallParams.exportLossGain)
	{
		//allocate change type SF
		s_VoxFallParams.changeTypeSF = new ccScalarField(CHANGE_TYPE_SF_NAME);
		s_VoxFallParams.changeTypeSF->link();
		if (!s_VoxFallParams.changeTypeSF->resizeSafe(voxelGrid.innerCellCount(), true, CCCoreLib::NAN_VALUE))
		{
			errorMessage = "Failed to allocate memory for change type values!";
			return false;
		}
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

// 	   BLOCK DETECTION
//=======================================================================================================================

	if (!voxelGrid.intersectWith(mesh,
								s_VoxFallParams.voxelSize,
								s_VoxFallParams.minBound,
								GetVoxelOccupancy,
								&pDlg))
	{
		errorMessage = "Failed to compute  grid occupancy!";
		return false;
	}

	//cluster DBSCAN
	if (!ClusterEmptySpace(	maxThreadCount,
							voxelGrid.innerCellCount(),
							&pDlg))
	{
		errorMessage = "Failed to compute grid occupancy!";
		return false;
	}
	//errorMessage = QString("[VoxFall] label for the next patch: %1").arg(patchFirstLabel);
	//return true;


// 	   COMPUTE VOLUMES
//=======================================================================================================================

	int currentVolumeIdx = 0;
	std::vector<std::pair<float, int>> pairVolumeLabel; //pair volumes-labels to sort them by volume later
	s_VoxFallParams.nonEmptyVoxelsVisited.resize(voxelGrid.innerCellCount(), false);
	for (int label = patchFirstLabel; label < s_VoxFallParams.clusterLabel; ++label)
	{
		// if the label is one of the excluded cluster labels, we skip it
		// first we check if it is within the vector so we can skip checking for no reason
		if (!(label > s_VoxFallParams.maxExcludedClusterLabel))
		{
			if (std::find(s_VoxFallParams.excludedClusterLabels.begin(), s_VoxFallParams.excludedClusterLabels.end(), label) != s_VoxFallParams.excludedClusterLabels.end())
			{
				continue;
			}
		}

		for (unsigned i = 0; i < static_cast<unsigned>(s_VoxFallParams.clusterSF->size()); ++i)
		{
			if (s_VoxFallParams.clusterSF->getValue(i) == static_cast<ScalarType>(label))
			{
				s_VoxFallParams.clusterIndices.push_back(i);
			}
		}

		s_VoxFallParams.currentLabel = label;
		s_VoxFallParams.clusterOutterVoxelCount = 0;

		if (!ComputeClusterVolume(maxThreadCount, static_cast<int>(s_VoxFallParams.clusterIndices.size())))
		{
			errorMessage = "Failed to compute cluster volume!";
			return false;
		}

		ScalarType changeType = static_cast<ScalarType>(sign);
		ScalarType uncertainty = static_cast<ScalarType>(pow(s_VoxFallParams.voxelSize, 3) * s_VoxFallParams.clusterOutterVoxelCount / 2);
		ScalarType volume = static_cast<ScalarType>(pow(s_VoxFallParams.voxelSize, 3) * s_VoxFallParams.clusterIndices.size() + uncertainty);
		pairVolumeLabel.push_back({ volume, label });

		for (unsigned i = 0; i < s_VoxFallParams.clusterIndices.size(); i++)
		{
			s_VoxFallParams.volumeSF->setValue(s_VoxFallParams.clusterIndices[i], volume);
			s_VoxFallParams.uncertaintySF->setValue(s_VoxFallParams.clusterIndices[i], volume / uncertainty / 100);
		}
		s_VoxFallParams.clusterIndices.clear();
		currentVolumeIdx++;
	}

// 	   EXPORT BLOCKS AS VOXEL MESH MODELS (IF SELECTED)
//=======================================================================================================================

	if (s_VoxFallParams.exportBlocksAsMeshes)
	{
		//Duration: block meshing
		QElapsedTimer meshTimer;
		meshTimer.start();

		////progress notification
		//pDlg.reset();
		//NormalizedProgress nProgress(&pDlg, s_VoxFallParams.emptyVoxelCount);
		//char buffer[64];
		//snprintf(buffer, 64, "Blocks: %u", clusterCount);
		//pDlg.setInfo(buffer);
		//pDlg.setMethodTitle(QObject::tr("Exporting blocks as meshes"));
		//pDlg.update(0);
		//pDlg.start();

		//sort the pair volumes-labels vector by volume for the output order
		std::sort(pairVolumeLabel.begin(), pairVolumeLabel.end(), [](const std::pair<float, int>& a, const std::pair<float, int>& b) {
			return a.first > b.first;  // Compare by the first element (int) in descending order
			});

		for (int c = 0; c < pairVolumeLabel.size(); c++)
		{
			auto volume = pairVolumeLabel[c].first;
			auto label = pairVolumeLabel[c].second;

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
					clusterMesh->addTriangle(tri->i1 + vertCount, tri->i2 + vertCount, tri->i3 + vertCount);
				}

				////progress bar
				//if (!nProgress.oneStep())
				//{
				//	return false;
				//}
			}
			clusterMesh->setName(QString("Cluster#%1 - (v: %2 m3)").arg(label).arg(volume));
			clusterMesh->computePerVertexNormals();
			clusterCloud->resize(clusterCloud->size());
			clusterMesh->addChild(clusterCloud);
			outputGroup->addChild(clusterMesh);
			indices.clear();
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
	//associate change type scalar fields to the voxel grid
	if (s_VoxFallParams.changeTypeSF)
	{
		//add cluster ID SF to voxel grid
		s_VoxFallParams.changeTypeSF->computeMinAndMax();
		sfIdx = s_VoxFallParams.voxfall->addScalarField(s_VoxFallParams.changeTypeSF);
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
	s_VoxFallParams.voxfall->applyGLTransformation_recursive(&transform.inverse);
	sfIdx = s_VoxFallParams.voxfall->getScalarFieldIndexByName(CLUSTER_SF_NAME);
	s_VoxFallParams.voxfall->setCurrentDisplayedScalarField(sfIdx);
	s_VoxFallParams.voxfall->showSF(true);

	//outputCloud = s_VoxFallParams.voxfall;
	//update label for the next patch
	patchFirstLabel = s_VoxFallParams.clusterLabel;

	return true;
}


