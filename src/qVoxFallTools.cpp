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


#include "qVoxFallTools.h"

//qCC_db
#include <ccPointCloud.h>

//qCC
#include <ccMainAppInterface.h>
#include <ccQtHelpers.h>

//CCCoreLib
#include "AutoSegmentationTools.h"

//local
#include "qVoxFallDialog.h"

//Qt
#include <QtCore>
#include <QApplication>
#include <QMainWindow>
#include <QProgressDialog>
#include <QtConcurrentMap>

const unsigned char DEFAULT_OCTREE_LEVEL = 7;
static const char TEMP_APPROX_DISTANCES_SF_NAME[] = "ApproxDists";
static const char TEMP_DISTANCES_SF_NAME[] = "Distances";
static const char CONNECTED_COMPONENTS_SF_NAME[] = "CC labels";

ccBox* qVoxFallTransform::CreateVoxelMesh(CCVector3 V, float voxelSize, int voxelIdx)
{
	CCVector3 dims = { voxelSize, voxelSize, voxelSize };
	QString name = QString("voxel#%1").arg(voxelIdx);

	const Vector3Tpl<float> X(1, 0, 0);
	const Vector3Tpl<float> Y(0, 1, 0);
	const Vector3Tpl<float> Z(0, 0, 1);
	const ccGLMatrix matrix(X, Y, Z, V);

	ccBox* voxel = new ccBox(dims, &matrix, name);
	return voxel;
}


std::vector<Tuple3i> qVoxFallTools::FindAdjacents(Tuple3i V, CCVector3 steps, bool facetsOnly=false)
{
	std::vector<Tuple3i> set;
	std::vector<std::vector<int>> adjacencyMatrix;

	if (!facetsOnly)
	{
		adjacencyMatrix = {
			{1, 0, 0}, {-1, 0, 0}, {0, 1, 0},
			{0, -1, 0}, {0, 0, 1}, {0, 0, -1},
			{1, 1, 0},  {-1, 1, 0}, {1, -1, 0},
			{-1, -1, 0}, {0, 1, 1}, {0, 1, -1},
			{0, -1, 1}, {0, -1, -1}, {1, 0, 1},
			{1, 0, -1}, {-1, 0, 1}, {-1, 0, -1},
			{1, 1, 1}, {-1, -1, -1}, {1, 1, -1},
			{1, -1, 1}, {-1, 1, 1}, {1, -1, -1},
			{-1, -1, 1}, {-1, 1, -1}
		};
	}
	else
	{
		adjacencyMatrix = {
				{1, 0, 0},  {-1, 0, 0}, {0, 1, 0},
				{0, -1, 0}, {0, 0, 1},  {0, 0, -1},
		};
	}

	for (unsigned n = 0; n < adjacencyMatrix.size(); n++)
	{

		int x = int(V.x) + adjacencyMatrix[n][0];
		int y = int(V.y) + adjacencyMatrix[n][1];
		int z = int(V.z) + adjacencyMatrix[n][2];

		if (x < 0 || y < 0 || z < 0 || x >= int(steps.x) || y >= int(steps.y) || z >= int(steps.z))
		{
			continue;
		}
							
		set.push_back({ x, y, z });
	}
	
	return set;
}

bool qVoxFallTools::IsGridCorner(const Tuple3i& V, const CCVector3& steps)
{
	int maxX = static_cast<int>(steps.x) - 1;
	int maxY = static_cast<int>(steps.y) - 1;
	int maxZ = static_cast<int>(steps.z) - 1;

	bool isXCorner = (V.x == 0 || V.x == maxX);
	bool isYCorner = (V.y == 0 || V.y == maxY);
	bool isZCorner = (V.z == 0 || V.z == maxZ);

	return isXCorner && isYCorner && isZCorner;
}

int qVoxFallTools::Grid2Index(Tuple3i n, CCVector3 steps)
{
	int i = n.x;
	int j = n.y;
	int k = n.z;

	int index = (i)+(j * int(steps.x)) + (k * int(steps.x) * int(steps.y));
	return index;
}


Tuple3i qVoxFallTools::Index2Grid(unsigned index, CCVector3 steps)
{
	int k = std::floor(index / (int(steps.y) * int(steps.x)));
	int remain = index - (int(steps.y) * int(steps.x) * k);
	int j = std::floor(remain / int(steps.x));
	int i = remain - (int(steps.x) * j);

	Tuple3i V(	static_cast<int>(i),
				static_cast<int>(j),
				static_cast<int>(k)	);
	return V;
}


int qVoxFallTools::GetBestOctreeLevel(double maxSearchDist, ccPointCloud* m_compCloud, ccMesh* refMesh, ccOctree::Shared m_compOctree, ccOctree::Shared m_refOctree)
{
	//make sure a the temporary dist. SF is activated
	int sfIdx = m_compCloud->getScalarFieldIndexByName(TEMP_APPROX_DISTANCES_SF_NAME);

	const CCCoreLib::ScalarField* approxDistances = m_compCloud->getScalarField(sfIdx);
	if (!approxDistances)
	{
		assert(sfIdx >= 0);
		return -1;
	}

	//evalutate the theoretical time for each octree level
	const int MAX_OCTREE_LEVEL = refMesh ? 9 : CCCoreLib::DgmOctree::MAX_OCTREE_LEVEL; //DGM: can't go higher than level 9 with a mesh as the grid is 'plain' and would take too much memory!
	std::vector<double> timings;
	try
	{
		timings.resize(MAX_OCTREE_LEVEL, 0);
	}
	catch (const std::bad_alloc&)
	{
		ccLog::Warning("Can't determine best octree level: not enough memory!");
		return -1;
	}

	//if the reference is a mesh
	double meanTriangleSurface = 1.0;
	CCCoreLib::GenericIndexedMesh* mesh = nullptr;
	mesh = static_cast<CCCoreLib::GenericIndexedMesh*>(refMesh);
	if (!mesh || mesh->size() == 0)
	{
		ccLog::Warning("Can't determine best octree level: mesh is empty!");
		return -1;
	}
	//total mesh surface
	double meshSurface = CCCoreLib::MeshSamplingTools::computeMeshArea(mesh);
	//average triangle surface
	if (meshSurface > 0)
	{
			meanTriangleSurface = meshSurface / mesh->size();
	}

	//we skip the lowest subdivision levels (useless + incompatible with below formulas ;)
	static const int s_minOctreeLevel = 6;
	int theBestOctreeLevel = s_minOctreeLevel;

	PointCoordinateType maxDistance = maxSearchDist;

	uint64_t maxNeighbourhoodVolume = static_cast<uint64_t>(1) << (3 * MAX_OCTREE_LEVEL);

	//for each level
	for (int level = s_minOctreeLevel; level < MAX_OCTREE_LEVEL; ++level)
	{
		const unsigned char bitDec = CCCoreLib::DgmOctree::GET_BIT_SHIFT(level);
		unsigned numberOfPointsInCell = 0;
		unsigned index = 0;
		double cellDist = -1;
		//unsigned skippedCells = 0;

		//we compute a 'correction factor' that converts an approximate distance into an
		//approximate size of the neighborhood (in terms of cells)
		PointCoordinateType cellSize = m_compOctree->getCellSize(static_cast<unsigned char>(level));

		//we also use the reference cloud density (points/cell) if we have the info
		double refListDensity = 1.0;
		if (m_refOctree)
		{
			refListDensity = m_refOctree->computeMeanOctreeDensity(static_cast<unsigned char>(level));
		}

		CCCoreLib::DgmOctree::CellCode tempCode = 0xFFFFFFFF;

		//scan the octree structure
		const CCCoreLib::DgmOctree::cellsContainer& compCodes = m_compOctree->pointsAndTheirCellCodes();
		for (CCCoreLib::DgmOctree::cellsContainer::const_iterator c = compCodes.begin(); c != compCodes.end(); ++c)
		{
			CCCoreLib::DgmOctree::CellCode truncatedCode = (c->theCode >> bitDec);

			//new cell?
			if (truncatedCode != tempCode)
			{
				//if it's a real cell
				if (numberOfPointsInCell != 0)
				{
					//if 'maxSearchDist' has been defined by the user, we must take it into account!
					//(in this case we skip the cell if its approx. distance is superior)
					if (maxSearchDist <= 0 || cellDist <= maxSearchDist)
					{
						//approx. neighborhood radius
						cellDist /= cellSize;

						//approx. neighborhood width (in terms of cells)
						double neighbourSize = 2.0 * cellDist + 1.0;

						//if the reference is a mesh
						if (mesh)
						{
							//(integer) approximation of the neighborhood size (in terms of cells)
							int nCell = static_cast<int>(ceil(cellDist));

							//Probable mesh surface in this neighborhood
							double crossingMeshSurface = (2.0 * nCell + 1.0) * cellSize;
							//squared surface!
							crossingMeshSurface *= crossingMeshSurface;

							//neighborhood "volume" (in terms of cells)
							double neighbourSize3 = (neighbourSize * neighbourSize * neighbourSize) / maxNeighbourhoodVolume;

							//TIME = NEIGHBORS SEARCH + proportional factor * POINTS/TRIANGLES COMPARISONS
							timings[level] += neighbourSize3 + ((0.5 * numberOfPointsInCell) / maxNeighbourhoodVolume) * (crossingMeshSurface / meanTriangleSurface);
						}
						else
						{
							//we ignore the "central" cell
							neighbourSize -= 1.0;
							//neighborhood "volume" (in terms of cells)
							double neighbourSize3 = (neighbourSize * neighbourSize * neighbourSize) / maxNeighbourhoodVolume;
							//volume of the last "slice" (in terms of cells)
							//=V(n)-V(n-1) = (2*n+1)^3 - (2*n-1)^3 = 24 * n^2 + 2 (if n > 0)
							double lastSliceCellCount = (cellDist > 0 ? cellDist * cellDist * 24.0 + 2.0 : 1.0);
							//TIME = NEIGHBORS SEARCH + proportional factor * POINTS/TRIANGLES COMPARISONS
							//(we admit that the filled cells roughly correspond to the sqrt of the total number of cells)
							timings[level] += neighbourSize3 + 0.1 * ((numberOfPointsInCell * sqrt(lastSliceCellCount) * refListDensity) / maxNeighbourhoodVolume);
						}
					}
				}

				numberOfPointsInCell = 0;
				cellDist = 0;
				tempCode = truncatedCode;
			}

			ScalarType pointDist = approxDistances->getValue(index);
			if (pointDist > maxDistance)
			{
				pointDist = maxDistance;
			}

			//cellDist += pointDist;
			cellDist = std::max<double>(cellDist, pointDist);
			++index;
			++numberOfPointsInCell;
		}

		if (timings[level] * 1.05 < timings[theBestOctreeLevel]) //avoid increasing the octree level for super small differences (which is generally counter productive)

		{
			theBestOctreeLevel = level;
		}
	}

	ccLog::PrintDebug("[Distances] Best level: %i (maxSearchDist = %f)", theBestOctreeLevel, maxSearchDist);

	return theBestOctreeLevel;
}


bool qVoxFallTools::ComputeDistances(ccMesh& compMesh, ccMesh& refMesh, ccProgressDialog pDlg, int threadCount, int& octreeLevel)
{
	CCCoreLib::DistanceComputationTools::Cloud2MeshDistancesComputationParams c2mParams;
	{
		//setup parameters
		{
			c2mParams.octreeLevel = DEFAULT_OCTREE_LEVEL;;
			c2mParams.maxSearchDist = 0;
			c2mParams.useDistanceMap = true;
			c2mParams.signedDistances = false;
			c2mParams.flipNormals = false;
			c2mParams.multiThread = false;
			c2mParams.robust = true;
		}
	}

	ccPointCloud* m_compCloud = static_cast<ccPointCloud*>(compMesh.getAssociatedCloud());
	ccGenericPointCloud* m_refCloud = refMesh.getAssociatedCloud();

	//whatever the case, we always need the compared cloud's octree
	auto m_compOctree = m_compCloud->getOctree();
	if (!m_compOctree)
	{
		m_compOctree = ccOctree::Shared(new ccOctree(m_compCloud));
	}
	auto m_refOctree = m_refCloud->getOctree();
	if (!m_refOctree)
	{
		m_refOctree = ccOctree::Shared(new ccOctree(m_refCloud));
	}

	//backup currently displayed SF (on compared cloud)
	int oldSfIdx = m_compCloud->getCurrentDisplayedScalarFieldIndex();
	if (oldSfIdx >= 0)
	{
		auto m_oldSfName = QString::fromStdString(m_compCloud->getScalarFieldName(oldSfIdx));
	}

	//create the approximate dist. SF if necessary
	int sfIdx = m_compCloud->getScalarFieldIndexByName(TEMP_APPROX_DISTANCES_SF_NAME);
	if (sfIdx < 0)
	{
		sfIdx = m_compCloud->addScalarField(TEMP_APPROX_DISTANCES_SF_NAME);
		if (sfIdx < 0)
		{
			ccLog::Error("Failed to allocate a new scalar field for computing approx. distances! Try to free some memory ...");
			return false;
		}
	}

	m_compCloud->setCurrentScalarField(sfIdx);
	CCCoreLib::ScalarField* sf = m_compCloud->getCurrentInScalarField();
	assert(sf);

	int approxResult = -1;
	approxResult = CCCoreLib::DistanceComputationTools::computeCloud2MeshDistances(m_compCloud,
																					&refMesh,
																					c2mParams,
																					&pDlg,
																					m_compOctree.data());

	sf->computeMinAndMax();
	auto maxDistance = sf->getMax();
	octreeLevel = qVoxFallTools::GetBestOctreeLevel(maxDistance,
													m_compCloud,
													&refMesh,
													m_compOctree,
													m_refOctree);
	m_compCloud->deleteScalarField(sfIdx);

	//does the cloud has already a temporary scalar field that we can use?
	sfIdx = m_compCloud->getScalarFieldIndexByName(TEMP_DISTANCES_SF_NAME);
	if (sfIdx < 0)
	{
		//we need to create a new scalar field
		sfIdx = m_compCloud->addScalarField(TEMP_DISTANCES_SF_NAME);
		if (sfIdx < 0)
		{
			ccLog::Error("Couldn't allocate a new scalar field for computing distances! Try to free some memory ...");
			return false;
		}
	}

	m_compCloud->setCurrentScalarField(sfIdx);
	sf = m_compCloud->getCurrentInScalarField();
	assert(sf);

	// Update parameters
	c2mParams.octreeLevel = static_cast<unsigned char>(octreeLevel);
	c2mParams.useDistanceMap = false;
	c2mParams.maxSearchDist = maxDistance;
	c2mParams.signedDistances = true;
	c2mParams.multiThread = true;
	c2mParams.maxThreadCount = threadCount;

	int result = -1;
	result = CCCoreLib::DistanceComputationTools::computeCloud2MeshDistances(m_compCloud,
																			&refMesh,
																			c2mParams,
																			&pDlg,
																			m_compOctree.data());
	sf->computeMinAndMax();

	return true;
}


bool qVoxFallTools::ConnectedComponents(ccPointCloud* cloud, int octreeLevel, ccProgressDialog pDlg, QString& errorMessage)
{
	ccOctree::Shared octree = cloud->getOctree();
	if (!octree)
	{
		octree = cloud->computeOctree(&pDlg);
		if (!octree)
		{
			errorMessage = "Failed to compute octree for connected components";
			return false;
		}
	}
	
	//we create/activate CCs label scalar field
	int sfIdx = cloud->getScalarFieldIndexByName(CONNECTED_COMPONENTS_SF_NAME);
	if (sfIdx < 0)
	{
		sfIdx = cloud->addScalarField(CONNECTED_COMPONENTS_SF_NAME);
	}
	if (sfIdx < 0)
	{
		errorMessage = "Couldn't allocate a new scalar field for computing CC labels";
		return false;
	}
	cloud->setCurrentScalarField(sfIdx);

	CCCoreLib::ReferenceCloudContainer components;
	int componentCount = CCCoreLib::AutoSegmentationTools::labelConnectedComponents(static_cast<CCCoreLib::GenericIndexedCloudPersist*>(cloud),
																					static_cast<unsigned char>(octreeLevel),
																					false,
																					&pDlg,
																					octree.data());
	//static_cast<ccPointCloud*>(cloud);
	cloud->getCurrentInScalarField()->computeMinAndMax();

	return true;
}


ccBBox qVoxFallTools::ScaleBBox(const ccBBox& bbox, float a)
{
	if (!bbox.isValid())
		return bbox;

	// Center of the bounding box
	CCVector3 center = (bbox.minCorner() + bbox.maxCorner()) * 0.5f;

	// Half size
	CCVector3 halfSize = (bbox.maxCorner() - bbox.minCorner()) * 0.5f;

	// Apply non-uniform scaling
	CCVector3 scaledHalfSize(
		halfSize.x * a,
		halfSize.y * a * 2.0f,  // double factor in Y
		halfSize.z * a
	);

	// New min and max
	CCVector3 newMin = center - scaledHalfSize;
	CCVector3 newMax = center + scaledHalfSize;

	return ccBBox(newMin, newMax, true);
}

ccMesh* qVoxFallTools::CropMeshByBBox(const ccMesh* mesh, const ccBBox& bbox)
{
	if (!mesh || !bbox.isValid())
		return nullptr;

	ccPointCloud* vertices = dynamic_cast<ccPointCloud*>(mesh->getAssociatedCloud());
	if (!vertices)
		return nullptr;

	std::vector<unsigned> indexMap(vertices->size(), -1); // map old -> new index
	ccPointCloud* newVertices = new ccPointCloud();

	for (unsigned i = 0; i < vertices->size(); ++i)
	{
		const CCVector3* P = vertices->getPoint(i);
		if (bbox.contains(*P))
		{
			indexMap[i] = newVertices->size();
			newVertices->addPoint(*P);
		}
	}

	ccMesh* newMesh = new ccMesh(newVertices);
	newMesh->reserve(vertices->size());

	for (unsigned i = 0; i < mesh->size(); ++i)
	{
		auto tri = mesh->getTriangleVertIndexes(i);
		unsigned i1 = tri->i1;
		unsigned i2 = tri->i2;
		unsigned i3 = tri->i3;

		if (indexMap[i1] != -1 && indexMap[i2] != -1 && indexMap[i3] != -1)
		{
			newMesh->addTriangle(indexMap[i1], indexMap[i2], indexMap[i3]);
		}
	}


	return newMesh;
}
