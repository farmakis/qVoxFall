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


#include "qVoxFallCluster.h"

//qCC_db
#include <ccPointCloud.h>

//qCC
#include <ccMainAppInterface.h>
#include <ccQtHelpers.h>

//local
#include "qVoxFallDialog.h"

//Qt
#include <QtCore>
#include <QApplication>
#include <QMainWindow>
#include <QProgressDialog>
#include <QtConcurrentMap>


qVoxFallCluster::qVoxFallCluster(int label, int comp_assign, uint32_t first_node, uint32_t last_node, float voxelSize)
{
	this->label = label;
	this->comp_assign = comp_assign;
	this->first_node = first_node;
	this->last_node = last_node;
	this->voxelSize = voxelSize;
	this->surfaceVoxelCount = 0;

	InitializeClusterMesh();
}
qVoxFallCluster::~qVoxFallCluster(){}


void qVoxFallCluster::InitializeClusterMesh()
{
	clusterCloud = new ccPointCloud("Vertices");
	clusterMesh = new ccMesh(clusterCloud);
}


ccMesh* qVoxFallCluster::GetClusterMesh()
{
	clusterMesh->setName(QString("Cluster#%1 - (v: %2 m3)").arg(GetLabel()).arg(GetVolume()));
	clusterMesh->computePerVertexNormals();
	clusterCloud->resize(clusterCloud->size());
	clusterMesh->addChild(clusterCloud);
	return clusterMesh;
}


void qVoxFallCluster::AddVoxelMesh(CCVector3 V, int voxel, qVoxFallTransform* transform)
{
	auto voxelMesh = CreateVoxelMesh(V, voxel);
	ccPointCloud* voxelCloud = dynamic_cast<ccPointCloud*>(voxelMesh->getAssociatedCloud());
	voxelCloud->applyGLTransformation_recursive(&transform->inverse);

	// we append voxel vertices in the cluster cloud;
	unsigned vertCount = clusterCloud->size();
	clusterCloud->append(voxelCloud, clusterCloud->size());

	// we add triangles from the voxel mesh to the cluster mesh
	for (unsigned i = 0; i < voxelMesh->size(); ++i)
	{
		auto tri = voxelMesh->getTriangleVertIndexes(i);
		clusterMesh->addTriangle(tri->i1+vertCount, tri->i2+vertCount, tri->i3+vertCount);
	}
}


ccBox* qVoxFallCluster::CreateVoxelMesh(CCVector3 V, int voxelIdx)
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
