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

#ifndef Q_VOXFALL_CLUSTER_HEADER
#define Q_VOXFALL_CLUSTER_HEADER

//CCCoreLib
#include <GenericProgressCallback.h>

//local
#include "qVoxFallDialog.h"

//qCC_db
#include <ccMesh.h>
#include <ccPointCloud.h>
#include <ccScalarField.h>
#include <ccBox.h>
#include <ccGLMatrix.h>


#include <unordered_map>


class qVoxFallTransform
{
public:
	ccGLMatrix matrix;
	ccGLMatrix inverse;

	qVoxFallTransform(double dip, double azimuth)
	{
		float zRot = azimuth * 3.14159 / 180;
		float xRot = (90 - dip) * 3.14159 / 180;

		ccGLMatrix zRotMatrix;
		const Vector3Tpl<float> zX(std::cos(zRot), std::sin(zRot), 0);
		const Vector3Tpl<float> zY(-std::sin(zRot), std::cos(zRot), 0);
		const Vector3Tpl<float> zZ(0, 0, 1);
		const Vector3Tpl<float> zTr(0, 0, 0);
		zRotMatrix = ccGLMatrix(zX, zY, zZ, zTr);

		ccGLMatrix xRotMatrix;
		const Vector3Tpl<float> xX(1, 0, 0);
		const Vector3Tpl<float> xY(0, std::cos(xRot), -std::sin(xRot));
		const Vector3Tpl<float> xZ(0, std::sin(xRot), std::cos(xRot));
		const Vector3Tpl<float> xTr(0, 0, 0);
		xRotMatrix = ccGLMatrix(xX, xY, xZ, xTr);

		matrix = zRotMatrix * xRotMatrix;
		inverse = matrix.inverse();
	}

};


class qVoxFallCluster
{
public:	

	/*  constructor, destructor  */
	qVoxFallCluster(int label, int comp_assign, uint32_t first_node, uint32_t last_node, float voxelSize);
	~qVoxFallCluster();


	/* the 'get' methods */
	int GetLabel() const { return label; }
	int GetSize() const { return last_node - first_node; }
	int GetVoxel(int index) const { return first_node + index; }
	int GetSurfaceVoxelCount() const { return surfaceVoxelCount; }
	float GetUncertainty() const { return pow(voxelSize, 3) * GetSurfaceVoxelCount() / 2; }
	float GetVolume() const { return pow(voxelSize, 3) * GetSize() + GetUncertainty(); }
	int GetChangeType() const { return changeType; }
	ccMesh* GetClusterMesh();
	ccPointCloud* GetClusterCloud() const { return clusterCloud; }
	ccBBox GetBB() const { return clusterCloud->getOwnBB(); }


	/*  methods for manipulating parameters  */
	void IncrementSurfaceVoxelCount() { surfaceVoxelCount++; }
	void SetChangeType(int changeType) { this->changeType = changeType; }
	void AddVoxelMesh(CCVector3 V, int voxel, qVoxFallTransform* transform);
	void InitializeClusterMesh();

protected:

	int label;
	int comp_assign;
	uint32_t first_node;
	uint32_t last_node;
	int surfaceVoxelCount;
	int changeType;
	float voxelSize;

	ccPointCloud* clusterCloud; 
	ccMesh* clusterMesh; 

private:
	ccBox* CreateVoxelMesh(CCVector3 V, int voxelIdx);
};

#endif //Q_VOXFALL_CLUSTER_HEADER