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

#ifndef Q_VOXFALL_TOOLS_HEADER
#define Q_VOXFALL_TOOLS_HEADER

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

	static ccBox* CreateVoxelMesh(CCVector3 V, float voxelSize, int voxelIdx);
};


class qVoxFallTools
{
public:

	static std::vector<Tuple3i> FindAdjacents(Tuple3i V, Tuple3i steps, bool facetsOnly);

	static int Grid2Index(Tuple3i n, Tuple3i steps);

	static Tuple3i Index2Grid(unsigned index, Tuple3i steps);

};


#endif //Q_VOXFALL_PROCESS_HEADER
