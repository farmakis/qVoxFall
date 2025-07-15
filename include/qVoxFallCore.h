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

#ifndef Q_VOXFALL_CORES_HEADER
#define Q_VOXFALL_CORE_HEADER

//Local
#include "qVoxFallTools.h"
#include "qVoxFallDialog.h"

//qCC
#include "ccPointCloud.h"
#include "ccMesh.h"

class qVoxFallCore
{
public:

	static bool Run(ccMesh* mesh,
					ccBBox* bb,
					int sign,
					qVoxFallTransform transform,
					int& patchFirstLabel,
					int maxThreadCount,
					QString& errorMessage,
					ccPointCloud* outputCloud,
					ccHObject* outputGroup,
					const qVoxFallDialog& dlg,
					ccProgressDialog& pDlg);

};

#endif //Q_VOXFALL_CORE_HEADER
