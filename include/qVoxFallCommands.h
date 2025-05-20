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

#ifndef Q_VOXFALL_COMMANDS_HEADER
#define Q_VOXFALL_COMMANDS_HEADER

//CloudCompare
#include "ccCommandLineInterface.h"

//Local
#include "qVoxFallProcess.h"

static const char COMMAND_VOXFALL[] = "VOXFALL";

struct CommandVoxFall : public ccCommandLineInterface::Command
{
	CommandVoxFall() : ccCommandLineInterface::Command("VOXFALL", COMMAND_VOXFALL) {}

	virtual bool process(ccCommandLineInterface& cmd) override
	{
		cmd.print("[VOXFALL]");
		if (cmd.arguments().empty())
		{
			return cmd.error(QString("Missing parameter: parameters filename after \"-%1\"").arg(COMMAND_VOXFALL));
		}

		if (cmd.meshes().size() < 2)
		{
			cmd.error("Not enough mesh models loaded (2 are expected: mesh 1, mesh 2)");
			return false;
		}

		//ccMesh* mesh1 = ccHObjectCaster::ToMesh(cmd.meshes()[0].mesh);
		//ccMesh* mesh2 = ccHObjectCaster::ToMesh(cmd.meshes()[1].mesh);

		////display dialog
		//qVoxFallDialog dlg(mesh1, mesh2, nullptr);

		//QString errorMessage;
		//ccPointCloud* outputCloud = nullptr; //only necessary for the command line version in fact
		//if (!qVoxFallProcess::Compute(dlg, errorMessage, !cmd.silentMode(), cmd.widgetParent()))
		//{
		//	return cmd.error(errorMessage);
		//}
	}
};

#endif //Q_VOXFALL_PLUGIN_COMMANDS_HEADER
