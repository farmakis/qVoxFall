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

#include <FileIOFilter.h>
#include <ccMesh.h>
#include <ccHObjectCaster.h>
#include <ccHObject.h>
#include <QFileInfo>

//CloudCompare
#include "ccCommandLineInterface.h"

//Local
#include "qVoxFallDialog.h"
#include "qVoxFallProcess.h"

static const char COMMAND_VOXFALL[] = "VOXFALL";
static const char COMMAND_VOXFALL_VOXEL_SIZE[] = "VOXEL_SIZE";
static const char COMMAND_VOXFALL_DIP[] = "DIP";
static const char COMMAND_VOXFALL_DIP_DIR[] = "DIP_DIR";
static const char COMMAND_VOXFALL_GENERATE_REPORT[] = "GENERATE_REPORT";
static const char COMMAND_VOXFALL_REPORT_PATH[] = "REPORT_PATH";
static const char COMMAND_VOXFALL_EXPORT_CLUSTERS[] = "EXPORT_CLUSTERS";
static const char COMMAND_VOXFALL_LOSS_GAIN[] = "LOSS_GAIN";

struct CommandVOXFALL : public ccCommandLineInterface::Command
{
	CommandVOXFALL() : ccCommandLineInterface::Command("VOXFALL", COMMAND_VOXFALL) {}

	virtual bool process(ccCommandLineInterface& cmd) override
	{
		if (cmd.arguments().empty())
		{
			return cmd.error(QString("Missing parameter: parameters filename after \"-%1\"").arg(COMMAND_VOXFALL));
		}

		// initialize settings
		bool exportMeshesEnabled = false;
		bool generateReportEnabled = false;
		bool lossGainEnabled = false;
		QSettings settings("qVoxFall");
		settings.setValue("ExportMeshesEnabled", exportMeshesEnabled);
		settings.setValue("GenerateReportEnabled", generateReportEnabled);

		while (!cmd.arguments().empty())
		{
			const QString& ARGUMENT = cmd.arguments().front();
			if (ccCommandLineInterface::IsCommand(ARGUMENT, COMMAND_VOXFALL_VOXEL_SIZE))
			{
				cmd.arguments().pop_front();
				bool conv = false;
				double voxelSize = cmd.arguments().takeFirst().toDouble(&conv);
				settings.setValue("VoxelSize", voxelSize);
				if (!conv)
				{
					return cmd.error(QObject::tr("Invalid parameter: value after \"-%1\"").arg(COMMAND_VOXFALL_VOXEL_SIZE));
				}
				cmd.print(QString("Voxel size: %1 m").arg(voxelSize));
			}
			else if (ccCommandLineInterface::IsCommand(ARGUMENT, COMMAND_VOXFALL_DIP))
			{
				cmd.arguments().pop_front();
				bool conv = false;
				double dip = cmd.arguments().takeFirst().toDouble(&conv);
				settings.setValue("Dip", dip);
				if (!conv)
				{
					return cmd.error(QObject::tr("Invalid parameter: value after \"-%1\"").arg(COMMAND_VOXFALL_DIP));
				}
				cmd.print(QString("Dip: %1 degrees").arg(dip));
			}
			else if (ccCommandLineInterface::IsCommand(ARGUMENT, COMMAND_VOXFALL_DIP_DIR))
			{
				cmd.arguments().pop_front();
				bool conv = false;
				double dipdir = cmd.arguments().takeFirst().toDouble(&conv);
				settings.setValue("DipDir", dipdir);
				if (!conv)
				{
					return cmd.error(QObject::tr("Invalid parameter: value after \"-%1\"").arg(COMMAND_VOXFALL_DIP_DIR));
				}
				cmd.print(QString("Dip direction: %1 degrees").arg(dipdir));
			}
			else if (ccCommandLineInterface::IsCommand(ARGUMENT, COMMAND_VOXFALL_GENERATE_REPORT))
			{
				cmd.arguments().pop_front();
				cmd.print("Report will be generated");
				generateReportEnabled = true;
				settings.setValue("GenerateReportEnabled", generateReportEnabled);
			}
			else if (ccCommandLineInterface::IsCommand(ARGUMENT, COMMAND_VOXFALL_REPORT_PATH))
			{
				cmd.arguments().pop_front();
				QString reportPath = cmd.arguments().takeFirst();
				settings.setValue("ReportPath", reportPath);
			}
			else if (ccCommandLineInterface::IsCommand(ARGUMENT, COMMAND_VOXFALL_EXPORT_CLUSTERS))
			{
				cmd.arguments().pop_front();
				cmd.print("Block meshes will be generated as .bin");
				exportMeshesEnabled = true;
				settings.setValue("ExportMeshesEnabled", exportMeshesEnabled);
			}
			else if (ccCommandLineInterface::IsCommand(ARGUMENT, COMMAND_VOXFALL_LOSS_GAIN))
			{
				cmd.arguments().pop_front();
				cmd.print("Loss/gain scalar field will be generated");
				lossGainEnabled = true;
				settings.setValue("LossGainEnabled", lossGainEnabled);
			}
			else
			{
				cmd.print("Set input parameters");
				break;
			}
		}

		if (cmd.meshes().size() < 2)
		{
			cmd.error("Not enough mesh models loaded (2 are expected: mesh 1, mesh 2)");
			return false;
		}

		ccMesh* mesh1 = ccHObjectCaster::ToMesh(cmd.meshes()[0].mesh);
		ccMesh* mesh2 = ccHObjectCaster::ToMesh(cmd.meshes()[1].mesh);

		//display dialog
		qVoxFallDialog dlg(mesh1, mesh2, nullptr);
		dlg.loadParamsFrom(settings);

		QString errorMessage;
		ccPointCloud* outputCloud = nullptr; //only necessary for the command line version in fact
		ccHObject* outputGroup = nullptr; //only necessary for the command line version in fact
		if (!qVoxFallProcess::Compute(dlg, errorMessage, outputCloud, outputGroup, !cmd.silentMode(), cmd.widgetParent()))
		{
			if (!errorMessage.isEmpty())
			{
				return cmd.error(errorMessage);
			}
		}

		auto v = settings.value("VoxelSize").toDouble();
		QString prefixTemplate = "%1_to_%2 [VoxFall] (voxel %3m)";
		QString prefix = prefixTemplate.arg(cmd.meshes().front().basename)
										.arg(cmd.meshes().back().basename)
										.arg(v);
		// store VoxFall labeled grid as point cloud
		if (outputCloud)
		{
			CLCloudDesc cloudDesc(outputCloud, prefix, cmd.meshes().front().path);
			if (cmd.autoSaveMode())
			{
				QString errorCStr = cmd.exportEntity(cloudDesc, QString(), 0, ccCommandLineInterface::ExportOption::ForceNoTimestamp);
				if (!errorCStr.isEmpty())
				{
					return cmd.error(errorCStr);
				}
			}
		}

		// store VoxFall clusters mesh objects as group in .bin
		if (outputGroup)
		{
			if (exportMeshesEnabled)
			{
				CLGroupDesc groupDesc(outputGroup, prefix, cmd.meshes().front().path);
				QString errorGStr = cmd.exportEntity(groupDesc, QString(), nullptr, ccCommandLineInterface::ExportOption::ForceNoTimestamp);
				if (!errorGStr.isEmpty())
				{
					return cmd.error(errorGStr);
				}
			}
		}

		return true;
	}
};

#endif //Q_VOXFALL_PLUGIN_COMMANDS_HEADER
