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

#ifndef Q_VOXFALL_PLUGIN_HEADER
#define Q_VOXFALL_PLUGIN_HEADER

#include "ccStdPluginInterface.h"


//qCC_db
#include <ccHObject.h>


class qVoxFall : public QObject, public ccStdPluginInterface
{
	Q_OBJECT
	Q_INTERFACES( ccPluginInterface ccStdPluginInterface )
	Q_PLUGIN_METADATA( IID "cccorp.cloudcompare.plugin.qVoxFall" FILE "../info.json" )

public:
	//! Default constructor
	qVoxFall(QObject* parent = nullptr);

	virtual ~qVoxFall() = default;

	// Inherited from ccStdPluginInterface
	virtual void onNewSelection( const ccHObject::Container &selectedEntities ) override;
	virtual QList<QAction *> getActions() override;
	virtual void registerCommands(ccCommandLineInterface* cmd) override;

private:
	
	void doAction();

	//! Default action
	QAction* m_action;

	//! Currently selected entities
	ccHObject::Container m_selectedEntities;
};

#endif //Q_VOXFALL_PLUGIN_HEADER
