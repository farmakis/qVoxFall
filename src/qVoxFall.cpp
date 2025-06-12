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


#include "qVoxFall.h"

//Qt
#include <QMainWindow>
#include <QMessageBox>

//local
#include "qVoxFallDialog.h"
#include "qVoxFallDisclaimerDialog.h"
#include "qVoxFallProcess.h"

//qCC_db
#include <ccMesh.h>



qVoxFall::qVoxFall( QObject *parent )
	: QObject( parent )
	, ccStdPluginInterface( ":/CC/plugin/qVoxFall/info.json" )
	, m_action( nullptr )
{
}

void qVoxFall::onNewSelection( const ccHObject::Container &selectedEntities )
{
	if ( m_action )
	{
		m_action->setEnabled( selectedEntities.size() == 1
							&& selectedEntities[0]->isA(CC_TYPES::MESH) );
	}
	
	m_selectedEntities = selectedEntities;
}

QList<QAction *> qVoxFall::getActions()
{
	if ( !m_action )
	{
		m_action = new QAction( getName(), this );
		m_action->setToolTip( getDescription() );
		m_action->setIcon( getIcon() );
		
		// Connect signal
		connect( m_action, &QAction::triggered, this, &qVoxFall::doAction );
	}

	return { m_action };
}


void qVoxFall::doAction()
{
	//disclaimer accepted?
	if (!DisclaimerDialog::show(m_app))
		return;
	
	//m_app should have already been initialized by CC when plugin is loaded!
	assert(m_app);
	if (!m_app)
		return;

	if (m_selectedEntities.size() != 1
		|| !m_selectedEntities[0]->isA(CC_TYPES::MESH))
	{
		m_app->dispToConsole("Select two meshes !", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}

	ccMesh* mesh = ccHObjectCaster::ToMesh(m_selectedEntities[0]);


	m_app->dispToConsole("[VoxFall] Meshes loaded successfully", ccMainAppInterface::STD_CONSOLE_MESSAGE);

	//display dialog
	qVoxFallDialog dlg(mesh, m_app);
	if (!dlg.exec())
	{
		//process cancelled by the user
		return;
	}

	//if "generate report" is selected, check if the output filepath already exists
	if (dlg.getGenerateReportActivation())
	{
		QString filename = dlg.destinationPathLineEdit->text();
		QFile outFile(filename);
		if (outFile.exists())
		{
			//if the file already exists, ask for confirmation!
			if (QMessageBox::warning(m_app->getMainWindow(),
				"Overwrite",
				"[VoxFall] File already exists! Are you sure you want to overwrite it?",
				QMessageBox::Yes,
				QMessageBox::No) == QMessageBox::No)
				return;
		}
	}

	////display the voxel size in console
	m_app->dispToConsole(QString("[VoxFall] Voxel size: %1 m").arg(dlg.getVoxelSize()), ccMainAppInterface::STD_CONSOLE_MESSAGE);

	QString errorMessage;
	if (!qVoxFallProcess::Compute(dlg, errorMessage, true, m_app->getMainWindow(), m_app))
	{
		if (!errorMessage.isEmpty())
		{
			m_app->dispToConsole(errorMessage, ccMainAppInterface::WRN_CONSOLE_MESSAGE);
		}
		else
		{
			mesh->setEnabled(false);
			m_app->dispToConsole("[VoxFall] Completed!", ccMainAppInterface::STD_CONSOLE_MESSAGE);
		}	
	}

	//'Compute' may change some parameters of the dialog
	dlg.saveParamsToPersistentSettings();
}
