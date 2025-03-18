//##########################################################################
//#                                                                        #
//#                     CLOUDCOMPARE PLUGIN: qVoxFall                      #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 of the License.               #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#                 COPYRIGHT: THE UNIVERSITY OF NEWCASTLE                 #
//#                                                                        #
//##########################################################################

#include "qVoxFallDialog.h"

//CCPluginAPI
#include <ccMainAppInterface.h>
#include <ccQtHelpers.h>

//qCC_db
#include <ccMesh.h>
#include <ccPlane.h>
#include <ccFileUtils.h>

//Qt
#include <QMainWindow>
#include <QComboBox>
#include <QFileDialog>

//other
#include <ccNormalVectors.h>
#include <Neighbourhood.h>

#define RAD2DEG (180.0 / M_PI)


/*** HELPERS ***/
static QString GetEntityName(ccHObject* obj)
{
	if (!obj)
	{
		assert(false);
		return QString();
	}

	QString name = obj->getName();
	if (name.isEmpty())
		name = "unnamed";
	name += QString(" [ID %1]").arg(obj->getUniqueID());

	return name;
}

static ccMesh* GetMeshFromCombo(QComboBox* comboBox, ccHObject* dbRoot)
{
	assert(comboBox && dbRoot);
	if (!comboBox || !dbRoot)
	{
		assert(false);
		return nullptr;
	}

	//return the mesh currently selected in the combox box
	int index = comboBox->currentIndex();
	if (index < 0)
	{
		assert(false);
		return nullptr;
	}
	assert(comboBox->itemData(index).isValid());
	unsigned uniqueID = comboBox->itemData(index).toUInt();
	ccHObject* item = dbRoot->find(uniqueID);
	if (!item || !item->isA(CC_TYPES::MESH))
	{
		assert(false);
		return nullptr;
	}
	return static_cast<ccMesh*>(item);
}

/*** HELPERS (END) ***/

qVoxFallDialog::qVoxFallDialog(ccMesh* mesh1, ccMesh* mesh2, ccMainAppInterface* app)
	: QDialog(app ? app->getMainWindow() : nullptr)
	, Ui::VoxFallDialog()
	, m_app(app)
	, m_mesh1(nullptr)
	, m_mesh2(nullptr)
{
	setupUi(this);

	connect(showMesh1CheckBox,		&QAbstractButton::toggled,	this, &qVoxFallDialog::setMesh1Visibility);
	connect(showMesh2CheckBox,		&QAbstractButton::toggled,	this, &qVoxFallDialog::setMesh2Visibility);

	connect(swapMeshesToolButton,	&QAbstractButton::clicked,	this, &qVoxFallDialog::swapMeshes);

	connect(browseToolButton, &QAbstractButton::clicked, this, &qVoxFallDialog::browseDestination);

	connect(autoAzimuthButton, &QAbstractButton::clicked, this, &qVoxFallDialog::autoComputeAzimuth);

	setMeshes(mesh1, mesh2);

	loadParamsFromPersistentSettings();
}

void qVoxFallDialog::swapMeshes()
{
	setMeshes(m_mesh2, m_mesh1);
}

void qVoxFallDialog::setMeshes(ccMesh* mesh1, ccMesh* mesh2)
{
	if (!mesh1 || !mesh2)
	{
		assert(false);
		return;
	}

	m_mesh1 = mesh1;
	m_mesh2 = mesh2;

	//mesh #1
	mesh1LineEdit->setText(GetEntityName(mesh1));
	showMesh1CheckBox->blockSignals(true);
	showMesh1CheckBox->setChecked(mesh1->isVisible());
	showMesh1CheckBox->blockSignals(false);

	//mesh #2
	mesh2LineEdit->setText(GetEntityName(mesh2));
	showMesh2CheckBox->blockSignals(true);
	showMesh2CheckBox->setChecked(mesh2->isVisible());
	showMesh2CheckBox->blockSignals(false);
}

void qVoxFallDialog::setMesh1Visibility(bool state)
{
	if (m_mesh1)
	{
		m_mesh1->setVisible(state);
		m_mesh1->prepareDisplayForRefresh();
	}
	if (m_app)
	{
		m_app->refreshAll();
		m_app->updateUI();
	}
}

void qVoxFallDialog::setMesh2Visibility(bool state)
{
	if (m_mesh2)
	{
		m_mesh2->setVisible(state);
		m_mesh2->prepareDisplayForRefresh();
	}
	if (m_app)
	{
		m_app->refreshAll();
		m_app->updateUI();
	}
}

double qVoxFallDialog::getVoxelSize() const
{
	double voxelSize = voxelSizeDoubleSpinBox->value();
	return voxelSize;
}

double qVoxFallDialog::getAzimuth() const
{
	double azimuth = azDoubleSpinBox->value();
	return azimuth;
}

bool qVoxFallDialog::getGenerateReportActivation() const
{
	return generateReportBox->isChecked();
}

void qVoxFallDialog::browseDestination()
{
	QString fileType;
	fileType = "ASCII table (*.csv)";

	//open file saving dialog
	QString outputFilename = QFileDialog::getSaveFileName(nullptr, "Select destination", destinationPathLineEdit->text(), fileType);

	if (outputFilename.isEmpty())
		return;

	destinationPathLineEdit->setText(outputFilename);
}

bool qVoxFallDialog::getExportMeshesActivation() const
{
	return exportCheckBox->isChecked();
}

bool qVoxFallDialog::getLossGainActivation() const
{
	return lossCheckBox->isChecked();
}

int qVoxFallDialog::getMaxThreadCount() const
{
	return QThread::idealThreadCount();
}

void qVoxFallDialog::loadParamsFromPersistentSettings()
{
	QSettings settings("qVoxFall");
	loadParamsFrom(settings);
}

void qVoxFallDialog::loadParamsFrom(const QSettings& settings)
{
	//read parameters
	double voxelSize = settings.value("VoxelSize", voxelSizeDoubleSpinBox->value()).toDouble();
	double azimuth = settings.value("Azimuth", azDoubleSpinBox->value()).toDouble();
	bool generateReportEnabled = settings.value("GenerateReportEnabled", generateReportBox->isChecked()).toBool();
	QString reportPath = settings.value("ReportPath", ccFileUtils::defaultDocPath()).toString();
	//for the first time it is launched
	if (!reportPath.endsWith(".csv"))
	{
		reportPath = QFileInfo(reportPath).absolutePath() + QString("/VoxFall-report.csv");
	}
	bool exportMeshesEnabled = settings.value("ExportMeshesEnabled", exportCheckBox->isChecked()).toBool();
	bool lossGainEnabled = settings.value("LossGainEnabled", lossCheckBox->isChecked()).toBool();

	//apply parameters
	voxelSizeDoubleSpinBox->setValue(voxelSize);
	azDoubleSpinBox->setValue(azimuth);
	generateReportBox->setChecked(generateReportEnabled);
	destinationPathLineEdit->setText(reportPath);
	exportCheckBox->setChecked(exportMeshesEnabled);
	lossCheckBox->setChecked(lossGainEnabled);
}

void qVoxFallDialog::saveParamsToPersistentSettings()
{
	QSettings settings("qVoxFall");
	saveParamsTo(settings);
}

void qVoxFallDialog::saveParamsTo(QSettings& settings)
{
	//save parameters
	settings.setValue("VoxelSize", voxelSizeDoubleSpinBox->value());
	settings.setValue("Azimuth", azDoubleSpinBox->value());
	settings.setValue("GenerateReportEnabled", generateReportBox->isChecked());
	settings.setValue("ReportPath", destinationPathLineEdit->text());
	settings.setValue("ExportMeshesEnabled", exportCheckBox->isChecked());
	settings.setValue("LossGainEnabled", lossCheckBox->isChecked());
}

void qVoxFallDialog::autoComputeAzimuth()
{

	//check if there is an already fitted plane and get the dip direction
	for (unsigned i = 0; i < m_mesh1->getChildrenNumber(); ++i)
	{
		ccHObject* child = m_mesh1->getChild(i);
		if (child && child->isA(CC_TYPES::PLANE))
		{
			ccPlane* plane = static_cast<ccPlane*>(child);
			CCVector3 N = plane->getNormal();

			//we compute strike & dip by the way
			PointCoordinateType dip = 0.0f;
			PointCoordinateType dipDir = 0.0f;
			ccNormalVectors::ConvertNormalToDipAndDipDir(N, dip, dipDir);
			QString dipAndDipDirStr = ccNormalVectors::ConvertDipAndDipDirToString(dip, dipDir);
			m_app->dispToConsole("[VoxFall] Azimuth estimation: From existing plane");
			m_app->dispToConsole(QString("\t- %1").arg(dipAndDipDirStr));
			azDoubleSpinBox->setValue(dipDir);
			return;
		}
	}

	ccShiftedObject* shifted = nullptr;
	CCCoreLib::GenericIndexedCloudPersist* cloud = nullptr;

	ccGenericPointCloud* gencloud = ccHObjectCaster::ToGenericPointCloud(m_mesh1);
	if (gencloud)
	{
		cloud = static_cast<CCCoreLib::GenericIndexedCloudPersist*>(gencloud);
		shifted = gencloud;
	}

	if (cloud)
	{
		double rms = 0.0;
		CCVector3 C;
		CCVector3 N;
		ccHObject* plane = nullptr;

		ccPlane* pPlane = ccPlane::Fit(cloud, &rms);
		if (pPlane)
		{
			plane = static_cast<ccHObject*>(pPlane);
			N = pPlane->getNormal();
			C = *CCCoreLib::Neighbourhood(cloud).getGravityCenter();
			pPlane->enableStippling(true);

			if (shifted)
			{
				pPlane->copyGlobalShiftAndScale(*shifted);
			}
		}

		if (plane)
		{
			m_app->dispToConsole(tr("[VoxFall] Azimuth estimation: Entity '%1'").arg(m_mesh1->getName()));
			m_app->dispToConsole(tr("\t- plane fitting RMS: %1").arg(rms));

			//We always consider the normal with a positive 'Z' by default!
			if (N.z < 0.0)
				N *= -1.0;
			m_app->dispToConsole(tr("\t- normal: (%1, %2, %3)").arg(N.x).arg(N.y).arg(N.z));

			//we compute strike & dip by the way
			PointCoordinateType dip = 0.0f;
			PointCoordinateType dipDir = 0.0f;
			ccNormalVectors::ConvertNormalToDipAndDipDir(N, dip, dipDir);
			QString dipAndDipDirStr = ccNormalVectors::ConvertDipAndDipDirToString(dip, dipDir);
			m_app->dispToConsole(QString("\t- %1").arg(dipAndDipDirStr));
			azDoubleSpinBox->setValue(dipDir);

			//hack: output the transformation matrix that would make this normal points towards +Z
			ccGLMatrix makeZPosMatrix = ccGLMatrix::FromToRotation(N, CCVector3(0, 0, CCCoreLib::PC_ONE));
			CCVector3 Gt = C;
			makeZPosMatrix.applyRotation(Gt);
			makeZPosMatrix.setTranslation(C - Gt);
			
			plane->setName(dipAndDipDirStr);
			plane->applyGLTransformation_recursive(); //not yet in DB
			plane->setVisible(true);
			plane->setSelectionBehavior(ccHObject::SELECTION_FIT_BBOX);

			m_mesh1->addChild(plane);
			plane->setDisplay(m_mesh1->getDisplay());
			plane->prepareDisplayForRefresh_recursive();
			m_app->addToDB(plane);
		}
		else
		{
			m_app->dispToConsole(tr("Failed to fit a plane/facet on entity '%1'").arg(m_mesh1->getName()));
		}
	}
	
	m_app->refreshAll();
	m_app->updateUI();
}
