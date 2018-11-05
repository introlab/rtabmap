/*
Copyright (c) 2010-2016, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Universite de Sherbrooke nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "rtabmap/gui/PostProcessingDialog.h"
#include "ui_postProcessingDialog.h"

#include <QPushButton>
#include <QMessageBox>
#include <rtabmap/core/Optimizer.h>

namespace rtabmap {

PostProcessingDialog::PostProcessingDialog(QWidget * parent) :
	QDialog(parent)
{
	_ui = new Ui_PostProcessingDialog();
	_ui->setupUi(this);

	if(!Optimizer::isAvailable(Optimizer::kTypeCVSBA) && !Optimizer::isAvailable(Optimizer::kTypeG2O))
	{
		_ui->sba->setEnabled(false);
		_ui->sba->setChecked(false);
	}
	else if(!Optimizer::isAvailable(Optimizer::kTypeCVSBA))
	{
		_ui->comboBox_sbaType->setItemData(1, 0, Qt::UserRole - 1);
		_ui->comboBox_sbaType->setCurrentIndex(0);
	}
	else if(!Optimizer::isAvailable(Optimizer::kTypeG2O))
	{
		_ui->comboBox_sbaType->setItemData(0, 0, Qt::UserRole - 1);
		_ui->comboBox_sbaType->setCurrentIndex(1);
	}

	restoreDefaults();

	connect(_ui->buttonBox, SIGNAL(clicked(QAbstractButton *)), this, SLOT(closeDialog(QAbstractButton *)));

	connect(_ui->detectMoreLoopClosures, SIGNAL(clicked(bool)), this, SLOT(updateButtonBox()));
	connect(_ui->refineNeighborLinks, SIGNAL(stateChanged(int)), this, SLOT(updateButtonBox()));
	connect(_ui->refineLoopClosureLinks, SIGNAL(stateChanged(int)), this, SLOT(updateButtonBox()));
	connect(_ui->sba, SIGNAL(clicked(bool)), this, SLOT(updateButtonBox()));
	connect(_ui->buttonBox->button(QDialogButtonBox::RestoreDefaults), SIGNAL(clicked()), this, SLOT(restoreDefaults()));

	connect(_ui->detectMoreLoopClosures, SIGNAL(clicked(bool)), this, SIGNAL(configChanged()));
	connect(_ui->clusterRadius, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
	connect(_ui->clusterAngle, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
	connect(_ui->iterations, SIGNAL(valueChanged(int)), this, SIGNAL(configChanged()));
	connect(_ui->intraSession, SIGNAL(stateChanged(int)), this, SIGNAL(configChanged()));
	connect(_ui->interSession, SIGNAL(stateChanged(int)), this, SIGNAL(configChanged()));
	connect(_ui->refineNeighborLinks, SIGNAL(stateChanged(int)), this, SIGNAL(configChanged()));
	connect(_ui->refineLoopClosureLinks, SIGNAL(stateChanged(int)), this, SIGNAL(configChanged()));

	connect(_ui->sba, SIGNAL(clicked(bool)), this, SIGNAL(configChanged()));
	connect(_ui->sba_iterations, SIGNAL(valueChanged(int)), this, SIGNAL(configChanged()));
	connect(_ui->comboBox_sbaType, SIGNAL(currentIndexChanged(int)), this, SIGNAL(configChanged()));
	connect(_ui->comboBox_sbaType, SIGNAL(currentIndexChanged(int)), this, SLOT(updateVisibility()));
	connect(_ui->sba_rematchFeatures, SIGNAL(stateChanged(int)), this, SIGNAL(configChanged()));

	updateVisibility();
}

PostProcessingDialog::~PostProcessingDialog()
{
	delete _ui;
}

void PostProcessingDialog::closeDialog ( QAbstractButton * button )
{
	UDEBUG("");

	QDialogButtonBox::ButtonRole role = _ui->buttonBox->buttonRole(button);
	switch(role)
	{
	case QDialogButtonBox::RejectRole:
		this->reject();
		break;

	case QDialogButtonBox::AcceptRole:
		if(validateForm())
		{
			this->accept();
		}
		break;

	default:
		break;
	}
}

bool PostProcessingDialog::validateForm()
{
	if(_ui->detectMoreLoopClosures->isChecked() && !this->intraSession() && !this->interSession())
	{
		QMessageBox::warning(this, tr("Configuration error"), tr("Intra-session and inter-session parameters cannot be both disabled at the same time. Please select one (or both)."));
		return false;
	}
	return true;
}

void PostProcessingDialog::updateVisibility()
{
	_ui->sba_variance->setVisible(_ui->comboBox_sbaType->currentIndex() == 0);
	_ui->label_variance->setVisible(_ui->comboBox_sbaType->currentIndex() == 0);
}

void PostProcessingDialog::saveSettings(QSettings & settings, const QString & group) const
{
	if(!group.isEmpty())
	{
		settings.beginGroup(group);
	}
	settings.setValue("detect_more_lc", this->isDetectMoreLoopClosures());
	settings.setValue("cluster_radius", this->clusterRadius());
	settings.setValue("cluster_angle", this->clusterAngle());
	settings.setValue("iterations", this->iterations());
	settings.setValue("intra_session", this->intraSession());
	settings.setValue("inter_session", this->interSession());
	settings.setValue("refine_neigbors", this->isRefineNeighborLinks());
	settings.setValue("refine_lc", this->isRefineLoopClosureLinks());
	settings.setValue("sba", this->isSBA());
	settings.setValue("sba_iterations", this->sbaIterations());
	settings.setValue("sba_type", this->sbaType());
	settings.setValue("sba_variance", this->sbaVariance());
	settings.setValue("sba_rematch_features", this->sbaRematchFeatures());
	if(!group.isEmpty())
	{
		settings.endGroup();
	}
}

void PostProcessingDialog::loadSettings(QSettings & settings, const QString & group)
{
	if(!group.isEmpty())
	{
		settings.beginGroup(group);
	}
	this->setDetectMoreLoopClosures(settings.value("detect_more_lc", this->isDetectMoreLoopClosures()).toBool());
	this->setClusterRadius(settings.value("cluster_radius", this->clusterRadius()).toDouble());
	this->setClusterAngle(settings.value("cluster_angle", this->clusterAngle()).toDouble());
	this->setIterations(settings.value("iterations", this->iterations()).toInt());
	this->setIntraSession(settings.value("intra_session", this->intraSession()).toBool());
	this->setInterSession(settings.value("inter_session", this->interSession()).toBool());
	this->setRefineNeighborLinks(settings.value("refine_neigbors", this->isRefineNeighborLinks()).toBool());
	this->setRefineLoopClosureLinks(settings.value("refine_lc", this->isRefineLoopClosureLinks()).toBool());
	this->setSBA(settings.value("sba", this->isSBA()).toBool());
	this->setSBAIterations(settings.value("sba_iterations", this->sbaIterations()).toInt());
	this->setSBAType((Optimizer::Type)settings.value("sba_type", this->sbaType()).toInt());
	this->setSBAVariance(settings.value("sba_variance", this->sbaVariance()).toDouble());
	this->setSBARematchFeatures(settings.value("sba_rematch_features", this->sbaRematchFeatures()).toBool());

	if(!group.isEmpty())
	{
		settings.endGroup();
	}
}

void PostProcessingDialog::restoreDefaults()
{
	setDetectMoreLoopClosures(true);
	setClusterRadius(1);
	setClusterAngle(30);
	setIterations(5);
	setIntraSession(true);
	setInterSession(true);
	setRefineNeighborLinks(false);
	setRefineLoopClosureLinks(false);
	setSBA(false);
	setSBAIterations(20);
	setSBAType(!Optimizer::isAvailable(Optimizer::kTypeG2O)&&Optimizer::isAvailable(Optimizer::kTypeCVSBA)?Optimizer::kTypeCVSBA:Optimizer::kTypeG2O);
	setSBAVariance(1.0);
	setSBARematchFeatures(true);
}

void PostProcessingDialog::updateButtonBox()
{
	_ui->buttonBox->button(QDialogButtonBox::Ok)->setEnabled(
			isDetectMoreLoopClosures() || isRefineNeighborLinks() || isRefineLoopClosureLinks() || isSBA());
}

bool PostProcessingDialog::isDetectMoreLoopClosures() const
{
	return _ui->detectMoreLoopClosures->isChecked();
}

double PostProcessingDialog::clusterRadius() const
{
	return _ui->clusterRadius->value();
}

double PostProcessingDialog::clusterAngle() const
{
	return _ui->clusterAngle->value();
}

int PostProcessingDialog::iterations() const
{
	return _ui->iterations->value();
}

bool PostProcessingDialog::intraSession() const
{
	return _ui->intraSession->isChecked();
}

bool PostProcessingDialog::interSession() const
{
	return _ui->interSession->isChecked();
}

bool PostProcessingDialog::isRefineNeighborLinks() const
{
	return _ui->refineNeighborLinks->isChecked();
}

bool PostProcessingDialog::isRefineLoopClosureLinks() const
{
	return _ui->refineLoopClosureLinks->isChecked();
}

bool PostProcessingDialog::isSBA() const
{
	return _ui->sba->isChecked();
}

int PostProcessingDialog::sbaIterations() const
{
	return _ui->sba_iterations->value();
}
double PostProcessingDialog::sbaVariance() const
{
	return _ui->sba_variance->value();
}
Optimizer::Type PostProcessingDialog::sbaType() const
{
	return _ui->comboBox_sbaType->currentIndex()==0?Optimizer::kTypeG2O:Optimizer::kTypeCVSBA;
}
bool PostProcessingDialog::sbaRematchFeatures() const
{
	return _ui->sba_rematchFeatures->isChecked();
}

//setters
void PostProcessingDialog::setDetectMoreLoopClosures(bool on)
{
	_ui->detectMoreLoopClosures->setChecked(on);
}
void PostProcessingDialog::setClusterRadius(double radius)
{
	_ui->clusterRadius->setValue(radius);
}
void PostProcessingDialog::setClusterAngle(double angle)
{
	_ui->clusterAngle->setValue(angle);
}
void PostProcessingDialog::setIterations(int iterations)
{
	_ui->iterations->setValue(iterations);
}
void PostProcessingDialog::setIntraSession(bool enabled)
{
	_ui->intraSession->setChecked(enabled);
}
void PostProcessingDialog::setInterSession(bool enabled)
{
	_ui->interSession->setChecked(enabled);
}
void PostProcessingDialog::setRefineNeighborLinks(bool on)
{
	_ui->refineNeighborLinks->setChecked(on);
}
void PostProcessingDialog::setRefineLoopClosureLinks(bool on)
{
	_ui->refineLoopClosureLinks->setChecked(on);
}
void PostProcessingDialog::setSBA(bool on)
{
	_ui->sba->setChecked((Optimizer::isAvailable(Optimizer::kTypeCVSBA) || Optimizer::isAvailable(Optimizer::kTypeG2O)) && on);
}
void PostProcessingDialog::setSBAIterations(int iterations)
{
	_ui->sba_iterations->setValue(iterations);
}
void PostProcessingDialog::setSBAVariance(double variance)
{
	_ui->sba_variance->setValue(variance);
}
void PostProcessingDialog::setSBAType(Optimizer::Type type)
{
	if(type == Optimizer::kTypeCVSBA)
	{
		_ui->comboBox_sbaType->setCurrentIndex(1);
	}
	else
	{
		_ui->comboBox_sbaType->setCurrentIndex(0);
	}
}
void PostProcessingDialog::setSBARematchFeatures(bool value)
{
	_ui->sba_rematchFeatures->setChecked(value);
}


}
