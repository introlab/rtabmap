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

#include "rtabmap/gui/ExportDialog.h"
#include "ui_exportDialog.h"

#include <QFileDialog>
#include <QPushButton>

namespace rtabmap {

ExportDialog::ExportDialog(QWidget * parent) :
	QDialog(parent)
{
	_ui = new Ui_ExportDialog();
	_ui->setupUi(this);

	connect(_ui->toolButton_path, SIGNAL(clicked()), this, SLOT(getPath()));

	restoreDefaults();
	connect(_ui->buttonBox->button(QDialogButtonBox::RestoreDefaults), SIGNAL(clicked()), this, SLOT(restoreDefaults()));

	connect(_ui->spinBox_ignored, SIGNAL(valueChanged(int)), this, SIGNAL(configChanged()));
	connect(_ui->doubleSpinBox_framerate, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
	connect(_ui->spinBox_session, SIGNAL(valueChanged(int)), this, SIGNAL(configChanged()));
	connect(_ui->checkBox_rgb, SIGNAL(stateChanged(int)), this, SIGNAL(configChanged()));
	connect(_ui->checkBox_depth, SIGNAL(stateChanged(int)), this, SIGNAL(configChanged()));
	connect(_ui->checkBox_depth2d, SIGNAL(stateChanged(int)), this, SIGNAL(configChanged()));
	connect(_ui->checkBox_odom, SIGNAL(stateChanged(int)), this, SIGNAL(configChanged()));
	connect(_ui->checkBox_userData, SIGNAL(stateChanged(int)), this, SIGNAL(configChanged()));

	_ui->lineEdit_path->setText(QDir::currentPath()+QDir::separator()+"output.db");
}

ExportDialog::~ExportDialog()
{
	delete _ui;
}

void ExportDialog::saveSettings(QSettings & settings, const QString & group) const
{
	if(!group.isEmpty())
	{
		settings.beginGroup(group);
	}
	settings.setValue("framesIgnored", this->framesIgnored());
	settings.setValue("targetFramerate", this->targetFramerate());
	settings.setValue("sessionExported", this->sessionExported());
	settings.setValue("rgbExported", this->isRgbExported());
	settings.setValue("depthExported", this->isDepthExported());
	settings.setValue("depth2dExported", this->isDepth2dExported());
	settings.setValue("odomExported", this->isOdomExported());
	settings.setValue("userDataExported", this->isUserDataExported());
	if(!group.isEmpty())
	{
		settings.endGroup();
	}
}

void ExportDialog::loadSettings(QSettings & settings, const QString & group)
{
	if(!group.isEmpty())
	{
		settings.beginGroup(group);
	}
	_ui->spinBox_ignored->setValue(settings.value("framesIgnored", this->framesIgnored()).toInt());
	_ui->doubleSpinBox_framerate->setValue(settings.value("targetFramerate", this->targetFramerate()).toDouble());
	_ui->spinBox_session->setValue(settings.value("sessionExported", this->sessionExported()).toInt());
	_ui->checkBox_rgb->setChecked(settings.value("rgbExported", this->isRgbExported()).toBool());
	_ui->checkBox_depth->setChecked(settings.value("depthExported", this->isDepthExported()).toBool());
	_ui->checkBox_depth2d->setChecked(settings.value("depth2dExported", this->isDepth2dExported()).toBool());
	_ui->checkBox_odom->setChecked(settings.value("odomExported", this->isOdomExported()).toBool());
	_ui->checkBox_userData->setChecked(settings.value("userDataExported", this->isUserDataExported()).toBool());
	if(!group.isEmpty())
	{
		settings.endGroup();
	}
}

void ExportDialog::restoreDefaults()
{
	_ui->spinBox_ignored->setValue(0);
	_ui->doubleSpinBox_framerate->setValue(0);
	_ui->spinBox_session->setValue(-1);
	_ui->checkBox_rgb->setChecked(true);
	_ui->checkBox_depth->setChecked(true);
	_ui->checkBox_depth2d->setChecked(true);
	_ui->checkBox_odom->setChecked(true);
	_ui->checkBox_userData->setChecked(false);
}

void ExportDialog::getPath()
{
	QString path = QFileDialog::getSaveFileName(this, tr("Output database path..."), _ui->lineEdit_path->text(), tr("RTAB-Map database (*.db)"));
	if(!path.isEmpty())
	{
		_ui->lineEdit_path->setText(path);
	}
}

QString ExportDialog::outputPath() const
{
	return _ui->lineEdit_path->text();
}

int ExportDialog::framesIgnored() const
{
	return _ui->spinBox_ignored->value();
}

double ExportDialog::targetFramerate() const
{
	return _ui->doubleSpinBox_framerate->value();
}

int ExportDialog::sessionExported() const
{
	return _ui->spinBox_session->value();
}

bool ExportDialog::isRgbExported() const
{
	return _ui->checkBox_rgb->isChecked();
}

bool ExportDialog::isDepthExported() const
{
	return _ui->checkBox_depth->isChecked();
}

bool ExportDialog::isDepth2dExported() const
{
	return _ui->checkBox_depth2d->isChecked();
}

bool ExportDialog::isOdomExported() const
{
	return _ui->checkBox_odom->isChecked();
}

bool ExportDialog::isUserDataExported() const
{
	return _ui->checkBox_userData->isChecked();
}

}
