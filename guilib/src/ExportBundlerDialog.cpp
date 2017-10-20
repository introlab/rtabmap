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

#include "rtabmap/gui/ExportBundlerDialog.h"
#include "ui_exportBundlerDialog.h"

#include <QFileDialog>
#include <QPushButton>

namespace rtabmap {

ExportBundlerDialog::ExportBundlerDialog(QWidget * parent) :
	QDialog(parent)
{
	_ui = new Ui_ExportBundlerDialog();
	_ui->setupUi(this);

	connect(_ui->toolButton_path, SIGNAL(clicked()), this, SLOT(getPath()));

	restoreDefaults();
	connect(_ui->buttonBox->button(QDialogButtonBox::RestoreDefaults), SIGNAL(clicked()), this, SLOT(restoreDefaults()));

	connect(_ui->doubleSpinBox_laplacianVariance, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
	connect(_ui->doubleSpinBox_linearSpeed, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));
	connect(_ui->doubleSpinBox_angularSpeed, SIGNAL(valueChanged(double)), this, SIGNAL(configChanged()));

	_ui->lineEdit_path->setText(QDir::currentPath());
}

ExportBundlerDialog::~ExportBundlerDialog()
{
	delete _ui;
}

void ExportBundlerDialog::saveSettings(QSettings & settings, const QString & group) const
{
	if(!group.isEmpty())
	{
		settings.beginGroup(group);
	}
	settings.setValue("maxLinearSpeed", this->maxLinearSpeed());
	settings.setValue("maxAngularSpeed", this->maxAngularSpeed());
	settings.setValue("laplacianThr", this->laplacianThreshold());
	if(!group.isEmpty())
	{
		settings.endGroup();
	}
}

void ExportBundlerDialog::loadSettings(QSettings & settings, const QString & group)
{
	if(!group.isEmpty())
	{
		settings.beginGroup(group);
	}
	_ui->doubleSpinBox_linearSpeed->setValue(settings.value("maxLinearSpeed", this->maxLinearSpeed()).toDouble());
	_ui->doubleSpinBox_angularSpeed->setValue(settings.value("maxAngularSpeed", this->maxAngularSpeed()).toDouble());
	_ui->doubleSpinBox_laplacianVariance->setValue(settings.value("laplacianThr", this->laplacianThreshold()).toDouble());
	if(!group.isEmpty())
	{
		settings.endGroup();
	}
}

void ExportBundlerDialog::setWorkingDirectory(const QString & path)
{
	_ui->lineEdit_path->setText((path.isEmpty()?QDir::currentPath():path) + "/bundler");
}

void ExportBundlerDialog::restoreDefaults()
{
	_ui->doubleSpinBox_linearSpeed->setValue(0);
	_ui->doubleSpinBox_angularSpeed->setValue(0);
	_ui->doubleSpinBox_laplacianVariance->setValue(0);
}

void ExportBundlerDialog::getPath()
{
	QString path = QFileDialog::getExistingDirectory(this, tr("Exporting cameras in Bundler format..."), _ui->lineEdit_path->text());
	if(!path.isEmpty())
	{
		_ui->lineEdit_path->setText(path);
	}
}

QString ExportBundlerDialog::outputPath() const
{
	return _ui->lineEdit_path->text();
}

double ExportBundlerDialog::maxLinearSpeed() const
{
	return _ui->doubleSpinBox_linearSpeed->value();
}
double ExportBundlerDialog::maxAngularSpeed() const
{
	return _ui->doubleSpinBox_angularSpeed->value();
}
double ExportBundlerDialog::laplacianThreshold() const
{
	return _ui->doubleSpinBox_laplacianVariance->value();
}

}
