/*
Copyright (c) 2010-2014, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
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

#include "CreateSimpleCalibrationDialog.h"
#include "ui_createSimpleCalibrationDialog.h"

#include "rtabmap/core/CameraModel.h"
#include "rtabmap/utilite/ULogger.h"

#include <QFileDialog>
#include <QPushButton>
#include <QMessageBox>

namespace rtabmap {

CreateSimpleCalibrationDialog::CreateSimpleCalibrationDialog(
		const QString & savingFolder,
		const QString & cameraName,
		QWidget * parent) :
	QDialog(parent),
	savingFolder_(savingFolder),
	cameraName_(cameraName)
{
	ui_ = new Ui_createSimpleCalibrationDialog();
	ui_->setupUi(this);

	connect(ui_->buttonBox->button(QDialogButtonBox::Save), SIGNAL(clicked()), this, SLOT(saveCalibration()));
	connect(ui_->buttonBox, SIGNAL(rejected()), this, SLOT(reject()));

	connect(ui_->doubleSpinBox_fx, SIGNAL(valueChanged(double)), this, SLOT(updateSaveStatus()));
	connect(ui_->doubleSpinBox_fy, SIGNAL(valueChanged(double)), this, SLOT(updateSaveStatus()));

	ui_->buttonBox->button(QDialogButtonBox::Save)->setEnabled(false);
}

CreateSimpleCalibrationDialog::~CreateSimpleCalibrationDialog()
{
	delete ui_;
}

void CreateSimpleCalibrationDialog::updateSaveStatus()
{
	ui_->buttonBox->button(QDialogButtonBox::Save)->setEnabled(ui_->doubleSpinBox_fx->value() > 0.0 && ui_->doubleSpinBox_fy->value() > 0.0);
}

void CreateSimpleCalibrationDialog::saveCalibration()
{
	if(ui_->doubleSpinBox_baseline->value()==0)
	{
		QString filePath = QFileDialog::getSaveFileName(this, tr("Save"), savingFolder_+"/"+cameraName_+".yaml", "*.yaml");
		QString name = QFileInfo(filePath).baseName();
		QString dir = QFileInfo(filePath).absoluteDir().absolutePath();
		if(!filePath.isEmpty())
		{
			cameraName_ = name;
			CameraModel model(
					name.toStdString(),
					ui_->doubleSpinBox_fx->value(),
					ui_->doubleSpinBox_fy->value(),
					ui_->doubleSpinBox_cx->value(),
					ui_->doubleSpinBox_cy->value());
			UASSERT(model.isValid());
			if(model.save(dir.toStdString()))
			{
				QMessageBox::information(this, tr("Save"), tr("Calibration file saved to \"%1\".").arg(filePath));
				this->accept();
			}
			else
			{
				QMessageBox::warning(this, tr("Save"), tr("Error saving \"%1\"").arg(filePath));
			}
		}
	}
	else
	{
		QString filePath = QFileDialog::getSaveFileName(this, tr("Save"), savingFolder_ + "/" + cameraName_, "*.yaml");
		QString name = QFileInfo(filePath).baseName();
		QString dir = QFileInfo(filePath).absoluteDir().absolutePath();
		if(!name.isEmpty())
		{
			std::string base = (dir+QDir::separator()+name).toStdString();
			std::string leftPath = base+"_left.yaml";
			std::string rightPath = base+"_right.yaml";
			std::string posePath = base+"_pose.yaml";

			StereoCameraModel model(
					name.toStdString(),
					ui_->doubleSpinBox_fx->value(),
					ui_->doubleSpinBox_fy->value(),
					ui_->doubleSpinBox_cx->value(),
					ui_->doubleSpinBox_cy->value(),
					ui_->doubleSpinBox_baseline->value());
			UASSERT(model.left().isValid() &&
					model.right().isValid()&&
					model.baseline() > 0.0);

			if(model.save(dir.toStdString(), true))
			{
				QMessageBox::information(this, tr("Save"), tr("Calibration files saved:\n  \"%1\"\n  \"%2\"\n  \"%3\".").
						arg(leftPath.c_str()).arg(rightPath.c_str()).arg(posePath.c_str()));
				this->accept();
			}
			else
			{
				QMessageBox::warning(this, tr("Save"), tr("Error saving \"%1\" and \"%2\"").arg(leftPath.c_str()).arg(rightPath.c_str()));
			}
		}
	}
}

}
