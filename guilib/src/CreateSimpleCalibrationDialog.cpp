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

#include "rtabmap/gui/CreateSimpleCalibrationDialog.h"
#include "ui_createSimpleCalibrationDialog.h"

#include "rtabmap/core/CameraModel.h"
#include "rtabmap/core/StereoCameraModel.h"
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
	if(cameraName_.isEmpty())
	{
		cameraName_ = "calib";
	}

	ui_ = new Ui_createSimpleCalibrationDialog();
	ui_->setupUi(this);

	connect(ui_->buttonBox->button(QDialogButtonBox::Save), SIGNAL(clicked()), this, SLOT(saveCalibration()));
	connect(ui_->buttonBox, SIGNAL(rejected()), this, SLOT(reject()));

	connect(ui_->comboBox_advanced, SIGNAL(currentIndexChanged(int)), ui_->stackedWidget, SLOT(setCurrentIndex(int)));

	connect(ui_->checkBox_stereo, SIGNAL(stateChanged(int)), this, SLOT(updateStereoView()));
	connect(ui_->comboBox_advanced, SIGNAL(currentIndexChanged(int)), this, SLOT(updateStereoView()));
	connect(ui_->checkBox_stereo, SIGNAL(stateChanged(int)), this, SLOT(updateSaveStatus()));
	connect(ui_->comboBox_advanced, SIGNAL(currentIndexChanged(int)), this, SLOT(updateSaveStatus()));

	connect(ui_->doubleSpinBox_fx, SIGNAL(valueChanged(double)), this, SLOT(updateSaveStatus()));
	connect(ui_->doubleSpinBox_fy, SIGNAL(valueChanged(double)), this, SLOT(updateSaveStatus()));

	connect(ui_->doubleSpinBox_fx_l, SIGNAL(valueChanged(double)), this, SLOT(updateSaveStatus()));
	connect(ui_->doubleSpinBox_fy_l, SIGNAL(valueChanged(double)), this, SLOT(updateSaveStatus()));
	connect(ui_->doubleSpinBox_cx_l, SIGNAL(valueChanged(double)), this, SLOT(updateSaveStatus()));
	connect(ui_->doubleSpinBox_cy_l, SIGNAL(valueChanged(double)), this, SLOT(updateSaveStatus()));
	connect(ui_->lineEdit_D_l, SIGNAL(textEdited(const QString &)), this, SLOT(updateSaveStatus()));

	connect(ui_->doubleSpinBox_fx_r, SIGNAL(valueChanged(double)), this, SLOT(updateSaveStatus()));
	connect(ui_->doubleSpinBox_fy_r, SIGNAL(valueChanged(double)), this, SLOT(updateSaveStatus()));
	connect(ui_->doubleSpinBox_cx_r, SIGNAL(valueChanged(double)), this, SLOT(updateSaveStatus()));
	connect(ui_->doubleSpinBox_cy_r, SIGNAL(valueChanged(double)), this, SLOT(updateSaveStatus()));
	connect(ui_->lineEdit_D_r, SIGNAL(textEdited(const QString &)), this, SLOT(updateSaveStatus()));

	connect(ui_->spinBox_width, SIGNAL(valueChanged(int)), this, SLOT(updateSaveStatus()));
	connect(ui_->spinBox_height, SIGNAL(valueChanged(int)), this, SLOT(updateSaveStatus()));

	connect(ui_->doubleSpinBox_baseline, SIGNAL(valueChanged(double)), this, SLOT(updateSaveStatus()));
	connect(ui_->lineEdit_RT, SIGNAL(textEdited(const QString &)), this, SLOT(updateSaveStatus()));

	ui_->stackedWidget->setCurrentIndex(ui_->comboBox_advanced->currentIndex());

	ui_->buttonBox->button(QDialogButtonBox::Save)->setEnabled(false);

	updateStereoView();
}

CreateSimpleCalibrationDialog::~CreateSimpleCalibrationDialog()
{
	delete ui_;
}

void CreateSimpleCalibrationDialog::updateStereoView()
{
	bool checked = ui_->checkBox_stereo->isChecked();
	ui_->doubleSpinBox_baseline->setVisible(checked);
	ui_->label_baseline->setVisible(checked);
	ui_->label_right->setVisible(checked);
	ui_->doubleSpinBox_fx_r->setVisible(checked);
	ui_->doubleSpinBox_fy_r->setVisible(checked);
	ui_->doubleSpinBox_cx_r->setVisible(checked);
	ui_->doubleSpinBox_cy_r->setVisible(checked);
	ui_->lineEdit_D_r->setVisible(checked);
	ui_->groupBox_stereo_extrinsics->setVisible(ui_->comboBox_advanced->currentIndex() == 1 && checked);
	ui_->label_left->setVisible(checked);
}

void CreateSimpleCalibrationDialog::updateSaveStatus()
{
	bool valid = false;
	if(ui_->comboBox_advanced->currentIndex() == 0 &&
	   ui_->doubleSpinBox_fx->value() > 0.0 &&
	   ui_->doubleSpinBox_fy->value() > 0.0 &&
	   ui_->spinBox_width->value() > 0 &&
	   ui_->spinBox_height->value() > 0 &&
	   (!ui_->checkBox_stereo->isChecked() || ui_->doubleSpinBox_baseline->value() != 0.0))
	{
		// basic
		valid = true;
	}
	else if(ui_->comboBox_advanced->currentIndex() == 1 &&
			 ui_->doubleSpinBox_fx_l->value() > 0.0 &&
			 ui_->doubleSpinBox_fy_l->value() > 0.0 &&
			 ui_->doubleSpinBox_cx_l->value() > 0.0 &&
			 ui_->doubleSpinBox_cy_l->value() > 0.0 &&
			 (!ui_->checkBox_stereo->isChecked() || ui_->doubleSpinBox_fx_r->value() > 0.0) &&
			 (!ui_->checkBox_stereo->isChecked() || ui_->doubleSpinBox_fy_r->value() > 0.0) &&
			 (!ui_->checkBox_stereo->isChecked() || ui_->doubleSpinBox_cx_r->value() > 0.0) &&
			 (!ui_->checkBox_stereo->isChecked() || ui_->doubleSpinBox_cy_r->value() > 0.0) &&
			 ui_->spinBox_width->value() > 0 &&
			 ui_->spinBox_height->value() > 0 &&
			 !ui_->lineEdit_D_l->text().isEmpty() &&
			 (!ui_->checkBox_stereo->isChecked() || !ui_->lineEdit_D_r->text().isEmpty()) &&
			 (!ui_->checkBox_stereo->isChecked() || !ui_->lineEdit_RT->text().isEmpty()))
	{
		//advanced
		QStringList distorsionsStrListL = ui_->lineEdit_D_l->text().remove('[').remove(']').replace(',',' ').replace(';',' ').simplified().trimmed().split(' ');
		QStringList distorsionsStrListR = ui_->lineEdit_D_r->text().remove('[').remove(']').replace(',',' ').replace(';',' ').simplified().trimmed().split(' ');
		std::string RT = ui_->lineEdit_RT->text().remove('[').remove(']').replace(',',' ').replace(';',' ').simplified().trimmed().toStdString();

		if((distorsionsStrListL.size() == 4 || distorsionsStrListL.size() == 5 || distorsionsStrListL.size() == 8) &&
				(!ui_->checkBox_stereo->isChecked() || (distorsionsStrListR.size() == 4 || distorsionsStrListR.size() == 5 || distorsionsStrListR.size() == 8)) &&
				(!ui_->checkBox_stereo->isChecked() || (!RT.empty() && Transform::canParseString(RT))))
		{
			valid = true;
		}
	}
	ui_->buttonBox->button(QDialogButtonBox::Save)->setEnabled(valid);
}

void CreateSimpleCalibrationDialog::saveCalibration()
{
	QString filePath = QFileDialog::getSaveFileName(this, tr("Save"), savingFolder_+"/"+cameraName_+".yaml", "*.yaml");
	QString name = QFileInfo(filePath).baseName();
	QString dir = QFileInfo(filePath).absoluteDir().absolutePath();
	if(!filePath.isEmpty())
	{
		cameraName_ = name;
		CameraModel modelLeft;
		float width = ui_->spinBox_width->value();
		float height = ui_->spinBox_height->value();
		if(ui_->comboBox_advanced->currentIndex() == 0)
		{
			//basic
			modelLeft = CameraModel(
					name.toStdString(),
					ui_->doubleSpinBox_fx->value(),
					ui_->doubleSpinBox_fy->value(),
					ui_->doubleSpinBox_cx->value(),
					ui_->doubleSpinBox_cy->value(),
					CameraModel::opticalRotation(),
					0,
					cv::Size(width, height));
			UASSERT(modelLeft.isValidForProjection());
		}
		else
		{
			//advanced
			cv::Mat K = cv::Mat::eye(3, 3, CV_64FC1);
			K.at<double>(0,0) = ui_->doubleSpinBox_fx_l->value();
			K.at<double>(1,1) = ui_->doubleSpinBox_fy_l->value();
			K.at<double>(0,2) = ui_->doubleSpinBox_cx_l->value();
			K.at<double>(1,2) = ui_->doubleSpinBox_cy_l->value();
			cv::Mat R = cv::Mat::eye(3, 3, CV_64FC1);
			cv::Mat P = cv::Mat::zeros(3, 4, CV_64FC1);
			K.copyTo(cv::Mat(P, cv::Range(0,3), cv::Range(0,3)));

			QStringList distorsionCoeffs = ui_->lineEdit_D_l->text().remove('[').remove(']').replace(',',' ').replace(';',' ').simplified().trimmed().split(' ');
			UASSERT(distorsionCoeffs.size() == 4 || distorsionCoeffs.size() == 5 || distorsionCoeffs.size() == 8);
			cv::Mat D = cv::Mat::zeros(1, distorsionCoeffs.size(), CV_64FC1);
			bool ok;
			for(int i=0; i<distorsionCoeffs.size(); ++i)
			{
				D.at<double>(i) = distorsionCoeffs.at(i).toDouble(&ok);
				if(!ok)
				{
					QMessageBox::warning(this, tr("Save"), tr("Error parsing left distortion coefficients \"%1\".").arg(ui_->lineEdit_D_l->text()));
					return;
				}
			}

			modelLeft = CameraModel(name.toStdString(), cv::Size(width,height), K, D, R, P);
			UASSERT(modelLeft.isValidForRectification());
		}

		if(ui_->checkBox_stereo->isChecked())
		{
			StereoCameraModel stereoModel;
			if(ui_->comboBox_advanced->currentIndex() == 0)
			{
				CameraModel modelRight(
						name.toStdString(),
						ui_->doubleSpinBox_fx->value(),
						ui_->doubleSpinBox_fy->value(),
						ui_->doubleSpinBox_cx->value(),
						ui_->doubleSpinBox_cy->value(),
						Transform::getIdentity(),
						ui_->doubleSpinBox_baseline->value()*-ui_->doubleSpinBox_fx->value(),
						cv::Size(width, height));
				UASSERT(modelRight.isValidForProjection());
				stereoModel = StereoCameraModel(name.toStdString(), modelLeft, modelRight, Transform());
				UASSERT(stereoModel.isValidForProjection());
			}
			else if(ui_->comboBox_advanced->currentIndex() == 1)
			{
				CameraModel modelRight;
				cv::Mat K = cv::Mat::eye(3, 3, CV_64FC1);
				K.at<double>(0,0) = ui_->doubleSpinBox_fx_r->value();
				K.at<double>(1,1) = ui_->doubleSpinBox_fy_r->value();
				K.at<double>(0,2) = ui_->doubleSpinBox_cx_r->value();
				K.at<double>(1,2) = ui_->doubleSpinBox_cy_r->value();
				cv::Mat R = cv::Mat::eye(3, 3, CV_64FC1);
				cv::Mat P = cv::Mat::zeros(3, 4, CV_64FC1);
				K.copyTo(cv::Mat(P, cv::Range(0,3), cv::Range(0,3)));

				QStringList distorsionCoeffs = ui_->lineEdit_D_r->text().remove('[').remove(']').replace(',',' ').replace(';',' ').simplified().trimmed().split(' ');
				UASSERT(distorsionCoeffs.size() == 4 || distorsionCoeffs.size() == 5 || distorsionCoeffs.size() == 8);
				cv::Mat D = cv::Mat::zeros(1, distorsionCoeffs.size(), CV_64FC1);
				bool ok;
				for(int i=0; i<distorsionCoeffs.size(); ++i)
				{
					D.at<double>(i) = distorsionCoeffs.at(i).toDouble(&ok);
					if(!ok)
					{
						QMessageBox::warning(this, tr("Save"), tr("Error parsing right distortion coefficients \"%1\".").arg(ui_->lineEdit_D_r->text()));
						return;
					}
				}

				modelRight = CameraModel(name.toStdString(), cv::Size(width,height), K, D, R, P);
				UASSERT(modelRight.isValidForRectification());

				UASSERT(Transform::canParseString(ui_->lineEdit_RT->text().remove('[').remove(']').replace(',',' ').replace(';',' ').simplified().trimmed().toStdString()));
				stereoModel = StereoCameraModel(name.toStdString(), modelLeft, modelRight, Transform::fromString(ui_->lineEdit_RT->text().remove('[').remove(']').replace(',',' ').replace(';',' ').trimmed().toStdString()));
				if(stereoModel.baseline() < 0)
				{
					QMessageBox::warning(this, tr("Save"), tr("Error parsing the extrinsics \"%1\", resulting baseline (%f) is negative!").arg(ui_->lineEdit_RT->text()).arg(stereoModel.baseline()));
					return;
				}
				UASSERT(stereoModel.isValidForRectification());
			}

			std::string base = (dir+QDir::separator()+name).toStdString();
			std::string leftPath = base+"_left.yaml";
			std::string rightPath = base+"_right.yaml";

			if(stereoModel.save(dir.toStdString(), ui_->comboBox_advanced->currentIndex() != 1))
			{
				if(ui_->comboBox_advanced->currentIndex() == 0)
				{
					QMessageBox::information(this, tr("Save"), tr("Calibration files saved:\n  \"%1\"\n  \"%2\".").
							arg(leftPath.c_str()).arg(rightPath.c_str()));
				}
				else
				{
					std::string posePath = base+"_pose.yaml";
					QMessageBox::information(this, tr("Save"), tr("Calibration files saved:\n  \"%1\"\n  \"%2\"\n  \"%3\".").
							arg(leftPath.c_str()).arg(rightPath.c_str()).arg(posePath.c_str()));
				}
				this->accept();
			}
			else
			{
				QMessageBox::warning(this, tr("Save"), tr("Error saving \"%1\" and \"%2\"").arg(leftPath.c_str()).arg(rightPath.c_str()));
			}
		}
		else
		{
			if(modelLeft.save(dir.toStdString()))
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
}

}
