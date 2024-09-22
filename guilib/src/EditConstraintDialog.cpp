/*
Copyright (c) 2010-2019, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
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

#include "rtabmap/gui/EditConstraintDialog.h"
#include "ui_editConstraintDialog.h"

#include <rtabmap/utilite/ULogger.h>
#include <iostream>

#ifndef M_PI
#define M_PI       3.14159265358979323846
#endif

namespace rtabmap {

EditConstraintDialog::EditConstraintDialog(const Transform & constraint, const cv::Mat & covariance, QWidget * parent) :
	QDialog(parent)
{
	_ui = new Ui_EditConstraintDialog();
	_ui->setupUi(this);

	float x,y,z,roll,pitch,yaw;
	constraint.getTranslationAndEulerAngles(x, y, z, roll, pitch, yaw);
	_ui->x->setValue(x);
	_ui->y->setValue(y);
	_ui->z->setValue(z);
	_ui->roll->setValue(roll);
	_ui->pitch->setValue(pitch);
	_ui->yaw->setValue(yaw);

	UASSERT(covariance.empty() || (covariance.cols == 6 && covariance.rows == 6 && covariance.type() == CV_64FC1));

	_ui->checkBox_radians->setChecked(true);
	_ui->linear_sigma_x->setValue(covariance.empty() || covariance.at<double>(0,0)>=9999 || covariance.at<double>(0,0)<=0?0:sqrt(covariance.at<double>(0,0)));
	_ui->linear_sigma_y->setValue(covariance.empty() || covariance.at<double>(1,1)>=9999 || covariance.at<double>(1,1)<=0?0:sqrt(covariance.at<double>(1,1)));
	_ui->linear_sigma_z->setValue(covariance.empty() || covariance.at<double>(2,2)>=9999 || covariance.at<double>(2,2)<=0?0:sqrt(covariance.at<double>(2,2)));
	_ui->angular_sigma_roll->setValue(covariance.empty() || covariance.at<double>(3,3)>=9999 || covariance.at<double>(3,3)<=0?0:sqrt(covariance.at<double>(3,3)));
	_ui->angular_sigma_pitch->setValue(covariance.empty() || covariance.at<double>(4,4)>=9999 || covariance.at<double>(4,4)<=0?0:sqrt(covariance.at<double>(4,4)));
	_ui->angular_sigma_yaw->setValue(covariance.empty() || covariance.at<double>(5,5)>=9999 || covariance.at<double>(5,5)<=0?0:sqrt(covariance.at<double>(5,5)));

	connect(_ui->checkBox_radians, SIGNAL(stateChanged(int)), this, SLOT(switchUnits()));
}

EditConstraintDialog::~EditConstraintDialog()
{
	delete _ui;
}

void EditConstraintDialog::setPoseGroupVisible(bool visible)
{
	_ui->groupBox_pose->setVisible(visible);
}
void EditConstraintDialog::setCovarianceGroupVisible(bool visible)
{
	_ui->groupBox_covariance->setVisible(visible);
}

void EditConstraintDialog::switchUnits()
{
	double conversion = 180.0/M_PI;
	if(_ui->checkBox_radians->isChecked())
	{
		conversion = M_PI/180.0;
	}
	QVector<QDoubleSpinBox*> boxes;
	boxes.push_back(_ui->roll);
	boxes.push_back(_ui->pitch);
	boxes.push_back(_ui->yaw);
	boxes.push_back(_ui->angular_sigma_roll);
	boxes.push_back(_ui->angular_sigma_pitch);
	boxes.push_back(_ui->angular_sigma_yaw);
	for(int i=0; i<boxes.size(); ++i)
	{
		double value = boxes[i]->value()*conversion;
		if(_ui->checkBox_radians->isChecked())
		{
			if(boxes[i]!=_ui->angular_sigma_roll && boxes[i]!=_ui->angular_sigma_pitch && boxes[i]!=_ui->angular_sigma_yaw)
			{
				boxes[i]->setMinimum(-M_PI);
			}
			boxes[i]->setMaximum(M_PI);
			boxes[i]->setSuffix(" rad");
			boxes[i]->setSingleStep(0.01);
		}
		else
		{
			if(boxes[i]!=_ui->angular_sigma_roll && boxes[i]!=_ui->angular_sigma_pitch && boxes[i]!=_ui->angular_sigma_yaw)
			{
				boxes[i]->setMinimum(-180);
			}
			boxes[i]->setMaximum(180);
			boxes[i]->setSuffix(" deg");
			boxes[i]->setSingleStep(1);
		}
		boxes[i]->setValue(value);
	}
}

Transform EditConstraintDialog::getTransform() const
{
	double conversion = 1.0f;
	if(!_ui->checkBox_radians->isChecked())
	{
		conversion = M_PI/180.0;
	}
	return Transform(_ui->x->value(), _ui->y->value(), _ui->z->value(), _ui->roll->value()*conversion, _ui->pitch->value()*conversion, _ui->yaw->value()*conversion);
}

cv::Mat EditConstraintDialog::getCovariance() const
{
	cv::Mat covariance = cv::Mat::eye(6,6,CV_64FC1);
	covariance.at<double>(0,0) = _ui->linear_sigma_x->value()==0?9999:_ui->linear_sigma_x->value()*_ui->linear_sigma_x->value();
	covariance.at<double>(1,1) = _ui->linear_sigma_y->value()==0?9999:_ui->linear_sigma_y->value()*_ui->linear_sigma_y->value();
	covariance.at<double>(2,2) = _ui->linear_sigma_z->value()==0?9999:_ui->linear_sigma_z->value()*_ui->linear_sigma_z->value();
	double conversion = 1.0f;
	if(!_ui->checkBox_radians->isChecked())
	{
		conversion = M_PI/180.0;
	}
	double sigma = _ui->angular_sigma_roll->value()*conversion;
	covariance.at<double>(3,3) = sigma==0?9999:sigma*sigma;
	sigma = _ui->angular_sigma_pitch->value()*conversion;
	covariance.at<double>(4,4) = sigma==0?9999:sigma*sigma;
	sigma = _ui->angular_sigma_yaw->value()*conversion;
	covariance.at<double>(5,5) = sigma==0?9999:sigma*sigma;
	return covariance;
}

}
