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

#ifndef M_PI
#define M_PI       3.14159265358979323846
#endif

namespace rtabmap {

EditConstraintDialog::EditConstraintDialog(const Transform & constraint, double linearSigma, double angularSigma, QWidget * parent) :
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
	_ui->linear_sigma->setValue(linearSigma);
	_ui->angular_sigma->setValue(angularSigma);

	connect(_ui->checkBox_radians, SIGNAL(stateChanged(int)), this, SLOT(switchUnits()));
	_ui->checkBox_radians->setChecked(false);
}

EditConstraintDialog::~EditConstraintDialog()
{
	delete _ui;
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
	boxes.push_back(_ui->angular_sigma);
	for(int i=0; i<boxes.size(); ++i)
	{
		double value = boxes[i]->value()*conversion;
		if(_ui->checkBox_radians->isChecked())
		{
			if(boxes[i]!=_ui->angular_sigma)
			{
				boxes[i]->setMinimum(-M_PI);
			}
			boxes[i]->setMaximum(M_PI);
			boxes[i]->setSuffix(" rad");
			boxes[i]->setSingleStep(0.01);
		}
		else
		{
			if(boxes[i]!=_ui->angular_sigma)
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

double EditConstraintDialog::getLinearVariance() const
{
	return _ui->linear_sigma->value();
}
double EditConstraintDialog::getAngularVariance() const
{
	double conversion = 1.0f;
	if(!_ui->checkBox_radians->isChecked())
	{
		conversion = M_PI/180.0;
	}
	return _ui->angular_sigma->value()*conversion;
}

}
