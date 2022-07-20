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

#include "rtabmap/gui/CameraViewer.h"

#include <rtabmap/core/CameraEvent.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/core/util2d.h>
#include <rtabmap/core/util3d_filtering.h>
#include <rtabmap/gui/ImageView.h>
#include <rtabmap/gui/CloudViewer.h>
#include <rtabmap/utilite/UCv2Qt.h>
#include <rtabmap/utilite/ULogger.h>
#include <QtCore/QMetaType>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QLabel>
#include <QSpinBox>
#include <QDialogButtonBox>
#include <QCheckBox>
#include <QPushButton>

namespace rtabmap {


CameraViewer::CameraViewer(QWidget * parent, const ParametersMap & parameters) :
		QDialog(parent),
	imageView_(new ImageView(this)),
	cloudView_(new CloudViewer(this)),
	processingImages_(false),
	parameters_(parameters)
{
	qRegisterMetaType<rtabmap::SensorData>("rtabmap::SensorData");

	imageView_->setImageDepthShown(true);
	imageView_->setMinimumSize(320, 240);
	QHBoxLayout * layout = new QHBoxLayout();
	layout->setMargin(0);
	layout->addWidget(imageView_,1);
	layout->addWidget(cloudView_,1);

	QLabel * decimationLabel = new QLabel("Decimation", this);
	decimationSpin_ = new QSpinBox(this);
	decimationSpin_->setMinimum(-16);
	decimationSpin_->setMaximum(16);
	decimationSpin_->setValue(2);

	pause_ = new QPushButton("Pause", this);
	pause_->setCheckable(true);
	showCloudCheckbox_ = new QCheckBox("Show RGB-D cloud", this);
	showCloudCheckbox_->setEnabled(false);
	showCloudCheckbox_->setChecked(true);
	showScanCheckbox_ = new QCheckBox("Show scan", this);
	showScanCheckbox_->setEnabled(false);
	showScanCheckbox_->setChecked(true);

	imageSizeLabel_ = new QLabel(this);

	QDialogButtonBox * buttonBox = new QDialogButtonBox(this);
	buttonBox->setStandardButtons(QDialogButtonBox::Close);
	connect(buttonBox, SIGNAL(rejected()), this, SLOT(reject()));

	QHBoxLayout * layout2 = new QHBoxLayout();
	layout2->addWidget(pause_);
	layout2->addWidget(decimationLabel);
	layout2->addWidget(decimationSpin_);
	layout2->addWidget(showCloudCheckbox_);
	layout2->addWidget(showScanCheckbox_);
	layout2->addWidget(imageSizeLabel_);
	layout2->addStretch(1);
	layout2->addWidget(buttonBox);

	QVBoxLayout * vlayout = new QVBoxLayout(this);
	vlayout->setMargin(0);
	vlayout->setSpacing(0);
	vlayout->addLayout(layout, 1);
	vlayout->addLayout(layout2);

	this->setLayout(vlayout);
}

CameraViewer::~CameraViewer()
{
	this->unregisterFromEventsManager();
}

void CameraViewer::showImage(const rtabmap::SensorData & data)
{
	processingImages_ = true;
	QString sizes;
	if(!data.imageRaw().empty())
	{
		imageView_->setImage(uCvMat2QImage(data.imageRaw()));
		sizes.append(QString("Color=%1x%2").arg(data.imageRaw().cols).arg(data.imageRaw().rows));
	}
	if(!data.depthOrRightRaw().empty())
	{
		imageView_->setImageDepth(data.depthOrRightRaw());
		sizes.append(QString(" Depth=%1x%2").arg(data.depthOrRightRaw().cols).arg(data.depthOrRightRaw().rows));
	}
	imageSizeLabel_->setText(sizes);

	if(!data.depthOrRightRaw().empty() &&
	   ((data.stereoCameraModels().size() && data.stereoCameraModels()[0].isValidForProjection()) || (data.cameraModels().size() && data.cameraModels().at(0).isValidForProjection())))
	{
		if(showCloudCheckbox_->isChecked())
		{
			if(!data.imageRaw().empty() && !data.depthOrRightRaw().empty())
			{
				showCloudCheckbox_->setEnabled(true);
				cloudView_->addCloud("cloud", util3d::cloudRGBFromSensorData(data, decimationSpin_->value()!=0?decimationSpin_->value():1, 0, 0, 0, parameters_));
			}
			else if(!data.depthOrRightRaw().empty())
			{
				showCloudCheckbox_->setEnabled(true);
				cloudView_->addCloud("cloud", util3d::cloudFromSensorData(data, decimationSpin_->value()!=0?fabs(decimationSpin_->value()):1, 0, 0, 0, parameters_));
			}
		}
	}

	if(!data.laserScanRaw().isEmpty())
	{
		showScanCheckbox_->setEnabled(true);
		if(showScanCheckbox_->isChecked())
		{
			if(data.laserScanRaw().hasNormals())
			{
				if(data.laserScanRaw().hasIntensity())
				{
					cloudView_->addCloud("scan", util3d::downsample(util3d::laserScanToPointCloudINormal(data.laserScanRaw()), decimationSpin_->value()!=0?fabs(decimationSpin_->value()):1), data.laserScanRaw().localTransform(), Qt::yellow);
				}
				else if(data.laserScanRaw().hasRGB())
				{
					cloudView_->addCloud("scan", util3d::downsample(util3d::laserScanToPointCloudRGBNormal(data.laserScanRaw()), decimationSpin_->value()!=0?fabs(decimationSpin_->value()):1), data.laserScanRaw().localTransform(), Qt::yellow);
				}
				else
				{
					cloudView_->addCloud("scan", util3d::downsample(util3d::laserScanToPointCloudNormal(data.laserScanRaw()), decimationSpin_->value()!=0?fabs(decimationSpin_->value()):1), data.laserScanRaw().localTransform(), Qt::yellow);
				}
			}
			else if(data.laserScanRaw().hasIntensity())
			{
				cloudView_->addCloud("scan", util3d::downsample(util3d::laserScanToPointCloudI(data.laserScanRaw()), decimationSpin_->value()!=0?fabs(decimationSpin_->value()):1), data.laserScanRaw().localTransform(), Qt::yellow);
			}
			else if(data.laserScanRaw().hasRGB())
			{
				cloudView_->addCloud("scan", util3d::downsample(util3d::laserScanToPointCloudRGB(data.laserScanRaw()), decimationSpin_->value()!=0?fabs(decimationSpin_->value()):1), data.laserScanRaw().localTransform(), Qt::yellow);
			}
			else
			{
				cloudView_->addCloud("scan", util3d::downsample(util3d::laserScanToPointCloud(data.laserScanRaw()), decimationSpin_->value()!=0?fabs(decimationSpin_->value()):1), data.laserScanRaw().localTransform(), Qt::yellow);
			}
		}
	}

	cloudView_->setVisible((showCloudCheckbox_->isEnabled() && showCloudCheckbox_->isChecked()) ||
						   (showScanCheckbox_->isEnabled() && showScanCheckbox_->isChecked()));
	if(cloudView_->isVisible())
	{
		cloudView_->refreshView();
	}
	if(cloudView_->getAddedClouds().contains("cloud"))
	{
		cloudView_->setCloudVisibility("cloud", showCloudCheckbox_->isChecked());
	}
	if(cloudView_->getAddedClouds().contains("scan"))
	{
		cloudView_->setCloudVisibility("scan", showScanCheckbox_->isChecked());
	}

	processingImages_ = false;
}

bool CameraViewer::handleEvent(UEvent * event)
{
	if(!pause_->isChecked())
	{
		if(event->getClassName().compare("CameraEvent") == 0)
		{
			CameraEvent * camEvent = (CameraEvent*)event;
			if(camEvent->getCode() == CameraEvent::kCodeData)
			{
				if(camEvent->data().isValid())
				{
					if(!processingImages_ && this->isVisible() && camEvent->data().isValid())
					{
						processingImages_ = true;
						QMetaObject::invokeMethod(this, "showImage",
								Q_ARG(rtabmap::SensorData, camEvent->data()));
					}
				}
			}
		}
	}
	return false;
}

} /* namespace rtabmap */
