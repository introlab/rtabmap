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

#include "rtabmap/gui/DataRecorder.h"

#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/core/Memory.h>
#include <rtabmap/core/CameraEvent.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/gui/ImageView.h>
#include <rtabmap/gui/UCv2Qt.h>
#include <QtCore/QMetaType>
#include <QtGui/QHBoxLayout>

namespace rtabmap {


DataRecorder::DataRecorder(QWidget * parent) :
		QWidget(parent),
	memory_(0),
	imageView_(new ImageView(this)),
	dataQueue_(0)
{
	qRegisterMetaType<rtabmap::SensorData>("rtabmap::SensorData");

	imageView_->setImageDepthShown(true);
	QHBoxLayout * layout = new QHBoxLayout(this);
	layout->addWidget(imageView_);
	this->setLayout(layout);
}
bool DataRecorder::init(const QString & path, bool recordInRAM)
{
	if(!memory_)
	{
		ParametersMap customParameters;
		customParameters.insert(ParametersPair(Parameters::kMemRehearsalSimilarity(), "1.0")); // desactivate rehearsal
		customParameters.insert(ParametersPair(Parameters::kKpWordsPerImage(), "-1")); // desactivate keypoints extraction
		customParameters.insert(ParametersPair(Parameters::kMemImageKept(), "true")); // to keep images
		if(!recordInRAM)
		{
			customParameters.insert(ParametersPair(Parameters::kDbSqlite3InMemory(), "false")); // to keep images
		}
		memory_ = new Memory();
		if(!memory_->init(path.toStdString(), true, customParameters))
		{
			delete memory_;
			memory_ = 0;
			UERROR("Error initializing the memory.");
			return false;
		}
		return true;
	}
	else
	{
		UERROR("Already initialized, close it first.");
		return false;
	}
}

void DataRecorder::close()
{
	if(memory_)
	{
		delete memory_;
		memory_ = 0;
	}
}

DataRecorder::~DataRecorder()
{
	this->close();
}

void DataRecorder::addData(const rtabmap::SensorData & data)
{
	if(memory_)
	{
		//save to database
		UTimer time;
		memory_->update(data);
		memory_->cleanup();

		if(data.id() % 30)
		{
			memory_->emptyTrash();
		}

		UDEBUG("Time to process a message = %f s", time.ticks());
	}
	--dataQueue_;
}

void DataRecorder::showImage(const rtabmap::SensorData & data)
{
	if(this->isVisible() && data.isValid())
	{
		imageView_->setImage(uCvMat2QImage(data.image()));
		imageView_->setImageDepth(uCvMat2QImage(data.depth()));
		imageView_->fitInView(imageView_->sceneRect(), Qt::KeepAspectRatio);
	}
}

void DataRecorder::handleEvent(UEvent * event)
{
	if(event->getClassName().compare("CameraEvent") == 0)
	{
		CameraEvent * camEvent = (CameraEvent*)event;
		if(camEvent->getCode() == CameraEvent::kCodeImageDepth ||
		   camEvent->getCode() == CameraEvent::kCodeImage)
		{
			if(camEvent->data().isValid())
			{
				UINFO("Receiving rate = %f Hz", 1.0f/timer_.ticks());
				if(memory_)
				{
					QMetaObject::invokeMethod(this, "addData", Q_ARG(rtabmap::SensorData, camEvent->data()));
					++dataQueue_;
				}

				if(dataQueue_ < 2 && this->isVisible())
				{
					QMetaObject::invokeMethod(this, "showImage", Q_ARG(rtabmap::SensorData, camEvent->data()));
				}
			}
		}
	}
}

} /* namespace rtabmap */
