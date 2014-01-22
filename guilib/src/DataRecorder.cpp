/*
 * CloudRecorder.cpp
 *
 *  Created on: 2013-10-30
 *      Author: Mathieu
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
	qRegisterMetaType<rtabmap::Image>("rtabmap::Image");

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
		if(!memory_->init(path.toStdString(), true, customParameters, false))
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

void DataRecorder::addData(const rtabmap::Image & image)
{
	if(memory_)
	{
		//save to database
		UTimer time;
		memory_->update(image);
		memory_->cleanup();

		if(image.id() % 30)
		{
			memory_->emptyTrash();
		}

		UDEBUG("Time to process a message = %f s", time.ticks());
	}
	else
	{
		UWARN("CloudRecorder not initialized!");
	}
	--dataQueue_;
}

void DataRecorder::showImage(const rtabmap::Image & image)
{
	if(this->isVisible() && !image.empty())
	{
		imageView_->setImage(uCvMat2QImage(image.image()));
		imageView_->setImageDepth(uCvMat2QImage(image.depth()));
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
			if(!camEvent->image().empty())
			{
				UINFO("Receiving rate = %f Hz", 1.0f/timer_.ticks());
				QMetaObject::invokeMethod(this, "addData", Q_ARG(rtabmap::Image, camEvent->image()));
				++dataQueue_;

				if(dataQueue_ < 2 && this->isVisible())
				{
					QMetaObject::invokeMethod(this, "showImage", Q_ARG(rtabmap::Image, camEvent->image()));
				}
			}
		}
	}
}

} /* namespace rtabmap */
