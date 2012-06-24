/*
 * CameraMicro.h
 *
 *  Created on: 2012-05-27
 *      Author: mathieu
 */

#ifndef CAMERAMICRO_H_
#define CAMERAMICRO_H_

#include "rtabmap/core/Camera.h"
#include "rtabmap/core/Micro.h"
#include "rtabmap/core/SensorimotorEvent.h"
#include <utilite/UThreadNode.h>
#include <utilite/UEventsManager.h>

namespace rtabmap
{

class CameraMicro : public UThreadNode
{
public:
	CameraMicro(Camera * camera, Micro * micro) :
		camera_(camera),
		micro_(micro)
	{
	}

protected:
	void mainLoopBegin()
	{
		if(!camera_ || !micro_ )
		{
			UERROR("Camera and/or Micro are null");
			this->kill();
			return;
		}
		micro_->startRecorder();
	}

	void mainLoop()
	{
		cv::Mat frameFreq;
		// frame rate should depend on micro and camera frame rate
		// (getFrame() and takeImage() are blocking calls)
		cv::Mat frame = micro_->getFrame(frameFreq, true);
		if(frame.empty())
		{
			if(this->isRunning())
			{
				UEventsManager::post(new MicroEvent(MicroEvent::kTypeNoMoreFrames));
				this->kill();
			}
			return;
		}
		cv::Mat image = camera_->takeImage();
		if(image.empty())
		{
			UEventsManager::post(new CameraEvent(CameraEvent::kCodeNoMoreImages));
			this->kill();
			return;
		}
		std::list<Sensor> sensors;
		sensors.push_back(Sensor(image, Sensor::kTypeImage));
		sensors.push_back(Sensor(frameFreq, Sensor::kTypeAudioFreqSqrdMagn));
		UEventsManager::post(new SensorimotorEvent(sensors, std::list<Actuator>()));
	}

	void mainLoopKill()
	{
		if(micro_)
		{
			micro_->stop();
		}
	}

private:
	Camera * camera_;
	Micro * micro_;
};

}

#endif /* CAMERAMICRO_H_ */
