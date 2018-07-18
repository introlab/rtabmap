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

#include "rtabmap/core/IMUThread.h"
#include "rtabmap/core/IMU.h"
#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UConversion.h>

namespace rtabmap
{

IMUThread::IMUThread(int rate, const Transform & localTransform) :
		rate_(rate),
		localTransform_(localTransform),
		captureDelay_(0.0),
		previousStamp_(0.0)
{
}

IMUThread::~IMUThread()
{
	imuFile_.close();
}

bool IMUThread::init(const std::string & path)
{
	imuFile_.close();
	captureDelay_ = 0.0;
	previousStamp_ = 0.0;

	// open the IMU file
	std::string line;
	imuFile_.open(path.c_str());
	if (!imuFile_.good()) {
		UERROR("no imu file found at %s",path.c_str());
		return false;
	}
	int number_of_lines = 0;
	while (std::getline(imuFile_, line))
		++number_of_lines;
	printf("No. IMU measurements: %d\n", number_of_lines-1);
	if (number_of_lines - 1 <= 0) {
		UERROR("no imu messages present in %s", path.c_str());
		return false;
	}
	// set reading position to second line
	imuFile_.clear();
	imuFile_.seekg(0, std::ios::beg);
	std::getline(imuFile_, line);

	return true;
}

void IMUThread::setRate(int rate)
{
	rate_ = rate;
}

void IMUThread::mainLoopBegin()
{
	ULogger::registerCurrentThread("IMU");
	frameRateTimer_.start();
}

void IMUThread::mainLoop()
{
	UTimer totalTime;
	UDEBUG("");

	if(rate_>0 || captureDelay_)
	{
		double delay = rate_>0?1000.0/double(rate_):1000.0f*captureDelay_;
		int sleepTime = delay - 1000.0f*frameRateTimer_.getElapsedTime();
		if(sleepTime > 2)
		{
			uSleep(sleepTime-2);
		}

		// Add precision at the cost of a small overhead
		delay/=1000.0;
		while(frameRateTimer_.getElapsedTime() < delay-0.000001)
		{
			//
		}

		frameRateTimer_.start();
	}
	captureDelay_ = 0.0;

	std::string line;
	if (std::getline(imuFile_, line))
	{
		std::stringstream stream(line);
		std::string s;
		std::getline(stream, s, ',');
		std::string nanoseconds = s.substr(s.size() - 9, 9);
		std::string seconds = s.substr(0, s.size() - 9);

		cv::Vec3d gyr;
		for (int j = 0; j < 3; ++j) {
			std::getline(stream, s, ',');
			gyr[j] = uStr2Double(s);
		}

		cv::Vec3d acc;
		for (int j = 0; j < 3; ++j) {
			std::getline(stream, s, ',');
			acc[j] = uStr2Double(s);
		}

		double stamp = double(uStr2Int(seconds)) + double(uStr2Int(nanoseconds))*1e-9;
		if(previousStamp_>0 && stamp > previousStamp_)
		{
			captureDelay_ = stamp - previousStamp_;
		}
		previousStamp_ = stamp;

		IMU imu(gyr, cv::Mat(), acc, cv::Mat(), localTransform_);
		this->post(new IMUEvent(imu, stamp));
	}
	else if(!this->isKilled())
	{
		UWARN("no more imu data...");
		this->kill();
		this->post(new IMUEvent());
	}
}

} // namespace rtabmap
