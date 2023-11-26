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

#include "rtabmap/core/Camera.h"
#include "rtabmap/core/IMUFilter.h"

#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/UFile.h>
#include <rtabmap/utilite/UDirectory.h>
#include <rtabmap/utilite/UEventsManager.h>

namespace rtabmap
{

Camera::Camera(float imageRate, const Transform & localTransform) :
		SensorCapture(imageRate, localTransform*CameraModel::opticalRotation()),
		imuFilter_(0),
		publishInterIMU_(false)
{}

Camera::~Camera()
{
	delete imuFilter_;
}

bool Camera::initFromFile(const std::string & calibrationPath)
{
	return init(UDirectory::getDir(calibrationPath), uSplit(UFile::getName(calibrationPath), '.').front());
}

void Camera::setInterIMUPublishing(bool enabled, IMUFilter * filter)
{
	publishInterIMU_ = enabled;
	delete imuFilter_;
	imuFilter_ = filter;
}

void Camera::postInterIMU(const IMU & imu, double stamp)
{
	if(imuFilter_)
	{
		imuFilter_->update(
				imu.angularVelocity()[0], imu.angularVelocity()[1], imu.angularVelocity()[2],
				imu.linearAcceleration()[0], imu.linearAcceleration()[1], imu.linearAcceleration()[2],
				stamp);
		cv::Vec4d q;
		imuFilter_->getOrientation(q[0],q[1],q[2],q[3]);
		UEventsManager::post(new IMUEvent(IMU(
				q, cv::Mat(),
				imu.angularVelocity(), imu.angularVelocityCovariance(),
				imu.linearAcceleration(), imu.linearAccelerationCovariance(),
				imu.localTransform()),
				stamp));
		return;
	}
	UEventsManager::post(new IMUEvent(imu, stamp));
}

} // namespace rtabmap
