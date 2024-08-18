/*
Copyright (c) 2010-2024, Mathieu Labbe
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the names of its contributors may be used to endorse or promote products
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
#ifndef CORELIB_INCLUDE_RTABMAP_CORE_LIDAR_LIDAROUSTER_H_
#define CORELIB_INCLUDE_RTABMAP_CORE_LIDAR_LIDAROUSTER_H_

#include <rtabmap/core/Lidar.h>

namespace rtabmap {

class IMUFilter;
class OusterCaptureThread;

class RTABMAP_CORE_EXPORT LidarOuster :public Lidar {
public:
	static bool available();
public:
	LidarOuster(
		const std::string& sensorHostname,
		int lidarMode = 0,
		int timestampMode = 0,
		const std::string& dataDestination = 0,
		bool useReflectivityForIntensityChannel = true,
		bool publishIMU = false,
		float frameRate = 0.0f,
		Transform localTransform = Transform::getIdentity());

	LidarOuster(
			const std::string& pcapFile,
			const std::string& jsonFile,
			bool useReflectivityForIntensityChannel = true,
			bool publishIMU = false,
			float frameRate = 0.0f,
			Transform localTransform = Transform::getIdentity());
	virtual ~LidarOuster();

	SensorData takeScan(SensorCaptureInfo * info = 0) {return takeData(info);}

	virtual bool init(const std::string & calibrationFolder = ".", const std::string & cameraName = "") override;
	virtual std::string getSerial() const override;

protected:
    virtual SensorData captureData(SensorCaptureInfo * info = 0) override;

private:
	OusterCaptureThread * ousterCaptureThread_;
	bool imuPublished_;
	bool useReflectivityForIntensityChannel_;
	std::string pcapFile_;
	std::string jsonFile_;
	std::string sensorHostname_;
	std::string dataDestination_;
	int lidarMode_;
	int timestampMode_;

};

} /* namespace rtabmap */

#endif /* CORELIB_INCLUDE_RTABMAP_CORE_LIDAR_LIDAROUSTER_H_ */
