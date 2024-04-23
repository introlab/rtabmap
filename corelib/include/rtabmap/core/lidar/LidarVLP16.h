/*
Copyright (c) 2010-2022, Mathieu Labbe
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
#ifndef CORELIB_INCLUDE_RTABMAP_CORE_LIDAR_LIDARVLP16_H_
#define CORELIB_INCLUDE_RTABMAP_CORE_LIDAR_LIDARVLP16_H_

// Should be first on windows to avoid "WinSock.h has already been included" error
#include <pcl/io/vlp_grabber.h>

#include <rtabmap/core/Lidar.h>
#include <rtabmap/utilite/USemaphore.h>

namespace rtabmap {

struct PointXYZIT {
	float x;
	float y;
	float z;
	float i;
	float t;
};

class RTABMAP_CORE_EXPORT LidarVLP16 :public Lidar, public pcl::VLPGrabber {
public:
	LidarVLP16(
			const std::string& pcapFile,
			bool organized = false,
			bool stampLast = true,
			float frameRate = 0.0f,
			Transform localTransform = Transform::getIdentity());
	LidarVLP16(
			const boost::asio::ip::address& ipAddress,
			const std::uint16_t port = 2368,
			bool organized = false,
			bool useHostTime = true,
			bool stampLast = true,
			float frameRate = 0.0f,
			Transform localTransform = Transform::getIdentity());
	virtual ~LidarVLP16();

	SensorData takeScan(SensorCaptureInfo * info = 0) {return takeData(info);}

	virtual bool init(const std::string & calibrationFolder = ".", const std::string & cameraName = "") override;
	virtual std::string getSerial() const override {return getName();}

	void setOrganized(bool enable);

private:
	void buildTimings(bool dualMode);
    virtual void toPointClouds (HDLDataPacket *dataPacket) override;

protected:
    virtual SensorData captureData(SensorCaptureInfo * info = 0) override;

private:
    // timing offset lookup table
    std::vector< std::vector<float> > timingOffsets_;
    bool timingOffsetsDualMode_;
    double startSweepTime_;
    double startSweepTimeHost_;
    bool organized_;
    bool useHostTime_;
    bool stampLast_;
    SensorData lastScan_;
    std::vector<std::vector<PointXYZIT> > accumulatedScans_;
    USemaphore scanReady_;
    UMutex lastScanMutex_;
};

} /* namespace rtabmap */

#endif /* CORELIB_INCLUDE_RTABMAP_CORE_LIDAR_LIDARVLP16_H_ */
