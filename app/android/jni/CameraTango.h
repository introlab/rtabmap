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

#ifndef CAMERATANGO_H_
#define CAMERATANGO_H_

#include "CameraMobile.h"
#include <rtabmap/core/Camera.h>
#include <rtabmap/core/GeodeticCoords.h>
#include <rtabmap/utilite/UMutex.h>
#include <rtabmap/utilite/USemaphore.h>
#include <rtabmap/utilite/UEventsSender.h>
#include <rtabmap/utilite/UThread.h>
#include <rtabmap/utilite/UEvent.h>
#include <rtabmap/utilite/UTimer.h>
#include <boost/thread/mutex.hpp>
#include <tango_client_api.h>
#include <tango_support_api.h>

namespace rtabmap {

class CameraTango : public CameraMobile {
public:
	CameraTango(bool colorCamera, int decimation, bool publishRawScan, bool smoothing);
	virtual ~CameraTango();

	virtual bool init(const std::string & calibrationFolder = ".", const std::string & cameraName = "");
	virtual void close(); // close Tango connection
	virtual std::string getSerial() const;
	rtabmap::Transform tangoPoseToTransform(const TangoPoseData * tangoPose) const;
	void setDecimation(int value) {decimation_ = value;}
	void setRawScanPublished(bool enabled) {rawScanPublished_ = enabled;}

	void cloudReceived(const cv::Mat & cloud, double timestamp);
	void rgbReceived(const cv::Mat & tangoImage, int type, double timestamp);
	void tangoEventReceived(int type, const char * key, const char * value);

protected:
	virtual SensorData updateDataOnRender(Transform & pose);

private:
	rtabmap::Transform getPoseAtTimestamp(double timestamp);

private:
	void * tango_config_;
	bool colorCamera_;
	int decimation_;
	bool rawScanPublished_;
	SensorData tangoData_;
	cv::Mat tangoColor_;
	int tangoColorType_;
	double tangoColorStamp_;
	boost::mutex tangoDataMutex_;
	USemaphore tangoDataReady_;
	cv::Mat fisheyeRectifyMapX_;
	cv::Mat fisheyeRectifyMapY_;
};

} /* namespace rtabmap */
#endif /* CAMERATANGO_H_ */
