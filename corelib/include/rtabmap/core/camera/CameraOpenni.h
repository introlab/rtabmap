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

#pragma once

#include "rtabmap/utilite/UMutex.h"
#include "rtabmap/utilite/USemaphore.h"
#include "rtabmap/core/Camera.h"
#include "rtabmap/core/Version.h"

#include <pcl/pcl_config.h>

#ifdef HAVE_OPENNI
#if __linux__ && __i386__ && __cplusplus >= 201103L
#warning "Openni driver is not available on i386 when building with c++11 support"
#else
#define RTABMAP_OPENNI
#include <pcl/io/openni_camera/openni_depth_image.h>
#include <pcl/io/openni_camera/openni_image.h>
#endif
#endif

#include <boost/signals2/connection.hpp>

namespace pcl
{
class Grabber;
}

namespace rtabmap
{

class RTABMAP_CORE_EXPORT CameraOpenni :
	public Camera
{
public:
	static bool available();

public:
	// default local transform z in, x right, y down));
	CameraOpenni(const std::string & deviceId="",
			float imageRate = 0,
			const Transform & localTransform = Transform::getIdentity());
	virtual ~CameraOpenni();
#ifdef RTABMAP_OPENNI
#if PCL_VERSION_COMPARE(>=, 1, 10, 0)
	void image_cb (
			const std::shared_ptr<openni_wrapper::Image>& rgb,
			const std::shared_ptr<openni_wrapper::DepthImage>& depth,
			float constant);
#else
    void image_cb (
    		const boost::shared_ptr<openni_wrapper::Image>& rgb,
			const boost::shared_ptr<openni_wrapper::DepthImage>& depth,
			float constant);
#endif
#endif

    virtual bool init(const std::string & calibrationFolder = ".", const std::string & cameraName = "");
    virtual bool isCalibrated() const;
    virtual std::string getSerial() const;

protected:
	virtual SensorData captureImage(CameraInfo * info = 0);

private:
    pcl::Grabber* interface_;
    std::string deviceId_;
    boost::signals2::connection connection_;
    cv::Mat depth_;
    cv::Mat rgb_;
    float depthConstant_;
    UMutex dataMutex_;
    USemaphore dataReady_;
};

} // namespace rtabmap
