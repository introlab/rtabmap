/*
Copyright (c) 2010-2025, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
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

#include "rtabmap/core/CameraModel.h"
#include "rtabmap/core/Camera.h"
#include "rtabmap/core/Version.h"

#ifdef RTABMAP_ORBBEC_SDK
namespace ob
{
    class Pipeline;
    class Align;
}
#endif

namespace rtabmap
{

class RTABMAP_CORE_EXPORT CameraOrbbecSDK :
    public Camera
{
public:
    static bool available();

public:
    // deviceId can be either an index (e.g., "0"), an UID (e.g, "2-1-2" or "gmsl-1") or a serial ("AAA6454S")
    CameraOrbbecSDK(
        std::string deviceId = "",
        int colorWidth = 800,
        int colorHeight = 600,
        int depthWidth = 800,
        int depthHeight = 600,
        float imageRate = 0.0f,
        const Transform & localTransform = Transform::getIdentity());
    virtual ~CameraOrbbecSDK();

    virtual bool init(const std::string & calibrationFolder = ".", const std::string & cameraName = "");
    virtual bool isCalibrated() const;
    virtual std::string getSerial() const;

    void close();

    // Should be set before initializing
    void enableColorRectification(bool enabled);
    void enableImu(bool enabled);

    void enableDepthMM(bool enabled);

protected:
    virtual SensorData captureImage(SensorCaptureInfo * info = 0);

private:

#ifdef RTABMAP_ORBBEC_SDK
    std::string deviceId_;
    int colorWidth_;
    int colorHeight_;
    int depthWidth_;
    int depthHeight_;
    ob::Pipeline * pipeline_;
    ob::Pipeline * imuPipeline_;
    ob::Align * alignFilter_;
    CameraModel model_;
    Transform imuLocalTransform_;
    bool imuLocalTransformInitialized_;
    uint64_t lastAccStamp_;
    uint64_t lastImageStamp_;
    bool globalTimestampAvailable_;
    bool rectifyColor_;
    bool convertDepthToMM_;
    bool imuPublished_;
    std::map<double, cv::Vec6f> imuBuffer_;
    UMutex imuMutex_;
#endif

};


} // namespace rtabmap
