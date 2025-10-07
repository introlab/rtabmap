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
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <rtabmap/core/camera/CameraOrbbecSDK.h>
#include <rtabmap/core/util2d.h>
#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/UThread.h>

#ifdef RTABMAP_ORBBEC_SDK
#include <libobsensor/ObSensor.hpp>
#endif

namespace rtabmap
{
#ifdef RTABMAP_ORBBEC_SDK
Transform obToRtabmap(const OBExtrinsic & t)
{
    return Transform(t.rot[0], t.rot[1], t.rot[2], t.trans[0]/1000.0f,
                     t.rot[3], t.rot[4], t.rot[5], t.trans[1]/1000.0f,
                     t.rot[6], t.rot[7], t.rot[8], t.trans[2]/1000.0f);
}


cv::Mat obColorFrameToCv(const ob::VideoFrame & videoFrame)
{
    cv::Mat rgb;
    switch(videoFrame.getFormat()) {
    case OB_FORMAT_MJPG: {
        cv::Mat rawMat(1, videoFrame.getDataSize(), CV_8UC1, videoFrame.getData());
        rgb = cv::imdecode(rawMat, 1);
    } break;
    case OB_FORMAT_NV21: {
        cv::Mat rawMat(videoFrame.getHeight() * 3 / 2, videoFrame.getWidth(), CV_8UC1, videoFrame.getData());
        cv::cvtColor(rawMat, rgb, cv::COLOR_YUV2BGR_NV21);
    } break;
    case OB_FORMAT_YUYV:
    case OB_FORMAT_YUY2: {
        cv::Mat rawMat(videoFrame.getHeight(), videoFrame.getWidth(), CV_8UC2, videoFrame.getData());
        cv::cvtColor(rawMat, rgb, cv::COLOR_YUV2BGR_YUY2);
    } break;
    case OB_FORMAT_BGR: {
        cv::Mat rawMat(videoFrame.getHeight(), videoFrame.getWidth(), CV_8UC3, videoFrame.getData());
        cv::cvtColor(rawMat, rgb, cv::COLOR_BGR2RGB);
    } break;
    case OB_FORMAT_RGB: {
        cv::Mat rawMat(videoFrame.getHeight(), videoFrame.getWidth(), CV_8UC3, videoFrame.getData());
        cv::cvtColor(rawMat, rgb, cv::COLOR_RGB2BGR);
    } break;
    case OB_FORMAT_RGBA: {
        cv::Mat rawMat(videoFrame.getHeight(), videoFrame.getWidth(), CV_8UC4, videoFrame.getData());
        cv::cvtColor(rawMat, rgb, cv::COLOR_RGBA2BGR);
    } break;
    case OB_FORMAT_BGRA: {
        cv::Mat rawMat(videoFrame.getHeight(), videoFrame.getWidth(), CV_8UC4, videoFrame.getData());
        cv::cvtColor(rawMat, rgb, cv::COLOR_BGRA2RGB);
    } break;
    case OB_FORMAT_UYVY: {
        cv::Mat rawMat(videoFrame.getHeight(), videoFrame.getWidth(), CV_8UC2, videoFrame.getData());
        cv::cvtColor(rawMat, rgb, cv::COLOR_YUV2BGR_UYVY);
    } break;
    case OB_FORMAT_I420: {
        cv::Mat rawMat(videoFrame.getHeight() * 3 / 2, videoFrame.getWidth(), CV_8UC1, videoFrame.getData());
        cv::cvtColor(rawMat, rgb, cv::COLOR_YUV2BGR_I420);
    } break;
    case OB_FORMAT_Y8: {
        rgb = cv::Mat(videoFrame.getHeight(), videoFrame.getWidth(), CV_8UC1, videoFrame.getData()).clone();
    } break;
    case OB_FORMAT_Y16: {
        cv::Mat rawMat(videoFrame.getHeight(), videoFrame.getWidth(), CV_16UC1, videoFrame.getData());
        rawMat.convertTo(rgb, CV_8UC1, 255.0 / 65535.0);
    } break;
    default:
        break;
    }
    return rgb;
}

cv::Mat obDepthFrameToCv(const ob::DepthFrame & depthFrame)
{
    cv::Mat depth;
    if(depthFrame.getFormat() == OB_FORMAT_Y16 || depthFrame.getFormat() == OB_FORMAT_Z16 || depthFrame.getFormat() == OB_FORMAT_Y12C4) {
        cv::Mat rawMat = cv::Mat(depthFrame.getHeight(), depthFrame.getWidth(), CV_16UC1, depthFrame.getData());
        float scale = depthFrame.getValueScale() / 1000.0f;
        rawMat.convertTo(depth, CV_32F, scale);
    }
    return depth;
}

cv::Mat obIntrinsicToK(const OBCameraIntrinsic & intrinsics)
{
    cv::Mat K = cv::Mat::eye(3,3,CV_64FC1);
    K.at<double>(0,0) = intrinsics.fx;
    K.at<double>(1,1) = intrinsics.fy;
    K.at<double>(0,2) = intrinsics.cx;
    K.at<double>(1,2) = intrinsics.cy;
    return K;
}
cv::Mat obIntrinsicToP(const OBCameraIntrinsic & intrinsics)
{
    cv::Mat P = cv::Mat::eye(3,4,CV_64FC1);
    obIntrinsicToK(intrinsics).copyTo(P.colRange(0,3));
    return P;
}
cv::Mat obDistortionToD(const OBCameraDistortion & distortion)
{
    cv::Mat D = cv::Mat(1,8,CV_64FC1);
    D.at<double>(0,0) = distortion.k1;
    D.at<double>(0,1) = distortion.k2;
    D.at<double>(0,2) = distortion.p1;
    D.at<double>(0,3) = distortion.p2;
    D.at<double>(0,4) = distortion.k3;
    D.at<double>(0,5) = distortion.k4;
    D.at<double>(0,6) = distortion.k5;
    D.at<double>(0,7) = distortion.k6;
    if(distortion.k4 == 0 && distortion.k5 == 0 && distortion.k6 == 0)
    {
        D = D.colRange(0,5);
    }
    return D;
}
#endif

bool CameraOrbbecSDK::available()
{
#ifdef RTABMAP_ORBBEC_SDK
    return true;
#else
    return false;
#endif
}

CameraOrbbecSDK::CameraOrbbecSDK(
        std::string deviceId,
        int colorWidth,
        int colorHeight,
        int depthWidth,
        int depthHeight,
        float imageRate,
        const Transform & localTransform) :
            Camera(imageRate, localTransform)
#ifdef RTABMAP_ORBBEC_SDK
            , deviceId_(deviceId),
            colorWidth_(colorWidth),
            colorHeight_(colorHeight),
            depthWidth_(depthWidth),
            depthHeight_(depthHeight),
            pipeline_(nullptr),
            imuPipeline_(nullptr),
            alignFilter_(nullptr),
            imuLocalTransformInitialized_(false),
            lastAccStamp_(0),
            lastImageStamp_(0),
            globalTimestampAvailable_(false),
            rectifyColor_(false),
            convertDepthToMM_(true),
            imuPublished_(true)
#endif
{
}

CameraOrbbecSDK::~CameraOrbbecSDK()
{
#ifdef RTABMAP_ORBBEC_SDK
    this->close();
#endif
}

void CameraOrbbecSDK::close()
{
#ifdef RTABMAP_ORBBEC_SDK
    if(imuPipeline_) {
        imuPipeline_->stop();
        delete imuPipeline_;
        imuPipeline_=nullptr;
    }
    
    if(pipeline_) {
        pipeline_->stop();
        delete pipeline_;
        pipeline_=nullptr;
    }
    delete alignFilter_;
    alignFilter_ = nullptr;
    imuLocalTransform_ = Transform();
    imuLocalTransformInitialized_ = false;
    lastAccStamp_ = 0;
    lastImageStamp_ = 0;
    globalTimestampAvailable_ = false;
    model_ = CameraModel();
    imuBuffer_.clear();
#endif
}

bool CameraOrbbecSDK::init(const std::string & calibrationFolder, const std::string & cameraName)
{
#ifdef RTABMAP_ORBBEC_SDK
    this->close();

    std::shared_ptr<ob::Device> device;

    ob::Context context;
    auto devices = context.queryDeviceList();
    UINFO("%d device(s) found", devices->getCount());
    for(uint32_t i=0; i<devices->getCount(); ++i)
    {
        auto currentDevice = devices->getDevice(i);
        auto info = currentDevice->getDeviceInfo();

        if(deviceId_.find('-') != std::string::npos)
        {
            // UID
            if(deviceId_.compare(info->getUid()) == 0) {
                device = currentDevice;
            }
        }
        else if(uSplitNumChar(deviceId_).size() > 1)
        {
            // Serial
            if(deviceId_.compare(info->getSerialNumber()) == 0) {
                device = currentDevice;
            }
        }
        else if((deviceId_.empty() && i==0) ||
                (!deviceId_.empty() && uStr2Int(deviceId_) == (int)i)) {
            // Index
            device = currentDevice;
        }

        std::string type = "Unknown";
        switch(info->getDeviceType()) 
        {
            case OB_STRUCTURED_LIGHT_MONOCULAR_CAMERA:
                type = "Structured Light Monocular Camera";
                break;
            case OB_STRUCTURED_LIGHT_BINOCULAR_CAMERA:
                type = "Structured Light Binocular Camera";
                break;
            case OB_TOF_CAMERA:
                type = "TOF Camera";
                break;
            default:
                break;
        }
        UINFO("Device %ld:", i);
        UINFO("   Name: %s", info->getName());
        UINFO("   Type: %s", type.c_str());
        UINFO("   Serial: %s", info->getSerialNumber());
        UINFO("   UID: %s", info->getUid());
        UINFO("   Chip: %s", info->getAsicName());
        UINFO("   Hardware version: %s", info->getHardwareVersion());
        UINFO("   Firmware version: %s", info->getFirmwareVersion());
    }

    if(device.get() == nullptr) {
        if(deviceId_.empty()) {
            UERROR( "Could not find any orbbec compatible devices! Verify that the "
                    "camera is correctly connected and the udev rules are installed.");
        }
        else {
            UERROR("Could not find an orbbec device with ID \"%s\"! Verify that the "
                   "camera is correctly connected and the udev rules are installed. "
                   "Unset the ID to choose the first camera found.");
        }
        
        return false;
    }

    bool hasGyro = false;
    bool hasAccel = false;
    auto sensors = device->getSensorList();

    if(device->isGlobalTimestampSupported())
    {
        UINFO("Global (host time sync) timestamp is supported.");
        device->enableGlobalTimestamp(true);
        globalTimestampAvailable_ = true;
    }
    else
    {
        UWARN("Global (host time sync) timestamp is not supported! We will use device timestamp, so the camera frames won't be synchronizable with other sensors.");
    }

    uint32_t maxColorFps = 0;
    uint32_t maxDepthFps = 0;
    for(uint32_t i=0; i<sensors->getCount(); ++i)
    {
        if(sensors->getSensorType(i) == OB_SENSOR_GYRO)
        {
            hasGyro = true;
        }
        if(sensors->getSensorType(i) == OB_SENSOR_ACCEL)
        {
            hasAccel = true;
        }
        if( sensors->getSensorType(i) == OB_SENSOR_DEPTH ||
            sensors->getSensorType(i) == OB_SENSOR_COLOR)
        {
            auto profiles = sensors->getSensor(i)->getStreamProfileList();
            UINFO("Supported %s profiles:", sensors->getSensorType(i) == OB_SENSOR_DEPTH?"depth":"color");
            for(uint32_t j=0; j<profiles->getCount(); ++j)
            {
                auto profile = profiles->getProfile(j)->as<ob::VideoStreamProfile>();
                UINFO("Resolution: %ldx%ld, FPS: %ld, Format: %d", 
                    profile->getWidth(), profile->getHeight(), profile->getFps(), profile->getFormat(), j==0?" (default)":"");
                
                if(sensors->getSensorType(i) == OB_SENSOR_DEPTH) {
                    if(profile->getFps() > maxDepthFps) {
                        maxDepthFps = profile->getFps();
                    }
                }
                else
                {
                    if(profile->getFps() > maxColorFps) {
                        maxColorFps = profile->getFps();
                    }
                }
                
            }
        }
    }

    std::shared_ptr<ob::Config> imuConfig;
    if(imuPublished_)
    {
        if(hasGyro && hasAccel)
        {
            imuPipeline_ = new ob::Pipeline(device);
            imuConfig = std::make_shared<ob::Config>();
            imuConfig->enableGyroStream();
            imuConfig->enableAccelStream();
            try {
                UINFO("Starting imu pipeline");
                imuPipeline_->start(imuConfig, [&](std::shared_ptr<ob::FrameSet> frameSet) {
                    if(frameSet->getCount() != 2)
                    {
                        return;
                    }

                    if(!imuLocalTransformInitialized_)
                    {
                        return;
                    }

                    UASSERT(frameSet->getFrame(OB_FRAME_ACCEL) != nullptr && 
                            frameSet->getFrame(OB_FRAME_GYRO) != nullptr);

                    auto accel = frameSet->getFrame(OB_FRAME_ACCEL)->as<const ob::AccelFrame>();
                    auto gyro = frameSet->getFrame(OB_FRAME_GYRO)->as<const ob::GyroFrame>();

                    uint64_t accelStampUs = globalTimestampAvailable_?accel->getGlobalTimeStampUs():accel->getTimeStampUs();
                    uint64_t gyroStampUs = globalTimestampAvailable_?gyro->getGlobalTimeStampUs():gyro->getTimeStampUs();
    
                    if(accelStampUs != gyroStampUs)
                    {
                        UWARN("Received accel and gyro frames with different timestamps (%llu vs %llu), skipping.",
                            accelStampUs, gyroStampUs);
                        return;
                    }
                    
                    double accStamp = double(accelStampUs)/1e6;

                    if(accelStampUs <= lastAccStamp_) {
                        return;
                    }

                    lastAccStamp_ = accelStampUs;

                    auto accelValue = accel->getValue();
                    auto gyroValue = gyro->getValue();
                    if(isInterIMUPublishing())
                    {
                        IMU imu(cv::Vec3f(gyroValue.x, gyroValue.y, gyroValue.z), cv::Mat::eye(3,3,CV_64FC1),
                                cv::Vec3f(accelValue.x, accelValue.y, accelValue.z), cv::Mat::eye(3,3,CV_64FC1),
                                imuLocalTransform_);
                        this->postInterIMU(imu, accStamp);
                    }
                    else
                    {
                        UScopeMutex lock(imuMutex_);
                        imuBuffer_.emplace_hint(imuBuffer_.end(), accStamp, cv::Vec6f(gyroValue.x, gyroValue.y, gyroValue.z, accelValue.x, accelValue.y, accelValue.z));
                        if(imuBuffer_.size()>1000) {
                            imuBuffer_.erase(imuBuffer_.begin());
                        }
                    }
                });
            }
            catch(const ob::Error & e) {
                UERROR("Unexpected error when configuring IMU stream: %s", e.what());
            }
        }
        else
        {
            UWARN("IMU option is enabled but the camera doesn't have an IMU, ignoring.");
        }
    }

    pipeline_ = new ob::Pipeline(device);
    auto config = std::make_shared<ob::Config>();

    // Set highest frame rate possible to reduce color/depth sync diff
    config->enableVideoStream(OB_STREAM_COLOR, colorWidth_, colorHeight_, maxColorFps, OB_FORMAT_RGB);
    config->enableVideoStream(OB_STREAM_DEPTH, depthWidth_, depthHeight_, maxDepthFps, OB_FORMAT_Y16);

    UINFO("Using color profile: %dx%d", colorWidth_, colorHeight_);
    UINFO("Using depth profile: %dx%d", depthWidth_, depthHeight_);

    config->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_ALL_TYPE_FRAME_REQUIRE);

    config->setAlignMode(ALIGN_DISABLE);
    config->setDepthScaleRequire(true);

    pipeline_->enableFrameSync();

    try {
        UINFO("Starting camera pipeline");
           pipeline_->start(config);

        auto enabledStreams = pipeline_->getConfig()->getEnabledStreamProfileList();
        if(imuPipeline_ != nullptr) {
            for(uint32_t i=0; i<enabledStreams->getCount() && !imuLocalTransformInitialized_; ++i)
            {
                if(enabledStreams->getProfile(i)->getType() == OB_STREAM_COLOR)
                {
                    auto enabledImuStreams = imuPipeline_->getConfig()->getEnabledStreamProfileList();
                    for(uint32_t j=0; j<enabledImuStreams->getCount(); ++j)
                    {
                        if(enabledImuStreams->getProfile(j)->getType() == OB_STREAM_ACCEL)
                        {
                            auto extrinsics = enabledStreams->getProfile(i)->as<ob::VideoStreamProfile>()->getExtrinsicTo(enabledImuStreams->getProfile(j)->as<ob::AccelStreamProfile>());
                            // base -> color -> imu
                            imuLocalTransform_ = this->getLocalTransform() * obToRtabmap(extrinsics);
                            UINFO("IMU local transform: %s", imuLocalTransform_.prettyPrint().c_str());
                            imuLocalTransformInitialized_ = true;
                            break;
                        }
                    }
                }
            }
        }

        std::shared_ptr<ob::StreamProfile> colorProfile;
        std::shared_ptr<ob::StreamProfile> depthProfile;
        for(uint32_t i=0; i<enabledStreams->getCount(); ++i)
        {
            if(enabledStreams->getProfile(i)->getType() == OB_STREAM_COLOR) {
                colorProfile = enabledStreams->getProfile(i);
            }
            else if(enabledStreams->getProfile(i)->getType() == OB_STREAM_DEPTH) {
                depthProfile = enabledStreams->getProfile(i);
            }
        }
        bool currentSelectionSupportsHwD2C = false;
        auto hwD2CSupportedDepthStreamProfiles = pipeline_->getD2CDepthProfileList(colorProfile, ALIGN_D2C_HW_MODE);
        if(hwD2CSupportedDepthStreamProfiles->count() == 0) {
            UWARN("Current color profile selected doesn't support any hardware depth to color registration. Software registration is done instead.");
        }
        else
        {
            auto depthVsp = depthProfile->as<ob::VideoStreamProfile>();
            auto count    = hwD2CSupportedDepthStreamProfiles->getCount();
            for(uint32_t i = 0; i < count; i++) {
                auto vsp = hwD2CSupportedDepthStreamProfiles->getProfile(i)->as<ob::VideoStreamProfile>();
                UINFO("Supported depth to color format: Resolution: %ldx%ld, FPS: %ld, Format: %d", vsp->getWidth(), vsp->getHeight(), vsp->getFps(), vsp->getFormat(), i==0?" (default)":"");
                if(vsp->getWidth() == depthVsp->getWidth() && vsp->getHeight() == depthVsp->getHeight() && vsp->getFormat() == depthVsp->getFormat()
                && vsp->getFps() == depthVsp->getFps()) {
                    currentSelectionSupportsHwD2C = true;
                }
            }
        }
        if(!currentSelectionSupportsHwD2C) {
            UWARN("Hardware depth to color registration cannot be done with the selected color and depth profiles. "
                  "Software registration is done instead, so more CPU will be needed on the host computer. "
                  "Set logger level to info to see comptible depth formats for the selected color profile.");
            alignFilter_ = new ob::Align(OB_STREAM_COLOR);
            alignFilter_->setMatchTargetResolution(false);
        }
        else {
            UINFO("Enabling hardware depth to color registration!");
            config->setAlignMode(ALIGN_D2C_HW_MODE);
            config->setDepthScaleRequire(false);
            pipeline_->stop();
            pipeline_->start(config);
        }
    }
    catch(const ob::Error & e)
    {
        UERROR("Configuration not supported! Exception: %s", e.what());
        UERROR("Supported formats:");
        for(uint32_t i=0; i<sensors->getCount(); ++i)
        {
            if( sensors->getSensorType(i) == OB_SENSOR_DEPTH ||
                sensors->getSensorType(i) == OB_SENSOR_COLOR)
            {
                auto profiles = sensors->getSensor(i)->getStreamProfileList();
                for(uint32_t j=0; j<profiles->getCount(); ++j)
                {
                    auto profile = profiles->getProfile(j)->as<ob::VideoStreamProfile>();
                    UERROR("%sResolution: %ldx%ld, FPS: %ld, Format: %d", 
                        sensors->getSensorType(i) == OB_SENSOR_DEPTH?"Depth":"Color", 
                        profile->getWidth(), 
                        profile->getHeight(), 
                        profile->getFps(), 
                        profile->getFormat(), 
                        j==0?" (default)":"");
                }
            }
        }
        return false;
    }
    
    return true;
#else
    UERROR("CameraOrbbecSDK: RTAB-Map is not built with Orbbec SDK support!");
    return false;
#endif
}

bool CameraOrbbecSDK::isCalibrated() const
{
    return true;
}

std::string CameraOrbbecSDK::getSerial() const
{
#ifdef RTABMAP_ORBBEC_SDK
    if(pipeline_) {
        return pipeline_->getDevice()->getDeviceInfo()->getSerialNumber();
    }
#endif
    return "";
}

void CameraOrbbecSDK::enableColorRectification(bool enabled)
{
#ifdef RTABMAP_ORBBEC_SDK
    rectifyColor_ = enabled;
#endif
}

void CameraOrbbecSDK::enableImu(bool enabled)
{
#ifdef RTABMAP_ORBBEC_SDK
    imuPublished_ = enabled;
#endif
}

void CameraOrbbecSDK::enableDepthMM(bool enabled)
{
#ifdef RTABMAP_ORBBEC_SDK
    convertDepthToMM_ = enabled;
#endif
}

SensorData CameraOrbbecSDK::captureImage(SensorCaptureInfo * info)
{
    SensorData data;

#ifdef RTABMAP_ORBBEC_SDK
    if(!pipeline_) {
        UERROR("Camera is not initialized!");
        return data;
    }
    auto frameset = pipeline_->waitForFrameset();
    if(frameset == nullptr || frameset->getCount() == 0) {
        UWARN("No frame received!");
        return data;
    }
    if(frameset->getCount() != 2) {
        UWARN("Received %s frames, expecting 2!", frameset->getCount());
        return data;
    }

    if(alignFilter_ != nullptr) {
        // Software depth to color registration
        frameset = alignFilter_->process(frameset)->as<ob::FrameSet>();
        UASSERT(frameset != nullptr);
    }

    auto colorFrame = frameset->getFrame(OB_FRAME_COLOR);
    UASSERT(colorFrame != nullptr);
    
    auto depthFrame = frameset->getFrame(OB_FRAME_DEPTH);
    UASSERT(depthFrame != nullptr);

    auto colorVideoFrame = colorFrame->as<const ob::VideoFrame>();
    auto depthVideoFrame = depthFrame->as<const ob::DepthFrame>();

    cv::Mat rgb = obColorFrameToCv(*colorVideoFrame);
    cv::Mat depth = obDepthFrameToCv(*depthVideoFrame);

    if(rgb.empty()) {
        UERROR("Could not convert the color frame! Type=%d Format=%d", colorFrame->getType(), colorVideoFrame->getFormat());
    }
    else if(depth.empty()) {
        UERROR("Could not convert the depth frame! Type=%d Format=%d", depthFrame->getType(), depthVideoFrame->getFormat());
    }
    else if(!rgb.empty() && !depth.empty())
    {
        if(!model_.isValidForProjection())
        {
            auto streamProfile = colorFrame->getStreamProfile();
            auto videoStreamProfile = streamProfile->as<ob::VideoStreamProfile>();

            auto intrinsics = videoStreamProfile->getIntrinsic();
            model_ = CameraModel(
                getSerial(), 
                cv::Size(intrinsics.width, intrinsics.height),
                obIntrinsicToK(intrinsics),
                obDistortionToD(videoStreamProfile->getDistortion()),
                cv::Mat::eye(3,3,CV_64FC1),
                obIntrinsicToP(intrinsics),
                this->getLocalTransform());
            if(rectifyColor_ && !model_.initRectificationMap()) {
                UWARN("Could not initialize rectification map, color images won't be rectified.");
            }
        }
    
        if(rectifyColor_ && model_.isValidForRectification())
        {
            rgb = model_.rectifyImage(rgb);
        }

        if(convertDepthToMM_)
        {
            depth = util2d::cvtDepthFromFloat(depth);
        }

        uint64_t colorStampUs = globalTimestampAvailable_?colorFrame->getGlobalTimeStampUs():colorFrame->getTimeStampUs();
        uint64_t depthStampUs = globalTimestampAvailable_?depthFrame->getGlobalTimeStampUs():depthFrame->getTimeStampUs();
        double colorStamp = double(colorStampUs) / 1e6;
        double depthStamp = double(depthStampUs) / 1e6;
        if(fabs(colorStamp - depthStamp) > 0.018) {
            // The difference seems varying between 0 and 17 ms normally
            UWARN("Large timestamp difference (%fs) between color (%f) and depth (%f) frames. "
                "Depth registration would be wrong on fast motion.", 
                colorStamp - depthStamp, colorStamp, depthStamp);
        }

        uint64_t stampUs = colorStampUs < depthStampUs ? colorStampUs : depthStampUs;

#ifdef WIN32
        // On Windows, there is an issue that timestamps are not populated by default without following instructions from:
        // https://github.com/orbbec/OrbbecSDK_v2/blob/main/scripts/env_setup/obsensor_metadata_win10.md
        // Detect if the consecutive timestamps are identical, then send error!
        if (stampUs <= lastImageStamp_)
        {
            UERROR("We detected non-consecutive timestamps, make sure you applied the fix from https://github.com/orbbec/OrbbecSDK_v2/blob/main/scripts/env_setup/obsensor_metadata_win10.md .");
        }
        lastImageStamp_ = stampUs;
#endif
        double stamp = double(stampUs)/1e6;

        data = SensorData(rgb, depth, model_, this->getNextSeqID(), stamp);

        if(imuPublished_ && !imuBuffer_.empty() && !this->isInterIMUPublishing())
        {
            cv::Vec6f imuVec;
            std::map<double, cv::Vec6f>::const_iterator iterA, iterB;
        
            imuMutex_.lock();
            int maximumTries = 10;
            while(imuBuffer_.rbegin()->first < stamp && maximumTries-- > 0)
            {
                imuMutex_.unlock();
                uSleep(1);
                imuMutex_.lock();
            }

            if(imuBuffer_.rbegin()->first < stamp)
            {
                UWARN("Could not get IMU data at request image stamp %f after waiting 10 ms, latest imu stamp is %f", stamp, imuBuffer_.rbegin()->first);
                imuMutex_.unlock();
            }
            else
            {
                // Interpolate imu data on image stamp
                iterB = imuBuffer_.lower_bound(stamp);
                iterA = iterB;
                if(iterA != imuBuffer_.begin())
                    iterA = --iterA;
                if(iterA == iterB || stamp == iterB->first)
                {
                    imuVec = iterB->second;
                }
                else if(stamp > iterA->first && stamp < iterB->first)
                {
                    float t = (stamp-iterA->first) / (iterB->first-iterA->first);
                    imuVec = iterA->second + t*(iterB->second - iterA->second);
                }
                imuBuffer_.erase(imuBuffer_.begin(), iterB);

                imuMutex_.unlock();
                data.setIMU(IMU(cv::Vec3d(imuVec[0], imuVec[1], imuVec[2]), cv::Mat::eye(3, 3, CV_64FC1), cv::Vec3d(imuVec[3], imuVec[4], imuVec[5]), cv::Mat::eye(3, 3, CV_64FC1), imuLocalTransform_));
            }
        }
    }

#else
    UERROR("CameraOrbbecSDK: RTAB-Map is not built with Orbbec SDK support!");
#endif
    return data;
}

} // namespace rtabmap
