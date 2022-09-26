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

#include "rtabmap/core/CameraModel.h"
#include "rtabmap/core/Camera.h"
#include "rtabmap/core/Version.h"

#include <pcl/pcl_config.h>

#ifdef RTABMAP_REALSENSE2
#include <librealsense2/rs.hpp>
#include <librealsense2/hpp/rs_frame.hpp>
#endif


namespace rs2
{
	class context;
	class device;
	class syncer;
}
struct rs2_intrinsics;
struct rs2_extrinsics;

namespace rtabmap
{

class RTABMAP_CORE_EXPORT CameraRealSense2 :
	public Camera
{
public:
	static bool available();

public:
	CameraRealSense2(
		const std::string & deviceId = "",
		float imageRate = 0,
		const Transform & localTransform = Transform::getIdentity());
	virtual ~CameraRealSense2();

	virtual bool init(const std::string & calibrationFolder = ".", const std::string & cameraName = "");
	virtual bool isCalibrated() const;
	virtual std::string getSerial() const;
	virtual bool odomProvided() const;
	virtual bool getPose(double stamp, Transform & pose, cv::Mat & covariance);

	// parameters are set during initialization
	// D400 series
	void setEmitterEnabled(bool enabled);
	void setIRFormat(bool enabled, bool useDepthInsteadOfRightImage);
	void setResolution(int width, int height, int fps = 30);
	void setDepthResolution(int width, int height, int fps = 30);
	void setGlobalTimeSync(bool enabled);
	void publishInterIMU(bool enabled);
	/**
	 * Dual mode (D400+T265 or L500+T265)
	 * @param enabled enable dual mode
	 * @param extrinsics the extrinsics between T265 pose frame (middle of the camera) to D400/L500 main camera (without optical rotation).
	 */
	void setDualMode(bool enabled, const Transform & extrinsics);
	void setJsonConfig(const std::string & json);
	// T265 related parameters
	void setImagesRectified(bool enabled);
	void setOdomProvided(bool enabled, bool imageStreamsDisabled=false, bool onlyLeftStream = false);

#ifdef RTABMAP_REALSENSE2
private:
	void close();
	void imu_callback(rs2::frame frame);
	void pose_callback(rs2::frame frame);
	void frame_callback(rs2::frame frame);
	void multiple_message_callback(rs2::frame frame);
	void getPoseAndIMU(
			const double & stamp,
			Transform & pose,
			unsigned int & poseConfidence,
			IMU & imu,
			int maxWaitTimeMs = 35);
#endif

protected:
	virtual SensorData captureImage(CameraInfo * info = 0);

private:
#ifdef RTABMAP_REALSENSE2
	rs2::context ctx_;
	std::vector<rs2::device> dev_;
	std::string deviceId_;
	rs2::syncer syncer_;
	float depth_scale_meters_;
	cv::Mat depthBuffer_;
	cv::Mat rgbBuffer_;
	CameraModel model_;
	StereoCameraModel stereoModel_;
	Transform imuLocalTransform_;
	std::map<double, cv::Vec3f> accBuffer_;
	std::map<double, cv::Vec3f> gyroBuffer_;
	std::map<double, std::pair<Transform, unsigned int> > poseBuffer_; // <stamp, <Pose, confidence: 1=lost, 2=low, 3=high> >
	UMutex poseMutex_;
	UMutex imuMutex_;
	double lastImuStamp_;
	bool clockSyncWarningShown_;
	bool imuGlobalSyncWarningShown_;

	bool emitterEnabled_;
	bool ir_;
	bool irDepth_;
	bool rectifyImages_;
	bool odometryProvided_;
	bool odometryImagesDisabled_;
	bool odometryOnlyLeftStream_;
	int cameraWidth_;
	int cameraHeight_;
	int cameraFps_;
	int cameraDepthWidth_;
	int cameraDepthHeight_;
	int cameraDepthFps_;
	bool globalTimeSync_;
	bool publishInterIMU_;
	bool dualMode_;
	Transform dualExtrinsics_;
	std::string jsonConfig_;
	bool closing_;

	static Transform realsense2PoseRotation_;
	static Transform realsense2PoseRotationInv_;
#endif
};


} // namespace rtabmap
