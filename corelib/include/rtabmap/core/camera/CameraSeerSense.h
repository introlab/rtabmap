#pragma once

#include "rtabmap/core/Camera.h"
#include "rtabmap/core/Version.h"
#include "rtabmap/utilite/USemaphore.h"

#ifdef RTABMAP_XVSDK
#include <xv-sdk.h>
#endif

namespace rtabmap
{

class RTABMAP_CORE_EXPORT CameraSeerSense :
	public Camera
{
public:
	static bool available();

public:
	CameraSeerSense(
		bool computeOdometry = false,
		float imageRate = 0.0f,
		const Transform & localTransform = Transform::getIdentity()
	);
	virtual ~CameraSeerSense();

	virtual bool init(const std::string & calibrationFolder = ".", const std::string & cameraName = "");
	virtual bool isCalibrated() const;
	virtual std::string getSerial() const;
	virtual bool odomProvided() const;
	virtual bool getPose(double stamp, Transform & pose, cv::Mat & covariance, double maxWaitTime = 0.0);

protected:
	virtual SensorData captureImage(SensorCaptureInfo * info = 0);

private:
#ifdef RTABMAP_XVSDK
	CameraModel cameraModel_;
	bool computeOdometry_;
	int imuId_;
	int tofId_;
	std::shared_ptr<xv::Device> device_;
	std::map<double, std::pair<cv::Vec3d, cv::Vec3d>> imuBuffer_;
	std::pair<double, std::pair<cv::Mat, cv::Mat>> lastData_;
	UMutex imuMutex_;
	UMutex dataMutex_;
	USemaphore dataReady_;
#endif
};

} // namespace rtabmap
