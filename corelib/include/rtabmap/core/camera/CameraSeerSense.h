#pragma once

#include "rtabmap/core/CameraModel.h"
#include "rtabmap/core/Camera.h"
#include "rtabmap/core/Version.h"

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
		float imageRate=0.0f,
		const Transform & localTransform = Transform::getIdentity()
	);
	virtual ~CameraSeerSense();

	void setIMU(bool imuPublished, bool publishInterIMU);

	virtual bool init(const std::string & calibrationFolder = ".", const std::string & cameraName = "");

private:
#ifdef RTABMAP_XVSDK
	CameraModel cameraModel_;
	bool imuPublished_;
	bool publishInterIMU_;
	std::shared_ptr<xv::Device> device_;
	std::map<double, std::pair<cv::Vec3d, cv::Vec3d>> imuBuffer_;
	std::pair<double, std::pair<cv::Mat, cv::Mat>> lastData_;
	UMutex imuMutex_;
	UMutex dataMutex_;
#endif
};

} // namespace rtabmap
