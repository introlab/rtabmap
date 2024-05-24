#pragma once

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
	Transform imuLocalTransform_;
	bool imuPublished_;
	bool publishInterIMU_;
	std::shared_ptr<xv::Device> device_;
	std::map<double, cv::Vec3f> accBuffer_;
	std::map<double, cv::Vec3f> gyroBuffer_;
	UMutex imuMutex_;
#endif
};

} // namespace rtabmap
