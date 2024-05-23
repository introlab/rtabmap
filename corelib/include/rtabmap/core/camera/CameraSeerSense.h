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
	CameraSeerSense();
	virtual ~CameraSeerSense();
};

} // namespace rtabmap
