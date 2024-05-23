#include <rtabmap/core/camera/CameraSeerSense.h>

namespace rtabmap {

bool CameraSeerSense::available()
{
#ifdef RTABMAP_XVSDK
	return true;
#else
	return false;
#endif
}

CameraSeerSense::CameraSeerSense()
{
}

CameraSeerSense::~CameraSeerSense()
{
}

} // namespace rtabmap
