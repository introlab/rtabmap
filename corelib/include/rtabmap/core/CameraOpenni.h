/*
 * CameraOpenni.h
 *
 *  Created on: 2013-08-22
 *      Author: Mathieu
 */

#ifndef CAMERAOPENNI_H_
#define CAMERAOPENNI_H_

#include <rtabmap/core/RtabmapExp.h>

#include <pcl/io/openni_camera/openni_depth_image.h>
#include <pcl/io/openni_camera/openni_image.h>

#include <boost/signals2/connection.hpp>

#include <rtabmap/core/Transform.h>
#include <rtabmap/utilite/UEventsSender.h>

class UTimer;

namespace pcl
{
	class Grabber;
}

namespace rtabmap {

class RTABMAP_EXP CameraOpenni : public UEventsSender
{
public:
	// default local transform z in, x right, y down));
	CameraOpenni(const std::string & deviceId="",
			float rate=0,
			const Transform & localTRansform = Transform::getIdentity());
	virtual ~CameraOpenni();

    void image_cb (
    		const boost::shared_ptr<openni_wrapper::Image>& rgb,
			const boost::shared_ptr<openni_wrapper::DepthImage>& depth,
			float constant);

    bool init();
    void start();
    void pause();
    void kill();
    bool isRunning();

    void setFrameRate(float rate);

private:
    pcl::Grabber* interface_;
    std::string deviceId_;
    float rate_;
    UTimer * frameRateTimer_;
    Transform localTransform_; // transform from camera_optical_link to base_link
    int seq_;
    boost::signals2::connection connection_;
};

} /* namespace rtabmap */
#endif /* CAMERAOPENNI_H_ */
