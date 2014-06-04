/*
 * CameraFreenect.h
 *
 *  Created on: 2014-06-02
 *      Author: Mathieu
 */

#ifndef CAMERAFREENECT_H_
#define CAMERAFREENECT_H_

#include <rtabmap/core/RtabmapExp.h>

#include <rtabmap/core/Transform.h>
#include <rtabmap/utilite/UEventsSender.h>
#include <rtabmap/utilite/UThread.h>

#include <opencv2/opencv.hpp>

class UTimer;
typedef struct _freenect_context freenect_context;
typedef struct _freenect_device freenect_device;

namespace rtabmap {

class RTABMAP_EXP FreenectDevice {
  public:
	FreenectDevice(freenect_context *ctx, int index);
	virtual ~FreenectDevice();
	void startVideo();
	void stopVideo();
	void startDepth();
	void stopDepth();

	bool init();

	cv::Mat getRgb();
	cv::Mat getDepth();

	// Do not call directly even in child
	void VideoCallback(void *video, uint32_t timestamp);
	// Do not call directly even in child
	void DepthCallback(void *depth, uint32_t timestamp);
  private:
	static void freenect_depth_callback(freenect_device *dev, void *depth, uint32_t timestamp);
	static void freenect_video_callback(freenect_device *dev, void *video, uint32_t timestamp);

	//noncopyable
	FreenectDevice( const FreenectDevice& );
	const FreenectDevice& operator=( const FreenectDevice& );

  private:
	int index_;
	freenect_context * ctx_;
	freenect_device * device_;
	cv::Mat depthMat_;
	cv::Mat rgbMat_;
	UMutex depthMutex_;
	UMutex rgbMutex_;
	bool depthReady_;
	bool rgbReady_;
};

class RTABMAP_EXP CameraFreenect : public UEventsSender, public UThread
{
public:
	static bool available();

public:
	// default local transform z in, x right, y down));
	CameraFreenect(int deviceId= 0,
			float rate=0,
			const Transform & localTransform = Transform::getIdentity());
	virtual ~CameraFreenect();

    bool init();

    void setFrameRate(float rate);

private:
    virtual void mainLoopBegin();
    virtual void mainLoop();
    virtual void mainLoopEnd();

private:
    int deviceId_;
    float rate_;
    UTimer * frameRateTimer_;
    Transform localTransform_; // transform from camera_optical_link to base_link
    int seq_;
    freenect_context * ctx_;
    FreenectDevice * freenectDevice_;
};

} /* namespace rtabmap */
#endif /* CAMERAFREENECT_H_ */
