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
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <rtabmap/core/camera/CameraFreenect.h>
#include <rtabmap/utilite/UThread.h>
#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/core/util2d.h>
#include <opencv2/imgproc/types_c.h>

#ifdef RTABMAP_FREENECT
#include <libfreenect.h>
#ifdef FREENECT_DASH_INCLUDES
#include <libfreenect-registration.h>
#else
#include <libfreenect_registration.h>
#endif
#endif

namespace rtabmap
{

#ifdef RTABMAP_FREENECT

class FreenectDevice : public UThread {
  public:
	FreenectDevice(freenect_context * ctx, int index, bool color = true, bool registered = true) :
		index_(index),
		color_(color),
		registered_(registered),
		ctx_(ctx),
		device_(0),
		depthFocal_(0.0f)
	{
		UASSERT(ctx_ != 0);
	}

	virtual ~FreenectDevice()
	{
		this->join(true);
		if(device_ && freenect_close_device(device_) < 0){} //FN_WARNING("Device did not shutdown in a clean fashion");
	}

	const std::string & getSerial() const {return serial_;}

	bool init()
	{
		if(device_)
		{
			this->join(true);
			freenect_close_device(device_);
			device_ = 0;
		}
		serial_.clear();
		std::vector<std::string> deviceSerials;
		freenect_device_attributes* attr_list;
		freenect_device_attributes* item;
		freenect_list_device_attributes(ctx_, &attr_list);
		for (item = attr_list; item != NULL; item = item->next) {
			deviceSerials.push_back(std::string(item->camera_serial));
		}
		freenect_free_device_attributes(attr_list);

		if(freenect_open_device(ctx_, &device_, index_) < 0)
		{
			UERROR("FreenectDevice: Cannot open Kinect");
			return false;
		}

		if(index_ >= 0 && index_ < (int)deviceSerials.size())
		{
			serial_ = deviceSerials[index_];
		}
		else
		{
			UERROR("Could not get serial for index %d", index_);
		}

		UINFO("color=%d registered=%d", color_?1:0, registered_?1:0);

		freenect_set_user(device_, this);
		freenect_frame_mode videoMode = freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, color_?FREENECT_VIDEO_RGB:FREENECT_VIDEO_IR_8BIT);
		freenect_frame_mode depthMode = freenect_find_depth_mode(FREENECT_RESOLUTION_MEDIUM, color_ && registered_?FREENECT_DEPTH_REGISTERED:FREENECT_DEPTH_MM);
		if(!videoMode.is_valid)
		{
			UERROR("Freenect: video mode selected not valid!");
			return false;
		}
		if(!depthMode.is_valid)
		{
			UERROR("Freenect: depth mode selected not valid!");
			return false;
		}
		UASSERT(videoMode.data_bits_per_pixel == 8 || videoMode.data_bits_per_pixel == 24);
		UASSERT(depthMode.data_bits_per_pixel == 16);
		freenect_set_video_mode(device_, videoMode);
		freenect_set_depth_mode(device_, depthMode);
		rgbIrBuffer_ = cv::Mat(cv::Size(videoMode.width,videoMode.height), color_?CV_8UC3:CV_8UC1);
		depthBuffer_ = cv::Mat(cv::Size(depthMode.width,depthMode.height), CV_16UC1);
		freenect_set_depth_buffer(device_, depthBuffer_.data);
		freenect_set_video_buffer(device_, rgbIrBuffer_.data);
		freenect_set_depth_callback(device_, freenect_depth_callback);
		freenect_set_video_callback(device_, freenect_video_callback);

		float rgb_focal_length_sxga = 1050.0f;
		float width_sxga = 1280.0f;
		float width = freenect_get_current_depth_mode(device_).width;
		float scale = width / width_sxga;
		if(color_ && registered_)
		{
			depthFocal_ =  rgb_focal_length_sxga * scale;
		}
		else
		{
			freenect_registration reg = freenect_copy_registration(device_);
			float depth_focal_length_sxga = reg.zero_plane_info.reference_distance / reg.zero_plane_info.reference_pixel_size;
			freenect_destroy_registration(&reg);

			depthFocal_ =  depth_focal_length_sxga * scale;
		}

		UINFO("FreenectDevice: Depth focal = %f", depthFocal_);
		return true;
	}

	float getDepthFocal() const {return depthFocal_;}

	void getData(cv::Mat & rgb, cv::Mat & depth)
	{
		if(this->isRunning())
		{
			if(!dataReady_.acquire(1, 5000))
			{
				UERROR("Not received any frames since 5 seconds, try to restart the camera again.");
			}
			else
			{
				UScopeMutex s(dataMutex_);
				rgb = rgbIrLastFrame_;
				depth = depthLastFrame_;
				rgbIrLastFrame_ = cv::Mat();
				depthLastFrame_= cv::Mat();
			}
		}
	}

	void getAccelerometerValues(double & x, double & y, double & z)
	{
		freenect_update_tilt_state(device_);
		freenect_raw_tilt_state* state = freenect_get_tilt_state(device_);
		freenect_get_mks_accel(state, &x,&y,&z);
	}

private:
	// Do not call directly even in child
	void VideoCallback(void* rgb)
	{
		UASSERT(rgbIrBuffer_.data == rgb);
		UScopeMutex s(dataMutex_);
		bool notify = rgbIrLastFrame_.empty();

		if(color_)
		{
			cv::cvtColor(rgbIrBuffer_, rgbIrLastFrame_, CV_RGB2BGR);
		}
		else // IrDepth
		{
			rgbIrLastFrame_ = rgbIrBuffer_.clone();
		}
		if(!depthLastFrame_.empty() && notify)
		{
			dataReady_.release();
		}
	}

	// Do not call directly even in child
	void DepthCallback(void* depth)
	{
		UASSERT(depthBuffer_.data == depth);
		UScopeMutex s(dataMutex_);
		bool notify = depthLastFrame_.empty();
		depthLastFrame_ = depthBuffer_.clone();
		if(!rgbIrLastFrame_.empty() && notify)
		{
			dataReady_.release();
		}
	}

	void startVideo() {
		if(device_ && freenect_start_video(device_) < 0) UERROR("Cannot start RGB callback");
	}
	void stopVideo() {
		if(device_ && freenect_stop_video(device_) < 0) UERROR("Cannot stop RGB callback");
	}
	void startDepth() {
		if(device_ && freenect_start_depth(device_) < 0) UERROR("Cannot start depth callback");
	}
	void stopDepth() {
		if(device_ && freenect_stop_depth(device_) < 0) UERROR("Cannot stop depth callback");
	}

	virtual void mainLoopBegin()
	{
		if(device_) freenect_set_led(device_, LED_RED);
		this->startDepth();
		this->startVideo();
	}

	virtual void mainLoop()
	{
		timeval t;
		t.tv_sec = 0;
		t.tv_usec = 10000;
		if(freenect_process_events_timeout(ctx_, &t) < 0)
		{
			UERROR("FreenectDevice: Cannot process freenect events");
			this->kill();
		}
	}

	virtual void mainLoopEnd()
	{
		if(device_) freenect_set_led(device_, LED_GREEN);
		this->stopDepth();
		this->stopVideo();
		dataReady_.release();
	}

	static void freenect_depth_callback(freenect_device *dev, void *depth, uint32_t timestamp) {
		FreenectDevice* device = static_cast<FreenectDevice*>(freenect_get_user(dev));
		device->DepthCallback(depth);
	}
	static void freenect_video_callback(freenect_device *dev, void *video, uint32_t timestamp) {
		FreenectDevice* device = static_cast<FreenectDevice*>(freenect_get_user(dev));
		device->VideoCallback(video);
	}

	//noncopyable
	FreenectDevice( const FreenectDevice& );
	const FreenectDevice& operator=( const FreenectDevice& );

  private:
	int index_;
	bool color_;
	bool registered_;
	std::string serial_;
	freenect_context * ctx_;
	freenect_device * device_;
	cv::Mat depthBuffer_;
	cv::Mat rgbIrBuffer_;
	UMutex dataMutex_;
	cv::Mat depthLastFrame_;
	cv::Mat rgbIrLastFrame_;
	float depthFocal_;
	USemaphore dataReady_;
};
#endif

//
// CameraFreenect
//
bool CameraFreenect::available()
{
#ifdef RTABMAP_FREENECT
	return true;
#else
	return false;
#endif
}

CameraFreenect::CameraFreenect(int deviceId, Type type, float imageRate, const Transform & localTransform) :
		Camera(imageRate, localTransform)
#ifdef RTABMAP_FREENECT
        ,
		deviceId_(deviceId),
		type_(type),
		ctx_(0),
		freenectDevice_(0)
#endif
{
#ifdef RTABMAP_FREENECT
	if(freenect_init(&ctx_, NULL) < 0) UERROR("Cannot initialize freenect library");
	// claim camera
	freenect_select_subdevices(ctx_, static_cast<freenect_device_flags>(FREENECT_DEVICE_MOTOR | FREENECT_DEVICE_CAMERA));
#endif
}

CameraFreenect::~CameraFreenect()
{
#ifdef RTABMAP_FREENECT
	if(freenectDevice_)
	{
		freenectDevice_->join(true);
		delete freenectDevice_;
		freenectDevice_ = 0;
	}
	if(ctx_)
	{
		if(freenect_shutdown(ctx_) < 0){} //FN_WARNING("Freenect did not shutdown in a clean fashion");
	}
#endif
}

bool CameraFreenect::init(const std::string & calibrationFolder, const std::string & cameraName)
{
#ifdef RTABMAP_FREENECT
	if(freenectDevice_)
	{
		freenectDevice_->join(true);
		delete freenectDevice_;
		freenectDevice_ = 0;
	}

	if(ctx_ && freenect_num_devices(ctx_) > 0)
	{
		// look for calibration files
		bool hardwareRegistration = true;
		stereoModel_ = StereoCameraModel();
		if(!calibrationFolder.empty())
		{
			// we need the serial, HACK: init a temp device to get it
			FreenectDevice dev(ctx_, deviceId_);
			if(!dev.init())
			{
				UERROR("CameraFreenect: Init failed!");
			}
			std::string calibrationName = dev.getSerial();
			if(!cameraName.empty())
			{
				calibrationName = cameraName;
			}
			stereoModel_.setName(calibrationName, "depth", "rgb");
			hardwareRegistration = !stereoModel_.load(calibrationFolder, calibrationName, false);

			if(type_ == kTypeIRDepth)
			{
				hardwareRegistration = false;
			}


			if((type_ == kTypeIRDepth && !stereoModel_.left().isValidForRectification()) ||
			   (type_ == kTypeColorDepth && !stereoModel_.right().isValidForRectification()))
			{
				UWARN("Missing calibration files for camera \"%s\" in \"%s\" folder, default calibration used.",
						calibrationName.c_str(), calibrationFolder.c_str());
			}
			else if(type_ == kTypeColorDepth && stereoModel_.right().isValidForRectification() && hardwareRegistration)
			{
				UWARN("Missing extrinsic calibration file for camera \"%s\" in \"%s\" folder, default registration is used even if rgb is rectified!",
						calibrationName.c_str(), calibrationFolder.c_str());
			}
			else if(type_ == kTypeColorDepth && stereoModel_.right().isValidForRectification() && !hardwareRegistration)
			{
				UINFO("Custom calibration files for \"%s\" were found in \"%s\" folder. To use "
					  "factory calibration, remove the corresponding files from that directory.", calibrationName.c_str(), calibrationFolder.c_str());
			}
		}

		freenectDevice_ = new FreenectDevice(ctx_, deviceId_, type_==kTypeColorDepth, hardwareRegistration);
		if(freenectDevice_->init())
		{
			freenectDevice_->start();
			uSleep(3000);
			return true;
		}
		else
		{
			UERROR("CameraFreenect: Init failed!");
		}
		delete freenectDevice_;
		freenectDevice_ = 0;
	}
	else
	{
		UERROR("CameraFreenect: No devices connected!");
	}
#else
	UERROR("CameraFreenect: RTAB-Map is not built with Freenect support!");
#endif
	return false;
}

bool CameraFreenect::isCalibrated() const
{
	return true;
}

std::string CameraFreenect::getSerial() const
{
#ifdef RTABMAP_FREENECT
	if(freenectDevice_)
	{
		return freenectDevice_->getSerial();
	}
#endif
	return "";
}

SensorData CameraFreenect::captureImage(CameraInfo * info)
{
	SensorData data;
#ifdef RTABMAP_FREENECT
	if(ctx_ && freenectDevice_)
	{
		if(freenectDevice_->isRunning())
		{
			cv::Mat depth,rgb;
			freenectDevice_->getData(rgb, depth);
			if(!rgb.empty() && !depth.empty())
			{
				UASSERT(freenectDevice_->getDepthFocal() != 0.0f);

				// default calibration
				CameraModel model(
						freenectDevice_->getDepthFocal(), //fx
						freenectDevice_->getDepthFocal(), //fy
						float(rgb.cols/2) - 0.5f,  //cx
						float(rgb.rows/2) - 0.5f,  //cy
						this->getLocalTransform(),
						0,
						rgb.size());

				if(type_==kTypeIRDepth)
				{
					if(stereoModel_.left().isValidForRectification())
					{
						rgb = stereoModel_.left().rectifyImage(rgb);
						depth = stereoModel_.left().rectifyImage(depth, 0);
						model = stereoModel_.left();
					}
				}
				else
				{
					if(stereoModel_.right().isValidForRectification())
					{
						rgb = stereoModel_.right().rectifyImage(rgb);
						model = stereoModel_.right();

						if(stereoModel_.left().isValidForRectification() && !stereoModel_.stereoTransform().isNull())
						{
							depth = stereoModel_.left().rectifyImage(depth, 0);
							depth = util2d::registerDepth(depth, stereoModel_.left().K(), rgb.size(), stereoModel_.right().K(), stereoModel_.stereoTransform());
						}
					}
				}
				model.setLocalTransform(this->getLocalTransform());

				data = SensorData(rgb, depth, model, this->getNextSeqID(), UTimer::now());

				double x=0,y=0,z=0;
				freenectDevice_->getAccelerometerValues(x,y,z);
				if(x != 0.0 && y != 0.0 && z != 0.0)
				{
					// frame of imu on kinect is x->right, y->down, z->backward
					data.setIMU(IMU(cv::Vec3d(0,0,0), cv::Mat(), cv::Vec3d(x, y, z), cv::Mat(), Transform(0,0,-1,0, -1,0,0,0, 0,-1,0,0)));
				}
			}
		}
		else
		{
			UERROR("CameraFreenect: Re-initialization needed!");
			delete freenectDevice_;
			freenectDevice_ = 0;
		}
	}
#else
	UERROR("CameraFreenect: RTAB-Map is not built with Freenect support!");
#endif
	return data;
}


} // namespace rtabmap
