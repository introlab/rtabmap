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

#include <rtabmap/core/camera/CameraStereoZed.h>
#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/utilite/UThreadC.h>
#include <rtabmap/utilite/UConversion.h>

#ifdef RTABMAP_ZED
#include <sl/Camera.hpp>
#endif

namespace rtabmap
{

bool CameraStereoZed::available()
{
#ifdef RTABMAP_ZED
	return true;
#else
	return false;
#endif
}

CameraStereoZed::CameraStereoZed(
		int deviceId,
		int resolution,
		int quality,
		int sensingMode,
		int confidenceThr,
		bool computeOdometry,
		float imageRate,
		const Transform & localTransform,
		bool selfCalibration,
		bool odomForce3DoF) :
	Camera(imageRate, localTransform)
#ifdef RTABMAP_ZED
    ,
	zed_(0),
	src_(CameraVideo::kUsbDevice),
	usbDevice_(deviceId),
	svoFilePath_(""),
	resolution_(resolution),
	quality_(quality),
	selfCalibration_(selfCalibration),
	sensingMode_(sensingMode),
	confidenceThr_(confidenceThr),
	computeOdometry_(computeOdometry),
	lost_(true),
	force3DoF_(odomForce3DoF)
#endif
{
	UDEBUG("");
#ifdef RTABMAP_ZED
	UASSERT(resolution_ >= sl::RESOLUTION_HD2K && resolution_ <sl::RESOLUTION_LAST);
	UASSERT(quality_ >= sl::DEPTH_MODE_NONE && quality_ <sl::DEPTH_MODE_LAST);
	UASSERT(sensingMode_ >= sl::SENSING_MODE_STANDARD && sensingMode_ <sl::SENSING_MODE_LAST);
	UASSERT(confidenceThr_ >= 0 && confidenceThr_ <=100);
#endif
}

CameraStereoZed::CameraStereoZed(
		const std::string & filePath,
		int quality,
		int sensingMode,
		int confidenceThr,
		bool computeOdometry,
		float imageRate,
		const Transform & localTransform,
		bool selfCalibration,
		bool odomForce3DoF) :
	Camera(imageRate, localTransform)
#ifdef RTABMAP_ZED
    ,
	zed_(0),
	src_(CameraVideo::kVideoFile),
	usbDevice_(0),
	svoFilePath_(filePath),
	resolution_(2),
	quality_(quality),
	selfCalibration_(selfCalibration),
	sensingMode_(sensingMode),
	confidenceThr_(confidenceThr),
	computeOdometry_(computeOdometry),
	lost_(true),
	force3DoF_(odomForce3DoF)
#endif
{
	UDEBUG("");
#ifdef RTABMAP_ZED
	UASSERT(resolution_ >= sl::RESOLUTION_HD2K && resolution_ <sl::RESOLUTION_LAST);
	UASSERT(quality_ >= sl::DEPTH_MODE_NONE && quality_ <sl::DEPTH_MODE_LAST);
	UASSERT(sensingMode_ >= sl::SENSING_MODE_STANDARD && sensingMode_ <sl::SENSING_MODE_LAST);
	UASSERT(confidenceThr_ >= 0 && confidenceThr_ <=100);
#endif
}

CameraStereoZed::~CameraStereoZed()
{
#ifdef RTABMAP_ZED
	delete zed_;
#endif
}

bool CameraStereoZed::init(const std::string & calibrationFolder, const std::string & cameraName)
{
	UDEBUG("");
#ifdef RTABMAP_ZED
	if(zed_)
	{
		delete zed_;
		zed_ = 0;
	}
	
	lost_ = true;

	sl::InitParameters param;
	param.camera_resolution=static_cast<sl::RESOLUTION>(resolution_);
	param.camera_fps=getImageRate();
	param.camera_linux_id=usbDevice_;
	param.depth_mode=(sl::DEPTH_MODE)quality_;
	param.coordinate_units=sl::UNIT_METER;
	param.coordinate_system=(sl::COORDINATE_SYSTEM)sl::COORDINATE_SYSTEM_IMAGE ;
	param.sdk_verbose=true;
	param.sdk_gpu_id=-1;
	param.depth_minimum_distance=-1;
	param.camera_disable_self_calib=!selfCalibration_;

	sl::ERROR_CODE r = sl::ERROR_CODE::SUCCESS;
	if(src_ == CameraVideo::kVideoFile)
	{
		UINFO("svo file = %s", svoFilePath_.c_str());
		zed_ = new sl::Camera(); // Use in SVO playback mode
		param.svo_input_filename=svoFilePath_.c_str();
		r = zed_->open(param);
	}
	else
	{
		UINFO("Resolution=%d imagerate=%f device=%d", resolution_, getImageRate(), usbDevice_);
		zed_ = new sl::Camera(); // Use in Live Mode
		r = zed_->open(param);
	}

	if(r!=sl::ERROR_CODE::SUCCESS)
	{
		UERROR("Camera initialization failed: \"%s\"", toString(r).c_str());
		delete zed_;
		zed_ = 0;
		return false;
	}


	UINFO("Init ZED: Mode=%d Unit=%d CoordinateSystem=%d Verbose=false device=-1 minDist=-1 self-calibration=%s vflip=false",
	      quality_, sl::UNIT_METER, sl::COORDINATE_SYSTEM_IMAGE , selfCalibration_?"true":"false");
	UDEBUG("");

	if(quality_!=sl::DEPTH_MODE_NONE)
	{
		zed_->setConfidenceThreshold(confidenceThr_);
	}

	if (computeOdometry_)
	{
		sl::TrackingParameters tparam;
		tparam.enable_spatial_memory=false;
		zed_->enableTracking(tparam);
		if(r!=sl::ERROR_CODE::SUCCESS)
		{
			UERROR("Camera tracking initialization failed: \"%s\"", toString(r).c_str());
		}
	}

	sl::CameraInformation infos = zed_->getCameraInformation();
	sl::CalibrationParameters *stereoParams = &(infos.calibration_parameters );
	sl::Resolution res = stereoParams->left_cam.image_size;

	stereoModel_ = StereoCameraModel(
		stereoParams->left_cam.fx, 
		stereoParams->left_cam.fy, 
		stereoParams->left_cam.cx, 
		stereoParams->left_cam.cy, 
		stereoParams->T[0],//baseline
		this->getLocalTransform(),
		cv::Size(res.width, res.height));

	UINFO("Calibration: fx=%f, fy=%f, cx=%f, cy=%f, baseline=%f, width=%d, height=%d, transform=%s",
			stereoParams->left_cam.fx,
			stereoParams->left_cam.fy,
			stereoParams->left_cam.cx,
			stereoParams->left_cam.cy,
			stereoParams->T[0],//baseline
			(int)res.width,
			(int)res.height,
			this->getLocalTransform().prettyPrint().c_str());

	return true;
#else
	UERROR("CameraStereoZED: RTAB-Map is not built with ZED sdk support!");
#endif
	return false;
}

bool CameraStereoZed::isCalibrated() const
{
#ifdef RTABMAP_ZED
	return stereoModel_.isValidForProjection();
#else
	return false;
#endif
}

std::string CameraStereoZed::getSerial() const
{
#ifdef RTABMAP_ZED
	if(zed_)
	{
		return uFormat("%x", zed_->getCameraInformation ().serial_number);
	}
#endif
	return "";
}

bool CameraStereoZed::odomProvided() const
{
#ifdef RTABMAP_ZED
	return computeOdometry_;
#else
	return false;
#endif
}
#ifdef RTABMAP_ZED
static cv::Mat slMat2cvMat(sl::Mat& input) {
	//convert MAT_TYPE to CV_TYPE
	int cv_type = -1;
	switch (input.getDataType()) {
	case sl::MAT_TYPE_32F_C1: cv_type = CV_32FC1; break;
	case sl::MAT_TYPE_32F_C2: cv_type = CV_32FC2; break;
	case sl::MAT_TYPE_32F_C3: cv_type = CV_32FC3; break;
	case sl::MAT_TYPE_32F_C4: cv_type = CV_32FC4; break;
	case sl::MAT_TYPE_8U_C1: cv_type = CV_8UC1; break;
	case sl::MAT_TYPE_8U_C2: cv_type = CV_8UC2; break;
	case sl::MAT_TYPE_8U_C3: cv_type = CV_8UC3; break;
	case sl::MAT_TYPE_8U_C4: cv_type = CV_8UC4; break;
	default: break;
	}	
	// cv::Mat data requires a uchar* pointer. Therefore, we get the uchar1 pointer from sl::Mat (getPtr<T>())
	//cv::Mat and sl::Mat will share the same memory pointer
	return cv::Mat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(sl::MEM_CPU));
}

Transform zedPoseToTransform(const sl::Pose & pose)
{
	return Transform(
			pose.pose_data.m[0], pose.pose_data.m[1], pose.pose_data.m[2], pose.pose_data.m[3],
			pose.pose_data.m[4], pose.pose_data.m[5], pose.pose_data.m[6], pose.pose_data.m[7],
			pose.pose_data.m[8], pose.pose_data.m[9], pose.pose_data.m[10], pose.pose_data.m[11]);
}
#endif

SensorData CameraStereoZed::captureImage(CameraInfo * info)
{
	SensorData data;
#ifdef RTABMAP_ZED
	sl::RuntimeParameters rparam((sl::SENSING_MODE)sensingMode_, quality_ > 0, quality_ > 0, sl::REFERENCE_FRAME_CAMERA);
	if(zed_)
	{
		UTimer timer;
		bool res = zed_->grab(rparam);
		while (src_ == CameraVideo::kUsbDevice && res!=sl::SUCCESS && timer.elapsed() < 2.0)
		{
			// maybe there is a latency with the USB, try again in 10 ms (for the next 2 seconds)
			uSleep(10);
			res = zed_->grab(rparam);
		}
		if(res==sl::SUCCESS)
		{
			// get left image
			sl::Mat tmp;
			zed_->retrieveImage(tmp,sl::VIEW_LEFT);
			cv::Mat rgbaLeft = slMat2cvMat(tmp);

			cv::Mat left;
			cv::cvtColor(rgbaLeft, left, cv::COLOR_BGRA2BGR);

			if(quality_ > 0)
			{
				// get depth image
				cv::Mat depth;
				sl::Mat tmp;
				zed_->retrieveMeasure(tmp,sl::MEASURE_DEPTH);
				slMat2cvMat(tmp).copyTo(depth);

				data = SensorData(left, depth, stereoModel_.left(), this->getNextSeqID(), UTimer::now());
			}
			else
			{
				// get right image
				sl::Mat tmp;zed_->retrieveImage(tmp,sl::VIEW_RIGHT );
				cv::Mat rgbaRight = slMat2cvMat(tmp);
				cv::Mat right;
				cv::cvtColor(rgbaRight, right, cv::COLOR_BGRA2GRAY);
			
				data = SensorData(left, right, stereoModel_, this->getNextSeqID(), UTimer::now());
			}

			if (computeOdometry_ && info)
			{
				sl::Pose pose;
				sl::TRACKING_STATE tracking_state = zed_->getPosition(pose);
				if (tracking_state == sl::TRACKING_STATE_OK)
				{
					int trackingConfidence = pose.pose_confidence;
					// FIXME What does pose_confidence == -1 mean?
					if (trackingConfidence>0)
					{
						info->odomPose = zedPoseToTransform(pose);
						if (!info->odomPose.isNull())
						{
							//transform x->forward, y->left, z->up
							Transform opticalTransform(0, 0, 1, 0, -1, 0, 0, 0, 0, -1, 0, 0);
							info->odomPose = opticalTransform * info->odomPose * opticalTransform.inverse();
							if(force3DoF_)
							{
								info->odomPose = info->odomPose.to3DoF();
							}
							if (lost_)
							{
								info->odomCovariance = cv::Mat::eye(6, 6, CV_64FC1) * 9999.0f; // don't know transform with previous pose
								lost_ = false;
								UDEBUG("Init %s (var=%f)", info->odomPose.prettyPrint().c_str(), 9999.0f);
							}
							else
							{
								info->odomCovariance = cv::Mat::eye(6, 6, CV_64FC1) * 1.0f / float(trackingConfidence);
								UDEBUG("Run %s (var=%f)", info->odomPose.prettyPrint().c_str(), 1.0f / float(trackingConfidence));
							}
						}
						else
						{
							info->odomCovariance = cv::Mat::eye(6, 6, CV_64FC1) * 9999.0f; // lost
							lost_ = true;
							UWARN("ZED lost! (trackingConfidence=%d)", trackingConfidence);
						}
					}
					else
					{
						info->odomCovariance = cv::Mat::eye(6, 6, CV_64FC1) * 9999.0f; // lost
						lost_ = true;
						UWARN("ZED lost! (trackingConfidence=%d)", trackingConfidence);
					}
				}
				else
				{
					UWARN("Tracking not ok: state=\"%s\"", toString(tracking_state).c_str());
				}
			}
		}
		else if(src_ == CameraVideo::kUsbDevice)
		{
			UERROR("CameraStereoZed: Failed to grab images after 2 seconds!");
		}
		else
		{
			UWARN("CameraStereoZed: end of stream is reached!");
		}
	}
#else
	UERROR("CameraStereoZED: RTAB-Map is not built with ZED sdk support!");
#endif
	return data;
}

} // namespace rtabmap
