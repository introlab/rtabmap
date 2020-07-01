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

#include "rtabmap/utilite/UThread.h"
#include "rtabmap/utilite/UTimer.h"
#include "rtabmap/utilite/UEventsManager.h"
#include <rtabmap/core/camera/CameraMyntEye.h>
#ifdef RTABMAP_MYNTEYE
#include <mynteye/api.h>
#include <mynteye/device.h>
#include <mynteye/context.h>

#ifndef M_PI
#define M_PI       3.14159265358979323846
#endif
#endif

namespace rtabmap
{

CameraMyntEye::CameraMyntEye(const std::string & device, bool apiRectification, bool apiDepth, float imageRate, const Transform & localTransform) :
			Camera(imageRate, localTransform)
#ifdef RTABMAP_MYNTEYE
			,
			deviceName_(device),
			apiRectification_(apiRectification),
			apiDepth_(apiDepth),
			autoExposure_(true),
			gain_(24),
			brightness_(120),
			contrast_(116),
			dataReady_(0),
			lastFramesStamp_(0.0),
			stamp_(0),
			publishInterIMU_(false),
			softTimeBegin_(0.0),
			hardTimeBegin_(0),
			unitHardTime_(std::numeric_limits<std::uint32_t>::max()*10)
#endif
{
#ifdef RTABMAP_MYNTEYE
	lastHardTimes_ = std::vector<std::uint64_t>((size_t)mynteye::Stream::LAST+1, 0);
	acc_ = std::vector<std::uint64_t>((size_t)mynteye::Stream::LAST+1, 0);
#endif
}
CameraMyntEye::~CameraMyntEye()
{
#ifdef RTABMAP_MYNTEYE
	if (api_) {
		api_->Stop(mynteye::Source::ALL);
	}
	dataReady_.release();
#endif
}

#ifdef RTABMAP_MYNTEYE
std::shared_ptr<mynteye::IntrinsicsBase> getDefaultIntrinsics() {
	auto res = std::make_shared<mynteye::IntrinsicsPinhole>();
	res->width = 640;
	res->height = 400;
	res->model = 0;
	res->fx = 3.6220059643202876e+02;
	res->fy = 3.6350065250745848e+02;
	res->cx = 4.0658699068023441e+02;
	res->cy = 2.3435161110061483e+02;
	double codffs[5] = {
			-2.5034765682756088e-01,
			5.0579399202897619e-02,
			-7.0536676161976066e-04,
			-8.5255451307033846e-03,
			0.
	};
	for (unsigned int i = 0; i < 5; i++) {
		res->coeffs[i] = codffs[i];
	}
	return res;
}

std::shared_ptr<mynteye::Extrinsics> getDefaultExtrinsics() {
	auto res = std::make_shared<mynteye::Extrinsics>();
	double rotation[9] = {
			9.9867908939669447e-01,  -6.3445566137485428e-03, 5.0988459509619687e-02,
			5.9890316389333252e-03,  9.9995670037792639e-01,  7.1224201868366971e-03,
			-5.1031440326695092e-02, -6.8076406092671274e-03, 9.9867384471984544e-01
	};
	double translation[3] = {-1.2002489764113250e+02, -1.1782637409050747e+00,
			-5.2058205159996538e+00};
	for (unsigned int i = 0; i < 3; i++) {
		for (unsigned int j = 0; j < 3; j++) {
			res->rotation[i][j] = rotation[i*3 + j];
		}
	}
	for (unsigned int i = 0; i < 3; i++) {
		res->translation[i] = translation[i];
	}
	return res;
}

double CameraMyntEye::hardTimeToSoftTime(std::uint64_t hardTime) {
	if (hardTimeBegin_==0) {
		softTimeBegin_ = UTimer::now();
		hardTimeBegin_ = hardTime;
	}

	std::uint64_t time_ns_detal = (hardTime - hardTimeBegin_);
	std::uint64_t time_ns_detal_s = time_ns_detal / 1000000;
	std::uint64_t time_ns_detal_ns = time_ns_detal % 1000000;
	double time_sec_double = static_cast<double>(time_ns_detal_s) + 1e-9*static_cast<double>(time_ns_detal_ns * 1000);

	return softTimeBegin_ + time_sec_double;
}

inline bool is_overflow(std::uint64_t now, std::uint64_t pre, std::uint64_t unit_hard_time) {

	return (now < pre) && ((pre - now) > (unit_hard_time / 2));
}

double CameraMyntEye::checkUpTimeStamp(std::uint64_t _hard_time, std::uint8_t stream) {
	UASSERT(stream < (std::uint8_t)mynteye::Stream::LAST+1);
	if (is_overflow(_hard_time, lastHardTimes_[stream], unitHardTime_)) {
		acc_[stream]++;
	}

	lastHardTimes_[stream] = _hard_time;

	return hardTimeToSoftTime(acc_[stream] * unitHardTime_ + _hard_time);
}
#endif

void CameraMyntEye::publishInterIMU(bool enabled)
{
#ifdef RTABMAP_MYNTEYE
	publishInterIMU_ = enabled;
#endif
}

void CameraMyntEye::setAutoExposure()
{
#ifdef RTABMAP_MYNTEYE
	autoExposure_ = true;
#endif
}

void CameraMyntEye::setManualExposure(int gain, int brightness, int constrast)
{
#ifdef RTABMAP_MYNTEYE
	UASSERT(gain>=0 && gain<=48);
	UASSERT(brightness>=0 && brightness<=240);
	UASSERT(constrast>=0 && constrast<=254);
	autoExposure_ = false;
	gain_ = gain;
	brightness_ = brightness;
	contrast_ = constrast;
#endif
}

void CameraMyntEye::setIrControl(int value)
{
#ifdef RTABMAP_MYNTEYE
	UASSERT(value>=0 && value<=160);
	irControl_ = value;
#endif
}

bool CameraMyntEye::init(const std::string & calibrationFolder, const std::string & cameraName)
{
#ifdef RTABMAP_MYNTEYE
	mynteye::Context context;
	auto &&devices = context.devices();

	softTimeBegin_ = 0.0;
	hardTimeBegin_ = 0;
	imuBuffer_.clear();
	lastHardTimes_ = std::vector<std::uint64_t>((size_t)mynteye::Stream::LAST+1, 0);
	acc_ = std::vector<std::uint64_t>((size_t)mynteye::Stream::LAST+1, 0);

	size_t n = devices.size();
	if(n==0)
	{
		UERROR("No Mynt Eye devices detected!");
		return false;
	}
	UINFO("MYNT EYE devices:");

	for (size_t i = 0; i < n; i++) {
		auto &&device = devices[i];
		auto &&name = device->GetInfo(mynteye::Info::DEVICE_NAME);
		auto &&serial_number = device->GetInfo(mynteye::Info::SERIAL_NUMBER);
		if(!deviceName_.empty() && serial_number.compare(deviceName_) == 0)
		{
			device_ = devices[i];
		}
		UINFO("  index: %d, name: %s, serial number: %s", (int)i, name.c_str(), serial_number.c_str());
	}

	if(device_.get() == 0)
	{
		//take first one by default
		device_ = devices[0];
	}

	UINFO("");
	api_ = mynteye::API::Create(device_);

	UINFO("");
	auto in_left_base = api_->GetIntrinsicsBase(mynteye::Stream::LEFT);
	auto in_right_base = api_->GetIntrinsicsBase(mynteye::Stream::RIGHT);
	UINFO("");
	auto ex = api_->GetExtrinsics(mynteye::Stream::RIGHT, mynteye::Stream::LEFT);

	UINFO("");
	if(!in_left_base || !in_right_base)
	{
		UERROR("Unknown calibration model! Using default ones");
		in_left_base = getDefaultIntrinsics();
		in_right_base = getDefaultIntrinsics();
		ex = *(getDefaultExtrinsics());
	}
	cv::Size size{in_left_base->width, in_left_base->height};

	cv::Mat K1, K2, D1, D2;

	if(in_left_base->calib_model() == mynteye::CalibrationModel::PINHOLE &&
			in_right_base->calib_model() == mynteye::CalibrationModel::PINHOLE)
	{
		auto in_left = *std::dynamic_pointer_cast<mynteye::IntrinsicsPinhole>(in_left_base);
		auto in_right = *std::dynamic_pointer_cast<mynteye::IntrinsicsPinhole>(in_right_base);

		K1 = (cv::Mat_<double>(3, 3) << in_left.fx, 0, in_left.cx, 0, in_left.fy, in_left.cy, 0, 0, 1);
		K2 = (cv::Mat_<double>(3, 3) << in_right.fx, 0, in_right.cx, 0, in_right.fy, in_right.cy, 0, 0, 1);
		D1 = cv::Mat(1, 5, CV_64F, in_left.coeffs).clone();
		D2 = cv::Mat(1, 5, CV_64F, in_right.coeffs).clone();
	}
	else if(in_left_base->calib_model() == mynteye::CalibrationModel::KANNALA_BRANDT &&
			in_right_base->calib_model() == mynteye::CalibrationModel::KANNALA_BRANDT)
	{
		//equidistant
		auto in_left = *std::dynamic_pointer_cast<mynteye::IntrinsicsEquidistant>(in_left_base);
		auto in_right = *std::dynamic_pointer_cast<mynteye::IntrinsicsEquidistant>(in_right_base);

		/** The distortion coefficients: k2,k3,k4,k5,mu,mv,u0,v0 */
		UINFO("left coeff = %f %f %f %f %f %f %f %f",
				in_left.coeffs[0], in_left.coeffs[1], in_left.coeffs[2], in_left.coeffs[3],
				in_left.coeffs[4], in_left.coeffs[5], in_left.coeffs[6], in_left.coeffs[7]);
		UINFO("right coeff = %f %f %f %f %f %f %f %f",
				in_left.coeffs[0], in_left.coeffs[1], in_left.coeffs[2], in_left.coeffs[3],
				in_left.coeffs[4], in_left.coeffs[5], in_left.coeffs[6], in_left.coeffs[7]);

		K1 = (cv::Mat_<double>(3, 3) << in_left.coeffs[4], 0, in_left.coeffs[6], 0, in_left.coeffs[5], in_left.coeffs[7], 0, 0, 1);
		K2 = (cv::Mat_<double>(3, 3) << in_right.coeffs[4], 0, in_right.coeffs[6], 0, in_right.coeffs[5], in_right.coeffs[7], 0, 0, 1);
		// convert to (k1,k2,p1,p2,k3,k4)
		D1 = (cv::Mat_<double>(1, 6) << in_left.coeffs[0], in_left.coeffs[1], 0, 0, in_left.coeffs[2], in_left.coeffs[3]);
		D2 = (cv::Mat_<double>(1, 6) << in_right.coeffs[0], in_right.coeffs[1], 0, 0, in_right.coeffs[2], in_right.coeffs[3]);
	}

	bool is_data_use_mm_instead_of_m =
			abs(ex.translation[0]) > 1.0 ||
			abs(ex.translation[1]) > 1.0 ||
			abs(ex.translation[2]) > 1.0;
	if(is_data_use_mm_instead_of_m)
	{
		ex.translation[0] *= 0.001;
		ex.translation[1] *= 0.001;
		ex.translation[2] *= 0.001;
	}

	Transform extrinsics(
			ex.rotation[0][0], ex.rotation[0][1], ex.rotation[0][2], ex.translation[0],
			ex.rotation[1][0], ex.rotation[1][1], ex.rotation[1][2], ex.translation[1],
			ex.rotation[2][0], ex.rotation[2][1], ex.rotation[2][2], ex.translation[2]);

	cv::Mat P1 = cv::Mat::eye(3, 4, CV_64FC1);
	P1.at<double>(0,0) = K1.at<double>(0,0);
	P1.at<double>(1,1) = K1.at<double>(1,1);
	P1.at<double>(0,2) = K1.at<double>(0,2);
	P1.at<double>(1,2) = K1.at<double>(1,2);

	cv::Mat P2 = cv::Mat::eye(3, 4, CV_64FC1);
	P2.at<double>(0,0) = K2.at<double>(0,0);
	P2.at<double>(1,1) = K2.at<double>(1,1);
	P2.at<double>(0,2) = K2.at<double>(0,2);
	P2.at<double>(1,2) = K2.at<double>(1,2);

	CameraModel leftModel(this->getSerial(), size, K1, D1, cv::Mat::eye(3, 3, CV_64F), P1, this->getLocalTransform());
	CameraModel rightModel(this->getSerial(), size, K2, D2, cv::Mat::eye(3, 3, CV_64F), P2, this->getLocalTransform());
	UINFO("raw: fx=%f fy=%f cx=%f cy=%f",
				leftModel.fx(),
				leftModel.fy(),
				leftModel.cx(),
				leftModel.cy());

	UINFO("stereo extrinsics = %s", extrinsics.prettyPrint().c_str());
	stereoModel_ = StereoCameraModel(this->getSerial(), leftModel, rightModel, extrinsics);

	if(!stereoModel_.isValidForRectification())
	{
		UERROR("Could not initialize stereo rectification.");
		return false;
	}

	stereoModel_.initRectificationMap();
	UINFO("baseline = %f rectified: fx=%f fy=%f cx=%f cy=%f",
			stereoModel_.baseline(),
			stereoModel_.left().fx(),
			stereoModel_.left().fy(),
			stereoModel_.left().cx(),
			stereoModel_.left().cy());

	// get left to imu camera transform
	auto &&exImu = api_->GetMotionExtrinsics(mynteye::Stream::LEFT);
	is_data_use_mm_instead_of_m =
			abs(exImu.translation[0]) > 1.0 ||
			abs(exImu.translation[1]) > 1.0 ||
			abs(exImu.translation[2]) > 1.0;

	if (is_data_use_mm_instead_of_m) {
		exImu.translation[0] *= 0.001;
		exImu.translation[1] *= 0.001;
		exImu.translation[2] *= 0.001;
	}

	if (exImu.rotation[0][0] == 0 && exImu.rotation[2][2] == 0)
	{
		imuLocalTransform_ = Transform(
				0, 0, 1, exImu.translation[0],
				0,-1, 0, exImu.translation[1],
				1, 0, 0, exImu.translation[2]);
	}
	else
	{
		imuLocalTransform_ = Transform(
				exImu.rotation[0][0], exImu.rotation[0][1], exImu.rotation[0][2], exImu.translation[0],
				exImu.rotation[1][0], exImu.rotation[1][1], exImu.rotation[1][2], exImu.translation[1],
				exImu.rotation[2][0], exImu.rotation[2][1], exImu.rotation[2][2], exImu.translation[2]);
	}
	UINFO("imu extrinsics = %s", imuLocalTransform_.prettyPrint().c_str());

	for(int i=0;i<(int)mynteye::Stream::LAST; ++i)
	{
		UINFO("Support stream %d = %s", i, api_->Supports((mynteye::Stream)i)?"true":"false");
	}

	if (api_->Supports(apiRectification_?mynteye::Stream::LEFT_RECTIFIED:mynteye::Stream::LEFT) &&
		api_->Supports(apiRectification_?mynteye::Stream::RIGHT_RECTIFIED:mynteye::Stream::RIGHT))
	{
		api_->EnableStreamData(apiRectification_?mynteye::Stream::LEFT_RECTIFIED:mynteye::Stream::LEFT);
		api_->SetStreamCallback(apiRectification_?mynteye::Stream::LEFT_RECTIFIED:mynteye::Stream::LEFT,
				[&](const mynteye::api::StreamData &data)
		{
			double stamp = checkUpTimeStamp(data.img->timestamp, (std::uint8_t)(apiRectification_?mynteye::Stream::LEFT_RECTIFIED:mynteye::Stream::LEFT));

			UScopeMutex s(dataMutex_);
			bool notify = false;
			leftFrameBuffer_ = data.frame;
			if(stamp_>0 && stamp_ == data.img->timestamp && !rightFrameBuffer_.empty())
			{
				notify = lastFrames_.first.empty();
				lastFrames_.first = leftFrameBuffer_;
				lastFrames_.second = rightFrameBuffer_;
				lastFramesStamp_ = stamp;
				leftFrameBuffer_ = cv::Mat();
				rightFrameBuffer_ = cv::Mat();
				stamp_ = 0;
			}
			else
			{
				stamp_ = data.img->timestamp;
			}
			if(notify)
			{
				dataReady_.release();
			}
		});

		if(apiRectification_ && apiDepth_ && api_->Supports(mynteye::Stream::DEPTH))
		{
			api_->EnableStreamData(mynteye::Stream::DEPTH);
		}
		else
		{
			api_->EnableStreamData(apiRectification_?mynteye::Stream::RIGHT_RECTIFIED:mynteye::Stream::RIGHT);
			apiDepth_ = false;
		}
		api_->SetStreamCallback(
				apiDepth_?mynteye::Stream::DEPTH:apiRectification_?mynteye::Stream::RIGHT_RECTIFIED:mynteye::Stream::RIGHT,
				[&](const mynteye::api::StreamData &data)
		{
			double stamp = checkUpTimeStamp(data.img->timestamp, (std::uint8_t)(apiDepth_?mynteye::Stream::DEPTH:apiRectification_?mynteye::Stream::RIGHT_RECTIFIED:mynteye::Stream::RIGHT));

			UScopeMutex s(dataMutex_);
			bool notify = false;
			rightFrameBuffer_ = data.frame;
			if(stamp_>0 && stamp_ == data.img->timestamp && !leftFrameBuffer_.empty())
			{
				notify = lastFrames_.first.empty();
				lastFrames_.first = leftFrameBuffer_;
				lastFrames_.second = rightFrameBuffer_;
				lastFramesStamp_ = stamp;
				leftFrameBuffer_ = cv::Mat();
				rightFrameBuffer_ = cv::Mat();
				stamp_ = 0;
			}
			else
			{
				stamp_ = data.img->timestamp;
			}
			if(notify)
			{
				dataReady_.release();
			}
		});

		api_->SetMotionCallback([this](const mynteye::api::MotionData &data) {

			double stamp = checkUpTimeStamp(data.imu->timestamp, (std::uint8_t)mynteye::Stream::LAST);
			if(data.imu->flag == 0)
			{
				//UWARN("%f %f %f %f %f %f",
				//		data.imu->gyro[0] * M_PI / 180, data.imu->gyro[1] * M_PI / 180, data.imu->gyro[2] * M_PI / 180,
				//		data.imu->accel[0] * 9.8, data.imu->accel[1] * 9.8, data.imu->accel[2] * 9.8);
				cv::Vec3d gyro(data.imu->gyro[0] * M_PI / 180, data.imu->gyro[1] * M_PI / 180, data.imu->gyro[2] * M_PI / 180);
				cv::Vec3d acc(data.imu->accel[0] * 9.8, data.imu->accel[1] * 9.8, data.imu->accel[2] * 9.8);
				if(publishInterIMU_)
				{
					IMU imu(gyro, cv::Mat::eye(3,3,CV_64FC1),
							acc, cv::Mat::eye(3,3,CV_64FC1),
							imuLocalTransform_);
					UEventsManager::post(new IMUEvent(imu, stamp));
				}
				else
				{
					UScopeMutex lock(imuMutex_);
					imuBuffer_.insert(imuBuffer_.end(), std::make_pair(stamp, std::make_pair(gyro, acc)));
					if(imuBuffer_.size() > 1000)
					{
						imuBuffer_.erase(imuBuffer_.begin());
					}
				}
			}

			uSleep(1);

		});

		api_->SetOptionValue(mynteye::Option::EXPOSURE_MODE, autoExposure_?0:1);
		if(!autoExposure_)
		{
			api_->SetOptionValue(mynteye::Option::GAIN, gain_);
			api_->SetOptionValue(mynteye::Option::BRIGHTNESS, brightness_);
			api_->SetOptionValue(mynteye::Option::CONTRAST, contrast_);
		}
		api_->SetOptionValue(mynteye::Option::IR_CONTROL, irControl_);

		api_->Start(mynteye::Source::ALL);
		uSleep(500); // To buffer some imus before sending images
		return true;
	}
	UERROR("Streams missing.");
	return false;
#else
	UERROR("Not built with Mynt Eye support!");
	return false;
#endif
}

bool CameraMyntEye::isCalibrated() const
{
	return true;
}

std::string CameraMyntEye::getSerial() const
{
#ifdef RTABMAP_MYNTEYE
	if(device_.get())
	{
		return device_->GetInfo(mynteye::Info::SERIAL_NUMBER);
	}
#endif
	return "";
}

bool CameraMyntEye::available()
{
#ifdef RTABMAP_MYNTEYE
	return true;
#else
	return false;
#endif
}

#ifdef RTABMAP_MYNTEYE
void CameraMyntEye::getPoseAndIMU(
		const double & stamp,
		IMU & imu,
		int maxWaitTimeMs) const
{
	imu = IMU();
	if(imuBuffer_.empty())
	{
		return;
	}

	// Interpolate gyro,acc
	cv::Vec3d gyro;
	cv::Vec3d acc;
	{
		imuMutex_.lock();
		int waitTry = 0;
		while(maxWaitTimeMs > 0 && imuBuffer_.rbegin()->first < stamp && waitTry < maxWaitTimeMs)
		{
			imuMutex_.unlock();
			++waitTry;
			uSleep(1);
			imuMutex_.lock();
		}
		if(imuBuffer_.rbegin()->first < stamp)
		{
			if(maxWaitTimeMs>0)
			{
				UWARN("Could not find gyro/acc data to interpolate at time %f after waiting %d ms (last is %f)...", stamp, maxWaitTimeMs, imuBuffer_.rbegin()->first);
			}
			imuMutex_.unlock();
			return;
		}
		else
		{
			std::map<double, std::pair<cv::Vec3f, cv::Vec3f> >::const_iterator iterB = imuBuffer_.lower_bound(stamp);
			std::map<double, std::pair<cv::Vec3f, cv::Vec3f> >::const_iterator iterA = iterB;
			if(iterA != imuBuffer_.begin())
			{
				iterA = --iterA;
			}
			if(iterB == imuBuffer_.end())
			{
				iterB = --iterB;
			}
			if(iterA == iterB && stamp == iterA->first)
			{
				gyro[0] = iterA->second.first[0];
				gyro[1] = iterA->second.first[1];
				gyro[2] = iterA->second.first[2];
				acc[0] = iterA->second.second[0];
				acc[1] = iterA->second.second[1];
				acc[2] = iterA->second.second[2];
			}
			else if(stamp >= iterA->first && stamp <= iterB->first)
			{
				float t = (stamp-iterA->first) / (iterB->first-iterA->first);
				gyro[0] = iterA->second.first[0] + t*(iterB->second.first[0] - iterA->second.first[0]);
				gyro[1] = iterA->second.first[1] + t*(iterB->second.first[1] - iterA->second.first[1]);
				gyro[2] = iterA->second.first[2] + t*(iterB->second.first[2] - iterA->second.first[2]);
				acc[0] = iterA->second.second[0] + t*(iterB->second.second[0] - iterA->second.second[0]);
				acc[1] = iterA->second.second[1] + t*(iterB->second.second[1] - iterA->second.second[1]);
				acc[2] = iterA->second.second[2] + t*(iterB->second.second[2] - iterA->second.second[2]);
			}
			else
			{
				if(stamp < iterA->first)
				{
					UWARN("Could not find acc data to interpolate at image time %f (earliest is %f). Are sensors synchronized?", stamp, iterA->first);
				}
				else
				{
					UWARN("Could not find acc data to interpolate at image time %f (between %f and %f). Are sensors synchronized?", stamp, iterA->first, iterB->first);
				}
				imuMutex_.unlock();
				return;
			}
		}
		imuMutex_.unlock();
	}

	imu = IMU(gyro, cv::Mat::eye(3, 3, CV_64FC1), acc, cv::Mat::eye(3, 3, CV_64FC1), imuLocalTransform_);
}
#endif

SensorData CameraMyntEye::captureImage(CameraInfo * info)
{
	SensorData data;
#ifdef RTABMAP_MYNTEYE
	if(!dataReady_.acquire(1, 3000))
	{
		UERROR("Did not receive frame since 3 seconds...");
		return data;
	}

	cv::Mat left;
	cv::Mat right;
	double stamp = 0.0;

	dataMutex_.lock();
	if(!lastFrames_.first.empty())
	{
		left = lastFrames_.first;
		right = lastFrames_.second;
		stamp = lastFramesStamp_;
		lastFrames_ = std::pair<cv::Mat, cv::Mat>();
	}
	dataMutex_.unlock();

	if(!left.empty() && !right.empty())
	{
		if(right.type() == CV_16UC1)
		{
			data = SensorData(left, right, stereoModel_.left(), 0, stamp);
		}
		else
		{
			if(!apiRectification_)
			{
				left = stereoModel_.left().rectifyImage(left);
				right = stereoModel_.right().rectifyImage(right);
			}
			data = SensorData(left, right, stereoModel_, 0, stamp);
		}
		if(!publishInterIMU_ && imuBuffer_.size())
		{
			IMU imu;
			getPoseAndIMU(stamp, imu, 60);
			if(!imu.empty())
			{
				data.setIMU(imu);
			}
		}
	}
#else
	UERROR("Not built with Mynt Eye support!");
#endif
	return data;
}

} // namespace rtabmap
