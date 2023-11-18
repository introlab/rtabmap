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

#include "rtabmap/core/OdometryInfo.h"
#include "rtabmap/core/util2d.h"
#include "rtabmap/core/util3d_transforms.h"
#include "rtabmap/utilite/ULogger.h"
#include "rtabmap/utilite/UTimer.h"
#include "rtabmap/utilite/UStl.h"
#include "rtabmap/utilite/UDirectory.h"
#include <pcl/common/transforms.h>
#include <opencv2/imgproc/types_c.h>
#include <rtabmap/core/odometry/OdometryORBSLAM3.h>

#if defined(RTABMAP_ORB_SLAM) and RTABMAP_ORB_SLAM == 3
#include <thread>
#include <Converter.h>

using namespace std;

#endif

namespace rtabmap {

OdometryORBSLAM3::OdometryORBSLAM3(const ParametersMap & parameters) :
	Odometry(parameters)
#if defined(RTABMAP_ORB_SLAM) and RTABMAP_ORB_SLAM == 3
    ,
	orbslam_(0),
	firstFrame_(true),
	previousPose_(Transform::getIdentity()),
	useIMU_(Parameters::defaultOdomORBSLAMInertial()),
	parameters_(parameters),
	lastImuStamp_(0.0),
	lastImageStamp_(0.0)
#endif
{
#if defined(RTABMAP_ORB_SLAM) and RTABMAP_ORB_SLAM == 3
	Parameters::parse(parameters, Parameters::kOdomORBSLAMInertial(), useIMU_);
#endif
}

OdometryORBSLAM3::~OdometryORBSLAM3()
{
#if defined(RTABMAP_ORB_SLAM) and RTABMAP_ORB_SLAM == 3
	if(orbslam_)
	{
		orbslam_->Shutdown();
		delete orbslam_;
	}
#endif
}

void OdometryORBSLAM3::reset(const Transform & initialPose)
{
	Odometry::reset(initialPose);
#if defined(RTABMAP_ORB_SLAM) and RTABMAP_ORB_SLAM == 3
	if(orbslam_)
	{
		orbslam_->Shutdown();
		delete orbslam_;
		orbslam_=0;
	}
	firstFrame_ = true;
	originLocalTransform_.setNull();
	previousPose_.setIdentity();
	imuLocalTransform_.setNull();
	lastImuStamp_ = 0.0;
	lastImageStamp_ = 0.0;
#endif
}

bool OdometryORBSLAM3::canProcessAsyncIMU() const
{
#if defined(RTABMAP_ORB_SLAM) and RTABMAP_ORB_SLAM == 3
	return useIMU_;
#else
	return false;
#endif
}

bool OdometryORBSLAM3::init(const rtabmap::CameraModel & model, double stamp,  bool stereo, double baseline)
{
#if defined(RTABMAP_ORB_SLAM) and RTABMAP_ORB_SLAM == 3
	std::string vocabularyPath;
	rtabmap::Parameters::parse(parameters_, rtabmap::Parameters::kOdomORBSLAMVocPath(), vocabularyPath);

	if(vocabularyPath.empty())
	{
		UERROR("ORB_SLAM vocabulary path should be set! (Parameter name=\"%s\")", rtabmap::Parameters::kOdomORBSLAMVocPath().c_str());
		return false;
	}
	//Load ORB Vocabulary
	vocabularyPath = uReplaceChar(vocabularyPath, '~', UDirectory::homeDir());
	UWARN("Loading ORB Vocabulary: \"%s\". This could take a while...", vocabularyPath.c_str());

	// Create configuration file
	std::string workingDir;
	rtabmap::Parameters::parse(parameters_, rtabmap::Parameters::kRtabmapWorkingDirectory(), workingDir);
	if(workingDir.empty())
	{
		workingDir = ".";
	}
	std::string configPath = workingDir+"/rtabmap_orbslam.yaml";
	std::ofstream ofs (configPath, std::ofstream::out);
	ofs << "%YAML:1.0" << std::endl;
	ofs << std::endl;

	ofs << "File.version: \"1.0\"" << std::endl;
	ofs << std::endl;

	ofs << "Camera.type: \"PinHole\"" << std::endl;
	ofs << std::endl;

	ofs << fixed << setprecision(13);

	//# Camera calibration and distortion parameters (OpenCV)
	ofs << "Camera1.fx: " << model.fx() << std::endl;
	ofs << "Camera1.fy: " << model.fy() << std::endl;
	ofs << "Camera1.cx: " << model.cx() << std::endl;
	ofs << "Camera1.cy: " << model.cy() << std::endl;
	ofs << std::endl;

	if(model.D().cols < 4)
	{
		ofs << "Camera1.k1: " << 0.0 << std::endl;
		ofs << "Camera1.k2: " << 0.0 << std::endl;
		ofs << "Camera1.p1: " << 0.0 << std::endl;
		ofs << "Camera1.p2: " << 0.0 << std::endl;
		if(!stereo)
		{
			ofs << "Camera1.k3: " << 0.0 << std::endl;
		}
	}
	if(model.D().cols >= 4)
	{
		ofs << "Camera1.k1: " << model.D().at<double>(0,0) << std::endl;
		ofs << "Camera1.k2: " << model.D().at<double>(0,1) << std::endl;
		ofs << "Camera1.p1: " << model.D().at<double>(0,2) << std::endl;
		ofs << "Camera1.p2: " << model.D().at<double>(0,3) << std::endl;
	}
	if(model.D().cols >= 5)
	{
		ofs << "Camera1.k3: " << model.D().at<double>(0,4) << std::endl;
	}
	if(model.D().cols > 5)
	{
		UWARN("Unhandled camera distortion size %d, only 5 first coefficients used", model.D().cols);
	}
	ofs << std::endl;

	//# IR projector baseline times fx (aprox.)
	if(baseline <= 0.0)
	{
		baseline = rtabmap::Parameters::defaultOdomORBSLAMBf();
		rtabmap::Parameters::parse(parameters_, rtabmap::Parameters::kOdomORBSLAMBf(), baseline);
	}
	ofs << "Camera.bf: " << model.fx()*baseline << std::endl;

	ofs << "Camera.width: " << model.imageWidth() << std::endl;
	ofs << "Camera.height: " << model.imageHeight() << std::endl;
	ofs << std::endl;

	//# Color order of the images (0: BGR, 1: RGB. It is ignored if images are grayscale)
	//Camera.RGB: 1
	ofs << "Camera.RGB: 1" << std::endl;
	ofs << std::endl;

	float fps = rtabmap::Parameters::defaultOdomORBSLAMFps();
	rtabmap::Parameters::parse(parameters_, rtabmap::Parameters::kOdomORBSLAMFps(), fps);
	if(fps == 0)
	{
		UASSERT(stamp > lastImageStamp_);
		fps = std::round(1./(stamp - lastImageStamp_));
		UWARN("Camera FPS estimated at %d Hz. If this doesn't look good, "
			  "set explicitly parameter %s to expected frequency.",
			  int(fps), Parameters::kOdomORBSLAMFps().c_str());
	}
	ofs << "Camera.fps: " << (int)fps << std::endl;
	ofs << std::endl;

	//# Close/Far threshold. Baseline times.
	double thDepth = rtabmap::Parameters::defaultOdomORBSLAMThDepth();
	rtabmap::Parameters::parse(parameters_, rtabmap::Parameters::kOdomORBSLAMThDepth(), thDepth);
	ofs << "Stereo.ThDepth: " << thDepth << std::endl;
	ofs << "Stereo.b: " << baseline << std::endl;
	ofs << std::endl;

	//# Deptmap values factor
	ofs << "RGBD.DepthMapFactor: " << 1.0 << std::endl;
	ofs << std::endl;

	bool withIMU = false;
	if(!imuLocalTransform_.isNull())
	{
		withIMU = true;
		//#--------------------------------------------------------------------------------------------
		//# IMU Parameters TODO: hard-coded, not used
		//#--------------------------------------------------------------------------------------------
		//  Transformation from camera 0 to body-frame (imu)
		rtabmap::Transform camImuT = model.localTransform()*imuLocalTransform_;
		ofs << "IMU.T_b_c1: !!opencv-matrix" << std::endl;
		ofs << "   rows: 4" << std::endl;
		ofs << "   cols: 4" << std::endl;
		ofs << "   dt: f" << std::endl;
		ofs << "   data: [" << camImuT.data()[0] << ", " << camImuT.data()[1] << ", " << camImuT.data()[2]  << ", " << camImuT.data()[3]  << ", " << std::endl;
		ofs << "         "  << camImuT.data()[4] << ", " << camImuT.data()[5] << ", " << camImuT.data()[6]  << ", " << camImuT.data()[7]  << ", " << std::endl;
		ofs << "         "  << camImuT.data()[8] << ", " << camImuT.data()[9] << ", " << camImuT.data()[10] << ", " << camImuT.data()[11] << ", " << std::endl;
		ofs << "         0.0, 0.0, 0.0, 1.0]" << std::endl;
		ofs << std::endl;

		ofs << "IMU.InsertKFsWhenLost: " << 0 << std::endl;
		ofs << std::endl;

		double gyroNoise = rtabmap::Parameters::defaultOdomORBSLAMGyroNoise();
		double accNoise = rtabmap::Parameters::defaultOdomORBSLAMAccNoise();
		double gyroWalk = rtabmap::Parameters::defaultOdomORBSLAMGyroWalk();
		double accWalk = rtabmap::Parameters::defaultOdomORBSLAMAccWalk();
		double samplingRate = rtabmap::Parameters::defaultOdomORBSLAMSamplingRate();
		rtabmap::Parameters::parse(parameters_, rtabmap::Parameters::kOdomORBSLAMGyroNoise(), gyroNoise);
		rtabmap::Parameters::parse(parameters_, rtabmap::Parameters::kOdomORBSLAMAccNoise(), accNoise);
		rtabmap::Parameters::parse(parameters_, rtabmap::Parameters::kOdomORBSLAMGyroWalk(), gyroWalk);
		rtabmap::Parameters::parse(parameters_, rtabmap::Parameters::kOdomORBSLAMAccWalk(), accWalk);
		rtabmap::Parameters::parse(parameters_, rtabmap::Parameters::kOdomORBSLAMSamplingRate(), samplingRate);

		ofs << "IMU.NoiseGyro: " << gyroNoise << std::endl; // 1e-2
		ofs << "IMU.NoiseAcc: " << accNoise << std::endl; // 1e-1
		ofs << "IMU.GyroWalk: " << gyroWalk << std::endl; // 1e-6
		ofs << "IMU.AccWalk: " << accWalk << std::endl; // 1e-4
		if(samplingRate == 0)
		{
			// estimate rate from imu received.
			UASSERT(orbslamImus_.size() > 1 && orbslamImus_[0].t < orbslamImus_[1].t);
			samplingRate = 1./(orbslamImus_[1].t - orbslamImus_[0].t);
			samplingRate = std::round(samplingRate);
			UWARN("IMU sampling rate estimated at %.0f Hz. If this doesn't look good, "
				  "set explicitly parameter %s to expected frequency.",
				  samplingRate,	Parameters::kOdomORBSLAMSamplingRate().c_str());
		}
		ofs << "IMU.Frequency: " << samplingRate << std::endl; // 200
		ofs << std::endl;
	}



	//#--------------------------------------------------------------------------------------------
	//# ORB Parameters
	//#--------------------------------------------------------------------------------------------
	//# ORB Extractor: Number of features per image
	int features = rtabmap::Parameters::defaultOdomORBSLAMMaxFeatures();
	rtabmap::Parameters::parse(parameters_, rtabmap::Parameters::kOdomORBSLAMMaxFeatures(), features);
	ofs << "ORBextractor.nFeatures: " << features << std::endl;
	ofs << std::endl;

	//# ORB Extractor: Scale factor between levels in the scale pyramid
	double scaleFactor = rtabmap::Parameters::defaultORBScaleFactor();
	rtabmap::Parameters::parse(parameters_, rtabmap::Parameters::kORBScaleFactor(), scaleFactor);
	ofs << "ORBextractor.scaleFactor: " << scaleFactor << std::endl;
	ofs << std::endl;

	//# ORB Extractor: Number of levels in the scale pyramid
	int levels = rtabmap::Parameters::defaultORBNLevels();
	rtabmap::Parameters::parse(parameters_, rtabmap::Parameters::kORBNLevels(), levels);
	ofs << "ORBextractor.nLevels: " << levels << std::endl;
	ofs << std::endl;

	//# ORB Extractor: Fast threshold
	//# Image is divided in a grid. At each cell FAST are extracted imposing a minimum response.
	//# Firstly we impose iniThFAST. If no corners are detected we impose a lower value minThFAST
	//# You can lower these values if your images have low contrast
	int iniThFAST = rtabmap::Parameters::defaultFASTThreshold();
	int minThFAST = rtabmap::Parameters::defaultFASTMinThreshold();
	rtabmap::Parameters::parse(parameters_, rtabmap::Parameters::kFASTThreshold(), iniThFAST);
	rtabmap::Parameters::parse(parameters_, rtabmap::Parameters::kFASTMinThreshold(), minThFAST);
	ofs << "ORBextractor.iniThFAST: " << iniThFAST << std::endl;
	ofs << "ORBextractor.minThFAST: " << minThFAST << std::endl;
	ofs << std::endl;

	int maxFeatureMapSize = rtabmap::Parameters::defaultOdomORBSLAMMapSize();
	rtabmap::Parameters::parse(parameters_, rtabmap::Parameters::kOdomORBSLAMMapSize(), maxFeatureMapSize);

	//# Disable loop closure detection
	ofs << "loopClosing: " << 0 << std::endl;
	ofs << std::endl;

	//# Set dummy Viewer parameters
	ofs << "Viewer.KeyFrameSize: " << 0.05 << std::endl;
	ofs << "Viewer.KeyFrameLineWidth: " << 1.0 << std::endl;
	ofs << "Viewer.GraphLineWidth: " << 0.9 << std::endl;
	ofs << "Viewer.PointSize: " << 2.0 << std::endl;
	ofs << "Viewer.CameraSize: " << 0.08 << std::endl;
	ofs << "Viewer.CameraLineWidth: " << 3.0 << std::endl;
	ofs << "Viewer.ViewpointX: " << 0.0 << std::endl;
	ofs << "Viewer.ViewpointY: " << -0.7 << std::endl;
	ofs << "Viewer.ViewpointZ: " << -3.5 << std::endl;
	ofs << "Viewer.ViewpointF: " << 500.0 << std::endl;
	ofs << std::endl;

	ofs.close();

	orbslam_ = new ORB_SLAM3::System(
			vocabularyPath,
			configPath,
			stereo && withIMU?ORB_SLAM3::System::IMU_STEREO:
			stereo?ORB_SLAM3::System::STEREO:
			withIMU?ORB_SLAM3::System::IMU_RGBD:
					ORB_SLAM3::System::RGBD,
			false);
	return true;
#else
	UERROR("RTAB-Map is not built with ORB_SLAM support! Select another visual odometry approach.");
#endif
	return false;
}

// return not null transform if odometry is correctly computed
Transform OdometryORBSLAM3::computeTransform(
		SensorData & data,
		const Transform & guess,
		OdometryInfo * info)
{
	Transform t;

#if defined(RTABMAP_ORB_SLAM) and RTABMAP_ORB_SLAM == 3
	UTimer timer;

	if(useIMU_)
	{
		bool added = false;
		if(!data.imu().empty())
		{
			if(lastImuStamp_ == 0.0 || lastImuStamp_ < data.stamp())
			{
				orbslamImus_.push_back(ORB_SLAM3::IMU::Point(
						data.imu().linearAcceleration().val[0],
						data.imu().linearAcceleration().val[1],
						data.imu().linearAcceleration().val[2],
						data.imu().angularVelocity().val[0],
						data.imu().angularVelocity().val[1],
						data.imu().angularVelocity().val[2],
						data.stamp()));
				lastImuStamp_ = data.stamp();
				added = true;
			}
			else
			{
				UERROR("Received IMU with stamp (%f) <= than the previous IMU (%f), ignoring it!", data.stamp(), lastImuStamp_);
			}
		}

		if(orbslam_ == 0)
		{
			// We need two samples to estimate imu frame rate
			if(orbslamImus_.size()>1 && added)
			{
				imuLocalTransform_ = data.imu().localTransform();
			}
		}

		if(data.imageRaw().empty() || imuLocalTransform_.isNull())
		{
			return Transform();
		}
	}

	if(data.imageRaw().empty() ||
		data.imageRaw().rows != data.depthOrRightRaw().rows ||
		data.imageRaw().cols != data.depthOrRightRaw().cols)
	{
		UERROR("Not supported input! RGB (%dx%d) and depth (%dx%d) should have the same size.",
				data.imageRaw().cols, data.imageRaw().rows, data.depthOrRightRaw().cols, data.depthOrRightRaw().rows);
		return t;
	}

	if(!((data.cameraModels().size() == 1 &&
			data.cameraModels()[0].isValidForReprojection()) ||
		(data.stereoCameraModels().size() == 1 &&
			data.stereoCameraModels()[0].isValidForProjection())))
	{
		UERROR("Invalid camera model!");
		return t;
	}

	bool stereo = data.cameraModels().size() == 0;

	cv::Mat covariance;
	if(orbslam_ == 0)
	{
		// We need two frames to estimate camera frame rate
		if(lastImageStamp_ == 0.0)
		{
			lastImageStamp_ = data.stamp();
			return t;
		}

		CameraModel model = data.cameraModels().size()==1?data.cameraModels()[0]:data.stereoCameraModels()[0].left();
		if(!init(model, data.stamp(), stereo, data.cameraModels().size()==1?0.0:data.stereoCameraModels()[0].baseline()))
		{
			return t;
		}
	}

	Sophus::SE3f Tcw;
	Transform localTransform;
	if(stereo)
	{
		localTransform = data.stereoCameraModels()[0].localTransform();

		Tcw = orbslam_->TrackStereo(data.imageRaw(), data.rightRaw(), data.stamp(), orbslamImus_);
		orbslamImus_.clear();
	}
	else
	{
		localTransform = data.cameraModels()[0].localTransform();
		cv::Mat depth;
		if(data.depthRaw().type() == CV_32FC1)
		{
			depth = data.depthRaw();
		}
		else if(data.depthRaw().type() == CV_16UC1)
		{
			depth = util2d::cvtDepthToFloat(data.depthRaw());
		}
		Tcw = orbslam_->TrackRGBD(data.imageRaw(), depth, data.stamp(), orbslamImus_);
		orbslamImus_.clear();
	}

	Transform previousPoseInv = previousPose_.inverse();
	std::vector<ORB_SLAM3::MapPoint*> mapPoints = orbslam_->GetTrackedMapPoints();
	if(orbslam_->isLost() || mapPoints.empty())
	{
		covariance = cv::Mat::eye(6,6,CV_64FC1)*9999.0f;
	}
	else
	{
		cv::Mat TcwMat = ORB_SLAM3::Converter::toCvMat(ORB_SLAM3::Converter::toSE3Quat(Tcw)).clone();
		UASSERT(TcwMat.cols == 4 && TcwMat.rows == 4);
		Transform p = Transform(cv::Mat(TcwMat, cv::Range(0,3), cv::Range(0,4)));

		if(!p.isNull())
		{
			if(!localTransform.isNull())
			{
				if(originLocalTransform_.isNull())
				{
					originLocalTransform_ = localTransform;
				}
				// transform in base frame
				p = originLocalTransform_ * p.inverse() * localTransform.inverse();
			}
			t = previousPoseInv*p;
		}
		previousPose_ = p;

		if(firstFrame_)
		{
			// just recovered of being lost, set high covariance
			covariance = cv::Mat::eye(6,6,CV_64FC1)*9999.0f;
			firstFrame_ = false;
		}
		else
		{
			float baseline = data.cameraModels().size()==1?0.0f:data.stereoCameraModels()[0].baseline();
			if(baseline <= 0.0f)
			{
				baseline = rtabmap::Parameters::defaultOdomORBSLAMBf();
				rtabmap::Parameters::parse(parameters_, rtabmap::Parameters::kOdomORBSLAMBf(), baseline);
			}
			double linearVar = 0.0001;
			if(baseline > 0.0f)
			{
				linearVar = baseline/8.0;
				linearVar *= linearVar;
			}

			covariance = cv::Mat::eye(6,6, CV_64FC1);
			covariance.at<double>(0,0) = linearVar;
			covariance.at<double>(1,1) = linearVar;
			covariance.at<double>(2,2) = linearVar;
			covariance.at<double>(3,3) = 0.0001;
			covariance.at<double>(4,4) = 0.0001;
			covariance.at<double>(5,5) = 0.0001;
		}
	}

	if(info)
	{
		info->lost = t.isNull();
		info->type = (int)kTypeORBSLAM;
		info->reg.covariance = covariance;
		info->localMapSize = mapPoints.size();
		info->localKeyFrames = 0;

		if(this->isInfoDataFilled())
		{
			std::vector<cv::KeyPoint> kpts = orbslam_->GetTrackedKeyPointsUn();
			info->reg.matchesIDs.resize(kpts.size());
			info->reg.inliersIDs.resize(kpts.size());
			int oi = 0;

			UASSERT(mapPoints.size() == kpts.size());
			for (unsigned int i = 0; i < kpts.size(); ++i)
			{
				int wordId;
				if(mapPoints[i] != 0)
				{
					wordId = mapPoints[i]->mnId;
				}
				else
				{
					wordId = -(i+1);
				}
				info->words.insert(std::make_pair(wordId, kpts[i]));
				if(mapPoints[i] != 0)
				{
					info->reg.matchesIDs[oi] = wordId;
					info->reg.inliersIDs[oi] = wordId;
					++oi;
				}
			}
			info->reg.matchesIDs.resize(oi);
			info->reg.inliersIDs.resize(oi);
			info->reg.inliers = oi;
			info->reg.matches = oi;

			Eigen::Affine3f fixRot = (this->getPose()*previousPoseInv*originLocalTransform_).toEigen3f();
			for (unsigned int i = 0; i < mapPoints.size(); ++i)
			{
				if(mapPoints[i])
				{
					Eigen::Vector3f pt = mapPoints[i]->GetWorldPos();
					pcl::PointXYZ ptt = pcl::transformPoint(pcl::PointXYZ(pt[0], pt[1], pt[2]), fixRot);
					info->localMap.insert(std::make_pair(mapPoints[i]->mnId, cv::Point3f(ptt.x, ptt.y, ptt.z)));
				}
			}
		}
	}

	UINFO("Odom update time = %fs, map points=%ld, lost=%s", timer.elapsed(), mapPoints.size(), t.isNull()?"true":"false");

#else
	UERROR("RTAB-Map is not built with ORB_SLAM support! Select another visual odometry approach.");
#endif
	return t;
}

} // namespace rtabmap
