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

#include "rtabmap/core/odometry/OdometryVINSFusion.h"
#include "rtabmap/core/OdometryInfo.h"
#include "rtabmap/core/util3d_transforms.h"
#include "rtabmap/utilite/ULogger.h"
#include "rtabmap/utilite/UTimer.h"
#include "rtabmap/utilite/UStl.h"
#include "rtabmap/utilite/UThread.h"
#include "rtabmap/utilite/UDirectory.h"
#include <opencv2/imgproc/types_c.h>

#ifdef RTABMAP_VINS_FUSION
#include <estimator/estimator.h>
#include <estimator/parameters.h>
#include <camodocal/camera_models/PinholeCamera.h>
#include <camodocal/camera_models/EquidistantCamera.h>
#include <utility/visualization.h>
#endif

namespace rtabmap {

#ifdef RTABMAP_VINS_FUSION
class VinsFusionEstimator: public Estimator
{
public:
	VinsFusionEstimator(
			const Transform & imuLocalTransform,
			const StereoCameraModel & model,
			bool rectified) : Estimator()
	{
		MULTIPLE_THREAD = 0;
		setParameter();

		ROW=model.left().imageHeight();
		COL=model.left().imageWidth();

		//overwrite camera calibration only if received model is radtan, otherwise use config
		UASSERT(NUM_OF_CAM >= 1 && NUM_OF_CAM <=2);

		if( (NUM_OF_CAM == 2 && model.left().D_raw().cols == 4 && model.right().D_raw().cols == 4) ||
			(NUM_OF_CAM == 1 && model.left().D_raw().cols == 4))
		{
			UWARN("Overwriting VINS camera calibration config with received pinhole model... rectified=%d", rectified?1:0);
			featureTracker.m_camera.clear();

			camodocal::PinholeCameraPtr camera( new camodocal::PinholeCamera );
			camodocal::PinholeCamera::Parameters params(
					model.name(),
					model.left().imageWidth(), model.left().imageHeight(),
					rectified?0:model.left().D_raw().at<double>(0,0),
					rectified?0:model.left().D_raw().at<double>(0,1),
					rectified?0:model.left().D_raw().at<double>(0,2),
					rectified?0:model.left().D_raw().at<double>(0,3),
					rectified?model.left().fx():model.left().K_raw().at<double>(0,0),
					rectified?model.left().fy():model.left().K_raw().at<double>(1,1),
					rectified?model.left().cx():model.left().K_raw().at<double>(0,2),
					rectified?model.left().cy():model.left().K_raw().at<double>(1,2));
			camera->setParameters(params);
			featureTracker.m_camera.push_back(camera);

			double originalParalax = MIN_PARALLAX * FOCAL_LENGTH;
			// If you have compiler error about FOCAL_LENGTH being const, make sure to use the following patch:
			// https://gist.github.com/matlabbe/795ab37067367dca58bbadd8201d986c#file-vins-fusion_pull136-patch
			FOCAL_LENGTH = params.fx();
			MIN_PARALLAX = originalParalax / FOCAL_LENGTH;
			ProjectionTwoFrameOneCamFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
			ProjectionTwoFrameTwoCamFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
			ProjectionOneFrameTwoCamFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();

			if(NUM_OF_CAM == 2)
			{
				camodocal::PinholeCameraPtr camera( new camodocal::PinholeCamera );
				camodocal::PinholeCamera::Parameters params(
						model.name(),
						model.right().imageWidth(), model.right().imageHeight(),
						rectified?0:model.right().D_raw().at<double>(0,0),
						rectified?0:model.right().D_raw().at<double>(0,1),
						rectified?0:model.right().D_raw().at<double>(0,2),
						rectified?0:model.right().D_raw().at<double>(0,3),
						rectified?model.right().fx():model.right().K_raw().at<double>(0,0),
						rectified?model.right().fy():model.right().K_raw().at<double>(1,1),
						rectified?model.right().cx():model.right().K_raw().at<double>(0,2),
						rectified?model.right().cy():model.right().K_raw().at<double>(1,2));
				camera->setParameters(params);
				featureTracker.m_camera.push_back(camera);
			}
		}
		else if(rectified)
		{
			UWARN("Images are rectified but received calibration cannot be "
					"used, make sure calibration in config file doesn't have "
					"distortion or send raw images to VINS odometry.");
			if(!featureTracker.m_camera.empty())
			{
				if(featureTracker.m_camera.front()->imageWidth() != model.left().imageWidth() ||
				   featureTracker.m_camera.front()->imageHeight() != model.left().imageHeight())
				{
					UERROR("Received images don't have same size (%dx%d) than in the config file (%dx%d)!",
							model.left().imageWidth(),
							model.left().imageHeight(),
							featureTracker.m_camera.front()->imageWidth(),
							featureTracker.m_camera.front()->imageHeight());
				}
			}
		}

		Transform imuCam0 = imuLocalTransform.inverse() * model.localTransform();

		tic[0] = TIC[0] = Vector3d(imuCam0.x(), imuCam0.y(), imuCam0.z());
		ric[0] = RIC[0] = imuCam0.toEigen4d().block<3,3>(0,0);

		if(NUM_OF_CAM == 2)
		{
			Transform cam0cam1;
			if(rectified)
			{
				cam0cam1 = Transform(
						1, 0, 0, model.baseline(),
						0, 1, 0, 0,
						0, 0, 1, 0);
			}
			else
			{
				cam0cam1 = model.stereoTransform().inverse();
			}
			UASSERT(!cam0cam1.isNull());
			Transform imuCam1 = imuCam0 * cam0cam1;

			tic[1] = TIC[0] = Vector3d(imuCam1.x(), imuCam1.y(), imuCam1.z());
			ric[1] = RIC[0] = imuCam1.toEigen4d().block<3,3>(0,0);
		}

		for (int i = 0; i < NUM_OF_CAM; i++)
		{
			cout << " new extrinsic cam " << i << endl  << ric[i] << endl << tic[i].transpose() << endl;
		}
		for (int i = 0; i < NUM_OF_CAM; i++)
		{
			cout << " new intrinsic cam " << i << endl  << featureTracker.m_camera[i]->parametersToString() << endl;
		}
		f_manager.setRic(ric);
	}

	// Copy of original inputImage() so that overridden processMeasurements() is used and threading is disabled.
	void inputImage(double t, const cv::Mat &_img, const cv::Mat &_img1)
	{
		TicToc processTime;
	    inputImageCnt++;
	    map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> featureFrame;
	    if(_img1.empty()) {
	        featureFrame = featureTracker.trackImage(t, _img);
		}
	    else {
	        featureFrame = featureTracker.trackImage(t, _img, _img1);
		}

		mBuf.lock();
		featureBuf.push(make_pair(t, featureFrame));
		mBuf.unlock();
		processMeasurements();
		UDEBUG("VINS process time: %f", processTime.toc());
	}

	// Copy of original inputIMU() but with publisher commented
	void inputIMU(double t, const Vector3d &linearAcceleration, const Vector3d &angularVelocity)
	{
		mBuf.lock();
		accBuf.push(make_pair(t, linearAcceleration));
		gyrBuf.push(make_pair(t, angularVelocity));
		//printf("input imu with time %f \n", t);
		mBuf.unlock();

		if (solver_flag == NON_LINEAR)
		{
			mPropagate.lock();
			fastPredictIMU(t, linearAcceleration, angularVelocity);
			mPropagate.unlock();
		}
	}

	// Copy of original processMeasurements() but with publishers commented and threading disabled
	void processMeasurements()
	{
		pair<double, map<int, vector<pair<int, Eigen::Matrix<double, 7, 1> > > > > feature;
		vector<pair<double, Eigen::Vector3d>> accVector, gyrVector;
		if(!featureBuf.empty())
		{
			feature = featureBuf.front();
			curTime = feature.first + td;
			if (USE_IMU && !IMUAvailable(feature.first + td))
			{
				printf("wait for imu ... \n");
				return;
			}
			mBuf.lock();
			if(USE_IMU)
				getIMUInterval(prevTime, curTime, accVector, gyrVector);

			featureBuf.pop();
			mBuf.unlock();

			if(USE_IMU)
			{
				if(!initFirstPoseFlag)
					initFirstIMUPose(accVector);

				for(size_t i = 0; i < accVector.size(); i++)
				{
					double dt;
					if(i == 0)
						dt = accVector[i].first - prevTime;
					else if (i == accVector.size() - 1)
						dt = curTime - accVector[i - 1].first;
					else
						dt = accVector[i].first - accVector[i - 1].first;
					processIMU(accVector[i].first, dt, accVector[i].second, gyrVector[i].second);
				}
			}
			mProcess.lock();
			processImage(feature.second, feature.first);
			prevTime = curTime;																
			mProcess.unlock();
		}
	}
};
#endif

OdometryVINSFusion::OdometryVINSFusion(const ParametersMap & parameters) :
	Odometry(parameters)
#ifdef RTABMAP_VINS_FUSION
    ,
	vinsEstimator_(0),
	initGravity_(false),
	previousPose_(Transform::getIdentity())
#endif
{
#ifdef RTABMAP_VINS_FUSION
	// intialize
	std::string configFilename;
	Parameters::parse(parameters, Parameters::kOdomVINSFusionConfigPath(), configFilename);
	if(configFilename.empty())
	{
		UERROR("VINS config file is empty (%s)!",
				Parameters::kOdomVINSFusionConfigPath().c_str());
	}
	else
	{
		UINFO("Using config file %s", configFilename.c_str());
		readParameters(uReplaceChar(configFilename, '~', UDirectory::homeDir()));
	}
#endif
}

OdometryVINSFusion::~OdometryVINSFusion()
{
#ifdef RTABMAP_VINS_FUSION
	delete vinsEstimator_;
#endif
}

void OdometryVINSFusion::reset(const Transform & initialPose)
{
	Odometry::reset(initialPose);
#ifdef RTABMAP_VINS_FUSION
	if(!initGravity_)
	{
		delete vinsEstimator_;
		vinsEstimator_ = 0;
		previousPose_.setIdentity();
		lastImu_ = IMU();
		previousLocalTransform_.setNull();
	}
	initGravity_ = false;
#endif
}

// return not null transform if odometry is correctly computed
Transform OdometryVINSFusion::computeTransform(
		SensorData & data,
		const Transform & guess,
		OdometryInfo * info)
{
	Transform t;
#ifdef RTABMAP_VINS_FUSION
	UTimer timer;

	bool hasImage = !data.imageRaw().empty() && !data.rightRaw().empty() && data.stereoCameraModels().size() == 1 && data.stereoCameraModels()[0].isValidForProjection();

	if(USE_IMU!=0 && !data.imu().empty())
	{
		double dx = data.imu().linearAcceleration().val[0];
		double dy = data.imu().linearAcceleration().val[1];
		double dz = data.imu().linearAcceleration().val[2];
		double rx = data.imu().angularVelocity().val[0];
		double ry = data.imu().angularVelocity().val[1];
		double rz = data.imu().angularVelocity().val[2];
		Vector3d acc(dx, dy, dz);
		Vector3d gyr(rx, ry, rz);

		UDEBUG("IMU update stamp=%f", data.stamp());

		if(vinsEstimator_ != 0)
		{
			vinsEstimator_->inputIMU(data.stamp(), acc, gyr);
		}
		else
		{
			lastImu_ = data.imu();
			lastImuStamp_ = data.stamp();
			if(!hasImage) {
				UWARN("Waiting an image for initialization...");
			}
		}
	}

	if(hasImage)
	{
		if(USE_IMU==1 && lastImu_.localTransform().isNull())
		{
			UWARN("Waiting IMU for initialization...");
			return t;
		}
		if(vinsEstimator_ == 0)
		{
			// intialize
			UINFO("Initializing with image %f", data.stamp());
			vinsEstimator_ = new VinsFusionEstimator(
					lastImu_.localTransform().isNull()?Transform::getIdentity():lastImu_.localTransform(),
					data.stereoCameraModels()[0],
					this->imagesAlreadyRectified());

			if(USE_IMU) {
				double dx = lastImu_.linearAcceleration().val[0];
				double dy = lastImu_.linearAcceleration().val[1];
				double dz = lastImu_.linearAcceleration().val[2];
				double rx = lastImu_.angularVelocity().val[0];
				double ry = lastImu_.angularVelocity().val[1];
				double rz = lastImu_.angularVelocity().val[2];
				Vector3d acc(dx, dy, dz);
				Vector3d gyr(rx, ry, rz);
				vinsEstimator_->inputIMU(lastImuStamp_, acc, gyr);
			}
		}

		UDEBUG("Image update stamp=%f", data.stamp());
		cv::Mat left;
		cv::Mat right;
		if(data.imageRaw().type() == CV_8UC3)
		{
			cv::cvtColor(data.imageRaw(), left, CV_BGR2GRAY);
		}
		else if(data.imageRaw().type() == CV_8UC1)
		{
			left = data.imageRaw().clone();
		}
		else
		{
			UFATAL("Not supported color type!");
		}
		if(data.rightRaw().type() == CV_8UC3)
		{
			cv::cvtColor(data.rightRaw(), right, CV_BGR2GRAY);
		}
		else if(data.rightRaw().type() == CV_8UC1)
		{
			right = data.rightRaw().clone();
		}
		else
		{
			UFATAL("Not supported color type!");
		}

		vinsEstimator_->inputImage(data.stamp(), left, right);

		if(vinsEstimator_->solver_flag == Estimator::NON_LINEAR)
		{
			Quaterniond tmp_Q;
			tmp_Q = Quaterniond(vinsEstimator_->Rs[WINDOW_SIZE]);
			Transform p(
					vinsEstimator_->Ps[WINDOW_SIZE].x(),
					vinsEstimator_->Ps[WINDOW_SIZE].y(),
					vinsEstimator_->Ps[WINDOW_SIZE].z(),
					tmp_Q.x(),
					tmp_Q.y(),
					tmp_Q.z(),
					tmp_Q.w());

			if(!p.isNull())
			{
				if(!lastImu_.localTransform().isNull())
				{
					p = p * lastImu_.localTransform().inverse();
				}

				if(this->getPose().rotation().isIdentity())
				{
					initGravity_ = true;
					this->reset(this->getPose()*p.rotation());
				}

				if(previousPose_.isIdentity())
				{
					previousPose_ = p;
				}

				// make it incremental
				Transform previousPoseInv = previousPose_.inverse();
				t = previousPoseInv*p;
				previousPose_ = p;

				if(info)
				{
					info->type = this->getType();
					info->reg.covariance = cv::Mat::eye(6,6, CV_64FC1);
					info->reg.covariance *= this->framesProcessed() == 0?9999:0.0001;

					// feature map: based on code from pubPointCloud() of vins's visualization.cpp
					for (auto &it_per_id : vinsEstimator_->f_manager.feature)
					{
						if(it_per_id.feature_per_frame.size() < 2) {
							// feature just added but not tracked, or old feature not tracked anymore
							continue;
						}

						int imu_i = it_per_id.start_frame;
						Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
						Vector3d w_pts_i = vinsEstimator_->Rs[imu_i] * (vinsEstimator_->ric[0] * pts_i + vinsEstimator_->tic[0]) + vinsEstimator_->Ps[imu_i];

						cv::Point3f p;
						p.x = w_pts_i(0);
						p.y = w_pts_i(1);
						p.z = w_pts_i(2);

						int featureIndex = info->localMap.size();
						info->localMap.insert(std::make_pair(featureIndex, p));

						FeaturePerFrame & refFrame = it_per_id.feature_per_frame[0]; // First frame it was seen
						FeaturePerFrame & newFrame = it_per_id.feature_per_frame[it_per_id.feature_per_frame.size()-1]; // Last frame it was seen (not necessary in last frame)
						cv::Point2f refUV(refFrame.uv[0], refFrame.uv[1]);
						cv::Point2f newUV(newFrame.uv[0], newFrame.uv[1]);
						info->refCorners.push_back(refUV);
						info->newCorners.push_back(newUV);

						info->reg.matchesIDs.push_back(featureIndex);
						if(it_per_id.solve_flag > 0) {
							// Feature correctly tracked
							info->words.insert(std::make_pair(featureIndex, cv::KeyPoint(newUV, 3.0f)));
							info->cornerInliers.push_back(featureIndex);
							info->reg.inliersIDs.push_back(featureIndex);
						}

						++featureIndex;
					}
					info->features = info->localMap.size();
					info->reg.inliers = info->reg.inliersIDs.size();
					info->localMapSize = info->localMap.size();
				}
				UINFO("Odom update time = %fs p=%s", timer.elapsed(), p.prettyPrint().c_str());
			}
		}
		else
		{
			UWARN("VINS-Fusion not yet initialized... needing more data.");
		}
	}
	else if(!data.imageRaw().empty() && !data.depthRaw().empty())
	{
		UERROR("VINS-Fusion doesn't work with RGB-D data, stereo images are required!");
	}
	else if(!data.imageRaw().empty() && data.depthOrRightRaw().empty())
	{
		UERROR("VINS-Fusion requires stereo images!");
	}
	else if(data.imu().empty())
	{
		UERROR("VINS-Fusion requires stereo images (and only one stereo camera with valid calibration)!");
	}
#else
	UERROR("RTAB-Map is not built with VINS support! Select another visual odometry approach.");
#endif
	return t;
}

} // namespace rtabmap
