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

#include "rtabmap/core/odometry/OdometryVINS.h"
#include "rtabmap/core/OdometryInfo.h"
#include "rtabmap/core/util3d_transforms.h"
#include "rtabmap/utilite/ULogger.h"
#include "rtabmap/utilite/UTimer.h"
#include "rtabmap/utilite/UStl.h"
#include "rtabmap/utilite/UThread.h"
#include "rtabmap/utilite/UDirectory.h"
#include <opencv2/imgproc/types_c.h>

#ifdef RTABMAP_VINS
#include <estimator/estimator.h>
#include <estimator/parameters.h>
#include <camodocal/camera_models/PinholeCamera.h>
#include <camodocal/camera_models/EquidistantCamera.h>
#include <utility/visualization.h>
#endif

namespace rtabmap {

#ifdef RTABMAP_VINS
class VinsEstimator: public Estimator
{
public:
	VinsEstimator(
			const Transform & imuLocalTransform,
			const StereoCameraModel & model,
			bool rectified) : Estimator()
	{
		MULTIPLE_THREAD = 0;
		setParameter();

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

		tic[0] = Vector3d(imuCam0.x(), imuCam0.y(), imuCam0.z());
		ric[0] = imuCam0.toEigen4d().block<3,3>(0,0);

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

			tic[1] = Vector3d(imuCam1.x(), imuCam1.y(), imuCam1.z());
			ric[1] = imuCam1.toEigen4d().block<3,3>(0,0);
		}

		for (int i = 0; i < NUM_OF_CAM; i++)
		{
			cout << " exitrinsic cam " << i << endl  << ric[i] << endl << tic[i].transpose() << endl;
		}
		f_manager.setRic(ric);
		ProjectionTwoFrameOneCamFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
		ProjectionTwoFrameTwoCamFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
		ProjectionOneFrameTwoCamFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
		td = TD;
		g = G;
		cout << "set g " << g.transpose() << endl;
	}

	// Copy of original inputImage() so that overridden processMeasurements() is used and threading is disabled.
	void inputImage(double t, const cv::Mat &_img, const cv::Mat &_img1)
	{
	    inputImageCnt++;
	    map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> featureFrame;
	    TicToc featureTrackerTime;
	    if(_img1.empty())
	        featureFrame = featureTracker.trackImage(t, _img);
	    else
	        featureFrame = featureTracker.trackImage(t, _img, _img1);
	    //printf("featureTracker time: %f\n", featureTrackerTime.toc());

	    //if(MULTIPLE_THREAD)
	    //{
	    //    if(inputImageCnt % 2 == 0)
	    //    {
	    //        mBuf.lock();
	    //        featureBuf.push(make_pair(t, featureFrame));
	    //        mBuf.unlock();
	    //    }
	    //}
	    //else
	    {
	        mBuf.lock();
	        featureBuf.push(make_pair(t, featureFrame));
	        mBuf.unlock();
	        TicToc processTime;
	        processMeasurements();
	        UDEBUG("VINS process time: %f", processTime.toc());
	    }

	}

	// Copy of original inputIMU() but with publisher commented
	void inputIMU(double t, const Vector3d &linearAcceleration, const Vector3d &angularVelocity)
	{
		mBuf.lock();
		accBuf.push(make_pair(t, linearAcceleration));
		gyrBuf.push(make_pair(t, angularVelocity));
		//printf("input imu with time %f \n", t);
		mBuf.unlock();

		fastPredictIMU(t, linearAcceleration, angularVelocity);
		//if (solver_flag == NON_LINEAR)
		//	pubLatestOdometry(latest_P, latest_Q, latest_V, t);
	}

	// Copy of original processMeasurements() but with publishers commented and threading disabled
	void processMeasurements()
	{
		//while (1)
		{
			//printf("process measurments\n");
			pair<double, map<int, vector<pair<int, Eigen::Matrix<double, 7, 1> > > > > feature;
			vector<pair<double, Eigen::Vector3d>> accVector, gyrVector;
			if(!featureBuf.empty())
			{
				feature = featureBuf.front();
				curTime = feature.first + td;
				//while(1)
				//{
					if (!((!USE_IMU  || IMUAvailable(feature.first + td))))
					//if ((!USE_IMU  || IMUAvailable(feature.first + td)))
					//	break;
					//else
					{
						printf("wait for imu ... \n");
						//if (! MULTIPLE_THREAD)
							return;
						//std::chrono::milliseconds dura(5);
						//std::this_thread::sleep_for(dura);
					}
				//}
				mBuf.lock();
				if(USE_IMU)
					getIMUInterval(prevTime, curTime, accVector, gyrVector);

				featureBuf.pop();
				mBuf.unlock();

				if(USE_IMU)
				{
					if(!initFirstPoseFlag)
						initFirstIMUPose(accVector);
					UDEBUG("accVector.size() = %d", accVector.size());
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

				processImage(feature.second, feature.first);

				prevTime = curTime;

				printStatistics(*this, 0);

				//std_msgs::Header header;
				//header.frame_id = "world";
				//header.stamp = ros::Time(feature.first);

				//pubOdometry(*this, header);
				//pubKeyPoses(*this, header);
				//pubCameraPose(*this, header);
				//pubPointCloud(*this, header);
				//pubKeyframe(*this);
				//pubTF(*this, header);
			}

			//if (! MULTIPLE_THREAD)
			//	break;

			//std::chrono::milliseconds dura(2);
			//std::this_thread::sleep_for(dura);
		}
	}
};
#endif

OdometryVINS::OdometryVINS(const ParametersMap & parameters) :
	Odometry(parameters)
#ifdef RTABMAP_VINS
    ,
	vinsEstimator_(0),
	initGravity_(false),
	previousPose_(Transform::getIdentity())
#endif
{
#ifdef RTABMAP_VINS
	// intialize
	std::string configFilename;
	Parameters::parse(parameters, Parameters::kOdomVINSConfigPath(), configFilename);
	if(configFilename.empty())
	{
		UERROR("VINS config file is empty (%s=%s)!",
				Parameters::kOdomVINSConfigPath().c_str(),
				Parameters::kOdomVINSConfigPath().c_str());
	}
	else
	{
		readParameters(uReplaceChar(configFilename, '~', UDirectory::homeDir()));
	}
#endif
}

OdometryVINS::~OdometryVINS()
{
#ifdef RTABMAP_VINS
	delete vinsEstimator_;
#endif
}

void OdometryVINS::reset(const Transform & initialPose)
{
	Odometry::reset(initialPose);
#ifdef RTABMAP_VINS
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
Transform OdometryVINS::computeTransform(
		SensorData & data,
		const Transform & guess,
		OdometryInfo * info)
{
	Transform t;
#ifdef RTABMAP_VINS
	UTimer timer;

	if(USE_IMU!=0 && !data.imu().empty())
	{
		double t = data.stamp();
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
			vinsEstimator_->inputIMU(t, acc, gyr);
		}
		else
		{
			lastImu_ = data.imu();
			UWARN("Waiting an image for initialization...");
		}
	}

	if(!data.imageRaw().empty() && !data.rightRaw().empty() && data.stereoCameraModels().size() == 1 && data.stereoCameraModels()[0].isValidForProjection())
	{
		if(USE_IMU==1 && lastImu_.localTransform().isNull())
		{
			UWARN("Waiting IMU for initialization...");
			return t;
		}
		if(vinsEstimator_ == 0)
		{
			// intialize
			vinsEstimator_ = new VinsEstimator(
					lastImu_.localTransform().isNull()?Transform::getIdentity():lastImu_.localTransform(),
					data.stereoCameraModels()[0],
					this->imagesAlreadyRectified());
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

					// feature map
					Transform fixT = this->getPose()*previousPoseInv;
					for (auto &it_per_id : vinsEstimator_->f_manager.feature)
					{
						int used_num;
						used_num = it_per_id.feature_per_frame.size();
						if (!(used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
							continue;
						if (it_per_id.start_frame > WINDOW_SIZE * 3.0 / 4.0 || it_per_id.solve_flag != 1)
							continue;
						int imu_i = it_per_id.start_frame;
						Vector3d pts_i = it_per_id.feature_per_frame[it_per_id.feature_per_frame.size()-1].point * it_per_id.estimated_depth;
						Vector3d w_pts_i = vinsEstimator_->Rs[imu_i] * (vinsEstimator_->ric[0] * pts_i + vinsEstimator_->tic[0]) + vinsEstimator_->Ps[imu_i];

						cv::Point3f p;
						p.x = w_pts_i(0);
						p.y = w_pts_i(1);
						p.z = w_pts_i(2);
						p = util3d::transformPoint(p, fixT);
						info->localMap.insert(std::make_pair(it_per_id.feature_id, p));

						if(this->imagesAlreadyRectified())
						{
							cv::Point2f pt;
							data.stereoCameraModels()[0].left().reproject(pts_i(0), pts_i(1), pts_i(2), pt.x, pt.y);
							info->reg.inliersIDs.push_back(info->newCorners.size());
							info->newCorners.push_back(pt);
						}
					}
					info->features = info->newCorners.size();
					info->localMapSize = info->localMap.size();
				}
				UINFO("Odom update time = %fs p=%s", timer.elapsed(), p.prettyPrint().c_str());
			}
		}
		else
		{
			UWARN("VINS not yet initialized... waiting to get enough IMU messages");
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
	else
	{
		UERROR("VINS-Fusion requires stereo images (and only one stereo camera with valid calibration)!");
	}
#else
	UERROR("RTAB-Map is not built with VINS support! Select another visual odometry approach.");
#endif
	return t;
}

} // namespace rtabmap
