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

#include <rtabmap/core/OdometryF2M.h>
#include "rtabmap/core/Odometry.h"
#include "rtabmap/core/OdometryF2F.h"
#include "rtabmap/core/OdometryInfo.h"
#include "rtabmap/core/util3d.h"
#include "rtabmap/core/util3d_mapping.h"
#include "rtabmap/core/util3d_filtering.h"
#include "rtabmap/utilite/ULogger.h"
#include "rtabmap/utilite/UTimer.h"
#include "rtabmap/utilite/UConversion.h"
#include "rtabmap/core/ParticleFilter.h"
#include "rtabmap/core/util2d.h"

#include <pcl/pcl_base.h>

namespace rtabmap {

Odometry * Odometry::create(const ParametersMap & parameters)
{
	int odomTypeInt = Parameters::defaultOdomStrategy();
	Parameters::parse(parameters, Parameters::kOdomStrategy(), odomTypeInt);
	Odometry::Type type = (Odometry::Type)odomTypeInt;
	return create(type, parameters);
}

Odometry * Odometry::create(Odometry::Type & type, const ParametersMap & parameters)
{
	UDEBUG("type=%d", (int)type);
	Odometry * odometry = 0;
	switch(type)
	{
	case Odometry::kTypeF2F:
		odometry = new OdometryF2F(parameters);
		break;
	default:
		odometry = new OdometryF2M(parameters);
		type = Odometry::kTypeF2M;
		break;
	}
	return odometry;
}

Odometry::Odometry(const rtabmap::ParametersMap & parameters) :
		_resetCountdown(Parameters::defaultOdomResetCountdown()),
		_force3DoF(Parameters::defaultRegForce3DoF()),
		_holonomic(Parameters::defaultOdomHolonomic()),
		guessFromMotion_(Parameters::defaultOdomGuessMotion()),
		_filteringStrategy(Parameters::defaultOdomFilteringStrategy()),
		_particleSize(Parameters::defaultOdomParticleSize()),
		_particleNoiseT(Parameters::defaultOdomParticleNoiseT()),
		_particleLambdaT(Parameters::defaultOdomParticleLambdaT()),
		_particleNoiseR(Parameters::defaultOdomParticleNoiseR()),
		_particleLambdaR(Parameters::defaultOdomParticleLambdaR()),
		_fillInfoData(Parameters::defaultOdomFillInfoData()),
		_kalmanProcessNoise(Parameters::defaultOdomKalmanProcessNoise()),
		_kalmanMeasurementNoise(Parameters::defaultOdomKalmanMeasurementNoise()),
		_imageDecimation(Parameters::defaultOdomImageDecimation()),
		_alignWithGround(Parameters::defaultOdomAlignWithGround()),
		_pose(Transform::getIdentity()),
		_resetCurrentCount(0),
		previousStamp_(0),
		distanceTravelled_(0)
{
	Parameters::parse(parameters, Parameters::kOdomResetCountdown(), _resetCountdown);

	Parameters::parse(parameters, Parameters::kRegForce3DoF(), _force3DoF);
	Parameters::parse(parameters, Parameters::kOdomHolonomic(), _holonomic);
	Parameters::parse(parameters, Parameters::kOdomGuessMotion(), guessFromMotion_);
	Parameters::parse(parameters, Parameters::kOdomFillInfoData(), _fillInfoData);
	Parameters::parse(parameters, Parameters::kOdomFilteringStrategy(), _filteringStrategy);
	Parameters::parse(parameters, Parameters::kOdomParticleSize(), _particleSize);
	Parameters::parse(parameters, Parameters::kOdomParticleNoiseT(), _particleNoiseT);
	Parameters::parse(parameters, Parameters::kOdomParticleLambdaT(), _particleLambdaT);
	Parameters::parse(parameters, Parameters::kOdomParticleNoiseR(), _particleNoiseR);
	Parameters::parse(parameters, Parameters::kOdomParticleLambdaR(), _particleLambdaR);
	UASSERT(_particleNoiseT>0);
	UASSERT(_particleLambdaT>0);
	UASSERT(_particleNoiseR>0);
	UASSERT(_particleLambdaR>0);
	Parameters::parse(parameters, Parameters::kOdomKalmanProcessNoise(), _kalmanProcessNoise);
	Parameters::parse(parameters, Parameters::kOdomKalmanMeasurementNoise(), _kalmanMeasurementNoise);
	Parameters::parse(parameters, Parameters::kOdomImageDecimation(), _imageDecimation);
	Parameters::parse(parameters, Parameters::kOdomAlignWithGround(), _alignWithGround);
	if(_imageDecimation == 0)
	{
		_imageDecimation = 1;
	}

	if(_filteringStrategy == 2)
	{
		// Initialize the Particle filters
		particleFilters_.resize(6);
		for(unsigned int i = 0; i<particleFilters_.size(); ++i)
		{
			if(i<3)
			{
				particleFilters_[i] = new ParticleFilter(_particleSize, _particleNoiseT, _particleLambdaT);
			}
			else
			{
				particleFilters_[i] = new ParticleFilter(_particleSize, _particleNoiseR, _particleLambdaR);
			}
		}
	}
	else if(_filteringStrategy == 1)
	{
		initKalmanFilter();
	}
}

Odometry::~Odometry()
{
	for(unsigned int i=0; i<particleFilters_.size(); ++i)
	{
		delete particleFilters_[i];
	}
	particleFilters_.clear();
}

void Odometry::reset(const Transform & initialPose)
{
	UASSERT(!initialPose.isNull());
	previousVelocityTransform_.setNull();
	previousGroundTruthPose_.setNull();
	_resetCurrentCount = 0;
	previousStamp_ = 0;
	distanceTravelled_ = 0;
	if(_force3DoF || particleFilters_.size())
	{
		float x,y,z, roll,pitch,yaw;
		initialPose.getTranslationAndEulerAngles(x, y, z, roll, pitch, yaw);

		if(_force3DoF)
		{
			if(z != 0.0f || roll != 0.0f || pitch != 0.0f)
			{
				UWARN("Force2D=true and the initial pose contains z, roll or pitch values (%s). They are set to null.", initialPose.prettyPrint().c_str());
			}
			z = 0;
			roll = 0;
			pitch = 0;
			Transform pose(x, y, z, roll, pitch, yaw);
			_pose = pose;
		}
		else
		{
			_pose = initialPose;
		}

		if(particleFilters_.size())
		{
			UASSERT(particleFilters_.size() == 6);
			particleFilters_[0]->init(x);
			particleFilters_[1]->init(y);
			particleFilters_[2]->init(z);
			particleFilters_[3]->init(roll);
			particleFilters_[4]->init(pitch);
			particleFilters_[5]->init(yaw);
		}

		if(_filteringStrategy == 1)
		{
			initKalmanFilter(initialPose);
		}
	}
	else
	{
		_pose = initialPose;
	}
}

Transform Odometry::process(SensorData & data, OdometryInfo * info)
{
	return process(data, Transform(), info);
}

Transform Odometry::process(SensorData & data, const Transform & guessIn, OdometryInfo * info)
{
	UASSERT_MSG(data.id() >= 0, uFormat("Input data should have ID greater or equal than 0 (id=%d)!", data.id()).c_str());

	// Ground alignment
	if(_pose.isIdentity() && _alignWithGround)
	{
		if(data.depthOrRightRaw().empty())
		{
			UWARN("\"%s\" is true but the input has no depth information, ignoring alignment with ground...", Parameters::kOdomAlignWithGround().c_str());
		}
		else
		{
			UTimer alignTimer;
			pcl::IndicesPtr indices(new std::vector<int>);
			pcl::IndicesPtr ground, obstacles;
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = util3d::cloudFromSensorData(data, 1, 0, 0, indices.get());
			cloud = util3d::voxelize(cloud, indices, 0.01);
			bool success = false;
			if(cloud->size())
			{
				util3d::segmentObstaclesFromGround<pcl::PointXYZ>(cloud, ground, obstacles, 20, M_PI/4.0f, 0.02, 200, true);
				if(ground->size())
				{
					pcl::ModelCoefficients coefficients;
					util3d::extractPlane(cloud, ground, 0.02, 100, &coefficients);
					if(coefficients.values.at(3) >= 0)
					{
						UWARN("Ground detected! coefficients=(%f, %f, %f, %f) time=%fs",
								coefficients.values.at(0),
								coefficients.values.at(1),
								coefficients.values.at(2),
								coefficients.values.at(3),
								alignTimer.ticks());
					}
					else
					{
						UWARN("Ceiling detected! coefficients=(%f, %f, %f, %f) time=%fs",
								coefficients.values.at(0),
								coefficients.values.at(1),
								coefficients.values.at(2),
								coefficients.values.at(3),
								alignTimer.ticks());
					}
					Eigen::Vector3f n(coefficients.values.at(0), coefficients.values.at(1), coefficients.values.at(2));
					Eigen::Vector3f z(0,0,1);
					//get rotation from z to n;
					Eigen::Matrix3f R;
					R = Eigen::Quaternionf().setFromTwoVectors(n,z);
					Transform rotation(
							R(0,0), R(0,1), R(0,2), 0,
							R(1,0), R(1,1), R(1,2), 0,
							R(2,0), R(2,1), R(2,2), coefficients.values.at(3));
					_pose *= rotation;
					success = true;
				}
			}
			if(!success)
			{
				UERROR("Odometry failed to detect the ground. You have this "
						"error because parameter \"%s\" is true. "
						"Make sure the camera is seeing the ground (e.g., tilt ~30 "
						"degrees toward the ground).", Parameters::kOdomAlignWithGround().c_str());
			}
		}
	}

	double dt = previousStamp_>0.0f?data.stamp() - previousStamp_:0.0;
	Transform guess = dt && guessFromMotion_ && !previousVelocityTransform_.isNull()?Transform::getIdentity():Transform();
	UASSERT_MSG(dt>0.0 || (dt == 0.0 && previousVelocityTransform_.isNull()), uFormat("dt=%f previous transform=%s", dt, previousVelocityTransform_.prettyPrint().c_str()).c_str());
	if(!previousVelocityTransform_.isNull())
	{
		if(guessFromMotion_)
		{
			if(_filteringStrategy == 1)
			{
				// use Kalman predict transform
				float vx,vy,vz, vroll,vpitch,vyaw;
				predictKalmanFilter(dt, &vx,&vy,&vz,&vroll,&vpitch,&vyaw);
				guess = Transform(vx*dt, vy*dt, vz*dt, vroll*dt, vpitch*dt, vyaw*dt);
			}
			else
			{
				float vx,vy,vz, vroll,vpitch,vyaw;
				previousVelocityTransform_.getTranslationAndEulerAngles(vx,vy,vz, vroll,vpitch,vyaw);
				guess = Transform(vx*dt, vy*dt, vz*dt, vroll*dt, vpitch*dt, vyaw*dt);
			}
		}
		else if(_filteringStrategy == 1)
		{
			predictKalmanFilter(dt);
		}
	}

	if(!guessIn.isNull())
	{
		guess = guessIn;
	}

	UTimer time;
	Transform t;
	if(_imageDecimation > 1)
	{
		// Decimation of images with calibrations
		SensorData decimatedData = data;
		decimatedData.setImageRaw(util2d::decimate(decimatedData.imageRaw(), _imageDecimation));
		decimatedData.setDepthOrRightRaw(util2d::decimate(decimatedData.depthOrRightRaw(), _imageDecimation));
		std::vector<CameraModel> cameraModels = decimatedData.cameraModels();
		for(unsigned int i=0; i<cameraModels.size(); ++i)
		{
			cameraModels[i] = cameraModels[i].scaled(1.0/double(_imageDecimation));
		}
		decimatedData.setCameraModels(cameraModels);
		StereoCameraModel stereoModel = decimatedData.stereoCameraModel();
		if(stereoModel.isValidForProjection())
		{
			stereoModel.scale(1.0/double(_imageDecimation));
		}
		decimatedData.setStereoCameraModel(stereoModel);

		// compute transform
		t = this->computeTransform(decimatedData, guess, info);

		// transform back the keypoints in the original image
		std::vector<cv::KeyPoint> kpts = decimatedData.keypoints();
		double log2value = log(double(_imageDecimation))/log(2.0);
		for(unsigned int i=0; i<kpts.size(); ++i)
		{
			kpts[i].pt.x *= _imageDecimation;
			kpts[i].pt.y *= _imageDecimation;
			kpts[i].size *= _imageDecimation;
			kpts[i].octave += log2value;
		}
		data.setFeatures(kpts, decimatedData.keypoints3D(), decimatedData.descriptors());

		if(info)
		{
			UASSERT(info->newCorners.size() == info->refCorners.size());
			for(unsigned int i=0; i<info->newCorners.size(); ++i)
			{
				info->refCorners[i].x *= _imageDecimation;
				info->refCorners[i].y *= _imageDecimation;
				info->newCorners[i].x *= _imageDecimation;
				info->newCorners[i].y *= _imageDecimation;
			}
			for(std::multimap<int, cv::KeyPoint>::iterator iter=info->words.begin(); iter!=info->words.end(); ++iter)
			{
				iter->second.pt.x *= _imageDecimation;
				iter->second.pt.y *= _imageDecimation;
				iter->second.size *= _imageDecimation;
				iter->second.octave += log2value;
			}
		}
	}
	else
	{
		t = this->computeTransform(data, guess, info);
	}

	if(info)
	{
		info->timeEstimation = time.ticks();
		info->lost = t.isNull();
		info->stamp = data.stamp();
		info->interval = dt;
		info->transform = t;

		if(!data.groundTruth().isNull())
		{
			if(!previousGroundTruthPose_.isNull())
			{
				info->transformGroundTruth = previousGroundTruthPose_.inverse() * data.groundTruth();
			}
			previousGroundTruthPose_ = data.groundTruth();
		}
	}

	if(!t.isNull())
	{
		_resetCurrentCount = _resetCountdown;

		float vx,vy,vz, vroll,vpitch,vyaw;
		t.getTranslationAndEulerAngles(vx,vy,vz, vroll,vpitch,vyaw);

		// transform to velocity
		if(dt)
		{
			vx /= dt;
			vy /= dt;
			vz /= dt;
			vroll /= dt;
			vpitch /= dt;
			vyaw /= dt;
		}

		if(_force3DoF || !_holonomic || particleFilters_.size() || _filteringStrategy==1)
		{
			if(_filteringStrategy == 1)
			{
				if(previousVelocityTransform_.isNull())
				{
					// reset Kalman
					if(dt)
					{
						initKalmanFilter(t, vx,vy,vz,vroll,vpitch,vyaw);
					}
					else
					{
						initKalmanFilter(t);
					}
				}
				else
				{
					// Kalman filtering
					updateKalmanFilter(vx,vy,vz,vroll,vpitch,vyaw);
				}
			}
			else if(particleFilters_.size())
			{
				// Particle filtering
				UASSERT(particleFilters_.size()==6);
				if(previousVelocityTransform_.isNull())
				{
					particleFilters_[0]->init(vx);
					particleFilters_[1]->init(vy);
					particleFilters_[2]->init(vz);
					particleFilters_[3]->init(vroll);
					particleFilters_[4]->init(vpitch);
					particleFilters_[5]->init(vyaw);
				}
				else
				{
					vx = particleFilters_[0]->filter(vx);
					vy = particleFilters_[1]->filter(vy);
					vyaw = particleFilters_[5]->filter(vyaw);

					if(!_holonomic)
					{
						// arc trajectory around ICR
						float tmpY = vyaw!=0.0f ? vx / tan((CV_PI-vyaw)/2.0f) : 0.0f;
						if(fabs(tmpY) < fabs(vy) || (tmpY<=0 && vy >=0) || (tmpY>=0 && vy<=0))
						{
							vy = tmpY;
						}
						else
						{
							vyaw = (atan(vx/vy)*2.0f-CV_PI)*-1;
						}
					}

					if(!_force3DoF)
					{
						vz = particleFilters_[2]->filter(vz);
						vroll = particleFilters_[3]->filter(vroll);
						vpitch = particleFilters_[4]->filter(vpitch);
					}
				}

				if(info)
				{
					info->timeParticleFiltering = time.ticks();
				}

				if(_force3DoF)
				{
					vz = 0.0f;
					vroll = 0.0f;
					vpitch = 0.0f;
				}
			}
			else if(!_holonomic)
			{
				// arc trajectory around ICR
				vy = vyaw!=0.0f ? vx / tan((CV_PI-vyaw)/2.0f) : 0.0f;
				if(_force3DoF)
				{
					vz = 0.0f;
					vroll = 0.0f;
					vpitch = 0.0f;
				}
			}

			if(dt)
			{
				t = Transform(vx*dt, vy*dt, vz*dt, vroll*dt, vpitch*dt, vyaw*dt);
			}
			else
			{
				t = Transform(vx, vy, vz, vroll, vpitch, vyaw);
			}

			if(info)
			{
				info->transformFiltered = t;
			}
		}

		if(data.stamp() == 0)
		{
			UWARN("Null stamp detected");
		}

		previousStamp_ = data.stamp();
		previousVelocityTransform_.setNull();

		if(dt)
		{
			previousVelocityTransform_ = Transform(vx, vy, vz, vroll, vpitch, vyaw);
		}

		if(info)
		{
			distanceTravelled_ += t.getNorm();
			info->distanceTravelled = distanceTravelled_;
		}

		info->varianceLin *= t.getNorm();
		info->varianceAng *= t.getAngle();
		info->varianceLin = info->varianceLin>0.0f?info->varianceLin:0.0001f; // epsilon if exact transform
		info->varianceAng = info->varianceAng>0.0f?info->varianceAng:0.0001f; // epsilon if exact transform

		return _pose *= t; // update
	}
	else if(_resetCurrentCount > 0)
	{
		UWARN("Odometry lost! Odometry will be reset after next %d consecutive unsuccessful odometry updates...", _resetCurrentCount);

		--_resetCurrentCount;
		if(_resetCurrentCount == 0)
		{
			UWARN("Odometry automatically reset to latest pose!");
			this->reset(_pose);
		}
	}

	previousVelocityTransform_.setNull();
	previousStamp_ = 0;

	return Transform();
}

void Odometry::initKalmanFilter(const Transform & initialPose, float vx, float vy, float vz, float vroll, float vpitch, float vyaw)
{
	UDEBUG("");
	// See OpenCV tutorial: http://docs.opencv.org/master/dc/d2c/tutorial_real_time_pose.html
	// See Kalman filter pose/orientation estimation theory: http://campar.in.tum.de/Chair/KalmanFilter

	// initialize the Kalman filter
	int nStates = 18;            // the number of states (x,y,z,x',y',z',x'',y'',z'',roll,pitch,yaw,roll',pitch',yaw',roll'',pitch'',yaw'')
	int nMeasurements = 6;       // the number of measured states (x',y',z',roll',pitch',yaw')
	if(_force3DoF)
	{
		nStates = 9;             // the number of states (x,y,x',y',x'',y'',yaw,yaw',yaw'')
		nMeasurements = 3;       // the number of measured states (x',y',yaw')
	}
	int nInputs = 0;             // the number of action control

	/* From viso2, measurement covariance
	 * static const boost::array<double, 36> STANDARD_POSE_COVARIANCE =
	{ { 0.1, 0, 0, 0, 0, 0,
	    0, 0.1, 0, 0, 0, 0,
	    0, 0, 0.1, 0, 0, 0,
	    0, 0, 0, 0.17, 0, 0,
	    0, 0, 0, 0, 0.17, 0,
	    0, 0, 0, 0, 0, 0.17 } };
	static const boost::array<double, 36> STANDARD_TWIST_COVARIANCE =
	{ { 0.05, 0, 0, 0, 0, 0,
	    0, 0.05, 0, 0, 0, 0,
	    0, 0, 0.05, 0, 0, 0,
	    0, 0, 0, 0.09, 0, 0,
	    0, 0, 0, 0, 0.09, 0,
	    0, 0, 0, 0, 0, 0.09 } };
	 */


	kalmanFilter_.init(nStates, nMeasurements, nInputs);                 // init Kalman Filter
	cv::setIdentity(kalmanFilter_.processNoiseCov, cv::Scalar::all(_kalmanProcessNoise));  // set process noise
	cv::setIdentity(kalmanFilter_.measurementNoiseCov, cv::Scalar::all(_kalmanMeasurementNoise));   // set measurement noise
	cv::setIdentity(kalmanFilter_.errorCovPost, cv::Scalar::all(1));             // error covariance

	float x,y,z,roll,pitch,yaw;
	initialPose.getTranslationAndEulerAngles(x,y,z,roll,pitch,yaw);

	if(_force3DoF)
	{
        /* MEASUREMENT MODEL (velocity) */
		//  [0 0 1 0 0 0 0 0 0]
		//  [0 0 0 1 0 0 0 0 0]
		//  [0 0 0 0 0 0 0 1 0]
		kalmanFilter_.measurementMatrix.at<float>(0,2) = 1;  // x'
		kalmanFilter_.measurementMatrix.at<float>(1,3) = 1;  // y'
		kalmanFilter_.measurementMatrix.at<float>(2,7) = 1; // yaw'

		kalmanFilter_.statePost.at<float>(0) = x;
		kalmanFilter_.statePost.at<float>(1) = y;
		kalmanFilter_.statePost.at<float>(6) = yaw;

		kalmanFilter_.statePost.at<float>(2) = vx;
		kalmanFilter_.statePost.at<float>(3) = vy;
		kalmanFilter_.statePost.at<float>(7) = vyaw;
	}
	else
	{
	    /* MEASUREMENT MODEL (velocity) */
		//  [0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
		//  [0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0]
		//  [0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0]
		//  [0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0]
		//  [0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0]
		//  [0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0]
		kalmanFilter_.measurementMatrix.at<float>(0,3) = 1;  // x'
		kalmanFilter_.measurementMatrix.at<float>(1,4) = 1;  // y'
		kalmanFilter_.measurementMatrix.at<float>(2,5) = 1;  // z'
		kalmanFilter_.measurementMatrix.at<float>(3,12) = 1; // roll'
		kalmanFilter_.measurementMatrix.at<float>(4,13) = 1; // pitch'
		kalmanFilter_.measurementMatrix.at<float>(5,14) = 1; // yaw'

		kalmanFilter_.statePost.at<float>(0) = x;
		kalmanFilter_.statePost.at<float>(1) = y;
		kalmanFilter_.statePost.at<float>(2) = z;
		kalmanFilter_.statePost.at<float>(9) = roll;
		kalmanFilter_.statePost.at<float>(10) = pitch;
		kalmanFilter_.statePost.at<float>(11) = yaw;

		kalmanFilter_.statePost.at<float>(3) = vx;
		kalmanFilter_.statePost.at<float>(4) = vy;
		kalmanFilter_.statePost.at<float>(5) = vz;
		kalmanFilter_.statePost.at<float>(12) = vroll;
		kalmanFilter_.statePost.at<float>(13) = vpitch;
		kalmanFilter_.statePost.at<float>(14) = vyaw;
	}
}

void Odometry::predictKalmanFilter(float dt, float * vx, float * vy, float * vz, float * vroll, float * vpitch, float * vyaw)
{
	// Set transition matrix with current dt
	if(_force3DoF)
	{
		// 2D:
		//  [1 0 dt  0 dt2    0   0    0     0] x
		//  [0 1  0 dt   0  dt2   0    0     0] y
		//  [0 0  1  0   dt   0   0    0     0] x'
		//  [0 0  0  1   0   dt   0    0     0] y'
		//  [0 0  0  0   1    0   0    0     0] x''
		//  [0 0  0  0   0    0   0    0     0] y''
		//  [0 0  0  0   0    0   1   dt   dt2] yaw
		//  [0 0  0  0   0    0   0    1    dt] yaw'
		//  [0 0  0  0   0    0   0    0     1] yaw''
		kalmanFilter_.transitionMatrix.at<float>(0,2) = dt;
		kalmanFilter_.transitionMatrix.at<float>(1,3) = dt;
		kalmanFilter_.transitionMatrix.at<float>(2,4) = dt;
		kalmanFilter_.transitionMatrix.at<float>(3,5) = dt;
		kalmanFilter_.transitionMatrix.at<float>(0,4) = 0.5*pow(dt,2);
		kalmanFilter_.transitionMatrix.at<float>(1,5) = 0.5*pow(dt,2);
		// orientation
		kalmanFilter_.transitionMatrix.at<float>(6,7) = dt;
		kalmanFilter_.transitionMatrix.at<float>(7,8) = dt;
		kalmanFilter_.transitionMatrix.at<float>(6,8) = 0.5*pow(dt,2);
	}
	else
	{
		//  [1 0 0 dt  0  0 dt2   0   0 0 0 0  0  0  0   0   0   0] x
		//  [0 1 0  0 dt  0   0 dt2   0 0 0 0  0  0  0   0   0   0] y
		//  [0 0 1  0  0 dt   0   0 dt2 0 0 0  0  0  0   0   0   0] z
		//  [0 0 0  1  0  0  dt   0   0 0 0 0  0  0  0   0   0   0] x'
		//  [0 0 0  0  1  0   0  dt   0 0 0 0  0  0  0   0   0   0] y'
		//  [0 0 0  0  0  1   0   0  dt 0 0 0  0  0  0   0   0   0] z'
		//  [0 0 0  0  0  0   1   0   0 0 0 0  0  0  0   0   0   0] x''
		//  [0 0 0  0  0  0   0   1   0 0 0 0  0  0  0   0   0   0] y''
		//  [0 0 0  0  0  0   0   0   1 0 0 0  0  0  0   0   0   0] z''
		//  [0 0 0  0  0  0   0   0   0 1 0 0 dt  0  0 dt2   0   0]
		//  [0 0 0  0  0  0   0   0   0 0 1 0  0 dt  0   0 dt2   0]
		//  [0 0 0  0  0  0   0   0   0 0 0 1  0  0 dt   0   0 dt2]
		//  [0 0 0  0  0  0   0   0   0 0 0 0  1  0  0  dt   0   0]
		//  [0 0 0  0  0  0   0   0   0 0 0 0  0  1  0   0  dt   0]
		//  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  1   0   0  dt]
		//  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  0   1   0   0]
		//  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  0   0   1   0]
		//  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  0   0   0   1]
		// position
		kalmanFilter_.transitionMatrix.at<float>(0,3) = dt;
		kalmanFilter_.transitionMatrix.at<float>(1,4) = dt;
		kalmanFilter_.transitionMatrix.at<float>(2,5) = dt;
		kalmanFilter_.transitionMatrix.at<float>(3,6) = dt;
		kalmanFilter_.transitionMatrix.at<float>(4,7) = dt;
		kalmanFilter_.transitionMatrix.at<float>(5,8) = dt;
		kalmanFilter_.transitionMatrix.at<float>(0,6) = 0.5*pow(dt,2);
		kalmanFilter_.transitionMatrix.at<float>(1,7) = 0.5*pow(dt,2);
		kalmanFilter_.transitionMatrix.at<float>(2,8) = 0.5*pow(dt,2);
		// orientation
		kalmanFilter_.transitionMatrix.at<float>(9,12) = dt;
		kalmanFilter_.transitionMatrix.at<float>(10,13) = dt;
		kalmanFilter_.transitionMatrix.at<float>(11,14) = dt;
		kalmanFilter_.transitionMatrix.at<float>(12,15) = dt;
		kalmanFilter_.transitionMatrix.at<float>(13,16) = dt;
		kalmanFilter_.transitionMatrix.at<float>(14,17) = dt;
		kalmanFilter_.transitionMatrix.at<float>(9,15) = 0.5*pow(dt,2);
		kalmanFilter_.transitionMatrix.at<float>(10,16) = 0.5*pow(dt,2);
		kalmanFilter_.transitionMatrix.at<float>(11,17) = 0.5*pow(dt,2);
	}

	// First predict, to update the internal statePre variable
	UDEBUG("Predict");
	const cv::Mat & prediction = kalmanFilter_.predict();

	if(vx)
		*vx = prediction.at<float>(3);                      // x'
	if(vy)
		*vy = prediction.at<float>(4);                      // y'
	if(vz)
		*vz = _force3DoF?0.0f:prediction.at<float>(5);      // z'
	if(vroll)
		*vroll = _force3DoF?0.0f:prediction.at<float>(12);  // roll'
	if(vpitch)
		*vpitch = _force3DoF?0.0f:prediction.at<float>(13); // pitch'
	if(vyaw)
		*vyaw = prediction.at<float>(_force3DoF?7:14);      // yaw'
}

void Odometry::updateKalmanFilter(float & vx, float & vy, float & vz, float & vroll, float & vpitch, float & vyaw)
{
	// Set measurement to predict
	cv::Mat measurements;
	if(!_force3DoF)
	{
		measurements = cv::Mat(6,1,CV_32FC1);
		measurements.at<float>(0) = vx;     // x'
		measurements.at<float>(1) = vy;     // y'
		measurements.at<float>(2) = vz;     // z'
		measurements.at<float>(3) = vroll;  // roll'
		measurements.at<float>(4) = vpitch; // pitch'
		measurements.at<float>(5) = vyaw;   // yaw'
	}
	else
	{
		measurements = cv::Mat(3,1,CV_32FC1);
		measurements.at<float>(0) = vx;     // x'
		measurements.at<float>(1) = vy;     // y'
		measurements.at<float>(2) = vyaw;   // yaw',
	}

	// The "correct" phase that is going to use the predicted value and our measurement
	UDEBUG("Correct");
	const cv::Mat & estimated = kalmanFilter_.correct(measurements);


	vx = estimated.at<float>(3);                      // x'
	vy = estimated.at<float>(4);                      // y'
	vz = _force3DoF?0.0f:estimated.at<float>(5);      // z'
	vroll = _force3DoF?0.0f:estimated.at<float>(12);  // roll'
	vpitch = _force3DoF?0.0f:estimated.at<float>(13); // pitch'
	vyaw = estimated.at<float>(_force3DoF?7:14);      // yaw'
}

} /* namespace rtabmap */
