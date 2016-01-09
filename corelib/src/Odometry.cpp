/*
Copyright (c) 2010-2014, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
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

#include "rtabmap/core/Odometry.h"
#include "rtabmap/core/OdometryF2F.h"
#include "rtabmap/core/OdometryLocalMap.h"
#include "rtabmap/core/OdometryInfo.h"
#include "rtabmap/utilite/ULogger.h"
#include "rtabmap/utilite/UTimer.h"
#include "rtabmap/utilite/UConversion.h"
#include "rtabmap/core/ParticleFilter.h"

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
		odometry = new OdometryLocalMap(parameters);
		type = Odometry::kTypeLocalMap;
		break;
	}
	return odometry;
}

Odometry::Odometry(const rtabmap::ParametersMap & parameters) :
		_resetCountdown(Parameters::defaultOdomResetCountdown()),
		_force3DoF(Parameters::defaultRegForce3DoF()),
		_holonomic(Parameters::defaultOdomHolonomic()),
		_filteringStrategy(Parameters::defaultOdomFilteringStrategy()),
		_particleSize(Parameters::defaultOdomParticleSize()),
		_particleNoiseT(Parameters::defaultOdomParticleNoiseT()),
		_particleLambdaT(Parameters::defaultOdomParticleLambdaT()),
		_particleNoiseR(Parameters::defaultOdomParticleNoiseR()),
		_particleLambdaR(Parameters::defaultOdomParticleLambdaR()),
		_fillInfoData(Parameters::defaultOdomFillInfoData()),
		_kalmanProcessNoise(Parameters::defaultOdomKalmanProcessNoise()),
		_kalmanMeasurementNoise(Parameters::defaultOdomKalmanMeasurementNoise()),
		_resetCurrentCount(0),
		previousStamp_(0),
		previousTransform_(Transform::getIdentity()),
		distanceTravelled_(0)
{
	Parameters::parse(parameters, Parameters::kOdomResetCountdown(), _resetCountdown);

	Parameters::parse(parameters, Parameters::kRegForce3DoF(), _force3DoF);
	Parameters::parse(parameters, Parameters::kOdomHolonomic(), _holonomic);
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
	if(_filteringStrategy == 2)
	{
		// Initialize the Particle filters
		filters_.resize(6);
		for(unsigned int i = 0; i<filters_.size(); ++i)
		{
			if(i<3)
			{
				filters_[i] = new ParticleFilter(_particleSize, _particleNoiseT, _particleLambdaT);
			}
			else
			{
				filters_[i] = new ParticleFilter(_particleSize, _particleNoiseR, _particleLambdaR);
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
	for(unsigned int i=0; i<filters_.size(); ++i)
	{
		delete filters_[i];
	}
	filters_.clear();
}

void Odometry::reset(const Transform & initialPose)
{
	previousTransform_.setIdentity();
	previousGroundTruthPose_.setNull();
	_resetCurrentCount = 0;
	previousStamp_ = 0;
	distanceTravelled_ = 0;
	if(_force3DoF || filters_.size())
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

		if(filters_.size())
		{
			UASSERT(filters_.size() == 6);
			filters_[0]->init(x);
			filters_[1]->init(y);
			filters_[2]->init(z);
			filters_[3]->init(roll);
			filters_[4]->init(pitch);
			filters_[5]->init(yaw);
		}

		if(_filteringStrategy == 1)
		{
			if(_force3DoF)
			{
				kalmanFilter_.statePost.at<float>(0) = x;
				kalmanFilter_.statePost.at<float>(1) = y;
				kalmanFilter_.statePost.at<float>(6) = yaw;
			}
			else
			{
				kalmanFilter_.statePost.at<float>(0) = x;
				kalmanFilter_.statePost.at<float>(1) = y;
				kalmanFilter_.statePost.at<float>(2) = z;
				kalmanFilter_.statePost.at<float>(9) = roll;
				kalmanFilter_.statePost.at<float>(10) = pitch;
				kalmanFilter_.statePost.at<float>(11) = yaw;
			}
		}
	}
	else
	{
		_pose = initialPose;
	}
}

Transform Odometry::process(const SensorData & data, OdometryInfo * info)
{
	if(_pose.isNull())
	{
		_pose.setIdentity(); // initialized
	}

	UASSERT(!data.imageRaw().empty());

	if(!data.stereoCameraModel().isValid() &&
	   (data.cameraModels().size() == 0 || !data.cameraModels()[0].isValid()))
	{
		UERROR("Rectified images required! Calibrate your camera.");
		return Transform();
	}

	UTimer time;
	Transform t = this->computeTransform(data, info);

	double dt = data.stamp() - previousStamp_;
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

	previousTransform_.setIdentity();
	previousStamp_ = data.stamp();

	if(!t.isNull())
	{
		_resetCurrentCount = _resetCountdown;

		if(_force3DoF || !_holonomic || filters_.size() || _filteringStrategy==1)
		{
			float x,y,z, roll,pitch,yaw;
			t.getTranslationAndEulerAngles(x, y, z, roll, pitch, yaw);

			if(_filteringStrategy == 1)
			{
				if(_pose.isIdentity())
				{
					// reset Kalman
					initKalmanFilter();
				}
				else
				{
					// Kalman filtering
					updateKalmanFilter(dt,x,y,z,roll,pitch,yaw);
				}
			}
			else if(filters_.size())
			{
				// Particle filtering
				UASSERT(filters_.size()==6);
				if(_pose.isIdentity())
				{
					filters_[0]->init(x);
					filters_[1]->init(y);
					filters_[2]->init(z);
					filters_[3]->init(roll);
					filters_[4]->init(pitch);
					filters_[5]->init(yaw);
				}
				else
				{
					x = filters_[0]->filter(x);
					y = filters_[1]->filter(y);
					yaw = filters_[5]->filter(yaw);

					if(!_holonomic)
					{
						// arc trajectory around ICR
						float tmpY = yaw!=0.0f ? x / tan((CV_PI-yaw)/2.0f) : 0.0f;
						if(fabs(tmpY) < fabs(y) || (tmpY<=0 && y >=0) || (tmpY>=0 && y<=0))
						{
							y = tmpY;
						}
						else
						{
							yaw = (atan(x/y)*2.0f-CV_PI)*-1;
						}
					}

					if(!_force3DoF)
					{
						z = filters_[2]->filter(z);
						roll = filters_[3]->filter(roll);
						pitch = filters_[4]->filter(pitch);
					}
				}

				if(info)
				{
					info->timeParticleFiltering = time.ticks();
				}
			}
			else if(!_holonomic)
			{
				// arc trajectory around ICR
				float tmpY = yaw!=0.0f ? x / tan((CV_PI-yaw)/2.0f) : 0.0f;
				if(fabs(tmpY) < fabs(y) || (tmpY<=0 && y >=0) || (tmpY>=0 && y<=0))
				{
					y = tmpY;
				}
				else
				{
					yaw = (atan(x/y)*2.0f-CV_PI)*-1;
				}
			}
			UASSERT_MSG(uIsFinite(x) && uIsFinite(y) && uIsFinite(z) &&
					uIsFinite(roll) && uIsFinite(pitch) && uIsFinite(yaw),
					uFormat("x=%f y=%f z=%f roll=%f pitch=%f yaw=%f org T=%s",
							x, y, z, roll, pitch, yaw, t.prettyPrint().c_str()).c_str());
			t = Transform(x,y,_force3DoF?0:z, _force3DoF?0:roll,_force3DoF?0:pitch,yaw);

			info->transformFiltered = t;
		}

		previousTransform_ = t;
		if(info)
		{
			distanceTravelled_ += t.getNorm();
			info->distanceTravelled = distanceTravelled_;
		}

		return _pose *= t; // updated
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

	return Transform();
}

void Odometry::initKalmanFilter()
{
	UDEBUG("");
	// See OpenCV tutorial: http://docs.opencv.org/master/dc/d2c/tutorial_real_time_pose.html
	// See Kalman filter pose/orientation estimation theory: http://campar.in.tum.de/Chair/KalmanFilter

	// initialize the Kalman filter
	int nStates = 18;            // the number of states (x,y,z,x',y',z',x'',y'',z'',roll,pitch,yaw,roll',pitch',yaw',roll'',pitch'',yaw'')
	int nMeasurements = 6;       // the number of measured states (x,y,z,roll,pitch,yaw)
	if(_force3DoF)
	{
		nStates = 9;             // the number of states (x,y,x',y',x'',y'',yaw,yaw',yaw'')
		nMeasurements = 3;       // the number of measured states (x,y,z,roll,pitch,yaw)
	}
	int nInputs = 0;             // the number of action control

	kalmanFilter_.init(nStates, nMeasurements, nInputs);                 // init Kalman Filter
	cv::setIdentity(kalmanFilter_.processNoiseCov, cv::Scalar::all(_kalmanProcessNoise));       // set process noise
	cv::setIdentity(kalmanFilter_.measurementNoiseCov, cv::Scalar::all(_kalmanMeasurementNoise));   // set measurement noise
	cv::setIdentity(kalmanFilter_.errorCovPost, cv::Scalar::all(1));             // error covariance

	if(_force3DoF)
	{
        /* MEASUREMENT MODEL */
		//  [1 0 0 0 0 0 0 0 0]
		//  [0 1 0 0 0 0 0 0 0]
		//  [0 0 0 0 0 0 1 0 0]
		kalmanFilter_.measurementMatrix.at<float>(0,0) = 1;  // x
		kalmanFilter_.measurementMatrix.at<float>(1,1) = 1;  // y
		kalmanFilter_.measurementMatrix.at<float>(2,6) = 1; // yaw
	}
	else
	{
	    /* MEASUREMENT MODEL */
		//  [1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
		//  [0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
		//  [0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
		//  [0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0]
		//  [0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0]
		//  [0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0]
		kalmanFilter_.measurementMatrix.at<float>(0,0) = 1;  // x
		kalmanFilter_.measurementMatrix.at<float>(1,1) = 1;  // y
		kalmanFilter_.measurementMatrix.at<float>(2,2) = 1;  // z
		kalmanFilter_.measurementMatrix.at<float>(3,9) = 1;  // roll
		kalmanFilter_.measurementMatrix.at<float>(4,10) = 1; // pitch
		kalmanFilter_.measurementMatrix.at<float>(5,11) = 1; // yaw
	}
}

void Odometry::updateKalmanFilter(float dt, float & x, float & y, float & z, float & roll, float & pitch, float & yaw)
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

	// Set measurement to predict
	cv::Mat measurements;
	if(!_force3DoF)
	{
		measurements = cv::Mat(6,1,CV_32FC1);
		measurements.at<float>(0) = x;     // x
		measurements.at<float>(1) = y;     // y
		measurements.at<float>(2) = z;     // z
		measurements.at<float>(3) = roll;  // roll
		measurements.at<float>(4) = pitch; // pitch
		measurements.at<float>(5) = yaw;   // yaw
	}
	else
	{
		measurements = cv::Mat(3,1,CV_32FC1);
		measurements.at<float>(0) = x;     // x
		measurements.at<float>(1) = y;     // y
		measurements.at<float>(5) = yaw;   // yaw
	}

	// First predict, to update the internal statePre variable
	UDEBUG("Predict");
	cv::Mat prediction = kalmanFilter_.predict();
	// The "correct" phase that is going to use the predicted value and our measurement
	UDEBUG("Correct");
	cv::Mat estimated = kalmanFilter_.correct(measurements);

	if(_force3DoF)
	{
		x = estimated.at<float>(0);
		y = estimated.at<float>(1);
		yaw = estimated.at<float>(6);
	}
	else
	{
		x = estimated.at<float>(0);
		y = estimated.at<float>(1);
		z = estimated.at<float>(2);
		roll = estimated.at<float>(9);
		pitch = estimated.at<float>(10);
		yaw = estimated.at<float>(11);
	}
}

} /* namespace rtabmap */
