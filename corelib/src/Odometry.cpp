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
#include "rtabmap/core/OdometryInfo.h"
#include "rtabmap/utilite/ULogger.h"
#include "rtabmap/utilite/UTimer.h"
#include "rtabmap/utilite/UConversion.h"
#include "ParticleFilter.h"

namespace rtabmap {

Odometry::Odometry(const rtabmap::ParametersMap & parameters) :
		_roiRatios(Parameters::defaultOdomRoiRatios()),
		_minInliers(Parameters::defaultOdomMinInliers()),
		_inlierDistance(Parameters::defaultOdomInlierDistance()),
		_iterations(Parameters::defaultOdomIterations()),
		_refineIterations(Parameters::defaultOdomRefineIterations()),
		_maxDepth(Parameters::defaultOdomMaxDepth()),
		_resetCountdown(Parameters::defaultOdomResetCountdown()),
		_force2D(Parameters::defaultOdomForce2D()),
		_holonomic(Parameters::defaultOdomHolonomic()),
		_particleFiltering(Parameters::defaultOdomParticleFiltering()),
		_particleSize(Parameters::defaultOdomParticleSize()),
		_particleNoiseT(Parameters::defaultOdomParticleNoiseT()),
		_particleLambdaT(Parameters::defaultOdomParticleLambdaT()),
		_particleNoiseR(Parameters::defaultOdomParticleNoiseR()),
		_particleLambdaR(Parameters::defaultOdomParticleLambdaR()),
		_fillInfoData(Parameters::defaultOdomFillInfoData()),
		_estimationType(Parameters::defaultOdomEstimationType()),
		_pnpReprojError(Parameters::defaultOdomPnPReprojError()),
		_pnpFlags(Parameters::defaultOdomPnPFlags()),
		_resetCurrentCount(0),
		previousStamp_(0),
		previousTransform_(Transform::getIdentity()),
		distanceTravelled_(0)
{
	Parameters::parse(parameters, Parameters::kOdomResetCountdown(), _resetCountdown);
	Parameters::parse(parameters, Parameters::kOdomMinInliers(), _minInliers);
	Parameters::parse(parameters, Parameters::kOdomInlierDistance(), _inlierDistance);
	Parameters::parse(parameters, Parameters::kOdomIterations(), _iterations);
	Parameters::parse(parameters, Parameters::kOdomRefineIterations(), _refineIterations);
	Parameters::parse(parameters, Parameters::kOdomMaxDepth(), _maxDepth);
	Parameters::parse(parameters, Parameters::kOdomRoiRatios(), _roiRatios);
	Parameters::parse(parameters, Parameters::kOdomForce2D(), _force2D);
	Parameters::parse(parameters, Parameters::kOdomHolonomic(), _holonomic);
	Parameters::parse(parameters, Parameters::kOdomFillInfoData(), _fillInfoData);
	Parameters::parse(parameters, Parameters::kOdomEstimationType(), _estimationType);
	Parameters::parse(parameters, Parameters::kOdomPnPReprojError(), _pnpReprojError);
	Parameters::parse(parameters, Parameters::kOdomPnPFlags(), _pnpFlags);
	UASSERT(_pnpFlags>=0 && _pnpFlags <=2);
	Parameters::parse(parameters, Parameters::kOdomParticleFiltering(), _particleFiltering);
	Parameters::parse(parameters, Parameters::kOdomParticleSize(), _particleSize);
	Parameters::parse(parameters, Parameters::kOdomParticleNoiseT(), _particleNoiseT);
	Parameters::parse(parameters, Parameters::kOdomParticleLambdaT(), _particleLambdaT);
	Parameters::parse(parameters, Parameters::kOdomParticleNoiseR(), _particleNoiseR);
	Parameters::parse(parameters, Parameters::kOdomParticleLambdaR(), _particleLambdaR);
	UASSERT(_particleNoiseT>0);
	UASSERT(_particleLambdaT>0);
	UASSERT(_particleNoiseR>0);
	UASSERT(_particleLambdaR>0);
	if(_particleFiltering)
	{
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
	_resetCurrentCount = 0;
	previousStamp_ = 0;
	distanceTravelled_ = 0;
	if(_force2D || filters_.size())
	{
		float x,y,z, roll,pitch,yaw;
		initialPose.getTranslationAndEulerAngles(x, y, z, roll, pitch, yaw);

		if(_force2D)
		{
			if(z != 0.0f || roll != 0.0f || yaw != 0.0f)
			{
				UWARN("Force2D=true and the initial pose contains z, roll or pitch values (%s). They are set to null.", initialPose.prettyPrint().c_str());
			}
			z = 0;
			roll = 0;
			yaw = 0;
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

	if(info)
	{
		info->timeEstimation = time.ticks();
		info->lost = t.isNull();
		info->stamp = data.stamp();
		info->interval = data.stamp() - previousStamp_;
		info->transform = t;
	}

	previousTransform_.setIdentity();
	previousStamp_ = data.stamp();

	if(!t.isNull())
	{
		_resetCurrentCount = _resetCountdown;

		if(_force2D || !_holonomic || filters_.size())
		{
			float x,y,z, roll,pitch,yaw;
			t.getTranslationAndEulerAngles(x, y, z, roll, pitch, yaw);

			if(filters_.size())
			{
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

					if(!_force2D)
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
			t = Transform(x,y,_force2D?0:z, _force2D?0:roll,_force2D?0:pitch,yaw);

			if(info && filters_.size())
			{
				info->transformFiltered = t;
			}
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

} /* namespace rtabmap */
