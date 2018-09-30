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

#ifndef ODOMETRY_H_
#define ODOMETRY_H_

#include <rtabmap/core/RtabmapExp.h>

#include <rtabmap/core/Transform.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/Parameters.h>

namespace rtabmap {

class OdometryInfo;
class ParticleFilter;

class RTABMAP_EXP Odometry
{
public:
	enum Type {
		kTypeUndef = -1,
		kTypeF2M = 0,
		kTypeF2F = 1,
		kTypeFovis = 2,
		kTypeViso2 = 3,
		kTypeDVO = 4,
		kTypeORBSLAM2 = 5,
		kTypeOkvis = 6,
		kTypeLOAM = 7,
		kTypeMSCKF = 8
	};

public:
	static Odometry * create(const ParametersMap & parameters = ParametersMap());
	static Odometry * create(Type & type, const ParametersMap & parameters = ParametersMap());

public:
	virtual ~Odometry();
	Transform process(SensorData & data, OdometryInfo * info = 0);
	Transform process(SensorData & data, const Transform & guess, OdometryInfo * info = 0);
	virtual void reset(const Transform & initialPose = Transform::getIdentity());
	virtual Odometry::Type getType() = 0;
	virtual bool canProcessRawImages() const {return false;}

	//getters
	const Transform & getPose() const {return _pose;}
	bool isInfoDataFilled() const {return _fillInfoData;}
	const Transform & previousVelocityTransform() const {return previousVelocityTransform_;}
	double previousStamp() const {return previousStamp_;}
	unsigned int framesProcessed() const {return framesProcessed_;}
	bool imagesAlreadyRectified() const {return _imagesAlreadyRectified;}

private:
	virtual Transform computeTransform(SensorData & data, const Transform & guess = Transform(), OdometryInfo * info = 0) = 0;

	void initKalmanFilter(const Transform & initialPose = Transform::getIdentity(), float vx=0.0f, float vy=0.0f, float vz=0.0f, float vroll=0.0f, float vpitch=0.0f, float vyaw=0.0f);
	void predictKalmanFilter(float dt, float * vx=0, float * vy=0, float * vz=0, float * vroll=0, float * vpitch=0, float * vyaw=0);
	void updateKalmanFilter(float & vx, float & vy, float & vz, float & vroll, float & vpitch, float & vyaw);

private:
	int _resetCountdown;
	bool _force3DoF;
	bool _holonomic;
	bool guessFromMotion_;
	int _filteringStrategy;
	int _particleSize;
	float _particleNoiseT;
	float _particleLambdaT;
	float _particleNoiseR;
	float _particleLambdaR;
	bool _fillInfoData;
	float _kalmanProcessNoise;
	float _kalmanMeasurementNoise;
	int _imageDecimation;
	bool _alignWithGround;
	bool _publishRAMUsage;
	bool _imagesAlreadyRectified;
	Transform _pose;
	int _resetCurrentCount;
	double previousStamp_;
	Transform previousVelocityTransform_;
	Transform previousGroundTruthPose_;
	float distanceTravelled_;
	unsigned int framesProcessed_;

	std::vector<ParticleFilter *> particleFilters_;
	cv::KalmanFilter kalmanFilter_;

protected:
	Odometry(const rtabmap::ParametersMap & parameters);
};

} /* namespace rtabmap */
#endif /* ODOMETRY_H_ */
