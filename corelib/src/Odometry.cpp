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

#include "rtabmap/core/Odometry.h"
#include <rtabmap/core/odometry/OdometryF2M.h>
#include "rtabmap/core/odometry/OdometryF2F.h"
#include "rtabmap/core/odometry/OdometryFovis.h"
#include "rtabmap/core/odometry/OdometryViso2.h"
#include "rtabmap/core/odometry/OdometryDVO.h"
#include "rtabmap/core/odometry/OdometryOkvis.h"
#include "rtabmap/core/odometry/OdometryORBSLAM.h"
#include "rtabmap/core/odometry/OdometryLOAM.h"
#include "rtabmap/core/odometry/OdometryFLOAM.h"
#include "rtabmap/core/odometry/OdometryMSCKF.h"
#include "rtabmap/core/odometry/OdometryVINS.h"
#include "rtabmap/core/odometry/OdometryOpenVINS.h"
#include "rtabmap/core/odometry/OdometryOpen3D.h"
#include "rtabmap/core/OdometryInfo.h"
#include "rtabmap/core/util3d.h"
#include "rtabmap/core/util3d_mapping.h"
#include "rtabmap/core/util3d_filtering.h"
#include "rtabmap/utilite/ULogger.h"
#include "rtabmap/utilite/UTimer.h"
#include "rtabmap/utilite/UConversion.h"
#include "rtabmap/utilite/UProcessInfo.h"
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
	case Odometry::kTypeF2M:
		odometry = new OdometryF2M(parameters);
		break;
	case Odometry::kTypeF2F:
		odometry = new OdometryF2F(parameters);
		break;
	case Odometry::kTypeFovis:
		odometry = new OdometryFovis(parameters);
		break;
	case Odometry::kTypeViso2:
		odometry = new OdometryViso2(parameters);
		break;
	case Odometry::kTypeDVO:
		odometry = new OdometryDVO(parameters);
		break;
	case Odometry::kTypeORBSLAM:
		odometry = new OdometryORBSLAM(parameters);
		break;
	case Odometry::kTypeOkvis:
		odometry = new OdometryOkvis(parameters);
		break;
	case Odometry::kTypeLOAM:
		odometry = new OdometryLOAM(parameters);
		break;
	case Odometry::kTypeFLOAM:
		odometry = new OdometryFLOAM(parameters);
		break;
	case Odometry::kTypeMSCKF:
		odometry = new OdometryMSCKF(parameters);
		break;
	case Odometry::kTypeVINS:
		odometry = new OdometryVINS(parameters);
		break;
	case Odometry::kTypeOpenVINS:
		odometry = new OdometryOpenVINS(parameters);
		break;
	case Odometry::kTypeOpen3D:
		odometry = new OdometryOpen3D(parameters);
		break;
	default:
		UERROR("Unknown odometry type %d, using F2M instead...", (int)type);
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
		guessSmoothingDelay_(Parameters::defaultOdomGuessSmoothingDelay()),
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
		_publishRAMUsage(Parameters::defaultRtabmapPublishRAMUsage()),
		_imagesAlreadyRectified(Parameters::defaultRtabmapImagesAlreadyRectified()),
		_pose(Transform::getIdentity()),
		_resetCurrentCount(0),
		previousStamp_(0),
		distanceTravelled_(0),
		framesProcessed_(0)
{
	Parameters::parse(parameters, Parameters::kOdomResetCountdown(), _resetCountdown);

	Parameters::parse(parameters, Parameters::kRegForce3DoF(), _force3DoF);
	Parameters::parse(parameters, Parameters::kOdomHolonomic(), _holonomic);
	Parameters::parse(parameters, Parameters::kOdomGuessMotion(), guessFromMotion_);
	Parameters::parse(parameters, Parameters::kOdomGuessSmoothingDelay(), guessSmoothingDelay_);
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
	Parameters::parse(parameters, Parameters::kRtabmapPublishRAMUsage(), _publishRAMUsage);
	Parameters::parse(parameters, Parameters::kRtabmapImagesAlreadyRectified(), _imagesAlreadyRectified);

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
}

void Odometry::reset(const Transform & initialPose)
{
	UDEBUG("");
	UASSERT(!initialPose.isNull());
	previousVelocities_.clear();
	velocityGuess_.setNull();
	previousGroundTruthPose_.setNull();
	_resetCurrentCount = 0;
	previousStamp_ = 0;
	distanceTravelled_ = 0;
	framesProcessed_ = 0;
	imuLastTransform_.setNull();
	imus_.clear();
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

const Transform & Odometry::previousVelocityTransform() const
{
	return getVelocityGuess();
}

Transform getMeanVelocity(const std::list<std::pair<std::vector<float>, double> > & transforms)
{
	if(transforms.size())
	{
		float tvx=0.0f,tvy=0.0f,tvz=0.0f, tvroll=0.0f,tvpitch=0.0f,tvyaw=0.0f;
		for(std::list<std::pair<std::vector<float>, double> >::const_iterator iter=transforms.begin(); iter!=transforms.end(); ++iter)
		{
			UASSERT(iter->first.size() == 6);
			tvx+=iter->first[0];
			tvy+=iter->first[1];
			tvz+=iter->first[2];
			tvroll+=iter->first[3];
			tvpitch+=iter->first[4];
			tvyaw+=iter->first[5];
		}
		tvx/=float(transforms.size());
		tvy/=float(transforms.size());
		tvz/=float(transforms.size());
		tvroll/=float(transforms.size());
		tvpitch/=float(transforms.size());
		tvyaw/=float(transforms.size());
		return Transform(tvx, tvy, tvz, tvroll, tvpitch, tvyaw);
	}
	return Transform();
}

Transform Odometry::process(SensorData & data, OdometryInfo * info)
{
	return process(data, Transform(), info);
}

Transform Odometry::process(SensorData & data, const Transform & guessIn, OdometryInfo * info)
{
	UASSERT_MSG(data.id() >= 0, uFormat("Input data should have ID greater or equal than 0 (id=%d)!", data.id()).c_str());

	// cache imu data
	if(!data.imu().empty() && !this->canProcessAsyncIMU())
	{
		if(!(data.imu().orientation()[0] == 0.0 && data.imu().orientation()[1] == 0.0 && data.imu().orientation()[2] == 0.0))
		{
			Transform orientation(0,0,0, data.imu().orientation()[0], data.imu().orientation()[1], data.imu().orientation()[2], data.imu().orientation()[3]);
			// orientation includes roll and pitch but not yaw in local transform
			Transform imuT = Transform(data.imu().localTransform().x(),data.imu().localTransform().y(),data.imu().localTransform().z(), 0,0,data.imu().localTransform().theta()) *
					orientation*
					data.imu().localTransform().rotation().inverse();

			if(	this->getPose().r11() == 1.0f && this->getPose().r22() == 1.0f && this->getPose().r33() == 1.0f &&
				this->framesProcessed() == 0)
			{
				Eigen::Quaterniond imuQuat = imuT.getQuaterniond();
				Transform previous = this->getPose();
				Transform newFramePose = Transform(previous.x(), previous.y(), previous.z(), imuQuat.x(), imuQuat.y(), imuQuat.z(), imuQuat.w());
				UWARN("Updated initial pose from %s to %s with IMU orientation", previous.prettyPrint().c_str(), newFramePose.prettyPrint().c_str());
				this->reset(newFramePose);
			}

			imus_.insert(std::make_pair(data.stamp(), imuT));
			if(imus_.size() > 1000)
			{
				imus_.erase(imus_.begin());
			}
		}
		else
		{
			UWARN("Received IMU doesn't have orientation set! It is ignored.");
		}
	}


	if(!_imagesAlreadyRectified && !this->canProcessRawImages() && !data.imageRaw().empty())
	{
		if(!data.stereoCameraModels().empty())
		{
			bool valid = true;
			if(data.stereoCameraModels().size() != stereoModels_.size())
			{
				stereoModels_.clear();
				valid = false;
			}
			else
			{
				for(size_t i=0; i<data.stereoCameraModels().size() && valid; ++i)
				{
					valid = stereoModels_[i].isRectificationMapInitialized() &&
							stereoModels_[i].left().imageSize() == data.stereoCameraModels()[i].left().imageSize();
				}
			}

			if(!valid)
			{
				stereoModels_ = data.stereoCameraModels();
				valid = true;
				for(size_t i=0; i<stereoModels_.size() && valid; ++i)
				{
					stereoModels_[i].initRectificationMap();
					valid = stereoModels_[i].isRectificationMapInitialized();
				}
				if(valid)
				{
					UWARN("%s parameter is set to false but the selected odometry approach cannot "
							"process raw stereo images. We will rectify them for convenience.",
							Parameters::kRtabmapImagesAlreadyRectified().c_str());
				}
				else
				{
					UERROR("Odometry approach chosen cannot process raw stereo images (not rectified images) "
							"and we cannot rectify them as the rectification map failed to initialize (valid calibration?). "
							"Make sure images are rectified and set %s parameter back to true, or "
							"make sure calibration is valid for rectification",
							Parameters::kRtabmapImagesAlreadyRectified().c_str());
					stereoModels_.clear();
				}
			}
			if(valid)
			{
				if(stereoModels_.size()==1)
				{
					data.setStereoImage(
							stereoModels_[0].left().rectifyImage(data.imageRaw()),
							stereoModels_[0].right().rectifyImage(data.rightRaw()),
							stereoModels_,
							false);
				}
				else
				{
					UASSERT(int((data.imageRaw().cols/data.stereoCameraModels().size())*data.stereoCameraModels().size()) == data.imageRaw().cols);
					int subImageWidth = data.imageRaw().cols/data.stereoCameraModels().size();
					cv::Mat rectifiedLeftImages = data.imageRaw().clone();
					cv::Mat rectifiedRightImages = data.imageRaw().clone();
					for(size_t i=0; i<stereoModels_.size() && valid; ++i)
					{
						cv::Mat rectifiedLeft = stereoModels_[i].left().rectifyImage(cv::Mat(data.imageRaw(), cv::Rect(subImageWidth*i, 0, subImageWidth, data.imageRaw().rows)));
						cv::Mat rectifiedRight = stereoModels_[i].right().rectifyImage(cv::Mat(data.rightRaw(), cv::Rect(subImageWidth*i, 0, subImageWidth, data.rightRaw().rows)));
						rectifiedLeft.copyTo(cv::Mat(rectifiedLeftImages, cv::Rect(subImageWidth*i, 0, subImageWidth, data.imageRaw().rows)));
						rectifiedRight.copyTo(cv::Mat(rectifiedRightImages, cv::Rect(subImageWidth*i, 0, subImageWidth, data.imageRaw().rows)));
					}
					data.setStereoImage(rectifiedLeftImages, rectifiedRightImages, stereoModels_, false);
				}
			}
		}
		else if(!data.cameraModels().empty())
		{
			bool valid = true;
			if(data.cameraModels().size() != models_.size())
			{
				models_.clear();
				valid = false;
			}
			else
			{
				for(size_t i=0; i<data.cameraModels().size() && valid; ++i)
				{
					valid = models_[i].isRectificationMapInitialized() &&
							models_[i].imageSize() == data.cameraModels()[i].imageSize();
				}
			}

			if(!valid)
			{
				models_ = data.cameraModels();
				valid = true;
				for(size_t i=0; i<models_.size() && valid; ++i)
				{
					valid = models_[i].initRectificationMap();
				}
				if(valid)
				{
					UWARN("%s parameter is set to false but the selected odometry approach cannot "
							"process raw images. We will rectify them for convenience (only "
							"rgb is rectified, we assume depth image is already rectified!).",
							Parameters::kRtabmapImagesAlreadyRectified().c_str());
				}
				else
				{
					UERROR("Odometry approach chosen cannot process raw images (not rectified images) "
							"and we cannot rectify them as the rectification map failed to initialize (valid calibration?). "
							"Make sure images are rectified and set %s parameter back to true, or "
							"make sure calibration is valid for rectification",
							Parameters::kRtabmapImagesAlreadyRectified().c_str());
					models_.clear();
				}
			}
			if(valid)
			{
				// Note that only RGB image is rectified, the depth image is assumed to be already registered to rectified RGB camera.
				if(models_.size()==1)
				{
					data.setRGBDImage(models_[0].rectifyImage(data.imageRaw()), data.depthRaw(), models_, false);
				}
				else
				{
					UASSERT(int((data.imageRaw().cols/data.cameraModels().size())*data.cameraModels().size()) == data.imageRaw().cols);
					int subImageWidth = data.imageRaw().cols/data.cameraModels().size();
					cv::Mat rectifiedImages = data.imageRaw().clone();
					for(size_t i=0; i<models_.size() && valid; ++i)
					{
						cv::Mat rectifiedImage = models_[i].rectifyImage(cv::Mat(data.imageRaw(), cv::Rect(subImageWidth*i, 0, subImageWidth, data.imageRaw().rows)));
						rectifiedImage.copyTo(cv::Mat(rectifiedImages, cv::Rect(subImageWidth*i, 0, subImageWidth, data.imageRaw().rows)));
					}
					data.setRGBDImage(rectifiedImages, data.depthRaw(), models_, false);
				}
			}
		}
		else
		{
			UERROR("Odometry approach chosen cannot process raw images (not rectified images). Make sure images "
					"are rectified, and set %s parameter back to true, or make sure that calibration is valid "
					"for rectification so we can rectifiy them for convenience",
					Parameters::kRtabmapImagesAlreadyRectified().c_str());
		}
	}

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
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = util3d::cloudFromSensorData(data, 1, 10, 0, indices.get());
			bool success = false;
			if(indices->size())
			{
				cloud = util3d::voxelize(cloud, indices, 0.01);
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
					this->reset(rotation);
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

	// KITTI datasets start with stamp=0
	double dt = previousStamp_>0.0f || (previousStamp_==0.0f && framesProcessed()==1)?data.stamp() - previousStamp_:0.0;
	Transform guess = dt>0.0 && guessFromMotion_ && !velocityGuess_.isNull()?Transform::getIdentity():Transform();
	if(!(dt>0.0 || (dt == 0.0 && velocityGuess_.isNull())))
	{
		if(guessFromMotion_ && (!data.imageRaw().empty() || !data.laserScanRaw().isEmpty()))
		{
			UERROR("Guess from motion is set but dt is invalid! Odometry is then computed without guess. (dt=%f previous transform=%s)", dt, velocityGuess_.prettyPrint().c_str());
		}
		else if(_filteringStrategy==1)
		{
			UERROR("Kalman filtering is enabled but dt is invalid! Odometry is then computed without Kalman filtering. (dt=%f previous transform=%s)", dt, velocityGuess_.prettyPrint().c_str());
		}
		dt=0;
		previousVelocities_.clear();
		velocityGuess_.setNull();
	}
	if(!velocityGuess_.isNull())
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
				velocityGuess_.getTranslationAndEulerAngles(vx,vy,vz, vroll,vpitch,vyaw);
				guess = Transform(vx*dt, vy*dt, vz*dt, vroll*dt, vpitch*dt, vyaw*dt);
			}
		}
		else if(_filteringStrategy == 1)
		{
			predictKalmanFilter(dt);
		}
	}

	Transform imuCurrentTransform;
	if(!guessIn.isNull())
	{
		guess = guessIn;
	}
	else if(!imus_.empty())
	{
		// replace orientation guess with IMU (if available)
		imuCurrentTransform = Transform::getTransform(imus_, data.stamp());
		if(!imuCurrentTransform.isNull() && !imuLastTransform_.isNull())
		{
			Transform orientation = imuLastTransform_.inverse() * imuCurrentTransform;
			guess = Transform(
					orientation.r11(), orientation.r12(), orientation.r13(), guess.x(),
					orientation.r21(), orientation.r22(), orientation.r23(), guess.y(),
					orientation.r31(), orientation.r32(), orientation.r33(), guess.z());
			if(_force3DoF)
			{
				guess = guess.to3DoF();
			}
		}
		else if(!imuLastTransform_.isNull())
		{
			UWARN("Could not find imu transform at %f", data.stamp());
		}
	}

	UTimer time;
	Transform t;
	if(_imageDecimation > 1 && !data.imageRaw().empty())
	{
		// Decimation of images with calibrations
		SensorData decimatedData = data;
		int decimationDepth = _imageDecimation;
		if(	!data.cameraModels().empty() &&
			data.cameraModels()[0].imageHeight()>0 &&
			data.cameraModels()[0].imageWidth()>0)
		{
			// decimate from RGB image size
			int targetSize = data.cameraModels()[0].imageHeight() / _imageDecimation;
			if(targetSize >= data.depthRaw().rows)
			{
				decimationDepth = 1;
			}
			else
			{
				decimationDepth = (int)ceil(float(data.depthRaw().rows) / float(targetSize));
			}
		}
		UDEBUG("decimation rgbOrLeft(rows=%d)=%d, depthOrRight(rows=%d)=%d", data.imageRaw().rows, _imageDecimation, data.depthOrRightRaw().rows, decimationDepth);

		cv::Mat rgbLeft = util2d::decimate(decimatedData.imageRaw(), _imageDecimation);
		cv::Mat depthRight = util2d::decimate(decimatedData.depthOrRightRaw(), decimationDepth);
		std::vector<CameraModel> cameraModels = decimatedData.cameraModels();
		for(unsigned int i=0; i<cameraModels.size(); ++i)
		{
			cameraModels[i] = cameraModels[i].scaled(1.0/double(_imageDecimation));
		}
		if(!cameraModels.empty())
		{
			decimatedData.setRGBDImage(rgbLeft, depthRight, cameraModels);
		}
		else
		{
			std::vector<StereoCameraModel> stereoModels = decimatedData.stereoCameraModels();
			for(unsigned int i=0; i<stereoModels.size(); ++i)
			{
				stereoModels[i].scale(1.0/double(_imageDecimation));
			}
			if(!stereoModels.empty())
			{
				decimatedData.setStereoImage(rgbLeft, depthRight, stereoModels);
			}
		}


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
		data.setLaserScan(decimatedData.laserScanRaw());

		if(info)
		{
			UASSERT(info->newCorners.size() == info->refCorners.size() || info->refCorners.empty());
			for(unsigned int i=0; i<info->newCorners.size(); ++i)
			{
				info->refCorners[i].x *= _imageDecimation;
				info->refCorners[i].y *= _imageDecimation;
				if(!info->refCorners.empty())
				{
					info->newCorners[i].x *= _imageDecimation;
					info->newCorners[i].y *= _imageDecimation;
				}
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
	else if(!data.imageRaw().empty() || !data.laserScanRaw().isEmpty() || (this->canProcessAsyncIMU() && !data.imu().empty()))
	{
		t = this->computeTransform(data, guess, info);
	}

	if(data.imageRaw().empty() && data.laserScanRaw().isEmpty() && !data.imu().empty())
	{
		return Transform(); // Return null on IMU-only updates
	}

	if(info)
	{
		info->timeEstimation = time.ticks();
		info->lost = t.isNull();
		info->stamp = data.stamp();
		info->interval = dt;
		info->transform = t;
		info->guess = guess;
		if(_publishRAMUsage)
		{
			info->memoryUsage = UProcessInfo::getMemoryUsage()/(1024*1024);
		}

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
				if(velocityGuess_.isNull())
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
			else
			{
				if(particleFilters_.size())
				{
					// Particle filtering
					UASSERT(particleFilters_.size()==6);
					if(velocityGuess_.isNull())
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
				}
				else if(!_holonomic)
				{
					// arc trajectory around ICR
					vy = vyaw!=0.0f ? vx / tan((CV_PI-vyaw)/2.0f) : 0.0f;
				}

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

		if(data.stamp() == 0 && framesProcessed_ != 0)
		{
			UWARN("Null stamp detected");
		}

		previousStamp_ = data.stamp();

		if(dt)
		{
			if(dt >= (guessSmoothingDelay_/2.0) || particleFilters_.size() || _filteringStrategy==1)
			{
				velocityGuess_ = Transform(vx, vy, vz, vroll, vpitch, vyaw);
				previousVelocities_.clear();
			}
			else
			{
				// smooth velocity estimation over the past X seconds
				std::vector<float> v(6);
				v[0] = vx;
				v[1] = vy;
				v[2] = vz;
				v[3] = vroll;
				v[4] = vpitch;
				v[5] = vyaw;
				previousVelocities_.push_back(std::make_pair(v, data.stamp()));
				while(previousVelocities_.size() > 1 && previousVelocities_.front().second < previousVelocities_.back().second-guessSmoothingDelay_)
				{
					previousVelocities_.pop_front();
				}
				velocityGuess_ = getMeanVelocity(previousVelocities_);
			}
		}
		else
		{
			previousVelocities_.clear();
			velocityGuess_.setNull();
		}

		if(info)
		{
			distanceTravelled_ += t.getNorm();
			info->distanceTravelled = distanceTravelled_;
			info->guessVelocity = velocityGuess_;
		}
		++framesProcessed_;

		imuLastTransform_ = imuCurrentTransform;

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
			if(info)
			{
				*info = OdometryInfo();
			}
			return this->computeTransform(data, Transform(), info);
		}
	}

	previousVelocities_.clear();
	velocityGuess_.setNull();
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
