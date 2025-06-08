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

#include "rtabmap/core/SensorCaptureThread.h"
#include "rtabmap/core/Camera.h"
#include "rtabmap/core/Lidar.h"
#include "rtabmap/core/SensorEvent.h"
#include "rtabmap/core/CameraRGBD.h"
#include "rtabmap/core/util2d.h"
#include "rtabmap/core/util3d.h"
#include "rtabmap/core/util3d_surface.h"
#include "rtabmap/core/util3d_filtering.h"
#include "rtabmap/core/StereoDense.h"
#include "rtabmap/core/DBReader.h"
#include "rtabmap/core/IMUFilter.h"
#include "rtabmap/core/Features2d.h"
#include "rtabmap/core/clams/discrete_depth_distortion_model.h"
#include <opencv2/imgproc/types_c.h>
#include <opencv2/stitching/detail/exposure_compensate.hpp>
#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UStl.h>

#include <pcl/common/io.h>

namespace rtabmap
{

// ownership transferred
SensorCaptureThread::SensorCaptureThread(
		Camera * camera,
		const ParametersMap & parameters) :
			SensorCaptureThread(0, camera, 0, Transform(), 0.0, 1.0f, 0.1, parameters)
{
	UASSERT(camera != 0);
}

// ownership transferred
SensorCaptureThread::SensorCaptureThread(
		Camera * camera,
		SensorCapture * odomSensor,
		const Transform & extrinsics,
		double poseTimeOffset,
		float poseScaleFactor,
		double poseWaitTime,
		const ParametersMap & parameters) :
			SensorCaptureThread(0, camera, odomSensor, extrinsics, poseTimeOffset, poseScaleFactor, poseWaitTime, parameters)
{
	UASSERT(camera != 0 && odomSensor != 0 && !extrinsics.isNull());
}

SensorCaptureThread::SensorCaptureThread(
		Lidar * lidar,
		const ParametersMap & parameters) :
			SensorCaptureThread(lidar, 0, 0, Transform(), 0.0, 1.0f, 0.1, parameters)
{
	UASSERT(lidar != 0);
}

SensorCaptureThread::SensorCaptureThread(
		Lidar * lidar,
		Camera * camera,
		const ParametersMap & parameters) :
			SensorCaptureThread(lidar, camera, 0, Transform(), 0.0, 1.0f, 0.1, parameters)
{
	UASSERT(lidar != 0 && camera != 0);
}

SensorCaptureThread::SensorCaptureThread(
		Lidar * lidar,
		SensorCapture * odomSensor,
		double poseTimeOffset,
		float poseScaleFactor,
		double poseWaitTime,
		const ParametersMap & parameters) :
			SensorCaptureThread(lidar, 0, odomSensor, Transform(), poseTimeOffset, poseScaleFactor, poseWaitTime, parameters)
{
	UASSERT(lidar != 0 && odomSensor != 0);
}

SensorCaptureThread::SensorCaptureThread(
		Lidar * lidar,
		Camera * camera,
		SensorCapture * odomSensor,
		const Transform & extrinsics,
		double poseTimeOffset,
		float poseScaleFactor,
		double poseWaitTime,
		const ParametersMap & parameters) :
				_camera(camera),
				_odomSensor(odomSensor),
				_lidar(lidar),
				_extrinsicsOdomToCamera(extrinsics * CameraModel::opticalRotation()),
				_odomAsGt(false),
				_poseTimeOffset(poseTimeOffset),
				_poseScaleFactor(poseScaleFactor),
				_poseWaitTime(poseWaitTime),
				_mirroring(false),
				_stereoExposureCompensation(false),
				_colorOnly(false),
				_imageDecimation(1),
				_histogramMethod(0),
				_stereoToDepth(false),
				_scanDeskewing(false),
				_scanFromDepth(false),
				_scanDownsampleStep(1),
				_scanRangeMin(0.0f),
				_scanRangeMax(0.0f),
				_scanVoxelSize(0.0f),
				_scanNormalsK(0),
				_scanNormalsRadius(0.0f),
				_scanForceGroundNormalsUp(false),
				_stereoDense(StereoDense::create(parameters)),
				_distortionModel(0),
				_bilateralFiltering(false),
				_bilateralSigmaS(10),
				_bilateralSigmaR(0.1),
				_imuFilter(0),
				_imuBaseFrameConversion(false),
				_featureDetector(0),
				_depthAsMask(Parameters::defaultVisDepthAsMask())
{
	UASSERT(_camera != 0 || _lidar != 0);
	if(_lidar && _camera)
	{
		_camera->setFrameRate(0);
	}
	if(_odomSensor)
	{
		if(_camera)
		{
			if(_odomSensor == _camera && _extrinsicsOdomToCamera.isNull())
			{
				_extrinsicsOdomToCamera.setIdentity();
			}
			UASSERT(!_extrinsicsOdomToCamera.isNull());
			UDEBUG("_extrinsicsOdomToCamera=%s", _extrinsicsOdomToCamera.prettyPrint().c_str());
		}
		UDEBUG("_poseTimeOffset        =%f", _poseTimeOffset);
		UDEBUG("_poseScaleFactor       =%f", _poseScaleFactor);
		UDEBUG("_poseWaitTime          =%f", _poseWaitTime);
	}
}

SensorCaptureThread::~SensorCaptureThread()
{
	join(true);
	if(_odomSensor != _camera && _odomSensor != _lidar)
	{
		delete _odomSensor;
	}
	delete _camera;
	delete _lidar;
	delete _distortionModel;
	delete _stereoDense;
	delete _imuFilter;
	delete _featureDetector;
}

void SensorCaptureThread::setFrameRate(float frameRate)
{
	if(_lidar)
	{
		_lidar->setFrameRate(frameRate);
	}
	else if(_camera)
	{
		_camera->setFrameRate(frameRate);
	}
}

void SensorCaptureThread::setDistortionModel(const std::string & path)
{
	if(_distortionModel)
	{
		delete _distortionModel;
		_distortionModel = 0;
	}
	if(!path.empty())
	{
		_distortionModel = new clams::DiscreteDepthDistortionModel();
		_distortionModel->load(path);
		if(!_distortionModel->isValid())
		{
			UERROR("Loaded distortion model \"%s\" is not valid!", path.c_str());
			delete _distortionModel;
			_distortionModel = 0;
		}
	}
}

void SensorCaptureThread::enableBilateralFiltering(float sigmaS, float sigmaR)
{
	UASSERT(sigmaS > 0.0f && sigmaR > 0.0f);
	_bilateralFiltering = true;
	_bilateralSigmaS = sigmaS;
	_bilateralSigmaR = sigmaR;
}

void SensorCaptureThread::enableIMUFiltering(int filteringStrategy, const ParametersMap & parameters, bool baseFrameConversion)
{
	delete _imuFilter;
	_imuFilter = IMUFilter::create((IMUFilter::Type)filteringStrategy, parameters);
	_imuBaseFrameConversion = baseFrameConversion;
}

void SensorCaptureThread::disableIMUFiltering()
{
	delete _imuFilter;
	_imuFilter = 0;
}

void SensorCaptureThread::enableFeatureDetection(const ParametersMap & parameters)
{
	delete _featureDetector;
	ParametersMap params = parameters;
	ParametersMap defaultParams = Parameters::getDefaultParameters("Vis");
	uInsert(params, ParametersPair(Parameters::kKpDetectorStrategy(), uValue(params, Parameters::kVisFeatureType(), defaultParams.at(Parameters::kVisFeatureType()))));
	uInsert(params, ParametersPair(Parameters::kKpMaxFeatures(), uValue(params, Parameters::kVisMaxFeatures(), defaultParams.at(Parameters::kVisMaxFeatures()))));
	uInsert(params, ParametersPair(Parameters::kKpSSC(), uValue(params, Parameters::kVisSSC(), defaultParams.at(Parameters::kVisSSC()))));
	uInsert(params, ParametersPair(Parameters::kKpMaxDepth(), uValue(params, Parameters::kVisMaxDepth(), defaultParams.at(Parameters::kVisMaxDepth()))));
	uInsert(params, ParametersPair(Parameters::kKpMinDepth(), uValue(params, Parameters::kVisMinDepth(), defaultParams.at(Parameters::kVisMinDepth()))));
	uInsert(params, ParametersPair(Parameters::kKpRoiRatios(), uValue(params, Parameters::kVisRoiRatios(), defaultParams.at(Parameters::kVisRoiRatios()))));
	uInsert(params, ParametersPair(Parameters::kKpSubPixEps(), uValue(params, Parameters::kVisSubPixEps(), defaultParams.at(Parameters::kVisSubPixEps()))));
	uInsert(params, ParametersPair(Parameters::kKpSubPixIterations(), uValue(params, Parameters::kVisSubPixIterations(), defaultParams.at(Parameters::kVisSubPixIterations()))));
	uInsert(params, ParametersPair(Parameters::kKpSubPixWinSize(), uValue(params, Parameters::kVisSubPixWinSize(), defaultParams.at(Parameters::kVisSubPixWinSize()))));
	uInsert(params, ParametersPair(Parameters::kKpGridRows(), uValue(params, Parameters::kVisGridRows(), defaultParams.at(Parameters::kVisGridRows()))));
	uInsert(params, ParametersPair(Parameters::kKpGridCols(), uValue(params, Parameters::kVisGridCols(), defaultParams.at(Parameters::kVisGridCols()))));
	_featureDetector = Feature2D::create(params);
	_depthAsMask = Parameters::parse(params, Parameters::kVisDepthAsMask(), _depthAsMask);
}
void SensorCaptureThread::disableFeatureDetection()
{
	delete _featureDetector;
	_featureDetector = 0;
}

void SensorCaptureThread::setScanParameters(
	bool fromDepth,
	int downsampleStep,
	float rangeMin,
	float rangeMax,
	float voxelSize,
	int normalsK,
	float normalsRadius,
	bool forceGroundNormalsUp,
	bool deskewing)
{
	setScanParameters(fromDepth, downsampleStep, rangeMin, rangeMax, voxelSize, normalsK, normalsRadius, forceGroundNormalsUp?0.8f:0.0f, deskewing);
}

void SensorCaptureThread::setScanParameters(
			bool fromDepth,
			int downsampleStep, // decimation of the depth image in case the scan is from depth image
			float rangeMin,
			float rangeMax,
			float voxelSize,
			int normalsK,
			float normalsRadius,
			float groundNormalsUp,
			bool deskewing)
{
	_scanDeskewing = deskewing;
	_scanFromDepth = fromDepth;
	_scanDownsampleStep=downsampleStep;
	_scanRangeMin = rangeMin;
	_scanRangeMax = rangeMax;
	_scanVoxelSize = voxelSize;
	_scanNormalsK = normalsK;
	_scanNormalsRadius = normalsRadius;
	_scanForceGroundNormalsUp = groundNormalsUp;
}

bool SensorCaptureThread::odomProvided() const
{
	if(_odomAsGt)
	{
		return false;
	}
	return _odomSensor != 0;
}

void SensorCaptureThread::mainLoopBegin()
{
	ULogger::registerCurrentThread("Camera");
	if(_lidar)
	{
		_lidar->resetTimer();
	}
	else if(_camera)
	{
		_camera->resetTimer();
	}
	if(_imuFilter)
	{
		// In case we paused the camera and moved somewhere else, restart filtering.
		_imuFilter->reset();
	}
}

void SensorCaptureThread::mainLoop()
{
	UASSERT(_lidar || _camera);
	UTimer totalTime;
	SensorCaptureInfo info;
	SensorData data;
	SensorData cameraData;
	double lidarStamp = 0.0;
	double cameraStamp = 0.0;
	if(_lidar)
	{
		data = _lidar->takeData(&info);
		if(data.stamp() == 0.0)
		{
			UWARN("Could not capture scan!");
		}
		else
		{
			lidarStamp = data.stamp();
			if(_camera)
			{
				cameraData = _camera->takeData();
				if(cameraData.stamp() == 0.0)
				{
					UWARN("Could not capture image!");
				}
				else
				{
					double stampStart = UTimer::now();
					while(cameraData.stamp() < data.stamp() &&
						!isKilled() &&
						UTimer::now() - stampStart < _poseWaitTime &&
						!cameraData.imageRaw().empty())
					{
						// Make sure the camera frame is newer than lidar frame so
						// that if there are imus published by the cameras, we can get
						// them all in odometry before deskewing.
						cameraData = _camera->takeData();
					}

					cameraStamp = cameraData.stamp();
					if(cameraData.stamp() < data.stamp())
					{
						UWARN("Could not get camera frame (%f) with stamp more recent than lidar frame (%f) after waiting for %f seconds.",
								cameraData.stamp(),
								data.stamp(),
								_poseWaitTime);
					}

					if(!cameraData.stereoCameraModels().empty())
					{
						data.setStereoImage(cameraData.imageRaw(), cameraData.depthOrRightRaw(), cameraData.stereoCameraModels(), true);
					}
					else
					{
						data.setRGBDImage(cameraData.imageRaw(), cameraData.depthOrRightRaw(), cameraData.cameraModels(), true);
					}
				}
			}
		}
	}
	else if(_camera)
	{
		data = _camera->takeData(&info);
		if(data.stamp() == 0.0)
		{
			UWARN("Could not capture image!");
		}
		else
		{
			cameraStamp = cameraData.stamp();
		}
	}

	if(_odomSensor && data.stamp() != 0.0)
	{
		if(lidarStamp!=0.0 && _scanDeskewing)
		{
			UDEBUG("Deskewing begin");
			if(!data.laserScanRaw().empty() && data.laserScanRaw().hasTime())
			{
				float scanTime =
					data.laserScanRaw().data().ptr<float>(0, data.laserScanRaw().size()-1)[data.laserScanRaw().getTimeOffset()] -
					data.laserScanRaw().data().ptr<float>(0, 0)[data.laserScanRaw().getTimeOffset()];

				Transform poseFirstScan;
				Transform poseLastScan;
				cv::Mat cov;
				double firstStamp = data.stamp() + data.laserScanRaw().data().ptr<float>(0, 0)[data.laserScanRaw().getTimeOffset()];
				double lastStamp = data.stamp() + data.laserScanRaw().data().ptr<float>(0, data.laserScanRaw().size()-1)[data.laserScanRaw().getTimeOffset()];
				if(_odomSensor->getPose(firstStamp+_poseTimeOffset, poseFirstScan, cov, _poseWaitTime>0?_poseWaitTime:0) &&
				   _odomSensor->getPose(lastStamp+_poseTimeOffset, poseLastScan, cov, _poseWaitTime>0?_poseWaitTime:0))
				{
					if(_poseScaleFactor>0 && _poseScaleFactor!=1.0f)
					{
						poseFirstScan.x() *= _poseScaleFactor;
						poseFirstScan.y() *= _poseScaleFactor;
						poseFirstScan.z() *= _poseScaleFactor;
						poseLastScan.x() *= _poseScaleFactor;
						poseLastScan.y() *= _poseScaleFactor;
						poseLastScan.z() *= _poseScaleFactor;
					}

					UASSERT(!poseFirstScan.isNull() && !poseLastScan.isNull());

					Transform transform = poseFirstScan.inverse() * poseLastScan;

					// convert to velocity
					float x,y,z,roll,pitch,yaw;
					transform.getTranslationAndEulerAngles(x, y, z, roll, pitch, yaw);
					x/=scanTime;
					y/=scanTime;
					z/=scanTime;
					roll /= scanTime;
					pitch /= scanTime;
					yaw /= scanTime;

					Transform velocity(x,y,z,roll,pitch,yaw);
					UTimer timeDeskewing;
					LaserScan scanDeskewed = util3d::deskew(data.laserScanRaw(), data.stamp(), velocity);
					info.timeDeskewing = timeDeskewing.ticks();
					if(!scanDeskewed.isEmpty())
					{
						data.setLaserScan(scanDeskewed);
					}
				}
				else if(!data.laserScanRaw().empty())
				{
					UWARN("Failed to get poses for stamps %f and %f! Lidar won't be deskewed!", firstStamp+_poseTimeOffset, lastStamp+_poseTimeOffset);
				}
			}
			else if(!data.laserScanRaw().empty())
			{
				UWARN("The input scan doesn't have time channel (scan format received=%s)!. Lidar won't be deskewed!", data.laserScanRaw().formatName().c_str());
			}
			UDEBUG("Deskewing end");
		}

		Transform pose;
		cv::Mat covariance;
		if(!info.odomPose.isNull() && _lidar == 0 && _odomSensor == _camera)
		{
			// Do nothing, we have already the pose
		}
		else if(_odomSensor->getPose(data.stamp()+_poseTimeOffset, pose, covariance, _poseWaitTime>0?_poseWaitTime:0))
		{
			info.odomPose = pose;
			info.odomCovariance = covariance;
			if(_poseScaleFactor>0 && _poseScaleFactor!=1.0f)
			{
				info.odomPose.x() *= _poseScaleFactor;
				info.odomPose.y() *= _poseScaleFactor;
				info.odomPose.z() *= _poseScaleFactor;
			}

			if(cameraStamp != 0.0)
			{
				Transform cameraCorrection = Transform::getIdentity();
				if(lidarStamp > 0.0 && lidarStamp != cameraStamp)
				{
					if(_odomSensor->getPose(cameraStamp+_poseTimeOffset, pose, covariance, _poseWaitTime>0?_poseWaitTime:0))
					{
						cameraCorrection = info.odomPose.inverse() * pose;
					}
					else
					{
						UWARN("Could not get pose at stamp %f, the camera local motion against lidar won't be adjusted.", cameraStamp);
					}
				}

				// Adjust local transform of the camera based on the pose frame
				if(!data.cameraModels().empty())
				{
					UASSERT(data.cameraModels().size()==1);
					CameraModel model = data.cameraModels()[0];
					model.setLocalTransform(cameraCorrection*_extrinsicsOdomToCamera);
					data.setCameraModel(model);
				}
				else if(!data.stereoCameraModels().empty())
				{
					UASSERT(data.stereoCameraModels().size()==1);
					StereoCameraModel model = data.stereoCameraModels()[0];
					model.setLocalTransform(cameraCorrection*_extrinsicsOdomToCamera);
					data.setStereoCameraModel(model);
				}
			}

			// Fake IMU to intialize gravity (assuming pose is aligned with gravity!)
			Eigen::Quaterniond q = info.odomPose.getQuaterniond();
			data.setIMU(IMU(
					cv::Vec4d(q.x(), q.y(), q.z(), q.w()), cv::Mat(),
					cv::Vec3d(), cv::Mat(),
					cv::Vec3d(), cv::Mat(),
					Transform::getIdentity()));
			this->disableIMUFiltering();
		}
		else
		{
			UWARN("Could not get pose at stamp %f", data.stamp());
		}
	}

	if(_odomAsGt && !info.odomPose.isNull())
	{
		data.setGroundTruth(info.odomPose);
		info.odomPose.setNull();
	}

	if(!data.imageCompressed().empty() || !data.imageRaw().empty() || !data.laserScanRaw().empty() || (dynamic_cast<DBReader*>(_camera) != 0 && data.id()>0)) // intermediate nodes could not have image set
	{
		postUpdate(&data, &info);
		info.cameraName = _lidar?_lidar->getSerial():_camera->getSerial();
		info.timeTotal = totalTime.ticks();
		this->post(new SensorEvent(data, info));
	}
	else if(!this->isKilled())
	{
		UWARN("no more data...");
		this->kill();
		this->post(new SensorEvent());
	}
}

void SensorCaptureThread::mainLoopKill()
{
	if(dynamic_cast<CameraFreenect2*>(_camera) != 0)
	{
		int i=20;
		while(i-->0)
		{
			uSleep(100);
			if(!this->isKilled())
			{
				break;
			}
		}
		if(this->isKilled())
		{
			//still in killed state, maybe a deadlock
			UERROR("CameraFreenect2: Failed to kill normally the Freenect2 driver! The thread is locked "
				   "on waitForNewFrame() method of libfreenect2. This maybe caused by not linking on the right libusb. "
				   "Note that rtabmap should link on libusb of libfreenect2. "
				   "Tip before starting rtabmap: \"$ export LD_LIBRARY_PATH=~/libfreenect2/depends/libusb/lib:$LD_LIBRARY_PATH\"");
		}

	}
}

void SensorCaptureThread::postUpdate(SensorData * dataPtr, SensorCaptureInfo * info) const
{
	UASSERT(dataPtr!=0);
	SensorData & data = *dataPtr;
	if(_colorOnly)
	{
		if(!data.depthRaw().empty())
		{
			data.setRGBDImage(data.imageRaw(), cv::Mat(), data.cameraModels());
		}
		else if(!data.rightRaw().empty())
		{
			std::vector<CameraModel> models;
			for(size_t i=0; i<data.stereoCameraModels().size(); ++i)
			{
				models.push_back(data.stereoCameraModels()[i].left());
			}
			data.setRGBDImage(data.imageRaw(), cv::Mat(), models);
		}
	}

	if(_distortionModel && !data.depthRaw().empty())
	{
		UTimer timer;
		if(_distortionModel->getWidth() >= data.depthRaw().cols &&
		   _distortionModel->getHeight() >= data.depthRaw().rows &&
		   _distortionModel->getWidth() % data.depthRaw().cols == 0 &&
		   _distortionModel->getHeight() % data.depthRaw().rows == 0)
		{
			cv::Mat depth = data.depthRaw().clone();// make sure we are not modifying data in cached signatures.
			_distortionModel->undistort(depth);
			data.setRGBDImage(data.imageRaw(), depth, data.cameraModels());
		}
		else
		{
			UERROR("Distortion model size is %dx%d but depth image is %dx%d!",
					_distortionModel->getWidth(), _distortionModel->getHeight(),
					data.depthRaw().cols, data.depthRaw().rows);
		}
		if(info) info->timeUndistortDepth = timer.ticks();
	}

	if(_bilateralFiltering && !data.depthRaw().empty())
	{
		UTimer timer;
		data.setRGBDImage(data.imageRaw(), util2d::fastBilateralFiltering(data.depthRaw(), _bilateralSigmaS, _bilateralSigmaR), data.cameraModels());
		if(info) info->timeBilateralFiltering = timer.ticks();
	}

	if(_imageDecimation>1 && !data.imageRaw().empty())
	{
		UDEBUG("");
		UTimer timer;
		if(!data.depthRaw().empty() &&
		   !(data.depthRaw().rows % _imageDecimation == 0 && data.depthRaw().cols % _imageDecimation == 0))
		{
			UERROR("Decimation of depth images should be exact (decimation=%d, size=(%d,%d))! "
				   "Images won't be resized.", _imageDecimation, data.depthRaw().cols, data.depthRaw().rows);
		}
		else
		{
			cv::Mat image = util2d::decimate(data.imageRaw(), _imageDecimation);

			int depthDecimation = _imageDecimation;
			if(data.depthOrRightRaw().rows <= image.rows || data.depthOrRightRaw().cols <= image.cols)
			{
				depthDecimation = 1;
			}
			else
			{
				depthDecimation = 2;
				while(data.depthOrRightRaw().rows / depthDecimation > image.rows ||
					  data.depthOrRightRaw().cols / depthDecimation > image.cols ||
					  data.depthOrRightRaw().rows % depthDecimation != 0 ||
					  data.depthOrRightRaw().cols % depthDecimation != 0)
				{
					++depthDecimation;
				}
				UDEBUG("depthDecimation=%d", depthDecimation);
			}
			cv::Mat depthOrRight = util2d::decimate(data.depthOrRightRaw(), depthDecimation);

			std::vector<CameraModel> models = data.cameraModels();
			for(unsigned int i=0; i<models.size(); ++i)
			{
				if(models[i].isValidForProjection())
				{
					models[i] = models[i].scaled(1.0/double(_imageDecimation));
				}
			}
			if(!models.empty())
			{
				data.setRGBDImage(image, depthOrRight, models);
			}


			std::vector<StereoCameraModel> stereoModels = data.stereoCameraModels();
			for(unsigned int i=0; i<stereoModels.size(); ++i)
			{
				if(stereoModels[i].isValidForProjection())
				{
					stereoModels[i].scale(1.0/double(_imageDecimation));
				}
			}
			if(!stereoModels.empty())
			{
				data.setStereoImage(image, depthOrRight, stereoModels);
			}

			std::vector<cv::KeyPoint> kpts = data.keypoints();
			double log2value = log(double(_imageDecimation))/log(2.0);
			for(unsigned int i=0; i<kpts.size(); ++i)
			{
				kpts[i].pt.x /= _imageDecimation;
				kpts[i].pt.y /= _imageDecimation;
				kpts[i].size /= _imageDecimation;
				kpts[i].octave -= log2value;
			}
			data.setFeatures(kpts, data.keypoints3D(), data.descriptors());
		}
		if(info) info->timeImageDecimation = timer.ticks();
	}

	if(_mirroring && !data.imageRaw().empty() && data.cameraModels().size()>=1)
	{
		if(data.cameraModels().size() == 1)
		{
			UDEBUG("");
			UTimer timer;
			cv::Mat tmpRgb;
			cv::flip(data.imageRaw(), tmpRgb, 1);

			CameraModel tmpModel = data.cameraModels()[0];
			if(data.cameraModels()[0].cx())
			{
				tmpModel = CameraModel(
						data.cameraModels()[0].fx(),
						data.cameraModels()[0].fy(),
						float(data.imageRaw().cols) - data.cameraModels()[0].cx(),
						data.cameraModels()[0].cy(),
						data.cameraModels()[0].localTransform(),
						data.cameraModels()[0].Tx(),
						data.cameraModels()[0].imageSize());
			}
			cv::Mat tmpDepth = data.depthOrRightRaw();
			if(!data.depthRaw().empty())
			{
				cv::flip(data.depthRaw(), tmpDepth, 1);
			}
			data.setRGBDImage(tmpRgb, tmpDepth, tmpModel);
			if(info) info->timeMirroring = timer.ticks();
		}
		else
		{
			UWARN("Mirroring is not implemented for multiple cameras or stereo...");
		}
	}

	if(_histogramMethod && !data.imageRaw().empty())
	{
		UDEBUG("");
		UTimer timer;
		cv::Mat image;
		if(_histogramMethod == 1)
		{
			if(data.imageRaw().type() == CV_8UC1)
			{
				cv::equalizeHist(data.imageRaw(), image);
			}
			else if(data.imageRaw().type() == CV_8UC3)
			{
				cv::Mat channels[3];
				cv::cvtColor(data.imageRaw(), image, CV_BGR2YCrCb);
				cv::split(image, channels);
				cv::equalizeHist(channels[0], channels[0]);
				cv::merge(channels, 3, image);
				cv::cvtColor(image, image, CV_YCrCb2BGR);
			}
			if(!data.depthRaw().empty())
			{
				data.setRGBDImage(image, data.depthRaw(), data.cameraModels());
			}
			else if(!data.rightRaw().empty())
			{
				cv::Mat right;
				if(data.rightRaw().type() == CV_8UC1)
				{
					cv::equalizeHist(data.rightRaw(), right);
				}
				else if(data.rightRaw().type() == CV_8UC3)
				{
					cv::Mat channels[3];
					cv::cvtColor(data.rightRaw(), right, CV_BGR2YCrCb);
					cv::split(right, channels);
					cv::equalizeHist(channels[0], channels[0]);
					cv::merge(channels, 3, right);
					cv::cvtColor(right, right, CV_YCrCb2BGR);
				}
				data.setStereoImage(image, right, data.stereoCameraModels()[0]);
			}
		}
		else if(_histogramMethod == 2)
		{
			cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0);
			if(data.imageRaw().type() == CV_8UC1)
			{
				clahe->apply(data.imageRaw(), image);
			}
			else if(data.imageRaw().type() == CV_8UC3)
			{
				cv::Mat channels[3];
				cv::cvtColor(data.imageRaw(), image, CV_BGR2YCrCb);
				cv::split(image, channels);
				clahe->apply(channels[0], channels[0]);
				cv::merge(channels, 3, image);
				cv::cvtColor(image, image, CV_YCrCb2BGR);
			}
			if(!data.depthRaw().empty())
			{
				data.setRGBDImage(image, data.depthRaw(), data.cameraModels());
			}
			else if(!data.rightRaw().empty())
			{
				cv::Mat right;
				if(data.rightRaw().type() == CV_8UC1)
				{
					clahe->apply(data.rightRaw(), right);
				}
				else if(data.rightRaw().type() == CV_8UC3)
				{
					cv::Mat channels[3];
					cv::cvtColor(data.rightRaw(), right, CV_BGR2YCrCb);
					cv::split(right, channels);
					clahe->apply(channels[0], channels[0]);
					cv::merge(channels, 3, right);
					cv::cvtColor(right, right, CV_YCrCb2BGR);
				}
				data.setStereoImage(image, right, data.stereoCameraModels()[0]);
			}
		}
		if(info) info->timeHistogramEqualization = timer.ticks();
	}

	if(_stereoExposureCompensation && !data.imageRaw().empty() && !data.rightRaw().empty())
	{
		if(data.stereoCameraModels().size()==1)
		{
#if CV_MAJOR_VERSION < 3
			UWARN("Stereo exposure compensation not implemented for OpenCV version under 3.");
#else
			UDEBUG("");
			UTimer timer;
			cv::Ptr<cv::detail::ExposureCompensator> compensator = cv::detail::ExposureCompensator::createDefault(cv::detail::ExposureCompensator::GAIN);
			std::vector<cv::Point> topLeftCorners(2, cv::Point(0,0));
			std::vector<cv::UMat> images;
			std::vector<cv::UMat> masks(2, cv::UMat(data.imageRaw().size(), CV_8UC1,  cv::Scalar(255)));
			images.push_back(data.imageRaw().getUMat(cv::ACCESS_READ));
			images.push_back(data.rightRaw().getUMat(cv::ACCESS_READ));
			compensator->feed(topLeftCorners, images, masks);
			cv::Mat imgLeft = data.imageRaw().clone();
			compensator->apply(0, cv::Point(0,0), imgLeft, masks[0]);
			cv::Mat imgRight = data.rightRaw().clone();
			compensator->apply(1, cv::Point(0,0), imgRight, masks[1]);
			data.setStereoImage(imgLeft, imgRight, data.stereoCameraModels()[0]);
			cv::detail::GainCompensator * gainCompensator = (cv::detail::GainCompensator*)compensator.get();
			UDEBUG("gains = %f %f ", gainCompensator->gains()[0], gainCompensator->gains()[1]);
			if(info) info->timeStereoExposureCompensation = timer.ticks();
#endif
		}
		else
		{
			UWARN("Stereo exposure compensation only is not implemented to multiple stereo cameras...");
		}
	}

	if(_stereoToDepth && !data.imageRaw().empty() && !data.stereoCameraModels().empty() && data.stereoCameraModels()[0].isValidForProjection() && !data.rightRaw().empty())
	{
		if(data.stereoCameraModels().size()==1)
		{
			UDEBUG("");
			UTimer timer;
			cv::Mat depth = util2d::depthFromDisparity(
					_stereoDense->computeDisparity(data.imageRaw(), data.rightRaw()),
					data.stereoCameraModels()[0].left().fx(),
					data.stereoCameraModels()[0].baseline());
			// set Tx for stereo bundle adjustment (when used)
			CameraModel model = CameraModel(
					data.stereoCameraModels()[0].left().fx(),
					data.stereoCameraModels()[0].left().fy(),
					data.stereoCameraModels()[0].left().cx(),
					data.stereoCameraModels()[0].left().cy(),
					data.stereoCameraModels()[0].localTransform(),
					-data.stereoCameraModels()[0].baseline()*data.stereoCameraModels()[0].left().fx(),
					data.stereoCameraModels()[0].left().imageSize());
			data.setRGBDImage(data.imageRaw(), depth, model);
			if(info) info->timeDisparity = timer.ticks();
		}
		else
		{
			UWARN("Stereo to depth is not implemented for multiple stereo cameras...");
		}
	}
	if(_scanFromDepth &&
		data.cameraModels().size() &&
		data.cameraModels().at(0).isValidForProjection() &&
		!data.depthRaw().empty())
	{
		UDEBUG("");
		if(data.laserScanRaw().isEmpty())
		{
			UASSERT(_scanDownsampleStep >= 1);
			UTimer timer;
			pcl::IndicesPtr validIndices(new std::vector<int>);
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = util3d::cloudRGBFromSensorData(
					data,
					_scanDownsampleStep,
					_scanRangeMax,
					_scanRangeMin,
					validIndices.get());
			float maxPoints = (data.depthRaw().rows/_scanDownsampleStep)*(data.depthRaw().cols/_scanDownsampleStep);
			LaserScan scan;
			const Transform & baseToScan = data.cameraModels()[0].localTransform();
			if(validIndices->size())
			{
				if(_scanVoxelSize>0.0f)
				{
					cloud = util3d::voxelize(cloud, validIndices, _scanVoxelSize);
					float ratio = float(cloud->size()) / float(validIndices->size());
					maxPoints = ratio * maxPoints;
				}
				else if(!cloud->is_dense)
				{
					pcl::PointCloud<pcl::PointXYZRGB>::Ptr denseCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
					pcl::copyPointCloud(*cloud, *validIndices, *denseCloud);
					cloud = denseCloud;
				}

				if(cloud->size())
				{
					if(_scanNormalsK>0 || _scanNormalsRadius>0.0f)
					{
						Eigen::Vector3f viewPoint(baseToScan.x(), baseToScan.y(), baseToScan.z());
						pcl::PointCloud<pcl::Normal>::Ptr normals = util3d::computeNormals(cloud, _scanNormalsK, _scanNormalsRadius, viewPoint);
						pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudNormals(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
						pcl::concatenateFields(*cloud, *normals, *cloudNormals);
						scan = util3d::laserScanFromPointCloud(*cloudNormals, baseToScan.inverse());
					}
					else
					{
						scan = util3d::laserScanFromPointCloud(*cloud, baseToScan.inverse());
					}
				}
			}
			data.setLaserScan(LaserScan(scan, (int)maxPoints, _scanRangeMax, baseToScan));
			if(info) info->timeScanFromDepth = timer.ticks();
		}
		else
		{
			UWARN("Option to create laser scan from depth image is enabled, but "
				  "there is already a laser scan in the captured sensor data. Scan from "
				  "depth will not be created.");
		}
	}
	else if(!data.laserScanRaw().isEmpty())
	{
		UDEBUG("");
		// filter the scan after registration
		data.setLaserScan(util3d::commonFiltering(data.laserScanRaw(), _scanDownsampleStep, _scanRangeMin, _scanRangeMax, _scanVoxelSize, _scanNormalsK, _scanNormalsRadius, _scanForceGroundNormalsUp));
	}

	// IMU filtering
	if(_imuFilter && !data.imu().empty())
	{
		if(data.imu().angularVelocity()[0] == 0 &&
		   data.imu().angularVelocity()[1] == 0 &&
		   data.imu().angularVelocity()[2] == 0 &&
		   data.imu().linearAcceleration()[0] == 0 &&
		   data.imu().linearAcceleration()[1] == 0 &&
		   data.imu().linearAcceleration()[2] == 0)
		{
			UWARN("IMU's acc and gyr values are null! Please disable IMU filtering.");
		}
		else
		{
			// Transform IMU data in base_link to correctly initialize yaw
			IMU imu = data.imu();
			if(_imuBaseFrameConversion)
			{
				UASSERT(!data.imu().localTransform().isNull());
				imu.convertToBaseFrame();

			}
			_imuFilter->update(
					imu.angularVelocity()[0],
					imu.angularVelocity()[1],
					imu.angularVelocity()[2],
					imu.linearAcceleration()[0],
					imu.linearAcceleration()[1],
					imu.linearAcceleration()[2],
					data.stamp());
			double qx,qy,qz,qw;
			_imuFilter->getOrientation(qx,qy,qz,qw);

			data.setIMU(IMU(
					cv::Vec4d(qx,qy,qz,qw), cv::Mat::eye(3,3,CV_64FC1),
					imu.angularVelocity(), imu.angularVelocityCovariance(),
					imu.linearAcceleration(), imu.linearAccelerationCovariance(),
					imu.localTransform()));

			UDEBUG("%f %f %f %f (gyro=%f %f %f, acc=%f %f %f, %fs)",
						data.imu().orientation()[0],
						data.imu().orientation()[1],
						data.imu().orientation()[2],
						data.imu().orientation()[3],
						data.imu().angularVelocity()[0],
						data.imu().angularVelocity()[1],
						data.imu().angularVelocity()[2],
						data.imu().linearAcceleration()[0],
						data.imu().linearAcceleration()[1],
						data.imu().linearAcceleration()[2],
						data.stamp());
		}
	}

	if(_featureDetector && !data.imageRaw().empty())
	{
		UDEBUG("Detecting features");
		cv::Mat grayScaleImg = data.imageRaw();
		if(data.imageRaw().channels() > 1)
		{
			cv::Mat tmp;
			cv::cvtColor(grayScaleImg, tmp, cv::COLOR_BGR2GRAY);
			grayScaleImg = tmp;
		}

		cv::Mat depthMask;
		if(!data.depthRaw().empty() && _depthAsMask)
		{
			if( data.imageRaw().rows % data.depthRaw().rows == 0 &&
				data.imageRaw().cols % data.depthRaw().cols == 0 &&
				data.imageRaw().rows/data.depthRaw().rows == data.imageRaw().cols/data.depthRaw().cols)
			{
				depthMask = util2d::interpolate(data.depthRaw(), data.imageRaw().rows/data.depthRaw().rows, 0.1f);
			}
			else
			{
				UWARN("%s is true, but RGB size (%dx%d) modulo depth size (%dx%d) is not 0. Ignoring depth mask for feature detection.",
						Parameters::kVisDepthAsMask().c_str(),
						data.imageRaw().rows, data.imageRaw().cols,
						data.depthRaw().rows, data.depthRaw().cols);
			}
		}

		std::vector<cv::KeyPoint> keypoints = _featureDetector->generateKeypoints(grayScaleImg, depthMask);
		cv::Mat descriptors;
		std::vector<cv::Point3f> keypoints3D;
		if(!keypoints.empty())
		{
			descriptors = _featureDetector->generateDescriptors(grayScaleImg, keypoints);
			if(!keypoints.empty())
			{
				keypoints3D = _featureDetector->generateKeypoints3D(data, keypoints);
			}
		}

		data.setFeatures(keypoints, keypoints3D, descriptors);
	}
}

} // namespace rtabmap
