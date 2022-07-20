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

#include "rtabmap/core/CameraThread.h"
#include "rtabmap/core/Camera.h"
#include "rtabmap/core/CameraEvent.h"
#include "rtabmap/core/CameraRGBD.h"
#include "rtabmap/core/util2d.h"
#include "rtabmap/core/util3d.h"
#include "rtabmap/core/util3d_surface.h"
#include "rtabmap/core/util3d_filtering.h"
#include "rtabmap/core/StereoDense.h"
#include "rtabmap/core/DBReader.h"
#include "rtabmap/core/IMUFilter.h"
#include "rtabmap/core/clams/discrete_depth_distortion_model.h"
#include <opencv2/stitching/detail/exposure_compensate.hpp>
#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/utilite/ULogger.h>

#include <pcl/io/io.h>

namespace rtabmap
{

// ownership transferred
CameraThread::CameraThread(Camera * camera, const ParametersMap & parameters) :
		_camera(camera),
		_odomSensor(0),
		_odomAsGt(false),
		_poseTimeOffset(0.0),
		_poseScaleFactor(1.0f),
		_mirroring(false),
		_stereoExposureCompensation(false),
		_colorOnly(false),
		_imageDecimation(1),
		_stereoToDepth(false),
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
		_imuBaseFrameConversion(false)
{
	UASSERT(_camera != 0);
}

// ownership transferred
CameraThread::CameraThread(
		Camera * camera,
		Camera * odomSensor,
		const Transform & extrinsics,
		double poseTimeOffset,
		float poseScaleFactor,
		bool odomAsGt,
		const ParametersMap & parameters) :
			_camera(camera),
			_odomSensor(odomSensor),
			_extrinsicsOdomToCamera(extrinsics * CameraModel::opticalRotation()),
			_odomAsGt(odomAsGt),
			_poseTimeOffset(poseTimeOffset),
			_poseScaleFactor(poseScaleFactor),
			_mirroring(false),
			_stereoExposureCompensation(false),
			_colorOnly(false),
			_imageDecimation(1),
			_stereoToDepth(false),
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
			_imuBaseFrameConversion(false)
{
	UASSERT(_camera != 0 && _odomSensor != 0 && !_extrinsicsOdomToCamera.isNull());
	UDEBUG("_extrinsicsOdomToCamera=%s", _extrinsicsOdomToCamera.prettyPrint().c_str());
	UDEBUG("_poseTimeOffset        =%f", _poseTimeOffset);
	UDEBUG("_poseScaleFactor       =%f", _poseScaleFactor);
	UDEBUG("_odomAsGt              =%s", _odomAsGt?"true":"false");
}

// ownership transferred
CameraThread::CameraThread(
		Camera * camera,
		bool odomAsGt,
		const ParametersMap & parameters) :
			_camera(camera),
			_odomSensor(0),
			_odomAsGt(odomAsGt),
			_poseTimeOffset(0.0),
			_poseScaleFactor(1.0f),
			_mirroring(false),
			_stereoExposureCompensation(false),
			_colorOnly(false),
			_imageDecimation(1),
			_stereoToDepth(false),
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
			_imuBaseFrameConversion(false)
{
	UASSERT(_camera != 0);
	UDEBUG("_odomAsGt              =%s", _odomAsGt?"true":"false");
}

CameraThread::~CameraThread()
{
	join(true);
	delete _camera;
	delete _odomSensor;
	delete _distortionModel;
	delete _stereoDense;
	delete _imuFilter;
}

void CameraThread::setImageRate(float imageRate)
{
	if(_camera)
	{
		_camera->setImageRate(imageRate);
	}
}

void CameraThread::setDistortionModel(const std::string & path)
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

void CameraThread::enableBilateralFiltering(float sigmaS, float sigmaR)
{
	UASSERT(sigmaS > 0.0f && sigmaR > 0.0f);
	_bilateralFiltering = true;
	_bilateralSigmaS = sigmaS;
	_bilateralSigmaR = sigmaR;
}

void CameraThread::enableIMUFiltering(int filteringStrategy, const ParametersMap & parameters, bool baseFrameConversion)
{
	delete _imuFilter;
	_imuFilter = IMUFilter::create((IMUFilter::Type)filteringStrategy, parameters);
	_imuBaseFrameConversion = baseFrameConversion;
}

void CameraThread::disableIMUFiltering()
{
	delete _imuFilter;
	_imuFilter = 0;
}

void CameraThread::setScanParameters(
	bool fromDepth,
	int downsampleStep,
	float rangeMin,
	float rangeMax,
	float voxelSize,
	int normalsK,
	int normalsRadius,
	bool forceGroundNormalsUp)
{
	setScanParameters(fromDepth, downsampleStep, rangeMin, rangeMax, voxelSize, normalsK, normalsRadius, forceGroundNormalsUp?0.8f:0.0f);
}

void CameraThread::setScanParameters(
			bool fromDepth,
			int downsampleStep, // decimation of the depth image in case the scan is from depth image
			float rangeMin,
			float rangeMax,
			float voxelSize,
			int normalsK,
			int normalsRadius,
			float groundNormalsUp)
{
	_scanFromDepth = fromDepth;
	_scanDownsampleStep=downsampleStep;
	_scanRangeMin = rangeMin;
	_scanRangeMax = rangeMax;
	_scanVoxelSize = voxelSize;
	_scanNormalsK = normalsK;
	_scanNormalsRadius = normalsRadius;
	_scanForceGroundNormalsUp = groundNormalsUp;
}

bool CameraThread::odomProvided() const
{
	return _camera && (_camera->odomProvided() || (_odomSensor && _odomSensor->odomProvided()));
}

void CameraThread::mainLoopBegin()
{
	ULogger::registerCurrentThread("Camera");
	_camera->resetTimer();
}

void CameraThread::mainLoop()
{
	UTimer totalTime;
	CameraInfo info;
	SensorData data = _camera->takeImage(&info);

	if(_odomSensor)
	{
		Transform pose;
		Transform poseToLeftCam;
		cv::Mat covariance;
		if(_odomSensor->getPose(data.stamp()+_poseTimeOffset, pose, covariance))
		{
			info.odomPose = pose;
			info.odomCovariance = covariance;
			if(_poseScaleFactor>0 && _poseScaleFactor!=1.0f)
			{
				info.odomPose.x() *= _poseScaleFactor;
				info.odomPose.y() *= _poseScaleFactor;
				info.odomPose.z() *= _poseScaleFactor;
			}
			// Adjust local transform of the camera based on the pose frame
			if(!data.cameraModels().empty())
			{
				UASSERT(data.cameraModels().size()==1);
				CameraModel model = data.cameraModels()[0];
				model.setLocalTransform(_extrinsicsOdomToCamera);
				data.setCameraModel(model);
			}
			else if(!data.stereoCameraModels().empty())
			{
				UASSERT(data.stereoCameraModels().size()==1);
				StereoCameraModel model = data.stereoCameraModels()[0];
				model.setLocalTransform(_extrinsicsOdomToCamera);
				data.setStereoCameraModel(model);
			}
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

	if(!data.imageRaw().empty() || !data.laserScanRaw().empty() || (dynamic_cast<DBReader*>(_camera) != 0 && data.id()>0)) // intermediate nodes could not have image set
	{
		postUpdate(&data, &info);
		info.cameraName = _camera->getSerial();
		info.timeTotal = totalTime.ticks();
		this->post(new CameraEvent(data, info));
	}
	else if(!this->isKilled())
	{
		UWARN("no more images...");
		this->kill();
		this->post(new CameraEvent());
	}
}

void CameraThread::mainLoopKill()
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

void CameraThread::postUpdate(SensorData * dataPtr, CameraInfo * info) const
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
}

} // namespace rtabmap
