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
		_bilateralSigmaR(0.1)
{
	UASSERT(_camera != 0);
}

CameraThread::~CameraThread()
{
	UDEBUG("");
	join(true);
	delete _camera;
	delete _distortionModel;
	delete _stereoDense;
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

void CameraThread::mainLoopBegin()
{
	ULogger::registerCurrentThread("Camera");
	_camera->resetTimer();
}

void CameraThread::mainLoop()
{
	UTimer totalTime;
	UDEBUG("");
	CameraInfo info;
	SensorData data = _camera->takeImage(&info);

	if(!data.imageRaw().empty() || (dynamic_cast<DBReader*>(_camera) != 0 && data.id()>0)) // intermediate nodes could not have image set
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
	UDEBUG("");
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
	if(_colorOnly && !data.depthRaw().empty())
	{
		data.setRGBDImage(data.imageRaw(), cv::Mat(), data.cameraModels());
	}

	if(_distortionModel && !data.depthRaw().empty())
	{
		UTimer timer;
		if(_distortionModel->getWidth() == data.depthRaw().cols &&
		   _distortionModel->getHeight() == data.depthRaw().rows	)
		{
			cv::Mat depth = data.depthRaw().clone();// make sure we are not modifying data in cached signatures.
			_distortionModel->undistort(depth);
			data.setRGBDImage(data.imageRaw(), depth, data.cameraModels());
		}
		else
		{
			UERROR("Distortion model size is %dx%d but dpeth image is %dx%d!",
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
			cv::Mat depthOrRight = util2d::decimate(data.depthOrRightRaw(), _imageDecimation);
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
			else
			{
				StereoCameraModel stereoModel = data.stereoCameraModel();
				if(stereoModel.isValidForProjection())
				{
					stereoModel.scale(1.0/double(_imageDecimation));
				}
				data.setStereoImage(image, depthOrRight, stereoModel);
			}
		}
		if(info) info->timeImageDecimation = timer.ticks();
	}
	if(_mirroring && !data.imageRaw().empty() && data.cameraModels().size() == 1)
	{
		UDEBUG("");
		UTimer timer;
		cv::Mat tmpRgb;
		cv::flip(data.imageRaw(), tmpRgb, 1);

		UASSERT_MSG(data.cameraModels().size() <= 1 && !data.stereoCameraModel().isValidForProjection(), "Only single RGBD cameras are supported for mirroring.");
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

	if(_stereoExposureCompensation && !data.imageRaw().empty() && !data.rightRaw().empty())
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
		data.setStereoImage(imgLeft, imgRight, data.stereoCameraModel());
		cv::detail::GainCompensator * gainCompensator = (cv::detail::GainCompensator*)compensator.get();
		UDEBUG("gains = %f %f ", gainCompensator->gains()[0], gainCompensator->gains()[1]);
		if(info) info->timeStereoExposureCompensation = timer.ticks();
#endif
	}

	if(_stereoToDepth && !data.imageRaw().empty() && data.stereoCameraModel().isValidForProjection() && !data.rightRaw().empty())
	{
		UDEBUG("");
		UTimer timer;
		cv::Mat depth = util2d::depthFromDisparity(
				_stereoDense->computeDisparity(data.imageRaw(), data.rightRaw()),
				data.stereoCameraModel().left().fx(),
				data.stereoCameraModel().baseline());
		// set Tx for stereo bundle adjustment (when used)
		CameraModel model = CameraModel(
				data.stereoCameraModel().left().fx(),
				data.stereoCameraModel().left().fy(),
				data.stereoCameraModel().left().cx(),
				data.stereoCameraModel().left().cy(),
				data.stereoCameraModel().localTransform(),
				-data.stereoCameraModel().baseline()*data.stereoCameraModel().left().fx(),
				data.stereoCameraModel().left().imageSize());
		data.setRGBDImage(data.imageRaw(), depth, model);
		if(info) info->timeDisparity = timer.ticks();
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
			cv::Mat scan;
			const Transform & baseToScan = data.cameraModels()[0].localTransform();
			LaserScan::Format format = LaserScan::kXYZRGB;
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
						format = LaserScan::kXYZRGBNormal;
					}
					else
					{
						scan = util3d::laserScanFromPointCloud(*cloud, baseToScan.inverse());
					}
				}
			}
			data.setLaserScan(LaserScan(scan, (int)maxPoints, _scanRangeMax, format, baseToScan));
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
}

} // namespace rtabmap
