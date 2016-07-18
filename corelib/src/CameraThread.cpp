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

#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/utilite/ULogger.h>

#include <pcl/io/io.h>

namespace rtabmap
{

// ownership transferred
CameraThread::CameraThread(Camera * camera, const ParametersMap & parameters) :
		_camera(camera),
		_mirroring(false),
		_colorOnly(false),
		_imageDecimation(1),
		_stereoToDepth(false),
		_scanFromDepth(false),
		_scanDecimation(4),
		_scanMaxDepth(4.0f),
		_scanMinDepth(0.0f),
		_scanVoxelSize(0.0f),
		_scanNormalsK(0),
		_stereoDense(new StereoBM(parameters))
{
	UASSERT(_camera != 0);
}

CameraThread::~CameraThread()
{
	UDEBUG("");
	join(true);
	if(_camera)
	{
		delete _camera;
	}
	delete _stereoDense;
}

void CameraThread::setImageRate(float imageRate)
{
	if(_camera)
	{
		_camera->setImageRate(imageRate);
	}
}

void CameraThread::mainLoop()
{
	UDEBUG("");
	CameraInfo info;
	SensorData data = _camera->takeImage(&info);

	if(!data.imageRaw().empty())
	{
		if(_colorOnly && !data.depthRaw().empty())
		{
			data.setDepthOrRightRaw(cv::Mat());
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
				data.setImageRaw(util2d::decimate(data.imageRaw(), _imageDecimation));
				data.setDepthOrRightRaw(util2d::decimate(data.depthOrRightRaw(), _imageDecimation));
				std::vector<CameraModel> models = data.cameraModels();
				for(unsigned int i=0; i<models.size(); ++i)
				{
					if(models[i].isValidForProjection())
					{
						models[i] = models[i].scaled(1.0/double(_imageDecimation));
					}
				}
				data.setCameraModels(models);
				StereoCameraModel stereoModel = data.stereoCameraModel();
				if(stereoModel.isValidForProjection())
				{
					stereoModel.scale(1.0/double(_imageDecimation));
					data.setStereoCameraModel(stereoModel);
				}
			}
			info.timeImageDecimation = timer.ticks();
		}
		if(_mirroring && data.cameraModels().size() == 1)
		{
			UDEBUG("");
			UTimer timer;
			cv::Mat tmpRgb;
			cv::flip(data.imageRaw(), tmpRgb, 1);
			data.setImageRaw(tmpRgb);
			UASSERT_MSG(data.cameraModels().size() <= 1 && !data.stereoCameraModel().isValidForProjection(), "Only single RGBD cameras are supported for mirroring.");
			if(data.cameraModels().size() && data.cameraModels()[0].cx())
			{
				CameraModel tmpModel(
						data.cameraModels()[0].fx(),
						data.cameraModels()[0].fy(),
						float(data.imageRaw().cols) - data.cameraModels()[0].cx(),
						data.cameraModels()[0].cy(),
						data.cameraModels()[0].localTransform());
				data.setCameraModel(tmpModel);
			}
			if(!data.depthRaw().empty())
			{
				cv::Mat tmpDepth;
				cv::flip(data.depthRaw(), tmpDepth, 1);
				data.setDepthOrRightRaw(tmpDepth);
			}
			info.timeMirroring = timer.ticks();
		}
		if(_stereoToDepth && data.stereoCameraModel().isValidForProjection() && !data.rightRaw().empty())
		{
			UDEBUG("");
			UTimer timer;
			cv::Mat depth = util2d::depthFromDisparity(
					_stereoDense->computeDisparity(data.imageRaw(), data.rightRaw()),
					data.stereoCameraModel().left().fx(),
					data.stereoCameraModel().baseline());
			data.setCameraModel(data.stereoCameraModel().left());
			data.setDepthOrRightRaw(depth);
			data.setStereoCameraModel(StereoCameraModel());
			info.timeDisparity = timer.ticks();
			UDEBUG("Computing disparity = %f s", info.timeDisparity);
		}
		if(_scanFromDepth &&
			data.cameraModels().size() &&
			data.cameraModels().at(0).isValidForProjection() &&
			!data.depthRaw().empty())
		{
			UDEBUG("");
			if(data.laserScanRaw().empty())
			{
				UASSERT(_scanDecimation >= 1);
				UTimer timer;
				pcl::IndicesPtr validIndices(new std::vector<int>);
				pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = util3d::cloudFromSensorData(data, _scanDecimation, _scanMaxDepth, _scanMinDepth, validIndices.get());
				float maxPoints = (data.depthRaw().rows/_scanDecimation)*(data.depthRaw().cols/_scanDecimation);
				cv::Mat scan;
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
						pcl::PointCloud<pcl::PointXYZ>::Ptr denseCloud(new pcl::PointCloud<pcl::PointXYZ>);
						pcl::copyPointCloud(*cloud, *validIndices, *denseCloud);
						cloud = denseCloud;
					}

					if(cloud->size())
					{
						if(_scanNormalsK>0)
						{
							// view point
							Eigen::Vector3f viewPoint(0.0f,0.0f,0.0f);
							if(data.cameraModels().size() && !data.cameraModels()[0].localTransform().isNull())
							{
								viewPoint[0] = data.cameraModels()[0].localTransform().x();
								viewPoint[1] = data.cameraModels()[0].localTransform().y();
								viewPoint[2] = data.cameraModels()[0].localTransform().z();
							}
							else if(!data.stereoCameraModel().localTransform().isNull())
							{
								viewPoint[0] = data.stereoCameraModel().localTransform().x();
								viewPoint[1] = data.stereoCameraModel().localTransform().y();
								viewPoint[2] = data.stereoCameraModel().localTransform().z();
							}
							pcl::PointCloud<pcl::Normal>::Ptr normals = util3d::computeNormals(cloud, _scanNormalsK, viewPoint);
							pcl::PointCloud<pcl::PointNormal>::Ptr cloudNormals(new pcl::PointCloud<pcl::PointNormal>);
							pcl::concatenateFields(*cloud, *normals, *cloudNormals);
							scan = util3d::laserScanFromPointCloud(*cloudNormals);
						}
						else
						{
							scan = util3d::laserScanFromPointCloud(*cloud);
						}
					}
				}
				data.setLaserScanRaw(scan, (int)maxPoints, _scanMaxDepth);
				info.timeScanFromDepth = timer.ticks();
				UDEBUG("Computing scan from depth = %f s", info.timeScanFromDepth);
			}
			else
			{
				UWARN("Option to create laser scan from depth image is enabled, but "
					  "there is already a laser scan in the captured sensor data. Scan from "
					  "depth will not be created.");
			}
		}

		info.cameraName = _camera->getSerial();
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

} // namespace rtabmap
