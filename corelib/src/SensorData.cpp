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


#include "rtabmap/core/SensorData.h"
#include "rtabmap/core/Compression.h"
#include "rtabmap/core/util3d_transforms.h"
#include "rtabmap/utilite/ULogger.h"
#include <rtabmap/utilite/UMath.h>
#include <rtabmap/utilite/UConversion.h>

namespace rtabmap
{

// empty constructor
SensorData::SensorData() :
		_id(0),
		_stamp(0.0),
		_cellSize(0.0f)
{
}

// Appearance-only constructor
SensorData::SensorData(
		const cv::Mat & image,
		int id,
		double stamp,
		const cv::Mat & userData) :
		_id(id),
		_stamp(stamp),
		_cellSize(0.0f)
{
	setRGBDImage(image, cv::Mat(), CameraModel());
	setUserData(userData);
}

// Mono constructor
SensorData::SensorData(
		const cv::Mat & image,
		const CameraModel & cameraModel,
		int id,
		double stamp,
		const cv::Mat & userData) :
		_id(id),
		_stamp(stamp),
		_cellSize(0.0f)
{
	setRGBDImage(image, cv::Mat(), cameraModel);
	setUserData(userData);
}

// RGB-D constructor
SensorData::SensorData(
		const cv::Mat & rgb,
		const cv::Mat & depth,
		const CameraModel & cameraModel,
		int id,
		double stamp,
		const cv::Mat & userData) :
		_id(id),
		_stamp(stamp),
		_cellSize(0.0f)
{
	setRGBDImage(rgb, depth, cameraModel);
	setUserData(userData);
}

// RGB-D constructor + laser scan
SensorData::SensorData(
		const LaserScan & laserScan,
		const cv::Mat & rgb,
		const cv::Mat & depth,
		const CameraModel & cameraModel,
		int id,
		double stamp,
		const cv::Mat & userData) :
		_id(id),
		_stamp(stamp),
		_cellSize(0.0f)
{
	setRGBDImage(rgb, depth, cameraModel);
	setLaserScan(laserScan);
	setUserData(userData);
}

// Multi-cameras RGB-D constructor
SensorData::SensorData(
		const cv::Mat & rgb,
		const cv::Mat & depth,
		const std::vector<CameraModel> & cameraModels,
		int id,
		double stamp,
		const cv::Mat & userData) :
		_id(id),
		_stamp(stamp),
		_cellSize(0.0f)
{
	setRGBDImage(rgb, depth, cameraModels);
	setUserData(userData);
}

// Multi-cameras RGB-D constructor + laser scan
SensorData::SensorData(
		const LaserScan & laserScan,
		const cv::Mat & rgb,
		const cv::Mat & depth,
		const std::vector<CameraModel> & cameraModels,
		int id,
		double stamp,
		const cv::Mat & userData) :
		_id(id),
		_stamp(stamp),
		_cellSize(0.0f)
{
	setRGBDImage(rgb, depth, cameraModels);
	setLaserScan(laserScan);
	setUserData(userData);
}

// Stereo constructor
SensorData::SensorData(
		const cv::Mat & left,
		const cv::Mat & right,
		const StereoCameraModel & cameraModel,
		int id,
		double stamp,
		const cv::Mat & userData):
		_id(id),
		_stamp(stamp),
		_cellSize(0.0f)
{
	setStereoImage(left, right, cameraModel);
	setUserData(userData);
}

// Stereo constructor + 2d laser scan
SensorData::SensorData(
		const LaserScan & laserScan,
		const cv::Mat & left,
		const cv::Mat & right,
		const StereoCameraModel & cameraModel,
		int id,
		double stamp,
		const cv::Mat & userData) :
		_id(id),
		_stamp(stamp),
		_cellSize(0.0f)
{
	setStereoImage(left, right, cameraModel);
	setLaserScan(laserScan);
	setUserData(userData);
}

// Multi-Stereo constructor
SensorData::SensorData(
		const cv::Mat & left,
		const cv::Mat & right,
		const std::vector<StereoCameraModel> & cameraModels,
		int id,
		double stamp,
		const cv::Mat & userData):
		_id(id),
		_stamp(stamp),
		_cellSize(0.0f)
{
	setStereoImage(left, right, cameraModels);
	setUserData(userData);
}

// Multi-Stereo constructor + 2d laser scan
SensorData::SensorData(
		const LaserScan & laserScan,
		const cv::Mat & left,
		const cv::Mat & right,
		const std::vector<StereoCameraModel> & cameraModels,
		int id,
		double stamp,
		const cv::Mat & userData) :
		_id(id),
		_stamp(stamp),
		_cellSize(0.0f)
{
	setStereoImage(left, right, cameraModels);
	setLaserScan(laserScan);
	setUserData(userData);
}

SensorData::SensorData(
	const IMU & imu,
	int id,
	double stamp) :
		_id(id),
		_stamp(stamp),
		_cellSize(0.0f)
{
	imu_ = imu;
}

SensorData::~SensorData()
{
}

void SensorData::setRGBDImage(
		const cv::Mat & rgb,
		const cv::Mat & depth,
		const CameraModel & model,
		bool clearPreviousData)
{
	std::vector<CameraModel> models;
	models.push_back(model);
	setRGBDImage(rgb, depth, models, clearPreviousData);
}
void SensorData::setRGBDImage(
		const cv::Mat & rgb,
		const cv::Mat & depth,
		const std::vector<CameraModel> & models,
		bool clearPreviousData)
{
	if(!clearPreviousData && !_stereoCameraModels.empty())
	{
		UERROR("Sensor data has previously stereo images "
				"but clearPreviousData parameter is false. We "
				"will still clear previous data to avoid incompatibilities "
				"between raw and compressed data!");
	}
	bool clearData = clearPreviousData || !_stereoCameraModels.empty();

	_stereoCameraModels.clear();
	_cameraModels = models;
	if(rgb.rows == 1)
	{
		UASSERT(rgb.type() == CV_8UC1); // Bytes
		_imageCompressed = rgb;
		if(clearData)
		{
			_imageRaw = cv::Mat();
		}
	}
	else if(!rgb.empty())
	{
		UASSERT(rgb.type() == CV_8UC1 || // Mono
				rgb.type() == CV_8UC3);  // RGB
		_imageRaw = rgb;
		if(clearData)
		{
			_imageCompressed = cv::Mat();
		}
	}
	else if(clearData)
	{
		_imageRaw = cv::Mat();
		_imageCompressed = cv::Mat();
	}

	if(depth.rows == 1)
	{
		UASSERT(depth.type() == CV_8UC1); // Bytes
		_depthOrRightCompressed = depth;
		if(clearData)
		{
			_depthOrRightRaw = cv::Mat();
		}
	}
	else if(!depth.empty())
	{
		UASSERT(depth.type() == CV_32FC1 || // Depth in meter
				depth.type() == CV_16UC1); // Depth in millimetre
		_depthOrRightRaw = depth;
		if(clearData)
		{
			_depthOrRightCompressed = cv::Mat();
		}
	}
	else if(clearData)
	{
		_depthOrRightRaw = cv::Mat();
		_depthOrRightCompressed = cv::Mat();
	}
}
void SensorData::setStereoImage(
		const cv::Mat & left,
		const cv::Mat & right,
		const StereoCameraModel & stereoCameraModel,
		bool clearPreviousData)
{
	std::vector<StereoCameraModel> models;
	models.push_back(stereoCameraModel);
	setStereoImage(left, right, models, clearPreviousData);
}
void SensorData::setStereoImage(
		const cv::Mat & left,
		const cv::Mat & right,
		const std::vector<StereoCameraModel> & stereoCameraModels,
		bool clearPreviousData)
{
	if(!clearPreviousData && !_cameraModels.empty())
	{
		UERROR("Sensor data has previously RGB-D/RGB images "
				"but clearPreviousData parameter is false. We "
				"will still clear previous data to avoid incompatibilities "
				"between raw and compressed data!");
	}
	bool clearData = clearPreviousData || !_cameraModels.empty();

	_cameraModels.clear();
	_stereoCameraModels = stereoCameraModels;

	if(left.rows == 1)
	{
		UASSERT(left.type() == CV_8UC1); // Bytes
		_imageCompressed = left;
		if(clearData)
		{
			_imageRaw = cv::Mat();
		}
	}
	else if(!left.empty())
	{
		UASSERT(left.type() == CV_8UC1 || // Mono
				left.type() == CV_8UC3);  // RGB
		_imageRaw = left;
		if(clearData)
		{
			_imageCompressed = cv::Mat();
		}
	}
	else if(clearData)
	{
		_imageRaw = cv::Mat();
		_imageCompressed = cv::Mat();
	}

	if(right.rows == 1)
	{
		UASSERT(right.type() == CV_8UC1); // Bytes
		_depthOrRightCompressed = right;
		if(clearData)
		{
			_depthOrRightRaw = cv::Mat();
		}
	}
	else if(!right.empty())
	{
		UASSERT(right.type() == CV_8UC1); // Mono
		_depthOrRightRaw = right;
		if(clearData)
		{
			_depthOrRightCompressed = cv::Mat();
		}
	}
	else if(clearData)
	{
		_depthOrRightRaw = cv::Mat();
		_depthOrRightCompressed = cv::Mat();
	}
}
void SensorData::setLaserScan(const LaserScan & laserScan, bool clearPreviousData)
{
	if(!laserScan.isCompressed())
	{
		_laserScanRaw = laserScan;
		if(clearPreviousData)
		{
			_laserScanCompressed = LaserScan();
		}
	}
	else
	{
		_laserScanCompressed = laserScan;
		if(clearPreviousData)
		{
			_laserScanRaw = LaserScan();
		}
	}
}

void SensorData::setImageRaw(const cv::Mat & image)
{
	UASSERT(image.empty() || image.rows > 1);
	_imageRaw = image;
}
void SensorData::setDepthOrRightRaw(const cv::Mat & image)
{
	UASSERT(image.empty() || image.rows > 1);
	_depthOrRightRaw = image;
}
void SensorData::setLaserScanRaw(const LaserScan & scan)
{
	UASSERT(scan.isEmpty() || !scan.isCompressed());
	_laserScanRaw = scan;
}

void SensorData::setUserDataRaw(const cv::Mat & userDataRaw)
{
	_userDataRaw = userDataRaw;
}

void SensorData::setUserData(const cv::Mat & userData, bool clearPreviousData)
{
	if(clearPreviousData)
	{
		_userDataRaw = cv::Mat();
		_userDataCompressed = cv::Mat();
	}

	if(userData.type() == CV_8UC1 &&  userData.rows == 1 && userData.cols > int(3*sizeof(int))) // Bytes
	{
		_userDataCompressed = userData; // assume compressed
	}
	else
	{
		_userDataRaw = userData;
		if(!userData.empty())
		{
			_userDataCompressed = compressData2(userData);
		}
	}
}

void SensorData::setOccupancyGrid(
			const cv::Mat & ground,
			const cv::Mat & obstacles,
			const cv::Mat & empty,
			float cellSize,
			const cv::Point3f & viewPoint)
{
	UDEBUG("ground=%d obstacles=%d empty=%d", ground.cols, obstacles.cols, empty.cols);
	if((!ground.empty() && (!_groundCellsCompressed.empty() || !_groundCellsRaw.empty())) ||
	   (!obstacles.empty() && (!_obstacleCellsCompressed.empty() || !_obstacleCellsRaw.empty())) ||
	   (!empty.empty() && (!_emptyCellsCompressed.empty() || !_emptyCellsRaw.empty())))
	{
		UWARN("Occupancy grid cannot be overwritten! id=%d, Set occupancy grid of %d to null "
			  "before setting a new one.", this->id());
		return;
	}

	_groundCellsRaw = cv::Mat();
	_groundCellsCompressed = cv::Mat();
	_obstacleCellsRaw = cv::Mat();
	_obstacleCellsCompressed = cv::Mat();
	_emptyCellsRaw = cv::Mat();
	_emptyCellsCompressed = cv::Mat();

	CompressionThread ctGround(ground);
	CompressionThread ctObstacles(obstacles);
	CompressionThread ctEmpty(empty);

	if(!ground.empty())
	{
		if(ground.type() == CV_32FC2 || ground.type() == CV_32FC3 || ground.type() == CV_32FC(4) || ground.type() == CV_32FC(5) || ground.type() == CV_32FC(6) || ground.type() == CV_32FC(7))
		{
			_groundCellsRaw = ground;
			ctGround.start();
		}
		else if(ground.type() == CV_8UC1)
		{
			_groundCellsCompressed = ground;
		}
	}
	if(!obstacles.empty())
	{
		if(obstacles.type() == CV_32FC2 || obstacles.type() == CV_32FC3 || obstacles.type() == CV_32FC(4) || obstacles.type() == CV_32FC(5) || obstacles.type() == CV_32FC(6) || obstacles.type() == CV_32FC(7))
		{
			_obstacleCellsRaw = obstacles;
			ctObstacles.start();
		}
		else if(obstacles.type() == CV_8UC1)
		{
			_obstacleCellsCompressed = obstacles;
		}
	}
	if(!empty.empty())
	{
		if(empty.type() == CV_32FC2 || empty.type() == CV_32FC3 || empty.type() == CV_32FC(4) || empty.type() == CV_32FC(5) || empty.type() == CV_32FC(6) || empty.type() == CV_32FC(7))
		{
			_emptyCellsRaw = empty;
			ctEmpty.start();
		}
		else if(empty.type() == CV_8UC1)
		{
			_emptyCellsCompressed = empty;
		}
	}
	ctGround.join();
	ctObstacles.join();
	ctEmpty.join();
	if(!_groundCellsRaw.empty())
	{
		_groundCellsCompressed = ctGround.getCompressedData();
	}
	if(!_obstacleCellsRaw.empty())
	{
		_obstacleCellsCompressed = ctObstacles.getCompressedData();
	}
	if(!_emptyCellsRaw.empty())
	{
		_emptyCellsCompressed = ctEmpty.getCompressedData();
	}

	_cellSize = cellSize;
	_viewPoint = viewPoint;
}

void SensorData::uncompressData()
{
	cv::Mat tmpA, tmpB, tmpD, tmpE, tmpF, tmpG;
	LaserScan tmpC;
	uncompressData(_imageCompressed.empty()?0:&tmpA,
				_depthOrRightCompressed.empty()?0:&tmpB,
				_laserScanCompressed.isEmpty()?0:&tmpC,
				_userDataCompressed.empty()?0:&tmpD,
				_groundCellsCompressed.empty()?0:&tmpE,
				_obstacleCellsCompressed.empty()?0:&tmpF,
				_emptyCellsCompressed.empty()?0:&tmpG);
}

void SensorData::uncompressData(
		cv::Mat * imageRaw,
		cv::Mat * depthRaw,
		LaserScan * laserScanRaw,
		cv::Mat * userDataRaw,
		cv::Mat * groundCellsRaw,
		cv::Mat * obstacleCellsRaw,
		cv::Mat * emptyCellsRaw)
{
	UDEBUG("%d data(%d,%d,%d,%d,%d,%d,%d)", this->id(), imageRaw?1:0, depthRaw?1:0, laserScanRaw?1:0, userDataRaw?1:0, groundCellsRaw?1:0, obstacleCellsRaw?1:0, emptyCellsRaw?1:0);
	if(imageRaw == 0 &&
		depthRaw == 0 &&
		laserScanRaw == 0 &&
		userDataRaw == 0 &&
		groundCellsRaw == 0 &&
		obstacleCellsRaw == 0 &&
		emptyCellsRaw == 0)
	{
		return;
	}
	uncompressDataConst(
			imageRaw,
			depthRaw,
			laserScanRaw,
			userDataRaw,
			groundCellsRaw,
			obstacleCellsRaw,
			emptyCellsRaw);

	if(imageRaw && !imageRaw->empty() && _imageRaw.empty())
	{
		_imageRaw = *imageRaw;
		//backward compatibility, set image size in camera model if not set
		if(!_imageRaw.empty() && _cameraModels.size())
		{
			cv::Size size(_imageRaw.cols/_cameraModels.size(), _imageRaw.rows);
			for(unsigned int i=0; i<_cameraModels.size(); ++i)
			{
				if(_cameraModels[i].fx() && _cameraModels[i].fy() && _cameraModels[i].imageWidth() == 0)
				{
					_cameraModels[i].setImageSize(size);
				}
			}
		}
	}
	if(depthRaw && !depthRaw->empty() && _depthOrRightRaw.empty())
	{
		_depthOrRightRaw = *depthRaw;
	}
	if(laserScanRaw && !laserScanRaw->isEmpty() && _laserScanRaw.isEmpty())
	{
		_laserScanRaw = *laserScanRaw;
		if(_laserScanCompressed.format() == LaserScan::kUnknown)
		{
			if(_laserScanCompressed.angleIncrement() > 0.0f)
			{
				_laserScanCompressed = LaserScan(_laserScanCompressed.data(), _laserScanRaw.format(), _laserScanCompressed.rangeMin(), _laserScanCompressed.rangeMax(), _laserScanCompressed.angleMin(), _laserScanCompressed.angleMax(), _laserScanCompressed.angleIncrement(), _laserScanCompressed.localTransform());
			}
			else
			{
				_laserScanCompressed = LaserScan(_laserScanCompressed.data(), _laserScanCompressed.maxPoints(), _laserScanCompressed.rangeMax(), _laserScanRaw.format(), _laserScanCompressed.localTransform());
			}
		}
	}
	if(userDataRaw && !userDataRaw->empty() && _userDataRaw.empty())
	{
		_userDataRaw = *userDataRaw;
	}
	if(groundCellsRaw && !groundCellsRaw->empty() && _groundCellsRaw.empty())
	{
		_groundCellsRaw = *groundCellsRaw;
	}
	if(obstacleCellsRaw && !obstacleCellsRaw->empty() && _obstacleCellsRaw.empty())
	{
		_obstacleCellsRaw = *obstacleCellsRaw;
	}
	if(emptyCellsRaw && !emptyCellsRaw->empty() && _emptyCellsRaw.empty())
	{
		_emptyCellsRaw = *emptyCellsRaw;
	}
}

void SensorData::uncompressDataConst(
		cv::Mat * imageRaw,
		cv::Mat * depthRaw,
		LaserScan * laserScanRaw,
		cv::Mat * userDataRaw,
		cv::Mat * groundCellsRaw,
		cv::Mat * obstacleCellsRaw,
		cv::Mat * emptyCellsRaw) const
{
	if(imageRaw)
	{
		*imageRaw = _imageRaw;
	}
	if(depthRaw)
	{
		*depthRaw = _depthOrRightRaw;
	}
	if(laserScanRaw)
	{
		*laserScanRaw = _laserScanRaw;
	}
	if(userDataRaw)
	{
		*userDataRaw = _userDataRaw;
	}
	if(groundCellsRaw)
	{
		*groundCellsRaw = _groundCellsRaw;
	}
	if(obstacleCellsRaw)
	{
		*obstacleCellsRaw = _obstacleCellsRaw;
	}
	if(emptyCellsRaw)
	{
		*emptyCellsRaw = _emptyCellsRaw;
	}
	if( (imageRaw && imageRaw->empty()) ||
		(depthRaw && depthRaw->empty()) ||
		(laserScanRaw && laserScanRaw->isEmpty()) ||
		(userDataRaw && userDataRaw->empty()) ||
		(groundCellsRaw && groundCellsRaw->empty()) ||
		(obstacleCellsRaw && obstacleCellsRaw->empty()) ||
		(emptyCellsRaw && emptyCellsRaw->empty()))
	{
		rtabmap::CompressionThread ctImage(_imageCompressed, true);
		rtabmap::CompressionThread ctDepth(_depthOrRightCompressed, true);
		rtabmap::CompressionThread ctLaserScan(_laserScanCompressed.data(), false);
		rtabmap::CompressionThread ctUserData(_userDataCompressed, false);
		rtabmap::CompressionThread ctGroundCells(_groundCellsCompressed, false);
		rtabmap::CompressionThread ctObstacleCells(_obstacleCellsCompressed, false);
		rtabmap::CompressionThread ctEmptyCells(_emptyCellsCompressed, false);
		if(imageRaw && imageRaw->empty() && !_imageCompressed.empty())
		{
			UASSERT(_imageCompressed.type() == CV_8UC1);
			ctImage.start();
		}
		if(depthRaw && depthRaw->empty() && !_depthOrRightCompressed.empty())
		{
			UASSERT(_depthOrRightCompressed.type() == CV_8UC1);
			ctDepth.start();
		}
		if(laserScanRaw && laserScanRaw->isEmpty() && !_laserScanCompressed.isEmpty())
		{
			UASSERT(_laserScanCompressed.isCompressed());
			ctLaserScan.start();
		}
		if(userDataRaw && userDataRaw->empty() && !_userDataCompressed.empty())
		{
			UASSERT(_userDataCompressed.type() == CV_8UC1);
			ctUserData.start();
		}
		if(groundCellsRaw && groundCellsRaw->empty() && !_groundCellsCompressed.empty())
		{
			UASSERT(_groundCellsCompressed.type() == CV_8UC1);
			ctGroundCells.start();
		}
		if(obstacleCellsRaw && obstacleCellsRaw->empty() && !_obstacleCellsCompressed.empty())
		{
			UASSERT(_obstacleCellsCompressed.type() == CV_8UC1);
			ctObstacleCells.start();
		}
		if(emptyCellsRaw && emptyCellsRaw->empty() && !_emptyCellsCompressed.empty())
		{
			UASSERT(_emptyCellsCompressed.type() == CV_8UC1);
			ctEmptyCells.start();
		}
		ctImage.join();
		ctDepth.join();
		ctLaserScan.join();
		ctUserData.join();
		ctGroundCells.join();
		ctObstacleCells.join();
		ctEmptyCells.join();

		if(imageRaw && imageRaw->empty())
		{
			*imageRaw = ctImage.getUncompressedData();
			if(imageRaw->empty())
			{
				if(_imageCompressed.empty())
				{
					UWARN("Requested raw image data, but the sensor data (%d) doesn't have image.", this->id());
				}
				else
				{
					UERROR("Requested image data, but failed to uncompress (%d).", this->id());
				}
			}
		}
		if(depthRaw && depthRaw->empty())
		{
			*depthRaw = ctDepth.getUncompressedData();
			if(depthRaw->empty())
			{
				if(_depthOrRightCompressed.empty())
				{
					UWARN("Requested depth/right image data, but the sensor data (%d) doesn't have depth/right image.", this->id());
				}
				else
				{
					UERROR("Requested depth/right image data, but failed to uncompress (%d).", this->id());
				}
			}
		}
		if(laserScanRaw && laserScanRaw->isEmpty())
		{
			if(_laserScanCompressed.angleIncrement() > 0.0f)
			{
				*laserScanRaw = LaserScan(ctLaserScan.getUncompressedData(), _laserScanCompressed.format(), _laserScanCompressed.rangeMin(), _laserScanCompressed.rangeMax(), _laserScanCompressed.angleMin(), _laserScanCompressed.angleMax(), _laserScanCompressed.angleIncrement(), _laserScanCompressed.localTransform());
			}
			else
			{
				*laserScanRaw = LaserScan(ctLaserScan.getUncompressedData(), _laserScanCompressed.maxPoints(), _laserScanCompressed.rangeMax(), _laserScanCompressed.format(), _laserScanCompressed.localTransform());
			}
			if(laserScanRaw->isEmpty())
			{
				if(_laserScanCompressed.isEmpty())
				{
					UWARN("Requested laser scan data, but the sensor data (%d) doesn't have laser scan.", this->id());
				}
				else
				{
					UERROR("Requested laser scan data, but failed to uncompress (%d).", this->id());
				}
			}
		}
		if(userDataRaw && userDataRaw->empty())
		{
			*userDataRaw = ctUserData.getUncompressedData();

			if(userDataRaw->empty())
			{
				if(_userDataCompressed.empty())
				{
					UWARN("Requested user data, but the sensor data (%d) doesn't have user data.", this->id());
				}
				else
				{
					UERROR("Requested user data, but failed to uncompress (%d).", this->id());
				}
			}
		}
		if(groundCellsRaw && groundCellsRaw->empty())
		{
			*groundCellsRaw = ctGroundCells.getUncompressedData();
		}
		if(obstacleCellsRaw && obstacleCellsRaw->empty())
		{
			*obstacleCellsRaw = ctObstacleCells.getUncompressedData();
		}
		if(emptyCellsRaw && emptyCellsRaw->empty())
		{
			*emptyCellsRaw = ctEmptyCells.getUncompressedData();
		}
	}
}

void SensorData::setFeatures(const std::vector<cv::KeyPoint> & keypoints, const std::vector<cv::Point3f> & keypoints3D, const cv::Mat & descriptors)
{
	UASSERT_MSG(keypoints3D.empty() || keypoints.size() == keypoints3D.size(), uFormat("keypoints=%d keypoints3D=%d", (int)keypoints.size(), (int)keypoints3D.size()).c_str());
	UASSERT_MSG(descriptors.empty() || (int)keypoints.size() == descriptors.rows, uFormat("keypoints=%d descriptors=%d", (int)keypoints.size(), descriptors.rows).c_str());
	_keypoints = keypoints;
	_keypoints3D = keypoints3D;
	_descriptors = descriptors;
}

unsigned long SensorData::getMemoryUsed() const // Return memory usage in Bytes
{
	return sizeof(SensorData) +
			(_imageCompressed.empty()?0:_imageCompressed.total()*_imageCompressed.elemSize()) +
			(_imageRaw.empty()?0:_imageRaw.total()*_imageRaw.elemSize()) +
			(_depthOrRightCompressed.empty()?0:_depthOrRightCompressed.total()*_depthOrRightCompressed.elemSize()) +
			(_depthOrRightRaw.empty()?0:_depthOrRightRaw.total()*_depthOrRightRaw.elemSize()) +
			(_userDataCompressed.empty()?0:_userDataCompressed.total()*_userDataCompressed.elemSize()) +
			(_userDataRaw.empty()?0:_userDataRaw.total()*_userDataRaw.elemSize()) +
			(_laserScanCompressed.empty()?0:_laserScanCompressed.data().total()*_laserScanCompressed.data().elemSize()) +
			(_laserScanRaw.empty()?0:_laserScanRaw.data().total()*_laserScanRaw.data().elemSize()) +
			(_groundCellsCompressed.empty()?0:_groundCellsCompressed.total()*_groundCellsCompressed.elemSize()) +
			(_groundCellsRaw.empty()?0:_groundCellsRaw.total()*_groundCellsRaw.elemSize()) +
			(_obstacleCellsCompressed.empty()?0:_obstacleCellsCompressed.total()*_obstacleCellsCompressed.elemSize()) +
			(_obstacleCellsRaw.empty()?0:_obstacleCellsRaw.total()*_obstacleCellsRaw.elemSize())+
			(_emptyCellsCompressed.empty()?0:_emptyCellsCompressed.total()*_emptyCellsCompressed.elemSize()) +
			(_emptyCellsRaw.empty()?0:_emptyCellsRaw.total()*_emptyCellsRaw.elemSize())+
			_keypoints.size() * sizeof(cv::KeyPoint) +
			_keypoints3D.size() * sizeof(cv::Point3f) +
			(_descriptors.empty()?0:_descriptors.total()*_descriptors.elemSize());
}

void SensorData::clearCompressedData(bool images, bool scan, bool userData)
{
	if(images)
	{
		_imageCompressed=cv::Mat();
		_depthOrRightCompressed=cv::Mat();
	}
	if(scan)
	{
		_laserScanCompressed.clear();
	}
	if(userData)
	{
		_userDataCompressed=cv::Mat();
	}
}
void SensorData::clearRawData(bool images, bool scan, bool userData)
{
	if(images)
	{
		_imageRaw=cv::Mat();
		_depthOrRightRaw=cv::Mat();
	}
	if(scan)
	{
		_laserScanRaw.clear();
	}
	if(userData)
	{
		_userDataRaw=cv::Mat();
	}
}


bool SensorData::isPointVisibleFromCameras(const cv::Point3f & pt) const
{
	if(_cameraModels.size() >= 1)
	{
		for(unsigned int i=0; i<_cameraModels.size(); ++i)
		{
			if(_cameraModels[i].isValidForProjection() && !_cameraModels[i].localTransform().isNull())
			{
				cv::Point3f ptInCameraFrame = util3d::transformPoint(pt, _cameraModels[i].localTransform().inverse());
				if(ptInCameraFrame.z > 0.0f)
				{
					int borderWidth = int(float(_cameraModels[i].imageWidth())* 0.2);
					int u, v;
					_cameraModels[i].reproject(ptInCameraFrame.x, ptInCameraFrame.y, ptInCameraFrame.z, u, v);
					if(uIsInBounds(u, borderWidth, _cameraModels[i].imageWidth()-2*borderWidth) &&
					   uIsInBounds(v, borderWidth, _cameraModels[i].imageHeight()-2*borderWidth))
					{
						return true;
					}
				}
			}
		}
	}
	else if(_stereoCameraModels.size() >= 1)
	{
		for(unsigned int i=0; i<_stereoCameraModels.size(); ++i)
		{
			if(_stereoCameraModels[i].isValidForProjection() && !_stereoCameraModels[i].localTransform().isNull())
			{
				cv::Point3f ptInCameraFrame = util3d::transformPoint(pt, _stereoCameraModels[i].localTransform().inverse());
				if(ptInCameraFrame.z > 0.0f)
				{
					int u, v;
					_stereoCameraModels[i].left().reproject(ptInCameraFrame.x, ptInCameraFrame.y, ptInCameraFrame.z, u, v);
					if(uIsInBounds(u, 0, _stereoCameraModels[i].left().imageWidth()) &&
					   uIsInBounds(v, 0, _stereoCameraModels[i].left().imageHeight()))
					{
						return true;
					}
				}
			}
		}
	}
	else
	{
		UERROR("no valid camera model!");
	}
	return false;
}

} // namespace rtabmap

