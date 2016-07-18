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
#include "rtabmap/utilite/ULogger.h"
#include <rtabmap/utilite/UMath.h>

namespace rtabmap
{

// empty constructor
SensorData::SensorData() :
		_id(0),
		_stamp(0.0),
		_laserScanMaxPts(0),
		_laserScanMaxRange(0.0f)
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
		_laserScanMaxPts(0),
		_laserScanMaxRange(0.0f)
{
	if(image.rows == 1)
	{
		UASSERT(image.type() == CV_8UC1); // Bytes
		_imageCompressed = image;
	}
	else if(!image.empty())
	{
		UASSERT(image.type() == CV_8UC1 || // Mono
				image.type() == CV_8UC3);  // RGB
		_imageRaw = image;
	}

	if(userData.type() == CV_8UC1) // Bytes
	{
		_userDataCompressed = userData; // assume compressed
	}
	else
	{
		_userDataRaw = userData;
	}
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
		_laserScanMaxPts(0),
		_laserScanMaxRange(0.0f),
		_cameraModels(std::vector<CameraModel>(1, cameraModel))
{
	if(image.rows == 1)
	{
		UASSERT(image.type() == CV_8UC1); // Bytes
		_imageCompressed = image;
	}
	else if(!image.empty())
	{
		UASSERT(image.type() == CV_8UC1 || // Mono
				image.type() == CV_8UC3);  // RGB
		_imageRaw = image;
	}

	if(userData.type() == CV_8UC1) // Bytes
	{
		_userDataCompressed = userData; // assume compressed
	}
	else
	{
		_userDataRaw = userData;
	}
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
		_laserScanMaxPts(0),
		_laserScanMaxRange(0.0f),
		_cameraModels(std::vector<CameraModel>(1, cameraModel))
{
	if(rgb.rows == 1)
	{
		UASSERT(rgb.type() == CV_8UC1); // Bytes
		_imageCompressed = rgb;
	}
	else if(!rgb.empty())
	{
		UASSERT(rgb.type() == CV_8UC1 || // Mono
				rgb.type() == CV_8UC3);  // RGB
		_imageRaw = rgb;
	}

	if(depth.rows == 1)
	{
		UASSERT(depth.type() == CV_8UC1); // Bytes
		_depthOrRightCompressed = depth;
	}
	else if(!depth.empty())
	{
		UASSERT(depth.type() == CV_32FC1 || // Depth in meter
				depth.type() == CV_16UC1); // Depth in millimetre
		_depthOrRightRaw = depth;
	}

	if(userData.type() == CV_8UC1) // Bytes
	{
		_userDataCompressed = userData; // assume compressed
	}
	else
	{
		_userDataRaw = userData;
	}
}

// RGB-D constructor + laser scan
SensorData::SensorData(
		const cv::Mat & laserScan,
		int laserScanMaxPts,
		float laserScanMaxRange,
		const cv::Mat & rgb,
		const cv::Mat & depth,
		const CameraModel & cameraModel,
		int id,
		double stamp,
		const cv::Mat & userData) :
		_id(id),
		_stamp(stamp),
		_laserScanMaxPts(laserScanMaxPts),
		_laserScanMaxRange(laserScanMaxRange),
		_cameraModels(std::vector<CameraModel>(1, cameraModel))
{
	if(rgb.rows == 1)
	{
		UASSERT(rgb.type() == CV_8UC1); // Bytes
		_imageCompressed = rgb;
	}
	else if(!rgb.empty())
	{
		UASSERT(rgb.type() == CV_8UC1 || // Mono
				rgb.type() == CV_8UC3);  // RGB
		_imageRaw = rgb;
	}
	if(depth.rows == 1)
	{
		UASSERT(depth.type() == CV_8UC1); // Bytes
		_depthOrRightCompressed = depth;
	}
	else if(!depth.empty())
	{
		UASSERT(depth.type() == CV_32FC1 || // Depth in meter
				depth.type() == CV_16UC1); // Depth in millimetre
		_depthOrRightRaw = depth;
	}

	if(laserScan.type() == CV_32FC2 || laserScan.type() == CV_32FC3 || laserScan.type() == CV_32FC(6))
	{
		_laserScanRaw = laserScan;
	}
	else if(!laserScan.empty())
	{
		UASSERT(laserScan.type() == CV_8UC1); // Bytes
		_laserScanCompressed = laserScan;
	}

	if(userData.type() == CV_8UC1) // Bytes
	{
		_userDataCompressed = userData; // assume compressed
	}
	else
	{
		_userDataRaw = userData;
	}
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
		_laserScanMaxPts(0),
		_laserScanMaxRange(0.0f),
		_cameraModels(cameraModels)
{
	if(rgb.rows == 1)
	{
		UASSERT(rgb.type() == CV_8UC1); // Bytes
		_imageCompressed = rgb;
	}
	else if(!rgb.empty())
	{
		UASSERT(rgb.type() == CV_8UC1 || // Mono
				rgb.type() == CV_8UC3);  // RGB
		_imageRaw = rgb;
	}
	if(depth.rows == 1)
	{
		UASSERT(depth.type() == CV_8UC1); // Bytes
		_depthOrRightCompressed = depth;
	}
	else if(!depth.empty())
	{
		UASSERT(depth.type() == CV_32FC1 || // Depth in meter
				depth.type() == CV_16UC1); // Depth in millimetre
		_depthOrRightRaw = depth;
	}

	if(userData.type() == CV_8UC1) // Bytes
	{
		_userDataCompressed = userData; // assume compressed
	}
	else
	{
		_userDataRaw = userData;
	}
}

// Multi-cameras RGB-D constructor + laser scan
SensorData::SensorData(
		const cv::Mat & laserScan,
		int laserScanMaxPts,
		float laserScanMaxRange,
		const cv::Mat & rgb,
		const cv::Mat & depth,
		const std::vector<CameraModel> & cameraModels,
		int id,
		double stamp,
		const cv::Mat & userData) :
		_id(id),
		_stamp(stamp),
		_laserScanMaxPts(laserScanMaxPts),
		_laserScanMaxRange(laserScanMaxRange),
		_cameraModels(cameraModels)
{
	if(rgb.rows == 1)
	{
		UASSERT(rgb.type() == CV_8UC1); // Bytes
		_imageCompressed = rgb;
	}
	else if(!rgb.empty())
	{
		UASSERT(rgb.type() == CV_8UC1 || // Mono
				rgb.type() == CV_8UC3);  // RGB
		_imageRaw = rgb;
	}
	if(depth.rows == 1)
	{
		UASSERT(depth.type() == CV_8UC1); // Bytes
		_depthOrRightCompressed = depth;
	}
	else if(!depth.empty())
	{
		UASSERT(depth.type() == CV_32FC1 || // Depth in meter
				depth.type() == CV_16UC1); // Depth in millimetre
		_depthOrRightRaw = depth;
	}

	if(laserScan.type() == CV_32FC2 || laserScan.type() == CV_32FC3 || laserScan.type() == CV_32FC(6))
	{
		_laserScanRaw = laserScan;
	}
	else if(!laserScan.empty())
	{
		UASSERT(laserScan.type() == CV_8UC1); // Bytes
		_laserScanCompressed = laserScan;
	}

	if(userData.type() == CV_8UC1) // Bytes
	{
		_userDataCompressed = userData; // assume compressed
	}
	else
	{
		_userDataRaw = userData;
	}
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
		_laserScanMaxPts(0),
		_laserScanMaxRange(0.0f),
		_stereoCameraModel(cameraModel)
{
	if(left.rows == 1)
	{
		UASSERT(left.type() == CV_8UC1); // Bytes
		_imageCompressed = left;
	}
	else if(!left.empty())
	{
		UASSERT(left.type() == CV_8UC1 || // Mono
				left.type() == CV_8UC3 || // RGB
				left.type() == CV_16UC1); // IR
		_imageRaw = left;
	}
	if(right.rows == 1)
	{
		UASSERT(right.type() == CV_8UC1); // Bytes
		_depthOrRightCompressed = right;
	}
	else if(!right.empty())
	{
		UASSERT(right.type() == CV_8UC1 || // Mono
				right.type() == CV_16UC1);  // IR
		_depthOrRightRaw = right;
	}

	if(userData.type() == CV_8UC1) // Bytes
	{
		_userDataCompressed = userData; // assume compressed
	}
	else
	{
		_userDataRaw = userData;
	}

}

// Stereo constructor + 2d laser scan
SensorData::SensorData(
		const cv::Mat & laserScan,
		int laserScanMaxPts,
		float laserScanMaxRange,
		const cv::Mat & left,
		const cv::Mat & right,
		const StereoCameraModel & cameraModel,
		int id,
		double stamp,
		const cv::Mat & userData) :
		_id(id),
		_stamp(stamp),
		_laserScanMaxPts(laserScanMaxPts),
		_laserScanMaxRange(laserScanMaxRange),
		_stereoCameraModel(cameraModel)
{
	if(left.rows == 1)
	{
		UASSERT(left.type() == CV_8UC1); // Bytes
		_imageCompressed = left;
	}
	else if(!left.empty())
	{
		UASSERT(left.type() == CV_8UC1 || // Mono
				left.type() == CV_8UC3);  // RGB
		_imageRaw = left;
	}
	if(right.rows == 1)
	{
		UASSERT(right.type() == CV_8UC1); // Bytes
		_depthOrRightCompressed = right;
	}
	else if(!right.empty())
	{
		UASSERT(right.type() == CV_8UC1); // Mono
		_depthOrRightRaw = right;
	}

	if(laserScan.type() == CV_32FC2 || laserScan.type() == CV_32FC3 || laserScan.type() == CV_32FC(6))
	{
		_laserScanRaw = laserScan;
	}
	else if(!laserScan.empty())
	{
		UASSERT(laserScan.type() == CV_8UC1); // Bytes
		_laserScanCompressed = laserScan;
	}

	if(userData.type() == CV_8UC1) // Bytes
	{
		_userDataCompressed = userData; // assume compressed
	}
	else
	{
		_userDataRaw = userData;
	}
}

void SensorData::setUserDataRaw(const cv::Mat & userDataRaw)
{
	if(!userDataRaw.empty() && (!_userDataCompressed.empty() || !_userDataRaw.empty()))
	{
		UWARN("Cannot write new user data (%d bytes) over existing user "
			  "data (%d bytes, %d compressed). Set user data of %d to null "
			  "before setting a new one.",
			  int(userDataRaw.total()*userDataRaw.elemSize()),
			  int(_userDataRaw.total()*_userDataRaw.elemSize()),
			  _userDataCompressed.cols,
			  this->id());
		return;
	}
	_userDataRaw = userDataRaw;
}

void SensorData::setUserData(const cv::Mat & userData)
{
	if(!userData.empty() && (!_userDataCompressed.empty() || !_userDataRaw.empty()))
	{
		UWARN("Cannot write new user data (%d bytes) over existing user "
			  "data (%d bytes, %d compressed). Set user data of %d to null "
			  "before setting a new one.",
			  int(userData.total()*userData.elemSize()),
			  int(_userDataRaw.total()*_userDataRaw.elemSize()),
			  _userDataCompressed.cols,
			  this->id());
		return;
	}
	_userDataRaw = cv::Mat();
	_userDataCompressed = cv::Mat();

	if(!userData.empty())
	{
		if(userData.type() == CV_8UC1) // Bytes
		{
			_userDataCompressed = userData; // assume compressed
		}
		else
		{
			_userDataRaw = userData;
			_userDataCompressed = compressData2(userData);
		}
	}
}

void SensorData::uncompressData()
{
	cv::Mat tmpA, tmpB, tmpC, tmpD;
	uncompressData(_imageCompressed.empty()?0:&tmpA,
				_depthOrRightCompressed.empty()?0:&tmpB,
				_laserScanCompressed.empty()?0:&tmpC,
				_userDataCompressed.empty()?0:&tmpD);
}

void SensorData::uncompressData(cv::Mat * imageRaw, cv::Mat * depthRaw, cv::Mat * laserScanRaw, cv::Mat * userDataRaw)
{
	uncompressDataConst(imageRaw, depthRaw, laserScanRaw, userDataRaw);
	if(imageRaw && !imageRaw->empty() && _imageRaw.empty())
	{
		_imageRaw = *imageRaw;
		//backward compatibility, set image size in camera model if not set
		if(!_imageRaw.empty() && _cameraModels.size())
		{
			cv::Size size(_imageRaw.cols/_cameraModels.size(), _imageRaw.rows/_cameraModels.size());
			for(unsigned int i=0; i<_cameraModels.size(); ++i)
			{
				if(_cameraModels[i].isValidForProjection() && _cameraModels[i].imageWidth() == 0)
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
	if(laserScanRaw && !laserScanRaw->empty() && _laserScanRaw.empty())
	{
		_laserScanRaw = *laserScanRaw;
	}
	if(userDataRaw && !userDataRaw->empty() && _userDataRaw.empty())
	{
		_userDataRaw = *userDataRaw;
	}
}

void SensorData::uncompressDataConst(cv::Mat * imageRaw, cv::Mat * depthRaw, cv::Mat * laserScanRaw, cv::Mat * userDataRaw) const
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
	if( (imageRaw && imageRaw->empty()) ||
		(depthRaw && depthRaw->empty()) ||
		(laserScanRaw && laserScanRaw->empty()) ||
		(userDataRaw && userDataRaw->empty()))
	{
		rtabmap::CompressionThread ctImage(_imageCompressed, true);
		rtabmap::CompressionThread ctDepth(_depthOrRightCompressed, true);
		rtabmap::CompressionThread ctLaserScan(_laserScanCompressed, false);
		rtabmap::CompressionThread ctUserData(_userDataCompressed, false);
		if(imageRaw && imageRaw->empty())
		{
			ctImage.start();
		}
		if(depthRaw && depthRaw->empty())
		{
			ctDepth.start();
		}
		if(laserScanRaw && laserScanRaw->empty())
		{
			ctLaserScan.start();
		}
		if(userDataRaw && userDataRaw->empty())
		{
			ctUserData.start();
		}
		ctImage.join();
		ctDepth.join();
		ctLaserScan.join();
		ctUserData.join();
		if(imageRaw && imageRaw->empty())
		{
			*imageRaw = ctImage.getUncompressedData();
			if(imageRaw->empty())
			{
				UWARN("Requested raw image data, but the sensor data (%d) doesn't have image.", this->id());
			}
		}
		if(depthRaw && depthRaw->empty())
		{
			*depthRaw = ctDepth.getUncompressedData();
			if(depthRaw->empty())
			{
				UWARN("Requested depth/right image data, but the sensor data (%d) doesn't have depth/right image.", this->id());
			}
		}
		if(laserScanRaw && laserScanRaw->empty())
		{
			*laserScanRaw = ctLaserScan.getUncompressedData();

			if(laserScanRaw->empty())
			{
				UWARN("Requested laser scan data, but the sensor data (%d) doesn't have laser scan.", this->id());
			}
		}
		if(userDataRaw && userDataRaw->empty())
		{
			*userDataRaw = ctUserData.getUncompressedData();

			if(userDataRaw->empty())
			{
				UWARN("Requested user data, but the sensor data (%d) doesn't have user data.", this->id());
			}
		}
	}
}

long SensorData::getMemoryUsed() const // Return memory usage in Bytes
{
	return _imageCompressed.total()*_imageCompressed.elemSize() +
			_imageRaw.total()*_imageRaw.elemSize() +
			_depthOrRightCompressed.total()*_depthOrRightCompressed.elemSize() +
			_depthOrRightRaw.total()*_depthOrRightRaw.elemSize() +
			_userDataCompressed.total()*_userDataCompressed.elemSize() +
			_userDataRaw.total()*_userDataRaw.elemSize() +
			_laserScanCompressed.total()*_laserScanCompressed.elemSize() +
			_laserScanRaw.total()*_laserScanRaw.elemSize();
}

} // namespace rtabmap

