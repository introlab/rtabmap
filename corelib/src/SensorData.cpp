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
		_laserScanMaxPts(0)
{
}

// Appearance-only constructor
SensorData::SensorData(
		const cv::Mat & image,
		int id,
		double stamp,
		const std::vector<unsigned char> & userData) :
		_id(id),
		_stamp(stamp),
		_laserScanMaxPts(0),
		_userData(userData)
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
}

// Mono constructor
SensorData::SensorData(
		const cv::Mat & image,
		const CameraModel & cameraModel,
		int id,
		double stamp,
		const std::vector<unsigned char> & userData) :
		_id(id),
		_stamp(stamp),
		_laserScanMaxPts(0),
		_cameraModels(std::vector<CameraModel>(1, cameraModel)),
		_userData(userData)
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
}

// RGB-D constructor
SensorData::SensorData(
		const cv::Mat & rgb,
		const cv::Mat & depth,
		const CameraModel & cameraModel,
		int id,
		double stamp,
		const std::vector<unsigned char> & userData) :
		_id(id),
		_stamp(stamp),
		_laserScanMaxPts(0),
		_cameraModels(std::vector<CameraModel>(1, cameraModel)),
		_userData(userData)
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
}

// RGB-D constructor + 2d laser scan
SensorData::SensorData(
		const cv::Mat & laserScan,
		int laserScanMaxPts,
		const cv::Mat & rgb,
		const cv::Mat & depth,
		const CameraModel & cameraModel,
		int id,
		double stamp,
		const std::vector<unsigned char> & userData) :
		_id(id),
		_stamp(stamp),
		_laserScanMaxPts(laserScanMaxPts),
		_cameraModels(std::vector<CameraModel>(1, cameraModel)),
		_userData(userData)
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
	if(laserScan.rows == 1)
	{
		UASSERT(laserScan.type() == CV_8UC1); // Bytes
		_laserScanCompressed = laserScan;
	}
	else if(!laserScan.empty())
	{
		UASSERT(laserScan.type() == CV_32FC2);
		_laserScanRaw = laserScan;
	}
}

// Multi-cameras RGB-D constructor
SensorData::SensorData(
		const cv::Mat & rgb,
		const cv::Mat & depth,
		const std::vector<CameraModel> & cameraModels,
		int id,
		double stamp,
		const std::vector<unsigned char> & userData) :
		_id(id),
		_stamp(stamp),
		_laserScanMaxPts(0),
		_cameraModels(cameraModels),
		_userData(userData)
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
	for(unsigned int i=0; i<cameraModels.size(); ++i)
	{
		UASSERT(cameraModels[i].isValid());
	}
}

// Multi-cameras RGB-D constructor + 2d laser scan
SensorData::SensorData(
		const cv::Mat & laserScan,
		int laserScanMaxPts,
		const cv::Mat & rgb,
		const cv::Mat & depth,
		const std::vector<CameraModel> & cameraModels,
		int id,
		double stamp,
		const std::vector<unsigned char> & userData) :
		_id(id),
		_stamp(stamp),
		_laserScanMaxPts(laserScanMaxPts),
		_cameraModels(cameraModels),
		_userData(userData)
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

	if(laserScan.rows == 1)
	{
		UASSERT(laserScan.type() == CV_8UC1); // Bytes
		_laserScanCompressed = laserScan;
	}
	else if(!laserScan.empty())
	{
		UASSERT(laserScan.type() == CV_32FC2);
		_laserScanRaw = laserScan;
	}

	for(unsigned int i=0; i<cameraModels.size(); ++i)
	{
		UASSERT(cameraModels[i].isValid());
	}
}

// Stereo constructor
SensorData::SensorData(
		const cv::Mat & left,
		const cv::Mat & right,
		const StereoCameraModel & cameraModel,
		int id,
		double stamp,
		const std::vector<unsigned char> & userData):
		_id(id),
		_stamp(stamp),
		_laserScanMaxPts(0),
		_stereoCameraModel(cameraModel),
		_userData(userData)
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

}

// Stereo constructor + 2d laser scan
SensorData::SensorData(
		const cv::Mat & laserScan,
		int laserScanMaxPts,
		const cv::Mat & left,
		const cv::Mat & right,
		const StereoCameraModel & cameraModel,
		int id,
		double stamp,
		const std::vector<unsigned char> & userData) :
		_id(id),
		_stamp(stamp),
		_laserScanMaxPts(laserScanMaxPts),
		_stereoCameraModel(cameraModel),
		_userData(userData)
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

	if(laserScan.rows == 1)
	{
		UASSERT(laserScan.type() == CV_8UC1); // Bytes
		_laserScanCompressed = laserScan;
	}
	else if(!laserScan.empty())
	{
		UASSERT(laserScan.type() == CV_32FC2);
		_laserScanRaw = laserScan;
	}
}

void SensorData::uncompressData()
{
	uncompressData(&_imageRaw, &_depthOrRightRaw, &_laserScanRaw);
}

void SensorData::uncompressData(cv::Mat * imageRaw, cv::Mat * depthRaw, cv::Mat * laserScanRaw)
{
	uncompressDataConst(imageRaw, depthRaw, laserScanRaw);
	if(imageRaw && !imageRaw->empty() && _imageRaw.empty())
	{
		_imageRaw = *imageRaw;
	}
	if(depthRaw && !depthRaw->empty() && _depthOrRightRaw.empty())
	{
		_depthOrRightRaw = *depthRaw;
	}
	if(laserScanRaw && !laserScanRaw->empty() && _laserScanRaw.empty())
	{
		_laserScanRaw = *laserScanRaw;
	}
}

void SensorData::uncompressDataConst(cv::Mat * imageRaw, cv::Mat * depthRaw, cv::Mat * laserScanRaw) const
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
	if( (imageRaw && imageRaw->empty()) ||
		(depthRaw && depthRaw->empty()) ||
		(laserScanRaw && laserScanRaw->empty()))
	{
		rtabmap::CompressionThread ctImage(_imageCompressed, true);
		rtabmap::CompressionThread ctDepth(_depthOrRightCompressed, true);
		rtabmap::CompressionThread ctLaserScan(_laserScanCompressed, false);
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
		ctImage.join();
		ctDepth.join();
		ctLaserScan.join();
		if(imageRaw && imageRaw->empty())
		{
			*imageRaw = ctImage.getUncompressedData();
		}
		if(depthRaw && depthRaw->empty())
		{
			*depthRaw = ctDepth.getUncompressedData();
		}
		if(laserScanRaw && laserScanRaw->empty())
		{
			*laserScanRaw = ctLaserScan.getUncompressedData();
		}
	}
}

} // namespace rtabmap

