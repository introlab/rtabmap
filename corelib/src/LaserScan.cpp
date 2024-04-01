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

#include <rtabmap/core/LaserScan.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UConversion.h>

namespace rtabmap {

std::string LaserScan::formatName(const Format & format)
{
	std::string name;
	switch (format) {
		case kXY:
			name = "XY";
			break;
		case kXYZ:
			name = "XYZ";
			break;
		case kXYI:
			name = "XYI";
			break;
		case kXYZI:
			name = "XYZI";
			break;
		case kXYZRGB:
			name = "XYZRGB";
			break;
		case kXYNormal:
			name = "XYNormal";
			break;
		case kXYZNormal:
			name = "XYZNormal";
			break;
		case kXYINormal:
			name = "XYINormal";
			break;
		case kXYZINormal:
			name = "XYZINormal";
			break;
		case kXYZRGBNormal:
			name = "XYZRGBNormal";
			break;
		default:
			name = "Unknown";
			break;
	}
	return name;
}

int LaserScan::channels(const Format & format)
{
	int channels=0;
	switch (format) {
		case kXY:
			channels = 2;
			break;
		case kXYZ:
		case kXYI:
			channels = 3;
			break;
		case kXYZI:
		case kXYZRGB:
			channels = 4;
			break;
		case kXYNormal:
			channels = 5;
			break;
		case kXYZNormal:
		case kXYINormal:
			channels = 6;
			break;
		case kXYZINormal:
		case kXYZRGBNormal:
			channels = 7;
			break;
		default:
			UFATAL("Unhandled type %d!", (int)format);
			break;
	}
	return channels;
}

bool LaserScan::isScan2d(const Format & format)
{
	return format==kXY || format==kXYI || format == kXYNormal || format == kXYINormal;
}
bool LaserScan::isScanHasNormals(const Format & format)
{
	return format==kXYZNormal || format==kXYZINormal || format==kXYZRGBNormal || format == kXYNormal || format == kXYINormal;
}
bool LaserScan::isScanHasRGB(const Format & format)
{
	return format==kXYZRGB || format==kXYZRGBNormal;
}
bool LaserScan::isScanHasIntensity(const Format & format)
{
	return format==kXYZI || format==kXYZINormal || format == kXYI || format == kXYINormal;
}

LaserScan LaserScan::backwardCompatibility(
		const cv::Mat & oldScanFormat,
		int maxPoints,
		int maxRange,
		const Transform & localTransform)
{
	if(!oldScanFormat.empty())
	{
		if(oldScanFormat.channels() == 2)
		{
			return LaserScan(oldScanFormat, maxPoints, maxRange, kXY, localTransform);
		}
		else if(oldScanFormat.channels() == 3)
		{
			return LaserScan(oldScanFormat, maxPoints, maxRange, kXYZ, localTransform);
		}
		else if(oldScanFormat.channels() == 4)
		{
			return LaserScan(oldScanFormat, maxPoints, maxRange, kXYZRGB, localTransform);
		}
		else if(oldScanFormat.channels() == 5)
		{
			return LaserScan(oldScanFormat, maxPoints, maxRange, kXYNormal, localTransform);
		}
		else if(oldScanFormat.channels() == 6)
		{
			return LaserScan(oldScanFormat, maxPoints, maxRange, kXYZNormal, localTransform);
		}
		else if(oldScanFormat.channels() == 7)
		{
			return LaserScan(oldScanFormat, maxPoints, maxRange, kXYZRGBNormal, localTransform);
		}
	}
	return LaserScan();
}

LaserScan LaserScan::backwardCompatibility(
		const cv::Mat & oldScanFormat,
		float minRange,
		float maxRange,
		float angleMin,
		float angleMax,
		float angleInc,
		const Transform & localTransform)
{
	if(!oldScanFormat.empty())
	{
		if(oldScanFormat.channels() == 2)
		{
			return LaserScan(oldScanFormat, kXY, minRange, maxRange, angleMin, angleMax, angleInc, localTransform);
		}
		else if(oldScanFormat.channels() == 3)
		{
			return LaserScan(oldScanFormat, kXYZ, minRange, maxRange, angleMin, angleMax, angleInc, localTransform);
		}
		else if(oldScanFormat.channels() == 4)
		{
			return LaserScan(oldScanFormat, kXYZRGB, minRange, maxRange, angleMin, angleMax, angleInc, localTransform);
		}
		else if(oldScanFormat.channels() == 5)
		{
			return LaserScan(oldScanFormat, kXYNormal, minRange, maxRange, angleMin, angleMax, angleInc, localTransform);
		}
		else if(oldScanFormat.channels() == 6)
		{
			return LaserScan(oldScanFormat, kXYZNormal, minRange, maxRange, angleMin, angleMax, angleInc, localTransform);
		}
		else if(oldScanFormat.channels() == 7)
		{
			return LaserScan(oldScanFormat, kXYZRGBNormal, minRange, maxRange, angleMin, angleMax, angleInc, localTransform);
		}
	}
	return LaserScan();
}

LaserScan::LaserScan() :
		format_(kUnknown),
		maxPoints_(0),
		rangeMin_(0),
		rangeMax_(0),
		angleMin_(0),
		angleMax_(0),
		angleIncrement_(0),
		localTransform_(Transform::getIdentity())
{
}

LaserScan::LaserScan(
		const LaserScan & scan,
		int maxPoints,
		float maxRange,
		const Transform & localTransform)
{
	UASSERT(scan.empty() || scan.format() != kUnknown);
	init(scan.data(), scan.format(), 0, maxRange, 0, 0, 0, maxPoints, localTransform);
}

LaserScan::LaserScan(
		const LaserScan & scan,
		int maxPoints,
		float maxRange,
		Format format,
		const Transform & localTransform)
{
	init(scan.data(), format, 0, maxRange, 0, 0, 0, maxPoints, localTransform);
}

LaserScan::LaserScan(
		const cv::Mat & data,
		int maxPoints,
		float maxRange,
		Format format,
		const Transform & localTransform)
{
	init(data, format, 0, maxRange, 0, 0, 0, maxPoints, localTransform);
}

LaserScan::LaserScan(
		const LaserScan & scan,
		float minRange,
		float maxRange,
		float angleMin,
		float angleMax,
		float angleIncrement,
		const Transform & localTransform)
{
	UASSERT(scan.empty() || scan.format() != kUnknown);
	init(scan.data(), scan.format(), minRange, maxRange, angleMin, angleMax, angleIncrement, 0, localTransform);
}

LaserScan::LaserScan(
		const LaserScan & scan,
		Format format,
		float minRange,
		float maxRange,
		float angleMin,
		float angleMax,
		float angleIncrement,
		const Transform & localTransform)
{
	init(scan.data(), format, minRange, maxRange, angleMin, angleMax, angleIncrement, 0, localTransform);
}

LaserScan::LaserScan(
		const cv::Mat & data,
		Format format,
		float minRange,
		float maxRange,
		float angleMin,
		float angleMax,
		float angleIncrement,
		const Transform & localTransform)
{
	init(data, format, minRange, maxRange, angleMin, angleMax, angleIncrement, 0, localTransform);
}

void LaserScan::init(
		const cv::Mat & data,
		Format format,
		float rangeMin,
		float rangeMax,
		float angleMin,
		float angleMax,
		float angleIncrement,
		int maxPoints,
		const Transform & localTransform)
{
	UASSERT(data.empty() || data.rows == 1);
	UASSERT(data.empty() || data.type() == CV_8UC1 || data.type() == CV_32FC2 || data.type() == CV_32FC3 || data.type() == CV_32FC(4) || data.type() == CV_32FC(5) || data.type() == CV_32FC(6)  || data.type() == CV_32FC(7));
	UASSERT(!localTransform.isNull());

	bool is2D = false;
	if(angleIncrement != 0.0f)
	{
		// 2D scan
		is2D = true;
		UASSERT(rangeMax>rangeMin);
		UASSERT((angleIncrement>0 && angleMax>angleMin) || (angleIncrement<0 && angleMax<angleMin));
		maxPoints_ = std::ceil((angleMax - angleMin) / angleIncrement)+1;
	}
	else
	{
		// 3D scan
		UASSERT(rangeMax>=rangeMin);
		maxPoints_ = maxPoints;
	}

	data_ = data;
	format_ = format;
	rangeMin_ = rangeMin;
	rangeMax_ = rangeMax;
	angleMin_ = angleMin;
	angleMax_ = angleMax;
	angleIncrement_ = angleIncrement;
	localTransform_ = localTransform;

	if(!data.empty() && !isCompressed())
	{
		if(is2D && data_.cols > maxPoints_)
		{
			UWARN("The number of points (%d) in the scan is over the maximum "
				  "points (%d) defined by angle settings (min=%f max=%f inc=%f). "
				  "The scan info may be wrong!",
				  data_.cols, maxPoints_, angleMin_, angleMax_, angleIncrement_);
		}
		else if(!is2D && maxPoints_>0 && data_.cols > maxPoints_)
		{
			UDEBUG("The number of points (%d) in the scan is over the maximum "
				  "points (%d) defined by max points setting.",
				  data_.cols, maxPoints_);
		}

		if(format == kUnknown)
		{
			if(angleIncrement_ != 0)
			{
				*this = backwardCompatibility(data_, rangeMin_, rangeMax_, angleMin_, angleMax_, angleIncrement_, localTransform_);
			}
			else
			{
				*this = backwardCompatibility(data_, maxPoints_, rangeMax_, localTransform_);
			}
		}
		else // verify that format corresponds to expected number of channels
		{
			UASSERT_MSG(data.channels() != 2 || (data.channels() == 2 && format == kXY), uFormat("format=%s", LaserScan::formatName(format).c_str()).c_str());
			UASSERT_MSG(data.channels() != 3 || (data.channels() == 3 && (format == kXYZ || format == kXYI)), uFormat("format=%s", LaserScan::formatName(format).c_str()).c_str());
			UASSERT_MSG(data.channels() != 4 || (data.channels() == 4 && (format == kXYZI || format == kXYZRGB)), uFormat("format=%s", LaserScan::formatName(format).c_str()).c_str());
			UASSERT_MSG(data.channels() != 5 || (data.channels() == 5 && (format == kXYNormal)), uFormat("format=%s", LaserScan::formatName(format).c_str()).c_str());
			UASSERT_MSG(data.channels() != 6 || (data.channels() == 6 && (format == kXYINormal || format == kXYZNormal)), uFormat("format=%s", LaserScan::formatName(format).c_str()).c_str());
			UASSERT_MSG(data.channels() != 7 || (data.channels() == 7 && (format == kXYZRGBNormal || format == kXYZINormal)), uFormat("format=%s", LaserScan::formatName(format).c_str()).c_str());
		}
	}
}

LaserScan LaserScan::clone() const
{
	if(angleIncrement_ > 0.0f)
	{
		return LaserScan(data_.clone(), format_, rangeMin_, rangeMax_, angleMin_, angleMax_, angleIncrement_, localTransform_.clone());
	}
	return LaserScan(data_.clone(), maxPoints_, rangeMax_, format_, localTransform_.clone());
}

float & LaserScan::field(unsigned int pointIndex, unsigned int channelOffset)
{
	UASSERT(pointIndex < (unsigned int)data_.cols);
	UASSERT(channelOffset < (unsigned int)data_.channels());
	return data_.ptr<float>(0, pointIndex)[channelOffset];
}

LaserScan & LaserScan::operator+=(const LaserScan & scan)
{
	*this = *this+scan;
	return *this;
}

LaserScan LaserScan::operator+(const LaserScan & scan)
{
	UASSERT(this->empty() || scan.empty() || this->format() == scan.format());
	LaserScan dest;
	if(!scan.empty())
	{
		if(this->empty())
		{
			dest = scan.clone();
		}
		else
		{
			cv::Mat destData(1, data_.cols + scan.data().cols, data_.type());
			data_.copyTo(destData(cv::Range::all(), cv::Range(0,data_.cols)));
			scan.data().copyTo(destData(cv::Range::all(), cv::Range(data_.cols, data_.cols+scan.data().cols)));
			dest = LaserScan(destData, 0, 0, this->format());
		}
	}
	else
	{
		dest = this->clone();
	}
	return dest;
}

}
