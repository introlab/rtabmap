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

#include "rtabmap/core/Compression.h"
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UConversion.h>
#include <opencv2/opencv.hpp>

#include <zlib.h>

namespace rtabmap {

// format : ".png" ".jpg" "" (empty is general)
CompressionThread::CompressionThread(const cv::Mat & mat, const std::string & format) :
	uncompressedData_(mat),
	format_(format),
	image_(!format.empty()),
	compressMode_(true)
{
	UASSERT(format.empty() || format.compare(".png") == 0 || format.compare(".jpg") == 0);
}
// assume image
CompressionThread::CompressionThread(const cv::Mat & bytes, bool isImage) :
	compressedData_(bytes),
	image_(isImage),
	compressMode_(false)
{}
void CompressionThread::mainLoop()
{
	try
	{
		if(compressMode_)
		{
			if(!uncompressedData_.empty())
			{
				if(image_)
				{
					compressedData_ = compressImage2(uncompressedData_, format_);
				}
				else
				{
					compressedData_ = compressData2(uncompressedData_);
				}
			}
		}
		else // uncompress
		{
			if(!compressedData_.empty())
			{
				if(image_)
				{
					uncompressedData_ = uncompressImage(compressedData_);
				}
				else
				{
					uncompressedData_ = uncompressData(compressedData_);
				}
			}
		}
	}
	catch (cv::Exception & e) {
		UERROR("Exception while compressing/uncompressing data: %s", e.what());
		if(compressMode_)
		{
			compressedData_ = cv::Mat();
		}
		else
		{
			uncompressedData_ = cv::Mat();
		}
	}
	this->kill();
}

// ".png" or ".jpg"
std::vector<unsigned char> compressImage(const cv::Mat & image, const std::string & format)
{
	std::vector<unsigned char> bytes;
	if(!image.empty())
	{
		if(image.type() == CV_32FC1)
		{
			//save in 8bits-4channel
			cv::Mat bgra(image.size(), CV_8UC4, image.data);
			cv::imencode(format, bgra, bytes);
		}
		else
		{
			cv::imencode(format, image, bytes);
		}
	}
	return bytes;
}

// ".png" or ".jpg"
cv::Mat compressImage2(const cv::Mat & image, const std::string & format)
{
	std::vector<unsigned char> bytes = compressImage(image, format);
	if(bytes.size())
	{
		return cv::Mat(1, (int)bytes.size(), CV_8UC1, bytes.data()).clone();
	}
	return cv::Mat();
}

cv::Mat uncompressImage(const cv::Mat & bytes)
{
	 cv::Mat image;
	if(!bytes.empty())
	{
#if CV_MAJOR_VERSION>2 || (CV_MAJOR_VERSION >=2 && CV_MINOR_VERSION >=4)
		image = cv::imdecode(bytes, cv::IMREAD_UNCHANGED);
#else
		image = cv::imdecode(bytes, -1);
#endif
		if(image.type() == CV_8UC4)
		{
			// Using clone() or copyTo() caused a memory leak !?!?
			// image = cv::Mat(image.size(), CV_32FC1, image.data).clone();
			cv::Mat depth(image.size(), CV_32FC1);
			memcpy(depth.data, image.data, image.total()*image.elemSize());
			image = depth;
		}
	}
	return image;
}

cv::Mat uncompressImage(const std::vector<unsigned char> & bytes)
{
	 cv::Mat image;
	if(bytes.size())
	{
#if CV_MAJOR_VERSION>2 || (CV_MAJOR_VERSION >=2 && CV_MINOR_VERSION >=4)
		image = cv::imdecode(bytes, cv::IMREAD_UNCHANGED);
#else
		image = cv::imdecode(bytes, -1);
#endif
		if(image.type() == CV_8UC4)
		{
			image = cv::Mat(image.size(), CV_32FC1, image.data).clone();
		}
	}
	return image;
}

std::vector<unsigned char> compressData(const cv::Mat & data)
{
	std::vector<unsigned char> bytes;
	if(!data.empty())
	{
		uLong sourceLen = uLong(data.total())*uLong(data.elemSize());
		uLong destLen = compressBound(sourceLen);
		bytes.resize(destLen);
		int errCode = compress(
						(Bytef *)bytes.data(),
						&destLen,
						(const Bytef *)data.data,
						sourceLen);

		bytes.resize(destLen+3*sizeof(int));
		*((int*)&bytes[destLen]) = data.rows;
		*((int*)&bytes[destLen+sizeof(int)]) = data.cols;
		*((int*)&bytes[destLen+2*sizeof(int)]) = data.type();

		if(errCode == Z_MEM_ERROR)
		{
			UERROR("Z_MEM_ERROR : Insufficient memory.");
		}
		else if(errCode == Z_BUF_ERROR)
		{
			UERROR("Z_BUF_ERROR : The buffer dest was not large enough to hold the uncompressed data.");
		}
	}
	return bytes;
}

cv::Mat compressData2(const cv::Mat & data)
{
	cv::Mat bytes;
	if(!data.empty())
	{
		uLong sourceLen = uLong(data.total())*uLong(data.elemSize());
		uLong destLen = compressBound(sourceLen);
		bytes = cv::Mat(1, destLen+3*sizeof(int), CV_8UC1);
		int errCode = compress(
						(Bytef *)bytes.data,
						&destLen,
						(const Bytef *)data.data,
						sourceLen);
		bytes = cv::Mat(bytes, cv::Rect(0,0, destLen+3*sizeof(int), 1));
		*((int*)&bytes.data[destLen]) = data.rows;
		*((int*)&bytes.data[destLen+sizeof(int)]) = data.cols;
		*((int*)&bytes.data[destLen+2*sizeof(int)]) = data.type();

		if(errCode == Z_MEM_ERROR)
		{
			UERROR("Z_MEM_ERROR : Insufficient memory.");
		}
		else if(errCode == Z_BUF_ERROR)
		{
			UERROR("Z_BUF_ERROR : The buffer dest was not large enough to hold the uncompressed data.");
		}
	}
	return bytes;
}

cv::Mat uncompressData(const cv::Mat & bytes)
{
	UASSERT(bytes.empty() || bytes.type() == CV_8UC1);
	return uncompressData(bytes.data, bytes.cols*bytes.rows);
}

cv::Mat uncompressData(const std::vector<unsigned char> & bytes)
{
	return uncompressData(bytes.data(), (unsigned long)bytes.size());
}

cv::Mat uncompressData(const unsigned char * bytes, unsigned long size)
{
	cv::Mat data;
	if(bytes && size>=3*sizeof(int))
	{
		//last 3 int elements are matrix size and type
		int height = *((int*)&bytes[size-3*sizeof(int)]);
		int width = *((int*)&bytes[size-2*sizeof(int)]);
		int type = *((int*)&bytes[size-1*sizeof(int)]);

		data = cv::Mat(height, width, type);
		uLongf totalUncompressed = uLongf(data.total())*uLongf(data.elemSize());

		int errCode = uncompress(
						(Bytef*)data.data,
						&totalUncompressed,
						(const Bytef*)bytes,
						uLong(size));

		if(errCode == Z_MEM_ERROR)
		{
			UERROR("Z_MEM_ERROR : Insufficient memory.");
		}
		else if(errCode == Z_BUF_ERROR)
		{
			UERROR("Z_BUF_ERROR : The buffer dest was not large enough to hold the uncompressed data.");
		}
		else if(errCode == Z_DATA_ERROR)
		{
			UERROR("Z_DATA_ERROR : The compressed data (referenced by source) was corrupted.");
		}
	}
	return data;
}

cv::Mat compressString(const std::string & str)
{
	// +1 to include null character
	return compressData2(cv::Mat(1, str.size()+1, CV_8SC1, (void *)str.data()));
}

std::string uncompressString(const cv::Mat & bytes)
{
	cv::Mat strMat = uncompressData(bytes);
	if(!strMat.empty())
	{
		UASSERT(strMat.type() == CV_8SC1 && strMat.rows == 1);
		return (const char*)strMat.data;
	}
	return "";
}

} /* namespace rtabmap */
