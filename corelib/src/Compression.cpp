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
#include <rtabmap/utilite/UStl.h>
#include <opencv2/opencv.hpp>

#include <zlib.h>
#include <cmath>
#include <cstring>

namespace rtabmap {

namespace {

// The compressed blob trailer stores the cv::Mat type as a raw int, and the
// database keeps those blobs forever. OpenCV 5 changed CV_CN_SHIFT from 3 to 5,
// so the very same type has a different numeric value across major versions
// (CV_32FC2 is 13 under OpenCV 4 but 37 under OpenCV 5). Reading an OpenCV 4
// database with an OpenCV 5 build would decode 13 as a 1-channel Mat of a
// nonsense depth.
//
// Persist the OpenCV 4 encoding in both cases: existing databases stay
// readable, and databases written by an OpenCV 5 build stay readable by
// OpenCV 4 builds. Under OpenCV 4 both helpers are the identity.
const int kSerializedCnShift = 3;
const int kSerializedDepthMask = (1 << kSerializedCnShift) - 1;

int serializeMatType(int type)
{
	const int depth = CV_MAT_DEPTH(type);
	// Only the 8 original depths (CV_8U..CV_16F) fit the on-disk encoding;
	// rtabmap never persists the types OpenCV 5 added beyond them.
	UASSERT_MSG(depth <= kSerializedDepthMask,
			uFormat("Cannot serialize cv::Mat depth %d (type %d): the database "
					"format only supports depths 0-%d.",
					depth, type, kSerializedDepthMask).c_str());
	return depth + ((CV_MAT_CN(type) - 1) << kSerializedCnShift);
}

int deserializeMatType(int serializedType)
{
	return CV_MAKETYPE(serializedType & kSerializedDepthMask,
			((serializedType >> kSerializedCnShift) & 511) + 1);
}

// Default quantization when only the maximum depth is set in the format.
const float kDefaultDepthQuantization = 100.0f;

bool hasSignature(const unsigned char * bytes, size_t size, const void * signature)
{
	return bytes && size >= 8 && memcmp(bytes, signature, 8) == 0;
}

// Values over maxDepth, NaN, inf, 0 and negative values are set to 0 (invalid),
// as well as values too close to be represented on 16 bits (under
// depthQuantA / (65535 - depthQuantB) meters).
cv::Mat depthToInvDepth(const cv::Mat & depth, float maxDepth, float quantization, float & depthQuantA, float & depthQuantB)
{
	UASSERT(depth.type() == CV_32FC1);
	depthQuantA = quantization * (quantization + 1.0f);
	depthQuantB = 1.0f - depthQuantA / maxDepth;
	cv::Mat invDepth(depth.size(), CV_16UC1);
	for(int i=0; i<depth.rows; ++i)
	{
		const float * in = depth.ptr<float>(i);
		uint16_t * out = invDepth.ptr<uint16_t>(i);
		for(int j=0; j<depth.cols; ++j)
		{
			const float d = in[j];
			if(d > 0.0f && d < maxDepth) // false for NaN
			{
				// Rounded (ROS truncates), the decoding is the same.
				const float v = depthQuantA / d + depthQuantB + 0.5f;
				out[j] = v < 65536.0f ? (uint16_t)v : 0;
			}
			else
			{
				out[j] = 0;
			}
		}
	}
	return invDepth;
}

cv::Mat invDepthToDepth(const cv::Mat & invDepth, float depthQuantA, float depthQuantB)
{
	UASSERT(invDepth.type() == CV_16UC1);
	cv::Mat depth(invDepth.size(), CV_32FC1);
	for(int i=0; i<invDepth.rows; ++i)
	{
		const uint16_t * in = invDepth.ptr<uint16_t>(i);
		float * out = depth.ptr<float>(i);
		for(int j=0; j<invDepth.cols; ++j)
		{
			out[j] = in[j] ? depthQuantA / (float(in[j]) - depthQuantB) : 0.0f;
		}
	}
	return depth;
}

void invDepthParameters(float depthQuantA, float depthQuantB, float & maxDepth, float & quantization)
{
	// inverse of depthQuantA = q*(q+1) and depthQuantB = 1 - depthQuantA/maxDepth
	quantization = (std::sqrt(1.0f + 4.0f*depthQuantA) - 1.0f) / 2.0f;
	maxDepth = depthQuantA / (1.0f - depthQuantB);
}

}  // namespace

bool parseImageCompressionFormat(const std::string & format, std::string & codec, float & maxDepth, float & quantization)
{
	codec.clear();
	maxDepth = 0.0f;
	quantization = 0.0f;
	std::vector<std::string> fields = uListToVector(uSplit(format, ':'));
	if(fields.empty())
	{
		return format.empty(); // empty is general (zlib)
	}
	if(fields[0].size() < 2 || fields[0][0] != '.')
	{
		return false;
	}
	if(fields.size() > 1)
	{
		// Inverse depth parameters only for formats supporting 16UC1
		if((fields[0] != ".png" && fields[0] != ".rvl") || fields.size() > 3)
		{
			return false;
		}
		for(size_t i=1; i<fields.size(); ++i)
		{
			if(!uIsNumber(fields[i]) || uStr2Float(fields[i]) <= 0.0f)
			{
				return false;
			}
		}
		maxDepth = uStr2Float(fields[1]);
		quantization = fields.size() == 3 ? uStr2Float(fields[2]) : kDefaultDepthQuantization;
	}
	codec = fields[0];
	return true;
}

// format : ".jpg" ".png" ".rvl" "" (empty is general), see parseImageCompressionFormat()
CompressionThread::CompressionThread(const cv::Mat & mat, const std::string & format) :
	uncompressedData_(mat),
	format_(format),
	image_(!format.empty()),
	compressMode_(true)
{
	std::string codec;
	float maxDepth, quantization;
	UASSERT_MSG(parseImageCompressionFormat(format, codec, maxDepth, quantization) &&
			(codec.empty() || codec == ".jpg" || codec == ".png" || codec == ".rvl"),
			uFormat("Invalid compression format \"%s\"", format.c_str()).c_str());
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

// ".jpg" or ".png" or ".rvl", with optional ":maxDepth[:quantization]" for 32FC1 depth images
std::vector<unsigned char> compressImage(const cv::Mat & image, const std::string & format)
{
	std::vector<unsigned char> bytes;
	if(!image.empty())
	{
		std::string codec;
		float maxDepth, quantization;
		if(!parseImageCompressionFormat(format, codec, maxDepth, quantization) || codec.empty())
		{
			UERROR("Invalid image compression format \"%s\"", format.c_str());
			return bytes;
		}

		if(image.type() == CV_32FC1 && maxDepth > 0.0f)
		{
			float depthQuantA, depthQuantB;
			cv::Mat invDepth = depthToInvDepth(image, maxDepth, quantization, depthQuantA, depthQuantB);
			std::vector<unsigned char> invDepthBytes = compressImage(invDepth, codec);
			if(!invDepthBytes.empty())
			{
				bytes.resize(kCompressedDepthInvHeaderSize + invDepthBytes.size());
				memcpy(&bytes[0], kCompressedDepthInvSignature, 8);
				memcpy(&bytes[8], &depthQuantA, 4);
				memcpy(&bytes[12], &depthQuantB, 4);
				memcpy(&bytes[kCompressedDepthInvHeaderSize], invDepthBytes.data(), invDepthBytes.size());
			}
		}
		else if(image.type() == CV_32FC1)
		{
			//save in 8bits-4channel
			cv::Mat bgra(image.size(), CV_8UC4, image.data);
			cv::imencode(".png", bgra, bytes);
		}
		else if(codec == ".rvl")
		{
			bytes.assign(kCompressedDepthRvlSignature, kCompressedDepthRvlSignature+8);
			int numPixels = image.rows * image.cols;
        	// In the worst case, RVL compression results in ~1.5x larger data.
        	bytes.resize(3 * numPixels + 20);
        	uint32_t cols = image.cols;
        	uint32_t rows = image.rows;
        	memcpy(&bytes[8], &cols, 4);
        	memcpy(&bytes[12], &rows, 4);
        	RvlCodec rvl;
        	int compressedSize = rvl.CompressRVL(image.ptr<uint16_t>(), &bytes[kCompressedDepthRvlHeaderSize], numPixels);
        	bytes.resize(kCompressedDepthRvlHeaderSize + compressedSize);
		}
		else
		{
			cv::imencode(codec, image, bytes);
		}
	}
	return bytes;
}

// ".jpg" or ".png" or ".rvl", with optional ":maxDepth[:quantization]" for 32FC1 depth images
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
	if(bytes.empty())
	{
		return cv::Mat();
	}
	return uncompressImage(bytes.data, bytes.total()*bytes.elemSize());
}

cv::Mat uncompressImage(const std::vector<unsigned char> & bytes)
{
	return uncompressImage(bytes.data(), bytes.size());
}

cv::Mat uncompressImage(const unsigned char * bytes, size_t size)
{
	cv::Mat image;
	if(bytes && size)
	{
		if(hasSignature(bytes, size, kCompressedDepthInvSignature))
		{
			if(size <= kCompressedDepthInvHeaderSize)
			{
				UERROR("Inverse depth image is truncated (%d bytes).", (int)size);
				return image;
			}
			float depthQuantA, depthQuantB;
			memcpy(&depthQuantA, &bytes[8], 4);
			memcpy(&depthQuantB, &bytes[12], 4);
			cv::Mat invDepth = uncompressImage(&bytes[kCompressedDepthInvHeaderSize], size - kCompressedDepthInvHeaderSize);
			if(invDepth.type() == CV_16UC1)
			{
				image = invDepthToDepth(invDepth, depthQuantA, depthQuantB);
			}
			else if(!invDepth.empty())
			{
				UERROR("Inverse depth image should be 16UC1 (type=%d).", invDepth.type());
			}
		}
		else if(hasSignature(bytes, size, kCompressedDepthRvlSignature))
		{
			if(size < kCompressedDepthRvlHeaderSize)
			{
				UERROR("RVL depth image is truncated (%d bytes).", (int)size);
				return image;
			}
			uint32_t cols, rows;
        	memcpy(&cols, &bytes[8], 4);
        	memcpy(&rows, &bytes[12], 4);
			image = cv::Mat(rows, cols, CV_16UC1);
			RvlCodec rvl;
        	rvl.DecompressRVL(&bytes[kCompressedDepthRvlHeaderSize], image.ptr<uint16_t>(), cols * rows);
		}
		else
		{
			const cv::Mat buf(1, (int)size, CV_8UC1, (void *)bytes);
#if CV_MAJOR_VERSION>2 || (CV_MAJOR_VERSION >=2 && CV_MINOR_VERSION >=4)
			image = cv::imdecode(buf, cv::IMREAD_UNCHANGED);
#else
			image = cv::imdecode(buf, -1);
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
		*((int*)&bytes[destLen+2*sizeof(int)]) = serializeMatType(data.type());

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
		*((int*)&bytes.data[destLen+2*sizeof(int)]) = serializeMatType(data.type());

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
		int type = deserializeMatType(*((int*)&bytes[size-1*sizeof(int)]));

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

std::string compressedDepthFormat(const cv::Mat & bytes)
{
	if(bytes.empty())
	{
		return std::string();
	}
	return compressedDepthFormat(bytes.data, bytes.rows * bytes.cols * bytes.elemSize());
}
std::string compressedDepthFormat(const std::vector<unsigned char> & bytes)
{
	return compressedDepthFormat(bytes.data(), bytes.size());
}
std::string compressedDepthFormat(const unsigned char * bytes, size_t size)
{
	std::string format;
	if(bytes && size)
	{
		if(hasSignature(bytes, size, kCompressedDepthInvSignature) && size > kCompressedDepthInvHeaderSize)
		{
			float depthQuantA, depthQuantB, maxDepth, quantization;
			memcpy(&depthQuantA, &bytes[8], 4);
			memcpy(&depthQuantB, &bytes[12], 4);
			invDepthParameters(depthQuantA, depthQuantB, maxDepth, quantization);
			format = uFormat("%s:%g:%g",
					compressedDepthFormat(&bytes[kCompressedDepthInvHeaderSize], size - kCompressedDepthInvHeaderSize).c_str(),
					maxDepth, quantization);
		}
		else if(hasSignature(bytes, size, kCompressedDepthRvlSignature))
		{
			format = ".rvl";
		}
		else
		{
			// Assuming png by default
			format = ".png";
		}
	}
	return format;
}

} /* namespace rtabmap */
