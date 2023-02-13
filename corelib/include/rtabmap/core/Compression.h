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

#ifndef COMPRESSION_H_
#define COMPRESSION_H_

#include "rtabmap/core/rtabmap_core_export.h" // DLL export/import defines

#include <rtabmap/utilite/UThread.h>
#include <opencv2/opencv.hpp>

namespace rtabmap {

/**
 * Compress image or data
 *
 * Example compression:
 *   cv::Mat image;// an image
 *   CompressionThread ct(image);
 *   ct.start();
 *   ct.join();
 *   std::vector<unsigned char> bytes = ct.getCompressedData();
 *
 * Example uncompression
 *   std::vector<unsigned char> bytes;// a compressed image
 *   CompressionThread ct(bytes);
 *   ct.start();
 *   ct.join();
 *   cv::Mat image = ct.getUncompressedData();
 */
class RTABMAP_CORE_EXPORT CompressionThread : public UThread
{
public:
	// format : ".png" ".jpg" "" (empty is general)
	CompressionThread(const cv::Mat & mat, const std::string & format = "");
	CompressionThread(const cv::Mat & bytes, bool isImage);
	const cv::Mat & getCompressedData() const {return compressedData_;}
	cv::Mat & getUncompressedData() {return uncompressedData_;}
protected:
	virtual void mainLoop();
private:
	cv::Mat compressedData_;
	cv::Mat uncompressedData_;
	std::string format_;
	bool image_;
	bool compressMode_;
};

std::vector<unsigned char> RTABMAP_CORE_EXPORT compressImage(const cv::Mat & image, const std::string & format = ".png");
cv::Mat RTABMAP_CORE_EXPORT compressImage2(const cv::Mat & image, const std::string & format = ".png");

cv::Mat RTABMAP_CORE_EXPORT uncompressImage(const cv::Mat & bytes);
cv::Mat RTABMAP_CORE_EXPORT uncompressImage(const std::vector<unsigned char> & bytes);

std::vector<unsigned char> RTABMAP_CORE_EXPORT compressData(const cv::Mat & data);
cv::Mat RTABMAP_CORE_EXPORT compressData2(const cv::Mat & data);

cv::Mat RTABMAP_CORE_EXPORT uncompressData(const cv::Mat & bytes);
cv::Mat RTABMAP_CORE_EXPORT uncompressData(const std::vector<unsigned char> & bytes);
cv::Mat RTABMAP_CORE_EXPORT uncompressData(const unsigned char * bytes, unsigned long size);

cv::Mat RTABMAP_CORE_EXPORT compressString(const std::string & str);
std::string RTABMAP_CORE_EXPORT uncompressString(const cv::Mat & bytes);

} /* namespace rtabmap */
#endif /* COMPRESSION_H_ */
