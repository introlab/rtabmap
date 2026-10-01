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

#include <rtabmap/core/rvl_codec.h>
#include <rtabmap/utilite/UThread.h>
#include <opencv2/opencv.hpp>

namespace rtabmap {

/**
 * @class CompressionThread
 * @brief Background thread to compress or uncompress images and generic matrices.
 *
 * In compress mode, pass a source matrix to the constructor with an optional image
 * format (".png", ".jpg", ".rvl", or empty for zlib data). In uncompress mode, pass
 * compressed bytes and set @c isImage accordingly. Call @ref UThread::start() then
 * @ref UThread::join() to obtain the result from @ref getCompressedData() or
 * @ref getUncompressedData().
 *
 * Example compression:
 * @code
 * cv::Mat image;
 * CompressionThread ct(image, ".png");
 * ct.start();
 * ct.join();
 * cv::Mat bytes = ct.getCompressedData();
 * @endcode
 *
 * Example uncompression:
 * @code
 * cv::Mat bytes;
 * CompressionThread ct(bytes, true);
 * ct.start();
 * ct.join();
 * cv::Mat image = ct.getUncompressedData();
 * @endcode
 */
class RTABMAP_CORE_EXPORT CompressionThread : public UThread
{
public:
	/**
	 * @brief Constructs a thread in compress mode.
	 * @param mat Source image or data matrix to compress.
	 * @param format Image format: @c ".png", @c ".jpg", @c ".rvl", or empty for zlib (@ref compressData2).
	 */
	CompressionThread(const cv::Mat & mat, const std::string & format = "");
	/**
	 * @brief Constructs a thread in uncompress mode.
	 * @param bytes Compressed bytes (@c CV_8UC1).
	 * @param isImage If true, decode as image; otherwise decode as zlib data.
	 */
	CompressionThread(const cv::Mat & bytes, bool isImage);
	/** @return Compressed output (@c CV_8UC1), valid after compress mode completes. */
	const cv::Mat & getCompressedData() const {return compressedData_;}
	/** @return Uncompressed output, valid after uncompress mode completes. */
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

/** @brief Encodes @p image to a byte buffer (OpenCV @c imencode or RVL for depth). */
std::vector<unsigned char> RTABMAP_CORE_EXPORT compressImage(const cv::Mat & image, const std::string & format = ".png");
/** @brief Same as @ref compressImage() but returns a @c CV_8UC1 row matrix. */
cv::Mat RTABMAP_CORE_EXPORT compressImage2(const cv::Mat & image, const std::string & format = ".png");

/** @brief Decodes compressed image bytes to a @cv::Mat. */
cv::Mat RTABMAP_CORE_EXPORT uncompressImage(const cv::Mat & bytes);
/** @brief Decodes compressed image bytes to a @cv::Mat. */
cv::Mat RTABMAP_CORE_EXPORT uncompressImage(const std::vector<unsigned char> & bytes);

/** @brief Compresses a matrix with zlib; appends rows, cols and type at the end. */
std::vector<unsigned char> RTABMAP_CORE_EXPORT compressData(const cv::Mat & data);
/** @brief Same as @ref compressData() but returns a @c CV_8UC1 row matrix. */
cv::Mat RTABMAP_CORE_EXPORT compressData2(const cv::Mat & data);

/** @brief Restores a matrix compressed with @ref compressData() or @ref compressData2(). */
cv::Mat RTABMAP_CORE_EXPORT uncompressData(const cv::Mat & bytes);
/** @brief Restores a matrix compressed with @ref compressData() or @ref compressData2(). */
cv::Mat RTABMAP_CORE_EXPORT uncompressData(const std::vector<unsigned char> & bytes);
/** @brief Restores a matrix from a raw compressed buffer. */
cv::Mat RTABMAP_CORE_EXPORT uncompressData(const unsigned char * bytes, unsigned long size);

/** @brief Compresses a null-terminated string using @ref compressData2(). */
cv::Mat RTABMAP_CORE_EXPORT compressString(const std::string & str);
/** @brief Decompresses a string produced by @ref compressString(). */
std::string RTABMAP_CORE_EXPORT uncompressString(const cv::Mat & bytes);

/**
 * @brief Detects the compression format of depth image bytes.
 * @return @c ".rvl" if the buffer has an RVL signature, otherwise @c ".png".
 */
std::string RTABMAP_CORE_EXPORT compressedDepthFormat(const cv::Mat & bytes);
/** @overload */
std::string RTABMAP_CORE_EXPORT compressedDepthFormat(const std::vector<unsigned char> & bytes);
/** @overload */
std::string RTABMAP_CORE_EXPORT compressedDepthFormat(const unsigned char * bytes, size_t size);

} /* namespace rtabmap */
#endif /* COMPRESSION_H_ */
