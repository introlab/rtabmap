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
 * format (".png", ".jpg", ".rvl", or empty for zlib data, see @ref compressImage() for
 * depth options). In uncompress mode, pass
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
	 * @param format Image format: @c ".png", @c ".jpg", @c ".rvl" (see @ref compressImage()), or empty for zlib (@ref compressData2).
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

/*
 * Compressed depth image layouts (all values little-endian). They are stable: they are
 * saved in databases, and rtabmap_ros converts them to and from ROS's
 * compressed_depth_image_transport messages without decompressing the images.
 *
 *  - ".png": a standard PNG file. 16UC1 depth images are 16 bits grayscale PNGs,
 *    32FC1 depth images (legacy) are 4-channel 8 bits PNGs holding the float bytes.
 *  - ".rvl" (16UC1):
 *      [0..7]   "DEPTHRVL"
 *      [8..11]  uint32 cols
 *      [12..15] uint32 rows
 *      [16..]   RVL data (see RvlCodec)
 *  - ".png:<maxDepth>:<quantization>" or ".rvl:<maxDepth>:<quantization>" (32FC1):
 *      [0..7]   "DEPTHINV"
 *      [8..11]  float depthQuantA = quantization*(quantization+1)
 *      [12..15] float depthQuantB = 1 - depthQuantA/maxDepth
 *      [16..]   the 16UC1 inverse depth image in ".png" or ".rvl" layout above, where
 *               0 is invalid and v>0 is the depth depthQuantA/(v-depthQuantB).
 */

/** @brief Signature of the ".rvl" layout (8 bytes, not null-terminated). */
const char kCompressedDepthRvlSignature[8] = {'D', 'E', 'P', 'T', 'H', 'R', 'V', 'L'};
/** @brief Size of the ".rvl" header: signature, uint32 cols, uint32 rows. */
const size_t kCompressedDepthRvlHeaderSize = 16;
/** @brief Signature of the inverse depth layout (8 bytes, not null-terminated). */
const char kCompressedDepthInvSignature[8] = {'D', 'E', 'P', 'T', 'H', 'I', 'N', 'V'};
/** @brief Size of the inverse depth header: signature, float depthQuantA, float depthQuantB. */
const size_t kCompressedDepthInvHeaderSize = 16;

/**
 * @brief Parses an image compression format "<codec>[:<maxDepth>[:<quantization>]]".
 *
 * @param format Format, e.g., @c ".jpg", @c ".png", @c ".rvl", @c ".png:10" or @c ".rvl:20:100".
 *        The empty format is valid (general zlib compression, see @ref CompressionThread).
 * @param codec Output codec (e.g., @c ".png").
 * @param maxDepth Output maximum depth (m) of the inverse depth format, 0 if not set.
 * @param quantization Output depth quantization of the inverse depth format
 *        (100 if not set but @p maxDepth is), 0 if @p maxDepth is not set.
 * @return false if the format is invalid. The inverse depth parameters are only
 *        valid with @c ".png" and @c ".rvl", and should be positive.
 */
bool RTABMAP_CORE_EXPORT parseImageCompressionFormat(const std::string & format, std::string & codec, float & maxDepth, float & quantization);

/**
 * @brief Encodes @p image to a byte buffer (OpenCV @c imencode or RVL for depth).
 *
 * @param format @c ".png", @c ".jpg" or @c ".rvl" (16UC1 only), optionally followed by
 *        @c ":<maxDepth>[:<quantization>]" (see @ref parseImageCompressionFormat()).
 *        For 32FC1 depth images, if @c maxDepth is set, depth is quantized on 16 bits
 *        as inverse depth (as ROS's @c compressed_depth_image_transport) and compressed
 *        with the codec. With A=quantization*(quantization+1) and B=1-A/maxDepth, the
 *        precision is ~d^2/(2A), and depth values over @c maxDepth or under A/(65535-B)
 *        are lost (set to 0). For example, ".png:10:100" keeps depth between 0.15 and
 *        10 m with errors of 0.05 mm at 1 m and 5 mm at 10 m. Otherwise, 32FC1 depth
 *        images are compressed losslessly as 4-channel 8 bits PNG (legacy format). Other
 *        image types ignore the depth parameters.
 */
std::vector<unsigned char> RTABMAP_CORE_EXPORT compressImage(const cv::Mat & image, const std::string & format = ".png");
/** @brief Same as @ref compressImage() but returns a @c CV_8UC1 row matrix. */
cv::Mat RTABMAP_CORE_EXPORT compressImage2(const cv::Mat & image, const std::string & format = ".png");

/** @brief Decodes compressed image bytes to a @cv::Mat. */
cv::Mat RTABMAP_CORE_EXPORT uncompressImage(const cv::Mat & bytes);
/** @brief Decodes compressed image bytes to a @cv::Mat. */
cv::Mat RTABMAP_CORE_EXPORT uncompressImage(const std::vector<unsigned char> & bytes);
/** @brief Decodes compressed image bytes to a @cv::Mat. */
cv::Mat RTABMAP_CORE_EXPORT uncompressImage(const unsigned char * bytes, size_t size);

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
 * @return @c ".rvl" if the buffer has an RVL signature, @c ".png:<maxDepth>:<quantization>"
 *         or @c ".rvl:<maxDepth>:<quantization>" for inverse depth images (see
 *         @ref compressImage()), otherwise @c ".png". The returned format can be passed
 *         back to @ref compressImage() to compress in the same format.
 */
std::string RTABMAP_CORE_EXPORT compressedDepthFormat(const cv::Mat & bytes);
/** @overload */
std::string RTABMAP_CORE_EXPORT compressedDepthFormat(const std::vector<unsigned char> & bytes);
/** @overload */
std::string RTABMAP_CORE_EXPORT compressedDepthFormat(const unsigned char * bytes, size_t size);


} /* namespace rtabmap */
#endif /* COMPRESSION_H_ */
