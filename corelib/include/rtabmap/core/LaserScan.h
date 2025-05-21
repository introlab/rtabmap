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

#ifndef CORELIB_INCLUDE_RTABMAP_CORE_LASERSCAN_H_
#define CORELIB_INCLUDE_RTABMAP_CORE_LASERSCAN_H_

#include "rtabmap/core/rtabmap_core_export.h" // DLL export/import defines

#include <rtabmap/core/Transform.h>

namespace rtabmap {

/**
 * @class LaserScan
 * @brief Represents 2D or 3D laser scan data with support for multiple point data formats.
 *
 * The LaserScan class stores structured laser scan data used in SLAM, mapping, and perception.
 * It supports various formats including point coordinates, intensity, normals, RGB colors,
 * and timestamps. Utility methods are provided for format checking, cloning, and combining scans.
 */
class RTABMAP_CORE_EXPORT LaserScan
{
public:
	/**
	 * @brief Enumeration of possible formats for laser scan data.
	 *
	 * These values represent different combinations of point attributes
	 * that can be stored in a laser scan. The format determines how each
	 * point in the scan is structured.
	 */
	enum Format{
		kUnknown=0,			/**< Unknown format. */
		kXY=1,				/**< 2D points with X and Y coordinates. */
		kXYI=2,				/**< 2D points with X, Y and intensity. */
		kXYNormal=3,		/**< 2D points with X, Y and normal vectors. */
		kXYINormal=4,		/**< 2D points with X, Y, intensity and normal vectors. */
		kXYZ=5,				/**< 3D points with X, Y and Z coordinates. */
		kXYZI=6,			/**< 3D points with X, Y, Z and intensity. */
		kXYZRGB=7,			/**< 3D points with X, Y, Z and RGB color. */
		kXYZNormal=8,		/**< 3D points with X, Y, Z and normal vectors. */
		kXYZINormal=9,		/**< 3D points with X, Y, Z, intensity and normal vectors. */
		kXYZRGBNormal=10,	/**< 3D points with X, Y, Z, RGB color and normal vectors. */
		kXYZIT=11			/**< 3D points with X, Y, Z, intensity and time. */
	};

	/// @name Static Utility Functions
	/// @{
	static std::string formatName(const Format & format);
	static int channels(const Format & format);
	static bool isScan2d(const Format & format);
	static bool isScanHasNormals(const Format & format);
	static bool isScanHasRGB(const Format & format);
	static bool isScanHasIntensity(const Format & format);
	static bool isScanHasTime(const Format & format);
	static float packRGB(unsigned char r, unsigned char g, unsigned char b);
	static void unpackRGB(float rgb, unsigned char & r, unsigned char & g, unsigned char & b);

	/**
	 * @brief Converts legacy scan format to a LaserScan object.
	 */
	static LaserScan backwardCompatibility(
			const cv::Mat & oldScanFormat,
			int maxPoints = 0,
			int maxRange = 0,
			const Transform & localTransform = Transform::getIdentity());
	
	/**
	 * @brief Converts legacy scan format with additional metadata to a LaserScan.
	 */
	static LaserScan backwardCompatibility(
			const cv::Mat & oldScanFormat,
			float minRange,
			float maxRange,
			float angleMin,
			float angleMax,
			float angleInc,
			const Transform & localTransform = Transform::getIdentity());
	/// @}

public:

	/// @name Constructors
	/// @{
	LaserScan();
	LaserScan(const LaserScan & data,
			int maxPoints,
			float maxRange,
			const Transform & localTransform = Transform::getIdentity());
	
	/// @deprecated Use constructor without `format` argument.
	RTABMAP_DEPRECATED LaserScan(const LaserScan & data,
			int maxPoints,
			float maxRange,
			Format format,
			const Transform & localTransform = Transform::getIdentity());
	LaserScan(const cv::Mat & data,
			int maxPoints,
			float maxRange,
			Format format,
			const Transform & localTransform = Transform::getIdentity());
	
	/// @deprecated Use constructor without `format` argument.
	RTABMAP_DEPRECATED LaserScan(const LaserScan & data,
			Format format,
			float minRange,
			float maxRange,
			float angleMin,
			float angleMax,
			float angleIncrement,
			const Transform & localTransform = Transform::getIdentity());
	LaserScan(const LaserScan & data,
			float minRange,
			float maxRange,
			float angleMin,
			float angleMax,
			float angleIncrement,
			const Transform & localTransform = Transform::getIdentity());
	LaserScan(const cv::Mat & data,
			Format format,
			float minRange,
			float maxRange,
			float angleMin,
			float angleMax,
			float angleIncrement,
			const Transform & localTransform = Transform::getIdentity());
	/// @}

	/// @name Accessors
	/// @{
	const cv::Mat & data() const {return data_;}
	Format format() const {return format_;}
	std::string formatName() const {return formatName(format_);}
	int channels() const {return data_.channels();}
	int maxPoints() const {return maxPoints_;}
	float rangeMin() const {return rangeMin_;}
	float rangeMax() const {return rangeMax_;}
	float angleMin() const {return angleMin_;}
	float angleMax() const {return angleMax_;}
	float angleIncrement() const {return angleIncrement_;}
	void setLocalTransform(const Transform & t) {localTransform_ = t;}
	Transform localTransform() const {return localTransform_;}
	/// @}

	/// @name Status and Format Checks
	/// @{
	bool empty() const {return data_.empty();}
	bool isEmpty() const {return data_.empty();}
	int size() const {return data_.total();}
	int dataType() const {return data_.type();}
	bool is2d() const {return isScan2d(format_);}
	bool hasNormals() const {return isScanHasNormals(format_);}
	bool hasRGB() const {return isScanHasRGB(format_);}
	bool hasIntensity() const {return isScanHasIntensity(format_);}
	bool hasTime() const {return isScanHasTime(format_);}
	bool isCompressed() const {return !data_.empty() && data_.type()==CV_8UC1;}
	bool isOrganized() const {return data_.rows > 1;}
	LaserScan clone() const;
	LaserScan densify() const;
	/// @}

	/// @name Operations
	/// @{
	int getIntensityOffset() const {return hasIntensity()?(is2d()?2:3):-1;}
	int getRGBOffset() const {return hasRGB()?(is2d()?2:3):-1;}
	int getNormalsOffset() const {return hasNormals()?(2 + (is2d()?0:1) + ((hasRGB() || hasIntensity())?1:0)):-1;}
	int getTimeOffset() const {return hasTime()?4:-1;}

	/**
	 * @brief Access a specific field value of a point.
	 * @param pointIndex Index of the point.
	 * @param channelOffset Channel offset to access (e.g., 0=X, 1=Y, 2=Z if 2D, for other fields, use corresponding getter functions).
	 * @return Reference to the field value.
	 * @see getIntensityOffset() getRGBOffset() getNormalsOffset() getTimeOffset()
	 */
	float & field(unsigned int pointIndex, unsigned int channelOffset);

	/**
	 * @brief Clear the scan data.
	 */
	void clear() {data_ = cv::Mat();}

	/**
	 * @brief Concatenate scan's data (localTransform is ignored).
	 */
	LaserScan & operator+=(const LaserScan &);
	/**
	 * @brief Concatenate scan's data (localTransform is ignored).
	 */
	LaserScan operator+(const LaserScan &);
	/// @}

private:
	void init(const cv::Mat & data,
			Format format,
			float minRange,
			float maxRange,
			float angleMin,
			float angleMax,
			float angleIncrement,
			int maxPoints,
			const Transform & localTransform = Transform::getIdentity());

private:
	cv::Mat data_;				///< The scan data matrix.
	Format format_;				///< The scan data format.
	int maxPoints_;				///< Maximum number of points allowed.
	float rangeMin_;			///< Minimum valid range.
	float rangeMax_;			///< Maximum valid range.
	float angleMin_;			///< Minimum angle (for 2D scans).
	float angleMax_;			///< Maximum angle (for 2D scans).
	float angleIncrement_;		///< Angular increment (for 2D scans).
	Transform localTransform_;	///< Transform from base frame to scan frame.
};

} // namespace rtabmap

#endif /* CORELIB_INCLUDE_RTABMAP_CORE_LASERSCAN_H_ */
