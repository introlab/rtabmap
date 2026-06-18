/*
Copyright (c) 2010-2018, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
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

#ifndef CORELIB_INCLUDE_RTABMAP_CORE_LANDMARK_H_
#define CORELIB_INCLUDE_RTABMAP_CORE_LANDMARK_H_

#include <rtabmap/core/rtabmap_core_export.h>
#include <rtabmap/core/Transform.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UMath.h>
#include <rtabmap/utilite/UConversion.h>

namespace rtabmap {

/**
 * @class Landmark
 * @brief Optimized pose of a visual landmark (e.g. ArUco/AprilTag) in the map.
 *
 * Stores a positive landmark @ref id(), optional physical @ref size() (m), world
 * @ref pose(), and a 6×6 pose covariance matrix (x, y, z, roll, pitch, yaw).
 *
 * Used in @ref Landmarks maps on @ref SensorData and in memory during SLAM.
 * Graph constraints to landmarks use @ref Link::kLandmark with a **negative**
 * landmark id on the link (`to` is typically `-id()`).
 *
 * Covariance diagonal entries must be finite and strictly positive. Use a large
 * value (e.g. `9999`) on angular axes when orientation uncertainty is unknown.
 *
 * @see SensorData::setLandmarks()
 * @see Signature::addLandmark()
 */
class RTABMAP_CORE_EXPORT Landmark
{
public:
	/** @brief Default-constructs an invalid landmark (`id == 0`). */
	Landmark() :
		id_(0),
		size_(0.0f)
	{}

	/**
	 * @brief Constructs a landmark with size, pose, and covariance.
	 * @param id Positive landmark identifier.
	 * @param size Physical size of the marker (m); `0` if unknown.
	 * @param pose Landmark pose in the base frame (typically the robot frame).
	 * @param covariance 6×6 `CV_64FC1` covariance (diagonal entries must be > 0).
	 */
	Landmark(const int & id, const float & size, const Transform & pose, const cv::Mat & covariance) :
		id_(id),
		size_(size),
		pose_(pose),
		covariance_(covariance)
	{
		UASSERT(id_>0);
		UASSERT(!pose_.isNull());
		UASSERT(covariance_.cols == 6 && covariance_.rows == 6 && covariance_.type() == CV_64FC1);
		UASSERT_MSG(uIsFinite(covariance_.at<double>(0,0)) && covariance_.at<double>(0,0)>0, uFormat("Linear covariance should not be null! Value=%f.", covariance_.at<double>(0,0)).c_str());
		UASSERT_MSG(uIsFinite(covariance_.at<double>(1,1)) && covariance_.at<double>(1,1)>0, uFormat("Linear covariance should not be null! Value=%f.", covariance_.at<double>(1,1)).c_str());
		UASSERT_MSG(uIsFinite(covariance_.at<double>(2,2)) && covariance_.at<double>(2,2)>0, uFormat("Linear covariance should not be null! Value=%f.", covariance_.at<double>(2,2)).c_str());
		UASSERT_MSG(uIsFinite(covariance_.at<double>(3,3)) && covariance_.at<double>(3,3)>0, uFormat("Angular covariance should not be null! Value=%f (set to 9999 if unknown).", covariance_.at<double>(3,3)).c_str());
		UASSERT_MSG(uIsFinite(covariance_.at<double>(4,4)) && covariance_.at<double>(4,4)>0, uFormat("Angular covariance should not be null! Value=%f (set to 9999 if unknown).", covariance_.at<double>(4,4)).c_str());
		UASSERT_MSG(uIsFinite(covariance_.at<double>(5,5)) && covariance_.at<double>(5,5)>0, uFormat("Angular covariance should not be null! Value=%f (set to 9999 if unknown).", covariance_.at<double>(5,5)).c_str());
	}

	/**
	 * @brief Deprecated; use the constructor with explicit @p size (`0` if unknown).
	 */
	RTABMAP_DEPRECATED Landmark(const int & id, const Transform & pose, const cv::Mat & covariance);

	virtual ~Landmark() {}

	/** @return Positive landmark id (map key in @ref Landmarks). */
	const int & id() const {return id_;}
	/** @return Marker size in metres (`0` if unknown). */
	const float & size() const {return size_;}
	/** @return Landmark pose in the map frame. */
	const Transform & pose() const {return pose_;}
	/** @return 6×6 pose covariance (`CV_64FC1`). */
	const cv::Mat & covariance() const {return covariance_;}

private:
	int id_;
	float size_;
	Transform pose_;
	cv::Mat covariance_;
};

/** @brief Map of landmark id → @ref Landmark (typically positive keys). */
typedef std::map<int, Landmark> Landmarks;

inline Landmark::Landmark(const int & id, const Transform & pose, const cv::Mat & covariance) :
	id_(id),
	size_(0.0f),
	pose_(pose),
	covariance_(covariance)
{
	UASSERT(id_>0);
	UASSERT(!pose_.isNull());
	UASSERT(covariance_.cols == 6 && covariance_.rows == 6 && covariance_.type() == CV_64FC1);
	UASSERT_MSG(uIsFinite(covariance_.at<double>(0,0)) && covariance_.at<double>(0,0)>0, uFormat("Linear covariance should not be null! Value=%f.", covariance_.at<double>(0,0)).c_str());
	UASSERT_MSG(uIsFinite(covariance_.at<double>(1,1)) && covariance_.at<double>(1,1)>0, uFormat("Linear covariance should not be null! Value=%f.", covariance_.at<double>(1,1)).c_str());
	UASSERT_MSG(uIsFinite(covariance_.at<double>(2,2)) && covariance_.at<double>(2,2)>0, uFormat("Linear covariance should not be null! Value=%f.", covariance_.at<double>(2,2)).c_str());
	UASSERT_MSG(uIsFinite(covariance_.at<double>(3,3)) && covariance_.at<double>(3,3)>0, uFormat("Angular covariance should not be null! Value=%f (set to 9999 if unknown).", covariance_.at<double>(3,3)).c_str());
	UASSERT_MSG(uIsFinite(covariance_.at<double>(4,4)) && covariance_.at<double>(4,4)>0, uFormat("Angular covariance should not be null! Value=%f (set to 9999 if unknown).", covariance_.at<double>(4,4)).c_str());
	UASSERT_MSG(uIsFinite(covariance_.at<double>(5,5)) && covariance_.at<double>(5,5)>0, uFormat("Angular covariance should not be null! Value=%f (set to 9999 if unknown).", covariance_.at<double>(5,5)).c_str());
}

} // namespace rtabmap

#endif /* CORELIB_INCLUDE_RTABMAP_CORE_LANDMARK_H_ */
