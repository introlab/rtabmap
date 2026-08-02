/*
Copyright (c) 2010-2025, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
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

#ifndef ODOMETRYINFO_H_
#define ODOMETRYINFO_H_

#include <rtabmap/core/rtabmap_core_export.h>
#include <map>
#include "rtabmap/core/Transform.h"
#include "rtabmap/core/RegistrationInfo.h"
#include "rtabmap/core/CameraModel.h"
#include "rtabmap/core/LaserScan.h"
#if CV_MAJOR_VERSION < 5
#include <opencv2/features2d/features2d.hpp>
#else
#include <opencv2/features.hpp>
#endif

namespace rtabmap {

/**
 * @class OdometryInfo
 * @brief What one @ref Odometry iteration produced, beyond the pose.
 *
 * Filled by @ref Odometry::process() when a pointer is passed to it, and carried
 * to the rest of the application by @ref OdometryEvent. It holds the incremental
 * motion, the quality indicators used to decide whether the estimate can be
 * trusted, the timings, and the intermediate data a viewer needs to draw what
 * the front-end sees (local map, matched features).
 *
 * Which fields are filled depends on the front-end: the feature-related ones
 * come from @ref OdometryF2M and @ref OdometryF2F, and an ICP-only or external
 * strategy leaves them empty.
 *
 * @see Odometry
 * @see OdometryEvent
 * @see RegistrationInfo
 */
class RTABMAP_CORE_EXPORT OdometryInfo
{
public:
	OdometryInfo();
	/** @brief A copy without the heavy members (features, local map, scan). */
	OdometryInfo copyWithoutData() const;
	/**
	 * @brief Formats the content as the `Odometry/...` statistics.
	 * @param pose Current pose, added to the output when not null.
	 * @return Statistic name (with its unit) to value, as published in @ref Statistics.
	 */
	std::map<std::string, float> statistics(const Transform & pose = Transform());

	bool lost;               ///< True when the motion could not be estimated on this frame (@ref transform is then null).
	RegistrationInfo reg;    ///< Registration result: matches, inliers, covariance, ICP indicators and timings.
	int features;            ///< Number of features extracted in the current frame.
	int localMapSize;        ///< Number of 3D points in the local feature map (F2M).
	int localScanMapSize;    ///< Number of points in the local scan map (F2M).
	int localKeyFrames;      ///< Number of key frames forming the local map (F2M).
	int localBundleOutliers; ///< Features rejected by the last local bundle adjustment.
	int localBundleConstraints; ///< Feature observations kept by the last local bundle adjustment.
	float localBundleTime;   ///< Time spent in the local bundle adjustment (s).
	std::map<int, Transform> localBundlePoses; ///< Key frame poses optimized by the local bundle adjustment.
	std::map<int, std::vector<CameraModel> > localBundleModels; ///< Camera models of @ref localBundlePoses.
	float localBundleAvgInlierDistance;    ///< Average distance of the bundle adjustment inliers (m).
	int localBundleMaxKeyFramesForInlier;  ///< Highest number of key frames observing a same inlier.
	std::vector<int> localBundleOutliersPerCam; ///< Outliers of the last local bundle adjustment, per camera.
	bool keyFrameAdded;      ///< True if this frame became a key frame of the local map.
	float timeDeskewing;     ///< Time spent deskewing the laser scan (s).
	float timeEstimation;    ///< Time spent estimating the motion (s).
	float timeParticleFiltering; ///< Time spent in the particle filter (s), when enabled.
	double stamp;            ///< Stamp of the processed frame.
	double interval;         ///< Time since the previous processed frame (s); the divisor for velocities.
	Transform transform;     ///< Motion since the previous frame, null when @ref lost.
	Transform transformFiltered; ///< @ref transform after Kalman or particle filtering, when enabled.
	Transform transformGroundTruth; ///< Ground truth motion since the previous frame, when the data provides it.
	Transform guessVelocity; ///< @deprecated Use @ref guess and @ref interval instead.
	Transform guess;         ///< Motion guess given to the front-end (from the velocity model, an external pose or an IMU).
	float distanceTravelled; ///< Distance travelled since the odometry was last reset (m).
	int memoryUsage;         ///< Process memory used (MB), only with @ref Parameters::kRtabmapPublishRAMUsage().
	double gravityRollError; ///< Absolute roll difference between the estimated pose and the IMU gravity (rad).
	double gravityPitchError;///< Absolute pitch difference between the estimated pose and the IMU gravity (rad).

	int type;                ///< Odometry strategy that produced this, see @ref Parameters::kOdomStrategy().

	// F2M
	std::multimap<int, cv::KeyPoint> words; ///< Keypoints of the current frame, by word id (F2M).
	std::map<int, cv::Point3f> localMap;    ///< Local feature map in the odometry frame, by word id (F2M).
	LaserScan localScanMap;                 ///< Local scan map in the odometry frame (F2M).

	// F2F
	std::vector<cv::Point2f> refCorners; ///< Corners of the reference frame (F2F).
	std::vector<cv::Point2f> newCorners; ///< Where those corners were found in the current frame (F2F).
	std::vector<int> cornerInliers;      ///< Indices in @ref refCorners / @ref newCorners kept as inliers (F2F).
};

}

#endif /* ODOMETRYINFO_H_ */
