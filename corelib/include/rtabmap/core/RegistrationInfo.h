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

#ifndef REGISTRATIONINFO_H_
#define REGISTRATIONINFO_H_


namespace rtabmap {

/**
 * @class RegistrationInfo
 * @brief Statistics and diagnostics returned by @ref Registration.
 *
 * Filled by @ref RegistrationVis and/or @ref RegistrationIcp during
 * @ref Registration::computeTransformation(). The 6×6 @ref covariance matrix
 * is clamped to @ref Registration::COVARIANCE_LINEAR_EPSILON and
 * @ref Registration::COVARIANCE_ANGULAR_EPSILON on the diagonal when empty or
 * too small.
 *
 * Visual fields are set by @ref RegistrationVis; ICP fields by @ref RegistrationIcp.
 * In a combined Vis+ICP pipeline, both sets may be populated in the same object.
 */
class RegistrationInfo
{
public:
	RegistrationInfo() :
		totalTime(0.0),
		inliers(0),
		inliersRatio(0),
		inliersMeanDistance(0.0f),
		inliersDistribution(0.0f),
		matches(0),
		variance(0.0f),
		icpInliersRatio(0),
		icpTranslation(0.0f),
		icpRotation(0.0f),
		icpStructuralComplexity(0.0f),
		icpStructuralDistribution(0.0f),
		icpCorrespondences(0),
		icpRMS(0)

	{
	}

	/** @brief Copies scalar metrics and @ref covariance; omits correspondence ID vectors. */
	RegistrationInfo copyWithoutData() const
	{
		RegistrationInfo output;
		output.totalTime = totalTime;
		output.covariance = covariance.clone();
		output.rejectedMsg = rejectedMsg;
		output.inliers = inliers;
		output.inliersPerCam = inliersPerCam;
		output.inliersMeanDistance = inliersMeanDistance;
		output.inliersDistribution = inliersDistribution;
		output.matches = matches;
		output.matchesPerCam = matchesPerCam;
		output.variance = variance;
		output.icpInliersRatio = icpInliersRatio;
		output.icpTranslation = icpTranslation;
		output.icpRotation = icpRotation;
		output.icpStructuralComplexity = icpStructuralComplexity;
		output.icpStructuralDistribution = icpStructuralDistribution;
		output.icpCorrespondences = icpCorrespondences;
        output.icpRMS = icpRMS;
		return output;
	}

	cv::Mat covariance;           /**< 6×6 registration uncertainty (CV_64FC1). */
	std::string rejectedMsg;      /**< Reason the registration was rejected, if any. */
	double totalTime;             /**< Total registration time (seconds). */

	// RegistrationVis
	int inliers;                  /**< Number of visual inliers. */
	float inliersRatio;           /**< Ratio of inliers to matches. */
	float inliersMeanDistance;    /**< Mean reprojection or descriptor distance of inliers. */
	float inliersDistribution;  /**< Spatial spread of inliers in the image. */
	std::vector<int> inliersIDs; /**< Indices of inlier keypoints. */
	int matches;                  /**< Total visual matches before outlier rejection. */
	float variance;               /**< Estimated variance of the visual constraint. */
	std::vector<int> matchesIDs; /**< Indices of all matches. */
	std::vector<int> projectedIDs; /**< Source ("from") feature IDs used in projection. */
	std::vector<int> inliersPerCam; /**< Inlier count per camera (multi-camera). */
	std::vector<int> matchesPerCam; /**< Match count per camera (multi-camera). */

	// RegistrationIcp
	float icpInliersRatio;        /**< Ratio of ICP inlier correspondences. */
	float icpTranslation;         /**< Translation component of the ICP correction (m). */
	float icpRotation;            /**< Rotation component of the ICP correction (rad). */
	float icpStructuralComplexity; /**< Structural complexity of the scan overlap. */
	float icpStructuralDistribution; /**< Distribution of structural features in the overlap. */
	int   icpCorrespondences;     /**< Number of ICP point correspondences. */
	float icpRMS;                 /**< Root-mean-square error of ICP correspondences (m). */
};

}

#endif /* REGISTRATIONINFO_H_ */
