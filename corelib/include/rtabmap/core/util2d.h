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

#ifndef UTIL2D_H_
#define UTIL2D_H_

#include <rtabmap/core/RtabmapExp.h>

#include <opencv2/core/core.hpp>
#include <rtabmap/core/Transform.h>
#include <rtabmap/core/Parameters.h>
#include <vector>

namespace rtabmap
{

namespace util2d
{

// SSD: Sum of Squared Differences
float RTABMAP_EXP ssd(const cv::Mat & windowLeft, const cv::Mat & windowRight);
// SAD: Sum of Absolute intensity Differences
float RTABMAP_EXP sad(const cv::Mat & windowLeft, const cv::Mat & windowRight);

std::vector<cv::Point2f> RTABMAP_EXP calcStereoCorrespondences(
		const cv::Mat & leftImage,
		const cv::Mat & rightImage,
		const std::vector<cv::Point2f> & leftCorners,
		std::vector<unsigned char> & status,
		cv::Size winSize = cv::Size(6,3),
		int maxLevel = 3,
		int iterations = 5,
		float minDisparity = 0.0f,
		float maxDisparity = 64.0f,
		bool ssdApproach = true); // SSD by default, otherwise it is SAD

// exactly as cv::calcOpticalFlowPyrLK but it should be called with pyramid (from cv::buildOpticalFlowPyramid()) and delta drops the y error.
void RTABMAP_EXP calcOpticalFlowPyrLKStereo( cv::InputArray _prevImg, cv::InputArray _nextImg,
                           cv::InputArray _prevPts, cv::InputOutputArray _nextPts,
                           cv::OutputArray _status, cv::OutputArray _err,
                           cv::Size winSize = cv::Size(15,3), int maxLevel = 3,
						   cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01),
						   int flags = 0, double minEigThreshold = 1e-4 );


cv::Mat RTABMAP_EXP disparityFromStereoImages(
		const cv::Mat & leftImage,
		const cv::Mat & rightImage,
	    const ParametersMap & parameters = ParametersMap());

cv::Mat RTABMAP_EXP depthFromDisparity(const cv::Mat & disparity,
		float fx, float baseline,
		int type = CV_32FC1); // CV_32FC1 or CV_16UC1

cv::Mat RTABMAP_EXP depthFromStereoImages(
		const cv::Mat & leftImage,
		const cv::Mat & rightImage,
		const std::vector<cv::Point2f> & leftCorners,
		float fx,
		float baseline,
		int flowWinSize = 9,
		int flowMaxLevel = 4,
		int flowIterations = 20,
		double flowEps = 0.02);

cv::Mat RTABMAP_EXP disparityFromStereoCorrespondences(
		const cv::Size & disparitySize,
		const std::vector<cv::Point2f> & leftCorners,
		const std::vector<cv::Point2f> & rightCorners,
		const std::vector<unsigned char> & mask);

cv::Mat RTABMAP_EXP depthFromStereoCorrespondences(
		const cv::Mat & leftImage,
		const std::vector<cv::Point2f> & leftCorners,
		const std::vector<cv::Point2f> & rightCorners,
		const std::vector<unsigned char> & mask,
		float fx, float baseline);

cv::Mat RTABMAP_EXP cvtDepthFromFloat(const cv::Mat & depth32F);
cv::Mat RTABMAP_EXP cvtDepthToFloat(const cv::Mat & depth16U);

float RTABMAP_EXP getDepth(
		const cv::Mat & depthImage,
		float x, float y,
		bool smoothing,
		float depthErrorRatio = 0.02f, //ratio
		bool estWithNeighborsIfNull = false);

cv::Rect RTABMAP_EXP computeRoi(const cv::Mat & image, const std::string & roiRatios);
cv::Rect RTABMAP_EXP computeRoi(const cv::Size & imageSize, const std::string & roiRatios);
cv::Rect RTABMAP_EXP computeRoi(const cv::Mat & image, const std::vector<float> & roiRatios);
cv::Rect RTABMAP_EXP computeRoi(const cv::Size & imageSize, const std::vector<float> & roiRatios);

cv::Mat RTABMAP_EXP decimate(const cv::Mat & image, int d);
cv::Mat RTABMAP_EXP interpolate(const cv::Mat & image, int factor, float depthErrorRatio = 0.02f);

// Registration Depth to RGB (return registered depth image)
cv::Mat RTABMAP_EXP registerDepth(
		const cv::Mat & depth,
		const cv::Mat & depthK,
		const cv::Size & colorSize,
		const cv::Mat & colorK,
		const rtabmap::Transform & transform);

cv::Mat RTABMAP_EXP fillDepthHoles(
		const cv::Mat & depth,
		int maximumHoleSize = 1,
		float errorRatio = 0.02f);

void RTABMAP_EXP fillRegisteredDepthHoles(
		cv::Mat & depthRegistered,
		bool vertical,
		bool horizontal,
		bool fillDoubleHoles = false);

cv::Mat RTABMAP_EXP fastBilateralFiltering(
		const cv::Mat & depth,
		float sigmaS = 15.0f,
		float sigmaR = 0.05f,
		bool earlyDivision = false);

cv::Mat RTABMAP_EXP brightnessAndContrastAuto(
		const cv::Mat & src,
		const cv::Mat & mask,
		float clipLowHistPercent=0,
		float clipHighHistPercent=0);

cv::Mat RTABMAP_EXP exposureFusion(
	const std::vector<cv::Mat> & images);

} // namespace util3d
} // namespace rtabmap

#endif /* UTIL2D_H_ */
