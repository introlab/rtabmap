/*
 * Copyright (C) 2010-2011, Mathieu Labbe and IntRoLab - Universite de Sherbrooke
 *
 * This file is part of RTAB-Map.
 *
 * RTAB-Map is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * RTAB-Map is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with RTAB-Map.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef KEYPOINTDESCRIPTOR_H_
#define KEYPOINTDESCRIPTOR_H_

#include "rtabmap/core/RtabmapExp.h" // DLL export/import defines

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <list>
#include "rtabmap/core/Parameters.h"

namespace cv{
class SURF;
class SIFT;
namespace gpu {
	class SURF_GPU;
	class ORB_GPU;
	class FAST_GPU;
}
}

namespace rtabmap {

void RTABMAP_EXP filterKeypointsByDepth(
			std::vector<cv::KeyPoint> & keypoints,
			const cv::Mat & depth,
			float depthConstant,
			float maxDepth);
void RTABMAP_EXP filterKeypointsByDepth(
		std::vector<cv::KeyPoint> & keypoints,
		cv::Mat & descriptors,
		const cv::Mat & depth,
		float depthConstant,
		float maxDepth);

void RTABMAP_EXP limitKeypoints(std::vector<cv::KeyPoint> & keypoints, int maxKeypoints);
void RTABMAP_EXP limitKeypoints(std::vector<cv::KeyPoint> & keypoints, cv::Mat & descriptors, int maxKeypoints);

cv::Rect RTABMAP_EXP computeRoi(const cv::Mat & image, const std::vector<float> & roiRatios);

// Feature2D
class RTABMAP_EXP Feature2D {
public:
	enum Type {kFeatureUndef=-1, kFeatureSurf=0, kFeatureSift=1, kFeatureOrb=2, kFeatureFastFreak=3, kFeatureFastBrief=4};

public:
	virtual ~Feature2D() {}

	std::vector<cv::KeyPoint> generateKeypoints(const cv::Mat & image, int maxKeypoints=0, const cv::Rect & roi = cv::Rect()) const;
	cv::Mat generateDescriptors(const cv::Mat & image, std::vector<cv::KeyPoint> & keypoints) const;

protected:
	Feature2D(const ParametersMap & parameters = ParametersMap()) {}

private:
	virtual std::vector<cv::KeyPoint> generateKeypointsImpl(const cv::Mat & image, const cv::Rect & roi) const = 0;
	virtual cv::Mat generateDescriptorsImpl(const cv::Mat & image, std::vector<cv::KeyPoint> & keypoints) const = 0;
};

//SURF
class RTABMAP_EXP SURF : public Feature2D
{
public:
	SURF(const ParametersMap & parameters = ParametersMap());
	virtual ~SURF();

private:
	virtual std::vector<cv::KeyPoint> generateKeypointsImpl(const cv::Mat & image, const cv::Rect & roi) const;
	virtual cv::Mat generateDescriptorsImpl(const cv::Mat & image, std::vector<cv::KeyPoint> & keypoints) const;

private:
	cv::SURF * _surf;
	cv::gpu::SURF_GPU * _gpuSurf;
};

//SIFT
class RTABMAP_EXP SIFT : public Feature2D
{
public:
	SIFT(const ParametersMap & parameters = ParametersMap());
	virtual ~SIFT();

private:
	virtual std::vector<cv::KeyPoint> generateKeypointsImpl(const cv::Mat & image, const cv::Rect & roi) const;
	virtual cv::Mat generateDescriptorsImpl(const cv::Mat & image, std::vector<cv::KeyPoint> & keypoints) const;

private:
	cv::SIFT * _sift;
};

//ORB
class RTABMAP_EXP ORB : public Feature2D
{
public:
	ORB(const ParametersMap & parameters = ParametersMap());
	virtual ~ORB();

private:
	virtual std::vector<cv::KeyPoint> generateKeypointsImpl(const cv::Mat & image, const cv::Rect & roi) const;
	virtual cv::Mat generateDescriptorsImpl(const cv::Mat & image, std::vector<cv::KeyPoint> & keypoints) const;

private:
	cv::ORB * _orb;
	cv::gpu::ORB_GPU * _gpuOrb;
};

//FAST
class RTABMAP_EXP FAST : public Feature2D
{
public:
	FAST(const ParametersMap & parameters = ParametersMap());
	virtual ~FAST();

private:
	virtual std::vector<cv::KeyPoint> generateKeypointsImpl(const cv::Mat & image, const cv::Rect & roi) const;

private:
	cv::FastFeatureDetector * _fast;
	cv::gpu::FAST_GPU * _gpuFast;
};

//FAST_BRIEF
class RTABMAP_EXP FAST_BRIEF : public FAST
{
public:
	FAST_BRIEF(const ParametersMap & parameters = ParametersMap());
	virtual ~FAST_BRIEF();

private:
	virtual cv::Mat generateDescriptorsImpl(const cv::Mat & image, std::vector<cv::KeyPoint> & keypoints) const;

private:
	cv::BriefDescriptorExtractor * _brief;
};

//FAST_FREAK
class RTABMAP_EXP FAST_FREAK : public FAST
{
public:
	FAST_FREAK(const ParametersMap & parameters = ParametersMap());
	virtual ~FAST_FREAK();

private:
	virtual cv::Mat generateDescriptorsImpl(const cv::Mat & image, std::vector<cv::KeyPoint> & keypoints) const;

private:
	cv::FREAK * _freak;
};

}

#endif /* KEYPOINTDESCRIPTOR_H_ */
