/*
Copyright (c) 2010-2014, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
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

// Feature2D
class RTABMAP_EXP Feature2D {
public:
	enum Type {kFeatureUndef=-1,
		kFeatureSurf=0,
		kFeatureSift=1,
		kFeatureOrb=2,
		kFeatureFastFreak=3,
		kFeatureFastBrief=4,
		kFeatureGfttFreak=5,
		kFeatureGfttBrief=6,
		kFeatureBrisk=7};

	static Feature2D * create(Feature2D::Type & type, const ParametersMap & parameters);

	static void filterKeypointsByDepth(
				std::vector<cv::KeyPoint> & keypoints,
				const cv::Mat & depth,
				float maxDepth);
	static void filterKeypointsByDepth(
			std::vector<cv::KeyPoint> & keypoints,
			cv::Mat & descriptors,
			const cv::Mat & depth,
			float maxDepth);

	static void filterKeypointsByDisparity(
				std::vector<cv::KeyPoint> & keypoints,
				const cv::Mat & disparity,
				float minDisparity);
	static void filterKeypointsByDisparity(
			std::vector<cv::KeyPoint> & keypoints,
			cv::Mat & descriptors,
			const cv::Mat & disparity,
			float minDisparity);

	static void limitKeypoints(std::vector<cv::KeyPoint> & keypoints, int maxKeypoints);
	static void limitKeypoints(std::vector<cv::KeyPoint> & keypoints, cv::Mat & descriptors, int maxKeypoints);

	static cv::Rect computeRoi(const cv::Mat & image, const std::string & roiRatios);
	static cv::Rect computeRoi(const cv::Mat & image, const std::vector<float> & roiRatios);

public:
	virtual ~Feature2D() {}

	std::vector<cv::KeyPoint> generateKeypoints(const cv::Mat & image, int maxKeypoints=0, const cv::Rect & roi = cv::Rect()) const;
	cv::Mat generateDescriptors(const cv::Mat & image, std::vector<cv::KeyPoint> & keypoints) const;

	virtual void parseParameters(const ParametersMap & parameters) {}
	virtual Feature2D::Type getType() const = 0;

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

	virtual void parseParameters(const ParametersMap & parameters);
	virtual Feature2D::Type getType() const {return kFeatureSurf;}

private:
	virtual std::vector<cv::KeyPoint> generateKeypointsImpl(const cv::Mat & image, const cv::Rect & roi) const;
	virtual cv::Mat generateDescriptorsImpl(const cv::Mat & image, std::vector<cv::KeyPoint> & keypoints) const;

private:
	double hessianThreshold_;
	int nOctaves_;
	int nOctaveLayers_;
	bool extended_;
	bool upright_;
	float gpuKeypointsRatio_;
	bool gpuVersion_;

	cv::SURF * _surf;
	cv::gpu::SURF_GPU * _gpuSurf;
};

//SIFT
class RTABMAP_EXP SIFT : public Feature2D
{
public:
	SIFT(const ParametersMap & parameters = ParametersMap());
	virtual ~SIFT();

	virtual void parseParameters(const ParametersMap & parameters);
	virtual Feature2D::Type getType() const {return kFeatureSift;}

private:
	virtual std::vector<cv::KeyPoint> generateKeypointsImpl(const cv::Mat & image, const cv::Rect & roi) const;
	virtual cv::Mat generateDescriptorsImpl(const cv::Mat & image, std::vector<cv::KeyPoint> & keypoints) const;

private:
	int nfeatures_;
	int nOctaveLayers_;
	double contrastThreshold_;
	double edgeThreshold_;
	double sigma_;

	cv::SIFT * _sift;
};

//ORB
class RTABMAP_EXP ORB : public Feature2D
{
public:
	ORB(const ParametersMap & parameters = ParametersMap());
	virtual ~ORB();

	virtual void parseParameters(const ParametersMap & parameters);
	virtual Feature2D::Type getType() const {return kFeatureOrb;}

private:
	virtual std::vector<cv::KeyPoint> generateKeypointsImpl(const cv::Mat & image, const cv::Rect & roi) const;
	virtual cv::Mat generateDescriptorsImpl(const cv::Mat & image, std::vector<cv::KeyPoint> & keypoints) const;

private:
	int nFeatures_;
	float scaleFactor_;
	int nLevels_;
	int edgeThreshold_;
	int firstLevel_;
	int WTA_K_;
	int scoreType_;
	int patchSize_;
	bool gpu_;

	int fastThreshold_;
	bool nonmaxSuppresion_;

	cv::ORB * _orb;
	cv::gpu::ORB_GPU * _gpuOrb;
};

//FAST
class RTABMAP_EXP FAST : public Feature2D
{
public:
	FAST(const ParametersMap & parameters = ParametersMap());
	virtual ~FAST();

	virtual void parseParameters(const ParametersMap & parameters);

private:
	virtual std::vector<cv::KeyPoint> generateKeypointsImpl(const cv::Mat & image, const cv::Rect & roi) const;

private:
	int threshold_;
	bool nonmaxSuppression_;
	bool gpu_;
	double gpuKeypointsRatio_;

	cv::FastFeatureDetector * _fast;
	cv::gpu::FAST_GPU * _gpuFast;
};

//FAST_BRIEF
class RTABMAP_EXP FAST_BRIEF : public FAST
{
public:
	FAST_BRIEF(const ParametersMap & parameters = ParametersMap());
	virtual ~FAST_BRIEF();

	virtual void parseParameters(const ParametersMap & parameters);
	virtual Feature2D::Type getType() const {return kFeatureFastBrief;}

private:
	virtual cv::Mat generateDescriptorsImpl(const cv::Mat & image, std::vector<cv::KeyPoint> & keypoints) const;

private:
	int bytes_;

	cv::BriefDescriptorExtractor * _brief;
};

//FAST_FREAK
class RTABMAP_EXP FAST_FREAK : public FAST
{
public:
	FAST_FREAK(const ParametersMap & parameters = ParametersMap());
	virtual ~FAST_FREAK();

	virtual void parseParameters(const ParametersMap & parameters);
	virtual Feature2D::Type getType() const {return kFeatureFastFreak;}

private:
	virtual cv::Mat generateDescriptorsImpl(const cv::Mat & image, std::vector<cv::KeyPoint> & keypoints) const;

private:
	bool orientationNormalized_;
	bool scaleNormalized_;
	float patternScale_;
	int nOctaves_;

	cv::FREAK * _freak;
};

//GFTT
class RTABMAP_EXP GFTT : public Feature2D
{
public:
	GFTT(const ParametersMap & parameters = ParametersMap());
	virtual ~GFTT();

	virtual void parseParameters(const ParametersMap & parameters);

private:
	virtual std::vector<cv::KeyPoint> generateKeypointsImpl(const cv::Mat & image, const cv::Rect & roi) const;

private:
	int _maxCorners;
	double _qualityLevel;
	double _minDistance;
	int _blockSize;
	bool _useHarrisDetector;
	double _k;

	cv::GFTTDetector * _gftt;
};

//GFTT_BRIEF
class RTABMAP_EXP GFTT_BRIEF : public GFTT
{
public:
	GFTT_BRIEF(const ParametersMap & parameters = ParametersMap());
	virtual ~GFTT_BRIEF();

	virtual void parseParameters(const ParametersMap & parameters);
	virtual Feature2D::Type getType() const {return kFeatureGfttBrief;}

private:
	virtual cv::Mat generateDescriptorsImpl(const cv::Mat & image, std::vector<cv::KeyPoint> & keypoints) const;

private:
	int bytes_;

	cv::BriefDescriptorExtractor * _brief;
};

//GFTT_FREAK
class RTABMAP_EXP GFTT_FREAK : public GFTT
{
public:
	GFTT_FREAK(const ParametersMap & parameters = ParametersMap());
	virtual ~GFTT_FREAK();

	virtual void parseParameters(const ParametersMap & parameters);
	virtual Feature2D::Type getType() const {return kFeatureGfttFreak;}

private:
	virtual cv::Mat generateDescriptorsImpl(const cv::Mat & image, std::vector<cv::KeyPoint> & keypoints) const;

private:
	bool orientationNormalized_;
	bool scaleNormalized_;
	float patternScale_;
	int nOctaves_;

	cv::FREAK * _freak;
};

//BRISK
class RTABMAP_EXP BRISK : public Feature2D
{
public:
	BRISK(const ParametersMap & parameters = ParametersMap());
	virtual ~BRISK();

	virtual void parseParameters(const ParametersMap & parameters);
	virtual Feature2D::Type getType() const {return kFeatureBrisk;}

private:
	virtual std::vector<cv::KeyPoint> generateKeypointsImpl(const cv::Mat & image, const cv::Rect & roi) const;
	virtual cv::Mat generateDescriptorsImpl(const cv::Mat & image, std::vector<cv::KeyPoint> & keypoints) const;

private:
	int thresh_;
	int octaves_;
	float patternScale_;

	cv::BRISK * brisk_;
};


}

#endif /* KEYPOINTDESCRIPTOR_H_ */
