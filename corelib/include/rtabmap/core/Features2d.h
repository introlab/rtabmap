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

#ifndef FEATURES2D_H_
#define FEATURES2D_H_

#include "rtabmap/core/RtabmapExp.h" // DLL export/import defines

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <list>
#include "rtabmap/core/Parameters.h"
#include "rtabmap/core/SensorData.h"

#if CV_MAJOR_VERSION < 3
namespace cv{
class SURF;
class SIFT;
namespace gpu {
	class SURF_GPU;
	class ORB_GPU;
	class FAST_GPU;
}
}
typedef cv::SIFT CV_SIFT;
typedef cv::SURF CV_SURF;
typedef cv::FastFeatureDetector CV_FAST;
typedef cv::FREAK CV_FREAK;
typedef cv::GFTTDetector CV_GFTT;
typedef cv::BriefDescriptorExtractor CV_BRIEF;
typedef cv::BRISK CV_BRISK;
typedef cv::gpu::SURF_GPU CV_SURF_GPU;
typedef cv::gpu::ORB_GPU CV_ORB_GPU;
typedef cv::gpu::FAST_GPU CV_FAST_GPU;
#else
namespace cv{
namespace xfeatures2d {
class FREAK;
class BriefDescriptorExtractor;
class SIFT;
class SURF;
}
namespace cuda {
class FastFeatureDetector;
class ORB;
class SURF_CUDA;
}
}
typedef cv::xfeatures2d::SIFT CV_SIFT;
typedef cv::xfeatures2d::SURF CV_SURF;
typedef cv::FastFeatureDetector CV_FAST;
typedef cv::xfeatures2d::FREAK CV_FREAK;
typedef cv::GFTTDetector CV_GFTT;
typedef cv::xfeatures2d::BriefDescriptorExtractor CV_BRIEF;
typedef cv::BRISK CV_BRISK;
typedef cv::ORB CV_ORB;
typedef cv::cuda::SURF_CUDA CV_SURF_GPU;
typedef cv::cuda::ORB CV_ORB_GPU;
typedef cv::cuda::FastFeatureDetector CV_FAST_GPU;
#endif


namespace rtabmap {

class Stereo;
#if CV_MAJOR_VERSION < 3
class CV_ORB;
#endif

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
		kFeatureBrisk=7,
		kFeatureGfttOrb=8,  //new 0.10.11
		kFeatureKaze=9};    //new 0.13.2

	static Feature2D * create(const ParametersMap & parameters = ParametersMap());
	static Feature2D * create(Feature2D::Type type, const ParametersMap & parameters = ParametersMap()); // for convenience

	static void filterKeypointsByDepth(
				std::vector<cv::KeyPoint> & keypoints,
				const cv::Mat & depth,
				float minDepth,
				float maxDepth);
	static void filterKeypointsByDepth(
			std::vector<cv::KeyPoint> & keypoints,
			cv::Mat & descriptors,
			const cv::Mat & depth,
			float minDepth,
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
	static void limitKeypoints(std::vector<cv::KeyPoint> & keypoints, std::vector<cv::Point3f> & keypoints3D, cv::Mat & descriptors, int maxKeypoints);
	static void limitKeypoints(const std::vector<cv::KeyPoint> & keypoints, std::vector<bool> & inliers, int maxKeypoints);

	static cv::Rect computeRoi(const cv::Mat & image, const std::string & roiRatios);
	static cv::Rect computeRoi(const cv::Mat & image, const std::vector<float> & roiRatios);

	int getMaxFeatures() const {return maxFeatures_;}
	float getMinDepth() const {return _minDepth;}
	float getMaxDepth() const {return _maxDepth;}

public:
	virtual ~Feature2D();

	std::vector<cv::KeyPoint> generateKeypoints(
			const cv::Mat & image,
			const cv::Mat & mask = cv::Mat()) const;
	cv::Mat generateDescriptors(
			const cv::Mat & image,
			std::vector<cv::KeyPoint> & keypoints) const;
	std::vector<cv::Point3f> generateKeypoints3D(
			const SensorData & data,
			const std::vector<cv::KeyPoint> & keypoints) const;

	virtual void parseParameters(const ParametersMap & parameters);
	virtual const ParametersMap & getParameters() const {return parameters_;}
	virtual Feature2D::Type getType() const = 0;

protected:
	Feature2D(const ParametersMap & parameters = ParametersMap());

private:
	virtual std::vector<cv::KeyPoint> generateKeypointsImpl(const cv::Mat & image, const cv::Rect & roi, const cv::Mat & mask = cv::Mat()) const = 0;
	virtual cv::Mat generateDescriptorsImpl(const cv::Mat & image, std::vector<cv::KeyPoint> & keypoints) const = 0;

private:
	ParametersMap parameters_;
	int maxFeatures_;
	float _maxDepth; // 0=inf
	float _minDepth;
	std::vector<float> _roiRatios; // size 4
	int _subPixWinSize;
	int _subPixIterations;
	double _subPixEps;
	int gridRows_;
	int gridCols_;
	// Stereo stuff
	Stereo * _stereo;
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
	virtual std::vector<cv::KeyPoint> generateKeypointsImpl(const cv::Mat & image, const cv::Rect & roi, const cv::Mat & mask = cv::Mat()) const;
	virtual cv::Mat generateDescriptorsImpl(const cv::Mat & image, std::vector<cv::KeyPoint> & keypoints) const;

private:
	double hessianThreshold_;
	int nOctaves_;
	int nOctaveLayers_;
	bool extended_;
	bool upright_;
	float gpuKeypointsRatio_;
	bool gpuVersion_;

	cv::Ptr<CV_SURF> _surf;
	cv::Ptr<CV_SURF_GPU> _gpuSurf;
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
	virtual std::vector<cv::KeyPoint> generateKeypointsImpl(const cv::Mat & image, const cv::Rect & roi, const cv::Mat & mask = cv::Mat()) const;
	virtual cv::Mat generateDescriptorsImpl(const cv::Mat & image, std::vector<cv::KeyPoint> & keypoints) const;

private:
	int nOctaveLayers_;
	double contrastThreshold_;
	double edgeThreshold_;
	double sigma_;

	cv::Ptr<CV_SIFT> _sift;
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
	virtual std::vector<cv::KeyPoint> generateKeypointsImpl(const cv::Mat & image, const cv::Rect & roi, const cv::Mat & mask = cv::Mat()) const;
	virtual cv::Mat generateDescriptorsImpl(const cv::Mat & image, std::vector<cv::KeyPoint> & keypoints) const;

private:
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

	cv::Ptr<CV_ORB> _orb;
	cv::Ptr<CV_ORB_GPU> _gpuOrb;
};

//FAST
class RTABMAP_EXP FAST : public Feature2D
{
public:
	FAST(const ParametersMap & parameters = ParametersMap());
	virtual ~FAST();

	virtual void parseParameters(const ParametersMap & parameters);
	virtual Feature2D::Type getType() const {return kFeatureUndef;}

private:
	virtual std::vector<cv::KeyPoint> generateKeypointsImpl(const cv::Mat & image, const cv::Rect & roi, const cv::Mat & mask = cv::Mat()) const;
	virtual cv::Mat generateDescriptorsImpl(const cv::Mat & image, std::vector<cv::KeyPoint> & keypoints) const {return cv::Mat();}

private:
	int threshold_;
	bool nonmaxSuppression_;
	bool gpu_;
	double gpuKeypointsRatio_;
	int minThreshold_;
	int maxThreshold_;
	int gridRows_;
	int gridCols_;

	cv::Ptr<cv::FeatureDetector> _fast;
	cv::Ptr<CV_FAST_GPU> _gpuFast;
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

	cv::Ptr<CV_BRIEF> _brief;
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

	cv::Ptr<CV_FREAK> _freak;
};

//GFTT
class RTABMAP_EXP GFTT : public Feature2D
{
public:
	GFTT(const ParametersMap & parameters = ParametersMap());
	virtual ~GFTT();

	virtual void parseParameters(const ParametersMap & parameters);

private:
	virtual std::vector<cv::KeyPoint> generateKeypointsImpl(const cv::Mat & image, const cv::Rect & roi, const cv::Mat & mask = cv::Mat()) const;

private:
	double _qualityLevel;
	double _minDistance;
	int _blockSize;
	bool _useHarrisDetector;
	double _k;

	cv::Ptr<CV_GFTT> _gftt;
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

	cv::Ptr<CV_BRIEF> _brief;
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

	cv::Ptr<CV_FREAK> _freak;
};

//GFTT_ORB
class RTABMAP_EXP GFTT_ORB : public GFTT
{
public:
	GFTT_ORB(const ParametersMap & parameters = ParametersMap());
	virtual ~GFTT_ORB();

	virtual void parseParameters(const ParametersMap & parameters);
	virtual Feature2D::Type getType() const {return kFeatureGfttOrb;}

private:
	virtual cv::Mat generateDescriptorsImpl(const cv::Mat & image, std::vector<cv::KeyPoint> & keypoints) const;

private:
	ORB _orb;
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
	virtual std::vector<cv::KeyPoint> generateKeypointsImpl(const cv::Mat & image, const cv::Rect & roi, const cv::Mat & mask = cv::Mat()) const;
	virtual cv::Mat generateDescriptorsImpl(const cv::Mat & image, std::vector<cv::KeyPoint> & keypoints) const;

private:
	int thresh_;
	int octaves_;
	float patternScale_;

	cv::Ptr<CV_BRISK> brisk_;
};

//KAZE
class RTABMAP_EXP KAZE : public Feature2D
{
public:
	KAZE(const ParametersMap & parameters = ParametersMap());
	virtual ~KAZE();

	virtual void parseParameters(const ParametersMap & parameters);
	virtual Feature2D::Type getType() const { return kFeatureKaze; }

private:
	virtual std::vector<cv::KeyPoint> generateKeypointsImpl(const cv::Mat & image, const cv::Rect & roi, const cv::Mat & mask = cv::Mat()) const;
	virtual cv::Mat generateDescriptorsImpl(const cv::Mat & image, std::vector<cv::KeyPoint> & keypoints) const;

private:
	bool extended_;
	bool upright_;
	float threshold_;
	int nOctaves_;
	int nOctaveLayers_;
	int diffusivity_;

#if CV_MAJOR_VERSION > 2
	cv::Ptr<cv::KAZE> kaze_;
#endif
};


}

#endif /* FEATURES2D_H_ */
