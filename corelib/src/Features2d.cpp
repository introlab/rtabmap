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

#include "rtabmap/core/Features2d.h"
#include "rtabmap/core/util3d.h"
#include "rtabmap/core/util3d_features.h"
#include "rtabmap/core/Stereo.h"
#include "rtabmap/core/util2d.h"
#include "rtabmap/utilite/UStl.h"
#include "rtabmap/utilite/UConversion.h"
#include "rtabmap/utilite/ULogger.h"
#include "rtabmap/utilite/UMath.h"
#include "rtabmap/utilite/ULogger.h"
#include "rtabmap/utilite/UTimer.h"
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/core/version.hpp>
#include <opencv2/opencv_modules.hpp>

#ifdef RTABMAP_ORB_OCTREE
#include "opencv/ORBextractor.h"
#endif

#ifdef RTABMAP_TORCH
#include "superpoint_torch/SuperPoint.h"
#endif

#ifdef RTABMAP_PYTHON
#include "python/PyDetector.h"
#endif

#if CV_MAJOR_VERSION < 3
#include "opencv/Orb.h"
#ifdef HAVE_OPENCV_GPU
#include <opencv2/gpu/gpu.hpp>
#endif
#else
#include <opencv2/core/cuda.hpp>
#endif

#ifdef HAVE_OPENCV_NONFREE
  #if CV_MAJOR_VERSION == 2 && CV_MINOR_VERSION >=4
  #include <opencv2/nonfree/gpu.hpp>
  #include <opencv2/nonfree/features2d.hpp>
  #endif
#endif
#ifdef HAVE_OPENCV_XFEATURES2D
  #include <opencv2/xfeatures2d.hpp>
  #include <opencv2/xfeatures2d/nonfree.hpp>
  #include <opencv2/xfeatures2d/cuda.hpp>
#endif
#ifdef HAVE_OPENCV_CUDAFEATURES2D
  #include <opencv2/cudafeatures2d.hpp>
#endif

#ifdef RTABMAP_FASTCV
#include <fastcv.h>
#endif

namespace rtabmap {

void Feature2D::filterKeypointsByDepth(
		std::vector<cv::KeyPoint> & keypoints,
		const cv::Mat & depth,
		float minDepth,
		float maxDepth)
{
	cv::Mat descriptors;
	filterKeypointsByDepth(keypoints, descriptors, depth, minDepth, maxDepth);
}

void Feature2D::filterKeypointsByDepth(
		std::vector<cv::KeyPoint> & keypoints,
		cv::Mat & descriptors,
		const cv::Mat & depth,
		float minDepth,
		float maxDepth)
{
	UASSERT(minDepth >= 0.0f);
	UASSERT(maxDepth <= 0.0f || maxDepth > minDepth);
	if(!depth.empty() && (descriptors.empty() || descriptors.rows == (int)keypoints.size()))
	{
		std::vector<cv::KeyPoint> output(keypoints.size());
		std::vector<int> indexes(keypoints.size(), 0);
		int oi=0;
		bool isInMM = depth.type() == CV_16UC1;
		for(unsigned int i=0; i<keypoints.size(); ++i)
		{
			int u = int(keypoints[i].pt.x+0.5f);
			int v = int(keypoints[i].pt.y+0.5f);
			if(u >=0 && u<depth.cols && v >=0 && v<depth.rows)
			{
				float d = isInMM?(float)depth.at<uint16_t>(v,u)*0.001f:depth.at<float>(v,u);
				if(uIsFinite(d) && d>minDepth && (maxDepth <= 0.0f || d < maxDepth))
				{
					output[oi++] = keypoints[i];
					indexes[i] = 1;
				}
			}
		}
		output.resize(oi);
		keypoints = output;

		if(!descriptors.empty() && (int)keypoints.size() != descriptors.rows)
		{
			if(keypoints.size() == 0)
			{
				descriptors = cv::Mat();
			}
			else
			{
				cv::Mat newDescriptors((int)keypoints.size(), descriptors.cols, descriptors.type());
				int di = 0;
				for(unsigned int i=0; i<indexes.size(); ++i)
				{
					if(indexes[i] == 1)
					{
						if(descriptors.type() == CV_32FC1)
						{
							memcpy(newDescriptors.ptr<float>(di++), descriptors.ptr<float>(i), descriptors.cols*sizeof(float));
						}
						else // CV_8UC1
						{
							memcpy(newDescriptors.ptr<char>(di++), descriptors.ptr<char>(i), descriptors.cols*sizeof(char));
						}
					}
				}
				descriptors = newDescriptors;
			}
		}
	}
}

void Feature2D::filterKeypointsByDepth(
		std::vector<cv::KeyPoint> & keypoints,
		cv::Mat & descriptors,
		std::vector<cv::Point3f> & keypoints3D,
		float minDepth,
		float maxDepth)
{
	UDEBUG("");
	//remove all keypoints/descriptors with no valid 3D points
	UASSERT(((int)keypoints.size() == descriptors.rows || descriptors.empty()) &&
			keypoints3D.size() == keypoints.size());
	std::vector<cv::KeyPoint> validKeypoints(keypoints.size());
	std::vector<cv::Point3f> validKeypoints3D(keypoints.size());
	cv::Mat validDescriptors(descriptors.size(), descriptors.type());

	int oi=0;
	float minDepthSqr = minDepth * minDepth;
	float maxDepthSqr = maxDepth * maxDepth;
	for(unsigned int i=0; i<keypoints3D.size(); ++i)
	{
		cv::Point3f & pt = keypoints3D[i];
		if(util3d::isFinite(pt))
		{
			float distSqr = pt.x*pt.x+pt.y*pt.y+pt.z*pt.z;
			if(distSqr >= minDepthSqr && (maxDepthSqr==0.0f || distSqr <= maxDepthSqr))
			{
				validKeypoints[oi] = keypoints[i];
				validKeypoints3D[oi] = pt;
				if(!descriptors.empty())
				{
					descriptors.row(i).copyTo(validDescriptors.row(oi));
				}
				++oi;
			}
		}
	}
	UDEBUG("Removed %d invalid 3D points", (int)keypoints3D.size()-oi);
	validKeypoints.resize(oi);
	validKeypoints3D.resize(oi);
	keypoints = validKeypoints;
	keypoints3D = validKeypoints3D;
	if(!descriptors.empty())
	{
		descriptors = validDescriptors.rowRange(0, oi).clone();
	}
}

void Feature2D::filterKeypointsByDisparity(
		std::vector<cv::KeyPoint> & keypoints,
		const cv::Mat & disparity,
		float minDisparity)
{
	cv::Mat descriptors;
	filterKeypointsByDisparity(keypoints, descriptors, disparity, minDisparity);
}

void Feature2D::filterKeypointsByDisparity(
		std::vector<cv::KeyPoint> & keypoints,
		cv::Mat & descriptors,
		const cv::Mat & disparity,
		float minDisparity)
{
	if(!disparity.empty() && minDisparity > 0.0f && (descriptors.empty() || descriptors.rows == (int)keypoints.size()))
	{
		std::vector<cv::KeyPoint> output(keypoints.size());
		std::vector<int> indexes(keypoints.size(), 0);
		int oi=0;
		for(unsigned int i=0; i<keypoints.size(); ++i)
		{
			int u = int(keypoints[i].pt.x+0.5f);
			int v = int(keypoints[i].pt.y+0.5f);
			if(u >=0 && u<disparity.cols && v >=0 && v<disparity.rows)
			{
				float d = disparity.type() == CV_16SC1?float(disparity.at<short>(v,u))/16.0f:disparity.at<float>(v,u);
				if(d!=0.0f && uIsFinite(d) && d >= minDisparity)
				{
					output[oi++] = keypoints[i];
					indexes[i] = 1;
				}
			}
		}
		output.resize(oi);
		keypoints = output;

		if(!descriptors.empty() && (int)keypoints.size() != descriptors.rows)
		{
			if(keypoints.size() == 0)
			{
				descriptors = cv::Mat();
			}
			else
			{
				cv::Mat newDescriptors((int)keypoints.size(), descriptors.cols, descriptors.type());
				int di = 0;
				for(unsigned int i=0; i<indexes.size(); ++i)
				{
					if(indexes[i] == 1)
					{
						if(descriptors.type() == CV_32FC1)
						{
							memcpy(newDescriptors.ptr<float>(di++), descriptors.ptr<float>(i), descriptors.cols*sizeof(float));
						}
						else // CV_8UC1
						{
							memcpy(newDescriptors.ptr<char>(di++), descriptors.ptr<char>(i), descriptors.cols*sizeof(char));
						}
					}
				}
				descriptors = newDescriptors;
			}
		}
	}
}

void Feature2D::limitKeypoints(std::vector<cv::KeyPoint> & keypoints, int maxKeypoints)
{
	cv::Mat descriptors;
	limitKeypoints(keypoints, descriptors, maxKeypoints);
}

void Feature2D::limitKeypoints(std::vector<cv::KeyPoint> & keypoints, cv::Mat & descriptors, int maxKeypoints)
{
	std::vector<cv::Point3f> keypoints3D;
	limitKeypoints(keypoints, keypoints3D, descriptors, maxKeypoints);
}

void Feature2D::limitKeypoints(std::vector<cv::KeyPoint> & keypoints, std::vector<cv::Point3f> & keypoints3D, cv::Mat & descriptors, int maxKeypoints)
{
	UASSERT_MSG((int)keypoints.size() == descriptors.rows || descriptors.rows == 0, uFormat("keypoints=%d descriptors=%d", (int)keypoints.size(), descriptors.rows).c_str());
	UASSERT_MSG(keypoints.size() == keypoints3D.size() || keypoints3D.size() == 0, uFormat("keypoints=%d keypoints3D=%d", (int)keypoints.size(), (int)keypoints3D.size()).c_str());
	if(maxKeypoints > 0 && (int)keypoints.size() > maxKeypoints)
	{
		UTimer timer;
		ULOGGER_DEBUG("too much words (%d), removing words with the hessian threshold", keypoints.size());
		// Remove words under the new hessian threshold

		// Sort words by hessian
		std::multimap<float, int> hessianMap; // <hessian,id>
		for(unsigned int i = 0; i <keypoints.size(); ++i)
		{
			//Keep track of the data, to be easier to manage the data in the next step
			hessianMap.insert(std::pair<float, int>(fabs(keypoints[i].response), i));
		}

		// Remove them from the signature
		int removed = (int)hessianMap.size()-maxKeypoints;
		std::multimap<float, int>::reverse_iterator iter = hessianMap.rbegin();
		std::vector<cv::KeyPoint> kptsTmp(maxKeypoints);
		std::vector<cv::Point3f> kpts3DTmp;
		if(!keypoints3D.empty())
		{
			kpts3DTmp.resize(maxKeypoints);
		}
		cv::Mat descriptorsTmp;
		if(descriptors.rows)
		{
			descriptorsTmp = cv::Mat(maxKeypoints, descriptors.cols, descriptors.type());
		}
		for(unsigned int k=0; k < kptsTmp.size() && iter!=hessianMap.rend(); ++k, ++iter)
		{
			kptsTmp[k] = keypoints[iter->second];
			if(keypoints3D.size())
			{
				kpts3DTmp[k] = keypoints3D[iter->second];
			}
			if(descriptors.rows)
			{
				if(descriptors.type() == CV_32FC1)
				{
					memcpy(descriptorsTmp.ptr<float>(k), descriptors.ptr<float>(iter->second), descriptors.cols*sizeof(float));
				}
				else
				{
					memcpy(descriptorsTmp.ptr<char>(k), descriptors.ptr<char>(iter->second), descriptors.cols*sizeof(char));
				}
			}
		}
		ULOGGER_DEBUG("%d keypoints removed, (kept %d), minimum response=%f", removed, (int)kptsTmp.size(), kptsTmp.size()?kptsTmp.back().response:0.0f);
		ULOGGER_DEBUG("removing words time = %f s", timer.ticks());
		keypoints = kptsTmp;
		keypoints3D = kpts3DTmp;
		if(descriptors.rows)
		{
			descriptors = descriptorsTmp;
		}
	}
}

void Feature2D::limitKeypoints(const std::vector<cv::KeyPoint> & keypoints, std::vector<bool> & inliers, int maxKeypoints)
{
	if(maxKeypoints > 0 && (int)keypoints.size() > maxKeypoints)
	{
		UTimer timer;
		ULOGGER_DEBUG("too much words (%d), removing words with the hessian threshold", (int)keypoints.size());
		// Remove words under the new hessian threshold

		// Sort words by hessian
		std::multimap<float, int> hessianMap; // <hessian,id>
		for(unsigned int i = 0; i <keypoints.size(); ++i)
		{
			//Keep track of the data, to be easier to manage the data in the next step
			hessianMap.insert(std::pair<float, int>(fabs(keypoints[i].response), i));
		}

		// Keep keypoints with highest response
		int removed = (int)hessianMap.size()-maxKeypoints;
		std::multimap<float, int>::reverse_iterator iter = hessianMap.rbegin();
		inliers.resize(keypoints.size(), false);
		float minimumHessian = 0.0f;
		for(int k=0; k < maxKeypoints && iter!=hessianMap.rend(); ++k, ++iter)
		{
			inliers[iter->second] = true;
			minimumHessian = iter->first;
		}
		ULOGGER_DEBUG("%d keypoints removed, (kept %d), minimum response=%f", removed, maxKeypoints, minimumHessian);
		ULOGGER_DEBUG("filter keypoints time = %f s", timer.ticks());
	}
	else
	{
		ULOGGER_DEBUG("keeping all %d keypoints", (int)keypoints.size());
		inliers.resize(keypoints.size(), true);
	}
}

void Feature2D::limitKeypoints(const std::vector<cv::KeyPoint> & keypoints, std::vector<bool> & inliers, int maxKeypoints, const cv::Size & imageSize, int gridRows, int gridCols)
{
	if(maxKeypoints <= 0 || (int)keypoints.size() <= maxKeypoints)
	{
		inliers.resize(keypoints.size(), true);
		return;
	}
	UASSERT(gridCols>=1 && gridRows >=1);
	UASSERT(imageSize.height>gridRows && imageSize.width>gridCols);
	int rowSize = imageSize.height / gridRows;
	int colSize = imageSize.width / gridCols;
	int maxKeypointsPerCell = maxKeypoints / (gridRows * gridCols);
	std::vector<std::vector<cv::KeyPoint> > keypointsPerCell(gridRows * gridCols);
	std::vector<std::vector<int> > indexesPerCell(gridRows * gridCols);
	for(size_t i=0; i<keypoints.size(); ++i)
	{
		int cellRow = int(keypoints[i].pt.y)/rowSize;
		int cellCol = int(keypoints[i].pt.x)/colSize;
		UASSERT(cellRow >=0 && cellRow < gridRows);
		UASSERT(cellCol >=0 && cellCol < gridCols);

		keypointsPerCell[cellRow*gridCols + cellCol].push_back(keypoints[i]);
		indexesPerCell[cellRow*gridCols + cellCol].push_back(i);
	}
	inliers.resize(keypoints.size(), false);
	for(size_t i=0; i<keypointsPerCell.size(); ++i)
	{
		std::vector<bool> inliersCell;
		limitKeypoints(keypointsPerCell[i], inliersCell, maxKeypointsPerCell);
		for(size_t j=0; j<inliersCell.size(); ++j)
		{
			if(inliersCell[j])
			{
				inliers.at(indexesPerCell[i][j]) = true;
			}
		}
	}
}

cv::Rect Feature2D::computeRoi(const cv::Mat & image, const std::string & roiRatios)
{
	return util2d::computeRoi(image, roiRatios);
}

cv::Rect Feature2D::computeRoi(const cv::Mat & image, const std::vector<float> & roiRatios)
{
	return util2d::computeRoi(image, roiRatios);
}

/////////////////////
// Feature2D
/////////////////////
Feature2D::Feature2D(const ParametersMap & parameters) :
		maxFeatures_(Parameters::defaultKpMaxFeatures()),
		_maxDepth(Parameters::defaultKpMaxDepth()),
		_minDepth(Parameters::defaultKpMinDepth()),
		_roiRatios(std::vector<float>(4, 0.0f)),
		_subPixWinSize(Parameters::defaultKpSubPixWinSize()),
		_subPixIterations(Parameters::defaultKpSubPixIterations()),
		_subPixEps(Parameters::defaultKpSubPixEps()),
		gridRows_(Parameters::defaultKpGridRows()),
		gridCols_(Parameters::defaultKpGridCols())
{
	_stereo = new Stereo(parameters);
	this->parseParameters(parameters);
}
Feature2D::~Feature2D()
{
	delete _stereo;
}
void Feature2D::parseParameters(const ParametersMap & parameters)
{
	uInsert(parameters_, parameters);

	Parameters::parse(parameters, Parameters::kKpMaxFeatures(), maxFeatures_);
	Parameters::parse(parameters, Parameters::kKpMaxDepth(), _maxDepth);
	Parameters::parse(parameters, Parameters::kKpMinDepth(), _minDepth);
	Parameters::parse(parameters, Parameters::kKpSubPixWinSize(), _subPixWinSize);
	Parameters::parse(parameters, Parameters::kKpSubPixIterations(), _subPixIterations);
	Parameters::parse(parameters, Parameters::kKpSubPixEps(), _subPixEps);
	Parameters::parse(parameters, Parameters::kKpGridRows(), gridRows_);
	Parameters::parse(parameters, Parameters::kKpGridCols(), gridCols_);

	UASSERT(gridRows_ >= 1 && gridCols_>=1);

	// convert ROI from string to vector
	ParametersMap::const_iterator iter;
	if((iter=parameters.find(Parameters::kKpRoiRatios())) != parameters.end())
	{
		std::list<std::string> strValues = uSplit(iter->second, ' ');
		if(strValues.size() != 4)
		{
			ULOGGER_ERROR("The number of values must be 4 (roi=\"%s\")", iter->second.c_str());
		}
		else
		{
			std::vector<float> tmpValues(4);
			unsigned int i=0;
			for(std::list<std::string>::iterator jter = strValues.begin(); jter!=strValues.end(); ++jter)
			{
				tmpValues[i] = uStr2Float(*jter);
				++i;
			}

			if(tmpValues[0] >= 0 && tmpValues[0] < 1 && tmpValues[0] < 1.0f-tmpValues[1] &&
				tmpValues[1] >= 0 && tmpValues[1] < 1 && tmpValues[1] < 1.0f-tmpValues[0] &&
				tmpValues[2] >= 0 && tmpValues[2] < 1 && tmpValues[2] < 1.0f-tmpValues[3] &&
				tmpValues[3] >= 0 && tmpValues[3] < 1 && tmpValues[3] < 1.0f-tmpValues[2])
			{
				_roiRatios = tmpValues;
			}
			else
			{
				ULOGGER_ERROR("The roi ratios are not valid (roi=\"%s\")", iter->second.c_str());
			}
		}
	}

	//stereo
	UASSERT(_stereo != 0);
	if((iter=parameters.find(Parameters::kStereoOpticalFlow())) != parameters.end())
	{
		delete _stereo;
		_stereo = Stereo::create(parameters_);
	}
	else
	{
		_stereo->parseParameters(parameters);
	}
}
Feature2D * Feature2D::create(const ParametersMap & parameters)
{
	int type = Parameters::defaultKpDetectorStrategy();
	Parameters::parse(parameters, Parameters::kKpDetectorStrategy(), type);
	return create((Feature2D::Type)type, parameters);
}
Feature2D * Feature2D::create(Feature2D::Type type, const ParametersMap & parameters)
{

// NONFREE checks
#if CV_MAJOR_VERSION < 3 || (CV_MAJOR_VERSION == 4 && CV_MINOR_VERSION <= 3) || (CV_MAJOR_VERSION == 3 && (CV_MINOR_VERSION < 4 || (CV_MINOR_VERSION==4 && CV_SUBMINOR_VERSION<11)))

  #ifndef RTABMAP_NONFREE
	if(type == Feature2D::kFeatureSurf || type == Feature2D::kFeatureSift || type == Feature2D::kFeatureSurfFreak || type == Feature2D::kFeatureSurfDaisy)
	{
    #if CV_MAJOR_VERSION < 3
		UWARN("SURF and SIFT features cannot be used because OpenCV was not built with nonfree module. GFTT/ORB is used instead.");
    #else
		UWARN("SURF and SIFT features cannot be used because OpenCV was not built with xfeatures2d module. GFTT/ORB is used instead.");
    #endif
		type = Feature2D::kFeatureGfttOrb;
	}
  #endif

#else // >= 4.4.0 >= 3.4.11

  #ifndef RTABMAP_NONFREE
	if(type == Feature2D::kFeatureSurf)
	{
		UWARN("SURF features cannot be used because OpenCV was not built with nonfree module. SIFT is used instead.");
		type = Feature2D::kFeatureSift;
	}
	else if(type == Feature2D::kFeatureSurfFreak || type == Feature2D::kFeatureSurfDaisy)
	{
		UWARN("SURF detector cannot be used because OpenCV was not built with nonfree module. GFTT/ORB is used instead.");
		type = Feature2D::kFeatureGfttOrb;
	}
  #endif

#endif // >= 4.4.0 >= 3.4.11

#if !defined(HAVE_OPENCV_XFEATURES2D) && CV_MAJOR_VERSION >= 3
	if(type == Feature2D::kFeatureFastBrief ||
	   type == Feature2D::kFeatureFastFreak ||
	   type == Feature2D::kFeatureGfttBrief ||
	   type == Feature2D::kFeatureGfttFreak ||
	   type == Feature2D::kFeatureSurfFreak ||
	   type == Feature2D::kFeatureGfttDaisy ||
	   type == Feature2D::kFeatureSurfDaisy)
	{
		UWARN("BRIEF, FREAK and DAISY features cannot be used because OpenCV was not built with xfeatures2d module. GFTT/ORB is used instead.");
		type = Feature2D::kFeatureGfttOrb;
	}
#elif CV_MAJOR_VERSION < 3
	if(type == Feature2D::kFeatureKaze)
	{
  #ifdef RTABMAP_NONFREE
		UWARN("KAZE detector/descriptor can be used only with OpenCV3. SURF is used instead.");
		type = Feature2D::kFeatureSurf;
  #else
		UWARN("KAZE detector/descriptor can be used only with OpenCV3. GFTT/ORB is used instead.");
		type = Feature2D::kFeatureGfttOrb;
  #endif
	}
	if(type == Feature2D::kFeatureGfttDaisy || type == Feature2D::kFeatureSurfDaisy)
	{
		UWARN("DAISY detector/descriptor can be used only with OpenCV3. GFTT/BRIEF is used instead.");
		type = Feature2D::kFeatureGfttBrief;
	}
#endif


#ifndef RTABMAP_ORB_OCTREE
	if(type == Feature2D::kFeatureOrbOctree)
	{
		UWARN("ORB OcTree feature cannot be used as RTAB-Map is not built with the option enabled. GFTT/ORB is used instead.");
		type = Feature2D::kFeatureGfttOrb;
	}
#endif

#ifndef RTABMAP_TORCH
	if(type == Feature2D::kFeatureSuperPointTorch)
	{
		UWARN("SupertPoint Torch feature cannot be used as RTAB-Map is not built with the option enabled. GFTT/ORB is used instead.");
		type = Feature2D::kFeatureGfttOrb;
	}
#endif

	Feature2D * feature2D = 0;
	switch(type)
	{
	case Feature2D::kFeatureSurf:
		feature2D = new SURF(parameters);
		break;
	case Feature2D::kFeatureSift:
		feature2D = new SIFT(parameters);
		break;
	case Feature2D::kFeatureOrb:
		feature2D = new ORB(parameters);
		break;
	case Feature2D::kFeatureFastBrief:
		feature2D = new FAST_BRIEF(parameters);
		break;
	case Feature2D::kFeatureFastFreak:
		feature2D = new FAST_FREAK(parameters);
		break;
	case Feature2D::kFeatureGfttFreak:
		feature2D = new GFTT_FREAK(parameters);
		break;
	case Feature2D::kFeatureGfttBrief:
		feature2D = new GFTT_BRIEF(parameters);
		break;
	case Feature2D::kFeatureGfttOrb:
		feature2D = new GFTT_ORB(parameters);
		break;
	case Feature2D::kFeatureBrisk:
		feature2D = new BRISK(parameters);
		break;
	case Feature2D::kFeatureKaze:
		feature2D = new KAZE(parameters);
		break;
	case Feature2D::kFeatureOrbOctree:
		feature2D = new ORBOctree(parameters);
		break;
#ifdef RTABMAP_TORCH
	case Feature2D::kFeatureSuperPointTorch:
		feature2D = new SuperPointTorch(parameters);
		break;
#endif
	case Feature2D::kFeatureSurfFreak:
		feature2D = new SURF_FREAK(parameters);
		break;
	case Feature2D::kFeatureGfttDaisy:
		feature2D = new GFTT_DAISY(parameters);
		break;
	case Feature2D::kFeatureSurfDaisy:
		feature2D = new SURF_DAISY(parameters);
		break;
#ifdef RTABMAP_PYTHON
	case Feature2D::kFeaturePyDetector:
		feature2D = new PyDetector(parameters);
		break;
#endif
#ifdef RTABMAP_NONFREE
	default:
		feature2D = new SURF(parameters);
		type = Feature2D::kFeatureSurf;
		break;
#else
	default:
		feature2D = new ORB(parameters);
		type = Feature2D::kFeatureGfttOrb;
		break;
#endif

	}
	return feature2D;
}

std::vector<cv::KeyPoint> Feature2D::generateKeypoints(const cv::Mat & image, const cv::Mat & maskIn)
{
	UASSERT(!image.empty());
	UASSERT(image.type() == CV_8UC1);

	cv::Mat mask;
	if(!maskIn.empty())
	{
		if(maskIn.type()==CV_16UC1 || maskIn.type() == CV_32FC1)
		{
			mask = cv::Mat::zeros(maskIn.rows, maskIn.cols, CV_8UC1);
			for(int i=0; i<(int)mask.total(); ++i)
			{
				float value = 0.0f;
				if(maskIn.type()==CV_16UC1)
				{
					if(((unsigned short*)maskIn.data)[i] > 0 &&
					   ((unsigned short*)maskIn.data)[i] < std::numeric_limits<unsigned short>::max())
					{
						value = float(((unsigned short*)maskIn.data)[i])*0.001f;
					}
				}
				else
				{
					value = ((float*)maskIn.data)[i];
				}

				if(value>_minDepth &&
				   (_maxDepth == 0.0f || value <= _maxDepth) &&
				   uIsFinite(value))
				{
					((unsigned char*)mask.data)[i] = 255; // ORB uses 255 to handle pyramids
				}
			}
		}
		else if(maskIn.type()==CV_8UC1)
		{
			// assume a standard mask
			mask = maskIn;
		}
		else
		{
			UERROR("Wrong mask type (%d)! Should be 8UC1, 16UC1 or 32FC1.", maskIn.type());
		}
	}

	UASSERT(mask.empty() || (mask.cols == image.cols && mask.rows == image.rows));

	std::vector<cv::KeyPoint> keypoints;
	UTimer timer;
	cv::Rect globalRoi = Feature2D::computeRoi(image, _roiRatios);
	if(!(globalRoi.width && globalRoi.height))
	{
		globalRoi = cv::Rect(0,0,image.cols, image.rows);
	}

	// Get keypoints
	int rowSize = globalRoi.height / gridRows_;
	int colSize = globalRoi.width / gridCols_;
	int maxFeatures =	maxFeatures_ / (gridRows_ * gridCols_);
	for (int i = 0; i<gridRows_; ++i)
	{
		for (int j = 0; j<gridCols_; ++j)
		{
			cv::Rect roi(globalRoi.x + j*colSize, globalRoi.y + i*rowSize, colSize, rowSize);
			std::vector<cv::KeyPoint> sub_keypoints;
			sub_keypoints = this->generateKeypointsImpl(image, roi, mask);
			limitKeypoints(sub_keypoints, maxFeatures);
			if(roi.x || roi.y)
			{
				// Adjust keypoint position to raw image
				for(std::vector<cv::KeyPoint>::iterator iter=sub_keypoints.begin(); iter!=sub_keypoints.end(); ++iter)
				{
					iter->pt.x += roi.x;
					iter->pt.y += roi.y;
				}
			}
			keypoints.insert( keypoints.end(), sub_keypoints.begin(), sub_keypoints.end() );
		}
	}
	UDEBUG("Keypoints extraction time = %f s, keypoints extracted = %d (grid=%dx%d, mask empty=%d)",
			timer.ticks(), keypoints.size(), gridCols_, gridRows_,  mask.empty()?1:0);

	if(keypoints.size() && _subPixWinSize > 0 && _subPixIterations > 0)
	{
		std::vector<cv::Point2f> corners;
		cv::KeyPoint::convert(keypoints, corners);
		cv::cornerSubPix( image, corners,
				cv::Size( _subPixWinSize, _subPixWinSize ),
				cv::Size( -1, -1 ),
				cv::TermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, _subPixIterations, _subPixEps ) );

		for(unsigned int i=0;i<corners.size(); ++i)
		{
			keypoints[i].pt = corners[i];
		}
		UDEBUG("subpixel time = %f s", timer.ticks());
	}

	return keypoints;
}

cv::Mat Feature2D::generateDescriptors(
		const cv::Mat & image,
		std::vector<cv::KeyPoint> & keypoints) const
{
	cv::Mat descriptors;
	if(keypoints.size())
	{
		UASSERT(!image.empty());
		UASSERT(image.type() == CV_8UC1);
		descriptors = generateDescriptorsImpl(image, keypoints);
		UASSERT_MSG(descriptors.rows == (int)keypoints.size(), uFormat("descriptors=%d, keypoints=%d", descriptors.rows, (int)keypoints.size()).c_str());
		UDEBUG("Descriptors extracted = %d, remaining kpts=%d", descriptors.rows, (int)keypoints.size());
	}
	return descriptors;
}

std::vector<cv::Point3f> Feature2D::generateKeypoints3D(
		const SensorData & data,
		const std::vector<cv::KeyPoint> & keypoints) const
{
	std::vector<cv::Point3f> keypoints3D;
	if(keypoints.size())
	{
		if(!data.rightRaw().empty() && !data.imageRaw().empty() &&
			!data.stereoCameraModels().empty() &&
			data.stereoCameraModels()[0].isValidForProjection())
		{
			//stereo
			cv::Mat imageMono;
			// convert to grayscale
			if(data.imageRaw().channels() > 1)
			{
				cv::cvtColor(data.imageRaw(), imageMono, cv::COLOR_BGR2GRAY);
			}
			else
			{
				imageMono = data.imageRaw();
			}

			std::vector<cv::Point2f> leftCorners;
			cv::KeyPoint::convert(keypoints, leftCorners);

			std::vector<cv::Point2f> rightCorners;

			if(data.stereoCameraModels().size() == 1)
			{
				std::vector<unsigned char> status;
				rightCorners = _stereo->computeCorrespondences(
						imageMono,
						data.rightRaw(),
						leftCorners,
						status);

				if(ULogger::level() >= ULogger::kWarning)
				{
					int rejected = 0;
					for(size_t i=0; i<status.size(); ++i)
					{
						if(status[i]==0)
						{
							++rejected;
						}
					}
					if(rejected > (int)status.size()/2)
					{
						UWARN("A large number (%d/%d) of stereo correspondences are rejected! "
								"Optical flow may have failed because images are not calibrated, "
								"the background is too far (no disparity between the images), "
								"maximum disparity may be too small (%f) or that exposure between "
								"left and right images is too different.",
								rejected,
								(int)status.size(),
								_stereo->maxDisparity());
					}
				}

				keypoints3D = util3d::generateKeypoints3DStereo(
						leftCorners,
						rightCorners,
						data.stereoCameraModels()[0],
						status,
						_minDepth,
						_maxDepth);
			}
			else
			{
				int subImageWith = imageMono.cols / data.stereoCameraModels().size();
				UASSERT(imageMono.cols % subImageWith == 0);
				std::vector<std::vector<cv::Point2f> > subLeftCorners(data.stereoCameraModels().size());
				std::vector<std::vector<int> > subIndex(data.stereoCameraModels().size());
				// Assign keypoints per camera
				for(size_t i=0; i<leftCorners.size(); ++i)
				{
					int cameraIndex = int(leftCorners[i].x / subImageWith);
					leftCorners[i].x -= cameraIndex*subImageWith;
					subLeftCorners[cameraIndex].push_back(leftCorners[i]);
					subIndex[cameraIndex].push_back(i);
				}

				keypoints3D.resize(keypoints.size());
				int total = 0;
				int rejected = 0;
				for(size_t i=0; i<data.stereoCameraModels().size(); ++i)
				{
					if(!subLeftCorners[i].empty())
					{
						std::vector<unsigned char> status;
						rightCorners = _stereo->computeCorrespondences(
								imageMono.colRange(cv::Range(subImageWith*i, subImageWith*(i+1))),
								data.rightRaw().colRange(cv::Range(subImageWith*i, subImageWith*(i+1))),
								subLeftCorners[i],
								status);

						std::vector<cv::Point3f> subKeypoints3D = util3d::generateKeypoints3DStereo(
								subLeftCorners[i],
								rightCorners,
								data.stereoCameraModels()[i],
								status,
								_minDepth,
								_maxDepth);

						if(ULogger::level() >= ULogger::kWarning)
						{
							for(size_t i=0; i<status.size(); ++i)
							{
								if(status[i]==0)
								{
									++rejected;
								}
							}
							total+=status.size();
						}

						UASSERT(subIndex[i].size() == subKeypoints3D.size());
						for(size_t j=0; j<subKeypoints3D.size(); ++j)
						{
							keypoints3D[subIndex[i][j]] = subKeypoints3D[j];
						}
					}
				}

				if(ULogger::level() >= ULogger::kWarning)
				{
					if(rejected > total/2)
					{
						UWARN("A large number (%d/%d) of stereo correspondences are rejected! "
								"Optical flow may have failed because images are not calibrated, "
								"the background is too far (no disparity between the images), "
								"maximum disparity may be too small (%f) or that exposure between "
								"left and right images is too different.",
								rejected,
								total,
								_stereo->maxDisparity());
					}
				}
			}
		}
		else if(!data.depthRaw().empty() && data.cameraModels().size())
		{
			keypoints3D = util3d::generateKeypoints3DDepth(
					keypoints,
					data.depthOrRightRaw(),
					data.cameraModels(),
					_minDepth,
					_maxDepth);
		}
	}

	return keypoints3D;
}

//////////////////////////
//SURF
//////////////////////////
SURF::SURF(const ParametersMap & parameters) :
		hessianThreshold_(Parameters::defaultSURFHessianThreshold()),
		nOctaves_(Parameters::defaultSURFOctaves()),
		nOctaveLayers_(Parameters::defaultSURFOctaveLayers()),
		extended_(Parameters::defaultSURFExtended()),
		upright_(Parameters::defaultSURFUpright()),
		gpuKeypointsRatio_(Parameters::defaultSURFGpuKeypointsRatio()),
		gpuVersion_(Parameters::defaultSURFGpuVersion())
{
	parseParameters(parameters);
}

SURF::~SURF()
{
}

void SURF::parseParameters(const ParametersMap & parameters)
{
	Feature2D::parseParameters(parameters);

	Parameters::parse(parameters, Parameters::kSURFExtended(), extended_);
	Parameters::parse(parameters, Parameters::kSURFHessianThreshold(), hessianThreshold_);
	Parameters::parse(parameters, Parameters::kSURFOctaveLayers(), nOctaveLayers_);
	Parameters::parse(parameters, Parameters::kSURFOctaves(), nOctaves_);
	Parameters::parse(parameters, Parameters::kSURFUpright(), upright_);
	Parameters::parse(parameters, Parameters::kSURFGpuKeypointsRatio(), gpuKeypointsRatio_);
	Parameters::parse(parameters, Parameters::kSURFGpuVersion(), gpuVersion_);

#ifdef RTABMAP_NONFREE
#if CV_MAJOR_VERSION < 3
	if(gpuVersion_ && cv::gpu::getCudaEnabledDeviceCount() == 0)
	{
		UWARN("GPU version of SURF not available! Using CPU version instead...");
		gpuVersion_ = false;
	}
#else
	if(gpuVersion_ && cv::cuda::getCudaEnabledDeviceCount() == 0)
	{
		UWARN("GPU version of SURF not available! Using CPU version instead...");
		gpuVersion_ = false;
	}
#endif
	if(gpuVersion_)
	{
		_gpuSurf = cv::Ptr<CV_SURF_GPU>(new CV_SURF_GPU(hessianThreshold_, nOctaves_, nOctaveLayers_, extended_, gpuKeypointsRatio_, upright_));
	}
	else
	{
#if CV_MAJOR_VERSION < 3
		_surf = cv::Ptr<CV_SURF>(new CV_SURF(hessianThreshold_, nOctaves_, nOctaveLayers_, extended_, upright_));
#else
		_surf = CV_SURF::create(hessianThreshold_, nOctaves_, nOctaveLayers_, extended_, upright_);
#endif
	}
#else
	UWARN("RTAB-Map is not built with OpenCV nonfree module so SURF cannot be used!");
#endif
}

std::vector<cv::KeyPoint> SURF::generateKeypointsImpl(const cv::Mat & image, const cv::Rect & roi, const cv::Mat & mask)
{
	UASSERT(!image.empty() && image.channels() == 1 && image.depth() == CV_8U);
	std::vector<cv::KeyPoint> keypoints;

#ifdef RTABMAP_NONFREE
	cv::Mat imgRoi(image, roi);
	cv::Mat maskRoi;
	if(!mask.empty())
	{
		maskRoi = cv::Mat(mask, roi);
	}
	if(gpuVersion_)
	{
#if CV_MAJOR_VERSION < 3
		cv::gpu::GpuMat imgGpu(imgRoi);
		cv::gpu::GpuMat maskGpu(maskRoi);
		(*_gpuSurf.obj)(imgGpu, maskGpu, keypoints);
#else
		cv::cuda::GpuMat imgGpu(imgRoi);
		cv::cuda::GpuMat maskGpu(maskRoi);
		(*_gpuSurf.get())(imgGpu, maskGpu, keypoints);
#endif
	}
	else
	{
		_surf->detect(imgRoi, keypoints, maskRoi);
	}
#else
	UWARN("RTAB-Map is not built with OpenCV nonfree module so SURF cannot be used!");
#endif
	return keypoints;
}

cv::Mat SURF::generateDescriptorsImpl(const cv::Mat & image, std::vector<cv::KeyPoint> & keypoints) const
{
	UASSERT(!image.empty() && image.channels() == 1 && image.depth() == CV_8U);
	cv::Mat descriptors;
#ifdef RTABMAP_NONFREE
	if(gpuVersion_)
	{
#if CV_MAJOR_VERSION < 3
		cv::gpu::GpuMat imgGpu(image);
		cv::gpu::GpuMat descriptorsGPU;
		(*_gpuSurf.obj)(imgGpu, cv::gpu::GpuMat(), keypoints, descriptorsGPU, true);
#else
		cv::cuda::GpuMat imgGpu(image);
		cv::cuda::GpuMat descriptorsGPU;
		(*_gpuSurf.get())(imgGpu, cv::cuda::GpuMat(), keypoints, descriptorsGPU, true);
#endif

		// Download descriptors
		if (descriptorsGPU.empty())
			descriptors = cv::Mat();
		else
		{
			UASSERT(descriptorsGPU.type() == CV_32F);
			descriptors = cv::Mat(descriptorsGPU.size(), CV_32F);
			descriptorsGPU.download(descriptors);
		}
	}
	else
	{
		_surf->compute(image, keypoints, descriptors);
	}
#else
	UWARN("RTAB-Map is not built with OpenCV nonfree module so SURF cannot be used!");
#endif

	return descriptors;
}

//////////////////////////
//SIFT
//////////////////////////
SIFT::SIFT(const ParametersMap & parameters) :
	nOctaveLayers_(Parameters::defaultSIFTNOctaveLayers()),
	contrastThreshold_(Parameters::defaultSIFTContrastThreshold()),
	edgeThreshold_(Parameters::defaultSIFTEdgeThreshold()),
	sigma_(Parameters::defaultSIFTSigma()),
	rootSIFT_(Parameters::defaultSIFTRootSIFT())
{
	parseParameters(parameters);
}

SIFT::~SIFT()
{
}

void SIFT::parseParameters(const ParametersMap & parameters)
{
	Feature2D::parseParameters(parameters);

	Parameters::parse(parameters, Parameters::kSIFTContrastThreshold(), contrastThreshold_);
	Parameters::parse(parameters, Parameters::kSIFTEdgeThreshold(), edgeThreshold_);
	Parameters::parse(parameters, Parameters::kSIFTNOctaveLayers(), nOctaveLayers_);
	Parameters::parse(parameters, Parameters::kSIFTSigma(), sigma_);
	Parameters::parse(parameters, Parameters::kSIFTRootSIFT(), rootSIFT_);

#if CV_MAJOR_VERSION < 3 || (CV_MAJOR_VERSION == 4 && CV_MINOR_VERSION <= 3) || (CV_MAJOR_VERSION == 3 && (CV_MINOR_VERSION < 4 || (CV_MINOR_VERSION==4 && CV_SUBMINOR_VERSION<11)))
#ifdef RTABMAP_NONFREE
#if CV_MAJOR_VERSION < 3
	_sift = cv::Ptr<CV_SIFT>(new CV_SIFT(this->getMaxFeatures(), nOctaveLayers_, contrastThreshold_, edgeThreshold_, sigma_));
#else
	_sift = CV_SIFT::create(this->getMaxFeatures(), nOctaveLayers_, contrastThreshold_, edgeThreshold_, sigma_);
#endif
#else
	UWARN("RTAB-Map is not built with OpenCV nonfree module so SIFT cannot be used!");
#endif
#else // >=4.4, >=3.4.11
	_sift = CV_SIFT::create(this->getMaxFeatures(), nOctaveLayers_, contrastThreshold_, edgeThreshold_, sigma_);
#endif
}

std::vector<cv::KeyPoint> SIFT::generateKeypointsImpl(const cv::Mat & image, const cv::Rect & roi, const cv::Mat & mask)
{
	UASSERT(!image.empty() && image.channels() == 1 && image.depth() == CV_8U);
	std::vector<cv::KeyPoint> keypoints;
	cv::Mat imgRoi(image, roi);
	cv::Mat maskRoi;
	if(!mask.empty())
	{
		maskRoi = cv::Mat(mask, roi);
	}
#if CV_MAJOR_VERSION < 3 || (CV_MAJOR_VERSION == 4 && CV_MINOR_VERSION <= 3) || (CV_MAJOR_VERSION == 3 && (CV_MINOR_VERSION < 4 || (CV_MINOR_VERSION==4 && CV_SUBMINOR_VERSION<11)))
#ifdef RTABMAP_NONFREE
	_sift->detect(imgRoi, keypoints, maskRoi); // Opencv keypoints
#else
	UWARN("RTAB-Map is not built with OpenCV nonfree module so SIFT cannot be used!");
#endif
#else // >=4.4, >=3.4.11
	_sift->detect(imgRoi, keypoints, maskRoi); // Opencv keypoints
#endif
	return keypoints;
}

cv::Mat SIFT::generateDescriptorsImpl(const cv::Mat & image, std::vector<cv::KeyPoint> & keypoints) const
{
	UASSERT(!image.empty() && image.channels() == 1 && image.depth() == CV_8U);
	cv::Mat descriptors;
#if CV_MAJOR_VERSION < 3 || (CV_MAJOR_VERSION == 4 && CV_MINOR_VERSION <= 3) || (CV_MAJOR_VERSION == 3 && (CV_MINOR_VERSION < 4 || (CV_MINOR_VERSION==4 && CV_SUBMINOR_VERSION<11)))
#ifdef RTABMAP_NONFREE
	_sift->compute(image, keypoints, descriptors);
#else
	UWARN("RTAB-Map is not built with OpenCV nonfree module so SIFT cannot be used!");
#endif
#else // >=4.4, >=3.4.11
	_sift->compute(image, keypoints, descriptors);
#endif
	if( rootSIFT_ && !descriptors.empty())
	{
		UDEBUG("Performing RootSIFT...");
		// see http://www.pyimagesearch.com/2015/04/13/implementing-rootsift-in-python-and-opencv/
		// apply the Hellinger kernel by first L1-normalizing and taking the
		// square-root
		for(int i=0; i<descriptors.rows; ++i)
		{
			// By taking the L1 norm, followed by the square-root, we have
			// already L2 normalized the feature vector and further normalization
			// is not needed.
			descriptors.row(i) = descriptors.row(i) / cv::sum(descriptors.row(i))[0];
			cv::sqrt(descriptors.row(i), descriptors.row(i));
		}
	}
	return descriptors;
}

//////////////////////////
//ORB
//////////////////////////
ORB::ORB(const ParametersMap & parameters) :
		scaleFactor_(Parameters::defaultORBScaleFactor()),
		nLevels_(Parameters::defaultORBNLevels()),
		edgeThreshold_(Parameters::defaultORBEdgeThreshold()),
		firstLevel_(Parameters::defaultORBFirstLevel()),
		WTA_K_(Parameters::defaultORBWTA_K()),
		scoreType_(Parameters::defaultORBScoreType()),
		patchSize_(Parameters::defaultORBPatchSize()),
		gpu_(Parameters::defaultORBGpu()),
		fastThreshold_(Parameters::defaultFASTThreshold()),
		nonmaxSuppresion_(Parameters::defaultFASTNonmaxSuppression())
{
	parseParameters(parameters);
}

ORB::~ORB()
{
}

void ORB::parseParameters(const ParametersMap & parameters)
{
	Feature2D::parseParameters(parameters);

	Parameters::parse(parameters, Parameters::kORBScaleFactor(), scaleFactor_);
	Parameters::parse(parameters, Parameters::kORBNLevels(), nLevels_);
	Parameters::parse(parameters, Parameters::kORBEdgeThreshold(), edgeThreshold_);
	Parameters::parse(parameters, Parameters::kORBFirstLevel(), firstLevel_);
	Parameters::parse(parameters, Parameters::kORBWTA_K(), WTA_K_);
	Parameters::parse(parameters, Parameters::kORBScoreType(), scoreType_);
	Parameters::parse(parameters, Parameters::kORBPatchSize(), patchSize_);
	Parameters::parse(parameters, Parameters::kORBGpu(), gpu_);

	Parameters::parse(parameters, Parameters::kFASTThreshold(), fastThreshold_);
	Parameters::parse(parameters, Parameters::kFASTNonmaxSuppression(), nonmaxSuppresion_);

#if CV_MAJOR_VERSION < 3
#ifdef HAVE_OPENCV_GPU
	if(gpu_ && cv::gpu::getCudaEnabledDeviceCount() == 0)
	{
		UWARN("GPU version of ORB not available! Using CPU version instead...");
		gpu_ = false;
	}
#else
	if(gpu_)
	{
		UWARN("GPU version of ORB not available (OpenCV not built with gpu/cuda module)! Using CPU version instead...");
		gpu_ = false;
	}
#endif
#else
#ifndef HAVE_OPENCV_CUDAFEATURES2D
	if(gpu_)
	{
		UWARN("GPU version of ORB not available (OpenCV cudafeatures2d module)! Using CPU version instead...");
		gpu_ = false;
	}
#endif
	if(gpu_ && cv::cuda::getCudaEnabledDeviceCount() == 0)
	{
		UWARN("GPU version of ORB not available (no GPU found)! Using CPU version instead...");
		gpu_ = false;
	}
#endif
	if(gpu_)
	{
#if CV_MAJOR_VERSION < 3
#ifdef HAVE_OPENCV_GPU
		_gpuOrb = cv::Ptr<CV_ORB_GPU>(new CV_ORB_GPU(this->getMaxFeatures(), scaleFactor_, nLevels_, edgeThreshold_, firstLevel_, WTA_K_, scoreType_, patchSize_));
		_gpuOrb->setFastParams(fastThreshold_, nonmaxSuppresion_);
#else
		UFATAL("not supposed to be here");
#endif
#else
#ifdef HAVE_OPENCV_CUDAFEATURES2D
		_gpuOrb = CV_ORB_GPU::create(this->getMaxFeatures(), scaleFactor_, nLevels_, edgeThreshold_, firstLevel_, WTA_K_, scoreType_, patchSize_, fastThreshold_);
#endif
#endif
	}
	else
	{
#if CV_MAJOR_VERSION < 3
		_orb = cv::Ptr<CV_ORB>(new CV_ORB(this->getMaxFeatures(), scaleFactor_, nLevels_, edgeThreshold_, firstLevel_, WTA_K_, scoreType_, patchSize_, parameters));
#elif CV_MAJOR_VERSION > 3
		_orb = CV_ORB::create(this->getMaxFeatures(), scaleFactor_, nLevels_, edgeThreshold_, firstLevel_, WTA_K_, (cv::ORB::ScoreType)scoreType_, patchSize_, fastThreshold_);
#else
		_orb = CV_ORB::create(this->getMaxFeatures(), scaleFactor_, nLevels_, edgeThreshold_, firstLevel_, WTA_K_, scoreType_, patchSize_, fastThreshold_);
#endif
	}
}

std::vector<cv::KeyPoint> ORB::generateKeypointsImpl(const cv::Mat & image, const cv::Rect & roi, const cv::Mat & mask)
{
	UASSERT(!image.empty() && image.channels() == 1 && image.depth() == CV_8U);
	std::vector<cv::KeyPoint> keypoints;
	cv::Mat imgRoi(image, roi);
	cv::Mat maskRoi;
	if(!mask.empty())
	{
		maskRoi = cv::Mat(mask, roi);
	}

	if(gpu_)
	{
#if CV_MAJOR_VERSION < 3
#ifdef HAVE_OPENCV_GPU
		cv::gpu::GpuMat imgGpu(imgRoi);
		cv::gpu::GpuMat maskGpu(maskRoi);
		(*_gpuOrb.obj)(imgGpu, maskGpu, keypoints);
#else
		UERROR("Cannot use ORBGPU because OpenCV is not built with gpu module.");
#endif
#else
#ifdef HAVE_OPENCV_CUDAFEATURES2D
		cv::cuda::GpuMat d_image(imgRoi);
		cv::cuda::GpuMat d_mask(maskRoi);
		try {
			_gpuOrb->detectAndCompute(d_image, d_mask, keypoints, cv::cuda::GpuMat(), false);
		} catch (cv::Exception& e) {
			const char* err_msg = e.what();
			UWARN("OpenCV exception caught: %s", err_msg);
		}
#endif
#endif
	}
	else
	{
		_orb->detect(imgRoi, keypoints, maskRoi);
	}

	return keypoints;
}

cv::Mat ORB::generateDescriptorsImpl(const cv::Mat & image, std::vector<cv::KeyPoint> & keypoints) const
{
	UASSERT(!image.empty() && image.channels() == 1 && image.depth() == CV_8U);
	cv::Mat descriptors;
	if(image.empty())
	{
		ULOGGER_ERROR("Image is null ?!?");
		return descriptors;
	}
	if(gpu_)
	{
#if CV_MAJOR_VERSION < 3
#ifdef HAVE_OPENCV_GPU
		cv::gpu::GpuMat imgGpu(image);
		cv::gpu::GpuMat descriptorsGPU;
		(*_gpuOrb.obj)(imgGpu, cv::gpu::GpuMat(), keypoints, descriptorsGPU);
		// Download descriptors
		if (descriptorsGPU.empty())
			descriptors = cv::Mat();
		else
		{
			UASSERT(descriptorsGPU.type() == CV_32F);
			descriptors = cv::Mat(descriptorsGPU.size(), CV_32F);
			descriptorsGPU.download(descriptors);
		}
#else
		UERROR("GPU version of ORB not available (OpenCV not built with gpu/cuda module)! Using CPU version instead...");
#endif
#else
#ifdef HAVE_OPENCV_CUDAFEATURES2D
		cv::cuda::GpuMat d_image(image);
		cv::cuda::GpuMat d_descriptors;
		try {
			_gpuOrb->detectAndCompute(d_image, cv::cuda::GpuMat(), keypoints, d_descriptors, true);
		} catch (cv::Exception& e) {
			const char* err_msg = e.what();
			UWARN("OpenCV exception caught: %s", err_msg);
		}
		// Download descriptors
		if (d_descriptors.empty())
			descriptors = cv::Mat();
		else
		{
			UASSERT(d_descriptors.type() == CV_32F || d_descriptors.type() == CV_8U);
			d_descriptors.download(descriptors);
		}
#endif
#endif
	}
	else
	{
		_orb->compute(image, keypoints, descriptors);
	}

	return descriptors;
}

//////////////////////////
//FAST
//////////////////////////
FAST::FAST(const ParametersMap & parameters) :
		threshold_(Parameters::defaultFASTThreshold()),
		nonmaxSuppression_(Parameters::defaultFASTNonmaxSuppression()),
		gpu_(Parameters::defaultFASTGpu()),
		gpuKeypointsRatio_(Parameters::defaultFASTGpuKeypointsRatio()),
		minThreshold_(Parameters::defaultFASTMinThreshold()),
		maxThreshold_(Parameters::defaultFASTMaxThreshold()),
		gridRows_(Parameters::defaultFASTGridRows()),
		gridCols_(Parameters::defaultFASTGridCols()),
		fastCV_(Parameters::defaultFASTCV()),
		fastCVinit_(false),
		fastCVMaxFeatures_(10000),
		fastCVLastImageHeight_(0)
{
#ifdef RTABMAP_FASTCV
	char sVersion[128] = { 0 };
	fcvGetVersion(sVersion, 128);
	UINFO("fastcv version = %s", sVersion);
	int ix;
	if ((ix = fcvSetOperationMode(FASTCV_OP_PERFORMANCE)))
	{
		UERROR("fcvSetOperationMode return=%d, OpenCV FAST will be used instead!", ix);
		fastCV_ = 0;
	}
	else
	{
		fcvMemInit();

		if (!(fastCVCorners_ = (uint32_t*)fcvMemAlloc(fastCVMaxFeatures_ * sizeof(uint32_t) * 2, 16)) ||
			!(fastCVCornerScores_ = (uint32_t*)fcvMemAlloc( fastCVMaxFeatures_ * sizeof(uint32_t), 16 )))
		{
			UERROR("could not alloc fastcv mem, using opencv fast instead!");

			if (fastCVCorners_)
			{
				fcvMemFree(fastCVCorners_);
				fastCVCorners_ = NULL;
			}
			if (fastCVCornerScores_)
			{
				fcvMemFree(fastCVCornerScores_);
				fastCVCornerScores_ = NULL;
			}
		}
		else
		{
			fastCVinit_ = true;
		}
	}
	#endif
	parseParameters(parameters);
}

FAST::~FAST()
{
#ifdef RTABMAP_FASTCV
	if(fastCVinit_)
	{
		fcvMemDeInit();

		if (fastCVCorners_)
			fcvMemFree(fastCVCorners_);
		if (fastCVCornerScores_)
			fcvMemFree(fastCVCornerScores_);
		if (fastCVTempBuf_)
			fcvMemFree(fastCVTempBuf_);
	}
#endif
}

void FAST::parseParameters(const ParametersMap & parameters)
{
	Feature2D::parseParameters(parameters);

	Parameters::parse(parameters, Parameters::kFASTThreshold(), threshold_);
	Parameters::parse(parameters, Parameters::kFASTNonmaxSuppression(), nonmaxSuppression_);
	Parameters::parse(parameters, Parameters::kFASTGpu(), gpu_);
	Parameters::parse(parameters, Parameters::kFASTGpuKeypointsRatio(), gpuKeypointsRatio_);

	Parameters::parse(parameters, Parameters::kFASTMinThreshold(), minThreshold_);
	Parameters::parse(parameters, Parameters::kFASTMaxThreshold(), maxThreshold_);
	Parameters::parse(parameters, Parameters::kFASTGridRows(), gridRows_);
	Parameters::parse(parameters, Parameters::kFASTGridCols(), gridCols_);

	Parameters::parse(parameters, Parameters::kFASTCV(), fastCV_);
	UASSERT(fastCV_ == 0 || fastCV_ == 9 || fastCV_ == 10);

	UASSERT_MSG(threshold_ >= minThreshold_, uFormat("%d vs %d", threshold_, minThreshold_).c_str());
	UASSERT_MSG(threshold_ <= maxThreshold_, uFormat("%d vs %d", threshold_, maxThreshold_).c_str());

#if CV_MAJOR_VERSION < 3
#ifdef HAVE_OPENCV_GPU
	if(gpu_ && cv::gpu::getCudaEnabledDeviceCount() == 0)
	{
		UWARN("GPU version of FAST not available! Using CPU version instead...");
		gpu_ = false;
	}
#else
	if(gpu_)
	{
		UWARN("GPU version of FAST not available (OpenCV not built with gpu/cuda module)! Using CPU version instead...");
		gpu_ = false;
	}
#endif
#else
#ifdef HAVE_OPENCV_CUDAFEATURES2D
	if(gpu_ && cv::cuda::getCudaEnabledDeviceCount() == 0)
	{
		UWARN("GPU version of FAST not available! Using CPU version instead...");
		gpu_ = false;
	}
#else
	if(gpu_)
	{
		UWARN("GPU version of FAST not available (OpenCV cudafeatures2d module)! Using CPU version instead...");
		gpu_ = false;
	}
#endif
#endif
	if(gpu_)
	{
#if CV_MAJOR_VERSION < 3
#ifdef HAVE_OPENCV_GPU
		_gpuFast = new CV_FAST_GPU(threshold_, nonmaxSuppression_, gpuKeypointsRatio_);
#else
		UFATAL("not supposed to be here!");
#endif
#else
#ifdef HAVE_OPENCV_CUDAFEATURES2D
		UFATAL("not implemented");
#endif
#endif
	}
	else
	{
#if CV_MAJOR_VERSION < 3
		if(gridRows_ > 0 && gridCols_ > 0)
		{
			UDEBUG("grid max features = %d", this->getMaxFeatures());
			cv::Ptr<cv::FeatureDetector> fastAdjuster = cv::Ptr<cv::FastAdjuster>(new cv::FastAdjuster(threshold_, nonmaxSuppression_, minThreshold_, maxThreshold_));
			_fast = cv::Ptr<cv::FeatureDetector>(new cv::GridAdaptedFeatureDetector(fastAdjuster, this->getMaxFeatures(), gridRows_, gridCols_));
		}
		else
		{
			if(gridRows_ > 0)
			{
				UWARN("Parameter \"%s\" is set (value=%d) but not \"%s\"! Grid adaptor will not be added.",
						Parameters::kFASTGridRows().c_str(), gridRows_, Parameters::kFASTGridCols().c_str());
			}
			else if(gridCols_ > 0)
			{
				UWARN("Parameter \"%s\" is set (value=%d) but not \"%s\"! Grid adaptor will not be added.",
						Parameters::kFASTGridCols().c_str(), gridCols_, Parameters::kFASTGridRows().c_str());
			}
			_fast = cv::Ptr<cv::FeatureDetector>(new CV_FAST(threshold_, nonmaxSuppression_));
		}
#else
		_fast = CV_FAST::create(threshold_, nonmaxSuppression_);
#endif
	}
}

std::vector<cv::KeyPoint> FAST::generateKeypointsImpl(const cv::Mat & image, const cv::Rect & roi, const cv::Mat & mask)
{
	UASSERT(!image.empty() && image.channels() == 1 && image.depth() == CV_8U);
	std::vector<cv::KeyPoint> keypoints;

#ifdef RTABMAP_FASTCV
	if(fastCV_>0)
	{
		// Note: mask not supported, it should be the inverse of the current mask used (0=where to extract)
		uint32_t nCorners = 0;

		UASSERT(fastCVCorners_ != NULL && fastCVCornerScores_ != NULL);
		if (nonmaxSuppression_)
		{
			if(fastCVTempBuf_==NULL || (fastCVTempBuf_!= NULL && fastCVLastImageHeight_!= image.rows))
			{
				if (fastCVTempBuf_)
				{
					fcvMemFree(fastCVTempBuf_);
					fastCVTempBuf_ = NULL;
				}
				if(!(fastCVTempBuf_ = (uint32_t*)fcvMemAlloc( (3*fastCVMaxFeatures_+image.rows+1)*4, 16 )))
				{
					UERROR("could not alloc fastcv mem for temp buf (%s=true)", Parameters::kFASTNonmaxSuppression().c_str());
					fastCVLastImageHeight_ = 0;
					return keypoints;
				}
				fastCVLastImageHeight_ = image.rows;
			}
		}

		// image.data should be 128 bits aligned
		UDEBUG("%dx%d (step=%d) thr=%d maxFeatures=%d", image.cols, image.rows, image.step1(), threshold_, fastCVMaxFeatures_);
		if(fastCV_ == 10)
		{
			fcvCornerFast10Scoreu8(image.data, image.cols, image.rows, 0, threshold_, 0, fastCVCorners_, fastCVCornerScores_, fastCVMaxFeatures_, &nCorners, nonmaxSuppression_?1:0, fastCVTempBuf_);
		}
		else
		{
			fcvCornerFast9Scoreu8_v2(image.data, image.cols, image.rows, image.step1(), threshold_, 0, fastCVCorners_, fastCVCornerScores_, fastCVMaxFeatures_, &nCorners, nonmaxSuppression_?1:0, fastCVTempBuf_);
		}
		UDEBUG("number of corners found = %d:", nCorners);
		keypoints.resize(nCorners);
		for (uint32_t i = 0; i < nCorners; i++)
		{
			keypoints[i].pt.x = fastCVCorners_[i * 2];
			keypoints[i].pt.y = fastCVCorners_[(i * 2) + 1];
			keypoints[i].size = 3;
			keypoints[i].response = fastCVCornerScores_[i];
		}

		if(this->getMaxFeatures() > 0)
		{
			this->limitKeypoints(keypoints, this->getMaxFeatures());
		}
		return keypoints;
	}
#endif

	if(fastCV_>0)
	{
		UWARN(  "RTAB-Map is not built with FastCV support. OpenCV's FAST is used instead. "
				"Please set %s to 0. This message will only appear once.",
				Parameters::kFASTCV().c_str());
		fastCV_ = 0;
	}

	cv::Mat imgRoi(image, roi);
	cv::Mat maskRoi;
	if(!mask.empty())
	{
		maskRoi = cv::Mat(mask, roi);
	}
	if(gpu_)
	{
#if CV_MAJOR_VERSION < 3
#ifdef HAVE_OPENCV_GPU
		cv::gpu::GpuMat imgGpu(imgRoi);
		cv::gpu::GpuMat maskGpu(maskRoi);
		(*_gpuFast.obj)(imgGpu, maskGpu, keypoints);
#else
		UERROR("Cannot use FAST GPU because OpenCV is not built with gpu module.");
#endif
#else
#ifdef HAVE_OPENCV_CUDAFEATURES2D
		UFATAL("not implemented");
#endif
#endif
	}
	else
	{
		_fast->detect(imgRoi, keypoints, maskRoi); // Opencv keypoints
	}
	return keypoints;
}

//////////////////////////
//FAST-BRIEF
//////////////////////////
FAST_BRIEF::FAST_BRIEF(const ParametersMap & parameters) :
	FAST(parameters),
	bytes_(Parameters::defaultBRIEFBytes())
{
	parseParameters(parameters);
}

FAST_BRIEF::~FAST_BRIEF()
{
}

void FAST_BRIEF::parseParameters(const ParametersMap & parameters)
{
	FAST::parseParameters(parameters);

	Parameters::parse(parameters, Parameters::kBRIEFBytes(), bytes_);
#if CV_MAJOR_VERSION < 3
	_brief = cv::Ptr<CV_BRIEF>(new CV_BRIEF(bytes_));
#else
#ifdef HAVE_OPENCV_XFEATURES2D
	_brief = CV_BRIEF::create(bytes_);
#else
	UWARN("RTAB-Map is not built with OpenCV xfeatures2d module so Brief cannot be used!");
#endif
#endif
}

cv::Mat FAST_BRIEF::generateDescriptorsImpl(const cv::Mat & image, std::vector<cv::KeyPoint> & keypoints) const
{
	UASSERT(!image.empty() && image.channels() == 1 && image.depth() == CV_8U);
	cv::Mat descriptors;
#if CV_MAJOR_VERSION < 3
	_brief->compute(image, keypoints, descriptors);
#else
#ifdef HAVE_OPENCV_XFEATURES2D
	_brief->compute(image, keypoints, descriptors);
#else
	UWARN("RTAB-Map is not built with OpenCV xfeatures2d module so Brief cannot be used!");
#endif
#endif
	return descriptors;
}

//////////////////////////
//FAST-FREAK
//////////////////////////
FAST_FREAK::FAST_FREAK(const ParametersMap & parameters) :
	FAST(parameters),
	orientationNormalized_(Parameters::defaultFREAKOrientationNormalized()),
	scaleNormalized_(Parameters::defaultFREAKScaleNormalized()),
	patternScale_(Parameters::defaultFREAKPatternScale()),
	nOctaves_(Parameters::defaultFREAKNOctaves())
{
	parseParameters(parameters);
}

FAST_FREAK::~FAST_FREAK()
{
}

void FAST_FREAK::parseParameters(const ParametersMap & parameters)
{
	FAST::parseParameters(parameters);

	Parameters::parse(parameters, Parameters::kFREAKOrientationNormalized(), orientationNormalized_);
	Parameters::parse(parameters, Parameters::kFREAKScaleNormalized(), scaleNormalized_);
	Parameters::parse(parameters, Parameters::kFREAKPatternScale(), patternScale_);
	Parameters::parse(parameters, Parameters::kFREAKNOctaves(), nOctaves_);

#if CV_MAJOR_VERSION < 3
	_freak = cv::Ptr<CV_FREAK>(new CV_FREAK(orientationNormalized_, scaleNormalized_, patternScale_, nOctaves_));
#else
#ifdef HAVE_OPENCV_XFEATURES2D
	_freak = CV_FREAK::create(orientationNormalized_, scaleNormalized_, patternScale_, nOctaves_);
#else
	UWARN("RTAB-Map is not built with OpenCV xfeatures2d module so Freak cannot be used!");
#endif
#endif
}

cv::Mat FAST_FREAK::generateDescriptorsImpl(const cv::Mat & image, std::vector<cv::KeyPoint> & keypoints) const
{
	UASSERT(!image.empty() && image.channels() == 1 && image.depth() == CV_8U);
	cv::Mat descriptors;
#if CV_MAJOR_VERSION < 3
	_freak->compute(image, keypoints, descriptors);
#else
#ifdef HAVE_OPENCV_XFEATURES2D
	_freak->compute(image, keypoints, descriptors);
#else
	UWARN("RTAB-Map is not built with OpenCV xfeatures2d module so Freak cannot be used!");
#endif
#endif
	return descriptors;
}

//////////////////////////
//GFTT
//////////////////////////
GFTT::GFTT(const ParametersMap & parameters) :
		_qualityLevel(Parameters::defaultGFTTQualityLevel()),
		_minDistance(Parameters::defaultGFTTMinDistance()),
		_blockSize(Parameters::defaultGFTTBlockSize()),
		_useHarrisDetector(Parameters::defaultGFTTUseHarrisDetector()),
		_k(Parameters::defaultGFTTK())
{
	parseParameters(parameters);
}

GFTT::~GFTT()
{
}

void GFTT::parseParameters(const ParametersMap & parameters)
{
	Feature2D::parseParameters(parameters);

	Parameters::parse(parameters, Parameters::kGFTTQualityLevel(), _qualityLevel);
	Parameters::parse(parameters, Parameters::kGFTTMinDistance(), _minDistance);
	Parameters::parse(parameters, Parameters::kGFTTBlockSize(), _blockSize);
	Parameters::parse(parameters, Parameters::kGFTTUseHarrisDetector(), _useHarrisDetector);
	Parameters::parse(parameters, Parameters::kGFTTK(), _k);

#if CV_MAJOR_VERSION < 3
	_gftt = cv::Ptr<CV_GFTT>(new CV_GFTT(this->getMaxFeatures(), _qualityLevel, _minDistance, _blockSize, _useHarrisDetector ,_k));
#else
	_gftt = CV_GFTT::create(this->getMaxFeatures(), _qualityLevel, _minDistance, _blockSize, _useHarrisDetector ,_k);
#endif
}

std::vector<cv::KeyPoint> GFTT::generateKeypointsImpl(const cv::Mat & image, const cv::Rect & roi, const cv::Mat & mask)
{
	UASSERT(!image.empty() && image.channels() == 1 && image.depth() == CV_8U);
	std::vector<cv::KeyPoint> keypoints;
	cv::Mat imgRoi(image, roi);
	cv::Mat maskRoi;
	if(!mask.empty())
	{
		maskRoi = cv::Mat(mask, roi);
	}
	_gftt->detect(imgRoi, keypoints, maskRoi); // Opencv keypoints
	return keypoints;
}

//////////////////////////
//FAST-BRIEF
//////////////////////////
GFTT_BRIEF::GFTT_BRIEF(const ParametersMap & parameters) :
	GFTT(parameters),
	bytes_(Parameters::defaultBRIEFBytes())
{
	parseParameters(parameters);
}

GFTT_BRIEF::~GFTT_BRIEF()
{
}

void GFTT_BRIEF::parseParameters(const ParametersMap & parameters)
{
	GFTT::parseParameters(parameters);

	Parameters::parse(parameters, Parameters::kBRIEFBytes(), bytes_);
#if CV_MAJOR_VERSION < 3
	_brief = cv::Ptr<CV_BRIEF>(new CV_BRIEF(bytes_));
#else
#ifdef HAVE_OPENCV_XFEATURES2D
	_brief = CV_BRIEF::create(bytes_);
#else
	UWARN("RTAB-Map is not built with OpenCV xfeatures2d module so Brief cannot be used!");
#endif
#endif
}

cv::Mat GFTT_BRIEF::generateDescriptorsImpl(const cv::Mat & image, std::vector<cv::KeyPoint> & keypoints) const
{
	UASSERT(!image.empty() && image.channels() == 1 && image.depth() == CV_8U);
	cv::Mat descriptors;
#if CV_MAJOR_VERSION < 3
	_brief->compute(image, keypoints, descriptors);
#else
#ifdef HAVE_OPENCV_XFEATURES2D
	_brief->compute(image, keypoints, descriptors);
#else
	UWARN("RTAB-Map is not built with OpenCV xfeatures2d module so Brief cannot be used!");
#endif
#endif
	return descriptors;
}

//////////////////////////
//GFTT-FREAK
//////////////////////////
GFTT_FREAK::GFTT_FREAK(const ParametersMap & parameters) :
	GFTT(parameters),
	orientationNormalized_(Parameters::defaultFREAKOrientationNormalized()),
	scaleNormalized_(Parameters::defaultFREAKScaleNormalized()),
	patternScale_(Parameters::defaultFREAKPatternScale()),
	nOctaves_(Parameters::defaultFREAKNOctaves())
{
	parseParameters(parameters);
}

GFTT_FREAK::~GFTT_FREAK()
{
}

void GFTT_FREAK::parseParameters(const ParametersMap & parameters)
{
	GFTT::parseParameters(parameters);

	Parameters::parse(parameters, Parameters::kFREAKOrientationNormalized(), orientationNormalized_);
	Parameters::parse(parameters, Parameters::kFREAKScaleNormalized(), scaleNormalized_);
	Parameters::parse(parameters, Parameters::kFREAKPatternScale(), patternScale_);
	Parameters::parse(parameters, Parameters::kFREAKNOctaves(), nOctaves_);

#if CV_MAJOR_VERSION < 3
	_freak = cv::Ptr<CV_FREAK>(new CV_FREAK(orientationNormalized_, scaleNormalized_, patternScale_, nOctaves_));
#else
#ifdef HAVE_OPENCV_XFEATURES2D
	_freak = CV_FREAK::create(orientationNormalized_, scaleNormalized_, patternScale_, nOctaves_);
#else
	UWARN("RTAB-Map is not built with OpenCV xfeatures2d module so Freak cannot be used!");
#endif
#endif
}

cv::Mat GFTT_FREAK::generateDescriptorsImpl(const cv::Mat & image, std::vector<cv::KeyPoint> & keypoints) const
{
	UASSERT(!image.empty() && image.channels() == 1 && image.depth() == CV_8U);
	cv::Mat descriptors;
#if CV_MAJOR_VERSION < 3
	_freak->compute(image, keypoints, descriptors);
#else
#ifdef HAVE_OPENCV_XFEATURES2D
	_freak->compute(image, keypoints, descriptors);
#else
	UWARN("RTAB-Map is not built with OpenCV xfeatures2d module so Freak cannot be used!");
#endif
#endif
	return descriptors;
}

//////////////////////////
//SURF-FREAK
//////////////////////////
SURF_FREAK::SURF_FREAK(const ParametersMap & parameters) :
	SURF(parameters),
	orientationNormalized_(Parameters::defaultFREAKOrientationNormalized()),
	scaleNormalized_(Parameters::defaultFREAKScaleNormalized()),
	patternScale_(Parameters::defaultFREAKPatternScale()),
	nOctaves_(Parameters::defaultFREAKNOctaves())
{
	parseParameters(parameters);
}

SURF_FREAK::~SURF_FREAK()
{
}

void SURF_FREAK::parseParameters(const ParametersMap & parameters)
{
	SURF::parseParameters(parameters);

	Parameters::parse(parameters, Parameters::kFREAKOrientationNormalized(), orientationNormalized_);
	Parameters::parse(parameters, Parameters::kFREAKScaleNormalized(), scaleNormalized_);
	Parameters::parse(parameters, Parameters::kFREAKPatternScale(), patternScale_);
	Parameters::parse(parameters, Parameters::kFREAKNOctaves(), nOctaves_);

#if CV_MAJOR_VERSION < 3
	_freak = cv::Ptr<CV_FREAK>(new CV_FREAK(orientationNormalized_, scaleNormalized_, patternScale_, nOctaves_));
#else
#ifdef HAVE_OPENCV_XFEATURES2D
	_freak = CV_FREAK::create(orientationNormalized_, scaleNormalized_, patternScale_, nOctaves_);
#else
	UWARN("RTAB-Map is not built with OpenCV xfeatures2d module so Freak cannot be used!");
#endif
#endif
}

cv::Mat SURF_FREAK::generateDescriptorsImpl(const cv::Mat & image, std::vector<cv::KeyPoint> & keypoints) const
{
	UASSERT(!image.empty() && image.channels() == 1 && image.depth() == CV_8U);
	cv::Mat descriptors;
#if CV_MAJOR_VERSION < 3
	_freak->compute(image, keypoints, descriptors);
#else
#ifdef HAVE_OPENCV_XFEATURES2D
	_freak->compute(image, keypoints, descriptors);
#else
	UWARN("RTAB-Map is not built with OpenCV xfeatures2d module so Freak cannot be used!");
#endif
#endif
	return descriptors;
}

//////////////////////////
//GFTT-ORB
//////////////////////////
GFTT_ORB::GFTT_ORB(const ParametersMap & parameters) :
	GFTT(parameters),
	_orb(parameters)
{
	parseParameters(parameters);
}

GFTT_ORB::~GFTT_ORB()
{
}

void GFTT_ORB::parseParameters(const ParametersMap & parameters)
{
	GFTT::parseParameters(parameters);
	_orb.parseParameters(parameters);
}

cv::Mat GFTT_ORB::generateDescriptorsImpl(const cv::Mat & image, std::vector<cv::KeyPoint> & keypoints) const
{
	UASSERT(!image.empty() && image.channels() == 1 && image.depth() == CV_8U);
	return _orb.generateDescriptors(image, keypoints);
}

//////////////////////////
//BRISK
//////////////////////////
BRISK::BRISK(const ParametersMap & parameters) :
	thresh_(Parameters::defaultBRISKThresh()),
	octaves_(Parameters::defaultBRISKOctaves()),
	patternScale_(Parameters::defaultBRISKPatternScale())
{
	parseParameters(parameters);
}

BRISK::~BRISK()
{
}

void BRISK::parseParameters(const ParametersMap & parameters)
{
	Feature2D::parseParameters(parameters);

	Parameters::parse(parameters, Parameters::kBRISKThresh(), thresh_);
	Parameters::parse(parameters, Parameters::kBRISKOctaves(), octaves_);
	Parameters::parse(parameters, Parameters::kBRISKPatternScale(), patternScale_);

#if CV_MAJOR_VERSION < 3
	brisk_ = cv::Ptr<CV_BRISK>(new CV_BRISK(thresh_, octaves_, patternScale_));
#else
	brisk_ = CV_BRISK::create(thresh_, octaves_, patternScale_);
#endif
}

std::vector<cv::KeyPoint> BRISK::generateKeypointsImpl(const cv::Mat & image, const cv::Rect & roi, const cv::Mat & mask)
{
	UASSERT(!image.empty() && image.channels() == 1 && image.depth() == CV_8U);
	std::vector<cv::KeyPoint> keypoints;
	cv::Mat imgRoi(image, roi);
	cv::Mat maskRoi;
	if(!mask.empty())
	{
		maskRoi = cv::Mat(mask, roi);
	}
	brisk_->detect(imgRoi, keypoints, maskRoi); // Opencv keypoints
	return keypoints;
}

cv::Mat BRISK::generateDescriptorsImpl(const cv::Mat & image, std::vector<cv::KeyPoint> & keypoints) const
{
	UASSERT(!image.empty() && image.channels() == 1 && image.depth() == CV_8U);
	cv::Mat descriptors;
	brisk_->compute(image, keypoints, descriptors);
	return descriptors;
}

//////////////////////////
//KAZE
//////////////////////////
KAZE::KAZE(const ParametersMap & parameters) :
		extended_(Parameters::defaultKAZEExtended()),
		upright_(Parameters::defaultKAZEUpright()),
		threshold_(Parameters::defaultKAZEThreshold()),
		nOctaves_(Parameters::defaultKAZENOctaves()),
		nOctaveLayers_(Parameters::defaultKAZENOctaveLayers()),
		diffusivity_(Parameters::defaultKAZEDiffusivity())
{
	parseParameters(parameters);
}

KAZE::~KAZE()
{
}

void KAZE::parseParameters(const ParametersMap & parameters)
{
	Feature2D::parseParameters(parameters);
	
	Parameters::parse(parameters, Parameters::kKAZEExtended(), extended_);
	Parameters::parse(parameters, Parameters::kKAZEUpright(), upright_);
	Parameters::parse(parameters, Parameters::kKAZEThreshold(), threshold_);
	Parameters::parse(parameters, Parameters::kKAZENOctaves(), nOctaves_);
	Parameters::parse(parameters, Parameters::kKAZENOctaveLayers(), nOctaveLayers_);
	Parameters::parse(parameters, Parameters::kKAZEDiffusivity(), diffusivity_);

#if CV_MAJOR_VERSION > 3
	kaze_ = cv::KAZE::create(extended_, upright_, threshold_, nOctaves_, nOctaveLayers_, (cv::KAZE::DiffusivityType)diffusivity_);
#elif CV_MAJOR_VERSION > 2
	kaze_ = cv::KAZE::create(extended_, upright_, threshold_, nOctaves_, nOctaveLayers_, diffusivity_);
#else
	UWARN("RTAB-Map is not built with OpenCV3 so Kaze feature cannot be used!");
#endif
}

std::vector<cv::KeyPoint> KAZE::generateKeypointsImpl(const cv::Mat & image, const cv::Rect & roi, const cv::Mat & mask)
{
	UASSERT(!image.empty() && image.channels() == 1 && image.depth() == CV_8U);
	std::vector<cv::KeyPoint> keypoints;
#if CV_MAJOR_VERSION > 2
	cv::Mat imgRoi(image, roi);
	cv::Mat maskRoi;
	if (!mask.empty())
	{
		maskRoi = cv::Mat(mask, roi);
	}
	kaze_->detect(imgRoi, keypoints, maskRoi); // Opencv keypoints
#else
	UWARN("RTAB-Map is not built with OpenCV3 so Kaze feature cannot be used!");
#endif
	return keypoints;
}

cv::Mat KAZE::generateDescriptorsImpl(const cv::Mat & image, std::vector<cv::KeyPoint> & keypoints) const
{
	UASSERT(!image.empty() && image.channels() == 1 && image.depth() == CV_8U);
	cv::Mat descriptors;
#if CV_MAJOR_VERSION > 2
	kaze_->compute(image, keypoints, descriptors);
#else
	UWARN("RTAB-Map is not built with OpenCV3 so Kaze feature cannot be used!");
#endif
	return descriptors;
}

//////////////////////////
//ORBOctree
//////////////////////////
ORBOctree::ORBOctree(const ParametersMap & parameters) :
		scaleFactor_(Parameters::defaultORBScaleFactor()),
		nLevels_(Parameters::defaultORBNLevels()),
		patchSize_(Parameters::defaultORBPatchSize()),
		edgeThreshold_(Parameters::defaultORBEdgeThreshold()),
		fastThreshold_(Parameters::defaultFASTThreshold()),
		fastMinThreshold_(Parameters::defaultFASTMinThreshold())
{
	parseParameters(parameters);
}

ORBOctree::~ORBOctree()
{
}

void ORBOctree::parseParameters(const ParametersMap & parameters)
{
	Feature2D::parseParameters(parameters);

	Parameters::parse(parameters, Parameters::kORBScaleFactor(), scaleFactor_);
	Parameters::parse(parameters, Parameters::kORBNLevels(), nLevels_);
	Parameters::parse(parameters, Parameters::kORBPatchSize(), patchSize_);
	Parameters::parse(parameters, Parameters::kORBEdgeThreshold(), edgeThreshold_);

	Parameters::parse(parameters, Parameters::kFASTThreshold(), fastThreshold_);
	Parameters::parse(parameters, Parameters::kFASTMinThreshold(), fastMinThreshold_);

#ifdef RTABMAP_ORB_OCTREE
	_orb = cv::Ptr<ORBextractor>(new ORBextractor(this->getMaxFeatures(), scaleFactor_, nLevels_, fastThreshold_, fastMinThreshold_, patchSize_, edgeThreshold_));
#else
	UWARN("RTAB-Map is not built with ORB OcTree option enabled so ORB OcTree feature cannot be used!");
#endif
}

std::vector<cv::KeyPoint> ORBOctree::generateKeypointsImpl(const cv::Mat & image, const cv::Rect & roi, const cv::Mat & mask)
{
	std::vector<cv::KeyPoint> keypoints;
	descriptors_ = cv::Mat();
#ifdef RTABMAP_ORB_OCTREE
	UASSERT(!image.empty() && image.channels() == 1 && image.depth() == CV_8U);
	cv::Mat imgRoi(image, roi);
	cv::Mat maskRoi;
	if(!mask.empty())
	{
		maskRoi = cv::Mat(mask, roi);
	}

	(*_orb)(imgRoi, maskRoi, keypoints, descriptors_);

	if((int)keypoints.size() > this->getMaxFeatures())
	{
		limitKeypoints(keypoints, descriptors_, this->getMaxFeatures());
	}
#else
	UWARN("RTAB-Map is not built with ORB OcTree option enabled so ORB OcTree feature cannot be used!");
#endif
	return keypoints;
}

cv::Mat ORBOctree::generateDescriptorsImpl(const cv::Mat & image, std::vector<cv::KeyPoint> & keypoints) const
{
#ifdef RTABMAP_ORB_OCTREE
	UASSERT_MSG((int)keypoints.size() == descriptors_.rows, uFormat("keypoints=%d descriptors=%d", (int)keypoints.size(), descriptors_.rows).c_str());
#else
	UWARN("RTAB-Map is not built with ORB OcTree option enabled so ORB OcTree feature cannot be used!");
#endif
	return descriptors_;
}

//////////////////////////
//SuperPointTorch
//////////////////////////
SuperPointTorch::SuperPointTorch(const ParametersMap & parameters) :
		path_(Parameters::defaultSuperPointModelPath()),
		threshold_(Parameters::defaultSuperPointThreshold()),
		nms_(Parameters::defaultSuperPointNMS()),
		minDistance_(Parameters::defaultSuperPointNMSRadius()),
		cuda_(Parameters::defaultSuperPointCuda())
{
	parseParameters(parameters);
}

SuperPointTorch::~SuperPointTorch()
{
}

void SuperPointTorch::parseParameters(const ParametersMap & parameters)
{
	Feature2D::parseParameters(parameters);

	std::string previousPath = path_;
#ifdef RTABMAP_TORCH
	bool previousCuda = cuda_;
#endif
	Parameters::parse(parameters, Parameters::kSuperPointModelPath(), path_);
	Parameters::parse(parameters, Parameters::kSuperPointThreshold(), threshold_);
	Parameters::parse(parameters, Parameters::kSuperPointNMS(), nms_);
	Parameters::parse(parameters, Parameters::kSuperPointNMSRadius(), minDistance_);
	Parameters::parse(parameters, Parameters::kSuperPointCuda(), cuda_);

#ifdef RTABMAP_TORCH
	if(superPoint_.get() == 0 || path_.compare(previousPath) != 0 || previousCuda != cuda_)
	{
		superPoint_ = cv::Ptr<SPDetector>(new SPDetector(path_, threshold_, nms_, minDistance_, cuda_));
	}
	else
	{
		superPoint_->setThreshold(threshold_);
		superPoint_->SetNMS(nms_);
		superPoint_->setMinDistance(minDistance_);
	}
#else
	UWARN("RTAB-Map is not built with Torch support so SuperPoint Torch feature cannot be used!");
#endif
}

std::vector<cv::KeyPoint> SuperPointTorch::generateKeypointsImpl(const cv::Mat & image, const cv::Rect & roi, const cv::Mat & mask)
{
#ifdef RTABMAP_TORCH
	UASSERT(!image.empty() && image.channels() == 1 && image.depth() == CV_8U);
	if(roi.x!=0 || roi.y !=0)
	{
		UERROR("SuperPoint: Not supporting ROI (%d,%d,%d,%d). Make sure %s, %s, %s, %s, %s, %s are all set to default values.",
				roi.x, roi.y, roi.width, roi.height,
				Parameters::kKpRoiRatios().c_str(),
				Parameters::kVisRoiRatios().c_str(),
				Parameters::kVisGridRows().c_str(),
				Parameters::kVisGridCols().c_str(),
				Parameters::kKpGridRows().c_str(),
				Parameters::kKpGridCols().c_str());
		return std::vector<cv::KeyPoint>();
	}
	return superPoint_->detect(image, mask);
#else
	UWARN("RTAB-Map is not built with Torch support so SuperPoint Torch feature cannot be used!");
	return std::vector<cv::KeyPoint>();
#endif
}

cv::Mat SuperPointTorch::generateDescriptorsImpl(const cv::Mat & image, std::vector<cv::KeyPoint> & keypoints) const
{
#ifdef RTABMAP_TORCH
	UASSERT(!image.empty() && image.channels() == 1 && image.depth() == CV_8U);
	return superPoint_->compute(keypoints);
#else
	UWARN("RTAB-Map is not built with Torch support so SuperPoint Torch feature cannot be used!");
	return cv::Mat();
#endif
}


//////////////////////////
//GFTT-DAISY
//////////////////////////
GFTT_DAISY::GFTT_DAISY(const ParametersMap & parameters) :
	GFTT(parameters),
	orientationNormalized_(Parameters::defaultFREAKOrientationNormalized()),
	scaleNormalized_(Parameters::defaultFREAKScaleNormalized()),
	patternScale_(Parameters::defaultFREAKPatternScale()),
	nOctaves_(Parameters::defaultFREAKNOctaves())
{
	parseParameters(parameters);
}

GFTT_DAISY::~GFTT_DAISY()
{
}

void GFTT_DAISY::parseParameters(const ParametersMap & parameters)
{
	GFTT::parseParameters(parameters);

	Parameters::parse(parameters, Parameters::kFREAKOrientationNormalized(), orientationNormalized_);
	Parameters::parse(parameters, Parameters::kFREAKScaleNormalized(), scaleNormalized_);
	Parameters::parse(parameters, Parameters::kFREAKPatternScale(), patternScale_);
	Parameters::parse(parameters, Parameters::kFREAKNOctaves(), nOctaves_);

#ifdef HAVE_OPENCV_XFEATURES2D
	_daisy = CV_DAISY::create();
#else
	UWARN("RTAB-Map is not built with OpenCV xfeatures2d module so DAISY cannot be used!");
#endif
}

cv::Mat GFTT_DAISY::generateDescriptorsImpl(const cv::Mat & image, std::vector<cv::KeyPoint> & keypoints) const
{
	UASSERT(!image.empty() && image.channels() == 1 && image.depth() == CV_8U);
	cv::Mat descriptors;
#ifdef HAVE_OPENCV_XFEATURES2D
	_daisy->compute(image, keypoints, descriptors);
#else
	UWARN("RTAB-Map is not built with OpenCV xfeatures2d module so DAISY cannot be used!");
#endif
	return descriptors;
}

//////////////////////////
//SURF-DAISY
//////////////////////////
SURF_DAISY::SURF_DAISY(const ParametersMap & parameters) :
	SURF(parameters),
	orientationNormalized_(Parameters::defaultFREAKOrientationNormalized()),
	scaleNormalized_(Parameters::defaultFREAKScaleNormalized()),
	patternScale_(Parameters::defaultFREAKPatternScale()),
	nOctaves_(Parameters::defaultFREAKNOctaves())
{
	parseParameters(parameters);
}

SURF_DAISY::~SURF_DAISY()
{
}

void SURF_DAISY::parseParameters(const ParametersMap & parameters)
{
	SURF::parseParameters(parameters);

	Parameters::parse(parameters, Parameters::kFREAKOrientationNormalized(), orientationNormalized_);
	Parameters::parse(parameters, Parameters::kFREAKScaleNormalized(), scaleNormalized_);
	Parameters::parse(parameters, Parameters::kFREAKPatternScale(), patternScale_);
	Parameters::parse(parameters, Parameters::kFREAKNOctaves(), nOctaves_);

#ifdef HAVE_OPENCV_XFEATURES2D
	_daisy = CV_DAISY::create();
#else
	UWARN("RTAB-Map is not built with OpenCV xfeatures2d module so DAISY cannot be used!");
#endif
}

cv::Mat SURF_DAISY::generateDescriptorsImpl(const cv::Mat & image, std::vector<cv::KeyPoint> & keypoints) const
{
	UASSERT(!image.empty() && image.channels() == 1 && image.depth() == CV_8U);
	cv::Mat descriptors;
#ifdef HAVE_OPENCV_XFEATURES2D
	_daisy->compute(image, keypoints, descriptors);
#else
	UWARN("RTAB-Map is not built with OpenCV xfeatures2d module so DAISY cannot be used!");
#endif
	return descriptors;
}

}
