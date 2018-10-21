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
		std::vector<cv::Point3f> kpts3DTmp(maxKeypoints);
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
		ULOGGER_DEBUG("too much words (%d), removing words with the hessian threshold", keypoints.size());
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
		inliers.resize(keypoints.size(), true);
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
	if(maxFeatures_ > 0)
	{
		maxFeatures_ =	maxFeatures_ / (gridRows_ * gridCols_);
	}

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
#ifndef RTABMAP_NONFREE
	if(type == Feature2D::kFeatureSurf || type == Feature2D::kFeatureSift)
	{
#if CV_MAJOR_VERSION < 3
		UWARN("SURF and SIFT features cannot be used because OpenCV was not built with nonfree module. ORB is used instead.");
#else
		UWARN("SURF and SIFT features cannot be used because OpenCV was not built with xfeatures2d module. ORB is used instead.");
#endif
		type = Feature2D::kFeatureOrb;
	}
#if CV_MAJOR_VERSION == 3
	if(type == Feature2D::kFeatureFastBrief ||
	   type == Feature2D::kFeatureFastFreak ||
	   type == Feature2D::kFeatureGfttBrief ||
	   type == Feature2D::kFeatureGfttFreak)
	{
		UWARN("BRIEF and FREAK features cannot be used because OpenCV was not built with xfeatures2d module. ORB is used instead.");
		type = Feature2D::kFeatureOrb;
	}
#endif
#endif

#if CV_MAJOR_VERSION < 3
	if(type == Feature2D::kFeatureKaze)
	{
#ifdef RTABMAP_NONFREE
		UWARN("KAZE detector/descriptor can be used only with OpenCV3. SURF is used instead.");
		type = Feature2D::kFeatureSurf;
#else
		UWARN("KAZE detector/descriptor can be used only with OpenCV3. ORB is used instead.");
		type = Feature2D::kFeatureOrb;
#endif
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
#ifdef RTABMAP_NONFREE
	default:
		feature2D = new SURF(parameters);
		type = Feature2D::kFeatureSurf;
		break;
#else
	default:
		feature2D = new ORB(parameters);
		type = Feature2D::kFeatureOrb;
		break;
#endif

	}
	return feature2D;
}

std::vector<cv::KeyPoint> Feature2D::generateKeypoints(const cv::Mat & image, const cv::Mat & maskIn) const
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
				   (_maxDepth == 0.0f || value <= _maxDepth))
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
	for (int i = 0; i<gridRows_; ++i)
	{
		for (int j = 0; j<gridCols_; ++j)
		{
			cv::Rect roi(globalRoi.x + j*colSize, globalRoi.y + i*rowSize, colSize, rowSize);
			std::vector<cv::KeyPoint> sub_keypoints;
			sub_keypoints = this->generateKeypointsImpl(image, roi, mask);
			limitKeypoints(sub_keypoints, maxFeatures_);
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
	UDEBUG("Keypoints extraction time = %f s, keypoints extracted = %d (mask empty=%d)", timer.ticks(), keypoints.size(), mask.empty()?1:0);

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
		if(!data.rightRaw().empty() && !data.imageRaw().empty() && data.stereoCameraModel().isValidForProjection())
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
			std::vector<unsigned char> status;

			std::vector<cv::Point2f> rightCorners;
			rightCorners = _stereo->computeCorrespondences(
					imageMono,
					data.rightRaw(),
					leftCorners,
					status);

			keypoints3D = util3d::generateKeypoints3DStereo(
					leftCorners,
					rightCorners,
					data.stereoCameraModel(),
					status,
					_minDepth,
					_maxDepth);
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

std::vector<cv::KeyPoint> SURF::generateKeypointsImpl(const cv::Mat & image, const cv::Rect & roi, const cv::Mat & mask) const
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
	sigma_(Parameters::defaultSIFTSigma())
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

#ifdef RTABMAP_NONFREE
#if CV_MAJOR_VERSION < 3
	_sift = cv::Ptr<CV_SIFT>(new CV_SIFT(this->getMaxFeatures(), nOctaveLayers_, contrastThreshold_, edgeThreshold_, sigma_));
#else
	_sift = CV_SIFT::create(this->getMaxFeatures(), nOctaveLayers_, contrastThreshold_, edgeThreshold_, sigma_);
#endif
#else
	UWARN("RTAB-Map is not built with OpenCV nonfree module so SIFT cannot be used!");
#endif
}

std::vector<cv::KeyPoint> SIFT::generateKeypointsImpl(const cv::Mat & image, const cv::Rect & roi, const cv::Mat & mask) const
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
	_sift->detect(imgRoi, keypoints, maskRoi); // Opencv keypoints
#else
	UWARN("RTAB-Map is not built with OpenCV nonfree module so SIFT cannot be used!");
#endif
	return keypoints;
}

cv::Mat SIFT::generateDescriptorsImpl(const cv::Mat & image, std::vector<cv::KeyPoint> & keypoints) const
{
	UASSERT(!image.empty() && image.channels() == 1 && image.depth() == CV_8U);
	cv::Mat descriptors;
#ifdef RTABMAP_NONFREE
	_sift->compute(image, keypoints, descriptors);
#else
	UWARN("RTAB-Map is not built with OpenCV nonfree module so SIFT cannot be used!");
#endif
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

std::vector<cv::KeyPoint> ORB::generateKeypointsImpl(const cv::Mat & image, const cv::Rect & roi, const cv::Mat & mask) const
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
		gridCols_(Parameters::defaultFASTGridCols())
{
	parseParameters(parameters);
}

FAST::~FAST()
{
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

std::vector<cv::KeyPoint> FAST::generateKeypointsImpl(const cv::Mat & image, const cv::Rect & roi, const cv::Mat & mask) const
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

std::vector<cv::KeyPoint> GFTT::generateKeypointsImpl(const cv::Mat & image, const cv::Rect & roi, const cv::Mat & mask) const
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

std::vector<cv::KeyPoint> BRISK::generateKeypointsImpl(const cv::Mat & image, const cv::Rect & roi, const cv::Mat & mask) const
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

std::vector<cv::KeyPoint> KAZE::generateKeypointsImpl(const cv::Mat & image, const cv::Rect & roi, const cv::Mat & mask) const
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

}
