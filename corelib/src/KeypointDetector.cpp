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

#include "rtabmap/core/KeypointDetector.h"
#include "rtabmap/core/VWDictionary.h"
#include "utilite/ULogger.h"
#include "utilite/UTimer.h"
#include "utilite/UStl.h"
#include "rtabmap/core/Parameters.h"
#include "utilite/UConversion.h"
#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/gpu/gpu.hpp>
#include <opencv2/core/version.hpp>
#if CV_MAJOR_VERSION >=2 and CV_MINOR_VERSION >=4
#include <opencv2/nonfree/features2d.hpp>
#endif

#define OPENCV_SURF_GPU CV_MAJOR_VERSION >= 2 and CV_MINOR_VERSION >=2 and CV_SUBMINOR_VERSION>=1

namespace rtabmap
{

KeypointDetector::KeypointDetector(const ParametersMap & parameters) :
		_wordsPerImageTarget(Parameters::defaultKpWordsPerImage()),
		_roiRatios(std::vector<float>(4, 0.0f))
{
	this->setRoi(Parameters::defaultKpRoiRatios());
	this->parseParameters(parameters);
}

void KeypointDetector::parseParameters(const ParametersMap & parameters)
{
	ParametersMap::const_iterator iter;
	if((iter=parameters.find(Parameters::kKpWordsPerImage())) != parameters.end())
	{
		_wordsPerImageTarget = std::atoi((*iter).second.c_str());
	}
	if((iter=parameters.find(Parameters::kKpRoiRatios())) != parameters.end())
	{
		this->setRoi((*iter).second);
	}
}

std::vector<cv::KeyPoint> KeypointDetector::generateKeypoints(const cv::Mat & image)
{
	ULOGGER_DEBUG("");
	std::vector<cv::KeyPoint> keypoints;
	if(!image.empty())
	{
		UTimer timer;
		timer.start();

		cv::Rect roi = computeRoi(image);

		// Get keypoints
		keypoints = this->_generateKeypoints(image, roi);
		ULOGGER_DEBUG("Keypoints extraction time = %f s, keypoints extracted = %d", timer.ticks(), keypoints.size());

		//clip the number of words... to _wordsPerImageTarget
		// Variable hessian threshold
		if(_wordsPerImageTarget > 0)
		{
			if(keypoints.size() > 0)
			{
				// 10% margin...
				if(keypoints.size() > 1.1 * _wordsPerImageTarget)
				{
					ULOGGER_DEBUG("too much words (%d), removing words under the new hessian threshold", keypoints.size());
					// Remove words under the new hessian threshold

					// Sort words by hessian
					std::multimap<float, std::vector<cv::KeyPoint>::iterator> hessianMap; // <hessian,id>
					for(std::vector<cv::KeyPoint>::iterator itKey = keypoints.begin(); itKey != keypoints.end(); ++itKey)
					{
						//Keep track of the data, to be easier to manage the data in the next step
						hessianMap.insert(std::pair<float, std::vector<cv::KeyPoint>::iterator>(fabs(itKey->response), itKey));
					}

					// Remove them from the signature
					int removed = hessianMap.size()-_wordsPerImageTarget;
					std::multimap<float, std::vector<cv::KeyPoint>::iterator>::reverse_iterator iter = hessianMap.rbegin();
					std::vector<cv::KeyPoint> kptsTmp(_wordsPerImageTarget);
					for(unsigned int k=0; k < kptsTmp.size() && iter!=hessianMap.rend(); ++k, ++iter)
					{
						kptsTmp[k] = *iter->second;
						// Adjust keypoint position to raw image
						kptsTmp[k].pt.x += roi.x;
						kptsTmp[k].pt.y += roi.y;
					}
					keypoints = kptsTmp;
					ULOGGER_DEBUG("%d keypoints removed, (kept %d), minimum response=%f", removed, keypoints.size(), kptsTmp.size()?kptsTmp.back().response:0.0f);
				}
				else if(roi.x || roi.y)
				{
					// Adjust keypoint position to raw image
					for(std::vector<cv::KeyPoint>::iterator iter=keypoints.begin(); iter!=keypoints.end(); ++iter)
					{
						iter->pt.x += roi.x;
						iter->pt.y += roi.y;
					}
				}
			}

			ULOGGER_DEBUG("removing words time = %f s", timer.ticks());
		}
		else if(roi.x || roi.y)
		{
			// Adjust keypoint position to raw image
			for(std::vector<cv::KeyPoint>::iterator iter=keypoints.begin(); iter!=keypoints.end(); ++iter)
			{
				iter->pt.x += roi.x;
				iter->pt.y += roi.y;
			}
		}
	}
	else
	{
		ULOGGER_ERROR("Image is null!");
	}
	return keypoints;
}

void KeypointDetector::setRoi(const std::string & roi)
{
	std::list<std::string> strValues = uSplit(roi, ' ');
	if(strValues.size() != 4)
	{
		ULOGGER_ERROR("The number of values must be 4 (roi=\"%s\")", roi.c_str());
	}
	else
	{
		std::vector<float> tmpValues(4);
		unsigned int i=0;
		for(std::list<std::string>::iterator iter = strValues.begin(); iter!=strValues.end(); ++iter)
		{
			tmpValues[i] = std::atof((*iter).c_str());
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
			ULOGGER_ERROR("The roi ratios are not valid (roi=\"%s\")", roi.c_str());
		}
	}
}

cv::Rect KeypointDetector::computeRoi(const cv::Mat & image) const
{
	if(!image.empty() && _roiRatios.size() == 4)
	{
		float width = image.cols;
		float height = image.rows;
		cv::Rect roi(0, 0, width, height);
		UDEBUG("roi ratios = %f, %f, %f, %f", _roiRatios[0],_roiRatios[1],_roiRatios[2],_roiRatios[3]);
		UDEBUG("roi = %d, %d, %d, %d", roi.x, roi.y, roi.width, roi.height);

		//left roi
		if(_roiRatios[0] > 0 && _roiRatios[0] < 1 - _roiRatios[1])
		{
			roi.x = width * _roiRatios[0];
		}

		//right roi
		roi.width = width - roi.x;
		if(_roiRatios[1] > 0 && _roiRatios[1] < 1 - _roiRatios[0])
		{
			roi.width -= width * _roiRatios[1];
		}

		//top roi
		if(_roiRatios[2] > 0 && _roiRatios[2] < 1 - _roiRatios[3])
		{
			roi.y = height * _roiRatios[2];
		}

		//bottom roi
		roi.height = height - roi.y;
		if(_roiRatios[3] > 0 && _roiRatios[3] < 1 - _roiRatios[2])
		{
			roi.height -= height * _roiRatios[3];
		}
		UDEBUG("roi = %d, %d, %d, %d", roi.x, roi.y, roi.width, roi.height);

		return roi;
	}
	else
	{
		UERROR("Image is null or _roiRatios(=%d) != 4", _roiRatios.size());
		return cv::Rect();
	}
}


//////////////////////////
//SURFDetector
//////////////////////////
SURFDetector::SURFDetector(const ParametersMap & parameters) :
		KeypointDetector(parameters),
		_hessianThreshold(Parameters::defaultSURFHessianThreshold()),
		_nOctaves(Parameters::defaultSURFOctaves()),
		_nOctaveLayers(Parameters::defaultSURFOctaveLayers()),
		_extended(Parameters::defaultSURFExtended()),
		_upright(Parameters::defaultSURFUpright()),
		_gpuVersion(Parameters::defaultSURFGpuVersion())
{
	this->parseParameters(parameters);
}

SURFDetector::~SURFDetector()
{
}

void SURFDetector::parseParameters(const ParametersMap & parameters)
{
	ParametersMap::const_iterator iter;
	if((iter=parameters.find(Parameters::kSURFExtended())) != parameters.end())
	{
		_extended = uStr2Bool((*iter).second.c_str());
	}
	if((iter=parameters.find(Parameters::kSURFHessianThreshold())) != parameters.end())
	{
		_hessianThreshold = std::atof((*iter).second.c_str());
	}
	if((iter=parameters.find(Parameters::kSURFOctaveLayers())) != parameters.end())
	{
		_nOctaveLayers = std::atoi((*iter).second.c_str());
	}
	if((iter=parameters.find(Parameters::kSURFOctaves())) != parameters.end())
	{
		_nOctaves = std::atoi((*iter).second.c_str());
	}
	if((iter=parameters.find(Parameters::kSURFOctaves())) != parameters.end())
	{
		_nOctaves = std::atoi((*iter).second.c_str());
	}
	if((iter=parameters.find(Parameters::kSURFUpright())) != parameters.end())
	{
		_upright = uStr2Bool((*iter).second.c_str());
	}
	if((iter=parameters.find(Parameters::kSURFGpuVersion())) != parameters.end())
	{
		_gpuVersion = uStr2Bool((*iter).second.c_str());
	}
	KeypointDetector::parseParameters(parameters);
}

std::vector<cv::KeyPoint> SURFDetector::_generateKeypoints(const cv::Mat & image, const cv::Rect & roi) const
{
	ULOGGER_DEBUG("");
	std::vector<cv::KeyPoint> keypoints;
	if(image.empty())
	{
		ULOGGER_ERROR("Image is null ?!?");
		return keypoints;
	}
	// SURF support only grayscale images
	cv::Mat imageGrayScale;
	if(image.channels() != 1 || image.depth() != CV_8U)
	{
		cv::cvtColor(image, imageGrayScale, CV_BGR2GRAY);
	}
	cv::Mat img;
	if(!imageGrayScale.empty())
	{
		img = imageGrayScale;
	}
	else
	{
		img = image;
	}

	cv::Mat imgRoi(img, roi);
/*#if OPENCV_SURF_GPU
	if(_gpuVersion )
	{
		cv::gpu::GpuMat imgGpu(imgRoi);
		cv::gpu::GpuMat keypointsGpu;
		cv::gpu::SURF_GPU surfGpu(params.hessianThreshold, params.nOctaves, params.nOctaveLayers, params.extended, 0.01f, params.upright);
		surfGpu(imgGpu, cv::gpu::GpuMat(), keypointsGpu);
		surfGpu.downloadKeypoints(keypointsGpu, keypoints);
	}
	else
	{
		cv::SurfFeatureDetector detector(params.hessianThreshold, params.nOctaves, params.nOctaveLayers, params.upright);
		detector.detect(imgRoi, keypoints);
	}
#else*/
	cv::SURF detector(_hessianThreshold, _nOctaves, _nOctaveLayers, _extended, _upright);
#if CV_MAJOR_VERSION >=2 and CV_MINOR_VERSION >=4
	detector.detect(imgRoi, keypoints);
#else
	detector(imgRoi, cv::Mat(), keypoints);
#endif
//#endif

	return keypoints;
}

//////////////////////////
//SIFTDetector
//////////////////////////
SIFTDetector::SIFTDetector(const ParametersMap & parameters) :
		KeypointDetector(parameters),
		_nfeatures(Parameters::defaultSIFTNFeatures()),
		_nOctaveLayers(Parameters::defaultSIFTNOctaveLayers()),
		_contrastThreshold(Parameters::defaultSIFTContrastThreshold()),
		_edgeThreshold(Parameters::defaultSIFTEdgeThreshold()),
		_sigma(Parameters::defaultSIFTSigma())
{
	this->parseParameters(parameters);
}

SIFTDetector::~SIFTDetector()
{
}

void SIFTDetector::parseParameters(const ParametersMap & parameters)
{
	ParametersMap::const_iterator iter;
	if((iter=parameters.find(Parameters::kSIFTContrastThreshold())) != parameters.end())
	{
		_contrastThreshold = std::atof((*iter).second.c_str());
	}
	if((iter=parameters.find(Parameters::kSIFTEdgeThreshold())) != parameters.end())
	{
		_edgeThreshold = std::atof((*iter).second.c_str());
	}
	if((iter=parameters.find(Parameters::kSIFTNFeatures())) != parameters.end())
	{
		_nfeatures = std::atoi((*iter).second.c_str());
	}
	if((iter=parameters.find(Parameters::kSIFTNOctaveLayers())) != parameters.end())
	{
		_nOctaveLayers = std::atoi((*iter).second.c_str());
	}
	if((iter=parameters.find(Parameters::kSIFTSigma())) != parameters.end())
	{
		_sigma = std::atof((*iter).second.c_str());
	}
	KeypointDetector::parseParameters(parameters);
}

std::vector<cv::KeyPoint> SIFTDetector::_generateKeypoints(const cv::Mat & image, const cv::Rect & roi) const
{
	ULOGGER_DEBUG("");
	std::vector<cv::KeyPoint> keypoints;
	if(image.empty())
	{
		ULOGGER_ERROR("Image is null ?!?");
		return keypoints;
	}
	// SURF support only grayscale images
	cv::Mat imageGrayScale;
	if(image.channels() != 1 || image.depth() != CV_8U)
	{
		cv::cvtColor(image, imageGrayScale, CV_BGR2GRAY);
	}
	cv::Mat img;
	if(!imageGrayScale.empty())
	{
		img = imageGrayScale;
	}
	else
	{
		img = image;
	}

	cv::Mat imgRoi(img, roi);
#if CV_MAJOR_VERSION >=2 and CV_MINOR_VERSION >=4
	cv::SIFT detector(_nfeatures, _nOctaveLayers, _contrastThreshold, _edgeThreshold, _sigma);
	detector.detect(imgRoi, keypoints); // Opencv surf keypoints
#else
	cv::SIFT detector(_contrastThreshold, _edgeThreshold, cv::SIFT::CommonParams::DEFAULT_NOCTAVES, _nOctaveLayers);
	detector(imgRoi, cv::Mat(), keypoints); // Opencv surf keypoints
#endif
	return keypoints;
}


//////////////////////////
//StarDetector
//////////////////////////
StarDetector::StarDetector(const ParametersMap & parameters) :
	KeypointDetector(parameters),
	_maxSize(Parameters::defaultStarMaxSize()),
	_responseThreshold(Parameters::defaultStarResponseThreshold()),
	_lineThresholdProjected(Parameters::defaultStarLineThresholdProjected()),
	_lineThresholdBinarized(Parameters::defaultStarLineThresholdBinarized()),
	_suppressNonmaxSize(Parameters::defaultStarSuppressNonmaxSize())
{
	this->parseParameters(parameters);
}

StarDetector::~StarDetector()
{
}

void StarDetector::parseParameters(const ParametersMap & parameters)
{
	ULOGGER_WARN("The StarDetector parameters can't be changed on ROS (this is an issue with the default (and too old) opencv revision used in ROS)");
	ParametersMap::const_iterator iter;
	if((iter=parameters.find(Parameters::kStarLineThresholdBinarized())) != parameters.end())
	{
		_lineThresholdBinarized = std::atoi((*iter).second.c_str());
	}
	if((iter=parameters.find(Parameters::kStarLineThresholdProjected())) != parameters.end())
	{
		_lineThresholdProjected = std::atoi((*iter).second.c_str());
	}
	if((iter=parameters.find(Parameters::kStarMaxSize())) != parameters.end())
	{
		_maxSize = std::atoi((*iter).second.c_str());
	}
	if((iter=parameters.find(Parameters::kStarResponseThreshold())) != parameters.end())
	{
		_responseThreshold = int(std::atof((*iter).second.c_str()));
	}
	if((iter=parameters.find(Parameters::kStarSuppressNonmaxSize())) != parameters.end())
	{
		_suppressNonmaxSize = std::atoi((*iter).second.c_str());
	}
	KeypointDetector::parseParameters(parameters);
}

std::vector<cv::KeyPoint> StarDetector::_generateKeypoints(const cv::Mat & image, const cv::Rect & roi) const
{
	ULOGGER_DEBUG("");
	std::vector<cv::KeyPoint> keypoints;
	if(image.empty())
	{
		ULOGGER_ERROR("Image is null ?!?");
		return keypoints;
	}

	cv::Mat img(image);

	// Get keypoints with the star detector
	cv::Mat imgRoi(img, roi);
	cv::StarDetector detector(_maxSize, _responseThreshold, _lineThresholdProjected, _lineThresholdBinarized, _suppressNonmaxSize);
#if CV_MAJOR_VERSION >=2 and CV_MINOR_VERSION >=4
	detector.detect(imgRoi, keypoints);
#else
	detector(imgRoi, keypoints);
#endif
	return keypoints;
}

//////////////////////////
//FastDetector
//////////////////////////
FASTDetector::FASTDetector(const ParametersMap & parameters) :
	KeypointDetector(parameters),
	_threshold(Parameters::defaultFASTThreshold()),
	_nonmaxSuppression(Parameters::defaultFASTNonmaxSuppression())
{
	this->parseParameters(parameters);
}

FASTDetector::~FASTDetector()
{
}

void FASTDetector::parseParameters(const ParametersMap & parameters)
{
	ParametersMap::const_iterator iter;
	if((iter=parameters.find(Parameters::kFASTThreshold())) != parameters.end())
	{
		_threshold = std::atoi((*iter).second.c_str());
	}
	if((iter=parameters.find(Parameters::kFASTNonmaxSuppression())) != parameters.end())
	{
		_nonmaxSuppression = uStr2Bool((*iter).second.c_str());
	}
	KeypointDetector::parseParameters(parameters);
}

std::vector<cv::KeyPoint> FASTDetector::_generateKeypoints(const cv::Mat & image, const cv::Rect & roi) const
{
	ULOGGER_DEBUG("");
	std::vector<cv::KeyPoint> keypoints;
	if(image.empty())
	{
		ULOGGER_ERROR("Image is null ?!?");
		return keypoints;
	}

	cv::Mat img(image);
	cv::Mat imgRoi(img, roi);

	cv::FastFeatureDetector fast(_threshold, _nonmaxSuppression);

	// Get keypoints with the fast detector
	fast.detect(imgRoi, keypoints);
	return keypoints;
}

}
