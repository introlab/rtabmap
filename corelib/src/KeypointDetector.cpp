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
#include "VWDictionary.h"
#include "utilite/ULogger.h"
#include "utilite/UTimer.h"
#include "utilite/UStl.h"
#include "rtabmap/core/Parameters.h"
#include "utilite/UConversion.h"
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/gpu/gpu.hpp>
#include <opencv2/core/version.hpp>

#define OPENCV_SURF_GPU CV_MAJOR_VERSION >= 2 and CV_MINOR_VERSION >=2 and CV_SUBMINOR_VERSION>=1

namespace rtabmap
{

KeypointDetector::KeypointDetector(const ParametersMap & parameters) :
		_wordsPerImageTarget(Parameters::defaultKpWordsPerImage()),
		_usingAdaptiveResponseThr(Parameters::defaultKpUsingAdaptiveResponseThr()),
		_adaptiveResponseThr(1),
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
	if((iter=parameters.find(Parameters::kKpUsingAdaptiveResponseThr())) != parameters.end())
	{
		_usingAdaptiveResponseThr = uStr2Bool((*iter).second.c_str());
	}
	if((iter=parameters.find(Parameters::kKpRoiRatios())) != parameters.end())
	{
		this->setRoi((*iter).second);
	}
}

std::vector<cv::KeyPoint> KeypointDetector::generateKeypoints(const IplImage * image)
{
	ULOGGER_DEBUG("");
	std::vector<cv::KeyPoint> keypoints;
	if(image)
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
			ULOGGER_DEBUG("_adaptiveResponseThr=%f", _adaptiveResponseThr);
			if(keypoints.size() > 0)
			{
				if(keypoints.size() > _wordsPerImageTarget)
				{
					_adaptiveResponseThr *= 1+((float(keypoints.size())/float(_wordsPerImageTarget)-1)/1000);
				}
				else if(keypoints.size() < _wordsPerImageTarget)
				{
					_adaptiveResponseThr *= 1-((1-float(keypoints.size())/float(_wordsPerImageTarget))/1);
				}
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
					if(iter->first!=0)
					{
						_adaptiveResponseThr = iter->first;
					}
					keypoints = kptsTmp;
					ULOGGER_DEBUG("%d keypoints removed, (kept %d)", removed, keypoints.size());
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
				_adaptiveResponseThr /= 2;
			}

			if(_adaptiveResponseThr < this->getMinimumResponseThr())
			{
				_adaptiveResponseThr = this->getMinimumResponseThr();
			}

			ULOGGER_DEBUG("new _adaptiveResponseThr=%f", _adaptiveResponseThr);
			ULOGGER_DEBUG("adjusting hessian threshold time = %f s", timer.ticks());
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

cv::Rect KeypointDetector::computeRoi(const IplImage * image) const
{
	if(image && _roiRatios.size() == 4)
	{
		cv::Rect roi(0, 0, image->width, image->height);
		UDEBUG("roi ratios = %f, %f, %f, %f", _roiRatios[0],_roiRatios[1],_roiRatios[2],_roiRatios[3]);
		UDEBUG("roi = %d, %d, %d, %d", roi.x, roi.y, roi.width, roi.height);

		float width = image->width;
		float height = image->height;
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
		KeypointDetector(parameters)
{
	_params.hessianThreshold = Parameters::defaultSURFHessianThreshold();
	_params.extended = Parameters::defaultSURFExtended();
	_params.nOctaveLayers = Parameters::defaultSURFOctaveLayers();
	_params.nOctaves = Parameters::defaultSURFOctaves();
	_gpuVersion = Parameters::defaultSURFGpuVersion();
	_params.upright = Parameters::defaultSURFUpright();
	this->parseParameters(parameters);
	this->setAdaptiveResponseThr(_params.hessianThreshold);
}

SURFDetector::~SURFDetector()
{
}

void SURFDetector::parseParameters(const ParametersMap & parameters)
{
	ParametersMap::const_iterator iter;
	if((iter=parameters.find(Parameters::kSURFExtended())) != parameters.end())
	{
		_params.extended = uStr2Bool((*iter).second.c_str());
	}
	if((iter=parameters.find(Parameters::kSURFHessianThreshold())) != parameters.end())
	{
		_params.hessianThreshold = std::atof((*iter).second.c_str());
		this->setAdaptiveResponseThr(_params.hessianThreshold);
	}
	if((iter=parameters.find(Parameters::kSURFOctaveLayers())) != parameters.end())
	{
		_params.nOctaveLayers = std::atoi((*iter).second.c_str());
	}
	if((iter=parameters.find(Parameters::kSURFOctaves())) != parameters.end())
	{
		_params.nOctaves = std::atoi((*iter).second.c_str());
	}
	if((iter=parameters.find(Parameters::kSURFOctaves())) != parameters.end())
	{
		_params.nOctaves = std::atoi((*iter).second.c_str());
	}
	if((iter=parameters.find(Parameters::kSURFGpuVersion())) != parameters.end())
	{
		_gpuVersion = uStr2Bool((*iter).second.c_str());
	}
	if((iter=parameters.find(Parameters::kSURFUpright())) != parameters.end())
	{
		_params.upright = uStr2Bool((*iter).second.c_str());
	}
	KeypointDetector::parseParameters(parameters);
}

std::vector<cv::KeyPoint> SURFDetector::_generateKeypoints(const IplImage * image, const cv::Rect & roi) const
{
	ULOGGER_DEBUG("");
	std::vector<cv::KeyPoint> keypoints;
	if(!image)
	{
		ULOGGER_ERROR("Image is null ?!?");
		return keypoints;
	}
	// SURF support only grayscale images
	IplImage * imageGrayScale = 0;
	if(image->nChannels != 1 || image->depth != IPL_DEPTH_8U)
	{
		imageGrayScale = cvCreateImage(cvSize(image->width,image->height), IPL_DEPTH_8U, 1);
		cvCvtColor(image, imageGrayScale, CV_BGR2GRAY);
	}
	cv::Mat img;
	if(imageGrayScale)
	{
		img = cv::Mat(imageGrayScale);
	}
	else
	{
		img =  cv::Mat(image);
	}

	CvSURFParams params = _params;
	if(this->isUsingAdaptiveResponseThr())
	{
		params.hessianThreshold = this->getAdaptiveResponseThr(); // use the adaptive threshold
	}

	cv::Mat imgRoi(img, roi);
#if OPENCV_SURF_GPU
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
#else
	cv::SurfFeatureDetector detector(params.hessianThreshold, params.nOctaves, params.nOctaveLayers, params.upright);
	detector.detect(imgRoi, keypoints);
#endif

	if(imageGrayScale)
	{
		cvReleaseImage(&imageGrayScale);
	}
	return keypoints;
}

//////////////////////////
//SIFTDetector
//////////////////////////
SIFTDetector::SIFTDetector(const ParametersMap & parameters) :
		KeypointDetector(parameters)
{
	_detectorParams.threshold = Parameters::defaultSIFTThreshold();
	_detectorParams.edgeThreshold = Parameters::defaultSIFTEdgeThreshold();
	this->parseParameters(parameters);
	this->setAdaptiveResponseThr(_detectorParams.threshold);
}

SIFTDetector::~SIFTDetector()
{
}

void SIFTDetector::parseParameters(const ParametersMap & parameters)
{
	ParametersMap::const_iterator iter;
	if((iter=parameters.find(Parameters::kSIFTThreshold())) != parameters.end())
	{
		_detectorParams.threshold = std::atof((*iter).second.c_str());
		this->setAdaptiveResponseThr(_detectorParams.threshold);
	}
	if((iter=parameters.find(Parameters::kSIFTEdgeThreshold())) != parameters.end())
	{
		_detectorParams.edgeThreshold = std::atof((*iter).second.c_str());
	}
	KeypointDetector::parseParameters(parameters);
}

std::vector<cv::KeyPoint> SIFTDetector::_generateKeypoints(const IplImage * image, const cv::Rect & roi) const
{
	ULOGGER_DEBUG("");
	std::vector<cv::KeyPoint> keypoints;
	if(!image)
	{
		ULOGGER_ERROR("Image is null ?!?");
		return keypoints;
	}
	// SURF support only grayscale images
	IplImage * imageGrayScale = 0;
	if(image->nChannels != 1 || image->depth != IPL_DEPTH_8U)
	{
		imageGrayScale = cvCreateImage(cvSize(image->width,image->height), IPL_DEPTH_8U, 1);
		cvCvtColor(image, imageGrayScale, CV_BGR2GRAY);
	}
	cv::Mat img;
	if(imageGrayScale)
	{
		img = cv::Mat(imageGrayScale);
	}
	else
	{
		img =  cv::Mat(image);
	}

	cv::SIFT::DetectorParams detectorParam = _detectorParams;
	if(this->isUsingAdaptiveResponseThr())
	{
		detectorParam.threshold = this->getAdaptiveResponseThr(); // use the adaptive threshold
	}

	cv::SiftFeatureDetector detector(detectorParam, _commonParams);
	cv::Mat imgRoi(img, roi);
	detector.detect(imgRoi, keypoints); // Opencv surf keypoints
	if(imageGrayScale)
	{
		cvReleaseImage(&imageGrayScale);
	}
	return keypoints;
}


//////////////////////////
//StarDetector
//////////////////////////
StarDetector::StarDetector(const ParametersMap & parameters) :
	KeypointDetector(parameters)
{
	_params.lineThresholdBinarized = Parameters::defaultStarLineThresholdBinarized();
	_params.lineThresholdProjected = Parameters::defaultStarLineThresholdProjected();
	_params.maxSize = Parameters::defaultStarMaxSize();
	_params.responseThreshold = Parameters::defaultStarResponseThreshold();
	_params.suppressNonmaxSize = Parameters::defaultStarSuppressNonmaxSize();
	this->parseParameters(parameters);
	this->setAdaptiveResponseThr(_params.responseThreshold);
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
		_params.lineThresholdBinarized = std::atoi((*iter).second.c_str());
	}
	if((iter=parameters.find(Parameters::kStarLineThresholdProjected())) != parameters.end())
	{
		_params.lineThresholdProjected = std::atoi((*iter).second.c_str());
	}
	if((iter=parameters.find(Parameters::kStarMaxSize())) != parameters.end())
	{
		_params.maxSize = std::atoi((*iter).second.c_str());
	}
	if((iter=parameters.find(Parameters::kStarResponseThreshold())) != parameters.end())
	{
		_params.responseThreshold = int(std::atof((*iter).second.c_str()));
		this->setAdaptiveResponseThr(_params.responseThreshold);
	}
	if((iter=parameters.find(Parameters::kStarSuppressNonmaxSize())) != parameters.end())
	{
		_params.suppressNonmaxSize = std::atoi((*iter).second.c_str());
	}
	KeypointDetector::parseParameters(parameters);
}

std::vector<cv::KeyPoint> StarDetector::_generateKeypoints(const IplImage * image, const cv::Rect & roi) const
{
	ULOGGER_DEBUG("");
	std::vector<cv::KeyPoint> keypoints;
	if(!image)
	{
		ULOGGER_ERROR("Image is null ?!?");
		return keypoints;
	}

	cv::Mat img(image);
	// TODO More testing needed with the star detector, NN search distance must be changed to 0.8
	//find keypoints with the star detector
	CvStarDetectorParams params = _params;
	if(this->isUsingAdaptiveResponseThr())
	{
		params.responseThreshold = this->getAdaptiveResponseThr(); // use the adaptive threshold
	}

	// Get keypoints with the star detector
	cv::Mat imgRoi(img, roi);
	cv::StarFeatureDetector detector(params);
	detector.detect(imgRoi, keypoints);
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
	this->setAdaptiveResponseThr(_threshold);
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

std::vector<cv::KeyPoint> FASTDetector::_generateKeypoints(const IplImage * image, const cv::Rect & roi) const
{
	ULOGGER_DEBUG("");
	std::vector<cv::KeyPoint> keypoints;
	if(!image)
	{
		ULOGGER_ERROR("Image is null ?!?");
		return keypoints;
	}

	cv::Mat img(image);
	cv::Mat imgRoi(img, roi);
	int threshold = _threshold;
	if(this->isUsingAdaptiveResponseThr())
	{
		threshold = (int)this->getAdaptiveResponseThr(); // use the adaptive threshold
	}
	cv::FastFeatureDetector fast(threshold, _nonmaxSuppression);

	// Get keypoints with the fast detector
	fast.detect(imgRoi, keypoints);
	return keypoints;
}

}
