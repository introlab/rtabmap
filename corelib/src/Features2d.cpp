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

#include "rtabmap/core/Features2d.h"
#include "rtabmap/core/util3d.h"
#include "rtabmap/utilite/UStl.h"
#include "rtabmap/utilite/UConversion.h"
#include "rtabmap/utilite/ULogger.h"
#include "rtabmap/utilite/UMath.h"
#include "rtabmap/utilite/ULogger.h"
#include "rtabmap/utilite/UTimer.h"
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/gpu/gpu.hpp>
#include <opencv2/nonfree/gpu.hpp>
#include <opencv2/core/version.hpp>
#if CV_MAJOR_VERSION >=2 and CV_MINOR_VERSION >=4
#include <opencv2/nonfree/features2d.hpp>
#endif

#define OPENCV_SURF_GPU CV_MAJOR_VERSION >= 2 and CV_MINOR_VERSION >=2 and CV_SUBMINOR_VERSION>=1

namespace rtabmap {

void filterKeypointsByDepth(
		std::vector<cv::KeyPoint> & keypoints,
		const cv::Mat & depth,
		float depthConstant,
		float maxDepth)
{
	cv::Mat descriptors;
	filterKeypointsByDepth(keypoints, descriptors, depth, depthConstant, maxDepth);
}

void filterKeypointsByDepth(
		std::vector<cv::KeyPoint> & keypoints,
		cv::Mat & descriptors,
		const cv::Mat & depth,
		float depthConstant,
		float maxDepth)
{
	if(!depth.empty() && depthConstant > 0.0f && maxDepth > 0.0f && (descriptors.empty() || descriptors.rows == keypoints.size()))
	{
		std::vector<cv::KeyPoint> output(keypoints.size());
		std::vector<int> indexes(keypoints.size(), 0);
		int oi=0;
		for(unsigned int i=0; i<keypoints.size(); ++i)
		{
			pcl::PointXYZ pt = util3d::getDepth(depth, keypoints[i].pt.x, keypoints[i].pt.y, depthConstant);
			if(uIsFinite(pt.z) && pt.z < maxDepth)
			{
				output[oi++] = keypoints[i];
				indexes[i] = 1;
			}
		}
		output.resize(oi);
		keypoints = output;

		if(!descriptors.empty() && keypoints.size() != descriptors.rows)
		{
			if(keypoints.size() == 0)
			{
				descriptors = cv::Mat();
			}
			else
			{
				cv::Mat newDescriptors(keypoints.size(), descriptors.cols, descriptors.type());
				int di = 0;
				for(unsigned int i=0; i<indexes.size(); ++i)
				{
					if(indexes[i] == 1)
					{
						memcpy(newDescriptors.ptr<float>(di++), descriptors.ptr<float>(i), descriptors.cols*sizeof(float));
					}
				}
				descriptors = newDescriptors;
			}
		}
	}
}

void limitKeypoints(std::vector<cv::KeyPoint> & keypoints, int maxKeypoints)
{
	cv::Mat descriptors;
	limitKeypoints(keypoints, descriptors, maxKeypoints);
}

void limitKeypoints(std::vector<cv::KeyPoint> & keypoints, cv::Mat & descriptors, int maxKeypoints)
{
	UASSERT((int)keypoints.size() == descriptors.rows || descriptors.rows == 0);
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
		int removed = hessianMap.size()-maxKeypoints;
		std::multimap<float, int>::reverse_iterator iter = hessianMap.rbegin();
		std::vector<cv::KeyPoint> kptsTmp(maxKeypoints);
		cv::Mat descriptorsTmp;
		if(descriptors.rows)
		{
			descriptorsTmp = cv::Mat(maxKeypoints, descriptors.cols, descriptors.type());
		}
		for(unsigned int k=0; k < kptsTmp.size() && iter!=hessianMap.rend(); ++k, ++iter)
		{
			kptsTmp[k] = keypoints[iter->second];
			if(descriptors.rows)
			{
				memcpy(descriptorsTmp.ptr<float>(k), descriptors.ptr<float>(iter->second), descriptors.cols*sizeof(float));
			}
		}
		ULOGGER_DEBUG("%d keypoints removed, (kept %d), minimum response=%f", removed, keypoints.size(), kptsTmp.size()?kptsTmp.back().response:0.0f);
		ULOGGER_DEBUG("removing words time = %f s", timer.ticks());
		keypoints = kptsTmp;
		if(descriptors.rows)
		{
			descriptors = descriptorsTmp;
		}
	}
}

/////////////////////
// KeypointDescriptor
/////////////////////
KeypointDescriptor::KeypointDescriptor(const ParametersMap & parameters)
{
	this->parseParameters(parameters);
}

KeypointDescriptor::~KeypointDescriptor()
{
}

void KeypointDescriptor::parseParameters(const ParametersMap & parameters)
{
}

//////////////////////////
//SURFDescriptor
//////////////////////////
SURFDescriptor::SURFDescriptor(const ParametersMap & parameters) :
	KeypointDescriptor(parameters),
	_hessianThreshold(Parameters::defaultSURFHessianThreshold()),
	_nOctaves(Parameters::defaultSURFOctaves()),
	_nOctaveLayers(Parameters::defaultSURFOctaveLayers()),
	_extended(Parameters::defaultSURFExtended()),
	_upright(Parameters::defaultSURFUpright()),
	_gpuVersion(Parameters::defaultSURFGpuVersion())
{
	this->parseParameters(parameters);
}

SURFDescriptor::~SURFDescriptor()
{
}

void SURFDescriptor::parseParameters(const ParametersMap & parameters)
{
	Parameters::parse(parameters, Parameters::kSURFExtended(), _extended);
	Parameters::parse(parameters, Parameters::kSURFHessianThreshold(), _hessianThreshold);
	Parameters::parse(parameters, Parameters::kSURFOctaveLayers(), _nOctaveLayers);
	Parameters::parse(parameters, Parameters::kSURFOctaves(), _nOctaves);
	Parameters::parse(parameters, Parameters::kSURFUpright(), _upright);
	Parameters::parse(parameters, Parameters::kSURFGpuVersion(), _gpuVersion);
	KeypointDescriptor::parseParameters(parameters);
}

cv::Mat SURFDescriptor::generateDescriptors(const cv::Mat & image, std::vector<cv::KeyPoint> & keypoints) const
{
	ULOGGER_DEBUG("");
	cv::Mat descriptors;
	if(image.empty())
	{
		ULOGGER_ERROR("Image is null ?!?");
		return descriptors;
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
		img =  image;
	}
	if(_gpuVersion && cv::gpu::getCudaEnabledDeviceCount())
	{
		std::vector<float> d;
		cv::gpu::GpuMat imgGpu(img);
		cv::gpu::GpuMat descriptorsGpu;
		cv::gpu::GpuMat keypointsGpu;
		cv::gpu::SURF_GPU surfGpu(_hessianThreshold, _nOctaves, _nOctaveLayers, _extended, 0.01f, _upright);
		surfGpu.uploadKeypoints(keypoints, keypointsGpu);
		surfGpu(imgGpu, cv::gpu::GpuMat(), keypointsGpu, descriptorsGpu, true);
		surfGpu.downloadDescriptors(descriptorsGpu, d);
		unsigned int dim = _extended?128:64;
		descriptors = cv::Mat(d.size()/dim, dim, CV_32F);
		for(int i=0; i<descriptors.rows; ++i)
		{
			float * rowFl = descriptors.ptr<float>(i);
			memcpy(rowFl, &d[i*dim], dim*sizeof(float));
		}
	}
	else
	{
		if(_gpuVersion)
		{
			UWARN("GPU version of SURF not available! Using CPU version instead...");
		}

		cv::SURF extractor(_hessianThreshold, _nOctaves, _nOctaveLayers, _extended, _upright);
		extractor.compute(img, keypoints, descriptors);
	}

	return descriptors;
}

//////////////////////////
//SIFTDescriptor
//////////////////////////
SIFTDescriptor::SIFTDescriptor(const ParametersMap & parameters) :
	KeypointDescriptor(parameters),
	_nfeatures(Parameters::defaultSIFTNFeatures()),
	_nOctaveLayers(Parameters::defaultSIFTNOctaveLayers()),
	_contrastThreshold(Parameters::defaultSIFTContrastThreshold()),
	_edgeThreshold(Parameters::defaultSIFTEdgeThreshold()),
	_sigma(Parameters::defaultSIFTSigma())
{
	this->parseParameters(parameters);
}

SIFTDescriptor::~SIFTDescriptor()
{
}

void SIFTDescriptor::parseParameters(const ParametersMap & parameters)
{
	ParametersMap::const_iterator iter;
	Parameters::parse(parameters, Parameters::kSIFTContrastThreshold(), _contrastThreshold);
	Parameters::parse(parameters, Parameters::kSIFTEdgeThreshold(), _edgeThreshold);
	Parameters::parse(parameters, Parameters::kSIFTNFeatures(), _nfeatures);
	Parameters::parse(parameters, Parameters::kSIFTNOctaveLayers(), _nOctaveLayers);
	Parameters::parse(parameters, Parameters::kSIFTSigma(), _sigma);
	KeypointDescriptor::parseParameters(parameters);
}

cv::Mat SIFTDescriptor::generateDescriptors(const cv::Mat & image, std::vector<cv::KeyPoint> & keypoints) const
{
	ULOGGER_DEBUG("");
	cv::Mat descriptors;
	if(image.empty())
	{
		ULOGGER_ERROR("Image is null ?!?");
		return descriptors;
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
		img =  image;
	}

	cv::SIFT extractor(_nfeatures, _nOctaveLayers, _contrastThreshold, _edgeThreshold, _sigma);
	extractor.compute(img, keypoints, descriptors);

	return descriptors;
}




/////////////////////
// KeypointDetector
/////////////////////
KeypointDetector::KeypointDetector(const ParametersMap & parameters)
{
	this->parseParameters(parameters);
}

void KeypointDetector::parseParameters(const ParametersMap & parameters)
{
}

std::vector<cv::KeyPoint> KeypointDetector::generateKeypoints(
		const cv::Mat & image,
		int maxKeypoints,
		const cv::Rect & roi)
{
	ULOGGER_DEBUG("");
	std::vector<cv::KeyPoint> keypoints;
	if(!image.empty())
	{
		UTimer timer;

		// Get keypoints
		keypoints = this->_generateKeypoints(image, roi.width && roi.height?roi:cv::Rect(0,0,image.cols, image.rows));
		ULOGGER_DEBUG("Keypoints extraction time = %f s, keypoints extracted = %d", timer.ticks(), keypoints.size());

		limitKeypoints(keypoints, maxKeypoints);

		if(roi.x || roi.y)
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

cv::Rect KeypointDetector::computeRoi(const cv::Mat & image, const std::vector<float> & roiRatios)
{
	if(!image.empty() && roiRatios.size() == 4)
	{
		float width = image.cols;
		float height = image.rows;
		cv::Rect roi(0, 0, width, height);
		UDEBUG("roi ratios = %f, %f, %f, %f", roiRatios[0],roiRatios[1],roiRatios[2],roiRatios[3]);
		UDEBUG("roi = %d, %d, %d, %d", roi.x, roi.y, roi.width, roi.height);

		//left roi
		if(roiRatios[0] > 0 && roiRatios[0] < 1 - roiRatios[1])
		{
			roi.x = width * roiRatios[0];
		}

		//right roi
		roi.width = width - roi.x;
		if(roiRatios[1] > 0 && roiRatios[1] < 1 - roiRatios[0])
		{
			roi.width -= width * roiRatios[1];
		}

		//top roi
		if(roiRatios[2] > 0 && roiRatios[2] < 1 - roiRatios[3])
		{
			roi.y = height * roiRatios[2];
		}

		//bottom roi
		roi.height = height - roi.y;
		if(roiRatios[3] > 0 && roiRatios[3] < 1 - roiRatios[2])
		{
			roi.height -= height * roiRatios[3];
		}
		UDEBUG("roi = %d, %d, %d, %d", roi.x, roi.y, roi.width, roi.height);

		return roi;
	}
	else
	{
		UERROR("Image is null or _roiRatios(=%d) != 4", roiRatios.size());
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
	Parameters::parse(parameters, Parameters::kSURFExtended(), _extended);
	Parameters::parse(parameters, Parameters::kSURFHessianThreshold(), _hessianThreshold);
	Parameters::parse(parameters, Parameters::kSURFOctaveLayers(), _nOctaveLayers);
	Parameters::parse(parameters, Parameters::kSURFOctaves(), _nOctaves);
	Parameters::parse(parameters, Parameters::kSURFUpright(), _upright);
	Parameters::parse(parameters, Parameters::kSURFGpuVersion(), _gpuVersion);
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
		ULOGGER_DEBUG("");
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
	if(_gpuVersion && cv::gpu::getCudaEnabledDeviceCount())
	{
		cv::gpu::GpuMat imgGpu(imgRoi);
		cv::gpu::GpuMat keypointsGpu;
		cv::gpu::SURF_GPU surfGpu(_hessianThreshold, _nOctaves, _nOctaveLayers, _extended, 0.01f, _upright);
		surfGpu(imgGpu, cv::gpu::GpuMat(), keypointsGpu);
		surfGpu.downloadKeypoints(keypointsGpu, keypoints);
	}
	else
	{
		if(_gpuVersion)
		{
			UWARN("GPU version of SURF not available! Using CPU version instead...");
		}
		ULOGGER_DEBUG("%f %d %d %d %d", _hessianThreshold, _nOctaves, _nOctaveLayers, _extended?1:0, _upright?1:0);
		cv::SURF detector(_hessianThreshold, _nOctaves, _nOctaveLayers, _extended, _upright);
		detector.detect(imgRoi, keypoints);
	}

	ULOGGER_DEBUG("");
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
	Parameters::parse(parameters, Parameters::kSIFTContrastThreshold(), _contrastThreshold);
	Parameters::parse(parameters, Parameters::kSIFTEdgeThreshold(), _edgeThreshold);
	Parameters::parse(parameters, Parameters::kSIFTNFeatures(), _nfeatures);
	Parameters::parse(parameters, Parameters::kSIFTNOctaveLayers(), _nOctaveLayers);
	Parameters::parse(parameters, Parameters::kSIFTSigma(), _sigma);
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
	cv::SIFT detector(_nfeatures, _nOctaveLayers, _contrastThreshold, _edgeThreshold, _sigma);
	detector.detect(imgRoi, keypoints); // Opencv surf keypoints
	return keypoints;
}

}
