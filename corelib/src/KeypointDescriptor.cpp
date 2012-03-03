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

#include "rtabmap/core/KeypointDescriptor.h"
#include "utilite/UStl.h"
#include "utilite/UConversion.h"
#include "utilite/ULogger.h"
#include "utilite/UMath.h"
#include "utilite/ULogger.h"
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/gpu/gpu.hpp>
#include <opencv2/core/version.hpp>

#define OPENCV_SURF_GPU CV_MAJOR_VERSION >= 2 and CV_MINOR_VERSION >=2 and CV_SUBMINOR_VERSION>=1

namespace rtabmap {

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
	KeypointDescriptor(parameters)
{
	_params.hessianThreshold = Parameters::defaultSURFHessianThreshold();
	_params.extended = Parameters::defaultSURFExtended();
	_params.nOctaveLayers = Parameters::defaultSURFOctaveLayers();
	_params.nOctaves = Parameters::defaultSURFOctaves();
	_params.upright = Parameters::defaultSURFUpright();
	_gpuVersion = Parameters::defaultSURFGpuVersion();
	this->parseParameters(parameters);
}

SURFDescriptor::~SURFDescriptor()
{
}

void SURFDescriptor::parseParameters(const ParametersMap & parameters)
{
	ParametersMap::const_iterator iter;
	if((iter=parameters.find(Parameters::kSURFExtended())) != parameters.end())
	{
		_params.extended = uStr2Bool((*iter).second.c_str());
	}
	if((iter=parameters.find(Parameters::kSURFHessianThreshold())) != parameters.end())
	{
		_params.hessianThreshold = std::atof((*iter).second.c_str()); // is it needed for the descriptor?
	}
	if((iter=parameters.find(Parameters::kSURFOctaveLayers())) != parameters.end())
	{
		_params.nOctaveLayers = std::atoi((*iter).second.c_str()); // is it needed for the descriptor?
	}
	if((iter=parameters.find(Parameters::kSURFOctaves())) != parameters.end())
	{
		_params.nOctaves = std::atoi((*iter).second.c_str()); // is it needed for the descriptor?
	}
	if((iter=parameters.find(Parameters::kSURFUpright())) != parameters.end())
	{
		_params.upright = uStr2Bool((*iter).second.c_str());
	}
	if((iter=parameters.find(Parameters::kSURFGpuVersion())) != parameters.end())
	{
		_gpuVersion = uStr2Bool((*iter).second.c_str());
	}
	KeypointDescriptor::parseParameters(parameters);
}

cv::Mat SURFDescriptor::generateDescriptors(const IplImage * image, std::vector<cv::KeyPoint> & keypoints) const
{
	ULOGGER_DEBUG("");
	cv::Mat descriptors;
	if(!image)
	{
		ULOGGER_ERROR("Image is null ?!?");
		return descriptors;
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
#if OPENCV_SURF_GPU
	if(_gpuVersion)
	{
		std::vector<float> d;
		cv::gpu::GpuMat imgGpu(img);
		cv::gpu::GpuMat descriptorsGpu;
		cv::gpu::GpuMat keypointsGpu;
		cv::gpu::SURF_GPU surfGpu(_params.hessianThreshold, _params.nOctaves, _params.nOctaveLayers, _params.extended, 0.01f, _params.upright);
		surfGpu.uploadKeypoints(keypoints, keypointsGpu);
		surfGpu(imgGpu, cv::gpu::GpuMat(), keypointsGpu, descriptorsGpu, true);
		surfGpu.downloadDescriptors(descriptorsGpu, d);
		unsigned int dim = _params.extended?128:64;
		descriptors = cv::Mat(d.size()/dim, dim, CV_32F);
		for(int i=0; i<descriptors.rows; ++i)
		{
			float * rowFl = descriptors.ptr<float>(i);
			memcpy(rowFl, &d[i*dim], dim*sizeof(float));
		}
	}
	else
	{
		cv::SurfDescriptorExtractor extractor(_params.nOctaves, _params.nOctaveLayers, _params.extended, _params.upright);
		extractor.compute(img, keypoints, descriptors);
	}
#else
	cv::SurfDescriptorExtractor extractor(_params.nOctaves, _params.nOctaveLayers, _params.extended, _params.upright);
	extractor.compute(img, keypoints, descriptors);
#endif

	if(imageGrayScale)
	{
		cvReleaseImage(&imageGrayScale);
	}
	return descriptors;
}

//////////////////////////
//SIFTDescriptor
//////////////////////////
SIFTDescriptor::SIFTDescriptor(const ParametersMap & parameters) :
	KeypointDescriptor(parameters)
{
	this->parseParameters(parameters);
}

SIFTDescriptor::~SIFTDescriptor()
{
}

void SIFTDescriptor::parseParameters(const ParametersMap & parameters)
{
	ParametersMap::const_iterator iter;
	KeypointDescriptor::parseParameters(parameters);
}

cv::Mat SIFTDescriptor::generateDescriptors(const IplImage * image, std::vector<cv::KeyPoint> & keypoints) const
{
	ULOGGER_DEBUG("");
	cv::Mat descriptors;
	if(!image)
	{
		ULOGGER_ERROR("Image is null ?!?");
		return descriptors;
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
	cv::SiftDescriptorExtractor extractor(_descriptorParams, _commonParams);
	extractor.compute(img, keypoints, descriptors);
	if(imageGrayScale)
	{
		cvReleaseImage(&imageGrayScale);
	}
	return descriptors;
}

//////////////////////////
//BRIEFDescriptor
//////////////////////////
BRIEFDescriptor::BRIEFDescriptor(const ParametersMap & parameters) :
	KeypointDescriptor(parameters),
	_size(Parameters::defaultBRIEFSize())
{
	this->parseParameters(parameters);
}

BRIEFDescriptor::~BRIEFDescriptor()
{
}

void BRIEFDescriptor::parseParameters(const ParametersMap & parameters)
{
	ParametersMap::const_iterator iter;
	if((iter=parameters.find(Parameters::kBRIEFSize())) != parameters.end())
	{
		_size = std::atoi((*iter).second.c_str());
	}
	KeypointDescriptor::parseParameters(parameters);
}

cv::Mat BRIEFDescriptor::generateDescriptors(const IplImage * image, std::vector<cv::KeyPoint> & keypoints) const
{
	ULOGGER_DEBUG("");
	cv::Mat descriptors;
	if(!image)
	{
		ULOGGER_ERROR("Image is null ?!?");
		return descriptors;
	}
	// BRIEF support only grayscale images ?
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
	cv::BriefDescriptorExtractor brief(_size);
	brief.compute(img, keypoints, descriptors);

	if(imageGrayScale)
	{
		cvReleaseImage(&imageGrayScale);
	}
	return descriptors;
}

//////////////////////////
//ColorDescriptor
//////////////////////////
ColorDescriptor::ColorDescriptor(const ParametersMap & parameters) :
	KeypointDescriptor(parameters)
{
	this->parseParameters(parameters);
}

ColorDescriptor::~ColorDescriptor()
{
}

void ColorDescriptor::parseParameters(const ParametersMap & parameters)
{
	// No parameter...
	KeypointDescriptor::parseParameters(parameters);
}

cv::Mat ColorDescriptor::generateDescriptors(const IplImage * image, std::vector<cv::KeyPoint> & keypoints) const
{
	ULOGGER_DEBUG("");
	cv::Mat descriptors;
	if(!image)
	{
		ULOGGER_ERROR("Image is null ?!?");
		return descriptors;
	}

	IplImage * imageConverted = 0;
	if(image->nChannels != 3 || image->depth != IPL_DEPTH_8U)
	{
		imageConverted = cvCreateImage(cvSize(image->width,image->height), IPL_DEPTH_8U, 3);
		cvCvtColor(image, imageConverted, CV_GRAY2BGR);
	}
	cv::Mat imgMat;
	if(imageConverted)
	{
		imgMat = cv::Mat(imageConverted);
	}
	else
	{
		imgMat =  cv::Mat(image);
	}

	//create descriptors...
	descriptors = cv::Mat(keypoints.size(), 6, CV_32F);
	int i=0;
	for(std::vector<cv::KeyPoint>::const_iterator key=keypoints.begin(); key!=keypoints.end(); ++key)
	{

		int grayMax = -1;  // grayValue
		int grayMin = -1;  // grayValue
		float d[6] = {0};
		std::vector<int> RxV;
		cv::Point center = cv::Point(cvRound(key->pt.x), cvRound(key->pt.y));
		int R = cvRound(key->size*1.2/9.*2);
		this->getCircularROI(R, RxV);
		cv::Mat_<cv::Vec3b>& img = (cv::Mat_<cv::Vec3b>&)imgMat; //3 channel pointer to image
		// find the brighter and darker pixels
		for( int dy = -R; dy <= R; ++dy )
		{
			int Rx = RxV[abs(dy)];
			for( int dx = -Rx; dx <= Rx; ++dx )
			{
				if(center.y+dy < img.rows && center.y+dy >= 0 && center.x+dx < img.cols && center.x+dx >= 0)
				{
					//bgr
					uchar b = img(center.y+dy, center.x+dx)[0];
					uchar g = img(center.y+dy, center.x+dx)[1];
					uchar r = img(center.y+dy, center.x+dx)[2];
					int gray = b*0.114 + g*0.587 + r*0.299;
					if(grayMax<0 || gray > grayMax)
					{
						grayMax = gray;
						d[0] = b;
						d[1] = g;
						d[2] = r;
					}
					if(grayMin<0 || gray < grayMin)
					{
						grayMin = gray;
						d[3] = b;
						d[4] = g;
						d[5] = r;
					}
				}
				else
				{
					//ULOGGER_WARN("The keypoint size is outside of the image ranges (x,y)=(%d,%d) radius=%d", center.y+dy, center.x+dx, R);
				}
			}
		}
		for(int j=0; j<6; ++j)
		{
			descriptors.at<float>(i,j) = d[j] / 255; // Normalize between 0 and 1
		}
		++i;
	}

	if(imageConverted)
	{
		cvReleaseImage(&imageConverted);
	}

	return descriptors;
}

// the function returns x boundary coordinates of
// the circle for each y. RxV[y1] = x1 means that
// when y=y1, -x1 <=x<=x1 is inside the circle
// (from OpenCv doc, C++ Cheatsheet)
void ColorDescriptor::getCircularROI(int R, std::vector<int> & RxV) const
{
    RxV.resize(R+1);
    for( int y = 0; y <= R; y++ )
        RxV[y] = cvRound(sqrt(double(R*R - y*y)));
}

//////////////////////////
//HueDescriptor
//////////////////////////
HueDescriptor::HueDescriptor(const ParametersMap & parameters) :
		ColorDescriptor(parameters)
{
	this->parseParameters(parameters);
}

HueDescriptor::~HueDescriptor()
{
}

void HueDescriptor::parseParameters(const ParametersMap & parameters)
{
	// No parameter...
	KeypointDescriptor::parseParameters(parameters);
}

cv::Mat HueDescriptor::generateDescriptors(const IplImage * image, std::vector<cv::KeyPoint> & keypoints) const
{
	ULOGGER_DEBUG("");
	cv::Mat descriptors;
	if(!image)
	{
		ULOGGER_ERROR("Image is null ?!?");
		return descriptors;
	}

	IplImage * imageConverted = 0;
	if(image->nChannels != 3 || image->depth != IPL_DEPTH_8U)
	{
		imageConverted = cvCreateImage(cvSize(image->width,image->height), IPL_DEPTH_8U, 3);
		cvCvtColor(image, imageConverted, CV_GRAY2BGR);
	}
	cv::Mat imgMat;
	if(imageConverted)
	{
		imgMat = cv::Mat(imageConverted);
	}
	else
	{
		imgMat =  cv::Mat(image);
	}

	//create descriptors...
	descriptors = cv::Mat(keypoints.size(), 2, CV_32F);
	int i=0;
	for(std::vector<cv::KeyPoint>::const_iterator key=keypoints.begin(); key!=keypoints.end(); ++key)
	{

		int intensityMax = -1;
		int intensityMin = -1;
		float d[2] = {0};
		std::vector<int> RxV;
		cv::Point center = cv::Point(cvRound(key->pt.x), cvRound(key->pt.y));
		int R = cvRound(key->size*1.2/9.*2);
		this->getCircularROI(R, RxV);
		cv::Mat_<cv::Vec3b>& img = (cv::Mat_<cv::Vec3b>&)imgMat; //3 channel pointer to image
		// find the brighter and darker pixels using the intensity
		int dxb=0;
		int dyb=0;
		int dxd=0;
		int dyd=0;
		for( int dy = -R; dy <= R; ++dy )
		{
			int Rx = RxV[abs(dy)];
			for( int dx = -Rx; dx <= Rx; ++dx )
			{
				if(center.y+dy < img.rows && center.y+dy >= 0 && center.x+dx < img.cols && center.x+dx >= 0)
				{
					//bgr
					float b = float(img(center.y+dy, center.x+dx)[0]) / 255.0f;
					float g = float(img(center.y+dy, center.x+dx)[1]) / 255.0f;
					float r = float(img(center.y+dy, center.x+dx)[2]) / 255.0f;
					int intensity = rgb2intensity(r, g, b);
					if(intensityMax<0 || intensity > intensityMax)
					{
						intensityMax = intensity;
						dxb = dx;
						dyb = dy;
					}
					if(intensityMin<0 || intensity < intensityMin)
					{
						intensityMin = intensity;
						dxd = dx;
						dyd = dy;
					}
				}
				else
				{
					//ULOGGER_WARN("The keypoint size is outside of the image ranges (x,y)=(%d,%d) radius=%d", center.y+dy, center.x+dx, R);
				}
			}
		}

		// brighter
		float b = float(img(center.y+dyb, center.x+dxb)[0]) / 255.0f;
		float g = float(img(center.y+dyb, center.x+dxb)[1]) / 255.0f;
		float r = float(img(center.y+dyb, center.x+dxb)[2]) / 255.0f;
		d[0] = rgb2hue(r, g, b);

		// darker
		b = float(img(center.y+dyd, center.x+dxd)[0]) / 255.0f;
		g = float(img(center.y+dyd, center.x+dxd)[1]) / 255.0f;
		r = float(img(center.y+dyd, center.x+dxd)[2]) / 255.0f;
		d[1] = rgb2hue(r, g, b);

		float * rowFl = descriptors.ptr<float>(i);
		memcpy(rowFl, &d[i*2], 2*sizeof(float));
		++i;
	}

	if(imageConverted)
	{
		cvReleaseImage(&imageConverted);
	}

	return descriptors;
}

// assuming that rgb values are normalized [0,1]
float HueDescriptor::rgb2hue(float r, float g, float b) const
{
	double pi = 3.14159265359;
	if(b<=g)
	{
		return acos(((r-g)+(r-b))/(2*sqrt((r-g)*(r-g)+(r-b)*(g-b))))/pi;
	}
	else
	{
		return (pi-acos(((r-g)+(r-b))/(2*sqrt((r-g)*(r-g)+(r-b)*(g-b)))))/pi;
	}
}


}
