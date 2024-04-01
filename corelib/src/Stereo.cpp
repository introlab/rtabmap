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

#include <rtabmap/core/Stereo.h>
#include <rtabmap/core/util2d.h>
#include <rtabmap/utilite/ULogger.h>
#include <opencv2/video/tracking.hpp>

namespace rtabmap {

Stereo * Stereo::create(const ParametersMap & parameters)
{
	bool opticalFlow = Parameters::defaultStereoOpticalFlow();
	Parameters::parse(parameters, Parameters::kStereoOpticalFlow(), opticalFlow);
	if(opticalFlow)
	{
		return new StereoOpticalFlow(parameters);
	}
	else
	{
		return new Stereo(parameters);
	}
}

Stereo::Stereo(const ParametersMap & parameters) :
		winWidth_(Parameters::defaultStereoWinWidth()),
		winHeight_(Parameters::defaultStereoWinHeight()),
		iterations_(Parameters::defaultStereoIterations()),
		maxLevel_(Parameters::defaultStereoMaxLevel()),
		minDisparity_(Parameters::defaultStereoMinDisparity()),
		maxDisparity_(Parameters::defaultStereoMaxDisparity()),
		winSSD_(Parameters::defaultStereoSSD())
{
	this->parseParameters(parameters);
}

void Stereo::parseParameters(const ParametersMap & parameters)
{
	Parameters::parse(parameters, Parameters::kStereoWinWidth(), winWidth_);
	Parameters::parse(parameters, Parameters::kStereoWinHeight(), winHeight_);
	Parameters::parse(parameters, Parameters::kStereoIterations(), iterations_);
	Parameters::parse(parameters, Parameters::kStereoMaxLevel(), maxLevel_);
	Parameters::parse(parameters, Parameters::kStereoMinDisparity(), minDisparity_);
	Parameters::parse(parameters, Parameters::kStereoMaxDisparity(), maxDisparity_);
	Parameters::parse(parameters, Parameters::kStereoSSD(), winSSD_);
}

std::vector<cv::Point2f> Stereo::computeCorrespondences(
		const cv::Mat & leftImage,
		const cv::Mat & rightImage,
		const std::vector<cv::Point2f> & leftCorners,
		std::vector<unsigned char> & status) const
{
	std::vector<cv::Point2f> rightCorners;
	UDEBUG("util2d::calcStereoCorrespondences() begin");
	rightCorners = util2d::calcStereoCorrespondences(
					leftImage,
					rightImage,
					leftCorners,
					status,
					cv::Size(winWidth_, winHeight_),
					maxLevel_,
					iterations_,
					minDisparity_,
					maxDisparity_,
					winSSD_);
	UDEBUG("util2d::calcStereoCorrespondences() end");
	return rightCorners;
}

StereoOpticalFlow::StereoOpticalFlow(const ParametersMap & parameters) :
		Stereo(parameters),
		epsilon_(Parameters::defaultStereoEps())
{
	this->parseParameters(parameters);
}

void StereoOpticalFlow::parseParameters(const ParametersMap & parameters)
{
	Stereo::parseParameters(parameters);
	Parameters::parse(parameters, Parameters::kStereoEps(), epsilon_);
}


std::vector<cv::Point2f> StereoOpticalFlow::computeCorrespondences(
		const cv::Mat & leftImage,
		const cv::Mat & rightImage,
		const std::vector<cv::Point2f> & leftCorners,
		std::vector<unsigned char> & status) const
{
	std::vector<cv::Point2f> rightCorners;
	UDEBUG("util2d::calcOpticalFlowPyrLKStereo() begin");
	std::vector<float> err;
	util2d::calcOpticalFlowPyrLKStereo(
			leftImage,
			rightImage,
			leftCorners,
			rightCorners,
			status,
			err,
			this->winSize(),
			this->maxLevel(),
			cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, this->iterations(), epsilon_),
			cv::OPTFLOW_LK_GET_MIN_EIGENVALS, 1e-4);
	UDEBUG("util2d::calcOpticalFlowPyrLKStereo() end");
	UASSERT(leftCorners.size() == rightCorners.size() && status.size() == leftCorners.size());
	int countFlowRejected = 0;
	int countDisparityRejected = 0;
	for(unsigned int i=0; i<status.size(); ++i)
	{
		if(status[i]!=0)
		{
			float disparity = leftCorners[i].x - rightCorners[i].x;
			if(disparity <= this->minDisparity() || disparity > this->maxDisparity())
			{
				status[i] = 0;
				++countDisparityRejected;
			}
		}
		else
		{
			++countFlowRejected;
		}
	}
	UDEBUG("total=%d countFlowRejected=%d countDisparityRejected=%d", (int)status.size(), countFlowRejected, countDisparityRejected);

	return rightCorners;
}

} /* namespace rtabmap */
