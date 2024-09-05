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

#ifdef HAVE_OPENCV_CUDAOPTFLOW
#include <opencv2/cudaoptflow.hpp>
#endif

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

#ifdef HAVE_OPENCV_CUDEV
std::vector<cv::Point2f> Stereo::computeCorrespondences(
		const cv::cuda::GpuMat & leftImage,
		const cv::cuda::GpuMat & rightImage,
		const std::vector<cv::Point2f> & leftCorners,
		std::vector<unsigned char> & status) const
{
	UERROR("GPU support for this approach is not implemented!");
	return std::vector<cv::Point2f>();
}
#endif

StereoOpticalFlow::StereoOpticalFlow(const ParametersMap & parameters) :
		Stereo(parameters),
		epsilon_(Parameters::defaultStereoEps()),
		gpu_(Parameters::defaultStereoGpu())
{
	this->parseParameters(parameters);
}

void StereoOpticalFlow::parseParameters(const ParametersMap & parameters)
{
	Stereo::parseParameters(parameters);
	Parameters::parse(parameters, Parameters::kStereoEps(), epsilon_);
	Parameters::parse(parameters, Parameters::kStereoGpu(), gpu_);
#ifndef HAVE_OPENCV_CUDAOPTFLOW
	if(gpu_)
	{
		UERROR("%s is enabled but RTAB-Map is not built with OpenCV CUDA, disabling it.", Parameters::kStereoGpu().c_str());
		gpu_ = false;
	}
#endif
}

bool StereoOpticalFlow::isGpuEnabled() const
{
#ifdef HAVE_OPENCV_CUDAOPTFLOW
	return gpu_;
#else
	return false;
#endif
}

std::vector<cv::Point2f> StereoOpticalFlow::computeCorrespondences(
		const cv::Mat & leftImage,
		const cv::Mat & rightImage,
		const std::vector<cv::Point2f> & leftCorners,
		std::vector<unsigned char> & status) const
{
	std::vector<cv::Point2f> rightCorners;
	std::vector<float> err;
#ifdef HAVE_OPENCV_CUDAOPTFLOW
	if(gpu_)
	{
		cv::cuda::GpuMat d_leftImage(leftImage);
		cv::cuda::GpuMat d_rightImage(rightImage);
		return computeCorrespondences(d_leftImage, d_rightImage, leftCorners, status);
	}
	else
#endif
	{
		UDEBUG("util2d::calcOpticalFlowPyrLKStereo() begin");
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
	}
	updateStatus(leftCorners, rightCorners, status);
	return rightCorners;
}

#ifdef HAVE_OPENCV_CUDEV
std::vector<cv::Point2f> StereoOpticalFlow::computeCorrespondences(
		const cv::cuda::GpuMat & leftImage,
		const cv::cuda::GpuMat & rightImage,
		const std::vector<cv::Point2f> & leftCorners,
		std::vector<unsigned char> & status) const
{
	std::vector<cv::Point2f> rightCorners;
#ifdef HAVE_OPENCV_CUDAOPTFLOW
	UDEBUG("cv::cuda::SparsePyrLKOpticalFlow transfer host to device begin");
	cv::cuda::GpuMat d_leftImage(leftImage);
	cv::cuda::GpuMat d_rightImage(rightImage);
	cv::cuda::GpuMat d_leftCorners(leftCorners);
	cv::cuda::GpuMat d_rightCorners;
	UDEBUG("cv::cuda::SparsePyrLKOpticalFlow transfer host to device end");
	cv::cuda::GpuMat d_status;
	cv::Ptr<cv::cuda::SparsePyrLKOpticalFlow> d_pyrLK_sparse = cv::cuda::SparsePyrLKOpticalFlow::create(
		this->winSize(), this->maxLevel(), this->iterations());

	UDEBUG("cv::cuda::SparsePyrLKOpticalFlow calc begin");
	d_pyrLK_sparse->calc(d_leftImage, d_rightImage, d_leftCorners, d_rightCorners, d_status);
	UDEBUG("cv::cuda::SparsePyrLKOpticalFlow calc end");

	UDEBUG("cv::cuda::SparsePyrLKOpticalFlow transfer device to host begin");
	// Transfer back data to CPU
	rightCorners = std::vector<cv::Point2f>(d_rightCorners.cols);
	cv::Mat matRightCorners(1, d_rightCorners.cols, CV_32FC2, (void*)&rightCorners[0]);
	d_rightCorners.download(matRightCorners);

	status = std::vector<unsigned char>(d_status.cols);
	cv::Mat matStatus(1, d_status.cols, CV_8UC1, (void*)&status[0]);
	d_status.download(matStatus);
	UDEBUG("cv::cuda::SparsePyrLKOpticalFlow transfer device to host end");

	updateStatus(leftCorners, rightCorners, status);

#else
	UERROR("GPU support for this approach is not implemented!");
#endif
	return rightCorners;
}
#endif

void StereoOpticalFlow::updateStatus(
		const std::vector<cv::Point2f> & leftCorners,
		const std::vector<cv::Point2f> & rightCorners,
		std::vector<unsigned char> & status) const
{
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
}

} /* namespace rtabmap */
