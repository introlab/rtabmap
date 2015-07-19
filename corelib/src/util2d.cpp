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

#include "rtabmap/core/util2d.h"

#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UMath.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

namespace rtabmap
{

namespace util2d
{


cv::Mat disparityFromStereoImages(
		const cv::Mat & leftImage,
		const cv::Mat & rightImage)
{
	UASSERT(!leftImage.empty() && !rightImage.empty() &&
			(leftImage.type() == CV_8UC1 || leftImage.type() == CV_8UC3) && rightImage.type() == CV_8UC1 &&
			leftImage.cols == rightImage.cols &&
			leftImage.rows == rightImage.rows);

	cv::Mat leftMono;
	if(leftImage.channels() == 3)
	{
		cv::cvtColor(leftImage, leftMono, CV_BGR2GRAY);
	}
	else
	{
		leftMono = leftImage;
	}
	cv::Mat disparity;
#if CV_MAJOR_VERSION < 3
	cv::StereoBM stereo(cv::StereoBM::BASIC_PRESET);
	stereo.state->SADWindowSize = 15;
	stereo.state->minDisparity = 0;
	stereo.state->numberOfDisparities = 64;
	stereo.state->preFilterSize = 9;
	stereo.state->preFilterCap = 31;
	stereo.state->uniquenessRatio = 15;
	stereo.state->textureThreshold = 10;
	stereo.state->speckleWindowSize = 100;
	stereo.state->speckleRange = 4;
	stereo(leftMono, rightImage, disparity, CV_16SC1);
#else
	cv::Ptr<cv::StereoBM> stereo = cv::StereoBM::create();
	stereo->setBlockSize(15);
	stereo->setMinDisparity(0);
	stereo->setNumDisparities(64);
	stereo->setPreFilterSize(9);
	stereo->setPreFilterCap(31);
	stereo->setUniquenessRatio(15);
	stereo->setTextureThreshold(10);
	stereo->setSpeckleWindowSize(100);
	stereo->setSpeckleRange(4);
	stereo->compute(leftMono, rightImage, disparity);
#endif
	return disparity;
}

cv::Mat disparityFromStereoImages(
		const cv::Mat & leftImage,
		const cv::Mat & rightImage,
		const std::vector<cv::Point2f> & leftCorners,
		int flowWinSize,
		int flowMaxLevel,
		int flowIterations,
		double flowEps,
		float maxCorrespondencesSlope)
{
	UASSERT(!leftImage.empty() && !rightImage.empty() &&
			leftImage.type() == CV_8UC1 && rightImage.type() == CV_8UC1 &&
			leftImage.cols == rightImage.cols &&
			leftImage.rows == rightImage.rows);

	// Find features in the new left image
	std::vector<unsigned char> status;
	std::vector<float> err;
	std::vector<cv::Point2f> rightCorners;
	UDEBUG("cv::calcOpticalFlowPyrLK() begin");
	cv::calcOpticalFlowPyrLK(
			leftImage,
			rightImage,
			leftCorners,
			rightCorners,
			status,
			err,
			cv::Size(flowWinSize, flowWinSize), flowMaxLevel,
			cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, flowIterations, flowEps),
			cv::OPTFLOW_LK_GET_MIN_EIGENVALS, 1e-4);
	UDEBUG("cv::calcOpticalFlowPyrLK() end");

	return disparityFromStereoCorrespondences(leftImage, leftCorners, rightCorners, status, maxCorrespondencesSlope);
}

cv::Mat depthFromStereoImages(
		const cv::Mat & leftImage,
		const cv::Mat & rightImage,
		const std::vector<cv::Point2f> & leftCorners,
		float fx,
		float baseline,
		int flowWinSize,
		int flowMaxLevel,
		int flowIterations,
		double flowEps)
{
	UASSERT(!leftImage.empty() && !rightImage.empty() &&
			leftImage.type() == CV_8UC1 && rightImage.type() == CV_8UC1 &&
			leftImage.cols == rightImage.cols &&
			leftImage.rows == rightImage.rows);
	UASSERT(fx > 0.0f && baseline > 0.0f);

	// Find features in the new left image
	std::vector<unsigned char> status;
	std::vector<float> err;
	std::vector<cv::Point2f> rightCorners;
	UDEBUG("cv::calcOpticalFlowPyrLK() begin");
	cv::calcOpticalFlowPyrLK(
			leftImage,
			rightImage,
			leftCorners,
			rightCorners,
			status,
			err,
			cv::Size(flowWinSize, flowWinSize), flowMaxLevel,
			cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, flowIterations, flowEps),
			cv::OPTFLOW_LK_GET_MIN_EIGENVALS, 1e-4);
	UDEBUG("cv::calcOpticalFlowPyrLK() end");

	return depthFromStereoCorrespondences(leftImage, leftCorners, rightCorners, status, fx, baseline);
}

cv::Mat disparityFromStereoCorrespondences(
		const cv::Mat & leftImage,
		const std::vector<cv::Point2f> & leftCorners,
		const std::vector<cv::Point2f> & rightCorners,
		const std::vector<unsigned char> & mask,
		float maxSlope)
{
	UASSERT(!leftImage.empty() && leftCorners.size() == rightCorners.size());
	UASSERT(mask.size() == 0 || mask.size() == leftCorners.size());
	cv::Mat disparity = cv::Mat::zeros(leftImage.rows, leftImage.cols, CV_32FC1);
	for(unsigned int i=0; i<leftCorners.size(); ++i)
	{
		if(mask.size() == 0 || mask[i])
		{
			float d = leftCorners[i].x - rightCorners[i].x;
			float slope = fabs((leftCorners[i].y - rightCorners[i].y) / (leftCorners[i].x - rightCorners[i].x));
			if(d > 0.0f && (maxSlope <= 0 || fabs(leftCorners[i].y-rightCorners[i].y) <= 1.0f || slope <= maxSlope))
			{
				disparity.at<float>(int(leftCorners[i].y+0.5f), int(leftCorners[i].x+0.5f)) = d;
			}
		}
	}
	return disparity;
}

cv::Mat depthFromStereoCorrespondences(
		const cv::Mat & leftImage,
		const std::vector<cv::Point2f> & leftCorners,
		const std::vector<cv::Point2f> & rightCorners,
		const std::vector<unsigned char> & mask,
		float fx, float baseline)
{
	UASSERT(!leftImage.empty() && leftCorners.size() == rightCorners.size());
	UASSERT(mask.size() == 0 || mask.size() == leftCorners.size());
	cv::Mat depth = cv::Mat::zeros(leftImage.rows, leftImage.cols, CV_32FC1);
	for(unsigned int i=0; i<leftCorners.size(); ++i)
	{
		if(mask.size() == 0 || mask[i])
		{
			float disparity = leftCorners[i].x - rightCorners[i].x;
			if(disparity > 0.0f)
			{
				float d = baseline * fx / disparity;
				depth.at<float>(int(leftCorners[i].y+0.5f), int(leftCorners[i].x+0.5f)) = d;
			}
		}
	}
	return depth;
}

float getDepth(
		const cv::Mat & depthImage,
		float x, float y,
		bool smoothing,
		float maxZError)
{
	UASSERT(!depthImage.empty());
	UASSERT(depthImage.type() == CV_16UC1 || depthImage.type() == CV_32FC1);

	int u = int(x+0.5f);
	int v = int(y+0.5f);

	if(!(u >=0 && u<depthImage.cols && v >=0 && v<depthImage.rows))
	{
		UDEBUG("!(x >=0 && x<depthImage.cols && y >=0 && y<depthImage.rows) cond failed! returning bad point. (x=%f (u=%d), y=%f (v=%d), cols=%d, rows=%d)",
				x,u,y,v,depthImage.cols, depthImage.rows);
		return 0;
	}

	bool isInMM = depthImage.type() == CV_16UC1; // is in mm?

	// Inspired from RGBDFrame::getGaussianMixtureDistribution() method from
	// https://github.com/ccny-ros-pkg/rgbdtools/blob/master/src/rgbd_frame.cpp
	// Window weights:
	//  | 1 | 2 | 1 |
	//  | 2 | 4 | 2 |
	//  | 1 | 2 | 1 |
	int u_start = std::max(u-1, 0);
	int v_start = std::max(v-1, 0);
	int u_end = std::min(u+1, depthImage.cols-1);
	int v_end = std::min(v+1, depthImage.rows-1);

	float depth = isInMM?(float)depthImage.at<unsigned short>(v,u)*0.001f:depthImage.at<float>(v,u);
	if(depth!=0.0f && uIsFinite(depth))
	{
		if(smoothing)
		{
			float sumWeights = 0.0f;
			float sumDepths = 0.0f;
			for(int uu = u_start; uu <= u_end; ++uu)
			{
				for(int vv = v_start; vv <= v_end; ++vv)
				{
					if(!(uu == u && vv == v))
					{
						float d = isInMM?(float)depthImage.at<unsigned short>(vv,uu)*0.001f:depthImage.at<float>(vv,uu);
						// ignore if not valid or depth difference is too high
						if(d != 0.0f && uIsFinite(d) && fabs(d - depth) < maxZError)
						{
							if(uu == u || vv == v)
							{
								sumWeights+=2.0f;
								d*=2.0f;
							}
							else
							{
								sumWeights+=1.0f;
							}
							sumDepths += d;
						}
					}
				}
			}
			// set window weight to center point
			depth *= 4.0f;
			sumWeights += 4.0f;

			// mean
			depth = (depth+sumDepths)/sumWeights;
		}
	}
	else
	{
		depth = 0;
	}
	return depth;
}

cv::Mat decimate(const cv::Mat & image, int decimation)
{
	UASSERT(decimation >= 1);
	cv::Mat out;
	if(!image.empty())
	{
		if(decimation > 1)
		{
			if((image.type() == CV_32FC1 || image.type()==CV_16UC1))
			{
				UASSERT_MSG(image.rows % decimation == 0 && image.cols % decimation == 0, "Decimation of depth images should be exact!");

				out = cv::Mat(image.rows/decimation, image.cols/decimation, image.type());
				if(image.type() == CV_32FC1)
				{
					for(int j=0; j<out.rows; ++j)
					{
						for(int i=0; i<out.cols; ++i)
						{
							out.at<float>(j, i) = image.at<float>(j*decimation, i*decimation);
						}
					}
				}
				else // CV_16UC1
				{
					for(int j=0; j<out.rows; ++j)
					{
						for(int i=0; i<out.cols; ++i)
						{
							out.at<unsigned short>(j, i) = image.at<unsigned short>(j*decimation, i*decimation);
						}
					}
				}
			}
			else
			{
				cv::resize(image, out, cv::Size(), 1.0f/float(decimation), 1.0f/float(decimation), cv::INTER_AREA);
			}
		}
		else
		{
			out = image;
		}
	}
	return out;
}

// Registration Depth to RGB
cv::Mat registerDepth(
		const cv::Mat & depth,
		const cv::Mat & depthK,
		const cv::Mat & colorK,
		const rtabmap::Transform & transform)
{
	UASSERT(!transform.isNull());
	UASSERT(!depth.empty());
	UASSERT(depth.type() == CV_16UC1); // mm
	UASSERT(depthK.type() == CV_64FC1 && depthK.cols == 3 && depthK.cols == 3);
	UASSERT(colorK.type() == CV_64FC1 && colorK.cols == 3 && colorK.cols == 3);

	float fx = depthK.at<double>(0,0);
	float fy = depthK.at<double>(1,1);
	float cx = depthK.at<double>(0,2);
	float cy = depthK.at<double>(1,2);

	float rfx = colorK.at<double>(0,0);
	float rfy = colorK.at<double>(1,1);
	float rcx = colorK.at<double>(0,2);
	float rcy = colorK.at<double>(1,2);

	Eigen::Affine3f proj = transform.toEigen3f();
	Eigen::Vector4f P4,P3;
	P4[3] = 1;
	cv::Mat registered = cv::Mat::zeros(depth.rows, depth.cols, depth.type());

	for(int y=0; y<depth.rows; ++y)
	{
		for(int x=0; x<depth.cols; ++x)
		{
			//filtering
			float dz = float(depth.at<unsigned short>(y,x))*0.001f; // put in meter for projection
			if(dz>=0.0f)
			{
				// Project to 3D
				P4[0] = (x - cx) * dz / fx; // Optimization: we could have (x-cx)/fx in a lookup table
				P4[1] = (y - cy) * dz / fy; // Optimization: we could have (y-cy)/fy in a lookup table
				P4[2] = dz;

				P3 = proj * P4;
				float z = P3[2];
				float invZ = 1.0f/z;
				int dx = (rfx*P3[0])*invZ + rcx;
				int dy = (rfy*P3[1])*invZ + rcy;

				if(uIsInBounds(dx, 0, registered.cols) && uIsInBounds(dy, 0, registered.rows))
				{
					unsigned short z16 = z * 1000; //mm
					unsigned short &zReg = registered.at<unsigned short>(dy, dx);
					if(zReg == 0 || z16 < zReg)
					{
						zReg = z16;
					}
				}
			}
		}
	}
	return registered;
}

void fillRegisteredDepthHoles(cv::Mat & registeredDepth, bool vertical, bool horizontal, bool fillDoubleHoles)
{
	UASSERT(registeredDepth.type() == CV_16UC1);
	int margin = fillDoubleHoles?2:1;
	for(int x=1; x<registeredDepth.cols-margin; ++x)
	{
		for(int y=1; y<registeredDepth.rows-margin; ++y)
		{
			unsigned short & b = registeredDepth.at<unsigned short>(y, x);
			bool set = false;
			if(vertical)
			{
				const unsigned short & a = registeredDepth.at<unsigned short>(y-1, x);
				unsigned short & c = registeredDepth.at<unsigned short>(y+1, x);
				if(a && c)
				{
					unsigned short error = 0.01*((a+c)/2);
					if(((b == 0 && a && c) || (b > a+error && b > c+error)) &&
						(a>c?a-c<=error:c-a<=error))
					{
						b = (a+c)/2;
						set = true;
						if(!horizontal)
						{
							++y;
						}
					}
				}
				if(!set && fillDoubleHoles)
				{
					const unsigned short & d = registeredDepth.at<unsigned short>(y+2, x);
					if(a && d && (b==0 || c==0))
					{
						unsigned short error = 0.01*((a+d)/2);
						if(((b == 0 && a && d) || (b > a+error && b > d+error)) &&
						   ((c == 0 && a && d) || (c > a+error && c > d+error)) &&
							(a>d?a-d<=error:d-a<=error))
						{
							if(a>d)
							{
								unsigned short tmp = (a-d)/4;
								b = d + tmp;
								c = d + 3*tmp;
							}
							else
							{
								unsigned short tmp = (d-a)/4;
								b = a + tmp;
								c = a + 3*tmp;
							}
							set = true;
							if(!horizontal)
							{
								y+=2;
							}
						}
					}
				}
			}
			if(!set && horizontal)
			{
				const unsigned short & a = registeredDepth.at<unsigned short>(y, x-1);
				unsigned short & c = registeredDepth.at<unsigned short>(y, x+1);
				if(a && c)
				{
					unsigned short error = 0.01*((a+c)/2);
					if(((b == 0 && a && c) || (b > a+error && b > c+error)) &&
						(a>c?a-c<=error:c-a<=error))
					{
						b = (a+c)/2;
						set = true;
					}
				}
				if(!set && fillDoubleHoles)
				{
					const unsigned short & d = registeredDepth.at<unsigned short>(y, x+2);
					if(a && d && (b==0 || c==0))
					{
						unsigned short error = 0.01*((a+d)/2);
						if(((b == 0 && a && d) || (b > a+error && b > d+error)) &&
						   ((c == 0 && a && d) || (c > a+error && c > d+error)) &&
							(a>d?a-d<=error:d-a<=error))
						{
							if(a>d)
							{
								unsigned short tmp = (a-d)/4;
								b = d + tmp;
								c = d + 3*tmp;
							}
							else
							{
								unsigned short tmp = (d-a)/4;
								b = a + tmp;
								c = a + 3*tmp;
							}
						}
					}
				}
			}
		}
	}
}

}

}
