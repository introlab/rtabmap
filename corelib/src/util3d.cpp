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

#include <rtabmap/core/util3d.h>
#include <rtabmap/core/util3d_transforms.h>
#include <rtabmap/core/util3d_filtering.h>
#include <rtabmap/core/util3d_surface.h>
#include <rtabmap/core/util2d.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UMath.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UFile.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/transforms.h>
#include <opencv2/imgproc/imgproc.hpp>

namespace rtabmap
{

namespace util3d
{

cv::Mat bgrFromCloud(const pcl::PointCloud<pcl::PointXYZRGBA> & cloud, bool bgrOrder)
{
	cv::Mat frameBGR = cv::Mat(cloud.height,cloud.width,CV_8UC3);

	for(unsigned int h = 0; h < cloud.height; h++)
	{
		for(unsigned int w = 0; w < cloud.width; w++)
		{
			if(bgrOrder)
			{
				frameBGR.at<cv::Vec3b>(h,w)[0] = cloud.at(h*cloud.width + w).b;
				frameBGR.at<cv::Vec3b>(h,w)[1] = cloud.at(h*cloud.width + w).g;
				frameBGR.at<cv::Vec3b>(h,w)[2] = cloud.at(h*cloud.width + w).r;
			}
			else
			{
				frameBGR.at<cv::Vec3b>(h,w)[0] = cloud.at(h*cloud.width + w).r;
				frameBGR.at<cv::Vec3b>(h,w)[1] = cloud.at(h*cloud.width + w).g;
				frameBGR.at<cv::Vec3b>(h,w)[2] = cloud.at(h*cloud.width + w).b;
			}
		}
	}
	return frameBGR;
}

// return float image in meter
cv::Mat depthFromCloud(
		const pcl::PointCloud<pcl::PointXYZRGBA> & cloud,
		float & fx,
		float & fy,
		bool depth16U)
{
	cv::Mat frameDepth = cv::Mat(cloud.height,cloud.width,depth16U?CV_16UC1:CV_32FC1);
	fx = 0.0f; // needed to reconstruct the cloud
	fy = 0.0f; // needed to reconstruct the cloud
	for(unsigned int h = 0; h < cloud.height; h++)
	{
		for(unsigned int w = 0; w < cloud.width; w++)
		{
			float depth = cloud.at(h*cloud.width + w).z;
			if(depth16U)
			{
				depth *= 1000.0f;
				unsigned short depthMM = 0;
				if(depth <= (float)USHRT_MAX)
				{
					depthMM = (unsigned short)depth;
				}
				frameDepth.at<unsigned short>(h,w) = depthMM;
			}
			else
			{
				frameDepth.at<float>(h,w) = depth;
			}

			// update constants
			if(fx == 0.0f &&
			   uIsFinite(cloud.at(h*cloud.width + w).x) &&
			   uIsFinite(depth) &&
			   w != cloud.width/2 &&
			   depth > 0)
			{
				fx = cloud.at(h*cloud.width + w).x / ((float(w) - float(cloud.width)/2.0f) * depth);
				if(depth16U)
				{
					fx*=1000.0f;
				}
			}
			if(fy == 0.0f &&
			   uIsFinite(cloud.at(h*cloud.width + w).y) &&
			   uIsFinite(depth) &&
			   h != cloud.height/2 &&
			   depth > 0)
			{
				fy = cloud.at(h*cloud.width + w).y / ((float(h) - float(cloud.height)/2.0f) * depth);
				if(depth16U)
				{
					fy*=1000.0f;
				}
			}
		}
	}
	return frameDepth;
}

// return (unsigned short 16bits image in mm) (float 32bits image in m)
void rgbdFromCloud(const pcl::PointCloud<pcl::PointXYZRGBA> & cloud,
		cv::Mat & frameBGR,
		cv::Mat & frameDepth,
		float & fx,
		float & fy,
		bool bgrOrder,
		bool depth16U)
{
	frameDepth = cv::Mat(cloud.height,cloud.width,depth16U?CV_16UC1:CV_32FC1);
	frameBGR = cv::Mat(cloud.height,cloud.width,CV_8UC3);

	fx = 0.0f; // needed to reconstruct the cloud
	fy = 0.0f; // needed to reconstruct the cloud
	for(unsigned int h = 0; h < cloud.height; h++)
	{
		for(unsigned int w = 0; w < cloud.width; w++)
		{
			//rgb
			if(bgrOrder)
			{
				frameBGR.at<cv::Vec3b>(h,w)[0] = cloud.at(h*cloud.width + w).b;
				frameBGR.at<cv::Vec3b>(h,w)[1] = cloud.at(h*cloud.width + w).g;
				frameBGR.at<cv::Vec3b>(h,w)[2] = cloud.at(h*cloud.width + w).r;
			}
			else
			{
				frameBGR.at<cv::Vec3b>(h,w)[0] = cloud.at(h*cloud.width + w).r;
				frameBGR.at<cv::Vec3b>(h,w)[1] = cloud.at(h*cloud.width + w).g;
				frameBGR.at<cv::Vec3b>(h,w)[2] = cloud.at(h*cloud.width + w).b;
			}

			//depth
			float depth = cloud.at(h*cloud.width + w).z;
			if(depth16U)
			{
				depth *= 1000.0f;
				unsigned short depthMM = 0;
				if(depth <= (float)USHRT_MAX)
				{
					depthMM = (unsigned short)depth;
				}
				frameDepth.at<unsigned short>(h,w) = depthMM;
			}
			else
			{
				frameDepth.at<float>(h,w) = depth;
			}

			// update constants
			if(fx == 0.0f &&
			   uIsFinite(cloud.at(h*cloud.width + w).x) &&
			   uIsFinite(depth) &&
			   w != cloud.width/2 &&
			   depth > 0)
			{
				fx = 1.0f/(cloud.at(h*cloud.width + w).x / ((float(w) - float(cloud.width)/2.0f) * depth));
				if(depth16U)
				{
					fx/=1000.0f;
				}
			}
			if(fy == 0.0f &&
			   uIsFinite(cloud.at(h*cloud.width + w).y) &&
			   uIsFinite(depth) &&
			   h != cloud.height/2 &&
			   depth > 0)
			{
				fy = 1.0f/(cloud.at(h*cloud.width + w).y / ((float(h) - float(cloud.height)/2.0f) * depth));
				if(depth16U)
				{
					fy/=1000.0f;
				}
			}
		}
	}
}

pcl::PointXYZ projectDepthTo3D(
		const cv::Mat & depthImage,
		float x, float y,
		float cx, float cy,
		float fx, float fy,
		bool smoothing,
		float maxZError)
{
	UASSERT(depthImage.type() == CV_16UC1 || depthImage.type() == CV_32FC1);

	pcl::PointXYZ pt;

	float depth = util2d::getDepth(depthImage, x, y, smoothing, maxZError);
	if(depth > 0.0f)
	{
		// Use correct principal point from calibration
		cx = cx > 0.0f ? cx : float(depthImage.cols/2) - 0.5f; //cameraInfo.K.at(2)
		cy = cy > 0.0f ? cy : float(depthImage.rows/2) - 0.5f; //cameraInfo.K.at(5)

		// Fill in XYZ
		pt.x = (x - cx) * depth / fx;
		pt.y = (y - cy) * depth / fy;
		pt.z = depth;
	}
	else
	{
		pt.x = pt.y = pt.z = std::numeric_limits<float>::quiet_NaN();
	}
	return pt;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFromDepth(
		const cv::Mat & imageDepth,
		float cx, float cy,
		float fx, float fy,
		int decimation,
		float maxDepth,
		float minDepth,
		std::vector<int> * validIndices)
{
	CameraModel model(fx, fy, cx, cy);
	return cloudFromDepth(imageDepth, model, decimation, maxDepth, minDepth, validIndices);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFromDepth(
		const cv::Mat & imageDepth,
		const CameraModel & model,
		int decimation,
		float maxDepth,
		float minDepth,
		std::vector<int> * validIndices)
{
	float rgbToDepthFactorX = 1.0f;
	float rgbToDepthFactorY = 1.0f;

	UASSERT(model.isValidForProjection());
	UASSERT(!imageDepth.empty() && (imageDepth.type() == CV_16UC1 || imageDepth.type() == CV_32FC1));

	int imageRows = imageDepth.rows;
	int imageCols = imageDepth.cols;

	if(model.imageHeight()>0 && model.imageWidth()>0)
	{
		UASSERT(model.imageHeight() % imageDepth.rows == 0 && model.imageWidth() % imageDepth.cols == 0);
		UASSERT_MSG(model.imageHeight() % decimation == 0, uFormat("model.imageHeight()=%d decimation=%d", model.imageHeight(), decimation).c_str());
		UASSERT_MSG(model.imageWidth() % decimation == 0, uFormat("model.imageWidth()=%d decimation=%d", model.imageWidth(), decimation).c_str());
		rgbToDepthFactorX = 1.0f/float((model.imageWidth() / imageDepth.cols));
		rgbToDepthFactorY = 1.0f/float((model.imageHeight() / imageDepth.rows));
		imageRows = model.imageHeight();
		imageCols = model.imageWidth();
	}
	else
	{
		UASSERT_MSG(imageDepth.rows % decimation == 0, uFormat("rows=%d decimation=%d", imageDepth.rows, decimation).c_str());
		UASSERT_MSG(imageDepth.cols % decimation == 0, uFormat("cols=%d decimation=%d", imageDepth.cols, decimation).c_str());
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if(decimation < 1)
	{
		return cloud;
	}

	//cloud.header = cameraInfo.header;
	cloud->height = imageRows/decimation;
	cloud->width  = imageCols/decimation;
	cloud->is_dense = false;
	cloud->resize(cloud->height * cloud->width);
	if(validIndices)
	{
		validIndices->resize(cloud->size());
	}

	float depthFx = model.fx() * rgbToDepthFactorX;
	float depthFy = model.fy() * rgbToDepthFactorY;
	float depthCx = model.cx() * rgbToDepthFactorX;
	float depthCy = model.cy() * rgbToDepthFactorY;

	UDEBUG("rgb=%dx%d depth=%dx%d fx=%f fy=%f cx=%f cy=%f (depth factors=%f %f) decimation=%d",
			imageCols, imageRows,
			imageDepth.cols, imageDepth.rows,
			model.fx(), model.fy(), model.cx(), model.cy(),
			rgbToDepthFactorX,
			rgbToDepthFactorY,
			decimation);

	int oi = 0;
	for(int h = 0; h < imageRows && h/decimation < (int)cloud->height; h+=decimation)
	{
		for(int w = 0; w < imageCols && w/decimation < (int)cloud->width; w+=decimation)
		{
			pcl::PointXYZ & pt = cloud->at((h/decimation)*cloud->width + (w/decimation));

			pcl::PointXYZ ptXYZ = projectDepthTo3D(imageDepth, w*rgbToDepthFactorX, h*rgbToDepthFactorY, depthCx, depthCy, depthFx, depthFy, false);
			if(pcl::isFinite(ptXYZ) && ptXYZ.z>=minDepth && (maxDepth<=0.0f || ptXYZ.z <= maxDepth))
			{
				pt.x = ptXYZ.x;
				pt.y = ptXYZ.y;
				pt.z = ptXYZ.z;
				if(validIndices)
				{
					validIndices->at(oi++) = (h/decimation)*cloud->width + (w/decimation);
				}
			}
			else
			{
				pt.x = pt.y = pt.z = std::numeric_limits<float>::quiet_NaN();
			}
		}
	}

	if(validIndices)
	{
		validIndices->resize(oi);
	}

	return cloud;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudFromDepthRGB(
		const cv::Mat & imageRgb,
		const cv::Mat & imageDepth,
		float cx, float cy,
		float fx, float fy,
		int decimation,
		float maxDepth,
		float minDepth,
		std::vector<int> * validIndices)
{
	CameraModel model(fx, fy, cx, cy);
	return cloudFromDepthRGB(imageRgb, imageDepth, model, decimation, maxDepth, minDepth, validIndices);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudFromDepthRGB(
		const cv::Mat & imageRgb,
		const cv::Mat & imageDepth,
		const CameraModel & model,
		int decimation,
		float maxDepth,
		float minDepth,
		std::vector<int> * validIndices)
{
	UDEBUG("");
	UASSERT(model.isValidForProjection());
	UASSERT((model.imageHeight() == 0 && model.imageWidth() == 0) || (model.imageHeight() == imageRgb.rows && model.imageWidth() == imageRgb.cols));
	UASSERT(imageRgb.rows % imageDepth.rows == 0 && imageRgb.cols % imageDepth.cols == 0);
	UASSERT(!imageDepth.empty() && (imageDepth.type() == CV_16UC1 || imageDepth.type() == CV_32FC1));
	UASSERT_MSG(imageRgb.rows % decimation == 0, uFormat("imageDepth.rows=%d decimation=%d", imageRgb.rows, decimation).c_str());
	UASSERT_MSG(imageRgb.cols % decimation == 0, uFormat("imageDepth.cols=%d decimation=%d", imageRgb.cols, decimation).c_str());

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	if(decimation < 1)
	{
		return cloud;
	}

	bool mono;
	if(imageRgb.channels() == 3) // BGR
	{
		mono = false;
	}
	else if(imageRgb.channels() == 1) // Mono
	{
		mono = true;
	}
	else
	{
		return cloud;
	}

	//cloud.header = cameraInfo.header;
	cloud->height = imageRgb.rows/decimation;
	cloud->width  = imageRgb.cols/decimation;
	cloud->is_dense = false;
	cloud->resize(cloud->height * cloud->width);
	if(validIndices)
	{
		validIndices->resize(cloud->size());
	}

	float rgbToDepthFactorX = 1.0f/float((imageRgb.cols / imageDepth.cols));
	float rgbToDepthFactorY = 1.0f/float((imageRgb.rows / imageDepth.rows));
	float depthFx = model.fx() * rgbToDepthFactorX;
	float depthFy = model.fy() * rgbToDepthFactorY;
	float depthCx = model.cx() * rgbToDepthFactorX;
	float depthCy = model.cy() * rgbToDepthFactorY;

	UDEBUG("rgb=%dx%d depth=%dx%d fx=%f fy=%f cx=%f cy=%f (depth factors=%f %f) decimation=%d",
			imageRgb.cols, imageRgb.rows,
			imageDepth.cols, imageDepth.rows,
			model.fx(), model.fy(), model.cx(), model.cy(),
			rgbToDepthFactorX,
			rgbToDepthFactorY,
			decimation);

	int oi = 0;
	for(int h = 0; h < imageRgb.rows && h/decimation < (int)cloud->height; h+=decimation)
	{
		for(int w = 0; w < imageRgb.cols && w/decimation < (int)cloud->width; w+=decimation)
		{
			pcl::PointXYZRGB & pt = cloud->at((h/decimation)*cloud->width + (w/decimation));
			if(!mono)
			{
				pt.b = imageRgb.at<cv::Vec3b>(h,w)[0];
				pt.g = imageRgb.at<cv::Vec3b>(h,w)[1];
				pt.r = imageRgb.at<cv::Vec3b>(h,w)[2];
			}
			else
			{
				unsigned char v = imageRgb.at<unsigned char>(h,w);
				pt.b = v;
				pt.g = v;
				pt.r = v;
			}

			pcl::PointXYZ ptXYZ = projectDepthTo3D(imageDepth, w*rgbToDepthFactorX, h*rgbToDepthFactorY, depthCx, depthCy, depthFx, depthFy, false);
			if(pcl::isFinite(ptXYZ) && ptXYZ.z>=minDepth && (maxDepth<=0.0f || ptXYZ.z <= maxDepth))
			{
				pt.x = ptXYZ.x;
				pt.y = ptXYZ.y;
				pt.z = ptXYZ.z;
				if(validIndices)
				{
					validIndices->at(oi) = (h/decimation)*cloud->width + (w/decimation);
				}
				++oi;
			}
			else
			{
				pt.x = pt.y = pt.z = std::numeric_limits<float>::quiet_NaN();
			}
		}
	}
	if(validIndices)
	{
		validIndices->resize(oi);
	}
	if(oi == 0)
	{
		UWARN("Cloud with only NaN values created!");
	}
	UDEBUG("");
	return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFromDisparity(
		const cv::Mat & imageDisparity,
		const StereoCameraModel & model,
		int decimation,
		float maxDepth,
		float minDepth,
		std::vector<int> * validIndices)
{
	UASSERT(imageDisparity.type() == CV_32FC1 || imageDisparity.type()==CV_16SC1);
	UASSERT(imageDisparity.rows % decimation == 0);
	UASSERT(imageDisparity.cols % decimation == 0);
	UASSERT(decimation >= 1);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	//cloud.header = cameraInfo.header;
	cloud->height = imageDisparity.rows/decimation;
	cloud->width  = imageDisparity.cols/decimation;
	cloud->is_dense = false;
	cloud->resize(cloud->height * cloud->width);
	if(validIndices)
	{
		validIndices->resize(cloud->size());
	}

	int oi = 0;
	if(imageDisparity.type()==CV_16SC1)
	{
		for(int h = 0; h < imageDisparity.rows && h/decimation < (int)cloud->height; h+=decimation)
		{
			for(int w = 0; w < imageDisparity.cols && w/decimation < (int)cloud->width; w+=decimation)
			{
				float disp = float(imageDisparity.at<short>(h,w))/16.0f;
				cv::Point3f pt = projectDisparityTo3D(cv::Point2f(w, h), disp, model);
				if(pt.z >= minDepth && (maxDepth <= 0.0f || pt.z <= maxDepth))
				{
					cloud->at((h/decimation)*cloud->width + (w/decimation)) = pcl::PointXYZ(pt.x, pt.y, pt.z);
					if(validIndices)
					{
						validIndices->at(oi++) = (h/decimation)*cloud->width + (w/decimation);
					}
				}
				else
				{
					cloud->at((h/decimation)*cloud->width + (w/decimation)) = pcl::PointXYZ(
							std::numeric_limits<float>::quiet_NaN(),
							std::numeric_limits<float>::quiet_NaN(),
							std::numeric_limits<float>::quiet_NaN());
				}
			}
		}
	}
	else
	{
		for(int h = 0; h < imageDisparity.rows && h/decimation < (int)cloud->height; h+=decimation)
		{
			for(int w = 0; w < imageDisparity.cols && w/decimation < (int)cloud->width; w+=decimation)
			{
				float disp = imageDisparity.at<float>(h,w);
				cv::Point3f pt = projectDisparityTo3D(cv::Point2f(w, h), disp, model);
				if(pt.z > minDepth && (maxDepth <= 0.0f || pt.z <= maxDepth))
				{
					cloud->at((h/decimation)*cloud->width + (w/decimation)) = pcl::PointXYZ(pt.x, pt.y, pt.z);
					if(validIndices)
					{
						validIndices->at(oi++) = (h/decimation)*cloud->width + (w/decimation);
					}
				}
				else
				{
					cloud->at((h/decimation)*cloud->width + (w/decimation)) = pcl::PointXYZ(
							std::numeric_limits<float>::quiet_NaN(),
							std::numeric_limits<float>::quiet_NaN(),
							std::numeric_limits<float>::quiet_NaN());
				}
			}
		}
	}
	if(validIndices)
	{
		validIndices->resize(oi);
	}
	return cloud;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudFromDisparityRGB(
		const cv::Mat & imageRgb,
		const cv::Mat & imageDisparity,
		const StereoCameraModel & model,
		int decimation,
		float maxDepth,
		float minDepth,
		std::vector<int> * validIndices)
{
	UASSERT(!imageRgb.empty() && !imageDisparity.empty());
	UASSERT(imageRgb.rows == imageDisparity.rows &&
			imageRgb.cols == imageDisparity.cols &&
			(imageDisparity.type() == CV_32FC1 || imageDisparity.type()==CV_16SC1));
	UASSERT(imageRgb.channels() == 3 || imageRgb.channels() == 1);
	UASSERT(decimation >= 1);
	UASSERT(imageDisparity.rows % decimation == 0 && imageDisparity.cols % decimation == 0);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	bool mono;
	if(imageRgb.channels() == 3) // BGR
	{
		mono = false;
	}
	else // Mono
	{
		mono = true;
	}

	//cloud.header = cameraInfo.header;
	cloud->height = imageRgb.rows/decimation;
	cloud->width  = imageRgb.cols/decimation;
	cloud->is_dense = false;
	cloud->resize(cloud->height * cloud->width);
	if(validIndices)
	{
		validIndices->resize(cloud->size());
	}

	int oi=0;
	for(int h = 0; h < imageRgb.rows && h/decimation < (int)cloud->height; h+=decimation)
	{
		for(int w = 0; w < imageRgb.cols && w/decimation < (int)cloud->width; w+=decimation)
		{
			pcl::PointXYZRGB & pt = cloud->at((h/decimation)*cloud->width + (w/decimation));
			if(!mono)
			{
				pt.b = imageRgb.at<cv::Vec3b>(h,w)[0];
				pt.g = imageRgb.at<cv::Vec3b>(h,w)[1];
				pt.r = imageRgb.at<cv::Vec3b>(h,w)[2];
			}
			else
			{
				unsigned char v = imageRgb.at<unsigned char>(h,w);
				pt.b = v;
				pt.g = v;
				pt.r = v;
			}

			float disp = imageDisparity.type()==CV_16SC1?float(imageDisparity.at<short>(h,w))/16.0f:imageDisparity.at<float>(h,w);
			cv::Point3f ptXYZ = projectDisparityTo3D(cv::Point2f(w, h), disp, model);
			if(util3d::isFinite(ptXYZ) && ptXYZ.z >= minDepth && (maxDepth<=0.0f || ptXYZ.z <= maxDepth))
			{
				pt.x = ptXYZ.x;
				pt.y = ptXYZ.y;
				pt.z = ptXYZ.z;
				if(validIndices)
				{
					validIndices->at(oi++) = (h/decimation)*cloud->width + (w/decimation);
				}
			}
			else
			{
				pt.x = pt.y = pt.z = std::numeric_limits<float>::quiet_NaN();
			}
		}
	}
	if(validIndices)
	{
		validIndices->resize(oi);
	}
	return cloud;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudFromStereoImages(
		const cv::Mat & imageLeft,
		const cv::Mat & imageRight,
		const StereoCameraModel & model,
		int decimation,
		float maxDepth,
		float minDepth,
		std::vector<int> * validIndices,
		const ParametersMap & parameters)
{
	UASSERT(!imageLeft.empty() && !imageRight.empty());
	UASSERT(imageRight.type() == CV_8UC1);
	UASSERT(imageLeft.channels() == 3 || imageLeft.channels() == 1);
	UASSERT(imageLeft.rows == imageRight.rows &&
			imageLeft.cols == imageRight.cols);
	UASSERT(decimation >= 1);

	cv::Mat leftColor = imageLeft;
	cv::Mat rightMono = imageRight;

	StereoCameraModel modelDecimation = model;

	if(leftColor.rows % decimation != 0 ||
	   leftColor.cols % decimation != 0)
	{
		leftColor = util2d::decimate(leftColor, decimation);
		rightMono = util2d::decimate(rightMono, decimation);
		modelDecimation.scale(1/float(decimation));
		decimation = 1;
	}

	cv::Mat leftMono;
	if(leftColor.channels() == 3)
	{
		cv::cvtColor(leftColor, leftMono, CV_BGR2GRAY);
	}
	else
	{
		leftMono = leftColor;
	}

	return cloudFromDisparityRGB(
			leftColor,
			util2d::disparityFromStereoImages(leftMono, rightMono, parameters),
			modelDecimation,
			decimation,
			maxDepth,
			minDepth,
			validIndices);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr RTABMAP_EXP cloudFromSensorData(
		const SensorData & sensorData,
		int decimation,
		float maxDepth,
		float minDepth,
		std::vector<int> * validIndices,
		const ParametersMap & parameters)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	if(!sensorData.depthRaw().empty() && sensorData.cameraModels().size())
	{
		//depth
		UASSERT(int((sensorData.depthRaw().cols/sensorData.cameraModels().size())*sensorData.cameraModels().size()) == sensorData.depthRaw().cols);
		int subImageWidth = sensorData.depthRaw().cols/sensorData.cameraModels().size();
		for(unsigned int i=0; i<sensorData.cameraModels().size(); ++i)
		{
			if(sensorData.cameraModels()[i].isValidForProjection())
			{
				pcl::PointCloud<pcl::PointXYZ>::Ptr tmp = util3d::cloudFromDepth(
						cv::Mat(sensorData.depthRaw(), cv::Rect(subImageWidth*i, 0, subImageWidth, sensorData.depthRaw().rows)),
						sensorData.cameraModels()[i],
						decimation,
						maxDepth,
						minDepth,
						sensorData.cameraModels().size()==1?validIndices:0);

				if(tmp->size())
				{
					tmp = util3d::transformPointCloud(tmp, sensorData.cameraModels()[i].localTransform());

					if(sensorData.cameraModels().size() > 1)
					{
						tmp = util3d::removeNaNFromPointCloud(tmp);
						*cloud += *tmp;
					}
					else
					{
						cloud = tmp;
					}
				}
			}
			else
			{
				UERROR("Camera model %d is invalid", i);
			}
		}

		if(cloud->is_dense && validIndices)
		{
			//generate indices for all points (they are all valid)
			validIndices->resize(cloud->size());
			for(unsigned int i=0; i<cloud->size(); ++i)
			{
				validIndices->at(i) = i;
			}
		}
	}
	else if(!sensorData.imageRaw().empty() && !sensorData.rightRaw().empty() && sensorData.stereoCameraModel().isValidForProjection())
	{
		//stereo
		UASSERT(sensorData.rightRaw().type() == CV_8UC1);

		cv::Mat leftMono;
		if(sensorData.imageRaw().channels() == 3)
		{
			cv::cvtColor(sensorData.imageRaw(), leftMono, CV_BGR2GRAY);
		}
		else
		{
			leftMono = sensorData.imageRaw();
		}
		cloud = cloudFromDisparity(
				util2d::disparityFromStereoImages(leftMono, sensorData.rightRaw(), parameters),
				sensorData.stereoCameraModel(),
				decimation,
				maxDepth,
				minDepth,
				validIndices);

		if(cloud->size())
		{
			if(cloud->size())
			{
				cloud = util3d::transformPointCloud(cloud, sensorData.stereoCameraModel().left().localTransform());
			}
		}
	}
	return cloud;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr RTABMAP_EXP cloudRGBFromSensorData(
		const SensorData & sensorData,
		int decimation,
		float maxDepth,
		float minDepth,
		std::vector<int> * validIndices,
		const ParametersMap & parameters)
{
	UASSERT(!sensorData.imageRaw().empty());
	UASSERT((!sensorData.depthRaw().empty() && sensorData.cameraModels().size()) ||
			(!sensorData.rightRaw().empty() && sensorData.stereoCameraModel().isValidForProjection()));
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	if(!sensorData.depthRaw().empty() && sensorData.cameraModels().size())
	{
		//depth
		UDEBUG("");
		UASSERT(int((sensorData.imageRaw().cols/sensorData.cameraModels().size())*sensorData.cameraModels().size()) == sensorData.imageRaw().cols);
		UASSERT(int((sensorData.depthRaw().cols/sensorData.cameraModels().size())*sensorData.cameraModels().size()) == sensorData.depthRaw().cols);
		UASSERT(sensorData.imageRaw().cols % sensorData.depthRaw().cols == 0);
		UASSERT(sensorData.imageRaw().rows % sensorData.depthRaw().rows == 0);
		int subRGBWidth = sensorData.imageRaw().cols/sensorData.cameraModels().size();
		int subDepthWidth = sensorData.depthRaw().cols/sensorData.cameraModels().size();

		if(subRGBWidth % decimation != 0 || subDepthWidth % decimation != 0)
		{
			UWARN("Image size (rgb=%d,%d depth=%d,%d) modulus decimation (%d) is not null "
				  "for the cloud creation! Setting decimation to 1...",
				  subRGBWidth, sensorData.imageRaw().rows,
				  subDepthWidth, sensorData.depthRaw().rows,
				  decimation);
			decimation = 1;
		}

		for(unsigned int i=0; i<sensorData.cameraModels().size(); ++i)
		{
			if(sensorData.cameraModels()[i].isValidForProjection())
			{
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp = util3d::cloudFromDepthRGB(
						cv::Mat(sensorData.imageRaw(), cv::Rect(subRGBWidth*i, 0, subRGBWidth, sensorData.imageRaw().rows)),
						cv::Mat(sensorData.depthRaw(), cv::Rect(subDepthWidth*i, 0, subDepthWidth, sensorData.depthRaw().rows)),
						sensorData.cameraModels()[i],
						decimation,
						maxDepth,
						minDepth,
						sensorData.cameraModels().size() == 1?validIndices:0);

				if(tmp->size())
				{
					tmp = util3d::transformPointCloud(tmp, sensorData.cameraModels()[i].localTransform());

					if(sensorData.cameraModels().size() > 1)
					{
						tmp = util3d::removeNaNFromPointCloud(tmp);
						*cloud += *tmp;
					}
					else
					{
						cloud = tmp;
					}
				}
			}
			else
			{
				UERROR("Camera model %d is invalid", i);
			}
		}

		if(cloud->is_dense && validIndices)
		{
			//generate indices for all points (they are all valid)
			validIndices->resize(cloud->size());
			for(unsigned int i=0; i<cloud->size(); ++i)
			{
				validIndices->at(i) = i;
			}
		}
	}
	else if(!sensorData.rightRaw().empty() && sensorData.stereoCameraModel().isValidForProjection())
	{
		//stereo
		UDEBUG("");
		cloud = cloudFromStereoImages(sensorData.imageRaw(),
				sensorData.rightRaw(),
				sensorData.stereoCameraModel(),
				decimation,
				maxDepth,
				minDepth,
				validIndices,
				parameters);

		if(cloud->size())
		{
			cloud = util3d::transformPointCloud(cloud, sensorData.stereoCameraModel().left().localTransform());
		}
	}
	return cloud;
}

pcl::PointCloud<pcl::PointXYZ> laserScanFromDepthImage(
					const cv::Mat & depthImage,
					float fx,
					float fy,
					float cx,
					float cy,
					float maxDepth,
					float minDepth,
					const Transform & localTransform)
{
	UASSERT(depthImage.type() == CV_16UC1 || depthImage.type() == CV_32FC1);
	UASSERT(!localTransform.isNull());

	pcl::PointCloud<pcl::PointXYZ> scan;
	int middle = depthImage.rows/2;
	if(middle)
	{
		scan.resize(depthImage.cols);
		int oi = 0;
		for(int i=0; i<depthImage.cols; ++i)
		{
			pcl::PointXYZ pt = util3d::projectDepthTo3D(depthImage, i, middle, cx, cy, fx, fy, false);
			if(pcl::isFinite(pt) && pt.z >= minDepth && (maxDepth == 0 || pt.z < maxDepth))
			{
				if(!localTransform.isIdentity())
				{
					pt = util3d::transformPoint(pt, localTransform);
				}
				scan[oi++] = pt;
			}
		}
		scan.resize(oi);
	}
	return scan;
}

cv::Mat laserScanFromPointCloud(const pcl::PointCloud<pcl::PointXYZ> & cloud, const Transform & transform)
{
	cv::Mat laserScan(1, (int)cloud.size(), CV_32FC3);
	bool nullTransform = transform.isNull() || transform.isIdentity();
	Eigen::Affine3f transform3f = transform.toEigen3f();
	for(unsigned int i=0; i<cloud.size(); ++i)
	{
		if(!nullTransform)
		{
			pcl::PointXYZ pt = pcl::transformPoint(cloud.at(i), transform3f);
			laserScan.at<cv::Vec3f>(i)[0] = pt.x;
			laserScan.at<cv::Vec3f>(i)[1] = pt.y;
			laserScan.at<cv::Vec3f>(i)[2] = pt.z;
		}
		else
		{
			laserScan.at<cv::Vec3f>(i)[0] = cloud.at(i).x;
			laserScan.at<cv::Vec3f>(i)[1] = cloud.at(i).y;
			laserScan.at<cv::Vec3f>(i)[2] = cloud.at(i).z;
		}

	}
	return laserScan;
}

cv::Mat laserScanFromPointCloud(const pcl::PointCloud<pcl::PointNormal> & cloud, const Transform & transform)
{
	cv::Mat laserScan(1, (int)cloud.size(), CV_32FC(6));
	bool nullTransform = transform.isNull() || transform.isIdentity();
	Eigen::Affine3f transform3f = transform.toEigen3f();
	for(unsigned int i=0; i<cloud.size(); ++i)
	{
		if(!nullTransform)
		{
			pcl::PointNormal pt = pcl::transformPoint(cloud.at(i), transform3f);
			laserScan.at<cv::Vec6f>(i)[0] = pt.x;
			laserScan.at<cv::Vec6f>(i)[1] = pt.y;
			laserScan.at<cv::Vec6f>(i)[2] = pt.z;
			laserScan.at<cv::Vec6f>(i)[3] = pt.normal_x;
			laserScan.at<cv::Vec6f>(i)[4] = pt.normal_y;
			laserScan.at<cv::Vec6f>(i)[5] = pt.normal_z;
		}
		else
		{
			laserScan.at<cv::Vec6f>(i)[0] = cloud.at(i).x;
			laserScan.at<cv::Vec6f>(i)[1] = cloud.at(i).y;
			laserScan.at<cv::Vec6f>(i)[2] = cloud.at(i).z;
			laserScan.at<cv::Vec6f>(i)[3] = cloud.at(i).normal_x;
			laserScan.at<cv::Vec6f>(i)[4] = cloud.at(i).normal_y;
			laserScan.at<cv::Vec6f>(i)[5] = cloud.at(i).normal_z;
		}
	}
	return laserScan;
}

cv::Mat laserScan2dFromPointCloud(const pcl::PointCloud<pcl::PointXYZ> & cloud, const Transform & transform)
{
	cv::Mat laserScan(1, (int)cloud.size(), CV_32FC2);
	bool nullTransform = transform.isNull();
	Eigen::Affine3f transform3f = transform.toEigen3f();
	for(unsigned int i=0; i<cloud.size(); ++i)
	{
		if(!nullTransform)
		{
			pcl::PointXYZ pt = pcl::transformPoint(cloud.at(i), transform3f);
			laserScan.at<cv::Vec2f>(i)[0] = pt.x;
			laserScan.at<cv::Vec2f>(i)[1] = pt.y;
		}
		else
		{
			laserScan.at<cv::Vec2f>(i)[0] = cloud.at(i).x;
			laserScan.at<cv::Vec2f>(i)[1] = cloud.at(i).y;
		}

	}
	return laserScan;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr laserScanToPointCloud(const cv::Mat & laserScan, const Transform & transform)
{
	UASSERT(laserScan.empty() || laserScan.type() == CV_32FC2 || laserScan.type() == CV_32FC3 || laserScan.type() == CV_32FC(6));

	pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
	output->resize(laserScan.cols);
	bool nullTransform = transform.isNull();
	Eigen::Affine3f transform3f = transform.toEigen3f();
	for(int i=0; i<laserScan.cols; ++i)
	{
		if(laserScan.type() == CV_32FC2)
		{
			output->at(i).x = laserScan.at<cv::Vec2f>(i)[0];
			output->at(i).y = laserScan.at<cv::Vec2f>(i)[1];
		}
		else if(laserScan.type() == CV_32FC3)
		{
			output->at(i).x = laserScan.at<cv::Vec3f>(i)[0];
			output->at(i).y = laserScan.at<cv::Vec3f>(i)[1];
			output->at(i).z = laserScan.at<cv::Vec3f>(i)[2];
		}
		else
		{
			output->at(i).x = laserScan.at<cv::Vec6f>(i)[0];
			output->at(i).y = laserScan.at<cv::Vec6f>(i)[1];
			output->at(i).z = laserScan.at<cv::Vec6f>(i)[2];
		}

		if(!nullTransform)
		{
			output->at(i) = pcl::transformPoint(output->at(i), transform3f);
		}
	}
	return output;
}

pcl::PointCloud<pcl::PointNormal>::Ptr laserScanToPointCloudNormal(const cv::Mat & laserScan, const Transform & transform)
{
	UASSERT(laserScan.empty() || laserScan.type() == CV_32FC2 || laserScan.type() == CV_32FC3 || laserScan.type() == CV_32FC(6));

	pcl::PointCloud<pcl::PointNormal>::Ptr output(new pcl::PointCloud<pcl::PointNormal>);
	output->resize(laserScan.cols);
	bool nullTransform = transform.isNull();
	for(int i=0; i<laserScan.cols; ++i)
	{
		if(laserScan.type() == CV_32FC2)
		{
			output->at(i).x = laserScan.at<cv::Vec2f>(i)[0];
			output->at(i).y = laserScan.at<cv::Vec2f>(i)[1];
		}
		else if(laserScan.type() == CV_32FC3)
		{
			output->at(i).x = laserScan.at<cv::Vec3f>(i)[0];
			output->at(i).y = laserScan.at<cv::Vec3f>(i)[1];
			output->at(i).z = laserScan.at<cv::Vec3f>(i)[2];
		}
		else
		{
			output->at(i).x = laserScan.at<cv::Vec6f>(i)[0];
			output->at(i).y = laserScan.at<cv::Vec6f>(i)[1];
			output->at(i).z = laserScan.at<cv::Vec6f>(i)[2];
			output->at(i).normal_x = laserScan.at<cv::Vec6f>(i)[3];
			output->at(i).normal_y = laserScan.at<cv::Vec6f>(i)[4];
			output->at(i).normal_z = laserScan.at<cv::Vec6f>(i)[5];
		}

		if(!nullTransform)
		{
			output->at(i) = util3d::transformPoint(output->at(i), transform);
		}
	}
	return output;
}

// inspired from ROS image_geometry/src/stereo_camera_model.cpp
cv::Point3f projectDisparityTo3D(
		const cv::Point2f & pt,
		float disparity,
		const StereoCameraModel & model)
{
	if(disparity > 0.0f && model.baseline() > 0.0f && model.left().fx() > 0.0f)
	{
		//Z = baseline * f / (d + cx1-cx0);
		float c = 0.0f;
		if(model.right().cx()>0.0f && model.left().cx()>0.0f)
		{
			c = model.right().cx() - model.left().cx();
		}
		float W = model.baseline()/(disparity + c);
		return cv::Point3f((pt.x - model.left().cx())*W, (pt.y - model.left().cy())*W, model.left().fx()*W);
	}
	float bad_point = std::numeric_limits<float>::quiet_NaN ();
	return cv::Point3f(bad_point, bad_point, bad_point);
}

cv::Point3f projectDisparityTo3D(
		const cv::Point2f & pt,
		const cv::Mat & disparity,
		const StereoCameraModel & model)
{
	UASSERT(!disparity.empty() && (disparity.type() == CV_32FC1 || disparity.type() == CV_16SC1));
	int u = int(pt.x+0.5f);
	int v = int(pt.y+0.5f);
	float bad_point = std::numeric_limits<float>::quiet_NaN ();
	if(uIsInBounds(u, 0, disparity.cols) &&
	   uIsInBounds(v, 0, disparity.rows))
	{
		float d = disparity.type() == CV_16SC1?float(disparity.at<short>(v,u))/16.0f:disparity.at<float>(v,u);
		return projectDisparityTo3D(pt, d, model);
	}
	return cv::Point3f(bad_point, bad_point, bad_point);
}

// Register point cloud to camera (return registered depth image)
cv::Mat projectCloudToCamera(
		const cv::Size & imageSize,
		const cv::Mat & cameraMatrixK, // /base_link -> /camera_link
		const cv::Mat & laserScan,     // assuming laser scan points are already in /base_link coordinate
		const rtabmap::Transform & cameraTransform)
{
	UASSERT(!cameraTransform.isNull());
	UASSERT(!laserScan.empty());
	UASSERT(laserScan.type() == CV_32FC2 || laserScan.type() == CV_32FC3 || laserScan.type() == CV_32FC(6));
	UASSERT(cameraMatrixK.type() == CV_64FC1 && cameraMatrixK.cols == 3 && cameraMatrixK.cols == 3);

	float fx = cameraMatrixK.at<double>(0,0);
	float fy = cameraMatrixK.at<double>(1,1);
	float cx = cameraMatrixK.at<double>(0,2);
	float cy = cameraMatrixK.at<double>(1,2);

	cv::Mat registered = cv::Mat::zeros(imageSize, CV_32FC1);
	Transform t = cameraTransform.inverse();

	int count = 0;
	for(int i=0; i<laserScan.cols; ++i)
	{
		// Get 3D from laser scan
		cv::Point3f ptScan;
		if(laserScan.type() == CV_32FC2)
		{
			ptScan.x = laserScan.at<cv::Vec2f>(i)[0];
			ptScan.y = laserScan.at<cv::Vec2f>(i)[1];
			ptScan.z = 0;
		}
		else if(laserScan.type() == CV_32FC3)
		{
			ptScan.x = laserScan.at<cv::Vec3f>(i)[0];
			ptScan.y = laserScan.at<cv::Vec3f>(i)[1];
			ptScan.z = laserScan.at<cv::Vec3f>(i)[2];
		}
		else
		{
			ptScan.x = laserScan.at<cv::Vec6f>(i)[0];
			ptScan.y = laserScan.at<cv::Vec6f>(i)[1];
			ptScan.z = laserScan.at<cv::Vec6f>(i)[2];
		}
		ptScan = util3d::transformPoint(ptScan, t);

		// re-project in camera frame
		float z = ptScan.z;
		float invZ = 1.0f/z;
		int dx = (fx*ptScan.x)*invZ + cx;
		int dy = (fy*ptScan.y)*invZ + cy;

		if(z > 0.0f && uIsInBounds(dx, 0, registered.cols) && uIsInBounds(dy, 0, registered.rows))
		{
			++count;
			float &zReg = registered.at<float>(dy, dx);
			if(zReg == 0 || z < zReg)
			{
				zReg = z;
			}
		}
	}
	UDEBUG("Points in camera=%d/%d", count, laserScan.cols);

	return registered;
}

cv::Mat projectCloudToCamera(
		const cv::Size & imageSize,
		const cv::Mat & cameraMatrixK, // /base_link -> /camera_link
		const pcl::PointCloud<pcl::PointXYZ>::Ptr laserScan,  // assuming points are already in /base_link coordinate
		const rtabmap::Transform & cameraTransform)
{
	UASSERT(!cameraTransform.isNull());
	UASSERT(!laserScan->empty());
	UASSERT(cameraMatrixK.type() == CV_64FC1 && cameraMatrixK.cols == 3 && cameraMatrixK.cols == 3);

	float fx = cameraMatrixK.at<double>(0,0);
	float fy = cameraMatrixK.at<double>(1,1);
	float cx = cameraMatrixK.at<double>(0,2);
	float cy = cameraMatrixK.at<double>(1,2);

	cv::Mat registered = cv::Mat::zeros(imageSize, CV_32FC1);
	Transform t = cameraTransform.inverse();

	int count = 0;
	for(int i=0; i<(int)laserScan->size(); ++i)
	{
		// Get 3D from laser scan
		pcl::PointXYZ ptScan = laserScan->at(i);
		ptScan = util3d::transformPoint(ptScan, t);

		// re-project in camera frame
		float z = ptScan.z;
		float invZ = 1.0f/z;
		int dx = (fx*ptScan.x)*invZ + cx;
		int dy = (fy*ptScan.y)*invZ + cy;

		if(z > 0.0f && uIsInBounds(dx, 0, registered.cols) && uIsInBounds(dy, 0, registered.rows))
		{
			++count;
			float &zReg = registered.at<float>(dy, dx);
			if(zReg == 0 || z < zReg)
			{
				zReg = z;
			}
		}
	}
	UDEBUG("Points in camera=%d/%d", count, (int)laserScan->size());

	return registered;
}

void fillProjectedCloudHoles(cv::Mat & registeredDepth, bool verticalDirection, bool fillToBorder)
{
	UASSERT(registeredDepth.type() == CV_32FC1);
	if(verticalDirection)
	{
		// vertical, for each column
		for(int x=0; x<registeredDepth.cols; ++x)
		{
			float valueA = 0.0f;
			int indexA = -1;
			for(int y=0; y<registeredDepth.rows; ++y)
			{
				float v = registeredDepth.at<float>(y,x);
				if(fillToBorder && y == registeredDepth.rows-1 && v<=0.0f && indexA>=0)
				{
					v = valueA;
				}
				if(v > 0.0f)
				{
					if(fillToBorder && indexA < 0)
					{
						indexA = 0;
						valueA = v;
					}
					if(indexA >=0)
					{
						int range = y-indexA;
						if(range > 1)
						{
							float slope = (v-valueA)/(range);
							for(int k=1; k<range; ++k)
							{
								registeredDepth.at<float>(indexA+k,x) = valueA+slope*float(k);
							}
						}
					}
					valueA = v;
					indexA = y;
				}
			}
		}
	}
	else
	{
		// horizontal, for each row
		for(int y=0; y<registeredDepth.rows; ++y)
		{
			float valueA = 0.0f;
			int indexA = -1;
			for(int x=0; x<registeredDepth.cols; ++x)
			{
				float v = registeredDepth.at<float>(y,x);
				if(fillToBorder && x == registeredDepth.cols-1 && v<=0.0f && indexA>=0)
				{
					v = valueA;
				}
				if(v > 0.0f)
				{
					if(fillToBorder && indexA < 0)
					{
						indexA = 0;
						valueA = v;
					}
					if(indexA >=0)
					{
						int range = x-indexA;
						if(range > 1)
						{
							float slope = (v-valueA)/(range);
							for(int k=1; k<range; ++k)
							{
								registeredDepth.at<float>(y,indexA+k) = valueA+slope*float(k);
							}
						}
					}
					valueA = v;
					indexA = x;
				}
			}
		}
	}
}

bool isFinite(const cv::Point3f & pt)
{
	return uIsFinite(pt.x) && uIsFinite(pt.y) && uIsFinite(pt.z);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr concatenateClouds(const std::list<pcl::PointCloud<pcl::PointXYZ>::Ptr> & clouds)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	for(std::list<pcl::PointCloud<pcl::PointXYZ>::Ptr>::const_iterator iter = clouds.begin(); iter!=clouds.end(); ++iter)
	{
		*cloud += *(*iter);
	}
	return cloud;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr concatenateClouds(const std::list<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> & clouds)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	for(std::list<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>::const_iterator iter = clouds.begin(); iter!=clouds.end(); ++iter)
	{
		*cloud+=*(*iter);
	}
	return cloud;
}

pcl::TextureMesh::Ptr concatenateTextureMeshes(const std::list<pcl::TextureMesh::Ptr> & meshes)
{
	pcl::TextureMesh::Ptr output(new pcl::TextureMesh);
	std::map<std::string, int> addedMaterials; //<file, index>
	for(std::list<pcl::TextureMesh::Ptr>::const_iterator iter = meshes.begin(); iter!=meshes.end(); ++iter)
	{
		// append point cloud
		int polygonStep = output->cloud.height * output->cloud.width;
		pcl::PCLPointCloud2 tmp;
		pcl::concatenatePointCloud(output->cloud, iter->get()->cloud, tmp);
		output->cloud = tmp;

		UASSERT((*iter)->tex_polygons.size() == (*iter)->tex_coordinates.size() &&
				(*iter)->tex_polygons.size() == (*iter)->tex_materials.size());

		int materialCount = (*iter)->tex_polygons.size();
		for(int i=0; i<materialCount; ++i)
		{
			std::map<std::string, int>::iterator jter = addedMaterials.find((*iter)->tex_materials[i].tex_file);
			int index;
			if(jter != addedMaterials.end())
			{
				index = jter->second;
			}
			else
			{
				addedMaterials.insert(std::make_pair((*iter)->tex_materials[i].tex_file, output->tex_materials.size()));
				index = output->tex_materials.size();
				output->tex_materials.push_back((*iter)->tex_materials[i]);
				output->tex_materials.back().tex_name = uFormat("material_%d", index);
				output->tex_polygons.resize(output->tex_polygons.size() + 1);
				output->tex_coordinates.resize(output->tex_coordinates.size() + 1);
			}

			// update and append polygon indices
			int oi = output->tex_polygons[index].size();
			output->tex_polygons[index].resize(output->tex_polygons[index].size() + (*iter)->tex_polygons[i].size());
			for(unsigned int j=0; j<(*iter)->tex_polygons[i].size(); ++j)
			{
				pcl::Vertices polygon = (*iter)->tex_polygons[i][j];
				for(unsigned int k=0; k<polygon.vertices.size(); ++k)
				{
					polygon.vertices[k] += polygonStep;
				}
				output->tex_polygons[index][oi+j] = polygon;
			}

			// append uv coordinates
			oi = output->tex_coordinates[index].size();
			output->tex_coordinates[index].resize(output->tex_coordinates[index].size() + (*iter)->tex_coordinates[i].size());
			for(unsigned int j=0; j<(*iter)->tex_coordinates[i].size(); ++j)
			{
				output->tex_coordinates[index][oi+j] = (*iter)->tex_coordinates[i][j];
			}
		}
	}
	return output;
}

pcl::IndicesPtr concatenate(const std::vector<pcl::IndicesPtr> & indices)
{
	//compute total size
	unsigned int totalSize = 0;
	for(unsigned int i=0; i<indices.size(); ++i)
	{
		totalSize += (unsigned int)indices[i]->size();
	}
	pcl::IndicesPtr ind(new std::vector<int>(totalSize));
	unsigned int io = 0;
	for(unsigned int i=0; i<indices.size(); ++i)
	{
		for(unsigned int j=0; j<indices[i]->size(); ++j)
		{
			ind->at(io++) = indices[i]->at(j);
		}
	}
	return ind;
}

pcl::IndicesPtr concatenate(const pcl::IndicesPtr & indicesA, const pcl::IndicesPtr & indicesB)
{
	pcl::IndicesPtr ind(new std::vector<int>(*indicesA));
	ind->resize(ind->size()+indicesB->size());
	unsigned int oi = (unsigned int)indicesA->size();
	for(unsigned int i=0; i<indicesB->size(); ++i)
	{
		ind->at(oi++) = indicesB->at(i);
	}
	return ind;
}

void savePCDWords(
		const std::string & fileName,
		const std::multimap<int, pcl::PointXYZ> & words,
		const Transform & transform)
{
	if(words.size())
	{
		pcl::PointCloud<pcl::PointXYZ> cloud;
		cloud.resize(words.size());
		int i=0;
		for(std::multimap<int, pcl::PointXYZ>::const_iterator iter=words.begin(); iter!=words.end(); ++iter)
		{
			cloud[i++] = transformPoint(iter->second, transform);
		}
		pcl::io::savePCDFile(fileName, cloud);
	}
}

void savePCDWords(
		const std::string & fileName,
		const std::multimap<int, cv::Point3f> & words,
		const Transform & transform)
{
	if(words.size())
	{
		pcl::PointCloud<pcl::PointXYZ> cloud;
		cloud.resize(words.size());
		int i=0;
		for(std::multimap<int, cv::Point3f>::const_iterator iter=words.begin(); iter!=words.end(); ++iter)
		{
			cv::Point3f pt = transformPoint(iter->second, transform);
			cloud[i++] = pcl::PointXYZ(pt.x, pt.y, pt.z);
		}
		pcl::io::savePCDFile(fileName, cloud);
	}
}

pcl::PointCloud<pcl::PointXYZ>::Ptr loadBINCloud(const std::string & fileName, int dim)
{
	UASSERT(dim > 0);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	long bytes = UFile::length(fileName);
	if(bytes)
	{
		UASSERT(bytes % sizeof(float) == 0);
		int32_t num = bytes/sizeof(float);
		UASSERT(num % dim == 0);
		float *data = (float*)malloc(num*sizeof(float));

		// pointers
		float *px = data+0;
		float *py = data+1;
		float *pz = data+2;
		float *pr = data+3;

		// load point cloud
		FILE *stream;
		stream = fopen (fileName.c_str(),"rb");
		num = fread(data,sizeof(float),num,stream)/4;
		cloud->resize(num);
		for (int32_t i=0; i<num; i++) {
			(*cloud)[i].x = *px;
			(*cloud)[i].y = *py;
			(*cloud)[i].z = *pz;
			px+=4; py+=4; pz+=4; pr+=4;
		}
		fclose(stream);
	}

	return cloud;
}

cv::Mat loadScan(
		const std::string & path,
		const Transform & transform,
		int downsampleStep,
		float voxelSize,
		int normalsK)
{
	cv::Mat scan;
	UDEBUG("Loading scan (normalsK=%d) : %s", normalsK, path.c_str());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = loadCloud(path, Transform::getIdentity(), downsampleStep, voxelSize);
	if(normalsK > 0 && cloud->size())
	{
		pcl::PointCloud<pcl::Normal>::Ptr normals = util3d::computeNormals(cloud, normalsK);
		pcl::PointCloud<pcl::PointNormal>::Ptr cloudNormals(new pcl::PointCloud<pcl::PointNormal>);
		pcl::concatenateFields(*cloud, *normals, *cloudNormals);
		scan = util3d::laserScanFromPointCloud(*cloudNormals, transform);
	}
	else
	{
		scan = util3d::laserScanFromPointCloud(*cloud, transform);
	}
	return scan;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr loadCloud(
		const std::string & path,
		const Transform & transform,
		int downsampleStep,
		float voxelSize)
{
	UASSERT(!transform.isNull());
	UDEBUG("Loading cloud (step=%d, voxel=%f m) : %s", downsampleStep, voxelSize, path.c_str());
	std::string fileName = UFile::getName(path);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if(UFile::getExtension(fileName).compare("bin") == 0)
	{
		cloud = util3d::loadBINCloud(path, 4); // Assume KITTI velodyne format
	}
	else if(UFile::getExtension(fileName).compare("pcd") == 0)
	{
		pcl::io::loadPCDFile(path, *cloud);
	}
	else
	{
		pcl::io::loadPLYFile(path, *cloud);
	}
	int previousSize = (int)cloud->size();
	if(downsampleStep > 1 && cloud->size())
	{
		cloud = util3d::downsample(cloud, downsampleStep);
		UDEBUG("Downsampling scan (step=%d): %d -> %d", downsampleStep, previousSize, (int)cloud->size());
	}
	previousSize = (int)cloud->size();
	if(voxelSize > 0.0f && cloud->size())
	{
		cloud = util3d::voxelize(cloud, voxelSize);
		UDEBUG("Voxel filtering scan (voxel=%f m): %d -> %d", voxelSize, previousSize, (int)cloud->size());
	}
	if(transform.isIdentity())
	{
		return cloud;
	}
	return util3d::transformPointCloud(cloud, transform);
}

}

}
