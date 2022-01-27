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
#include <rtabmap/core/util2d.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UMath.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UFile.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>

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
		float depthErrorRatio)
{
	UASSERT(depthImage.type() == CV_16UC1 || depthImage.type() == CV_32FC1);

	pcl::PointXYZ pt;

	float depth = util2d::getDepth(depthImage, x, y, smoothing, depthErrorRatio);
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

Eigen::Vector3f projectDepthTo3DRay(
		const cv::Size & imageSize,
		float x, float y,
		float cx, float cy,
		float fx, float fy)
{
	Eigen::Vector3f ray;

	// Use correct principal point from calibration
	cx = cx > 0.0f ? cx : float(imageSize.width/2) - 0.5f; //cameraInfo.K.at(2)
	cy = cy > 0.0f ? cy : float(imageSize.height/2) - 0.5f; //cameraInfo.K.at(5)

	// Fill in XYZ
	ray[0] = (x - cx) / fx;
	ray[1] = (y - cy) / fy;
	ray[2] = 1.0f;

	return ray;
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
		const cv::Mat & imageDepthIn,
		const CameraModel & model,
		int decimation,
		float maxDepth,
		float minDepth,
		std::vector<int> * validIndices)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if(decimation == 0)
	{
		decimation = 1;
	}
	float rgbToDepthFactorX = 1.0f;
	float rgbToDepthFactorY = 1.0f;

	UASSERT(model.isValidForProjection());
	UASSERT(!imageDepthIn.empty() && (imageDepthIn.type() == CV_16UC1 || imageDepthIn.type() == CV_32FC1));

	cv::Mat imageDepth = imageDepthIn;
	if(model.imageHeight()>0 && model.imageWidth()>0)
	{
		UASSERT(model.imageHeight() % imageDepthIn.rows == 0 && model.imageWidth() % imageDepthIn.cols == 0);

		if(decimation < 0)
		{
			UDEBUG("Decimation from model (%d)", decimation);
			if(model.imageHeight() % decimation != 0)
			{
				UERROR("Decimation is not valid for current image size (model.imageHeight()=%d decimation=%d). The cloud is not created.", model.imageHeight(), decimation);
				return cloud;
			}
			if(model.imageWidth() % decimation != 0)
			{
				UERROR("Decimation is not valid for current image size (model.imageWidth()=%d decimation=%d). The cloud is not created.", model.imageWidth(), decimation);
				return cloud;
			}

			// decimate from RGB image size, upsample depth if needed
			decimation = -1*decimation;

			int targetSize = model.imageHeight() / decimation;
			if(targetSize > imageDepthIn.rows)
			{
				UDEBUG("Depth interpolation factor=%d", targetSize/imageDepthIn.rows);
				imageDepth = util2d::interpolate(imageDepthIn, targetSize/imageDepthIn.rows);
				decimation = 1;
			}
			else if(targetSize == imageDepthIn.rows)
			{
				decimation = 1;
			}
			else
			{
				UASSERT(imageDepthIn.rows % targetSize == 0);
				decimation = imageDepthIn.rows / targetSize;
			}
		}
		else
		{
			if(imageDepthIn.rows % decimation != 0)
			{
				UERROR("Decimation is not valid for current image size (imageDepth.rows=%d decimation=%d). The cloud is not created.", imageDepthIn.rows, decimation);
				return cloud;
			}
			if(imageDepthIn.cols % decimation != 0)
			{
				UERROR("Decimation is not valid for current image size (imageDepth.cols=%d decimation=%d). The cloud is not created.", imageDepthIn.cols, decimation);
				return cloud;
			}
		}

		rgbToDepthFactorX = 1.0f/float((model.imageWidth() / imageDepth.cols));
		rgbToDepthFactorY = 1.0f/float((model.imageHeight() / imageDepth.rows));
	}
	else
	{
		decimation = abs(decimation);
		UASSERT_MSG(imageDepth.rows % decimation == 0, uFormat("rows=%d decimation=%d", imageDepth.rows, decimation).c_str());
		UASSERT_MSG(imageDepth.cols % decimation == 0, uFormat("cols=%d decimation=%d", imageDepth.cols, decimation).c_str());
	}

	//cloud.header = cameraInfo.header;
	cloud->height = imageDepth.rows/decimation;
	cloud->width  = imageDepth.cols/decimation;
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

	UDEBUG("depth=%dx%d fx=%f fy=%f cx=%f cy=%f (depth factors=%f %f) decimation=%d",
			imageDepth.cols, imageDepth.rows,
			model.fx(), model.fy(), model.cx(), model.cy(),
			rgbToDepthFactorX,
			rgbToDepthFactorY,
			decimation);

	int oi = 0;
	for(int h = 0; h < imageDepth.rows && h/decimation < (int)cloud->height; h+=decimation)
	{
		for(int w = 0; w < imageDepth.cols && w/decimation < (int)cloud->width; w+=decimation)
		{
			pcl::PointXYZ & pt = cloud->at((h/decimation)*cloud->width + (w/decimation));

			pcl::PointXYZ ptXYZ = projectDepthTo3D(imageDepth, w, h, depthCx, depthCy, depthFx, depthFy, false);
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
		const cv::Mat & imageDepthIn,
		const CameraModel & model,
		int decimation,
		float maxDepth,
		float minDepth,
		std::vector<int> * validIndices)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	if(decimation == 0)
	{
		decimation = 1;
	}
	UDEBUG("");
	UASSERT(model.isValidForProjection());
	UASSERT_MSG((model.imageHeight() == 0 && model.imageWidth() == 0) ||
			    (model.imageHeight() == imageRgb.rows && model.imageWidth() == imageRgb.cols),
				uFormat("model=%dx%d rgb=%dx%d", model.imageWidth(), model.imageHeight(), imageRgb.cols, imageRgb.rows).c_str());
	//UASSERT_MSG(imageRgb.rows % imageDepthIn.rows == 0 && imageRgb.cols % imageDepthIn.cols == 0,
	//		uFormat("rgb=%dx%d depth=%dx%d", imageRgb.cols, imageRgb.rows, imageDepthIn.cols, imageDepthIn.rows).c_str());
	UASSERT(!imageDepthIn.empty() && (imageDepthIn.type() == CV_16UC1 || imageDepthIn.type() == CV_32FC1));
	if(decimation < 0)
	{
		if(imageRgb.rows % decimation != 0 || imageRgb.cols % decimation != 0)
		{
			int oldDecimation = decimation;
			while(decimation <= -1)
			{
				if(imageRgb.rows % decimation == 0 && imageRgb.cols % decimation == 0)
				{
					break;
				}
				++decimation;
			}

			if(imageRgb.rows % oldDecimation != 0 || imageRgb.cols % oldDecimation != 0)
			{
				UWARN("Decimation (%d) is not valid for current image size (rgb=%dx%d). Highest compatible decimation used=%d.", oldDecimation, imageRgb.cols, imageRgb.rows, decimation);
			}
		}
	}
	else
	{
		if(imageDepthIn.rows % decimation != 0 || imageDepthIn.cols % decimation != 0)
		{
			int oldDecimation = decimation;
			while(decimation >= 1)
			{
				if(imageDepthIn.rows % decimation == 0 && imageDepthIn.cols % decimation == 0)
				{
					break;
				}
				--decimation;
			}

			if(imageDepthIn.rows % oldDecimation != 0 || imageDepthIn.cols % oldDecimation != 0)
			{
				UWARN("Decimation (%d) is not valid for current image size (depth=%dx%d). Highest compatible decimation used=%d.", oldDecimation, imageDepthIn.cols, imageDepthIn.rows, decimation);
			}
		}
	}

	cv::Mat imageDepth = imageDepthIn;
	if(decimation < 0)
	{
		UDEBUG("Decimation from RGB image (%d)", decimation);
		// decimate from RGB image size, upsample depth if needed
		decimation = -1*decimation;

		int targetSize = imageRgb.rows / decimation;
		if(targetSize > imageDepthIn.rows)
		{
			UDEBUG("Depth interpolation factor=%d", targetSize/imageDepthIn.rows);
			imageDepth = util2d::interpolate(imageDepthIn, targetSize/imageDepthIn.rows);
			decimation = 1;
		}
		else if(targetSize == imageDepthIn.rows)
		{
			decimation = 1;
		}
		else
		{
			UASSERT(imageDepthIn.rows % targetSize == 0);
			decimation = imageDepthIn.rows / targetSize;
		}
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
	cloud->height = imageDepth.rows/decimation;
	cloud->width  = imageDepth.cols/decimation;
	cloud->is_dense = false;
	cloud->resize(cloud->height * cloud->width);
	if(validIndices)
	{
		validIndices->resize(cloud->size());
	}

	float rgbToDepthFactorX = float(imageRgb.cols) / float(imageDepth.cols);
	float rgbToDepthFactorY = float(imageRgb.rows) / float(imageDepth.rows);
	float depthFx = model.fx() / rgbToDepthFactorX;
	float depthFy = model.fy() / rgbToDepthFactorY;
	float depthCx = model.cx() / rgbToDepthFactorX;
	float depthCy = model.cy() / rgbToDepthFactorY;

	UDEBUG("rgb=%dx%d depth=%dx%d fx=%f fy=%f cx=%f cy=%f (depth factors=%f %f) decimation=%d",
			imageRgb.cols, imageRgb.rows,
			imageDepth.cols, imageDepth.rows,
			model.fx(), model.fy(), model.cx(), model.cy(),
			rgbToDepthFactorX,
			rgbToDepthFactorY,
			decimation);

	int oi = 0;
	for(int h = 0; h < imageDepth.rows && h/decimation < (int)cloud->height; h+=decimation)
	{
		for(int w = 0; w < imageDepth.cols && w/decimation < (int)cloud->width; w+=decimation)
		{
			pcl::PointXYZRGB & pt = cloud->at((h/decimation)*cloud->width + (w/decimation));

			int x = int(w*rgbToDepthFactorX);
			int y = int(h*rgbToDepthFactorY);
			UASSERT(x >=0 && x<imageRgb.cols && y >=0 && y<imageRgb.rows);
			if(!mono)
			{
				const unsigned char * bgr = imageRgb.ptr<unsigned char>(y,x);
				pt.b = bgr[0];
				pt.g = bgr[1];
				pt.r = bgr[2];
			}
			else
			{
				unsigned char v = imageRgb.at<unsigned char>(y,x);
				pt.b = v;
				pt.g = v;
				pt.r = v;
			}

			pcl::PointXYZ ptXYZ = projectDepthTo3D(imageDepth, w, h, depthCx, depthCy, depthFx, depthFy, false);
			if (pcl::isFinite(ptXYZ) && ptXYZ.z >= minDepth && (maxDepth <= 0.0f || ptXYZ.z <= maxDepth))
			{
				pt.x = ptXYZ.x;
				pt.y = ptXYZ.y;
				pt.z = ptXYZ.z;
				if (validIndices)
				{
					validIndices->at(oi) = (h / decimation)*cloud->width + (w / decimation);
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
	UASSERT(decimation >= 1);

	if(imageDisparity.rows % decimation != 0 || imageDisparity.cols % decimation != 0)
	{
		int oldDecimation = decimation;
		while(decimation >= 1)
		{
			if(imageDisparity.rows % decimation == 0 && imageDisparity.cols % decimation == 0)
			{
				break;
			}
			--decimation;
		}

		if(imageDisparity.rows % oldDecimation != 0 || imageDisparity.cols % oldDecimation != 0)
		{
			UWARN("Decimation (%d) is not valid for current image size (depth=%dx%d). Highest compatible decimation used=%d.", oldDecimation, imageDisparity.cols, imageDisparity.rows, decimation);
		}
	}

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

	if(imageDisparity.rows % decimation != 0 || imageDisparity.cols % decimation != 0)
	{
		int oldDecimation = decimation;
		while(decimation >= 1)
		{
			if(imageDisparity.rows % decimation == 0 && imageDisparity.cols % decimation == 0)
			{
				break;
			}
			--decimation;
		}

		if(imageDisparity.rows % oldDecimation != 0 || imageDisparity.cols % oldDecimation != 0)
		{
			UWARN("Decimation (%d) is not valid for current image size (depth=%dx%d). Highest compatible decimation used=%d.", oldDecimation, imageDisparity.cols, imageDisparity.rows, decimation);
		}
	}

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
	UASSERT(decimation >= 1.0f);

	cv::Mat leftColor = imageLeft;
	cv::Mat rightMono = imageRight;

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
			model,
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
		const ParametersMap & stereoParameters,
		const std::vector<float> & roiRatios)
{
	if(decimation == 0)
	{
		decimation = 1;
	}

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
				cv::Mat depth = cv::Mat(sensorData.depthRaw(), cv::Rect(subImageWidth*i, 0, subImageWidth, sensorData.depthRaw().rows));
				CameraModel model = sensorData.cameraModels()[i];
				if( roiRatios.size() == 4 &&
					(roiRatios[0] > 0.0f ||
					roiRatios[1] > 0.0f ||
					roiRatios[2] > 0.0f ||
					roiRatios[3] > 0.0f))
				{
					cv::Rect roiDepth = util2d::computeRoi(depth, roiRatios);
					cv::Rect roiRgb;
					if(model.imageWidth() && model.imageHeight())
					{
						roiRgb = util2d::computeRoi(model.imageSize(), roiRatios);
					}
					if(	roiDepth.width%decimation==0 &&
						roiDepth.height%decimation==0 &&
						(roiRgb.width != 0 ||
						   (roiRgb.width%decimation==0 &&
							roiRgb.height%decimation==0)))
					{
						depth = cv::Mat(depth, roiDepth);
						if(model.imageWidth() != 0 && model.imageHeight() != 0)
						{
							model = model.roi(util2d::computeRoi(model.imageSize(), roiRatios));
						}
						else
						{
							model = model.roi(roiDepth);
						}
					}
					else
					{
						UERROR("Cannot apply ROI ratios [%f,%f,%f,%f] because resulting "
							  "dimension (depth=%dx%d rgb=%dx%d) cannot be divided exactly "
							  "by decimation parameter (%d). Ignoring ROI ratios...",
							  roiRatios[0],
							  roiRatios[1],
							  roiRatios[2],
							  roiRatios[3],
							  roiDepth.width,
							  roiDepth.height,
							  roiRgb.width,
							  roiRgb.height,
							  decimation);
					}
				}

				pcl::PointCloud<pcl::PointXYZ>::Ptr tmp = util3d::cloudFromDepth(
						depth,
						model,
						decimation,
						maxDepth,
						minDepth,
						sensorData.cameraModels().size()==1?validIndices:0);

				if(tmp->size())
				{
					if(!model.localTransform().isNull() && !model.localTransform().isIdentity())
					{
						tmp = util3d::transformPointCloud(tmp, model.localTransform());
					}

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

		cv::Mat right(sensorData.rightRaw());
		StereoCameraModel model = sensorData.stereoCameraModel();
		if( roiRatios.size() == 4 &&
			((roiRatios[0] > 0.0f && roiRatios[0] <= 1.0f) ||
			 (roiRatios[1] > 0.0f && roiRatios[1] <= 1.0f) ||
			 (roiRatios[2] > 0.0f && roiRatios[2] <= 1.0f) ||
			 (roiRatios[3] > 0.0f && roiRatios[3] <= 1.0f)))
		{
			cv::Rect roi = util2d::computeRoi(leftMono, roiRatios);
			if(	roi.width%decimation==0 &&
				roi.height%decimation==0)
			{
				leftMono = cv::Mat(leftMono, roi);
				right = cv::Mat(right, roi);
				model.roi(roi);
			}
			else
			{
				UERROR("Cannot apply ROI ratios [%f,%f,%f,%f] because resulting "
					  "dimension (left=%dx%d) cannot be divided exactly "
					  "by decimation parameter (%d). Ignoring ROI ratios...",
					  roiRatios[0],
					  roiRatios[1],
					  roiRatios[2],
					  roiRatios[3],
					  roi.width,
					  roi.height,
					  decimation);
			}
		}

		cloud = cloudFromDisparity(
				util2d::disparityFromStereoImages(leftMono, right, stereoParameters),
				model,
				decimation,
				maxDepth,
				minDepth,
				validIndices);

		if(cloud->size())
		{
			if(cloud->size() && !sensorData.stereoCameraModel().left().localTransform().isNull() && !sensorData.stereoCameraModel().left().localTransform().isIdentity())
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
		const ParametersMap & stereoParameters,
		const std::vector<float> & roiRatios)
{
	if(decimation == 0)
	{
		decimation = 1;
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	if(!sensorData.imageRaw().empty() && !sensorData.depthRaw().empty() && sensorData.cameraModels().size())
	{
		//depth
		UDEBUG("");
		UASSERT(int((sensorData.imageRaw().cols/sensorData.cameraModels().size())*sensorData.cameraModels().size()) == sensorData.imageRaw().cols);
		UASSERT(int((sensorData.depthRaw().cols/sensorData.cameraModels().size())*sensorData.cameraModels().size()) == sensorData.depthRaw().cols);
		//UASSERT_MSG(sensorData.imageRaw().cols % sensorData.depthRaw().cols == 0, uFormat("rgb=%d depth=%d", sensorData.imageRaw().cols, sensorData.depthRaw().cols).c_str());
		//UASSERT_MSG(sensorData.imageRaw().rows % sensorData.depthRaw().rows == 0, uFormat("rgb=%d depth=%d", sensorData.imageRaw().rows, sensorData.depthRaw().rows).c_str());
		int subRGBWidth = sensorData.imageRaw().cols/sensorData.cameraModels().size();
		int subDepthWidth = sensorData.depthRaw().cols/sensorData.cameraModels().size();

		for(unsigned int i=0; i<sensorData.cameraModels().size(); ++i)
		{
			if(sensorData.cameraModels()[i].isValidForProjection())
			{
				cv::Mat rgb(sensorData.imageRaw(), cv::Rect(subRGBWidth*i, 0, subRGBWidth, sensorData.imageRaw().rows));
				cv::Mat depth(sensorData.depthRaw(), cv::Rect(subDepthWidth*i, 0, subDepthWidth, sensorData.depthRaw().rows));
				CameraModel model = sensorData.cameraModels()[i];
				if( roiRatios.size() == 4 &&
					((roiRatios[0] > 0.0f && roiRatios[0] <= 1.0f) ||
					 (roiRatios[1] > 0.0f && roiRatios[1] <= 1.0f) ||
					 (roiRatios[2] > 0.0f && roiRatios[2] <= 1.0f) ||
					 (roiRatios[3] > 0.0f && roiRatios[3] <= 1.0f)))
				{
					cv::Rect roiDepth = util2d::computeRoi(depth, roiRatios);
					cv::Rect roiRgb = util2d::computeRoi(rgb, roiRatios);
					if(	roiDepth.width%decimation==0 &&
						roiDepth.height%decimation==0 &&
						roiRgb.width%decimation==0 &&
						roiRgb.height%decimation==0)
					{
						depth = cv::Mat(depth, roiDepth);
						rgb = cv::Mat(rgb, roiRgb);
						model = model.roi(roiRgb);
					}
					else
					{
						UERROR("Cannot apply ROI ratios [%f,%f,%f,%f] because resulting "
							  "dimension (depth=%dx%d rgb=%dx%d) cannot be divided exactly "
							  "by decimation parameter (%d). Ignoring ROI ratios...",
							  roiRatios[0],
							  roiRatios[1],
							  roiRatios[2],
							  roiRatios[3],
							  roiDepth.width,
							  roiDepth.height,
							  roiRgb.width,
							  roiRgb.height,
							  decimation);
					}
				}

				pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp = util3d::cloudFromDepthRGB(
						rgb,
						depth,
						model,
						decimation,
						maxDepth,
						minDepth,
						sensorData.cameraModels().size() == 1?validIndices:0);

				if(tmp->size())
				{
					if(!model.localTransform().isNull() && !model.localTransform().isIdentity())
					{
						tmp = util3d::transformPointCloud(tmp, model.localTransform());
					}

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
		UDEBUG("");

		cv::Mat left(sensorData.imageRaw());
		cv::Mat right(sensorData.rightRaw());
		StereoCameraModel model = sensorData.stereoCameraModel();
		if( roiRatios.size() == 4 &&
			((roiRatios[0] > 0.0f && roiRatios[0] <= 1.0f) ||
			 (roiRatios[1] > 0.0f && roiRatios[1] <= 1.0f) ||
			 (roiRatios[2] > 0.0f && roiRatios[2] <= 1.0f) ||
			 (roiRatios[3] > 0.0f && roiRatios[3] <= 1.0f)))
		{
			cv::Rect roi = util2d::computeRoi(left, roiRatios);
			if(	roi.width%decimation==0 &&
				roi.height%decimation==0)
			{
				left = cv::Mat(left, roi);
				right = cv::Mat(right, roi);
				model.roi(roi);
			}
			else
			{
				UERROR("Cannot apply ROI ratios [%f,%f,%f,%f] because resulting "
					  "dimension (left=%dx%d) cannot be divided exactly "
					  "by decimation parameter (%d). Ignoring ROI ratios...",
					  roiRatios[0],
					  roiRatios[1],
					  roiRatios[2],
					  roiRatios[3],
					  roi.width,
					  roi.height,
					  decimation);
			}
		}

		cloud = cloudFromStereoImages(
				left,
				right,
				model,
				decimation,
				maxDepth,
				minDepth,
				validIndices,
				stereoParameters);

		if(cloud->size() && !sensorData.stereoCameraModel().left().localTransform().isNull() && !sensorData.stereoCameraModel().left().localTransform().isIdentity())
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
		for(int i=depthImage.cols-1; i>=0; --i)
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

pcl::PointCloud<pcl::PointXYZ> laserScanFromDepthImages(
		const cv::Mat & depthImages,
		const std::vector<CameraModel> & cameraModels,
		float maxDepth,
		float minDepth)
{
	pcl::PointCloud<pcl::PointXYZ> scan;
	UASSERT(int((depthImages.cols/cameraModels.size())*cameraModels.size()) == depthImages.cols);
	int subImageWidth = depthImages.cols/cameraModels.size();
	for(int i=(int)cameraModels.size()-1; i>=0; --i)
	{
		if(cameraModels[i].isValidForProjection())
		{
			cv::Mat depth = cv::Mat(depthImages, cv::Rect(subImageWidth*i, 0, subImageWidth, depthImages.rows));

			scan += laserScanFromDepthImage(
					depth,
					cameraModels[i].fx(),
					cameraModels[i].fy(),
					cameraModels[i].cx(),
					cameraModels[i].cy(),
					maxDepth,
					minDepth,
					cameraModels[i].localTransform());
		}
		else
		{
			UERROR("Camera model %d is invalid", i);
		}
	}
	return scan;
}

LaserScan laserScanFromPointCloud(const pcl::PointCloud<pcl::PointXYZ> & cloud, const Transform & transform, bool filterNaNs)
{
	return laserScanFromPointCloud(cloud, pcl::IndicesPtr(), transform, filterNaNs);
}
LaserScan laserScanFromPointCloud(const pcl::PointCloud<pcl::PointXYZ> & cloud, const pcl::IndicesPtr & indices, const Transform & transform, bool filterNaNs)
{
	cv::Mat laserScan;
	bool nullTransform = transform.isNull() || transform.isIdentity();
	Eigen::Affine3f transform3f = transform.toEigen3f();
	int oi = 0;
	if(indices.get())
	{
		laserScan = cv::Mat(1, (int)indices->size(), CV_32FC3);
		for(unsigned int i=0; i<indices->size(); ++i)
		{
			int index = indices->at(i);
			if(!filterNaNs || pcl::isFinite(cloud.at(index)))
			{
				float * ptr = laserScan.ptr<float>(0, oi++);
				if(!nullTransform)
				{
					pcl::PointXYZ pt = pcl::transformPoint(cloud.at(index), transform3f);
					ptr[0] = pt.x;
					ptr[1] = pt.y;
					ptr[2] = pt.z;
				}
				else
				{
					ptr[0] = cloud.at(index).x;
					ptr[1] = cloud.at(index).y;
					ptr[2] = cloud.at(index).z;
				}
			}
		}
	}
	else
	{
		laserScan = cv::Mat(1, (int)cloud.size(), CV_32FC3);
		for(unsigned int i=0; i<cloud.size(); ++i)
		{
			if(!filterNaNs || pcl::isFinite(cloud.at(i)))
			{
				float * ptr = laserScan.ptr<float>(0, oi++);
				if(!nullTransform)
				{
					pcl::PointXYZ pt = pcl::transformPoint(cloud.at(i), transform3f);
					ptr[0] = pt.x;
					ptr[1] = pt.y;
					ptr[2] = pt.z;
				}
				else
				{
					ptr[0] = cloud.at(i).x;
					ptr[1] = cloud.at(i).y;
					ptr[2] = cloud.at(i).z;
				}
			}
		}
	}
	if(oi == 0)
	{
		return LaserScan();
	}
	return LaserScan(laserScan(cv::Range::all(), cv::Range(0,oi)), 0, 0.0f, LaserScan::kXYZ);
}

LaserScan laserScanFromPointCloud(const pcl::PointCloud<pcl::PointNormal> & cloud, const Transform & transform, bool filterNaNs)
{
	return laserScanFromPointCloud(cloud, pcl::IndicesPtr(), transform, filterNaNs);
}
LaserScan laserScanFromPointCloud(const pcl::PointCloud<pcl::PointNormal> & cloud, const pcl::IndicesPtr & indices, const Transform & transform, bool filterNaNs)
{
	cv::Mat laserScan;
	bool nullTransform = transform.isNull() || transform.isIdentity();
	int oi=0;
	if(indices.get())
	{
		laserScan = cv::Mat(1, (int)indices->size(), CV_32FC(6));
		for(unsigned int i=0; i<indices->size(); ++i)
		{
			int index = indices->at(i);
			if(!filterNaNs || (pcl::isFinite(cloud.at(index)) &&
					uIsFinite(cloud.at(index).normal_x) &&
					uIsFinite(cloud.at(index).normal_y) &&
					uIsFinite(cloud.at(index).normal_z)))
			{
				float * ptr = laserScan.ptr<float>(0, oi++);
				if(!nullTransform)
				{
					pcl::PointNormal pt = util3d::transformPoint(cloud.at(index), transform);
					ptr[0] = pt.x;
					ptr[1] = pt.y;
					ptr[2] = pt.z;
					ptr[3] = pt.normal_x;
					ptr[4] = pt.normal_y;
					ptr[5] = pt.normal_z;
				}
				else
				{
					ptr[0] = cloud.at(index).x;
					ptr[1] = cloud.at(index).y;
					ptr[2] = cloud.at(index).z;
					ptr[3] = cloud.at(index).normal_x;
					ptr[4] = cloud.at(index).normal_y;
					ptr[5] = cloud.at(index).normal_z;
				}
			}
		}
	}
	else
	{
		laserScan = cv::Mat(1, (int)cloud.size(), CV_32FC(6));
		for(unsigned int i=0; i<cloud.size(); ++i)
		{
			if(!filterNaNs || (pcl::isFinite(cloud.at(i)) &&
					uIsFinite(cloud.at(i).normal_x) &&
					uIsFinite(cloud.at(i).normal_y) &&
					uIsFinite(cloud.at(i).normal_z)))
			{
				float * ptr = laserScan.ptr<float>(0, oi++);
				if(!nullTransform)
				{
					pcl::PointNormal pt = util3d::transformPoint(cloud.at(i), transform);
					ptr[0] = pt.x;
					ptr[1] = pt.y;
					ptr[2] = pt.z;
					ptr[3] = pt.normal_x;
					ptr[4] = pt.normal_y;
					ptr[5] = pt.normal_z;
				}
				else
				{
					ptr[0] = cloud.at(i).x;
					ptr[1] = cloud.at(i).y;
					ptr[2] = cloud.at(i).z;
					ptr[3] = cloud.at(i).normal_x;
					ptr[4] = cloud.at(i).normal_y;
					ptr[5] = cloud.at(i).normal_z;
				}
			}
		}
	}
	if(oi == 0)
	{
		return LaserScan();
	}
	return LaserScan(laserScan(cv::Range::all(), cv::Range(0,oi)), 0, 0.0f, LaserScan::kXYZNormal);
}

LaserScan laserScanFromPointCloud(const pcl::PointCloud<pcl::PointXYZ> & cloud, const pcl::PointCloud<pcl::Normal> & normals, const Transform & transform, bool filterNaNs)
{
	UASSERT(cloud.size() == normals.size());
	cv::Mat laserScan = cv::Mat(1, (int)cloud.size(), CV_32FC(6));
	bool nullTransform = transform.isNull() || transform.isIdentity();
	int oi =0;
	for(unsigned int i=0; i<cloud.size(); ++i)
	{
		if(!filterNaNs || (pcl::isFinite(cloud.at(i)) && pcl::isFinite(normals.at(i))))
		{
			float * ptr = laserScan.ptr<float>(0, oi++);
			if(!nullTransform)
			{
				pcl::PointNormal pt;
				pt.x = cloud.at(i).x;
				pt.y = cloud.at(i).y;
				pt.z = cloud.at(i).z;
				pt.normal_x = normals.at(i).normal_x;
				pt.normal_y = normals.at(i).normal_y;
				pt.normal_z = normals.at(i).normal_z;
				pt = util3d::transformPoint(pt, transform);
				ptr[0] = pt.x;
				ptr[1] = pt.y;
				ptr[2] = pt.z;
				ptr[3] = pt.normal_x;
				ptr[4] = pt.normal_y;
				ptr[5] = pt.normal_z;
			}
			else
			{
				ptr[0] = cloud.at(i).x;
				ptr[1] = cloud.at(i).y;
				ptr[2] = cloud.at(i).z;
				ptr[3] = normals.at(i).normal_x;
				ptr[4] = normals.at(i).normal_y;
				ptr[5] = normals.at(i).normal_z;
			}
		}
	}
	if(oi == 0)
	{
		return LaserScan();
	}
	return LaserScan(laserScan(cv::Range::all(), cv::Range(0,oi)), 0, 0.0f, LaserScan::kXYZNormal);
}

LaserScan laserScanFromPointCloud(const pcl::PointCloud<pcl::PointXYZRGB> & cloud, const Transform & transform, bool filterNaNs)
{
	return laserScanFromPointCloud(cloud, pcl::IndicesPtr(), transform, filterNaNs);
}

LaserScan laserScanFromPointCloud(const pcl::PointCloud<pcl::PointXYZRGB> & cloud, const pcl::IndicesPtr & indices, const Transform & transform, bool filterNaNs)
{
	cv::Mat laserScan;
	bool nullTransform = transform.isNull() || transform.isIdentity();
	Eigen::Affine3f transform3f = transform.toEigen3f();
	int oi=0;
	if(indices.get())
	{
		laserScan = cv::Mat(1, (int)indices->size(), CV_32FC(4));
		for(unsigned int i=0; i<indices->size(); ++i)
		{
			int index = indices->at(i);
			if(!filterNaNs || pcl::isFinite(cloud.at(index)))
			{
				float * ptr = laserScan.ptr<float>(0, oi++);
				if(!nullTransform)
				{
					pcl::PointXYZRGB pt = pcl::transformPoint(cloud.at(index), transform3f);
					ptr[0] = pt.x;
					ptr[1] = pt.y;
					ptr[2] = pt.z;
				}
				else
				{
					ptr[0] = cloud.at(index).x;
					ptr[1] = cloud.at(index).y;
					ptr[2] = cloud.at(index).z;
				}
				int * ptrInt = (int*)ptr;
				ptrInt[3] = int(cloud.at(index).b) | (int(cloud.at(index).g) << 8) | (int(cloud.at(index).r) << 16);
			}
		}
	}
	else
	{
		laserScan = cv::Mat(1, (int)cloud.size(), CV_32FC(4));
		for(unsigned int i=0; i<cloud.size(); ++i)
		{
			if(!filterNaNs || pcl::isFinite(cloud.at(i)))
			{
				float * ptr = laserScan.ptr<float>(0, oi++);
				if(!nullTransform)
				{
					pcl::PointXYZRGB pt = pcl::transformPoint(cloud.at(i), transform3f);
					ptr[0] = pt.x;
					ptr[1] = pt.y;
					ptr[2] = pt.z;
				}
				else
				{
					ptr[0] = cloud.at(i).x;
					ptr[1] = cloud.at(i).y;
					ptr[2] = cloud.at(i).z;
				}
				int * ptrInt = (int*)ptr;
				ptrInt[3] = int(cloud.at(i).b) | (int(cloud.at(i).g) << 8) | (int(cloud.at(i).r) << 16);
			}
		}
	}
	if(oi == 0)
	{
		return LaserScan();
	}
	return LaserScan(laserScan(cv::Range::all(), cv::Range(0,oi)), 0, 0.0f, LaserScan::kXYZRGB);
}

LaserScan laserScanFromPointCloud(const pcl::PointCloud<pcl::PointXYZI> & cloud, const Transform & transform, bool filterNaNs)
{
	return laserScanFromPointCloud(cloud, pcl::IndicesPtr(), transform, filterNaNs);
}

LaserScan laserScanFromPointCloud(const pcl::PointCloud<pcl::PointXYZI> & cloud, const pcl::IndicesPtr & indices, const Transform & transform, bool filterNaNs)
{
	cv::Mat laserScan;
	bool nullTransform = transform.isNull() || transform.isIdentity();
	Eigen::Affine3f transform3f = transform.toEigen3f();
	int oi=0;
	if(indices.get())
	{
		laserScan = cv::Mat(1, (int)indices->size(), CV_32FC(4));
		for(unsigned int i=0; i<indices->size(); ++i)
		{
			int index = indices->at(i);
			if(!filterNaNs || pcl::isFinite(cloud.at(index)))
			{
				float * ptr = laserScan.ptr<float>(0, oi++);
				if(!nullTransform)
				{
					pcl::PointXYZI pt = pcl::transformPoint(cloud.at(index), transform3f);
					ptr[0] = pt.x;
					ptr[1] = pt.y;
					ptr[2] = pt.z;
				}
				else
				{
					ptr[0] = cloud.at(index).x;
					ptr[1] = cloud.at(index).y;
					ptr[2] = cloud.at(index).z;
				}
				ptr[3] = cloud.at(index).intensity;
			}
		}
	}
	else
	{
		laserScan = cv::Mat(1, (int)cloud.size(), CV_32FC(4));
		for(unsigned int i=0; i<cloud.size(); ++i)
		{
			if(!filterNaNs || pcl::isFinite(cloud.at(i)))
			{
				float * ptr = laserScan.ptr<float>(0, oi++);
				if(!nullTransform)
				{
					pcl::PointXYZI pt = pcl::transformPoint(cloud.at(i), transform3f);
					ptr[0] = pt.x;
					ptr[1] = pt.y;
					ptr[2] = pt.z;
				}
				else
				{
					ptr[0] = cloud.at(i).x;
					ptr[1] = cloud.at(i).y;
					ptr[2] = cloud.at(i).z;
				}
				ptr[3] = cloud.at(i).intensity;
			}
		}
	}
	if(oi == 0)
	{
		return LaserScan();
	}
	return LaserScan(laserScan(cv::Range::all(), cv::Range(0,oi)), 0, 0.0f, LaserScan::kXYZI);
}

LaserScan laserScanFromPointCloud(const pcl::PointCloud<pcl::PointXYZRGB> & cloud, const pcl::PointCloud<pcl::Normal> & normals, const Transform & transform, bool filterNaNs)
{
	UASSERT(cloud.size() == normals.size());
	cv::Mat laserScan(1, (int)cloud.size(), CV_32FC(7));
	bool nullTransform = transform.isNull() || transform.isIdentity();
	int oi = 0;
	for(unsigned int i=0; i<cloud.size(); ++i)
	{
		if(!filterNaNs || pcl::isFinite(cloud.at(i)))
		{
			float * ptr = laserScan.ptr<float>(0, oi++);
			if(!nullTransform)
			{
				pcl::PointXYZRGBNormal pt;
				pt.x = cloud.at(i).x;
				pt.y = cloud.at(i).y;
				pt.z = cloud.at(i).z;
				pt.normal_x = normals.at(i).normal_x;
				pt.normal_y = normals.at(i).normal_y;
				pt.normal_z = normals.at(i).normal_z;
				pt = util3d::transformPoint(pt, transform);
				ptr[0] = pt.x;
				ptr[1] = pt.y;
				ptr[2] = pt.z;
				ptr[4] = pt.normal_x;
				ptr[5] = pt.normal_y;
				ptr[6] = pt.normal_z;
			}
			else
			{
				ptr[0] = cloud.at(i).x;
				ptr[1] = cloud.at(i).y;
				ptr[2] = cloud.at(i).z;
				ptr[4] = normals.at(i).normal_x;
				ptr[5] = normals.at(i).normal_y;
				ptr[6] = normals.at(i).normal_z;
			}
			int * ptrInt = (int*)ptr;
			ptrInt[3] = int(cloud.at(i).b) | (int(cloud.at(i).g) << 8) | (int(cloud.at(i).r) << 16);
		}
	}
	if(oi == 0)
	{
		return LaserScan();
	}
	return LaserScan(laserScan(cv::Range::all(), cv::Range(0,oi)), 0, 0.0f, LaserScan::kXYZRGBNormal);
}

LaserScan laserScanFromPointCloud(const pcl::PointCloud<pcl::PointXYZRGBNormal> & cloud, const Transform & transform, bool filterNaNs)
{
	return laserScanFromPointCloud(cloud, pcl::IndicesPtr(), transform, filterNaNs);
}
LaserScan laserScanFromPointCloud(const pcl::PointCloud<pcl::PointXYZRGBNormal> & cloud, const pcl::IndicesPtr & indices, const Transform & transform, bool filterNaNs)
{
	cv::Mat laserScan;
	bool nullTransform = transform.isNull() || transform.isIdentity();
	int oi = 0;
	if(indices.get())
	{
		laserScan = cv::Mat(1, (int)indices->size(), CV_32FC(7));
		for(unsigned int i=0; i<indices->size(); ++i)
		{
			int index = indices->at(i);
			if(!filterNaNs || (pcl::isFinite(cloud.at(index)) &&
					uIsFinite(cloud.at(index).normal_x) &&
					uIsFinite(cloud.at(index).normal_y) &&
					uIsFinite(cloud.at(index).normal_z)))
			{
				float * ptr = laserScan.ptr<float>(0, oi++);
				if(!nullTransform)
				{
					pcl::PointXYZRGBNormal pt = util3d::transformPoint(cloud.at(index), transform);
					ptr[0] = pt.x;
					ptr[1] = pt.y;
					ptr[2] = pt.z;
					ptr[4] = pt.normal_x;
					ptr[5] = pt.normal_y;
					ptr[6] = pt.normal_z;
				}
				else
				{
					ptr[0] = cloud.at(index).x;
					ptr[1] = cloud.at(index).y;
					ptr[2] = cloud.at(index).z;
					ptr[4] = cloud.at(index).normal_x;
					ptr[5] = cloud.at(index).normal_y;
					ptr[6] = cloud.at(index).normal_z;
				}
				int * ptrInt = (int*)ptr;
				ptrInt[3] = int(cloud.at(index).b) | (int(cloud.at(index).g) << 8) | (int(cloud.at(index).r) << 16);
			}
		}
	}
	else
	{
		laserScan = cv::Mat(1, (int)cloud.size(), CV_32FC(7));
		for(unsigned int i=0; i<cloud.size(); ++i)
		{
			if(!filterNaNs || (pcl::isFinite(cloud.at(i)) &&
					uIsFinite(cloud.at(i).normal_x) &&
					uIsFinite(cloud.at(i).normal_y) &&
					uIsFinite(cloud.at(i).normal_z)))
			{
				float * ptr = laserScan.ptr<float>(0, oi++);
				if(!nullTransform)
				{
					pcl::PointXYZRGBNormal pt = util3d::transformPoint(cloud.at(i), transform);
					ptr[0] = pt.x;
					ptr[1] = pt.y;
					ptr[2] = pt.z;
					ptr[4] = pt.normal_x;
					ptr[5] = pt.normal_y;
					ptr[6] = pt.normal_z;
				}
				else
				{
					ptr[0] = cloud.at(i).x;
					ptr[1] = cloud.at(i).y;
					ptr[2] = cloud.at(i).z;
					ptr[4] = cloud.at(i).normal_x;
					ptr[5] = cloud.at(i).normal_y;
					ptr[6] = cloud.at(i).normal_z;
				}
				int * ptrInt = (int*)ptr;
				ptrInt[3] = int(cloud.at(i).b) | (int(cloud.at(i).g) << 8) | (int(cloud.at(i).r) << 16);
			}
		}
	}
	if(oi == 0)
	{
		return LaserScan();
	}
	return LaserScan(laserScan(cv::Range::all(), cv::Range(0,oi)), 0, 0.0f, LaserScan::kXYZRGBNormal);
}

LaserScan laserScanFromPointCloud(const pcl::PointCloud<pcl::PointXYZI> & cloud, const pcl::PointCloud<pcl::Normal> & normals, const Transform & transform, bool filterNaNs)
{
	UASSERT(cloud.size() == normals.size());
	cv::Mat laserScan(1, (int)cloud.size(), CV_32FC(7));
	bool nullTransform = transform.isNull() || transform.isIdentity();
	int oi=0;
	for(unsigned int i=0; i<cloud.size(); ++i)
	{
		if(!filterNaNs || (pcl::isFinite(cloud.at(i)) && pcl::isFinite(normals.at(i))))
		{
			float * ptr = laserScan.ptr<float>(0, oi++);
			if(!nullTransform)
			{
				pcl::PointXYZINormal pt;
				pt.x = cloud.at(i).x;
				pt.y = cloud.at(i).y;
				pt.z = cloud.at(i).z;
				pt.normal_x = normals.at(i).normal_x;
				pt.normal_y = normals.at(i).normal_y;
				pt.normal_z = normals.at(i).normal_z;
				pt = util3d::transformPoint(pt, transform);
				ptr[0] = pt.x;
				ptr[1] = pt.y;
				ptr[2] = pt.z;
				ptr[4] = pt.normal_x;
				ptr[5] = pt.normal_y;
				ptr[6] = pt.normal_z;
			}
			else
			{
				ptr[0] = cloud.at(i).x;
				ptr[1] = cloud.at(i).y;
				ptr[2] = cloud.at(i).z;
				ptr[4] = normals.at(i).normal_x;
				ptr[5] = normals.at(i).normal_y;
				ptr[6] = normals.at(i).normal_z;
			}
			ptr[3] = cloud.at(i).intensity;
		}
	}
	if(oi == 0)
	{
		return LaserScan();
	}
	return LaserScan(laserScan(cv::Range::all(), cv::Range(0,oi)), 0, 0.0f, LaserScan::kXYZINormal);
}

LaserScan laserScanFromPointCloud(const pcl::PointCloud<pcl::PointXYZINormal> & cloud, const Transform & transform, bool filterNaNs)
{
	return laserScanFromPointCloud(cloud, pcl::IndicesPtr(), transform, filterNaNs);
}
LaserScan laserScanFromPointCloud(const pcl::PointCloud<pcl::PointXYZINormal> & cloud, const pcl::IndicesPtr & indices, const Transform & transform, bool filterNaNs)
{
	cv::Mat laserScan;
	bool nullTransform = transform.isNull() || transform.isIdentity();
	int oi = 0;
	if(indices.get())
	{
		laserScan = cv::Mat(1, (int)indices->size(), CV_32FC(7));
		for(unsigned int i=0; i<indices->size(); ++i)
		{
			int index = indices->at(i);
			if(!filterNaNs || (pcl::isFinite(cloud.at(index)) &&
					uIsFinite(cloud.at(index).normal_x) &&
					uIsFinite(cloud.at(index).normal_y) &&
					uIsFinite(cloud.at(index).normal_z)))
			{
				float * ptr = laserScan.ptr<float>(0, oi++);
				if(!nullTransform)
				{
					pcl::PointXYZINormal pt = util3d::transformPoint(cloud.at(index), transform);
					ptr[0] = pt.x;
					ptr[1] = pt.y;
					ptr[2] = pt.z;
					ptr[4] = pt.normal_x;
					ptr[5] = pt.normal_y;
					ptr[6] = pt.normal_z;
				}
				else
				{
					ptr[0] = cloud.at(index).x;
					ptr[1] = cloud.at(index).y;
					ptr[2] = cloud.at(index).z;
					ptr[4] = cloud.at(index).normal_x;
					ptr[5] = cloud.at(index).normal_y;
					ptr[6] = cloud.at(index).normal_z;
				}
				ptr[3] = cloud.at(i).intensity;
			}
		}
	}
	else
	{
		laserScan = cv::Mat(1, (int)cloud.size(), CV_32FC(7));
		for(unsigned int i=0; i<cloud.size(); ++i)
		{
			if(!filterNaNs || (pcl::isFinite(cloud.at(i)) &&
					uIsFinite(cloud.at(i).normal_x) &&
					uIsFinite(cloud.at(i).normal_y) &&
					uIsFinite(cloud.at(i).normal_z)))
			{
				float * ptr = laserScan.ptr<float>(0, oi++);
				if(!nullTransform)
				{
					pcl::PointXYZINormal pt = util3d::transformPoint(cloud.at(i), transform);
					ptr[0] = pt.x;
					ptr[1] = pt.y;
					ptr[2] = pt.z;
					ptr[4] = pt.normal_x;
					ptr[5] = pt.normal_y;
					ptr[6] = pt.normal_z;
				}
				else
				{
					ptr[0] = cloud.at(i).x;
					ptr[1] = cloud.at(i).y;
					ptr[2] = cloud.at(i).z;
					ptr[4] = cloud.at(i).normal_x;
					ptr[5] = cloud.at(i).normal_y;
					ptr[6] = cloud.at(i).normal_z;
				}
				ptr[3] = cloud.at(i).intensity;
			}
		}
	}
	if(oi == 0)
	{
		return LaserScan();
	}
	return LaserScan(laserScan(cv::Range::all(), cv::Range(0,oi)), 0, 0.0f, LaserScan::kXYZINormal);
}

LaserScan laserScan2dFromPointCloud(const pcl::PointCloud<pcl::PointXYZ> & cloud, const Transform & transform, bool filterNaNs)
{
	cv::Mat laserScan(1, (int)cloud.size(), CV_32FC2);
	bool nullTransform = transform.isNull();
	Eigen::Affine3f transform3f = transform.toEigen3f();
	int oi=0;
	for(unsigned int i=0; i<cloud.size(); ++i)
	{
		if(!filterNaNs || pcl::isFinite(cloud.at(i)))
		{
			float * ptr = laserScan.ptr<float>(0, oi++);
			if(!nullTransform)
			{
				pcl::PointXYZ pt = pcl::transformPoint(cloud.at(i), transform3f);
				ptr[0] = pt.x;
				ptr[1] = pt.y;
			}
			else
			{
				ptr[0] = cloud.at(i).x;
				ptr[1] = cloud.at(i).y;
			}
		}

	}
	if(oi == 0)
	{
		return LaserScan();
	}
	return LaserScan(laserScan(cv::Range::all(), cv::Range(0,oi)), 0, 0.0f, LaserScan::kXY);
}

LaserScan laserScan2dFromPointCloud(const pcl::PointCloud<pcl::PointXYZI> & cloud, const Transform & transform, bool filterNaNs)
{
	cv::Mat laserScan(1, (int)cloud.size(), CV_32FC3);
	bool nullTransform = transform.isNull();
	Eigen::Affine3f transform3f = transform.toEigen3f();
	int oi=0;
	for(unsigned int i=0; i<cloud.size(); ++i)
	{
		if(!filterNaNs || pcl::isFinite(cloud.at(i)))
		{
			float * ptr = laserScan.ptr<float>(0, oi++);
			if(!nullTransform)
			{
				pcl::PointXYZI pt = pcl::transformPoint(cloud.at(i), transform3f);
				ptr[0] = pt.x;
				ptr[1] = pt.y;
				ptr[2] = pt.intensity;
			}
			else
			{
				ptr[0] = cloud.at(i).x;
				ptr[1] = cloud.at(i).y;
				ptr[2] = cloud.at(i).intensity;
			}
		}

	}
	if(oi == 0)
	{
		return LaserScan();
	}
	return LaserScan(laserScan(cv::Range::all(), cv::Range(0,oi)), 0, 0.0f, LaserScan::kXYI);
}

LaserScan laserScan2dFromPointCloud(const pcl::PointCloud<pcl::PointNormal> & cloud, const Transform & transform, bool filterNaNs)
{
	cv::Mat laserScan(1, (int)cloud.size(), CV_32FC(5));
	bool nullTransform = transform.isNull();
	int oi=0;
	for(unsigned int i=0; i<cloud.size(); ++i)
	{
		if(!filterNaNs || (pcl::isFinite(cloud.at(i)) &&
				uIsFinite(cloud.at(i).normal_x) &&
				uIsFinite(cloud.at(i).normal_y) &&
				uIsFinite(cloud.at(i).normal_z)))
		{
			float * ptr = laserScan.ptr<float>(0, oi++);
			if(!nullTransform)
			{
				pcl::PointNormal pt = util3d::transformPoint(cloud.at(i), transform);
				ptr[0] = pt.x;
				ptr[1] = pt.y;
				ptr[2] = pt.normal_x;
				ptr[3] = pt.normal_y;
				ptr[4] = pt.normal_z;
			}
			else
			{
				const pcl::PointNormal & pt = cloud.at(i);
				ptr[0] = pt.x;
				ptr[1] = pt.y;
				ptr[2] = pt.normal_x;
				ptr[3] = pt.normal_y;
				ptr[4] = pt.normal_z;
			}
		}
	}
	if(oi == 0)
	{
		return LaserScan();
	}
	return LaserScan(laserScan(cv::Range::all(), cv::Range(0,oi)), 0, 0.0f, LaserScan::kXYNormal);
}

LaserScan laserScan2dFromPointCloud(const pcl::PointCloud<pcl::PointXYZ> & cloud, const pcl::PointCloud<pcl::Normal> & normals, const Transform & transform, bool filterNaNs)
{
	UASSERT(cloud.size() == normals.size());
	cv::Mat laserScan(1, (int)cloud.size(), CV_32FC(5));
	bool nullTransform = transform.isNull() || transform.isIdentity();
	int oi=0;
	for(unsigned int i=0; i<cloud.size(); ++i)
	{
		if(!filterNaNs || (pcl::isFinite(cloud.at(i)) && pcl::isFinite(normals.at(i))))
		{
			float * ptr = laserScan.ptr<float>(0, oi++);
			if(!nullTransform)
			{
				pcl::PointNormal pt;
				pt.x = cloud.at(i).x;
				pt.y = cloud.at(i).y;
				pt.z = cloud.at(i).z;
				pt.normal_x = normals.at(i).normal_x;
				pt.normal_y = normals.at(i).normal_y;
				pt.normal_z = normals.at(i).normal_z;
				pt = util3d::transformPoint(pt, transform);
				ptr[0] = pt.x;
				ptr[1] = pt.y;
				ptr[2] = pt.normal_x;
				ptr[3] = pt.normal_y;
				ptr[4] = pt.normal_z;
			}
			else
			{
				ptr[0] = cloud.at(i).x;
				ptr[1] = cloud.at(i).y;
				ptr[2] = normals.at(i).normal_x;
				ptr[3] = normals.at(i).normal_y;
				ptr[4] = normals.at(i).normal_z;
			}
		}
	}
	if(oi == 0)
	{
		return LaserScan();
	}
	return LaserScan(laserScan(cv::Range::all(), cv::Range(0,oi)), 0, 0.0f, LaserScan::kXYNormal);
}

LaserScan laserScan2dFromPointCloud(const pcl::PointCloud<pcl::PointXYZINormal> & cloud, const Transform & transform, bool filterNaNs)
{
	cv::Mat laserScan(1, (int)cloud.size(), CV_32FC(6));
	bool nullTransform = transform.isNull();
	int oi=0;
	for(unsigned int i=0; i<cloud.size(); ++i)
	{
		if(!filterNaNs || (pcl::isFinite(cloud.at(i)) &&
				uIsFinite(cloud.at(i).normal_x) &&
				uIsFinite(cloud.at(i).normal_y) &&
				uIsFinite(cloud.at(i).normal_z)))
		{
			float * ptr = laserScan.ptr<float>(0, oi++);
			if(!nullTransform)
			{
				pcl::PointXYZINormal pt = util3d::transformPoint(cloud.at(i), transform);
				ptr[0] = pt.x;
				ptr[1] = pt.y;
				ptr[2] = pt.intensity;
				ptr[3] = pt.normal_x;
				ptr[4] = pt.normal_y;
				ptr[5] = pt.normal_z;
			}
			else
			{
				const pcl::PointXYZINormal & pt = cloud.at(i);
				ptr[0] = pt.x;
				ptr[1] = pt.y;
				ptr[2] = pt.intensity;
				ptr[3] = pt.normal_x;
				ptr[4] = pt.normal_y;
				ptr[5] = pt.normal_z;
			}
		}
	}
	if(oi == 0)
	{
		return LaserScan();
	}
	return LaserScan(laserScan(cv::Range::all(), cv::Range(0,oi)), 0, 0.0f, LaserScan::kXYINormal);
}

LaserScan laserScan2dFromPointCloud(const pcl::PointCloud<pcl::PointXYZI> & cloud, const pcl::PointCloud<pcl::Normal> & normals, const Transform & transform, bool filterNaNs)
{
	UASSERT(cloud.size() == normals.size());
	cv::Mat laserScan(1, (int)cloud.size(), CV_32FC(6));
	bool nullTransform = transform.isNull() || transform.isIdentity();
	int oi=0;
	for(unsigned int i=0; i<cloud.size(); ++i)
	{
		if(!filterNaNs || (pcl::isFinite(cloud.at(i)) && pcl::isFinite(normals.at(i))))
		{
			float * ptr = laserScan.ptr<float>(0, oi++);
			if(!nullTransform)
			{
				pcl::PointXYZINormal pt;
				pt.x = cloud.at(i).x;
				pt.y = cloud.at(i).y;
				pt.z = cloud.at(i).z;
				pt.normal_x = normals.at(i).normal_x;
				pt.normal_y = normals.at(i).normal_y;
				pt.normal_z = normals.at(i).normal_z;
				pt = util3d::transformPoint(pt, transform);
				ptr[0] = pt.x;
				ptr[1] = pt.y;
				ptr[2] = pt.intensity;
				ptr[3] = pt.normal_x;
				ptr[4] = pt.normal_y;
				ptr[5] = pt.normal_z;
			}
			else
			{
				ptr[0] = cloud.at(i).x;
				ptr[1] = cloud.at(i).y;
				ptr[2] = cloud.at(i).intensity;
				ptr[3] = normals.at(i).normal_x;
				ptr[4] = normals.at(i).normal_y;
				ptr[5] = normals.at(i).normal_z;
			}
		}
	}
	if(oi == 0)
	{
		return LaserScan();
	}
	return LaserScan(laserScan(cv::Range::all(), cv::Range(0,oi)), 0, 0.0f, LaserScan::kXYINormal);
}

pcl::PCLPointCloud2::Ptr laserScanToPointCloud2(const LaserScan & laserScan, const Transform & transform)
{
	pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2);
	if(laserScan.isEmpty())
	{
		return cloud;
	}

	if(laserScan.format() == LaserScan::kXY || laserScan.format() == LaserScan::kXYZ)
	{
		pcl::toPCLPointCloud2(*laserScanToPointCloud(laserScan, transform), *cloud);
	}
	else if(laserScan.format() == LaserScan::kXYI || laserScan.format() == LaserScan::kXYZI)
	{
		pcl::toPCLPointCloud2(*laserScanToPointCloudI(laserScan, transform), *cloud);
	}
	else if(laserScan.format() == LaserScan::kXYNormal || laserScan.format() == LaserScan::kXYZNormal)
	{
		pcl::toPCLPointCloud2(*laserScanToPointCloudNormal(laserScan, transform), *cloud);
	}
	else if(laserScan.format() == LaserScan::kXYINormal || laserScan.format() == LaserScan::kXYZINormal)
	{
		pcl::toPCLPointCloud2(*laserScanToPointCloudINormal(laserScan, transform), *cloud);
	}
	else if(laserScan.format() == LaserScan::kXYZRGB)
	{
		pcl::toPCLPointCloud2(*laserScanToPointCloudRGB(laserScan, transform), *cloud);
	}
	else if(laserScan.format() == LaserScan::kXYZRGBNormal)
	{
		pcl::toPCLPointCloud2(*laserScanToPointCloudRGBNormal(laserScan, transform), *cloud);
	}
	else
	{
		UERROR("Unknown conversion from LaserScan format %d to PointCloud2.", laserScan.format());
	}
	return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr laserScanToPointCloud(const LaserScan & laserScan, const Transform & transform)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
	output->resize(laserScan.size());
	output->is_dense = true;
	bool nullTransform = transform.isNull();
	Eigen::Affine3f transform3f = transform.toEigen3f();
	for(int i=0; i<laserScan.size(); ++i)
	{
		output->at(i) = util3d::laserScanToPoint(laserScan, i);
		if(!nullTransform)
		{
			output->at(i) = pcl::transformPoint(output->at(i), transform3f);
		}
	}
	return output;
}

pcl::PointCloud<pcl::PointNormal>::Ptr laserScanToPointCloudNormal(const LaserScan & laserScan, const Transform & transform)
{
	pcl::PointCloud<pcl::PointNormal>::Ptr output(new pcl::PointCloud<pcl::PointNormal>);
	output->resize(laserScan.size());
	output->is_dense = true;
	bool nullTransform = transform.isNull();
	for(int i=0; i<laserScan.size(); ++i)
	{
		output->at(i) = laserScanToPointNormal(laserScan, i);
		if(!nullTransform)
		{
			output->at(i) = util3d::transformPoint(output->at(i), transform);
		}
	}
	return output;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr laserScanToPointCloudRGB(const LaserScan & laserScan, const Transform & transform,  unsigned char r, unsigned char g, unsigned char b)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGB>);
	output->resize(laserScan.size());
	output->is_dense = true;
	bool nullTransform = transform.isNull() || transform.isIdentity();
	Eigen::Affine3f transform3f = transform.toEigen3f();
	for(int i=0; i<laserScan.size(); ++i)
	{
		output->at(i) = util3d::laserScanToPointRGB(laserScan, i, r, g, b);
		if(!nullTransform)
		{
			output->at(i) = pcl::transformPoint(output->at(i), transform3f);
		}
	}
	return output;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr laserScanToPointCloudI(const LaserScan & laserScan, const Transform & transform,  float intensity)
{
	pcl::PointCloud<pcl::PointXYZI>::Ptr output(new pcl::PointCloud<pcl::PointXYZI>);
	output->resize(laserScan.size());
	output->is_dense = true;
	bool nullTransform = transform.isNull() || transform.isIdentity();
	Eigen::Affine3f transform3f = transform.toEigen3f();
	for(int i=0; i<laserScan.size(); ++i)
	{
		output->at(i) = util3d::laserScanToPointI(laserScan, i, intensity);
		if(!nullTransform)
		{
			output->at(i) = pcl::transformPoint(output->at(i), transform3f);
		}
	}
	return output;
}

pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr laserScanToPointCloudRGBNormal(const LaserScan & laserScan, const Transform & transform,  unsigned char r, unsigned char g, unsigned char b)
{
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	output->resize(laserScan.size());
	output->is_dense = true;
	bool nullTransform = transform.isNull() || transform.isIdentity();
	for(int i=0; i<laserScan.size(); ++i)
	{
		output->at(i) = util3d::laserScanToPointRGBNormal(laserScan, i, r, g, b);
		if(!nullTransform)
		{
			output->at(i) = util3d::transformPoint(output->at(i), transform);
		}
	}
	return output;
}

pcl::PointCloud<pcl::PointXYZINormal>::Ptr laserScanToPointCloudINormal(const LaserScan & laserScan, const Transform & transform,  float intensity)
{
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr output(new pcl::PointCloud<pcl::PointXYZINormal>);
	output->resize(laserScan.size());
	output->is_dense = true;
	bool nullTransform = transform.isNull() || transform.isIdentity();
	for(int i=0; i<laserScan.size(); ++i)
	{
		output->at(i) = util3d::laserScanToPointINormal(laserScan, i, intensity);
		if(!nullTransform)
		{
			output->at(i) = util3d::transformPoint(output->at(i), transform);
		}
	}
	return output;
}

pcl::PointXYZ laserScanToPoint(const LaserScan & laserScan, int index)
{
	UASSERT(!laserScan.isEmpty() && !laserScan.isCompressed() && index < laserScan.size());
	pcl::PointXYZ output;
	const float * ptr = laserScan.data().ptr<float>(0, index);
	output.x = ptr[0];
	output.y = ptr[1];
	if(!laserScan.is2d())
	{
		output.z = ptr[2];
	}
	return output;
}

pcl::PointNormal laserScanToPointNormal(const LaserScan & laserScan, int index)
{
	UASSERT(!laserScan.isEmpty() && !laserScan.isCompressed() && index < laserScan.size());
	pcl::PointNormal output;
	const float * ptr = laserScan.data().ptr<float>(0, index);
	output.x = ptr[0];
	output.y = ptr[1];
	if(!laserScan.is2d())
	{
		output.z = ptr[2];
	}
	if(laserScan.hasNormals())
	{
		int offset = laserScan.getNormalsOffset();
		output.normal_x = ptr[offset];
		output.normal_y = ptr[offset+1];
		output.normal_z = ptr[offset+2];
	}
	return output;
}

pcl::PointXYZRGB laserScanToPointRGB(const LaserScan & laserScan, int index, unsigned char r, unsigned char g, unsigned char b)
{
	UASSERT(!laserScan.isEmpty() && !laserScan.isCompressed() && index < laserScan.size());
	pcl::PointXYZRGB output;
	const float * ptr = laserScan.data().ptr<float>(0, index);
	output.x = ptr[0];
	output.y = ptr[1];
	if(!laserScan.is2d())
	{
		output.z = ptr[2];
	}

	if(laserScan.hasRGB())
	{
		int * ptrInt = (int*)ptr;
		int indexRGB = laserScan.getRGBOffset();
		output.b = (unsigned char)(ptrInt[indexRGB] & 0xFF);
		output.g = (unsigned char)((ptrInt[indexRGB] >> 8) & 0xFF);
		output.r = (unsigned char)((ptrInt[indexRGB] >> 16) & 0xFF);
	}
	else if(laserScan.hasIntensity())
	{
		// package intensity float -> rgba
		int * ptrInt = (int*)ptr;
		int indexIntensity = laserScan.getIntensityOffset();
		output.r = (unsigned char)(ptrInt[indexIntensity] & 0xFF);
		output.g = (unsigned char)((ptrInt[indexIntensity] >> 8) & 0xFF);
		output.b = (unsigned char)((ptrInt[indexIntensity] >> 16) & 0xFF);
		output.a = (unsigned char)((ptrInt[indexIntensity] >> 24) & 0xFF);
	}
	else
	{
		output.r = r;
		output.g = g;
		output.b = b;
	}
	return output;
}

pcl::PointXYZI laserScanToPointI(const LaserScan & laserScan, int index, float intensity)
{
	UASSERT(!laserScan.isEmpty() && !laserScan.isCompressed() && index < laserScan.size());
	pcl::PointXYZI output;
	const float * ptr = laserScan.data().ptr<float>(0, index);
	output.x = ptr[0];
	output.y = ptr[1];
	if(!laserScan.is2d())
	{
		output.z = ptr[2];
	}

	if(laserScan.hasIntensity())
	{
		int offset = laserScan.getIntensityOffset();
		output.intensity = ptr[offset];
	}
	else
	{
		output.intensity = intensity;
	}

	return output;
}

pcl::PointXYZRGBNormal laserScanToPointRGBNormal(const LaserScan & laserScan, int index, unsigned char r, unsigned char g, unsigned char b)
{
	UASSERT(!laserScan.isEmpty() && !laserScan.isCompressed() && index < laserScan.size());
	pcl::PointXYZRGBNormal output;
	const float * ptr = laserScan.data().ptr<float>(0, index);
	output.x = ptr[0];
	output.y = ptr[1];
	if(!laserScan.is2d())
	{
		output.z = ptr[2];
	}

	if(laserScan.hasRGB())
	{
		int * ptrInt = (int*)ptr;
		int indexRGB = laserScan.getRGBOffset();
		output.b = (unsigned char)(ptrInt[indexRGB] & 0xFF);
		output.g = (unsigned char)((ptrInt[indexRGB] >> 8) & 0xFF);
		output.r = (unsigned char)((ptrInt[indexRGB] >> 16) & 0xFF);
	}
	else if(laserScan.hasIntensity())
	{
		int * ptrInt = (int*)ptr;
		int indexIntensity = laserScan.getIntensityOffset();
		output.r = (unsigned char)(ptrInt[indexIntensity] & 0xFF);
		output.g = (unsigned char)((ptrInt[indexIntensity] >> 8) & 0xFF);
		output.b = (unsigned char)((ptrInt[indexIntensity] >> 16) & 0xFF);
		output.a = (unsigned char)((ptrInt[indexIntensity] >> 24) & 0xFF);
	}
	else
	{
		output.r = r;
		output.g = g;
		output.b = b;
	}

	if(laserScan.hasNormals())
	{
		int offset = laserScan.getNormalsOffset();
		output.normal_x = ptr[offset];
		output.normal_y = ptr[offset+1];
		output.normal_z = ptr[offset+2];
	}

	return output;
}

pcl::PointXYZINormal laserScanToPointINormal(const LaserScan & laserScan, int index, float intensity)
{
	UASSERT(!laserScan.isEmpty() && !laserScan.isCompressed() && index < laserScan.size());
	pcl::PointXYZINormal output;
	const float * ptr = laserScan.data().ptr<float>(0, index);
	output.x = ptr[0];
	output.y = ptr[1];
	if(!laserScan.is2d())
	{
		output.z = ptr[2];
	}

	if(laserScan.hasIntensity())
	{
		int offset = laserScan.getIntensityOffset();
		output.intensity = ptr[offset];
	}
	else
	{
		output.intensity = intensity;
	}

	if(laserScan.hasNormals())
	{
		int offset = laserScan.getNormalsOffset();
		output.normal_x = ptr[offset];
		output.normal_y = ptr[offset+1];
		output.normal_z = ptr[offset+2];
	}

	return output;
}

void getMinMax3D(const cv::Mat & laserScan, cv::Point3f & min, cv::Point3f & max)
{
	UASSERT(!laserScan.empty());
	UASSERT(laserScan.type() == CV_32FC2 || laserScan.type() == CV_32FC3 || laserScan.type() == CV_32FC(4) || laserScan.type() == CV_32FC(5) || laserScan.type() == CV_32FC(6) || laserScan.type() == CV_32FC(7));

	const float * ptr = laserScan.ptr<float>(0, 0);
	min.x = max.x = ptr[0];
	min.y = max.y = ptr[1];
	bool is3d = laserScan.channels() >= 3 && laserScan.channels() != 5;
	min.z = max.z = is3d?ptr[2]:0.0f;
	for(int i=1; i<laserScan.cols; ++i)
	{
		ptr = laserScan.ptr<float>(0, i);

		if(ptr[0] < min.x) min.x = ptr[0];
		else if(ptr[0] > max.x) max.x = ptr[0];

		if(ptr[1] < min.y) min.y = ptr[1];
		else if(ptr[1] > max.y) max.y = ptr[1];

		if(is3d)
		{
			if(ptr[2] < min.z) min.z = ptr[2];
			else if(ptr[2] > max.z) max.z = ptr[2];
		}
	}
}
void getMinMax3D(const cv::Mat & laserScan, pcl::PointXYZ & min, pcl::PointXYZ & max)
{
	cv::Point3f minCV, maxCV;
	getMinMax3D(laserScan, minCV, maxCV);
	min.x = minCV.x;
	min.y = minCV.y;
	min.z = minCV.z;
	max.x = maxCV.x;
	max.y = maxCV.y;
	max.z = maxCV.z;
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
		const cv::Mat & cameraMatrixK,
		const cv::Mat & laserScan,     // assuming laser scan points are already in /base_link coordinate
		const rtabmap::Transform & cameraTransform) // /base_link -> /camera_link
{
	UASSERT(!cameraTransform.isNull());
	UASSERT(!laserScan.empty());
	UASSERT(laserScan.type() == CV_32FC2 || laserScan.type() == CV_32FC3 || laserScan.type() == CV_32FC(4) || laserScan.type() == CV_32FC(5) || laserScan.type() == CV_32FC(6) || laserScan.type() == CV_32FC(7));
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
		const float* ptr = laserScan.ptr<float>(0, i);

		// Get 3D from laser scan
		cv::Point3f ptScan;
		if(laserScan.type() == CV_32FC2 || laserScan.type() == CV_32FC(5))
		{
			// 2D scans
			ptScan.x = ptr[0];
			ptScan.y = ptr[1];
			ptScan.z = 0;
		}
		else // 3D scans
		{
			ptScan.x = ptr[0];
			ptScan.y = ptr[1];
			ptScan.z = ptr[2];
		}
		ptScan = util3d::transformPoint(ptScan, t);

		// re-project in camera frame
		float z = ptScan.z;

		bool set = false;
		if(z > 0.0f)
		{
			float invZ = 1.0f/z;
			float dx = (fx*ptScan.x)*invZ + cx;
			float dy = (fy*ptScan.y)*invZ + cy;
			int dx_low = dx;
			int dy_low = dy;
			int dx_high = dx + 0.5f;
			int dy_high = dy + 0.5f;

			if(uIsInBounds(dx_low, 0, registered.cols) && uIsInBounds(dy_low, 0, registered.rows))
			{
				float &zReg = registered.at<float>(dy_low, dx_low);
				if(zReg == 0 || z < zReg)
				{
					zReg = z;
				}
				set = true;
			}
			if((dx_low != dx_high || dy_low != dy_high) &&
				uIsInBounds(dx_high, 0, registered.cols) && uIsInBounds(dy_high, 0, registered.rows))
			{
				float &zReg = registered.at<float>(dy_high, dx_high);
				if(zReg == 0 || z < zReg)
				{
					zReg = z;
				}
				set = true;
			}
		}
		if(set)
		{
			count++;
		}
	}
	UDEBUG("Points in camera=%d/%d", count, laserScan.cols);

	return registered;
}

cv::Mat projectCloudToCamera(
		const cv::Size & imageSize,
		const cv::Mat & cameraMatrixK,
		const pcl::PointCloud<pcl::PointXYZ>::Ptr laserScan,  // assuming points are already in /base_link coordinate
		const rtabmap::Transform & cameraTransform)           // /base_link -> /camera_link
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
		bool set = false;
		if(z > 0.0f)
		{
			float invZ = 1.0f/z;
			float dx = (fx*ptScan.x)*invZ + cx;
			float dy = (fy*ptScan.y)*invZ + cy;
			int dx_low = dx;
			int dy_low = dy;
			int dx_high = dx + 0.5f;
			int dy_high = dy + 0.5f;
			if(uIsInBounds(dx_low, 0, registered.cols) && uIsInBounds(dy_low, 0, registered.rows))
			{
				set = true;
				float &zReg = registered.at<float>(dy_low, dx_low);
				if(zReg == 0 || z < zReg)
				{
					zReg = z;
				}
			}
			if((dx_low != dx_high || dy_low != dy_high) &&
				uIsInBounds(dx_high, 0, registered.cols) && uIsInBounds(dy_high, 0, registered.rows))
			{
				set = true;
				float &zReg = registered.at<float>(dy_high, dx_high);
				if(zReg == 0 || z < zReg)
				{
					zReg = z;
				}
			}
		}
		if(set)
		{
			count++;
		}
	}
	UDEBUG("Points in camera=%d/%d", count, (int)laserScan->size());

	return registered;
}

cv::Mat projectCloudToCamera(
		const cv::Size & imageSize,
		const cv::Mat & cameraMatrixK,
		const pcl::PCLPointCloud2::Ptr laserScan,  // assuming points are already in /base_link coordinate
		const rtabmap::Transform & cameraTransform)           // /base_link -> /camera_link
{
	UASSERT(!cameraTransform.isNull());
	UASSERT(!laserScan->data.empty());
	UASSERT(cameraMatrixK.type() == CV_64FC1 && cameraMatrixK.cols == 3 && cameraMatrixK.cols == 3);

	float fx = cameraMatrixK.at<double>(0,0);
	float fy = cameraMatrixK.at<double>(1,1);
	float cx = cameraMatrixK.at<double>(0,2);
	float cy = cameraMatrixK.at<double>(1,2);

	cv::Mat registered = cv::Mat::zeros(imageSize, CV_32FC1);
	Transform t = cameraTransform.inverse();

	pcl::MsgFieldMap field_map;
	pcl::createMapping<pcl::PointXYZ> (laserScan->fields, field_map);

	int count = 0;
	if(field_map.size() == 1)
	{
		for (uint32_t row = 0; row < (uint32_t)laserScan->height; ++row)
		{
			const uint8_t* row_data = &laserScan->data[row * laserScan->row_step];
			for (uint32_t col = 0; col < (uint32_t)laserScan->width; ++col)
			{
				const uint8_t* msg_data = row_data + col * laserScan->point_step;
				pcl::PointXYZ ptScan;
				memcpy (&ptScan, msg_data + field_map.front().serialized_offset, field_map.front().size);
				ptScan = util3d::transformPoint(ptScan, t);

				// re-project in camera frame
				float z = ptScan.z;
				bool set = false;
				if(z > 0.0f)
				{
					float invZ = 1.0f/z;
					float dx = (fx*ptScan.x)*invZ + cx;
					float dy = (fy*ptScan.y)*invZ + cy;
					int dx_low = dx;
					int dy_low = dy;
					int dx_high = dx + 0.5f;
					int dy_high = dy + 0.5f;
					if(uIsInBounds(dx_low, 0, registered.cols) && uIsInBounds(dy_low, 0, registered.rows))
					{
						set = true;
						float &zReg = registered.at<float>(dy_low, dx_low);
						if(zReg == 0 || z < zReg)
						{
							zReg = z;
						}
					}
					if((dx_low != dx_high || dy_low != dy_high) &&
						uIsInBounds(dx_high, 0, registered.cols) && uIsInBounds(dy_high, 0, registered.rows))
					{
						set = true;
						float &zReg = registered.at<float>(dy_high, dx_high);
						if(zReg == 0 || z < zReg)
						{
							zReg = z;
						}
					}
				}
				if(set)
				{
					count++;
				}
			}
		}
	}
	else
	{
		UERROR("field map pcl::pointXYZ not found!");
	}
	UDEBUG("Points in camera=%d/%d", count, (int)laserScan->data.size());

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

class ProjectionInfo {
public:
	ProjectionInfo():
		nodeID(-1),
		cameraIndex(-1),
		distance(-1)
	{}
	int nodeID;
	int cameraIndex;
	pcl::PointXY uv;
	float distance;
};

/**
 * For each point, return pixel of the best camera (NodeID->CameraIndex)
 * looking at it based on the policy and parameters
 */
template<class PointT>
std::vector<std::pair< std::pair<int, int>, pcl::PointXY> > projectCloudToCamerasImpl (
		const typename pcl::PointCloud<PointT> & cloud,
		const std::map<int, Transform> & cameraPoses,
		const std::map<int, std::vector<CameraModel> > & cameraModels,
		float maxDistance,
		float maxAngle,
		const std::vector<float> & roiRatios,
		const cv::Mat & projMask,
		bool distanceToCamPolicy,
		const ProgressState * state)
{
	UINFO("cloud=%d points", (int)cloud.size());
	UINFO("cameraPoses=%d", (int)cameraPoses.size());
	UINFO("cameraModels=%d", (int)cameraModels.size());
	UINFO("maxDistance=%f", maxDistance);
	UINFO("maxAngle=%f", maxAngle);
	UINFO("distanceToCamPolicy=%s", distanceToCamPolicy?"true":"false");
	UINFO("roiRatios=%s", roiRatios.size() == 4?uFormat("%f %f %f %f", roiRatios[0], roiRatios[1], roiRatios[2], roiRatios[3]):"");
	UINFO("projMask=%dx%d", projMask.cols, projMask.rows);
	std::vector<std::pair< std::pair<int, int>, pcl::PointXY> > pointToPixel;

	if (cloud.empty() || cameraPoses.empty() || cameraModels.empty())
		return pointToPixel;

	std::string msg = uFormat("Computing visible points per cam (%d points, %d cams)", (int)cloud.size(), (int)cameraPoses.size());
	UINFO(msg.c_str());
	if(state && !state->callback(msg))
	{
		//cancelled!
		UWARN("Projecting to cameras cancelled!");
		return pointToPixel;
	}

	std::vector<ProjectionInfo> invertedIndex(cloud.size()); // For each point: list of cameras
	int cameraProcessed = 0;
	bool wrongMaskFormatWarned = false;
	for(std::map<int, Transform>::const_iterator pter = cameraPoses.lower_bound(0); pter!=cameraPoses.end(); ++pter)
	{
		std::map<int, std::vector<CameraModel> >::const_iterator iter=cameraModels.find(pter->first);
		if(iter!=cameraModels.end() && !iter->second.empty())
		{
			cv::Mat validProjMask;
			if(!projMask.empty())
			{
				if(projMask.type() != CV_8UC1)
				{
					if(!wrongMaskFormatWarned)
						UERROR("Wrong camera projection mask type %d, should be CV_8UC1", projMask.type());
					wrongMaskFormatWarned = true;
				}
				else if(projMask.cols == iter->second[0].imageWidth() * (int)iter->second.size() &&
				        projMask.rows == iter->second[0].imageHeight())
				{
					validProjMask = projMask;
				}
				else
				{
					UWARN("Camera projection mask (%dx%d) is not valid for current "
						   "camera model(s) (count=%ld, image size=%dx%d). It will be "
						   "ignored for node %d",
						   projMask.cols, projMask.rows,
						   iter->second.size(),
						   iter->second[0].imageWidth(),
						   iter->second[0].imageHeight(),
						   pter->first);
				}
			}

			for(size_t camIndex=0; camIndex<iter->second.size(); ++camIndex)
			{
				Transform cameraTransform = (pter->second * iter->second[camIndex].localTransform());
				UASSERT(!cameraTransform.isNull());
				cv::Mat cameraMatrixK = iter->second[camIndex].K();
				UASSERT(cameraMatrixK.type() == CV_64FC1 && cameraMatrixK.cols == 3 && cameraMatrixK.cols == 3);
				const cv::Size & imageSize = iter->second[camIndex].imageSize();

				float fx = cameraMatrixK.at<double>(0,0);
				float fy = cameraMatrixK.at<double>(1,1);
				float cx = cameraMatrixK.at<double>(0,2);
				float cy = cameraMatrixK.at<double>(1,2);

				// depth: 2 channels UINT: [depthMM, indexPt]
				cv::Mat registered = cv::Mat::zeros(imageSize, CV_32SC2);
				Transform t = cameraTransform.inverse();

				cv::Rect roi(0,0,imageSize.width, imageSize.height);
				if(roiRatios.size()==4)
				{
					roi = util2d::computeRoi(imageSize, roiRatios);
				}

				int count = 0;
				for(size_t i=0; i<cloud.size(); ++i)
				{
					// Get 3D from laser scan
					PointT ptScan = cloud.at(i);
					ptScan = util3d::transformPoint(ptScan, t);

					// re-project in camera frame
					float z = ptScan.z;
					bool set = false;
					if(z > 0.0f && (maxDistance<=0 || z<maxDistance))
					{
						float invZ = 1.0f/z;
						float dx = (fx*ptScan.x)*invZ + cx;
						float dy = (fy*ptScan.y)*invZ + cy;
						int dx_low = dx;
						int dy_low = dy;
						int dx_high = dx + 0.5f;
						int dy_high = dy + 0.5f;
						int zMM = z * 1000;
						if(uIsInBounds(dx_low, roi.x, roi.x+roi.width) && uIsInBounds(dy_low, roi.y, roi.y+roi.height) &&
						   (validProjMask.empty() || validProjMask.at<unsigned char>(dy_low, imageSize.width*camIndex+dx_low) > 0))
						{
							set = true;
							cv::Vec2i &zReg = registered.at<cv::Vec2i>(dy_low, dx_low);
							if(zReg[0] == 0 || zMM < zReg[0])
							{
								zReg[0] = zMM;
								zReg[1] = i;
							}
						}
						if((dx_low != dx_high || dy_low != dy_high) &&
							uIsInBounds(dx_high, roi.x, roi.x+roi.width) && uIsInBounds(dy_high, roi.y, roi.y+roi.height) &&
							(validProjMask.empty() || validProjMask.at<unsigned char>(dy_high, imageSize.width*camIndex+dx_high) > 0))
						{
							set = true;
							cv::Vec2i &zReg = registered.at<cv::Vec2i>(dy_high, dx_high);
							if(zReg[0] == 0 || zMM < zReg[0])
							{
								zReg[0] = zMM;
								zReg[1] = i;
							}
						}
					}
					if(set)
					{
						count++;
					}
				}
				if(count == 0)
				{
					registered = cv::Mat();
					UINFO("No points projected in camera %d/%d", pter->first, camIndex);
				}
				else
				{
					UDEBUG("%d points projected in camera %d/%d", count, pter->first, camIndex);
				}
				for(int u=0; u<registered.cols; ++u)
				{
					for(int v=0; v<registered.rows; ++v)
					{
						cv::Vec2i &zReg = registered.at<cv::Vec2i>(v, u);
						if(zReg[0] > 0)
						{
							ProjectionInfo info;
							info.nodeID = pter->first;
							info.cameraIndex = camIndex;
							info.uv.x = float(u)/float(imageSize.width);
							info.uv.y = float(v)/float(imageSize.height);
							const Transform & cam = cameraPoses.at(info.nodeID);
							const PointT & pt = cloud.at(zReg[1]);
							Eigen::Vector4f camDir(cam.x()-pt.x, cam.y()-pt.y, cam.z()-pt.z, 0);
							Eigen::Vector4f normal(pt.normal_x, pt.normal_y, pt.normal_z, 0);
							float angleToCam = maxAngle<=0?0:pcl::getAngle3D(normal, camDir);
							float distanceToCam = zReg[0]/1000.0f;
							if( (maxAngle<=0 || (camDir.dot(normal) > 0 && angleToCam < maxAngle)) && // is facing camera? is point normal perpendicular to camera?
								(maxDistance<=0 || distanceToCam<maxDistance)) // is point not too far from camera?
							{
								float vx = info.uv.x-0.5f;
								float vy = info.uv.y-0.5f;

								float distanceToCenter = vx*vx+vy*vy;
								float distance = distanceToCenter;
								if(distanceToCamPolicy)
								{
									distance = distanceToCam;
								}

								info.distance = distance;

								if(invertedIndex[zReg[1]].distance != -1.0f)
								{
									if(distance <= invertedIndex[zReg[1]].distance)
									{
										invertedIndex[zReg[1]] = info;
									}
								}
								else
								{
									invertedIndex[zReg[1]] = info;
								}
							}
						}
					}
				}
			}
		}

		msg = uFormat("Processed camera %d/%d", (int)cameraProcessed+1, (int)cameraPoses.size());
		UINFO(msg.c_str());
		if(state && !state->callback(msg))
		{
			//cancelled!
			UWARN("Projecting to cameras cancelled!");
			return pointToPixel;
		}
		++cameraProcessed;
	}

	msg = uFormat("Select best camera for %d points...", (int)cloud.size());
	UINFO(msg.c_str());
	if(state && !state->callback(msg))
	{
		//cancelled!
		UWARN("Projecting to cameras cancelled!");
		return pointToPixel;
	}

	pointToPixel.resize(invertedIndex.size());
	int colorized = 0;

	// For each point
	for(size_t i=0; i<invertedIndex.size(); ++i)
	{
		int nodeID = -1;
		int cameraIndex = -1;
		pcl::PointXY uv_coords;
		if(invertedIndex[i].distance > -1.0f)
		{
			nodeID = invertedIndex[i].nodeID;
			cameraIndex = invertedIndex[i].cameraIndex;
			uv_coords = invertedIndex[i].uv;
		}

		if(nodeID>-1 && cameraIndex> -1)
		{
			pointToPixel[i].first.first = nodeID;
			pointToPixel[i].first.second = cameraIndex;
			pointToPixel[i].second = uv_coords;
			++colorized;
		}
	}

	msg = uFormat("Process %d points...done! (%d [%d%%] projected in cameras)", (int)cloud.size(), colorized, colorized*100/cloud.size());
	UINFO(msg.c_str());
	if(state)
	{
		state->callback(msg);
	}

	return pointToPixel;
}

std::vector<std::pair< std::pair<int, int>, pcl::PointXY> > projectCloudToCameras (
		const typename pcl::PointCloud<pcl::PointXYZRGBNormal> & cloud,
		const std::map<int, Transform> & cameraPoses,
		const std::map<int, std::vector<CameraModel> > & cameraModels,
		float maxDistance,
		float maxAngle,
		const std::vector<float> & roiRatios,
		const cv::Mat & projMask,
		bool distanceToCamPolicy,
		const ProgressState * state)
{
	return projectCloudToCamerasImpl(cloud,
			cameraPoses,
			cameraModels,
			maxDistance,
			maxAngle,
			roiRatios,
			projMask,
			distanceToCamPolicy,
			state);
}

std::vector<std::pair< std::pair<int, int>, pcl::PointXY> > projectCloudToCameras (
		const typename pcl::PointCloud<pcl::PointXYZINormal> & cloud,
		const std::map<int, Transform> & cameraPoses,
		const std::map<int, std::vector<CameraModel> > & cameraModels,
		float maxDistance,
		float maxAngle,
		const std::vector<float> & roiRatios,
		const cv::Mat & projMask,
		bool distanceToCamPolicy,
		const ProgressState * state)
{
	return projectCloudToCamerasImpl(cloud,
			cameraPoses,
			cameraModels,
			maxDistance,
			maxAngle,
			roiRatios,
			projMask,
			distanceToCamPolicy,
			state);
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

cv::Mat loadBINScan(const std::string & fileName)
{
	cv::Mat output;
	long bytes = UFile::length(fileName);
	if(bytes)
	{
		int dim = 4;
		UASSERT(bytes % sizeof(float) == 0);
		size_t num = bytes/sizeof(float);
		UASSERT(num % dim == 0);
		output = cv::Mat(1, num/dim, CV_32FC(dim));

		// load point cloud
		FILE *stream;
		stream = fopen (fileName.c_str(),"rb");
		size_t actualReadNum = fread(output.data,sizeof(float),num,stream);
		UASSERT(num == actualReadNum);
		fclose(stream);
	}

	return output;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr loadBINCloud(const std::string & fileName)
{
	return laserScanToPointCloud(loadScan(fileName));
}

pcl::PointCloud<pcl::PointXYZ>::Ptr loadBINCloud(const std::string & fileName, int dim)
{
	return loadBINCloud(fileName);
}

LaserScan loadScan(const std::string & path)
{
	std::string fileName = UFile::getName(path);
	if(UFile::getExtension(fileName).compare("bin") == 0)
	{
		return LaserScan(loadBINScan(path), 0, 0, LaserScan::kXYZI);
	}
	else
	{
		pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2);

		if(UFile::getExtension(fileName).compare("pcd") == 0)
		{
			pcl::io::loadPCDFile(path, *cloud);
		}
		else // PLY
		{
			pcl::io::loadPLYFile(path, *cloud);
		}
		if(cloud->height > 1)
		{
			cloud->is_dense = false;
		}

		bool is2D = false;
		if(!cloud->data.empty())
		{
			// If all z values are zeros, we assume it is a 2D scan
			int zOffset = -1;
			for(unsigned int i=0; i<cloud->fields.size(); ++i)
			{
				if(cloud->fields[i].name.compare("z") == 0)
				{
					zOffset = cloud->fields[i].offset;
					break;
				}
			}
			if(zOffset>=0)
			{
				is2D = true;
				for (uint32_t row = 0; row < (uint32_t)cloud->height && is2D; ++row)
				{
					const uint8_t* row_data = &cloud->data[row * cloud->row_step];
					for (uint32_t col = 0; col < (uint32_t)cloud->width && is2D; ++col)
					{
						const uint8_t* msg_data = row_data + col * cloud->point_step;
						float z = *(float*)(msg_data + zOffset);
						is2D = z == 0.0f;
					}
				}
			}
		}

		return laserScanFromPointCloud(*cloud, true, is2D);
	}
	return LaserScan();
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
		cloud = util3d::loadBINCloud(path); // Assume KITTI velodyne format
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
