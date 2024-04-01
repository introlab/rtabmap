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

#include "rtabmap/core/util3d_transforms.h"

#include <pcl/common/transforms.h>
#include <rtabmap/utilite/ULogger.h>

namespace rtabmap
{

namespace util3d
{

LaserScan transformLaserScan(const LaserScan & laserScan, const Transform & transform)
{
	cv::Mat output = laserScan.data().clone();

	if(!transform.isNull() && !transform.isIdentity())
	{
		Eigen::Affine3f transform3f = transform.toEigen3f();
		int nx = laserScan.getNormalsOffset();
		int ny = nx+1;
		int nz = ny+1;
		for(int i=0; i<laserScan.size(); ++i)
		{
			const float * ptr = laserScan.data().ptr<float>(0, i);
			float * out = output.ptr<float>(0, i);
			if(nx == -1)
			{
				pcl::PointXYZ pt(ptr[0], ptr[1], laserScan.is2d()?0:ptr[2]);
				pt = pcl::transformPoint(pt, transform3f);
				out[0] = pt.x;
				out[1] = pt.y;
				if(!laserScan.is2d())
				{
					out[2] = pt.z;
				}
			}
			else
			{
				pcl::PointNormal pt;
				pt.x=ptr[0];
				pt.y=ptr[1];
				pt.z=laserScan.is2d()?0:ptr[2];
				pt.normal_x=ptr[nx];
				pt.normal_y=ptr[ny];
				pt.normal_z=ptr[nz];
				pt = util3d::transformPoint(pt, transform);
				out[0] = pt.x;
				out[1] = pt.y;
				if(!laserScan.is2d())
				{
					out[2] = pt.z;
				}
				out[nx] = pt.normal_x;
				out[ny] = pt.normal_y;
				out[nz] = pt.normal_z;
			}
		}
	}
	if(laserScan.angleIncrement() > 0.0f)
	{
		return LaserScan(output,
				laserScan.format(),
				laserScan.rangeMin(),
				laserScan.rangeMax(),
				laserScan.angleMin(),
				laserScan.angleMax(),
				laserScan.angleIncrement(),
				laserScan.localTransform());
	}
	else
	{
		return LaserScan(output,
				laserScan.maxPoints(),
				laserScan.rangeMax(),
				laserScan.format(),
				laserScan.localTransform());
	}
}

pcl::PointCloud<pcl::PointXYZ>::Ptr transformPointCloud(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		const Transform & transform)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::transformPointCloud(*cloud, *output, transform.toEigen4f());
	return output;
}
pcl::PointCloud<pcl::PointXYZI>::Ptr transformPointCloud(
		const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud,
		const Transform & transform)
{
	pcl::PointCloud<pcl::PointXYZI>::Ptr output(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::transformPointCloud(*cloud, *output, transform.toEigen4f());
	return output;
}
pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformPointCloud(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		const Transform & transform)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::transformPointCloud(*cloud, *output, transform.toEigen4f());
	return output;
}
pcl::PointCloud<pcl::PointNormal>::Ptr transformPointCloud(
		const pcl::PointCloud<pcl::PointNormal>::Ptr & cloud,
		const Transform & transform)
{
	pcl::PointCloud<pcl::PointNormal>::Ptr output(new pcl::PointCloud<pcl::PointNormal>);
	pcl::transformPointCloudWithNormals(*cloud, *output, transform.toEigen4f());
	return output;
}
pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr transformPointCloud(
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud,
		const Transform & transform)
{
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::transformPointCloudWithNormals(*cloud, *output, transform.toEigen4f());
	return output;
}
pcl::PointCloud<pcl::PointXYZINormal>::Ptr transformPointCloud(
		const pcl::PointCloud<pcl::PointXYZINormal>::Ptr & cloud,
		const Transform & transform)
{
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr output(new pcl::PointCloud<pcl::PointXYZINormal>);
	pcl::transformPointCloudWithNormals(*cloud, *output, transform.toEigen4f());
	return output;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr transformPointCloud(
		const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		const Transform & transform)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::transformPointCloud(*cloud, *indices, *output, transform.toEigen4f());
	return output;
}
pcl::PointCloud<pcl::PointXYZI>::Ptr transformPointCloud(
		const pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		const Transform & transform)
{
	pcl::PointCloud<pcl::PointXYZI>::Ptr output(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::transformPointCloud(*cloud, *indices, *output, transform.toEigen4f());
	return output;
}
pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformPointCloud(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		const Transform & transform)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::transformPointCloud(*cloud, *indices, *output, transform.toEigen4f());
	return output;
}
pcl::PointCloud<pcl::PointNormal>::Ptr transformPointCloud(
		const pcl::PointCloud<pcl::PointNormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		const Transform & transform)
{
	pcl::PointCloud<pcl::PointNormal>::Ptr output(new pcl::PointCloud<pcl::PointNormal>);
	pcl::transformPointCloudWithNormals(*cloud, *indices, *output, transform.toEigen4f());
	return output;
}
pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr transformPointCloud(
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		const Transform & transform)
{
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::transformPointCloudWithNormals(*cloud, *indices, *output, transform.toEigen4f());
	return output;
}
pcl::PointCloud<pcl::PointXYZINormal>::Ptr transformPointCloud(
		const pcl::PointCloud<pcl::PointXYZINormal>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		const Transform & transform)
{
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr output(new pcl::PointCloud<pcl::PointXYZINormal>);
	pcl::transformPointCloudWithNormals(*cloud, *indices, *output, transform.toEigen4f());
	return output;
}

cv::Point3f transformPoint(
		const cv::Point3f & point,
		const Transform & transform)
{
	cv::Point3f ret = point;
	ret.x = transform (0, 0) * point.x + transform (0, 1) * point.y + transform (0, 2) * point.z + transform (0, 3);
	ret.y = transform (1, 0) * point.x + transform (1, 1) * point.y + transform (1, 2) * point.z + transform (1, 3);
	ret.z = transform (2, 0) * point.x + transform (2, 1) * point.y + transform (2, 2) * point.z + transform (2, 3);
	return ret;
}
cv::Point3d transformPoint(
		const cv::Point3d & point,
		const Transform & transform)
{
	cv::Point3d ret = point;
	ret.x = transform (0, 0) * point.x + transform (0, 1) * point.y + transform (0, 2) * point.z + transform (0, 3);
	ret.y = transform (1, 0) * point.x + transform (1, 1) * point.y + transform (1, 2) * point.z + transform (1, 3);
	ret.z = transform (2, 0) * point.x + transform (2, 1) * point.y + transform (2, 2) * point.z + transform (2, 3);
	return ret;
}
pcl::PointXYZ transformPoint(
		const pcl::PointXYZ & pt,
		const Transform & transform)
{
	return pcl::transformPoint(pt, transform.toEigen3f());
}
pcl::PointXYZI transformPoint(
		const pcl::PointXYZI & pt,
		const Transform & transform)
{
	return pcl::transformPoint(pt, transform.toEigen3f());
}
pcl::PointXYZRGB transformPoint(
		const pcl::PointXYZRGB & pt,
		const Transform & transform)
{
	pcl::PointXYZRGB ptRGB = pcl::transformPoint(pt, transform.toEigen3f());
	ptRGB.rgb = pt.rgb;
	return ptRGB;
}
pcl::PointNormal transformPoint(
		const pcl::PointNormal & point,
		const Transform & transform)
{
	pcl::PointNormal ret;
	Eigen::Matrix<float, 3, 1> pt (point.x, point.y, point.z);
	ret.x = static_cast<float> (transform (0, 0) * pt.coeffRef (0) + transform (0, 1) * pt.coeffRef (1) + transform (0, 2) * pt.coeffRef (2) + transform (0, 3));
	ret.y = static_cast<float> (transform (1, 0) * pt.coeffRef (0) + transform (1, 1) * pt.coeffRef (1) + transform (1, 2) * pt.coeffRef (2) + transform (1, 3));
	ret.z = static_cast<float> (transform (2, 0) * pt.coeffRef (0) + transform (2, 1) * pt.coeffRef (1) + transform (2, 2) * pt.coeffRef (2) + transform (2, 3));

	// Rotate normals
	Eigen::Matrix<float, 3, 1> nt (point.normal_x, point.normal_y, point.normal_z);
	ret.normal_x = static_cast<float> (transform (0, 0) * nt.coeffRef (0) + transform (0, 1) * nt.coeffRef (1) + transform (0, 2) * nt.coeffRef (2));
	ret.normal_y = static_cast<float> (transform (1, 0) * nt.coeffRef (0) + transform (1, 1) * nt.coeffRef (1) + transform (1, 2) * nt.coeffRef (2));
	ret.normal_z = static_cast<float> (transform (2, 0) * nt.coeffRef (0) + transform (2, 1) * nt.coeffRef (1) + transform (2, 2) * nt.coeffRef (2));
	return ret;
}
pcl::PointXYZRGBNormal transformPoint(
		const pcl::PointXYZRGBNormal & point,
		const Transform & transform)
{
	pcl::PointXYZRGBNormal ret;
	Eigen::Matrix<float, 3, 1> pt (point.x, point.y, point.z);
	ret.x = static_cast<float> (transform (0, 0) * pt.coeffRef (0) + transform (0, 1) * pt.coeffRef (1) + transform (0, 2) * pt.coeffRef (2) + transform (0, 3));
	ret.y = static_cast<float> (transform (1, 0) * pt.coeffRef (0) + transform (1, 1) * pt.coeffRef (1) + transform (1, 2) * pt.coeffRef (2) + transform (1, 3));
	ret.z = static_cast<float> (transform (2, 0) * pt.coeffRef (0) + transform (2, 1) * pt.coeffRef (1) + transform (2, 2) * pt.coeffRef (2) + transform (2, 3));

	// Rotate normals
	Eigen::Matrix<float, 3, 1> nt (point.normal_x, point.normal_y, point.normal_z);
	ret.normal_x = static_cast<float> (transform (0, 0) * nt.coeffRef (0) + transform (0, 1) * nt.coeffRef (1) + transform (0, 2) * nt.coeffRef (2));
	ret.normal_y = static_cast<float> (transform (1, 0) * nt.coeffRef (0) + transform (1, 1) * nt.coeffRef (1) + transform (1, 2) * nt.coeffRef (2));
	ret.normal_z = static_cast<float> (transform (2, 0) * nt.coeffRef (0) + transform (2, 1) * nt.coeffRef (1) + transform (2, 2) * nt.coeffRef (2));

	ret.rgb = point.rgb;
	return ret;
}
pcl::PointXYZINormal transformPoint(
		const pcl::PointXYZINormal & point,
		const Transform & transform)
{
	pcl::PointXYZINormal ret;
	Eigen::Matrix<float, 3, 1> pt (point.x, point.y, point.z);
	ret.x = static_cast<float> (transform (0, 0) * pt.coeffRef (0) + transform (0, 1) * pt.coeffRef (1) + transform (0, 2) * pt.coeffRef (2) + transform (0, 3));
	ret.y = static_cast<float> (transform (1, 0) * pt.coeffRef (0) + transform (1, 1) * pt.coeffRef (1) + transform (1, 2) * pt.coeffRef (2) + transform (1, 3));
	ret.z = static_cast<float> (transform (2, 0) * pt.coeffRef (0) + transform (2, 1) * pt.coeffRef (1) + transform (2, 2) * pt.coeffRef (2) + transform (2, 3));

	// Rotate normals
	Eigen::Matrix<float, 3, 1> nt (point.normal_x, point.normal_y, point.normal_z);
	ret.normal_x = static_cast<float> (transform (0, 0) * nt.coeffRef (0) + transform (0, 1) * nt.coeffRef (1) + transform (0, 2) * nt.coeffRef (2));
	ret.normal_y = static_cast<float> (transform (1, 0) * nt.coeffRef (0) + transform (1, 1) * nt.coeffRef (1) + transform (1, 2) * nt.coeffRef (2));
	ret.normal_z = static_cast<float> (transform (2, 0) * nt.coeffRef (0) + transform (2, 1) * nt.coeffRef (1) + transform (2, 2) * nt.coeffRef (2));

	ret.intensity = point.intensity;
	return ret;
}

}

}
