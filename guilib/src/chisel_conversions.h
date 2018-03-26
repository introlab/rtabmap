/*
 * chisel_conversions.h
 *
 *  Created on: 2018-03-25
 *      Author: mathieu
 */

#ifndef CHISEL_CONVERSIONS_H_
#define CHISEL_CONVERSIONS_H_

#include <rtabmap/core/CameraModel.h>
#include <open_chisel/Chisel.h>
#include <pcl/PolygonMesh.h>
#include <pcl/common/transforms.h>

namespace rtabmap
{

std::shared_ptr<chisel::ColorImage<unsigned char> > colorImageToChisel(const cv::Mat & image)
{
	UASSERT(image.type() == CV_8UC3 || image.type() == CV_8UC4);
	std::shared_ptr<chisel::ColorImage<unsigned char> > imageChisel(new chisel::ColorImage<unsigned char>(image.cols, image.rows, image.channels()));
	memcpy(imageChisel->GetMutableData(), image.data, image.total()*sizeof(unsigned char)*image.channels());
	return imageChisel;
}

std::shared_ptr<chisel::DepthImage<float> > depthImageToChisel(const cv::Mat & image)
{
	UASSERT(image.type() == CV_32FC1);
	std::shared_ptr<chisel::DepthImage<float> > imageChisel(new chisel::DepthImage<float>(image.cols, image.rows));
	memcpy(imageChisel->GetMutableData(), (float*)image.data, image.total()*sizeof(float));
	return imageChisel;
}

chisel::PinholeCamera cameraModelToChiselCamera(const CameraModel& camera)
{
   chisel::PinholeCamera cameraToReturn;
   chisel::Intrinsics intrinsics;
   intrinsics.SetFx(camera.fx());
   intrinsics.SetFy(camera.fy());
   intrinsics.SetCx(camera.cx());
   intrinsics.SetCy(camera.cy());
   cameraToReturn.SetIntrinsics(intrinsics);
   cameraToReturn.SetWidth(camera.imageWidth());
   cameraToReturn.SetHeight(camera.imageHeight());
   return cameraToReturn;
}

template<typename PointRGBT>
chisel::PointCloudPtr pointCloudRGBToChisel(const typename pcl::PointCloud<PointRGBT>& cloud, const Transform & transform = Transform::getIdentity())
{
	chisel::PointCloudPtr chiselCloud(new chisel::PointCloud());
	chiselCloud->GetMutablePoints().resize(cloud.size());
	chiselCloud->GetMutableColors().resize(cloud.size());
	float byteToFloat = 1.0f / 255.0f;
	int oi=0;
	Eigen::Affine3f transformf = transform.toEigen3f();
	for(unsigned int i=0; i<cloud.size(); ++i)
	{
		const PointRGBT & pt = cloud.at(i);
		if(pcl::isFinite(pt))
		{
			PointRGBT ptt = pcl::transformPoint(pt, transformf);

			chisel::Vec3& xyz =  chiselCloud->GetMutablePoints().at(oi);
			xyz(0) = ptt.x;
			xyz(1) = ptt.y;
			xyz(2) = ptt.z;

			chisel::Vec3& rgb = chiselCloud->GetMutableColors().at(oi);
			rgb(0) = ptt.r * byteToFloat;
			rgb(1) = ptt.g * byteToFloat;
			rgb(2) = ptt.b * byteToFloat;

			++oi;
		}
	}
	chiselCloud->GetMutablePoints().resize(oi);
	chiselCloud->GetMutableColors().resize(oi);
	return chiselCloud;
}

template<typename PointT>
chisel::PointCloudPtr pointCloudToChisel(const typename pcl::PointCloud<PointT>& cloud, const Transform & transform = Transform::getIdentity())
{
	chisel::PointCloudPtr chiselCloud(new chisel::PointCloud());
	chiselCloud->GetMutablePoints().resize(cloud.size());
	int oi=0;
	Eigen::Affine3f transformf = transform.toEigen3f();
	for(unsigned int i=0; i<cloud.size(); ++i)
	{
		const PointT & pt = cloud.at(i);
		if(pcl::isFinite(pt))
		{
			PointT ptt = pcl::transformPoint(pt, transformf);

			chisel::Vec3& xyz =  chiselCloud->GetMutablePoints().at(oi);
			xyz(0) = ptt.x;
			xyz(1) = ptt.y;
			xyz(2) = ptt.z;

			++oi;
		}
	}
	chiselCloud->GetMutablePoints().resize(oi);
	return chiselCloud;
}

pcl::PolygonMesh::Ptr chiselToPolygonMesh(const chisel::MeshMap& meshMap, unsigned char r=100, unsigned char g=100, unsigned char b=100)
{
	pcl::PolygonMesh::Ptr mesh (new pcl::PolygonMesh);

	if(meshMap.size())
	{
		bool hasColor = meshMap.begin()->second->colors.size();
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
		size_t v = 0;
		for (const std::pair<chisel::ChunkID, chisel::MeshPtr>& it : meshMap)
		{
			UASSERT((!hasColor || (it.second->vertices.size() == it.second->colors.size())) &&
					it.second->vertices.size() == it.second->normals.size());
			cloud->resize(cloud->size() + it.second->vertices.size());

			mesh->polygons.resize(mesh->polygons.size()+it.second->vertices.size()/3);

			for (unsigned int i=0;i<it.second->vertices.size(); ++i)
			{
				pcl::PointXYZRGBNormal & pt = cloud->at(v);
				pt.x = it.second->vertices[i][0];
				pt.y = it.second->vertices[i][1];
				pt.z = it.second->vertices[i][2];
				if(hasColor)
				{
					pt.r = it.second->colors[i][0] * 255.0f;
					pt.g = it.second->colors[i][1] * 255.0f;
					pt.b = it.second->colors[i][2] * 255.0f;
				}
				else
				{
					pt.r = r;
					pt.g = g;
					pt.b = b;
				}
				pt.normal_x = it.second->normals[i][0];
				pt.normal_y = it.second->normals[i][1];
				pt.normal_z = it.second->normals[i][2];
				pcl::Vertices & polygon = mesh->polygons.at(v/3);
				polygon.vertices.push_back(v++);
			}
		}
		pcl::toPCLPointCloud2(*cloud, mesh->cloud);
	}
	return mesh;
}

}


#endif /* CHISEL_CONVERSIONS_H_ */
