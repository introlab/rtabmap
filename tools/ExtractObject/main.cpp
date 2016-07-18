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

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <rtabmap/utilite/UConversion.h>

void showUsage()
{
	printf("\nUsage: extractObject [options] cloud.pcd\n"
			"Options:\n"
			"   -p #.#            plane distance threshold (default 0.02 m)\n"
			"   -a #.#            plane angle tolerance from z axis (default PI/6)\n"
			"   -c #.#            cluster tolerance (default 0.1 m)\n"
			"   -s #              minimum cluster size (default 50 points)\n"
			"   -center_obj       center the objects to their local reference\n"
			"   -save_plane       save the plane inliers to \"extracted_plane.pcd\"\n");
}

int main(int argc, char *argv[])
{
	if(argc < 2)
	{
		showUsage();
		return -1;
	}

	std::string cloudPath = argv[argc-1];
	double planeDistanceThreshold = 0.02f;
	double planeEpsAngle = 3.1416/6.0;
	double clusterTolerance = 0.1f;
	int minClusterSize = 50;
	bool centerObject = false;
	bool savePlane = false;

	for(int i=1; i<argc-1; ++i)
	{
		if(strcmp(argv[i], "-p") == 0)
		{
			++i;
			if(i < argc)
			{
				planeDistanceThreshold = uStr2Float(argv[i]);
				if(planeDistanceThreshold < 0.0f)
				{
					showUsage();
				}
			}
			else
			{
				showUsage();
			}
			continue;
		}
		if(strcmp(argv[i], "-a") == 0)
		{
			++i;
			if(i < argc)
			{
				planeEpsAngle = uStr2Float(argv[i]);
				if(planeEpsAngle < 0.0f)
				{
					showUsage();
				}
			}
			else
			{
				showUsage();
			}
			continue;
		}
		if(strcmp(argv[i], "-c") == 0)
		{
			++i;
			if(i < argc)
			{
				clusterTolerance = uStr2Float(argv[i]);
				if(clusterTolerance <= 0.0f)
				{
					showUsage();
				}
			}
			else
			{
				showUsage();
			}
			continue;
		}
		if(strcmp(argv[i], "-s") == 0)
		{
			++i;
			if(i < argc)
			{
				minClusterSize = std::atoi(argv[i]);
				if(minClusterSize < 1)
				{
					showUsage();
				}
			}
			else
			{
				showUsage();
			}
			continue;
		}
		if(strcmp(argv[i], "-center_obj") == 0)
		{
			centerObject = true;
			continue;
		}
		if(strcmp(argv[i], "-save_plane") == 0)
		{
			savePlane = true;
			continue;
		}
		printf("Unrecognized option : %s\n", argv[i]);
		showUsage();
	}

	printf("Parameters:\n"
			"   planeDistanceThreshold=%f\n"
			"   planeEpsAngle=%f\n"
			"   clusterTolerance=%f\n"
			"   minClusterSize=%d\n"
			"   centerObject=%s\n"
			"   savePlane=%s\n",
			planeDistanceThreshold,
			planeEpsAngle,
			clusterTolerance,
			minClusterSize,
			centerObject?"true":"false",
			savePlane?"true":"false");

	printf("Loading \"%s\"...", cloudPath.c_str());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::io::loadPCDFile(cloudPath, *cloud);
	printf("done! (%d points)\n", (int)cloud->size());

	// Extract plane
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZRGB> seg;
	// Optional
	seg.setOptimizeCoefficients (true);
	// Mandatory
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setMaxIterations(1000);
	seg.setDistanceThreshold (planeDistanceThreshold);
	seg.setAxis(Eigen::Vector3f(0,0,1));
	seg.setEpsAngle (planeEpsAngle);

	seg.setInputCloud (cloud);
	seg.segment (*inliers, *coefficients);

	printf("Plane coefficients: %f %f %f %f\n", coefficients->values[0],
										  coefficients->values[1],
										  coefficients->values[2],
										  coefficients->values[3]);

	float a = coefficients->values[0];
	float b = coefficients->values[1];
	float c = coefficients->values[2];
	float d = coefficients->values[3];

	// Create a quaternion for rotation into XY plane
	Eigen::Vector3f current(a, b, c);
	Eigen::Vector3f target(0.0, 0.0, 1.0);
	Eigen::Quaternion<float> q;
	q.setFromTwoVectors(current, target);
	Eigen::Matrix4f trans;
	trans.topLeftCorner<3,3>() = q.toRotationMatrix();

	float planeShift = -d;

	// Create transformed point cloud (polygon is aligned with XY plane)
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr output (new pcl::PointCloud<pcl::PointXYZRGB> ());
	pcl::transformPointCloud(*cloud, *output, Eigen::Vector3f(-a*planeShift, -b*planeShift, -c*planeShift), q);

	if(savePlane)
	{
		pcl::io::savePCDFile("extracted_plane.pcd", *output, inliers->indices);
		printf("Saved extracted_plane.pcd (%d points)\n", (int)inliers->indices.size());
	}
	else
	{
		printf("Plane size = %d points\n", (int)inliers->indices.size());
	}

	// remove plane inliers
	pcl::ExtractIndices<pcl::PointXYZRGB> extract;
	extract.setNegative (true);
	extract.setInputCloud (output);
	extract.setIndices(inliers);
	extract.filter (*output);

	// Get the biggest cluster
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdTree(new pcl::search::KdTree<pcl::PointXYZRGB>);
	kdTree->setInputCloud(output);
	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
	ec.setClusterTolerance (clusterTolerance);
	ec.setMinClusterSize (minClusterSize);
	ec.setMaxClusterSize (200000);
	ec.setSearchMethod (kdTree);
	ec.setInputCloud (output);
	ec.extract (cluster_indices);

	if(cluster_indices.size() == 0)
	{
		printf("No object found! (minimum cluster size=%d)\n", minClusterSize);
		return 1;
	}

	for(unsigned int i=0; i<cluster_indices.size(); ++i)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr object (new pcl::PointCloud<pcl::PointXYZRGB> ());
		pcl::copyPointCloud(*output, cluster_indices.at(i), *object);

		if(centerObject)
		{
			// recenter the object on xy plane, and set min z to 0
			Eigen::Vector4f min, max;
			pcl::getMinMax3D(*object, min, max);

			pcl::transformPointCloud(*object, *object, Eigen::Vector3f(-(max[0]+min[0])/2.0f, -(max[1]+min[1])/2.0f, -min[2]), Eigen::Quaternion<float>(0,0,0,0));
		}

		pcl::io::savePCDFile(uFormat("extracted_object%d.pcd", (int)i+1), *object);
		printf("Saved extracted_object%d.pcd (%d points)\n", (int)i+1, (int)object->size());
	}

	return 0;
}
