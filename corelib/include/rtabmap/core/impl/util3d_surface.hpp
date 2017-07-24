/*
 * util3d_surface.hpp
 *
 *  Created on: Sep 3, 2016
 *      Author: mathieu
 */

#ifndef CORELIB_INCLUDE_RTABMAP_CORE_IMPL_UTIL3D_SURFACE_HPP_
#define CORELIB_INCLUDE_RTABMAP_CORE_IMPL_UTIL3D_SURFACE_HPP_

#include <pcl/search/kdtree.h>
#include <rtabmap/utilite/UConversion.h>

namespace rtabmap {

namespace util3d {

template<typename pointT>
std::vector<pcl::Vertices> normalizePolygonsSide(
		const typename pcl::PointCloud<pointT> & cloud,
		const std::vector<pcl::Vertices> & polygons,
		const pcl::PointXYZ & viewPoint)
{
	std::vector<pcl::Vertices> output(polygons.size());
	for(unsigned int i=0; i<polygons.size(); ++i)
	{
		pcl::Vertices polygon = polygons[i];
		Eigen::Vector3f v1 = cloud.at(polygon.vertices[1]).getVector3fMap() - cloud.at(polygon.vertices[0]).getVector3fMap();
		Eigen::Vector3f v2 = cloud.at(polygon.vertices[2]).getVector3fMap() - cloud.at(polygon.vertices[0]).getVector3fMap();
		Eigen::Vector3f n = (v1.cross(v2)).normalized();

		Eigen::Vector3f p = Eigen::Vector3f(viewPoint.x, viewPoint.y, viewPoint.z) - cloud.at(polygon.vertices[1]).getVector3fMap();

		float result = n.dot(p);
		if(result < 0)
		{
			//reverse vertices order
			int tmp = polygon.vertices[0];
			polygon.vertices[0] = polygon.vertices[2];
			polygon.vertices[2] = tmp;
		}

		output[i] = polygon;
	}
	return output;
}

template<typename pointRGBT>
void denseMeshPostProcessing(
		pcl::PolygonMeshPtr & mesh,
		float meshDecimationFactor,
		int maximumPolygons,
		const typename pcl::PointCloud<pointRGBT>::Ptr & cloud,
		float transferColorRadius,
		bool coloredOutput,
		bool cleanMesh,
		int minClusterSize,
		ProgressState * progressState)
{
	// compute normals for the mesh if not already here
	bool hasNormals = false;
	bool hasColors = false;
	for(unsigned int i=0; i<mesh->cloud.fields.size(); ++i)
	{
		if(mesh->cloud.fields[i].name.compare("normal_x") == 0)
		{
			hasNormals = true;
		}
		else if(mesh->cloud.fields[i].name.compare("rgb") == 0)
		{
			hasColors = true;
		}
	}

	if(maximumPolygons > 0)
	{
		double factor = 1.0-double(maximumPolygons)/double(mesh->polygons.size());
		if(factor > meshDecimationFactor)
		{
			meshDecimationFactor = factor;
		}
	}
	if(meshDecimationFactor > 0.0)
	{
		unsigned int count = mesh->polygons.size();
		if(progressState) progressState->callback(uFormat("Mesh decimation (factor=%f) from %d polygons...",meshDecimationFactor, (int)count));

		mesh = util3d::meshDecimation(mesh, (float)meshDecimationFactor);
		if(progressState) progressState->callback(uFormat("Mesh decimated (factor=%f) from %d to %d polygons", meshDecimationFactor, (int)count, (int)mesh->polygons.size()));
		if(count < mesh->polygons.size())
		{
			if(progressState) progressState->callback(uFormat("Decimated mesh has more polygons than before!"));
		}
		hasNormals = false;
		hasColors = false;
	}

	if(cloud.get()!=0 &&
		!hasColors &&
		transferColorRadius >= 0.0)
	{
		if(progressState) progressState->callback(uFormat("Transferring color from point cloud to mesh..."));

		// transfer color from point cloud to mesh
		typename pcl::search::KdTree<pointRGBT>::Ptr tree (new pcl::search::KdTree<pointRGBT>(true));
		tree->setInputCloud(cloud);
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr coloredCloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
		pcl::fromPCLPointCloud2(mesh->cloud, *coloredCloud);
		std::vector<bool> coloredPts(coloredCloud->size());
		for(unsigned int i=0; i<coloredCloud->size(); ++i)
		{
			std::vector<int> kIndices;
			std::vector<float> kDistances;
			pointRGBT pt;
			pt.x = coloredCloud->at(i).x;
			pt.y = coloredCloud->at(i).y;
			pt.z = coloredCloud->at(i).z;
			if(transferColorRadius > 0.0)
			{
				tree->radiusSearch(pt, transferColorRadius, kIndices, kDistances);
			}
			else
			{
				tree->nearestKSearch(pt, 1, kIndices, kDistances);
			}
			if(kIndices.size())
			{
				//compute average color
				int r=0;
				int g=0;
				int b=0;
				int a=0;
				for(unsigned int j=0; j<kIndices.size(); ++j)
				{
					r+=(int)cloud->at(kIndices[j]).r;
					g+=(int)cloud->at(kIndices[j]).g;
					b+=(int)cloud->at(kIndices[j]).b;
					a+=(int)cloud->at(kIndices[j]).a;
				}
				coloredCloud->at(i).r = r/kIndices.size();
				coloredCloud->at(i).g = g/kIndices.size();
				coloredCloud->at(i).b = b/kIndices.size();
				coloredCloud->at(i).a = a/kIndices.size();
				coloredPts.at(i) = true;
			}
			else
			{
				//white
				coloredCloud->at(i).r = coloredCloud->at(i).g = coloredCloud->at(i).b = 255;
				coloredPts.at(i) = false;
			}
		}
		pcl::toPCLPointCloud2(*coloredCloud, mesh->cloud);

		// remove polygons with no color
		if(cleanMesh)
		{
			std::vector<pcl::Vertices> filteredPolygons(mesh->polygons.size());
			int oi=0;
			for(unsigned int i=0; i<mesh->polygons.size(); ++i)
			{
				bool coloredPolygon = true;
				for(unsigned int j=0; j<mesh->polygons[i].vertices.size(); ++j)
				{
					if(!coloredPts.at(mesh->polygons[i].vertices[j]))
					{
						coloredPolygon = false;
						break;
					}
				}
				if(coloredPolygon)
				{
					filteredPolygons[oi++] = mesh->polygons[i];
				}
			}
			filteredPolygons.resize(oi);
			mesh->polygons = filteredPolygons;
		}
		hasColors = true;
	}

	if(minClusterSize)
	{
		if(progressState) progressState->callback(uFormat("Filter small polygon clusters..."));

		// filter polygons
		std::vector<std::set<int> > neighbors;
		std::vector<std::set<int> > vertexToPolygons;
		util3d::createPolygonIndexes(mesh->polygons,
				mesh->cloud.height*mesh->cloud.width,
				neighbors,
				vertexToPolygons);
		std::list<std::list<int> > clusters = util3d::clusterPolygons(
				neighbors,
				minClusterSize<0?0:minClusterSize);

		std::vector<pcl::Vertices> filteredPolygons(mesh->polygons.size());
		if(minClusterSize < 0)
		{
			// only keep the biggest cluster
			std::list<std::list<int> >::iterator biggestClusterIndex = clusters.end();
			unsigned int biggestClusterSize = 0;
			for(std::list<std::list<int> >::iterator iter=clusters.begin(); iter!=clusters.end(); ++iter)
			{
				if(iter->size() > biggestClusterSize)
				{
					biggestClusterIndex = iter;
					biggestClusterSize = iter->size();
				}
			}
			if(biggestClusterIndex != clusters.end())
			{
				int oi=0;
				for(std::list<int>::iterator jter=biggestClusterIndex->begin(); jter!=biggestClusterIndex->end(); ++jter)
				{
					filteredPolygons[oi++] = mesh->polygons.at(*jter);
				}
				filteredPolygons.resize(oi);
			}
		}
		else
		{
			int oi=0;
			for(std::list<std::list<int> >::iterator iter=clusters.begin(); iter!=clusters.end(); ++iter)
			{
				for(std::list<int>::iterator jter=iter->begin(); jter!=iter->end(); ++jter)
				{
					filteredPolygons[oi++] = mesh->polygons.at(*jter);
				}
			}
			filteredPolygons.resize(oi);
		}

		int before = (int)mesh->polygons.size();
		mesh->polygons = filteredPolygons;

		if(progressState) progressState->callback(uFormat("Filtered %d polygons.", before-(int)mesh->polygons.size()));
	}

	// compute normals for the mesh if not already here, add also white color if colored output is required
	if(!hasNormals || (!hasColors && coloredOutput))
	{
		// use polygons
		if(hasColors || coloredOutput)
		{
			pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
			pcl::fromPCLPointCloud2(mesh->cloud, *cloud);

			Eigen::Vector3f normal(1,0,0);
			for(unsigned int i=0; i<mesh->polygons.size(); ++i)
			{
				pcl::Vertices & v = mesh->polygons[i];
				if(!hasNormals)
				{
					UASSERT(v.vertices.size()>2);
					Eigen::Vector3f v0(
							cloud->at(v.vertices[1]).x - cloud->at(v.vertices[0]).x,
							cloud->at(v.vertices[1]).y - cloud->at(v.vertices[0]).y,
							cloud->at(v.vertices[1]).z - cloud->at(v.vertices[0]).z);
					int last = v.vertices.size()-1;
					Eigen::Vector3f v1(
							cloud->at(v.vertices[last]).x - cloud->at(v.vertices[0]).x,
							cloud->at(v.vertices[last]).y - cloud->at(v.vertices[0]).y,
							cloud->at(v.vertices[last]).z - cloud->at(v.vertices[0]).z);
					normal = v0.cross(v1);
					normal.normalize();
				}
				// flat normal (per face)
				for(unsigned int j=0; j<v.vertices.size(); ++j)
				{
					if(!hasNormals)
					{
						cloud->at(v.vertices[j]).normal_x = normal[0];
						cloud->at(v.vertices[j]).normal_y = normal[1];
						cloud->at(v.vertices[j]).normal_z = normal[2];
					}
					if(!hasColors)
					{
						cloud->at(v.vertices[j]).r = 255;
						cloud->at(v.vertices[j]).g = 255;
						cloud->at(v.vertices[j]).b = 255;
					}
				}
			}
			pcl::toPCLPointCloud2 (*cloud, mesh->cloud);
		}
		else
		{
			pcl::PointCloud<pcl::PointNormal>::Ptr cloud (new pcl::PointCloud<pcl::PointNormal>);
			pcl::fromPCLPointCloud2(mesh->cloud, *cloud);

			for(unsigned int i=0; i<mesh->polygons.size(); ++i)
			{
				pcl::Vertices & v = mesh->polygons[i];
				UASSERT(v.vertices.size()>2);
				Eigen::Vector3f v0(
						cloud->at(v.vertices[1]).x - cloud->at(v.vertices[0]).x,
						cloud->at(v.vertices[1]).y - cloud->at(v.vertices[0]).y,
						cloud->at(v.vertices[1]).z - cloud->at(v.vertices[0]).z);
				int last = v.vertices.size()-1;
				Eigen::Vector3f v1(
						cloud->at(v.vertices[last]).x - cloud->at(v.vertices[0]).x,
						cloud->at(v.vertices[last]).y - cloud->at(v.vertices[0]).y,
						cloud->at(v.vertices[last]).z - cloud->at(v.vertices[0]).z);
				Eigen::Vector3f normal = v0.cross(v1);
				normal.normalize();
				// flat normal (per face)
				for(unsigned int j=0; j<v.vertices.size(); ++j)
				{
					cloud->at(v.vertices[j]).normal_x = normal[0];
					cloud->at(v.vertices[j]).normal_y = normal[1];
					cloud->at(v.vertices[j]).normal_z = normal[2];
				}
			}
			pcl::toPCLPointCloud2 (*cloud, mesh->cloud);
		}
	}
}

}

}


#endif /* CORELIB_INCLUDE_RTABMAP_CORE_IMPL_UTIL3D_SURFACE_HPP_ */
