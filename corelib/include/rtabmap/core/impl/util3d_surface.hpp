/*
 * util3d_surface.hpp
 *
 *  Created on: Sep 3, 2016
 *      Author: mathieu
 */

#ifndef CORELIB_INCLUDE_RTABMAP_CORE_IMPL_UTIL3D_SURFACE_HPP_
#define CORELIB_INCLUDE_RTABMAP_CORE_IMPL_UTIL3D_SURFACE_HPP_

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

}

}


#endif /* CORELIB_INCLUDE_RTABMAP_CORE_IMPL_UTIL3D_SURFACE_HPP_ */
