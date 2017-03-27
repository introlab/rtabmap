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

#ifndef CORELIB_SRC_OCCUPANCYGRID_H_
#define CORELIB_SRC_OCCUPANCYGRID_H_

#include "rtabmap/core/RtabmapExp.h" // DLL export/import defines

#include <pcl/point_cloud.h>
#include <pcl/pcl_base.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/Signature.h>

namespace rtabmap {

class RTABMAP_EXP OccupancyGrid
{
public:
	OccupancyGrid(const ParametersMap & parameters = ParametersMap());
	void parseParameters(const ParametersMap & parameters);
	void setCellSize(float cellSize);
	float getCellSize() const {return cellSize_;}
	bool isGridFromDepth() const {return occupancyFromCloud_;}

	template<typename PointT>
	typename pcl::PointCloud<PointT>::Ptr segmentCloud(
			const typename pcl::PointCloud<PointT>::Ptr & cloud,
			const pcl::IndicesPtr & indices,
			const Transform & pose,
			const cv::Point3f & viewPoint,
			pcl::IndicesPtr & groundIndices,        // output cloud indices
			pcl::IndicesPtr & obstaclesIndices,     // output cloud indices
			pcl::IndicesPtr * flatObstacles = 0) const; // output cloud indices

	void createLocalMap(
			const Signature & node,
			cv::Mat & ground,
			cv::Mat & obstacles,
			cv::Point3f & viewPoint) const;

	void clear();
	void addToCache(
			int nodeId,
			const cv::Mat & ground,
			const cv::Mat & obstacles);
	void update(const std::map<int, Transform> & poses, float minMapSize = 0.0f, float footprintRadius = 0.0f);
	const cv::Mat & getMap(float & xMin, float & yMin) const
	{
		xMin = xMin_;
		yMin = yMin_;
		return map_;
	}

private:
	ParametersMap parameters_;
	int cloudDecimation_;
	float cloudMaxDepth_;
	float cloudMinDepth_;
	std::vector<float> roiRatios_;
	float footprintLength_;
	float footprintWidth_;
	float footprintHeight_;
	int scanDecimation_;
	float cellSize_;
	bool occupancyFromCloud_;
	bool projMapFrame_;
	float maxObstacleHeight_;
	int normalKSearch_;
	float maxGroundAngle_;
	float clusterRadius_;
	int minClusterSize_;
	bool flatObstaclesDetected_;
	float minGroundHeight_;
	float maxGroundHeight_;
	bool normalsSegmentation_;
	bool grid3D_;
	bool groundIsObstacle_;
	float noiseFilteringRadius_;
	int noiseFilteringMinNeighbors_;
	bool scan2dUnknownSpaceFilled_;
	double scan2dMaxUnknownSpaceFilledRange_;
	bool projRayTracing_;

	std::map<int, std::pair<cv::Mat, cv::Mat> > cache_;
	cv::Mat map_;
	cv::Mat mapInfo_;
	std::map<int, std::pair<int, int> > cellCount_; //<node Id, cells>
	float xMin_;
	float yMin_;
	std::map<int, Transform> addedNodes_;
};

}

#include <rtabmap/core/impl/OccupancyGrid.hpp>

#endif /* CORELIB_SRC_OCCUPANCYGRID_H_ */
