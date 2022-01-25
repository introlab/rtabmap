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
	inline static float logodds(double probability)
	{
		return (float) log(probability/(1-probability));
	}

	inline static double probability(double logodds)
	{
		return 1. - ( 1. / (1. + exp(logodds)));
	}

public:
	OccupancyGrid(const ParametersMap & parameters = ParametersMap());
	void parseParameters(const ParametersMap & parameters);
	void setMap(const cv::Mat & map, float xMin, float yMin, float cellSize, const std::map<int, Transform> & poses);
	void setCellSize(float cellSize);
	float getCellSize() const {return cellSize_;}
	void setCloudAssembling(bool enabled);
	float getMinMapSize() const {return minMapSize_;}
	bool isGridFromDepth() const {return occupancySensor_;}
	bool isFullUpdate() const {return fullUpdate_;}
	float getUpdateError() const {return updateError_;}
	bool isMapFrameProjection() const {return projMapFrame_;}
	const std::map<int, Transform> & addedNodes() const {return addedNodes_;}
	int cacheSize() const {return (int)cache_.size();}
	const std::map<int, std::pair<std::pair<cv::Mat, cv::Mat>, cv::Mat> > & getCache() const {return cache_;}

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
			cv::Mat & groundCells,
			cv::Mat & obstacleCells,
			cv::Mat & emptyCells,
			cv::Point3f & viewPoint);

	void createLocalMap(
			const LaserScan & cloud,
			const Transform & pose,
			cv::Mat & groundCells,
			cv::Mat & obstacleCells,
			cv::Mat & emptyCells,
			cv::Point3f & viewPointInOut) const;

	void clear();
	void addToCache(
			int nodeId,
			const cv::Mat & ground,
			const cv::Mat & obstacles,
			const cv::Mat & empty);
	bool update(const std::map<int, Transform> & poses); // return true if map has changed
	cv::Mat getMap(float & xMin, float & yMin) const;
	cv::Mat getProbMap(float & xMin, float & yMin) const;
	const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & getMapGround() const {return assembledGround_;}
	const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & getMapObstacles() const {return assembledObstacles_;}
	const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & getMapEmptyCells() const {return assembledEmptyCells_;}

	unsigned long getMemoryUsed() const;

private:
	ParametersMap parameters_;
	unsigned int cloudDecimation_;
	float cloudMaxDepth_;
	float cloudMinDepth_;
	std::vector<float> roiRatios_;
	float footprintLength_;
	float footprintWidth_;
	float footprintHeight_;
	int scanDecimation_;
	float cellSize_;
	bool preVoxelFiltering_;
	int occupancySensor_;
	bool projMapFrame_;
	float maxObstacleHeight_;
	int normalKSearch_;
	float groundNormalsUp_;
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
	bool rayTracing_;
	bool fullUpdate_;
	float minMapSize_;
	bool erode_;
	float footprintRadius_;
	float updateError_;
	float occupancyThr_;
	float probHit_;
	float probMiss_;
	float probClampingMin_;
	float probClampingMax_;

	std::map<int, std::pair<std::pair<cv::Mat, cv::Mat>, cv::Mat> > cache_; //<node id, < <ground, obstacles>, empty> >
	cv::Mat map_;
	cv::Mat mapInfo_;
	std::map<int, std::pair<int, int> > cellCount_; //<node Id, cells>
	float xMin_;
	float yMin_;
	std::map<int, Transform> addedNodes_;

	bool cloudAssembling_;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr assembledGround_;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr assembledObstacles_;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr assembledEmptyCells_;
};

}

#include <rtabmap/core/impl/OccupancyGrid.hpp>

#endif /* CORELIB_SRC_OCCUPANCYGRID_H_ */
