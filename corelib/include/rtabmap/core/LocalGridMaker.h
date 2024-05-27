/*
Copyright (c) 2010-2023, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
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

#ifndef SRC_LOCAL_MAP_H_
#define SRC_LOCAL_MAP_H_

#include "rtabmap/core/rtabmap_core_export.h" // DLL export/import defines

#include <pcl/pcl_base.h>
#include <pcl/point_types.h>

#include <rtabmap/core/Transform.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/Signature.h>

namespace rtabmap {

class RTABMAP_CORE_EXPORT LocalGridMaker
{
public:
	LocalGridMaker(const ParametersMap & parameters = ParametersMap());
	virtual ~LocalGridMaker();
	virtual void parseParameters(const ParametersMap & parameters);

	float getCellSize() const {return cellSize_;}
	bool isGridFromDepth() const {return occupancySensor_;}
	bool isMapFrameProjection() const {return projMapFrame_;}

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

protected:
	ParametersMap parameters_;

	unsigned int cloudDecimation_;
	float rangeMax_;
	float rangeMin_;
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
};

} /* namespace rtabmap */

#include <rtabmap/core/impl/LocalMapMaker.hpp>

#endif /* SRC_MAP_H_ */
