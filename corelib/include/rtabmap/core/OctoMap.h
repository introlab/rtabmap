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

#ifndef SRC_OCTOMAP_H_
#define SRC_OCTOMAP_H_

#include "rtabmap/core/RtabmapExp.h" // DLL export/import defines

#include <octomap/ColorOcTree.h>
#include <octomap/OcTreeKey.h>

#include <pcl/pcl_base.h>
#include <pcl/point_types.h>

#include <rtabmap/core/Transform.h>

#include <map>
#include <string>

namespace rtabmap {

class OcTreeNodeInfo
{
public:
	OcTreeNodeInfo(int nodeRefId, const octomap::OcTreeKey & key, bool isObstacle) :
		nodeRefId_(nodeRefId),
		key_(key),
		isObstacle_(isObstacle) {}
	int nodeRefId_;
	octomap::OcTreeKey key_;
	bool isObstacle_;
};

class RTABMAP_EXP OctoMap {
public:
	OctoMap(float voxelSize = 0.1f);

	const std::map<int, Transform> & addedNodes() const {return addedNodes_;}
	void addToCache(int nodeId,
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr & ground,
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr & obstacles);
	void update(const std::map<int, Transform> & poses);

	const octomap::ColorOcTree * octree() const {return octree_;}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr createCloud(
			unsigned int treeDepth = 0,
			std::vector<int> * obstacleIndices = 0,
			std::vector<int> * emptyIndices = 0) const;

	cv::Mat createProjectionMap(
			float & xMin,
			float & yMin,
			float & gridCellSize,
			float minGridSize);

	bool writeBinary(const std::string & path);

	virtual ~OctoMap();
	void clear();

private:
	std::map<int, std::pair<pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr> > cache_;
	octomap::ColorOcTree * octree_;
	std::map<octomap::ColorOcTreeNode*, OcTreeNodeInfo> occupiedCells_;
	std::map<int, Transform> addedNodes_;
	octomap::KeyRay keyRay_;
};

} /* namespace rtabmap */

#endif /* SRC_OCTOMAP_H_ */
