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

#include "rtabmap/core/rtabmap_core_export.h" // DLL export/import defines

#include <octomap/ColorOcTree.h>
#include <octomap/OcTreeKey.h>

#include <pcl/pcl_base.h>
#include <pcl/point_types.h>

#include <rtabmap/core/Transform.h>
#include <rtabmap/core/Parameters.h>

#include <map>
#include <unordered_set>
#include <string>
#include <queue>

namespace rtabmap {

// forward declaraton for "friend"
class RtabmapColorOcTree;

class RtabmapColorOcTreeNode : public octomap::ColorOcTreeNode
{
public:
	enum OccupancyType {kTypeUnknown=-1, kTypeEmpty=0, kTypeGround=1, kTypeObstacle=100};

public:
	friend class RtabmapColorOcTree; // needs access to node children (inherited)

	RtabmapColorOcTreeNode() : ColorOcTreeNode(), nodeRefId_(0), type_(kTypeUnknown) {}
	RtabmapColorOcTreeNode(const RtabmapColorOcTreeNode& rhs) : ColorOcTreeNode(rhs), nodeRefId_(rhs.nodeRefId_), type_(rhs.type_) {}

	void setNodeRefId(int nodeRefId) {nodeRefId_ = nodeRefId;}
	void setOccupancyType(char type) {type_=type;}
	void setPointRef(const octomap::point3d & point) {pointRef_ = point;}
	int getNodeRefId() const {return nodeRefId_;}
	int getOccupancyType() const {return type_;}
	const octomap::point3d & getPointRef() const {return pointRef_;}

	// following methods defined for octomap < 1.8 compatibility
	RtabmapColorOcTreeNode* getChild(unsigned int i);
	const RtabmapColorOcTreeNode* getChild(unsigned int i) const;
	bool pruneNode();
	void expandNode();
	bool createChild(unsigned int i);

	void updateOccupancyTypeChildren();

private:
	int nodeRefId_;
	int type_; // -1=undefined, 0=empty, 100=obstacle, 1=ground
	octomap::point3d pointRef_;
};

// Same as official ColorOctree but using RtabmapColorOcTreeNode, which is inheriting ColorOcTreeNode
class RtabmapColorOcTree : public octomap::OccupancyOcTreeBase <RtabmapColorOcTreeNode> {

  public:
    /// Default constructor, sets resolution of leafs
	RtabmapColorOcTree(double resolution);
	virtual ~RtabmapColorOcTree() {}

    /// virtual constructor: creates a new object of same type
    /// (Covariant return type requires an up-to-date compiler)
	RtabmapColorOcTree* create() const {return new RtabmapColorOcTree(resolution); }

    std::string getTreeType() const {return "ColorOcTree";} // same type as ColorOcTree to be compatible with ROS OctoMap msg

     /**
     * Prunes a node when it is collapsible. This overloaded
     * version only considers the node occupancy for pruning,
     * different colors of child nodes are ignored.
     * @return true if pruning was successful
     */
    virtual bool pruneNode(RtabmapColorOcTreeNode* node);

    virtual bool isNodeCollapsible(const RtabmapColorOcTreeNode* node) const;

    // set node color at given key or coordinate. Replaces previous color.
    RtabmapColorOcTreeNode* setNodeColor(const octomap::OcTreeKey& key, uint8_t r,
                                 uint8_t g, uint8_t b);

    RtabmapColorOcTreeNode* setNodeColor(float x, float y,
                                 float z, uint8_t r,
                                 uint8_t g, uint8_t b) {
    	octomap::OcTreeKey key;
      if (!this->coordToKeyChecked(octomap::point3d(x,y,z), key)) return NULL;
      return setNodeColor(key,r,g,b);
    }

    // integrate color measurement at given key or coordinate. Average with previous color
    RtabmapColorOcTreeNode* averageNodeColor(const octomap::OcTreeKey& key, uint8_t r,
                                  uint8_t g, uint8_t b);

    RtabmapColorOcTreeNode* averageNodeColor(float x, float y,
                                      float z, uint8_t r,
                                      uint8_t g, uint8_t b) {
    	octomap:: OcTreeKey key;
      if (!this->coordToKeyChecked(octomap::point3d(x,y,z), key)) return NULL;
      return averageNodeColor(key,r,g,b);
    }

    // integrate color measurement at given key or coordinate. Average with previous color
    RtabmapColorOcTreeNode* integrateNodeColor(const octomap::OcTreeKey& key, uint8_t r,
                                  uint8_t g, uint8_t b);

    RtabmapColorOcTreeNode* integrateNodeColor(float x, float y,
                                      float z, uint8_t r,
                                      uint8_t g, uint8_t b) {
    	octomap::OcTreeKey key;
      if (!this->coordToKeyChecked(octomap::point3d(x,y,z), key)) return NULL;
      return integrateNodeColor(key,r,g,b);
    }

    // update inner nodes, sets color to average child color
    void updateInnerOccupancy();

  protected:
    void updateInnerOccupancyRecurs(RtabmapColorOcTreeNode* node, unsigned int depth);

    /**
     * Static member object which ensures that this OcTree's prototype
     * ends up in the classIDMapping only once. You need this as a
     * static member in any derived octree class in order to read .ot
     * files through the AbstractOcTree factory. You should also call
     * ensureLinking() once from the constructor.
     */
    class StaticMemberInitializer{
       public:
         StaticMemberInitializer();

         /**
         * Dummy function to ensure that MSVC does not drop the
         * StaticMemberInitializer, causing this tree failing to register.
         * Needs to be called from the constructor of this octree.
         */
         void ensureLinking() {};
    };
    /// static member to ensure static initialization (only once)
    static StaticMemberInitializer RtabmapColorOcTreeMemberInit;

  };

class RTABMAP_CORE_EXPORT OctoMap {
public:
	OctoMap(const ParametersMap & parameters = ParametersMap());

	const std::map<int, Transform> & addedNodes() const {return addedNodes_;}
	void addToCache(int nodeId,
			const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & ground,
			const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & obstacles,
			const pcl::PointXYZ & viewPoint);
	void addToCache(int nodeId,
			const cv::Mat & ground,
			const cv::Mat & obstacles,
			const cv::Mat & empty,
			const cv::Point3f & viewPoint);
	bool update(const std::map<int, Transform> & poses); // return true if map has changed

	const RtabmapColorOcTree * octree() const {return octree_;}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr createCloud(
			unsigned int treeDepth = 0,
			std::vector<int> * obstacleIndices = 0,
			std::vector<int> * emptyIndices = 0,
			std::vector<int> * groundIndices = 0,
			bool originalRefPoints = true,
			std::vector<int> * frontierIndices = 0,
			std::vector<double> * cloudProb = 0) const;

	cv::Mat createProjectionMap(
			float & xMin,
			float & yMin,
			float & gridCellSize,
			float minGridSize = 0.0f,
			unsigned int treeDepth = 0);

	bool writeBinary(const std::string & path);

	virtual ~OctoMap();
	void clear();

	void getGridMin(double & x, double & y, double & z) const {x=minValues_[0];y=minValues_[1];z=minValues_[2];}
	void getGridMax(double & x, double & y, double & z) const {x=maxValues_[0];y=maxValues_[1];z=maxValues_[2];}

	void setMaxRange(float value) {rangeMax_ = value;}
	void setRayTracing(bool enabled) {rayTracing_ = enabled;}
	bool hasColor() const {return hasColor_;}

    static std::unordered_set<octomap::OcTreeKey, octomap::OcTreeKey::KeyHash> findEmptyNode(RtabmapColorOcTree* octree_, unsigned int treeDepth, octomap::point3d startPosition);
    static void floodFill(RtabmapColorOcTree* octree_, unsigned int treeDepth,octomap::point3d startPosition, std::unordered_set<octomap::OcTreeKey, octomap::OcTreeKey::KeyHash> & EmptyNodes,std::queue<octomap::point3d>& positionToExplore);
    static bool isNodeVisited(std::unordered_set<octomap::OcTreeKey,octomap::OcTreeKey::KeyHash> const & EmptyNodes,octomap::OcTreeKey const key);
    static octomap::point3d findCloseEmpty(RtabmapColorOcTree* octree_, unsigned int treeDepth,octomap::point3d startPosition);
    static bool isValidEmpty(RtabmapColorOcTree* octree_, unsigned int treeDepth,octomap::point3d startPosition);

private:
	void updateMinMax(const octomap::point3d & point);

private:
	std::map<int, std::pair<std::pair<cv::Mat, cv::Mat>, cv::Mat> > cache_; // [id: < <ground, obstacles>, empty>]
	std::map<int, std::pair<const pcl::PointCloud<pcl::PointXYZRGB>::Ptr, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr> > cacheClouds_; // [id: <ground, obstacles>]
	std::map<int, cv::Point3f> cacheViewPoints_;
	RtabmapColorOcTree * octree_;
	std::map<int, Transform> addedNodes_;
	bool hasColor_;
	bool fullUpdate_;
	float updateError_;
	float rangeMax_;
	bool rayTracing_;
    unsigned int emptyFloodFillDepth_;
	double minValues_[3];
	double maxValues_[3];
};

} /* namespace rtabmap */

#endif /* SRC_OCTOMAP_H_ */
