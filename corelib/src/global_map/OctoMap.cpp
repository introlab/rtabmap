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

#include <rtabmap/core/global_map/OctoMap.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/core/util3d_transforms.h>
#include <rtabmap/core/util3d_filtering.h>
#include <rtabmap/core/util3d_mapping.h>
#include <rtabmap/core/util2d.h>
#include <pcl/common/transforms.h>


namespace rtabmap {

//////////////////////////////////////
// RtabmapColorOcTree
//////////////////////////////////////

RtabmapColorOcTreeNode* RtabmapColorOcTreeNode::getChild(unsigned int i) {
#ifdef OCTOMAP_PRE_18
  return static_cast<RtabmapColorOcTreeNode*> (OcTreeNode::getChild(i));
#else
	UFATAL("This function should not be used with octomap >= 1.8");
	return 0;
#endif
}
const RtabmapColorOcTreeNode* RtabmapColorOcTreeNode::getChild(unsigned int i) const {
#ifdef OCTOMAP_PRE_18
  return static_cast<const RtabmapColorOcTreeNode*> (OcTreeNode::getChild(i));
#else
	UFATAL("This function should not be used with octomap >= 1.8");
	return 0;
#endif
}

//octomap <1.8
bool RtabmapColorOcTreeNode::pruneNode() {
#ifdef OCTOMAP_PRE_18
	// checks for equal occupancy only, color ignored
	if (!this->collapsible()) return false;
	// set occupancy value
	setLogOdds(getChild(0)->getLogOdds());
	// set color to average color
	if (isColorSet()) color = getAverageChildColor();
	// delete children
	for (unsigned int i=0;i<8;i++) {
		delete children[i];
	}
	delete[] children;
	children = NULL;
	return true;
#else
	UFATAL("This function should not be used with octomap >= 1.8");
	return false;
#endif
}
//octomap <1.8
void RtabmapColorOcTreeNode::expandNode() {
#ifdef OCTOMAP_PRE_18
	assert(!hasChildren());
	for (unsigned int k=0; k<8; k++) {
		createChild(k);
		children[k]->setValue(value);
		getChild(k)->setColor(color);
	}
#else
	UFATAL("This function should not be used with octomap >= 1.8");
#endif
}

//octomap <1.8
bool RtabmapColorOcTreeNode::createChild(unsigned int i) {
#ifdef OCTOMAP_PRE_18
  if (children == NULL) allocChildren();
  children[i] = new RtabmapColorOcTreeNode();
  return true;
#else
	UFATAL("This function should not be used with octomap >= 1.8");
	return false;
#endif
}

void RtabmapColorOcTreeNode::updateOccupancyTypeChildren()
{
	if (children != NULL){
		int type = kTypeUnknown;
		for (int i=0; i<8 && type != kTypeObstacle; i++) {
			RtabmapColorOcTreeNode* child = static_cast<RtabmapColorOcTreeNode*>(children[i]);

			if (child != NULL && child->getOccupancyType() >= kTypeEmpty) {
				if(type == kTypeUnknown) {
					type = child->getOccupancyType();
				}
			}
		}
		type_ = type;
	}
}

RtabmapColorOcTree::RtabmapColorOcTree(double resolution)
	: OccupancyOcTreeBase<RtabmapColorOcTreeNode>(resolution) {
	RtabmapColorOcTreeMemberInit.ensureLinking();
};

RtabmapColorOcTreeNode* RtabmapColorOcTree::setNodeColor(const octomap::OcTreeKey& key,
		uint8_t r,
		uint8_t g,
		uint8_t b) {
	RtabmapColorOcTreeNode* n = search (key);
	if (n != 0) {
		n->setColor(r, g, b);
	}
	return n;
}

bool RtabmapColorOcTree::pruneNode(RtabmapColorOcTreeNode* node) {
#ifndef OCTOMAP_PRE_18
	if (!isNodeCollapsible(node))
		return false;

	// set value to children's values (all assumed equal)
	node->copyData(*(getNodeChild(node, 0)));

	if (node->isColorSet()) // TODO check
		node->setColor(node->getAverageChildColor());

	// delete children
	for (unsigned int i=0;i<8;i++) {
		deleteNodeChild(node, i);
	}
	delete[] node->children;
	node->children = NULL;

	return true;
#else
	UFATAL("This function should not be used with octomap < 1.8");
	return false;
#endif
}

bool RtabmapColorOcTree::isNodeCollapsible(const RtabmapColorOcTreeNode* node) const{
#ifndef OCTOMAP_PRE_18
	// all children must exist, must not have children of
	// their own and have the same occupancy probability
	if (!nodeChildExists(node, 0))
		return false;

	const RtabmapColorOcTreeNode* firstChild = getNodeChild(node, 0);
	if (nodeHasChildren(firstChild))
		return false;

	for (unsigned int i = 1; i<8; i++) {
		// compare nodes only using their occupancy, ignoring color for pruning
		if (!nodeChildExists(node, i) || nodeHasChildren(getNodeChild(node, i)) || !(getNodeChild(node, i)->getValue() == firstChild->getValue()))
			return false;
	}

	return true;
#else
	UFATAL("This function should not be used with octomap < 1.8");
	return false;
#endif
}

RtabmapColorOcTreeNode* RtabmapColorOcTree::averageNodeColor(const octomap::OcTreeKey& key,
		uint8_t r,
		uint8_t g,
		uint8_t b) {
	RtabmapColorOcTreeNode* n = search(key);
	if (n != 0) {
		if (n->isColorSet()) {
			RtabmapColorOcTreeNode::Color prev_color = n->getColor();
			n->setColor((prev_color.r + r)/2, (prev_color.g + g)/2, (prev_color.b + b)/2);
		}
		else {
			n->setColor(r, g, b);
		}
	}
	return n;
}

RtabmapColorOcTreeNode* RtabmapColorOcTree::integrateNodeColor(const octomap::OcTreeKey& key,
		uint8_t r,
		uint8_t g,
		uint8_t b) {
	RtabmapColorOcTreeNode* n = search (key);
	if (n != 0) {
		if (n->isColorSet()) {
			RtabmapColorOcTreeNode::Color prev_color = n->getColor();
			double node_prob = n->getOccupancy();
			uint8_t new_r = (uint8_t) ((double) prev_color.r * node_prob
					+  (double) r * (0.99-node_prob));
			uint8_t new_g = (uint8_t) ((double) prev_color.g * node_prob
					+  (double) g * (0.99-node_prob));
			uint8_t new_b = (uint8_t) ((double) prev_color.b * node_prob
					+  (double) b * (0.99-node_prob));
			n->setColor(new_r, new_g, new_b);
		}
		else {
			n->setColor(r, g, b);
		}
	}
	return n;
}


void RtabmapColorOcTree::updateInnerOccupancy() {
	this->updateInnerOccupancyRecurs(this->root, 0);
}

void RtabmapColorOcTree::updateInnerOccupancyRecurs(RtabmapColorOcTreeNode* node, unsigned int depth) {
#ifndef OCTOMAP_PRE_18
	// only recurse and update for inner nodes:
	if (nodeHasChildren(node)){
		// return early for last level:
		if (depth < this->tree_depth){
			for (unsigned int i=0; i<8; i++) {
				if (nodeChildExists(node, i)) {
					updateInnerOccupancyRecurs(getNodeChild(node, i), depth+1);
				}
			}
		}
		node->updateOccupancyChildren();
		node->updateColorChildren();
		node->updateOccupancyTypeChildren();
	}
#else
	// only recurse and update for inner nodes:
	if (node->hasChildren()){
	  // return early for last level:
	  if (depth < this->tree_depth){
		for (unsigned int i=0; i<8; i++) {
		  if (node->childExists(i)) {
			updateInnerOccupancyRecurs(node->getChild(i), depth+1);
		  }
		}
	  }
	  node->updateOccupancyChildren();
	  node->updateColorChildren();
	  node->updateOccupancyTypeChildren();
	}
#endif
}

RtabmapColorOcTree::StaticMemberInitializer::StaticMemberInitializer() {
	 RtabmapColorOcTree* tree = new RtabmapColorOcTree(0.1);

#ifndef OCTOMAP_PRE_18
     tree->clearKeyRays();
#endif

	 AbstractOcTree::registerTreeType(tree);
 }

#ifndef _WIN32
// On Windows, the app freezes on start if the following is defined
RtabmapColorOcTree::StaticMemberInitializer RtabmapColorOcTree::RtabmapColorOcTreeMemberInit;
#endif


//////////////////////////////////////
// OctoMap
//////////////////////////////////////

OctoMap::OctoMap(const LocalGridCache * cache, const ParametersMap & parameters) :
		GlobalMap(cache, parameters),
		hasColor_(false),
		rangeMax_(Parameters::defaultGridRangeMax()),
		rayTracing_(Parameters::defaultGridRayTracing()),
		emptyFloodFillDepth_(Parameters::defaultGridGlobalFloodFillDepth())
{
	octree_ = new RtabmapColorOcTree(cellSize_);
	if(occupancyThr_ <= 0.0f)
	{
		UWARN("Cannot set %s to null for OctoMap, using default value %f instead.",
				Parameters::kGridGlobalOccupancyThr().c_str(),
				Parameters::defaultGridGlobalOccupancyThr());
		occupancyThr_ = Parameters::defaultGridGlobalOccupancyThr();
	}
	
	UDEBUG("occupancyThr_=%f", occupancyThr_);
	UDEBUG("probHit_=%f", probability(logOddsHit_));
	UDEBUG("probMiss_=%f", probability(logOddsMiss_));
	UDEBUG("probClampingMin_=%f", probability(logOddsClampingMin_));
	UDEBUG("probClampingMax_=%f", probability(logOddsClampingMax_));
	
	octree_->setOccupancyThres(occupancyThr_);
	octree_->setProbHit(probability(logOddsHit_));
	octree_->setProbMiss(probability(logOddsMiss_));
	octree_->setClampingThresMin(probability(logOddsClampingMin_));
	octree_->setClampingThresMax(probability(logOddsClampingMax_));

	Parameters::parse(parameters, Parameters::kGridRangeMax(), rangeMax_);
	Parameters::parse(parameters, Parameters::kGridRayTracing(), rayTracing_);

	Parameters::parse(parameters, Parameters::kGridGlobalFloodFillDepth(), emptyFloodFillDepth_);
	UASSERT(emptyFloodFillDepth_>=0 && emptyFloodFillDepth_<=16);

	UDEBUG("rangeMax_           =%f", rangeMax_);
	UDEBUG("rayTracing_         =%s", rayTracing_?"true":"false");
	UDEBUG("emptyFloodFillDepth_=%d", emptyFloodFillDepth_);
}

OctoMap::~OctoMap()
{
	this->clear();
	delete octree_;
}

void OctoMap::clear()
{
	octree_->clear();
	hasColor_ = false;
	GlobalMap::clear();
}

unsigned long OctoMap::getMemoryUsed() const
{
	unsigned long memoryUsage = GlobalMap::getMemoryUsed();

	// Note: size of OctoMap object is missing.

	return memoryUsage;
}

bool OctoMap::isValidEmpty(RtabmapColorOcTree* octree_, unsigned int treeDepth,octomap::point3d startPosition)
{
    auto nodePtr = octree_->search(startPosition.x(), startPosition.y(), startPosition.z(), treeDepth);
    if(nodePtr != NULL)
    {
            if(!octree_->isNodeOccupied(*nodePtr))
            {
                return true;
            }
    }

    return false;
}

octomap::point3d OctoMap::findCloseEmpty(RtabmapColorOcTree* octree_, unsigned int treeDepth,octomap::point3d startPosition)
{

    //try current position
    if(isValidEmpty(octree_,treeDepth,startPosition))
    {
        return startPosition;
    }

    //x pos
    if(isValidEmpty(octree_,treeDepth,octomap::point3d(startPosition.x()+octree_->getNodeSize(treeDepth), startPosition.y(), startPosition.z())))
    {
        return octomap::point3d(startPosition.x()+octree_->getNodeSize(treeDepth), startPosition.y(), startPosition.z());
    }
     
    //x neg 
    if(isValidEmpty(octree_,treeDepth,octomap::point3d(startPosition.x()-octree_->getNodeSize(treeDepth), startPosition.y(), startPosition.z())))
    {
        return octomap::point3d(startPosition.x()-octree_->getNodeSize(treeDepth), startPosition.y(), startPosition.z());
    }

    //y pos
    if(isValidEmpty(octree_,treeDepth,octomap::point3d(startPosition.x(), startPosition.y()+octree_->getNodeSize(treeDepth), startPosition.z())))
    {
        return octomap::point3d(startPosition.x(), startPosition.y()+octree_->getNodeSize(treeDepth), startPosition.z());
    }


    //y neg
    if(isValidEmpty(octree_,treeDepth,octomap::point3d(startPosition.x()-octree_->getNodeSize(treeDepth), startPosition.y(), startPosition.z())))
    {
        return octomap::point3d(startPosition.x(), startPosition.y()-octree_->getNodeSize(treeDepth), startPosition.z());
    }

    //z pos
    if(isValidEmpty(octree_,treeDepth,octomap::point3d(startPosition.x(), startPosition.y(), startPosition.z()+octree_->getNodeSize(treeDepth))))
    {
        return octomap::point3d(startPosition.x(), startPosition.y(), startPosition.z()+octree_->getNodeSize(treeDepth));
    }

    //z neg
    if(isValidEmpty(octree_,treeDepth,octomap::point3d(startPosition.x(), startPosition.y(), startPosition.z()-octree_->getNodeSize(treeDepth))))
    {
        return octomap::point3d(startPosition.x(), startPosition.y(), startPosition.z()-octree_->getNodeSize(treeDepth));
    }

    //no valid position
    return startPosition;
}

bool OctoMap::isNodeVisited(std::unordered_set<octomap::OcTreeKey,octomap::OcTreeKey::KeyHash> const & EmptyNodes,octomap::OcTreeKey const key)
{
    for(auto it = EmptyNodes.find(key);it != EmptyNodes.end();it++)
    { 
        if(*it == key)
        {
            return true;
        }
                
    }        
    return false;
}

void OctoMap::floodFill(RtabmapColorOcTree* octree_, unsigned int treeDepth,octomap::point3d startPosition, std::unordered_set<octomap::OcTreeKey, octomap::OcTreeKey::KeyHash> & EmptyNodes,std::queue<octomap::point3d>& positionToExplore)
{
    
    auto key = octree_->coordToKey(startPosition,treeDepth);
    if(!isNodeVisited(EmptyNodes,key))
    {
        if(isValidEmpty(octree_,treeDepth,startPosition))
        {
            EmptyNodes.insert(key);
            positionToExplore.push(octomap::point3d(startPosition.x()+octree_->getNodeSize(treeDepth), startPosition.y(), startPosition.z()));
            positionToExplore.push(octomap::point3d(startPosition.x()-octree_->getNodeSize(treeDepth), startPosition.y(), startPosition.z()));
            positionToExplore.push(octomap::point3d(startPosition.x(), startPosition.y()+octree_->getNodeSize(treeDepth), startPosition.z()));
            positionToExplore.push(octomap::point3d(startPosition.x(), startPosition.y()-octree_->getNodeSize(treeDepth), startPosition.z()));
            positionToExplore.push(octomap::point3d(startPosition.x(), startPosition.y(), startPosition.z()+octree_->getNodeSize(treeDepth)));
            positionToExplore.push(octomap::point3d(startPosition.x(), startPosition.y(), startPosition.z()-octree_->getNodeSize(treeDepth)));
        }         
    }
}


std::unordered_set<octomap::OcTreeKey, octomap::OcTreeKey::KeyHash> OctoMap::findEmptyNode(RtabmapColorOcTree* octree_, unsigned int treeDepth, octomap::point3d startPosition)
{
    std::unordered_set<octomap::OcTreeKey, octomap::OcTreeKey::KeyHash> exploreNode;
    std::queue<octomap::point3d> positionToExplore;
    
    startPosition = findCloseEmpty(octree_,treeDepth, startPosition);
    
    floodFill(octree_, treeDepth, startPosition, exploreNode, positionToExplore);
    while(!positionToExplore.empty())
    {    
        floodFill(octree_, treeDepth, positionToExplore.front(), exploreNode, positionToExplore);
        positionToExplore.pop();
    }

    return exploreNode;
}


void OctoMap::assemble(const std::list<std::pair<int, Transform> > & newPoses)
{
	// Original version from A. Hornung:
	// https://github.com/OctoMap/octomap_mapping/blob/jade-devel/octomap_server/src/OctomapServer.cpp#L356
	//

	int lastId = assembledNodes().size()?assembledNodes().rbegin()->first:0;
	UDEBUG("Last id = %d", lastId);

	UDEBUG("newPoses = %d", (int)newPoses.size());

	if(!newPoses.empty())
	{
		float rangeMaxSqrd = rangeMax_*rangeMax_;
		float cellSize = octree_->getResolution();
		for(std::list<std::pair<int, Transform> >::const_iterator iter=newPoses.begin(); iter!=newPoses.end(); ++iter)
		{
			std::map<int, LocalGrid>::const_iterator localGridIter;
			localGridIter = cache().find(iter->first);
			if(localGridIter != cache().end())
			{
				cv::Mat ground = localGridIter->second.groundCells;
				cv::Mat obstacles = localGridIter->second.obstacleCells;
				cv::Mat emptyCells = localGridIter->second.emptyCells;

				if(!localGridIter->second.is3D())
				{
					UWARN("It seems the local occupancy grids are not 3d, cannot update OctoMap! (ground type=%d, obstacles type=%d, empty type=%d)",
											ground.type(), obstacles.type(), emptyCells.type());
					continue;
				}

				UDEBUG("Adding %d to octomap (resolution=%f)", iter->first, octree_->getResolution());

				octomap::point3d sensorOrigin(iter->second.x(), iter->second.y(), iter->second.z());
				sensorOrigin += octomap::point3d(localGridIter->second.viewPoint.x, localGridIter->second.viewPoint.y, localGridIter->second.viewPoint.z);

				updateMinMax(sensorOrigin);

				octomap::OcTreeKey tmpKey;
				if (!octree_->coordToKeyChecked(sensorOrigin, tmpKey))
				{
					UERROR("Could not generate Key for origin ", sensorOrigin.x(), sensorOrigin.y(), sensorOrigin.z());
				}

				bool computeRays = rayTracing_ && emptyCells.empty();

				// instead of direct scan insertion, compute update to filter ground:
				octomap::KeySet free_cells;
				// insert ground points only as free:
				unsigned int maxGroundPts = ground.cols;
				UDEBUG("%d: compute free cells (from %d ground points)", iter->first, (int)maxGroundPts);
				Eigen::Affine3f t = iter->second.toEigen3f();
				LaserScan tmpGround = LaserScan::backwardCompatibility(ground);
				UASSERT(tmpGround.size() == (int)maxGroundPts);
				for (unsigned int i=0; i<maxGroundPts; ++i)
				{
					pcl::PointXYZRGB pt;
					pt = util3d::laserScanToPointRGB(tmpGround, i);
					pt = pcl::transformPoint(pt, t);

					octomap::point3d point(pt.x, pt.y, pt.z);
					bool ignoreOccupiedCell = false;
					if(rangeMaxSqrd > 0.0f)
					{
						octomap::point3d v(pt.x - cellSize - sensorOrigin.x(), pt.y - cellSize - sensorOrigin.y(), pt.z - cellSize - sensorOrigin.z());
						if(v.norm_sq() > rangeMaxSqrd)
						{
							// compute new point to max range
							v.normalize();
							v*=rangeMax_;
							point = sensorOrigin + v;
							ignoreOccupiedCell=true;
						}
					}

					if(!ignoreOccupiedCell)
					{
						// occupied endpoint
						octomap::OcTreeKey key;
						if (octree_->coordToKeyChecked(point, key))
						{
							if(iter->first >0 && iter->first<lastId)
							{
								RtabmapColorOcTreeNode * n = octree_->search(key);
								if(n && n->getNodeRefId() > 0 && n->getNodeRefId() > iter->first)
								{
									// The cell has been updated from more recent node, don't update the cell
									continue;
								}
							}

							updateMinMax(point);
							RtabmapColorOcTreeNode * n = octree_->updateNode(key, true);

							if(n)
							{
								if(!hasColor_ && !(pt.r ==0 && pt.g == 0 && pt.b == 0) && !(pt.r ==255 && pt.g == 255 && pt.b == 255))
								{
									hasColor_ = true;
								}
								octree_->averageNodeColor(key, pt.r, pt.g, pt.b);
								if(iter->first > 0)
								{
									n->setNodeRefId(iter->first);
									n->setPointRef(point);
								}
								n->setOccupancyType(RtabmapColorOcTreeNode::kTypeGround);
							}
						}
					}

					// only clear space (ground points)
					octomap::KeyRay keyRay;
					if (computeRays &&
						(iter->first < 0 || iter->first>lastId) &&
						octree_->computeRayKeys(sensorOrigin, point, keyRay))
					{
						free_cells.insert(keyRay.begin(), keyRay.end());
					}
				}
				UDEBUG("%d: ground cells=%d free cells=%d", iter->first, (int)maxGroundPts, (int)free_cells.size());

				// all other points: free on ray, occupied on endpoint:
				unsigned int maxObstaclePts = obstacles.cols;
				UDEBUG("%d: compute occupied cells (from %d obstacle points)", iter->first, (int)maxObstaclePts);
				LaserScan tmpObstacle = LaserScan::backwardCompatibility(obstacles);
				UASSERT(tmpObstacle.size() == (int)maxObstaclePts);
				for (unsigned int i=0; i<maxObstaclePts; ++i)
				{
					pcl::PointXYZRGB pt;
					pt = util3d::laserScanToPointRGB(tmpObstacle, i);
					pt = pcl::transformPoint(pt, t);

					octomap::point3d point(pt.x, pt.y, pt.z);

					bool ignoreOccupiedCell = false;
					if(rangeMaxSqrd > 0.0f)
					{
						octomap::point3d v(pt.x - cellSize - sensorOrigin.x(), pt.y - cellSize - sensorOrigin.y(), pt.z - cellSize - sensorOrigin.z());
						if(v.norm_sq() > rangeMaxSqrd)
						{
							// compute new point to max range
							v.normalize();
							v*=rangeMax_;
							point = sensorOrigin + v;
							ignoreOccupiedCell=true;
						}
					}

					if(!ignoreOccupiedCell)
					{
						// occupied endpoint
						octomap::OcTreeKey key;
						if (octree_->coordToKeyChecked(point, key))
						{
							if(iter->first >0 && iter->first<lastId)
							{
								RtabmapColorOcTreeNode * n = octree_->search(key);
								if(n && n->getNodeRefId() > 0 && n->getNodeRefId() > iter->first)
								{
									// The cell has been updated from more recent node, don't update the cell
									continue;
								}
							}

							updateMinMax(point);

							RtabmapColorOcTreeNode * n = octree_->updateNode(key, true);
							if(n)
							{
								if(!hasColor_ && !(pt.r ==0 && pt.g == 0 && pt.b == 0) && !(pt.r ==255 && pt.g == 255 && pt.b == 255))
								{
									hasColor_ = true;
								}
								octree_->averageNodeColor(key, pt.r, pt.g, pt.b);
								if(iter->first > 0)
								{
									n->setNodeRefId(iter->first);
									n->setPointRef(point);
								}
								n->setOccupancyType(RtabmapColorOcTreeNode::kTypeObstacle);
							}
						}
					}

					// free cells
					octomap::KeyRay keyRay;
					if (computeRays &&
						(iter->first < 0 || iter->first>lastId) &&
						octree_->computeRayKeys(sensorOrigin, point, keyRay))
					{
						free_cells.insert(keyRay.begin(), keyRay.end());
					}
				}
				UDEBUG("%d: occupied cells=%d free cells=%d", iter->first, (int)maxObstaclePts, (int)free_cells.size());


				// mark free cells only if not seen occupied in this cloud
				for(octomap::KeySet::iterator it = free_cells.begin(), end=free_cells.end(); it!= end; ++it)
				{
					if(iter->first > 0)
					{
						RtabmapColorOcTreeNode * n = octree_->search(*it);
						if(n && n->getNodeRefId() > 0 && n->getNodeRefId() >= iter->first)
						{
							// The cell has been updated from current node or more recent node, don't update the cell
							continue;
						}
					}

					RtabmapColorOcTreeNode * n = octree_->updateNode(*it, false,  true);
					if(n && n->getOccupancyType() == RtabmapColorOcTreeNode::kTypeUnknown)
					{
						n->setOccupancyType(RtabmapColorOcTreeNode::kTypeEmpty);
						if(iter->first > 0)
						{
							n->setNodeRefId(iter->first);
						}
					}
				}

				// all empty cells
				if(emptyCells.cols)
				{
					unsigned int maxEmptyPts = emptyCells.cols;
					UDEBUG("%d: compute free cells (from %d empty points)", iter->first, (int)maxEmptyPts);
					LaserScan tmpEmpty = LaserScan::backwardCompatibility(emptyCells);
					UASSERT(tmpEmpty.size() == (int)maxEmptyPts);
					for (unsigned int i=0; i<maxEmptyPts; ++i)
					{
						pcl::PointXYZ pt;
						pt = util3d::laserScanToPoint(tmpEmpty, i);
						pt = pcl::transformPoint(pt, t);

						octomap::point3d point(pt.x, pt.y, pt.z);

						bool ignoreCell = false;
						if(rangeMaxSqrd > 0.0f)
						{
							octomap::point3d v(pt.x - sensorOrigin.x(), pt.y - sensorOrigin.y(), pt.z - sensorOrigin.z());
							if(v.norm_sq() > rangeMaxSqrd)
							{
								ignoreCell=true;
							}
						}

						if(!ignoreCell)
						{
							octomap::OcTreeKey key;
							if (octree_->coordToKeyChecked(point, key))
							{

								if(iter->first >0)
								{
									RtabmapColorOcTreeNode * n = octree_->search(key);
									if(n && n->getNodeRefId() > 0 && n->getNodeRefId() >= iter->first)
									{
										// The cell has been updated from current node or more recent node, don't update the cell
										continue;
									}
								}

								updateMinMax(point);

								RtabmapColorOcTreeNode * n = octree_->updateNode(key, false, true);
								if(n && n->getOccupancyType() == RtabmapColorOcTreeNode::kTypeUnknown)
								{
									n->setOccupancyType(RtabmapColorOcTreeNode::kTypeEmpty);
									if(iter->first > 0)
									{
										n->setNodeRefId(iter->first);
									}
								}
							}
						}
					}
				}

				if(emptyCells.cols || !free_cells.empty())
				{
					octree_->updateInnerOccupancy();
				}

				// compress map
				//if(newPoses.size() > 1)
				//{
				//	octree_->prune();
				//}

				addAssembledNode(iter->first, iter->second);
				UDEBUG("%d: end", iter->first);
			}
			else
			{
				UDEBUG("Did not find %d in cache", iter->first);
			}
		}
	}

    if(emptyFloodFillDepth_>0)
    { 
    	UTimer t;

        auto key = octree_->coordToKey(0, 0, 0, emptyFloodFillDepth_);
        auto pos = octree_->keyToCoord(key);
       
        std::unordered_set<octomap::OcTreeKey, octomap::OcTreeKey::KeyHash> EmptyNodes = findEmptyNode(octree_,emptyFloodFillDepth_, pos);
        std::vector<octomap::OcTreeKey> nodeToDelete;
        
        for (RtabmapColorOcTree::iterator it = octree_->begin_leafs(emptyFloodFillDepth_); it != octree_->end_leafs(); ++it)
	    {
            if(!octree_->isNodeOccupied(*it))
            {
                if(!isNodeVisited(EmptyNodes,it.getKey()))
                {   
                    nodeToDelete.push_back(it.getKey());            
                }
            }
        }

        for(unsigned int y=0; y < nodeToDelete.size(); y++)
        {
            octree_->deleteNode(nodeToDelete[y],emptyFloodFillDepth_);
        }
        UDEBUG("Flood Fill: deleted %d empty cells (%fs)", (int)nodeToDelete.size(), t.ticks());
    }
}

void OctoMap::updateMinMax(const octomap::point3d & point)
{
	if(point.x() < minValues_[0])
	{
		minValues_[0] = point.x();
	}
	if(point.y() < minValues_[1])
	{
		minValues_[1] = point.y();
	}
	if(point.z() < minValues_[2])
	{
		minValues_[2] = point.z();
	}
	if(point.x() > maxValues_[0])
	{
		maxValues_[0] = point.x();
	}
	if(point.y() > maxValues_[1])
	{
		maxValues_[1] = point.y();
	}
	if(point.z() > maxValues_[2])
	{
		maxValues_[2] = point.z();
	}
}


pcl::PointCloud<pcl::PointXYZRGB>::Ptr OctoMap::createCloud(
		unsigned int treeDepth,
		std::vector<int> * obstacleIndices,
		std::vector<int> * emptyIndices,
		std::vector<int> * groundIndices,
		bool originalRefPoints,
		std::vector<int> * frontierIndices,
		std::vector<double> * cloudProb) const
{
	UASSERT(treeDepth <= octree_->getTreeDepth());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	if(cloudProb)
	{
		cloudProb->resize(octree_->size());
	}
	UDEBUG("depth=%d (maxDepth=%d) octree = %d",
			(int)treeDepth, (int)octree_->getTreeDepth(), (int)octree_->size());
	cloud->resize(octree_->size());
	if(obstacleIndices)
	{
		obstacleIndices->resize(octree_->size());
	}
	if(emptyIndices)
	{
		emptyIndices->resize(octree_->size());
	}
	if(frontierIndices)
	{
		frontierIndices->resize(octree_->size());
	}
	if(groundIndices)
	{
		groundIndices->resize(octree_->size());
	}

	if(treeDepth == 0)
	{
		treeDepth = octree_->getTreeDepth();
	}

	double minZ = minValues_[2];
	double maxZ = maxValues_[2];

	bool addAllPoints = obstacleIndices == 0 && groundIndices == 0 && emptyIndices == 0;
	int oi=0;
	int si=0;
	int ei=0;
	int fi=0;
	int gi=0;
	float halfCellSize = octree_->getNodeSize(treeDepth)/2.0f;

	for (RtabmapColorOcTree::iterator it = octree_->begin(treeDepth); it != octree_->end(); ++it)
	{
		if(octree_->isNodeOccupied(*it) && (obstacleIndices != 0 || groundIndices != 0 || addAllPoints))
		{
			octomap::point3d pt = octree_->keyToCoord(it.getKey());
			if(cloudProb)
			{
				(*cloudProb)[oi] = it->getOccupancy();
			}
			if(octree_->getTreeDepth() == it.getDepth() && hasColor_)
			{
				(*cloud)[oi]  = pcl::PointXYZRGB(it->getColor().r, it->getColor().g, it->getColor().b);
			}
			else
			{
				// Gradiant color on z axis
				float H = (maxZ - pt.z())*299.0f/(maxZ-minZ);
				float r,g,b;
				util2d::HSVtoRGB(&r, &g, &b, H, 1, 1);
				(*cloud)[oi].r = r*255.0f;
				(*cloud)[oi].g = g*255.0f;
				(*cloud)[oi].b = b*255.0f;
			}

			if(originalRefPoints && it->getOccupancyType() > 0)
			{
				const octomap::point3d & p = it->getPointRef();
				(*cloud)[oi].x = p.x();
				(*cloud)[oi].y = p.y();
				(*cloud)[oi].z = p.z();
			}
			else
			{
				(*cloud)[oi].x = pt.x()-halfCellSize;
				(*cloud)[oi].y = pt.y()-halfCellSize;
				(*cloud)[oi].z = pt.z();
			}

			if(it->getOccupancyType() == RtabmapColorOcTreeNode::kTypeGround)
			{
				if(groundIndices)
				{
					groundIndices->at(gi++) = oi;
				}
			}
			else if(obstacleIndices)
			{
				obstacleIndices->at(si++) = oi;
			}

			++oi;
		}
		else if(!octree_->isNodeOccupied(*it) && (emptyIndices != 0 || addAllPoints || frontierIndices !=0))
		{
			octomap::point3d pt = octree_->keyToCoord(it.getKey());
			if(cloudProb)
			{
				(*cloudProb)[oi] = it->getOccupancy();
			}
            if(frontierIndices !=0 &&
                (!octree_->search( pt.x()+octree_->getNodeSize(treeDepth), pt.y(), pt.z(), treeDepth) || !octree_->search( pt.x()-octree_->getNodeSize(treeDepth), pt.y(), pt.z(), treeDepth) ||
                !octree_->search( pt.x(), pt.y()+octree_->getNodeSize(treeDepth), pt.z(), treeDepth) || !octree_->search( pt.x(), pt.y()-octree_->getNodeSize(treeDepth), pt.z(), treeDepth) ||
                !octree_->search( pt.x(), pt.y(), pt.z()+octree_->getNodeSize(treeDepth), treeDepth) || !octree_->search( pt.x(), pt.y(), pt.z()-octree_->getNodeSize(treeDepth), treeDepth) )) //ajouter 1 au key ?
            {
                //unknown neighbor FACE cell
                frontierIndices->at(fi++) = oi;
            }
            
			
            (*cloud)[oi]  = pcl::PointXYZRGB(it->getColor().r, it->getColor().g, it->getColor().b);
            (*cloud)[oi].x = pt.x()-halfCellSize;
            (*cloud)[oi].y = pt.y()-halfCellSize;
            (*cloud)[oi].z = pt.z();
          
            if(emptyIndices)
            {
                emptyIndices->at(ei++) = oi;
            }
            
			++oi;
		}	
	}

	cloud->resize(oi);
	if(cloudProb)
	{
		cloudProb->resize(oi);
	}
	if(obstacleIndices)
	{
		obstacleIndices->resize(si);
		UDEBUG("obstacle=%d", si);
	}
	if(emptyIndices)
	{
		emptyIndices->resize(ei);
		UDEBUG("empty=%d", ei);
	}
	if(frontierIndices)
	{
		frontierIndices->resize(fi);
		UDEBUG("frontier=%d", fi);
	}
	if(groundIndices)
	{
		groundIndices->resize(gi);
		UDEBUG("ground=%d", gi);
	}

	UDEBUG("");
	return cloud;
}

cv::Mat OctoMap::createProjectionMap(float & xMin, float & yMin, float & gridCellSize, float minGridSize, unsigned int treeDepth)
{
	UDEBUG("minGridSize=%f, treeDepth=%d", minGridSize, (int)treeDepth);
	UASSERT(treeDepth <= octree_->getTreeDepth());
	if(treeDepth == 0)
	{
		treeDepth = octree_->getTreeDepth();
	}

	gridCellSize = octree_->getNodeSize(treeDepth);

	cv::Mat obstaclesMat = cv::Mat(1, (int)octree_->size(), CV_32FC2);
	cv::Mat groundMat = cv::Mat(1, (int)octree_->size(), CV_32FC2);
	int gi=0;
	int oi=0;
	cv::Vec2f * oPtr = obstaclesMat.ptr<cv::Vec2f>(0,0);
	cv::Vec2f * gPtr = groundMat.ptr<cv::Vec2f>(0,0);
	float halfCellSize = octree_->getNodeSize(treeDepth)/2.0f;
	for (RtabmapColorOcTree::iterator it = octree_->begin(treeDepth); it != octree_->end(); ++it)
	{
		octomap::point3d pt = octree_->keyToCoord(it.getKey());
		if(octree_->isNodeOccupied(*it) &&
		   it->getOccupancyType() == RtabmapColorOcTreeNode::kTypeObstacle)
		{
			// projected on ground
			oPtr[oi][0] = pt.x()-halfCellSize;
			oPtr[oi][1] = pt.y()-halfCellSize;
			++oi;
		}
		else
		{
			// projected on ground
			gPtr[gi][0] = pt.x()-halfCellSize;
			gPtr[gi][1] = pt.y()-halfCellSize;
			++gi;
		}
	}
	obstaclesMat = obstaclesMat(cv::Range::all(), cv::Range(0, oi));
	groundMat = groundMat(cv::Range::all(), cv::Range(0, gi));

	std::map<int, Transform> poses;
	poses.insert(std::make_pair(1, Transform::getIdentity()));
	std::map<int, std::pair<cv::Mat, cv::Mat> > maps;
	maps.insert(std::make_pair(1, std::make_pair(groundMat, obstaclesMat)));

	cv::Mat map = util3d::create2DMapFromOccupancyLocalMaps(
			poses,
			maps,
			gridCellSize,
			xMin, yMin,
			minGridSize,
			false);
	UDEBUG("");
	return map;
}

bool OctoMap::writeBinary(const std::string & path)
{
	return octree_->writeBinary(path);
}

} /* namespace rtabmap */
