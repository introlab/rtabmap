/*
Copyright (c) 2010-2014, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
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

#include <rtabmap/core/OctoMap.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/core/util3d_transforms.h>
#include <rtabmap/core/util3d_filtering.h>
#include <rtabmap/core/util3d_mapping.h>

namespace rtabmap {

OctoMap::OctoMap(float voxelSize) :
		octree_(new octomap::ColorOcTree(voxelSize))
{
	UASSERT(voxelSize>0.0f);
}

OctoMap::~OctoMap()
{
	this->clear();
	delete octree_;
}

void OctoMap::clear()
{
	octree_->clear();
	occupiedCells_.clear();
	cache_.clear();
	addedNodes_.clear();
	keyRay_ = octomap::KeyRay();
}

void OctoMap::addToCache(int nodeId,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr & ground,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr & obstacles)
{
	cache_.insert(std::make_pair(nodeId, std::make_pair(ground, obstacles)));
}

void OctoMap::update(const std::map<int, Transform> & poses)
{
	UDEBUG("Update (poses=%d addedNodes_=%d)", (int)poses.size(), (int)addedNodes_.size());

	// First, check of the graph has changed. If so, re-create the octree by moving all occupied nodes.
	bool graphChanged = false;
	std::map<int, Transform> transforms;
	std::map<int, Transform> updatedAddedNodes;
	for(std::map<int, Transform>::iterator iter=addedNodes_.begin(); iter!=addedNodes_.end(); ++iter)
	{
		std::map<int, Transform>::const_iterator jter = poses.find(iter->first);
		if(jter != poses.end())
		{
			UASSERT(!iter->second.isNull() && !jter->second.isNull());
			Transform t = Transform::getIdentity();
			if(iter->second.getDistanceSquared(jter->second) > 0.0001)
			{
				t = jter->second * iter->second.inverse();
				graphChanged = true;
			}
			transforms.insert(std::make_pair(jter->first, t));
			updatedAddedNodes.insert(std::make_pair(jter->first, jter->second));
		}
		else
		{
			UWARN("Updated pose for node %d is not found, some points may not be copied.", jter->first);
		}
	}
	if(graphChanged)
	{
		UWARN("Graph changed!");
		octomap::ColorOcTree * newOcTree = new octomap::ColorOcTree(octree_->getResolution());
		std::map<octomap::ColorOcTreeNode*, OcTreeNodeInfo > newOccupiedCells;
		int copied=0;
		for(std::map<octomap::ColorOcTreeNode*, OcTreeNodeInfo >::iterator iter = occupiedCells_.begin();
				iter!=occupiedCells_.end();
				++iter)
		{
			std::map<int, Transform>::iterator jter = transforms.find(iter->second.nodeRefId_);
			if(jter != transforms.end())
			{
				octomap::point3d pt = octree_->keyToCoord(iter->second.key_);
				std::map<int, Transform>::iterator pter = addedNodes_.find(iter->second.nodeRefId_);
				UASSERT(pter != addedNodes_.end());

				cv::Point3f cvPt(pt.x(), pt.y(), pt.z());
				cvPt = util3d::transformPoint(cvPt, jter->second);

				octomap::OcTreeKey key;
				if(newOcTree->coordToKeyChecked(cvPt.x, cvPt.y, cvPt.z, key))
				{
					octomap::ColorOcTreeNode * n = newOcTree->updateNode(key, true);
					if(n)
					{
						++copied;
						uInsert(newOccupiedCells, std::make_pair(n, OcTreeNodeInfo(jter->first, key, iter->second.isObstacle_)));
						newOcTree->setNodeColor(key, iter->first->getColor().r, iter->first->getColor().g, iter->first->getColor().b);
					}
					else
					{
						UERROR("Could not update node at (%f,%f,%f)", cvPt.x, cvPt.y, cvPt.z);
					}
				}
				else
				{
					UERROR("Could not find key for (%f,%f,%f)", cvPt.x, cvPt.y, cvPt.z);
				}
			}
			else if(jter == transforms.end() && iter->second.nodeRefId_ > 0)
			{
				UWARN("Could not find a transform for point linked to node %d (transforms=%d)", iter->second.nodeRefId_, (int)transforms.size());
			}
		}
		UDEBUG("%d/%d", copied, (int)occupiedCells_.size());
		delete octree_;
		octree_ = newOcTree;
		occupiedCells_ = newOccupiedCells;

		//update added poses
		addedNodes_ = updatedAddedNodes;
	}

	// Original version from A. Hornung:
	// https://github.com/OctoMap/octomap_mapping/blob/jade-devel/octomap_server/src/OctomapServer.cpp#L356
	//
	int lastId = addedNodes_.size()?addedNodes_.rbegin()->first:0;
	UDEBUG("Last id = %d", lastId);
	if(lastId >= 0)
	{
		std::list<std::pair<int, Transform> > orderedPoses;
		for(std::map<int, Transform>::const_iterator iter=poses.upper_bound(lastId); iter!=poses.end(); ++iter)
		{
			orderedPoses.push_back(*iter);
		}
		// insert negative after
		for(std::map<int, Transform>::const_iterator iter=poses.begin(); iter!=poses.end(); ++iter)
		{
			if(iter->first < 0)
			{
				orderedPoses.push_back(*iter);
			}
			else
			{
				break;
			}
		}

		UDEBUG("orderedPoses = %d", (int)orderedPoses.size());
		for(std::list<std::pair<int, Transform> >::const_iterator iter=orderedPoses.begin(); iter!=orderedPoses.end(); ++iter)
		{
			std::map<int, std::pair<pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr> >::iterator cloudIter;
			cloudIter = cache_.find(iter->first);
			if(cloudIter != cache_.end())
			{
				UDEBUG("Adding %d to octomap (resolution=%f)", iter->first, octree_->getResolution());

				octomap::point3d sensorOrigin(iter->second.x(), iter->second.y(), iter->second.z());
				octomap::OcTreeKey tmpKey;
				if (!octree_->coordToKeyChecked(sensorOrigin, tmpKey)
						|| !octree_->coordToKeyChecked(sensorOrigin, tmpKey))
				{
					UERROR("Could not generate Key for origin ", sensorOrigin.x(), sensorOrigin.y(), sensorOrigin.z());
				}

				// instead of direct scan insertion, compute update to filter ground:
				octomap::KeySet free_cells, occupied_cells, ground_cells;
				// insert ground points only as free:
				UDEBUG("%d: compute free cells (from %d ground points)", iter->first, (int)cloudIter->second.first->size());
				for (unsigned int i=0; i<cloudIter->second.first->size(); ++i)
				{
					pcl::PointXYZRGB pt = util3d::transformPoint(cloudIter->second.first->at(i), iter->second);

					octomap::point3d point(pt.x, pt.y, pt.z);

					// only clear space (ground points)
					if (octree_->computeRayKeys(sensorOrigin, point, keyRay_))
					{
						free_cells.insert(keyRay_.begin(), keyRay_.end());
					}
					// occupied endpoint
					octomap::OcTreeKey key;
					if (octree_->coordToKeyChecked(point, key))
					{
						ground_cells.insert(key);

						octomap::ColorOcTreeNode * n = octree_->updateNode(key, false);
						if(n)
						{
							octree_->averageNodeColor(key, pt.r, pt.g, pt.b);
							if(iter->first > 0)
							{
								uInsert(occupiedCells_, std::make_pair(n, OcTreeNodeInfo(iter->first, key, false)));
							}
							else
							{
								occupiedCells_.insert(std::make_pair(n, OcTreeNodeInfo(iter->first, key, false)));
							}
						}
					}
				}
				UDEBUG("%d: free cells = %d", iter->first, (int)free_cells.size());

				// all other points: free on ray, occupied on endpoint:
				UDEBUG("%d: compute occupied cells (from %d obstacle points)", iter->first, (int) cloudIter->second.second->size());
				for (unsigned int i=0; i<cloudIter->second.second->size(); ++i)
				{
					pcl::PointXYZRGB pt = util3d::transformPoint(cloudIter->second.second->at(i), iter->second);

					octomap::point3d point(pt.x, pt.y, pt.z);

					// free cells
					if (octree_->computeRayKeys(sensorOrigin, point, keyRay_))
					{
						free_cells.insert(keyRay_.begin(), keyRay_.end());
					}
					// occupied endpoint
					octomap::OcTreeKey key;
					if (octree_->coordToKeyChecked(point, key))
					{
						occupied_cells.insert(key);

						octomap::ColorOcTreeNode * n = octree_->updateNode(key, true);
						if(n)
						{
							octree_->averageNodeColor(key, pt.r, pt.g, pt.b);
							if(iter->first > 0)
							{
								uInsert(occupiedCells_, std::make_pair(n, OcTreeNodeInfo(iter->first, key, true)));
							}
							else
							{
								occupiedCells_.insert(std::make_pair(n, OcTreeNodeInfo(iter->first, key, true)));
							}
						}
					}
				}
				UDEBUG("%d: occupied cells=%d free cells=%d", iter->first, (int)occupied_cells.size(), (int)free_cells.size());


				// mark free cells only if not seen occupied in this cloud
				for(octomap::KeySet::iterator it = free_cells.begin(), end=free_cells.end(); it!= end; ++it)
				{
					if (occupied_cells.find(*it) == occupied_cells.end() &&
						ground_cells.find(*it) == ground_cells.end())
					{
						octomap::ColorOcTreeNode * n = octree_->updateNode(*it, false);
						if(n)
						{
							std::map<octomap::ColorOcTreeNode*, OcTreeNodeInfo>::iterator gter;
							gter = occupiedCells_.find(n);
							if(gter != occupiedCells_.end() && gter->second.isObstacle_)
							{
								occupiedCells_.erase(gter);
							}
						}
					}
				}

				// compress map
				//octree_->prune();

				// ignore negative ids as they are temporary clouds
				if(iter->first > 0)
				{
					addedNodes_.insert(*iter);
				}
				UDEBUG("%d: end", iter->first);
			}
			else
			{
				UWARN("Did not find %d in cache", iter->first);
			}
		}
	}
	cache_.clear();
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr OctoMap::createCloud(
		std::vector<int> * obstacleIndices,
		std::vector<int> * groundIndices) const
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	UDEBUG("occupied cells = %d", (int)occupiedCells_.size());
	cloud->resize(occupiedCells_.size());
	if(obstacleIndices)
	{
		obstacleIndices->resize(occupiedCells_.size());
	}
	if(groundIndices)
	{
		groundIndices->resize(occupiedCells_.size());
	}
	int oi=0;
	int si=0;
	int gi=0;
	for(std::map<octomap::ColorOcTreeNode*, OcTreeNodeInfo>::const_iterator iter = occupiedCells_.begin();
		iter!=occupiedCells_.end();
		++iter)
	{
		if(iter->second.isObstacle_ && octree_->isNodeOccupied(iter->first))
		{
			octomap::point3d pt = octree_->keyToCoord(iter->second.key_);
			(*cloud)[oi]  = pcl::PointXYZRGB(iter->first->getColor().r, iter->first->getColor().g, iter->first->getColor().b);
			(*cloud)[oi].x = pt.x();
			(*cloud)[oi].y = pt.y();
			(*cloud)[oi].z = pt.z();
			if(obstacleIndices)
			{
				obstacleIndices->at(si++) = oi;
			}
			++oi;
		}
		else if(!iter->second.isObstacle_)
		{
			octomap::point3d pt = octree_->keyToCoord(iter->second.key_);
			(*cloud)[oi]  = pcl::PointXYZRGB(iter->first->getColor().r, iter->first->getColor().g, iter->first->getColor().b);
			(*cloud)[oi].x = pt.x();
			(*cloud)[oi].y = pt.y();
			(*cloud)[oi].z = pt.z();
			if(groundIndices)
			{
				groundIndices->at(gi++) = oi;
			}
			++oi;
		}
	}
	cloud->resize(oi);
	if(obstacleIndices)
	{
		obstacleIndices->resize(si);
	}
	if(groundIndices)
	{
		groundIndices->resize(gi);
	}

	UDEBUG("");
	return cloud;
}

cv::Mat OctoMap::createProjectionMap(float & xMin, float & yMin, float & gridCellSize, float minGridSize)
{
	gridCellSize = octree_->getResolution();

	pcl::PointCloud<pcl::PointXYZ>::Ptr ground(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr obstacles(new pcl::PointCloud<pcl::PointXYZ>);

	ground->resize(occupiedCells_.size());
	obstacles->resize(occupiedCells_.size());
	int gi=0;
	int oi=0;
	for(std::map<octomap::ColorOcTreeNode*, OcTreeNodeInfo>::const_iterator iter = occupiedCells_.begin();
		iter!=occupiedCells_.end();
		++iter)
	{
		if(iter->second.isObstacle_ && octree_->isNodeOccupied(iter->first))
		{
			octomap::point3d pt = octree_->keyToCoord(iter->second.key_);
			(*obstacles)[oi++]  = pcl::PointXYZ(pt.x(), pt.y(), 0); // projected on ground
		}
		else if(!iter->second.isObstacle_)
		{
			octomap::point3d pt = octree_->keyToCoord(iter->second.key_);
			(*ground)[gi++]  = pcl::PointXYZ(pt.x(), pt.y(), 0); // projected on ground
		}
	}
	obstacles->resize(oi);
	ground->resize(gi);

	if(obstacles->size())
	{
		obstacles = util3d::voxelize(obstacles, gridCellSize);
	}
	if(ground->size())
	{
		ground = util3d::voxelize(ground, gridCellSize);
	}

	cv::Mat obstaclesMat = cv::Mat((int)obstacles->size(), 1, CV_32FC2);
	for(unsigned int i=0;i<obstacles->size(); ++i)
	{
		obstaclesMat.at<cv::Vec2f>(i)[0] = obstacles->at(i).x;
		obstaclesMat.at<cv::Vec2f>(i)[1] = obstacles->at(i).y;
	}

	cv::Mat groundMat = cv::Mat((int)ground->size(), 1, CV_32FC2);
	for(unsigned int i=0;i<ground->size(); ++i)
	{
		groundMat.at<cv::Vec2f>(i)[0] = ground->at(i).x;
		groundMat.at<cv::Vec2f>(i)[1] = ground->at(i).y;
	}

	std::map<int, Transform> poses;
	poses.insert(std::make_pair(1, Transform::getIdentity()));
	std::map<int, std::pair<cv::Mat, cv::Mat> > maps;
	maps.insert(std::make_pair(1, std::make_pair(groundMat, obstaclesMat)));

	return util3d::create2DMapFromOccupancyLocalMaps(
			poses,
			maps,
			gridCellSize,
			xMin, yMin,
			minGridSize,
			false);
}

} /* namespace rtabmap */
