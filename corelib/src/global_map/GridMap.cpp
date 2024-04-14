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

#include <rtabmap/core/global_map/GridMap.h>
#include <rtabmap/core/util3d_transforms.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/core/util3d_surface.h>
#include <rtabmap/core/CameraModel.h>
#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UStl.h>
#include <list>

#include <opencv2/photo.hpp>
#include <grid_map_core/iterators/GridMapIterator.hpp>
#include <pcl/io/pcd_io.h>

namespace rtabmap {

GridMap::GridMap(const LocalGridCache * cache, const ParametersMap & parameters) :
	GlobalMap(cache, parameters),
	minMapSize_(Parameters::defaultGridGlobalMinSize())
{
	Parameters::parse(parameters, Parameters::kGridGlobalMinSize(), minMapSize_);
}

void GridMap::clear()
{
	gridMap_ = grid_map::GridMap();
	GlobalMap::clear();
}

cv::Mat GridMap::createHeightMap(float & xMin, float & yMin, float & cellSize) const
{
	return toImage("elevation", xMin, yMin, cellSize);
}

cv::Mat GridMap::createColorMap(float & xMin, float & yMin, float & cellSize) const
{
	return toImage("colors", xMin, yMin, cellSize);
}

cv::Mat GridMap::toImage(const std::string & layer, float & xMin, float & yMin, float & cellSize) const
{
	if( gridMap_.hasBasicLayers())
	{
		const grid_map::Matrix& data = gridMap_[layer];

		cv::Mat image;
		if(layer.compare("elevation") == 0)
		{
			image = cv::Mat::zeros(gridMap_.getSize()(1), gridMap_.getSize()(0), CV_32FC1);
			for(grid_map::GridMapIterator iterator(gridMap_); !iterator.isPastEnd(); ++iterator) {
				const grid_map::Index index(*iterator);
				const float& value = data(index(0), index(1));
				const grid_map::Index imageIndex(iterator.getUnwrappedIndex());
				if (std::isfinite(value))
				{
					image.at<float>(image.rows-1-imageIndex(1), image.cols-1-imageIndex(0)) = value;
				}
			}
		}
		else if(layer.compare("colors") == 0)
		{
			image = cv::Mat::zeros(gridMap_.getSize()(1), gridMap_.getSize()(0), CV_8UC3);
			for(grid_map::GridMapIterator iterator(gridMap_); !iterator.isPastEnd(); ++iterator) {
				const grid_map::Index index(*iterator);
				const float& value = data(index(0), index(1));
				const grid_map::Index imageIndex(iterator.getUnwrappedIndex());
				if (std::isfinite(value))
				{
					const int * ptr = (const int *)&value;
					cv::Vec3b & color = image.at<cv::Vec3b>(image.rows-1-imageIndex(1), image.cols-1-imageIndex(0));
					color[0] = (unsigned char)(*ptr & 0xFF); // B
					color[1] = (unsigned char)((*ptr >> 8) & 0xFF); // G
					color[2] = (unsigned char)((*ptr >> 16) & 0xFF); // R
				}
			}
		}
		else
		{
			UFATAL("Unknown layer \"%s\"", layer.c_str());
		}

		xMin = gridMap_.getPosition().x() - gridMap_.getLength().x()/2.0f;
		yMin = gridMap_.getPosition().y() - gridMap_.getLength().y()/2.0f;
		cellSize = gridMap_.getResolution();

		return image;

	}
	return cv::Mat();
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr GridMap::createTerrainCloud() const
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	if( gridMap_.hasBasicLayers())
	{
		const grid_map::Matrix& dataElevation = gridMap_["elevation"];
		const grid_map::Matrix& dataColors = gridMap_["colors"];

		cloud->width = gridMap_.getSize()(0);
		cloud->height = gridMap_.getSize()(1);
		cloud->resize(cloud->width * cloud->height);
		cloud->is_dense = false;

		float xMin = gridMap_.getPosition().x() - gridMap_.getLength().x()/2.0f;
		float yMin = gridMap_.getPosition().y() - gridMap_.getLength().y()/2.0f;
		float cellSize = gridMap_.getResolution();

		for(grid_map::GridMapIterator iterator(gridMap_); !iterator.isPastEnd(); ++iterator)
		{
			const grid_map::Index index(*iterator);
			const float& value = dataElevation(index(0), index(1));
			const int* color = (const int*)&dataColors(index(0), index(1));
			const grid_map::Index imageIndex(iterator.getUnwrappedIndex());
			pcl::PointXYZRGB & pt = cloud->at(cloud->width-1-imageIndex(0), imageIndex(1));
			if (std::isfinite(value))
			{
				pt.x = xMin + (cloud->width-1-imageIndex(0)) * cellSize;
				pt.y = yMin + (cloud->height-1-imageIndex(1)) * cellSize;
				pt.z = value;
				pt.b = (unsigned char)(*color & 0xFF);
				pt.g = (unsigned char)((*color >> 8) & 0xFF);
				pt.r = (unsigned char)((*color >> 16) & 0xFF);
			}
			else
			{
				pt.x = pt.y = pt.z = std::numeric_limits<float>::quiet_NaN();
			}
		}
	}
	return cloud;
}

pcl::PolygonMesh::Ptr GridMap::createTerrainMesh() const
{
	pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = createTerrainCloud();
	if(!cloud->empty())
	{
		mesh->polygons = util3d::organizedFastMesh(
				cloud,
				M_PI,
				true,
				1);

		pcl::toPCLPointCloud2(*cloud, mesh->cloud);
	}

	return mesh;
}

void GridMap::assemble(const std::list<std::pair<int, Transform> > & newPoses)
{
	UTimer timer;

	float margin = cellSize_*10.0f;

	float minX=-minMapSize_/2.0f;
	float minY=-minMapSize_/2.0f;
	float maxX=minMapSize_/2.0f;
	float maxY=minMapSize_/2.0f;
	bool undefinedSize = minMapSize_ == 0.0f;
	std::map<int, cv::Mat> occupiedLocalMaps;

	if(gridMap_.hasBasicLayers())
	{
		// update
		minX=minValues_[0]+margin+cellSize_/2.0f;
		minY=minValues_[1]+margin+cellSize_/2.0f;
		maxX=minValues_[0]+float(gridMap_.getSize()[0])*cellSize_ - margin;
		maxY=minValues_[1]+float(gridMap_.getSize()[1])*cellSize_ - margin;
		undefinedSize = false;
	}

	for(std::list<std::pair<int, Transform> >::const_iterator iter = newPoses.begin(); iter!=newPoses.end(); ++iter)
	{
		UASSERT(!iter->second.isNull());

		float x = iter->second.x();
		float y  =iter->second.y();
		if(undefinedSize)
		{
			minX = maxX = x;
			minY = maxY = y;
			undefinedSize = false;
		}
		else
		{
			if(minX > x)
				minX = x;
			else if(maxX < x)
				maxX = x;

			if(minY > y)
				minY = y;
			else if(maxY < y)
				maxY = y;
		}
	}

	if(!cache().empty())
	{
		UDEBUG("Updating from cache");
		for(std::list<std::pair<int, Transform> >::const_iterator iter = newPoses.begin(); iter!=newPoses.end(); ++iter)
		{
			if(uContains(cache(), iter->first))
			{
				const LocalGrid & localGrid = cache().at(iter->first);

				if(!localGrid.is3D())
				{
					UWARN("It seems the local occupancy grids are not 3d, cannot update GridMap! (ground type=%d, obstacles type=%d, empty type=%d)",
							localGrid.groundCells.type(), localGrid.obstacleCells.type(), localGrid.emptyCells.type());
					continue;
				}

				UDEBUG("Adding grid %d: ground=%d obstacles=%d empty=%d", iter->first, localGrid.groundCells.cols, localGrid.obstacleCells.cols, localGrid.emptyCells.cols);

				//ground
				cv::Mat occupied;
				if(localGrid.groundCells.cols || localGrid.obstacleCells.cols)
				{
					occupied = cv::Mat(1, localGrid.groundCells.cols+localGrid.obstacleCells.cols, CV_32FC4);
				}
				if(localGrid.groundCells.cols)
				{
					if(localGrid.groundCells.rows > 1 && localGrid.groundCells.cols == 1)
					{
						UFATAL("Occupancy local maps should be 1 row and X cols! (rows=%d cols=%d)", localGrid.groundCells.rows, localGrid.groundCells.cols);
					}
					for(int i=0; i<localGrid.groundCells.cols; ++i)
					{
						const float * vi = localGrid.groundCells.ptr<float>(0,i);
						float * vo = occupied.ptr<float>(0,i);
						cv::Point3f vt;
						vo[3] = 0xFFFFFFFF; // RGBA
						if(localGrid.groundCells.channels() != 2 && localGrid.groundCells.channels() != 5)
						{
							vt = util3d::transformPoint(cv::Point3f(vi[0], vi[1], vi[2]), iter->second);
							if(localGrid.groundCells.channels() == 4)
							{
								vo[3] = vi[3];
							}
						}
						else
						{
							vt = util3d::transformPoint(cv::Point3f(vi[0], vi[1], 0), iter->second);
						}
						vo[0] = vt.x;
						vo[1] = vt.y;
						vo[2] = vt.z;
						if(minX > vo[0])
							minX = vo[0];
						else if(maxX < vo[0])
							maxX = vo[0];

						if(minY > vo[1])
							minY = vo[1];
						else if(maxY < vo[1])
							maxY = vo[1];
					}
				}

				//obstacles
				if(localGrid.obstacleCells.cols)
				{
					if(localGrid.obstacleCells.rows > 1 && localGrid.obstacleCells.cols == 1)
					{
						UFATAL("Occupancy local maps should be 1 row and X cols! (rows=%d cols=%d)", localGrid.obstacleCells.rows, localGrid.obstacleCells.cols);
					}
					for(int i=0; i<localGrid.obstacleCells.cols; ++i)
					{
						const float * vi = localGrid.obstacleCells.ptr<float>(0,i);
						float * vo = occupied.ptr<float>(0,i+localGrid.groundCells.cols);
						cv::Point3f vt;
						vo[3] = 0xFFFFFFFF; // RGBA
						if(localGrid.obstacleCells.channels() != 2 && localGrid.obstacleCells.channels() != 5)
						{
							vt = util3d::transformPoint(cv::Point3f(vi[0], vi[1], vi[2]), iter->second);
							if(localGrid.obstacleCells.channels() == 4)
							{
								vo[3] = vi[3];
							}
						}
						else
						{
							vt = util3d::transformPoint(cv::Point3f(vi[0], vi[1], 0), iter->second);
						}
						vo[0] = vt.x;
						vo[1] = vt.y;
						vo[2] = vt.z;
						if(minX > vo[0])
							minX = vo[0];
						else if(maxX < vo[0])
							maxX = vo[0];

						if(minY > vo[1])
							minY = vo[1];
						else if(maxY < vo[1])
							maxY = vo[1];
					}
				}
				uInsert(occupiedLocalMaps, std::make_pair(iter->first, occupied));
			}
		}
	}

	if(minX != maxX && minY != maxY)
	{
		//Get map size
		float xMin = minX-margin;
		xMin -= cellSize_/2.0f;
		float yMin = minY-margin;
		yMin -= cellSize_/2.0f;
		float xMax = maxX+margin;
		float yMax = maxY+margin;

		if(fabs((yMax - yMin) / cellSize_) > 99999 ||
		   fabs((xMax - xMin) / cellSize_) > 99999)
		{
			UERROR("Large map size!! map min=(%f, %f) max=(%f,%f). "
					"There's maybe an error with the poses provided! The map will not be created!",
					xMin, yMin, xMax, yMax);
		}
		else
		{
			UDEBUG("map min=(%f, %f) odlMin(%f,%f) max=(%f,%f)", xMin, yMin, minValues_[0], minValues_[1], xMax, yMax);
			cv::Size newMapSize((xMax - xMin) / cellSize_+0.5f, (yMax - yMin) / cellSize_+0.5f);
			if(!gridMap_.hasBasicLayers())
			{
				UDEBUG("Map empty!");
				grid_map::Length length = grid_map::Length(xMax - xMin, yMax - yMin);
				grid_map::Position position = grid_map::Position((xMax+xMin)/2.0f, (yMax+yMin)/2.0f);

				UDEBUG("length: %f, %f position: %f, %f", length[0], length[1], position[0], position[1]);
				gridMap_.setGeometry(length, cellSize_, position);
				UDEBUG("size: %d, %d", gridMap_.getSize()[0], gridMap_.getSize()[1]);
				// Add elevation layer
				gridMap_.add("elevation");
				gridMap_.add("node_ids");
				gridMap_.add("colors");
				gridMap_.setBasicLayers({"elevation"});
			}
			else
			{
				if(xMin == minValues_[0] && yMin == minValues_[1] &&
						newMapSize.width == gridMap_.getSize()[0] &&
						newMapSize.height == gridMap_.getSize()[1])
				{
					// same map size and origin, don't do anything
					UDEBUG("Map same size!");
				}
				else
				{
					UASSERT_MSG(xMin <= minValues_[0]+cellSize_/2, uFormat("xMin=%f, xMin_=%f, cellSize_=%f", xMin, minValues_[0], cellSize_).c_str());
					UASSERT_MSG(yMin <= minValues_[1]+cellSize_/2, uFormat("yMin=%f, yMin_=%f, cellSize_=%f", yMin, minValues_[1], cellSize_).c_str());
					UASSERT_MSG(xMax >= minValues_[0]+float(gridMap_.getSize()[0])*cellSize_ - cellSize_/2, uFormat("xMin=%f, xMin_=%f, cols=%d cellSize_=%f", xMin, minValues_[0], gridMap_.getSize()[0], cellSize_).c_str());
					UASSERT_MSG(yMax >= minValues_[1]+float(gridMap_.getSize()[1])*cellSize_ - cellSize_/2, uFormat("yMin=%f, yMin_=%f, cols=%d cellSize_=%f", yMin, minValues_[1], gridMap_.getSize()[1], cellSize_).c_str());

					UDEBUG("Copy map");
					// copy the old map in the new map
					// make sure the translation is cellSize
					int deltaX = 0;
					if(xMin < minValues_[0])
					{
						deltaX = (minValues_[0] - xMin) / cellSize_ + 1.0f;
						xMin = minValues_[0]-float(deltaX)*cellSize_;
					}
					int deltaY = 0;
					if(yMin < minValues_[1])
					{
						deltaY = (minValues_[1] - yMin) / cellSize_ + 1.0f;
						yMin = minValues_[1]-float(deltaY)*cellSize_;
					}
					UDEBUG("deltaX=%d, deltaY=%d", deltaX, deltaY);
					newMapSize.width = (xMax - xMin) / cellSize_+0.5f;
					newMapSize.height = (yMax - yMin) / cellSize_+0.5f;
					UDEBUG("%d/%d -> %d/%d", gridMap_.getSize()[0], gridMap_.getSize()[1], newMapSize.width, newMapSize.height);
					UASSERT(newMapSize.width >= gridMap_.getSize()[0] && newMapSize.height >= gridMap_.getSize()[1]);
					UASSERT(newMapSize.width >= gridMap_.getSize()[0]+deltaX && newMapSize.height >= gridMap_.getSize()[1]+deltaY);
					UASSERT(deltaX>=0 && deltaY>=0);

					grid_map::Length length = grid_map::Length(xMax - xMin, yMax - yMin);
					grid_map::Position position = grid_map::Position((xMax+xMin)/2.0f, (yMax+yMin)/2.0f);

					grid_map::GridMap tmpExtendedMap;
					tmpExtendedMap.setGeometry(length, cellSize_, position);

					UDEBUG("%d/%d -> %d/%d", gridMap_.getSize()[0], gridMap_.getSize()[1], tmpExtendedMap.getSize()[0], tmpExtendedMap.getSize()[1]);

					UDEBUG("extendToInclude (%f,%f,%f,%f) -> (%f,%f,%f,%f)",
							gridMap_.getLength()[0], gridMap_.getLength()[1],
							gridMap_.getPosition()[0], gridMap_.getPosition()[1],
							tmpExtendedMap.getLength()[0], tmpExtendedMap.getLength()[1],
							tmpExtendedMap.getPosition()[0], tmpExtendedMap.getPosition()[1]);
					if(!gridMap_.extendToInclude(tmpExtendedMap))
					{
						UERROR("Failed to update size of the grid map");
					}
					UDEBUG("Updated side: %d %d", gridMap_.getSize()[0], gridMap_.getSize()[1]);
				}
			}
			UDEBUG("map %d %d", gridMap_.getSize()[0], gridMap_.getSize()[1]);
			if(newPoses.size())
			{
				UDEBUG("first pose= %d last pose=%d", newPoses.begin()->first, newPoses.rbegin()->first);
			}
			grid_map::Matrix& gridMapData = gridMap_["elevation"];
			grid_map::Matrix& gridMapNodeIds = gridMap_["node_ids"];
			grid_map::Matrix& gridMapColors = gridMap_["colors"];
			for(std::list<std::pair<int, Transform> >::const_iterator kter = newPoses.begin(); kter!=newPoses.end(); ++kter)
			{
				std::map<int, cv::Mat>::iterator iter = occupiedLocalMaps.find(kter->first);
				if(iter!=occupiedLocalMaps.end())
				{
					addAssembledNode(kter->first, kter->second);

					for(int i=0; i<iter->second.cols; ++i)
					{
						float * ptf = iter->second.ptr<float>(0,i);
						grid_map::Position position(ptf[0], ptf[1]);
						grid_map::Index index;
						if(gridMap_.getIndex(position, index))
						{
							// If no elevation has been set, use current elevation.
							if (!gridMap_.isValid(index))
							{
								gridMapData(index(0), index(1)) = ptf[2];
								gridMapNodeIds(index(0), index(1)) = kter->first;
								gridMapColors(index(0), index(1)) = ptf[3];
							}
							else
							{
								if ((gridMapData(index(0), index(1)) < ptf[2] && (gridMapNodeIds(index(0), index(1)) <= kter->first || kter->first == -1)) ||
									gridMapNodeIds(index(0), index(1)) < kter->first)
								{
									gridMapData(index(0), index(1)) = ptf[2];
									gridMapNodeIds(index(0), index(1)) = kter->first;
									gridMapColors(index(0), index(1)) = ptf[3];
								}
							}
						}
						else
						{
							UERROR("Outside map!? (%d) (%f,%f) -> (%d,%d)", i, ptf[0], ptf[1], index[0], index[1]);
						}
					}
				}
			}

			minValues_[0] = xMin;
			minValues_[1] = yMin;
		}
	}
	UDEBUG("Occupancy Grid update time = %f s", timer.ticks());
}

}
