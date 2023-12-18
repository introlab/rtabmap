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

#ifndef CORELIB_SRC_CLOUDMAP_H_
#define CORELIB_SRC_CLOUDMAP_H_

#include "rtabmap/core/rtabmap_core_export.h" // DLL export/import defines

#include <rtabmap/core/GlobalMap.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace rtabmap {

class RTABMAP_CORE_EXPORT CloudMap : public GlobalMap
{
public:
	CloudMap(const LocalGridCache * cache, const ParametersMap & parameters = ParametersMap());

	virtual void clear();

	const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & getMapGround() const {return assembledGround_;}
	const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & getMapObstacles() const {return assembledObstacles_;}
	const pcl::PointCloud<pcl::PointXYZ>::Ptr & getMapEmptyCells() const {return assembledEmptyCells_;}

	unsigned long getMemoryUsed() const;

protected:
	virtual void assemble(const std::list<std::pair<int, Transform> > & newPoses);

private:
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr assembledGround_;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr assembledObstacles_;
	pcl::PointCloud<pcl::PointXYZ>::Ptr assembledEmptyCells_;
};

}

#endif /* CORELIB_SRC_CLOUDMAP_H_ */
