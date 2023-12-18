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

#ifndef SRC_LOCALGRID_H_
#define SRC_LOCALGRID_H_

#include "rtabmap/core/rtabmap_core_export.h" // DLL export/import defines

#include <opencv2/core.hpp>
#include <map>

namespace rtabmap {

class RTABMAP_CORE_EXPORT LocalGrid
{
public:
	LocalGrid(const cv::Mat & ground,
		 const cv::Mat & obstacles,
		 const cv::Mat & empty,
		 float cellSize,
		 const cv::Point3f & viewPoint = cv::Point3f(0,0,0));
	virtual ~LocalGrid() {}
	bool is3D() const;
public:
	cv::Mat groundCells;
	cv::Mat obstacleCells;
	cv::Mat emptyCells;
	float cellSize;
	cv::Point3f viewPoint;
};

class RTABMAP_CORE_EXPORT LocalGridCache
{
public:
	LocalGridCache() {}
	virtual ~LocalGridCache() {}

	void add(int nodeId,
			const cv::Mat & ground,
			const cv::Mat & obstacles,
			const cv::Mat & empty,
			float cellSize,
			const cv::Point3f & viewPoint = cv::Point3f(0,0,0));

	void add(int nodeId, const LocalGrid & localGrid);

	bool shareTo(int nodeId, LocalGridCache & anotherCache) const;

	unsigned long getMemoryUsed() const;
	void clear(bool temporaryOnly = false);

	size_t size() const {return localGrids_.size();}
	bool empty() const {return localGrids_.empty();}
	const std::map<int, LocalGrid> & localGrids() const {return localGrids_;}

	std::map<int, LocalGrid>::const_iterator find(int nodeId) const {return localGrids_.find(nodeId);}
	std::map<int, LocalGrid>::const_iterator begin() const {return localGrids_.begin();}
	std::map<int, LocalGrid>::const_iterator end() const {return localGrids_.end();}

private:
	std::map<int, LocalGrid> localGrids_;
};

} /* namespace rtabmap */

#endif /* SRC_LOCALGRID_H_ */
