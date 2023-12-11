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

#ifndef SRC_MAP_H_
#define SRC_MAP_H_

#include "rtabmap/core/rtabmap_core_export.h" // DLL export/import defines

#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/Transform.h>
#include <map>

namespace rtabmap {

class RTABMAP_CORE_EXPORT Map
{
public:
	virtual ~Map();

	virtual void addToCache(int nodeId,
			const cv::Mat & ground,
			const cv::Mat & obstacles,
			const cv::Mat & empty,
			const cv::Point3f & viewPoint = cv::Point3f(0,0,0));

	virtual bool update(const std::map<int, Transform> & poses) = 0; // return true if map has changed

	virtual void clear(bool keepCache = false);

	float getCellSize() const {return cellSize_;}
	bool isFullUpdate() const {return fullUpdate_;}
	float getUpdateError() const {return updateError_;}
	const std::map<int, Transform> & addedNodes() const {return addedNodes_;}
	int cacheSize() const {return (int)cache_.size();}
	const std::map<int, std::pair<std::pair<cv::Mat, cv::Mat>, cv::Mat> > & getCache() const {return cache_;}

	void getGridMin(double & x, double & y) const {x=minValues_[0];y=minValues_[1];}
	void getGridMax(double & x, double & y) const {x=maxValues_[0];y=maxValues_[1];}
	void getGridMin(double & x, double & y, double & z) const {x=minValues_[0];y=minValues_[1];z=minValues_[2];}
	void getGridMax(double & x, double & y, double & z) const {x=maxValues_[0];y=maxValues_[1];z=maxValues_[2];}

	virtual unsigned long getMemoryUsed() const;

protected:
	Map(const ParametersMap & parameters = ParametersMap());
	void checkIfMapChanged(
			const std::map<int, Transform> & poses,
			bool & graphOptimized,
			bool & graphChanged,
			std::map<int, Transform> & transforms,
			std::map<int, Transform> & updatedAddedNodes);

protected:
	std::map<int, std::pair<std::pair<cv::Mat, cv::Mat>, cv::Mat> > cache_; // [id: < <ground, obstacles>, empty>]
	std::map<int, cv::Point3f> cacheViewPoints_;
	std::map<int, Transform> addedNodes_;
	float cellSize_;
	bool fullUpdate_;
	float updateError_;

	double minValues_[3];
	double maxValues_[3];
};

} /* namespace rtabmap */

#endif /* SRC_MAP_H_ */
