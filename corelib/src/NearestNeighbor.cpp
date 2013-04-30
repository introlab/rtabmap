/*
 * Copyright (C) 2010-2011, Mathieu Labbe and IntRoLab - Universite de Sherbrooke
 *
 * This file is part of RTAB-Map.
 *
 * RTAB-Map is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * RTAB-Map is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with RTAB-Map.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "NearestNeighbor.h"
#include "rtabmap/utilite/ULogger.h"
#include <opencv2/core/core.hpp>

namespace rtabmap
{

/////////////////////////
// FlannNN
/////////////////////////
FlannNN::FlannNN(Strategy strategy, const ParametersMap & parameters) :
		_treeFlannIndex(0),
		_strategy(strategy)
{
	ULOGGER_DEBUG("");
	this->parseParameters(parameters);
}

FlannNN::~FlannNN() {
	if(_treeFlannIndex)
	{
		delete _treeFlannIndex;
	}
}

void FlannNN::setData(const cv::Mat & data)
{
	if(_treeFlannIndex)
	{
		delete _treeFlannIndex;
		_treeFlannIndex = 0;
	}

	_treeFlannIndex = createIndex(data, _strategy);  // using 4 randomized trees
}

void FlannNN::search(const cv::Mat & queries, cv::Mat & indices, cv::Mat & dists, int knn, int emax)
{
	ULOGGER_DEBUG("");
	if(_treeFlannIndex)
	{
		// Note, the search params is ignored because we use an autotuned created index (see update())
		_treeFlannIndex->knnSearch(queries, indices, dists, knn, cv::flann::SearchParams(emax) ); // maximum number of leafs checked
	}
	else
	{
		ULOGGER_ERROR("The search index is not created, setData() must be called first");
	}
}

void FlannNN::search(const cv::Mat & data, const cv::Mat & queries, cv::Mat & indices, cv::Mat & dists, int knn, int emax) const
{
	ULOGGER_DEBUG("");
	cv::flann::Index * index = createIndex(data, _strategy);
	// Note, the search params is ignored because we use an autotuned created index (see update())
	index->knnSearch(queries, indices, dists, knn, cv::flann::SearchParams(emax) ); // maximum number of leafs checked
	delete index;
}

void FlannNN::parseParameters(const ParametersMap & parameters)
{

}

enum Strategy{kLinear, kKDTree, kMeans, kComposite, kAutoTuned, kUndefined};
cv::flann::Index * FlannNN::createIndex(const cv::Mat & data, Strategy s) const
{
	cv::flann::Index * index = 0;
	switch(s)
	{
	case kLinear:
		index = new cv::flann::Index(data, cv::flann::LinearIndexParams());
		break;
	case kKDTree:
		index = new cv::flann::Index(data, cv::flann::KDTreeIndexParams());
		break;
	case kMeans:
		index = new cv::flann::Index(data, cv::flann::KMeansIndexParams());
		break;
	case kComposite:
		index = new cv::flann::Index(data, cv::flann::CompositeIndexParams());
		break;
	case kAutoTuned:
	default:
		index = new cv::flann::Index(data, cv::flann::AutotunedIndexParams());
		break;
	}
	return index;
}


}
