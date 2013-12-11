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

#include "rtabmap/core/Signature.h"
#include "rtabmap/core/EpipolarGeometry.h"
#include "rtabmap/core/Memory.h"
#include "rtabmap/core/util3d.h"
#include <opencv2/highgui/highgui.hpp>

#include <rtabmap/utilite/UtiLite.h>

namespace rtabmap
{

Signature::~Signature()
{
	ULOGGER_DEBUG("id=%d", _id);
}

Signature::Signature(
		int id,
		int mapId,
		const std::multimap<int, cv::KeyPoint> & words,
		const std::multimap<int, pcl::PointXYZ> & words3, // in base_link frame (localTransform applied)
		const Transform & pose,
		const std::vector<unsigned char> & depth2D, // in base_link frame
		const std::vector<unsigned char> & image, // in camera_link frame
		const std::vector<unsigned char> & depth, // in camera_link frame
		float depthConstant,
		const Transform & localTransform) :
	_id(id),
	_mapId(mapId),
	_weight(0),
	_saved(false),
	_modified(true),
	_neighborsModified(true),
	_words(words),
	_enabled(false),
	_image(image),
	_depth(depth),
	_depth2D(depth2D),
	_depthConstant(depthConstant),
	_pose(pose),
	_localTransform(localTransform),
	_words3(words3)
{
}

void Signature::addNeighbors(const std::map<int, Transform> & neighbors)
{
	for(std::map<int, Transform>::const_iterator i=neighbors.begin(); i!=neighbors.end(); ++i)
	{
		this->addNeighbor(i->first, i->second);
	}
}

void Signature::addNeighbor(int neighbor, const Transform & transform)
{
	UDEBUG("Add neighbor %d to %d", neighbor, this->id());
	_neighbors.insert(std::pair<int, Transform>(neighbor, transform));
	_neighborsModified = true;
}

void Signature::removeNeighbor(int neighborId)
{
	int count = _neighbors.erase(neighborId);
	if(count)
	{
		_neighborsModified = true;
	}
}

void Signature::removeNeighbors()
{
	if(_neighbors.size())
		_neighborsModified = true;
	_neighbors.clear();
}

void Signature::changeNeighborIds(int idFrom, int idTo)
{
	std::map<int, Transform>::iterator iter = _neighbors.find(idFrom);
	if(iter != _neighbors.end())
	{
		Transform t = iter->second;
		_neighbors.erase(iter);
		_neighbors.insert(std::pair<int, Transform>(idTo, t));
		_neighborsModified = true;
	}
	UDEBUG("(%d) neighbor ids changed from %d to %d", _id, idFrom, idTo);
}

void Signature::addLoopClosureId(int loopClosureId, const Transform & transform)
{
	if(loopClosureId && _loopClosureIds.insert(std::pair<int, Transform>(loopClosureId, transform)).second)
	{
		_neighborsModified=true;
	}
}

void Signature::addChildLoopClosureId(int childLoopClosureId, const Transform & transform)
{
	if(childLoopClosureId && _childLoopClosureIds.insert(std::pair<int, Transform>(childLoopClosureId, transform)).second)
	{
		_neighborsModified=true;
	}
}

void Signature::changeLoopClosureId(int idFrom, int idTo)
{
	std::map<int, Transform>::iterator iter = _loopClosureIds.find(idFrom);
	if(iter != _loopClosureIds.end())
	{
		Transform t = iter->second;
		_loopClosureIds.erase(iter);
		_loopClosureIds.insert(std::pair<int, Transform>(idTo, t));
		_neighborsModified = true;
	}
	UDEBUG("(%d) loop closure ids changed from %d to %d", _id, idFrom, idTo);
}


float Signature::compareTo(const Signature * s) const
{
	float similarity = 0.0f;
	const std::multimap<int, cv::KeyPoint> & words = s->getWords();
	if(words.size() != 0 && _words.size() != 0)
	{
		std::list<std::pair<int, std::pair<cv::KeyPoint, cv::KeyPoint> > > pairs;
		int totalWords = _words.size()>words.size()?_words.size():words.size();
		EpipolarGeometry::findPairs(words, _words, pairs);

		similarity = float(pairs.size()) / float(totalWords);
	}
	return similarity;
}

void Signature::changeWordsRef(int oldWordId, int activeWordId)
{
	std::list<cv::KeyPoint> kps = uValues(_words, oldWordId);
	if(kps.size())
	{
		std::list<pcl::PointXYZ> pts = uValues(_words3, oldWordId);
		_words.erase(oldWordId);
		_words3.erase(oldWordId);
		_wordsChanged.insert(std::make_pair(oldWordId, activeWordId));
		for(std::list<cv::KeyPoint>::const_iterator iter=kps.begin(); iter!=kps.end(); ++iter)
		{
			_words.insert(std::pair<int, cv::KeyPoint>(activeWordId, (*iter)));
		}
		for(std::list<pcl::PointXYZ>::const_iterator iter=pts.begin(); iter!=pts.end(); ++iter)
		{
			_words3.insert(std::pair<int, pcl::PointXYZ>(activeWordId, (*iter)));
		}
	}
}

bool Signature::isBadSignature() const
{
	return !_words.size();
}

void Signature::removeAllWords()
{
	_words.clear();
	_words3.clear();
}

void Signature::removeWord(int wordId)
{
	_words.erase(wordId);
	_words3.erase(wordId);
}

void Signature::setDepth(const std::vector<unsigned char> & depth, float depthConstant)
{
	UASSERT_MSG(depth.empty() || (!depth.empty() && depthConstant > 0.0f), uFormat("depthConstant=%f",depthConstant).c_str());
	_depth = depth;
	_depthConstant=depthConstant;
}

} //namespace rtabmap
