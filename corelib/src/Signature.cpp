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

#include "Signature.h"
#include "rtabmap/core/EpipolarGeometry.h"
#include "rtabmap/core/Memory.h"
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
		const std::multimap<int, cv::KeyPoint> & words,
		const cv::Mat & image) :
	_id(id),
	_weight(0),
	_saved(false),
	_modified(true),
	_neighborsModified(true),
	_words(words),
	_enabled(false),
	_image(image)
{
}

void Signature::addNeighbors(const std::set<int> & neighbors)
{
	for(std::set<int>::const_iterator i=neighbors.begin(); i!=neighbors.end(); ++i)
	{
		this->addNeighbor(*i);
	}
}

void Signature::addNeighbor(int neighbor)
{
	UDEBUG("Add neighbor %d to %d", neighbor, this->id());
	_neighbors.insert(neighbor);
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
	if(_neighbors.find(idFrom) != _neighbors.end())
	{
		_neighbors.erase(idFrom);
		_neighbors.insert(idTo);
		_neighborsModified = true;
	}
	UDEBUG("(%d) neighbor ids changed from %d to %d", _id, idFrom, idTo);
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
		_words.erase(oldWordId);
		_wordsChanged.insert(std::make_pair(oldWordId, activeWordId));
		for(std::list<cv::KeyPoint>::const_iterator iter=kps.begin(); iter!=kps.end(); ++iter)
		{
			_words.insert(std::pair<int, cv::KeyPoint>(activeWordId, (*iter)));
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
}

void Signature::removeWord(int wordId)
{
	_words.erase(wordId);
}

} //namespace rtabmap
