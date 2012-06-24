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
#include <opencv2/highgui/highgui.hpp>
#include "rtabmap/core/VerifyHypotheses.h"

#include <utilite/UtiLite.h>

namespace rtabmap
{

bool NeighborLink::updateIds(int idFrom, int idTo)
{
	bool modified = false;
	if(_toId == idFrom)
	{
		_toId = idTo;
		modified = true;
	}
	for(unsigned int i=0; i<_baseIds.size(); ++i)
	{
		if(_baseIds[i] == idFrom)
		{
			_baseIds[i] = idTo;
			modified = true;
		}
	}
	return modified;
}

Signature::~Signature()
{
	ULOGGER_DEBUG("id=%d", _id);
}

Signature::Signature(int id) :
		_id(id),
		_weight(0),
		_saved(false),
		_modified(true)
{
}

Signature::Signature(int id, const std::list<Sensor> & rawData) :
		_id(id),
		_weight(0),
		_rawData(rawData),
		_saved(false),
		_modified(true)
{
}

void Signature::addNeighbors(const NeighborsMultiMap & neighbors)
{
	for(NeighborsMultiMap::const_iterator i=neighbors.begin(); i!=neighbors.end(); ++i)
	{
		this->addNeighbor(i->second);
	}
}

void Signature::addNeighbor(const NeighborLink & neighbor)
{
	UDEBUG("Add neighbor %d to %d", neighbor.toId(), this->id());

	if(ULogger::level() == ULogger::kDebug)
	{
		UTimer timer;
		std::string baseIdsDebug;
		const std::vector<int> & baseIds = neighbor.baseIds();
		for(unsigned int i=0; i<baseIds.size(); ++i)
		{
			baseIdsDebug.append(uFormat("%d", baseIds[i]));
			if(i+1 < baseIds.size())
			{
				baseIdsDebug.append(", ");
			}
		}
		UDEBUG("Adding neighbor %d to %d with %d actions, %d baseIds = [%s] (time print=%fs)", neighbor.toId(), this->id(), neighbor.actuators().size(), neighbor.baseIds().size(), baseIdsDebug.c_str(), timer.getElapsedTime());
	}

	_neighbors.insert(std::pair<int, NeighborLink>(neighbor.toId(), neighbor));
	if(neighbor.actuators().size())
	{
		_neighborsWithActuators.insert(neighbor.toId());
	}
	_neighborsAll.insert(neighbor.toId());
	_neighborsModified = true;
}

void Signature::changeNeighborIds(int idFrom, int idTo)
{
	std::pair<NeighborsMultiMap::iterator, NeighborsMultiMap::iterator> pair = _neighbors.equal_range(idFrom);

	if(pair.first != _neighbors.end() && pair.first != pair.second)
	{
		std::list<NeighborLink> linksToAdd;
		for(NeighborsMultiMap::iterator iter = pair.first; iter!=pair.second; ++iter)
		{
			NeighborLink link = iter->second;
			link.updateIds(idFrom, idTo);
			linksToAdd.push_back(link);
		}
		_neighbors.erase(idFrom);
		_neighborsWithActuators.erase(idFrom);
		_neighborsAll.erase(idFrom);
		for(std::list<NeighborLink>::iterator iter=linksToAdd.begin(); iter!=linksToAdd.end(); ++iter)
		{
			_neighbors.insert(std::pair<int, NeighborLink>(iter->toId(), *iter));
			if(iter->actuators().size())
			{
				_neighborsWithActuators.insert(iter->toId());
			}
			_neighborsAll.insert(iter->toId());
		}
		_neighborsModified = true;
		UDEBUG("(%d) neighbor ids changed from %d to %d", _id, idFrom, idTo);
	}
}



//KeypointSignature
KeypointSignature::KeypointSignature(int id) :
	Signature(id),
	_enabled(false)
{
}
KeypointSignature::KeypointSignature(const std::multimap<int, cv::KeyPoint> & words,
		int id) :
	Signature(id),
	_words(words),
	_enabled(false)
{
}
KeypointSignature::KeypointSignature(
		const std::multimap<int, cv::KeyPoint> & words,
		int id,
		const std::list<Sensor> & rawData) :
	Signature(id, rawData),
	_words(words),
	_enabled(false)
{
}

KeypointSignature::~KeypointSignature()
{
}

float KeypointSignature::compareTo(const Signature * s) const
{
	const KeypointSignature * ss = dynamic_cast<const KeypointSignature *>(s);
	float similarity = 0;

	if(ss) //Compatible
	{
		const std::multimap<int, cv::KeyPoint> & words = ss->getWords();

		if(words.size() != 0 && _words.size() != 0)
		{
			std::list<std::pair<int, std::pair<cv::KeyPoint, cv::KeyPoint> > > pairs;
			int totalWords = _words.size()>words.size()?_words.size():words.size();
			findPairs(words, _words, pairs);

			similarity = float(pairs.size()) / float(totalWords);
		}
	}
	return similarity;
}

void KeypointSignature::changeWordsRef(int oldWordId, int activeWordId)
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

bool KeypointSignature::isBadSignature() const
{
	return !_words.size();
}

void KeypointSignature::removeAllWords()
{
	_words.clear();
}

void KeypointSignature::removeWord(int wordId)
{
	_words.erase(wordId);
}





//SMSignature
SMSignature::SMSignature(
		const std::list<std::vector<int> > & data,
		int id) :
	Signature(id),
	_data(data)
{
	UDEBUG("data=%d", (int)_data.size());
}
SMSignature::SMSignature(
		const std::list<std::vector<int> > & data,
		int id,
		const std::list<Sensor> & rawData) :
	Signature(id, rawData),
	_data(data)
{
	UDEBUG("data=%d", (int)_data.size());
}

SMSignature::SMSignature(int id) :
	Signature(id)
{
}

SMSignature::~SMSignature()
{
}

float SMSignature::compareTo(const Signature * s) const
{
	const SMSignature * sm = dynamic_cast<const SMSignature *>(s);
	float similarity = 0;

	if(sm)
	{
		const std::list<std::vector<int> > & dataB = sm->getData();
		//const std::vector<unsigned char> & motionMaskB = sm->getMotionMask();

		//if(_data.size() == sensorsB.size() && _data.size()) //Compatible
		if(_data.size() == dataB.size()) //Compatible
		{
			std::vector<float> similarities(_data.size());
			// compare sensors
			std::list<std::vector<int> >::const_iterator iterA = _data.begin();
			std::list<std::vector<int> >::const_iterator iterB = dataB.begin();
			int j=0;
			while(iterA != _data.end() && iterB != dataB.end())
			{
				if(iterA->size() == iterB->size())
				{
					int sum = 0;
					int notNull = 0;
					for(unsigned int i=0; i<iterA->size(); ++i)
					{
						sum += iterA->at(i) && iterA->at(i) == iterB->at(i) ? 1 : 0;
						notNull += iterA->at(i) || iterB->at(i) ? 1 : 0;
					}
					if(notNull)
					{
						similarities[j] = float(sum)/float(notNull);
					}
					else
					{
						similarities[j] = 1.0f; // example, silence == 100% silence
					}
				}
				else
				{
					UERROR("Data are not the same size (%d vs %d)", (int)iterA->size(), (int)iterB->size());
				}
				++iterA;
				++iterB;
				++j;
			}

			similarity = uMean(similarities);
			if(ULogger::level() == ULogger::kDebug)
			{
				std::string str;
				for(unsigned int i=0; i<similarities.size(); ++i)
				{
					str.append(uFormat("%f", similarities[i]));
					if(i<similarities.size()-1)
					{
						str.append(", ");
					}
				}
				UDEBUG("similarities (%d vs %d) = [%s]", this->id(), s->id(), str.c_str());
			}

			if(similarity<0 || similarity>1)
			{
				UERROR("Something wrong! similarity is not between 0 and 1 (%f)", similarity);
			}
		}
		else if(!s->isBadSignature() && !this->isBadSignature())
		{
			UWARN("Not compatible nodes : nb sensors A=%d B=%d", (int)_data.size(), (int)dataB.size());
		}
	}
	else if(s)
	{
		UWARN("Only SM signatures are compared. (type tested=%s)", s->nodeType().c_str());
	}
	return similarity;
}


bool SMSignature::isBadSignature() const
{
	//return uSum(_data) == 0;
	return !_data.size();
}

} //namespace rtabmap
