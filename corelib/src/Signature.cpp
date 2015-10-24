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

#include "rtabmap/core/Signature.h"
#include "rtabmap/core/EpipolarGeometry.h"
#include "rtabmap/core/Memory.h"
#include "rtabmap/core/Compression.h"
#include <opencv2/highgui/highgui.hpp>

#include <rtabmap/utilite/UtiLite.h>

namespace rtabmap
{

Signature::Signature() :
	_id(0), // invalid id
	_mapId(-1),
	_stamp(0.0),
	_weight(0),
	_saved(false),
	_modified(true),
	_linksModified(true),
	_enabled(false)
{
}

Signature::Signature(
		int id,
		int mapId,
		int weight,
		double stamp,
		const std::string & label,
		const Transform & pose,
		const SensorData & sensorData):
	_id(id),
	_mapId(mapId),
	_stamp(stamp),
	_weight(weight),
	_label(label),
	_saved(false),
	_modified(true),
	_linksModified(true),
	_enabled(false),
	_pose(pose),
	_sensorData(sensorData)
{
	if(_sensorData.id() == 0)
	{
		_sensorData.setId(id);
	}
	UASSERT(_sensorData.id() == _id);
}

Signature::~Signature()
{
	//UDEBUG("id=%d", _id);
}

void Signature::addLinks(const std::list<Link> & links)
{
	for(std::list<Link>::const_iterator iter = links.begin(); iter!=links.end(); ++iter)
	{
		addLink(*iter);
	}
}
void Signature::addLinks(const std::map<int, Link> & links)
{
	for(std::map<int, Link>::const_iterator iter = links.begin(); iter!=links.end(); ++iter)
	{
		addLink(iter->second);
	}
}
void Signature::addLink(const Link & link)
{
	UDEBUG("Add link %d to %d (type=%d)", link.to(), this->id(), (int)link.type());
	UASSERT_MSG(link.from() == this->id(), uFormat("%d->%d for signature %d (type=%d)", link.from(), link.to(), this->id(), link.type()).c_str());
	UASSERT_MSG(link.to() != this->id(), uFormat("%d->%d for signature %d (type=%d)", link.from(), link.to(), this->id(), link.type()).c_str());
	std::pair<std::map<int, Link>::iterator, bool> pair = _links.insert(std::make_pair(link.to(), link));
	UASSERT_MSG(pair.second, uFormat("Link %d (type=%d) already added to signature %d!", link.to(), link.type(), this->id()).c_str());
	_linksModified = true;
}

bool Signature::hasLink(int idTo) const
{
	return _links.find(idTo) != _links.end();
}

void Signature::changeLinkIds(int idFrom, int idTo)
{
	std::map<int, Link>::iterator iter = _links.find(idFrom);
	if(iter != _links.end())
	{
		Link link = iter->second;
		_links.erase(iter);
		link.setTo(idTo);
		_links.insert(std::make_pair(idTo, link));
		_linksModified = true;
		UDEBUG("(%d) neighbor ids changed from %d to %d", _id, idFrom, idTo);
	}
}

void Signature::removeLinks()
{
	if(_links.size())
		_linksModified = true;
	_links.clear();
}

void Signature::removeLink(int idTo)
{
	int count = (int)_links.erase(idTo);
	if(count)
	{
		UDEBUG("Removed link %d from %d", idTo, this->id());
		_linksModified = true;
	}
}

void Signature::removeVirtualLinks()
{
	for(std::map<int, Link>::iterator iter=_links.begin(); iter!=_links.end();)
	{
		if(iter->second.type() == Link::kVirtualClosure)
		{
			_links.erase(iter++);
		}
		else
		{
			++iter;
		}
	}
}

float Signature::compareTo(const Signature & s) const
{
	float similarity = 0.0f;
	const std::multimap<int, cv::KeyPoint> & words = s.getWords();
	if(words.size() != 0 && _words.size() != 0)
	{
		std::list<std::pair<int, std::pair<cv::KeyPoint, cv::KeyPoint> > > pairs;
		unsigned int totalWords = _words.size()>words.size()?_words.size():words.size();
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

cv::Mat Signature::getPoseCovariance() const
{
	cv::Mat covariance = cv::Mat::eye(6,6,CV_64FC1);
	if(_links.size())
	{
		for(std::map<int, Link>::const_iterator iter = _links.begin(); iter!=_links.end(); ++iter)
		{
			if(iter->second.kNeighbor)
			{
				//Assume the first neighbor to be the backward neighbor link
				if(iter->second.to() < iter->second.from())
				{
					covariance = iter->second.infMatrix().inv();
					break;
				}
			}
		}
	}
	return covariance;
}

} //namespace rtabmap
