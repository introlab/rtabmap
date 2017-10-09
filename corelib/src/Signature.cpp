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
	_enabled(false),
	_invalidWordsCount(0)
{
}

Signature::Signature(
		int id,
		int mapId,
		int weight,
		double stamp,
		const std::string & label,
		const Transform & pose,
		const Transform & groundTruthPose,
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
	_invalidWordsCount(0),
	_pose(pose),
	_groundTruthPose(groundTruthPose),
	_sensorData(sensorData)
{
	if(_sensorData.id() == 0)
	{
		_sensorData.setId(id);
	}
	UASSERT(_sensorData.id() == _id);
}

Signature::Signature(const SensorData & data) :
	_id(data.id()),
	_mapId(-1),
	_stamp(data.stamp()),
	_weight(0),
	_label(""),
	_saved(false),
	_modified(true),
	_linksModified(true),
	_enabled(false),
	_invalidWordsCount(0),
	_pose(Transform::getIdentity()),
	_groundTruthPose(data.groundTruth()),
	_sensorData(data)
{

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
	UDEBUG("Add link %d to %d (type=%d var=%f,%f)", link.to(), this->id(), (int)link.type(), link.transVariance(), link.rotVariance());
	UASSERT_MSG(link.from() == this->id(), uFormat("%d->%d for signature %d (type=%d)", link.from(), link.to(), this->id(), link.type()).c_str());
	UASSERT_MSG((link.to() != this->id()) || link.type()==Link::kPosePrior, uFormat("%d->%d for signature %d (type=%d)", link.from(), link.to(), this->id(), link.type()).c_str());
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

	if(!s.isBadSignature() && !this->isBadSignature())
	{
		std::list<std::pair<int, std::pair<cv::KeyPoint, cv::KeyPoint> > > pairs;
		int totalWords = ((int)_words.size()-_invalidWordsCount)>((int)words.size()-s.getInvalidWordsCount())?((int)_words.size()-_invalidWordsCount):((int)words.size()-s.getInvalidWordsCount());
		UASSERT(totalWords > 0);
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
		std::list<cv::Point3f> pts = uValues(_words3, oldWordId);
		std::list<cv::Mat> descriptors = uValues(_wordsDescriptors, oldWordId);
		if(oldWordId<=0)
		{
			_invalidWordsCount-=(int)_words.erase(oldWordId);
			UASSERT(_invalidWordsCount>=0);
		}
		else
		{
			_words.erase(oldWordId);
		}
		_words3.erase(oldWordId);
		_wordsDescriptors.erase(oldWordId);
		_wordsChanged.insert(std::make_pair(oldWordId, activeWordId));
		for(std::list<cv::KeyPoint>::const_iterator iter=kps.begin(); iter!=kps.end(); ++iter)
		{
			_words.insert(std::pair<int, cv::KeyPoint>(activeWordId, (*iter)));
		}
		for(std::list<cv::Point3f>::const_iterator iter=pts.begin(); iter!=pts.end(); ++iter)
		{
			_words3.insert(std::pair<int, cv::Point3f>(activeWordId, (*iter)));
		}
		for(std::list<cv::Mat>::const_iterator iter=descriptors.begin(); iter!=descriptors.end(); ++iter)
		{
			_wordsDescriptors.insert(std::pair<int, cv::Mat>(activeWordId, (*iter)));
		}
	}
}

void Signature::setWords(const std::multimap<int, cv::KeyPoint> & words)
{
	_enabled = false;
	_words = words;
	_invalidWordsCount = 0;
	for(std::multimap<int, cv::KeyPoint>::iterator iter=_words.begin(); iter!=_words.end(); ++iter)
	{
		if(iter->first>0)
		{
			break;
		}
		++_invalidWordsCount;
	}
}

bool Signature::isBadSignature() const
{
	return _words.size()-_invalidWordsCount <= 0;
}

void Signature::removeAllWords()
{
	_words.clear();
	_words3.clear();
	_wordsDescriptors.clear();
	_invalidWordsCount = 0;
}

void Signature::removeWord(int wordId)
{
	if(wordId<=0)
	{
		_invalidWordsCount-=(int)_words.erase(wordId);
		UASSERT(_invalidWordsCount>=0);
	}
	else
	{
		_words.erase(wordId);
	}
	_words3.erase(wordId);
	_wordsDescriptors.clear();
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

long Signature::getMemoryUsed(bool withSensorData) const // Return memory usage in Bytes
{
	long total =  _words.size() * sizeof(float) * 8 +
				  _words3.size() * sizeof(float) * 4;
	if(!_wordsDescriptors.empty())
	{
		total += _wordsDescriptors.size() * sizeof(int);
		total += _wordsDescriptors.size() * _wordsDescriptors.begin()->second.total() * _wordsDescriptors.begin()->second.elemSize();
	}
	if(withSensorData)
	{
		total+=_sensorData.getMemoryUsed();
	}
	return total;
}

} //namespace rtabmap
