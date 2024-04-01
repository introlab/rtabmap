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
	UDEBUG("Add link %d to %d (type=%d/%s var=%f,%f)", link.to(), this->id(), (int)link.type(), link.typeName().c_str(), link.transVariance(), link.rotVariance());
	UASSERT_MSG(link.from() == this->id(), uFormat("%d->%d for signature %d (type=%d)", link.from(), link.to(), this->id(), link.type()).c_str());
	UASSERT_MSG((link.to() != this->id()) || link.type()==Link::kPosePrior || link.type()==Link::kGravity, uFormat("%d->%d for signature %d (type=%d)", link.from(), link.to(), this->id(), link.type()).c_str());
	UASSERT_MSG(link.to() == this->id() || _links.find(link.to()) == _links.end(), uFormat("Link %d (type=%d) already added to signature %d!", link.to(), link.type(), this->id()).c_str());
	_links.insert(std::make_pair(link.to(), link));
	_linksModified = true;
}

bool Signature::hasLink(int idTo, Link::Type type) const
{
	if(type == Link::kUndef)
	{
		return _links.find(idTo) != _links.end();
	}
	if(idTo==0)
	{
		for(std::multimap<int, Link>::const_iterator iter=_links.begin(); iter!=_links.end(); ++iter)
		{
			if(type == iter->second.type())
			{
				return true;
			}
		}
	}
	else
	{
		for(std::multimap<int, Link>::const_iterator iter=_links.find(idTo); iter!=_links.end() && iter->first == idTo; ++iter)
		{
			if(type == iter->second.type())
			{
				return true;
			}
		}
	}
	return false;
}

void Signature::changeLinkIds(int idFrom, int idTo)
{
	std::multimap<int, Link>::iterator iter = _links.find(idFrom);
	while(iter != _links.end() && iter->first == idFrom)
	{
		Link link = iter->second;
		_links.erase(iter++);
		link.setTo(idTo);
		_links.insert(std::make_pair(idTo, link));
		_linksModified = true;
		UDEBUG("(%d) neighbor ids changed from %d to %d", _id, idFrom, idTo);
	}
}

void Signature::removeLinks(bool keepSelfReferringLinks)
{
	size_t sizeBefore = _links.size();
	if(keepSelfReferringLinks)
	{
		for(std::multimap<int, Link>::iterator iter = _links.begin(); iter != _links.end();)
		{
			if(iter->second.from() == iter->second.to())
			{
				++iter;
			}
			else
			{
				_links.erase(iter++);
			}
		}
	}
	else
	{
		_links.clear();
	}
	if(_links.size() != sizeBefore)
		_linksModified = true;
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
	for(std::multimap<int, Link>::iterator iter=_links.begin(); iter!=_links.end();)
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
	const std::multimap<int, int> & words = s.getWords();

	if(!s.isBadSignature() && !this->isBadSignature())
	{
		std::list<std::pair<int, std::pair<int, int> > > pairs;
		int totalWords = ((int)_words.size()-_invalidWordsCount)>((int)words.size()-s.getInvalidWordsCount())?((int)_words.size()-_invalidWordsCount):((int)words.size()-s.getInvalidWordsCount());
		UASSERT(totalWords > 0);
		EpipolarGeometry::findPairs(words, _words, pairs);

		similarity = float(pairs.size()) / float(totalWords);
	}
	return similarity;
}

void Signature::changeWordsRef(int oldWordId, int activeWordId)
{
	std::list<int> words = uValues(_words, oldWordId);
	if(words.size())
	{
		if(oldWordId<=0)
		{
			_invalidWordsCount-=(int)_words.erase(oldWordId);
			UASSERT(_invalidWordsCount>=0);
		}
		else
		{
			_words.erase(oldWordId);
		}

		_wordsChanged.insert(std::make_pair(oldWordId, activeWordId));
		for(std::list<int>::const_iterator iter=words.begin(); iter!=words.end(); ++iter)
		{
			_words.insert(std::pair<int, int>(activeWordId, (*iter)));
		}
	}
}

void Signature::setWords(const std::multimap<int, int> & words,
		const std::vector<cv::KeyPoint> & keypoints,
		const std::vector<cv::Point3f> & points,
		const cv::Mat & descriptors)
{
	UASSERT_MSG(descriptors.empty() || descriptors.rows == (int)words.size(), uFormat("words=%d, descriptors=%d", (int)words.size(), descriptors.rows).c_str());
	UASSERT_MSG(points.empty() || points.size() == words.size(),  uFormat("words=%d, points=%d", (int)words.size(), (int)points.size()).c_str());
	UASSERT_MSG(keypoints.empty() || keypoints.size() == words.size(),  uFormat("words=%d, descriptors=%d", (int)words.size(), (int)keypoints.size()).c_str());
	UASSERT(words.empty() || !keypoints.empty() || !points.empty() || !descriptors.empty());

	_invalidWordsCount = 0;
	for(std::multimap<int, int>::const_iterator iter=words.begin(); iter!=words.end(); ++iter)
	{
		if(iter->first<=0)
		{
			++_invalidWordsCount;
		}
		// make sure indexes are all valid!
		UASSERT_MSG(iter->second >=0 && iter->second < (int)words.size(), uFormat("iter->second=%d words.size()=%d", iter->second, (int)words.size()).c_str());
	}

	_enabled = false;
	_words = words;
	_wordsKpts = keypoints;
	_words3 = points;
	_wordsDescriptors = descriptors.clone();
}

bool Signature::isBadSignature() const
{
	return _words.size()-_invalidWordsCount <= 0;
}

void Signature::removeAllWords()
{
	_words.clear();
	_wordsKpts.clear();
	_words3.clear();
	_wordsDescriptors = cv::Mat();
	_invalidWordsCount = 0;
}

void Signature::setWordsDescriptors(const cv::Mat & descriptors)
{
	if(descriptors.empty())
	{
		if(_wordsKpts.empty() && _words3.empty())
		{
			removeAllWords();
		}
		else
		{
			_wordsDescriptors = cv::Mat();
		}
	}
	else
	{
		UASSERT(descriptors.rows == (int)_words.size());
		_wordsDescriptors = descriptors.clone();
	}
}

cv::Mat Signature::getPoseCovariance() const
{
	cv::Mat covariance = cv::Mat::eye(6,6,CV_64FC1);
	if(_links.size())
	{
		for(std::multimap<int, Link>::const_iterator iter = _links.begin(); iter!=_links.end(); ++iter)
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

unsigned long Signature::getMemoryUsed(bool withSensorData) const // Return memory usage in Bytes
{
	unsigned long total = sizeof(Signature);
	total += _words.size() * (sizeof(int)*2+sizeof(std::multimap<int, cv::KeyPoint>::iterator)) + sizeof(std::multimap<int, cv::KeyPoint>);
	total += _wordsKpts.size() * sizeof(cv::KeyPoint) + sizeof(std::vector<cv::KeyPoint>);
	total += _words3.size() * sizeof(cv::Point3f) + sizeof(std::vector<cv::Point3f>);
	total += _wordsDescriptors.empty()?0:_wordsDescriptors.total() * _wordsDescriptors.elemSize() + sizeof(cv::Mat);
	total += _wordsChanged.size() * (sizeof(int)*2+sizeof(std::map<int, int>::iterator)) + sizeof(std::map<int, int>);
	if(withSensorData)
	{
		total+=_sensorData.getMemoryUsed();
	}
	total += _pose.size() * (sizeof(Transform) + sizeof(float)*12);
	total += _groundTruthPose.size() * (sizeof(Transform) + sizeof(float)*12);
	total += _velocity.size() * sizeof(float);
	total += _links.size() * (sizeof(int) + sizeof(Transform) + 12 * sizeof(float) + sizeof(cv::Mat) + 36 * sizeof(double)+sizeof(std::multimap<int, Link>::iterator)) + sizeof(std::multimap<int, Link>);
	total += _landmarks.size() * (sizeof(int) + sizeof(Transform) + 12 * sizeof(float) + sizeof(cv::Mat) + 36 * sizeof(double)+sizeof(std::map<int, Link>::iterator)) + sizeof(std::map<int, Link>);
	return total;
}

} //namespace rtabmap
