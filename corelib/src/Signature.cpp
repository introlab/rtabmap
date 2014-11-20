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
#include "rtabmap/core/util3d.h"
#include <opencv2/highgui/highgui.hpp>

#include <rtabmap/utilite/UtiLite.h>

namespace rtabmap
{

Signature::Signature() :
	_id(0), // invalid id
	_mapId(-1),
	_weight(-1),
	_saved(false),
	_modified(true),
	_neighborsModified(true),
	_enabled(false),
	_fx(0.0f),
	_fy(0.0f),
	_cx(0.0f),
	_cy(0.0f)
{
}

Signature::Signature(
		int id,
		int mapId,
		const std::multimap<int, cv::KeyPoint> & words,
		const std::multimap<int, pcl::PointXYZ> & words3, // in base_link frame (localTransform applied)
		const Transform & pose,
		const cv::Mat & depth2DCompressed, // in base_link frame
		const cv::Mat & imageCompressed, // in camera_link frame
		const cv::Mat & depthCompressed, // in camera_link frame
		float fx,
		float fy,
		float cx,
		float cy,
		const Transform & localTransform) :
	_id(id),
	_mapId(mapId),
	_weight(0),
	_saved(false),
	_modified(true),
	_neighborsModified(true),
	_words(words),
	_enabled(false),
	_imageCompressed(imageCompressed),
	_depthCompressed(depthCompressed),
	_depth2DCompressed(depth2DCompressed),
	_fx(fx),
	_fy(fy),
	_cx(cx),
	_cy(cy),
	_pose(pose),
	_localTransform(localTransform),
	_words3(words3)
{
}

Signature::~Signature()
{
	//UDEBUG("id=%d", _id);
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
	int count = (int)_neighbors.erase(neighborId);
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

void Signature::setDepthCompressed(const cv::Mat & bytes, float fx, float fy, float cx, float cy)
{
	UASSERT_MSG(bytes.empty() || (!bytes.empty() && fx > 0.0f && fy > 0.0f && cx >= 0.0f && cy >= 0.0f), uFormat("fx=%f fy=%f cx=%f cy=%f",fx,fy,cx,cy).c_str());
	_depthCompressed = bytes;
	_fx=fx;
	_fy=fy;
	_cx=cx;
	_cy=cy;
}

SensorData Signature::toSensorData()
{
	this->uncompressData();
	return SensorData(_imageRaw,
			_depthRaw,
			_depth2DRaw,
			_fx,
			_fy,
			_cx,
			_cy,
			_pose,
			_localTransform,
			_id);
}

void Signature::uncompressData()
{
	uncompressData(&_imageRaw, &_depthRaw, &_depth2DRaw);
}

void Signature::uncompressData(cv::Mat * imageRaw, cv::Mat * depthRaw, cv::Mat * depth2DRaw)
{
	uncompressDataConst(imageRaw, depthRaw, depth2DRaw);
	if(imageRaw && !imageRaw->empty() && _imageRaw.empty())
	{
		_imageRaw = *imageRaw;
	}
	if(depthRaw && !depthRaw->empty() && _depthRaw.empty())
	{
		_depthRaw = *depthRaw;
	}
	if(depth2DRaw && !depth2DRaw->empty() && _depth2DRaw.empty())
	{
		_depth2DRaw = *depth2DRaw;
	}
}

void Signature::uncompressDataConst(cv::Mat * imageRaw, cv::Mat * depthRaw, cv::Mat * depth2DRaw) const
{
	if(imageRaw)
	{
		*imageRaw = _imageRaw;
	}
	if(depthRaw)
	{
		*depthRaw = _depthRaw;
	}
	if(depth2DRaw)
	{
		*depth2DRaw = _depth2DRaw;
	}
	if( (imageRaw && imageRaw->empty()) ||
		(depthRaw && depthRaw->empty()) ||
		(depth2DRaw && depth2DRaw->empty()))
	{
		util3d::CompressionThread ctImage(_imageCompressed, true);
		util3d::CompressionThread ctDepth(_depthCompressed, true);
		util3d::CompressionThread ctDepth2D(_depth2DCompressed, false);
		if(imageRaw && imageRaw->empty())
		{
			ctImage.start();
		}
		if(depthRaw && depthRaw->empty())
		{
			ctDepth.start();
		}
		if(depth2DRaw && depth2DRaw->empty())
		{
			ctDepth2D.start();
		}
		ctImage.join();
		ctDepth.join();
		ctDepth2D.join();
		if(imageRaw && imageRaw->empty())
		{
			*imageRaw = ctImage.getUncompressedData();
		}
		if(depthRaw && depthRaw->empty())
		{
			*depthRaw = ctDepth.getUncompressedData();
		}
		if(depth2DRaw && depth2DRaw->empty())
		{
			*depth2DRaw = ctDepth2D.getUncompressedData();
		}
	}
}

} //namespace rtabmap
