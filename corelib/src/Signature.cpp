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
	_weight(-1),
	_saved(false),
	_modified(true),
	_linksModified(true),
	_enabled(false),
	_fx(0.0f),
	_fy(0.0f),
	_cx(0.0f),
	_cy(0.0f),
	_laserScanMaxPts(0)
{
}

Signature::Signature(
		int id,
		int mapId,
		int weight,
		double stamp,
		const std::string & label,
		const std::multimap<int, cv::KeyPoint> & words,
		const std::multimap<int, pcl::PointXYZ> & words3, // in base_link frame (localTransform applied)
		const Transform & pose,
		const std::vector<unsigned char> & userData,
		const cv::Mat & laserScanCompressed, // in base_link frame
		const cv::Mat & imageCompressed, // in camera_link frame
		const cv::Mat & depthCompressed, // in camera_link frame
		float fx,
		float fy,
		float cx,
		float cy,
		const Transform & localTransform,
		int laserScanMaxPts) :
	_id(id),
	_mapId(mapId),
	_stamp(stamp),
	_weight(weight),
	_label(label),
	_userData(userData),
	_saved(false),
	_modified(true),
	_linksModified(true),
	_words(words),
	_enabled(false),
	_imageCompressed(imageCompressed),
	_depthCompressed(depthCompressed),
	_laserScanCompressed(laserScanCompressed),
	_fx(fx),
	_fy(fy),
	_cx(cx),
	_cy(cy),
	_pose(pose),
	_localTransform(localTransform),
	_words3(words3),
	_laserScanMaxPts(laserScanMaxPts)
{
}

Signature::~Signature()
{
	//UDEBUG("id=%d", _id);
}

void Signature::setUserData(const std::vector<unsigned char> & data)
{
	if(!_userData.empty() && !data.empty())
	{
		UWARN("Node %d: Current user data (%d bytes) overwritten by new data (%d bytes)",
				_id, (int)_userData.size(), (int)data.size());
	}

	_modified = true;
	_userData = data;
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
	UASSERT(link.from() == this->id());
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

void Signature::setDepthCompressed(const cv::Mat & bytes, float fx, float fy, float cx, float cy)
{
	UASSERT_MSG(bytes.empty() || (!bytes.empty() && fx > 0.0f && fy > 0.0f && cx >= 0.0f && cy >= 0.0f), uFormat("fx=%f fy=%f cx=%f cy=%f",fx,fy,cx,cy).c_str());
	_depthCompressed = bytes;
	_fx=fx;
	_fy=fy;
	_cx=cx;
	_cy=cy;
}

float Signature::getDepthFx() const {return getFx();}
float Signature::getDepthFy() const {return getFy();}
float Signature::getDepthCx() const {return getCx();}
float Signature::getDepthCy() const {return getCy();}

void Signature::getPoseVariance(float & rotVariance, float & transVariance) const
{
	rotVariance = 1.0f;
	transVariance = 1.0f;
	if(_links.size())
	{
		for(std::map<int, Link>::const_iterator iter = _links.begin(); iter!=_links.end(); ++iter)
		{
			if(iter->second.kNeighbor)
			{
				//Assume the first neighbor to be the backward neighbor link
				if(iter->second.to() < iter->second.from())
				{
					rotVariance = iter->second.rotVariance();
					transVariance = iter->second.transVariance();
					break;
				}
			}
		}
	}
}

SensorData Signature::toSensorData()
{
	this->uncompressData();
	float rotVariance = 1.0f;
	float transVariance = 1.0f;
	this->getPoseVariance(rotVariance, transVariance);

	return SensorData(_laserScanRaw,
			_laserScanMaxPts,
			_imageRaw,
			_depthRaw,
			_fx,
			_fy,
			_cx,
			_cy,
			_localTransform,
			_pose,
			rotVariance,
			transVariance,
			_id,
			_stamp,
			_userData);
}

void Signature::uncompressData()
{
	uncompressData(&_imageRaw, &_depthRaw, &_laserScanRaw);
}

void Signature::uncompressData(cv::Mat * imageRaw, cv::Mat * depthRaw, cv::Mat * laserScanRaw)
{
	uncompressDataConst(imageRaw, depthRaw, laserScanRaw);
	if(imageRaw && !imageRaw->empty() && _imageRaw.empty())
	{
		_imageRaw = *imageRaw;
	}
	if(depthRaw && !depthRaw->empty() && _depthRaw.empty())
	{
		_depthRaw = *depthRaw;
	}
	if(laserScanRaw && !laserScanRaw->empty() && _laserScanRaw.empty())
	{
		_laserScanRaw = *laserScanRaw;
	}
}

void Signature::uncompressDataConst(cv::Mat * imageRaw, cv::Mat * depthRaw, cv::Mat * laserScanRaw) const
{
	if(imageRaw)
	{
		*imageRaw = _imageRaw;
	}
	if(depthRaw)
	{
		*depthRaw = _depthRaw;
	}
	if(laserScanRaw)
	{
		*laserScanRaw = _laserScanRaw;
	}
	if( (imageRaw && imageRaw->empty()) ||
		(depthRaw && depthRaw->empty()) ||
		(laserScanRaw && laserScanRaw->empty()))
	{
		rtabmap::CompressionThread ctImage(_imageCompressed, true);
		rtabmap::CompressionThread ctDepth(_depthCompressed, true);
		rtabmap::CompressionThread ctLaserScan(_laserScanCompressed, false);
		if(imageRaw && imageRaw->empty())
		{
			ctImage.start();
		}
		if(depthRaw && depthRaw->empty())
		{
			ctDepth.start();
		}
		if(laserScanRaw && laserScanRaw->empty())
		{
			ctLaserScan.start();
		}
		ctImage.join();
		ctDepth.join();
		ctLaserScan.join();
		if(imageRaw && imageRaw->empty())
		{
			*imageRaw = ctImage.getUncompressedData();
		}
		if(depthRaw && depthRaw->empty())
		{
			*depthRaw = ctDepth.getUncompressedData();
		}
		if(laserScanRaw && laserScanRaw->empty())
		{
			*laserScanRaw = ctLaserScan.getUncompressedData();
		}
	}
}

} //namespace rtabmap
