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
#include "Memory.h"
#include <opencv2/highgui/highgui.hpp>
#include "VerifyHypotheses.h"
#include "rtabmap/core/SMState.h"

#include "utilite/UtiLite.h"

namespace rtabmap
{

bool NeighborLink::updateIds(int idFrom, int idTo)
{
	bool modified = false;
	if(_id == idFrom)
	{
		_id = idTo;
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
	if(_image)
	{
		cvReleaseImage(&_image);
	}
}

Signature::Signature(int id, const IplImage * image, bool keepImage) :
		_id(id),
		_weight(0),
		_image(0),
		_saved(false),
		_modified(true)
{
	if(image)
	{
		if(keepImage)
		{
			_image = cvCloneImage(image);
		}
	}
}

// Warning, the image returned must be released
const IplImage * Signature::getImage() const
{
	return _image;
}

void Signature::setImage(const IplImage * image)
{
	if(_image && image)
	{
		cvReleaseImage(&_image);
		_image = cvCloneImage(image);
		_modified = true;
	}
	else
	{
		UDEBUG("Parameter is null or no image is saved.");
	}
}

// Warning, the matrix returned must be released
CvMat * Signature::compressImage(const IplImage * image)
{
	if(!image)
	{
		UERROR("The parameter must not be null.");
		return 0;
	}
	// Compress image
	int params[3] = {0};

	//JPEG compression
	std::string format = "jpeg";
	params[0] = CV_IMWRITE_JPEG_QUALITY;
	params[1] = 80; // default: 80% quality

	//PNG compression
	//std::string format = "png";
	//params[0] = CV_IMWRITE_PNG_COMPRESSION;
	//params[1] = 9; // default: maximum compression

	std::string extension = '.' + format;
	return cvEncodeImage(extension.c_str(), image, params);
}

// Warning, the image returned must be released
IplImage * Signature::decompressImage(const CvMat * imageCompressed)
{
	if(!imageCompressed)
	{
		UERROR("The parameter must not be null.");
		return 0;
	}
	return cvDecodeImage(imageCompressed, CV_LOAD_IMAGE_ANYCOLOR);
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
	UDEBUG("Add neighbor %d to %d", neighbor.id(), this->id());
	/*std::string baseIdsDebug;
	const std::vector<int> & baseIds = neighbor.baseIds();
	for(unsigned int i=0; i<baseIds.size(); ++i)
	{
		baseIdsDebug.append(uNumber2str(baseIds[i]));
		if(i+1 < baseIds.size())
		{
			baseIdsDebug.append(", ");
		}
	}
	UDEBUG("Adding neighbor %d to %d with %d actions, %d baseIds = [%s]", neighbor.id(), this->id(), neighbor.actions().size(), neighbor.baseIds().size(), baseIdsDebug.c_str());
    */

	_neighbors.insert(std::pair<int, NeighborLink>(neighbor.id(), neighbor));
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
		for(std::list<NeighborLink>::iterator iter=linksToAdd.begin(); iter!=linksToAdd.end(); ++iter)
		{
			_neighbors.insert(std::pair<int, NeighborLink>(iter->id(), *iter));
		}
		_modified = true;
		_neighborsModified = true;
		UDEBUG("(%d) neighbor ids changed from %d to %d", _id, idFrom, idTo);
	}
}



//KeypointSignature
KeypointSignature::KeypointSignature(
		const std::multimap<int, cv::KeyPoint> & words,
		int id,
		const IplImage * image,
		bool keepRawData) :
	Signature(id, image, keepRawData),
	_words(words),
	_enabled(false)
{
}

KeypointSignature::KeypointSignature(int id) :
	Signature(id),
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
			std::list<std::pair<cv::KeyPoint, cv::KeyPoint> > pairs;
			std::list<int> pairsId;
			int totalWords = _words.size()>words.size()?_words.size():words.size();
			HypVerificatorEpipolarGeo::findPairsDirect(words, _words, pairs, pairsId);

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

#define BAD_SIGNATURE_THRESHOLD 0 // elements
bool KeypointSignature::isBadSignature() const
{
	if(_words.size() <= BAD_SIGNATURE_THRESHOLD)
		return true;
	return false;
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
		const std::vector<int> & sensors,
		const std::vector<unsigned char> & motionMask,
		int id,
		const IplImage * image,
		bool keepRawData) :
	Signature(id, image, keepRawData),
	_sensors(sensors),
	_motionMask(motionMask)
{
	if(_sensors.size() != _motionMask.size() && _motionMask.size() > 0)
	{
		UFATAL("Sensors and mask must have the same size (%d vs %d)", (int)_sensors.size(), (int)_motionMask.size());
	}
	UDEBUG("sensors=%d", (int)_sensors.size());
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
		const std::vector<int> & sensorsB = sm->getSensors();
		const std::vector<unsigned char> & motionMaskB = sm->getMotionMask();

		if(_sensors.size() == sensorsB.size() && _sensors.size()) //Compatible
		{
			bool appearanceOnly = false;
			if(appearanceOnly)
			{
				std::multiset<int> sensorsSetA(_sensors.begin(), _sensors.end());
				std::multiset<int> sensorsSetB(sensorsB.begin(), sensorsB.end());
				std::set<int> ids(_sensors.begin(), _sensors.end());
				std::multiset<int>::iterator iterA;
				std::multiset<int>::iterator iterB;
				float realPairsCount = 0;
				for(std::set<int>::iterator i=ids.begin(); i!=ids.end(); ++i)
				{
					iterA = sensorsSetA.find(*i);
					iterB = sensorsSetB.find(*i);
					while(iterA != sensorsSetA.end() && iterB != sensorsSetB.end() && *iterA == *iterB && *iterA == *i)
					{
						++iterA;
						++iterB;
						++realPairsCount;
					}
				}
				similarity = realPairsCount / float(_sensors.size());
			}
			else if(_motionMask.size() == _sensors.size() &&
					_motionMask.size() == motionMaskB.size())
			{
				int sum = 0;
				int maskSumA = 0;
				int maskSumB = 0;

				// compare sensors
				for(unsigned int i=0; i<_sensors.size(); ++i)
				{
					maskSumA += _motionMask[i];
					maskSumB += motionMaskB[i];
					sum += _sensors.at(i) == sensorsB.at(i) && _motionMask[i] && motionMaskB[i] ? 1 : 0;
				}

				int totalSize = maskSumA>maskSumB?maskSumA:maskSumB;
				if(totalSize)
				{
					similarity = float(sum)/float(totalSize);
				}
			}
			else
			{
				int sum = 0;
				// compare sensors
				for(unsigned int i=0; i<_sensors.size(); ++i)
				{
					sum += _sensors.at(i) == sensorsB.at(i) ? 1 : 0;
				}
				similarity = float(sum)/float(_sensors.size());
			}

			if(similarity<0 || similarity>1)
			{
				UERROR("Something wrong! similarity is not between 0 and 1 (%f)", similarity);
			}
		}
		else if(!s->isBadSignature() && !this->isBadSignature())
		{
			UWARN("Not compatible signatures : nb sensors A=%d B=%d", (int)_sensors.size(), (int)sensorsB.size());
		}
	}
	else if(s)
	{
		UWARN("Only SM signatures are compared. (type tested=%s)", s->signatureType().c_str());
	}
	return similarity;
}


bool SMSignature::isBadSignature() const
{
	if(_sensors.size() == 0)
		return true;
	return false;
}

} //namespace rtabmap
