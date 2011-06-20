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
#include "Memory.h"
#include <opencv2/highgui/highgui.hpp>
#include "VerifyHypotheses.h"

#include "utilite/UtiLite.h"

namespace rtabmap
{

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
		_loopClosureId(0),
		_image(0),
		_saved(false),
		_width(0),
		_height(0)
{
	if(image)
	{
		_width = image->width;
		_height = image->height;
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
	}
	else
	{
		UWARN("Parameter is null or no image is saved.");
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

void Signature::addNeighbors(const NeighborsMap & neighbors)
{
	for(NeighborsMap::const_iterator i=neighbors.begin(); i!=neighbors.end(); ++i)
	{
		this->addNeighbor(i->first, i->second);
		//UDEBUG("%d -> %d, a=%d", this->id(), i->first, i->second.size());
	}
}

void Signature::addNeighbor(int neighbor, const std::list<std::vector<float> > & actions)
{
	ULOGGER_DEBUG("Adding neighbor %d to %d with %d actions", neighbor, this->id(), actions.size());
	std::pair<NeighborsMap::iterator, bool> inserted = _neighbors.insert(std::pair<int, std::list<std::vector<float> > >(neighbor, actions));
	//UDEBUG("%d -> %d, a=%d", this->id(), neighbor, actions.size());
	if(!inserted.second)
	{
		ULOGGER_ERROR("neighbor %d already added to %d", neighbor, this->id());
		return;
	}
	if(neighbor == _id)
	{
		ULOGGER_ERROR("same Id ? (%d)", neighbor, this->id());
		return;
	}
}

void Signature::removeNeighbor(int neighbor)
{
	ULOGGER_DEBUG("Removing neighbor %d to %d", neighbor, this->id());
	// we delete the first found because there is not supposed
	// to have more than one occurrence of this neighbor (see addNeighbor())
	int erased = _neighbors.erase(neighbor);
	if(!erased)
	{
		ULOGGER_WARN("neighbor %d not found in %d", neighbor, this->id());
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

			// Adjust similarity with the ratio of words between the signatures
			/*float ratio = 1;
			if(_words.size() > words.size() && _words.size())
			{
				ratio = float(words.size()) / float(_words.size());
			}
			else
			{
				ratio = float(_words.size()) / float(words.size());
			}

			similarity *= ratio;*/
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

}
