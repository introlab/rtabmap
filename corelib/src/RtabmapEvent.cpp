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

#include "rtabmap/core/RtabmapEvent.h"

namespace rtabmap {
std::map<std::string, float> Statistics::_defaultData;
bool Statistics::_defaultDataInitialized = false;

const std::map<std::string, float> & Statistics::defaultData()
{
	Statistics stat;
	return _defaultData;
}

Statistics::Statistics() :
	_extended(0),
	_refImageId(0),
	_loopClosureId(0),
	_refImage(0),
	_loopClosureImage(0)
{
	_defaultDataInitialized = true;
}
Statistics::Statistics(const Statistics & s) :
	_extended(0),
	_refImageId(0),
	_loopClosureId(0),
	_refImage(0),
	_loopClosureImage(0)
{
	*this = s;
}
Statistics::~Statistics()
{
	if(_refImage)
	{
		cvReleaseImage(&_refImage);
	}
	if(_loopClosureImage)
	{
		cvReleaseImage(&_loopClosureImage);
	}
}

// name format = "Grp/Name/unit"
void Statistics::addStatistic(const std::string & name, float value)
{
	_data.insert(std::pair<std::string, float>(name, value));
}

//take the ownership of the image, the image will be
//deleted in the 'Statistics' destructor
void Statistics::setRefImage(IplImage ** refImage)
{
	if(_refImage)
		cvReleaseImage(&_refImage);
	_refImage = *refImage;
}

// Copy the image
void Statistics::setRefImage(const IplImage * refImage)
{
	if(_refImage)
		cvReleaseImage(&_refImage);
	if(refImage)
	{
		_refImage = cvCloneImage(refImage);
	}
	else
	{
		_refImage = 0;
	}
}

//take the ownership of the image, the image will be
//deleted in the 'Statistics' destructor
void Statistics::setLoopClosureImage(IplImage ** loopClosureImage)
{
	if(_loopClosureImage)
		cvReleaseImage(&_loopClosureImage);
	_loopClosureImage = *loopClosureImage;
}

// Copy the image
void Statistics::setLoopClosureImage(const IplImage * loopClosureImage)
{
	if(_loopClosureImage)
		cvReleaseImage(&_loopClosureImage);
	if(loopClosureImage)
	{
		_loopClosureImage = cvCloneImage(loopClosureImage);
	}
	else
	{
		_loopClosureImage = 0;
	}
}

Statistics & Statistics::operator=(const Statistics & s)
{
	_data = s.data();
	if(_refImage)
	{
		cvReleaseImage(&_refImage);
		_refImage = 0;
	}
	if(_loopClosureImage)
	{
		cvReleaseImage(&_loopClosureImage);
		_loopClosureImage = 0;
	}
	_extended = s.extended();
	_refImageId = s.refImageId();
	_loopClosureId = s.loopClosureId();
	if(s.refImage())
	{
		_refImage = cvCloneImage(s.refImage());
	}
	if(s.loopClosureImage())
	{
		_loopClosureImage = cvCloneImage(s.loopClosureImage());
	}
	_posterior = s.posterior();
	_likelihood = s.likelihood();
	_weights = s.weights();
	_refWords = s.refWords();
	_loopWords = s.loopWords();
	_refMotionMask = s.refMotionMask();
	_loopMotionMask = s.loopMotionMask();
	_actions = s.getActions();
	return *this;
}

}
