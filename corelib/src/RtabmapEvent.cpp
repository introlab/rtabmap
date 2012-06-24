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
	_loopClosureId(0)
{
	_defaultDataInitialized = true;
}

Statistics::~Statistics()
{
}

// name format = "Grp/Name/unit"
void Statistics::addStatistic(const std::string & name, float value)
{
	_data.insert(std::pair<std::string, float>(name, value));
}

void Statistics::setRefRawData(const std::list<Sensor> & refRawData)
{
	_refRawData = refRawData;
}

void Statistics::setLoopClosureRawData(const std::list<Sensor> & loopClosureRawData)
{
	_loopClosureRawData = loopClosureRawData;
}

}
