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

#include <rtabmap/core/Statistics.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/UConversion.h>

namespace rtabmap {
std::map<std::string, float> Statistics::_defaultData;
bool Statistics::_defaultDataInitialized = false;

const std::map<std::string, float> & Statistics::defaultData()
{
	Statistics stat;
	return _defaultData;
}

std::string Statistics::serializeData(const std::map<std::string, float> & data)
{
	std::stringstream output;
	for(std::map<std::string, float>::const_iterator iter=data.begin(); iter!=data.end(); ++iter)
	{
		if(iter != data.begin())
		{
			output << ";";
		}
		// make sure there are no commas instead of dots
		output << iter->first << ":" << uReplaceChar(uNumber2Str(iter->second), ',', '.');
	}
	return output.str();
}

std::map<std::string, float> Statistics::deserializeData(const std::string & data)
{
	std::map<std::string, float> output;
	std::list<std::string> tuplets = uSplit(data, ';');
	for(std::list<std::string>::iterator iter=tuplets.begin(); iter!=tuplets.end(); ++iter)
	{
		std::list<std::string> p = uSplit(*iter, ':');
		if(p.size() == 2)
		{
			std::string key = p.front();
			std::string value = p.back();
			uInsert(output, std::make_pair(key, uStr2Float(value)));
		}
	}
	return output;
}

Statistics::Statistics() :
	_extended(0),
	_refImageId(0),
	_refImageMapId(-1),
	_loopClosureId(0),
	_loopClosureMapId(-1),
	_proximiyDetectionId(0),
	_proximiyDetectionMapId(-1),
	_stamp(0.0f),
	_currentGoalId(0)
{
	_defaultDataInitialized = true;
}

Statistics::~Statistics()
{
}

// name format = "Grp/Name/unit"
void Statistics::addStatistic(const std::string & name, float value)
{
	uInsert(_data, std::pair<std::string, float>(name, value));
}

//deprecated
void Statistics::setLastSignatureData(const Signature & data)
{
	_signaturesData.insert(std::make_pair(data.id(), data));
}

}
