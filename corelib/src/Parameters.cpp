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

#include "rtabmap/core/Parameters.h"
#include <rtabmap/utilite/UDirectory.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UConversion.h>
#include <math.h>
#include <stdlib.h>

namespace rtabmap
{

ParametersMap Parameters::parameters_;
ParametersMap Parameters::descriptions_;
Parameters Parameters::instance_;

Parameters::Parameters()
{
}

Parameters::~Parameters()
{
}

std::string Parameters::getDefaultWorkingDirectory()
{
	std::string path = UDirectory::homeDir();
	if(!path.empty())
	{
		UDirectory::makeDir(path += UDirectory::separator() + "Documents");
		UDirectory::makeDir(path += UDirectory::separator() + "RTAB-Map");

	}
	else
	{
		UFATAL("Can't get the HOME variable environment!");
	}

	path += UDirectory::separator(); // add trailing separator
	return path;
}

std::string Parameters::getDefaultDatabasePath()
{
	return getDefaultWorkingDirectory() + getDefaultDatabaseName();
}

std::string Parameters::getDefaultDatabaseName()
{
	return "rtabmap.db";
}

std::string Parameters::getDescription(const std::string & paramKey)
{
	std::string description;
	ParametersMap::iterator iter = descriptions_.find(paramKey);
	if(iter != descriptions_.end())
	{
		description = iter->second;
	}
	else
	{
		UERROR("Parameters \"%s\" doesn't exist!", paramKey.c_str());
	}
	return description;
}

void Parameters::parse(const ParametersMap & parameters, const std::string & key, bool & value)
{
	ParametersMap::const_iterator iter = parameters.find(key);
	if(iter != parameters.end())
	{
		value = uStr2Bool(iter->second.c_str());
	}
}
void Parameters::parse(const ParametersMap & parameters, const std::string & key, int & value)
{
	ParametersMap::const_iterator iter = parameters.find(key);
	if(iter != parameters.end())
	{
		value = atoi(iter->second.c_str());
	}
}
void Parameters::parse(const ParametersMap & parameters, const std::string & key, unsigned int & value)
{
	ParametersMap::const_iterator iter = parameters.find(key);
	if(iter != parameters.end())
	{
		value = atoi(iter->second.c_str());
	}
}
void Parameters::parse(const ParametersMap & parameters, const std::string & key, float & value)
{
	ParametersMap::const_iterator iter = parameters.find(key);
	if(iter != parameters.end())
	{
		value = atof(iter->second.c_str());
	}
}
void Parameters::parse(const ParametersMap & parameters, const std::string & key, double & value)
{
	ParametersMap::const_iterator iter = parameters.find(key);
	if(iter != parameters.end())
	{
		value = atof(iter->second.c_str());
	}
}
void Parameters::parse(const ParametersMap & parameters, const std::string & key, std::string & value)
{
	ParametersMap::const_iterator iter = parameters.find(key);
	if(iter != parameters.end())
	{
		value = iter->second;
	}
}

}
