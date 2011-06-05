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
#include <utilite/UDirectory.h>
#include <utilite/ULogger.h>

namespace rtabmap
{

Parameters * Parameters::instance_ = 0;
UDestroyer<Parameters> Parameters::destroyer_;
ParametersMap Parameters::parameters_;

Parameters::Parameters()
{
}

Parameters::~Parameters()
{
}

const ParametersMap & Parameters::getDefaultParameters()
{
	return Parameters::getInstance()->getParameters();
}

Parameters * Parameters::getInstance()
{
	if(!instance_)
	{
		instance_ = new Parameters();
		destroyer_.setDoomed(instance_);
	}
	return instance_;
}

const ParametersMap & Parameters::getParameters() const
{
	return parameters_;
}

void Parameters::addParameter(const std::string & key, const std::string & value)
{
	parameters_.insert(ParametersPair(key, value));
}

std::string Parameters::getDefaultWorkingDirectory()
{
	std::string path = UDirectory::homeDir();
	if(!path.empty())
	{
		UDirectory::makeDir(path += "/Documents");
		UDirectory::makeDir(path += "/RTAB-Map");
		path += "/"; // add trailing separator
	}
	else
	{
		UFATAL("Can't get the HOME variable environment!");
	}
	return path;
}

}
