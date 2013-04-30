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

#ifndef PARAMEVENT_H_
#define PARAMEVENT_H_

#include "rtabmap/core/Parameters.h"
#include <rtabmap/utilite/UEvent.h>

namespace rtabmap
{

/**
 * The parameters event. This event is used to send
 * parameters across the threads.
 */
class ParamEvent : public UEvent
{
public:
	ParamEvent(const ParametersMap & parameters) : UEvent(0), parameters_(parameters) {}
	ParamEvent(const std::string & parameterKey, const std::string & parameterValue) : UEvent(0)
	{
		parameters_.insert(std::pair<std::string, std::string>(parameterKey, parameterValue));
	}
	~ParamEvent() {}
	virtual std::string getClassName() const {return "ParamEvent";}

	const ParametersMap & getParameters() const {return parameters_;}

private:
	ParametersMap parameters_; /**< The parameters map (key,value). */
};

}

#endif /* PARAMEVENT_H_ */

