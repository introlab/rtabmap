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

#ifndef ACTUATOR_H_
#define ACTUATOR_H_

#include "rtabmap/core/RtabmapExp.h" // DLL export/import defines

#include <opencv2/core/core.hpp>
#include <utilite/UEvent.h>
#include <list>

namespace rtabmap {

class Actuator
{
public:
	enum Type{kTypeTwist=0, kTypeNotSpecified};
public:
	Actuator(const cv::Mat & data, Type type, int num = 0) :
		_data(data),
		_type(type),
		_num(num)
	{}
	const cv::Mat & data() const {return _data;}
	int type() const {return _type;}
	int num() const {return _num;}
	virtual ~Actuator() {};
private:
	cv::Mat _data;
	int _type;
	int _num;
};

}

#endif /* ACTUATOR_H_ */
