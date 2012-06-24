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

#ifndef SENSOR_H_
#define SENSOR_H_

#include "rtabmap/core/RtabmapExp.h" // DLL export/import defines

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <vector>
#include <list>
#include <utilite/UEvent.h>

namespace rtabmap {

class Sensor
{
public:
	enum Type{kTypeImage=0, kTypeImageFeatures2d, kTypeAudio, kTypeAudioFreq, kTypeAudioFreqSqrdMagn, kTypeJointState, kTypeTwist, kTypeNotSpecified};
public:
	Sensor(const cv::Mat & data, Type type, int num = 0) :
		_data(data),
		_type(type),
		_num(num)
	{}
	Sensor(const cv::Mat & descriptors, const std::vector<cv::KeyPoint> & keypoints, int num = 0) :
		_data(descriptors),
		_type(kTypeImageFeatures2d),
		_num(num),
		_keypoints(keypoints)
	{}
	const cv::Mat & data() const {return _data;}
	int type() const {return _type;}
	int num() const {return _num;}
	virtual ~Sensor() {};
	void setKeypoints(const std::vector<cv::KeyPoint> & keypoints) {_keypoints = keypoints;}
	const std::vector<cv::KeyPoint> & getKeypoints() const {return _keypoints;}
private:
	cv::Mat _data;
	int _type;
	int _num; // sensor number
	std::vector<cv::KeyPoint> _keypoints; // for convenience with kTypeImageFeatures
};

}

#endif /* SENSOR_H_ */
