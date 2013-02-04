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

#pragma once

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <utilite/UEvent.h>
#include "rtabmap/core/Image.h"

namespace rtabmap
{

class CameraEvent :
	public UEvent
{
public:
	enum Code {
		kCodeFeatures,
		kCodeImage,
		kCodeNoMoreImages
	};

public:
	CameraEvent(const cv::Mat & image, int cameraId = 0) :
		UEvent(kCodeImage),
		_cameraId(cameraId),
		_image(image)
	{
	}
	CameraEvent(const cv::Mat & descriptors, const std::vector<cv::KeyPoint> & keypoints, const cv::Mat & image = cv::Mat(), int cameraId = 0) :
		UEvent(kCodeFeatures),
		_cameraId(cameraId),
		_image(image, 0, descriptors, keypoints)
	{
	}
	CameraEvent(int cameraId = 0) :
		UEvent(kCodeNoMoreImages),
		_cameraId(cameraId)
	{
	}

	int cameraId() const {return _cameraId;}

	// Image or descriptors
	const Image & image() const {return _image;}

	virtual ~CameraEvent() {}
	virtual std::string getClassName() const {return std::string("CameraEvent");}

private:
	int _cameraId;
	Image _image;
};

} // namespace rtabmap
