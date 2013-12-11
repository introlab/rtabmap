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
#include <rtabmap/utilite/UEvent.h>
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
		kCodeImageDepth,
		kCodeNoMoreImages
	};

public:
	CameraEvent(const cv::Mat & image, int seq=0) :
		UEvent(kCodeImage),
		_image(image, seq)
	{
	}
	CameraEvent(const cv::Mat & descriptors, const std::vector<cv::KeyPoint> & keypoints, const cv::Mat & image = cv::Mat(), int seq=0) :
		UEvent(kCodeFeatures),
		_image(image, seq, descriptors, keypoints)
	{
	}
	CameraEvent() :
		UEvent(kCodeNoMoreImages)
	{
	}
	CameraEvent(const cv::Mat & image, const cv::Mat & depth, float depthConstant, const Transform & localTransform, int seq=0) :
		UEvent(kCodeImageDepth),
		_image(image, depth, depthConstant, Transform(), localTransform, seq)
	{
	}
	CameraEvent(const cv::Mat & image, const cv::Mat & depth, const cv::Mat & depth2d, float depthConstant, const Transform & localTransform, int seq=0) :
		UEvent(kCodeImageDepth),
		_image(image, depth, depth2d, depthConstant, Transform(), localTransform, seq)
	{
	}

	// Image or descriptors
	const Image & image() const {return _image;}

	virtual ~CameraEvent() {}
	virtual std::string getClassName() const {return std::string("CameraEvent");}

private:
	Image _image;
};

} // namespace rtabmap
