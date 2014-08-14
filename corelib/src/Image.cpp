/*
Copyright (c) 2010-2014, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
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


#include "rtabmap/core/Image.h"

#include <rtabmap/utilite/ULogger.h>

namespace rtabmap
{

/**
 * An id is automatically generated if id=0.
 */
Image::Image(const cv::Mat & image,
	  int id,
	  const cv::Mat & descriptors,
	  Feature2D::Type featureType,
	  const std::vector<cv::KeyPoint> & keypoints) :
	_image(image),
	_id(id),
	_descriptors(descriptors),
	_featureType(featureType),
	_keypoints(keypoints),
	_fx(0.0f),
	_fy(0.0f),
	_cx(0.0f),
	_cy(0.0f),
	_localTransform(Transform::getIdentity())
{
}

	// Metric constructor
Image::Image(const cv::Mat & image,
		  const cv::Mat & depth,
		  float fx,
		  float fy,
		  float cx,
		  float cy,
		  const Transform & pose,
		  const Transform & localTransform,
		  int id) :
	_image(image),
	_id(id),
	_featureType(Feature2D::kFeatureUndef),
	_depth(depth),
	_fx(fx),
	_fy(fy),
	_cx(cx),
	_cy(cy),
	_pose(pose),
	_localTransform(localTransform)
{
}

	// Metric constructor + 2d depth
Image::Image(const cv::Mat & image,
		  const cv::Mat & depth,
		  const cv::Mat & depth2d,
		  float fx,
		  float fy,
		  float cx,
		  float cy,
		  const Transform & pose,
		  const Transform & localTransform,
		  int id) :
	_image(image),
	_id(id),
	_featureType(Feature2D::kFeatureUndef),
	_depth(depth),
	_depth2d(depth2d),
	_fx(fx),
	_fy(fy),
	_cx(cx),
	_cy(cy),
	_pose(pose),
	_localTransform(localTransform)
{
}

void Image::setKeypoints(
		const std::vector<cv::KeyPoint> & keypoints,
		const std::vector<cv::Point3f> * keypoints3)
{
	_keypoints = keypoints;
	if(keypoints3 && keypoints3->size())
	{
		UASSERT(_keypoints.size() == keypoints3->size());
		_keypoints3 = *keypoints3;
	}
}

} // namespace rtabmap

