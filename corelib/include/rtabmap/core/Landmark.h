/*
Copyright (c) 2010-2018, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
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

#ifndef CORELIB_INCLUDE_RTABMAP_CORE_LANDMARK_H_
#define CORELIB_INCLUDE_RTABMAP_CORE_LANDMARK_H_

#include <rtabmap/core/Transform.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UMath.h>
#include <rtabmap/utilite/UConversion.h>

namespace rtabmap {

class Landmark
{
public:
	Landmark() :
		id_(0)
	{}
	Landmark(const int & id, const Transform & pose, const cv::Mat & covariance) :
		id_(id),
		pose_(pose),
		covariance_(covariance)
	{
		UASSERT(id_>0);
		UASSERT(!pose_.isNull());
		UASSERT(covariance_.cols == 6 && covariance_.rows == 6 && covariance_.type() == CV_64FC1);
		UASSERT_MSG(uIsFinite(covariance_.at<double>(0,0)) && covariance_.at<double>(0,0)>0, uFormat("Linear covariance should not be null! Value=%f.", covariance_.at<double>(0,0)).c_str());
		UASSERT_MSG(uIsFinite(covariance_.at<double>(1,1)) && covariance_.at<double>(1,1)>0, uFormat("Linear covariance should not be null! Value=%f.", covariance_.at<double>(1,1)).c_str());
		UASSERT_MSG(uIsFinite(covariance_.at<double>(2,2)) && covariance_.at<double>(2,2)>0, uFormat("Linear covariance should not be null! Value=%f.", covariance_.at<double>(2,2)).c_str());
		UASSERT_MSG(uIsFinite(covariance_.at<double>(3,3)) && covariance_.at<double>(3,3)>0, uFormat("Angular covariance should not be null! Value=%f (set to 9999 if unknown).", covariance_.at<double>(3,3)).c_str());
		UASSERT_MSG(uIsFinite(covariance_.at<double>(4,4)) && covariance_.at<double>(4,4)>0, uFormat("Angular covariance should not be null! Value=%f (set to 9999 if unknown).", covariance_.at<double>(4,4)).c_str());
		UASSERT_MSG(uIsFinite(covariance_.at<double>(5,5)) && covariance_.at<double>(5,5)>0, uFormat("Angular covariance should not be null! Value=%f (set to 9999 if unknown).", covariance_.at<double>(5,5)).c_str());
	}

	virtual ~Landmark() {}

	const int & id() const {return id_;}
	const Transform & pose() const {return pose_;}
	const cv::Mat & covariance() const {return covariance_;}

private:
	int id_;
	Transform pose_;
	cv::Mat covariance_;
};

typedef std::map<int, Landmark> Landmarks;

}

#endif /* CORELIB_INCLUDE_RTABMAP_CORE_LANDMARK_H_ */
