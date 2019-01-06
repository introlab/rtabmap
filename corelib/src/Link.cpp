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

#include "rtabmap/core/Link.h"
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UMath.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/core/Compression.h>

namespace rtabmap {

Link::Link() :
	from_(0),
	to_(0),
	type_(kUndef),
	infMatrix_(cv::Mat::eye(6,6,CV_64FC1))
{
}
Link::Link(int from,
		int to,
		Type type,
		const Transform & transform,
		const cv::Mat & infMatrix,
		const cv::Mat & userData) :
	from_(from),
	to_(to),
	transform_(transform),
	type_(type)
{
	setInfMatrix(infMatrix);

	if(userData.type() == CV_8UC1) // Bytes
	{
		_userDataCompressed = userData; // assume compressed
	}
	else
	{
		_userDataRaw = userData;
	}
}

double Link::rotVariance() const
{
	double min = uMax3(infMatrix_.at<double>(3,3), infMatrix_.at<double>(4,4), infMatrix_.at<double>(5,5));
	UASSERT(min > 0.0);
	return 1.0/min;
}
double Link::transVariance() const
{
	double min = uMax3(infMatrix_.at<double>(0,0), infMatrix_.at<double>(1,1), infMatrix_.at<double>(2,2));
	UASSERT(min > 0.0);
	return 1.0/min;
}

void Link::setInfMatrix(const cv::Mat & infMatrix) {
	UASSERT(infMatrix.cols == 6 && infMatrix.rows == 6 && infMatrix.type() == CV_64FC1);
	UASSERT_MSG(uIsFinite(infMatrix.at<double>(0,0)) && infMatrix.at<double>(0,0)>0, uFormat("Linear information X should not be null! Value=%f (set to 1 if unknown or <=1/9999 to be ignored in some computations).", infMatrix.at<double>(0,0)).c_str());
	UASSERT_MSG(uIsFinite(infMatrix.at<double>(1,1)) && infMatrix.at<double>(1,1)>0, uFormat("Linear information Y should not be null! Value=%f (set to 1 if unknown or <=1/9999 to be ignored in some computations).", infMatrix.at<double>(1,1)).c_str());
	UASSERT_MSG(uIsFinite(infMatrix.at<double>(2,2)) && infMatrix.at<double>(2,2)>0, uFormat("Linear information Z should not be null! Value=%f (set to 1 if unknown or <=1/9999 to be ignored in some computations).", infMatrix.at<double>(2,2)).c_str());
	UASSERT_MSG(uIsFinite(infMatrix.at<double>(3,3)) && infMatrix.at<double>(3,3)>0, uFormat("Angular information roll should not be null! Value=%f (set to 1 if unknown or <=1/9999 to be ignored in some computations).", infMatrix.at<double>(3,3)).c_str());
	UASSERT_MSG(uIsFinite(infMatrix.at<double>(4,4)) && infMatrix.at<double>(4,4)>0, uFormat("Angular information pitch should not be null! Value=%f (set to 1 if unknown or <=1/9999 to be ignored in some computations).", infMatrix.at<double>(4,4)).c_str());
	UASSERT_MSG(uIsFinite(infMatrix.at<double>(5,5)) && infMatrix.at<double>(5,5)>0, uFormat("Angular information yaw should not be null! Value=%f (set to 1 if unknown or <=1/9999 to be ignored in some computations).", infMatrix.at<double>(5,5)).c_str());
	infMatrix_ = infMatrix;
}

void Link::uncompressUserData()
{
	cv::Mat dataRaw = uncompressUserDataConst();
	if(!dataRaw.empty() && _userDataRaw.empty())
	{
		_userDataRaw = dataRaw;
	}
}

cv::Mat Link::uncompressUserDataConst() const
{
	if(!_userDataRaw.empty())
	{
		return _userDataRaw;
	}
	return uncompressData(_userDataCompressed);
}

Link Link::merge(const Link & link, Type outputType) const
{
	UASSERT(to_ == link.from());
	UASSERT(outputType != Link::kUndef);
	UASSERT((link.transform().isNull() && transform_.isNull()) || (!link.transform().isNull() && !transform_.isNull()));
	UASSERT(infMatrix_.cols == 6 && infMatrix_.rows == 6 && infMatrix_.type() == CV_64FC1);
	UASSERT(link.infMatrix().cols == 6 && link.infMatrix().rows == 6 && link.infMatrix().type() == CV_64FC1);
	if(outputType == kNeighborMerged)
	{
		return Link(
			from_,
			link.to(),
			outputType,
			transform_.isNull()?Transform():transform_ * link.transform(),
			transform_.isNull()?cv::Mat::eye(6,6,CV_64FC1):(infMatrix_.inv() + link.infMatrix().inv()).inv());
	}
	return Link(
			from_,
			link.to(),
			outputType,
			transform_.isNull()?Transform():transform_ * link.transform(), // FIXME, should be inf1^-1(inf1*t1 + inf2*t2)
			transform_.isNull()?cv::Mat::eye(6,6,CV_64FC1):(infMatrix_.at<double>(0,0)<link.infMatrix().at<double>(0,0)?infMatrix_:link.infMatrix()));
			//transform_.isNull()?cv::Mat::eye(6,6,CV_64FC1):(infMatrix_.inv() + link.infMatrix().inv()).inv());
}

Link Link::inverse() const
{
	return Link(
			to_,
			from_,
			type_,
			transform_.isNull()?Transform():transform_.inverse(),
			transform_.isNull()?cv::Mat::eye(6,6,CV_64FC1):infMatrix_);
}

}
