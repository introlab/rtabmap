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

#ifndef LINK_H_
#define LINK_H_

#include <rtabmap/core/Transform.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UMath.h>
#include <opencv2/core/core.hpp>

namespace rtabmap {

class Link
{
public:
	enum Type {kNeighbor, kGlobalClosure, kLocalSpaceClosure, kLocalTimeClosure, kUserClosure, kVirtualClosure, kUndef};
	Link() :
		from_(0),
		to_(0),
		type_(kUndef),
		infMatrix_(cv::Mat::eye(6,6,CV_64FC1))
	{
	}
	Link(int from,
			int to,
			Type type,
			const Transform & transform,
			const cv::Mat & infMatrix = cv::Mat::eye(6,6,CV_64FC1)) :
		from_(from),
		to_(to),
		transform_(transform),
		type_(type)
	{
		setInfMatrix(infMatrix);
	}
	Link(int from,
			int to,
			Type type,
			const Transform & transform,
			double rotVariance,
			double transVariance) :
		from_(from),
		to_(to),
		transform_(transform),
		type_(type)
	{
		setVariance(rotVariance, transVariance);
	}

	bool isValid() const {return from_ > 0 && to_ > 0 && !transform_.isNull() && type_!=kUndef;}

	int from() const {return from_;}
	int to() const {return to_;}
	const Transform & transform() const {return transform_;}
	Type type() const {return type_;}
	const cv::Mat & infMatrix() const {return infMatrix_;}
	double rotVariance() const
	{
		double min = uMin3(infMatrix_.at<double>(3,3), infMatrix_.at<double>(4,4), infMatrix_.at<double>(5,5));
		UASSERT(min > 0.0);
		return 1.0/min;
	}
	double transVariance() const
	{
		double min = uMin3(infMatrix_.at<double>(0,0), infMatrix_.at<double>(1,1), infMatrix_.at<double>(2,2));
		UASSERT(min > 0.0);
		return 1.0/min;
	}

	void setFrom(int from) {from_ = from;}
	void setTo(int to) {to_ = to;}
	void setTransform(const Transform & transform) {transform_ = transform;}
	void setType(Type type) {type_ = type;}
	void setInfMatrix(const cv::Mat & infMatrix) {
		UASSERT(infMatrix.cols == 6 && infMatrix.rows == 6 && infMatrix.type() == CV_64FC1);
		UASSERT_MSG(uIsFinite(infMatrix.at<double>(0,0)) && infMatrix.at<double>(0,0)>0, "Transitional information should not be null! (set to 1 if unknown)");
		UASSERT_MSG(uIsFinite(infMatrix.at<double>(1,1)) && infMatrix.at<double>(1,1)>0, "Transitional information should not be null! (set to 1 if unknown)");
		UASSERT_MSG(uIsFinite(infMatrix.at<double>(2,2)) && infMatrix.at<double>(2,2)>0, "Transitional information should not be null! (set to 1 if unknown)");
		UASSERT_MSG(uIsFinite(infMatrix.at<double>(3,3)) && infMatrix.at<double>(3,3)>0, "Rotational information should not be null! (set to 1 if unknown)");
		UASSERT_MSG(uIsFinite(infMatrix.at<double>(4,4)) && infMatrix.at<double>(4,4)>0, "Rotational information should not be null! (set to 1 if unknown)");
		UASSERT_MSG(uIsFinite(infMatrix.at<double>(5,5)) && infMatrix.at<double>(5,5)>0, "Rotational information should not be null! (set to 1 if unknown)");
		infMatrix_ = infMatrix;
	}
	void setVariance(double rotVariance, double transVariance) {
		UASSERT(uIsFinite(rotVariance) && rotVariance>0);
		UASSERT(uIsFinite(transVariance) && transVariance>0);
		infMatrix_ = cv::Mat::eye(6,6,CV_64FC1);
		infMatrix_.at<double>(0,0) = 1.0/transVariance;
		infMatrix_.at<double>(1,1) = 1.0/transVariance;
		infMatrix_.at<double>(2,2) = 1.0/transVariance;
		infMatrix_.at<double>(3,3) = 1.0/rotVariance;
		infMatrix_.at<double>(4,4) = 1.0/rotVariance;
		infMatrix_.at<double>(5,5) = 1.0/rotVariance;
	}

	Link merge(const Link & link) const
	{
		UASSERT(to_ == link.from());
		UASSERT(type_ == link.type());
		UASSERT(!transform_.isNull());
		UASSERT(!link.transform().isNull());
		UASSERT(infMatrix_.cols == 6 && infMatrix_.rows == 6 && infMatrix_.type() == CV_64FC1);
		UASSERT(link.infMatrix().cols == 6 && link.infMatrix().rows == 6 && link.infMatrix().type() == CV_64FC1);
		return Link(
				from_,
				link.to(),
				type_,
				transform_ * link.transform(),
				infMatrix_ + link.infMatrix());
	}

	Link inverse() const
	{
		return Link(to_, from_, type_, transform_.inverse(), infMatrix_);
	}

private:
	int from_;
	int to_;
	Transform transform_;
	Type type_;
	cv::Mat infMatrix_; // Information matrix = covariance matrix ^ -1
};

}


#endif /* LINK_H_ */
