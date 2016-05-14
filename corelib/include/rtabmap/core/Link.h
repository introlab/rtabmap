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

#include "rtabmap/core/RtabmapExp.h" // DLL export/import defines

#include <rtabmap/core/Transform.h>
#include <opencv2/core/core.hpp>

namespace rtabmap {

class RTABMAP_EXP Link
{
public:
	enum Type {
		kNeighbor,
		kGlobalClosure,
		kLocalSpaceClosure,
		kLocalTimeClosure,
		kUserClosure,
		kVirtualClosure,
		kNeighborMerged,
		kUndef};
	Link();
	Link(int from,
			int to,
			Type type,
			const Transform & transform,
			const cv::Mat & infMatrix = cv::Mat::eye(6,6,CV_64FC1),
			const cv::Mat & userData = cv::Mat());
	Link(int from,
			int to,
			Type type,
			const Transform & transform,
			double rotVariance,
			double transVariance,
			const cv::Mat & userData = cv::Mat());

	bool isValid() const {return from_ > 0 && to_ > 0 && !transform_.isNull() && type_!=kUndef;}

	int from() const {return from_;}
	int to() const {return to_;}
	const Transform & transform() const {return transform_;}
	Type type() const {return type_;}
	const cv::Mat & infMatrix() const {return infMatrix_;}
	double rotVariance() const;
	double transVariance() const;

	void setFrom(int from) {from_ = from;}
	void setTo(int to) {to_ = to;}
	void setTransform(const Transform & transform) {transform_ = transform;}
	void setType(Type type) {type_ = type;}
	void setInfMatrix(const cv::Mat & infMatrix);
	void setVariance(double rotVariance, double transVariance);

	void setUserDataRaw(const cv::Mat & userDataRaw); // only set raw
	void setUserData(const cv::Mat & userData); // detect automatically if raw or compressed. If raw, the data is compressed too.
	const cv::Mat & userDataRaw() const {return _userDataRaw;}
	const cv::Mat & userDataCompressed() const {return _userDataCompressed;}
	void uncompressUserData();
	cv::Mat uncompressUserDataConst() const;

	Link merge(const Link & link, Type outputType) const;
	Link inverse() const;

private:
	int from_;
	int to_;
	Transform transform_;
	Type type_;
	cv::Mat infMatrix_; // Information matrix = covariance matrix ^ -1

	// user data
	cv::Mat _userDataCompressed;      // compressed data
	cv::Mat _userDataRaw;
};

}


#endif /* LINK_H_ */
