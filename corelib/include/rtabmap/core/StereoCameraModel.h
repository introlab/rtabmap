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

#ifndef STEREOCAMERAMODEL_H_
#define STEREOCAMERAMODEL_H_

#include <rtabmap/core/CameraModel.h>

namespace rtabmap {

class RTABMAP_EXP StereoCameraModel
{
public:
	StereoCameraModel() {}
	StereoCameraModel(
			const std::string & name,
			const cv::Size & imageSize1,
			const cv::Mat & K1, const cv::Mat & D1, const cv::Mat & R1, const cv::Mat & P1,
			const cv::Size & imageSize2,
			const cv::Mat & K2, const cv::Mat & D2, const cv::Mat & R2, const cv::Mat & P2,
			const cv::Mat & R, const cv::Mat & T, const cv::Mat & E, const cv::Mat & F,
			const Transform & localTransform = Transform::getIdentity()) :
		left_(name+"_left", imageSize1, K1, D1, R1, P1, localTransform),
		right_(name+"_right", imageSize2, K2, D2, R2, P2, localTransform),
		name_(name),
		R_(R),
		T_(T),
		E_(E),
		F_(F)
	{
	}
	StereoCameraModel(
			const std::string & name,
			const CameraModel & leftCameraModel,
			const CameraModel & rightCameraModel,
			const cv::Mat & R = cv::Mat(),
			const cv::Mat & T = cv::Mat(),
			const cv::Mat & E = cv::Mat(),
			const cv::Mat & F = cv::Mat()) :
		left_(leftCameraModel),
		right_(rightCameraModel),
		name_(name),
		R_(R),
		T_(T),
		E_(E),
		F_(F)
	{
		left_.setName(name+"_left");
		right_.setName(name+"_right");
	}
	//minimal
	StereoCameraModel(
			double fx,
			double fy,
			double cx,
			double cy,
			double baseline,
			const Transform & localTransform = Transform::getIdentity()) :
		left_(fx, fy, cx, cy, localTransform),
		right_(fx, fy, cx, cy, localTransform, baseline*-fx)
	{
	}
	//minimal to be saved
	StereoCameraModel(
			const std::string & name,
			double fx,
			double fy,
			double cx,
			double cy,
			double baseline,
			const Transform & localTransform = Transform::getIdentity()) :
		left_(name+"_left", fx, fy, cx, cy, localTransform),
		right_(name+"_right", fx, fy, cx, cy, localTransform, baseline*-fx),
		name_(name)
	{
	}
	virtual ~StereoCameraModel() {}

	bool isValid() const {return left_.isValid() && right_.isValid() && baseline() > 0.0;}

	void setName(const std::string & name);
	const std::string & name() const {return name_;}

	bool load(const std::string & directory, const std::string & cameraName, bool ignoreStereoTransform = true);
	bool save(const std::string & directory, bool ignoreStereoTransform = true) const;

	double baseline() const {return -right_.Tx()/right_.fx();}

	float computeDepth(float disparity) const;
	float computeDisparity(float depth) const; // m
	float computeDisparity(unsigned short depth) const; // mm

	const cv::Mat & R() const {return R_;} //extrinsic rotation matrix
	const cv::Mat & T() const {return T_;} //extrinsic translation matrix
	const cv::Mat & E() const {return E_;} //extrinsic essential matrix
	const cv::Mat & F() const {return F_;} //extrinsic fundamental matrix

	void scale(double scale);

	void setLocalTransform(const Transform & transform) {left_.setLocalTransform(transform);}
	const Transform & localTransform() const {return left_.localTransform();}
	Transform stereoTransform() const;

	const CameraModel & left() const {return left_;}
	const CameraModel & right() const {return right_;}

private:
	CameraModel left_;
	CameraModel right_;
	std::string name_;
	cv::Mat R_;
	cv::Mat T_;
	cv::Mat E_;
	cv::Mat F_;
};

} // rtabmap

#endif /* STEREOCAMERAMODEL_H_ */
