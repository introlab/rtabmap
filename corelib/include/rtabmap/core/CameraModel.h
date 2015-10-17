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

#ifndef CAMERAMODEL_H_
#define CAMERAMODEL_H_

#include <opencv2/opencv.hpp>

#include "rtabmap/core/RtabmapExp.h" // DLL export/import defines
#include "rtabmap/core/Transform.h"

namespace rtabmap {

class RTABMAP_EXP CameraModel
{
public:
	CameraModel();
	// K is the camera intrinsic 3x3 CV_64FC1
	// D is the distortion coefficients 1x5 CV_64FC1
	// R is the rectification matrix 3x3 CV_64FC1 (computed from stereo or Identity)
	// P is the projection matrix 3x4 CV_64FC1 (computed from stereo or equal to [K [0 0 1]'])
	CameraModel(
			const std::string & name,
			const cv::Size & imageSize,
			const cv::Mat & K,
			const cv::Mat & D,
			const cv::Mat & R,
			const cv::Mat & P,
			const Transform & localTransform = Transform::getIdentity());

	// minimal
	CameraModel(
			double fx,
			double fy,
			double cx,
			double cy,
			const Transform & localTransform = Transform::getIdentity(),
			double Tx = 0.0f);
	// minimal to be saved
	CameraModel(
			const std::string & name,
			double fx,
			double fy,
			double cx,
			double cy,
			const Transform & localTransform = Transform::getIdentity(),
			double Tx = 0.0f);

	virtual ~CameraModel() {}

	bool isValid() const {return !K_.empty() &&
									!D_.empty() &&
									!R_.empty() &&
									!P_.empty() &&
									fx()>0.0 &&
									fy()>0.0;}

	void setName(const std::string & name) {name_=name;}
	const std::string & name() const {return name_;}

	double fx() const {return P_.at<double>(0,0);}
	double fy() const {return P_.at<double>(1,1);}
	double cx() const {return P_.at<double>(0,2);}
	double cy() const {return P_.at<double>(1,2);}
	double Tx() const {return P_.at<double>(0,3);}

	const cv::Mat & K() const {return K_;} //intrinsic camera matrix
	const cv::Mat & D() const {return D_;} //intrinsic distorsion matrix
	const cv::Mat & R() const {return R_;} //rectification matrix
	const cv::Mat & P() const {return P_;} //projection matrix

	void setLocalTransform(const Transform & transform) {localTransform_ = transform;}
	const Transform & localTransform() const {return localTransform_;}

	void setImageSize(const cv::Size & size) {imageSize_ = size;}
	const cv::Size & imageSize() const {return imageSize_;}
	int imageWidth() const {return imageSize_.width;}
	int imageHeight() const {return imageSize_.height;}

	bool load(const std::string & directory, const std::string & cameraName);
	bool save(const std::string & directory) const;

	void scale(double scale);

	double horizontalFOV() const; // in degrees
	double verticalFOV() const;   // in degrees

	// For depth images, your should use cv::INTER_NEAREST
	cv::Mat rectifyImage(const cv::Mat & raw, int interpolation = cv::INTER_LINEAR) const;
	cv::Mat rectifyDepth(const cv::Mat & raw) const;

private:
	std::string name_;
	cv::Size imageSize_;
	cv::Mat K_;
	cv::Mat D_;
	cv::Mat R_;
	cv::Mat P_;
	cv::Mat mapX_;
	cv::Mat mapY_;
	Transform localTransform_;
};

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


} /* namespace rtabmap */
#endif /* CAMERAMODEL_H_ */
