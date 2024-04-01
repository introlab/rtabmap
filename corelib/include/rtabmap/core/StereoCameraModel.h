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

#ifndef STEREOCAMERAMODEL_H_
#define STEREOCAMERAMODEL_H_

#include <rtabmap/core/CameraModel.h>

namespace rtabmap {

class RTABMAP_CORE_EXPORT StereoCameraModel
{
public:
	StereoCameraModel() : leftSuffix_("left"), rightSuffix_("right") {}
	StereoCameraModel(
			const std::string & name,
			const cv::Size & imageSize1,
			const cv::Mat & K1, const cv::Mat & D1, const cv::Mat & R1, const cv::Mat & P1,
			const cv::Size & imageSize2,
			const cv::Mat & K2, const cv::Mat & D2, const cv::Mat & R2, const cv::Mat & P2,
			const cv::Mat & R, const cv::Mat & T, const cv::Mat & E, const cv::Mat & F,
			const Transform & localTransform = Transform(0,0,1,0, -1,0,0,0, 0,-1,0,0));

	// if R and T are not null, left and right camera models should be valid to be rectified.
	StereoCameraModel(
			const std::string & name,
			const CameraModel & leftCameraModel,
			const CameraModel & rightCameraModel,
			const cv::Mat & R = cv::Mat(),
			const cv::Mat & T = cv::Mat(),
			const cv::Mat & E = cv::Mat(),
			const cv::Mat & F = cv::Mat());
	// if extrinsics transform is not null, left and right camera models should be valid to be rectified.
	StereoCameraModel(
			const std::string & name,
			const CameraModel & leftCameraModel,
			const CameraModel & rightCameraModel,
			const Transform & extrinsics);

	//minimal
	StereoCameraModel(
			double fx,
			double fy,
			double cx,
			double cy,
			double baseline,
			const Transform & localTransform = Transform(0,0,1,0, -1,0,0,0, 0,-1,0,0),
			const cv::Size & imageSize = cv::Size(0,0));
	//minimal to be saved
	StereoCameraModel(
			const std::string & name,
			double fx,
			double fy,
			double cx,
			double cy,
			double baseline,
			const Transform & localTransform = Transform(0,0,1,0, -1,0,0,0, 0,-1,0,0),
			const cv::Size & imageSize = cv::Size(0,0));
	virtual ~StereoCameraModel() {}

	bool isValidForProjection() const {return left_.isValidForProjection() && right_.isValidForProjection() && baseline() > 0.0;}
	bool isValidForRectification() const {return left_.isValidForRectification() && right_.isValidForRectification();}

	void initRectificationMap() {left_.initRectificationMap(); right_.initRectificationMap();}
	bool isRectificationMapInitialized() const {return left_.isRectificationMapInitialized() && right_.isRectificationMapInitialized();}

	void setName(const std::string & name, const std::string & leftSuffix = "left", const std::string & rightSuffix = "right");
	const std::string & name() const {return name_;}

	// backward compatibility
	void setImageSize(const cv::Size & size) {left_.setImageSize(size); right_.setImageSize(size);}

	bool load(const std::string & directory, const std::string & cameraName, bool ignoreStereoTransform = true);
	bool save(const std::string & directory, bool ignoreStereoTransform = true) const;
	bool saveStereoTransform(const std::string & directory) const;
	std::vector<unsigned char> serialize() const;
	unsigned int deserialize(const std::vector<unsigned char>& data);
	unsigned int deserialize(const unsigned char * data, unsigned int dataSize);

	double baseline() const {return right_.fx()!=0.0 && left_.fx() != 0.0 ? left_.Tx() / left_.fx() - right_.Tx()/right_.fx():0.0;}

	float computeDepth(float disparity) const;
	float computeDisparity(float depth) const; // m
	float computeDisparity(unsigned short depth) const; // mm

	const cv::Mat & R() const {return R_;} //extrinsic rotation matrix
	const cv::Mat & T() const {return T_;} //extrinsic translation matrix
	const cv::Mat & E() const {return E_;} //extrinsic essential matrix
	const cv::Mat & F() const {return F_;} //extrinsic fundamental matrix

	void scale(double scale);
	void roi(const cv::Rect & roi);

	void setLocalTransform(const Transform & transform) {left_.setLocalTransform(transform);}
	const Transform & localTransform() const {return left_.localTransform();}
	Transform stereoTransform() const;

	const CameraModel & left() const {return left_;}
	const CameraModel & right() const {return right_;}

	const std::string & getLeftSuffix() const {return leftSuffix_;}
	const std::string & getRightSuffix() const {return rightSuffix_;}

private:
	void updateStereoRectification();

private:
	std::string leftSuffix_;
	std::string rightSuffix_;
	CameraModel left_;
	CameraModel right_;
	std::string name_;
	cv::Mat R_;
	cv::Mat T_;
	cv::Mat E_;
	cv::Mat F_;
};

RTABMAP_CORE_EXPORT std::ostream& operator<<(std::ostream& os, const StereoCameraModel& model);

} // rtabmap

#endif /* STEREOCAMERAMODEL_H_ */
