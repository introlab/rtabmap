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

#ifndef CAMERAMODEL_H_
#define CAMERAMODEL_H_

#include <opencv2/opencv.hpp>

#include "rtabmap/core/rtabmap_core_export.h" // DLL export/import defines
#include "rtabmap/core/Transform.h"

namespace rtabmap {

class RTABMAP_CORE_EXPORT CameraModel
{
public:
	/**
	 * Optical rotation used to transform image coordinate frame (x->right, y->down, z->forward)
	 * to robot coordinate frame (x->forward, y->left, z->up).
	 */
	static Transform opticalRotation() {return Transform(0,0,1,0, -1,0,0,0, 0,-1,0,0);}

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
			const Transform & localTransform = opticalRotation());

	// minimal
	CameraModel(
			double fx,
			double fy,
			double cx,
			double cy,
			const Transform & localTransform = opticalRotation(),
			double Tx = 0.0f,
			const cv::Size & imageSize = cv::Size(0,0));
	// minimal to be saved
	CameraModel(
			const std::string & name,
			double fx,
			double fy,
			double cx,
			double cy,
			const Transform & localTransform = opticalRotation(),
			double Tx = 0.0f,
			const cv::Size & imageSize = cv::Size(0,0));

	virtual ~CameraModel() {}

	bool initRectificationMap();
	bool isRectificationMapInitialized() const {return !mapX_.empty() && !mapY_.empty();}

	bool isValidForProjection() const {return fx()>0.0 && fy()>0.0 && cx()>0.0 && cy()>0.0;}
	bool isValidForReprojection() const {return fx()>0.0 && fy()>0.0 && cx()>0.0 && cy()>0.0 && imageWidth()>0 && imageHeight()>0;}
	bool isValidForRectification() const
	{
		return imageSize_.width>0 &&
			   imageSize_.height>0 &&
			   !K_.empty() &&
			   !D_.empty() &&
			   !R_.empty() &&
			   !P_.empty();
	}

	void setName(const std::string & name) {name_=name;}
	const std::string & name() const {return name_;}

	double fx() const {return P_.empty()?K_.empty()?0.0:K_.at<double>(0,0):P_.at<double>(0,0);}
	double fy() const {return P_.empty()?K_.empty()?0.0:K_.at<double>(1,1):P_.at<double>(1,1);}
	double cx() const {return P_.empty()?K_.empty()?0.0:K_.at<double>(0,2):P_.at<double>(0,2);}
	double cy() const {return P_.empty()?K_.empty()?0.0:K_.at<double>(1,2):P_.at<double>(1,2);}
	double Tx() const {return P_.empty()?0.0:P_.at<double>(0,3);}

	cv::Mat K_raw() const {return K_;} //intrinsic camera matrix (before rectification)
	cv::Mat D_raw() const {return D_;} //intrinsic distorsion matrix (before rectification)
	cv::Mat K() const {return !P_.empty()?P_.colRange(0,3):K_;} // if P exists, return rectified version
	cv::Mat D() const {return P_.empty()&&!D_.empty()?D_:cv::Mat::zeros(1,5,CV_64FC1);} // if P exists, return rectified version
	cv::Mat R() const {return R_;} //rectification matrix
	cv::Mat P() const {return P_;} //projection matrix

	void setLocalTransform(const Transform & transform) {localTransform_ = transform;}
	const Transform & localTransform() const {return localTransform_;}

	void setImageSize(const cv::Size & size);
	const cv::Size & imageSize() const {return imageSize_;}
	int imageWidth() const {return imageSize_.width;}
	int imageHeight() const {return imageSize_.height;}

	double fovX() const; // in radians
	double fovY() const; // in radians
	double horizontalFOV() const; // in degrees
	double verticalFOV() const;   // in degrees
	bool isFisheye() const {return D_.cols == 6;}

	bool load(const std::string & filePath);
	bool load(const std::string & directory, const std::string & cameraName);
	bool save(const std::string & directory) const;
	std::vector<unsigned char> serialize() const;
	unsigned int deserialize(const std::vector<unsigned char>& data);
	unsigned int deserialize(const unsigned char * data, unsigned int dataSize);

	CameraModel scaled(double scale) const;
	CameraModel roi(const cv::Rect & roi) const;

	// For depth images, your should use cv::INTER_NEAREST
	cv::Mat rectifyImage(const cv::Mat & raw, int interpolation = cv::INTER_LINEAR) const;
	cv::Mat rectifyDepth(const cv::Mat & raw) const;

	// Project 2D pixel to 3D (in /camera_link frame)
	void project(float u, float v, float depth, float & x, float & y, float & z) const;
	// Reproject 3D point (in /camera_link frame) to pixel
	void reproject(float x, float y, float z, float & u, float & v) const;
	void reproject(float x, float y, float z, int & u, int & v) const;
	bool inFrame(int u, int v) const;

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

RTABMAP_CORE_EXPORT std::ostream& operator<<(std::ostream& os, const CameraModel& model);

} /* namespace rtabmap */
#endif /* CAMERAMODEL_H_ */
