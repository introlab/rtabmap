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

namespace rtabmap {

class CameraModel
{
public:
	CameraModel();
	virtual ~CameraModel() {}

	double fx() const {return P_.at<double>(0,0);}
	double fy() const {return P_.at<double>(1,1);}
	double cx() const {return P_.at<double>(0,2);}
	double cy() const {return P_.at<double>(1,2);}
	double Tx() const {return P_.at<double>(0,3);}

	const cv::Mat & K() const {return K_;}
	const cv::Mat & D() const {return D_;}
	const cv::Mat & R() const {return R_;}
	const cv::Mat & P() const {return P_;}

	int width() const {return width_;}
	int height() const {return height_;}

	bool load(const std::string & directory, const std::string & cameraName);
	void save(const std::string & directory, const std::string & cameraName);

	cv::Mat rectifyImage(const cv::Mat & raw) const;

private:
	int width_;
	int height_;
	cv::Mat K_;
	cv::Mat D_;
	cv::Mat R_;
	cv::Mat P_;
	cv::Mat rectificationMap1_;
	cv::Mat rectificationMap2_;
};

class StereoCameraModel
{
public:
	StereoCameraModel() {}
	virtual ~StereoCameraModel() {}

	bool load(const std::string & directory, const std::string & cameraName)
	{
		return left_.load(directory, cameraName+"_left") &&
				right_.load(directory, cameraName+"_right");
	}
	void save(const std::string & directory, const std::string & cameraName)
	{
		left_.save(directory, cameraName+"_left");
		right_.save(directory, cameraName+"_right");
	}
	double baseline() const {return -right_.Tx()/right_.fx();}

	const CameraModel & left() const {return left_;}
	const CameraModel & right() const {return right_;}

private:
	CameraModel left_;
	CameraModel right_;
};

} /* namespace rtabmap */
#endif /* CAMERAMODEL_H_ */
