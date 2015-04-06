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

#include <rtabmap/core/CameraModel.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UDirectory.h>
#include <rtabmap/utilite/UFile.h>
#include <opencv2/imgproc/imgproc.hpp>

namespace rtabmap {

CameraModel::CameraModel() :
		P_(cv::Mat::zeros(3, 4, CV_64FC1))
{

}

CameraModel::CameraModel(const std::string & cameraName, const cv::Size & imageSize, const cv::Mat & K, const cv::Mat & D, const cv::Mat & R, const cv::Mat & P) :
		name_(cameraName),
		imageSize_(imageSize),
		K_(K),
		D_(D),
		R_(R),
		P_(P)
{
	UASSERT(!name_.empty());
	UASSERT(imageSize_.width > 0 && imageSize_.height > 0);
	UASSERT(K_.rows == 3 && K_.cols == 3);
	UASSERT(D_.rows == 1 && (D_.cols == 4 || D_.cols == 5 || D_.cols == 8));
	UASSERT(R_.rows == 3 && R_.cols == 3);
	UASSERT(P_.rows == 3 && P_.cols == 4);

	// init rectification map
	UINFO("Initialize rectify map");
	cv::initUndistortRectifyMap(K_, D_, R_, P_, imageSize_, CV_16SC2, rectificationMap1_, rectificationMap2_);
}

bool CameraModel::load(const std::string & filePath)
{
	K_ = cv::Mat();
	D_ = cv::Mat();
	R_ = cv::Mat();
	P_ = cv::Mat::zeros(3, 4, CV_64FC1);
	rectificationMap1_ = cv::Mat();
	rectificationMap2_ = cv::Mat();

	if(UFile::exists(filePath))
	{
		UINFO("Reading calibration file \"%s\"", filePath.c_str());
		cv::FileStorage fs(filePath, cv::FileStorage::READ);

		name_ = (int)fs["camera_name"];
		imageSize_.width = (int)fs["image_width"];
		imageSize_.height = (int)fs["image_height"];
		UASSERT(!name_.empty());
		UASSERT(imageSize_.width > 0);
		UASSERT(imageSize_.height > 0);

		// import from ROS calibration format
		cv::FileNode n = fs["camera_matrix"];
		int rows = (int)n["rows"];
		int cols = (int)n["cols"];
		std::vector<double> data;
		n["data"] >> data;
		UASSERT(rows*cols == (int)data.size());
		UASSERT(rows == 3 && cols == 3);
		K_ = cv::Mat(rows, cols, CV_64FC1, data.data()).clone();

		n = fs["distortion_coefficients"];
		rows = (int)n["rows"];
		cols = (int)n["cols"];
		data.clear();
		n["data"] >> data;
		UASSERT(rows*cols == (int)data.size());
		UASSERT(rows == 1 && (cols == 4 || cols == 5 || cols == 8));
		D_ = cv::Mat(rows, cols, CV_64FC1, data.data()).clone();

		n = fs["rectification_matrix"];
		rows = (int)n["rows"];
		cols = (int)n["cols"];
		data.clear();
		n["data"] >> data;
		UASSERT(rows*cols == (int)data.size());
		UASSERT(rows == 3 && cols == 3);
		R_ = cv::Mat(rows, cols, CV_64FC1, data.data()).clone();

		n = fs["projection_matrix"];
		rows = (int)n["rows"];
		cols = (int)n["cols"];
		data.clear();
		n["data"] >> data;
		UASSERT(rows*cols == (int)data.size());
		UASSERT(rows == 3 && cols == 4);
		P_ = cv::Mat(rows, cols, CV_64FC1, data.data()).clone();

		fs.release();

		// init rectification map
		UINFO("Initialize rectify map");
		cv::initUndistortRectifyMap(K_, D_, R_, P_, imageSize_, CV_16SC2, rectificationMap1_, rectificationMap2_);

		return true;
	}
	return false;
}

bool CameraModel::save(const std::string & filePath)
{
	if(!filePath.empty() && !name_.empty() && !K_.empty() && !D_.empty() && !R_.empty() && !P_.empty())
	{
		UINFO("Saving calibration to file \"%s\"", filePath.c_str());
		cv::FileStorage fs(filePath, cv::FileStorage::WRITE);

		// export in ROS calibration format

		fs << "camera_name" << name_;
		fs << "image_width" << imageSize_.width;
		fs << "image_height" << imageSize_.height;

		fs << "camera_matrix" << "{";
		fs << "rows" << K_.rows;
		fs << "cols" << K_.cols;
		fs << "data" << std::vector<double>((double*)K_.data, ((double*)K_.data)+(K_.rows*K_.cols));
		fs << "}";

		fs << "distortion_coefficients" << "{";
		fs << "rows" << D_.rows;
		fs << "cols" << D_.cols;
		fs << "data" << std::vector<double>((double*)D_.data, ((double*)D_.data)+(D_.rows*D_.cols));
		fs << "}";

		fs << "rectification_matrix" << "{";
		fs << "rows" << R_.rows;
		fs << "cols" << R_.cols;
		fs << "data" << std::vector<double>((double*)R_.data, ((double*)R_.data)+(R_.rows*R_.cols));
		fs << "}";

		fs << "projection_matrix" << "{";
		fs << "rows" << P_.rows;
		fs << "cols" << P_.cols;
		fs << "data" << std::vector<double>((double*)P_.data, ((double*)P_.data)+(P_.rows*P_.cols));
		fs << "}";

		fs.release();

		return true;
	}
	return false;
}

cv::Mat CameraModel::rectifyImage(const cv::Mat & raw) const
{
	if(!rectificationMap1_.empty() && !rectificationMap2_.empty())
	{
		cv::Mat rectified;
		cv::remap(raw, rectified, rectificationMap1_, rectificationMap2_, cv::INTER_LINEAR);
		return rectified;
	}
	else
	{
		return raw;
	}
}

} /* namespace rtabmap */
