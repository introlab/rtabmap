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
		width_(0),
		height_(0),
		P_(cv::Mat::zeros(3, 4, CV_64FC1))
{

}

bool CameraModel::load(const std::string & directory, const std::string & cameraName)
{
	K_ = cv::Mat();
	D_ = cv::Mat();
	R_ = cv::Mat();
	P_ = cv::Mat::zeros(3, 4, CV_64FC1);
	rectificationMap1_ = cv::Mat();
	rectificationMap2_ = cv::Mat();

	std::string path = directory + UDirectory::separator() + cameraName + ".yaml";
	if(UFile::exists(path))
	{
		UINFO("Reading calibration file \"%s\"", path.c_str());
		cv::FileStorage fs(path, cv::FileStorage::READ);

		width_ = (int)fs["image_width"];
		height_ = (int)fs["image_height"];

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
		cv::initUndistortRectifyMap(K_, D_, R_, P_, cv::Size(width_, height_),
				CV_16SC2, rectificationMap1_, rectificationMap2_);

		return true;
	}
	return false;
}

void CameraModel::save(const std::string & directory, const std::string & cameraName)
{
	UFATAL("not implemented");
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
