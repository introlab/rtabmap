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
#include <rtabmap/utilite/UConversion.h>
#include <opencv2/imgproc/imgproc.hpp>

namespace rtabmap {

CameraModel::CameraModel() :
		P_(cv::Mat::zeros(3, 4, CV_64FC1))
{

}

CameraModel::CameraModel(
		const std::string & cameraName,
		const cv::Size & imageSize,
		const cv::Mat & K,
		const cv::Mat & D,
		const cv::Mat & R,
		const cv::Mat & P,
		const Transform & localTransform) :
		name_(cameraName),
		imageSize_(imageSize),
		K_(K),
		D_(D),
		R_(R),
		P_(P),
		localTransform_(localTransform)
{
	UASSERT(!name_.empty());
	UASSERT(imageSize_.width > 0 && imageSize_.height > 0);
	UASSERT(K_.rows == 3 && K_.cols == 3);
	UASSERT(D_.rows == 1 && (D_.cols == 4 || D_.cols == 5 || D_.cols == 8));
	UASSERT(R_.rows == 3 && R_.cols == 3);
	UASSERT(P_.rows == 3 && P_.cols == 4);

	// init rectification map
	UINFO("Initialize rectify map");
	cv::initUndistortRectifyMap(K_, D_, R_, P_, imageSize_, CV_32FC1, mapX_, mapY_);
}

CameraModel::CameraModel(
		double fx,
		double fy,
		double cx,
		double cy,
		const Transform & localTransform,
		double Tx) :
		K_(cv::Mat::eye(3, 3, CV_64FC1)),
		D_(cv::Mat::zeros(1, 5, CV_64FC1)),
		R_(cv::Mat::eye(3, 3, CV_64FC1)),
		P_(cv::Mat::eye(3, 4, CV_64FC1)),
		localTransform_(localTransform)
{
	UASSERT_MSG(fx >= 0.0, uFormat("fx=%f", fx).c_str());
	UASSERT_MSG(fy >= 0.0, uFormat("fy=%f", fy).c_str());
	UASSERT_MSG(cx >= 0.0, uFormat("cx=%f", cx).c_str());
	UASSERT_MSG(cy >= 0.0, uFormat("cy=%f", cy).c_str());
	P_.at<double>(0,0) = fx;
	P_.at<double>(1,1) = fy;
	P_.at<double>(0,2) = cx;
	P_.at<double>(1,2) = cy;
	P_.at<double>(0,3) = Tx;

	K_.at<double>(0,0) = fx;
	K_.at<double>(1,1) = fy;
	K_.at<double>(0,2) = cx;
	K_.at<double>(1,2) = cy;
}

bool CameraModel::load(const std::string & filePath)
{
	K_ = cv::Mat();
	D_ = cv::Mat();
	R_ = cv::Mat();
	P_ = cv::Mat::zeros(3, 4, CV_64FC1);
	mapX_ = cv::Mat();
	mapY_ = cv::Mat();

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
		cv::initUndistortRectifyMap(K_, D_, R_, P_, imageSize_, CV_32FC1, mapX_, mapY_);

		return true;
	}
	else
	{
		UWARN("Could not load calibration file \"%s\".", filePath.c_str());
	}
	return false;
}

bool CameraModel::save(const std::string & filePath) const
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

void CameraModel::scale(double scale)
{
	UASSERT(scale > 0.0);
	// has only effect on K and P
	imageSize_.width *= scale;
	imageSize_.height *= scale;
	K_.at<double>(0,0) *= scale;
	K_.at<double>(1,1) *= scale;
	K_.at<double>(0,2) *= scale;
	K_.at<double>(1,2) *= scale;
	P_.at<double>(0,0) *= scale;
	P_.at<double>(1,1) *= scale;
	P_.at<double>(0,2) *= scale;
	P_.at<double>(1,2) *= scale;
}

cv::Mat CameraModel::rectifyImage(const cv::Mat & raw, int interpolation) const
{
	if(!mapX_.empty() && !mapY_.empty())
	{
		cv::Mat rectified;
		cv::remap(raw, rectified, mapX_, mapY_, interpolation);
		return rectified;
	}
	else
	{
		return raw.clone();
	}
}

//inspired from https://github.com/code-iai/iai_kinect2/blob/master/depth_registration/src/depth_registration_cpu.cpp
cv::Mat CameraModel::rectifyDepth(const cv::Mat & raw) const
{
	UASSERT(raw.type() == CV_16UC1);
	if(!mapX_.empty() && !mapY_.empty())
	{
		cv::Mat rectified = cv::Mat::zeros(mapX_.rows, mapX_.cols, raw.type());
		for(int y=0; y<mapX_.rows; ++y)
		{
			for(int x=0; x<mapX_.cols; ++x)
			{
				cv::Point2f pt(mapX_.at<float>(y,x), mapY_.at<float>(y,x));
				int xL = (int)floor(pt.x);
				int xH = (int)ceil(pt.x);
				int yL = (int)floor(pt.y);
				int yH = (int)ceil(pt.y);
				if(xL >= 0 && yL >= 0 && xH < raw.cols && yH < raw.rows)
				{
					const unsigned short & pLT = raw.at<unsigned short>(yL, xL);
					const unsigned short & pRT = raw.at<unsigned short>(yL, xH);
					const unsigned short & pLB = raw.at<unsigned short>(yH, xL);
					const unsigned short & pRB = raw.at<unsigned short>(yH, xH);
					if(pLT > 0 && pRT > 0 && pLB > 0 && pRB > 0)
					{
						unsigned short avg = (pLT + pRT + pLB + pRB) / 4;
						unsigned short thres = 0.01 * avg;
						if( abs(pLT - avg) < thres &&
							abs(pRT - avg) < thres &&
							abs(pLB - avg) < thres &&
							abs(pRB - avg) < thres)
						{
							//bilinear interpolation
							float a = pt.x - (float)xL;
							float c = pt.y - (float)yL;

							//http://stackoverflow.com/questions/13299409/how-to-get-the-image-pixel-at-real-locations-in-opencv
							rectified.at<unsigned short>(y,x) =
									(raw.at<unsigned short>(yL, xL) * (1.f - a) + raw.at<unsigned short>(yL, xH) * a) * (1.f - c) +
									(raw.at<unsigned short>(yH, xL) * (1.f - a) + raw.at<unsigned short>(yH, xH) * a) * c;
						}
					}
				}
			}
		}
		return rectified;
	}
	else
	{
		return raw.clone();
	}
}

//
//StereoCameraModel
//
bool StereoCameraModel::load(const std::string & directory, const std::string & cameraName, bool ignoreStereoTransform)
{
	name_ = cameraName;
	if(left_.load(directory+"/"+cameraName+"_left.yaml") && right_.load(directory+"/"+cameraName+"_right.yaml"))
	{
		if(ignoreStereoTransform)
		{
			return true;
		}
		//load rotation, translation
		R_ = cv::Mat();
		T_ = cv::Mat();

		std::string filePath = directory+"/"+cameraName+"_pose.yaml";
		if(UFile::exists(filePath))
		{
			UINFO("Reading stereo calibration file \"%s\"", filePath.c_str());
			cv::FileStorage fs(filePath, cv::FileStorage::READ);

			name_ = (int)fs["camera_name"];

			// import from ROS calibration format
			cv::FileNode n = fs["rotation_matrix"];
			int rows = (int)n["rows"];
			int cols = (int)n["cols"];
			std::vector<double> data;
			n["data"] >> data;
			UASSERT(rows*cols == (int)data.size());
			UASSERT(rows == 3 && cols == 3);
			R_ = cv::Mat(rows, cols, CV_64FC1, data.data()).clone();

			n = fs["translation_matrix"];
			rows = (int)n["rows"];
			cols = (int)n["cols"];
			data.clear();
			n["data"] >> data;
			UASSERT(rows*cols == (int)data.size());
			UASSERT(rows == 3 && cols == 1);
			T_ = cv::Mat(rows, cols, CV_64FC1, data.data()).clone();

			n = fs["essential_matrix"];
			rows = (int)n["rows"];
			cols = (int)n["cols"];
			data.clear();
			n["data"] >> data;
			UASSERT(rows*cols == (int)data.size());
			UASSERT(rows == 3 && cols == 3);
			E_ = cv::Mat(rows, cols, CV_64FC1, data.data()).clone();

			n = fs["fundamental_matrix"];
			rows = (int)n["rows"];
			cols = (int)n["cols"];
			data.clear();
			n["data"] >> data;
			UASSERT(rows*cols == (int)data.size());
			UASSERT(rows == 3 && cols == 3);
			F_ = cv::Mat(rows, cols, CV_64FC1, data.data()).clone();

			fs.release();

			return true;
		}
		else
		{
			UWARN("Could not load stereo calibration file \"%s\".", filePath.c_str());
		}
	}
	return false;
}
bool StereoCameraModel::save(const std::string & directory, const std::string & cameraName, bool ignoreStereoTransform) const
{
	if(left_.save(directory+"/"+cameraName+"_left.yaml") && right_.save(directory+"/"+cameraName+"_right.yaml"))
	{
		if(ignoreStereoTransform)
		{
			return true;
		}
		std::string filePath = directory+"/"+cameraName+"_pose.yaml";
		if(!filePath.empty() && !name_.empty() && !R_.empty() && !T_.empty())
		{
			UINFO("Saving stereo calibration to file \"%s\"", filePath.c_str());
			cv::FileStorage fs(filePath, cv::FileStorage::WRITE);

			// export in ROS calibration format

			fs << "camera_name" << name_;

			fs << "rotation_matrix" << "{";
			fs << "rows" << R_.rows;
			fs << "cols" << R_.cols;
			fs << "data" << std::vector<double>((double*)R_.data, ((double*)R_.data)+(R_.rows*R_.cols));
			fs << "}";

			fs << "translation_matrix" << "{";
			fs << "rows" << T_.rows;
			fs << "cols" << T_.cols;
			fs << "data" << std::vector<double>((double*)T_.data, ((double*)T_.data)+(T_.rows*T_.cols));
			fs << "}";

			fs << "essential_matrix" << "{";
			fs << "rows" << E_.rows;
			fs << "cols" << E_.cols;
			fs << "data" << std::vector<double>((double*)E_.data, ((double*)E_.data)+(E_.rows*E_.cols));
			fs << "}";

			fs << "fundamental_matrix" << "{";
			fs << "rows" << F_.rows;
			fs << "cols" << F_.cols;
			fs << "data" << std::vector<double>((double*)F_.data, ((double*)F_.data)+(F_.rows*F_.cols));
			fs << "}";

			fs.release();

			return true;
		}
	}
	return false;
}

void StereoCameraModel::scale(double scale)
{
	left_.scale(scale);
	right_.scale(scale);
}

Transform StereoCameraModel::stereoTransform() const
{
	if(!R_.empty() && !T_.empty())
	{
		return Transform(
				R_.at<double>(0,0), R_.at<double>(0,1), R_.at<double>(0,2), T_.at<double>(0),
				R_.at<double>(1,0), R_.at<double>(1,1), R_.at<double>(1,2), T_.at<double>(1),
				R_.at<double>(2,0), R_.at<double>(2,1), R_.at<double>(2,2), T_.at<double>(2));
	}
	return Transform();
}

} /* namespace rtabmap */
