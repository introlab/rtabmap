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

#include <rtabmap/core/StereoCameraModel.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UDirectory.h>
#include <rtabmap/utilite/UFile.h>
#include <rtabmap/utilite/UConversion.h>
#include <opencv2/imgproc/imgproc.hpp>

namespace rtabmap {

StereoCameraModel::StereoCameraModel(
		const std::string & name,
		const cv::Size & imageSize1,
		const cv::Mat & K1, const cv::Mat & D1, const cv::Mat & R1, const cv::Mat & P1,
		const cv::Size & imageSize2,
		const cv::Mat & K2, const cv::Mat & D2, const cv::Mat & R2, const cv::Mat & P2,
		const cv::Mat & R, const cv::Mat & T, const cv::Mat & E, const cv::Mat & F,
		const Transform & localTransform) :
				leftSuffix_("left"),
				rightSuffix_("right"),
				left_(name+"_"+leftSuffix_, imageSize1, K1, D1, R1, P1, localTransform),
				right_(name+"_"+rightSuffix_, imageSize2, K2, D2, R2, P2, localTransform),
				name_(name),
				R_(R),
				T_(T),
				E_(E),
				F_(F)
{
	UASSERT(R_.empty() || (R_.rows == 3 && R_.cols == 3 && R_.type() == CV_64FC1));
	UASSERT(T_.empty() || (T_.rows == 3 && T_.cols == 1 && T_.type() == CV_64FC1));
	UASSERT(E_.empty() || (E_.rows == 3 && E_.cols == 3 && E_.type() == CV_64FC1));
	UASSERT(F_.empty() || (F_.rows == 3 && F_.cols == 3 && F_.type() == CV_64FC1));
}

StereoCameraModel::StereoCameraModel(
		const std::string & name,
		const CameraModel & leftCameraModel,
		const CameraModel & rightCameraModel,
		const cv::Mat & R,
		const cv::Mat & T,
		const cv::Mat & E,
		const cv::Mat & F) :
				leftSuffix_("left"),
				rightSuffix_("right"),
				left_(leftCameraModel),
				right_(rightCameraModel),
				name_(name),
				R_(R),
				T_(T),
				E_(E),
				F_(F)
{
	left_.setName(name+"_"+getLeftSuffix());
	right_.setName(name+"_"+getRightSuffix());
	UASSERT(R_.empty() || (R_.rows == 3 && R_.cols == 3 && R_.type() == CV_64FC1));
	UASSERT(T_.empty() || (T_.rows == 3 && T_.cols == 1 && T_.type() == CV_64FC1));
	UASSERT(E_.empty() || (E_.rows == 3 && E_.cols == 3 && E_.type() == CV_64FC1));
	UASSERT(F_.empty() || (F_.rows == 3 && F_.cols == 3 && F_.type() == CV_64FC1));

	if(!R_.empty() && !T_.empty())
	{
		UASSERT(leftCameraModel.isValidForRectification() && rightCameraModel.isValidForRectification());

		cv::Mat R1,R2,P1,P2,Q;
		cv::stereoRectify(left_.K_raw(), left_.D_raw(),
				right_.K_raw(), right_.D_raw(),
				left_.imageSize(), R_, T_, R1, R2, P1, P2, Q,
				cv::CALIB_ZERO_DISPARITY, 0, left_.imageSize());

		left_ = CameraModel(left_.name(), left_.imageSize(), left_.K_raw(), left_.D_raw(), R1, P1, left_.localTransform());
		right_ = CameraModel(right_.name(), right_.imageSize(), right_.K_raw(), right_.D_raw(), R2, P2, right_.localTransform());
	}
}

StereoCameraModel::StereoCameraModel(
		const std::string & name,
		const CameraModel & leftCameraModel,
		const CameraModel & rightCameraModel,
		const Transform & extrinsics) :
				leftSuffix_("left"),
				rightSuffix_("right"),
				left_(leftCameraModel),
				right_(rightCameraModel),
				name_(name)
{
	left_.setName(name+"_"+getLeftSuffix());
	right_.setName(name+"_"+getRightSuffix());

	if(!extrinsics.isNull())
	{
		UASSERT(leftCameraModel.isValidForRectification() && rightCameraModel.isValidForRectification());

		extrinsics.rotationMatrix().convertTo(R_, CV_64FC1);
		extrinsics.translationMatrix().convertTo(T_, CV_64FC1);

		cv::Mat R1,R2,P1,P2,Q;
		cv::stereoRectify(left_.K_raw(), left_.D_raw(),
				right_.K_raw(), right_.D_raw(),
				left_.imageSize(), R_, T_, R1, R2, P1, P2, Q,
				cv::CALIB_ZERO_DISPARITY, 0, left_.imageSize());

		left_ = CameraModel(left_.name(), left_.imageSize(), left_.K_raw(), left_.D_raw(), R1, P1, left_.localTransform());
		right_ = CameraModel(right_.name(), right_.imageSize(), right_.K_raw(), right_.D_raw(), R2, P2, right_.localTransform());
	}
}

StereoCameraModel::StereoCameraModel(
		double fx,
		double fy,
		double cx,
		double cy,
		double baseline,
		const Transform & localTransform,
		const cv::Size & imageSize) :
				leftSuffix_("left"),
				rightSuffix_("right"),
				left_(fx, fy, cx, cy, localTransform, 0, imageSize),
				right_(fx, fy, cx, cy, localTransform, baseline*-fx, imageSize)
{
}

//minimal to be saved
StereoCameraModel::StereoCameraModel(
		const std::string & name,
		double fx,
		double fy,
		double cx,
		double cy,
		double baseline,
		const Transform & localTransform,
		const cv::Size & imageSize) :
				leftSuffix_("left"),
				rightSuffix_("right"),
				left_(name+"_"+getLeftSuffix(), fx, fy, cx, cy, localTransform, 0, imageSize),
				right_(name+"_"+getRightSuffix(), fx, fy, cx, cy, localTransform, baseline*-fx, imageSize),
				name_(name)
{
}

void StereoCameraModel::setName(const std::string & name, const std::string & leftSuffix, const std::string & rightSuffix)
{
	name_=name;
	leftSuffix_ = leftSuffix;
	rightSuffix_ = rightSuffix;
	left_.setName(name_+"_"+getLeftSuffix());
	right_.setName(name_+"_"+getRightSuffix());
}

bool StereoCameraModel::load(const std::string & directory, const std::string & cameraName, bool ignoreStereoTransform)
{
	name_ = cameraName;
	bool leftLoaded = left_.load(directory, cameraName+"_"+getLeftSuffix());
	bool rightLoaded = right_.load(directory, cameraName+"_"+getRightSuffix());
	if(leftLoaded && rightLoaded)
	{
		if(ignoreStereoTransform)
		{
			return true;
		}
		//load rotation, translation
		R_ = cv::Mat();
		T_ = cv::Mat();
		E_ = cv::Mat();
		F_ = cv::Mat();

		std::string filePath = directory+"/"+cameraName+"_pose.yaml";
		if(UFile::exists(filePath))
		{
			UINFO("Reading stereo calibration file \"%s\"", filePath.c_str());
			cv::FileStorage fs(filePath, cv::FileStorage::READ);

			cv::FileNode n;

			n = fs["camera_name"];
			if(n.type() != cv::FileNode::NONE)
			{
				name_ = (int)n;
			}
			else
			{
				UWARN("Missing \"camera_name\" field in \"%s\"", filePath.c_str());
			}

			// import from ROS calibration format
			n = fs["rotation_matrix"];
			if(n.type() != cv::FileNode::NONE)
			{
				int rows = (int)n["rows"];
				int cols = (int)n["cols"];
				std::vector<double> data;
				n["data"] >> data;
				UASSERT(rows*cols == (int)data.size());
				UASSERT(rows == 3 && cols == 3);
				R_ = cv::Mat(rows, cols, CV_64FC1, data.data()).clone();
			}
			else
			{
				UWARN("Missing \"rotation_matrix\" field in \"%s\"", filePath.c_str());
			}

			n = fs["translation_matrix"];
			if(n.type() != cv::FileNode::NONE)
			{
				int rows = (int)n["rows"];
				int cols = (int)n["cols"];
				std::vector<double> data;
				n["data"] >> data;
				UASSERT(rows*cols == (int)data.size());
				UASSERT(rows == 3 && cols == 1);
				T_ = cv::Mat(rows, cols, CV_64FC1, data.data()).clone();
			}
			else
			{
				UWARN("Missing \"translation_matrix\" field in \"%s\"", filePath.c_str());
			}

			n = fs["essential_matrix"];
			if(n.type() != cv::FileNode::NONE)
			{
				int rows = (int)n["rows"];
				int cols = (int)n["cols"];
				std::vector<double> data;
				n["data"] >> data;
				UASSERT(rows*cols == (int)data.size());
				UASSERT(rows == 3 && cols == 3);
				E_ = cv::Mat(rows, cols, CV_64FC1, data.data()).clone();
			}
			else
			{
				UWARN("Missing \"essential_matrix\" field in \"%s\"", filePath.c_str());
			}

			n = fs["fundamental_matrix"];
			if(n.type() != cv::FileNode::NONE)
			{
				int rows = (int)n["rows"];
				int cols = (int)n["cols"];
				std::vector<double> data;
				n["data"] >> data;
				UASSERT(rows*cols == (int)data.size());
				UASSERT(rows == 3 && cols == 3);
				F_ = cv::Mat(rows, cols, CV_64FC1, data.data()).clone();
			}
			else
			{
				UWARN("Missing \"fundamental_matrix\" field in \"%s\"", filePath.c_str());
			}

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
bool StereoCameraModel::save(const std::string & directory, bool ignoreStereoTransform) const
{
	if(left_.save(directory) && right_.save(directory))
	{
		if(ignoreStereoTransform)
		{
			return true;
		}
		return saveStereoTransform(directory);
	}
	return false;
}

bool StereoCameraModel::saveStereoTransform(const std::string & directory) const
{
	std::string filePath = directory+"/"+name_+"_pose.yaml";
	if(!filePath.empty() && (!R_.empty() && !T_.empty()))
	{
		UINFO("Saving stereo calibration to file \"%s\"", filePath.c_str());
		cv::FileStorage fs(filePath, cv::FileStorage::WRITE);

		// export in ROS calibration format

		if(!name_.empty())
		{
			fs << "camera_name" << name_;
		}

		if(!R_.empty())
		{
			fs << "rotation_matrix" << "{";
			fs << "rows" << R_.rows;
			fs << "cols" << R_.cols;
			fs << "data" << std::vector<double>((double*)R_.data, ((double*)R_.data)+(R_.rows*R_.cols));
			fs << "}";
		}

		if(!T_.empty())
		{
			fs << "translation_matrix" << "{";
			fs << "rows" << T_.rows;
			fs << "cols" << T_.cols;
			fs << "data" << std::vector<double>((double*)T_.data, ((double*)T_.data)+(T_.rows*T_.cols));
			fs << "}";
		}

		if(!E_.empty())
		{
			fs << "essential_matrix" << "{";
			fs << "rows" << E_.rows;
			fs << "cols" << E_.cols;
			fs << "data" << std::vector<double>((double*)E_.data, ((double*)E_.data)+(E_.rows*E_.cols));
			fs << "}";
		}

		if(!F_.empty())
		{
			fs << "fundamental_matrix" << "{";
			fs << "rows" << F_.rows;
			fs << "cols" << F_.cols;
			fs << "data" << std::vector<double>((double*)F_.data, ((double*)F_.data)+(F_.rows*F_.cols));
			fs << "}";
		}

		fs.release();

		return true;
	}
	else
	{
		UERROR("Failed saving stereo extrinsics (they are null).");
	}
	return false;
}

void StereoCameraModel::scale(double scale)
{
	left_ = left_.scaled(scale);
	right_ = right_.scaled(scale);
}

void StereoCameraModel::roi(const cv::Rect & roi)
{
	left_ = left_.roi(roi);
	right_ = right_.roi(roi);
}

float StereoCameraModel::computeDepth(float disparity) const
{
	//depth = baseline * f / (disparity + cx1-cx0);
	UASSERT(this->isValidForProjection());
	if(disparity == 0.0f)
	{
		return 0.0f;
	}
	return baseline() * left().fx() / (disparity + right().cx() - left().cx());
}

float StereoCameraModel::computeDisparity(float depth) const
{
	// disparity = (baseline * fx / depth) - (cx1-cx0);
	UASSERT(this->isValidForProjection());
	if(depth == 0.0f)
	{
		return 0.0f;
	}
	return baseline() * left().fx() / depth - right().cx() + left().cx();
}

float StereoCameraModel::computeDisparity(unsigned short depth) const
{
	// disparity = (baseline * fx / depth) - (cx1-cx0);
	UASSERT(this->isValidForProjection());
	if(depth == 0)
	{
		return 0.0f;
	}
	return baseline() * left().fx() / (float(depth)/1000.0f) - right().cx() + left().cx();
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
