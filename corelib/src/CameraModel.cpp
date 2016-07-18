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

#include <rtabmap/core/CameraModel.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UDirectory.h>
#include <rtabmap/utilite/UFile.h>
#include <rtabmap/utilite/UConversion.h>
#include <opencv2/imgproc/imgproc.hpp>

namespace rtabmap {

CameraModel::CameraModel()
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
	UASSERT(K_.empty() || (K_.rows == 3 && K_.cols == 3 && K_.type() == CV_64FC1));
	UASSERT(D_.empty() || (D_.rows == 1 && (D_.cols == 4 || D_.cols == 5 || D_.cols == 8) && D_.type() == CV_64FC1));
	UASSERT(R_.empty() || (R_.rows == 3 && R_.cols == 3 && R_.type() == CV_64FC1));
	UASSERT(P_.empty() || (P_.rows == 3 && P_.cols == 4 && P_.type() == CV_64FC1));
}

CameraModel::CameraModel(
		double fx,
		double fy,
		double cx,
		double cy,
		const Transform & localTransform,
		double Tx,
		const cv::Size & imageSize) :
		imageSize_(imageSize),
		K_(cv::Mat::eye(3, 3, CV_64FC1)),
		localTransform_(localTransform)
{
	UASSERT_MSG(fx > 0.0, uFormat("fx=%f", fx).c_str());
	UASSERT_MSG(fy > 0.0, uFormat("fy=%f", fy).c_str());
	UASSERT_MSG(cx >= 0.0, uFormat("cx=%f", cx).c_str());
	UASSERT_MSG(cy >= 0.0, uFormat("cy=%f", cy).c_str());
	UASSERT(!localTransform.isNull());
	if(Tx != 0.0)
	{
		P_ = cv::Mat::eye(3, 4, CV_64FC1),
		P_.at<double>(0,0) = fx;
		P_.at<double>(1,1) = fy;
		P_.at<double>(0,2) = cx;
		P_.at<double>(1,2) = cy;
		P_.at<double>(0,3) = Tx;
	}

	K_.at<double>(0,0) = fx;
	K_.at<double>(1,1) = fy;
	K_.at<double>(0,2) = cx;
	K_.at<double>(1,2) = cy;
}

CameraModel::CameraModel(
		const std::string & name,
		double fx,
		double fy,
		double cx,
		double cy,
		const Transform & localTransform,
		double Tx,
		const cv::Size & imageSize) :
		name_(name),
		imageSize_(imageSize),
		K_(cv::Mat::eye(3, 3, CV_64FC1)),
		localTransform_(localTransform)
{
	UASSERT_MSG(fx > 0.0, uFormat("fx=%f", fx).c_str());
	UASSERT_MSG(fy > 0.0, uFormat("fy=%f", fy).c_str());
	UASSERT_MSG(cx >= 0.0, uFormat("cx=%f", cx).c_str());
	UASSERT_MSG(cy >= 0.0, uFormat("cy=%f", cy).c_str());
	UASSERT(!localTransform.isNull());
	if(Tx != 0.0)
	{
		P_ = cv::Mat::eye(3, 4, CV_64FC1),
		P_.at<double>(0,0) = fx;
		P_.at<double>(1,1) = fy;
		P_.at<double>(0,2) = cx;
		P_.at<double>(1,2) = cy;
		P_.at<double>(0,3) = Tx;
	}

	K_.at<double>(0,0) = fx;
	K_.at<double>(1,1) = fy;
	K_.at<double>(0,2) = cx;
	K_.at<double>(1,2) = cy;
}

void CameraModel::initRectificationMap()
{
	UASSERT(imageSize_.height > 0 && imageSize_.width > 0);
	UASSERT(D_.rows == 1 && (D_.cols == 4 || D_.cols == 5 || D_.cols == 8));
	UASSERT(R_.rows == 3 && R_.cols == 3);
	UASSERT(P_.rows == 3 && P_.cols == 4);
	// init rectification map
	UINFO("Initialize rectify map");
	cv::initUndistortRectifyMap(K_, D_, R_, P_, imageSize_, CV_32FC1, mapX_, mapY_);
}

bool CameraModel::load(const std::string & directory, const std::string & cameraName)
{
	K_ = cv::Mat();
	D_ = cv::Mat();
	R_ = cv::Mat();
	P_ = cv::Mat();
	mapX_ = cv::Mat();
	mapY_ = cv::Mat();
	name_.clear();
	imageSize_ = cv::Size();

	std::string filePath = directory+"/"+cameraName+".yaml";
	if(UFile::exists(filePath))
	{
		try
		{
			UINFO("Reading calibration file \"%s\"", filePath.c_str());
			cv::FileStorage fs(filePath, cv::FileStorage::READ);

			cv::FileNode n,n2;

			n = fs["camera_name"];
			if(n.type() != cv::FileNode::NONE)
			{
				name_ = (int)n;
			}
			else
			{
				UWARN("Missing \"camera_name\" field in \"%s\"", filePath.c_str());
			}

			n = fs["image_width"];
			n2 = fs["image_height"];
			if(n.type() != cv::FileNode::NONE)
			{
				imageSize_.width = (int)fs["image_width"];
				imageSize_.height = (int)fs["image_height"];
			}
			else
			{
				UWARN("Missing \"image_width\" and/or \"image_height\" fields in \"%s\"", filePath.c_str());
			}

			// import from ROS calibration format
			n = fs["camera_matrix"];
			if(n.type() != cv::FileNode::NONE)
			{
				int rows = (int)n["rows"];
				int cols = (int)n["cols"];
				std::vector<double> data;
				n["data"] >> data;
				UASSERT(rows*cols == (int)data.size());
				UASSERT(rows == 3 && cols == 3);
				K_ = cv::Mat(rows, cols, CV_64FC1, data.data()).clone();
			}
			else
			{
				UWARN("Missing \"camera_matrix\" field in \"%s\"", filePath.c_str());
			}

			n = fs["distortion_coefficients"];
			if(n.type() != cv::FileNode::NONE)
			{
				int rows = (int)n["rows"];
				int cols = (int)n["cols"];
				std::vector<double> data;
				n["data"] >> data;
				UASSERT(rows*cols == (int)data.size());
				UASSERT(rows == 1 && (cols == 4 || cols == 5 || cols == 8));
				D_ = cv::Mat(rows, cols, CV_64FC1, data.data()).clone();
			}
			else
			{
				UWARN("Missing \"distorsion_coefficients\" field in \"%s\"", filePath.c_str());
			}

			n = fs["rectification_matrix"];
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
				UWARN("Missing \"rectification_matrix\" field in \"%s\"", filePath.c_str());
			}

			n = fs["projection_matrix"];
			if(n.type() != cv::FileNode::NONE)
			{
				int rows = (int)n["rows"];
				int cols = (int)n["cols"];
				std::vector<double> data;
				n["data"] >> data;
				UASSERT(rows*cols == (int)data.size());
				UASSERT(rows == 3 && cols == 4);
				P_ = cv::Mat(rows, cols, CV_64FC1, data.data()).clone();
			}
			else
			{
				UWARN("Missing \"projection_matrix\" field in \"%s\"", filePath.c_str());
			}

			fs.release();

			if(isValidForRectification())
			{
				initRectificationMap();
			}

			return true;
		}
		catch(const cv::Exception & e)
		{
			UERROR("Error reading calibration file \"%s\": %s", filePath.c_str(), e.what());
		}
	}
	else
	{
		UWARN("Could not load calibration file \"%s\".", filePath.c_str());
	}
	return false;
}

bool CameraModel::save(const std::string & directory) const
{
	std::string filePath = directory+"/"+name_+".yaml";
	if(!filePath.empty() && (!K_.empty() || !D_.empty() || !R_.empty() || !P_.empty()))
	{
		UINFO("Saving calibration to file \"%s\"", filePath.c_str());
		cv::FileStorage fs(filePath, cv::FileStorage::WRITE);

		// export in ROS calibration format

		if(!name_.empty())
		{
			fs << "camera_name" << name_;
		}
		if(imageSize_.width>0 && imageSize_.height>0)
		{
			fs << "image_width" << imageSize_.width;
			fs << "image_height" << imageSize_.height;
		}

		if(!K_.empty())
		{
			fs << "camera_matrix" << "{";
			fs << "rows" << K_.rows;
			fs << "cols" << K_.cols;
			fs << "data" << std::vector<double>((double*)K_.data, ((double*)K_.data)+(K_.rows*K_.cols));
			fs << "}";
		}

		if(!D_.empty())
		{
			fs << "distortion_coefficients" << "{";
			fs << "rows" << D_.rows;
			fs << "cols" << D_.cols;
			fs << "data" << std::vector<double>((double*)D_.data, ((double*)D_.data)+(D_.rows*D_.cols));
			fs << "}";

			// compaibility with ROS
			if(D_.cols > 5)
			{
				fs << "distortion_model" << "rational_polynomial";
			}
			else
			{
				fs << "distortion_model" << "plumb_bob";
			}
		}

		if(!R_.empty())
		{
			fs << "rectification_matrix" << "{";
			fs << "rows" << R_.rows;
			fs << "cols" << R_.cols;
			fs << "data" << std::vector<double>((double*)R_.data, ((double*)R_.data)+(R_.rows*R_.cols));
			fs << "}";
		}

		if(!P_.empty())
		{
			fs << "projection_matrix" << "{";
			fs << "rows" << P_.rows;
			fs << "cols" << P_.cols;
			fs << "data" << std::vector<double>((double*)P_.data, ((double*)P_.data)+(P_.rows*P_.cols));
			fs << "}";
		}

		fs.release();

		return true;
	}
	else
	{
		UERROR("Cannot save calibration to \"%s\" because it is empty.", filePath.c_str());
	}
	return false;
}

CameraModel CameraModel::scaled(double scale) const
{
	CameraModel scaledModel = *this;
	UASSERT(scale > 0.0);
	if(this->isValidForProjection())
	{
		// has only effect on K and P
		cv::Mat K;
		if(!K_.empty())
		{
			K = K_.clone();
			K.at<double>(0,0) *= scale;
			K.at<double>(1,1) *= scale;
			K.at<double>(0,2) *= scale;
			K.at<double>(1,2) *= scale;
		}

		cv::Mat P;
		if(!P_.empty())
		{
			P = P_.clone();
			P.at<double>(0,0) *= scale;
			P.at<double>(1,1) *= scale;
			P.at<double>(0,2) *= scale;
			P.at<double>(1,2) *= scale;
			P.at<double>(0,3) *= scale;
			P.at<double>(1,3) *= scale;
		}
		scaledModel = CameraModel(name_, cv::Size(double(imageSize_.width)*scale, double(imageSize_.height)*scale), K, D_, R_, P, localTransform_);
	}
	else
	{
		UWARN("Trying to scale a camera model not valid! Ignoring scaling...");
	}
	return scaledModel;
}

double CameraModel::horizontalFOV() const
{
	if(imageWidth() > 0 && fx() > 0.0)
	{
		return atan((double(imageWidth())/2.0)/fx())*2.0*180.0/CV_PI;
	}
	return 0.0;
}

double CameraModel::verticalFOV() const
{
	if(imageHeight() > 0 && fy() > 0.0)
	{
		return atan((double(imageHeight())/2.0)/fy())*2.0*180.0/CV_PI;
	}
	return 0.0;
}

cv::Mat CameraModel::rectifyImage(const cv::Mat & raw, int interpolation) const
{
	UDEBUG("");
	if(!mapX_.empty() && !mapY_.empty())
	{
		cv::Mat rectified;
		cv::remap(raw, rectified, mapX_, mapY_, interpolation);
		return rectified;
	}
	else
	{
		UERROR("Cannot rectify image because the rectify map is not initialized.");
		return raw.clone();
	}
}

//inspired from https://github.com/code-iai/iai_kinect2/blob/master/depth_registration/src/depth_registration_cpu.cpp
cv::Mat CameraModel::rectifyDepth(const cv::Mat & raw) const
{
	UDEBUG("");
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
		UERROR("Cannot rectify image because the rectify map is not initialized.");
		return raw.clone();
	}
}

} /* namespace rtabmap */
