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
#include <rtabmap/core/Version.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UDirectory.h>
#include <rtabmap/utilite/UFile.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UMath.h>
#include <rtabmap/utilite/UStl.h>
#include <opencv2/imgproc/imgproc.hpp>

namespace rtabmap {

CameraModel::CameraModel() :
		localTransform_(opticalRotation())
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
	UASSERT(D_.empty() || (D_.rows == 1 && (D_.cols == 4 || D_.cols == 5 || D_.cols == 6 || D_.cols == 8) && D_.type() == CV_64FC1));
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
	UASSERT_MSG(cx >= 0.0 && imageSize.width>=0, uFormat("cx=%f imageSize.width=%d", cx, imageSize.width).c_str());
	UASSERT_MSG(cy >= 0.0 && imageSize.height>=0, uFormat("cy=%f imageSize.height=%d", cy, imageSize.height).c_str());
	UASSERT(!localTransform.isNull());

	if(cx==0.0 && imageSize.width > 0)
	{
		cx = double(imageSize.width)/2.0-0.5;
	}
	if(cy==0.0 && imageSize.height > 0)
	{
		cy = double(imageSize.height)/2.0-0.5;
	}

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
	UASSERT_MSG(cx >= 0.0 && imageSize.width>=0, uFormat("cx=%f imageSize.width=%d", cx, imageSize.width).c_str());
	UASSERT_MSG(cy >= 0.0 && imageSize.height>=0, uFormat("cy=%f imageSize.height=%d", cy, imageSize.height).c_str());
	UASSERT(!localTransform.isNull());

	if(cx==0.0 && imageSize.width > 0)
	{
		cx = double(imageSize.width)/2.0-0.5;
	}
	if(cy==0.0 && imageSize.height > 0)
	{
		cy = double(imageSize.height)/2.0-0.5;
	}

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

bool CameraModel::initRectificationMap()
{
	UASSERT(imageSize_.height > 0 && imageSize_.width > 0);
	UASSERT(D_.rows == 1 && (D_.cols == 4 || D_.cols == 5 || D_.cols == 6 || D_.cols == 8));
	UASSERT(R_.rows == 3 && R_.cols == 3);
	UASSERT(P_.rows == 3 && P_.cols == 4);
	// init rectification map
	UINFO("Initialize rectify map");
	if(D_.cols == 6)
	{
#if CV_MAJOR_VERSION > 2 or (CV_MAJOR_VERSION == 2 and (CV_MINOR_VERSION >4 or (CV_MINOR_VERSION == 4 and CV_SUBMINOR_VERSION >=10)))
		// Equidistant / FishEye
		// get only k parameters (k1,k2,p1,p2,k3,k4)
		cv::Mat D(1, 4, CV_64FC1);
		D.at<double>(0,0) = D_.at<double>(0,0);
		D.at<double>(0,1) = D_.at<double>(0,1);
		D.at<double>(0,2) = D_.at<double>(0,4);
		D.at<double>(0,3) = D_.at<double>(0,5);
		cv::fisheye::initUndistortRectifyMap(K_, D, R_, P_, imageSize_, CV_32FC1, mapX_, mapY_);
	}
	else
#else
		UWARN("Too old opencv version (%d,%d,%d) to support fisheye model (min 2.4.10 required)!",
				CV_MAJOR_VERSION, CV_MINOR_VERSION, CV_SUBMINOR_VERSION);
	}
#endif
	{
		// RadialTangential
		cv::initUndistortRectifyMap(K_, D_, R_, P_, imageSize_, CV_32FC1, mapX_, mapY_);
	}
	return isRectificationMapInitialized();
}

void CameraModel::setImageSize(const cv::Size & size)
{
	UASSERT((size.height > 0 && size.width > 0) || (size.height == 0 && size.width == 0));
	imageSize_ = size;
	double ncx = cx();
	double ncy = cy();
	if(ncx==0.0 && imageSize_.width > 0)
	{
		ncx = double(imageSize_.width)/2.0-0.5;
	}
	if(ncy==0.0 && imageSize_.height > 0)
	{
		ncy = double(imageSize_.height)/2.0-0.5;
	}
	if(!P_.empty())
	{
		P_.at<double>(0,2) = ncx;
		P_.at<double>(1,2) = ncy;
	}
	if(!K_.empty())
	{
		K_.at<double>(0,2) = ncx;
		K_.at<double>(1,2) = ncy;
	}
}

bool CameraModel::load(const std::string & filePath)
{
	K_ = cv::Mat();
	D_ = cv::Mat();
	R_ = cv::Mat();
	P_ = cv::Mat();
	mapX_ = cv::Mat();
	mapY_ = cv::Mat();
	name_.clear();
	imageSize_ = cv::Size();

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
				name_ = (std::string)n;
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

			n = fs["distortion_model"];
			if(n.type() != cv::FileNode::NONE)
			{
				std::string distortionModel = (std::string)n;
				if(D_.cols>=4 &&
				  (uStrContains(distortionModel, "fisheye") ||
				   uStrContains(distortionModel, "equidistant")))
				{
					cv::Mat D = cv::Mat::zeros(1,6,CV_64FC1);
					D.at<double>(0,0) = D_.at<double>(0,0);
					D.at<double>(0,1) = D_.at<double>(0,1);
					D.at<double>(0,4) = D_.at<double>(0,2);
					D.at<double>(0,5) = D_.at<double>(0,3);
					D_ = D;
				}
			}
			else
			{
				UWARN("Missing \"distortion_model\" field in \"%s\"", filePath.c_str());
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

			n = fs["local_transform"];
			if(n.type() != cv::FileNode::NONE)
			{
				int rows = (int)n["rows"];
				int cols = (int)n["cols"];
				std::vector<float> data;
				n["data"] >> data;
				UASSERT(rows*cols == (int)data.size());
				UASSERT(rows == 3 && cols == 4);
				localTransform_ = Transform(
						data[0], data[1], data[2], data[3],
						data[4], data[5], data[6], data[7],
						data[8], data[9], data[10], data[11]);
			}
			else
			{
				UWARN("Missing \"local_transform\" field in \"%s\"", filePath.c_str());
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
			UERROR("Error reading calibration file \"%s\": %s (Make sure the first line of the yaml file is \"%YAML:1.0\")", filePath.c_str(), e.what());
		}
	}
	else
	{
		UWARN("Could not load calibration file \"%s\".", filePath.c_str());
	}
	return false;
}

bool CameraModel::load(const std::string & directory, const std::string & cameraName)
{
	return load(directory+"/"+cameraName+".yaml");
}

bool CameraModel::save(const std::string & directory) const
{
	if(name_.empty())
	{
		UWARN("Camera name is empty, will use general \"camera\" as name.");
	}
	std::string filePath = directory+"/"+(name_.empty()?"camera":name_)+".yaml";
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
			cv::Mat D = D_;
			if(D_.cols == 6)
			{
				D = cv::Mat(1,4,CV_64FC1);
				D.at<double>(0,0) = D_.at<double>(0,0);
				D.at<double>(0,1) = D_.at<double>(0,1);
				D.at<double>(0,2) = D_.at<double>(0,4);
				D.at<double>(0,3) = D_.at<double>(0,5);
			}
			fs << "distortion_coefficients" << "{";
			fs << "rows" << D.rows;
			fs << "cols" << D.cols;
			fs << "data" << std::vector<double>((double*)D.data, ((double*)D.data)+(D.rows*D.cols));
			fs << "}";

			// compaibility with ROS
			if(D_.cols == 6)
			{
				fs << "distortion_model" << "equidistant"; // equidistant, fisheye
			}
			else if(D.cols > 5)
			{
				fs << "distortion_model" << "rational_polynomial"; // rad tan
			}
			else
			{
				fs << "distortion_model" << "plumb_bob"; // rad tan
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

		if(!localTransform_.isNull())
		{
			fs << "local_transform" << "{";
			fs << "rows" << 3;
			fs << "cols" << 4;
			fs << "data" << std::vector<float>((float*)localTransform_.data(), ((float*)localTransform_.data())+12);
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

std::vector<unsigned char> CameraModel::serialize() const
{
	const int headerSize = 11;
	int header[headerSize] = {
			RTABMAP_VERSION_MAJOR, RTABMAP_VERSION_MINOR, RTABMAP_VERSION_PATCH, // 0,1,2
			0, //mono                                                            // 3,
			imageSize_.width, imageSize_.height,                                 // 4,5
			(int)K_.total(), (int)D_.total(), (int)R_.total(), (int)P_.total(),  // 6,7,8,9
			localTransform_.isNull()?0:localTransform_.size()};                  // 10
	UDEBUG("Header: %d %d %d %d %d %d %d %d %d %d %d", header[0],header[1],header[2],header[3],header[4],header[5],header[6],header[7],header[8],header[9],header[10]);
	std::vector<unsigned char> data(
			sizeof(int)*headerSize +
			sizeof(double)*(K_.total()+D_.total()+R_.total()+P_.total()) +
			(localTransform_.isNull()?0:sizeof(float)*localTransform_.size()));
	memcpy(data.data(), header, sizeof(int)*headerSize);
	int index = sizeof(int)*headerSize;
	if(!K_.empty())
	{
		memcpy(data.data()+index, K_.data, sizeof(double)*(K_.total()));
		index+=sizeof(double)*(K_.total());
	}
	if(!D_.empty())
	{
		memcpy(data.data()+index, D_.data, sizeof(double)*(D_.total()));
		index+=sizeof(double)*(D_.total());
	}
	if(!R_.empty())
	{
		memcpy(data.data()+index, R_.data, sizeof(double)*(R_.total()));
		index+=sizeof(double)*(R_.total());
	}
	if(!P_.empty())
	{
		memcpy(data.data()+index, P_.data, sizeof(double)*(P_.total()));
		index+=sizeof(double)*(P_.total());
	}
	if(!localTransform_.isNull())
	{
		memcpy(data.data()+index, localTransform_.data(), sizeof(float)*(localTransform_.size()));
		index+=sizeof(float)*(localTransform_.size());
	}
	return data;
}

unsigned int CameraModel::deserialize(const std::vector<unsigned char>& data)
{
	return deserialize(data.data(), data.size());
}
unsigned int CameraModel::deserialize(const unsigned char * data, unsigned int dataSize)
{
	*this = CameraModel();
	int headerSize = 11;
	if(dataSize >= sizeof(int)*headerSize)
	{
		UASSERT(data != 0);
		const int * header = (const int *)data;
		int type = header[3];
		if(type == 0)
		{
			imageSize_.width = header[4];
			imageSize_.height = header[5];
			int iK = 6;
			int iD = 7;
			int iR = 8;
			int iP = 9;
			int iL = 10;
			UDEBUG("Header: %d %d %d %d %d %d %d %d %d %d %d", header[0],header[1],header[2],header[3],header[4],header[5],header[6],header[7],header[8],header[9],header[10]);
			unsigned int requiredDataSize = sizeof(int)*headerSize +
					sizeof(double)*(header[iK]+header[iD]+header[iR]+header[iP]) +
					sizeof(float)*header[iL];
			UASSERT_MSG(dataSize >= requiredDataSize,
					uFormat("dataSize=%d != required=%d (header: version %d.%d.%d %dx%d type=%d K=%d D=%d R=%d P=%d L=%d)",
							dataSize,
							requiredDataSize,
							header[0], header[1], header[2], header[4], header[5], header[3],
							header[iK], header[iD], header[iR],header[iP], header[iL]).c_str());
			unsigned int index = sizeof(int)*headerSize;
			if(header[iK] != 0)
			{
				UASSERT(header[iK] == 9);
				K_ = cv::Mat(3, 3, CV_64FC1, (void*)(data+index)).clone();
				index+=sizeof(double)*(K_.total());
			}
			if(header[iD] != 0)
			{
				D_ = cv::Mat(1, header[iD], CV_64FC1, (void*)(data+index)).clone();
				index+=sizeof(double)*(D_.total());
			}
			if(header[iR] != 0)
			{
				UASSERT(header[iR] == 9);
				R_ = cv::Mat(3, 3, CV_64FC1, (void*)(data+index)).clone();
				index+=sizeof(double)*(R_.total());
			}
			if(header[iP] != 0)
			{
				UASSERT(header[iP] == 12);
				P_ = cv::Mat(3, 4, CV_64FC1, (void*)(data+index)).clone();
				index+=sizeof(double)*(P_.total());
			}
			if(header[iL] != 0)
			{
				UASSERT(header[iL] == 12);
				memcpy(localTransform_.data(), data+index, sizeof(float)*localTransform_.size());
				index+=sizeof(float)*localTransform_.size();
			}
			UASSERT(index <= dataSize);
			return index;
		}
		else
		{
			UERROR("Serialized calibration is not mono (type=%d), use the appropriate class matching the type to deserialize.", type);
		}
	}
	UERROR("Wrong serialized calibration data format detected (size in bytes=%d)! Cannot deserialize the data.", (int)dataSize);
	return 0;
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

CameraModel CameraModel::roi(const cv::Rect & roi) const
{
	CameraModel roiModel = *this;
	if(this->isValidForProjection())
	{
		// has only effect on cx and cy
		cv::Mat K;
		if(!K_.empty())
		{
			K = K_.clone();
			K.at<double>(0,2) -= roi.x;
			K.at<double>(1,2) -= roi.y;
		}

		cv::Mat P;
		if(!P_.empty())
		{
			P = P_.clone();
			P.at<double>(0,2) -= roi.x;
			P.at<double>(1,2) -= roi.y;
		}
		roiModel = CameraModel(name_, roi.size(), K, D_, R_, P, localTransform_);
	}
	else
	{
		UWARN("Trying to extract roi from a camera model not valid! Ignoring roi...");
	}
	return roiModel;
}

double CameraModel::fovX() const
{
	return imageSize_.width>0 && fx()>0?2.0*atan(imageSize_.width/(fx()*2.0)):0.0;
}
double CameraModel::fovY() const
{
	return imageSize_.height>0 && fy()>0?2.0*atan(imageSize_.height/(fy()*2.0)):0.0;
}

double CameraModel::horizontalFOV() const
{
	return fovX()*180.0/CV_PI;
}

double CameraModel::verticalFOV() const
{
	return fovY()*180.0/CV_PI;
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
									(pLT * (1.f - a) + pRT * a) * (1.f - c) +
									(pLB * (1.f - a) + pRB * a) * c;
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

// resulting 3D point is in /camera_link frame
void CameraModel::project(float u, float v, float depth, float & x, float & y, float & z) const
{
	if(depth > 0.0f)
	{
		// Fill in XYZ
		x = (u - cx()) * depth / fx();
		y = (v - cy()) * depth / fy();
		z = depth;
	}
	else
	{
		x = y = z = std::numeric_limits<float>::quiet_NaN();
	}
}
// 3D point is in /camera_link frame
void CameraModel::reproject(float x, float y, float z, float & u, float & v) const
{
	UASSERT(z!=0.0f);
	float invZ = 1.0f/z;
	u = (fx()*x)*invZ + cx();
	v = (fy()*y)*invZ + cy();
}
void CameraModel::reproject(float x, float y, float z, int & u, int & v) const
{
	UASSERT(z!=0.0f);
	float invZ = 1.0f/z;
	u = (fx()*x)*invZ + cx();
	v = (fy()*y)*invZ + cy();
}

bool CameraModel::inFrame(int u, int v) const
{
	return uIsInBounds(u, 0, imageWidth()) && uIsInBounds(v, 0, imageHeight());
}

std::ostream& operator<<(std::ostream& os, const CameraModel& model)
{
	os << "Name: " << model.name().c_str() << std::endl
	   << "Size: " << model.imageWidth() << "x" << model.imageHeight() << std::endl
	   << "K= " << model.K_raw() << std::endl
	   << "D= " << model.D_raw() << std::endl
	   << "R= " << model.R() << std::endl
	   << "P= " << model.P() << std::endl
	   << "LocalTransform= " << model.localTransform();
	return os;
}

} /* namespace rtabmap */
