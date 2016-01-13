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

#include "rtabmap/core/CameraRGB.h"
#include "rtabmap/core/Graph.h"

#include <rtabmap/utilite/UEventsManager.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UFile.h>
#include <rtabmap/utilite/UDirectory.h>
#include <rtabmap/utilite/UTimer.h>

#include <opencv2/imgproc/imgproc.hpp>

#include <rtabmap/core/util3d.h>
#include <rtabmap/core/util3d_filtering.h>
#include <rtabmap/core/util3d_surface.h>

#include <iostream>
#include <cmath>

namespace rtabmap
{

/////////////////////////
// CameraImages
/////////////////////////
CameraImages::CameraImages() :
		_startAt(0),
		_refreshDir(false),
		_rectifyImages(false),
		_isDepth(false),
		_depthScaleFactor(1.0f),
		_count(0),
		_dir(0),
		_countScan(0),
		_scanDir(0),
		_scanMaxPts(0),
		_scanDownsampleStep(1),
		_scanVoxelSize(0.0f),
		_scanNormalsK(0),
		_depthFromScan(false),
		_filenamesAreTimestamps(false),
		_groundTruthFormat(0)
	{}
CameraImages::CameraImages(const std::string & path,
					 float imageRate,
					 const Transform & localTransform) :
	Camera(imageRate, localTransform),
	_path(path),
	_startAt(0),
	_refreshDir(false),
	_rectifyImages(false),
	_isDepth(false),
	_depthScaleFactor(1.0f),
	_count(0),
	_dir(0),
	_countScan(0),
	_scanDir(0),
	_scanMaxPts(0),
	_scanDownsampleStep(1),
	_scanVoxelSize(0.0f),
	_scanNormalsK(0),
	_depthFromScan(false),
	_filenamesAreTimestamps(false),
	_groundTruthFormat(0)
{

}

CameraImages::~CameraImages()
{
	UDEBUG("");
	if(_dir)
	{
		delete _dir;
	}
	if(_scanDir)
	{
		delete _scanDir;
	}
}

bool CameraImages::init(const std::string & calibrationFolder, const std::string & cameraName)
{
	_lastFileName.clear();
	_lastScanFileName.clear();
	_count = 0;
	_countScan = 0;

	UDEBUG("");
	if(_dir)
	{
		_dir->setPath(_path, "jpg ppm png bmp pnm tiff");
	}
	else
	{
		_dir = new UDirectory(_path, "jpg ppm png bmp pnm tiff");
	}
	if(_path[_path.size()-1] != '\\' && _path[_path.size()-1] != '/')
	{
		_path.append("/");
	}
	if(!_dir->isValid())
	{
		ULOGGER_ERROR("Directory path is not valid \"%s\"", _path.c_str());
	}
	else if(_dir->getFileNames().size() == 0)
	{
		UWARN("Directory is empty \"%s\"", _path.c_str());
	}
	else
	{
		UINFO("path=%s images=%d", _path.c_str(), (int)this->imagesCount());
	}

	// check for scan directory
	if(_scanDir)
	{
		delete _scanDir;
		_scanDir = 0;
	}
	if(!_scanPath.empty())
	{
		UINFO("scan path=%s", _scanPath.c_str());
		_scanDir = new UDirectory(_scanPath, "pcd bin ply"); // "bin" is for KITTI format
		if(_scanPath[_scanPath.size()-1] != '\\' && _scanPath[_scanPath.size()-1] != '/')
		{
			_scanPath.append("/");
		}
		if(!_scanDir->isValid())
		{
			UERROR("Scan directory path is not valid \"%s\"", _scanPath.c_str());
			delete _scanDir;
			_scanDir = 0;
		}
		else if(_scanDir->getFileNames().size() == 0)
		{
			UWARN("Scan directory is empty \"%s\"", _scanPath.c_str());
			delete _scanDir;
			_scanDir = 0;
		}
		else if(_scanDir->getFileNames().size() != _dir->getFileNames().size())
		{
			UERROR("Scan and image directories should be the same size \"%s\"(%d) vs \"%s\"(%d)",
					_scanPath.c_str(),
					(int)_scanDir->getFileNames().size(),
					_path.c_str(),
					(int)_dir->getFileNames().size());
			delete _scanDir;
			_scanDir = 0;
		}
		else
		{
			UINFO("path=%s scans=%d", _scanPath.c_str(), (int)this->imagesCount());
		}
	}

	// look for calibration files
	UINFO("calibration folder=%s name=%s", calibrationFolder.c_str(), cameraName.c_str());
	if(!calibrationFolder.empty() && !cameraName.empty())
	{
		if(!_model.load(calibrationFolder, cameraName))
		{
			UWARN("Missing calibration files for camera \"%s\" in \"%s\" folder, you should calibrate the camera!",
					cameraName.c_str(), calibrationFolder.c_str());
		}
		else
		{
			UINFO("Camera parameters: fx=%f fy=%f cx=%f cy=%f",
					_model.fx(),
					_model.fy(),
					_model.cx(),
					_model.cy());
		}
	}
	_model.setName(cameraName);

	_model.setLocalTransform(this->getLocalTransform());
	if(_rectifyImages && !_model.isValid())
	{
		UERROR("Parameter \"rectifyImages\" is set, but no camera model is loaded or valid.");
		return false;
	}

	bool success = _dir->isValid();
	stamps_.clear();
	groundTruth_.clear();
	if(success)
	{
		if(_filenamesAreTimestamps)
		{
			const std::list<std::string> & filenames = _dir->getFileNames();
			for(std::list<std::string>::const_iterator iter=filenames.begin(); iter!=filenames.end(); ++iter)
			{
				// format is text_12234456.12334_text.png
				std::list<std::string> list = uSplit(*iter, '.');
				if(list.size() == 3)
				{
					list.pop_back(); // remove extension
					std::string decimals = uSplitNumChar(list.back()).front();
					list.pop_back();
					std::string sec = uSplitNumChar(list.back()).back();
					double stamp = uStr2Double(sec + "." + decimals);
					if(stamp > 0.0)
					{
						stamps_.push_back(stamp);
					}
					else
					{
						UERROR("Conversion filename to timestamp failed! (filename=%s)", iter->c_str());
					}
				}
			}
			if(stamps_.size() != this->imagesCount())
			{
				UERROR("The stamps count is not the same as the images (%d vs %d)! "
					   "Converting filenames to timestamps is activated.",
						(int)stamps_.size(), this->imagesCount());
				stamps_.clear();
				success = false;
			}
		}
		else if(timestampsPath_.size())
		{
			FILE * file = 0;
#ifdef _MSC_VER
			fopen_s(&file, timestampsPath_.c_str(), "r");
#else
			file = fopen(timestampsPath_.c_str(), "r");
#endif
			if(file)
			{
				char line[16];
				while ( fgets (line , 16 , file) != NULL )
				{
					stamps_.push_back(uStr2Double(uReplaceChar(line, '\n', 0)));
				}
				fclose(file);
			}
			if(stamps_.size() != this->imagesCount())
			{
				UERROR("The stamps count is not the same as the images (%d vs %d)! Please remove "
						"the timestamps file path if you don't want to use them (current file path=%s).",
						(int)stamps_.size(), this->imagesCount(), timestampsPath_.c_str());
				stamps_.clear();
				success = false;
			}
		}

		if(groundTruthPath_.size())
		{
			std::map<int, Transform> poses;
			std::map<int, double> stamps;
			if(!graph::importPoses(groundTruthPath_, _groundTruthFormat, poses, 0, &stamps))
			{
				UERROR("Cannot read ground truth file \"%s\".", groundTruthPath_.c_str());
				success = false;
			}
			else if(_groundTruthFormat != 1 && poses.size() != this->imagesCount())
			{
				UERROR("The ground truth count is not the same as the images (%d vs %d)! Please remove "
						"the ground truth file path if you don't want to use it (current file path=%s).",
						(int)poses.size(), this->imagesCount(), groundTruthPath_.c_str());
				success = false;
			}
			else if(_groundTruthFormat == 1 && stamps_.size() == 0)
			{
				UERROR("When using rgbd-slam format for ground truth, images must have timestamps!");
				success = false;
			}
			else if(_groundTruthFormat == 1)
			{
				UDEBUG("");
				//Match ground truth values with images
				groundTruth_.clear();
				std::map<double, int> stampsToIds;
				for(std::map<int, double>::iterator iter=stamps.begin(); iter!=stamps.end(); ++iter)
				{
					stampsToIds.insert(std::make_pair(iter->second, iter->first));
				}
				std::vector<double> values = uValues(stamps);

				for(std::list<double>::iterator ster=stamps_.begin(); ster!=stamps_.end(); ++ster)
				{
					Transform pose; // null transform
					std::map<double, int>::iterator endIter = stampsToIds.lower_bound(*ster);
					if(endIter != stampsToIds.end())
					{
						if(endIter->first == *ster)
						{
							pose = poses.at(endIter->second);
						}
						else if(endIter != stampsToIds.begin())
						{
							//interpolate
							std::map<double, int>::iterator beginIter = endIter;
							--beginIter;
							double stampBeg = beginIter->first;
							double stampEnd = endIter->first;
							UASSERT(stampEnd > stampBeg && *ster>stampBeg && *ster < stampEnd);
							float t = (*ster - stampBeg) / (stampEnd-stampBeg);
							Transform & ta = poses.at(beginIter->second);
							Transform & tb = poses.at(endIter->second);
							if(!ta.isNull() && !tb.isNull())
							{
								pose = ta.interpolate(t, tb);
							}
						}
					}
					if(pose.isNull())
					{
						UWARN("Ground truth pose not found for stamp %f", *ster);
					}
					groundTruth_.push_back(pose);
				}
			}
			else
			{
				UDEBUG("");
				groundTruth_ = uValuesList(poses);
			}
			UASSERT_MSG(groundTruth_.size() == stamps_.size(), uFormat("%d vs %d", (int)groundTruth_.size(), (int)stamps_.size()).c_str());
		}
	}

	return success;
}

bool CameraImages::isCalibrated() const
{
	return _model.isValid();
}

std::string CameraImages::getSerial() const
{
	return _model.name();
}

unsigned int CameraImages::imagesCount() const
{
	if(_dir)
	{
		return (unsigned int)_dir->getFileNames().size();
	}
	return 0;
}

std::vector<std::string> CameraImages::filenames() const
{
	if(_dir)
	{
		return uListToVector(_dir->getFileNames());
	}
	return std::vector<std::string>();
}

SensorData CameraImages::captureImage()
{
	cv::Mat img;
	cv::Mat scan;
	double stamp = UTimer::now();
	Transform groundTruthPose;
	cv::Mat depthFromScan;
	UDEBUG("");
	if(_dir->isValid())
	{
		if(_refreshDir)
		{
			_dir->update();
			if(_scanDir)
			{
				_scanDir->update();
			}
		}
		std::string imageFilePath;
		std::string scanFilePath;
		if(_startAt < 0)
		{
			const std::list<std::string> & fileNames = _dir->getFileNames();
			if(fileNames.size())
			{
				if(_lastFileName.empty() || uStrNumCmp(_lastFileName,*fileNames.rbegin()) < 0)
				{
					_lastFileName = *fileNames.rbegin();
					imageFilePath = _path + _lastFileName;
				}
			}
			if(_scanDir)
			{
				const std::list<std::string> & scanFileNames = _scanDir->getFileNames();
				if(scanFileNames.size())
				{
					if(_lastScanFileName.empty() || uStrNumCmp(_lastScanFileName,*scanFileNames.rbegin()) < 0)
					{
						_lastScanFileName = *scanFileNames.rbegin();
						scanFilePath = _scanPath + _lastScanFileName;
					}
				}
			}
		}
		else
		{
			std::string fileName;
			fileName = _dir->getNextFileName();
			if(!fileName.empty())
			{
				imageFilePath = _path + fileName;
				while(_count++ < _startAt && (fileName = _dir->getNextFileName()).size())
				{
					imageFilePath = _path + fileName;
				}
			}
			if(_scanDir)
			{
				fileName = _scanDir->getNextFileName();
				if(!fileName.empty())
				{
					scanFilePath = _scanPath + fileName;
					while(++_countScan < _startAt && (fileName = _scanDir->getNextFileName()).size())
					{
						scanFilePath = _scanPath + fileName;
					}
				}
			}
		}

		if(stamps_.size())
		{
			stamp = stamps_.front();
			stamps_.pop_front();
			if(groundTruth_.size())
			{
				groundTruthPose = groundTruth_.front();
				groundTruth_.pop_front();
			}
		}

		if(!imageFilePath.empty())
		{
			ULOGGER_DEBUG("Loading image : %s", imageFilePath.c_str());

#if CV_MAJOR_VERSION >2 || (CV_MAJOR_VERSION >=2 && CV_MINOR_VERSION >=4)
			img = cv::imread(imageFilePath.c_str(), cv::IMREAD_UNCHANGED);
#else
			img = cv::imread(imageFilePath.c_str(), -1);
#endif
			UDEBUG("width=%d, height=%d, channels=%d, elementSize=%d, total=%d",
					img.cols, img.rows, img.channels(), img.elemSize(), img.total());

			if(_isDepth)
			{
				if(img.type() != CV_16UC1 && img.type() != CV_32FC1)
				{
					UERROR("Depth is on and the loaded image has not a format supported (file = \"%s\"). "
							"Formats supported are 16 bits 1 channel and 32 bits 1 channel.",
							imageFilePath.c_str());
					img = cv::Mat();
				}

				if(_depthScaleFactor > 1.0f)
				{
					img /= _depthScaleFactor;
				}
			}
			else
			{
#if CV_MAJOR_VERSION < 3
				// FIXME : it seems that some png are incorrectly loaded with opencv c++ interface, where c interface works...
				if(img.depth() != CV_8U)
				{
					// The depth should be 8U
					UWARN("Cannot read the image correctly, falling back to old OpenCV C interface...");
					IplImage * i = cvLoadImage(imageFilePath.c_str());
					img = cv::Mat(i, true);
					cvReleaseImage(&i);
				}
#endif

				if(img.channels()>3)
				{
					UWARN("Conversion from 4 channels to 3 channels (file=%s)", imageFilePath.c_str());
					cv::Mat out;
					cv::cvtColor(img, out, CV_BGRA2BGR);
					img = out;
				}
			}

			if(!img.empty() && _model.isValid() && _rectifyImages)
			{
				img = _model.rectifyImage(img);
			}
		}

		if(!scanFilePath.empty())
		{
			// load without filtering
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = util3d::loadCloud(scanFilePath, _scanLocalTransform);
			UDEBUG("Loaded scan=%d points", (int)cloud->size());
			if(_depthFromScan && !img.empty())
			{
				UDEBUG("Computing depth from scan...");
				if(!_model.isValid())
				{
					UWARN("Depth from laser scan: Camera model should be valid.");
				}
				else if(_isDepth)
				{
					UWARN("Depth from laser scan: Loading already a depth image.");
				}
				else
				{
					depthFromScan = util3d::projectCloudToCamera(img.size(), _model.K(), cloud, _model.localTransform());
					util3d::fillProjectedCloudHoles(depthFromScan, _depthFromScanFillHolesVertical, _depthFromScanFillHolesFromBorder);
				}
			}
			// filter the scan after registration
			int previousSize = (int)cloud->size();
			if(_scanDownsampleStep > 1 && cloud->size())
			{
				cloud = util3d::downsample(cloud, _scanDownsampleStep);
				UDEBUG("Downsampling scan (step=%d): %d -> %d", _scanDownsampleStep, previousSize, (int)cloud->size());
			}
			previousSize = (int)cloud->size();
			if(_scanVoxelSize > 0.0f && cloud->size())
			{
				cloud = util3d::voxelize(cloud, _scanVoxelSize);
				UDEBUG("Voxel filtering scan (voxel=%f m): %d -> %d", _scanVoxelSize, previousSize, (int)cloud->size());
			}
			if(_scanNormalsK > 0 && cloud->size())
			{
				pcl::PointCloud<pcl::PointNormal>::Ptr cloudNormals = util3d::computeNormals(cloud, _scanNormalsK);
				scan = util3d::laserScanFromPointCloud(*cloudNormals);
			}
			else
			{
				scan = util3d::laserScanFromPointCloud(*cloud);
			}
		}
	}
	else
	{
		UWARN("Directory is not set, camera must be initialized.");
	}

	SensorData data(scan, scan.empty()?0:_scanMaxPts, 0, _isDepth?cv::Mat():img, _isDepth?img:depthFromScan, _model, this->getNextSeqID(), stamp);
	data.setGroundTruth(groundTruthPose);
	return data;
}



/////////////////////////
// CameraVideo
/////////////////////////
CameraVideo::CameraVideo(
		int usbDevice,
		float imageRate,
		const Transform & localTransform) :
	Camera(imageRate, localTransform),
	_rectifyImages(false),
	_src(kUsbDevice),
	_usbDevice(usbDevice)
{

}

CameraVideo::CameraVideo(
		const std::string & filePath,
		bool rectifyImages,
		float imageRate,
		const Transform & localTransform) :
	Camera(imageRate, localTransform),
	_filePath(filePath),
	_rectifyImages(rectifyImages),
	_src(kVideoFile),
	_usbDevice(0)
{
}

CameraVideo::~CameraVideo()
{
	_capture.release();
}

bool CameraVideo::init(const std::string & calibrationFolder, const std::string & cameraName)
{
	_guid.clear();
	if(_capture.isOpened())
	{
		_capture.release();
	}

	if(_src == kUsbDevice)
	{
		ULOGGER_DEBUG("CameraVideo::init() Usb device initialization on device %d", _usbDevice);
		_capture.open(_usbDevice);
	}
	else if(_src == kVideoFile)
	{
		ULOGGER_DEBUG("Camera: filename=\"%s\"", _filePath.c_str());
		_capture.open(_filePath.c_str());
	}
	else
	{
		ULOGGER_ERROR("Camera: Unknown source...");
	}
	if(!_capture.isOpened())
	{
		ULOGGER_ERROR("Camera: Failed to create a capture object!");
		_capture.release();
		return false;
	}
	else
	{
		unsigned int guid = (unsigned int)_capture.get(CV_CAP_PROP_GUID);
		if(guid != 0 && guid != 0xffffffff)
		{
			_guid = uFormat("%08x", guid);
		}

		// look for calibration files
		if(!calibrationFolder.empty() && (!_guid.empty() || !cameraName.empty()))
		{
			if(!_model.load(calibrationFolder, (cameraName.empty()?_guid:cameraName)))
			{
				UWARN("Missing calibration files for camera \"%s\" in \"%s\" folder, you should calibrate the camera!",
						cameraName.empty()?_guid.c_str():cameraName.c_str(), calibrationFolder.c_str());
			}
			else
			{
				UINFO("Camera parameters: fx=%f fy=%f cx=%f cy=%f",
						_model.fx(),
						_model.fy(),
						_model.cx(),
						_model.cy());
			}
		}
		_model.setLocalTransform(this->getLocalTransform());
		if(_rectifyImages && !_model.isValid())
		{
			UERROR("Parameter \"rectifyImages\" is set, but no camera model is loaded or valid.");
			return false;
		}
	}
	return true;
}

bool CameraVideo::isCalibrated() const
{
	return _model.isValid();
}

std::string CameraVideo::getSerial() const
{
	return _guid;
}

SensorData CameraVideo::captureImage()
{
	cv::Mat img;
	if(_capture.isOpened())
	{
		if(_capture.read(img))
		{
			if(_model.isValid() && (_src != kVideoFile || _rectifyImages))
			{
				img = _model.rectifyImage(img);
			}
			else
			{
				// clone required
				img = img.clone();
			}
		}
		else if(_usbDevice)
		{
			UERROR("Camera has been disconnected!");
		}
	}
	else
	{
		ULOGGER_WARN("The camera must be initialized before requesting an image.");
	}

	return SensorData(img, _model, this->getNextSeqID(), UTimer::now());
}

} // namespace rtabmap
