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

#include <rtabmap/core/camera/CameraImages.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UDirectory.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/UFile.h>
#include <rtabmap/utilite/UThreadC.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/core/util3d_filtering.h>
#include <rtabmap/core/Graph.h>
#include <opencv2/imgproc/types_c.h>
#include <fstream>

namespace rtabmap
{

CameraImages::CameraImages() :
		_startAt(0),
		_maxFrames(0),
		_refreshDir(false),
		_rectifyImages(false),
		_bayerMode(-1),
		_isDepth(false),
		_depthScaleFactor(1.0f),
		_count(0),
		_framesPublished(0),
		_dir(0),
		_countScan(0),
		_scanDir(0),
		_scanLocalTransform(Transform::getIdentity()),
		_scanMaxPts(0),
		_depthFromScan(false),
		_depthFromScanFillHoles(1),
		_depthFromScanFillHolesFromBorder(false),
		_filenamesAreTimestamps(false),
		_hasConfigForEachFrame(false),
		_syncImageRateWithStamps(true),
		_odometryFormat(0),
		_groundTruthFormat(0),
		_maxPoseTimeDiff(0.02),
		_captureDelay(0.0)
	{}
CameraImages::CameraImages(const std::string & path,
					 float imageRate,
					 const Transform & localTransform) :
	Camera(imageRate, localTransform),
	_path(path),
	_startAt(0),
	_maxFrames(0),
	_refreshDir(false),
	_rectifyImages(false),
	_bayerMode(-1),
	_isDepth(false),
	_depthScaleFactor(1.0f),
	_count(0),
	_framesPublished(0),
	_dir(0),
	_countScan(0),
	_scanDir(0),
	_scanLocalTransform(Transform::getIdentity()),
	_scanMaxPts(0),
	_depthFromScan(false),
	_depthFromScanFillHoles(1),
	_depthFromScanFillHolesFromBorder(false),
	_filenamesAreTimestamps(false),
	_hasConfigForEachFrame(false),
	_syncImageRateWithStamps(true),
	_odometryFormat(0),
	_groundTruthFormat(0),
	_maxPoseTimeDiff(0.02),
	_captureDelay(0.0)
{

}

CameraImages::~CameraImages()
{
	UDEBUG("");
	delete _dir;
	delete _scanDir;
}

bool CameraImages::init(const std::string & calibrationFolder, const std::string & cameraName)
{
	_lastFileName.clear();
	_lastScanFileName.clear();
	_count = 0;
	_countScan = 0;
	_captureDelay = 0.0;
	_framesPublished=0;
	_model = cameraModel();
	_models.clear();
	covariances_.clear();

	UDEBUG("");
	if(_dir)
	{
		delete _dir;
		_dir = 0;
	}
	if(!_path.empty())
	{
		_dir = new UDirectory(_path, "jpg ppm png bmp pnm tiff pgm");
		if(_path[_path.size()-1] != '\\' && _path[_path.size()-1] != '/')
		{
			_path.append("/");
		}
		if(!_dir->isValid())
		{
			ULOGGER_ERROR("Directory path is not valid \"%s\"", _path.c_str());
			delete _dir;
			_dir = 0;
		}
		else if(_dir->getFileNames().size() == 0)
		{
			UWARN("Directory is empty \"%s\"", _path.c_str());
			delete _dir;
			_dir = 0;
		}
		else
		{
			UINFO("path=%s images=%d", _path.c_str(), (int)this->imagesCount());
		}
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
		else if(_dir && _scanDir->getFileNames().size() != _dir->getFileNames().size())
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

	if(_dir==0 && _scanDir == 0)
	{
		ULOGGER_ERROR("Images path or scans path should be set!");
		return false;
	}

	if(_dir)
	{
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

				cv::FileStorage fs(calibrationFolder+"/"+cameraName+".yaml", 0);
				cv::FileNode poseNode = fs["local_transform"];
				if(!poseNode.isNone())
				{
					UWARN("Using local transform from calibration file (%s) instead of the parameter one (%s).",
							_model.localTransform().prettyPrint().c_str(),
							this->getLocalTransform().prettyPrint().c_str());
					this->setLocalTransform(_model.localTransform());
				}
			}
		}
		_model.setName(cameraName);

		_model.setLocalTransform(this->getLocalTransform());
		if(_rectifyImages && !_model.isValidForRectification())
		{
			UERROR("Parameter \"rectifyImages\" is set, but no camera model is loaded or valid.");
			return false;
		}
	}

	bool success = _dir|| _scanDir;
	_stamps.clear();
	odometry_.clear();
	groundTruth_.clear();
	if(success)
	{
		if(_dir && _hasConfigForEachFrame)
		{
#if CV_MAJOR_VERSION < 3 || (CV_MAJOR_VERSION == 3 && CV_MAJOR_VERSION < 2)
			UDirectory dirJson(_path, "yaml xml");
#else
			UDirectory dirJson(_path, "yaml xml json");
#endif
			if(dirJson.getFileNames().size() == _dir->getFileNames().size())
			{
				bool modelsWarned = false;
				bool localTWarned = false;
				for(std::list<std::string>::const_iterator iter=dirJson.getFileNames().begin(); iter!=dirJson.getFileNames().end() && success; ++iter)
				{
					std::string filePath = _path+"/"+*iter;
					cv::FileStorage fs(filePath, 0);
					cv::FileNode poseNode = fs["cameraPoseARFrame"]; // Check if it is 3DScannerApp(iOS) format
					if(poseNode.isNone())
					{
						cv::FileNode n = fs["local_transform"];
						bool hasLocalTransform = !n.isNone();

						fs.release();
						if(_model.isValidForProjection() && !modelsWarned)
						{
							UWARN("Camera model loaded for each frame is overridden by "
									"general calibration file provided. Remove general calibration "
									"file to use camera model of each frame. This warning will "
									"be shown only one time.");
							modelsWarned = true;
						}
						else
						{
							CameraModel model;
							model.load(filePath);

							if(!hasLocalTransform)
							{
								if(!localTWarned)
								{
									UWARN("Loaded calibration file doesn't have local_transform field, "
											"the global local_transform parameter is used by default (%s).",
											this->getLocalTransform().prettyPrint().c_str());
									localTWarned = true;
								}
								model.setLocalTransform(this->getLocalTransform());
							}

							_models.push_back(model);
						}
					}
					else
					{
						cv::FileNode timeNode = fs["time"];
						cv::FileNode intrinsicsNode = fs["intrinsics"];
						if(poseNode.isNone() || poseNode.size() != 16)
						{
							UERROR("Failed reading \"cameraPoseARFrame\" parameter, it should have 16 values (file=%s)", filePath.c_str());
							success = false;
							break;
						}
						else if(timeNode.isNone() || !timeNode.isReal())
						{
							UERROR("Failed reading \"time\" parameter (file=%s)", filePath.c_str());
							success = false;
							break;
						}
						else if(intrinsicsNode.isNone() || intrinsicsNode.size()!=9)
						{
							UERROR("Failed reading \"intrinsics\" parameter (file=%s)", filePath.c_str());
							success = false;
							break;
						}
						else
						{
							_stamps.push_back((double)timeNode);
							if(_model.isValidForProjection() && !modelsWarned)
							{
								UWARN("Camera model loaded for each frame is overridden by "
										"general calibration file provided. Remove general calibration "
										"file to use camera model of each frame. This warning will "
										"be shown only one time.");
								modelsWarned = true;
							}
							else
							{
								_models.push_back(CameraModel(
										(double)intrinsicsNode[0], //fx
										(double)intrinsicsNode[4], //fy
										(double)intrinsicsNode[2], //cx
										(double)intrinsicsNode[5], //cy
										CameraModel::opticalRotation()));
							}
							// we need to rotate from opengl world to rtabmap world
							Transform pose(
									(float)poseNode[0], (float)poseNode[1], (float)poseNode[2], (float)poseNode[3],
									(float)poseNode[4], (float)poseNode[5], (float)poseNode[6], (float)poseNode[7],
									(float)poseNode[8], (float)poseNode[9], (float)poseNode[10], (float)poseNode[11]);
							pose =  Transform::rtabmap_T_opengl() * pose * Transform::opengl_T_rtabmap();
							odometry_.push_back(pose);
						}
					}
				}
				if(!success)
				{
					odometry_.clear();
					_stamps.clear();
					_models.clear();
				}
			}
			else
			{
				std::string opencv32warn;
#if CV_MAJOR_VERSION < 3 || (CV_MAJOR_VERSION == 3 && CV_MAJOR_VERSION < 2)
				opencv32warn = " RTAB-Map is currently built with OpenCV < 3.2, only xml and yaml files are supported (not json).";
#endif
				UERROR("Parameter \"Config for each frame\" is true, but the "
						"number of config files (%d) is not equal to number "
						"of images (%d) in this directory \"%s\".%s",
						(int)dirJson.getFileNames().size(),
						(int)_dir->getFileNames().size(),
						_path.c_str(),
						opencv32warn.c_str());
				success = false;
			}
		}

		if(_stamps.empty())
		{
			if(_filenamesAreTimestamps)
			{
				std::list<std::string> filenames = _dir?_dir->getFileNames():_scanDir->getFileNames();
				for(std::list<std::string>::const_iterator iter=filenames.begin(); iter!=filenames.end(); ++iter)
				{
					// format is text_1223445645.12334_text.png or text_122344564512334_text.png
					// If no decimals, 10 first number are the seconds
					std::list<std::string> list = uSplit(*iter, '.');
					if(list.size() == 3 || list.size() == 2)
					{
						list.pop_back(); // remove extension
						double stamp = 0.0;
						if(list.size() == 1)
						{
							std::list<std::string> numberList = uSplitNumChar(list.front());
							for(std::list<std::string>::iterator iter=numberList.begin(); iter!=numberList.end(); ++iter)
							{
								if(uIsNumber(*iter))
								{
									std::string decimals;
									std::string sec;
									if(iter->length()>10)
									{
										decimals = iter->substr(10, iter->size()-10);
										sec = iter->substr(0, 10);
									}
									else
									{
										sec = *iter;
									}
									stamp = uStr2Double(sec + "." + decimals);
									break;
								}
							}
						}
						else
						{
							std::string decimals = uSplitNumChar(list.back()).front();
							list.pop_back();
							std::string sec = uSplitNumChar(list.back()).back();
							stamp = uStr2Double(sec + "." + decimals);
						}
						if(stamp > 0.0)
						{
							_stamps.push_back(stamp);
						}
						else
						{
							UERROR("Conversion filename to timestamp failed! (filename=%s)", iter->c_str());
						}
					}
				}
				if(_stamps.size() != this->imagesCount())
				{
					UERROR("The stamps count is not the same as the images (%d vs %d)! "
						   "Converting filenames to timestamps is activated.",
							(int)_stamps.size(), this->imagesCount());
					_stamps.clear();
					success = false;
				}
			}
			else if(_timestampsPath.size())
			{
				std::ifstream file;
				file.open(_timestampsPath.c_str(), std::ifstream::in);
				while(file.good())
				{
					std::string str;
					std::getline(file, str);

					if(str.empty() || str.at(0) == '#' || str.at(0) == '%')
					{
						continue;
					}

					std::list<std::string> strList = uSplit(str, ' ');
					std::string stampStr = strList.front();
					if(strList.size() == 2)
					{
						// format "seconds millisec"
						// the millisec str needs 0-padding if size < 6
						std::string millisecStr = strList.back();
						while(millisecStr.size() < 6)
						{
							millisecStr = "0" + millisecStr;
						}
						stampStr = stampStr+'.'+millisecStr;
					}
					_stamps.push_back(uStr2Double(stampStr));
				}

				file.close();

				if(_stamps.size() != this->imagesCount())
				{
					UERROR("The stamps count (%d) is not the same as the images (%d)! Please remove "
							"the timestamps file path if you don't want to use them (current file path=%s).",
							(int)_stamps.size(), this->imagesCount(), _timestampsPath.c_str());
					_stamps.clear();
					success = false;
				}
			}
		}

		if(success && _odometryPath.size() && odometry_.empty())
		{
			success = readPoses(odometry_, _stamps, _odometryPath, _odometryFormat, _maxPoseTimeDiff);
		}

		if(success && _groundTruthPath.size())
		{
			success = readPoses(groundTruth_, _stamps, _groundTruthPath, _groundTruthFormat, _maxPoseTimeDiff);
		}

		if(!odometry_.empty())
		{
			for(size_t i=0; i<odometry_.size(); ++i)
			{
				// linear cov = 0.0001
				cv::Mat covariance = cv::Mat::eye(6,6,CV_64FC1) * (i==0?9999.0:0.0001);
				if(i!=0)
				{
					// angular cov = 0.000001
					covariance.at<double>(3,3) *= 0.01;
					covariance.at<double>(4,4) *= 0.01;
					covariance.at<double>(5,5) *= 0.01;
				}
				covariances_.push_back(covariance);
			}
		}
	}

	_captureTimer.restart();

	return success;
}

bool CameraImages::readPoses(
		std::list<Transform> & outputPoses,
		std::list<double> & inOutStamps,
		const std::string & filePath,
		int format,
		double maxTimeDiff) const
{
	outputPoses.clear();
	std::map<int, Transform> poses;
	std::map<int, double> stamps;
	if(!graph::importPoses(filePath, format, poses, 0, &stamps))
	{
		UERROR("Cannot read pose file \"%s\".", filePath.c_str());
		return false;
	}
	else if((format != 1 && format != 10 && format != 5 && format != 6 && format != 7 && format != 9) && poses.size() != this->imagesCount())
	{
		UERROR("The pose count is not the same as the images (%d vs %d)! Please remove "
				"the pose file path if you don't want to use it (current file path=%s).",
				(int)poses.size(), this->imagesCount(), filePath.c_str());
		return false;
	}
	else if((format == 1 || format == 10 || format == 5 || format == 6 || format == 7 || format == 9) && (inOutStamps.empty() && stamps.size()!=poses.size()))
	{
		UERROR("When using RGBD-SLAM, GPS, MALAGA, ST LUCIA and EuRoC MAV formats, images must have timestamps!");
		return false;
	}
	else if(format == 1 || format == 10 || format == 5 || format == 6 || format == 7 || format == 9)
	{
		UDEBUG("");
		//Match ground truth values with images
		outputPoses.clear();
		std::map<double, int> stampsToIds;
		for(std::map<int, double>::iterator iter=stamps.begin(); iter!=stamps.end(); ++iter)
		{
			stampsToIds.insert(std::make_pair(iter->second, iter->first));
		}
		std::vector<double> values = uValues(stamps);

		if(inOutStamps.empty())
		{
			inOutStamps = uValuesList(stamps);
		}

		int validPoses = 0;
		for(std::list<double>::iterator ster=inOutStamps.begin(); ster!=inOutStamps.end(); ++ster)
		{
			Transform pose; // null transform
			std::map<double, int>::iterator endIter = stampsToIds.lower_bound(*ster);
			bool warned = false;
			if(endIter != stampsToIds.end())
			{
				if(endIter->first == *ster)
				{
					pose = poses.at(endIter->second);
					++validPoses;
				}
				else if(endIter != stampsToIds.begin())
				{
					//interpolate
					std::map<double, int>::iterator beginIter = endIter;
					--beginIter;
					double stampBeg = beginIter->first;
					double stampEnd = endIter->first;
					UASSERT(stampEnd > stampBeg && *ster>stampBeg && *ster < stampEnd);
					if(fabs(*ster-stampEnd) > maxTimeDiff || fabs(*ster-stampBeg) > maxTimeDiff)
					{
						if(!warned)
						{
							UWARN("Cannot interpolate pose for stamp %f between %f and %f (> maximum time diff of %f sec)",
								*ster,
								stampBeg,
								stampEnd,
								maxTimeDiff);
						}
						warned=true;
					}
					else
					{
						warned=false;
						float t = (*ster - stampBeg) / (stampEnd-stampBeg);
						Transform & ta = poses.at(beginIter->second);
						Transform & tb = poses.at(endIter->second);
						if(!ta.isNull() && !tb.isNull())
						{
							++validPoses;
							pose = ta.interpolate(t, tb);
						}
					}
				}
			}
			if(pose.isNull() && !warned)
			{
				UDEBUG("Pose not found for stamp %f", *ster);
			}
			outputPoses.push_back(pose);
		}
		if(validPoses != (int)inOutStamps.size())
		{
			UWARN("%d valid poses of %d stamps", validPoses, (int)inOutStamps.size());
		}
	}
	else
	{
		UDEBUG("");
		outputPoses = uValuesList(poses);
		if(inOutStamps.size() == 0 && stamps.size() == poses.size())
		{
			inOutStamps = uValuesList(stamps);
		}
		else if(format==8 && inOutStamps.size() == 0 && stamps.size()>0 && stamps.size() != poses.size())
		{
			UERROR("With Karlsruhe format, timestamps (%d) and poses (%d) should match!", (int)stamps.size(), (int)poses.size());
			return false;
		}
		else if(!outputPoses.empty() && inOutStamps.empty() && stamps.empty())
		{
			UERROR("Timestamps are empty (poses=%d)! Forgot the set a timestamp file?", (int)outputPoses.size());
			return false;
		}
	}
	UASSERT_MSG(outputPoses.size() == inOutStamps.size(), uFormat("%d vs %d", (int)outputPoses.size(), (int)inOutStamps.size()).c_str());
	return true;
}

bool CameraImages::isCalibrated() const
{
	return (_dir && (_model.isValidForProjection() || (_models.size() && _models.front().isValidForProjection()))) ||
			_scanDir;
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
	else if(_scanDir)
	{
		return (unsigned int)_scanDir->getFileNames().size();
	}
	return 0;
}

std::vector<std::string> CameraImages::filenames() const
{
	if(_dir)
	{
		return uListToVector(_dir->getFileNames());
	}
	else if(_scanDir)
	{
		return uListToVector(_scanDir->getFileNames());
	}
	return std::vector<std::string>();
}

SensorData CameraImages::captureImage(CameraInfo * info)
{
	if(_syncImageRateWithStamps && _captureDelay>0.0)
	{
		int sleepTime = (1000*_captureDelay - 1000.0f*_captureTimer.getElapsedTime());
		if(sleepTime > 2)
		{
			uSleep(sleepTime-2);
		}
		else if(sleepTime < 0)
		{
			if(this->getImageRate() > 0.0f)
			{
				UWARN("CameraImages: Cannot read images as fast as their timestamps (target delay=%fs, capture time=%fs). Disable "
					  "source image rate or disable synchronization of capture time with timestamps.", 
					  _captureDelay, _captureTimer.getElapsedTime());
			}
			else
			{
				UWARN("CameraImages: Cannot read images as fast as their timestamps (target delay=%fs, capture time=%fs).", 
					_captureDelay, _captureTimer.getElapsedTime());
			}
		}

		// Add precision at the cost of a small overhead
		while(_captureTimer.getElapsedTime() < _captureDelay-0.000001)
		{
			//
		}
		_captureTimer.start();
	}
	_captureDelay = 0.0;

	cv::Mat img;
	LaserScan scan(cv::Mat(), _scanMaxPts, 0, LaserScan::kUnknown, _scanLocalTransform);
	double stamp = UTimer::now();
	Transform odometryPose;
	cv::Mat covariance;
	Transform groundTruthPose;
	cv::Mat depthFromScan;
	CameraModel model = _model;
	UDEBUG("");
	if(_dir || _scanDir)
	{
		if(_refreshDir)
		{
			if(_dir)
			{
				_dir->update();
			}
			if(_scanDir)
			{
				_scanDir->update();
			}
		}
		std::string imageFilePath;
		std::string scanFilePath;
		if(_startAt < 0)
		{
			if(_dir)
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

			if(_stamps.size())
			{
				stamp = _stamps.front();
				_stamps.pop_front();
				if(_stamps.size())
				{
					_captureDelay = _stamps.front() - stamp;
				}
			}
			if(odometry_.size())
			{
				odometryPose = odometry_.front();
				odometry_.pop_front();
				if(covariances_.size())
				{
					covariance = covariances_.front();
					covariances_.pop_front();
				}
			}
			if(groundTruth_.size())
			{
				groundTruthPose = groundTruth_.front();
				groundTruth_.pop_front();
			}
			if(_models.size() && !model.isValidForProjection())
			{
				model = _models.front();
				_models.pop_front();
			}
		}
		else
		{
			std::string imageFileName = _dir?_dir->getNextFileName():"";
			std::string scanFileName = _scanDir?_scanDir->getNextFileName():"";
			if((_dir && !imageFileName.empty()) || (!_dir && !scanFileName.empty()))
			{
				imageFilePath = _path + imageFileName;
				scanFilePath = _scanPath + scanFileName;
				if(_stamps.size())
				{
					stamp = _stamps.front();
					_stamps.pop_front();
					if(_stamps.size())
					{
						_captureDelay = _stamps.front() - stamp;
					}
				}
				if(odometry_.size())
				{
					odometryPose = odometry_.front();
					odometry_.pop_front();
					if(covariances_.size())
					{
						covariance = covariances_.front();
						covariances_.pop_front();
					}
				}
				if(groundTruth_.size())
				{
					groundTruthPose = groundTruth_.front();
					groundTruth_.pop_front();
				}
				if(_models.size() && !model.isValidForProjection())
				{
					model = _models.front();
					_models.pop_front();
				}

				while(_count++ < _startAt)
				{
					imageFileName = _dir?_dir->getNextFileName():"";
					scanFileName = _scanDir?_scanDir->getNextFileName():"";

					if((_dir && imageFileName.empty()) || (!_dir && scanFileName.empty()))
					{
						break;
					}

					imageFilePath = _path + imageFileName;
					scanFilePath = _scanPath + scanFileName;
					if(_stamps.size())
					{
						stamp = _stamps.front();
						_stamps.pop_front();
						if(_stamps.size())
						{
							_captureDelay = _stamps.front() - stamp;
						}
					}
					if(odometry_.size())
					{
						odometryPose = odometry_.front();
						odometry_.pop_front();
						if(covariances_.size())
						{
							covariance = covariances_.front();
							covariances_.pop_front();
						}
					}
					if(groundTruth_.size())
					{
						groundTruthPose = groundTruth_.front();
						groundTruth_.pop_front();
					}
					if(_models.size() && !model.isValidForProjection())
					{
						model = _models.front();
						_models.pop_front();
					}
				}
			}
		}

		if(_maxFrames <=0 || ++_framesPublished <= _maxFrames)
		{

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
						UERROR("Depth is on and the loaded image has not a format supported (file = \"%s\", type=%d). "
								"Formats supported are 16 bits 1 channel (mm) and 32 bits 1 channel (m).",
								imageFilePath.c_str(), img.type());
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
					else if(!img.empty() && _bayerMode >= 0 && _bayerMode <=3)
					{
						cv::Mat debayeredImg;
						try
						{
							cv::cvtColor(img, debayeredImg, CV_BayerBG2BGR + _bayerMode);
							img = debayeredImg;
						}
						catch(const cv::Exception & e)
						{
							UWARN("Error debayering images: \"%s\". Please set bayer mode to -1 if images are not bayered!", e.what());
						}
					}
				}

				if(!img.empty() && model.isValidForRectification() && _rectifyImages)
				{
					img = model.rectifyImage(img);
				}
			}

			if(!scanFilePath.empty())
			{
				// load without filtering
				scan = util3d::loadScan(scanFilePath);
				scan = LaserScan(scan.data(), _scanMaxPts, 0.0f, scan.format(), _scanLocalTransform);
				UDEBUG("Loaded scan=%d points", (int)scan.size());
				if(_depthFromScan && !img.empty())
				{
					UDEBUG("Computing depth from scan...");
					if(!model.isValidForProjection())
					{
						UWARN("Depth from laser scan: Camera model should be valid.");
					}
					else if(_isDepth)
					{
						UWARN("Depth from laser scan: Loading already a depth image.");
					}
					else
					{
						pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = util3d::laserScanToPointCloud(scan, scan.localTransform());
						depthFromScan = util3d::projectCloudToCamera(img.size(), model.K(), cloud, model.localTransform());
						if(_depthFromScanFillHoles!=0)
						{
							util3d::fillProjectedCloudHoles(depthFromScan, _depthFromScanFillHoles>0, _depthFromScanFillHolesFromBorder);
						}
					}
				}
			}
		}
	}
	else
	{
		UWARN("Directory is not set, camera must be initialized.");
	}

	if(model.imageHeight() == 0 || model.imageWidth() == 0)
	{
		model.setImageSize(img.size());
	}

	SensorData data;
	if(!img.empty() || !scan.empty())
	{
		data = SensorData(scan, _isDepth?cv::Mat():img, _isDepth?img:depthFromScan, model, this->getNextSeqID(), stamp);
		data.setGroundTruth(groundTruthPose);
	}

	if(info && !odometryPose.isNull())
	{
		info->odomPose = odometryPose;
		info->odomCovariance = covariance.empty()?cv::Mat::eye(6,6,CV_64FC1):covariance; // Note that with TORO and g2o file formats, we could get the covariance
	}

	return data;
}

} // namespace rtabmap
