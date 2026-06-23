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
		_multiCameraCalib(false),
		_multiCameraCount(0),
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
		_groundTruthLocalTransform(Transform::getIdentity()),
		_maxPoseTimeDiff(0.02),
		_captureDelay(0.0)
	{}
CameraImages::CameraImages(const std::string & path,
					 float imageRate,
					 const Transform & localTransform) :
	Camera(imageRate, localTransform),
	_multiCameraCalib(false),
	_multiCameraCount(0),
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
	_groundTruthLocalTransform(Transform::getIdentity()),
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
	_lastImageFileName.clear();
	_lastScanFileName.clear();
	_count = 0;
	_countScan = 0;
	_captureDelay = 0.0;
	_framesPublished=0;
	_model = cameraModel();
	_modelFileNames.clear();
	_configLocalTransformWarned = false;
	_multiModels.clear();
	_calibrationFolder.clear();
	_multiCameraCount = 0;
	covariances_.clear();
	_stamps.clear();
	odometry_.clear();
	groundTruth_.clear();

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

	bool success = _dir|| _scanDir;

	if(_dir)
	{
		if(_multiCameraCalib)
		{
			// Multi-camera mode: each image is a horizontal stack of N sub-camera
			// images. Load one CameraModel per sub-camera from calibration files
			// named "<prefix>_<index>.yaml" (index starting at 0). In config-for-each-frame
			// mode the prefix is each image's base name (one calibration set per frame);
			// otherwise a single calibration set is shared by all frames, using cameraName
			// as the prefix when provided (it may differ from any image name), falling back
			// to the first image's base name.
			if(calibrationFolder.empty())
			{
				UERROR("Multi-camera calibration is enabled but no calibration folder was provided.");
				return false;
			}
			const std::list<std::string> & imageFiles = _dir->getFileNames();
			std::string firstBase = imageFiles.front().substr(0, imageFiles.front().find_last_of('.'));

			// In config-for-each-frame mode the prefix is each image's base name. Otherwise
			// the shared prefix is cameraName when provided. cameraName may point to a specific
			// sub-camera calibration "<rig>_<index>" (e.g. when a single calibration file is
			// selected); in that case strip the trailing "_<index>" to recover the rig prefix
			// shared by all sub-cameras. Fall back to the first image's base name if empty.
			std::string sharedBase = cameraName;
			if(!sharedBase.empty() && !UFile::exists(calibrationFolder + "/" + sharedBase + "_0.yaml"))
			{
				std::size_t us = sharedBase.find_last_of('_');
				if(us != std::string::npos && us+1 < sharedBase.size() &&
					sharedBase.find_first_not_of("0123456789", us+1) == std::string::npos &&
					UFile::exists(calibrationFolder + "/" + sharedBase.substr(0, us) + "_0.yaml"))
				{
					sharedBase = sharedBase.substr(0, us);
				}
			}
			if(sharedBase.empty())
			{
				sharedBase = firstBase;
			}

			// auto-detect the number of cameras
			int numCameras = 0;
			std::string detectBase = _hasConfigForEachFrame ? firstBase : sharedBase;
			while(UFile::exists(calibrationFolder + "/" + detectBase + "_" + uNumber2Str(numCameras) + ".yaml"))
			{
				++numCameras;
			}

			if(numCameras == 0)
			{
				UERROR("Multi-camera calibration is enabled but no calibration file matching "
						"\"%s/%s_<index>.yaml\" was found.",
						calibrationFolder.c_str(), detectBase.c_str());
				return false;
			}

			UINFO("Multi-camera mode: %d sub-cameras detected from \"%s\" (%s).", numCameras, calibrationFolder.c_str(),
					_hasConfigForEachFrame?"one calibration per frame, loaded on demand":
							uFormat("calibration \"%s_<index>\" reused for all frames", sharedBase.c_str()).c_str());
			_calibrationFolder = calibrationFolder;
			_multiCameraCount = numCameras;
			if(_hasConfigForEachFrame)
			{
				// Validate the first frame now; the per-frame sub-camera models are loaded
				// on demand in captureImage() (keyed by each image's base name) so we don't
				// keep every frame's models - and their rectification maps - in memory.
				if(loadMultiCameraModels(firstBase).empty())
				{
					return false;
				}
			}
			else
			{
				// A single calibration set is shared by all frames: load it once.
				std::vector<CameraModel> models = loadMultiCameraModels(sharedBase);
				if(models.empty())
				{
					return false;
				}
				_multiModels = models;
			}
		}
		else if(_hasConfigForEachFrame)
		{
			// Per-frame calibration: one config file per image, taken from the
			// calibration folder (same convention as the single/multi-camera cases).
			// If the config files are in the same folder than the images, just set the
			// calibration folder to the images folder.
#if CV_MAJOR_VERSION < 3 || (CV_MAJOR_VERSION == 3 && CV_MAJOR_VERSION < 2)
			UDirectory dirJson(calibrationFolder, "yaml xml");
#else
			UDirectory dirJson(calibrationFolder, "yaml xml json");
#endif
			if(dirJson.getFileNames().size() == _dir->getFileNames().size())
			{
				// The per-frame models are loaded on demand in captureImage() (see
				// loadConfigModel) so we don't keep every frame's model - and its
				// rectification map - in memory. Only the stamps and poses of the
				// 3DScannerApp format are read eagerly here (needed for synchronization).
				for(std::list<std::string>::const_iterator iter=dirJson.getFileNames().begin(); iter!=dirJson.getFileNames().end() && success; ++iter)
				{
					std::string filePath = calibrationFolder+"/"+*iter;
					cv::FileStorage fs(filePath, 0);
					cv::FileNode poseNode = fs["cameraPoseARFrame"]; // Check if it is 3DScannerApp(iOS) format
					if(!poseNode.isNone())
					{
						cv::FileNode timeNode = fs["time"];
						cv::FileNode intrinsicsNode = fs["intrinsics"];
						if(poseNode.size() != 16)
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
							// we need to rotate from opengl world to rtabmap world
							Transform pose(
									(float)poseNode[0], (float)poseNode[1], (float)poseNode[2], (float)poseNode[3],
									(float)poseNode[4], (float)poseNode[5], (float)poseNode[6], (float)poseNode[7],
									(float)poseNode[8], (float)poseNode[9], (float)poseNode[10], (float)poseNode[11]);
							pose =  Transform::rtabmap_T_opengl() * pose * Transform::opengl_T_rtabmap();
							odometry_.push_back(pose);
						}
					}
					_modelFileNames.push_back(filePath);
				}
				// Validate the first frame's calibration now to fail early.
				if(success && !_modelFileNames.empty() && !loadConfigModel(_modelFileNames.front()).isValidForProjection())
				{
					success = false;
				}
				if(!success)
				{
					odometry_.clear();
					_stamps.clear();
					_modelFileNames.clear();
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
						calibrationFolder.c_str(),
						opencv32warn.c_str());
				success = false;
			}
		}
		else
		{
			// Config-for-each-frame is disabled: load a single general calibration
			// model (the per-frame branch above handles the config-for-each-frame case).
			// look for calibration files
			UINFO("calibration folder=%s name=%s", calibrationFolder.c_str(), cameraName.c_str());
			if(!calibrationFolder.empty() && !cameraName.empty())
			{
				if(!_model.load(calibrationFolder, cameraName, _rectifyImages))
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

				// Only validate rectification when a general calibration was requested.
				// Without it (e.g. images that are already rectified) the single model is
				// expected to be empty and must not fail init here.
				if(_rectifyImages && !_model.isValidForRectification())
				{
					UERROR("Parameter \"rectifyImages\" is set, but no camera model is loaded or valid.");
					return false;
				}
			}
			_model.setName(cameraName);

			_model.setLocalTransform(this->getLocalTransform());
		}
	}

	if(success)
	{
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
			if(!success)
			{
				UERROR("Failed to read odometry poses.");
			}
			
			if(success)
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

		if(success && _groundTruthPath.size())
		{
			success = readPoses(groundTruth_, _stamps, _groundTruthPath, _groundTruthFormat, _maxPoseTimeDiff);
			if(!success)
			{
				UERROR("Failed to read ground truth poses.");
			}
			else if(!_groundTruthLocalTransform.isIdentity())
			{
				Transform gtInv = _groundTruthLocalTransform.inverse();
				for(auto pose: groundTruth_)
				{
					pose = pose*gtInv; // pose of base_link, assuming ground truth frame and base frame are rigidly fixed
				}
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
	else if((format != 1 && format != 10 && format != 12 && format != 5 && format != 6 && format != 7 && format != 9) && poses.size() != this->imagesCount())
	{
		UERROR("The pose count is not the same as the images (%d vs %d)! Please remove "
				"the pose file path if you don't want to use it (current file path=%s).",
				(int)poses.size(), this->imagesCount(), filePath.c_str());
		return false;
	}
	else if((format == 1 || format == 10 || format == 12 || format == 5 || format == 6 || format == 7 || format == 9) && (inOutStamps.empty() && stamps.size()!=poses.size()))
	{
		UERROR("When using RGBD-SLAM, GPS, MALAGA, ST LUCIA and EuRoC MAV formats, images must have timestamps!");
		return false;
	}
	else if(format == 1 || format == 10 || format == 12 || format == 5 || format == 6 || format == 7 || format == 9)
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
			UWARN("%d/%ld valid poses of %ld stamps", validPoses, outputPoses.size(), inOutStamps.size());
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
	return (_dir && (_model.isValidForProjection() ||
			_modelFileNames.size() || // per-frame single-camera models loaded on demand (validated in init)
			(!_multiModels.empty() && _multiModels.front().isValidForProjection()) ||
			(_multiCameraCalib && _hasConfigForEachFrame && _multiCameraCount > 0))) || // per-frame multi-camera models loaded on demand
			_scanDir;
}

std::vector<CameraModel> CameraImages::loadMultiCameraModels(const std::string & baseName) const
{
	std::vector<CameraModel> models(_multiCameraCount);
	for(int i=0; i<_multiCameraCount; ++i)
	{
		std::string name = baseName + "_" + uNumber2Str(i);
		if(!models[i].load(_calibrationFolder, name, _rectifyImages) || !models[i].isValidForProjection())
		{
			UERROR("Failed to load a valid calibration \"%s/%s.yaml\" for multi-camera frame base \"%s\".",
					_calibrationFolder.c_str(), name.c_str(), baseName.c_str());
			return std::vector<CameraModel>();
		}
		if(models[i].localTransform().isNull())
		{
			// In multi-camera mode the rig extrinsics are required: each
			// sub-camera calibration must provide a "local_transform".
			UERROR("Calibration \"%s/%s.yaml\" has no \"local_transform\"; it is "
					"required in multi-camera mode (rig extrinsics).",
					_calibrationFolder.c_str(), name.c_str());
			return std::vector<CameraModel>();
		}
		if(_rectifyImages && !models[i].isValidForRectification())
		{
			UERROR("Parameter \"rectifyImages\" is set, but calibration \"%s/%s.yaml\" is not valid for rectification.",
					_calibrationFolder.c_str(), name.c_str());
			return std::vector<CameraModel>();
		}
	}
	return models;
}

CameraModel CameraImages::loadConfigModel(const std::string & filePath)
{
	cv::FileStorage fs(filePath, 0);
	cv::FileNode poseNode = fs["cameraPoseARFrame"]; // Check if it is 3DScannerApp(iOS) format
	CameraModel model;
	if(poseNode.isNone())
	{
		// RTAB-Map calibration format
		bool hasLocalTransform = !fs["local_transform"].isNone();
		fs.release();
		model.load(filePath, _rectifyImages);
		if(!hasLocalTransform)
		{
			if(!_configLocalTransformWarned)
			{
				UWARN("Loaded calibration file doesn't have local_transform field, "
						"the global local_transform parameter is used by default (%s).",
						this->getLocalTransform().prettyPrint().c_str());
				_configLocalTransformWarned = true;
			}
			model.setLocalTransform(this->getLocalTransform());
		}
	}
	else
	{
		// 3DScannerApp(iOS) format
		cv::FileNode intrinsicsNode = fs["intrinsics"];
		if(intrinsicsNode.isNone() || intrinsicsNode.size()!=9)
		{
			UERROR("Failed reading \"intrinsics\" parameter (file=%s)", filePath.c_str());
			return CameraModel();
		}
		model = CameraModel(
				(double)intrinsicsNode[0], //fx
				(double)intrinsicsNode[4], //fy
				(double)intrinsicsNode[2], //cx
				(double)intrinsicsNode[5], //cy
				CameraModel::opticalRotation());
	}
	if(!model.isValidForProjection())
	{
		UERROR("Camera model loaded from \"%s\" is not valid for projection.", filePath.c_str());
		return CameraModel();
	}
	if(_rectifyImages && !model.isValidForRectification())
	{
		UERROR("Parameter \"rectifyImages\" is set, but camera model loaded from \"%s\" is not valid for rectification.",
				filePath.c_str());
		return CameraModel();
	}
	return model;
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

SensorData CameraImages::captureImage(SensorCaptureInfo * info)
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
	std::vector<CameraModel> models; // one entry per camera (single entry in single-camera mode)
	if(!_multiCameraCalib)
	{
		// Single-camera mode keeps exactly one model in the vector; in multi-camera
		// mode the per-frame vector is taken from _multiModels below.
		models.push_back(_model);
	}
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
				UERROR("stamps cannot be used when startAt < 0");
			}
			if(odometry_.size())
			{
				UERROR("odometry cannot be used when startAt < 0");
			}
			if(groundTruth_.size())
			{
				UERROR("groundTruth cannot be used when startAt < 0");
			}
			if(_modelFileNames.size() || !_multiModels.empty() ||
				(_multiCameraCalib && _hasConfigForEachFrame))
			{
				UERROR("models cannot be used when startAt < 0");
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
				size_t stampsSize = _stamps.size();
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
					UASSERT_MSG(stampsSize==0 || stampsSize == odometry_.size(), 
						uFormat("Stamps=%ld odometry=%ld", _stamps.size(), odometry_.size()).c_str());
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
					UASSERT_MSG(stampsSize==0 || stampsSize == groundTruth_.size(), 
						uFormat("Stamps=%ld groundTruth=%ld", _stamps.size(), groundTruth_.size()).c_str());
					groundTruthPose = groundTruth_.front();
					groundTruth_.pop_front();
				}
				if(_modelFileNames.size())
				{
					UASSERT_MSG(stampsSize==0 || stampsSize == _modelFileNames.size(),
						uFormat("Stamps=%ld models=%ld", _stamps.size(), _modelFileNames.size()).c_str());
					// per-frame calibration loaded on demand (not kept in memory)
					models.back() = loadConfigModel(_modelFileNames.front());
					_modelFileNames.pop_front();
				}
				if(_multiCameraCalib && _hasConfigForEachFrame)
				{
					// Per-frame calibration loaded on demand (not kept in memory),
					// keyed by the current image's base name.
					models = loadMultiCameraModels(imageFileName.substr(0, imageFileName.find_last_of('.')));
				}
				else if(!_multiModels.empty())
				{
					// calibration is shared across all frames
					models = _multiModels;
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
					size_t stampsSize = _stamps.size();
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
						UASSERT_MSG(stampsSize==0 || stampsSize == odometry_.size(), 
							uFormat("Stamps=%ld odometry=%ld", stampsSize, odometry_.size()).c_str());
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
						UASSERT_MSG(stampsSize==0 || stampsSize == groundTruth_.size(), 
							uFormat("Stamps=%ld groundTruth=%ld", _stamps.size(), groundTruth_.size()).c_str());
						groundTruthPose = groundTruth_.front();
						groundTruth_.pop_front();
					}
					if(_modelFileNames.size())
					{
						UASSERT_MSG(stampsSize==0 || stampsSize == _modelFileNames.size(),
							uFormat("Stamps=%ld models=%ld", _stamps.size(), _modelFileNames.size()).c_str());
						// per-frame calibration loaded on demand (not kept in memory)
						models.back() = loadConfigModel(_modelFileNames.front());
						_modelFileNames.pop_front();
					}
					if(_multiCameraCalib && _hasConfigForEachFrame)
					{
						// Per-frame calibration loaded on demand (not kept in memory),
						// keyed by the current image's base name.
						models = loadMultiCameraModels(imageFileName.substr(0, imageFileName.find_last_of('.')));
					}
					else if(!_multiModels.empty())
					{
						// calibration is shared across all frames
						models = _multiModels;
					}
				}
				_lastImageFileName = imageFileName; // expose the current frame name to subclasses
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

				if(!img.empty() && !models.empty())
				{
					// The image is a horizontal stack of N sub-images (N==1 in
					// single-camera mode). Each sub-camera's width comes from its
					// calibrated image size; if a model has no size (==0), assume a
					// uniform split (stackedWidth / N) and set it. If requested, each
					// sub-image is rectified independently with its model and written
					// into a new stacked image of the same layout.
					cv::Mat rectified;
					int offset = 0;
					bool rectifyAborted = false;
					for(size_t i=0; i<models.size(); ++i)
					{
						if(models[i].imageWidth()==0 || models[i].imageHeight()==0)
						{
							models[i].setImageSize(cv::Size(img.cols/(int)models.size(), img.rows));
						}
						int subWidth = models[i].imageWidth();
						if(offset+subWidth > img.cols)
						{
							UERROR("Multi-camera: sum of sub-image widths (%d) exceeds the stacked "
									"image width (%d) at camera %d.", offset+subWidth, img.cols, (int)i);
							rectified = cv::Mat();
							rectifyAborted = true;
							break;
						}
						if(_rectifyImages)
						{
							if(!models[i].isValidForRectification())
							{
								// In multi-camera mode a valid calibration is expected for each
								// sub-camera. In single-camera mode this is a passthrough (e.g. the
								// base reader of a stereo/RGBD subclass that rectifies itself, or
								// images that are already rectified): skip silently to keep the
								// backward-compatible behavior.
								if(_multiCameraCalib)
								{
									UERROR("Parameter \"rectifyImages\" is set, but camera model %d is not valid for rectification.", (int)i);
								}
								rectified = cv::Mat();
								rectifyAborted = true;
								break;
							}
							if(rectified.empty())
							{
								rectified = cv::Mat::zeros(img.rows, img.cols, img.type());
							}
							cv::Rect roi(offset, 0, subWidth, img.rows);
							models[i].rectifyImage(img(roi)).copyTo(rectified(roi));
						}
						offset += subWidth;
					}
					if(!rectifyAborted && offset != img.cols)
					{
						UWARN("Multi-camera: sum of sub-image widths (%d) does not match the "
								"stacked image width (%d).", offset, img.cols);
					}
					if(!rectified.empty())
					{
						img = rectified;
					}
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
					if(models.empty() || !models.front().isValidForProjection())
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
						// There is no multi-camera version of projectCloudToCamera(): project
						// the scan into each (sub-)camera and stack the registered depth images
						// horizontally, matching the RGB layout (single iteration in single-camera
						// mode). Sub-image widths come from each model's (already set) image size.
						// Holes are filled per sub-image so they don't bleed across cameras.
						depthFromScan = cv::Mat::zeros(img.rows, img.cols, CV_32FC1);
						int offset = 0;
						for(size_t i=0; i<models.size() && offset+models[i].imageWidth()<=img.cols; ++i)
						{
							int subWidth = models[i].imageWidth();
							cv::Mat subDepth = util3d::projectCloudToCamera(cv::Size(subWidth, img.rows), models[i].K(), cloud, models[i].localTransform());
							if(_depthFromScanFillHoles!=0)
							{
								util3d::fillProjectedCloudHoles(subDepth, _depthFromScanFillHoles>0, _depthFromScanFillHolesFromBorder);
							}
							subDepth.copyTo(depthFromScan(cv::Rect(offset, 0, subWidth, img.rows)));
							offset += subWidth;
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

	SensorData data;
	if(!img.empty() || !scan.empty())
	{
		data = SensorData(scan, _isDepth?cv::Mat():img, _isDepth?img:depthFromScan, models, this->getNextSeqID(), stamp);
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
