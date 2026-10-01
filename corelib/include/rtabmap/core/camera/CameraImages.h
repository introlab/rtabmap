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

#pragma once

#include "rtabmap/core/Camera.h"
#include "rtabmap/utilite/UTimer.h"
#include <list>

class UDirectory;

namespace rtabmap
{

class RTABMAP_CORE_EXPORT CameraImages :
	public Camera
{
public:
	CameraImages();
	CameraImages(
			const std::string & path,
			float imageRate = 0,
			const Transform & localTransform = Transform::getIdentity());
	virtual ~CameraImages();

	virtual bool init(const std::string & calibrationFolder = ".", const std::string & cameraName = "");
	virtual bool isCalibrated() const;
	virtual std::string getSerial() const;
	virtual bool odomProvided() const { return odometry_.size() > 0; }
	std::string getPath() const {return _path;}
	unsigned int imagesCount() const;
	std::vector<std::string> filenames() const;
	bool isImagesRectified() const {return _rectifyImages;}
	int getBayerMode() const {return _bayerMode;}
	const CameraModel & cameraModel() const {return _model;}

	void setPath(const std::string & dir) {_path=dir;}
	virtual void setStartIndex(int index) {_startAt = index;} // negative means last
	virtual void setMaxFrames(int value) {_maxFrames = value;}
	void setDirRefreshed(bool enabled) {_refreshDir = enabled;}
	void setImagesRectified(bool enabled) {_rectifyImages = enabled;}
	void setBayerMode(int mode) {_bayerMode = mode;} // -1=disabled (default) 0=BayerBG, 1=BayerGB, 2=BayerRG, 3=BayerGR

	void setTimestamps(bool fileNamesAreStamps, const std::string & filePath = "", bool syncImageRateWithStamps=true)
	{
		_filenamesAreTimestamps = fileNamesAreStamps;
		_timestampsPath=filePath;
		_syncImageRateWithStamps = syncImageRateWithStamps;
	}

	void setConfigForEachFrame(bool value)
	{
		_hasConfigForEachFrame = value;
	}
	bool isConfigForEachFrame() const {return _hasConfigForEachFrame;}

	// Enable multi-camera mode. Each image in the folder is expected to be the
	// horizontal concatenation of N sub-camera images of a rig, sharing the same
	// base name (timestamp or node id), e.g. "1780687370.031791.jpg" or "1.jpg".
	// One calibration file per sub-camera must exist in the calibrationFolder
	// passed to init(), named "<prefix>_<index>.yaml" with index starting at 0
	// (e.g. "1_0.yaml", "1_1.yaml", ...). The number of cameras is auto-detected.
	// Each sub-image width is taken from the corresponding model's calibrated image
	// size; if a model has no size, a uniform split (stackedWidth / N) is assumed.
	// If setConfigForEachFrame() is enabled, one calibration set is loaded per frame
	// using each image's base name as prefix. Otherwise a single calibration set is
	// loaded and reused for all frames, using the cameraName passed to init() as
	// prefix (it may differ from any image name), falling back to the first image's
	// base name when cameraName is empty.
	// Note that it is not recommended to use setConfigForEachFrame() if images have
	// to be rectified, a rectification matrix would need to be re-initilaized for
	// each frame.
	void setMultiCameraCalibration(bool enabled)
	{
		_multiCameraCalib = enabled;
	}

	void setScanPath(
			const std::string & dir,
			int maxScanPts = 0,
			const Transform & localTransform=Transform::getIdentity())
	{
		_scanPath = dir;
		_scanLocalTransform = localTransform;
		_scanMaxPts = maxScanPts;
	}

	void setDepthFromScan(bool enabled, int fillHoles = 1, bool fillHolesFromBorder = false)
	{
		_depthFromScan = enabled;
		_depthFromScanFillHoles = fillHoles;
		_depthFromScanFillHolesFromBorder = fillHolesFromBorder;
	}

	// 0=Raw, 1=RGBD-SLAM motion capture (10=without change of coordinate frame, 11=10+ID), 2=KITTI, 3=TORO, 4=g2o, 5=NewCollege(t,x,y), 6=Malaga Urban GPS, 7=St Lucia INS, 8=Karlsruhe, 9=EuRoC MAV, 12=rgbd_bonn
	void setOdometryPath(const std::string & filePath, int format = 0)
	{
		_odometryPath = filePath;
		_odometryFormat = format;
	}

	// 0=Raw, 1=RGBD-SLAM motion capture (10=without change of coordinate frame, 11=10+ID), 2=KITTI, 3=TORO, 4=g2o, 5=NewCollege(t,x,y), 6=Malaga Urban GPS, 7=St Lucia INS, 8=Karlsruhe, 9=EuRoC MAV, 12=rgbd_bonn
	void setGroundTruthPath(const std::string & filePath, int format = 0, const Transform & localTransform = Transform::getIdentity())
	{
		_groundTruthPath = filePath;
		_groundTruthFormat = format;
		_groundTruthLocalTransform = localTransform;
	}

	void setMaxPoseTimeDiff(double diff) {_maxPoseTimeDiff = diff;}
	double getMaxPoseTimeDiff() const {return _maxPoseTimeDiff;}

	void setDepth(bool isDepth, float depthScaleFactor = 1.0f)
	{
		_isDepth = isDepth;
		_depthScaleFactor=depthScaleFactor;
	}

protected:
	virtual SensorData captureImage(SensorCaptureInfo * info = 0);

	// File name (with extension, no directory) of the image returned by the last
	// captureImage() call. Used by subclasses (e.g. stereo) to key per-frame
	// calibration loaded on demand. Empty if no image was read.
	const std::string & lastImageFileName() const {return _lastImageFileName;}

	// Calibration folder passed to init(), kept to load per-frame calibration on
	// demand. Shared with subclasses (e.g. stereo) that load calibration themselves.
	std::string _calibrationFolder;

	// Multi-camera mode state, shared with subclasses (e.g. stereo) that layer their
	// own multi-camera handling over the base reader. Such subclasses temporarily set
	// _multiCameraCalib to false while delegating to base init()/captureImage() so the
	// base returns the raw stacked image instead of doing its own split, then restore it.
	bool _multiCameraCalib;
	int _multiCameraCount; // number of sub-cameras detected in multi-camera mode

private:
	bool readPoses(
		std::list<Transform> & outputPoses,
		std::list<double> & stamps,
		const std::string & filePath,
		int format,
		double maxTimeDiff) const;

	// Load the sub-camera models of a multi-camera rig from calibration files named
	// "<baseName>_<index>.yaml" (index 0.._multiCameraCount-1) in _calibrationFolder.
	// Returns an empty vector (and logs an error) if any model is missing or invalid.
	std::vector<CameraModel> loadMultiCameraModels(const std::string & baseName) const;

	// Load the single-camera model for one frame from a per-frame config file (RTAB-Map
	// calibration or 3DScannerApp format). Returns an invalid model (and logs an error)
	// on failure.
	CameraModel loadConfigModel(const std::string & filePath);

private:
	std::string _path;
	int _startAt;
	int _maxFrames;
	// If the list of files in the directory is refreshed
	// on each call of takeImage()
	bool _refreshDir;
	bool _rectifyImages;
	int _bayerMode;
	bool _isDepth;
	float _depthScaleFactor;
	int _count;
	int _framesPublished;
	UDirectory * _dir;
	std::string _lastFileName;
	std::string _lastImageFileName; // file name of the image returned by the last captureImage()

	int _countScan;
	UDirectory * _scanDir;
	std::string _lastScanFileName;
	std::string _scanPath;
	Transform _scanLocalTransform;
	int _scanMaxPts;

	bool _depthFromScan;
	int _depthFromScanFillHoles; // <0:horizontal 0:disabled >0:vertical
	bool _depthFromScanFillHolesFromBorder;

	bool _filenamesAreTimestamps;
	bool _hasConfigForEachFrame;
	std::string _timestampsPath;
	bool _syncImageRateWithStamps;

	std::string _odometryPath;
	int _odometryFormat;
	std::string _groundTruthPath;
	int _groundTruthFormat;
	Transform _groundTruthLocalTransform;
	double _maxPoseTimeDiff;

	std::list<double> _stamps;
	std::list<Transform> odometry_;
	std::list<cv::Mat> covariances_;
	std::list<Transform> groundTruth_;
	CameraModel _model;
	std::list<std::string> _modelFileNames; // per-frame single-camera config file paths (config-for-each-frame)
	bool _configLocalTransformWarned; // warn only once when a per-frame config has no local_transform
	std::vector<CameraModel> _multiModels; // sub-camera models, shared by all frames (multi-camera mode); empty when loaded per-frame

	UTimer _captureTimer;
	double _captureDelay;
};


} // namespace rtabmap
