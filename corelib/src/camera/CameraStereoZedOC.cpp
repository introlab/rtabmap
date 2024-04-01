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

#include <rtabmap/core/camera/CameraStereoZedOC.h>
#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/utilite/UThread.h>
#include <rtabmap/utilite/UEventsManager.h>
#include <rtabmap/utilite/UConversion.h>

#ifdef RTABMAP_ZEDOC
#define VIDEO_MOD_AVAILABLE
#define SENSORS_MOD_AVAILABLE
#include <zed-open-capture/videocapture.hpp>
#include <zed-open-capture/sensorcapture.hpp>
#include "SimpleIni.h"

///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2018, STEREOLABS.
//
// All rights reserved.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////

inline std::vector<std::string> &split(const std::string &s, char delim, std::vector<std::string> &elems) {
    std::stringstream ss(s);
    std::string item;
    while (getline(ss, item, delim)) {
        elems.push_back(item);
    }
    return elems;
}

inline std::vector<std::string> split(const std::string &s, char delim) {
    std::vector<std::string> elems;
    split(s, delim, elems);
    return elems;
}

class ConfManager {
public:

    ConfManager(std::string filename) {
        filename_ = filename;
        ini_.SetUnicode();
        SI_Error rc = ini_.LoadFile(filename_.c_str());
        is_opened_ = !(rc < 0);
    }

    ~ConfManager() {
        //if (is_opened_) ini_.SaveFile(filename_.c_str());
    }

    float getValue(std::string key, float default_value = -1) {
        if (is_opened_) {
            std::vector<std::string> elems;
            split(key, ':', elems);

            return atof(ini_.GetValue(elems.front().c_str(), elems.back().c_str(), std::to_string(default_value).c_str()));
        } else
            return -1.f;
    }

    void setValue(std::string key, float value) {
        if (is_opened_) {
            std::vector<std::string> elems;
            split(key, ':', elems);

            /*SI_Error rc = */ini_.SetValue(elems.front().c_str(), elems.back().c_str(), std::to_string(value).c_str());
        }
    }

    inline bool isOpened() {
        return is_opened_;
    }

private:
    std::string filename_;
    bool is_opened_;
    CSimpleIniA ini_;
};

bool checkFile(std::string path) {
    std::ifstream f(path.c_str());
    return f.good();
}

static inline std::string getRootHiddenDir() {
#ifdef WIN32

#ifdef UNICODE
    wchar_t szPath[MAX_PATH];
#else
    TCHAR szPath[MAX_PATH];
#endif

    if (!SUCCEEDED(SHGetFolderPath(NULL, CSIDL_COMMON_APPDATA, NULL, 0, szPath)))
        return "";

    char snfile_path[MAX_PATH];

#ifndef UNICODE

    size_t newsize = strlen(szPath) + 1;
    wchar_t * wcstring = new wchar_t[newsize];
    // Convert char* string to a wchar_t* string.
    size_t convertedChars = 0;
    mbstowcs_s(&convertedChars, wcstring, newsize, szPath, _TRUNCATE);
    wcstombs(snfile_path, wcstring, MAX_PATH);
#else
    wcstombs(snfile_path, szPath, MAX_PATH);
#endif

    std::string filename(snfile_path);
    filename += "\\Stereolabs\\";

#else //LINUX
    std::string homepath = getenv("HOME");
    std::string filename = homepath + "/zed/";
#endif

    return filename;
}

/*return the path to the Sl ZED hidden dir*/
static inline std::string getHiddenDir() {
    std::string filename = getRootHiddenDir();
#ifdef WIN32
    filename += "settings\\";
#else //LINUX
    filename += "settings/";
#endif
    return filename;
}

bool downloadCalibrationFile(unsigned int serial_number, std::string &calibration_file) {
#ifndef _WIN32
    std::string path = getHiddenDir();
    char specific_name[128];
    sprintf(specific_name, "SN%d.conf", serial_number);
    calibration_file = path + specific_name;
    if (!checkFile(calibration_file)) {
        std::string cmd;
        int res;

        // Create download folder
        cmd = "mkdir -p " + path;
        res = system(cmd.c_str());

        // Download the file
        std::string url("'https://calib.stereolabs.com/?SN=");

        cmd = "wget " + url + std::to_string(serial_number) + "' -O " + calibration_file;
        std::cout << cmd << std::endl;
        res = system(cmd.c_str());

        if( res == EXIT_FAILURE )
        {
            std::cerr << "Error downloading the calibration file" << std::endl;
            return false;
        }

        if (!checkFile(calibration_file)) {
            std::cerr << "Invalid calibration file" << std::endl;
            return false;
        }
    }
#else
    std::string path = getHiddenDir();
    char specific_name[128];
    sprintf(specific_name, "SN%d.conf", serial_number);
    calibration_file = path + specific_name;
    if (!checkFile(calibration_file)) {
        TCHAR *settingFolder = new TCHAR[path.size() + 1];
        settingFolder[path.size()] = 0;
        std::copy(path.begin(), path.end(), settingFolder);
        SHCreateDirectoryEx(NULL, settingFolder, NULL); //recursive creation

        std::string url("https://calib.stereolabs.com/?SN=");
        url += std::to_string(serial_number);
        TCHAR *address = new TCHAR[url.size() + 1];
        address[url.size()] = 0;
        std::copy(url.begin(), url.end(), address);
        TCHAR *calibPath = new TCHAR[calibration_file.size() + 1];
        calibPath[calibration_file.size()] = 0;
        std::copy(calibration_file.begin(), calibration_file.end(), calibPath);

        HRESULT hr = URLDownloadToFile(NULL, address, calibPath, 0, NULL);
        if (hr != 0) {
            std::cout << "Fail to download calibration file" << std::endl;
            return false;
        }

        if (!checkFile(calibration_file)) {
            std::cout << "Invalid calibration file" << std::endl;
            return false;
        }
    }
#endif

    return true;
}

bool initCalibration(std::string calibration_file, cv::Size image_size, rtabmap::StereoCameraModel & model, const rtabmap::Transform & localTransform) {

    if (!checkFile(calibration_file)) {
        std::cout << "Calibration file missing." << std::endl;
        return false;
    }

    // Open camera configuration file
    ConfManager camerareader(calibration_file.c_str());
    if (!camerareader.isOpened())
        return false;

    std::string resolution_str;
    switch ((int) image_size.width) {
        case 2208:
            resolution_str = "2k";
            break;
        case 1920:
            resolution_str = "fhd";
            break;
        case 1280:
            resolution_str = "hd";
            break;
        case 672:
            resolution_str = "vga";
            break;
        default:
            resolution_str = "hd";
            break;
    }

    // Get translations
    float T_[3];
    T_[0] = camerareader.getValue("stereo:baseline", 0.0f);
    T_[1] = camerareader.getValue("stereo:ty_" + resolution_str, 0.f);
    if(T_[1] == 0.f)
    {
    	T_[1] = camerareader.getValue("stereo:ty", 0.f);
    }
    T_[2] = camerareader.getValue("stereo:tz_" + resolution_str, 0.f);
    if(T_[2] == 0.f)
    {
	    T_[2] = camerareader.getValue("stereo:tz", 0.f);
    }

    // Get left parameters
    float left_cam_cx = camerareader.getValue("left_cam_" + resolution_str + ":cx", 0.0f);
    float left_cam_cy = camerareader.getValue("left_cam_" + resolution_str + ":cy", 0.0f);
    float left_cam_fx = camerareader.getValue("left_cam_" + resolution_str + ":fx", 0.0f);
    float left_cam_fy = camerareader.getValue("left_cam_" + resolution_str + ":fy", 0.0f);
    float left_cam_k1 = camerareader.getValue("left_cam_" + resolution_str + ":k1", 0.0f);
    float left_cam_k2 = camerareader.getValue("left_cam_" + resolution_str + ":k2", 0.0f);
    float left_cam_p1 = camerareader.getValue("left_cam_" + resolution_str + ":p1", 0.0f);
    float left_cam_p2 = camerareader.getValue("left_cam_" + resolution_str + ":p2", 0.0f);
    float left_cam_k3 = camerareader.getValue("left_cam_" + resolution_str + ":k3", 0.0f);

    // Get right parameters
    float right_cam_cx = camerareader.getValue("right_cam_" + resolution_str + ":cx", 0.0f);
    float right_cam_cy = camerareader.getValue("right_cam_" + resolution_str + ":cy", 0.0f);
    float right_cam_fx = camerareader.getValue("right_cam_" + resolution_str + ":fx", 0.0f);
    float right_cam_fy = camerareader.getValue("right_cam_" + resolution_str + ":fy", 0.0f);
    float right_cam_k1 = camerareader.getValue("right_cam_" + resolution_str + ":k1", 0.0f);
    float right_cam_k2 = camerareader.getValue("right_cam_" + resolution_str + ":k2", 0.0f);
    float right_cam_p1 = camerareader.getValue("right_cam_" + resolution_str + ":p1", 0.0f);
    float right_cam_p2 = camerareader.getValue("right_cam_" + resolution_str + ":p2", 0.0f);
    float right_cam_k3 = camerareader.getValue("right_cam_" + resolution_str + ":k3", 0.0f);

    // (Linux only) Safety check A: Wrong "." or "," reading in file conf.
#ifndef _WIN32
    if (right_cam_k1 == 0 && left_cam_k1 == 0 && left_cam_k2 == 0 && right_cam_k2 == 0) {
        UERROR("ZED File invalid");

        std::string cmd = "rm " + calibration_file;
        int res = system(cmd.c_str());
        if( res == EXIT_FAILURE )
	   {
		 	return false;
	   }
       	return false;
    }
#endif

    // Get rotations
    cv::Mat R_zed = (cv::Mat_<double>(1, 3) << camerareader.getValue("stereo:rx_" + resolution_str, 0.f), camerareader.getValue("stereo:cv_" + resolution_str, 0.f), camerareader.getValue("stereo:rz_" + resolution_str, 0.f));
    //R_zed *= -1.f; // FIXME: we had to invert T below, do we need to do this with R? I don't see much difference looking at the disparity image
    cv::Mat R;

    cv::Rodrigues(R_zed /*in*/, R /*out*/);

    cv::Mat distCoeffs_left, distCoeffs_right;

    // Left
    cv::Mat cameraMatrix_left = (cv::Mat_<double>(3, 3) << left_cam_fx, 0, left_cam_cx, 0, left_cam_fy, left_cam_cy, 0, 0, 1);
    distCoeffs_left = (cv::Mat_<double>(1, 5) << left_cam_k1, left_cam_k2, left_cam_p1, left_cam_p2, left_cam_k3);

    // Right
    cv::Mat cameraMatrix_right = (cv::Mat_<double>(3, 3) << right_cam_fx, 0, right_cam_cx, 0, right_cam_fy, right_cam_cy, 0, 0, 1);
    distCoeffs_right = (cv::Mat_<double>(1, 5) << right_cam_k1, right_cam_k2, right_cam_p1, right_cam_p2, right_cam_k3);

    // Stereo
    cv::Mat T = (cv::Mat_<double>(3, 1) << T_[0], T_[1], T_[2]);
    T /= -1000.f; // convert in meters, inverted to get positive baseline
    //std::cout << " Camera Matrix L: \n" << cameraMatrix_left << std::endl << std::endl;
    //std::cout << " Camera Matrix R: \n" << cameraMatrix_right << std::endl << std::endl;
    //std::cout << " Camera Rotation: \n" << R << std::endl << std::endl;
    //std::cout << " Camera Translation: \n" << T << std::endl << std::endl;

    cv::Mat R1, R2, P1, P2, Q;
	cv::stereoRectify(cameraMatrix_left, distCoeffs_left, cameraMatrix_right, distCoeffs_right, image_size, R, T,
			R1, R2, P1, P2, Q, cv::CALIB_ZERO_DISPARITY, 0, image_size);

	model = rtabmap::StereoCameraModel("zed",
			image_size, cameraMatrix_left, distCoeffs_left, R1, P1,
			image_size, cameraMatrix_right, distCoeffs_right, R2, P2,
			R, T, cv::Mat(), cv::Mat(), localTransform);
	return true;
}

#endif

namespace rtabmap
{

#ifdef RTABMAP_ZEDOC

class ZedOCThread: public UThread
{
public:
	ZedOCThread(sl_oc::sensors::SensorCapture* sensCap, const Transform & imuLocalTransform)
	{
		sensCap_= sensCap;
		imuLocalTransform_ = imuLocalTransform;
	}

	void getIMU(
			const double & stamp,
			IMU & imu,
			int maxWaitTimeMs)
	{
		imu = IMU();
		if(imuBuffer_.empty())
		{
			return;
		}

		// Interpolate imu
		cv::Vec3d acc;
		cv::Vec3d gyr;

		int waitTry = 0;
		imuMutex_.lock();
		while(maxWaitTimeMs > 0 && imuBuffer_.rbegin()->first < stamp && waitTry < maxWaitTimeMs)
		{
			imuMutex_.unlock();
			++waitTry;
			uSleep(1);
			imuMutex_.lock();
		}
		bool set = false;
		if(imuBuffer_.rbegin()->first < stamp)
		{
			if(maxWaitTimeMs>0)
			{
				UWARN("Could not find imu data to interpolate at image time %f after waiting %d ms (last is %f)...", stamp, maxWaitTimeMs, imuBuffer_.rbegin()->first);
			}
		}
		else
		{
			std::map<double, std::pair<cv::Vec3f, cv::Vec3f> >::const_iterator iterB = imuBuffer_.lower_bound(stamp);
			std::map<double, std::pair<cv::Vec3f, cv::Vec3f> >::const_iterator iterA = iterB;
			if(iterA != imuBuffer_.begin())
			{
				iterA = --iterA;
			}
			if(iterB == imuBuffer_.end())
			{
				iterB = --iterB;
			}
			if(iterA == iterB && stamp == iterA->first)
			{
				acc[0] = iterA->second.first[0];
				acc[1] = iterA->second.first[1];
				acc[2] = iterA->second.first[2];
				gyr[0] = iterA->second.second[0];
				gyr[1] = iterA->second.second[1];
				gyr[2] = iterA->second.second[2];
				set = true;
			}
			else if(stamp >= iterA->first && stamp <= iterB->first)
			{
				float t = (stamp-iterA->first) / (iterB->first-iterA->first);
				acc[0] = iterA->second.first[0] + t*(iterB->second.first[0] - iterA->second.first[0]);
				acc[1] = iterA->second.first[1] + t*(iterB->second.first[1] - iterA->second.first[1]);
				acc[2] = iterA->second.first[2] + t*(iterB->second.first[2] - iterA->second.first[2]);
				gyr[0] = iterA->second.second[0] + t*(iterB->second.second[0] - iterA->second.second[0]);
				gyr[1] = iterA->second.second[1] + t*(iterB->second.second[1] - iterA->second.second[1]);
				gyr[2] = iterA->second.second[2] + t*(iterB->second.second[2] - iterA->second.second[2]);
				set = true;
			}
			else
			{
				if(stamp < iterA->first)
				{
					UDEBUG("Could not find imu data to interpolate at image time %f (earliest is %f). Are sensors synchronized? (may take some time to be synchronized...)", stamp, iterA->first);
				}
				else
				{
					UDEBUG("Could not find imu data to interpolate at image time %f (between %f and %f). Are sensors synchronized? (may take some time to be synchronized...)", stamp, iterA->first, iterB->first);
				}
			}
		}
		imuMutex_.unlock();

		if(set)
		{
			imu = IMU(gyr, cv::Mat::eye(3, 3, CV_64FC1),
					acc, cv::Mat::eye(3, 3, CV_64FC1),
					imuLocalTransform_);
		}

		return;
	}
private:
	virtual void mainLoop()
	{
		// ----> Get IMU data
		const sl_oc::sensors::data::Imu imuData = sensCap_->getLastIMUData(2000);

		// Process data only if valid
		if(imuData.valid == sl_oc::sensors::data::Imu::NEW_VAL ) // Uncomment to use only data syncronized with the video frames
		{
			UScopeMutex sm(imuMutex_);
			static double deg2rad = 0.017453293;
			std::pair<cv::Vec3d, cv::Vec3d> imu(
					cv::Vec3d(imuData.aX, imuData.aY, imuData.aZ),
					cv::Vec3d(imuData.gX*deg2rad, imuData.gY*deg2rad, imuData.gZ*deg2rad));

			double stamp = double(imuData.timestamp)/10e8;
			if(!imuBuffer_.empty() && imuBuffer_.rbegin()->first > stamp)
			{
				UWARN("IMU data not received in order, reset buffer! (previous=%f new=%f)", imuBuffer_.rbegin()->first, stamp);
				imuBuffer_.clear();
			}
			imuBuffer_.insert(imuBuffer_.end(), std::make_pair(stamp, imu));
			if(imuBuffer_.size() > 1000)
			{
				imuBuffer_.erase(imuBuffer_.begin());
			}
		}

	}
	sl_oc::sensors::SensorCapture* sensCap_;
	Transform imuLocalTransform_;
	UMutex imuMutex_;
	std::map<double, std::pair<cv::Vec3f, cv::Vec3f> > imuBuffer_;
};

#endif

bool CameraStereoZedOC::available()
{
#ifdef RTABMAP_ZEDOC
	return true;
#else
	return false;
#endif
}

CameraStereoZedOC::CameraStereoZedOC(
		int deviceId,
		int resolution,
		float imageRate,
		const Transform & localTransform) :
	Camera(imageRate, localTransform)
#ifdef RTABMAP_ZEDOC
    ,
	zed_(0),
	sensors_(0),
	imuThread_(0),
	usbDevice_(deviceId),
	resolution_(resolution),
	lastStamp_(0)
#endif
{
	UDEBUG("");
#ifdef RTABMAP_ZEDOC

	sl_oc::video::RESOLUTION res = static_cast<sl_oc::video::RESOLUTION>(resolution_);
    UASSERT(res >= sl_oc::video::RESOLUTION::HD2K && res < sl_oc::video::RESOLUTION::LAST);
#endif
}

CameraStereoZedOC::~CameraStereoZedOC()
{
#ifdef RTABMAP_ZEDOC
	if(imuThread_)
	{
		imuThread_->join(true);
		delete imuThread_;
	}
	delete zed_;
	delete sensors_;
#endif
}

bool CameraStereoZedOC::init(const std::string & calibrationFolder, const std::string & cameraName)
{
	UDEBUG("");
#ifdef RTABMAP_ZEDOC
	if(imuThread_)
	{
		imuThread_->join(true);
		delete imuThread_;
		imuThread_=0;
	}
	if(zed_)
	{
		delete zed_;
		zed_ = 0;
	}
	if(sensors_)
	{
		delete sensors_;
		sensors_ = 0;
	}
	lastStamp_ = 0;

	// ----> Set Video parameters
	sl_oc::video::VideoParams params;
	params.res = static_cast<sl_oc::video::RESOLUTION>(resolution_);

	params.fps = sl_oc::video::FPS::FPS_15;
	if(this->getImageRate() > 60)
	{
		params.fps = sl_oc::video::FPS::FPS_100;
	}
	else if(this->getImageRate() > 30)
	{
		params.fps = sl_oc::video::FPS::FPS_60;
	}
	else if(this->getImageRate() > 15)
	{
		params.fps = sl_oc::video::FPS::FPS_30;
	}

	if(ULogger::level() <= ULogger::kInfo)
	{
		params.verbose = sl_oc::VERBOSITY::INFO;
	}
	else if(ULogger::level() <= ULogger::kWarning)
	{
		params.verbose = sl_oc::VERBOSITY::WARNING;
	}
	// <---- Set Video parameters

	// ----> Create Video Capture
	zed_ = new sl_oc::video::VideoCapture(params);
	if( !zed_->initializeVideo(usbDevice_) )
	{
		UERROR("Cannot open camera video capture. Set log level <= info for more details.");

		delete zed_;
		zed_ = 0;
		return false;
	}
	int sn = zed_->getSerialNumber();
	UINFO("Connected to camera sn: %d", sn);
	// <---- Create Video Capture

	// ----> Retrieve calibration file from Stereolabs server
	std::string calibration_file;
	// ZED Calibration
	unsigned int serial_number = sn;
	// Download camera calibration file
	if( !downloadCalibrationFile(serial_number, calibration_file) )
	{
		UERROR("Could not load calibration file from Stereolabs servers");
		delete zed_;
		zed_ = 0;
		return false;
	}
	UINFO("Calibration file found. Loading...");

	// ----> Frame size
	int w,h;
	zed_->getFrameSize(w,h);
	// <---- Frame size

	 // ----> Initialize calibration
	if(initCalibration(calibration_file, cv::Size(w/2,h), stereoModel_, this->getLocalTransform()))
	{
		if(ULogger::level() <= ULogger::kInfo)
		{
			std::cout << "Calibration left:" << std::endl << stereoModel_.left() << std::endl;
			std::cout << "Calibration right:" << std::endl << stereoModel_.right() << std::endl;
		}
		stereoModel_.initRectificationMap();
	}

	// ----> Create a Sensors Capture object
	sensors_ = new sl_oc::sensors::SensorCapture((sl_oc::VERBOSITY)params.verbose);
	if( !sensors_->initializeSensors(serial_number) ) // Note: we use the serial number acquired by the VideoCapture object
	{
		UERROR("Cannot open sensors capture. Set log level <= info for more details.");
		delete sensors_;
		sensors_ = 0;
	}
	else
	{
		UINFO("Sensors Capture connected to camera sn: %d", sensors_->getSerialNumber());
		UINFO("Wait max 5 sec to see if the camera has imu...");
		// Check is IMU data is available
		UTimer timer;
		while(timer.elapsed() < 5 &&
			  sensors_->getLastIMUData().valid != sl_oc::sensors::data::Imu::NEW_VAL)
		{
			// wait 5 sec to see if we can get an imu stream...
			uSleep(100);
		}
		if(timer.elapsed() > 5)
		{
			UINFO("Camera doesn't have IMU sensor");
		}
		else
		{
			UINFO("Camera has IMU");

			// Start the sensor capture thread. Note: since sensor data can be retrieved at 400Hz and video data frequency is
			// minor (max 100Hz), we use a separated thread for sensors.
			// Transform based on ZED2: x->down, y->right, z->backward
			Transform imuLocalTransform_ = this->getLocalTransform() * Transform(0,1,0,0, 1,0,0,0, 0,0,-1,0);
			//std::cout << imuLocalTransform_ << std::endl;
			imuThread_ = new ZedOCThread(sensors_, imuLocalTransform_);
			// <---- Create Sensors Capture

			// ----> Enable video/sensors synchronization
			if(!zed_->enableSensorSync(sensors_))
			{
				UWARN("Failed to enable image/imu synchronization");
			}
			// <---- Enable video/sensors synchronization

			imuThread_->start();
		}
	}

	return true;
#else
	UERROR("CameraStereoZEDOC: RTAB-Map is not built with ZED Open Capture support!");
#endif
	return false;
}

bool CameraStereoZedOC::isCalibrated() const
{
#ifdef RTABMAP_ZEDOC
	return stereoModel_.isValidForProjection();
#else
	return false;
#endif
}

std::string CameraStereoZedOC::getSerial() const
{
#ifdef RTABMAP_ZEDOC
	if(zed_)
	{
		return uFormat("%x", zed_->getSerialNumber());
	}
#endif
	return "";
}

SensorData CameraStereoZedOC::captureImage(CameraInfo * info)
{
	SensorData data;
#ifdef RTABMAP_ZEDOC
	// Get a new frame from camera
	if(zed_)
	{
		UTimer timer;

		bool imuReceived = imuThread_!=0;
		bool warned = false;
		do
		{
			const sl_oc::video::Frame frame = zed_->getLastFrame();

			if(frame.data!=nullptr && frame.timestamp!=lastStamp_)
			{
				lastStamp_ = frame.timestamp;

				double stamp = double(lastStamp_)/10e8;

				// If the sensor supports IMU, wait IMU to be available before sending data.
				IMU imu;
				if(imuThread_)
				{
					imuThread_->getIMU(stamp, imu, 10);
					imuReceived = !imu.empty();
					if(!imuReceived && !warned && timer.elapsed() > 1.0)
					{
						UWARN("Waiting for synchronized imu (this can take several seconds when camera has been just started)...");
						warned = true;
					}
				}

				if(imuReceived)
				{
					cv::Mat frameBGR, left, right;

					// ----> Conversion from YUV 4:2:2 to BGR for visualization
					cv::Mat frameYUV = cv::Mat( frame.height, frame.width, CV_8UC2, frame.data );
					cv::cvtColor(frameYUV,frameBGR,cv::COLOR_YUV2BGR_YUYV);
					// <---- Conversion from YUV 4:2:2 to BGR for visualization

					// ----> Extract left and right images from side-by-side
					left = frameBGR(cv::Rect(0, 0, frameBGR.cols / 2, frameBGR.rows));
					cv::cvtColor(frameBGR(cv::Rect(frameBGR.cols / 2, 0, frameBGR.cols / 2, frameBGR.rows)),right,cv::COLOR_BGR2GRAY);
					// <---- Extract left and right images from side-by-side

					if(stereoModel_.isValidForRectification())
					{
						left = stereoModel_.left().rectifyImage(left);
						right = stereoModel_.right().rectifyImage(right);
					}

					data = SensorData(left, right, stereoModel_, this->getNextSeqID(), stamp);

					if(!imu.empty())
					{
						data.setIMU(imu);
					}
				}
			}
			else
			{
				UERROR("CameraStereoZEDOC: Cannot get frame from the camera since 100 msec!");
				imuReceived = true;
			}
		}
		while(!imuReceived && timer.elapsed() < 15.0);

		// ----> If the frame is valid we can convert, rectify and display it
		if(!imuReceived)
		{
			UERROR("CameraStereoZEDOC: Cannot get synchronized IMU with camera for 15 sec!");
		}
	}
#else
	UERROR("CameraStereoZEDOC: RTAB-Map is not built with ZED Open Capture support!");
#endif
	return data;
}

} // namespace rtabmap
