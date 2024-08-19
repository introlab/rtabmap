/*
Copyright (c) 2010-2024, Mathieu Labbe
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the names of its contributors may be used to endorse or promote products
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

#include <rtabmap/core/lidar/LidarOuster.h>
#include <rtabmap/utilite/UEventsManager.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/core/IMUFilter.h>
#include <rtabmap/utilite/UTimer.h>

#ifdef RTABMAP_OUSTER
#include "ouster/client.h"
#include "ouster/os_pcap.h"
#include "ouster/types.h"
#include "ouster/impl/build.h"
#include "ouster/lidar_scan.h"
#endif

namespace rtabmap {

#ifdef RTABMAP_OUSTER
class OusterCaptureThread : public UThread {
  public:
	OusterCaptureThread(
		std::shared_ptr<ouster::sensor::client> & clientHandle,
		const ouster::sensor::sensor_info & info,
		bool imuPublished,
		bool useReflectivityForIntensityChannel) :
			scanBatcher_(info),
			pf_(ouster::sensor::get_format(info)),
			clientHandle_(clientHandle),
			info_(info),
			intensityChannel_(useReflectivityForIntensityChannel?
				ouster::sensor::ChanField::REFLECTIVITY:
				ouster::sensor::ChanField::SIGNAL),
		sleepTimeMs_(0)
	{
		UDEBUG("");
		init(imuPublished);
	}
	OusterCaptureThread(
		std::shared_ptr<ouster::sensor_utils::playback_handle> & pcapHandle,
		const ouster::sensor::sensor_info & info,
		bool imuPublished,
		bool useReflectivityForIntensityChannel) :
			scanBatcher_(info),
			pf_(ouster::sensor::get_format(info)),
			pcapHandle_(pcapHandle),
			info_(info),
			intensityChannel_(useReflectivityForIntensityChannel?
				ouster::sensor::ChanField::REFLECTIVITY:
				ouster::sensor::ChanField::SIGNAL),
			sleepTimeMs_(100)
	{
		UDEBUG("");
		std::string lidarMode = ouster::sensor::to_string(info_.config.lidar_mode.value_or(ouster::sensor::lidar_mode::MODE_UNSPEC));
		std::list<std::string> lidarModeSplit = uSplit(lidarMode, 'x');
		if(lidarModeSplit.size() == 2) {
			int rate = uStr2Int(*lidarModeSplit.rbegin());
			UASSERT(rate > 0);
			sleepTimeMs_ = 1000/rate;
			UINFO("Setting frame rate to %d Hz (lidar mode = %s) wil sleep %ld ms after each frame", rate, lidarMode.c_str(), sleepTimeMs_);
		}

		init(imuPublished);
	}

	virtual ~OusterCaptureThread()
	{
		this->join(true);
		delete imuFilter_;
	}

	const ouster::sensor::sensor_info & getInfo() const {
		return info_;
	}

	bool getScan(LaserScan & output, double & stamp)
	{
		if(this->isRunning())
		{
			if(!scanReady_.acquire(1, 5000))
			{
				UERROR("Not received any frames since 5 seconds, try to restart the camera again.");
			}
			else if(this->isRunning())
			{
				ouster::LidarScan scan;
				{
					UScopeMutex s(scanMutex_);
					scan = lastScan_;
					lastScan_ = ouster::LidarScan();
				}

				UASSERT(scan.has_field(ouster::sensor::ChanField::RANGE));
				UASSERT(scan.has_field(intensityChannel_));

				ouster::LidarScan::Points cloud = ouster::cartesian(scan, lut_);

				// Convert to our LaserScan format XYZIT
				output = LaserScan(cv::Mat(cv::Size(scan.w,scan.h), CV_32FC(5)), LaserScan::kXYZIT, 0, 0, 0, 0, 0, lidarSensorT_);

				auto timestamp = scan.timestamp();
				auto scan_ts = timestamp[scan.w-1]; // stamp with last reading

				//UWARN("Prepare scan %f -> %f", double(timestamp[0])/10e8, double(timestamp[scan.w-1])/10e8);

				auto intensityOffset = output.getIntensityOffset();
				auto timeOffset = output.getTimeOffset();

				float bad_point = std::numeric_limits<float>::quiet_NaN ();
				auto intensity = scan.field<uint32_t>(intensityChannel_);
				for (size_t u = 0; u < scan.h; u++) {
					for (size_t v = 0; v < scan.w; v++) {
						auto ts = scan_ts-timestamp[v];
						auto index = u*scan.w+v;
						if(cloud(index, 0) != 0.0)
						{
							output.field(index, 0) = cloud(index, 0);
							output.field(index, 1) = cloud(index, 1);
							output.field(index, 2) = cloud(index, 2);
						}
						else
						{
							output.field(index, 0) = output.field(index, 1) = output.field(index, 2) = bad_point;
						}
						output.field(index, intensityOffset) = intensity.data()[index];
						output.field(index, timeOffset) = -float(ts)/10e8;
					}
				}

				stamp = double(scan_ts)/10e8;
				return true;
			}
		}
		return false;
	}

	IMU getIMU(double stamp, int maxWaitTimeMs = 100)
	{
		IMU imu;
		if(!imuBuffer_.empty())
		{
			imuMutex_.lock();
			int waitTry = 0;
			while(maxWaitTimeMs>0 && imuBuffer_.rbegin()->first < stamp && waitTry < maxWaitTimeMs)
			{
				imuMutex_.unlock();
				++waitTry;
				uSleep(1);
				imuMutex_.lock();
			}

			if(imuBuffer_.rbegin()->first < stamp)
			{
				if(maxWaitTimeMs > 0)
				{
					UWARN("Could not find imus to interpolate at scan time %f after waiting %d ms (last is %f)...", stamp/1000.0, maxWaitTimeMs, imuBuffer_.rbegin()->first/1000.0);
				}
			}
			else
			{
				std::map<double, IMU>::const_iterator iterB = imuBuffer_.lower_bound(stamp);
				std::map<double, IMU>::const_iterator iterA = iterB;
				if(iterA != imuBuffer_.begin())
				{
					iterA = --iterA;
				}
				if(iterB == imuBuffer_.end())
				{
					iterB = --iterB;
				}
				if(stamp >= iterA->first && stamp <= iterB->first)
				{
					float t = (stamp-iterA->first) / (iterB->first-iterA->first);

					cv::Vec4d quatA = iterA->second.orientation();
					cv::Vec3d accA = iterA->second.linearAcceleration();
					cv::Vec3d gyroA = iterA->second.angularVelocity();
					cv::Vec4d quatB = iterB->second.orientation();
					cv::Vec3d accB = iterB->second.linearAcceleration();
					cv::Vec3d gyroB = iterB->second.angularVelocity();

					Eigen::Quaterniond qa(quatA[3], quatA[0], quatA[1], quatA[2]);
					Eigen::Quaterniond qb(quatB[3], quatB[0], quatB[1], quatB[2]);
					Eigen::Quaterniond qres = qa.slerp(t, qb);

					accA[0] = accA[0] + t*(accB[0] - accA[0]);
					accA[1] = accA[1] + t*(accB[1] - accA[1]);
					accA[2] = accA[2] + t*(accB[2] - accA[2]);

					gyroA[0] = gyroA[0] + t*(gyroB[0] - gyroA[0]);
					gyroA[1] = gyroA[1] + t*(gyroB[1] - gyroA[1]);
					gyroA[2] = gyroA[2] + t*(gyroB[2] - gyroA[2]);

					imu = IMU(cv::Vec4d(qres.x(), qres.y(), qres.z(), qres.w()), iterA->second.orientationCovariance(),
							gyroA, iterA->second.angularVelocityCovariance(),
							accA, iterA->second.linearAccelerationCovariance(),
							imuLocalTransform_);
				}
			}
			imuMutex_.unlock();
		}
		return imu;
	}

private:
	void init(bool imuPublished)
	{
		imuFilter_ = 0;
		lastSleepStamp_ = UTimer::now();
		
		UINFO("imuPublished=%d", imuPublished?1:0);
		size_t w = info_.format.columns_per_frame;
		size_t h = info_.format.pixels_per_column;

		lut_ = ouster::make_xyz_lut(info_);

		// Buffer to store raw packet data
		lidarPacket_ = ouster::sensor::LidarPacket (pf_.lidar_packet_size);
		imuPacket_ = ouster::sensor::ImuPacket(pf_.imu_packet_size);

		const std::array<ouster::FieldType, 2> reducedSlots{
			{{ouster::sensor::ChanField::RANGE, ouster::sensor::ChanFieldType::UINT32},
			{intensityChannel_, ouster::sensor::ChanFieldType::UINT32}}};
		scanBuffer_ = ouster::LidarScan(w, h, reducedSlots.begin(), reducedSlots.end());

		lidarSensorT_ = Transform::fromEigen4d(info_.lidar_to_sensor_transform);
		lidarSensorT_.x() /= 1000;
		lidarSensorT_.y() /= 1000;
		lidarSensorT_.z() /= 1000;
		UINFO("Lidar to sensor: %s", lidarSensorT_.prettyPrint().c_str());

		if(imuPublished)
		{
			imuLocalTransform_ = Transform::fromEigen4d(info_.imu_to_sensor_transform);
			imuLocalTransform_.x() /= 1000;
			imuLocalTransform_.y() /= 1000;
			imuLocalTransform_.z() /= 1000;
			UINFO("IMU to sensor: %s", imuLocalTransform_.prettyPrint().c_str());

			imuFilter_ = IMUFilter::create(IMUFilter::kMadgwick);
		}
	}

	virtual void mainLoop()
	{
		bool isLidarPacket = false;
		bool isImuPacket = false;
		
		if(clientHandle_)
		{
			// wait until sensor data is available
			ouster::sensor::client_state st = ouster::sensor::poll_client(*clientHandle_);
			// check for timeout
			if (st == ouster::sensor::TIMEOUT){
				UERROR("Client has timed out");
				this->kill();
				return;
			}

			if (st & ouster::sensor::EXIT){
				UERROR("Exit was requested");
				this->kill();
				return;
			}

			// check for error status
			if (st & ouster::sensor::CLIENT_ERROR) {
				UERROR("Sensor client returned error state!");
				this->kill();
				return;
			}

			if (st & ouster::sensor::LIDAR_DATA) {
				if (!ouster::sensor::read_lidar_packet(*clientHandle_, lidarPacket_)) {
					UERROR("Failed to read a lidar packet of the expected size!");
					this->kill();
					return;
				}
				isLidarPacket = true;
			}

			if (imuFilter_ && st & ouster::sensor::IMU_DATA) {
				if (!ouster::sensor::read_imu_packet(*clientHandle_, imuPacket_)) {
					UERROR("Failed to read an imu packet of the expected size!");
					this->kill();
					return;
				}
				isImuPacket = true;
			}
		}
		else if(pcapHandle_) //pcap
		{
			ouster::sensor_utils::packet_info packet_info;
			if(!ouster::sensor_utils::next_packet_info(*pcapHandle_, packet_info))
			{
				UWARN("No more data");
				this->kill();
				return;
			}

			if(packet_info.dst_port == info_.config.udp_port_lidar)
			{
				auto packet_size = ouster::sensor_utils::read_packet(
					*pcapHandle_, lidarPacket_.buf.data(), lidarPacket_.buf.size());

				if (packet_size == pf_.lidar_packet_size) {
					isLidarPacket = true;
				}
			}
			else if(imuFilter_ && packet_info.dst_port == info_.config.udp_port_imu)
			{
				// async IMU, can be used for deskewing in Odometry
				auto packet_size = ouster::sensor_utils::read_packet(
					*pcapHandle_, imuPacket_.buf.data(), imuPacket_.buf.size());
				if (packet_size == pf_.imu_packet_size) {
					isImuPacket = true;
				}
			}
		}
		else {
			UFATAL("");
		}

		if (isLidarPacket) {
			// batcher will return "true" when the current scan is complete
            if (scanBatcher_(lidarPacket_, scanBuffer_)) {
                // retry until we receive a full set of valid measurements
                // (accounting for azimuth_window settings if any)
                if (scanBuffer_.complete(info_.format.column_window))
				{
					{
						UScopeMutex s(scanMutex_);
						bool notify = lastScan_.w == 0;
						lastScan_ = scanBuffer_;
						if(lastScan_.w != 0 && notify)
						{
							scanReady_.release();
						}
					}
					if(sleepTimeMs_>0) {
						unsigned int deltaMs = int((UTimer::now() - lastSleepStamp_)*1000);
						if(deltaMs < sleepTimeMs_) {
							uSleep(sleepTimeMs_-deltaMs);
						}
						lastSleepStamp_ = UTimer::now();
					}
                }
            }
		}

		// async IMU, can be used for deskewing in Odometry
		if (isImuPacket) {
			static const double standard_g = 9.80665;
			uchar* buf = imuPacket_.buf.data();
			uint64_t ts = pf_.imu_gyro_ts(buf);
			double imuStamp = double(ts)/10e8;
			cv::Vec3d linearAcc;
			cv::Vec3d angularVel;
			linearAcc[0] = pf_.imu_la_x(buf) * standard_g;
			linearAcc[1] = pf_.imu_la_y(buf) * standard_g;
			linearAcc[2] = pf_.imu_la_z(buf) * standard_g;
			angularVel[0] = pf_.imu_av_x(buf) * M_PI / 180.0;
			angularVel[1] = pf_.imu_av_y(buf) * M_PI / 180.0;
			angularVel[2] = pf_.imu_av_z(buf) * M_PI / 180.0;
			imuFilter_->update(
				angularVel[0], angularVel[1], angularVel[2],
				linearAcc[0], linearAcc[1], linearAcc[2],
				imuStamp);
			cv::Vec4d quat;
			imuFilter_->getOrientation(quat[0], quat[1], quat[2], quat[3]);
			IMU imu(quat, cv::Mat::eye(3,3,CV_64FC1)*6e-4,
					angularVel, cv::Mat::eye(3,3,CV_64FC1)*6e-4,
					linearAcc, cv::Mat::eye(3,3,CV_64FC1)*0.01,
					imuLocalTransform_);
			UEventsManager::post(new IMUEvent(imu, imuStamp));
			UScopeMutex s(imuMutex_);
			imuBuffer_.insert(imuBuffer_.end(), std::make_pair(imuStamp, imu));
			if(imuBuffer_.size() > 1000)
			{
				imuBuffer_.erase(imuBuffer_.begin());
			}
		}
	}

	virtual void mainLoopEnd()
	{
		scanReady_.release();
	}

  private:
	UMutex scanMutex_;
	UMutex imuMutex_;
	USemaphore scanReady_;
	ouster::ScanBatcher scanBatcher_;
	ouster::sensor::packet_format pf_;
	ouster::LidarScan scanBuffer_;
	ouster::LidarScan lastScan_;
	ouster::sensor::LidarPacket lidarPacket_;
	ouster::sensor::ImuPacket imuPacket_;
	std::shared_ptr<ouster::sensor::client> clientHandle_;
	std::shared_ptr<ouster::sensor_utils::playback_handle> pcapHandle_;
	ouster::sensor::sensor_info info_;
	ouster::sensor::cf_type intensityChannel_;
	ouster::XYZLut lut_;
	Transform lidarSensorT_;
	Transform imuLocalTransform_;
	IMUFilter * imuFilter_;
	std::map<double, IMU> imuBuffer_;
	unsigned int sleepTimeMs_;
	double lastSleepStamp_;
};
#endif

LidarOuster::LidarOuster(
		const std::string& sensorHostname,
		int lidarMode,
		int timestampMode,
		const std::string& dataDestination,
		bool useReflectivityForIntensityChannel,
		bool publishIMU,
		float frameRate,
		Transform localTransform) :
	Lidar(frameRate, localTransform),
	ousterCaptureThread_(0),
	imuPublished_(publishIMU),
	useReflectivityForIntensityChannel_(useReflectivityForIntensityChannel),
	sensorHostname_(sensorHostname),
	dataDestination_(dataDestination),
	lidarMode_(lidarMode),
	timestampMode_(timestampMode)
{
	UASSERT(!sensorHostname_.empty());
	UDEBUG("Using sensor hostname \"%s\" and destination (optional) \"%s\"", sensorHostname_.c_str(), dataDestination_.c_str());
#ifdef RTABMAP_OUSTER
	UASSERT(lidarMode>=ouster::sensor::lidar_mode::MODE_UNSPEC && lidarMode<=ouster::sensor::lidar_mode::MODE_4096x5);
	UASSERT(timestampMode>=ouster::sensor::timestamp_mode::TIME_FROM_UNSPEC && timestampMode<=ouster::sensor::timestamp_mode::TIME_FROM_PTP_1588);
#endif
}
LidarOuster::LidarOuster(
		const std::string& pcapFile,
		const std::string& jsonFile,
		bool useReflectivityForIntensityChannel,
		bool publishIMU,
		float frameRate,
		Transform localTransform) :
	Lidar(frameRate, localTransform),
	ousterCaptureThread_(0),
	imuPublished_(publishIMU),
	useReflectivityForIntensityChannel_(useReflectivityForIntensityChannel),
	pcapFile_(pcapFile),
	jsonFile_(jsonFile)
{
	UASSERT(!pcapFile.empty());
	UDEBUG("Using PCAP file \"%s\" and JSON file \"%s\"", pcapFile.c_str(), jsonFile.c_str());
}
LidarOuster::~LidarOuster()
{
#ifdef RTABMAP_OUSTER
	if(ousterCaptureThread_)
		ousterCaptureThread_->join(true);
	delete ousterCaptureThread_;
#endif
}

bool LidarOuster::available()
{
#ifdef RTABMAP_OUSTER
	return true;
#else
	return false;
#endif
}

std::string LidarOuster::getSerial() const
{
#ifdef RTABMAP_OUSTER
	if(ousterCaptureThread_) {
		ousterCaptureThread_->getInfo().sn.c_str();
	}
#endif
	return "";
}

bool LidarOuster::init(const std::string &, const std::string &)
{
#ifdef RTABMAP_OUSTER
	if(ousterCaptureThread_)
		ousterCaptureThread_->join(true);
	delete ousterCaptureThread_;
	ousterCaptureThread_ = 0;

	ouster::sensor::sensor_info info;

	if(!sensorHostname_.empty())
	{
		UDEBUG("");
		auto clientHandle = ouster::sensor::init_client(
			sensorHostname_, 
			dataDestination_,
			(ouster::sensor::lidar_mode)lidarMode_,
			(ouster::sensor::timestamp_mode)timestampMode_);
		
		if(!clientHandle) {
			UERROR("Failed to connect to sensor! Verify that the Ouster can be reached on the network at this address \"%s\".", sensorHostname_.c_str());
			return false;
		}
		
		UINFO("Connection to sensor succeeded");

		auto metadata = ouster::sensor::get_metadata(*clientHandle);

		// Raw metadata can be parsed into a `sensor_info` struct
		info = ouster::sensor::sensor_info(metadata);

		ousterCaptureThread_ = new OusterCaptureThread(clientHandle, info, imuPublished_, useReflectivityForIntensityChannel_);
	}
	else if(!pcapFile_.empty())
	{
		if(jsonFile_.empty())
		{
			UERROR("A JSON path should be provided when a PCAP path is used.");
			return false;
		}
		UDEBUG("");
		auto pcapHandle = ouster::sensor_utils::replay_initialize(pcapFile_);
		if(!pcapHandle) {
			UERROR("Failed to open pcap file \"%s\"!", pcapFile_.c_str());
			return false;
		}
    	info = ouster::sensor::metadata_from_json(jsonFile_);

		ousterCaptureThread_ = new OusterCaptureThread(pcapHandle, info, imuPublished_, useReflectivityForIntensityChannel_);
	}
	else // OSF?
	{
		UERROR("Not implemented");
		return false;
	}
	
	UINFO("Firmware version: %s", info.fw_rev.c_str());
	UINFO("Serial number:    %s", info.sn.c_str());
	UINFO("Product line:     %s", info.prod_line.c_str());
	UINFO("Scan dimensions:  %ld x %ld", info.format.columns_per_frame, info.format.pixels_per_column);
	//UINFO("Column window:    [%d,%d]", info.column_window.first, info.column_window.second);

	ousterCaptureThread_->start();

	return true;
#else
	UERROR("RTAB-Map is not built with OusterSDK");
	return false;
#endif
}

SensorData LidarOuster::captureData(SensorCaptureInfo * info)
{
	SensorData data;
#ifdef RTABMAP_OUSTER
	double stamp = 0.0;
	LaserScan scan;
	if(!ousterCaptureThread_->getScan(scan, stamp))
	{
		// End of stream
		return data;
	}

	data.setLaserScan(scan);
	data.setStamp(stamp);

	IMU imu = ousterCaptureThread_->getIMU(stamp);
	if(!imu.empty())
	{
		data.setIMU(imu);
	}
#else
	UERROR("RTAB-Map is not built with OusterSDK");
#endif
	return data;
}


} /* namespace rtabmap */
