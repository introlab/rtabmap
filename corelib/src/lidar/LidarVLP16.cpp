/*
Copyright (c) 2010-2022, Mathieu Labbe
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

#include <rtabmap/core/lidar/LidarVLP16.h>
#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/utilite/UThread.h>
#include <pcl/pcl_config.h>

#if PCL_VERSION_COMPARE(<, 1, 9, 0)
#define VLP_MAX_NUM_LASERS 16
#define VLP_DUAL_MODE 0x39
#endif

// RTABMAP_VLP16_USE_CUSTOM_SOCKET is defined (Apple only, and toggleable) in
// LidarVLP16.h. When set, network mode uses our own single-threaded UDP receive
// loop; otherwise start()/stop()/isRunning() delegate to pcl::VLPGrabber.

namespace rtabmap {

/** @brief Function used to check that hour assigned to timestamp in conversion is
 * correct. Velodyne only returns time since the top of the hour, so if the computer clock
 * and the velodyne clock (gps-synchronized) are a little off, there is a chance the wrong
 * hour may be associated with the timestamp
 *
 * Original author: Copyright (C) 2019 Matthew Pitropov, Joshua Whitley
 * Original license: BSD License 2.0
 * Original code: https://github.com/ros-drivers/velodyne/blob/master/velodyne_driver/include/velodyne_driver/time_conversion.hpp
 *
 * @param stamp timestamp recovered from velodyne
 * @param nominal_stamp time coming from computer's clock
 * @return timestamp from velodyne, possibly shifted by 1 hour if the function arguments
 * disagree by more than a half-hour.
 */
double resolveHourAmbiguity(const double &stamp, const double &nominal_stamp) {
    const int HALFHOUR_TO_SEC = 1800;
    double retval = stamp;
    if (nominal_stamp > stamp) {
        if (nominal_stamp - stamp > HALFHOUR_TO_SEC) {
            retval = retval + 2*HALFHOUR_TO_SEC;
        }
    } else if (stamp - nominal_stamp > HALFHOUR_TO_SEC) {
        retval = retval - 2*HALFHOUR_TO_SEC;
    }
    return retval;
}

/*
 * Original author: Copyright (C) 2019 Matthew Pitropov, Joshua Whitley
 * Original license: BSD License 2.0
 * Original code: https://github.com/ros-drivers/velodyne/blob/master/velodyne_driver/include/velodyne_driver/time_conversion.hpp
 */
double rosTimeFromGpsTimestamp(const uint32_t data) {
    const int HOUR_TO_SEC = 3600;
    // time for each packet is a 4 byte uint
    // It is the number of microseconds from the top of the hour
    double time_nom = UTimer::now();
    uint32_t cur_hour = time_nom / HOUR_TO_SEC;
    double stamp = double(cur_hour * HOUR_TO_SEC) + double(data) / 1000000;
    stamp = resolveHourAmbiguity(stamp, time_nom);
    return stamp;
}

LidarVLP16::LidarVLP16(
		const std::string& pcapFile,
		bool organized,
		bool stampLast,
		float frameRate,
		Transform localTransform) :
	Lidar(frameRate, localTransform),
	pcl::VLPGrabber(pcapFile),
	timingOffsetsDualMode_(false),
	startSweepTime_(0),
	startSweepTimeHost_(0),
	organized_(organized),
	useHostTime_(false),
	stampLast_(stampLast),
	networkMode_(false),
	port_(0),
	receivedScan_(false)
#ifdef RTABMAP_VLP16_USE_CUSTOM_SOCKET
	,socket_(0)
	,readThread_(0)
	,terminate_(false)
#endif
{
	// ipAddress_ unused in PCAP mode (default-constructed / unspecified).
	UDEBUG("Using PCAP file \"%s\"", pcapFile.c_str());
}
LidarVLP16::LidarVLP16(
		const boost::asio::ip::address& ipAddress,
		const std::uint16_t port,
		bool organized,
		bool useHostTime,
		bool stampLast,
		float frameRate,
		Transform localTransform) :
	Lidar(frameRate, localTransform),
	pcl::VLPGrabber(ipAddress, port),
	timingOffsetsDualMode_(false),
	startSweepTime_(0),
	startSweepTimeHost_(0),
	organized_(organized),
	useHostTime_(useHostTime),
	stampLast_(stampLast),
	// networkMode_ = live network capture (vs PCAP playback), on all platforms;
	// it drives the "no data received" warning below. The custom socket path it
	// also selects in start()/stop() is Apple-only (RTABMAP_VLP16_USE_CUSTOM_SOCKET).
	networkMode_(true),
	ipAddress_(ipAddress),
	port_(port),
	receivedScan_(false)
#ifdef RTABMAP_VLP16_USE_CUSTOM_SOCKET
	,socket_(0)
	,readThread_(0)
	,terminate_(false)
#endif
{
	UDEBUG("Using network lidar with IP=%s port=%d", ipAddress.to_string().c_str(), port);
}

LidarVLP16::~LidarVLP16()
{
	UDEBUG("Stopping lidar...");
	stop();
	scanReady_.release();
	UDEBUG("Stopped lidar!");
}

void LidarVLP16::setOrganized(bool enable)
{
	organized_ = true;
}

#ifdef RTABMAP_VLP16_USE_CUSTOM_SOCKET
void LidarVLP16::start()
{
	if(networkMode_)
	{
		if(readThread_ != 0)
		{
			// already running
			return;
		}

		terminate_ = false;
		// ipAddress_ is the LOCAL interface to listen on (as in pcl::HDLGrabber),
		// not the sensor's address. Binding to a specific local IP scopes
		// reception to that NIC on a multi-homed host. If it is unspecified
		// (0.0.0.0) or not a local address (e.g. the sensor's IP was entered by
		// mistake), fall back to listening on all interfaces, like pcl::HDLGrabber.
		try
		{
			boost::asio::ip::udp::endpoint endpoint(
					ipAddress_.is_unspecified() ? boost::asio::ip::address(boost::asio::ip::address_v4::any()) : ipAddress_,
					port_);
			// Unlike pcl::HDLGrabber, we read and process packets in a single
			// thread instead of a producer/consumer queue. A large receive buffer
			// lets the kernel hold a backlog (~800 VLP16 packets here) so a brief
			// stall in toPointClouds() doesn't drop packets.
			const int receiveBufferSize = 1024 * 1024; // 1 MB
			try
			{
				socket_ = new boost::asio::ip::udp::socket(ioContext_);
				socket_->open(boost::asio::ip::udp::v4());
				socket_->set_option(boost::asio::socket_base::reuse_address(true));
				socket_->set_option(boost::asio::socket_base::receive_buffer_size(receiveBufferSize));
				socket_->bind(endpoint);
			}
			catch(const std::exception & e)
			{
				UWARN("Could not bind VLP16 socket to local address %s (%s); "
					"falling back to listening on all interfaces (0.0.0.0).",
					endpoint.address().to_string().c_str(), e.what());
				delete socket_;
				socket_ = new boost::asio::ip::udp::socket(ioContext_);
				socket_->open(boost::asio::ip::udp::v4());
				socket_->set_option(boost::asio::socket_base::reuse_address(true));
				socket_->set_option(boost::asio::socket_base::receive_buffer_size(receiveBufferSize));
				socket_->bind(boost::asio::ip::udp::endpoint(boost::asio::ip::address_v4::any(), port_));
			}
		}
		catch(const std::exception & e)
		{
			UERROR("Failed to bind to VLP16 UDP port %d: %s", (int)port_, e.what());
			delete socket_;
			socket_ = 0;
			return;
		}
		receivedScan_ = false;
		readThread_ = new std::thread(&LidarVLP16::readPackets, this);
		return;
	}
	// PCAP mode: use the base grabber.
	pcl::VLPGrabber::start();
}

void LidarVLP16::stop()
{
	if(networkMode_)
	{
		terminate_ = true;
		if(socket_ != 0)
		{
			// Closing the socket unblocks receive_from in the read thread. Because
			// that call uses the non-throwing (error_code) overload, the EBADF that
			// BSD/macOS returns here surfaces as an error code instead of an
			// exception, so the thread exits cleanly rather than aborting.
			boost::system::error_code ec;
			socket_->close(ec);
		}
		if(readThread_ != 0)
		{
			readThread_->join();
			delete readThread_;
			readThread_ = 0;
		}
		delete socket_;
		socket_ = 0;
		return;
	}
	pcl::VLPGrabber::stop();
}

bool LidarVLP16::isRunning() const
{
	if(networkMode_)
	{
		return readThread_ != 0;
	}
	return pcl::VLPGrabber::isRunning();
}

void LidarVLP16::readPackets()
{
	std::uint8_t data[1500];
	boost::asio::ip::udp::endpoint sender;
	while(!terminate_)
	{
		boost::system::error_code ec;
		std::size_t length = socket_->receive_from(boost::asio::buffer(data, sizeof(data)), sender, 0, ec);
		if(ec)
		{
			// On stop() the socket is closed under us (bad_descriptor /
			// operation_aborted); only warn if it wasn't an expected shutdown.
			if(!terminate_)
			{
				UWARN("VLP16 socket receive error: %s", ec.message().c_str());
			}
			break;
		}
		// VLP-16 data packets are 1206 bytes; ignore anything shorter.
		if(length >= 1206)
		{
			toPointClouds(reinterpret_cast<HDLDataPacket*>(data));
		}
	}
}
#endif // RTABMAP_VLP16_USE_CUSTOM_SOCKET

bool LidarVLP16::init(const std::string &, const std::string &)
{
	UDEBUG("Init lidar");
	if(isRunning())
	{
		UDEBUG("Stopping lidar...");
		stop();
		uSleep(2000); // make sure all callbacks are finished
		UDEBUG("Stopped lidar!");
	}
	startSweepTime_ = 0.0;
	startSweepTimeHost_ = 0.0;
	accumulatedScans_.clear();
	if(organized_)
	{
		accumulatedScans_.resize(16);
	}
	else
	{
		accumulatedScans_.resize(1);
	}
	buildTimings(false);
	start();
	UDEBUG("Lidar capture started");
	return true;
}

/**
 * Build a timing table for each block/firing. Stores in timing_offsets vector
 */
void LidarVLP16::buildTimings(bool dualMode)
{
	// vlp16
	// timing table calculation, from velodyne user manual
	timingOffsets_.resize(12);
	for (size_t i=0; i < timingOffsets_.size(); ++i){
		timingOffsets_[i].resize(32);
	}
	// constants
	double full_firing_cycle = 55.296 * 1e-6; // seconds
	double single_firing = 2.304 * 1e-6; // seconds
	double dataBlockIndex, dataPointIndex;
	// compute timing offsets
	for (size_t x = 0; x < timingOffsets_.size(); ++x){
		for (size_t y = 0; y < timingOffsets_[x].size(); ++y){
			if (dualMode){
				dataBlockIndex = (x - (x % 2)) + (y / 16);
			}
			else{
				dataBlockIndex = (x * 2) + (y / 16);
			}
			dataPointIndex = y % 16;
			//timing_offsets[block][firing]
									timingOffsets_[x][y] = (full_firing_cycle * dataBlockIndex) + (single_firing * dataPointIndex);
		}
	}
	timingOffsetsDualMode_ = dualMode;
}

void LidarVLP16::toPointClouds (HDLDataPacket *dataPacket)
{
	if (sizeof(HDLLaserReturn) != 3)
		return;

	double receivedHostTime = UTimer::now();
	double packetStamp = rosTimeFromGpsTimestamp(dataPacket->gpsTimestamp);
	if(startSweepTime_==0)
	{
		startSweepTime_ = packetStamp;
		startSweepTimeHost_ = receivedHostTime;
	}

	bool dualMode = dataPacket->mode == VLP_DUAL_MODE;
	if(timingOffsets_.empty() || timingOffsetsDualMode_ != dualMode)
	{
		// reset everything
		timingOffsets_.clear();
		buildTimings(dualMode);
		startSweepTime_ = packetStamp;
		startSweepTimeHost_ = receivedHostTime;
		for(size_t i=0; i<accumulatedScans_.size(); ++i)
		{
			accumulatedScans_[i].clear();
		}
	}

	double interpolated_azimuth_delta;
	std::uint8_t index = 1;
	if (dualMode)
	{
		index = 2;
	}
	if (dataPacket->firingData[index].rotationalPosition < dataPacket->firingData[0].rotationalPosition)
	{
		interpolated_azimuth_delta = ((dataPacket->firingData[index].rotationalPosition + 36000) - dataPacket->firingData[0].rotationalPosition) / 2.0;
	}
	else
	{
		interpolated_azimuth_delta = (dataPacket->firingData[index].rotationalPosition - dataPacket->firingData[0].rotationalPosition) / 2.0;
	}

	for (std::uint8_t i = 0; i < HDL_FIRING_PER_PKT; ++i)
	{
		HDLFiringData firing_data = dataPacket->firingData[i];

		for (std::uint8_t j = 0; j < HDL_LASER_PER_FIRING; j++)
		{
			double current_azimuth = firing_data.rotationalPosition;
			if (j >= VLP_MAX_NUM_LASERS)
			{
				current_azimuth += interpolated_azimuth_delta;
			}
			if (current_azimuth > 36000)
			{
				current_azimuth -= 36000;
			}

			double t = 0;
			if (timingOffsets_.size())
			  t = timingOffsets_[i][j];

			if (current_azimuth < HDLGrabber::last_azimuth_)
			{
				if (!accumulatedScans_[0].empty())
				{
					UScopeMutex lock(lastScanMutex_);
					bool notify = lastScan_.laserScanRaw().empty();
					if(stampLast_)
					{
						double lastStamp = startSweepTime_ + accumulatedScans_[accumulatedScans_.size()-1].back().t;
						double diff = lastStamp - startSweepTime_;
						lastScan_.setStamp(useHostTime_?startSweepTimeHost_+diff:lastStamp);
						for(size_t r=0; r<accumulatedScans_.size(); ++r)
						{
							for(size_t k=0; k<accumulatedScans_[r].size(); ++k)
							{
								accumulatedScans_[r][k].t -= diff;
							}
						}
					}
					else
					{
						lastScan_.setStamp(useHostTime_?startSweepTimeHost_:startSweepTime_);
					}
					if(accumulatedScans_.size() > 1)
					{
						cv::Mat organizedScan = cv::Mat(1, accumulatedScans_[0].size(), CV_32FC(5), accumulatedScans_[0].data()).clone();
						for(size_t k=1; k<accumulatedScans_.size(); ++k)
						{
							UASSERT((int)accumulatedScans_[k].size() == organizedScan.cols);
							organizedScan.push_back(cv::Mat(1, accumulatedScans_[k].size(), CV_32FC(5), accumulatedScans_[k].data()).clone());
						}
						lastScan_.setLaserScan(LaserScan(organizedScan, 0, 0, LaserScan::kXYZIT, getLocalTransform()));
					}
					else
					{
						lastScan_.setLaserScan(LaserScan(cv::Mat(1, accumulatedScans_[0].size(), CV_32FC(5), accumulatedScans_[0].data()).clone(), 0, 0, LaserScan::kXYZIT, getLocalTransform()));
					}
					if(notify)
					{
						scanReady_.release();
					}

					startSweepTime_ = packetStamp + t;
					startSweepTimeHost_ = receivedHostTime + t;
				}
				for(size_t k=0; k<accumulatedScans_.size(); ++k)
				{
					accumulatedScans_[k].clear();
				}
			}

			double timeSinceStartOfThisScan = packetStamp + t - startSweepTime_;

			pcl::PointXYZI xyzi;
			HDLGrabber::computeXYZI (xyzi, current_azimuth, firing_data.laserReturns[j], laser_corrections_[j % VLP_MAX_NUM_LASERS]);
			PointXYZIT xyzit;
			xyzit.x = xyzi.y;
			xyzit.y = -xyzi.x;
			xyzit.z = xyzi.z;
			xyzit.i = xyzi.intensity;
			xyzit.t = timeSinceStartOfThisScan;

			if(accumulatedScans_.size()>1)
			{
				accumulatedScans_[j % VLP_MAX_NUM_LASERS].push_back(xyzit);
			}
			else if (! (std::isnan (xyzit.x) || std::isnan (xyzit.y) || std::isnan (xyzit.z)))
			{
				accumulatedScans_[0].push_back (xyzit);
			}
			last_azimuth_ = current_azimuth;

			if (dualMode)
			{
				pcl::PointXYZI dual_xyzi;
				HDLGrabber::computeXYZI (dual_xyzi, current_azimuth, dataPacket->firingData[i + 1].laserReturns[j], laser_corrections_[j % VLP_MAX_NUM_LASERS]);

				if(accumulatedScans_.size()>1)
				{
					xyzit.x = dual_xyzi.y;
					xyzit.y = -dual_xyzi.x;
					xyzit.z = dual_xyzi.z;
					xyzit.i = dual_xyzi.intensity;
					xyzit.t = timeSinceStartOfThisScan;
					accumulatedScans_[j % VLP_MAX_NUM_LASERS].push_back (xyzit);
				}
				else if ((dual_xyzi.x != xyzi.x || dual_xyzi.y != xyzi.y || dual_xyzi.z != xyzi.z)
						&& ! (std::isnan (dual_xyzi.x) || std::isnan (dual_xyzi.y) || std::isnan (dual_xyzi.z)))
				{
					xyzit.x = dual_xyzi.y;
					xyzit.y = -dual_xyzi.x;
					xyzit.z = dual_xyzi.z;
					xyzit.i = dual_xyzi.intensity;
					xyzit.t = timeSinceStartOfThisScan;
					accumulatedScans_[0].push_back (xyzit);
				}
			}
		}
		if (dualMode)
		{
			i++;
		}
	}
}

SensorData LidarVLP16::captureData(SensorCaptureInfo * info)
{
	SensorData data;
	if(scanReady_.acquire(1, 5000))
	{
		UScopeMutex lock(lastScanMutex_);
		if(!lastScan_.laserScanRaw().empty())
		{
			data = lastScan_;
			lastScan_ = SensorData();
			receivedScan_ = true;
		}
	}
	else if(networkMode_ && !receivedScan_)
	{
		// No packet has ever arrived: most likely the sensor isn't sending data
		// to this computer. Point the user at the configured listening endpoint.
		UWARN("Did not receive any VLP16 data packets for the past 5 seconds. "
			"This computer is listening on %s:%d (%s). Make sure the LiDAR is "
			"powered on and configured to send its data to this computer's IP "
			"address on UDP port %d (set the data destination host in the "
			"LiDAR's own web/configuration interface).",
			ipAddress_.to_string().c_str(), (int)port_,
			ipAddress_.is_unspecified() ? "all local interfaces" : "this interface only",
			(int)port_);
	}
	else
	{
		UWARN("Did not receive any scans for the past 5 seconds.");
	}
	return data;
}


} /* namespace rtabmap */
