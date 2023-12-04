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
	stampLast_(stampLast)
{
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
	stampLast_(stampLast)
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
		}
	}
	else
	{
		UWARN("Did not receive any scans for the past 5 seconds.");
	}
	return data;
}


} /* namespace rtabmap */
