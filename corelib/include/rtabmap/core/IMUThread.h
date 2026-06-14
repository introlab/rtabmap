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

#include "rtabmap/core/rtabmap_core_export.h" // DLL export/import defines

#include <rtabmap/core/Transform.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/utilite/UThread.h>
#include <rtabmap/utilite/UEventsSender.h>
#include <rtabmap/utilite/UTimer.h>

#include <fstream>

namespace rtabmap {

class IMUFilter;

/**
 * @class IMUThread
 * @brief Background thread that replays IMU measurements from a CSV file.
 *
 * Reads gyroscope and accelerometer samples from a comma-separated file (EuRoC-style
 * or epoch timestamps), optionally fuses them with @ref IMUFilter, and posts
 * @ref IMUEvent on each step.
 *
 * CSV format:
 * - First line: header (skipped).
 * - Following lines: `stamp,gx,gy,gz,ax,ay,az` (stamp in seconds with a decimal
 *   point, or EuRoC nanoseconds without one).
 *
 * Playback rate is limited by @ref setRate() when `rate > 0`; otherwise samples are
 * read as fast as possible (or spaced using inter-sample timestamps after the first
 * pair). When the file ends, the thread posts an empty @ref IMUEvent and stops.
 *
 * @see IMUEvent
 * @see IMUFilter
 * @see SensorCaptureThread::enableIMUFiltering()
 */
class RTABMAP_CORE_EXPORT IMUThread :
	public UThread,
	public UEventsSender
{
public:
	/**
	 * @brief Constructs the replay thread.
	 * @param rate Target playback rate in Hz (`0` = no rate cap until timestamps apply).
	 * @param localTransform IMU frame relative to the robot base (stored in each @ref IMU).
	 */
	IMUThread(int rate, const Transform & localTransform);
	virtual ~IMUThread();

	/**
	 * @brief Opens and validates an IMU CSV file.
	 * @param path Path to the CSV file.
	 * @return False if the file is missing or contains no data rows.
	 */
	bool init(const std::string & path);

	/** @brief Sets the target playback rate in Hz. */
	void setRate(int rate);

	/**
	 * @brief Enables orientation fusion on replayed samples.
	 * @param filteringStrategy @ref IMUFilter::Type index (`0` = Madgwick, `1` = complementary).
	 * @param parameters Optional ImuFilter/... tuning parameters.
	 * @param baseFrameConversion If true, rotate IMU vectors into the base frame before filtering.
	 */
	void enableIMUFiltering(int filteringStrategy = 1, const ParametersMap & parameters = ParametersMap(), bool baseFrameConversion = false);

	/** @brief Disables fusion and deletes the internal @ref IMUFilter. */
	void disableIMUFiltering();

private:
	virtual void mainLoopBegin();
	virtual void mainLoop();

private:
	int rate_;
	Transform localTransform_;
	std::ifstream imuFile_;
	UTimer frameRateTimer_;
	double captureDelay_;
	double previousStamp_;
	IMUFilter * _imuFilter;
	bool _imuBaseFrameConversion;
};

} // namespace rtabmap
