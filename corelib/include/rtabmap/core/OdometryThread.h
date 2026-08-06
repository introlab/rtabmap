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

#ifndef ODOMETRYTHREAD_H_
#define ODOMETRYTHREAD_H_

#include <rtabmap/core/rtabmap_core_export.h>
#include <rtabmap/core/SensorEvent.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/utilite/UThread.h>
#include <rtabmap/utilite/UEventsHandler.h>
#include <list>

namespace rtabmap {

class Odometry;

/**
 * @class OdometryThread
 * @brief Runs an @ref Odometry front-end in its own thread, driven by events.
 *
 * The thread owns the @ref Odometry object and calls @ref Odometry::process() on
 * the frames it receives, so that a slow odometry update does not block the
 * sensor thread. It sits in the middle of the event-based pipeline:
 * @ref SensorCaptureThread &rarr; OdometryThread &rarr; @ref RtabmapThread.
 *
 * **Input events** (handled in @ref handleEvent(), i.e. in the caller's thread):
 * - @ref SensorEvent &mdash; a frame to process. It is rejected with an error if
 *   it carries neither a laser scan nor an image with its calibration (an
 *   @ref OdometryMono front-end accepts RGB alone).
 * - @ref IMUEvent &mdash; an IMU sample, kept in a separate buffer.
 * - @ref OdometryResetEvent &mdash; resets the odometry to the pose it carries
 *   (identity if null) and drops everything buffered. Unlike the others, it is
 *   handled even before the thread is started.
 *
 * **Output event**: one @ref OdometryEvent per processed frame, carrying the
 * data, the integrated pose and the @ref OdometryInfo. A null pose means
 * odometry is lost; that is what @ref RtabmapThread reads to start a new map.
 *
 * **Buffering.** The frame buffer holds @p dataBufferMaxSize frames and drops
 * the oldest when full, so with the default size of 1 the odometry always works
 * on the freshest frame rather than falling behind. IMU samples are buffered
 * apart and fed to the odometry up to the stamp of the frame about to be
 * processed, so that tightly-coupled back-ends see them in order. A frame whose
 * stamp falls outside the buffered IMU window is skipped with a warning: with an
 * asynchronous IMU, it must be published faster (less delay) than the camera or lidar.
 *
 * When the incoming @ref SensorEvent already carries an odometry pose (a robot
 * publishing its own odometry), the motion between two consecutive such poses is
 * passed to @ref Odometry::process() as a guess.
 *
 * @see Odometry
 * @see RtabmapThread
 * @see SensorCaptureThread
 */
class RTABMAP_CORE_EXPORT OdometryThread : public UThread, public UEventsHandler {
public:
	/**
	 * @brief Constructor.
	 * @param odometry The odometry to run; must not be null. The thread takes
	 *                 ownership and deletes it in the destructor.
	 * @param dataBufferMaxSize Maximum number of frames waiting to be processed
	 *                          (0 = unlimited). Beyond that the oldest frame is
	 *                          dropped, keeping the odometry on recent data.
	 */
	OdometryThread(Odometry * odometry, unsigned int dataBufferMaxSize = 1);
	virtual ~OdometryThread();

protected:
	/**
	 * @brief Receives the events listed in the class description.
	 *
	 * Runs in the posting thread: frames and IMU samples are only buffered here,
	 * the odometry itself runs in the thread's main loop. Data events are ignored
	 * until the thread is started, an @ref OdometryResetEvent is not.
	 *
	 * @return Always false, so the event keeps being dispatched to other handlers.
	 */
	virtual bool handleEvent(UEvent * event);

private:
	virtual void mainLoopBegin();
	virtual void mainLoopKill();

	//============================================================
	// MAIN LOOP
	//============================================================
	virtual void mainLoop();
	void addData(const SensorEvent & data);
	bool getData(SensorEvent & data);

private:
	USemaphore _dataAdded;
	UMutex _dataMutex;
	std::list<SensorEvent> _dataBuffer;
	std::list<SensorData> _imuBuffer;
	Odometry * _odometry;
	unsigned int _dataBufferMaxSize;
	bool _resetOdometry;
	Transform _resetPose;
	Transform _previousGuessPose;
	double _oldestAsyncImuStamp;
	double _newestAsyncImuStamp;
};

} // namespace rtabmap


#endif /* ODOMETRYTHREAD_H_ */
