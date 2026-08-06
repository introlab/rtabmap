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

#ifndef RTABMAPTHREAD_H_
#define RTABMAPTHREAD_H_

#include "rtabmap/core/rtabmap_core_export.h" // DLL export/import defines

#include <rtabmap/utilite/UThreadNode.h>
#include <rtabmap/utilite/UEventsHandler.h>
#include <rtabmap/utilite/USemaphore.h>
#include <rtabmap/utilite/UMutex.h>

#include "rtabmap/core/RtabmapEvent.h"
#include "rtabmap/core/SensorData.h"
#include "rtabmap/core/Parameters.h"
#include "rtabmap/core/OdometryEvent.h"

#include <queue>

class UTimer;

namespace rtabmap {

class Rtabmap;

/**
 * @class RtabmapThread
 * @brief Runs a @ref Rtabmap instance in its own thread, driven by events.
 *
 * The thread owns the @ref Rtabmap object and calls @ref Rtabmap::process() on the
 * data it receives, so that mapping never blocks the sensor or odometry threads.
 * It is the last stage of the event-based pipeline:
 * @ref SensorCaptureThread &rarr; @ref OdometryThread &rarr; RtabmapThread.
 *
 * **Input events** (handled in @ref handleEvent(), i.e. in the caller's thread):
 * - @ref OdometryEvent and @ref SensorEvent -- the frame and its odometry pose are
 *   buffered for the main loop. A null pose means odometry is lost and the frame
 *   is dropped (unless the memory is in localization mode).
 * - @ref UserDataEvent -- user data attached to the next processed frame.
 * - @ref RtabmapEventCmd -- a command (see @ref RtabmapEventCmd::Cmd), such as
 *   initializing or closing the map, triggering a new map, pausing, setting a
 *   goal, labelling a node or requesting the map.
 * - @ref ParamEvent -- applies new parameters, equivalent to a
 *   @ref RtabmapEventCmd::kCmdUpdateParams command.
 *
 * **Output events**, posted from the thread:
 * - @ref RtabmapEvent with the @ref Statistics of an iteration that added a node.
 * - @ref RtabmapEvent3DMap in answer to a map request.
 * - @ref RtabmapGlobalPathEvent and @ref RtabmapGoalStatusEvent for path planning.
 * - @ref RtabmapLabelErrorEvent when a label could not be set.
 *
 * **Input regulation**, on top of what @ref Rtabmap does:
 * - Frames are queued in a buffer of @ref Parameters::kRtabmapImageBufferSize()
 *   elements; when it is full the oldest frame is dropped, so a slow map update
 *   never delays the live pipeline.
 * - Frames arriving faster than @ref Parameters::kRtabmapDetectionRate() are
 *   skipped, which is the rate limiting that @ref Rtabmap::process() itself does
 *   not do. With @ref Parameters::kRtabmapCreateIntermediateNodes() enabled they
 *   are kept instead, and added as intermediate nodes (odometry-only, not used
 *   for loop closure detection).
 * - An identity odometry pose, or a covariance &ge; 9999, is read as an odometry
 *   reset and triggers a new map id on the next processed frame. The largest
 *   covariance seen between two processed frames is the one passed on, so the
 *   link weight does not depend on the odometry frame rate.
 *
 * The object registers itself as a @ref UEventsHandler, so posting an event to
 * @ref UEventsManager is enough to feed it:
 * @code
 * rtabmap::RtabmapThread rtabmapThread(new rtabmap::Rtabmap()); // takes ownership
 * rtabmapThread.start();
 * UEventsManager::addHandler(&rtabmapThread);
 * UEventsManager::createPipe(&odomThread, &rtabmapThread, "OdometryEvent");
 * @endcode
 *
 * @see Rtabmap
 * @see OdometryThread
 * @see SensorCaptureThread
 */
class RTABMAP_CORE_EXPORT RtabmapThread :
	public UThreadNode,
	public UEventsHandler
{
public:
	/** @brief What the main loop does on its next wake-up. */
	enum State {
		kStateDetecting,      /**< Process the next buffered frame. */
		kStateProcessCommand  /**< Execute the next queued @ref RtabmapEventCmd. */
	};

public:
	/**
	 * @brief Constructor.
	 * @param rtabmap The map to run; must not be null. The thread takes ownership
	 *                and deletes it in @ref close(), which the destructor calls.
	 *
	 * The buffer size, detection rate and intermediate node settings are taken
	 * from the default parameters here, then updated by every
	 * @ref RtabmapEventCmd::kCmdInit and @ref RtabmapEventCmd::kCmdUpdateParams
	 * command.
	 */
	RtabmapThread(Rtabmap * rtabmap);
	virtual ~RtabmapThread();

	/** @brief Drops the buffered frames, the pending user data and the odometry state. */
	void clearBufferedData();
	/**
	 * @brief Sets the maximum rate at which frames are processed (Hz, 0 = unlimited).
	 *
	 * Same setting as @ref Parameters::kRtabmapDetectionRate(). Frames arriving
	 * faster are dropped, or added as intermediate nodes if
	 * @ref createIntermediateNodes() is enabled.
	 */
	void setDetectorRate(float rate);
	/**
	 * @brief Sets how many frames may wait in the buffer (0 = unlimited).
	 *
	 * Same setting as @ref Parameters::kRtabmapImageBufferSize(). Once full, the
	 * oldest frame is dropped to make room for the new one.
	 */
	void setDataBufferSize(unsigned int bufferSize);
	/**
	 * @brief Keeps the frames skipped by @ref setDetectorRate() as intermediate nodes.
	 *
	 * Same setting as @ref Parameters::kRtabmapCreateIntermediateNodes(). Those
	 * nodes carry the odometry link but take no part in loop closure detection.
	 */
	void createIntermediateNodes(bool enabled);

	/** @return Maximum processing rate in Hz (0 = unlimited). */
	float getDetectorRate() const {return _rate;}
	/** @return Maximum number of buffered frames (0 = unlimited). */
	unsigned int getDataBufferSize() const {return _dataBufferMaxSize;}
	/** @return True if skipped frames are added as intermediate nodes. */
	bool getCreateIntermediateNodes() const {return _createIntermediateNodes;}

	/**
	 * @brief Joins the thread and closes rtabmap. This will delete rtabmap object if set.
	 * @param databaseSaved true=database saved, false=database discarded.
	 * @param databasePath output database file name, ignored if
	 *                     Db/Sqlite3InMemory=false (opened database is
	 *                     then overwritten).
	 *
	 * Called by the destructor with @p databaseSaved true. The object cannot be
	 * used afterwards: the @ref Rtabmap instance it owned is gone.
	 */
	void close(bool databaseSaved, const std::string & databasePath = "");

protected:
	/**
	 * @brief Receives the events listed in the class description.
	 *
	 * Runs in the posting thread: data events are only queued here, the work
	 * happens in the thread's main loop. Events are ignored until the thread is
	 * started.
	 *
	 * @return Always false, so the event keeps being dispatched to other handlers.
	 */
	virtual bool handleEvent(UEvent * anEvent);

private:
	virtual void mainLoopBegin();
	virtual void mainLoop();
	virtual void mainLoopKill();
	void process();
	void addData(const OdometryEvent & odomEvent);
	bool getData(OdometryEvent & data);
	void pushNewState(State newState, const RtabmapEventCmd & cmdEvent = RtabmapEventCmd(RtabmapEventCmd::kCmdUndef));
	void publishMap(bool optimized, bool full, bool graphOnly) const;

private:
	UMutex _stateMutex;
	std::queue<State> _state;
	std::queue<RtabmapEventCmd> _stateParam;

	std::list<OdometryEvent> _dataBuffer;
	std::list<double> _newMapEvents;
	UMutex _dataMutex;
	USemaphore _dataAdded;
	unsigned int _dataBufferMaxSize;
	float _rate;
	bool _createIntermediateNodes;
	double _previousStamp;

	Rtabmap * _rtabmap;
	bool _paused;
	Transform lastPose_;
	cv::Mat covariance_;

	cv::Mat _userData;
	UMutex _userDataMutex;
};

} /* namespace rtabmap */
#endif /* RTABMAPTHREAD_H_ */
