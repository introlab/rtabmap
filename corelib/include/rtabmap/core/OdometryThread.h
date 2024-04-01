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
#include <rtabmap/core/SensorData.h>
#include <rtabmap/utilite/UThread.h>
#include <rtabmap/utilite/UEventsHandler.h>
#include <list>

namespace rtabmap {

class Odometry;

class RTABMAP_CORE_EXPORT OdometryThread : public UThread, public UEventsHandler {
public:
	// take ownership of Odometry
	OdometryThread(Odometry * odometry, unsigned int dataBufferMaxSize = 1);
	virtual ~OdometryThread();

protected:
	virtual bool handleEvent(UEvent * event);

private:
	virtual void mainLoopBegin();
	virtual void mainLoopKill();

	//============================================================
	// MAIN LOOP
	//============================================================
	virtual void mainLoop();
	void addData(const SensorData & data);
	bool getData(SensorData & data);

private:
	USemaphore _dataAdded;
	UMutex _dataMutex;
	std::list<SensorData> _dataBuffer;
	std::list<SensorData> _imuBuffer;
	Odometry * _odometry;
	unsigned int _dataBufferMaxSize;
	bool _resetOdometry;
	Transform _resetPose;
	double _lastImuStamp;
	double _imuEstimatedDelay;
};

} // namespace rtabmap


#endif /* ODOMETRYTHREAD_H_ */
