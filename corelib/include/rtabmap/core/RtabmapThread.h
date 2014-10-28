/*
Copyright (c) 2010-2014, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
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

#include "rtabmap/core/RtabmapExp.h" // DLL export/import defines

#include <rtabmap/utilite/UThreadNode.h>
#include <rtabmap/utilite/UEventsHandler.h>
#include <rtabmap/utilite/USemaphore.h>
#include <rtabmap/utilite/UMutex.h>

#include "rtabmap/core/RtabmapEvent.h"
#include "rtabmap/core/SensorData.h"
#include "rtabmap/core/Parameters.h"

#include <stack>

class UTimer;

namespace rtabmap {

class Rtabmap;

class RTABMAP_EXP RtabmapThread :
	public UThreadNode,
	public UEventsHandler
{
public:
	enum State {
		kStateInit,
		kStateDetecting,
		kStateReseting,
		kStateClose,
		kStateChangingParameters,
		kStateDumpingMemory,
		kStateDumpingPrediction,
		kStateGeneratingDOTGraph,
		kStateGeneratingDOTLocalGraph,
		kStateGeneratingTOROGraphLocal,
		kStateGeneratingTOROGraphGlobal,
		kStateCleanDataBuffer,
		kStatePublishingMapLocal,
		kStatePublishingMapGlobal,
		kStatePublishingTOROGraphLocal,
		kStatePublishingTOROGraphGlobal,
		kStateTriggeringMap
	};

public:
	// take ownership
	RtabmapThread(Rtabmap * rtabmap);
	virtual ~RtabmapThread();

	void clearBufferedData();
	void setDetectorRate(float rate);
	void setBufferSize(int bufferSize);

protected:
	virtual void handleEvent(UEvent * anEvent);

private:
	virtual void mainLoop();
	virtual void mainLoopKill();
	void process();
	void addData(const SensorData & data);
	void getData(SensorData & data);
	void pushNewState(State newState, const ParametersMap & parameters = ParametersMap());
	void setDataBufferSize(int size);
	void publishMap(bool optimized, bool full) const;
	void publishTOROGraph(bool optimized, bool full) const;

private:
	UMutex _stateMutex;
	std::stack<State> _state;
	std::stack<ParametersMap> _stateParam;

	std::list<SensorData> _dataBuffer;
	UMutex _dataMutex;
	USemaphore _dataAdded;
	int _dataBufferMaxSize;
	float _rate;
	UTimer * _frameRateTimer;

	Rtabmap * _rtabmap;
	bool _paused;
};

} /* namespace rtabmap */
#endif /* RTABMAPTHREAD_H_ */
