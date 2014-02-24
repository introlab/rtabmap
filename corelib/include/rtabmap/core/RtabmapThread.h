/*
 * Copyright (C) 2010-2011, Mathieu Labbe and IntRoLab - Universite de Sherbrooke
 *
 * This file is part of RTAB-Map.
 *
 * RTAB-Map is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * RTAB-Map is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with RTAB-Map.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef RTABMAPTHREAD_H_
#define RTABMAPTHREAD_H_

#include "rtabmap/core/RtabmapExp.h" // DLL export/import defines

#include <rtabmap/utilite/UThreadNode.h>
#include <rtabmap/utilite/UEventsHandler.h>
#include <rtabmap/utilite/USemaphore.h>
#include <rtabmap/utilite/UMutex.h>

#include "rtabmap/core/RtabmapEvent.h"
#include "rtabmap/core/Image.h"
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
		kStateIdle,
		kStateDetecting,
		kStateReseting,
		kStateChangingParameters,
		kStateDumpingMemory,
		kStateDumpingPrediction,
		kStateGeneratingDOTGraph,
		kStateGeneratingDOTLocalGraph,
		kStateGeneratingTOROGraphLocal,
		kStateGeneratingTOROGraphGlobal,
		kStateDeletingMemory,
		kStateCleanDataBuffer,
		kStatePublishingMapLocal,
		kStatePublishingMapGlobal,
		kStatePublishingTOROGraphLocal,
		kStatePublishingTOROGraphGlobal,
		Graph,
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
	void addImage(const Image & image);
	void getImage(Image & image);
	void pushNewState(State newState, const ParametersMap & parameters = ParametersMap());
	void setDataBufferSize(int size);
	void publishMap(bool optimized, bool full) const;
	void publishTOROGraph(bool optimized, bool full) const;

private:
	UMutex _stateMutex;
	std::stack<State> _state;
	std::stack<ParametersMap> _stateParam;

	std::list<Image> _imageBuffer;
	UMutex _imageMutex;
	USemaphore _imageAdded;
	int _imageBufferMaxSize;
	float _rate;
	UTimer * _frameRateTimer;

	Rtabmap * _rtabmap;
	bool _paused;
};

} /* namespace rtabmap */
#endif /* RTABMAPTHREAD_H_ */
