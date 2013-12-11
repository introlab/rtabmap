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

#pragma once

#include "rtabmap/core/RtabmapExp.h" // DLL export/import defines

#include <rtabmap/utilite/UThreadNode.h>
#include <rtabmap/utilite/UEventsHandler.h>

#include "rtabmap/core/Parameters.h"

#include <stack>

namespace rtabmap
{

class Camera;

/**
 * Class CameraThread
 *
 */
class RTABMAP_EXP CameraThread :
	public UThreadNode,
	public UEventsHandler
{
public:
	enum State {kStateCapturing, kStateChangingParameters};

public:
	// ownership transferred
	CameraThread(Camera * camera, bool autoRestart = false);
	virtual ~CameraThread();

	bool init(); // call camera->init()

	//getters
	bool isPaused() const {return !this->isRunning();}
	bool isCapturing() const {return this->isRunning();}
	void setAutoRestart(bool autoRestart) {_autoRestart = autoRestart;}
	Camera * getCamera() {return _camera;}

protected:
	virtual void handleEvent(UEvent* anEvent);

private:
	virtual void mainLoop();
	void process();
	void pushNewState(State newState, const ParametersMap & parameters = ParametersMap());

private:
	Camera * _camera;
	UMutex _stateMutex;
	std::stack<State> _state;
	std::stack<ParametersMap> _stateParam;
	bool _autoRestart;
	int _seq;
};

} // namespace rtabmap
