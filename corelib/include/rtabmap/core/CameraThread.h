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

#include <rtabmap/utilite/UThread.h>
#include <rtabmap/utilite/UEventsSender.h>

namespace rtabmap
{

class Camera;
class CameraRGBD;

/**
 * Class CameraThread
 *
 */
class RTABMAP_EXP CameraThread :
	public UThread,
	public UEventsSender
{
public:
	// ownership transferred
	CameraThread(Camera * camera);
	CameraThread(CameraRGBD * camera);
	virtual ~CameraThread();

	bool init(); // call camera->init()

	//getters
	bool isPaused() const {return !this->isRunning();}
	bool isCapturing() const {return this->isRunning();}
	void setImageRate(float imageRate);

private:
	virtual void mainLoop();

private:
	Camera * _camera;
	CameraRGBD * _cameraRGBD;
	int _seq;
};

} // namespace rtabmap
