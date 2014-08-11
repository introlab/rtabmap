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

#include "rtabmap/core/Rtabmap.h"
#include "rtabmap/core/RtabmapThread.h"
#include "rtabmap/core/CameraRGBD.h"
#include "rtabmap/core/CameraThread.h"
#include "rtabmap/core/Odometry.h"
#include "rtabmap/utilite/UEventsManager.h"
#include <QtGui/QApplication>
#include <stdio.h>

#include "MapBuilder.h"

using namespace rtabmap;
int main(int argc, char * argv[])
{
	ULogger::setType(ULogger::kTypeConsole);
	ULogger::setLevel(ULogger::kWarning);

	// GUI stuff, there the handler will receive RtabmapEvent and construct the map
	QApplication app(argc, argv);
	MapBuilder mapBuilder;

	// Here is the pipeline that we will use:
	// CameraOpenni -> "CameraEvent" -> OdometryThread -> "OdometryEvent" -> RtabmapThread -> "RtabmapEvent"

	// Create the OpenNI camera, it will send a CameraEvent at the rate specified.
	// Set transform to camera so z is up, y is left and x going forward
	CameraRGBD * camera = new CameraOpenni("", 10, rtabmap::Transform(0,0,1,0, -1,0,0,0, 0,-1,0,0));
	//CameraRGBD * camera = new CameraFreenect(0, 10, rtabmap::Transform(0,0,1,0, -1,0,0,0, 0,-1,0,0));
	CameraThread cameraThread(camera);
	if(!cameraThread.init())
	{
		UERROR("Camera init failed!");
		exit(1);
	}

	// Create an odometry thread to process camera events, it will send OdometryEvent.
	OdometryThread odomThread(new OdometryBOW()); // 0=SURF 1=SIFT


	// Create RTAB-Map to process OdometryEvent
	Rtabmap * rtabmap = new Rtabmap();
	rtabmap->init();
	RtabmapThread rtabmapThread(rtabmap); // ownership is transfered

	// Setup handlers
	odomThread.registerToEventsManager();
	rtabmapThread.registerToEventsManager();
	mapBuilder.registerToEventsManager();

	// The RTAB-Map is subscribed by default to CameraEvent, but we want
	// RTAB-Map to process OdometryEvent instead, ignoring the CameraEvent.
	// We can do that by creating a "pipe" between the camera and odometry, then
	// only the odometry will receive CameraEvent from that camera. RTAB-Map is
	// also subscribed to OdometryEvent by default, so no need to create a pipe between
	// odometry and RTAB-Map.
	UEventsManager::createPipe(&cameraThread, &odomThread, "CameraEvent");

	// Let's start the threads
	rtabmapThread.start();
	odomThread.start();
	cameraThread.start();

	mapBuilder.show();
	app.exec(); // main loop

	// remove handlers
	mapBuilder.unregisterFromEventsManager();
	rtabmapThread.unregisterFromEventsManager();
	odomThread.unregisterFromEventsManager();

	// Kill all threads
	cameraThread.kill();
	odomThread.join(true);
	rtabmapThread.join(true);

	return 0;
}
