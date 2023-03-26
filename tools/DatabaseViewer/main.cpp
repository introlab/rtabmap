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

#include <QApplication>
#include "rtabmap/gui/DatabaseViewer.h"
#include "rtabmap/utilite/ULogger.h"

#include <vtkObject.h>
#include <vtkVersionMacros.h>

#if VTK_MAJOR_VERSION > 9 || (VTK_MAJOR_VERSION==9 && VTK_MINOR_VERSION >= 1)
#include <QVTKRenderWidget.h>
#endif

int main(int argc, char * argv[])
{
	ULogger::setType(ULogger::kTypeConsole);
	ULogger::setLevel(ULogger::kInfo);

#ifdef WIN32
	CoInitialize(nullptr);
#endif

#if VTK_MAJOR_VERSION > 9 || (VTK_MAJOR_VERSION==9 && VTK_MINOR_VERSION >= 1)
    // needed to ensure appropriate OpenGL context is created for VTK rendering.
    QSurfaceFormat::setDefaultFormat(QVTKRenderWidget::defaultFormat());
#endif

	QApplication * app = new QApplication(argc, argv);
	rtabmap::DatabaseViewer * mainWindow = new rtabmap::DatabaseViewer();

	mainWindow->showNormal();

	if(argc == 2)
	{
		mainWindow->openDatabase(argv[1]);
	}

	// Now wait for application to finish
	app->connect( app, SIGNAL( lastWindowClosed() ),
				app, SLOT( quit() ) );
	app->exec();// MUST be called by the Main Thread

	delete mainWindow;
	delete app;
	UINFO("All done! closing...");

	return 0;
}
