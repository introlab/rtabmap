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

#include <QtGui/QApplication>
#include <QtCore/QDir>
#include "rtabmap/utilite/UEventsManager.h"
#include "rtabmap/core/RtabmapThread.h"
#include "rtabmap/core/Rtabmap.h"
#include "rtabmap/gui/MainWindow.h"
#include <QtGui/QMessageBox>
#include "rtabmap/utilite/UObjDeletionThread.h"
#include "ObjDeletionHandler.h"

using namespace rtabmap;

int main(int argc, char* argv[])
{
	/* Set logger type */
	ULogger::setType(ULogger::kTypeConsole);
	ULogger::setLevel(ULogger::kInfo);

	ULOGGER_INFO("Program started...");

	/* Create tasks */
	QApplication * app = new QApplication(argc, argv);
	MainWindow * mainWindow = new MainWindow();

	UEventsManager::addHandler(mainWindow);

	/* Start thread's task */
	mainWindow->showNormal();

	RtabmapThread * rtabmap = new RtabmapThread(new Rtabmap());
	rtabmap->start(); // start it not initialized... will be initialized by event from the gui
	UEventsManager::addHandler(rtabmap);

	// Now wait for application to finish
	app->connect( app, SIGNAL( lastWindowClosed() ),
				app, SLOT( quit() ) );
	app->exec();// MUST be called by the Main Thread

	/* Remove handlers */
	UEventsManager::removeHandler(mainWindow);
	UEventsManager::removeHandler(rtabmap);

	ULOGGER_INFO("Killing threads...");
	rtabmap->join(true);

	ULogger::setLevel(ULogger::kInfo);

	ULOGGER_INFO("Closing RTAB-Map core...");

	//Since we can't put the Rtabmap object in the MainWindow class,
	//we pop up a message box indicating that the rtabmap object
	// is being deleted (saving data to the database)
	QMessageBox * msg = new QMessageBox(QMessageBox::Information,
			QObject::tr("RTAB-Map is closing..."),
			QObject::tr("The detector is saving the working memory to database (located in RTAB-Map's working directory)..."),
			QMessageBox::NoButton,
			mainWindow);
	msg->setEnabled(false);
	msg->setIconPixmap(QPixmap(":/images/RTAB-Map.ico"));
	msg->setWindowIcon(QIcon(":/images/RTAB-Map.ico"));
	msg->show();
	UObjDeletionThread<RtabmapThread> delThread(rtabmap);
	ObjDeletionHandler handler(delThread.id(), app, SLOT(quit()));
	UEventsManager::addHandler(&handler);
	delThread.startDeletion(1); // make sure that app-exec() is called before the deletion of the object
	app->exec();

	ULOGGER_INFO("Closing RTAB-Map gui...");
	delete mainWindow;

	delete app;

	ULOGGER_INFO("All done!");

    return 0;
}
