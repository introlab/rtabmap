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
