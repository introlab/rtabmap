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
#include <QtCore/QDir>
#include "rtabmap/utilite/UEventsManager.h"
#include "rtabmap/core/RtabmapThread.h"
#include "rtabmap/core/Rtabmap.h"
#include "rtabmap/gui/MainWindow.h"
#include <QMessageBox>
#include "rtabmap/utilite/UObjDeletionThread.h"
#include "rtabmap/utilite/UFile.h"
#include "rtabmap/utilite/UConversion.h"
#include "ObjDeletionHandler.h"

using namespace rtabmap;

int main(int argc, char* argv[])
{
	/* Set logger type */
	ULogger::setType(ULogger::kTypeConsole);
	ULogger::setLevel(ULogger::kInfo);

	/* Create tasks */
	QApplication * app = new QApplication(argc, argv);
	app->setStyleSheet("QMessageBox { messagebox-text-interaction-flags: 5; }"); // selectable message box

	ParametersMap parameters = Parameters::parseArguments(argc, argv, true);
	MainWindow * mainWindow = new MainWindow();
    app->installEventFilter(mainWindow); // to catch FileOpen events.
    
    std::string database;
    for(int i=1; i<argc; ++i)
    {
        std::string value = uReplaceChar(argv[i], '~', UDirectory::homeDir());
        if(UFile::exists(value) &&
           UFile::getExtension(value).compare("db") == 0)
        {
            database = value;
            break;
        }
    }

	UINFO("Program started...");

	UEventsManager::addHandler(mainWindow);

	/* Start thread's task */
	if(mainWindow->isSavedMaximized())
	{
		mainWindow->showMaximized();
	}
	else
	{
		mainWindow->show();
	}

	RtabmapThread * rtabmap = new RtabmapThread(new Rtabmap());
	rtabmap->start(); // start it not initialized... will be initialized by event from the gui
	UEventsManager::addHandler(rtabmap);
    
    if(!database.empty())
    {
    	mainWindow->openDatabase(database.c_str());
    }
    if(parameters.size())
    {
    	mainWindow->updateParameters(parameters);
    }

	// Now wait for application to finish
	app->connect( app, SIGNAL( lastWindowClosed() ),
				app, SLOT( quit() ) );
	app->exec();// MUST be called by the Main Thread

	/* Remove handlers */
	UEventsManager::removeHandler(mainWindow);
	UEventsManager::removeHandler(rtabmap);

	UINFO("Killing threads...");
	rtabmap->join(true);

	UINFO("Closing RTAB-Map...");
	delete rtabmap;
	delete mainWindow;
	delete app;
	UINFO("All done!");

    return 0;
}
