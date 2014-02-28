
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UFile.h>
#include <rtabmap/core/CameraOpenni.h>
#include <rtabmap/core/Camera.h>
#include <rtabmap/core/CameraThread.h>
#include <rtabmap/gui/DataRecorder.h>
#include <QtGui/QApplication>
#include <signal.h>

using namespace rtabmap;

void showUsage()
{
	printf("\nUsage:\n"
			"dataRecorder [options] output.db\n"
			"Options:\n"
			"  -hide                    Don't display the current cloud recorded.\n"
			"  -debug                   Set debug level for the logger.\n"
			"  -rate #.#                Input rate Hz (default 0=inf)\n"
			"  -openni                  Use openni camera instead of the usb camera.\n");
	exit(1);
}

rtabmap::CameraOpenni * openniCamera = 0;
rtabmap::CameraThread * cam = 0;
QApplication * app = 0;
// catch ctrl-c
void sighandler(int sig)
{
	printf("\nSignal %d caught...\n", sig);
	if(openniCamera)
	{
		openniCamera->kill();
	}
	if(cam)
	{
		cam->join(true);
	}
	if(app)
	{
		QMetaObject::invokeMethod(app, "quit");
	}
}

int main (int argc, char * argv[])
{
	ULogger::setType(ULogger::kTypeConsole);
	ULogger::setLevel(ULogger::kInfo);

	// parse arguments
	QString fileName;
	bool show = true;
	bool openni = false;
	float rate = 0.0f;

	if(argc < 2)
	{
		showUsage();
	}
	for(int i=1; i<argc-1; ++i)
	{
		if(strcmp(argv[i], "-rate") == 0)
		{
			++i;
			if(i < argc)
			{
				rate = std::atof(argv[i]);
				if(rate < 0.0f)
				{
					showUsage();
				}
			}
			else
			{
				showUsage();
			}
			continue;
		}
		if(strcmp(argv[i], "-debug") == 0)
		{
			ULogger::setLevel(ULogger::kDebug);
			continue;
		}
		if(strcmp(argv[i], "-hide") == 0)
		{
			show = false;
			continue;
		}
		if(strcmp(argv[i], "-openni") == 0)
		{
			openni = true;
			continue;
		}

		printf("Unrecognized option : %s\n", argv[i]);
		showUsage();
	}
	fileName = argv[argc-1]; // the last is the output path

	if(UFile::getExtension(fileName.toStdString()).compare("db") != 0)
	{
		printf("Database names must end with .db extension\n");
		showUsage();
	}

	UINFO("Output = %s", fileName.toStdString().c_str());
	UINFO("Show = %s", show?"true":"false");
	UINFO("Openni = %s", openni?"true":"false");
	UINFO("Rate =%f Hz", rate);

	app = new QApplication(argc, argv);

	// Catch ctrl-c to close the gui
	// (Place this after QApplication's constructor)
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	if(openni)
	{
		openniCamera = new rtabmap::CameraOpenni("", rate, rtabmap::Transform(0,0,1,0, -1,0,0,0, 0,-1,0,0));
	}
	else
	{
		cam = new rtabmap::CameraThread(new rtabmap::CameraVideo(0, rate));
	}

	DataRecorder recorder;

	if(recorder.init(fileName))
	{
		recorder.registerToEventsManager();
		if(show)
		{
			recorder.setWindowTitle("Cloud viewer");
			recorder.setMinimumWidth(500);
			recorder.setMinimumHeight(300);
			recorder.showNormal();
			app->processEvents();
		}

		if(openni?openniCamera->init():cam->init())
		{
			openni?openniCamera->start():cam->start();

			app->exec();

			UINFO("Closing...");

			recorder.close();
		}
		else
		{
			UERROR("Cannot initialize the camera!");
		}
	}
	else
	{
		UERROR("Cannot initialize the recorder! Maybe the path is wrong: \"%s\"", fileName.toStdString().c_str());
	}

	if(openniCamera)
	{
		delete openniCamera;
	}
	if(cam)
	{
		delete cam;
	}

	return 0;
}
