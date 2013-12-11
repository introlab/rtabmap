
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UEventsManager.h>
#include <rtabmap/core/Odometry.h>
#include <rtabmap/gui/OdometryViewer.h>
#include <rtabmap/core/CameraOpenni.h>
#include <QtGui/QApplication.h>

void showUsage()
{
	printf("\nUsage:\n"
			"odometryViewer [options]\n"
			"Options:\n"
			"  -bow #                    Use bag-of-words odometry (default 0): 0=SURF, 1=SIFT\n"
			"  -bin                      Use binary odometry (FAST+BRIEF)\n"
			"  -icp                      Use ICP odometry\n"
			"\n"
			"  -in #.#                   Inliers maximum distance (default 0.005 m)\n"
			"  -max #                    Max features used for matching (default 0=inf)\n"
			"  -min #                    Minimum inliers to accept the transform (default 20)\n"
			"  -depth #.#                Maximum features depth (default 5.0 m)\n"
			"  -i #                      RANSAC/ICP iterations (default 100)\n"
			"  -lu #                     Linear update (default 0.0 m)\n"
			"  -au #                     Angular update (default 0.0 radian)\n"
			"  -reset #                  Reset countdown (default 0 = disabled)\n"
			"  -d #                      ICP decimation (default 4)\n"
			"  -v #                      ICP voxel size (default 0.005)\n"
			"  -s #                      ICP samples (default 0, not used if voxel is set.)\n"
			"  -f #.#                    ICP fitness (default 0.01)\n"
			"  -debug                    Log debug messages\n"
			"\n"
			"Examples:\n"
			"  odometryViewer -bow 0                                SURF example\n"
			"  odometryViewer -bow 1                                SIFT example\n"
			"  odometryViewer -bin                                  FAST/BRIEF example\n"
			"  odometryViewer -icp -in 0.05 -i 30                   ICP example\n");
	exit(1);
}

int main (int argc, char * argv[])
{
	ULogger::setType(ULogger::kTypeConsole);
	ULogger::setLevel(ULogger::kInfo);

	// parse arguments
	int odomType = 0; // 0=bow 1=bin 2=ICP
	int bowType = 0;
	float distance = 0.005;
	int maxWords = 0;
	int minInliers = 20;
	float maxDepth = 5.0f;
	int iterations = 100;
	float linearUpdate = 0.0f;
	float angularUpdate = 0.0f;
	int resetCountdown = 0;
	int decimation = 4;
	float voxel = 0.005;
	int samples = 10000;
	float fitness = 0.01f;

	for(int i=1; i<argc; ++i)
	{
		if(strcmp(argv[i], "-bow") == 0)
		{
			++i;
			if(i < argc)
			{
				bowType = std::atoi(argv[i]);
				odomType = 0;
				if(bowType < 0 || bowType > 1)
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
		if(strcmp(argv[i], "-in") == 0)
		{
			++i;
			if(i < argc)
			{
				distance = std::atof(argv[i]);
				if(distance <= 0)
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
		if(strcmp(argv[i], "-max") == 0)
		{
			++i;
			if(i < argc)
			{
				maxWords = std::atoi(argv[i]);
				if(maxWords < 0)
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
		if(strcmp(argv[i], "-min") == 0)
		{
			++i;
			if(i < argc)
			{
				minInliers = std::atoi(argv[i]);
				if(minInliers < 0)
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
		if(strcmp(argv[i], "-depth") == 0)
		{
			++i;
			if(i < argc)
			{
				maxDepth = std::atof(argv[i]);
				if(maxDepth < 0)
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
		if(strcmp(argv[i], "-i") == 0)
		{
			++i;
			if(i < argc)
			{
				iterations = std::atoi(argv[i]);
				if(iterations <= 0)
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
		if(strcmp(argv[i], "-lu") == 0)
		{
			++i;
			if(i < argc)
			{
				linearUpdate = std::atof(argv[i]);
				if(linearUpdate < 0.0f)
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
		if(strcmp(argv[i], "-au") == 0)
		{
			++i;
			if(i < argc)
			{
				angularUpdate = std::atof(argv[i]);
				if(angularUpdate < 0.0f)
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
		if(strcmp(argv[i], "-reset") == 0)
		{
			++i;
			if(i < argc)
			{
				resetCountdown = std::atoi(argv[i]);
				if(resetCountdown < 0)
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
		if(strcmp(argv[i], "-d") == 0)
		{
			++i;
			if(i < argc)
			{
				decimation = std::atoi(argv[i]);
				if(decimation < 1)
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
		if(strcmp(argv[i], "-v") == 0)
		{
			++i;
			if(i < argc)
			{
				voxel = std::atof(argv[i]);
				if(voxel < 0)
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
		if(strcmp(argv[i], "-s") == 0)
		{
			++i;
			if(i < argc)
			{
				samples = std::atoi(argv[i]);
				if(samples < 0)
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
		if(strcmp(argv[i], "-f") == 0)
		{
			++i;
			if(i < argc)
			{
				fitness = std::atof(argv[i]);
				if(fitness < 0.0f)
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
		if(strcmp(argv[i], "-bin") == 0)
		{
			odomType = 1;
			continue;
		}
		if(strcmp(argv[i], "-icp") == 0)
		{
			odomType = 2;
			continue;
		}
		if(strcmp(argv[i], "-debug") == 0)
		{
			ULogger::setLevel(ULogger::kDebug);
			continue;
		}

		printf("Unrecognized option : %s\n", argv[i]);
		showUsage();
	}

	UINFO("Odometry used = %s", odomType==0?bowType==0?"Bag-of-words SURF":"Bag-of-words SIFT":odomType==1?"Binary (FAST+BRIEF)":"ICP");
	UINFO("Inlier/ICP maximum correspondences distance = %f", distance);
	UINFO("Max features = %d", maxWords);
	UINFO("Min inliers = %d", minInliers);
	UINFO("RANSAC/ICP iterations = %d", iterations);
	UINFO("Max depth = %f", maxDepth);
	UINFO("Linear update = %f", linearUpdate);
	UINFO("Angular update = %f", angularUpdate);
	UINFO("Reset odometry coutdown = %d", resetCountdown);
	UINFO("Cloud decimation = %d", decimation);
	UINFO("Cloud voxel size = %f", voxel);
	UINFO("Cloud samples = %d", samples);
	UINFO("Cloud fitness = %f", fitness);

	QApplication app(argc, argv);

	rtabmap::CameraOpenni camera("", 0, rtabmap::Transform(0,0,1,0, -1,0,0,0, 0,-1,0,0));
	rtabmap::Odometry * odom = 0;

	if(odomType == 0)
	{
		odom = new rtabmap::OdometryBOW(
				bowType,
				distance,
				maxWords,
				minInliers,
				iterations,
				maxDepth,
				linearUpdate,
				angularUpdate,
				resetCountdown);
	}
	else if(odomType == 1)
	{
		odom = new rtabmap::OdometryBinary(
				distance,
				maxWords,
				minInliers,
				iterations,
				maxDepth,
				linearUpdate,
				angularUpdate,
				resetCountdown);
	}
	else // ICP
	{
		odom = new rtabmap::OdometryICP(
				decimation,
				voxel,
				samples,
				distance,
				iterations,
				fitness,
				maxDepth,
				linearUpdate,
				angularUpdate,
				resetCountdown);
	}
	rtabmap::OdometryThread odomThread(odom);
	rtabmap::OdometryViewer odomViewer(100, 2, 0.0);
	UEventsManager::addHandler(&odomThread);
	UEventsManager::addHandler(&odomViewer);

	odomViewer.setWindowTitle("Odometry viewer");
	odomViewer.setMinimumWidth(500);
	odomViewer.setMinimumHeight(300);
	odomViewer.showMaximized();

	app.processEvents();

	if(camera.init())
	{
		odomThread.start();
		camera.start();

		app.exec();

		camera.kill();
		odomThread.join(true);
	}

	return 0;
}
