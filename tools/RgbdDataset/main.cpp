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

#include <rtabmap/core/Odometry.h>
#include "rtabmap/core/Rtabmap.h"
#include "rtabmap/core/CameraRGBD.h"
#include "rtabmap/core/CameraThread.h"
#include "rtabmap/core/Graph.h"
#include "rtabmap/core/OdometryInfo.h"
#include "rtabmap/core/OdometryEvent.h"
#include "rtabmap/core/Memory.h"
#include "rtabmap/core/util3d_registration.h"
#include "rtabmap/utilite/UConversion.h"
#include "rtabmap/utilite/UDirectory.h"
#include "rtabmap/utilite/UFile.h"
#include "rtabmap/utilite/UMath.h"
#include "rtabmap/utilite/UStl.h"
#include "rtabmap/utilite/UProcessInfo.h"
#include <pcl/common/common.h>
#include <stdio.h>
#include <signal.h>

using namespace rtabmap;

void showUsage()
{
	printf("\nUsage:\n"
			"rtabmap-rgbd_dataset [options] path\n"
			"  path               Folder of the sequence (e.g., \"~/rgbd_dataset_freiburg3_long_office_household\")\n"
			"                        containing least rgb_sync and depth_sync folders. These folders contain\n"
			"                        synchronized images using associate.py tool (use tool version from\n"
			"                        https://gist.github.com/matlabbe/484134a2d9da8ad425362c6669824798). If \n"
			"                        \"groundtruth.txt\" is found in the sequence folder, they will be saved in the database.\n"
			"  --output           Output directory. By default, results are saved in \"path\".\n"
			"  --output_name      Output database name (default \"rtabmap\").\n"
			"  --skip #           Skip X frames.\n"
			"  --quiet            Don't show log messages and iteration updates.\n"
			"%s\n"
			"Example:\n\n"
			"   $ rtabmap-rgbd_dataset \\\n"
			"       --Rtabmap/PublishRAMUsage true\\\n"
			"       --Rtabmap/DetectionRate 2\\\n"
			"       --RGBD/LinearUpdate 0\\\n"
			"       --Mem/STMSize 30\\\n"
			"       ~/rgbd_dataset_freiburg3_long_office_household\n\n", rtabmap::Parameters::showUsage());
	exit(1);
}

// catch ctrl-c
bool g_forever = true;
void sighandler(int sig)
{
	printf("\nSignal %d caught...\n", sig);
	g_forever = false;
}

int main(int argc, char * argv[])
{
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	ULogger::setType(ULogger::kTypeConsole);
	ULogger::setLevel(ULogger::kWarning);

	ParametersMap parameters;
	std::string path;
	std::string output;
	std::string outputName = "rtabmap";
	int skipFrames = 0;
	bool quiet = false;
	if(argc < 2)
	{
		showUsage();
	}
	else
	{
		for(int i=1; i<argc; ++i)
		{
			if(std::strcmp(argv[i], "--output") == 0)
			{
				output = argv[++i];
			}
			else if(std::strcmp(argv[i], "--output_name") == 0)
			{
				outputName = argv[++i];
			}
			else if(std::strcmp(argv[i], "--skip") == 0)
			{
				skipFrames = atoi(argv[++i]);
				UASSERT(skipFrames > 0);
			}
			else if(std::strcmp(argv[i], "--quiet") == 0)
			{
				quiet = true;
			}
		}
		parameters = Parameters::parseArguments(argc, argv);
		path = argv[argc-1];
		path = uReplaceChar(path, '~', UDirectory::homeDir());
		path = uReplaceChar(path, '\\', '/');
		if(output.empty())
		{
			output = path;
		}
		else
		{
			output = uReplaceChar(output, '~', UDirectory::homeDir());
			UDirectory::makeDir(output);
		}
		parameters.insert(ParametersPair(Parameters::kRtabmapWorkingDirectory(), output));
		parameters.insert(ParametersPair(Parameters::kRtabmapPublishRAMUsage(), "true"));
	}

	std::string seq = uSplit(path, '/').back();
	std::string pathRgbImages  = path+"/rgb_sync";
	std::string pathDepthImages = path+"/depth_sync";
	std::string pathGt = path+"/groundtruth.txt";
	if(!UFile::exists(pathGt))
	{
		UWARN("Ground truth file path doesn't exist: \"%s\", benchmark values won't be computed.", pathGt.c_str());
		pathGt.clear();
	}

	if(quiet)
	{
		ULogger::setLevel(ULogger::kError);
	}

	printf("Paths:\n"
			"   Dataset name:    %s\n"
			"   Dataset path:    %s\n"
			"   RGB path:        %s\n"
			"   Depth path:      %s\n"
			"   Output:          %s\n"
			"   Output name:     %s\n"
			"   Skip frames:     %d\n",
			seq.c_str(),
			path.c_str(),
			pathRgbImages.c_str(),
			pathDepthImages.c_str(),
			output.c_str(),
			outputName.c_str(),
			skipFrames);
	if(!pathGt.empty())
	{
		printf("   groundtruth.txt: %s\n", pathGt.c_str());
	}
	if(!parameters.empty())
	{
		printf("Parameters:\n");
		for(ParametersMap::iterator iter=parameters.begin(); iter!=parameters.end(); ++iter)
		{
			printf("   %s=%s\n", iter->first.c_str(), iter->second.c_str());
		}
	}
	printf("RTAB-Map version: %s\n", RTABMAP_VERSION);

	// setup calibration file
	CameraModel model;
	std::string sequenceName = UFile(path).getName();
	Transform opticalRotation(0,0,1,0, -1,0,0,0, 0,-1,0,0);
	float depthFactor = 5.0f;
	if(sequenceName.find("freiburg1") != std::string::npos)
	{
		model = CameraModel(outputName+"_calib", 517.3, 516.5, 318.6, 255.3, opticalRotation, 0, cv::Size(640,480));
	}
	else if(sequenceName.find("freiburg2") != std::string::npos)
	{
		model = CameraModel(outputName+"_calib", 520.9, 521.0, 325.1, 249.7, opticalRotation, 0, cv::Size(640,480));
	}
	else //if(sequenceName.find("freiburg3") != std::string::npos)
	{
		model = CameraModel(outputName+"_calib", 535.4, 539.2, 320.1, 247.6, opticalRotation, 0, cv::Size(640,480));
	}
	//parameters.insert(ParametersPair(Parameters::kg2oBaseline(), uNumber2Str(40.0f/model.fx())));
	model.save(path);

	CameraThread cameraThread(new
		CameraRGBDImages(
				pathRgbImages,
				pathDepthImages,
				depthFactor,
				0.0f), parameters);
	((CameraRGBDImages*)cameraThread.camera())->setTimestamps(true, "", false);
	if(!pathGt.empty())
	{
		((CameraRGBDImages*)cameraThread.camera())->setGroundTruthPath(pathGt, 1);
	}

	bool intermediateNodes = Parameters::defaultRtabmapCreateIntermediateNodes();
	float detectionRate = Parameters::defaultRtabmapDetectionRate();
	int odomStrategy = Parameters::defaultOdomStrategy();
	Parameters::parse(parameters, Parameters::kRtabmapCreateIntermediateNodes(), intermediateNodes);
	Parameters::parse(parameters, Parameters::kOdomStrategy(), odomStrategy);
	Parameters::parse(parameters, Parameters::kRtabmapDetectionRate(), detectionRate);
	std::string databasePath = output+"/"+outputName+".db";
	UFile::erase(databasePath);
	if(cameraThread.camera()->init(path, outputName+"_calib"))
	{
		int totalImages = (int)((CameraRGBDImages*)cameraThread.camera())->filenames().size();

		if(skipFrames>0)
		{
			totalImages /= skipFrames+1;
		}

		printf("Processing %d images...\n", totalImages);

		ParametersMap odomParameters = parameters;
		odomParameters.erase(Parameters::kRtabmapPublishRAMUsage()); // as odometry is in the same process than rtabmap, don't get RAM usage in odometry.
		Odometry * odom = Odometry::create(odomParameters);
		Rtabmap rtabmap;
		rtabmap.init(parameters, databasePath);

		UTimer totalTime;
		UTimer timer;
		CameraInfo cameraInfo;
		SensorData data = cameraThread.camera()->takeImage(&cameraInfo);
		int iteration = 0;

		/////////////////////////////
		// Processing dataset begin
		/////////////////////////////
		int odomKeyFrames = 0;
		double previousStamp = 0.0;
		int skipCount = 0;
		while(data.isValid() && g_forever)
		{
			if(skipCount < skipFrames)
			{
				++skipCount;

				cameraInfo = CameraInfo();
				timer.restart();
				data = cameraThread.camera()->takeImage(&cameraInfo);
				continue;
			}
			skipCount = 0;

			cameraThread.postUpdate(&data, &cameraInfo);
			cameraInfo.timeTotal = timer.ticks();

			OdometryInfo odomInfo;
			Transform pose = odom->process(data, &odomInfo);

			if(odomStrategy == 2)
			{
				//special case for FOVIS, set covariance 1 if 9999 is detected
				if(!odomInfo.reg.covariance.empty() && odomInfo.reg.covariance.at<double>(0,0) >= 9999)
				{
					odomInfo.reg.covariance = cv::Mat::eye(6,6,CV_64FC1);
				}
			}
			
			if(iteration!=0 && !odomInfo.reg.covariance.empty() && odomInfo.reg.covariance.at<double>(0,0)>=9999)
            {
				UWARN("Odometry is reset (high variance (%f >=9999 detected). Increment map id!", odomInfo.reg.covariance.at<double>(0,0));
				rtabmap.triggerNewMap();
            }

			if(odomInfo.keyFrameAdded)
			{
				++odomKeyFrames;
			}

			bool processData = true;
			if(detectionRate>0.0f &&
				previousStamp>0.0 &&
				data.stamp()>previousStamp && data.stamp() - previousStamp < 1.0/detectionRate)
			{
				processData = false;
			}

			if(processData)
			{
				previousStamp = data.stamp();
			}

			if(!processData)
			{
				// set negative id so rtabmap will detect it as an intermediate node
				data.setId(-1);
				data.setFeatures(std::vector<cv::KeyPoint>(), std::vector<cv::Point3f>(), cv::Mat());// remove features
				processData = intermediateNodes;
			}

			timer.restart();
			if(processData)
			{
				std::map<std::string, float> externalStats;
				// save camera statistics to database
				externalStats.insert(std::make_pair("Camera/BilateralFiltering/ms", cameraInfo.timeBilateralFiltering*1000.0f));
				externalStats.insert(std::make_pair("Camera/Capture/ms", cameraInfo.timeCapture*1000.0f));
				externalStats.insert(std::make_pair("Camera/Disparity/ms", cameraInfo.timeDisparity*1000.0f));
				externalStats.insert(std::make_pair("Camera/ImageDecimation/ms", cameraInfo.timeImageDecimation*1000.0f));
				externalStats.insert(std::make_pair("Camera/Mirroring/ms", cameraInfo.timeMirroring*1000.0f));
				externalStats.insert(std::make_pair("Camera/ExposureCompensation/ms", cameraInfo.timeStereoExposureCompensation*1000.0f));
				externalStats.insert(std::make_pair("Camera/ScanFromDepth/ms", cameraInfo.timeScanFromDepth*1000.0f));
				externalStats.insert(std::make_pair("Camera/TotalTime/ms", cameraInfo.timeTotal*1000.0f));
				externalStats.insert(std::make_pair("Camera/UndistortDepth/ms", cameraInfo.timeUndistortDepth*1000.0f));
				// save odometry statistics to database
				externalStats.insert(std::make_pair("Odometry/LocalBundle/ms", odomInfo.localBundleTime*1000.0f));
				externalStats.insert(std::make_pair("Odometry/LocalBundleConstraints/", odomInfo.localBundleConstraints));
				externalStats.insert(std::make_pair("Odometry/LocalBundleOutliers/", odomInfo.localBundleOutliers));
				externalStats.insert(std::make_pair("Odometry/TotalTime/ms", odomInfo.timeEstimation*1000.0f));
				externalStats.insert(std::make_pair("Odometry/Registration/ms", odomInfo.reg.totalTime*1000.0f));
				externalStats.insert(std::make_pair("Odometry/Inliers/", odomInfo.reg.inliers));
				externalStats.insert(std::make_pair("Odometry/Features/", odomInfo.features));
				externalStats.insert(std::make_pair("Odometry/DistanceTravelled/m", odomInfo.distanceTravelled));
				externalStats.insert(std::make_pair("Odometry/KeyFrameAdded/", odomInfo.keyFrameAdded));
				externalStats.insert(std::make_pair("Odometry/LocalKeyFrames/", odomInfo.localKeyFrames));
				externalStats.insert(std::make_pair("Odometry/LocalMapSize/", odomInfo.localMapSize));
				externalStats.insert(std::make_pair("Odometry/LocalScanMapSize/", odomInfo.localScanMapSize));

				OdometryEvent e(SensorData(), Transform(), odomInfo);
				rtabmap.process(data, pose, odomInfo.reg.covariance, e.velocity(), externalStats);
			}

			++iteration;
			if(!quiet || iteration == totalImages)
			{
				double slamTime = timer.ticks();

				float rmse = -1;
				if(rtabmap.getStatistics().data().find(Statistics::kGtTranslational_rmse()) != rtabmap.getStatistics().data().end())
				{
					rmse = rtabmap.getStatistics().data().at(Statistics::kGtTranslational_rmse());
				}

				if(rmse >= 0.0f)
				{
					printf("Iteration %d/%d: camera=%dms, odom(quality=%d/%d, kfs=%d)=%dms, slam=%dms, rmse=%fm",
							iteration, totalImages, int(cameraInfo.timeTotal*1000.0f), odomInfo.reg.inliers, odomInfo.features, odomKeyFrames, int(odomInfo.timeEstimation*1000.0f), int(slamTime*1000.0f), rmse);
				}
				else
				{
					printf("Iteration %d/%d: camera=%dms, odom(quality=%d/%d, kfs=%d)=%dms, slam=%dms",
							iteration, totalImages, int(cameraInfo.timeTotal*1000.0f), odomInfo.reg.inliers, odomInfo.features, odomKeyFrames, int(odomInfo.timeEstimation*1000.0f), int(slamTime*1000.0f));
				}
				if(processData && rtabmap.getLoopClosureId()>0)
				{
					printf(" *");
				}
				printf("\n");
			}
			else if(iteration % (totalImages/10) == 0)
			{
				printf(".");
				fflush(stdout);
			}

			cameraInfo = CameraInfo();
			timer.restart();
			data = cameraThread.camera()->takeImage(&cameraInfo);
		}
		delete odom;
		printf("Total time=%fs\n", totalTime.ticks());
		/////////////////////////////
		// Processing dataset end
		/////////////////////////////

		// Save trajectory
		printf("Saving trajectory...\n");
		std::map<int, Transform> poses;
		std::multimap<int, Link> links;
		std::map<int, Signature> signatures;
		std::map<int, double> stamps;
		rtabmap.getGraph(poses, links, true, true, &signatures);
		for(std::map<int, Signature>::iterator iter=signatures.begin(); iter!=signatures.end(); ++iter)
		{
			stamps.insert(std::make_pair(iter->first, iter->second.getStamp()));
		}
		std::string pathTrajectory = output+"/"+outputName+"_poses.txt";
		if(poses.size() && graph::exportPoses(pathTrajectory, 1, poses, links, stamps))
		{
			printf("Saving %s... done!\n", pathTrajectory.c_str());
		}
		else
		{
			printf("Saving %s... failed!\n", pathTrajectory.c_str());
		}

		if(!pathGt.empty())
		{
			// Log ground truth statistics
			std::map<int, Transform> groundTruth;

			for(std::map<int, Transform>::const_iterator iter=poses.begin(); iter!=poses.end(); ++iter)
			{
				Transform o, gtPose;
				int m,w;
				std::string l;
				double s;
				std::vector<float> v;
				GPS gps;
				EnvSensors sensors;
				rtabmap.getMemory()->getNodeInfo(iter->first, o, m, w, l, s, gtPose, v, gps, sensors, true);
				if(!gtPose.isNull())
				{
					groundTruth.insert(std::make_pair(iter->first, gtPose));
				}
			}


			// compute RMSE statistics
			float translational_rmse = 0.0f;
			float translational_mean = 0.0f;
			float translational_median = 0.0f;
			float translational_std = 0.0f;
			float translational_min = 0.0f;
			float translational_max = 0.0f;
			float rotational_rmse = 0.0f;
			float rotational_mean = 0.0f;
			float rotational_median = 0.0f;
			float rotational_std = 0.0f;
			float rotational_min = 0.0f;
			float rotational_max = 0.0f;
			graph::calcRMSE(
					groundTruth,
					poses,
					translational_rmse,
					translational_mean,
					translational_median,
					translational_std,
					translational_min,
					translational_max,
					rotational_rmse,
					rotational_mean,
					rotational_median,
					rotational_std,
					rotational_min,
					rotational_max);

			printf("   translational_rmse=   %f m\n", translational_rmse);
			printf("   rotational_rmse=      %f deg\n", rotational_rmse);

			FILE * pFile = 0;
			std::string pathErrors = output+"/"+outputName+"_rmse.txt";
			pFile = fopen(pathErrors.c_str(),"w");
			if(!pFile)
			{
				UERROR("could not save RMSE results to \"%s\"", pathErrors.c_str());
			}
			fprintf(pFile, "Ground truth comparison:\n");
			fprintf(pFile, "  translational_rmse=   %f\n", translational_rmse);
			fprintf(pFile, "  translational_mean=   %f\n", translational_mean);
			fprintf(pFile, "  translational_median= %f\n", translational_median);
			fprintf(pFile, "  translational_std=    %f\n", translational_std);
			fprintf(pFile, "  translational_min=    %f\n", translational_min);
			fprintf(pFile, "  translational_max=    %f\n", translational_max);
			fprintf(pFile, "  rotational_rmse=      %f\n", rotational_rmse);
			fprintf(pFile, "  rotational_mean=      %f\n", rotational_mean);
			fprintf(pFile, "  rotational_median=    %f\n", rotational_median);
			fprintf(pFile, "  rotational_std=       %f\n", rotational_std);
			fprintf(pFile, "  rotational_min=       %f\n", rotational_min);
			fprintf(pFile, "  rotational_max=       %f\n", rotational_max);
			fclose(pFile);
		}
	}
	else
	{
		UERROR("Camera init failed!");
	}

	printf("Saving rtabmap database (with all statistics) to \"%s\"\n", (output+"/"+outputName+".db").c_str());
	printf("Do:\n"
			" $ rtabmap-databaseViewer %s\n\n", (output+"/"+outputName+".db").c_str());

	return 0;
}
