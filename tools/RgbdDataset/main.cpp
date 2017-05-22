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

#include <rtabmap/core/OdometryF2M.h>
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
			"  --output           Output directory. By default, results are saved in \"path\".\n\n"
			"%s\n"
			"Example:\n\n"
			"   $ rtabmap-kitti_dataset \\\n"
			"       --Vis/EstimationType 1\\\n"
			"       --Vis/BundleAdjustment 1\\\n"
			"       --Vis/PnPReprojError 1.5\\\n"
			"       --Odom/GuessMotion true\\\n"
			"       --OdomF2M/BundleAdjustment 1\\\n"
			"       --Rtabmap/CreateIntermediateNodes true\\\n"
			"       --Rtabmap/DetectionRate 1\\\n"
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
	}

	std::string pathRgbImages  = path+"/rgb_sync";
	std::string pathDepthImages = path+"/depth_sync";
	std::string pathGt = path+"/groundtruth.txt";
	if(!UFile::exists(pathGt))
	{
		UWARN("Ground truth file path doesn't exist: \"%s\", benchmark values won't be computed.", pathGt.c_str());
		pathGt.clear();
	}

	printf("Paths:\n"
			"   Dataset path:    %s\n"
			"   RGB path:        %s\n"
			"   Depth path:      %s\n"
			"   Output:          %s\n",
			path.c_str(),
			pathRgbImages.c_str(),
			pathDepthImages.c_str(),
			output.c_str());
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

	// setup calibraiton file
	CameraModel model;
	std::string sequenceName = UFile(path).getName();
	Transform opticalRotation(0,0,1,0, -1,0,0,0, 0,-1,0,0);
	float depthFactor = 5.0f;
	if(sequenceName.find("freiburg1") != std::string::npos)
	{
		model = CameraModel("rtabmap_calib", 517.3, 516.5, 318.6, 255.3, opticalRotation, 0, cv::Size(640,480));
	}
	else if(sequenceName.find("freiburg2") != std::string::npos)
	{
		model = CameraModel("rtabmap_calib", 520.9, 521.0, 325.1, 249.7, opticalRotation, 0, cv::Size(640,480));
		depthFactor = 5.208f;
	}
	else //if(sequenceName.find("freiburg3") != std::string::npos)
	{
		model = CameraModel("rtabmap_calib", 535.4, 539.2, 320.1, 247.6, opticalRotation, 0, cv::Size(640,480));
	}
	model.save(output);

	CameraThread cameraThread(new
		CameraRGBDImages(
				pathRgbImages,
				pathDepthImages,
				depthFactor,
				0.0f,
				opticalRotation), parameters);
	((CameraRGBDImages*)cameraThread.camera())->setTimestamps(true, "", false);
	if(!pathGt.empty())
	{
		((CameraRGBDImages*)cameraThread.camera())->setGroundTruthPath(pathGt, 1);
	}

	bool intermediateNodes = Parameters::defaultRtabmapCreateIntermediateNodes();
	float detectionRate = Parameters::defaultRtabmapDetectionRate();
	Parameters::parse(parameters, Parameters::kRtabmapCreateIntermediateNodes(), intermediateNodes);
	Parameters::parse(parameters, Parameters::kRtabmapDetectionRate(), detectionRate);
	std::string databasePath = output+"/rtabmap.db";
	UFile::erase(databasePath);
	if(cameraThread.camera()->init(output, "rtabmap_calib"))
	{
		int totalImages = (int)((CameraRGBDImages*)cameraThread.camera())->filenames().size();

		OdometryF2M odom(parameters);
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
		cv::Mat covariance;
		double previousStamp = 0.0;
		while(data.isValid() && g_forever)
		{
			std::map<std::string, float> externalStats;
			cameraThread.postUpdate(&data, &cameraInfo);
			cameraInfo.timeTotal = timer.ticks();

			// save camera statistics to database
			externalStats.insert(std::make_pair("Camera/BilateralFiltering/ms", cameraInfo.timeBilateralFiltering*1000.0f));
			externalStats.insert(std::make_pair("Camera/Capture/ms", cameraInfo.timeCapture*1000.0f));
			externalStats.insert(std::make_pair("Camera/Disparity/ms", cameraInfo.timeDisparity*1000.0f));
			externalStats.insert(std::make_pair("Camera/ImageDecimation/ms", cameraInfo.timeImageDecimation*1000.0f));
			externalStats.insert(std::make_pair("Camera/Mirroring/ms", cameraInfo.timeMirroring*1000.0f));
			externalStats.insert(std::make_pair("Camera/ScanFromDepth/ms", cameraInfo.timeScanFromDepth*1000.0f));
			externalStats.insert(std::make_pair("Camera/TotalTime/ms", cameraInfo.timeTotal*1000.0f));
			externalStats.insert(std::make_pair("Camera/UndistortDepth/ms", cameraInfo.timeUndistortDepth*1000.0f));

			OdometryInfo odomInfo;
			Transform pose = odom.process(data, &odomInfo);
			externalStats.insert(std::make_pair("Odometry/LocalBundle/ms", odomInfo.localBundleTime*1000.0f));
			externalStats.insert(std::make_pair("Odometry/TotalTime/ms", odomInfo.timeEstimation*1000.0f));
			externalStats.insert(std::make_pair("Odometry/Inliers/ms", odomInfo.inliers));
			externalStats.insert(std::make_pair("Odometry/Features/ms", odomInfo.features));

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
			if(covariance.empty())
			{
				covariance = odomInfo.covariance;
			}
			else
			{
				covariance += odomInfo.covariance;
			}

			timer.restart();
			if(processData)
			{
				OdometryEvent e(SensorData(), Transform(), odomInfo);
				rtabmap.process(data, pose, covariance, e.velocity(), externalStats);
				covariance = cv::Mat();
			}
			double slamTime = timer.ticks();

			++iteration;
			printf("Iteration %d/%d: camera=%dms, odom(quality=%d/%d)=%dms, slam=%dms",
					iteration, totalImages, int(cameraInfo.timeTotal*1000.0f), odomInfo.inliers, odomInfo.features, int(odomInfo.timeEstimation*1000.0f), int(slamTime*1000.0f));
			if(processData && rtabmap.getLoopClosureId()>0)
			{
				printf(" *");
			}
			printf("\n");

			cameraInfo = CameraInfo();
			timer.restart();
			data = cameraThread.camera()->takeImage(&cameraInfo);
		}
		printf("Total time=%fs\n", totalTime.ticks());
		/////////////////////////////
		// Processing dataset end
		/////////////////////////////

		// Save trajectory
		printf("Saving rtabmap_trajectory.txt ...\n");
		std::map<int, Transform> poses;
		std::multimap<int, Link> links;
		rtabmap.getGraph(poses, links, true, true);
		std::string pathTrajectory = output+"/rtabmap_poses.txt";
		if(poses.size() && graph::exportPoses(pathTrajectory, 2, poses, links))
		{
			printf("Saving %s... done!\n", pathTrajectory.c_str());
		}
		else
		{
			printf("Saving %s... failed!\n", pathTrajectory.c_str());
		}

		if(!pathGt.empty())
		{
			// Log ground truth statistics (in TUM's RGBD-SLAM format)
			std::map<int, Transform> groundTruth;

			//align with ground truth for more meaningful results
			pcl::PointCloud<pcl::PointXYZ> cloud1, cloud2;
			cloud1.resize(poses.size());
			cloud2.resize(poses.size());
			int oi = 0;
			int idFirst = 0;
			for(std::map<int, Transform>::const_iterator iter=poses.begin(); iter!=poses.end(); ++iter)
			{
				Transform o, gtPose;
				int m,w;
				std::string l;
				double s;
				std::vector<float> v;
				rtabmap.getMemory()->getNodeInfo(iter->first, o, m, w, l, s, gtPose, v, true);
				if(!gtPose.isNull())
				{
					groundTruth.insert(std::make_pair(iter->first, gtPose));
					if(oi==0)
					{
						idFirst = iter->first;
					}
					cloud1[oi] = pcl::PointXYZ(gtPose.x(), gtPose.y(), gtPose.z());
					cloud2[oi++] = pcl::PointXYZ(iter->second.x(), iter->second.y(), iter->second.z());
				}
			}

			Transform t = Transform::getIdentity();
			if(oi>5)
			{
				cloud1.resize(oi);
				cloud2.resize(oi);

				t = util3d::transformFromXYZCorrespondencesSVD(cloud2, cloud1);
			}
			else if(idFirst)
			{
				t = groundTruth.at(idFirst) * poses.at(idFirst).inverse();
			}
			if(!t.isIdentity())
			{
				for(std::map<int, Transform>::iterator iter=poses.begin(); iter!=poses.end(); ++iter)
				{
					iter->second = t * iter->second;
				}
			}

			std::vector<float> translationalErrors(poses.size());
			std::vector<float> rotationalErrors(poses.size());
			float sumTranslationalErrors = 0.0f;
			float sumRotationalErrors = 0.0f;
			float sumSqrdTranslationalErrors = 0.0f;
			float sumSqrdRotationalErrors = 0.0f;
			float radToDegree = 180.0f / M_PI;
			float translational_min = 0.0f;
			float translational_max = 0.0f;
			float rotational_min = 0.0f;
			float rotational_max = 0.0f;
			oi=0;
			for(std::map<int, Transform>::iterator iter=poses.begin(); iter!=poses.end(); ++iter)
			{
				std::map<int, Transform>::const_iterator jter = groundTruth.find(iter->first);
				if(jter!=groundTruth.end())
				{
					Eigen::Vector3f vA = iter->second.toEigen3f().rotation()*Eigen::Vector3f(1,0,0);
					Eigen::Vector3f vB = jter->second.toEigen3f().rotation()*Eigen::Vector3f(1,0,0);
					double a = pcl::getAngle3D(Eigen::Vector4f(vA[0], vA[1], vA[2], 0), Eigen::Vector4f(vB[0], vB[1], vB[2], 0));
					rotationalErrors[oi] = a*radToDegree;
					translationalErrors[oi] = iter->second.getDistance(jter->second);

					sumTranslationalErrors+=translationalErrors[oi];
					sumSqrdTranslationalErrors+=translationalErrors[oi]*translationalErrors[oi];
					sumRotationalErrors+=rotationalErrors[oi];
					sumSqrdRotationalErrors+=rotationalErrors[oi]*rotationalErrors[oi];

					if(oi == 0)
					{
						translational_min = translational_max = translationalErrors[oi];
						rotational_min = rotational_max = rotationalErrors[oi];
					}
					else
					{
						if(translationalErrors[oi] < translational_min)
						{
							translational_min = translationalErrors[oi];
						}
						else if(translationalErrors[oi] > translational_max)
						{
							translational_max = translationalErrors[oi];
						}

						if(rotationalErrors[oi] < rotational_min)
						{
							rotational_min = rotationalErrors[oi];
						}
						else if(rotationalErrors[oi] > rotational_max)
						{
							rotational_max = rotationalErrors[oi];
						}
					}

					++oi;
				}
			}
			translationalErrors.resize(oi);
			rotationalErrors.resize(oi);
			if(oi)
			{
				float total = float(oi);
				float translational_rmse = std::sqrt(sumSqrdTranslationalErrors/total);
				float translational_mean = sumTranslationalErrors/total;
				float translational_median = translationalErrors[oi/2];
				float translational_std = std::sqrt(uVariance(translationalErrors, translational_mean));

				float rotational_rmse = std::sqrt(sumSqrdRotationalErrors/total);
				float rotational_mean = sumRotationalErrors/total;
				float rotational_median = rotationalErrors[oi/2];
				float rotational_std = std::sqrt(uVariance(rotationalErrors, rotational_mean));

				printf("  translational_rmse=   %f\n", translational_rmse);
				printf("  rotational_rmse=      %f\n", rotational_rmse);

				FILE * pFile = 0;
				std::string pathErrors = output+"/rtabmap_rmse.txt";
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
	}
	else
	{
		UERROR("Camera init failed!");
	}

	printf("Saving rtabmap database (with all statistics) to \"%s\"\n", (output+"/rtabmap.db").c_str());
	printf("Do:\n"
			" $ rtabmap-databaseViewer %s\n\n", (output+"/rtabmap.db").c_str());

	return 0;
}
