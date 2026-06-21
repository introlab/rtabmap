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
#include <rtabmap/core/SensorCaptureThread.h>
#include <stdio.h>
#include <signal.h>
#include <fstream>

using namespace rtabmap;

void showUsage(const char * appName)
{
	printf("\nUsage:\n"
			"%s [options] path\n"
			"  path               Folder of the sequence (e.g., \"~/apartment1_1\")\n"
			"                        containing color, depth, groundtruth.txt, imu.txt and odom.txt.\n"
			"  --output           Output directory. By default, results are saved in \"path\".\n"
			"  --output_name      Output database name (default \"rtabmap\").\n"
			"  --max_time_diff #.#  Maximum time difference with frame to attribute a valid odometry and/or ground truth pose (default 0.1 s).\n"
			"  --quiet            Don't show log messages and iteration updates.\n"
			"  --use_imu          Use IMU.\n"
			"  --gt               Record ground truth.\n"
			"  --odom             Use wheel odometry as input guess to visual odometry.\n"
			"  --imu #            Use IMU and set filter: 0=madgwick, 1=complementary.\n"
			"  --quiet            Don't show log messages and iteration updates.\n"
			"%s\n"
			"Example:\n\n"
			"   $ %s \\\n"
			"       --Rtabmap/DetectionRate 2\\\n"
			"       --RGBD/OptimizeMaxError 5\\\n"
			"       --Mem/STMSize 30\\\n"
			"       --gt\\\n"
			"       --odom\\\n"
			"       --imu 1\\\n"
			"       ~/apartment1_1\n\n", appName, rtabmap::Parameters::showUsage(), appName);
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
	float maxTimeDiff = 0.1f;
	bool quiet = false;
	int imuFilter = 1;
	bool useImu = false;
	bool useOdom = false;
	bool recordGt = false;
	if(argc < 2)
	{
		showUsage(argv[0]);
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
			else if(std::strcmp(argv[i], "--max_time_diff") == 0)
			{
				maxTimeDiff = atof(argv[++i]);
				UASSERT(maxTimeDiff > 0.0f);
			}
			else if(std::strcmp(argv[i], "--odom") == 0)
			{
				useOdom = true;
			}
			else if(std::strcmp(argv[i], "--imu") == 0)
			{
				useImu = true;
				imuFilter = atoi(argv[++i]);
			}
			else if(std::strcmp(argv[i], "--gt") == 0)
			{
				recordGt = true;
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
	std::string pathRgbImages  = path+"/color";
	std::string pathDepthImages = path+"/depth";
	std::string pathGt = recordGt?path+"/groundtruth.txt":"";
	if(recordGt && !UFile::exists(pathGt))
	{
		UWARN("Ground truth file path doesn't exist: \"%s\", benchmark values won't be computed.", pathGt.c_str());
		pathGt.clear();
	}
	std::string pathOdom = useOdom?path+"/rtabmap_odom.txt":"";
	if(useOdom && !UFile::exists(pathOdom))
	{
		std::string orgOdom = path+"/odom.txt";
		if(UFile::exists(orgOdom))
		{
			printf("Converting odom.txt to rtabmap_odom.txt...");
			std::ifstream inputFile(orgOdom);
			std::ofstream outputFile(pathOdom);
			std::string line;

			if (!inputFile.is_open()) {
				UERROR("Error: Could not open input file: %s", orgOdom.c_str());
				return 1;
			}

			double previousStamp = 0.0;
			float odomX = 0.0f;
			float odomY = 0.0f;
			float odomTheta = 0.0f;
			while (std::getline(inputFile, line)) {
				auto strList = uListToVector(uSplit(line, ' '));
				if(line.empty())
				{
					break;
				}
				if(strList.size() != 14)
				{
					UERROR("Odometry shoud, have 14 entries per line, got %ld: \"%s\"", strList.size(), line.c_str());
					return 1;
				}
				// Recompute wheel odometry based on velocity
				double stamp = uStr2Double(strList[0]);
				if(previousStamp==0)
				{
					previousStamp = uStr2Double(strList[0]);
				}
				float dt = stamp - previousStamp;
				float vx = uStr2Float(strList[8]);
				float vtheta = uStr2Float(strList[13]);
				
				odomX += vx * cos(odomTheta) * dt;
				odomY += vx * sin(odomTheta) * dt;
				odomTheta = odomTheta + vtheta * dt;
				
				Transform t(odomX, odomY, odomTheta);
				Eigen::Quaternionf q = t.getQuaternionf();
				outputFile << strList[0] << ' ' << t.x() << ' ' << t.y() << ' ' << t.z() << ' ' << q.x() << ' ' << q.y() << ' ' << q.z() << ' ' << q.w() << std::endl;
				previousStamp = stamp;
			}

			inputFile.close();
			outputFile.close();
			printf("Converting odom.txt to rtabmap_odom.txt...done!");
		}
		else 
		{
			pathOdom.clear();
		}
	}
	std::string pathImu = useImu?path+"/imu.txt":"";
	if(useImu && !UFile::exists(pathImu))
	{
		pathImu.clear();
	}

	if(quiet)
	{
		ULogger::setLevel(ULogger::kError);
	}

	printf("Paths:\n"
			"   Dataset name:    %s\n"
			"   Dataset path:    %s\n"
			"   Color path:      %s\n"
			"   Depth path:      %s\n"
			"   Output:          %s\n"
			"   Output name:     %s\n"
			"   Max time diff:   %f\n",
			seq.c_str(),
			path.c_str(),
			pathRgbImages.c_str(),
			pathDepthImages.c_str(),
			output.c_str(),
			outputName.c_str(),
			maxTimeDiff);
	printf("   Ground Truth: %s\n", !pathGt.empty()?pathGt.c_str():"Set --gt to record ground truth");
	printf("   Odometry:     %s\n", !pathOdom.empty()?pathOdom.c_str():"Set --odom use wheel odometry");
	printf("   IMU:          %s\n", !pathImu.empty()?pathImu.c_str():"Set --imu 1 to use IMU");
	printf("   IMU Filter:   %d\n", imuFilter);
	if(!parameters.empty())
	{
		printf("Parameters:\n");
		for(ParametersMap::iterator iter=parameters.begin(); iter!=parameters.end(); ++iter)
		{
			printf("   %s=%s\n", iter->first.c_str(), iter->second.c_str());
		}
	}
	printf("RTAB-Map version: %s\n", RTABMAP_VERSION);

	// setup calibration file, based on https://cid-sims.github.io/calibration/calibration.yaml
	CameraModel model(outputName+"_calib", 
			386.52199190267083, 387.32300428823663, 
			326.5103569741365, 237.40293732598795, 
			Transform::getIdentity(), 0, cv::Size(640,480));
	std::string sequenceName = UFile(path).getName();
	// The ground truth corresponds to camera frame, thus make the camera the base frame
	Transform cameraHigh(
		0.013246,  0.0521412, 0.9982324, pathGt.empty()?0.28:0,
		-0.9962321, 0.0610127, 0.0024175, 0,
		-0.05647427, -0.9950674, 0.0591213, pathGt.empty()?0.20:0);
	Transform cameraLow(
		-0.00477153, 0.0888742, 0.995711, pathGt.empty()?0.369117:0, 
		-0.99571, 0.0665738, -0.018348, pathGt.empty()?0.0130432:0, 
		-0.0637357, -0.99188, 0.094639, pathGt.empty()?0.016175:0);
	Transform camImu(
		0.9999691, 0.00720362, -0.00314765, -0.02707507,
		-0.0071841, 0.99995517, 0.0061682, -0.004337,
		0.00319195, -0.0061454, 0.99997602, -0.01595186);
	Transform imuHigh = cameraHigh * camImu; // base->IMU
	Transform imuLow = cameraLow * camImu; // base->IMU
	Transform baseToImu;

	// Based on https://cid-sims.github.io/overview/index.html
	if( sequenceName.find("apartment1_1") != std::string::npos ||
		sequenceName.find("apartment2_1") != std::string::npos ||
		sequenceName.find("apartment2_3") != std::string::npos ||
		sequenceName.find("apartment3_1") != std::string::npos ||
		sequenceName.find("apartment3_1") != std::string::npos)
	{
		// using camera low
		model.setLocalTransform(cameraLow);
		baseToImu = imuLow;
	}
	else
	{
		// using camera high
		model.setLocalTransform(cameraHigh);
		baseToImu = imuHigh;
	}
	model.save(path);

	SensorCaptureThread cameraThread(new
		CameraRGBDImages(
				pathRgbImages,
				pathDepthImages), parameters);
	((CameraRGBDImages*)cameraThread.camera())->setTimestamps(true, "", false);
	if(!pathGt.empty())
	{
		((CameraRGBDImages*)cameraThread.camera())->setGroundTruthPath(pathGt, 1);
	}
	if(!pathOdom.empty())
	{
		((CameraRGBDImages*)cameraThread.camera())->setOdometryPath(pathOdom, 10);
	}
	((CameraRGBDImages*)cameraThread.camera())->setMaxPoseTimeDiff(maxTimeDiff);
	if(!pathImu.empty())
	{
		cameraThread.enableIMUFiltering(imuFilter, parameters);
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

		std::ifstream imu_file;

		if(!pathImu.empty())
		{
			// open the IMU file
			std::string line;
			imu_file.open(pathImu.c_str());
			if (!imu_file.good()) {
				UERROR("no imu file found at %s",pathImu.c_str());
				return -1;
			}
			int number_of_lines = 0;
			while (std::getline(imu_file, line))
				++number_of_lines;
			printf("No. IMU measurements: %d\n", number_of_lines-1);
			if (number_of_lines - 1 <= 0) {
				UERROR("no imu messages present in %s", pathImu.c_str());
				return -1;
			}
			// set reading position to second line
			imu_file.clear();
			imu_file.seekg(0, std::ios::beg);
			std::getline(imu_file, line);
		}

		UTimer totalTime;
		UTimer timer;
		SensorCaptureInfo cameraInfo;
		SensorData data = cameraThread.camera()->takeData(&cameraInfo);
		int iteration = 0;
		double start = data.stamp();

		/////////////////////////////
		// Processing dataset begin
		/////////////////////////////
		int odomKeyFrames = 0;
		double previousStamp = 0.0;
		Transform previousOdomPose;
		
		while(data.isValid() && g_forever)
		{
			// get all IMU measurements till then
			double t_imu = start;
			do {
				std::string line;
				if (!std::getline(imu_file, line)) {
					std::cout << std::endl << "Finished parsing IMU." << std::endl << std::flush;
					break;
				}

				std::stringstream stream(line);
				std::string s;
				std::getline(stream, s, ' ');
				t_imu = uStr2Double(s);

				cv::Vec3d gyr;
				for (int j = 0; j < 3; ++j) {
					std::getline(stream, s, ' ');
					gyr[j] = uStr2Double(s);
				}

				cv::Vec3d acc;
				for (int j = 0; j < 3; ++j) {
					std::getline(stream, s, ' ');
					acc[j] = uStr2Double(s);
				}

				if (t_imu - start + 1 > 0) {

					SensorData dataImu(IMU(gyr, cv::Mat(3,3,CV_64FC1), acc, cv::Mat(3,3,CV_64FC1), baseToImu), 0, t_imu);
					cameraThread.postUpdate(&dataImu);
					odom->process(dataImu);
				}

			} while (t_imu <= data.stamp());

			cameraThread.postUpdate(&data, &cameraInfo);
			cameraInfo.timeTotal = timer.ticks();

			Transform guess = (!pathOdom.empty() && !cameraInfo.odomPose.isNull() && !previousOdomPose.isNull())?previousOdomPose.inverse() * cameraInfo.odomPose:Transform();	

			OdometryInfo odomInfo;
			Transform previous = odom->getPose();
			Transform pose = odom->process(data,
				guess,
				&odomInfo);

			if(!pose.isNull() && odomInfo.reg.covariance.total() == 36)
			{
				previousOdomPose = cameraInfo.odomPose;
				if(uIsFinite(odomInfo.reg.covariance.at<double>(0,0)) &&
					odomInfo.reg.covariance.at<double>(0,0)>0.0)
				{
					if( !pathOdom.empty() && 
						odomInfo.reg.covariance.at<double>(0,0) >= 9999 && 
						!previousOdomPose.isNull() &&
						(pose.x() != 0.0f || pose.y() != 0.0f || pose.z() != 0.0f)) // not the first frame
					{
						// In case of external guess and auto reset, keep reporting lost till we
						// process the second frame with valid covariance. This way it
						// won't trigger a new map.
						pose = Transform();
					}
				}
			}

			if(odomStrategy == 2)
			{
				//special case for FOVIS, set covariance 1 if 9999 is detected
				if(!odomInfo.reg.covariance.empty() && odomInfo.reg.covariance.at<double>(0,0) >= 9999)
				{
					odomInfo.reg.covariance = cv::Mat::eye(6,6,CV_64FC1);
				}
			}
			
			if(iteration!=0 && !pose.isNull() && !odomInfo.reg.covariance.empty() && odomInfo.reg.covariance.at<double>(0,0)>=9999)
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
				externalStats.insert(std::make_pair("Camera/HistogramEqualization/ms", cameraInfo.timeHistogramEqualization*1000.0f));
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

			cameraInfo = SensorCaptureInfo();
			timer.restart();
			data = cameraThread.camera()->takeData(&cameraInfo);
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
