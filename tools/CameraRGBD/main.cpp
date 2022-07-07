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

#include "rtabmap/core/CameraRGBD.h"
#include "rtabmap/core/CameraStereo.h"
#include "rtabmap/core/util2d.h"
#include "rtabmap/core/util3d.h"
#include "rtabmap/core/util3d_transforms.h"
#include "rtabmap/utilite/ULogger.h"
#include "rtabmap/utilite/UMath.h"
#include "rtabmap/utilite/UFile.h"
#include "rtabmap/utilite/UDirectory.h"
#include "rtabmap/utilite/UConversion.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>
#if CV_MAJOR_VERSION >= 3
#include <opencv2/videoio/videoio_c.h>
#endif
#include <pcl/visualization/cloud_viewer.h>
#include <stdio.h>
#include <signal.h>

void showUsage()
{
	printf("\nUsage:\n"
			"rtabmap-rgbd_camera [options] driver\n"
			"  driver       Driver number to use: 0=OpenNI-PCL (Kinect)\n"
			"                                     1=OpenNI2    (Kinect and Xtion PRO Live)\n"
			"                                     2=Freenect   (Kinect)\n"
			"                                     3=OpenNI-CV  (Kinect)\n"
			"                                     4=OpenNI-CV-ASUS (Xtion PRO Live)\n"
			"                                     5=Freenect2  (Kinect v2)\n"
			"                                     6=DC1394     (Bumblebee2)\n"
			"                                     7=FlyCapture2 (Bumblebee2)\n"
			"                                     8=ZED stereo\n"
			"                                     9=RealSense\n"
			"                                     10=Kinect for Windows 2 SDK\n"
			"                                     11=RealSense2\n"
			"                                     12=Kinect for Azure SDK\n"
			"                                     13=MYNT EYE S\n"
			"                                     14=ZED Open Capture\n"
			"                                     15=depthai-core\n"
			"  Options:\n"
			"      -rate #.#                      Input rate Hz (default 0=inf)\n"
			"      -device #                      Device ID (number or string)\n"
			"      -save_stereo \"path\"            Save stereo images in a folder or a video file (side by side *.avi).\n"
			"      -fourcc \"XXXX\"               Four characters FourCC code (default is \"MJPG\") used\n"
			"                                       when saving stereo images to a video file.\n"
			"                                       See http://www.fourcc.org/codecs.php for more codes.\n");
	exit(1);
}

// catch ctrl-c
bool running = true;
void sighandler(int sig)
{
	printf("\nSignal %d caught...\n", sig);
	running = false;
}

int main(int argc, char * argv[])
{
	ULogger::setType(ULogger::kTypeConsole);
	ULogger::setLevel(ULogger::kInfo);
	//ULogger::setPrintTime(false);
	//ULogger::setPrintWhere(false);

	int driver = 0;
	std::string stereoSavePath;
	float rate = 0.0f;
	std::string fourcc = "MJPG";
	std::string deviceId;
	if(argc < 2)
	{
		showUsage();
	}
	else
	{
		for(int i=1; i<argc; ++i)
		{
			if(strcmp(argv[i], "-rate") == 0)
			{
				++i;
				if(i < argc)
				{
					rate = uStr2Float(argv[i]);
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
			if (strcmp(argv[i], "-device") == 0)
			{
				++i;
				if (i < argc)
				{
					deviceId = argv[i];
				}
				else
				{
					showUsage();
				}
				continue;
			}
			if(strcmp(argv[i], "-save_stereo") == 0)
			{
				++i;
				if(i < argc)
				{
					stereoSavePath = argv[i];
				}
				else
				{
					showUsage();
				}
				continue;
			}
			if(strcmp(argv[i], "-fourcc") == 0)
			{
				++i;
				if(i < argc)
				{
					fourcc = argv[i];
					if(fourcc.size() != 4)
					{
						UERROR("fourcc should be 4 characters.");
						showUsage();
					}
				}
				else
				{
					showUsage();
				}
				continue;
			}
			if(strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-help") == 0)
			{
				showUsage();
			}
			else if(i< argc-1)
			{
				printf("Unrecognized option \"%s\"", argv[i]);
				showUsage();
			}

			// last
			driver = atoi(argv[i]);
			if(driver < 0 || driver > 15)
			{
				UERROR("driver should be between 0 and 15.");
				showUsage();
			}
		}
	}
	UINFO("Using driver %d (device=%s)", driver, deviceId.empty()?"0": deviceId.c_str());

	rtabmap::Camera * camera = 0;
	if(driver < 6)
	{
		if(!stereoSavePath.empty())
		{
			UWARN("-save_stereo option cannot be used with RGB-D drivers.");
			stereoSavePath.clear();
		}

		if(driver == 0)
		{
			camera = new rtabmap::CameraOpenni(deviceId);
		}
		else if(driver == 1)
		{
			if(!rtabmap::CameraOpenNI2::available())
			{
				UERROR("Not built with OpenNI2 support...");
				exit(-1);
			}
			camera = new rtabmap::CameraOpenNI2(deviceId);
		}
		else if(driver == 2)
		{
			if(!rtabmap::CameraFreenect::available())
			{
				UERROR("Not built with Freenect support...");
				exit(-1);
			}
			camera = new rtabmap::CameraFreenect(deviceId.empty()?0:uStr2Int(deviceId));
		}
		else if(driver == 3)
		{
			if(!rtabmap::CameraOpenNICV::available())
			{
				UERROR("Not built with OpenNI from OpenCV support...");
				exit(-1);
			}
			camera = new rtabmap::CameraOpenNICV(false);
		}
		else if(driver == 4)
		{
			if(!rtabmap::CameraOpenNICV::available())
			{
				UERROR("Not built with OpenNI from OpenCV support...");
				exit(-1);
			}
			camera = new rtabmap::CameraOpenNICV(true);
		}
		else if(driver == 5)
		{
			if(!rtabmap::CameraFreenect2::available())
			{
				UERROR("Not built with Freenect2 support...");
				exit(-1);
			}
			camera = new rtabmap::CameraFreenect2(deviceId.empty()?0:uStr2Int(deviceId), rtabmap::CameraFreenect2::kTypeColor2DepthSD);
		}
	}
	else if(driver == 6)
	{
		if(!rtabmap::CameraStereoDC1394::available())
		{
			UERROR("Not built with DC1394 support...");
			exit(-1);
		}
		camera = new rtabmap::CameraStereoDC1394();
	}
	else if(driver == 7)
	{
		if(!rtabmap::CameraStereoFlyCapture2::available())
		{
			UERROR("Not built with FlyCapture2/Triclops support...");
			exit(-1);
		}
		camera = new rtabmap::CameraStereoFlyCapture2();
	}
	else if(driver == 8)
	{
		if(!rtabmap::CameraStereoZed::available())
		{
			UERROR("Not built with ZED sdk support...");
			exit(-1);
		}
		camera = new rtabmap::CameraStereoZed(deviceId.empty()?0:uStr2Int(deviceId));
	}
	else if (driver == 9)
	{
		if (!rtabmap::CameraRealSense::available())
		{
			UERROR("Not built with RealSense support...");
			exit(-1);
		}
		camera = new rtabmap::CameraRealSense(deviceId.empty()?0:uStr2Int(deviceId));
	}
	else if (driver == 10)
	{
		if (!rtabmap::CameraK4W2::available())
		{
			UERROR("Not built with Kinect for Windows 2 SDK support...");
			exit(-1);
		}
		camera = new rtabmap::CameraK4W2(deviceId.empty()?0:uStr2Int(deviceId));
	}
	else if (driver == 11)
	{
		if (!rtabmap::CameraRealSense2::available())
		{
			UERROR("Not built with RealSense2 SDK support...");
			exit(-1);
		}
		camera = new rtabmap::CameraRealSense2(deviceId);
	}
	else if (driver == 12)
	{
		if (!rtabmap::CameraK4A::available())
		{
			UERROR("Not built with Kinect for Azure SDK support...");
			exit(-1);
		}
		camera = new rtabmap::CameraK4A(1);
	}
	else if (driver == 13)
	{
		if (!rtabmap::CameraMyntEye::available())
		{
			UERROR("Not built with Mynt Eye S support...");
			exit(-1);
		}
		camera = new rtabmap::CameraMyntEye(deviceId);
	}
	else if (driver == 14)
	{
		if (!rtabmap::CameraStereoZedOC::available())
		{
			UERROR("Not built with Zed Open Capture support...");
			exit(-1);
		}
		camera = new rtabmap::CameraStereoZedOC(deviceId.empty()?-1:uStr2Int(deviceId));
	}
	else if (driver == 15)
	{
		if (!rtabmap::CameraDepthAI::available())
		{
			UERROR("Not built with depthai-core support...");
			exit(-1);
		}
		camera = new rtabmap::CameraDepthAI(deviceId);
	}
	else
	{
		UFATAL("");
	}

	if(!camera->init())
	{
		printf("Camera init failed! Please select another driver (see \"--help\").\n");
		delete camera;
		exit(1);
	}

	rtabmap::SensorData data = camera->takeImage();
	if (data.imageRaw().empty())
	{
		printf("Cloud not get frame from the camera!\n");
		delete camera;
		exit(1);
	}
	if(data.imageRaw().cols % data.depthOrRightRaw().cols != 0 || data.imageRaw().rows % data.depthOrRightRaw().rows != 0)
	{
		UWARN("RGB (%d/%d) and depth (%d/%d) frames are not the same size! The registered cloud cannot be shown.",
				data.imageRaw().cols, data.imageRaw().rows, data.depthOrRightRaw().cols, data.depthOrRightRaw().rows);
	}
	pcl::visualization::CloudViewer * viewer = 0;
	if((data.stereoCameraModels().empty() || data.stereoCameraModels()[0].isValidForProjection()) &&
	   (data.cameraModels().empty() || !data.cameraModels()[0].isValidForProjection()))
	{
		UWARN("Camera not calibrated! The registered cloud cannot be shown.");
	}
	else
	{
		viewer = new pcl::visualization::CloudViewer("cloud");
	}

	cv::VideoWriter videoWriter;
	UDirectory dir;
	if(!stereoSavePath.empty() &&
	   !data.imageRaw().empty() &&
	   !data.rightRaw().empty())
	{
		if(UFile::getExtension(stereoSavePath).compare("avi") == 0)
		{
			if(data.imageRaw().size() == data.rightRaw().size())
			{
				if(rate <= 0)
				{
					UERROR("You should set the input rate when saving stereo images to a video file.");
					showUsage();
				}
				cv::Size targetSize = data.imageRaw().size();
				targetSize.width *= 2;
				UASSERT(fourcc.size() == 4);
				videoWriter.open(
						stereoSavePath,
						CV_FOURCC(fourcc.at(0), fourcc.at(1), fourcc.at(2), fourcc.at(3)),
						rate,
						targetSize,
						data.imageRaw().channels() == 3);
			}
			else
			{
				UERROR("Images not the same size, cannot save stereo images to the video file.");
			}
		}
		else if(UDirectory::exists(stereoSavePath))
		{
			UDirectory::makeDir(stereoSavePath+"/"+"left");
			UDirectory::makeDir(stereoSavePath+"/"+"right");
		}
		else
		{
			UERROR("Directory \"%s\" doesn't exist.", stereoSavePath.c_str());
			stereoSavePath.clear();
		}
	}

	// to catch the ctrl-c
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	int id=1;
	while(!data.imageRaw().empty() && (viewer==0 || !viewer->wasStopped()) && running)
	{
		cv::Mat rgb = data.imageRaw();
		if(!data.depthRaw().empty() && (data.depthRaw().type() == CV_16UC1 || data.depthRaw().type() == CV_32FC1))
		{
			// depth
			cv::Mat depth = data.depthRaw();
			if(depth.type() == CV_32FC1)
			{
				depth = rtabmap::util2d::cvtDepthFromFloat(depth);
			}

			if(!rgb.empty() && rgb.cols % depth.cols == 0 && rgb.rows % depth.rows == 0 &&
					data.cameraModels().size() &&
					data.cameraModels()[0].isValidForProjection())
			{
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = rtabmap::util3d::cloudFromDepthRGB(
						rgb, depth,
						data.cameraModels()[0]);
				cloud = rtabmap::util3d::transformPointCloud(cloud, rtabmap::Transform::opengl_T_rtabmap()*data.cameraModels()[0].localTransform());
				if(viewer)
					viewer->showCloud(cloud, "cloud");
			}
			else if(!depth.empty() &&
					data.cameraModels().size() &&
					data.cameraModels()[0].isValidForProjection())
			{
				pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = rtabmap::util3d::cloudFromDepth(
						depth,
						data.cameraModels()[0]);
				cloud = rtabmap::util3d::transformPointCloud(cloud, rtabmap::Transform::opengl_T_rtabmap()*data.cameraModels()[0].localTransform());
				viewer->showCloud(cloud, "cloud");
			}

			cv::Mat tmp;
			unsigned short min=0, max = 2048;
			uMinMax((unsigned short*)depth.data, depth.rows*depth.cols, min, max);
			depth.convertTo(tmp, CV_8UC1, 255.0/max);

			cv::imshow("Video", rgb); // show frame
			cv::imshow("Depth", tmp);
		}
		else if(!data.rightRaw().empty())
		{
			// stereo
			cv::Mat right = data.rightRaw();
			cv::imshow("Left", rgb); // show frame
			cv::imshow("Right", right);

			if(rgb.cols == right.cols && rgb.rows == right.rows && data.stereoCameraModels().size()==1 && data.stereoCameraModels()[0].isValidForProjection())
			{
				if(right.channels() == 3)
				{
					cv::cvtColor(right, right, CV_BGR2GRAY);
				}
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = rtabmap::util3d::cloudFromStereoImages(
						rgb, right,
						data.stereoCameraModels()[0]);
				cloud = rtabmap::util3d::transformPointCloud(cloud, rtabmap::Transform::opengl_T_rtabmap()*data.stereoCameraModels()[0].localTransform());
				if(viewer)
					viewer->showCloud(cloud, "cloud");
			}
		}

		int c = cv::waitKey(10); // wait 10 ms or for key stroke
		if(c == 27)
			break; // if ESC, break and quit

		if(videoWriter.isOpened())
		{
			cv::Mat left = data.imageRaw();
			cv::Mat right = data.rightRaw();
			if(left.size() == right.size())
			{
				cv::Size targetSize = left.size();
				targetSize.width *= 2;
				cv::Mat targetImage(targetSize, left.type());
				if(right.type() != left.type())
				{
					cv::Mat tmp;
					cv::cvtColor(right, tmp, left.channels()==3?CV_GRAY2BGR:CV_BGR2GRAY);
					right = tmp;
				}
				UASSERT(left.type() == right.type());

				cv::Mat roiA(targetImage, cv::Rect( 0, 0, left.size().width, left.size().height ));
				left.copyTo(roiA);
				cv::Mat roiB( targetImage, cvRect( left.size().width, 0, left.size().width, left.size().height ) );
				right.copyTo(roiB);

				videoWriter.write(targetImage);
				printf("Saved frame %d to \"%s\"\n", id, stereoSavePath.c_str());
			}
			else
			{
				UERROR("Left and right images are not the same size!?");
			}
		}
		else if(!stereoSavePath.empty())
		{
			cv::imwrite(stereoSavePath+"/"+"left/"+uNumber2Str(id) + ".jpg", data.imageRaw());
			cv::imwrite(stereoSavePath+"/"+"right/"+uNumber2Str(id) + ".jpg", data.rightRaw());
			printf("Saved frames %d to \"%s/left\" and \"%s/right\" directories\n", id, stereoSavePath.c_str(), stereoSavePath.c_str());
		}
		++id;
		data = camera->takeImage();
	}
	printf("Closing...\n");
	if(viewer)
	{
		delete viewer;
	}
	cv::destroyWindow("Video");
	cv::destroyWindow("Depth");
	delete camera;
	return 0;
}
