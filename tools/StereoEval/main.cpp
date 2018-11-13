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

#include "io.h"
#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/Features2d.h>
#include <rtabmap/core/util2d.h>
#include <rtabmap/core/CameraModel.h>
#include <rtabmap/core/Stereo.h>
#include <rtabmap/core/StereoCameraModel.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/utilite/UMath.h>
#include <opencv2/imgproc/types_c.h>
#include <fstream>
#include <string>

using namespace rtabmap;

void showUsage()
{
	printf("Usage:\n"
			"evalStereo.exe left.png right.png calib.txt disp.pfm mask.png [Parameters]\n"
			"Example (with http://vision.middlebury.edu/stereo datasets):\n"
			"  $ ./rtabmap-stereoEval im0.png im1.png calib.txt disp0GT.pfm mask0nocc.png -Kp/DetectorStrategy 6 -Stereo/WinSize 5 -Stereo/MaxLevel 2 -Kp/WordsPerImage 1000 -Stereo/OpticalFlow false -Stereo/Iterations 5\n\n");
	exit(1);
}

int main(int argc, char * argv[])
{
	ULogger::setLevel(ULogger::kDebug);
	ULogger::setType(ULogger::kTypeConsole);

	if(argc < 6)
	{
		showUsage();
	}

	ParametersMap parameters = Parameters::getDefaultParameters();
	for(int i=6; i<argc; ++i)
	{
		// Check for RTAB-Map's parameters
		std::string key = argv[i];
		key = uSplit(key, '-').back();
		if(parameters.find(key) != parameters.end())
		{
			++i;
			if(i < argc)
			{
				std::string value = argv[i];
				if(value.empty())
				{
					showUsage();
				}
				else
				{
					value = uReplaceChar(value, ',', ' ');
				}
				std::pair<ParametersMap::iterator, bool> inserted = parameters.insert(ParametersPair(key, value));
				if(inserted.second == false)
				{
					inserted.first->second = value;
				}
			}
			else
			{
				showUsage();
			}
			continue;
		}

		//backward compatibility
		// look for old parameter name
		std::map<std::string, std::pair<bool, std::string> >::const_iterator oldIter = Parameters::getRemovedParameters().find(key);
		if(oldIter!=Parameters::getRemovedParameters().end())
		{
			++i;
			if(i < argc)
			{
				std::string value = argv[i];
				if(value.empty())
				{
					showUsage();
				}
				else
				{
					value = uReplaceChar(value, ',', ' ');
				}

				if(oldIter->second.first)
				{
					key = oldIter->second.second;
					UWARN("Parameter migration from \"%s\" to \"%s\" (value=%s).",
							oldIter->first.c_str(), oldIter->second.second.c_str(), value.c_str());
				}
				else if(oldIter->second.second.empty())
				{
					UERROR("Parameter \"%s\" doesn't exist anymore.", oldIter->first.c_str());
				}
				else
				{
					UERROR("Parameter \"%s\" doesn't exist anymore, check this similar parameter \"%s\".", oldIter->first.c_str(), oldIter->second.second.c_str());
				}
				if(oldIter->second.first)
				{
					std::pair<ParametersMap::iterator, bool> inserted = parameters.insert(ParametersPair(key, value));
					if(inserted.second == false)
					{
						inserted.first->second = value;
					}
				}
			}
			else
			{
				showUsage();
			}
			continue;
		}

		printf("Unrecognized option : %s\n", argv[i]);
		showUsage();
	}

	UINFO("Loading files...");

	cv::Mat left = cv::imread(argv[1]);
	cv::Mat right = cv::imread(argv[2]);
	cv::Mat disp = readPFM(argv[4]);
	cv::Mat mask = cv::imread(argv[5]);

	if(!left.empty() && !right.empty() && !disp.empty() && !mask.empty())
	{
		UASSERT(left.rows == disp.rows);
		UASSERT(left.cols == disp.cols);
		UASSERT(disp.rows == mask.rows);
		UASSERT(disp.cols == mask.cols);

		// read calib.txt
		// Example format:
		// --- calib.txt:
		// cam0=[1038.018 0 322.037; 0 1038.018 243.393; 0 0 1]
		// cam1=[1038.018 0 375.308; 0 1038.018 243.393; 0 0 1]
		// doffs=53.271
		// baseline=176.252
		// width=718
		// height=496
		// ndisp=73
		// isint=0
		// vmin=8
		// vmax=65
		// dyavg=0.184
		// dymax=0.423
		// ---
		std::string calibFile = argv[3];
		std::ifstream stream(calibFile.c_str());
		std::string line;

		// two first lines are camera intrinsics
		UINFO("Loading calibration... (%s)", calibFile.c_str());
		std::vector<cv::Mat> K(2);
		for(int i=0; i<2; ++i)
		{
			getline(stream, line);
			line.erase(0, 6);
			line = uReplaceChar(line, ']', "");
			line = uReplaceChar(line, ';', "");
			UINFO("K[%d] = %s", i, line.c_str());
			std::vector<std::string> valuesStr = uListToVector(uSplit(line, ' '));
			UASSERT(valuesStr.size() == 9);
			K[i] = cv::Mat(3,3,CV_64FC1);
			for(unsigned int j=0; j<valuesStr.size(); ++j)
			{
				K[i].at<double>(j) = uStr2Double(valuesStr[j]);
			}
		}

		// skip doffs line
		getline(stream, line);

		// baseline
		getline(stream, line);
		line.erase(0, 9);
		double baseline = uStr2Double(line);
		UINFO("Baseline = %f", baseline);

		StereoCameraModel model(
				calibFile,
				CameraModel(K[0].at<double>(0,0), K[0].at<double>(1,1), K[0].at<double>(0,2), K[0].at<double>(1,2)),
				CameraModel(K[1].at<double>(0,0), K[1].at<double>(1,1), K[1].at<double>(0,2), K[1].at<double>(1,2), Transform::getIdentity(), -baseline/K[1].at<double>(0,0)));

		UASSERT(model.isValidForProjection());

		UINFO("Processing...");

		// Processing...
		cv::Mat leftMono;
		if(left.channels() == 3)
		{
			cv::cvtColor(left, leftMono, CV_BGR2GRAY);
		}
		else
		{
			leftMono = left;
		}
		cv::Mat rightMono;
		if(right.channels() == 3)
		{
			cv::cvtColor(right, rightMono, CV_BGR2GRAY);
		}
		else
		{
			rightMono = right;
		}

		UTimer timer;
		double timeKpts;
		double timeSubPixel;
		double timeStereo;

		// generate kpts
		std::vector<cv::KeyPoint> kpts;
		uInsert(parameters, ParametersPair(Parameters::kKpRoiRatios(), "0.03 0.03 0.04 0.04"));
		Feature2D * kptDetector = Feature2D::create(parameters);
		kpts = kptDetector->generateKeypoints(leftMono);
		delete kptDetector;

		timeKpts = timer.ticks();

		std::vector<cv::Point2f> leftCorners(kpts.size());
		cv::KeyPoint::convert(kpts, leftCorners);
		int subPixWinSize = 0;
		int subPixIterations = 0;
		double subPixEps = 0;
		Parameters::parse(parameters, Parameters::kKpSubPixWinSize(), subPixWinSize);
		Parameters::parse(parameters, Parameters::kKpSubPixIterations(), subPixIterations);
		Parameters::parse(parameters, Parameters::kKpSubPixEps(), subPixEps);
		if(subPixWinSize > 0 && subPixIterations > 0)
		{
			UDEBUG("cv::cornerSubPix() begin");
			cv::cornerSubPix(leftMono, leftCorners,
				cv::Size( subPixWinSize, subPixWinSize ),
				cv::Size( -1, -1 ),
				cv::TermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, subPixIterations, subPixEps ) );
			UDEBUG("cv::cornerSubPix() end");
		}

		timeSubPixel = timer.ticks();

		// Find features in the new right image
		std::vector<unsigned char> status;
		std::vector<cv::Point2f> rightCorners;

		bool opticalFlow = false;
		Parameters::parse(parameters, Parameters::kStereoOpticalFlow(), opticalFlow);
		Stereo * stereo = 0;
		if(opticalFlow)
		{
			stereo = new StereoOpticalFlow(parameters);
		}
		else
		{
			stereo = new Stereo(parameters);
		}

		rightCorners = stereo->computeCorrespondences(
				leftMono,
				rightMono,
				leftCorners,
				status);
		delete stereo;

		timeStereo = timer.ticks();

		UINFO("Time: kpts:%f s, subpix=%f s, stereo=%f s", timeKpts, timeSubPixel, timeStereo);

		UDEBUG("Mask = %d", mask.type());

		int inliers = 0;
		int subInliers = 0;
		int badInliers = 0;
		float sumInliers = 0.0f;
		float sumSubInliers = 0.0f;
		int goodRejected = 0;
		int badRejected = 0;
		for(unsigned int i=0; i<leftCorners.size(); ++i)
		{
			float gt = disp.at<float>(int(rightCorners[i].y), int(leftCorners[i].x));
			if(status[i]!=0)
			{
				float d = leftCorners[i].x - rightCorners[i].x;
				//float err = fabs(d-gt);
				//UDEBUG("Pt(%f,%f): d=%f, gt=%f, error=%f", leftCorners[i].x, leftCorners[i].y, d, gt, err);

				if(uIsFinite(gt))
				{
					if(fabs(d-gt) < 1.0f)
					{
						cv::line(left,
							leftCorners[i],
							cv::Point2f(leftCorners[i].x - gt, leftCorners[i].y),
							cv::Scalar( 0, 255, 0 ));

						cv::line(left,
							cv::Point2f(leftCorners[i].x - (d<gt?d:gt), leftCorners[i].y),
							cv::Point2f(leftCorners[i].x - (d>gt?d:gt), leftCorners[i].y),
							cv::Scalar( 0, 0, 255 ));

						++inliers;
						sumInliers += fabs(d-gt);

						if(fabs(d-gt) < 0.5f)
						{
							++subInliers;
							sumSubInliers += fabs(d-gt);
						}
					}
					else if(mask.at<cv::Vec3b>(int(rightCorners[i].y), int(leftCorners[i].x))[0] == 255)
					{
						cv::line(left,
							leftCorners[i],
							cv::Point2f(leftCorners[i].x - gt, leftCorners[i].y),
							cv::Scalar( 0, 255, 0 ));

						cv::line(left,
							cv::Point2f(leftCorners[i].x - (d<gt?d:gt), leftCorners[i].y),
							cv::Point2f(leftCorners[i].x - (d>gt?d:gt), leftCorners[i].y),
							cv::Scalar( 0, 0, 255 ));
						++badInliers;
						//UDEBUG("should be rejected or refined: %d pt=(%f,%f) (d=%f gt=%f)", i, leftCorners[i].x, leftCorners[i].y, d, gt);
					}
					else
					{
						cv::line(left,
							leftCorners[i],
							cv::Point2f(leftCorners[i].x - gt, leftCorners[i].y),
							cv::Scalar( 255, 0, 0 ));

						cv::line(left,
							cv::Point2f(leftCorners[i].x - (d<gt?d:gt), leftCorners[i].y),
							cv::Point2f(leftCorners[i].x - (d>gt?d:gt), leftCorners[i].y),
							cv::Scalar( 0, 0, 255 ));
						++badInliers;
						//UDEBUG("should be rejected: %d", i);
					}
				}
				else
				{
					++badInliers;
				}
			}
			else if(mask.at<cv::Vec3b>(int(rightCorners[i].y), int(leftCorners[i].x))[0] == 255 &&
					rightCorners[i].x > 0.0f)
			{
				float d = leftCorners[i].x - rightCorners[i].x;
				if(fabs(d-gt) < 1.0f)
				{
					cv::line(left,
						leftCorners[i],
						cv::Point2f(leftCorners[i].x - gt, leftCorners[i].y),
						cv::Scalar( 0, 255, 255 ));

					cv::line(left,
							cv::Point2f(leftCorners[i].x - (d<gt?d:gt), leftCorners[i].y),
							cv::Point2f(leftCorners[i].x - (d>gt?d:gt), leftCorners[i].y),
						cv::Scalar( 0, 0, 255 ));
					++goodRejected;
					//UDEBUG("should not be rejected: %d", i);
				}
				else
				{
					++badRejected;
				}
			}
			else
			{
				++badRejected;
				//UDEBUG("correctly rejected: %d", i);
			}
		}

		UINFO("good accepted=%d (%d%%) bad accepted=%d (%d%%) good rejected=%d (%d%%) bad rejected=%d (%d%%)",
				inliers,
				(inliers*100)/leftCorners.size(),
				badInliers,
				(badInliers*100)/leftCorners.size(),
				goodRejected,
				(goodRejected*100)/leftCorners.size(),
				badRejected,
				(badRejected*100)/leftCorners.size());
		UINFO("avg inliers =%f (subInliers=%f)", sumInliers/float(inliers), sumSubInliers/float(subInliers));


		cv::namedWindow( "Right", cv::WINDOW_AUTOSIZE );
		cv::imshow( "Right", right );

		cv::namedWindow( "Mask", cv::WINDOW_AUTOSIZE );
		cv::imshow( "Mask", mask );

		cv::namedWindow( "Left", cv::WINDOW_AUTOSIZE );
		cv::imshow( "Left", left );

		cv::waitKey(0);

	}

	return 0;
}
