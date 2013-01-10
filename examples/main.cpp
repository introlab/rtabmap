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

#include "rtabmap/core/Rtabmap.h"
#include "rtabmap/core/Camera.h"
#include <opencv2/core/core.hpp>

void showUsage()
{
	printf("\nUsage:\n"
			"rtabmap-example \"path\"\n"
			"  path   Path to a directory of images\n ");
	exit(1);
}

int main(int argc, char * argv[])
{
	if(argc < 2)
	{
		showUsage();
	}
	std::string path = argv[1];

	// rtabmap::Camera is simply a convenience wrapper of OpenCV cv::VideoCapture and cv::imread
	rtabmap::CameraImages camera(path);
	if(!camera.init())
	{
		printf("Camera init failed, using path \"%s\"\n", path.c_str());
		exit(1);
	}

	// Create RTAB-Map
	rtabmap::Rtabmap rtabmap;

	// Set the time threshold
	rtabmap.setTimeThreshold(700.0f); // Time threshold : 700 ms, 0 ms means no limit

	// To set other parameters, the Parameters interface must be used (Parameters.h).
	// Example here to change the loop closure threshold (default 0.15).
	// Lower the threshold, more loop closures are detected but there is more chance of false positives.
	rtabmap::ParametersMap parameters;
	parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kRtabmapLoopThr(), "0.11"));

	// The time threshold set above is also a parameter, one could have set it the same way:
	//   parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kRtabmapTimeThr(), "700"));
	// Or SURF hessian treshold:
	//   parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kSURFHessianThreshold(), "150"));

	// Initialize rtabmap: load/create database...
	rtabmap.init(parameters);

	// Process each image of the directory...
	printf("\nProcessing images... from directory \"%s\"\n", path.c_str());

	int countLoopDetected=0;
	int i=0;
	cv::Mat img = camera.takeImage();
	while(!img.empty())
	{
		// Process image : Main loop of RTAB-Map
		rtabmap.process(img);

		// Check if a loop closure is detected and print some info
		if(rtabmap.getLoopClosureId())
		{
			++countLoopDetected;
		}
		++i;
		if(rtabmap.getLoopClosureId())
		{
			printf(" #%d ptime(%fs) STM(%d) WM(%d) hyp(%d) value(%.2f) *LOOP %d->%d*\n",
					i,
					rtabmap.getLastProcessTime(),
					rtabmap.getSTM().size(), // short-term memory
					rtabmap.getWM().size(), // working memory
					rtabmap.getLoopClosureId(),
					rtabmap.getLcHypValue(),
					rtabmap.getLastLocationId(),
					rtabmap.getLoopClosureId());
		}
		else
		{
			printf(" #%d ptime(%fs) STM(%d) WM(%d) hyp(%d) value(%.2f)\n",
					i,
					rtabmap.getLastProcessTime(),
					rtabmap.getSTM().size(), // short-term memory
					rtabmap.getWM().size(), // working memory
					rtabmap.getRetrievedId(), // highest loop closure hypothesis
					rtabmap.getLcHypValue());
		}

		//Get next image
		img = camera.takeImage();
	}

	printf("Processing images completed. Loop closures found = %d\n", countLoopDetected);

	// Generate a graph for visualization with Graphiz
	rtabmap.generateGraph("Graph.dot");
	printf("Generated graph \"Graph.dot\", viewable with Graphiz using \"neato -Tpdf Graph.dot -o out.pdf\"\n");

	// Cleanup... save database and logs
	printf("Saving Long-Term Memory to \"LTM.db\"...\n");
	rtabmap.close();

	return 0;
}
