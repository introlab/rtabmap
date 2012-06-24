#include <opencv2/core/core.hpp>
#include <opencv2/core/types_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <utilite/ULogger.h>
#include <utilite/UTimer.h>
#include <utilite/UConversion.h>
#include <utilite/UDirectory.h>
#include "rtabmap/core/SMMemory.h"

int main(int argc, char** argv)
{
	ULogger::setType(ULogger::kTypeConsole);
	ULogger::setLevel(ULogger::kInfo);

	std::string path = rtabmap::Parameters::defaultRtabmapWorkingDirectory() + "/LTM.db";
	if(argc > 1)
	{
		path = argv[1];
	}


    // Open database
    std::string driverType = "sqlite3";
	rtabmap::ParametersMap parameters;
	parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kDbSqlite3InMemory(), "false"));
	rtabmap::SMMemory * memory = new rtabmap::SMMemory(parameters);
	if(!memory)
	{
		UWARN("Can't create database driver \"%s\"", driverType.c_str());
	}
	else if(!memory->init(driverType, path))
	{
		UWARN("Can't open database \"%s\"", path.c_str());
	}

	if(memory)
	{
		UINFO("Using database %s", path.c_str());
		std::string saveDirectory = "imagesExtracted/";
		if(!UDirectory::exists(saveDirectory))
		{
			UDirectory::makeDir(saveDirectory);
		}
		std::set<int> ids = memory->getAllSignatureIds();
		for(std::set<int>::iterator iter=ids.begin(); iter!=ids.end(); ++iter)
		{
			std::list<rtabmap::Sensor> sensors = memory->getRawData(*iter);
			for(std::list<rtabmap::Sensor>::const_iterator jter=sensors.begin(); jter!=sensors.end(); ++jter)
			{
				int k=0;
				if(jter->type() == rtabmap::Sensor::kTypeImage)
				{
					std::string fileName = uFormat("%d-%d.png", *iter, k++);
					cv::imwrite(saveDirectory+fileName, jter->data());
					UINFO("Saved %s", (saveDirectory+fileName).c_str());
				}
			}
		}
	}

    return 0;
}
