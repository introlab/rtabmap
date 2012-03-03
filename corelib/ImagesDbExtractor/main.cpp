#include <opencv2/core/core.hpp>
#include <opencv2/core/types_c.h>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/imgproc/imgproc_c.h>
#include <iostream>
#include <utilite/ULogger.h>
#include <utilite/UTimer.h>
#include <utilite/UConversion.h>
#include <utilite/UDirectory.h>
#include "SMMemory.h"

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
			IplImage * image = memory->getImage(*iter);
			if(image)
			{
				std::string fileName = uNumber2Str(*iter) + ".jpg";
				cvSaveImage((saveDirectory+fileName).c_str(), image);
				cvReleaseImage(&image);
				UINFO("Saved %s", (saveDirectory+fileName).c_str());
			}
		}
	}

    return 0;
}
