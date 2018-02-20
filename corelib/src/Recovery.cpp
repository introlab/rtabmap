/*
Copyright (c) 2010-2017, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
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

#include <rtabmap/core/Recovery.h>
#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/core/DBDriver.h>
#include <rtabmap/core/DBReader.h>
#include <rtabmap/core/Graph.h>
#include <rtabmap/core/ProgressState.h>
#include <rtabmap/utilite/UFile.h>
#include <rtabmap/utilite/UStl.h>

namespace rtabmap {

bool databaseRecovery(
		const std::string & corruptedDatabase,
		bool keepCorruptedDatabase,
		std::string * errorMsg,
		ProgressState * progressState)
{
	UDEBUG("Recovering \"%s\"", corruptedDatabase.c_str());

	std::string databasePath = uReplaceChar(corruptedDatabase, '~', UDirectory::homeDir());
	if(!UFile::exists(databasePath))
	{
		if(errorMsg)
			*errorMsg = uFormat("File \"%s\" doesn't exist!", databasePath.c_str());
		return false;
	}

	std::string backupPath;
	if(UFile::getExtension(databasePath).compare("db") != 0)
	{
		if(errorMsg)
			*errorMsg = uFormat("File \"%s\" is not a database (*.db)!", databasePath.c_str());
		return false;
	}
	std::list<std::string> strList = uSplit(databasePath, '.');
	strList.pop_back();
	backupPath = uJoin(strList, ".") + ".backup.db";
	if(UFile::exists(backupPath))
	{
		if(errorMsg)
			*errorMsg = uFormat("Backup file \"%s\" already exists!", backupPath.c_str());
		return false;
	}

	DBDriver * dbDriver = DBDriver::create();
	if(!dbDriver->openConnection(databasePath, false))
	{
		if(errorMsg)
			*errorMsg = uFormat("Failed opening database!");
		delete dbDriver;
		return false;
	}

	ParametersMap parameters = dbDriver->getLastParameters();
	if(parameters.empty())
	{
		if(errorMsg)
			*errorMsg = uFormat("Failed getting parameters from database, recovery cannot be done.");
		dbDriver->closeConnection(false);
		delete dbDriver;
		return false;
	}
	std::set<int> ids;
	dbDriver->getAllNodeIds(ids);
	if(ids.empty())
	{
		if(errorMsg)
			*errorMsg = uFormat("Input database doesn't have any nodes saved in it.");
		dbDriver->closeConnection(false);
		delete dbDriver;
		return false;
	}
	if(progressState)
		progressState->callback(uFormat("Found %d nodes to recover.", (int)ids.size()));

	//Detect if the database is corrupted
	std::multimap<int, Link> links;
	dbDriver->getAllLinks(links, true);
	bool corrupted = false;
	for(std::multimap<int, Link>::iterator iter=links.begin(); iter!=links.end(); ++iter)
	{
		if(iter->second.type() == Link::kNeighbor &&
			graph::findLink(links, iter->second.to(), iter->second.from(), false) == links.end())
		{
			corrupted = true;
			break;
		}
	}

	if(progressState)
	{
		if(corrupted)
			progressState->callback("Database is indeed corrupted, found one or more neighbor links missing.");
		else
			progressState->callback("Database doesn't seem to be corrupted, still recovering it.");
	}

	dbDriver->closeConnection(false);
	delete dbDriver;

	if(progressState)
		progressState->callback(uFormat("Renaming \"%s\" to \"%s\"...", UFile::getName(databasePath).c_str(), UFile::getName(backupPath).c_str()));
	if(UFile::rename(databasePath, backupPath) != 0)
	{
		if(errorMsg)
			*errorMsg = uFormat("Failed renaming database file from \"%s\" to \"%s\". Is it opened by another app?", UFile::getName(databasePath).c_str(), UFile::getName(backupPath).c_str());
		return false;
	}
	Rtabmap rtabmap;
	rtabmap.init(parameters, databasePath);

	bool rgbdEnabled = Parameters::defaultRGBDEnabled();
	Parameters::parse(parameters, Parameters::kRGBDEnabled(), rgbdEnabled);
	bool odometryIgnored = !rgbdEnabled;
	DBReader dbReader(backupPath, 0, odometryIgnored);
	dbReader.init();

	CameraInfo info;
	SensorData data = dbReader.takeImage(&info);
	int processed = 0;
	if(progressState)
		progressState->callback(uFormat("Recovering data of \"%s\"...", backupPath.c_str()));
	while(data.isValid() && (progressState==0 || !progressState->isCanceled()))
	{
		std::string status;
		if(!odometryIgnored && info.odomPose.isNull())
		{
			status = uFormat("Skipping node %d as it doesn't have odometry pose set.", data.id());
		}
		else
		{
			if(!odometryIgnored && !info.odomCovariance.empty() && info.odomCovariance.at<double>(0,0)>=9999)
			{
				status = uFormat("High variance detected, triggering a new map...");
				rtabmap.triggerNewMap();
			}
			if(!rtabmap.process(data, info.odomPose, info.odomCovariance, info.odomVelocity))
			{
				status = uFormat("Failed processing node %d.", data.id());
			}
		}
		if(status.empty())
		{
			if(progressState)
				progressState->callback(status);
		}

		data = dbReader.takeImage(&info);

		if(progressState)
			progressState->callback(uFormat("Processed %d/%d nodes...", ++processed, (int)ids.size()));
	}

	if(progressState)
	{
		if(progressState->isCanceled())
		{
			rtabmap.close(false);
			if(errorMsg)
				*errorMsg = uFormat("Recovery canceled, renaming back \"%s\" to \"%s\".", backupPath.c_str(), databasePath.c_str());

			// put back the file as before
			UFile::erase(databasePath);
			UFile::rename(backupPath, databasePath);
			return false;
		}
	}

	if(progressState)
		progressState->callback(uFormat("Closing database \"%s\"...", databasePath.c_str()));
	rtabmap.close(true);
	if(progressState)
		progressState->callback(uFormat("Closing database \"%s\"... done!", databasePath.c_str()));

	if(!keepCorruptedDatabase)
	{
		UFile::erase(backupPath);
	}

	return true;
}

}




