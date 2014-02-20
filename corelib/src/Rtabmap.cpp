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
#include "rtabmap/core/Version.h"
#include "rtabmap/core/Features2d.h"

#include "rtabmap/core/Signature.h"

#include "rtabmap/core/EpipolarGeometry.h"

#include "rtabmap/core/Memory.h"
#include "BayesFilter.h"

#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UFile.h>
#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UMath.h>

#include "SimpleIni.h"

#include <pcl/search/kdtree.h>
#include <pcl/filters/crop_box.h>
#include <pcl/io/pcd_io.h>

#include "rtabmap/core/util3d.h"

#include <stdlib.h>
#include <set>

#define LOG_F "LogF.txt"
#define LOG_I "LogI.txt"

#define GRAPH_FILE_NAME "Graph.dot"

//
//
//
// =======================================================
// MAIN LOOP, see method "void Rtabmap::process();" below.
// =======================================================
//
//
//

namespace rtabmap
{

Rtabmap::Rtabmap() :
	_publishStats(Parameters::defaultRtabmapPublishStats()),
	_publishImage(Parameters::defaultRtabmapPublishImage()),
	_publishPdf(Parameters::defaultRtabmapPublishPdf()),
	_publishLikelihood(Parameters::defaultRtabmapPublishLikelihood()),
	_publishKeypoints(Parameters::defaultKpPublishKeypoints()),
	_maxTimeAllowed(Parameters::defaultRtabmapTimeThr()), // 700 ms
	_maxMemoryAllowed(Parameters::defaultRtabmapMemoryThr()), // 0=inf
	_loopThr(Parameters::defaultRtabmapLoopThr()),
	_loopRatio(Parameters::defaultRtabmapLoopRatio()),
	_maxRetrieved(Parameters::defaultRtabmapMaxRetrieved()),
	_statisticLogsBufferedInRAM(Parameters::defaultRtabmapStatisticLogsBufferedInRAM()),
	_statisticLogged(Parameters::defaultRtabmapStatisticLogged()),
	_rgbdSlamMode(Parameters::defaultRGBDEnabled()),
	_rgbdLinearUpdate(Parameters::defaultRGBDLinearUpdate()),
	_rgbdAngularUpdate(Parameters::defaultRGBDAngularUpdate()),
	_newMapOdomChangeDistance(Parameters::defaultRGBDNewMapOdomChangeDistance()),
	_globalLoopClosureIcpType(Parameters::defaultLccIcpType()),
	_globalLoopClosureIcpMaxDistance(Parameters::defaultLccIcpMaxDistance()),
	_scanMatchingSize(Parameters::defaultRGBDScanMatchingSize()),
	_localLoopClosureDetectionTime(Parameters::defaultRGBDLocalLoopDetectionTime()),
	_localLoopClosureDetectionSpace(Parameters::defaultRGBDLocalLoopDetectionSpace()),
	_localDetectRadius(Parameters::defaultRGBDLocalLoopDetectionRadius()),
	_localDetectMaxNeighbors(Parameters::defaultRGBDLocalLoopDetectionNeighbors()),
	_localDetectMaxDiffID(Parameters::defaultRGBDLocalLoopDetectionMaxDiffID()),
	_toroIterations(Parameters::defaultRGBDToroIterations()),
	_databasePath(""),
	_lcHypothesisId(0),
	_lcHypothesisValue(0),
	_retrievedId(0),
	_lastProcessTime(0.0),
	_epipolarGeometry(0),
	_bayesFilter(0),
	_memory(0),
	_foutFloat(0),
	_foutInt(0),
	_wDir(std::string(".")+UDirectory::separator()),
	_mapCorrection(Transform::getIdentity()),
	_mapTransform(Transform::getIdentity())
{
}

Rtabmap::~Rtabmap() {
	UDEBUG("");
	this->close();
}

std::string Rtabmap::getVersion()
{
	return RTABMAP_VERSION;
	return ""; // Second return only to avoid compiler warning with RTABMAP_VERSION not yet set.
}

void Rtabmap::setupLogFiles(bool overwrite)
{
	flushStatisticLogs();
	// Log files
	if(_foutFloat)
	{
		fclose(_foutFloat);
		_foutFloat = 0;
	}
	if(_foutInt)
	{
		fclose(_foutInt);
		_foutInt = 0;
	}

	if(_statisticLogged)
	{
		std::string attributes = "a+"; // append to log files
		if(overwrite)
		{
			// If a file with the same name already exists
			// its content is erased and the file is treated
			// as a new empty file.
			attributes = "w";
		}

		bool addLogFHeader = overwrite || !UFile::exists(_wDir+LOG_F);
		bool addLogIHeader = overwrite || !UFile::exists(_wDir+LOG_I);

	#ifdef _MSC_VER
		fopen_s(&_foutFloat, (_wDir+LOG_F).c_str(), attributes.c_str());
		fopen_s(&_foutInt, (_wDir+LOG_I).c_str(), attributes.c_str());
	#else
		_foutFloat = fopen((_wDir+LOG_F).c_str(), attributes.c_str());
		_foutInt = fopen((_wDir+LOG_I).c_str(), attributes.c_str());
	#endif
		// add header (column identification)
		if(addLogFHeader && _foutFloat)
		{
			fprintf(_foutFloat, "Column headers:\n");
			fprintf(_foutFloat, " 1-Total iteration time (s)\n");
			fprintf(_foutFloat, " 2-Memory update time (s)\n");
			fprintf(_foutFloat, " 3-Retrieval time (s)\n");
			fprintf(_foutFloat, " 4-Likelihood time (s)\n");
			fprintf(_foutFloat, " 5-Posterior time (s)\n");
			fprintf(_foutFloat, " 6-Hypothesis selection time (s)\n");
			fprintf(_foutFloat, " 7-Transfer time (s)\n");
			fprintf(_foutFloat, " 8-Statistics creation time (s)\n");
			fprintf(_foutFloat, " 9-Loop closure hypothesis value\n");
			fprintf(_foutFloat, " 10-NAN\n");
			fprintf(_foutFloat, " 11-Maximum likelihood\n");
			fprintf(_foutFloat, " 12-Sum likelihood\n");
			fprintf(_foutFloat, " 13-Mean likelihood\n");
			fprintf(_foutFloat, " 14-Std dev likelihood\n");
			fprintf(_foutFloat, " 15-Virtual place hypothesis\n");
			fprintf(_foutFloat, " 16-Join trash time (s)\n");
			fprintf(_foutFloat, " 17-Weight Update (rehearsal) similarity\n");
			fprintf(_foutFloat, " 18-Empty trash time (s)\n");
			fprintf(_foutFloat, " 19-Retrieval database access time (s)\n");
			fprintf(_foutFloat, " 20-Add loop closure link time (s)\n");
		}
		if(addLogIHeader && _foutInt)
		{
			fprintf(_foutInt, "Column headers:\n");
			fprintf(_foutInt, " 1-Loop closure ID\n");
			fprintf(_foutInt, " 2-Highest loop closure hypothesis\n");
			fprintf(_foutInt, " 3-Locations transferred\n");
			fprintf(_foutInt, " 4-NAN\n");
			fprintf(_foutInt, " 5-Words extracted from the last image\n");
			fprintf(_foutInt, " 6-Vocabulary size\n");
			fprintf(_foutInt, " 7-Working memory size\n");
			fprintf(_foutInt, " 8-Is loop closure hypothesis rejected?\n");
			fprintf(_foutInt, " 9-NAN\n");
			fprintf(_foutInt, " 10-NAN\n");
			fprintf(_foutInt, " 11-Locations retrieved\n");
			fprintf(_foutInt, " 12-Retrieval location ID\n");
			fprintf(_foutInt, " 13-Unique words extraced from last image\n");
			fprintf(_foutInt, " 14-Retrieval ID\n");
			fprintf(_foutInt, " 15-Non-null likelihood values\n");
			fprintf(_foutInt, " 16-Weight Update ID\n");
			fprintf(_foutInt, " 17-Is last location merged through Weight Update?\n");
		}

		ULOGGER_DEBUG("Log file (int)=%s", (_wDir+LOG_I).c_str());
		ULOGGER_DEBUG("Log file (float)=%s", (_wDir+LOG_F).c_str());
	}
	else
	{
		UDEBUG("Log disabled!");
	}
}

void Rtabmap::flushStatisticLogs()
{
	if(_foutFloat && _bufferedLogsF.size())
	{
		UDEBUG("_bufferedLogsF.size=%d", _bufferedLogsF.size());
		for(std::list<std::string>::iterator iter = _bufferedLogsF.begin(); iter!=_bufferedLogsF.end(); ++iter)
		{
			fprintf(_foutFloat, "%s", iter->c_str());
		}
		_bufferedLogsF.clear();
	}
	if(_foutInt && _bufferedLogsI.size())
	{
		UDEBUG("_bufferedLogsI.size=%d", _bufferedLogsI.size());
		for(std::list<std::string>::iterator iter = _bufferedLogsI.begin(); iter!=_bufferedLogsI.end(); ++iter)
		{
			fprintf(_foutInt, "%s", iter->c_str());
		}
		_bufferedLogsI.clear();
	}
}

void Rtabmap::init(const ParametersMap & parameters, bool deleteMemory)
{
	if(deleteMemory)
	{
		// Set the working directory and database path before
		ParametersMap::const_iterator iter;
		if((iter=parameters.find(Parameters::kRtabmapWorkingDirectory())) != parameters.end())
		{
			this->setWorkingDirectory(iter->second.c_str());
		}
		if((iter=parameters.find(Parameters::kRtabmapDatabasePath())) != parameters.end())
		{
			this->setDatabasePath(iter->second.c_str());
		}
		if((iter=parameters.find(Parameters::kRtabmapStatisticLogged())) != parameters.end())
		{
			_statisticLogged = uStr2Bool(iter->second.c_str());
		}
		this->resetMemory(true);
	}

	this->parseParameters(parameters);

	setupLogFiles();
}

void Rtabmap::init(const std::string & configFile, bool deleteMemory)
{
	// fill ctrl struct with values from the configuration file
	ParametersMap param;// = Parameters::defaultParameters;

	if(!configFile.empty())
	{
		ULOGGER_DEBUG("Read parameters from = %s", configFile.c_str());
		this->readParameters(configFile, param);
	}

	this->init(param, deleteMemory);
}

void Rtabmap::close()
{
	flushStatisticLogs();
	if(_foutFloat)
	{
		fclose(_foutFloat);
		_foutFloat = 0;
	}
	if(_foutInt)
	{
		fclose(_foutInt);
		_foutInt = 0;
	}

	if(_epipolarGeometry)
	{
		delete _epipolarGeometry;
		_epipolarGeometry = 0;
	}
	if(_memory)
	{
		delete _memory;
		_memory = 0;
	}
	if(_bayesFilter)
	{
		delete _bayesFilter;
		_bayesFilter = 0;
	}
}

void Rtabmap::parseParameters(const ParametersMap & parameters)
{
	ULOGGER_DEBUG("");
	ParametersMap::const_iterator iter;
	if((iter=parameters.find(Parameters::kRtabmapWorkingDirectory())) != parameters.end())
	{
		this->setWorkingDirectory(iter->second.c_str());
	}
	if((iter=parameters.find(Parameters::kRtabmapDatabasePath())) != parameters.end())
	{
		this->setDatabasePath(iter->second.c_str());
	}

	Parameters::parse(parameters, Parameters::kRtabmapPublishStats(), _publishStats);
	Parameters::parse(parameters, Parameters::kRtabmapPublishImage(), _publishImage);
	Parameters::parse(parameters, Parameters::kRtabmapPublishPdf(), _publishPdf);
	Parameters::parse(parameters, Parameters::kRtabmapPublishLikelihood(), _publishLikelihood);
	Parameters::parse(parameters, Parameters::kKpPublishKeypoints(), _publishKeypoints);
	Parameters::parse(parameters, Parameters::kRtabmapTimeThr(), _maxTimeAllowed);
	Parameters::parse(parameters, Parameters::kRtabmapMemoryThr(), _maxMemoryAllowed);
	Parameters::parse(parameters, Parameters::kRtabmapLoopThr(), _loopThr);
	Parameters::parse(parameters, Parameters::kRtabmapLoopRatio(), _loopRatio);
	Parameters::parse(parameters, Parameters::kRtabmapMaxRetrieved(), _maxRetrieved);
	Parameters::parse(parameters, Parameters::kRtabmapStatisticLogsBufferedInRAM(), _statisticLogsBufferedInRAM);
	Parameters::parse(parameters, Parameters::kRtabmapStatisticLogged(), _statisticLogged);
	Parameters::parse(parameters, Parameters::kRGBDEnabled(), _rgbdSlamMode);
	Parameters::parse(parameters, Parameters::kRGBDLinearUpdate(), _rgbdLinearUpdate);
	Parameters::parse(parameters, Parameters::kRGBDAngularUpdate(), _rgbdAngularUpdate);
	Parameters::parse(parameters, Parameters::kRGBDNewMapOdomChangeDistance(), _newMapOdomChangeDistance);
	Parameters::parse(parameters, Parameters::kRGBDScanMatchingSize(), _scanMatchingSize);
	Parameters::parse(parameters, Parameters::kLccIcpMaxDistance(), _globalLoopClosureIcpMaxDistance);
	Parameters::parse(parameters, Parameters::kRGBDLocalLoopDetectionTime(), _localLoopClosureDetectionTime);
	Parameters::parse(parameters, Parameters::kRGBDLocalLoopDetectionSpace(), _localLoopClosureDetectionSpace);
	Parameters::parse(parameters, Parameters::kRGBDLocalLoopDetectionRadius(), _localDetectRadius);
	Parameters::parse(parameters, Parameters::kRGBDLocalLoopDetectionNeighbors(), _localDetectMaxNeighbors);
	Parameters::parse(parameters, Parameters::kRGBDLocalLoopDetectionMaxDiffID(), _localDetectMaxDiffID);
	Parameters::parse(parameters, Parameters::kRGBDToroIterations(), _toroIterations);

	// RGB-D SLAM stuff
	if((iter=parameters.find(Parameters::kLccIcpType())) != parameters.end())
	{
		int icpType = std::atoi((*iter).second.c_str());
		if(icpType >= 0 && icpType <= 2)
		{
			_globalLoopClosureIcpType = icpType;
		}
		else
		{
			UERROR("Icp type must be 0, 1 or 2 (value=%d)", icpType);
		}
	}

	// By default, we create our strategies if they are not already created.
	// If they already exists, we check the parameters if a change is requested

	if(!_memory)
	{
		_memory = new Memory(parameters);
		_memory->init(getDatabasePath(), false, parameters);

		//generate map
		if(_rgbdSlamMode && _memory->getLastWorkingSignature())
		{
			optimizeCurrentMap(_memory->getLastWorkingSignature()->id(), false, _optimizedPoses, &_constraints);
		}
	}
	else
	{
		_memory->parseParameters(parameters);
	}

	VhStrategy vhStrategy = kVhUndef;
	// Verifying hypotheses strategy
	if((iter=parameters.find(Parameters::kRtabmapVhStrategy())) != parameters.end())
	{
		vhStrategy = (VhStrategy)std::atoi((*iter).second.c_str());
	}
	if(!_epipolarGeometry && vhStrategy == kVhEpipolar)
	{
		_epipolarGeometry = new EpipolarGeometry(parameters);
	}
	else if(_epipolarGeometry && vhStrategy == kVhNone)
	{
		delete _epipolarGeometry;
		_epipolarGeometry = 0;
	}
	else if(_epipolarGeometry)
	{
		_epipolarGeometry->parseParameters(parameters);
	}

	// Bayes filter, create one if not exists
	if(!_bayesFilter)
	{
		_bayesFilter = new BayesFilter(parameters);
	}
	else
	{
		_bayesFilter->parseParameters(parameters);
	}
}

std::string Rtabmap::getDatabasePath() const
{
	return _databasePath.empty()?_wDir + Parameters::getDefaultDatabaseName():_databasePath;
}

int Rtabmap::getLoopClosureId() const
{
	return _lcHypothesisId;
}

int Rtabmap::getRetrievedId() const
{
	return _retrievedId;
}

int Rtabmap::getLastLocationId() const
{
	int id = 0;
	if(_memory)
	{
		id = _memory->getLastSignatureId();
	}
	return id;
}

std::list<int> Rtabmap::getWM() const
{
	std::list<int> mem;
	if(_memory)
	{
		mem = std::list<int>(_memory->getWorkingMem().begin(), _memory->getWorkingMem().end());
		mem.remove(-1);// Ignore the virtual signature (if here)
	}
	return mem;
}

int Rtabmap::getWMSize() const
{
	if(_memory)
	{
		return _memory->getWorkingMem().size()-1; // remove virtual place
	}
	return 0;
}

std::map<int, int> Rtabmap::getWeights() const
{
	std::map<int, int> weights;
	if(_memory)
	{
		weights = _memory->getWeights();
		weights.erase(-1);// Ignore the virtual signature (if here)
	}
	return weights;
}

std::set<int> Rtabmap::getSTM() const
{
	if(_memory)
	{
		return _memory->getStMem();
	}
	return std::set<int>();
}

int Rtabmap::getSTMSize() const
{
	if(_memory)
	{
		return _memory->getStMem().size();
	}
	return 0;
}

int Rtabmap::getTotalMemSize() const
{
	if(_memory)
	{
		const Signature * s  =_memory->getLastWorkingSignature();
		if(s)
		{
			return s->id();
		}
	}
	return 0;
}

std::multimap<int, cv::KeyPoint> Rtabmap::getWords(int locationId) const
{
	if(_memory)
	{
		const Signature * s = _memory->getSignature(locationId);
		if(s)
		{
			return s->getWords();
		}
	}
	return std::multimap<int, cv::KeyPoint>();
}

bool Rtabmap::isInSTM(int locationId) const
{
	if(_memory)
	{
		return _memory->isInSTM(locationId);
	}
	return false;
}

bool Rtabmap::isIDsGenerated() const
{
	if(_memory)
	{
		return _memory->isIDsGenerated();
	}
	return Parameters::defaultMemGenerateIds();
}

const Statistics & Rtabmap::getStatistics() const
{
	return statistics_;
}
/*
bool Rtabmap::getMetricData(int locationId, cv::Mat & rgb, cv::Mat & depth, float & depthConstant, Transform & pose, Transform & localTransform) const
{
	if(_memory)
	{
		const Signature * s = _memory->getSignature(locationId);
		if(s && _optimizedPoses.find(s->id()) != _optimizedPoses.end())
		{
			rgb = s->getImage();
			depth = s->getDepth();
			depthConstant = s->getDepthConstant();
			pose = _optimizedPoses.at(s->id());
			localTransform = s->getLocalTransform();
			return true;
		}
	}
	return false;
}
*/
Transform Rtabmap::getPose(int locationId) const
{
	if(_memory)
	{
		const Signature * s = _memory->getSignature(locationId);
		if(s && _optimizedPoses.find(s->id()) != _optimizedPoses.end())
		{
			return _optimizedPoses.at(s->id());
		}
	}
	return Transform();
}

void Rtabmap::triggerNewMap()
{
	if(_memory)
	{
		int mapId = _memory->incrementMapId();
		UINFO("New map triggerred, new map = %d", mapId);
		_optimizedPoses.clear();
		_constraints.clear();
	}
}

void Rtabmap::generateDOTGraph(const std::string & path, int id, int margin)
{
	if(_memory)
	{
		_memory->joinTrashThread(); // make sure the trash is flushed

		if(id > 0)
		{
			std::map<int, int> ids = _memory->getNeighborsId(id, margin, -1, false);

			if(ids.size() > 0)
			{
				ids.insert(std::pair<int,int>(id, 0));
				std::set<int> idsSet;
				for(std::map<int, int>::iterator iter = ids.begin(); iter!=ids.end(); ++iter)
				{
					idsSet.insert(idsSet.end(), iter->first);
				}
				_memory->generateGraph(path, idsSet);
			}
			else
			{
				UERROR("No neighbors found for signature %d.", id);
			}
		}
		else
		{
			_memory->generateGraph(path);
		}
	}
}

void Rtabmap::generateTOROGraph(const std::string & path, bool optimized, bool global)
{
	if(_memory && _memory->getLastWorkingSignature())
	{
		std::map<int, Transform> poses;
		std::multimap<int, Link> constraints;

		if(optimized)
		{
			this->optimizeCurrentMap(_memory->getLastWorkingSignature()->id(), global, poses, &constraints);
		}
		else
		{
			std::map<int, int> ids = _memory->getNeighborsId(_memory->getLastWorkingSignature()->id(), 0, global?-1:0, true);
			_memory->getMetricConstraints(uKeys(ids), poses, constraints, global);
		}

		util3d::saveTOROGraph(path, poses, constraints);
	}
}

void Rtabmap::resetMemory(bool dbOverwritten)
{
	_retrievedId = 0;
	_lcHypothesisValue = 0;
	_lcHypothesisId = 0;
	_lastProcessTime = 0.0;
	_optimizedPoses.clear();
	_constraints.clear();
	_mapCorrection.setIdentity();
	_mapTransform.setIdentity();

	if(_memory)
	{
		_memory->init(getDatabasePath(), dbOverwritten);
		if(_memory->getLastWorkingSignature())
		{
			optimizeCurrentMap(_memory->getLastWorkingSignature()->id(), false, _optimizedPoses, &_constraints);
		}
		if(_bayesFilter)
		{
			_bayesFilter->reset();
		}
	}
	else if(dbOverwritten)
	{
		// May be memory should be already created here, and use init above...
		UINFO("Erasing file : \"%s\"", getDatabasePath().c_str());
		UFile::erase(getDatabasePath());
	}
	this->setupLogFiles(dbOverwritten);
}

//============================================================
// MAIN LOOP
//============================================================
bool Rtabmap::process(const Image & image)
{
	UDEBUG("");

	//============================================================
	// Initialization
	//============================================================
	UTimer timer;
	UTimer timerTotal;
	double timeMemoryUpdate = 0;
	double timeScanMatching = 0;
	double timeLocalTimeDetection = 0;
	double timeLocalSpaceDetection = 0;
	double timeCleaningNeighbors = 0;
	double timeReactivations = 0;
	double timeAddLoopClosureLink = 0;
	double timeMapOptimization = 0;
	double timeRetrievalDbAccess = 0;
	double timeLikelihoodCalculation = 0;
	double timePosteriorCalculation = 0;
	double timeHypothesesCreation = 0;
	double timeHypothesesValidation = 0;
	double timeRealTimeLimitReachedProcess = 0;
	double timeMemoryCleanup = 0;
	double timeEmptyingTrash = 0;
	double timeJoiningTrash = 0;
	double timeStatsCreation = 0;

	float hypothesisRatio = 0.0f; // Only used for statistics
	bool rejectedHypothesis = false;

	std::map<int, float> rawLikelihood;
	std::map<int, float> adjustedLikelihood;
	std::map<int, float> likelihood;
	std::map<int, int> weights;
	std::map<int, float> posterior;
	std::pair<int, float> hypothesis(0, 0.0f);
	std::list<std::pair<int, float> > reactivateHypotheses;

	std::map<int, int> childCount;
	std::set<int> signaturesRetrieved;
	int localLoopClosuresInTimeFound = 0;
	bool scanMatchingSuccess = false;

	const Signature * signature = 0;
	const Signature * sLoop = 0;

	_lcHypothesisId = 0;

	std::set<int> immunizedLocations;

	statistics_ = Statistics(); // reset

	//============================================================
	// Wait for an image...
	//============================================================
	ULOGGER_INFO("getting data...");
	if(image.empty())
	{
		ULOGGER_INFO("image is null...");
		return false;
	}

	timer.start();
	timerTotal.start();

	if(!_memory || !_bayesFilter)
	{
		UFATAL("RTAB-Map is not initialized, data received is ignored.");
	}

	//============================================================
	// If RGBD SLAM is enabled, a pose must be set.
	//============================================================
	if(_rgbdSlamMode)
	{
		if(image.pose().isNull())
		{
			UERROR("RGB-D SLAM mode is enabled and no odometry is provided. "
				   "Image %d is ignored!", image.id());
			return false;
		}
		else
		{
			// Detect if the odometry is reset. If yes, trigger a new map.
			if(_memory->getLastWorkingSignature())
			{
				const Transform & lastPose = _memory->getLastWorkingSignature()->getPose(); // use raw odometry
				if(!lastPose.isIdentity() && image.pose().isIdentity())
				{
					int mapId = _memory->incrementMapId();
					UWARN("Odometry is reset (transform identity detected). A new map (%d) is created!", mapId);
					_optimizedPoses.clear();
					_constraints.clear();
				}
				else
				{
					Transform lastPoseToNewPose = lastPose.inverse() * image.pose();
					float x,y,z, roll,pitch,yaw;
					lastPoseToNewPose.getTranslationAndEulerAngles(x,y,z, roll,pitch,yaw);
					if(_newMapOdomChangeDistance > 0.0 && (x*x + y*y + z*z) > _newMapOdomChangeDistance*_newMapOdomChangeDistance)
					{
						int mapId = _memory->incrementMapId();
						UWARN("Odometry is reset (large odometry change detected > %f). A new map (%d) is created! Last pose = %s, new pose = %s",
								_newMapOdomChangeDistance,
								mapId,
								lastPose.prettyPrint().c_str(),
								image.pose().prettyPrint().c_str());
						_optimizedPoses.clear();
						_constraints.clear();
					}
				}
			}
		}
	}

	//============================================================
	// Memory Update : Location creation + Add to STM + Weight Update (Rehearsal)
	//============================================================
	ULOGGER_INFO("Updating memory...");
	if(!_memory->update(image, &statistics_))
	{
		return false;
	}

	signature = _memory->getLastWorkingSignature();
	if(!signature)
	{
		UFATAL("Not supposed to be here...last signature is null?!?");
	}
	ULOGGER_INFO("Processing signature %d", signature->id());
	timeMemoryUpdate = timer.ticks();
	ULOGGER_INFO("timeMemoryUpdate=%fs", timeMemoryUpdate);

	//============================================================
	// Metric
	//============================================================
	if(_rgbdSlamMode)
	{
		//Verify if there was a rehearsal
		int rehearsedId = (int)uValue(statistics_.data(), Statistics::kMemoryRehearsal_merged(), 0.0f);
		if(rehearsedId > 0)
		{
			_optimizedPoses.erase(rehearsedId);
		}
		else
		{
			//============================================================
			// Minimum displacement required to add to Memory
			//============================================================
			const std::map<int, Transform> & neighbors = signature->getNeighbors();
			if(neighbors.size() == 1)
			{
				float x,y,z, roll,pitch,yaw;
				neighbors.begin()->second.getTranslationAndEulerAngles(x,y,z, roll,pitch,yaw);
				if(fabs(x) < _rgbdLinearUpdate &&
					fabs(y) < _rgbdLinearUpdate &&
					fabs(z) < _rgbdLinearUpdate &&
					fabs(roll) < _rgbdAngularUpdate &&
					fabs(pitch) < _rgbdAngularUpdate &&
					fabs(yaw) < _rgbdAngularUpdate)
				{
					UWARN("Ignoring location %d because the displacement is too small!", signature->id());
					_memory->deleteLocation(signature->id());
					return true;
				}
			}
		}

		// Reset map correction if we are mapping!
		if(_memory->isIncremental() && !_mapCorrection.isIdentity())
		{
			UWARN("Reset map correction because we are now mapping!");
			_mapCorrection.setIdentity();
		}

		Transform newPose = _mapCorrection * signature->getPose();
		_optimizedPoses.insert(std::make_pair(signature->id(), newPose));

		//============================================================
		// Scan matching
		//============================================================
		if(_scanMatchingSize>0 &&
			signature->getNeighbors().size() == 1 &&
			signature->getDepth2D().size() &&
			rehearsedId == 0) // don't do it if rehearsal happened
		{
			UINFO("Odometry correction by scan matching (size=%d)...", _scanMatchingSize);
			const std::set<int> & stm = _memory->getStMem();
			std::map<int, Transform> poses;
			int count = _scanMatchingSize;
			for(std::set<int>::const_reverse_iterator iter = stm.rbegin(); iter!=stm.rend() && count>0; ++iter)
			{
				if(_memory->getSignature(*iter)->mapId() == signature->mapId())
				{
					std::map<int, Transform>::iterator jter = _optimizedPoses.find(*iter);
					UASSERT_MSG(jter != _optimizedPoses.end(), uFormat("%d not found in optimized poses!", *iter).c_str());
					poses.insert(*jter);
					if(*iter != signature->id())
					{
						--count;
					}
				}
			}

			int oldId = signature->getNeighbors().begin()->first;
			if(poses.size())
			{
				const Signature * oldS = _memory->getSignature(oldId);
				UASSERT(oldS != 0);
				Transform t = _memory->computeScanMatchingTransform(signature->id(), oldId, poses);
				if(!t.isNull())
				{
					scanMatchingSuccess = true;
					UDEBUG("Update neighbor link (%d->%d) from %s to %s",
							signature->id(),
							oldId,
							signature->getNeighbors().at(oldId).prettyPrint().c_str(),
							t.prettyPrint().c_str());
					_memory->updateNeighborLink(signature->id(), oldId, t);
				}
			}
			else
			{
				UERROR("Poses are empty?!?");
			}
		}
		timeScanMatching = timer.ticks();
		ULOGGER_INFO("timeScanMatching=%fs", timeScanMatching);

		if(signature->getNeighbors().size() == 1)
		{
			_constraints.insert(std::make_pair(signature->id(), Link(signature->id(), signature->getNeighbors().begin()->first, signature->getNeighbors().begin()->second, Link::kNeighbor)));
		}

		//============================================================
		// Local loop closure in TIME
		//============================================================
		if(_localLoopClosureDetectionTime &&
		   rehearsedId == 0 && // don't do it if rehearsal happened
		   signature->getWords3().size())
		{
			const std::set<int> & stm = _memory->getStMem();
			for(std::set<int>::const_reverse_iterator iter = stm.rbegin(); iter!=stm.rend(); ++iter)
			{
				if(*iter != signature->id() &&
				   signature->getNeighbors().find(*iter) == signature->getNeighbors().end() &&
				   _memory->getSignature(*iter)->mapId() == signature->mapId())
				{
					UDEBUG("Check local transform between %d and %d", signature->id(), *iter);
					Transform transform = _memory->computeVisualTransform(*iter, signature->id());
					if(!transform.isNull() && _globalLoopClosureIcpType > 0)
					{
						Transform icpTransform = _memory->computeIcpTransform(*iter, signature->id(), transform, _globalLoopClosureIcpType==1);
						float squaredNorm = (transform.inverse()*icpTransform).getNormSquared();
						if(!icpTransform.isNull() &&
							_globalLoopClosureIcpMaxDistance>0.0f &&
							squaredNorm > _globalLoopClosureIcpMaxDistance*_globalLoopClosureIcpMaxDistance)
						{
							UWARN("Local loop closure rejected (%d->%d) (ICP correction too large %f > %f [squared norm])",
									signature->id(),
									*iter,
									squaredNorm,
									_globalLoopClosureIcpMaxDistance*_globalLoopClosureIcpMaxDistance);
							icpTransform.setNull();
						}
						transform = icpTransform;
					}
					if(!transform.isNull())
					{
						UDEBUG("Add local loop closure in TIME (%d->%d) %s",
								signature->id(),
								*iter,
								transform.prettyPrint().c_str());
						// Add a loop constraint
						if(_memory->addLoopClosureLink(*iter, signature->id(), transform, false))
						{
							++localLoopClosuresInTimeFound;
							UINFO("Local loop closure found between %d and %d with t=%s",
									*iter, signature->id(), transform.prettyPrint().c_str());
						}
						else
						{
							UWARN("Cannot add local loop closure between %d and %d ?!?",
									*iter, signature->id());
						}
					}
				}
			}
		}
	}

	timeLocalTimeDetection = timer.ticks();
	UINFO("timeLocalTimeDetection=%fs", timeLocalTimeDetection);

	//============================================================
	// Bayes filter update
	//============================================================
	if(!signature->isBadSignature())
	{
		// If the working memory is empty, don't do the detection. It happens when it
		// is the first time the detector is started (there needs some images to
		// fill the short-time memory before a signature is added to the working memory).
		if(_memory->getWorkingMem().size())
		{
			//============================================================
			// Likelihood computation
			// Get the likelihood of the new signature
			// with all images contained in the working memory + reactivated.
			//============================================================
			ULOGGER_INFO("computing likelihood...");
			const std::set<int> & wm = _memory->getWorkingMem();

			std::list<int> signaturesToCompare(wm.begin(), wm.end());
			rawLikelihood = _memory->computeLikelihood(signature, signaturesToCompare);

			// Adjust the likelihood (with mean and std dev)
			likelihood = rawLikelihood;
			this->adjustLikelihood(likelihood);

			timeLikelihoodCalculation = timer.ticks();
			ULOGGER_INFO("timeLikelihoodCalculation=%fs",timeLikelihoodCalculation);

			//============================================================
			// Apply the Bayes filter
			//  Posterior = Likelihood x Prior
			//============================================================
			ULOGGER_INFO("getting posterior...");

			// Compute the posterior
			posterior = _bayesFilter->computePosterior(_memory, likelihood);
			timePosteriorCalculation = timer.ticks();
			ULOGGER_INFO("timePosteriorCalculation=%fs",timePosteriorCalculation);


			timer.start();
			//============================================================
			// Select the highest hypothesis
			//============================================================
			ULOGGER_INFO("creating hypotheses...");
			if(posterior.size())
			{
				hypothesis.first = 0;
				hypothesis.second = 0.0f;
				for(std::map<int, float>::const_reverse_iterator iter = posterior.rbegin(); iter != posterior.rend(); ++iter)
				{
					if(iter->first > 0 && iter->second > hypothesis.second)
					{
						hypothesis.first = iter->first;
						hypothesis.second = iter->second;
					}
				}
				// With the virtual place, use sum of LC probabilities (1 - virtual place hypothesis).
				hypothesis.second = 1-posterior.begin()->second;
			}
			timeHypothesesCreation = timer.ticks();
			ULOGGER_INFO("Hypothesis=%d, value=%f, timeHypothesesCreation=%fs", hypothesis.first, hypothesis.second, timeHypothesesCreation);

			if(hypothesis.first > 0)
			{
				// Loop closure Threshold
				// When _loopThr=0, accept loop closure if the hypothesis is over
				// the virtual (new) place hypothesis.
				if(hypothesis.second >= _loopThr)
				{
					//============================================================
					// Hypothesis verification for loop closure with geometric
					// information (like the epipolar geometry or using the local map
					// associated with the signature)
					//============================================================
					if(_lcHypothesisValue && hypothesis.second >= _loopRatio*_lcHypothesisValue &&
					   (!_epipolarGeometry || _epipolarGeometry->check(signature, _memory->getSignature(hypothesis.first))))
					{
						_lcHypothesisId = hypothesis.first;
					}
					else
					{
						if(_lcHypothesisValue == 0.0f)
						{
							UDEBUG("rejected hypothesis: last closure hypothesis is null");
						}
						else if(hypothesis.second < _loopRatio*_lcHypothesisValue)
						{
							UDEBUG("rejected hypothesis: not satisfying ratio");
						}
						else
						{
							UDEBUG("rejected hypothesis: by epipolar geometry");
						}
						rejectedHypothesis = true;
					}

					timeHypothesesValidation = timer.ticks();
					ULOGGER_INFO("timeHypothesesValidation=%fs",timeHypothesesValidation);
				}
				else if(hypothesis.second < _loopRatio*_lcHypothesisValue)
				{
					// Used for Precision-Recall computation.
					// When analysing logs, it's convenient to know
					// if the hypothesis would be rejected if T_loop would be lower.
					rejectedHypothesis = true;
				}

				//============================================================
				// Retrieval id update
				//============================================================
				_retrievedId = hypothesis.first;

				//for statistic...
				hypothesisRatio = _lcHypothesisValue>0?hypothesis.second/_lcHypothesisValue:0;
				_lcHypothesisValue = hypothesis.second;

			}
		} // if(_memory->getWorkingMemSize())
	}// !isBadSignature

	//============================================================
	// Before retrieval, make sure the trash has finished
	//============================================================
	_memory->joinTrashThread();
	timeEmptyingTrash = _memory->getDbSavingTime();
	timeJoiningTrash = timer.ticks();
	ULOGGER_INFO("Time emptying memory trash = %fs,  joining (actual overhead) = %fs", timeEmptyingTrash, timeJoiningTrash);

	// also copy weights before the memory is changed
	if(_publishStats && (_publishLikelihood || _publishPdf))
	{
		weights = _memory->getWeights();
	}

	//============================================================
	// RETRIEVAL : Loop closure neighbors reactivation
	//============================================================
	if(_retrievedId > 0 )
	{
		//Load neighbors
		ULOGGER_INFO("Retrieving locations... around id=%d", _retrievedId);
		unsigned int neighborhoodSize = _bayesFilter->getPredictionLC().size()-1;
		unsigned int margin = neighborhoodSize;

		UTimer timeGetN;
		unsigned int nbLoadedFromDb = 0;
		std::set<int> reactivatedIdsSet;
		std::list<int> reactivatedIds;
		double timeGetNeighborsTimeDb = 0.0;
		double timeGetNeighborsSpaceDb = 0.0;
		std::map<int, int> neighbors;
		bool firstPassDone = false;
		unsigned int m = 0;
		int nbDirectNeighborsInDb = 0;

		// priority in time
		// Direct neighbors TIME
		ULOGGER_DEBUG("In TIME");
		neighbors = _memory->getNeighborsId(_retrievedId,
				margin,
				_maxRetrieved,
				true,
				true,
				&timeGetNeighborsTimeDb);
		//Priority to locations near in time (direct neighbor) then by space (loop closure)
		while(m < margin)
		{
			std::set<int> idsSorted;
			for(std::map<int, int>::iterator iter=neighbors.begin(); iter!=neighbors.end();)
			{
				if(!firstPassDone && _memory->isInSTM(iter->first))
				{
					neighbors.erase(iter++);
				}
				else if((unsigned int)iter->second == m)
				{
					if(reactivatedIdsSet.find(iter->first) == reactivatedIdsSet.end())
					{
						idsSorted.insert(iter->first);
						reactivatedIdsSet.insert(iter->first);

						if(m == 1 && _memory->getSignature(iter->first) == 0)
						{
							++nbDirectNeighborsInDb;
						}

						if(m<neighborhoodSize)
						{
							//immunized locations in the neighborhood from being transferred
							immunizedLocations.insert(iter->first);
						}
						UDEBUG("nt=%d m=%d immunized=1", iter->first, iter->second);
					}
					std::map<int, int>::iterator tmp = iter++;
					neighbors.erase(tmp);
				}
				else
				{
					++iter;
				}
			}
			firstPassDone = true;
			reactivatedIds.insert(reactivatedIds.end(), idsSorted.rbegin(), idsSorted.rend());
			++m;
		}

		// neighbors SPACE, already added direct neighbors will be ignored
		ULOGGER_DEBUG("In SPACE");
		neighbors = _memory->getNeighborsId(_retrievedId,
				margin,
				_maxRetrieved,
				true,
				false,
				&timeGetNeighborsSpaceDb);
		m = 0;
		firstPassDone = false;
		while(m < margin)
		{
			std::set<int> idsSorted;
			for(std::map<int, int>::iterator iter=neighbors.begin(); iter!=neighbors.end();)
			{
				if(!firstPassDone && _memory->isInSTM(iter->first))
				{
					neighbors.erase(iter++);
				}
				else if((unsigned int)iter->second == m)
				{
					if(reactivatedIdsSet.find(iter->first) == reactivatedIdsSet.end())
					{
						idsSorted.insert(iter->first);
						reactivatedIdsSet.insert(iter->first);

						if(m == 1 && _memory->getSignature(iter->first) == 0)
						{
							++nbDirectNeighborsInDb;
						}
					}
					std::map<int, int>::iterator tmp = iter++;
					neighbors.erase(tmp);
				}
				else
				{
					++iter;
				}
			}
			firstPassDone = true;
			reactivatedIds.insert(reactivatedIds.end(), idsSorted.rbegin(), idsSorted.rend());
			++m;
		}
		ULOGGER_INFO("margin=%d, "
				"neighborhoodSize=%d, "
				"reactivatedIds.size=%d, "
				"nbLoadedFromDb=%d, "
				"nbDirectNeighborsInDb=%d, "
				"time=%fs (%fs %fs)",
				margin,
				neighborhoodSize,
				reactivatedIds.size(),
				(int)nbLoadedFromDb,
				nbDirectNeighborsInDb,
				timeGetN.ticks(),
				timeGetNeighborsTimeDb,
				timeGetNeighborsSpaceDb);

		// Not important if the loop closure hypothesis don't have all its neighbors loaded,
		// only a loop closure link is added...
		signaturesRetrieved = _memory->reactivateSignatures(
				reactivatedIds,
				_maxRetrieved,
				timeRetrievalDbAccess);

		ULOGGER_INFO("retrieval of %d reactivatedIds=%fs (db time = %fs)", (int)signaturesRetrieved.size(), timeGetN.ticks(), timeRetrievalDbAccess);

		timeRetrievalDbAccess += timeGetNeighborsTimeDb + timeGetNeighborsSpaceDb;
		UINFO("total timeRetrievalDbAccess=%fs", timeRetrievalDbAccess);

		// Immunize just retrieved signatures
		immunizedLocations.insert(signaturesRetrieved.begin(), signaturesRetrieved.end());
	}
	timeReactivations = timer.ticks();
	ULOGGER_INFO("timeReactivations=%fs", timeReactivations);

	//=============================================================
	// Update loop closure links
	// (updated: place this after retrieval to be sure that neighbors of the loop closure are in RAM)
	//=============================================================
	// Make the new one the parent of the old one
	if(_lcHypothesisId>0)
	{
		//Compute transform if metric data are present
		Transform transform;
		if(_rgbdSlamMode)
		{
			transform = _memory->computeVisualTransform(_lcHypothesisId, signature->id());
			if(!transform.isNull() && _globalLoopClosureIcpType > 0)
			{
				Transform icpTransform  = _memory->computeIcpTransform(_lcHypothesisId, signature->id(), transform, _globalLoopClosureIcpType == 1);
				float squaredNorm = (transform.inverse()*icpTransform).getNormSquared();
				if(!icpTransform.isNull() &&
					_globalLoopClosureIcpMaxDistance>0.0f &&
					squaredNorm > _globalLoopClosureIcpMaxDistance*_globalLoopClosureIcpMaxDistance)
				{
					UWARN("Global loop closure rejected (%d->%d) (ICP correction too large %f > %f [squared norm])",
							signature->id(),
							_lcHypothesisId,
							squaredNorm,
							_globalLoopClosureIcpMaxDistance*_globalLoopClosureIcpMaxDistance);
					icpTransform.setNull();
				}
				transform = icpTransform;
			}
			rejectedHypothesis = transform.isNull();
			if(rejectedHypothesis)
			{
				UWARN("Cannot compute a loop closure transform between %d and %d", _lcHypothesisId, signature->id());
			}
		}
		if(!rejectedHypothesis)
		{
			rejectedHypothesis = !_memory->addLoopClosureLink(_lcHypothesisId, signature->id(), transform, true);
		}

		if(rejectedHypothesis)
		{
			_lcHypothesisId = 0;
		}
		else
		{
			// used for localization mode
			_mapTransform.setIdentity();
			const Signature * oldS = _memory->getSignature(_lcHypothesisId);
			UASSERT(oldS != 0);
			if(oldS->mapId() != signature->mapId())
			{
				// New map -> old map
				_mapTransform = oldS->getPose() * transform.inverse() * signature->getPose().inverse();

				UINFO("Adding loop closure between %d and %d which don't belong to the same map (%d vs %d). "
					  "Add map transform between map %d and %d (%s).",
					  oldS->id(), signature->id(),
					  oldS->mapId(), signature->mapId(),
					  oldS->mapId(), signature->mapId(),
					  _mapTransform.prettyPrint().c_str());
			}
		}
	}
	timeAddLoopClosureLink = timer.ticks();
	ULOGGER_INFO("timeAddLoopClosureLink=%fs", timeAddLoopClosureLink);

	int localSpaceDetectionPosesCount = 0;
	int localSpaceClosureId = 0;
	int localSpaceNearestId = 0;
	if(_lcHypothesisId == 0 &&
	   _localLoopClosureDetectionSpace &&
	   signature->getDepth2D().size())
	{
		//============================================================
		// Scan matching LOCAL LOOP CLOSURE SPACE
		//============================================================
		// get all nodes in radius of the current node
		std::map<int, Transform> poses;
		localSpaceNearestId = 0;
		poses = this->getOptimizedWMPosesInRadius(signature->id(), _localDetectMaxNeighbors, _localDetectRadius, _localDetectMaxDiffID, localSpaceNearestId);

		// add current node to poses
		UASSERT(_optimizedPoses.find(signature->id()) != _optimizedPoses.end());
		poses.insert(std::make_pair(signature->id(), _optimizedPoses.at(signature->id())));

		localSpaceDetectionPosesCount = poses.size()-1;
		//The nearest will be the reference for a loop closure transform
		if(poses.size() &&
				localSpaceNearestId &&
			signature->getChildLoopClosureIds().find(localSpaceNearestId) == signature->getChildLoopClosureIds().end())
		{
			Transform t = _memory->computeScanMatchingTransform(signature->id(), localSpaceNearestId, poses);
			if(!t.isNull())
			{
				localSpaceClosureId = localSpaceNearestId;
				UDEBUG("Add local loop closure in SPACE (%d->%d) %s",
						signature->id(),
						localSpaceNearestId,
						t.prettyPrint().c_str());
				_memory->addLoopClosureLink(localSpaceNearestId, signature->id(), t, false);
			}
		}
	}
	timeLocalSpaceDetection = timer.ticks();
	ULOGGER_INFO("timeLocalSpaceDetection=%fs", timeLocalSpaceDetection);

	//============================================================
	// Optimize map graph
	//============================================================
	if(_rgbdSlamMode &&
		(_lcHypothesisId>0 ||				// can be different map of the current one
		 localLoopClosuresInTimeFound>0 || 	// only same map of the current one
		 scanMatchingSuccess || 			// only same map of the current one
		 localSpaceClosureId>0 || 			// can be different map of the current one
		 signaturesRetrieved.size()))  		// can be different map of the current one
	{
		if(_memory->isIncremental())
		{
			UINFO("Update map correction: SLAM mode");
			// SLAM mode!
			optimizeCurrentMap(signature->id(), false, _optimizedPoses, &_constraints);
		}
		else if(_lcHypothesisId > 0 || localSpaceClosureId > 0 || signaturesRetrieved.size())
		{
			UINFO("Update map correction: Localization mode");
			int oldId = _lcHypothesisId>0?_lcHypothesisId:localSpaceClosureId?localSpaceClosureId:_retrievedId;
			UASSERT(oldId != 0);
			if(signaturesRetrieved.size() || _optimizedPoses.find(oldId) == _optimizedPoses.end())
			{
				// update optimized poses
				optimizeCurrentMap(_retrievedId, false, _optimizedPoses, &_constraints);
			}

			if(_optimizedPoses.find(oldId) == _optimizedPoses.end())
			{
				UERROR("Cannot find optimized pose for %d!?!?", oldId);
			}
			else
			{
				// Localization mode! only update map correction
				const Signature * oldS = _memory->getSignature(oldId);
				UASSERT(oldS != 0);
				Transform correction = _optimizedPoses.at(oldId) * oldS->getPose().inverse();
				_mapCorrection = correction * _mapTransform;
			}
		}
		else
		{
			UERROR("Not supposed to be here!");
		}
	}
	Transform currentRawOdomPose = signature->getPose();

	timeMapOptimization = timer.ticks();
	ULOGGER_INFO("timeMapOptimization=%fs", timeMapOptimization);

	//============================================================
	// Prepare statistics
	//============================================================
	// Data used for the statistics event and for the log files
	int dictionarySize = 0;
	int refWordsCount = 0;
	int refUniqueWordsCount = 0;
	int lcHypothesisReactivated = 0;
	float rehearsalValue = uValue(statistics_.data(), Statistics::kMemoryRehearsal_sim(), 0.0f);
	int rehearsalMaxId = (int)uValue(statistics_.data(), Statistics::kMemoryRehearsal_merged(), 0.0f);
	sLoop = _memory->getSignature(_lcHypothesisId?_lcHypothesisId:localSpaceClosureId?localSpaceClosureId:hypothesis.first);
	if(sLoop)
	{
		lcHypothesisReactivated = sLoop->isSaved()?1.0f:0.0f;
	}
	dictionarySize = _memory->getVWDictionarySize();
	refWordsCount = signature->getWords().size();
	refUniqueWordsCount = uUniqueKeys(signature->getWords()).size();

	// Posterior is empty if a bad signature is detected
	float vpHypothesis = posterior.size()?posterior.at(Memory::kIdVirtual):0.0f;

	// prepare statistics
	if(_lcHypothesisId || _publishStats)
	{
		ULOGGER_INFO("sending stats...");
		statistics_.setRefImageId(signature->id());
		if(_lcHypothesisId != Memory::kIdInvalid)
		{
			statistics_.setLoopClosureId(_lcHypothesisId);
			ULOGGER_INFO("Loop closure detected! With id=%d", _lcHypothesisId);
		}
		if(_publishStats)
		{
			ULOGGER_INFO("send all stats...");
			statistics_.setExtended(1);

			statistics_.addStatistic(Statistics::kLoopHighest_hypothesis_id(), hypothesis.first);
			statistics_.addStatistic(Statistics::kLoopHighest_hypothesis_value(), hypothesis.second);
			statistics_.addStatistic(Statistics::kLoopHypothesis_reactivated(), lcHypothesisReactivated);
			statistics_.addStatistic(Statistics::kLoopVp_hypothesis(), vpHypothesis);
			statistics_.addStatistic(Statistics::kLoopReactivateId(), _retrievedId);
			statistics_.addStatistic(Statistics::kLoopHypothesis_ratio(), hypothesisRatio);

			statistics_.addStatistic(Statistics::kLocalLoopOdom_corrected(), scanMatchingSuccess?1:0);
			statistics_.addStatistic(Statistics::kLocalLoopTime_closures(), localLoopClosuresInTimeFound);
			statistics_.addStatistic(Statistics::kLocalLoopSpace_neighbors(), localSpaceDetectionPosesCount);
			statistics_.addStatistic(Statistics::kLocalLoopSpace_closure_id(), localSpaceClosureId);
			statistics_.addStatistic(Statistics::kLocalLoopSpace_nearest_id(), localSpaceNearestId);
			if(localSpaceClosureId)
			{
				statistics_.setLocalLoopClosureId(localSpaceClosureId);
			}
			if(localSpaceNearestId)
			{
				int d1 = abs(signature->id() - localSpaceNearestId);
				int d2 = abs(localSpaceNearestId - _memory->getLastGlobalLoopClosureChildId());
				int d3 = abs(signature->id() - _memory->getLastGlobalLoopClosureParentId());
				int d = d1<=d2?d1:d2;
				d = d <= d3?d:d3;
				statistics_.addStatistic(Statistics::kLocalLoopSpace_diff_id(), d);
			}
			if(_lcHypothesisId || localSpaceClosureId)
			{
				UASSERT(uContains(sLoop->getLoopClosureIds(), signature->id()));
				UINFO("Set loop closure transform = %s", sLoop->getLoopClosureIds().at(signature->id()).prettyPrint().c_str());
				statistics_.setLoopClosureTransform(sLoop->getLoopClosureIds().at(signature->id()));
			}

			statistics_.addStatistic(Statistics::kMemoryWorking_memory_size(), _memory->getWorkingMem().size());
			statistics_.addStatistic(Statistics::kMemoryShort_time_memory_size(), _memory->getStMem().size());
			statistics_.addStatistic(Statistics::kMemorySignatures_retrieved(), (float)signaturesRetrieved.size());

			// timing...
			statistics_.addStatistic(Statistics::kTimingMemory_update(), timeMemoryUpdate*1000);
			statistics_.addStatistic(Statistics::kTimingScan_matching(), timeScanMatching*1000);
			statistics_.addStatistic(Statistics::kTimingLocal_detection_TIME(), timeLocalTimeDetection*1000);
			statistics_.addStatistic(Statistics::kTimingLocal_detection_SPACE(), timeLocalSpaceDetection*1000);
			statistics_.addStatistic(Statistics::kTimingReactivation(), timeReactivations*1000);
			statistics_.addStatistic(Statistics::kTimingAdd_loop_closure_link(), timeAddLoopClosureLink*1000);
			statistics_.addStatistic(Statistics::kTimingMap_optimization(), timeMapOptimization*1000);
			statistics_.addStatistic(Statistics::kTimingLikelihood_computation(), timeLikelihoodCalculation*1000);
			statistics_.addStatistic(Statistics::kTimingPosterior_computation(), timePosteriorCalculation*1000);
			statistics_.addStatistic(Statistics::kTimingHypotheses_creation(), timeHypothesesCreation*1000);
			statistics_.addStatistic(Statistics::kTimingHypotheses_validation(), timeHypothesesValidation*1000);
			statistics_.addStatistic(Statistics::kTimingCleaning_neighbors(), timeCleaningNeighbors*1000);

			// Surf specific parameters
			statistics_.addStatistic(Statistics::kKeypointDictionary_size(), dictionarySize);

			//Epipolar geometry constraint
			statistics_.addStatistic(Statistics::kLoopRejectedHypothesis(), rejectedHypothesis?1.0f:0);

			if(_publishImage)
			{
				std::map<int, int> mapIds;
				std::map<int, std::vector<unsigned char> > images;
				std::map<int, std::vector<unsigned char> > depths;
				std::map<int, std::vector<unsigned char> > depth2ds;
				std::map<int, float> depthConstants;
				std::map<int, Transform> localTransforms;

				std::vector<int> ids(signaturesRetrieved.begin(), signaturesRetrieved.end());
				ids.push_back(signature->id());
				if(sLoop)
				{
					ids.push_back(sLoop->id());
				}

				UTimer tmpTimer;
				for(unsigned int i=0; i<ids.size(); ++i)
				{
					// Add data
					std::vector<unsigned char> im;
					if(_rgbdSlamMode)
					{
						std::vector<unsigned char> depth, depth2d;
						float depthConstant;
						Transform localTransform;
						_memory->getImageDepth(ids[i], im, depth, depth2d, depthConstant, localTransform);

						if(!depth.empty())
						{
							depths.insert(std::make_pair(ids[i], depth));
							depthConstants.insert(std::make_pair(ids[i], depthConstant));
							localTransforms.insert(std::make_pair(ids[i], localTransform));
						}
						if(!depth2d.empty())
						{
							depth2ds.insert(std::make_pair(ids[i], depth2d));
						}
					}
					else
					{
						im = _memory->getImage(ids[i]);
					}
					UASSERT(_memory->getSignature(ids[i]) != 0);
					mapIds.insert(std::make_pair(ids[i], _memory->getSignature(ids[i])->mapId()));
					if(!im.empty())
					{
						images.insert(std::make_pair(ids[i], im));
					}
				}

				if(tmpTimer.elapsed() > 0.03)
				{
					UWARN("getting data[%d] time = %fs", (int)ids.size(), tmpTimer.ticks());
				}

				statistics_.setMapIds(mapIds);
				statistics_.setImages(images);
				statistics_.setDepths(depths);
				statistics_.setDepth2ds(depth2ds);
				statistics_.setDepthConstants(depthConstants);
				statistics_.setLocalTransforms(localTransforms);
			}

			if(_publishLikelihood || _publishPdf)
			{
				// Child count by parent signature on the root of the memory ... for statistics
				statistics_.setWeights(weights);
				if(_publishPdf)
				{
					statistics_.setPosterior(posterior);
				}
				if(_publishLikelihood)
				{
					statistics_.setLikelihood(likelihood);
					statistics_.setRawLikelihood(rawLikelihood);
				}
			}

			if(_publishKeypoints)
			{
				//Copy keypoints
				statistics_.setRefWords(signature->getWords());
				if(sLoop)
				{
					//Copy keypoints
					statistics_.setLoopWords(sLoop->getWords());
				}
			}
		}

		timeStatsCreation = timer.ticks();
		ULOGGER_INFO("Time creating stats = %f...", timeStatsCreation);
	}

	// Pass this point signature should not be used, since it could be transferred...
	signature = 0;

	//By default, remove all signatures with a loop closure link if they are not in reactivateIds
	//This will also remove rehearsed signatures
	std::list<int> signaturesRemoved = _memory->cleanup();
	timeMemoryCleanup = timer.ticks();
	ULOGGER_INFO("timeMemoryCleanup = %fs... %d signatures removed", timeMemoryCleanup, (int)signaturesRemoved.size());

	//============================================================
	// TRANSFER
	//============================================================
	// If time allowed for the detection exceeds the limit of
	// real-time, move the oldest signature with less frequency
	// entry (from X oldest) from the short term memory to the
	// long term memory.
	//============================================================
	double totalTime = timerTotal.ticks();
	ULOGGER_INFO("Total time processing = %fs...", totalTime);
	timer.start();
	if((_maxTimeAllowed != 0 && totalTime*1000>_maxTimeAllowed) ||
		(_maxMemoryAllowed != 0 && _memory->getWorkingMem().size() > _maxMemoryAllowed))
	{
		ULOGGER_INFO("Removing old signatures because time limit is reached %f>%f or memory is reached %d>%d...", totalTime*1000, _maxTimeAllowed, _memory->getWorkingMem().size(), _maxMemoryAllowed);
		std::list<int> transferred = _memory->forget(immunizedLocations);
		signaturesRemoved.insert(signaturesRemoved.end(), transferred.begin(), transferred.end());
	}
	_lastProcessTime = totalTime;

	//Remove optimized poses from signatures transferred
	for(std::list<int>::iterator iter = signaturesRemoved.begin(); iter!=signaturesRemoved.end(); ++iter)
	{
		UDEBUG("removing optimized pose %d...", *iter);
		_optimizedPoses.erase(*iter);
		_constraints.erase(*iter);
	}


	timeRealTimeLimitReachedProcess = timer.ticks();
	ULOGGER_INFO("Time limit reached processing = %f...", timeRealTimeLimitReachedProcess);

	//Start trashing
	_memory->emptyTrash();

	//==============================================================
	// Finalize statistics and log files
	//==============================================================
	if(_publishStats)
	{
		statistics_.addStatistic(Statistics::kTimingStatistics_creation(), timeStatsCreation*1000);
		statistics_.addStatistic(Statistics::kTimingTotal(), totalTime*1000);
		statistics_.addStatistic(Statistics::kTimingForgetting(), timeRealTimeLimitReachedProcess*1000);
		statistics_.addStatistic(Statistics::kTimingJoining_trash(), timeJoiningTrash*1000);
		statistics_.addStatistic(Statistics::kTimingEmptying_trash(), timeEmptyingTrash*1000);
		statistics_.addStatistic(Statistics::kTimingMemory_cleanup(), timeMemoryCleanup*1000);
		statistics_.addStatistic(Statistics::kMemorySignatures_removed(), signaturesRemoved.size());

		//Poses, place this after Transfer! (_optimizedPoses may change)
		if(_rgbdSlamMode)
		{
			statistics_.setPoses(_optimizedPoses);
			statistics_.setConstraints(_constraints);
			statistics_.setMapCorrection(_mapCorrection);
			statistics_.setCurrentPose(_mapCorrection * currentRawOdomPose);
			UINFO("Set map correction = %s", _mapCorrection.prettyPrint().c_str());
		}
	}

	// Log info...
	// TODO : use a specific class which will handle the RtabmapEvent
	std::string logF = uFormat("%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n",
								totalTime,
								timeMemoryUpdate,
								timeReactivations,
								timeLikelihoodCalculation,
								timePosteriorCalculation,
								timeHypothesesCreation,
								timeHypothesesValidation,
								timeRealTimeLimitReachedProcess,
								timeStatsCreation,
								_lcHypothesisValue,
								0.0f,
								0.0f,
								0.0f,
								0.0f,
								0.0f,
								vpHypothesis,
								timeJoiningTrash,
								rehearsalValue,
								timeEmptyingTrash,
								timeRetrievalDbAccess,
								timeAddLoopClosureLink);
	std::string logI = uFormat("%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d\n",
								_lcHypothesisId,
								hypothesis.first,
								(int)signaturesRemoved.size(),
								0,
								refWordsCount,
								dictionarySize,
								int(_memory->getWorkingMem().size()),
								rejectedHypothesis?1:0,
								0,
								0,
								int(signaturesRetrieved.size()),
								lcHypothesisReactivated,
								refUniqueWordsCount,
								_retrievedId,
								0.0f,
								rehearsalMaxId,
								rehearsalMaxId>0?1:0);
	if(_statisticLogsBufferedInRAM)
	{
		_bufferedLogsF.push_back(logF);
		_bufferedLogsI.push_back(logI);
	}
	else
	{
		if(_foutFloat)
		{
			fprintf(_foutFloat, "%s", logF.c_str());
		}
		if(_foutInt)
		{
			fprintf(_foutInt, "%s", logI.c_str());
		}
	}
	UINFO("Time logging = %f...", timer.ticks());
	//ULogger::flush();

	return true;
}

// SETTERS
void Rtabmap::setTimeThreshold(float maxTimeAllowed)
{
	//must be positive, 0 mean inf time allowed (no time limit)
	_maxTimeAllowed = maxTimeAllowed;
	if(_maxTimeAllowed < 0)
	{
		ULOGGER_WARN("maxTimeAllowed < 0, then setting it to 0 (inf).");
		_maxTimeAllowed = 0;
	}
	else if(_maxTimeAllowed < 1)
	{
		ULOGGER_WARN("Time threshold set to %fms, it is not in seconds!", _maxTimeAllowed);
	}
}

void Rtabmap::setWorkingDirectory(std::string path)
{
	if(path.size() && (path.at(path.size()-1) != '\\' || path.at(path.size()-1) != '/' ))
	{
		path += UDirectory::separator();
	}

	if(!path.empty() && UDirectory::exists(path))
	{
		ULOGGER_DEBUG("Comparing new working directory path \"%s\" with \"%s\"", path.c_str(), _wDir.c_str());
		if(path.compare(_wDir) != 0)
		{
			_wDir = path;
			if(_memory)
			{
				this->resetMemory();
			}
			else
			{
				setupLogFiles();
			}
		}
	}
	else
	{
		ULOGGER_ERROR("Directory \"%s\" doesn't exist!", path.c_str());
	}
}

void Rtabmap::setDatabasePath(const std::string & path)
{
	if(!path.empty())
	{
		ULOGGER_DEBUG("Comparing new database path \"%s\" with \"%s\"", path.c_str(), _databasePath.c_str());
		if(path.compare(_databasePath) != 0)
		{
			UDEBUG("Set new database path to \"%s\"", _databasePath.c_str());
			_databasePath = path;
			if(_memory)
			{
				UDEBUG("Reset memory!");
				this->resetMemory();
			}
		}
	}
	else
	{
		ULOGGER_ERROR("Cannot set null database path!");
	}
}

void Rtabmap::deleteLocation(int locationId)
{
	if(_memory)
	{
		_memory->deleteLocation(locationId);
	}
}

void Rtabmap::rejectLoopClosure(int oldId, int newId)
{
	UDEBUG("_lcHypothesisId=%d", _lcHypothesisId);
	if(_lcHypothesisId)
	{
		_lcHypothesisId = 0;
		if(_memory)
		{
			_memory->rejectLoopClosure(oldId, newId);
		}
		if(uContains(statistics_.data(), rtabmap::Statistics::kLoopRejectedHypothesis()))
		{
			statistics_.addStatistic(rtabmap::Statistics::kLoopRejectedHypothesis(), 1.0f);
		}
		statistics_.setLoopClosureId(0);
	}
}

bool Rtabmap::process(const cv::Mat & image, int id)
{
	return this->process(Image(image, id));
}

void Rtabmap::dumpData() const
{
	UDEBUG("");
	if(_memory)
	{
		_memory->dumpMemory(this->getWorkingDir());
	}
}

// fromId must be in _memory and in _optimizedPoses
std::map<int, Transform> Rtabmap::getOptimizedWMPosesInRadius(
		int fromId,
		int maxNearestNeighbors,
		float radius,
		int maxDiffID, // 0 means ignore
		int & nearestId) const
{
	UDEBUG("");
	const Signature * fromS = _memory->getSignature(fromId);
	UASSERT(fromS != 0);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	cloud->resize(_optimizedPoses.size());
	std::vector<int> ids(_optimizedPoses.size());
	int oi = 0;
	const std::set<int> & stm = _memory->getStMem();
	for(std::map<int, Transform>::const_iterator iter = _optimizedPoses.begin(); iter!=_optimizedPoses.end(); ++iter)
	{
		// Only locations in Working Memory with ID not too far from the last loop closure child id
		bool diffIdOk = maxDiffID == 0 || abs(fromId - iter->first) <= maxDiffID || (abs(iter->first - _memory->getLastGlobalLoopClosureChildId()) <= maxDiffID && abs(fromId - _memory->getLastGlobalLoopClosureParentId()) <= maxDiffID);
		if(stm.find(iter->first) == stm.end() && diffIdOk)
		{
			(*cloud)[oi] = pcl::PointXYZ(iter->second.x(), iter->second.y(), iter->second.z());
			ids[oi++] = iter->first;
		}
	}

	cloud->resize(oi);
	ids.resize(oi);

	UASSERT(_optimizedPoses.find(fromId) != _optimizedPoses.end());
	Transform fromT = _optimizedPoses.at(fromId);

	nearestId = 0;
	std::map<int, Transform> poses;
	float minDistance = -1;
	if(cloud->size())
	{
		//filter poses in front of the fromId
		//pcl::io::savePCDFile("radiusPoses.pcd", *cloud);

		Transform t=Transform::getIdentity();
		t.x() = radius*0.95f;
		float x,y,z, roll,pitch,yaw;
		(fromT*t).getTranslationAndEulerAngles(x,y,z, roll,pitch,yaw);

		pcl::CropBox<pcl::PointXYZ> cropbox;
		cropbox.setInputCloud(cloud);
		cropbox.setMin(Eigen::Vector4f(-radius, -radius, -radius, 0));
		cropbox.setMax(Eigen::Vector4f(radius, radius, radius, 0));
		cropbox.setRotation(Eigen::Vector3f(roll, pitch, yaw));
		cropbox.setTranslation(Eigen::Vector3f(x, y, z));
		pcl::IndicesPtr indices(new std::vector<int>());
		cropbox.filter(*indices);

		//pcl::io::savePCDFile("radiusCrop.pcd", *cloud, *indices);

		if(indices->size())
		{
			pcl::search::KdTree<pcl::PointXYZ>::Ptr kdTree(new pcl::search::KdTree<pcl::PointXYZ>);
			kdTree->setInputCloud(cloud, indices);
			std::vector<int> ind;
			std::vector<float> dist;
			pcl::PointXYZ pt(fromT.x(), fromT.y(), fromT.z());
			kdTree->radiusSearch(pt, radius, ind, dist, maxNearestNeighbors);
			//pcl::PointCloud<pcl::PointXYZ> inliers;
			for(unsigned int i=0; i<ind.size(); ++i)
			{
				if(ind[i] >=0)
				{
					Transform tmp = _optimizedPoses.find(ids[ind[i]])->second;
					//inliers.push_back(pcl::PointXYZ(tmp.x(), tmp.y(), tmp.z()));
					UDEBUG("Inlier %d: %s", ids[ind[i]], tmp.prettyPrint().c_str());
					poses.insert(std::make_pair(ids[ind[i]], tmp));
					if(fromS->getNeighbors().find(ids[ind[i]]) == fromS->getNeighbors().end() && // can't be a neighbor
					   (minDistance == -1 || minDistance > dist[i]))
					{
						nearestId = ids[ind[i]];
						minDistance = dist[i];
					}
				}
			}

			//if(inliers.size())
			//{
			//	pcl::io::savePCDFile("radiusInliers.pcd", inliers);
			//}
			//if(nearestId >0)
			//{
			//	pcl::PointCloud<pcl::PointXYZ> c;
			//	Transform ct = _optimizedPoses.find(nearestId)->second;
			//	c.push_back(pcl::PointXYZ(ct.x(), ct.y(), ct.z()));
			//	pcl::io::savePCDFile("radiusNearestPt.pcd", c);
			//}

			if(nearestId > 0)
			{
				// Only take nodes linked with the nearest node
				std::map<int, int> neighbors = _memory->getNeighborsId(nearestId, maxNearestNeighbors, 0, true, true);
				for(std::map<int, Transform>::iterator iter = poses.begin(); iter!= poses.end();)
				{
					if(!uContains(neighbors, iter->first))
					{
						poses.erase(iter++);
					}
					else
					{
						++iter;
					}
				}
			}
			else if(poses.size())
			{
				UWARN("Flushing poses (%d) because nearest id of %d can't be found!", (int)poses.size(), fromId);
				poses.clear();
			}
		}
	}
	UDEBUG("nearestId = %d, minDistance=%f poses=%d", nearestId, minDistance, (int)poses.size());
	return poses;
}

void Rtabmap::optimizeCurrentMap(
		int id,
		bool lookInDatabase,
		std::map<int, Transform> & optimizedPoses,
		std::multimap<int, Link> * constraints) const
{
	//Optimize the map
	optimizedPoses.clear();
	UDEBUG("Optimize map: around location %d", id);
	if(_memory && id > 0)
	{

		std::map<int, int> ids = _memory->getNeighborsId(id, 0, lookInDatabase?-1:0, true);
		UDEBUG("ids=%d", (int)ids.size());

		std::map<int, Transform> poses;
		std::multimap<int, Link> edgeConstraints;
		_memory->getMetricConstraints(uKeys(ids), poses, edgeConstraints, lookInDatabase);
		UDEBUG("poses=%d, edgeConstraints=%d", (int)poses.size(), (int)edgeConstraints.size());

		if(constraints)
		{
			*constraints = edgeConstraints;
		}

		// Modify IDs using the margin from the current signature (TORO root will be the last signature)
		int m = 0;
		int toroId = 1;
		std::map<int, int> rtabmapToToro; // <RTAB-Map ID, TORO ID>
		std::map<int, int> toroToRtabmap; // <TORO ID, RTAB-Map ID>
		while(ids.size())
		{
			for(std::map<int, int>::iterator iter = ids.begin(); iter!=ids.end();)
			{
				if(m == iter->second)
				{
					rtabmapToToro.insert(std::make_pair(iter->first, toroId));
					toroToRtabmap.insert(std::make_pair(toroId, iter->first));
					++toroId;
					ids.erase(iter++);
				}
				else
				{
					++iter;
				}
			}
			++m;
		}

		//
		std::map<int, Transform> posesToro;
		std::multimap<int, Link> edgeConstraintsToro;
		for(std::map<int, Transform>::iterator iter = poses.begin(); iter!=poses.end(); ++iter)
		{
			posesToro.insert(std::make_pair(rtabmapToToro.at(iter->first), iter->second));
		}
		for(std::multimap<int, Link>::iterator iter = edgeConstraints.begin();
			iter!=edgeConstraints.end();
			++iter)
		{
			edgeConstraintsToro.insert(std::make_pair(rtabmapToToro.at(iter->first), Link(rtabmapToToro.at(iter->first), rtabmapToToro.at(iter->second.to()), iter->second.transform(), iter->second.type())));
		}

		if(posesToro.size() > 1 && edgeConstraintsToro.size() > 0)
		{
			///UDEBUG("TORO optimize begin");
			//util3d::saveTOROGraph("toroIdRtabmap.graph", poses, edgeConstraints);
			//util3d::saveTOROGraph("toroIdToro.graph", posesToro, edgeConstraintsToro);
			//UDEBUG("graph saved");

			std::map<int, Transform> optimizedPosesToro;
			Transform mapCorrectionToro;
			util3d::optimizeTOROGraph(posesToro, edgeConstraintsToro, optimizedPosesToro, mapCorrectionToro, _toroIterations, true);

			//UDEBUG("saving optimized graph...");
			//util3d::saveTOROGraph("toroOptimized.graph", optimizedPosesToro, edgeConstraintsToro);
			//UDEBUG("TORO optimize end");

			for(std::map<int, Transform>::iterator iter=optimizedPosesToro.begin(); iter!=optimizedPosesToro.end(); ++iter)
			{
				optimizedPoses.insert(std::make_pair(toroToRtabmap.at(iter->first), iter->second));
			}

			//util3d::saveTOROGraph("toro.graph", poses, edgeConstraints);
			//util3d::saveTOROGraph("toroOptimized.graph", _optimizedPoses, edgeConstraints);
		}
		else if(poses.size() == 1 && edgeConstraints.size() == 0)
		{
			optimizedPoses = poses;
		}
		else if(poses.size() || edgeConstraints.size())
		{
			UFATAL("Poses=%d and edges=%d (poses must "
				   "not be null if there are edges, and edges must be null if poses <= 1)",
				   poses.size(), edgeConstraints.size());
		}
	}
}

void Rtabmap::adjustLikelihood(std::map<int, float> & likelihood) const
{
	ULOGGER_DEBUG("likelihood.size()=%d", likelihood.size());
	UTimer timer;
	timer.start();
	if(likelihood.size()==0)
	{
		return;
	}

	// Use only non-null values (ignore virtual place)
	std::list<float> values;
	bool likelihoodNullValuesIgnored = true;
	for(std::map<int, float>::iterator iter = ++likelihood.begin(); iter!=likelihood.end(); ++iter)
	{
		if((iter->second >= 0 && !likelihoodNullValuesIgnored) ||
		   (iter->second > 0 && likelihoodNullValuesIgnored))
		{
			values.push_back(iter->second);
		}
	}
	UDEBUG("values.size=%d", values.size());

	float mean = uMean(values);
	float stdDev = uStdDev(values, mean);


	//Adjust likelihood with mean and standard deviation (see Angeli phd)
	float epsilon = 0.0001;
	float max = 0.0f;
	int maxId = 0;
	for(std::map<int, float>::iterator iter=++likelihood.begin(); iter!= likelihood.end(); ++iter)
	{
		float value = iter->second;
		if(value > mean+stdDev && mean)
		{
			iter->second = (value-(stdDev-epsilon))/mean;
			if(value > max)
			{
				max = value;
				maxId = iter->first;
			}
		}
		else if(value == 1.0f && stdDev == 0)
		{
			iter->second = 1.0f;
			if(value > max)
			{
				max = value;
				maxId = iter->first;
			}
		}
		else
		{
			iter->second = 1.0f;
		}
	}

	if(stdDev > epsilon && max)
	{
		likelihood.begin()->second = mean/stdDev + 1.0f;
	}
	else
	{
		likelihood.begin()->second = 2.0f; //2 * std dev
	}

	double time = timer.ticks();
	UDEBUG("mean=%f, stdDev=%f, max=%f, maxId=%d, time=%fs", mean, stdDev, max, maxId, time);
}

void Rtabmap::dumpPrediction() const
{
	if(_memory && _bayesFilter)
	{
		const std::set<int> & wm = _memory->getWorkingMem();
		cv::Mat prediction = _bayesFilter->generatePrediction(_memory, std::vector<int>(wm.begin(), wm.end()));

		FILE* fout = 0;
		std::string fileName = this->getWorkingDir() + "/DumpPrediction.txt";
		#ifdef _MSC_VER
			fopen_s(&fout, fileName.c_str(), "w");
		#else
			fout = fopen(fileName.c_str(), "w");
		#endif

		if(fout)
		{
			for(int i=0; i<prediction.rows; ++i)
			{
				for(int j=0; j<prediction.cols; ++j)
				{
					fprintf(fout, "%f ",((float*)prediction.data)[j + i*prediction.cols]);
				}
				fprintf(fout, "\n");
			}
			fclose(fout);
		}
	}
	else
	{
		UWARN("Memory and/or the Bayes filter are not created");
	}
}

void Rtabmap::get3DMap(std::map<int, std::vector<unsigned char> > & images,
		std::map<int, std::vector<unsigned char> > & depths,
		std::map<int, std::vector<unsigned char> > & depths2d,
		std::map<int, float> & depthConstants,
		std::map<int, Transform> & localTransforms,
		std::map<int, Transform> & poses,
		std::multimap<int, Link> & constraints,
		bool optimized,
		bool global) const
{
	if(_memory && _memory->getLastWorkingSignature())
	{
		if(optimized)
		{
			this->optimizeCurrentMap(_memory->getLastWorkingSignature()->id(), global, poses, &constraints);
		}
		else
		{
			std::map<int, int> ids = _memory->getNeighborsId(_memory->getLastWorkingSignature()->id(), 0, global?-1:0, true);
			_memory->getMetricConstraints(uKeys(ids), poses, constraints, global);
		}

		std::set<int> ids = _memory->getWorkingMem(); // STM + WM
		ids.insert(_memory->getStMem().begin(), _memory->getStMem().end());
		if(global)
		{
			ids = _memory->getAllSignatureIds(); // STM + WM + LTM
		}

		for(std::set<int>::iterator iter = ids.begin(); iter!=ids.end(); ++iter)
		{
			std::vector<unsigned char> image, depth, depth2d;
			float depthConstant;
			Transform localTransform;
			_memory->getImageDepth(*iter, image, depth, depth2d, depthConstant, localTransform);

			if(image.size())
			{
				images.insert(std::make_pair(*iter, image));
			}
			if(depth.size())
			{
				depths.insert(std::make_pair(*iter, depth));
			}
			if(depth2d.size())
			{
				depths2d.insert(std::make_pair(*iter, depth2d));
			}
			if(depthConstant > 0)
			{
				depthConstants.insert(std::make_pair(*iter, depthConstant));
			}
			if(!localTransform.isNull())
			{
				localTransforms.insert(std::make_pair(*iter, localTransform));
			}
		}
	}
}

void Rtabmap::getGraph(
		std::map<int, Transform> & poses,
		std::multimap<int, Link> & constraints,
		bool optimized,
		bool full)
{
	if(_memory && _memory->getLastWorkingSignature())
	{
		if(optimized)
		{
			this->optimizeCurrentMap(_memory->getLastWorkingSignature()->id(), full, poses, &constraints);
		}
		else
		{
			std::map<int, int> ids = _memory->getNeighborsId(_memory->getLastWorkingSignature()->id(), 0, full?-1:0, true);
			_memory->getMetricConstraints(uKeys(ids), poses, constraints, full);
		}
	}
}

void Rtabmap::readParameters(const std::string & configFile, ParametersMap & parameters)
{
	CSimpleIniA ini;
	ini.LoadFile(configFile.c_str());
	const CSimpleIniA::TKeyVal * keyValMap = ini.GetSection("Core");
	if(keyValMap)
	{
		for(CSimpleIniA::TKeyVal::const_iterator iter=keyValMap->begin(); iter!=keyValMap->end(); ++iter)
		{
			std::string key = (*iter).first.pItem;
			key = uReplaceChar(key, '\\', '/'); // Ini files use \ by default for separators, so replace them
			ParametersMap::iterator jter = parameters.find(key);
			if(jter != parameters.end())
			{
				parameters.erase(jter);
			}
			parameters.insert(ParametersPair(key, (*iter).second));
		}
	}
	else
	{
		ULOGGER_WARN("Section \"Core\" in %s doesn't exist... "
				    "Ignore this warning if the ini file does not exist yet. "
				    "The ini file will be automatically created when this node will close.", configFile.c_str());
	}
}

void Rtabmap::writeParameters(const std::string & configFile, const ParametersMap & parameters)
{
	CSimpleIniA ini;
	ini.LoadFile(configFile.c_str());

	for(ParametersMap::const_iterator i=parameters.begin(); i!=parameters.end(); ++i)
	{
		std::string key = (*i).first;
		key = uReplaceChar(key, '/', '\\'); // Ini files use \ by default for separators, so replace the /
		ini.SetValue("Core", key.c_str(), (*i).second.c_str(), NULL, true);
	}

	ini.SaveFile(configFile.c_str());
}

} // namespace rtabmap
