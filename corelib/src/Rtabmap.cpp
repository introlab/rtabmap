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

#include "rtabmap/core/Rtabmap.h"
#include "rtabmap/core/Version.h"
#include "rtabmap/core/Features2d.h"
#include "rtabmap/core/Optimizer.h"
#include "rtabmap/core/Graph.h"
#include "rtabmap/core/Signature.h"

#include "rtabmap/core/EpipolarGeometry.h"

#include "rtabmap/core/util3d.h"
#include "rtabmap/core/util3d_transforms.h"
#include "rtabmap/core/util3d_filtering.h"
#include "rtabmap/core/util3d_surface.h"

#include "rtabmap/core/DBDriver.h"
#include "rtabmap/core/Memory.h"
#include "rtabmap/core/VWDictionary.h"
#include "rtabmap/core/BayesFilter.h"
#include "rtabmap/core/Compression.h"
#include "rtabmap/core/RegistrationInfo.h"

#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UFile.h>
#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UMath.h>
#include <rtabmap/utilite/UProcessInfo.h>

#include <pcl/search/kdtree.h>
#include <pcl/filters/crop_box.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/TextureMesh.h>

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
	_publishLastSignatureData(Parameters::defaultRtabmapPublishLastSignature()),
	_publishPdf(Parameters::defaultRtabmapPublishPdf()),
	_publishLikelihood(Parameters::defaultRtabmapPublishLikelihood()),
	_publishRAMUsage(Parameters::defaultRtabmapPublishRAMUsage()),
	_computeRMSE(Parameters::defaultRtabmapComputeRMSE()),
	_saveWMState(Parameters::defaultRtabmapSaveWMState()),
	_maxTimeAllowed(Parameters::defaultRtabmapTimeThr()), // 700 ms
	_maxMemoryAllowed(Parameters::defaultRtabmapMemoryThr()), // 0=inf
	_loopThr(Parameters::defaultRtabmapLoopThr()),
	_loopRatio(Parameters::defaultRtabmapLoopRatio()),
	_maxLoopClosureDistance(Parameters::defaultRGBDMaxLoopClosureDistance()),
	_verifyLoopClosureHypothesis(Parameters::defaultVhEpEnabled()),
	_maxRetrieved(Parameters::defaultRtabmapMaxRetrieved()),
	_maxLocalRetrieved(Parameters::defaultRGBDMaxLocalRetrieved()),
	_rawDataKept(Parameters::defaultMemImageKept()),
	_statisticLogsBufferedInRAM(Parameters::defaultRtabmapStatisticLogsBufferedInRAM()),
	_statisticLogged(Parameters::defaultRtabmapStatisticLogged()),
	_statisticLoggedHeaders(Parameters::defaultRtabmapStatisticLoggedHeaders()),
	_rgbdSlamMode(Parameters::defaultRGBDEnabled()),
	_rgbdLinearUpdate(Parameters::defaultRGBDLinearUpdate()),
	_rgbdAngularUpdate(Parameters::defaultRGBDAngularUpdate()),
	_rgbdLinearSpeedUpdate(Parameters::defaultRGBDLinearSpeedUpdate()),
	_rgbdAngularSpeedUpdate(Parameters::defaultRGBDAngularSpeedUpdate()),
	_newMapOdomChangeDistance(Parameters::defaultRGBDNewMapOdomChangeDistance()),
	_neighborLinkRefining(Parameters::defaultRGBDNeighborLinkRefining()),
	_proximityByTime(Parameters::defaultRGBDProximityByTime()),
	_proximityBySpace(Parameters::defaultRGBDProximityBySpace()),
	_scanMatchingIdsSavedInLinks(Parameters::defaultRGBDScanMatchingIdsSavedInLinks()),
	_loopClosureIdentityGuess(Parameters::defaultRGBDLoopClosureIdentityGuess()),
	_localRadius(Parameters::defaultRGBDLocalRadius()),
	_localImmunizationRatio(Parameters::defaultRGBDLocalImmunizationRatio()),
	_proximityMaxGraphDepth(Parameters::defaultRGBDProximityMaxGraphDepth()),
	_proximityMaxPaths(Parameters::defaultRGBDProximityMaxPaths()),
	_proximityMaxNeighbors(Parameters::defaultRGBDProximityPathMaxNeighbors()),
	_proximityFilteringRadius(Parameters::defaultRGBDProximityPathFilteringRadius()),
	_proximityRawPosesUsed(Parameters::defaultRGBDProximityPathRawPosesUsed()),
	_proximityAngle(Parameters::defaultRGBDProximityAngle()*M_PI/180.0f),
	_proximityOdomGuess(Parameters::defaultRGBDProximityOdomGuess()),
	_proximityMergedScanCovFactor(Parameters::defaultRGBDProximityMergedScanCovFactor()),
	_databasePath(""),
	_optimizeFromGraphEnd(Parameters::defaultRGBDOptimizeFromGraphEnd()),
	_optimizationMaxError(Parameters::defaultRGBDOptimizeMaxError()),
	_startNewMapOnLoopClosure(Parameters::defaultRtabmapStartNewMapOnLoopClosure()),
	_startNewMapOnGoodSignature(Parameters::defaultRtabmapStartNewMapOnGoodSignature()),
	_goalReachedRadius(Parameters::defaultRGBDGoalReachedRadius()),
	_goalsSavedInUserData(Parameters::defaultRGBDGoalsSavedInUserData()),
	_pathStuckIterations(Parameters::defaultRGBDPlanStuckIterations()),
	_pathLinearVelocity(Parameters::defaultRGBDPlanLinearVelocity()),
	_pathAngularVelocity(Parameters::defaultRGBDPlanAngularVelocity()),
	_restartAtOrigin(Parameters::defaultRGBDStartAtOrigin()),
	_loopCovLimited(Parameters::defaultRGBDLoopCovLimited()),
	_loopGPS(Parameters::defaultRtabmapLoopGPS()),
	_maxOdomCacheSize(Parameters::defaultRGBDMaxOdomCacheSize()),
	_createGlobalScanMap(Parameters::defaultRGBDProximityGlobalScanMap()),
	_loopClosureHypothesis(0,0.0f),
	_highestHypothesis(0,0.0f),
	_lastProcessTime(0.0),
	_someNodesHaveBeenTransferred(false),
	_distanceTravelled(0.0f),
	_distanceTravelledSinceLastLocalization(0.0f),
	_optimizeFromGraphEndChanged(false),
	_epipolarGeometry(0),
	_bayesFilter(0),
	_graphOptimizer(0),
	_memory(0),
	_foutFloat(0),
	_foutInt(0),
	_wDir(""),
	_mapCorrection(Transform::getIdentity()),
	_lastLocalizationNodeId(0),
	_currentSessionHasGPS(false),
	_odomCorrectionAcc(6,0),
	_pathStatus(0),
	_pathCurrentIndex(0),
	_pathGoalIndex(0),
	_pathTransformToGoal(Transform::getIdentity()),
	_pathStuckCount(0),
	_pathStuckDistance(0.0f)
{
}

Rtabmap::~Rtabmap() {
	UDEBUG("");
	this->close();
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

	if(_statisticLogged && !_wDir.empty())
	{
		std::string attributes = "a+"; // append to log files
		if(overwrite)
		{
			// If a file with the same name already exists
			// its content is erased and the file is treated
			// as a new empty file.
			attributes = "w";
		}

		bool addLogFHeader = overwrite || !UFile::exists(_wDir+"/"+LOG_F);
		bool addLogIHeader = overwrite || !UFile::exists(_wDir+"/"+LOG_I);

	#ifdef _MSC_VER
		fopen_s(&_foutFloat, (_wDir+"/"+LOG_F).c_str(), attributes.c_str());
		fopen_s(&_foutInt, (_wDir+"/"+LOG_I).c_str(), attributes.c_str());
	#else
		_foutFloat = fopen((_wDir+"/"+LOG_F).c_str(), attributes.c_str());
		_foutInt = fopen((_wDir+"/"+LOG_I).c_str(), attributes.c_str());
	#endif
		// add header (column identification)
		if(_statisticLoggedHeaders && addLogFHeader && _foutFloat)
		{
			fprintf(_foutFloat, "Column headers:\n");
			fprintf(_foutFloat, " 1-Total iteration time (s)\n");
			fprintf(_foutFloat, " 2-Memory update time (s)\n");
			fprintf(_foutFloat, " 3-Retrieval time (s)\n");
			fprintf(_foutFloat, " 4-Likelihood time (s)\n");
			fprintf(_foutFloat, " 5-Posterior time (s)\n");
			fprintf(_foutFloat, " 6-Hypothesis selection time (s)\n");
			fprintf(_foutFloat, " 7-Hypothesis validation time (s)\n");
			fprintf(_foutFloat, " 8-Transfer time (s)\n");
			fprintf(_foutFloat, " 9-Statistics creation time (s)\n");
			fprintf(_foutFloat, " 10-Loop closure hypothesis value\n");
			fprintf(_foutFloat, " 11-NAN\n");
			fprintf(_foutFloat, " 12-NAN\n");
			fprintf(_foutFloat, " 13-NAN\n");
			fprintf(_foutFloat, " 14-NAN\n");
			fprintf(_foutFloat, " 15-NAN\n");
			fprintf(_foutFloat, " 16-Virtual place hypothesis\n");
			fprintf(_foutFloat, " 17-Join trash time (s)\n");
			fprintf(_foutFloat, " 18-Weight Update (rehearsal) similarity\n");
			fprintf(_foutFloat, " 19-Empty trash time (s)\n");
			fprintf(_foutFloat, " 20-Retrieval database access time (s)\n");
			fprintf(_foutFloat, " 21-Add loop closure link time (s)\n");
			fprintf(_foutFloat, " 22-Memory cleanup time (s)\n");
			fprintf(_foutFloat, " 23-Scan matching (odometry correction) time (s)\n");
			fprintf(_foutFloat, " 24-Local time loop closure detection time (s)\n");
			fprintf(_foutFloat, " 25-Local space loop closure detection time (s)\n");
			fprintf(_foutFloat, " 26-Map optimization (s)\n");
		}
		if(_statisticLoggedHeaders && addLogIHeader && _foutInt)
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
			fprintf(_foutInt, " 18-Local graph size\n");
			fprintf(_foutInt, " 19-Sensor data id\n");
			fprintf(_foutInt, " 20-Indexed words\n");
			fprintf(_foutInt, " 21-Index memory usage (KB)\n");
		}

		ULOGGER_DEBUG("Log file (int)=%s", (_wDir+"/"+LOG_I).c_str());
		ULOGGER_DEBUG("Log file (float)=%s", (_wDir+"/"+LOG_F).c_str());
	}
	else
	{
		if(_statisticLogged)
		{
			UWARN("Working directory is not set, log disabled!");
		}
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

void Rtabmap::init(const ParametersMap & parameters, const std::string & databasePath, bool loadDatabaseParameters)
{
	UDEBUG("path=%s", databasePath.c_str());
	_databasePath = databasePath;
	if(!_databasePath.empty())
	{
		UASSERT(UFile::getExtension(_databasePath).compare("db") == 0);
		UINFO("Using database \"%s\".", _databasePath.c_str());
	}
	else
	{
		UWARN("Using empty database. Mapping session will not be saved unless it is closed with an output database path.");
	}

	bool newDatabase = _databasePath.empty() || !UFile::exists(_databasePath);

	ParametersMap allParameters;
	if(!newDatabase && loadDatabaseParameters)
	{
		DBDriver * driver = DBDriver::create();
		if(driver->openConnection(_databasePath, false))
		{
			allParameters = driver->getLastParameters();
			// ignore working directory (we may be on a different computer)
			allParameters.erase(Parameters::kRtabmapWorkingDirectory());
		}
		delete driver;
	}

	uInsert(allParameters, parameters);
	ParametersMap::const_iterator iter;
	if((iter=allParameters.find(Parameters::kRtabmapWorkingDirectory())) != allParameters.end())
	{
		this->setWorkingDirectory(iter->second.c_str());
	}

	// If doesn't exist, create a memory
	if(!_memory)
	{
		_memory = new Memory(allParameters);
		_memory->init(_databasePath, false, allParameters, true);
	}

	_optimizedPoses.clear();
	_constraints.clear();
	_globalScanMap.clear();
	_globalScanMapPoses.clear();
	_odomCachePoses.clear();
	_odomCacheConstraints.clear();

	// Parse all parameters
	this->parseParameters(allParameters);

	Transform lastPose;
	_optimizedPoses = _memory->loadOptimizedPoses(&lastPose);
	if(!_memory->isIncremental())
	{
		if(_optimizedPoses.empty() &&
			_memory->getWorkingMem().size()>1 &&
			_memory->getWorkingMem().lower_bound(1)!=_memory->getWorkingMem().end())
		{
			cv::Mat cov;
			this->optimizeCurrentMap(
					!_optimizeFromGraphEnd?_memory->getWorkingMem().lower_bound(1)->first:_memory->getWorkingMem().rbegin()->first,
					false, _optimizedPoses, cov, &_constraints);
		}
		if(!_optimizedPoses.empty())
		{
			if(_restartAtOrigin)
			{
				UWARN("last localization pose is ignored (%s=true), assuming we start at the origin of the map.", Parameters::kRGBDStartAtOrigin().c_str());
				lastPose = _optimizedPoses.begin()->second;
			}
			_lastLocalizationPose = lastPose;

			UINFO("Loaded optimizedPoses=%d firstPose %d=%s lastLocalizationPose=%s",
					_optimizedPoses.size(),
					_optimizedPoses.begin()->first,
					_optimizedPoses.begin()->second.prettyPrint().c_str(),
					_lastLocalizationPose.prettyPrint().c_str());

			if(_constraints.empty())
			{
				std::map<int, Transform> tmp;
				// Get just the links
				_memory->getMetricConstraints(uKeysSet(_optimizedPoses), tmp, _constraints, false, true);
			}

			// Initialize Bayes' prediction matrix
			UTimer time;
			std::map<int, float> likelihood;
			likelihood.insert(std::make_pair(Memory::kIdVirtual, 1));
			for(std::map<int, Transform>::iterator iter=_optimizedPoses.begin(); iter!=_optimizedPoses.end(); ++iter)
			{
				if(_memory->getSignature(iter->first))
				{
					likelihood.insert(std::make_pair(iter->first, 0));
				}
			}
			_bayesFilter->computePosterior(_memory, likelihood);
			UINFO("Time initializing Bayes' prediction with %ld nodes: %fs", _optimizedPoses.size(), time.ticks());

			if(_createGlobalScanMap)
				createGlobalScanMap();
		}
		else
		{
			UINFO("Loaded optimizedPoses=0, last localization pose is ignored!");
		}
	}
	else
	{
		_lastLocalizationPose = lastPose;
		if(!_optimizedPoses.empty())
		{
			std::map<int, Transform> tmp;
			// Get just the links
			_memory->getMetricConstraints(uKeysSet(_optimizedPoses), tmp, _constraints, false, true);
		}
	}

	if(_databasePath.empty())
	{
		_statisticLogged = false;
	}
	setupLogFiles(newDatabase);
}

void Rtabmap::init(const std::string & configFile, const std::string & databasePath, bool loadDatabaseParameters)
{
	// fill ctrl struct with values from the configuration file
	ParametersMap param;// = Parameters::defaultParameters;

	if(!configFile.empty())
	{
		ULOGGER_DEBUG("Read parameters from = %s", configFile.c_str());
		Parameters::readINI(configFile, param);
	}

	this->init(param, databasePath, loadDatabaseParameters);
}

void Rtabmap::close(bool databaseSaved, const std::string & ouputDatabasePath)
{
	UINFO("databaseSaved=%d", databaseSaved?1:0);
	_highestHypothesis = std::make_pair(0,0.0f);
	_loopClosureHypothesis = std::make_pair(0,0.0f);
	_lastProcessTime = 0.0;
	_someNodesHaveBeenTransferred = false;
	_constraints.clear();
	_mapCorrection.setIdentity();
	_mapCorrectionBackup.setNull();

	_lastLocalizationNodeId = 0;
	_odomCachePoses.clear();
	_odomCacheConstraints.clear();
	_odomCorrectionAcc = std::vector<float>(6,0);
	_distanceTravelled = 0.0f;
	_distanceTravelledSinceLastLocalization = 0.0f;
	_optimizeFromGraphEndChanged = false;
	this->clearPath(0);
	_gpsGeocentricCache.clear();
	_currentSessionHasGPS = false;

	_globalScanMap.clear();
	_globalScanMapPoses.clear();

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
		if(databaseSaved)
		{
			if(_memory->isGraphReduced() && _memory->isIncremental())
			{
				// Force reducing graph, then remove filtered nodes from the optimized poses
				std::map<int, int> reducedIds;
				_memory->incrementMapId(&reducedIds);
				for(std::map<int, int>::iterator iter=reducedIds.begin(); iter!=reducedIds.end(); ++iter)
				{
					_optimizedPoses.erase(iter->first);
				}
			}
			_memory->saveOptimizedPoses(_optimizedPoses, _lastLocalizationPose);
		}
		_memory->close(databaseSaved, true, ouputDatabasePath);
		delete _memory;
		_memory = 0;
	}
	_optimizedPoses.clear();
	_lastLocalizationPose.setNull();

	if(_bayesFilter)
	{
		delete _bayesFilter;
		_bayesFilter = 0;
	}
	if(_graphOptimizer)
	{
		delete _graphOptimizer;
		_graphOptimizer = 0;
	}
	_databasePath.clear();
	parseParameters(Parameters::getDefaultParameters()); // reset to default parameters
	_parameters.clear();
}

void Rtabmap::parseParameters(const ParametersMap & parameters)
{
	uInsert(_parameters, parameters);

	// place this before changing working directory
	Parameters::parse(parameters, Parameters::kRtabmapStatisticLogsBufferedInRAM(), _statisticLogsBufferedInRAM);
	Parameters::parse(parameters, Parameters::kRtabmapStatisticLogged(), _statisticLogged);
	Parameters::parse(parameters, Parameters::kRtabmapStatisticLoggedHeaders(), _statisticLoggedHeaders);

	ULOGGER_DEBUG("");
	ParametersMap::const_iterator iter;
	if((iter=parameters.find(Parameters::kRtabmapWorkingDirectory())) != parameters.end())
	{
		this->setWorkingDirectory(iter->second.c_str());
	}

	Parameters::parse(parameters, Parameters::kRtabmapPublishStats(), _publishStats);
	Parameters::parse(parameters, Parameters::kRtabmapPublishLastSignature(), _publishLastSignatureData);
	Parameters::parse(parameters, Parameters::kRtabmapPublishPdf(), _publishPdf);
	Parameters::parse(parameters, Parameters::kRtabmapPublishLikelihood(), _publishLikelihood);
	Parameters::parse(parameters, Parameters::kRtabmapPublishRAMUsage(), _publishRAMUsage);
	Parameters::parse(parameters, Parameters::kRtabmapComputeRMSE(), _computeRMSE);
	Parameters::parse(parameters, Parameters::kRtabmapSaveWMState(), _saveWMState);
	Parameters::parse(parameters, Parameters::kRtabmapTimeThr(), _maxTimeAllowed);
	Parameters::parse(parameters, Parameters::kRtabmapMemoryThr(), _maxMemoryAllowed);
	Parameters::parse(parameters, Parameters::kRtabmapLoopThr(), _loopThr);
	Parameters::parse(parameters, Parameters::kRtabmapLoopRatio(), _loopRatio);
	Parameters::parse(parameters, Parameters::kRGBDMaxLoopClosureDistance(), _maxLoopClosureDistance);
	Parameters::parse(parameters, Parameters::kVhEpEnabled(), _verifyLoopClosureHypothesis);
	Parameters::parse(parameters, Parameters::kRtabmapMaxRetrieved(), _maxRetrieved);
	Parameters::parse(parameters, Parameters::kRGBDMaxLocalRetrieved(), _maxLocalRetrieved);
	Parameters::parse(parameters, Parameters::kMemImageKept(), _rawDataKept);
	Parameters::parse(parameters, Parameters::kRGBDEnabled(), _rgbdSlamMode);
	Parameters::parse(parameters, Parameters::kRGBDLinearUpdate(), _rgbdLinearUpdate);
	Parameters::parse(parameters, Parameters::kRGBDAngularUpdate(), _rgbdAngularUpdate);
	Parameters::parse(parameters, Parameters::kRGBDLinearSpeedUpdate(), _rgbdLinearSpeedUpdate);
	Parameters::parse(parameters, Parameters::kRGBDAngularSpeedUpdate(), _rgbdAngularSpeedUpdate);
	Parameters::parse(parameters, Parameters::kRGBDNewMapOdomChangeDistance(), _newMapOdomChangeDistance);
	Parameters::parse(parameters, Parameters::kRGBDNeighborLinkRefining(), _neighborLinkRefining);
	Parameters::parse(parameters, Parameters::kRGBDProximityByTime(), _proximityByTime);
	Parameters::parse(parameters, Parameters::kRGBDProximityBySpace(), _proximityBySpace);
	Parameters::parse(parameters, Parameters::kRGBDScanMatchingIdsSavedInLinks(), _scanMatchingIdsSavedInLinks);
	Parameters::parse(parameters, Parameters::kRGBDLoopClosureIdentityGuess(), _loopClosureIdentityGuess);
	Parameters::parse(parameters, Parameters::kRGBDLocalRadius(), _localRadius);
	Parameters::parse(parameters, Parameters::kRGBDLocalImmunizationRatio(), _localImmunizationRatio);
	Parameters::parse(parameters, Parameters::kRGBDProximityMaxGraphDepth(), _proximityMaxGraphDepth);
	Parameters::parse(parameters, Parameters::kRGBDProximityMaxPaths(), _proximityMaxPaths);
	Parameters::parse(parameters, Parameters::kRGBDProximityPathMaxNeighbors(), _proximityMaxNeighbors);
	Parameters::parse(parameters, Parameters::kRGBDProximityPathFilteringRadius(), _proximityFilteringRadius);
	Parameters::parse(parameters, Parameters::kRGBDProximityPathRawPosesUsed(), _proximityRawPosesUsed);
	if(Parameters::parse(parameters, Parameters::kRGBDProximityAngle(), _proximityAngle))
	{
		_proximityAngle *= M_PI/180.0f;
	}
	Parameters::parse(parameters, Parameters::kRGBDProximityOdomGuess(), _proximityOdomGuess);
	Parameters::parse(parameters, Parameters::kRGBDProximityMergedScanCovFactor(), _proximityMergedScanCovFactor);
	UASSERT(_proximityMergedScanCovFactor>0.0);

	bool optimizeFromGraphEndPrevious = _optimizeFromGraphEnd;
	Parameters::parse(parameters, Parameters::kRGBDOptimizeFromGraphEnd(), _optimizeFromGraphEnd);
	if(optimizeFromGraphEndPrevious != _optimizeFromGraphEnd && !_optimizedPoses.empty())
	{
		_optimizeFromGraphEndChanged = true;
	}
	Parameters::parse(parameters, Parameters::kRGBDOptimizeMaxError(), _optimizationMaxError);
	if(_optimizationMaxError > 0.0 && _optimizationMaxError < 1.0)
	{
		UWARN("RGBD/OptimizeMaxError (value=%f) is smaller than 1.0, setting to default %f "
			  "instead (for backward compatibility issues when this parameter was previously "
			  "an absolute error value).", _optimizationMaxError, Parameters::defaultRGBDOptimizeMaxError());
		_optimizationMaxError = Parameters::defaultRGBDOptimizeMaxError();
	}
	Parameters::parse(parameters, Parameters::kRtabmapStartNewMapOnLoopClosure(), _startNewMapOnLoopClosure);
	Parameters::parse(parameters, Parameters::kRtabmapStartNewMapOnGoodSignature(), _startNewMapOnGoodSignature);
	Parameters::parse(parameters, Parameters::kRGBDGoalReachedRadius(), _goalReachedRadius);
	Parameters::parse(parameters, Parameters::kRGBDGoalsSavedInUserData(), _goalsSavedInUserData);
	Parameters::parse(parameters, Parameters::kRGBDPlanStuckIterations(), _pathStuckIterations);
	Parameters::parse(parameters, Parameters::kRGBDPlanLinearVelocity(), _pathLinearVelocity);
	Parameters::parse(parameters, Parameters::kRGBDPlanAngularVelocity(), _pathAngularVelocity);
	Parameters::parse(parameters, Parameters::kRGBDStartAtOrigin(), _restartAtOrigin);
	Parameters::parse(parameters, Parameters::kRGBDLoopCovLimited(), _loopCovLimited);
	Parameters::parse(parameters, Parameters::kRtabmapLoopGPS(), _loopGPS);
	Parameters::parse(parameters, Parameters::kRGBDMaxOdomCacheSize(), _maxOdomCacheSize);
	Parameters::parse(parameters, Parameters::kRGBDProximityGlobalScanMap(), _createGlobalScanMap);

	UASSERT(_rgbdLinearUpdate >= 0.0f);
	UASSERT(_rgbdAngularUpdate >= 0.0f);
	UASSERT(_rgbdLinearSpeedUpdate >= 0.0f);
	UASSERT(_rgbdAngularSpeedUpdate >= 0.0f);
	UASSERT(_maxOdomCacheSize >= 0);

	// By default, we create our strategies if they are not already created.
	// If they already exists, we check the parameters if a change is requested

	// Graph optimizer
	Optimizer::Type optimizerType = Optimizer::kTypeUndef;
	if((iter=parameters.find(Parameters::kOptimizerStrategy())) != parameters.end())
	{
		optimizerType = (Optimizer::Type)std::atoi((*iter).second.c_str());
	}
	if(optimizerType!=Optimizer::kTypeUndef)
	{
		UDEBUG("new detector strategy %d", int(optimizerType));
		if(_graphOptimizer)
		{
			delete _graphOptimizer;
			_graphOptimizer = 0;
		}

		_graphOptimizer = Optimizer::create(optimizerType, _parameters);
	}
	else if(_graphOptimizer)
	{
		_graphOptimizer->parseParameters(parameters);
	}
	else
	{
		optimizerType = (Optimizer::Type)Parameters::defaultOptimizerStrategy();
		_graphOptimizer = Optimizer::create(optimizerType, parameters);
	}

	if(!_createGlobalScanMap)
	{
		_globalScanMap.clear();
		_globalScanMapPoses.clear();
	}

	if(_memory)
	{
		_memory->parseParameters(parameters);
		if(_memory->isIncremental() && !_globalScanMap.empty())
		{
			UWARN("Map is now incremental, clearing global scan map...");
			_globalScanMap.clear();
			_globalScanMapPoses.clear();
		}

		if(_createGlobalScanMap && !_memory->isIncremental() && _globalScanMap.empty() && !_optimizedPoses.empty())
		{
			this->createGlobalScanMap();
		}

		if(_memory->isIncremental())
		{
			_odomCachePoses.clear();
			_odomCacheConstraints.clear();
		}
	}

	if(!_epipolarGeometry)
	{
		_epipolarGeometry = new EpipolarGeometry(_parameters);
	}
	else
	{
		_epipolarGeometry->parseParameters(parameters);
	}

	// Bayes filter, create one if not exists
	if(!_bayesFilter)
	{
		_bayesFilter = new BayesFilter(_parameters);
	}
	else
	{
		_bayesFilter->parseParameters(parameters);
	}
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
		mem = uKeysList(_memory->getWorkingMem());
		mem.remove(-1);// Ignore the virtual signature (if here)
	}
	return mem;
}

int Rtabmap::getWMSize() const
{
	if(_memory)
	{
		return (int)_memory->getWorkingMem().size()-1; // remove virtual place
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
		return (int)_memory->getStMem().size();
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

Transform Rtabmap::getPose(int locationId) const
{
	return uValue(_optimizedPoses, locationId, Transform());
}

void Rtabmap::setInitialPose(const Transform & initialPose)
{
	if(_memory)
	{
		if(!_memory->isIncremental())
		{
			_lastLocalizationPose = initialPose;
			_lastLocalizationNodeId = 0;
			_odomCachePoses.clear();
			_odomCacheConstraints.clear();
			_odomCorrectionAcc = std::vector<float>(6,0);
			_mapCorrection.setIdentity();
			_mapCorrectionBackup.setNull();

			if(_memory->getLastWorkingSignature()->id() &&
				_optimizedPoses.empty())
			{
				cv::Mat covariance;
				this->optimizeCurrentMap(_memory->getLastWorkingSignature()->id(), false, _optimizedPoses, covariance, &_constraints);
			}
		}
		else
		{
			UWARN("Initial pose can only be set in localization mode (%s=false), ignoring it...", Parameters::kMemIncrementalMemory().c_str());
		}
	}
}

int Rtabmap::triggerNewMap()
{
	int mapId = -1;
	if(_memory)
	{
		_lastLocalizationNodeId = 0;
		_odomCachePoses.clear();
		_odomCacheConstraints.clear();
		_odomCorrectionAcc = std::vector<float>(6,0);
		_distanceTravelled = 0.0f;
		_distanceTravelledSinceLastLocalization = 0.0f;

		if(!_memory->isIncremental())
		{
			_mapCorrection.setIdentity();
			if(_restartAtOrigin)
			{
				_lastLocalizationPose.setIdentity();
			}
			return mapId;
		}
		std::map<int, int> reducedIds;
		mapId = _memory->incrementMapId(&reducedIds);
		UINFO("New map triggered, new map = %d", mapId);
		_optimizedPoses.clear();
		_constraints.clear();

		if(_bayesFilter)
		{
			_bayesFilter->reset();
		}

		//Verify if there are nodes that were merged through graph reduction
		if(reducedIds.size() && _path.size())
		{
			for(unsigned int i=0; i<_path.size(); ++i)
			{
				std::map<int, int>::const_iterator iter = reducedIds.find(_path[i].first);
				if(iter!= reducedIds.end())
				{
					// change path ID to loop closure ID
					_path[i].first = iter->second;
				}
			}
		}
	}
	return mapId;
}

bool Rtabmap::labelLocation(int id, const std::string & label)
{
	if(_memory)
	{
		if(id > 0)
		{
			return _memory->labelSignature(id, label);
		}
		else if(_memory->isIncremental() && _memory->getLastWorkingSignature())
		{
			return _memory->labelSignature(_memory->getLastWorkingSignature()->id(), label);
		}
		else if(!_memory->isIncremental() && !_lastLocalizationPose.isNull() && !_lastLocalizationPose.isIdentity())
		{
			std::map<int, Transform> nearestNodes = getNodesInRadius(_lastLocalizationPose, _localRadius, 1);
			if(!nearestNodes.empty())
			{
				return _memory->labelSignature(nearestNodes.begin()->first, label);
			}
			else
			{
				UERROR("No nodes found inside %s=%fm of the current pose (%s). Cannot set label \"%s\"",
						Parameters::kRGBDLocalRadius().c_str(),
						_localRadius,
						_lastLocalizationPose.prettyPrint().c_str(),
						label.c_str());
			}
		}
		else
		{
			UERROR("Last signature is null! Cannot set label \"%s\"", label.c_str());
		}
	}
	return false;
}

bool Rtabmap::setUserData(int id, const cv::Mat & data)
{
	if(_memory)
	{
		if(id > 0)
		{
			return _memory->setUserData(id, data);
		}
		else if(_memory->getLastWorkingSignature())
		{
			return _memory->setUserData(_memory->getLastWorkingSignature()->id(), data);
		}
		else
		{
			UERROR("Last signature is null! Cannot set user data!");
		}
	}
	return false;
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

void Rtabmap::exportPoses(const std::string & path, bool optimized, bool global, int format)
{
	if(_memory && _memory->getLastWorkingSignature())
	{
		std::map<int, Transform> poses;
		std::multimap<int, Link> constraints;

		if(optimized)
		{
			cv::Mat covariance;
			this->optimizeCurrentMap(_memory->getLastWorkingSignature()->id(), global, poses, covariance, &constraints);
		}
		else
		{
			std::map<int, int> ids = _memory->getNeighborsId(_memory->getLastWorkingSignature()->id(), 0, global?-1:0, true);
			_memory->getMetricConstraints(uKeysSet(ids), poses, constraints, global);
		}

		std::map<int, double> stamps;
		if(format == 1)
		{
			for(std::map<int, Transform>::iterator iter=poses.begin(); iter!=poses.end(); ++iter)
			{
				Transform o,g;
				int m, w;
				std::string l;
				double stamp = 0.0;
				std::vector<float> v;
				GPS gps;
				EnvSensors sensors;
				_memory->getNodeInfo(iter->first, o, m, w, l, stamp, g, v, gps, sensors, true);
				stamps.insert(std::make_pair(iter->first, stamp));
			}
		}

		graph::exportPoses(path, format, poses, constraints, stamps, _parameters);
	}
}

void Rtabmap::resetMemory()
{
	UDEBUG("");
	_highestHypothesis = std::make_pair(0,0.0f);
	_loopClosureHypothesis = std::make_pair(0,0.0f);
	_lastProcessTime = 0.0;
	_someNodesHaveBeenTransferred = false;
	_optimizedPoses.clear();
	_constraints.clear();
	_mapCorrection.setIdentity();
	_mapCorrectionBackup.setNull();
	_lastLocalizationPose.setNull();
	_lastLocalizationNodeId = 0;
	_odomCachePoses.clear();
	_odomCacheConstraints.clear();
	_odomCorrectionAcc = std::vector<float>(6,0);
	_distanceTravelled = 0.0f;
	_distanceTravelledSinceLastLocalization = 0.0f;
	_optimizeFromGraphEndChanged = false;
	_globalScanMap.clear();
	_globalScanMapPoses.clear();
	this->clearPath(0);

	if(_memory)
	{
		_memory->init(_databasePath, true, _parameters, true);
		if(_memory->getLastWorkingSignature())
		{
			cv::Mat covariance;
			optimizeCurrentMap(_memory->getLastWorkingSignature()->id(), false, _optimizedPoses, covariance, &_constraints);
		}
		if(_bayesFilter)
		{
			_bayesFilter->reset();
		}
	}
	else
	{
		UERROR("RTAB-Map is not initialized. No memory to reset...");
	}
	this->setupLogFiles(true);
}

class NearestPathKey
{
public:
	NearestPathKey(float l, int i) :
		likelihood(l),
		id(i){}
	bool operator<(const NearestPathKey & k) const
	{
		if(likelihood < k.likelihood)
		{
			return true;
		}
		else if(likelihood == k.likelihood && id < k.id)
		{
			return true;
		}
		return false;
	}
	float likelihood;
	int id;
};

//============================================================
// MAIN LOOP
//============================================================
bool Rtabmap::process(
		const cv::Mat & image,
		int id,
		const std::map<std::string, float> & externalStats)
{
	return this->process(SensorData(image, id), Transform());
}
bool Rtabmap::process(
			const SensorData & data,
			Transform odomPose,
			float odomLinearVariance,
			float odomAngularVariance,
			const std::vector<float> & odomVelocity,
			const std::map<std::string, float> & externalStats)
{
	if(!odomPose.isNull())
	{
		UASSERT(odomLinearVariance>0.0f);
		UASSERT(odomAngularVariance>0.0f);
	}
	cv::Mat covariance = cv::Mat::eye(6,6,CV_64FC1);
	covariance.at<double>(0,0) = odomLinearVariance;
	covariance.at<double>(1,1) = odomLinearVariance;
	covariance.at<double>(2,2) = odomLinearVariance;
	covariance.at<double>(3,3) = odomAngularVariance;
	covariance.at<double>(4,4) = odomAngularVariance;
	covariance.at<double>(5,5) = odomAngularVariance;
	return process(data, odomPose, covariance, odomVelocity, externalStats);
}
bool Rtabmap::process(
		const SensorData & data,
		Transform odomPose,
		const cv::Mat & odomCovariance,
		const std::vector<float> & odomVelocity,
		const std::map<std::string, float> & externalStats)
{
	UDEBUG("");

	//============================================================
	// Initialization
	//============================================================
	UTimer timer;
	UTimer timerTotal;
	double timeMemoryUpdate = 0;
	double timeNeighborLinkRefining = 0;
	double timeProximityByTimeDetection = 0;
	double timeProximityBySpaceVisualDetection = 0;
	double timeProximityBySpaceDetection = 0;
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
	double timeFinalizingStatistics = 0;
	double timeJoiningTrash = 0;
	double timeStatsCreation = 0;

	float hypothesisRatio = 0.0f; // Only used for statistics
	bool rejectedGlobalLoopClosure = false;

	std::map<int, float> rawLikelihood;
	std::map<int, float> adjustedLikelihood;
	std::map<int, float> likelihood;
	std::map<int, int> weights;
	std::map<int, float> posterior;
	std::list<std::pair<int, float> > reactivateHypotheses;

	std::map<int, int> childCount;
	std::set<int> signaturesRetrieved;
	int proximityDetectionsInTimeFound = 0;

	const Signature * signature = 0;
	const Signature * sLoop = 0;

	_loopClosureHypothesis = std::make_pair(0,0.0f);
	std::pair<int, float> lastHighestHypothesis = _highestHypothesis;
	_highestHypothesis = std::make_pair(0,0.0f);

	std::set<int> immunizedLocations;

	statistics_ = Statistics(); // reset
	for(std::map<std::string, float>::const_iterator iter=externalStats.begin(); iter!=externalStats.end(); ++iter)
	{
		statistics_.addStatistic(iter->first, iter->second);
	}

	//============================================================
	// Wait for an image...
	//============================================================
	ULOGGER_INFO("getting data...");

	timer.start();
	timerTotal.start();

	UASSERT_MSG(_memory, "RTAB-Map is not initialized!");
	UASSERT_MSG(_bayesFilter, "RTAB-Map is not initialized!");
	UASSERT_MSG(_graphOptimizer, "RTAB-Map is not initialized!");

	//============================================================
	// If RGBD SLAM is enabled, a pose must be set.
	//============================================================
	bool fakeOdom = false;
	if(_rgbdSlamMode)
	{
		if(!odomPose.isNull())
		{
			// this will make sure that all inverse operations will work!
			if(!odomPose.isInvertible())
			{
				UWARN("Input odometry is not invertible! pose = %s\n"
						"[%f %f %f %f;\n"
						" %f %f %f %f;\n"
						" %f %f %f %f;\n"
						" 0 0 0 1]\n"
						"Trying to normalize rotation to see if it makes it invertible...",
						odomPose.prettyPrint().c_str(),
						odomPose.r11(), odomPose.r12(), odomPose.r13(), odomPose.o14(),
						odomPose.r21(), odomPose.r22(), odomPose.r23(), odomPose.o24(),
						odomPose.r31(), odomPose.r32(), odomPose.r33(), odomPose.o34());
				odomPose.normalizeRotation();
				UASSERT_MSG(odomPose.isInvertible(), uFormat("Odometry pose is not invertible!\n"
						"[%f %f %f %f;\n"
						" %f %f %f %f;\n"
						" %f %f %f %f;\n"
						" 0 0 0 1]", odomPose.prettyPrint().c_str(),
						odomPose.r11(), odomPose.r12(), odomPose.r13(), odomPose.o14(),
						odomPose.r21(), odomPose.r22(), odomPose.r23(), odomPose.o24(),
						odomPose.r31(), odomPose.r32(), odomPose.r33(), odomPose.o34()).c_str());
				UWARN("Normalizing rotation succeeded! fixed pose = %s\n"
						"[%f %f %f %f;\n"
						" %f %f %f %f;\n"
						" %f %f %f %f;\n"
						" 0 0 0 1]\n"
						"If the resulting rotation is very different from original one, try to fix the odometry or TF.",
						odomPose.prettyPrint().c_str(),
						odomPose.r11(), odomPose.r12(), odomPose.r13(), odomPose.o14(),
						odomPose.r21(), odomPose.r22(), odomPose.r23(), odomPose.o24(),
						odomPose.r31(), odomPose.r32(), odomPose.r33(), odomPose.o34());
			}
		}

		UDEBUG("incremental=%d odomPose=%s optimizedPoses=%d mapCorrection=%s lastLocalizationPose=%s lastLocalizationNodeId=%d",
				_memory->isIncremental()?1:0,
				odomPose.prettyPrint().c_str(),
				(int)_optimizedPoses.size(),
				_mapCorrection.prettyPrint().c_str(),
				_lastLocalizationPose.prettyPrint().c_str(),
				_lastLocalizationNodeId);

		if(!_memory->isIncremental() &&
			!odomPose.isNull() &&
			_optimizedPoses.size() &&
			_mapCorrection.isIdentity() &&
			!_lastLocalizationPose.isNull() &&
			_lastLocalizationNodeId == 0)
		{
			// Localization mode
			if(!_optimizeFromGraphEnd)
			{
				//set map->odom so that odom is moved back to last saved localization
				if(_graphOptimizer->isSlam2d())
				{
					_mapCorrection = _lastLocalizationPose.to3DoF() * odomPose.to3DoF().inverse();
				}
				else if((!data.imu().empty() || _memory->isOdomGravityUsed()) && _graphOptimizer->gravitySigma()>0.0f)
				{
					_mapCorrection = _lastLocalizationPose.to4DoF() * odomPose.to4DoF().inverse();
				}
				else
				{
					_mapCorrection = _lastLocalizationPose * odomPose.inverse();
				}
				std::map<int, Transform> nodesOnly(_optimizedPoses.lower_bound(1), _optimizedPoses.end());
				_lastLocalizationNodeId = graph::findNearestNode(nodesOnly, _lastLocalizationPose);
				UWARN("Update map correction based on last localization saved in database! correction = %s, nearest id = %d of last pose = %s, odom = %s",
						_mapCorrection.prettyPrint().c_str(),
						_lastLocalizationNodeId,
						_lastLocalizationPose.prettyPrint().c_str(),
						odomPose.prettyPrint().c_str());
			}
			else
			{
				//move optimized poses accordingly to last saved localization
				Transform mapCorrectionInv;
				if(_graphOptimizer->isSlam2d())
				{
					mapCorrectionInv = odomPose.to3DoF() * _lastLocalizationPose.to3DoF().inverse();
				}
				else if((!data.imu().empty() || _memory->isOdomGravityUsed()) && _graphOptimizer->gravitySigma()>0.0f)
				{
					mapCorrectionInv = odomPose.to4DoF() * _lastLocalizationPose.to4DoF().inverse();
				}
				else
				{
					mapCorrectionInv = odomPose * _lastLocalizationPose.inverse();
				}
				for(std::map<int, Transform>::iterator iter=_optimizedPoses.begin(); iter!=_optimizedPoses.end(); ++iter)
				{
					iter->second = mapCorrectionInv * iter->second;
				}
			}
		}

		if(odomPose.isNull())
		{
			if(_memory->isIncremental())
			{
				UERROR("RGB-D SLAM mode is enabled, memory is incremental but no odometry is provided. "
					   "Image %d is ignored!", data.id());
				return false;
			}
			else // fake localization
			{
				if(!_mapCorrectionBackup.isNull())
				{
					_mapCorrection = _mapCorrectionBackup;
					_mapCorrectionBackup.setNull();
				}
				if(_lastLocalizationPose.isNull())
				{
					_lastLocalizationPose = Transform::getIdentity();
				}
				fakeOdom = true;
				odomPose = _mapCorrection.inverse() * _lastLocalizationPose;
				UDEBUG("Map correction = %s", _mapCorrection.prettyPrint().c_str());
				UDEBUG("Last localization pose: %s", _lastLocalizationPose.prettyPrint().c_str());
				UDEBUG("Fake odom: %s", odomPose.prettyPrint().c_str());
			}
		}
		else if(_memory->isIncremental()) // only in mapping mode
		{
			// Detect if the odometry is reset. If yes, trigger a new map.
			if(_memory->getLastWorkingSignature())
			{
				const Transform & lastPose = _memory->getLastWorkingSignature()->getPose(); // use raw odometry

				// look for identity
				if(!lastPose.isIdentity() && odomPose.isIdentity())
				{
					int mapId = triggerNewMap();
					UWARN("Odometry is reset (identity pose detected). Increment map id to %d!", mapId);
				}
				else if(_newMapOdomChangeDistance > 0.0)
				{
					// look for large change
					Transform lastPoseToNewPose = lastPose.inverse() * odomPose;
					float x,y,z, roll,pitch,yaw;
					lastPoseToNewPose.getTranslationAndEulerAngles(x,y,z, roll,pitch,yaw);
					if((x*x + y*y + z*z) > _newMapOdomChangeDistance*_newMapOdomChangeDistance)
					{
						int mapId = triggerNewMap();
						UWARN("Odometry is reset (large odometry change detected > %f). A new map (%d) is created! Last pose = %s, new pose = %s",
								_newMapOdomChangeDistance,
								mapId,
								lastPose.prettyPrint().c_str(),
								odomPose.prettyPrint().c_str());
					}
				}
			}
		}
	}

	//============================================================
	// Memory Update : Location creation + Add to STM + Weight Update (Rehearsal)
	//============================================================
	ULOGGER_INFO("Updating memory...");
	if(_rgbdSlamMode)
	{
		if(!_memory->update(data, odomPose, odomCovariance, odomVelocity, &statistics_))
		{
			return false;
		}
	}
	else
	{
		if(!_memory->update(data, Transform(), cv::Mat(), std::vector<float>(), &statistics_))
		{
			return false;
		}
	}

	signature = _memory->getLastWorkingSignature();
	_currentSessionHasGPS = _currentSessionHasGPS || signature->sensorData().gps().stamp() > 0.0;
	if(!signature)
	{
		UFATAL("Not supposed to be here...last signature is null?!?");
	}

	ULOGGER_INFO("Processing signature %d w=%d map=%d", signature->id(), signature->getWeight(), signature->mapId());
	timeMemoryUpdate = timer.ticks();
	ULOGGER_INFO("timeMemoryUpdate=%fs", timeMemoryUpdate);

	//============================================================
	// Metric
	//============================================================
	bool smallDisplacement = false;
	bool tooFastMovement = false;
	std::list<int> signaturesRemoved;
	bool neighborLinkRefined = false;
	if(_rgbdSlamMode)
	{
		statistics_.addStatistic(Statistics::kMemoryOdometry_variance_lin(), odomCovariance.empty()?1.0f:(float)odomCovariance.at<double>(0,0));
		statistics_.addStatistic(Statistics::kMemoryOdometry_variance_ang(), odomCovariance.empty()?1.0f:(float)odomCovariance.at<double>(5,5));

		//Verify if there was a rehearsal
		int rehearsedId = (int)uValue(statistics_.data(), Statistics::kMemoryRehearsal_merged(), 0.0f);
		if(rehearsedId > 0)
		{
			_optimizedPoses.erase(rehearsedId);
		}
		else
		{
			if(_rgbdLinearUpdate > 0.0f || _rgbdAngularUpdate > 0.0f)
			{
				//============================================================
				// Minimum displacement required to add to Memory
				//============================================================
				const std::multimap<int, Link> & links = signature->getLinks();
				if(links.size() && links.begin()->second.type() == Link::kNeighbor)
				{
					const Signature * s = _memory->getSignature(links.begin()->second.to());
					UASSERT(s!=0);
					// don't filter if the new node is not intermediate but previous one is
					if(signature->getWeight() < 0 || s->getWeight() >= 0)
					{
						float x,y,z, roll,pitch,yaw;
						links.begin()->second.transform().getTranslationAndEulerAngles(x,y,z, roll,pitch,yaw);
						bool isMoving = fabs(x) > _rgbdLinearUpdate ||
										fabs(y) > _rgbdLinearUpdate ||
										fabs(z) > _rgbdLinearUpdate ||
									    (_rgbdAngularUpdate>0.0f && (
											fabs(roll) > _rgbdAngularUpdate ||
											fabs(pitch) > _rgbdAngularUpdate ||
											fabs(yaw) > _rgbdAngularUpdate));
						if(!isMoving)
						{
							// This will disable global loop closure detection, only retrieval will be done.
							// The location will also be deleted at the end.
							smallDisplacement = true;
							UDEBUG("smallDisplacement: %f %f %f %f %f %f", x,y,z, roll,pitch,yaw);
						}
					}
				}
			}
			if(odomVelocity.size() == 6)
			{
				// This will disable global loop closure detection, only retrieval will be done.
				// The location will also be deleted at the end.
				tooFastMovement =
						(_rgbdLinearSpeedUpdate>0.0f && uMax3(fabs(odomVelocity[0]), fabs(odomVelocity[1]), fabs(odomVelocity[2])) > _rgbdLinearSpeedUpdate) ||
						(_rgbdAngularSpeedUpdate>0.0f && uMax3(fabs(odomVelocity[3]), fabs(odomVelocity[4]), fabs(odomVelocity[5])) > _rgbdAngularSpeedUpdate);
			}
		}

		// Update optimizedPoses with the newly added node
		Transform newPose;
		bool intermediateNodeRefining = false;
		if(_neighborLinkRefining &&
			signature->getLinks().size() &&
			signature->getLinks().begin()->second.type() == Link::kNeighbor &&
		   _memory->isIncremental() && // ignore pose matching in localization mode
		   rehearsedId == 0) // don't do it if rehearsal happened
		{
			int oldId = signature->getLinks().begin()->first;
			const Signature * oldS = _memory->getSignature(oldId);
			UASSERT(oldS != 0);

			if(signature->getWeight() >= 0 && oldS->getWeight()>=0) // ignore intermediate nodes
			{
				Transform guess = signature->getLinks().begin()->second.transform().inverse();

				if(smallDisplacement)
				{
					if(signature->getLinks().begin()->second.transVariance() == 1)
					{
						// set small variance
						UDEBUG("Set small variance. The robot is not moving.");
						_memory->updateLink(Link(oldId, signature->id(), signature->getLinks().begin()->second.type(), guess, cv::Mat::eye(6,6,CV_64FC1)*1000));
					}
				}
				else
				{
					//============================================================
					// Refine neighbor links
					//============================================================
					UINFO("Odometry refining: guess = %s", guess.prettyPrint().c_str());
					RegistrationInfo info;
					Transform t = _memory->computeTransform(oldId, signature->id(), guess, &info);
					if(!t.isNull())
					{
						UINFO("Odometry refining: update neighbor link (%d->%d, variance:lin=%f, ang=%f) from %s to %s",
								oldId,
								signature->id(),
								info.covariance.at<double>(0,0),
								info.covariance.at<double>(5,5),
								guess.prettyPrint().c_str(),
								t.prettyPrint().c_str());
						UASSERT(info.covariance.at<double>(0,0) > 0.0 && info.covariance.at<double>(5,5) > 0.0);
						_memory->updateLink(Link(oldId, signature->id(), signature->getLinks().begin()->second.type(), t, info.covariance.inv()));

						if(_optimizeFromGraphEnd)
						{
							// update all previous nodes
							// Normally _mapCorrection should be identity, but if _optimizeFromGraphEnd
							// parameters just changed state, we should put back all poses without map correction.
							Transform u = guess * t.inverse();
							std::map<int, Transform>::iterator jter = _optimizedPoses.find(oldId);
							UASSERT(jter!=_optimizedPoses.end());
							Transform up = jter->second * u * jter->second.inverse();
							Transform mapCorrectionInv = _mapCorrection.inverse();
							for(std::map<int, Transform>::iterator iter=_optimizedPoses.begin(); iter!=_optimizedPoses.end(); ++iter)
							{
								iter->second = mapCorrectionInv * up * iter->second;
							}
						}
					}
					else
					{
						UINFO("Odometry refining rejected: %s", info.rejectedMsg.c_str());
						if(!info.covariance.empty() && info.covariance.at<double>(0,0) > 0.0 && info.covariance.at<double>(0,0) != 1.0 && info.covariance.at<double>(5,5) > 0.0 && info.covariance.at<double>(5,5) != 1.0)
						{
							if(ULogger::level() <= ULogger::kInfo)
							{
								std::cout << info.covariance << std::endl;
							}
							_memory->updateLink(Link(oldId, signature->id(), signature->getLinks().begin()->second.type(), guess, (info.covariance*100.0).inv()));
						}
					}
					neighborLinkRefined = !t.isNull();
					statistics_.addStatistic(Statistics::kNeighborLinkRefiningAccepted(),neighborLinkRefined?1.0f:0);
					statistics_.addStatistic(Statistics::kNeighborLinkRefiningInliers(), info.inliers);
					statistics_.addStatistic(Statistics::kNeighborLinkRefiningICP_inliers_ratio(), info.icpInliersRatio);
					statistics_.addStatistic(Statistics::kNeighborLinkRefiningICP_rotation(), info.icpRotation);
					statistics_.addStatistic(Statistics::kNeighborLinkRefiningICP_translation(), info.icpTranslation);
					statistics_.addStatistic(Statistics::kNeighborLinkRefiningICP_complexity(), info.icpStructuralComplexity);
					statistics_.addStatistic(Statistics::kNeighborLinkRefiningPts(), signature->sensorData().laserScanRaw().size());
				}
				timeNeighborLinkRefining = timer.ticks();
				ULOGGER_INFO("timeOdometryRefining=%fs", timeNeighborLinkRefining);

				UASSERT(oldS->hasLink(signature->id()));
				UASSERT(uContains(_optimizedPoses, oldId));

				statistics_.addStatistic(Statistics::kNeighborLinkRefiningVariance(), oldS->getLinks().find(signature->id())->second.transVariance());

				newPose = _optimizedPoses.at(oldId) * oldS->getLinks().find(signature->id())->second.transform();
				_mapCorrection = newPose * signature->getPose().inverse();
				if(_mapCorrection.getNormSquared() > 0.001f && _optimizeFromGraphEnd)
				{
					UERROR("Map correction should be identity when optimizing from the last node. T=%s NewPose=%s OldPose=%s",
							_mapCorrection.prettyPrint().c_str(),
							newPose.prettyPrint().c_str(),
							signature->getPose().prettyPrint().c_str());
				}
			}
			else
			{
				newPose = _mapCorrection * signature->getPose();
				intermediateNodeRefining = true;
			}
		}
		else
		{
			newPose = _mapCorrection * signature->getPose();
		}

		UDEBUG("Added pose %s (odom=%s)", newPose.prettyPrint().c_str(), signature->getPose().prettyPrint().c_str());
		// Update Poses and Constraints
		_optimizedPoses.insert(std::make_pair(signature->id(), newPose));
		if(_memory->isIncremental() && signature->getWeight() >= 0)
		{
			for(std::map<int, Link>::const_iterator iter = signature->getLandmarks().begin(); iter!=signature->getLandmarks().end(); ++iter)
			{
				if(_optimizedPoses.find(iter->first) == _optimizedPoses.end())
				{
					_optimizedPoses.insert(std::make_pair(iter->first, newPose*iter->second.transform()));
				}
				_constraints.insert(std::make_pair(iter->first, iter->second.inverse()));
			}
		}

		float distanceTravelledOld = _distanceTravelled;

		// only in mapping mode we add a neighbor link
		if(signature->getLinks().size() &&
		   signature->getLinks().begin()->second.type() == Link::kNeighbor)
		{
			// link should be old to new
			UASSERT_MSG(signature->id() > signature->getLinks().begin()->second.to(),
					"Only forward links should be added.");

			Link tmp = signature->getLinks().begin()->second.inverse();

			_distanceTravelled += tmp.transform().getNorm();

			// if the previous node is an intermediate node, remove it from the local graph
			if(_constraints.size() &&
			   _constraints.rbegin()->second.to() == signature->getLinks().begin()->second.to())
			{
				const Signature * s = _memory->getSignature(signature->getLinks().begin()->second.to());
				UASSERT(s!=0);
				if(s->getWeight() == -1)
				{
					tmp = _constraints.rbegin()->second.merge(tmp, tmp.type());
					_optimizedPoses.erase(s->id());
					_constraints.erase(--_constraints.end());
				}
			}
			_constraints.insert(std::make_pair(tmp.from(), tmp));
		}
		// Localization mode stuff
		_lastLocalizationPose = newPose; // keep in cache the latest corrected pose
		if(!_memory->isIncremental() && signature->getWeight() >= 0)
		{
			UDEBUG("Update odometry localization cache (size=%d/%d)", (int)_odomCachePoses.size(), _maxOdomCacheSize);
			if(!_odomCachePoses.empty())
			{
				float odomDistance = (_odomCachePoses.rbegin()->second.inverse() * signature->getPose()).getNorm();
				_distanceTravelled += odomDistance;

				while(!_odomCachePoses.empty() && (int)_odomCachePoses.size() > _maxOdomCacheSize)
				{
					_odomCacheConstraints.erase(_odomCachePoses.begin()->first);
					_odomCachePoses.erase(_odomCachePoses.begin());
				}
				if(!_odomCachePoses.empty())
				{
					Link odomLink(_odomCachePoses.rbegin()->first,
							signature->id(),
							Link::kNeighbor,
							_odomCachePoses.rbegin()->second.inverse() * signature->getPose(),
							odomCovariance.inv());
					_odomCacheConstraints.insert(std::make_pair(_odomCachePoses.rbegin()->first, odomLink));
					UDEBUG("Added odom cov = %f %f", odomLink.transVariance(), odomLink.rotVariance());
				}
			}

			_odomCachePoses.insert(std::make_pair(signature->id(), signature->getPose()));
		}
		_distanceTravelledSinceLastLocalization += _distanceTravelled - distanceTravelledOld;

		//============================================================
		// Reduced graph
		//============================================================
		//Verify if there are nodes that were merged through graph reduction
		if(statistics_.reducedIds().size())
		{
			for(unsigned int i=0; i<_path.size(); ++i)
			{
				std::map<int, int>::const_iterator iter = statistics_.reducedIds().find(_path[i].first);
				if(iter!= statistics_.reducedIds().end())
				{
					// change path ID to loop closure ID
					_path[i].first = iter->second;
				}
			}

			for(std::map<int, int>::const_iterator iter=statistics_.reducedIds().begin();
				iter!=statistics_.reducedIds().end();
				++iter)
			{
				int erased = (int)_optimizedPoses.erase(iter->first);
				if(erased)
				{
					for(std::multimap<int, Link>::iterator jter = _constraints.begin(); jter!=_constraints.end();)
					{
						if(jter->second.from() == iter->first || jter->second.to() == iter->first)
						{
							_constraints.erase(jter++);
						}
						else
						{
							++jter;
						}
					}
				}
			}
		}

		//============================================================
		// Local loop closure in TIME
		//============================================================
		if((_proximityByTime || intermediateNodeRefining) &&
		   rehearsedId == 0 && // don't do it if rehearsal happened
		   _memory->isIncremental() && // don't do it in localization mode
		   signature->getWeight()>=0)
		{
			const std::set<int> & stm = _memory->getStMem();
			for(std::set<int>::const_reverse_iterator iter = stm.rbegin(); iter!=stm.rend(); ++iter)
			{
				if(*iter != signature->id() &&
				   signature->getLinks().find(*iter) == signature->getLinks().end() &&
				   _memory->getSignature(*iter)->mapId() == signature->mapId() &&
				   _memory->getSignature(*iter)->getWeight()>=0)
				{
					std::string rejectedMsg;
					UDEBUG("Check local transform between %d and %d", signature->id(), *iter);
					RegistrationInfo info;
					Transform guess;
					if(_optimizedPoses.find(*iter) != _optimizedPoses.end())
					{
						guess = _optimizedPoses.at(*iter).inverse() * newPose;
					}

					// For proximity by time, correspondences should be already enough precise, so don't recompute them
					Transform transform = _memory->computeTransform(*iter, signature->id(), guess, &info, true);

					if(!transform.isNull())
					{
						transform = transform.inverse();
						UDEBUG("Add local loop closure in TIME (%d->%d) %s",
								signature->id(),
								*iter,
								transform.prettyPrint().c_str());
						// Add a loop constraint
						UASSERT(info.covariance.at<double>(0,0) > 0.0 && info.covariance.at<double>(5,5) > 0.0);
						if(_memory->addLink(Link(signature->id(), *iter, Link::kLocalTimeClosure, transform, getInformation(info.covariance))))
						{
							++proximityDetectionsInTimeFound;
							UINFO("Local loop closure found between %d and %d with t=%s",
									*iter, signature->id(), transform.prettyPrint().c_str());
						}
						else
						{
							UWARN("Cannot add local loop closure between %d and %d ?!?",
									*iter, signature->id());
						}
					}
					else
					{
						UINFO("Local loop closure (time) between %d and %d rejected: %s",
								*iter, signature->id(), rejectedMsg.c_str());
					}

					if(!_proximityByTime && intermediateNodeRefining)
					{
						// Do it only with the latest non-intermediate node
						break;
					}
				}
			}
		}
	}

	timeProximityByTimeDetection = timer.ticks();
	UINFO("timeProximityByTimeDetection=%fs", timeProximityByTimeDetection);

	//============================================================
	// Bayes filter update
	//============================================================
	int previousId = signature->getLinks().size() && signature->getLinks().begin()->first!=signature->id()?signature->getLinks().begin()->first:0;
	// Not a bad signature, not an intermediate node, not a small displacement unless the previous signature didn't have a loop closure, not too fast movement
	if(!signature->isBadSignature() && signature->getWeight()>=0 && (!smallDisplacement || _memory->getLoopClosureLinks(previousId, false).size() == 0) && !tooFastMovement)
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

			std::list<int> signaturesToCompare;
			GPS originGPS;
			Transform originOffsetENU = Transform::getIdentity();
			if(_loopGPS)
			{
				originGPS = signature->sensorData().gps();
				if(originGPS.stamp() == 0.0 && _currentSessionHasGPS)
				{
					UTimer tmpT;
					if(_optimizedPoses.size() && _memory->isIncremental())
					{
						//Search for latest node having GPS linked to current signature not too far.
						std::map<int, float> nearestIds = graph::findNearestNodes(signature->id(), _optimizedPoses, _localRadius);
						for(std::map<int, float>::reverse_iterator iter=nearestIds.rbegin(); iter!=nearestIds.rend() && iter->first>0; ++iter)
						{
							const Signature * s = _memory->getSignature(iter->first);
							UASSERT(s!=0);
							if(s->sensorData().gps().stamp() > 0.0)
							{
								originGPS = s->sensorData().gps();
								const Transform & sPose = _optimizedPoses.at(s->id());
								Transform localToENU(0,0,(float)((-(originGPS.bearing()-90))*M_PI/180.0) - sPose.theta());
								originOffsetENU = localToENU * (sPose.rotation()*(sPose.inverse()*_optimizedPoses.at(signature->id())));
								break;
							}
						}
					}
					//else if(!_memory->isIncremental()) // TODO, how can we estimate current GPS position in localization?
					//{
					//}
				}
				if(originGPS.stamp() > 0.0)
				{
					// no need to save it if it is in localization mode
					_gpsGeocentricCache.insert(std::make_pair(signature->id(), std::make_pair(originGPS.toGeodeticCoords().toGeocentric_WGS84(), originOffsetENU)));
				}
			}

			for(std::map<int, double>::const_iterator iter=_memory->getWorkingMem().begin();
				iter!=_memory->getWorkingMem().end();
				++iter)
			{
				if(iter->first > 0)
				{
					const Signature * s = _memory->getSignature(iter->first);
					UASSERT(s!=0);
					if(s->getWeight() != -1) // ignore intermediate nodes
					{
						bool accept = true;
						if(originGPS.stamp()>0.0)
						{
							std::map<int, std::pair<cv::Point3d, Transform> >::iterator cacheIter = _gpsGeocentricCache.find(s->id());
							if(cacheIter == _gpsGeocentricCache.end())
							{
								GPS gps = s->sensorData().gps();
								Transform offsetENU = Transform::getIdentity();
								if(gps.stamp()==0.0)
								{
									_memory->getGPS(s->id(), gps, offsetENU, false);
								}
								if(gps.stamp() > 0.0)
								{
									cacheIter = _gpsGeocentricCache.insert(
											std::make_pair(s->id(),
													std::make_pair(gps.toGeodeticCoords().toGeocentric_WGS84(), offsetENU))).first;
								}
							}


							if(cacheIter != _gpsGeocentricCache.end())
							{
								std::map<int, std::pair<cv::Point3d, Transform> >::iterator originIter = _gpsGeocentricCache.find(signature->id());
								UASSERT(originIter != _gpsGeocentricCache.end());
								cv::Point3d relativePose = GeodeticCoords::Geocentric_WGS84ToENU_WGS84(cacheIter->second.first, originIter->second.first, originGPS.toGeodeticCoords());
								const double & error = originGPS.error();
								const Transform & offsetENU = cacheIter->second.second;
								relativePose.x += offsetENU.x() - originOffsetENU.x();
								relativePose.y += offsetENU.y() - originOffsetENU.y();
								relativePose.z += offsetENU.z() - originOffsetENU.z();
								 // ignore altitude if difference is under GPS error
								if(relativePose.z>error)
								{
									relativePose.z -= error;
								}
								else if(relativePose.z < -error)
								{
									relativePose.z += error;
								}
								else
								{
									relativePose.z = 0;
								}
								accept = uNormSquared(relativePose.x, relativePose.y, relativePose.z) < _localRadius*_localRadius;
							}
						}

						if(accept)
						{
							signaturesToCompare.push_back(iter->first);
						}
					}
				}
				else
				{
					// virtual signature should be added
					signaturesToCompare.push_back(iter->first);
				}
			}

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

			// For statistics, copy weights
			if(_publishStats && (_publishLikelihood || _publishPdf))
			{
				weights = _memory->getWeights();
			}

			//============================================================
			// Select the highest hypothesis
			//============================================================
			ULOGGER_INFO("creating hypotheses...");
			if(posterior.size())
			{
				for(std::map<int, float>::const_reverse_iterator iter = posterior.rbegin(); iter != posterior.rend(); ++iter)
				{
					if(iter->first > 0 && iter->second > _highestHypothesis.second)
					{
						_highestHypothesis = *iter;
					}
				}
				// With the virtual place, use sum of LC probabilities (1 - virtual place hypothesis).
				_highestHypothesis.second = 1-posterior.begin()->second;
			}
			timeHypothesesCreation = timer.ticks();
			ULOGGER_INFO("Highest hypothesis=%d, value=%f, timeHypothesesCreation=%fs", _highestHypothesis.first, _highestHypothesis.second, timeHypothesesCreation);

			if(_highestHypothesis.first > 0)
			{
				float loopThr = _loopThr;
				if((_startNewMapOnLoopClosure || !_memory->isIncremental()) &&
					graph::filterLinks(signature->getLinks(), Link::kSelfRefLink).size() == 0 && // alone in the current map
					_memory->getWorkingMem().size()>1 && // should have an old map (beside virtual signature)
					(int)_memory->getWorkingMem().size()<=_memory->getMaxStMemSize() &&
					_rgbdSlamMode)
				{
					// If the map is very small (under STM size) and we need to find
					// a loop closure before continuing the map or localizing,
					// use the best hypothesis directly.
					loopThr = 0.0f;
				}

				// Loop closure Threshold
				// When _loopThr=0, accept loop closure if the hypothesis is over
				// the virtual (new) place hypothesis.
				if(_highestHypothesis.second >= loopThr)
				{
					rejectedGlobalLoopClosure = true;
					if(posterior.size() <= 2 && loopThr>0.0f)
					{
						// Ignore loop closure if there is only one loop closure hypothesis
						UDEBUG("rejected hypothesis: single hypothesis");
					}
					else if(_verifyLoopClosureHypothesis && !_epipolarGeometry->check(signature, _memory->getSignature(_highestHypothesis.first)))
					{
						UWARN("rejected hypothesis: by epipolar geometry");
					}
					else if(_loopRatio > 0.0f && lastHighestHypothesis.second && _highestHypothesis.second < _loopRatio*lastHighestHypothesis.second)
					{
						UWARN("rejected hypothesis: not satisfying hypothesis ratio (%f < %f * %f)",
								_highestHypothesis.second, _loopRatio, lastHighestHypothesis.second);
					}
					else if(_loopRatio > 0.0f && lastHighestHypothesis.second == 0)
					{
						UWARN("rejected hypothesis: last closure hypothesis is null (loop ratio is on)");
					}
					else
					{
						_loopClosureHypothesis = _highestHypothesis;
						rejectedGlobalLoopClosure = false;
					}

					timeHypothesesValidation = timer.ticks();
					ULOGGER_INFO("timeHypothesesValidation=%fs",timeHypothesesValidation);
				}
				else if(_highestHypothesis.second < _loopRatio*lastHighestHypothesis.second)
				{
					// Used for Precision-Recall computation.
					// When analyzing logs, it's convenient to know
					// if the hypothesis would be rejected if T_loop would be lower.
					rejectedGlobalLoopClosure = true;
					UDEBUG("rejected hypothesis: under loop ratio %f < %f", _highestHypothesis.second, _loopRatio*lastHighestHypothesis.second);
				}

				//for statistic...
				hypothesisRatio = _loopClosureHypothesis.second>0?_highestHypothesis.second/_loopClosureHypothesis.second:0;
			}
		} // if(_memory->getWorkingMemSize())
	}// !isBadSignature
	else if(!signature->isBadSignature() && (smallDisplacement || tooFastMovement))
	{
		_highestHypothesis = lastHighestHypothesis;
	}

	//============================================================
	// Before retrieval, make sure the trash has finished
	//============================================================
	_memory->joinTrashThread();
	timeEmptyingTrash = _memory->getDbSavingTime();
	timeJoiningTrash = timer.ticks();
	ULOGGER_INFO("Time emptying memory trash = %fs,  joining (actual overhead) = %fs", timeEmptyingTrash, timeJoiningTrash);

	//============================================================
	// RETRIEVAL 1/3 : Loop closure neighbors reactivation
	//============================================================
	int retrievalId = _highestHypothesis.first;
	std::list<int> reactivatedIds;
	double timeGetNeighborsTimeDb = 0.0;
	double timeGetNeighborsSpaceDb = 0.0;
	int immunizedGlobally = 0;
	int immunizedLocally = 0;
	int maxLocalLocationsImmunized = 0;
	if(_maxTimeAllowed != 0 || _maxMemoryAllowed != 0)
	{
		// with memory management, we have to immunize some nodes
		maxLocalLocationsImmunized = _localImmunizationRatio * float(_memory->getWorkingMem().size());
	}
	// no need to do retrieval or immunization of locations if memory management
	// is disabled and all nodes are in WM
	if(!(_memory->allNodesInWM() && maxLocalLocationsImmunized == 0))
	{
		if(retrievalId > 0)
		{
			//Load neighbors
			ULOGGER_INFO("Retrieving locations... around id=%d", retrievalId);
			int neighborhoodSize = (int)_bayesFilter->getPredictionLC().size()-1;
			UASSERT(neighborhoodSize >= 0);
			ULOGGER_DEBUG("margin=%d maxRetieved=%d", neighborhoodSize, _maxRetrieved);

			UTimer timeGetN;
			unsigned int nbLoadedFromDb = 0;
			std::set<int> reactivatedIdsSet;
			std::map<int, int> neighbors;
			int nbDirectNeighborsInDb = 0;

			// priority in time
			// Direct neighbors TIME
			ULOGGER_DEBUG("In TIME");
			neighbors = _memory->getNeighborsId(retrievalId,
					neighborhoodSize,
					_maxRetrieved,
					true,
					true,
					false,
					true,
					std::set<int>(),
					&timeGetNeighborsTimeDb);
			ULOGGER_DEBUG("neighbors of %d in time = %d", retrievalId, (int)neighbors.size());
			//Priority to locations near in time (direct neighbor) then by space (loop closure)
			bool firstPassDone = false; // just to avoid checking to STM after the first pass
			int m = 0;
			while(m < neighborhoodSize)
			{
				std::set<int> idsSorted;
				for(std::map<int, int>::iterator iter=neighbors.begin(); iter!=neighbors.end();)
				{
					if(!firstPassDone && _memory->isInSTM(iter->first))
					{
						neighbors.erase(iter++);
					}
					else if(iter->second == m)
					{
						if(reactivatedIdsSet.find(iter->first) == reactivatedIdsSet.end())
						{
							idsSorted.insert(iter->first);
							reactivatedIdsSet.insert(iter->first);

							if(m == 1 && _memory->getSignature(iter->first) == 0)
							{
								++nbDirectNeighborsInDb;
							}

							//immunized locations in the neighborhood from being transferred
							if(immunizedLocations.insert(iter->first).second)
							{
								++immunizedGlobally;
							}

							//UDEBUG("nt=%d m=%d immunized=1", iter->first, iter->second);
						}
						neighbors.erase(iter++);
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
			neighbors = _memory->getNeighborsId(retrievalId,
					neighborhoodSize,
					_maxRetrieved,
					true,
					false,
					false,
					false,
					std::set<int>(),
					&timeGetNeighborsSpaceDb);
			ULOGGER_DEBUG("neighbors of %d in space = %d", retrievalId, (int)neighbors.size());
			firstPassDone = false;
			m = 0;
			while(m < neighborhoodSize)
			{
				std::set<int> idsSorted;
				for(std::map<int, int>::iterator iter=neighbors.begin(); iter!=neighbors.end();)
				{
					if(!firstPassDone && _memory->isInSTM(iter->first))
					{
						neighbors.erase(iter++);
					}
					else if(iter->second == m)
					{
						if(reactivatedIdsSet.find(iter->first) == reactivatedIdsSet.end())
						{
							idsSorted.insert(iter->first);
							reactivatedIdsSet.insert(iter->first);

							if(m == 1 && _memory->getSignature(iter->first) == 0)
							{
								++nbDirectNeighborsInDb;
							}
							//UDEBUG("nt=%d m=%d", iter->first, iter->second);
						}
						neighbors.erase(iter++);
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
			ULOGGER_INFO("neighborhoodSize=%d, "
					"reactivatedIds.size=%d, "
					"nbLoadedFromDb=%d, "
					"nbDirectNeighborsInDb=%d, "
					"time=%fs (%fs %fs)",
					neighborhoodSize,
					reactivatedIds.size(),
					(int)nbLoadedFromDb,
					nbDirectNeighborsInDb,
					timeGetN.ticks(),
					timeGetNeighborsTimeDb,
					timeGetNeighborsSpaceDb);

		}
	}

	//============================================================
	// RETRIEVAL 2/3 : Update planned path and get next nodes to retrieve
	//============================================================
	std::list<int> retrievalLocalIds;
	if(_rgbdSlamMode)
	{
		// Priority on locations on the planned path
		if(_path.size())
		{
			updateGoalIndex();

			float distanceSoFar = 0.0f;
			// immunize all nodes after current node and
			// retrieve nodes after current node in the maximum radius from the current node
			for(unsigned int i=_pathCurrentIndex; i<_path.size(); ++i)
			{
				if(_localRadius > 0.0f && i != _pathCurrentIndex)
				{
					distanceSoFar += _path[i-1].second.getDistance(_path[i].second);
				}

				if(distanceSoFar <= _localRadius)
				{
					if(_memory->getSignature(_path[i].first) != 0)
					{
						if(immunizedLocations.insert(_path[i].first).second)
						{
							++immunizedLocally;
						}
						UDEBUG("Path immunization: node %d (dist=%fm)", _path[i].first, distanceSoFar);
					}
					else if(retrievalLocalIds.size() < _maxLocalRetrieved)
					{
						UINFO("retrieval of node %d on path (dist=%fm)", _path[i].first, distanceSoFar);
						retrievalLocalIds.push_back(_path[i].first);
						// retrieved locations are automatically immunized
					}
				}
				else
				{
					UDEBUG("Stop on node %d (dist=%fm > %fm)",
							_path[i].first, distanceSoFar, _localRadius);
					break;
				}
			}
		}

		if(!(_memory->allNodesInWM() && maxLocalLocationsImmunized == 0))
		{
			// immunize the path from the nearest local location to the current location
			if(immunizedLocally < maxLocalLocationsImmunized &&
				_memory->isIncremental()) // Can only work in mapping mode
			{
				std::map<int ,Transform> poses;
				// remove poses from STM
				for(std::map<int, Transform>::iterator iter=_optimizedPoses.begin(); iter!=_optimizedPoses.end(); ++iter)
				{
					if(iter->first > 0 && !_memory->isInSTM(iter->first))
					{
						poses.insert(*iter);
					}
				}
				int nearestId = graph::findNearestNode(poses, _optimizedPoses.at(signature->id()));

				if(nearestId > 0 &&
					(_localRadius==0 ||
					 _optimizedPoses.at(signature->id()).getDistance(_optimizedPoses.at(nearestId)) < _localRadius))
				{
					std::multimap<int, int> links;
					for(std::multimap<int, Link>::iterator iter=_constraints.begin(); iter!=_constraints.end(); ++iter)
					{
						if(uContains(_optimizedPoses, iter->second.from()) && uContains(_optimizedPoses, iter->second.to()))
						{
							links.insert(std::make_pair(iter->second.from(), iter->second.to()));
							links.insert(std::make_pair(iter->second.to(), iter->second.from())); // <->
						}
					}

					std::list<std::pair<int, Transform> > path = graph::computePath(_optimizedPoses, links, nearestId, signature->id());
					if(path.size() == 0)
					{
						UWARN("Could not compute a path between %d and %d", nearestId, signature->id());
					}
					else
					{
						for(std::list<std::pair<int, Transform> >::iterator iter=path.begin();
							iter!=path.end();
							++iter)
						{
							if(iter->first>0)
							{
								if(immunizedLocally >= maxLocalLocationsImmunized)
								{
									// set 20 to avoid this warning when starting mapping
									if(maxLocalLocationsImmunized > 20 && _someNodesHaveBeenTransferred)
									{
										UWARN("Could not immunize the whole local path (%d) between "
											  "%d and %d (max location immunized=%d). You may want "
											  "to increase RGBD/LocalImmunizationRatio (current=%f (%d of WM=%d)) "
											  "to be able to immunize longer paths.",
												(int)path.size(),
												nearestId,
												signature->id(),
												maxLocalLocationsImmunized,
												_localImmunizationRatio,
												maxLocalLocationsImmunized,
												(int)_memory->getWorkingMem().size());
									}
									break;
								}
								else if(!_memory->isInSTM(iter->first))
								{
									if(immunizedLocations.insert(iter->first).second)
									{
										++immunizedLocally;
									}
									//UDEBUG("local node %d on path immunized=1", iter->first);
								}
							}
						}
					}
				}
			}

			// retrieval based on the nodes close the the nearest pose in WM
			// immunize closest nodes
			std::map<int, float> nearNodes = graph::findNearestNodes(signature->id(), _optimizedPoses, _localRadius);
			// sort by distance
			std::multimap<float, int> nearNodesByDist;
			for(std::map<int, float>::iterator iter=nearNodes.lower_bound(1); iter!=nearNodes.end(); ++iter)
			{
				nearNodesByDist.insert(std::make_pair(iter->second, iter->first));
			}
			UINFO("near nodes=%d, max local immunized=%d, ratio=%f WM=%d",
					(int)nearNodesByDist.size(),
					maxLocalLocationsImmunized,
					_localImmunizationRatio,
					(int)_memory->getWorkingMem().size());
			for(std::multimap<float, int>::iterator iter=nearNodesByDist.begin();
				iter!=nearNodesByDist.end() && (retrievalLocalIds.size() < _maxLocalRetrieved || immunizedLocally < maxLocalLocationsImmunized);
				++iter)
			{
				const Signature * s = _memory->getSignature(iter->second);
				if(s!=0)
				{
					// If there is a change of direction, better to be retrieving
					// ALL nearest signatures than only newest neighbors
					const std::multimap<int, Link> & links = s->getLinks();
					for(std::multimap<int, Link>::const_reverse_iterator jter=links.rbegin();
						jter!=links.rend() && retrievalLocalIds.size() < _maxLocalRetrieved;
						++jter)
					{
						if(_memory->getSignature(jter->first) == 0)
						{
							UINFO("retrieval of node %d on local map", jter->first);
							retrievalLocalIds.push_back(jter->first);
						}
					}
					if(!_memory->isInSTM(s->id()) && immunizedLocally < maxLocalLocationsImmunized)
					{
						if(immunizedLocations.insert(s->id()).second)
						{
							++immunizedLocally;
						}
						//UDEBUG("local node %d (%f m) immunized=1", iter->second, iter->first);
					}
				}
			}
			// well, if the maximum retrieved is not reached, look for neighbors in database
			if(retrievalLocalIds.size() < _maxLocalRetrieved)
			{
				std::set<int> retrievalLocalIdsSet(retrievalLocalIds.begin(), retrievalLocalIds.end());
				for(std::list<int>::iterator iter=retrievalLocalIds.begin();
					iter!=retrievalLocalIds.end() && retrievalLocalIds.size() < _maxLocalRetrieved;
					++iter)
				{
					std::map<int, int> ids = _memory->getNeighborsId(*iter, 2, _maxLocalRetrieved - (unsigned int)retrievalLocalIds.size() + 1, true, false);
					for(std::map<int, int>::reverse_iterator jter=ids.rbegin();
						jter!=ids.rend() && retrievalLocalIds.size() < _maxLocalRetrieved;
						++jter)
					{
						if(_memory->getSignature(jter->first) == 0 &&
						   retrievalLocalIdsSet.find(jter->first) == retrievalLocalIdsSet.end())
						{
							UINFO("retrieval of node %d on local map", jter->first);
							retrievalLocalIds.push_back(jter->first);
							retrievalLocalIdsSet.insert(jter->first);
						}
					}
				}
			}

			// update Age of the close signatures (oldest the farthest)
			for(std::multimap<float, int>::reverse_iterator iter=nearNodesByDist.rbegin(); iter!=nearNodesByDist.rend(); ++iter)
			{
				_memory->updateAge(iter->second);
			}

			// insert them first to make sure they are loaded.
			reactivatedIds.insert(reactivatedIds.begin(), retrievalLocalIds.begin(), retrievalLocalIds.end());
		}

		//============================================================
		// RETRIEVAL 3/3 : Load signatures from the database
		//============================================================
		if(reactivatedIds.size())
		{
			// Not important if the loop closure hypothesis don't have all its neighbors loaded,
			// only a loop closure link is added...
			signaturesRetrieved = _memory->reactivateSignatures(
					reactivatedIds,
					_maxRetrieved+(unsigned int)retrievalLocalIds.size(), // add path retrieved
					timeRetrievalDbAccess);

			ULOGGER_INFO("retrieval of %d (db time = %fs)", (int)signaturesRetrieved.size(), timeRetrievalDbAccess);

			timeRetrievalDbAccess += timeGetNeighborsTimeDb + timeGetNeighborsSpaceDb;
			UINFO("total timeRetrievalDbAccess=%fs", timeRetrievalDbAccess);

			// Immunize just retrieved signatures
			immunizedLocations.insert(signaturesRetrieved.begin(), signaturesRetrieved.end());

			if(!signaturesRetrieved.empty() && !_globalScanMap.empty())
			{
				UWARN("Some signatures have been retrieved from memory management, clearing global scan map...");
				_globalScanMap.clear();
				_globalScanMapPoses.clear();
			}
		}
		timeReactivations = timer.ticks();
		ULOGGER_INFO("timeReactivations=%fs", timeReactivations);
	}

	//============================================================
	// Landmark
	//============================================================
	std::map<int, std::set<int> > landmarksDetected; // <Landmark ID, list of nodes that saw this landmark>
	if(!signature->getLandmarks().empty())
	{
		for(std::map<int, Link>::const_iterator iter=signature->getLandmarks().begin(); iter!=signature->getLandmarks().end(); ++iter)
		{
			if(uContains(_memory->getLandmarksIndex(), iter->first) &&
					_memory->getLandmarksIndex().find(iter->first)->second.size()>1)
			{
				UINFO("Landmark %d observed again! Seen the first time by node %d.", -iter->first, *_memory->getLandmarksIndex().find(iter->first)->second.begin());
				landmarksDetected.insert(std::make_pair(iter->first, _memory->getLandmarksIndex().find(iter->first)->second));
			}
		}
	}

	//============================================================
	// Proximity detections
	//============================================================
	std::list<std::pair<int, int> > loopClosureLinksAdded;
	int loopClosureVisualInliers = 0; // for statistics
	float loopClosureVisualInliersRatio = 0.0f;
	int loopClosureVisualMatches = 0;
	float loopClosureLinearVariance = 0.0f;
	float loopClosureAngularVariance = 0.0f;
	float loopClosureVisualInliersMeanDist = 0;
	float loopClosureVisualInliersDistribution = 0;

	int proximityDetectionsAddedVisually = 0;
	int proximityDetectionsAddedByICPMulti = 0;
	int proximityDetectionsAddedByICPGlobal = 0;
	int lastProximitySpaceClosureId = 0;
	int proximitySpacePaths = 0;
	int localVisualPathsChecked = 0;
	int localScanPathsChecked = 0;
	int loopIdSuppressedByProximity = 0;

	if(_proximityBySpace &&
	   _localRadius > 0 &&
	   _rgbdSlamMode &&
	   signature->getWeight() >= 0) // not an intermediate node
	{
		if(_startNewMapOnLoopClosure &&
			_memory->getWorkingMem().size()>=2 && // must have an old map (+1 virtual place)
			graph::filterLinks(signature->getLinks(), Link::kSelfRefLink).size() == 0) // alone in new session)
		{
			UINFO("Proximity detection by space disabled as if we force to have a global loop "
					"closure with previous map before doing proximity detections (%s=true).",
					Parameters::kRtabmapStartNewMapOnLoopClosure().c_str());
		}
		else if(_graphOptimizer->iterations() == 0)
		{
			UWARN("Cannot do local loop closure detection in space if graph optimization is disabled!");
		}
		else
		{
			// In localization mode, no need to check local loop
			// closures if we are already localized by a landmark.

			// don't do it if it is a small displacement unless the previous signature didn't have a loop closure
			// don't do it if there is a too fast movement
			if((!smallDisplacement || _memory->getLoopClosureLinks(previousId, false).size() == 0) && !tooFastMovement)
			{

				//============================================================
				// LOCAL LOOP CLOSURE SPACE
				//============================================================

				//
				// 1) compare visually with nearest locations
				//
				UDEBUG("Proximity detection (local loop closure in SPACE using matching images, local radius=%fm)", _localRadius);
				std::map<int, float> nearestIds;
				if(_memory->isIncremental() && _proximityMaxGraphDepth > 0)
				{
					nearestIds = _memory->getNeighborsIdRadius(signature->id(), _localRadius, _optimizedPoses, _proximityMaxGraphDepth);
				}
				else
				{
					nearestIds = graph::findNearestNodes(signature->id(), _optimizedPoses, _localRadius);
				}
				UDEBUG("nearestIds=%d/%d", (int)nearestIds.size(), (int)_optimizedPoses.size());
				std::map<int, Transform> nearestPoses;
				for(std::map<int, float>::iterator iter=nearestIds.lower_bound(1); iter!=nearestIds.end(); ++iter)
				{
					if(_memory->getStMem().find(iter->first) == _memory->getStMem().end())
					{
						nearestPoses.insert(std::make_pair(iter->first, _optimizedPoses.at(iter->first)));
					}
				}
				UDEBUG("nearestPoses=%d", (int)nearestPoses.size());

				// segment poses by paths, only one detection per path, landmarks are ignored
				std::map<int, std::map<int, Transform> > nearestPathsNotSorted = getPaths(nearestPoses, _optimizedPoses.at(signature->id()), _proximityMaxGraphDepth);
				UDEBUG("got %d paths", (int)nearestPathsNotSorted.size());
				// sort nearest paths by highest likelihood (if two have same likelihood, sort by id)
				std::map<NearestPathKey, std::map<int, Transform> > nearestPaths;
				for(std::map<int, std::map<int, Transform> >::const_iterator iter=nearestPathsNotSorted.begin();iter!=nearestPathsNotSorted.end(); ++iter)
				{
					const std::map<int, Transform> & path = iter->second;
					float highestLikelihood = 0.0f;
					int highestLikelihoodId = iter->first;
					for(std::map<int, Transform>::const_iterator jter=path.begin(); jter!=path.end(); ++jter)
					{
						float v = uValue(likelihood, jter->first, 0.0f);
						if(v > highestLikelihood)
						{
							highestLikelihood = v;
							highestLikelihoodId = jter->first;
						}
					}
					nearestPaths.insert(std::make_pair(NearestPathKey(highestLikelihood, highestLikelihoodId), path));
				}
				UDEBUG("nearestPaths=%d proximityMaxPaths=%d", (int)nearestPaths.size(), _proximityMaxPaths);

				float proximityFilteringRadius = _proximityFilteringRadius;
				if(_maxLoopClosureDistance>0.0f && (proximityFilteringRadius <= 0.0f || _maxLoopClosureDistance<proximityFilteringRadius))
				{
					proximityFilteringRadius = _maxLoopClosureDistance;
				}
				for(std::map<NearestPathKey, std::map<int, Transform> >::const_reverse_iterator iter=nearestPaths.rbegin();
					iter!=nearestPaths.rend() &&
					(_proximityMaxPaths <= 0 || localVisualPathsChecked < _proximityMaxPaths);
					++iter)
				{
					std::map<int, Transform> path = iter->second;
					UASSERT(path.size());

					//find the nearest pose on the path looking in the same direction
					path.insert(std::make_pair(signature->id(), _optimizedPoses.at(signature->id())));
					path = graph::findNearestPoses(signature->id(), path, _localRadius, _proximityAngle);
					//take the one with highest likelihood if not null
					int nearestId = 0;
					if(iter->first.likelihood > 0.0f &&
					   path.find(iter->first.id)!=path.end())
					{
						nearestId = iter->first.id;
					}
					else
					{
						nearestId = rtabmap::graph::findNearestNode(path, _optimizedPoses.at(signature->id()));
					}

					if(nearestId > 0)
					{
						// nearest pose must not be linked to current location and enough close
						if(!signature->hasLink(nearestId) &&
							(proximityFilteringRadius <= 0.0f ||
							 _optimizedPoses.at(signature->id()).getDistanceSquared(_optimizedPoses.at(nearestId)) < proximityFilteringRadius*proximityFilteringRadius))
						{
							++localVisualPathsChecked;
							RegistrationInfo info;
							Transform guess;
							if(_proximityOdomGuess)
							{
								// Use odometry as guess so that correspondences can be computed by projection
								guess = _optimizedPoses.at(nearestId).inverse()*_optimizedPoses.at(signature->id());
							} //else: guess is null to make sure visual correspondences are globally computed
							Transform transform = _memory->computeTransform(nearestId, signature->id(), guess, &info);
							if(!transform.isNull())
							{
								transform = transform.inverse();
								if(proximityFilteringRadius <= 0 || transform.getNormSquared() <= proximityFilteringRadius*proximityFilteringRadius)
								{
									UINFO("[Visual] Add local loop closure in SPACE (%d->%d) %s",
											signature->id(),
											nearestId,
											transform.prettyPrint().c_str());
									UASSERT(info.covariance.at<double>(0,0) > 0.0 && info.covariance.at<double>(5,5) > 0.0);

									//for statistics
									loopClosureVisualInliersMeanDist = info.inliersMeanDistance;
									loopClosureVisualInliersDistribution = info.inliersDistribution;

									++proximityDetectionsAddedVisually;
									lastProximitySpaceClosureId = nearestId;

									loopClosureVisualInliers = info.inliers;
									loopClosureVisualInliersRatio = info.inliersRatio;
									loopClosureVisualMatches = info.matches;

									cv::Mat information = getInformation(info.covariance);
									loopClosureLinearVariance = 1.0/information.at<double>(0,0);
									loopClosureAngularVariance = 1.0/information.at<double>(5,5);

									Link::Type type = Link::kLocalSpaceClosure;
									if(_loopClosureHypothesis.first>0 &&
										nearestIds.find(_loopClosureHypothesis.first)!=nearestIds.end())
									{
										UDEBUG("Proximity detection on %d is close to loop closure %d, ignoring loop closure transform estimation...",
												nearestId, _loopClosureHypothesis.first);
										if(nearestId == _loopClosureHypothesis.first)
										{
											type = Link::kGlobalClosure;
										}
										// In localization mode, avoid transform
										// computation on the global loop closure if a visual proximity
										// one has been detected close (inside proximity radius) to that hypothesis.
										loopIdSuppressedByProximity = _loopClosureHypothesis.first;
										_loopClosureHypothesis.first = 0;
									}

									_memory->addLink(Link(signature->id(), nearestId, type, transform, information));
									loopClosureLinksAdded.push_back(std::make_pair(signature->id(), nearestId));
								}
								else
								{
									UWARN("Ignoring local loop closure with %d because resulting "
										  "transform is too large!? (%fm > %fm)",
											nearestId, transform.getNorm(), proximityFilteringRadius);
								}
							}
						}
					}
				}

				timeProximityBySpaceVisualDetection = timer.ticks();
				ULOGGER_INFO("timeProximityBySpaceVisualDetection=%fs", timeProximityBySpaceVisualDetection);

				//
				// 2) compare locally with nearest locations by scan matching
				//
				UDEBUG("Proximity detection (local loop closure in SPACE with scan matching)");
				if( _proximityMaxNeighbors <= 0)
				{
					UDEBUG("Proximity by scan matching is disabled (%s=%d).", Parameters::kRGBDProximityPathMaxNeighbors().c_str(), _proximityMaxNeighbors);
				}
				else if(!signature->sensorData().laserScanCompressed().isEmpty())
				{
					proximitySpacePaths = (int)nearestPaths.size();
					for(std::map<NearestPathKey, std::map<int, Transform> >::const_reverse_iterator iter=nearestPaths.rbegin();
							iter!=nearestPaths.rend() &&
							(_proximityMaxPaths <= 0 || localScanPathsChecked < _proximityMaxPaths);
							++iter)
					{
						std::map<int, Transform> path = iter->second; // should contain only nodes (no landmarks)
						UASSERT(path.size());
						UASSERT(path.begin()->first > 0);

						//find the nearest pose on the path
						int nearestId = rtabmap::graph::findNearestNode(path, _optimizedPoses.at(signature->id()));
						UASSERT(nearestId > 0);
						//UDEBUG("Path %d (size=%d) distance=%fm", nearestId, (int)path.size(), _optimizedPoses.at(signature->id()).getDistance(_optimizedPoses.at(nearestId)));

						// nearest pose must be close and not linked to current location
						if(!signature->hasLink(nearestId))
						{
							if(_proximityMaxNeighbors < _proximityMaxGraphDepth || _proximityMaxGraphDepth == 0)
							{
								std::map<int, Transform> filteredPath;
								int i=0;
								std::map<int, Transform>::iterator nearestIdIter = path.find(nearestId);
								// "_proximityMaxNeighbors-1" means that if _proximityMaxNeighbors=1,
								// only nearest node on the path is taken (no scan merging). Useful to find
								// proximity detection between only 2 nodes with 360x360 lidar scans.
								for(std::map<int, Transform>::iterator iter=nearestIdIter; iter!=path.end() && i<=_proximityMaxNeighbors-1; ++iter, ++i)
								{
									filteredPath.insert(*iter);
								}
								i=1;
								for(std::map<int, Transform>::reverse_iterator iter(nearestIdIter); iter!=path.rend() && i<=_proximityMaxNeighbors-1; ++iter, ++i)
								{
									filteredPath.insert(*iter);
								}
								path = filteredPath;
							}

							// Assemble scans in the path and do ICP only
							std::map<int, Transform> optimizedLocalPath;
							if(_globalScanMap.empty() && _proximityRawPosesUsed)
							{
								//optimize the path's poses locally
								cv::Mat covariance;
								path = optimizeGraph(nearestId, uKeysSet(path), std::map<int, Transform>(), false, covariance);
								// transform local poses in optimized graph referential
								if(!uContains(path, nearestId))
								{
									UERROR("Proximity path not containing nearest ID ?! Skipping this path.");
									continue;
								}
								Transform t = _optimizedPoses.at(nearestId) * path.at(nearestId).inverse();

								for(std::map<int, Transform>::iterator jter=path.lower_bound(1); jter!=path.end(); ++jter)
								{
									optimizedLocalPath.insert(std::make_pair(jter->first, t * jter->second));
								}
							}
							else
							{
								optimizedLocalPath = path;
							}

							std::map<int, Transform> filteredPath;
							if(_globalScanMap.empty() && optimizedLocalPath.size() > 2 && proximityFilteringRadius > 0.0f)
							{
								// path filtering
								filteredPath = graph::radiusPosesFiltering(optimizedLocalPath, proximityFilteringRadius, 0, true);
								// make sure the current pose is still here
								filteredPath.insert(*optimizedLocalPath.find(nearestId));
							}
							else
							{
								filteredPath = optimizedLocalPath;
							}

							if(filteredPath.size() > 0)
							{
								// add current node to poses
								filteredPath.insert(std::make_pair(signature->id(), _optimizedPoses.at(signature->id())));
								//The nearest will be the reference for a loop closure transform
								if(signature->getLinks().find(nearestId) == signature->getLinks().end())
								{
									++localScanPathsChecked;
									RegistrationInfo info;
									Transform transform;
									bool icpMulti = true;
									if(_globalScanMap.empty())
									{
										transform = _memory->computeIcpTransformMulti(signature->id(), nearestId, filteredPath, &info);
									}
									else
									{
										UASSERT_MSG(_globalScanMapPoses.find(nearestId) != _globalScanMapPoses.end(), uFormat("Pose of %d not found in global scan poses", nearestId).c_str());
										icpMulti = false;
										// use pre-assembled scan map
										SensorData assembledData;
										assembledData.setId(nearestId);
										assembledData.setLaserScan(
												LaserScan(_globalScanMap,
													signature->sensorData().laserScanCompressed().maxPoints(),
													signature->sensorData().laserScanCompressed().rangeMax(),
													_globalScanMapPoses.at(nearestId).inverse() * (signature->sensorData().laserScanCompressed().is2d()?Transform(0,0,signature->sensorData().laserScanCompressed().localTransform().z(),0,0,0):Transform::getIdentity())));
										Signature nearestNode(assembledData);
										Transform guess = filteredPath.at(nearestId).inverse() * filteredPath.at(signature->id());
										transform = _memory->computeIcpTransform(nearestNode, *signature, guess, &info);
										if(!transform.isNull())
										{
											transform = transform.inverse();
										}
									}

									if(!transform.isNull())
									{
										UINFO("[Scan matching] Add local loop closure in SPACE (%d->%d) %s",
												signature->id(),
												nearestId,
												transform.prettyPrint().c_str());

										cv::Mat scanMatchingIds;
										if(_scanMatchingIdsSavedInLinks)
										{
											std::stringstream stream;
											stream << "SCANS:";
											for(std::map<int, Transform>::iterator iter=optimizedLocalPath.begin(); iter!=optimizedLocalPath.end(); ++iter)
											{
												if(iter != optimizedLocalPath.begin())
												{
													stream << ";";
												}
												stream << uNumber2Str(iter->first);
											}
											std::string scansStr = stream.str();
											scanMatchingIds = cv::Mat(1, int(scansStr.size()+1), CV_8SC1, (void *)scansStr.c_str());
											scanMatchingIds = compressData2(scanMatchingIds); // compressed
										}

										// set Identify covariance for laser scan matching only
										UASSERT(info.covariance.at<double>(0,0) > 0.0 && info.covariance.at<double>(5,5) > 0.0);
										_memory->addLink(Link(signature->id(), nearestId, Link::kLocalSpaceClosure, transform, getInformation(info.covariance)/_proximityMergedScanCovFactor, scanMatchingIds));
										loopClosureLinksAdded.push_back(std::make_pair(signature->id(), nearestId));

										if(icpMulti)
										{
											++proximityDetectionsAddedByICPMulti;
										}
										else
										{
											++proximityDetectionsAddedByICPGlobal;
										}

										// no local loop closure added visually
										if(proximityDetectionsAddedVisually == 0)
										{
											lastProximitySpaceClosureId = nearestId;
										}
									}
									else
									{
										UINFO("Local scan matching rejected: %s", info.rejectedMsg.c_str());
									}
									if(!_globalScanMap.empty())
									{
										break;
									}
								}
							}
						}
						else
						{
							//UDEBUG("Path %d ignored", nearestId);
						}
					}
				}
			}
		}
	}
	timeProximityBySpaceDetection = timer.ticks();
	ULOGGER_INFO("timeProximityBySpaceDetection=%fs", timeProximityBySpaceDetection);

	//=============================================================
	// Global loop closure detection
	// (updated: place this after retrieval to be sure that neighbors of the loop closure are in RAM)
	//=============================================================
	if(_loopClosureHypothesis.first>0)
	{
		//Compute transform if metric data are present
		Transform transform;
		RegistrationInfo info;
		info.covariance = cv::Mat::eye(6,6,CV_64FC1);
		if(_rgbdSlamMode)
		{
			transform = _memory->computeTransform(
					_loopClosureHypothesis.first,
					signature->id(),
					_loopClosureIdentityGuess?Transform::getIdentity():Transform(),
					&info);

			loopClosureVisualInliersMeanDist = info.inliersMeanDistance;
			loopClosureVisualInliersDistribution = info.inliersDistribution;

			loopClosureVisualInliers = info.inliers;
			loopClosureVisualInliersRatio = info.inliersRatio;
			loopClosureVisualMatches = info.matches;
			rejectedGlobalLoopClosure = transform.isNull();
			if(rejectedGlobalLoopClosure)
			{
				UWARN("Rejected loop closure %d -> %d: %s",
						_loopClosureHypothesis.first, signature->id(), info.rejectedMsg.c_str());
			}
			else if(_maxLoopClosureDistance>0.0f && transform.getNorm() > _maxLoopClosureDistance)
			{
				rejectedGlobalLoopClosure = true;
				UWARN("Rejected localization %d -> %d because distance to map (%fm) is over %s=%fm.",
						_loopClosureHypothesis.first, signature->id(), transform.getNorm(), Parameters::kRGBDMaxLoopClosureDistance().c_str(), _maxLoopClosureDistance);
			}
			else
			{
				transform = transform.inverse();
			}
		}
		if(!rejectedGlobalLoopClosure)
		{
			// Make the new one the parent of the old one
			UASSERT(info.covariance.at<double>(0,0) > 0.0 && info.covariance.at<double>(5,5) > 0.0);
			cv::Mat information = getInformation(info.covariance);
			loopClosureLinearVariance = 1.0/information.at<double>(0,0);
			loopClosureAngularVariance = 1.0/information.at<double>(5,5);
			rejectedGlobalLoopClosure = !_memory->addLink(Link(signature->id(), _loopClosureHypothesis.first, Link::kGlobalClosure, transform, information));
			if(!rejectedGlobalLoopClosure)
			{
				loopClosureLinksAdded.push_back(std::make_pair(signature->id(), _loopClosureHypothesis.first));
			}
		}

		if(rejectedGlobalLoopClosure)
		{
			_loopClosureHypothesis.first = 0;
		}
	}

	timeAddLoopClosureLink = timer.ticks();
	ULOGGER_INFO("timeAddLoopClosureLink=%fs", timeAddLoopClosureLink);

	//============================================================
	// Add virtual links if a path is activated
	//============================================================
	if(_path.size())
	{
		// Add a virtual loop closure link to keep the path linked to local map
		if( signature->id() != _path[_pathCurrentIndex].first &&
			!signature->hasLink(_path[_pathCurrentIndex].first))
		{
			UASSERT(uContains(_optimizedPoses, signature->id()));
			UASSERT_MSG(uContains(_optimizedPoses, _path[_pathCurrentIndex].first), uFormat("id=%d", _path[_pathCurrentIndex].first).c_str());
			Transform virtualLoop = _optimizedPoses.at(signature->id()).inverse() * _optimizedPoses.at(_path[_pathCurrentIndex].first);

			if(_localRadius == 0.0f || virtualLoop.getNorm() < _localRadius)
			{
				_memory->addLink(Link(signature->id(), _path[_pathCurrentIndex].first, Link::kVirtualClosure, virtualLoop, cv::Mat::eye(6,6,CV_64FC1)*0.01)); // set high variance
			}
			else
			{
				UERROR("Virtual link larger than local radius (%fm > %fm). Aborting the plan!",
						virtualLoop.getNorm(), _localRadius);
				this->clearPath(-1);
			}
		}
	}

	//============================================================
	// Optimize map graph
	//============================================================
	float maxLinearError = 0.0f;
	float maxLinearErrorRatio = 0.0f;
	float maxAngularError = 0.0f;
	float maxAngularErrorRatio = 0.0f;
	double optimizationError = 0.0;
	int optimizationIterations = 0;
	cv::Mat localizationCovariance;
	Transform previousMapCorrection;
	bool rejectedLandmark = false;
	UDEBUG("RGB-D SLAM mode: %d", _rgbdSlamMode?1:0);
	UDEBUG("Incremental: %d", _memory->isIncremental());
	UDEBUG("Loop hyp: %d", _loopClosureHypothesis.first);
	UDEBUG("Last prox: %d", lastProximitySpaceClosureId);
	UDEBUG("Reduced ids: %d", (int)statistics_.reducedIds().size());
	UDEBUG("Has prior: %d (prior ignored=%d)", signature->hasLink(signature->id(), Link::kPosePrior)?1:0, _graphOptimizer->priorsIgnored()?1:0);
	UDEBUG("Has gravity: %d (sigma=%f, odomGravity=%d, refined=%d)", signature->hasLink(signature->id(), Link::kGravity)?1:0, _graphOptimizer->gravitySigma(), _memory->isOdomGravityUsed()?1:0, neighborLinkRefined?1:0);
	UDEBUG("Has virtual link: %d", (int)graph::filterLinks(signature->getLinks(), Link::kVirtualClosure, true).size());
	UDEBUG("Prox Time: %d", proximityDetectionsInTimeFound);
	UDEBUG("Landmarks: %d", (int)landmarksDetected.size());
	UDEBUG("Retrieved: %d", (int)signaturesRetrieved.size());
	UDEBUG("Not self ref links: %d", (int)graph::filterLinks(signature->getLinks(), Link::kSelfRefLink).size());

	if(_rgbdSlamMode
		&&
		(_loopClosureHypothesis.first>0 ||
	     lastProximitySpaceClosureId>0 || // can be different map of the current one
	     statistics_.reducedIds().size() ||
		 (signature->hasLink(signature->id(), Link::kPosePrior) && !_graphOptimizer->priorsIgnored()) || // prior edge
		 (signature->hasLink(signature->id(), Link::kGravity) && _graphOptimizer->gravitySigma()>0.0f && (!_memory->isOdomGravityUsed() || neighborLinkRefined)) || // gravity edge
	     proximityDetectionsInTimeFound>0 ||
		 !landmarksDetected.empty() ||
		 signaturesRetrieved.size()) // can be different map of the current one
		 &&
		 (_memory->isIncremental() ||
		  // In localization mode, the new node should be linked to another node or a landmark already in the working memory
		  graph::filterLinks(graph::filterLinks(signature->getLinks(), Link::kVirtualClosure), Link::kSelfRefLink).size() ||
		  !landmarksDetected.empty()))
	{
		UASSERT(uContains(_optimizedPoses, signature->id()));

		//used in localization mode: filter virtual links
		std::multimap<int, Link> localizationLinks = graph::filterLinks(signature->getLinks(), Link::kVirtualClosure);
		localizationLinks = graph::filterLinks(localizationLinks, Link::kSelfRefLink);
		if(!landmarksDetected.empty() && !_memory->isIncremental())
		{
			for(std::map<int, std::set<int> >::iterator iter=landmarksDetected.begin(); iter!=landmarksDetected.end(); ++iter)
			{
				if(_optimizedPoses.find(iter->first)!=_optimizedPoses.end())
				{
					UASSERT(uContains(signature->getLandmarks(), iter->first));
					localizationLinks.insert(std::make_pair(iter->first, signature->getLandmarks().at(iter->first)));
				}
			}
		}

		// Note that in localization mode, we don't re-optimize the graph
		// if:
		//  1- there are no signatures retrieved,
		//  2- we are relocalizing on a node already in the optimized graph
		if(!_memory->isIncremental() &&
		   signaturesRetrieved.empty() &&
		   !localizationLinks.empty() &&
		   uContains(_optimizedPoses, localizationLinks.rbegin()->first))
		{
			bool rejectLocalization = _odomCachePoses.empty();
			if(!_odomCachePoses.empty())
			{
				// Verify if the new localization is valid by checking if there is
				// not too much deformation using current odometry poses
				// This will also refine localization links

				std::map<int, Transform> poses = _odomCachePoses;
				std::multimap<int, Link> constraints = _odomCacheConstraints;
				// add self referring links (e.g., gravity)
				std::multimap<int, Link> selfLinks = graph::filterLinks(signature->getLinks(), Link::kSelfRefLink, true);
				if(_graphOptimizer->priorsIgnored())
				{
					selfLinks = graph::filterLinks(selfLinks, Link::kPosePrior);
				}
				constraints.insert(selfLinks.begin(), selfLinks.end());
				for(std::multimap<int, Link>::iterator iter=localizationLinks.begin(); iter!=localizationLinks.end(); ++iter)
				{
					constraints.insert(std::make_pair(iter->second.from(), iter->second));
				}
				for(std::multimap<int, Link>::iterator iter=constraints.begin(); iter!=constraints.end(); ++iter)
				{
					std::map<int, Transform>::iterator iterPose = _optimizedPoses.find(iter->second.to());
					if(iterPose != _optimizedPoses.end() && poses.find(iterPose->first) == poses.end())
					{
						poses.insert(*iterPose);
						// make the poses in the map fixed
						constraints.insert(std::make_pair(iterPose->first, Link(iterPose->first, iterPose->first, Link::kPosePrior, iterPose->second, cv::Mat::eye(6,6, CV_64FC1)*1000000)));
						UDEBUG("Constraint %d->%d (type=%s)", iterPose->first, iterPose->first, Link::typeName(Link::kPosePrior).c_str());
					}
					UDEBUG("Constraint %d->%d (type=%s, var = %f %f)", iter->second.from(), iter->second.to(), iter->second.typeName().c_str(), iter->second.transVariance(), iter->second.rotVariance());
				}
				for(std::map<int, Transform>::iterator iter=poses.begin(); iter!=poses.end(); ++iter)
				{
					UDEBUG("Pose %d %s", iter->first, iter->second.prettyPrint().c_str());
				}


				std::map<int, Transform> posesOut;
				std::multimap<int, Link> edgeConstraintsOut;
				bool priorsIgnored = _graphOptimizer->priorsIgnored();
				UDEBUG("priorsIgnored was %s", priorsIgnored?"true":"false");
				_graphOptimizer->setPriorsIgnored(false); //temporary set false to use priors above to fix nodes of the map
				// If slam2d: get connected graph while keeping original roll,pitch,z values.
				_graphOptimizer->getConnectedGraph(signature->id(), poses, constraints, posesOut, edgeConstraintsOut, !_graphOptimizer->isSlam2d());
				std::map<int, Transform> optPoses = _graphOptimizer->optimize(poses.begin()->first, posesOut, edgeConstraintsOut);
				_graphOptimizer->setPriorsIgnored(priorsIgnored); // set back
				for(std::map<int, Transform>::iterator iter=optPoses.begin(); iter!=optPoses.end(); ++iter)
				{
					UDEBUG("Opt  %d %s", iter->first, iter->second.prettyPrint().c_str());
				}

				if(optPoses.empty())
				{
					UWARN("Optimization failed, rejecting localization!");
					rejectLocalization = true;
				}
				else if(_optimizationMaxError > 0.0f)
				{
					UINFO("Compute max graph errors...");
					const Link * maxLinearLink = 0;
					const Link * maxAngularLink = 0;
					graph::computeMaxGraphErrors(
							optPoses,
							edgeConstraintsOut,
							maxLinearErrorRatio,
							maxAngularErrorRatio,
							maxLinearError,
							maxAngularError,
							&maxLinearLink,
							&maxAngularLink,
							_graphOptimizer->isSlam2d());
					if(maxLinearLink == 0 && maxAngularLink==0 && _maxOdomCacheSize>0)
					{
						UWARN("Could not compute graph errors! Wrong loop closures could be accepted!");
						optPoses = posesOut;
					}

					if(maxLinearLink)
					{
						UINFO("Max optimization linear error = %f m (link %d->%d, var=%f, ratio error/std=%f, thr=%f)",
								maxLinearError,
								maxLinearLink->from(),
								maxLinearLink->to(),
								maxLinearLink->transVariance(),
								maxLinearError/sqrt(maxLinearLink->transVariance()),
								_optimizationMaxError);
						if(maxLinearErrorRatio > _optimizationMaxError)
						{
							UWARN("Rejecting localization (%d <-> %d) in this "
									"iteration because a wrong loop closure has been "
									"detected after graph optimization, resulting in "
									"a maximum graph error ratio of %f (edge %d->%d, type=%d, abs error=%f m, stddev=%f). The "
									"maximum error ratio parameter \"%s\" is %f of std deviation.",
									localizationLinks.rbegin()->second.from(),
									localizationLinks.rbegin()->second.to(),
									maxLinearErrorRatio,
									maxLinearLink->from(),
									maxLinearLink->to(),
									maxLinearLink->type(),
									maxLinearError,
									sqrt(maxLinearLink->transVariance()),
									Parameters::kRGBDOptimizeMaxError().c_str(),
									_optimizationMaxError);
							rejectLocalization = true;
						}
					}
					if(maxAngularLink)
					{
						UINFO("Max optimization angular error = %f deg (link %d->%d, var=%f, ratio error/std=%f, thr=%f)",
								maxAngularError*180.0f/CV_PI,
								maxAngularLink->from(),
								maxAngularLink->to(),
								maxAngularLink->rotVariance(),
								maxAngularError/sqrt(maxAngularLink->rotVariance()),
								_optimizationMaxError);
						if(maxAngularErrorRatio > _optimizationMaxError)
						{
							UWARN("Rejecting localization (%d <-> %d) in this "
									"iteration because a wrong loop closure has been "
									"detected after graph optimization, resulting in "
									"a maximum graph error ratio of %f (edge %d->%d, type=%d, abs error=%f deg, stddev=%f). The "
									"maximum error ratio parameter \"%s\" is %f of std deviation.",
									localizationLinks.rbegin()->second.from(),
									localizationLinks.rbegin()->second.to(),
									maxAngularErrorRatio,
									maxAngularLink->from(),
									maxAngularLink->to(),
									maxAngularLink->type(),
									maxAngularError*180.0f/CV_PI,
									sqrt(maxAngularLink->rotVariance()),
									Parameters::kRGBDOptimizeMaxError().c_str(),
									_optimizationMaxError);
							rejectLocalization = true;
						}
					}
				}

				bool hasGlobalLoopClosuresOrLandmarks = false;
				if(rejectLocalization)
				{
					// Let's try again without local loop closures
					localizationLinks = graph::filterLinks(localizationLinks, Link::kLocalSpaceClosure);
					constraints = graph::filterLinks(constraints, Link::kLocalSpaceClosure);
					for(std::multimap<int, Link>::iterator iter=constraints.begin(); iter!=constraints.end() && !hasGlobalLoopClosuresOrLandmarks; ++iter)
					{
						hasGlobalLoopClosuresOrLandmarks =
								iter->second.type() == Link::kGlobalClosure ||
								iter->second.type() == Link::kLandmark;
					}
					if(hasGlobalLoopClosuresOrLandmarks && !localizationLinks.empty())
					{
						rejectLocalization = false;
						UWARN("Global and loop closures seem not tallying together, try again to optimize without local loop closures...");
						priorsIgnored = _graphOptimizer->priorsIgnored();
						UDEBUG("priorsIgnored was %s", priorsIgnored?"true":"false");
						_graphOptimizer->setPriorsIgnored(false); //temporary set false to use priors above to fix nodes of the map
						// If slam2d: get connected graph while keeping original roll,pitch,z values.
						_graphOptimizer->getConnectedGraph(signature->id(), poses, constraints, posesOut, edgeConstraintsOut, !_graphOptimizer->isSlam2d());
						optPoses = _graphOptimizer->optimize(poses.begin()->first, posesOut, edgeConstraintsOut);
						_graphOptimizer->setPriorsIgnored(priorsIgnored); // set back
						for(std::map<int, Transform>::iterator iter=optPoses.begin(); iter!=optPoses.end(); ++iter)
						{
							UDEBUG("Opt2  %d %s", iter->first, iter->second.prettyPrint().c_str());
						}

						if(optPoses.empty())
						{
							UWARN("Optimization failed, rejecting localization!");
							rejectLocalization = true;
						}
						else if(_optimizationMaxError > 0.0f)
						{
							UINFO("Compute max graph errors...");
							const Link * maxLinearLink = 0;
							const Link * maxAngularLink = 0;
							graph::computeMaxGraphErrors(
									optPoses,
									edgeConstraintsOut,
									maxLinearErrorRatio,
									maxAngularErrorRatio,
									maxLinearError,
									maxAngularError,
									&maxLinearLink,
									&maxAngularLink,
									_graphOptimizer->isSlam2d());
							if(maxLinearLink == 0 && maxAngularLink==0 && _maxOdomCacheSize>0)
							{
								UWARN("Could not compute graph errors! Wrong loop closures could be accepted!");
								optPoses = posesOut;
							}

							if(maxLinearLink)
							{
								UINFO("Max optimization linear error = %f m (link %d->%d, var=%f, ratio error/std=%f, thr=%f)",
										maxLinearError,
										maxLinearLink->from(),
										maxLinearLink->to(),
										maxLinearLink->transVariance(),
										maxLinearError/sqrt(maxLinearLink->transVariance()),
										_optimizationMaxError);
								if(maxLinearErrorRatio > _optimizationMaxError)
								{
									UWARN("Rejecting localization (%d <-> %d) in this "
											"iteration because a wrong loop closure has been "
											"detected after graph optimization, resulting in "
											"a maximum graph error ratio of %f (edge %d->%d, type=%d, abs error=%f m, stddev=%f). The "
											"maximum error ratio parameter \"%s\" is %f of std deviation.",
											localizationLinks.rbegin()->second.from(),
											localizationLinks.rbegin()->second.to(),
											maxLinearErrorRatio,
											maxLinearLink->from(),
											maxLinearLink->to(),
											maxLinearLink->type(),
											maxLinearError,
											sqrt(maxLinearLink->transVariance()),
											Parameters::kRGBDOptimizeMaxError().c_str(),
											_optimizationMaxError);
									rejectLocalization = true;
								}
							}
							if(maxAngularLink)
							{
								UINFO("Max optimization angular error = %f deg (link %d->%d, var=%f, ratio error/std=%f, thr=%f)",
										maxAngularError*180.0f/CV_PI,
										maxAngularLink->from(),
										maxAngularLink->to(),
										maxAngularLink->rotVariance(),
										maxAngularError/sqrt(maxAngularLink->rotVariance()),
										_optimizationMaxError);
								if(maxAngularErrorRatio > _optimizationMaxError)
								{
									UWARN("Rejecting localization (%d <-> %d) in this "
											"iteration because a wrong loop closure has been "
											"detected after graph optimization, resulting in "
											"a maximum graph error ratio of %f (edge %d->%d, type=%d, abs error=%f deg, stddev=%f). The "
											"maximum error ratio parameter \"%s\" is %f of std deviation.",
											localizationLinks.rbegin()->second.from(),
											localizationLinks.rbegin()->second.to(),
											maxAngularErrorRatio,
											maxAngularLink->from(),
											maxAngularLink->to(),
											maxAngularLink->type(),
											maxAngularError*180.0f/CV_PI,
											sqrt(maxAngularLink->rotVariance()),
											Parameters::kRGBDOptimizeMaxError().c_str(),
											_optimizationMaxError);
									rejectLocalization = true;
								}
							}
						}
					}
				}

				if(!rejectLocalization)
				{
					if(hasGlobalLoopClosuresOrLandmarks)
					{
						// We successfully optimize the graph without local loop closures,
						// clear them as some of them may be wrong.
						size_t before = _odomCacheConstraints.size();
						_odomCacheConstraints = graph::filterLinks(_odomCacheConstraints, Link::kLocalSpaceClosure);
						if(before != _odomCacheConstraints.size())
						{
							UWARN("Successfully optimized without local loop closures! Clear them from local odometry cache. %ld/%ld have been removed.",
									before - _odomCacheConstraints.size(), before);
						}
						else
						{
							UWARN("Successfully optimized without local loop closures!");
						}
					}

					// Count how many localization links are in the constraints
					bool hadAlreadyLocalizationLinks = false;
					for(std::multimap<int, Link>::iterator iter=_odomCacheConstraints.begin();
							iter!=_odomCacheConstraints.end(); ++iter)
					{
						if(iter->second.type() == Link::kGlobalClosure ||
						   iter->second.type() == Link::kLocalSpaceClosure ||
						   iter->second.type() == Link::kLocalTimeClosure ||
						   iter->second.type() == Link::kUserClosure ||
						   iter->second.type() == Link::kNeighborMerged ||
						   iter->second.type() == Link::kLandmark)
						{
							hadAlreadyLocalizationLinks = true;
							break;
						}
					}

					// update localization links
					Transform newOptPoseInv = optPoses.at(signature->id()).inverse();
					for(std::multimap<int, Link>::iterator iter=localizationLinks.begin(); iter!=localizationLinks.end(); ++iter)
					{
						Transform newT = newOptPoseInv * optPoses.at(iter->first);
						UDEBUG("Adjusted localization link %d->%d after optimization", iter->second.from(), iter->second.to());
						UDEBUG("from %s", iter->second.transform().prettyPrint().c_str());
						UDEBUG("  to %s", newT.prettyPrint().c_str());
						iter->second.setTransform(newT);

						_odomCacheConstraints.insert(std::make_pair(signature->id(), iter->second));
					}
					_odomCacheConstraints.insert(selfLinks.begin(), selfLinks.end());

					// At least 2 localizations at 2 different time required
					if(hadAlreadyLocalizationLinks || _maxOdomCacheSize == 0)
					{
						UINFO("Update localization");
						if(_optimizeFromGraphEnd)
						{
							// update all previous nodes
							// Normally _mapCorrection should be identity, but if _optimizeFromGraphEnd
							// parameters just changed state, we should put back all poses without map correction.
							Transform oldPose = _optimizedPoses.at(localizationLinks.rbegin()->first);
							Transform mapCorrectionInv = _mapCorrection.inverse();
							Transform u = signature->getPose() * localizationLinks.rbegin()->second.transform();
							if(_graphOptimizer->gravitySigma() > 0)
							{
                                // Adjust transform with gravity
                                Transform transform = localizationLinks.rbegin()->second.transform();
                                int loopId = localizationLinks.rbegin()->first;
                                int landmarkId = 0;
                                if(loopId<0)
                                {
                                    //For landmarks, use transform against other node looking the landmark
                                    // (because we don't assume that landmarks are aligned with gravity)
                                    landmarkId = loopId;
                                    UASSERT(landmarksDetected.find(landmarkId) != landmarksDetected.end() &&
                                            !landmarksDetected.at(landmarkId).empty());
                                    loopId = *landmarksDetected.at(landmarkId).begin();
                                }
                                
                                const Signature * loopS = _memory->getSignature(loopId);
                                UASSERT(loopS !=0);
                                std::multimap<int, Link>::const_iterator iterGravityLoop = graph::findLink(loopS->getLinks(), loopS->id(), loopS->id(), false, Link::kGravity);
                                std::multimap<int, Link>::const_iterator iterGravitySign = graph::findLink(signature->getLinks(), signature->id(), signature->id(), false, Link::kGravity);
                                if(iterGravityLoop!=loopS->getLinks().end() &&
                                   iterGravitySign!=signature->getLinks().end())
                                {
                                    float roll,pitch,yaw;
                                    if(landmarkId < 0)
                                    {
                                        iterGravityLoop->second.transform().getEulerAngles(roll, pitch, yaw);
                                        Transform gravityCorr = Transform(_optimizedPoses.at(loopS->id()).x(),
                                                                              _optimizedPoses.at(loopS->id()).y(),
                                                                              _optimizedPoses.at(loopS->id()).z(),
                                                                              roll, pitch, _optimizedPoses.at(loopS->id()).theta()) * _optimizedPoses.at(loopS->id()).inverse();
                                        (gravityCorr * _optimizedPoses.at(landmarkId)).getEulerAngles(roll,pitch,yaw);
                                    }
                                    else
                                    {
                                        iterGravityLoop->second.transform().getEulerAngles(roll, pitch, yaw);
                                    }
                                    Transform targetRotation = iterGravitySign->second.transform().rotation()*transform.rotation();
                                    targetRotation = Transform(0,0,0,roll,pitch,targetRotation.theta());
                                    Transform error = transform.rotation().inverse() * iterGravitySign->second.transform().rotation().inverse() * targetRotation;
                                    transform *= error;
                                    u  = signature->getPose() * transform;
                                }
                                else if(iterGravityLoop!=loopS->getLinks().end() ||
                                        iterGravitySign!=signature->getLinks().end())
                                {
                                    UWARN("Gravity link not found for %d or %d, localization won't be corrected with gravity.", loopId, signature->id());
                                }
							}
							Transform up = u * oldPose.inverse();
							if(_graphOptimizer->isSlam2d())
							{
								// in case of 3d landmarks, transform constraint to 2D
								up.to3DoF();
							}
							for(std::map<int, Transform>::iterator iter=_optimizedPoses.begin(); iter!=_optimizedPoses.end(); ++iter)
							{
								iter->second = mapCorrectionInv * up * iter->second;
							}
							_optimizedPoses.at(signature->id()) = signature->getPose();
						}
						else
						{
							Transform newPose = _optimizedPoses.at(localizationLinks.rbegin()->first) * localizationLinks.rbegin()->second.transform().inverse();
							UDEBUG("newPose=%s", newPose.prettyPrint().c_str());
							if(_graphOptimizer->isSlam2d() && signature->getPose().is3DoF())
							{
								// in case of 3d landmarks, transform constraint to 2D
								newPose = newPose.to3DoF();
								UDEBUG("newPose 2D=%s", newPose.prettyPrint().c_str());
							}
							else if(_graphOptimizer->gravitySigma() > 0)
							{
								// Adjust transform with gravity
								std::multimap<int, Link>::const_iterator iterGravitySign = graph::findLink(signature->getLinks(), signature->id(), signature->id(), false, Link::kGravity);
								if(iterGravitySign!=signature->getLinks().end())
								{
                                    Transform transform = localizationLinks.rbegin()->second.transform();
									float roll,pitch,yaw;
									float tmp1,tmp2;
									UDEBUG("Gravity link = %s", iterGravitySign->second.transform().prettyPrint().c_str());
                                    _optimizedPoses.at(localizationLinks.rbegin()->first).getEulerAngles(roll, pitch, yaw);
                                    Transform targetRotation = iterGravitySign->second.transform().rotation()*transform.rotation();
                                    targetRotation = Transform(0,0,0,roll,pitch,targetRotation.theta());
                                    Transform error = transform.rotation().inverse() * iterGravitySign->second.transform().rotation().inverse() * targetRotation;
                                    transform *= error;
                                    newPose = _optimizedPoses.at(localizationLinks.rbegin()->first) * transform.inverse();
                                    iterGravitySign->second.transform().getEulerAngles(roll, pitch, tmp1);
                                    newPose.getEulerAngles(tmp1, tmp2, yaw);
                                    newPose = Transform(newPose.x(), newPose.y(), newPose.z(), roll, pitch, yaw);
									UDEBUG("newPose gravity=%s", newPose.prettyPrint().c_str());
								}
								else if(iterGravitySign!=signature->getLinks().end())
								{
									UWARN("Gravity link not found for %d, localization won't be corrected with gravity.", signature->id());
								}
							}
							_optimizedPoses.at(signature->id()) = newPose;
						}
						localizationCovariance = localizationLinks.rbegin()->second.infMatrix().inv();
					}
					else //delayed localization (wait for more than 1 link)
					{
						UWARN("Localization was good, but waiting for another one to be more accurate (%s>0)", Parameters::kRGBDMaxOdomCacheSize().c_str());
						rejectLocalization = true;
					}
				}
			}

			if(rejectLocalization)
			{
				_loopClosureHypothesis.first = 0;
				lastProximitySpaceClosureId = 0;
				rejectedGlobalLoopClosure = true;
				rejectedLandmark = true;
			}
		}
		else
		{
			UINFO("Update map correction");
			std::map<int, Transform> poses = _optimizedPoses;

			// if _optimizeFromGraphEnd parameter just changed state, don't use optimized poses as guess
			if(_optimizeFromGraphEndChanged)
			{
				UWARN("Optimization: clearing guess poses as %s has changed state, now %s",
						Parameters::kRGBDOptimizeFromGraphEnd().c_str(), _optimizeFromGraphEnd?"true":"false");
				poses.clear();
				_optimizeFromGraphEndChanged = false;
			}

			std::multimap<int, Link> constraints;
			cv::Mat covariance;
			optimizeCurrentMap(signature->id(), false, poses, covariance, &constraints, &optimizationError, &optimizationIterations);

			// Check added loop closures have broken the graph
			// (in case of wrong loop closures).
			bool updateConstraints = true;
			if(poses.empty())
			{
				UWARN("Graph optimization failed! Rejecting last loop closures added.");
				for(std::list<std::pair<int, int> >::iterator iter=loopClosureLinksAdded.begin(); iter!=loopClosureLinksAdded.end(); ++iter)
				{
					_memory->removeLink(iter->first, iter->second);
					UWARN("Loop closure %d->%d rejected!", iter->first, iter->second);
				}
				updateConstraints = false;
				_loopClosureHypothesis.first = 0;
				lastProximitySpaceClosureId = 0;
				rejectedGlobalLoopClosure = true;
				rejectedLandmark = true;
			}
			else if(_memory->isIncremental() &&
			  _optimizationMaxError > 0.0f &&
			  loopClosureLinksAdded.size() &&
			  optimizationIterations > 0 &&
			  constraints.size())
			{
				UINFO("Compute max graph errors...");
				const Link * maxLinearLink = 0;
				const Link * maxAngularLink = 0;
				graph::computeMaxGraphErrors(
						poses,
						constraints,
						maxLinearErrorRatio,
						maxAngularErrorRatio,
						maxLinearError,
						maxAngularError,
						&maxLinearLink,
						&maxAngularLink);
				if(maxLinearLink == 0 && maxAngularLink==0)
				{
					UWARN("Could not compute graph errors! Wrong loop closures could be accepted!");
				}

				bool reject = false;
				if(maxLinearLink)
				{
					UINFO("Max optimization linear error = %f m (link %d->%d, var=%f, ratio error/std=%f)", maxLinearError, maxLinearLink->from(), maxLinearLink->to(), maxLinearLink->transVariance(), maxLinearError/sqrt(maxLinearLink->transVariance()));
					if(maxLinearErrorRatio > _optimizationMaxError)
					{
						UWARN("Rejecting all added loop closures (%d, first is %d <-> %d) in this "
							  "iteration because a wrong loop closure has been "
							  "detected after graph optimization, resulting in "
							  "a maximum graph error ratio of %f (edge %d->%d, type=%d, abs error=%f m, stddev=%f). The "
							  "maximum error ratio parameter \"%s\" is %f of std deviation.",
							  (int)loopClosureLinksAdded.size(),
							  loopClosureLinksAdded.front().first,
							  loopClosureLinksAdded.front().second,
							  maxLinearErrorRatio,
							  maxLinearLink->from(),
							  maxLinearLink->to(),
							  maxLinearLink->type(),
							  maxLinearError,
							  sqrt(maxLinearLink->transVariance()),
							  Parameters::kRGBDOptimizeMaxError().c_str(),
							  _optimizationMaxError);
						reject = true;
					}
				}
				if(maxAngularLink)
				{
					UINFO("Max optimization angular error = %f deg (link %d->%d, var=%f, ratio error/std=%f)", maxAngularError*180.0f/CV_PI, maxAngularLink->from(), maxAngularLink->to(), maxAngularLink->rotVariance(), maxAngularError/sqrt(maxAngularLink->rotVariance()));
					if(maxAngularErrorRatio > _optimizationMaxError)
					{
						UWARN("Rejecting all added loop closures (%d, first is %d <-> %d) in this "
							  "iteration because a wrong loop closure has been "
							  "detected after graph optimization, resulting in "
							  "a maximum graph error ratio of %f (edge %d->%d, type=%d, abs error=%f deg, stddev=%f). The "
							  "maximum error ratio parameter \"%s\" is %f of std deviation.",
							  (int)loopClosureLinksAdded.size(),
							  loopClosureLinksAdded.front().first,
							  loopClosureLinksAdded.front().second,
							  maxAngularErrorRatio,
							  maxAngularLink->from(),
							  maxAngularLink->to(),
							  maxAngularLink->type(),
							  maxAngularError*180.0f/CV_PI,
							  sqrt(maxAngularLink->rotVariance()),
							  Parameters::kRGBDOptimizeMaxError().c_str(),
							  _optimizationMaxError);
						reject = true;
					}
				}

				if(reject)
				{
					for(std::list<std::pair<int, int> >::iterator iter=loopClosureLinksAdded.begin(); iter!=loopClosureLinksAdded.end(); ++iter)
					{
						_memory->removeLink(iter->first, iter->second);
						UWARN("Loop closure %d->%d rejected!", iter->first, iter->second);
					}
					updateConstraints = false;
					_loopClosureHypothesis.first = 0;
					lastProximitySpaceClosureId = 0;
					rejectedGlobalLoopClosure = true;
					rejectedLandmark = true;
				}
			}

			if(updateConstraints)
			{
				UINFO("Updated local map (old size=%d, new size=%d)", (int)_optimizedPoses.size(), (int)poses.size());
				_optimizedPoses = poses;
				_constraints = constraints;
				localizationCovariance = covariance;
			}
		}

		// Update map correction, it should be identify when optimizing from the last node
		UASSERT(_optimizedPoses.find(signature->id()) != _optimizedPoses.end());
		if(fakeOdom && _mapCorrectionBackup.isNull())
		{
			_mapCorrectionBackup = _mapCorrection;
		}
		previousMapCorrection = _mapCorrection;
		_mapCorrection = _optimizedPoses.at(signature->id()) * signature->getPose().inverse();
		_lastLocalizationPose = _optimizedPoses.at(signature->id()); // update
		if(_mapCorrection.getNormSquared() > 0.1f && _optimizeFromGraphEnd)
		{
			bool hasPrior = signature->hasLink(signature->id());
			if(!_graphOptimizer->priorsIgnored())
			{
				for(std::multimap<int, Link>::reverse_iterator iter=_constraints.rbegin(); !hasPrior && iter!=_constraints.rend(); ++iter)
				{
					if(iter->second.type() == Link::kPosePrior)
					{
						hasPrior = true;
					}
				}
			}
			if((!hasPrior || _graphOptimizer->priorsIgnored()) && _graphOptimizer->gravitySigma()==0.0f)
			{
				UERROR("Map correction should be identity when optimizing from the last node. T=%s", _mapCorrection.prettyPrint().c_str());
			}
		}
	}
	int newLocId = _loopClosureHypothesis.first>0?_loopClosureHypothesis.first:lastProximitySpaceClosureId>0?lastProximitySpaceClosureId:0;
	_lastLocalizationNodeId = newLocId!=0?newLocId:_lastLocalizationNodeId;
	if(newLocId==0 && !landmarksDetected.empty())
	{
		std::map<int, std::set<int> >::const_iterator iter = _memory->getLandmarksIndex().find(landmarksDetected.begin()->first);
		if(iter!=_memory->getLandmarksIndex().end())
		{
			if(iter->second.size() && *iter->second.begin()!=signature->id())
			{
				_lastLocalizationNodeId = *iter->second.begin();
			}
		}
	}

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
	sLoop = _memory->getSignature(_loopClosureHypothesis.first?_loopClosureHypothesis.first:lastProximitySpaceClosureId?lastProximitySpaceClosureId:_highestHypothesis.first);
	if(sLoop)
	{
		lcHypothesisReactivated = sLoop->isSaved()?1.0f:0.0f;
	}
	dictionarySize = (int)_memory->getVWDictionary()->getVisualWords().size();
	refWordsCount = (int)signature->getWords().size();
	refUniqueWordsCount = (int)uUniqueKeys(signature->getWords()).size();

	// Posterior is empty if a bad signature is detected
	float vpHypothesis = posterior.size()?posterior.at(Memory::kIdVirtual):0.0f;

	// prepare statistics
	if(_loopClosureHypothesis.first || _publishStats)
	{
		ULOGGER_INFO("sending stats...");
		statistics_.setRefImageId(_memory->getLastSignatureId()); // Use last id from Memory (in case of rehearsal)
		statistics_.setRefImageMapId(signature->mapId());
		statistics_.setStamp(data.stamp());
		if(_loopClosureHypothesis.first != Memory::kIdInvalid)
		{
			statistics_.setLoopClosureId(_loopClosureHypothesis.first);
			statistics_.setLoopClosureMapId(_memory->getMapId(_loopClosureHypothesis.first));
			ULOGGER_INFO("Loop closure detected! With id=%d", _loopClosureHypothesis.first);
		}
		if(_publishStats)
		{
			ULOGGER_INFO("send all stats...");
			statistics_.setExtended(1);

			statistics_.addStatistic(Statistics::kLoopAccepted_hypothesis_id(), _loopClosureHypothesis.first);
			statistics_.addStatistic(Statistics::kLoopSuppressed_hypothesis_id(), loopIdSuppressedByProximity);
			statistics_.addStatistic(Statistics::kLoopHighest_hypothesis_id(), _highestHypothesis.first);
			statistics_.addStatistic(Statistics::kLoopHighest_hypothesis_value(), _highestHypothesis.second);
			statistics_.addStatistic(Statistics::kLoopHypothesis_reactivated(), lcHypothesisReactivated);
			statistics_.addStatistic(Statistics::kLoopVp_hypothesis(), vpHypothesis);
			statistics_.addStatistic(Statistics::kLoopReactivate_id(), retrievalId);
			statistics_.addStatistic(Statistics::kLoopHypothesis_ratio(), hypothesisRatio);
			statistics_.addStatistic(Statistics::kLoopVisual_inliers(), loopClosureVisualInliers);
			statistics_.addStatistic(Statistics::kLoopVisual_inliers_ratio(), loopClosureVisualInliersRatio);
			statistics_.addStatistic(Statistics::kLoopVisual_matches(), loopClosureVisualMatches);
			statistics_.addStatistic(Statistics::kLoopLinear_variance(), loopClosureLinearVariance);
			statistics_.addStatistic(Statistics::kLoopAngular_variance(), loopClosureAngularVariance);
			statistics_.addStatistic(Statistics::kLoopLast_id(), _memory->getLastGlobalLoopClosureId());
			statistics_.addStatistic(Statistics::kLoopOptimization_max_error(), maxLinearError);
			statistics_.addStatistic(Statistics::kLoopOptimization_max_error_ratio(), maxLinearErrorRatio);
			statistics_.addStatistic(Statistics::kLoopOptimization_max_ang_error(), maxAngularError*180.0f/M_PI);
			statistics_.addStatistic(Statistics::kLoopOptimization_max_ang_error_ratio(), maxAngularErrorRatio);
			statistics_.addStatistic(Statistics::kLoopOptimization_error(), optimizationError);
			statistics_.addStatistic(Statistics::kLoopOptimization_iterations(), optimizationIterations);
			statistics_.addStatistic(Statistics::kLoopLandmark_detected(), landmarksDetected.empty()?0:-landmarksDetected.begin()->first);
			statistics_.addStatistic(Statistics::kLoopLandmark_detected_node_ref(), landmarksDetected.empty() || landmarksDetected.begin()->second.empty()?0:*landmarksDetected.begin()->second.begin());
			statistics_.addStatistic(Statistics::kLoopVisual_inliers_mean_dist(), loopClosureVisualInliersMeanDist);
			statistics_.addStatistic(Statistics::kLoopVisual_inliers_distribution(), loopClosureVisualInliersDistribution);

			statistics_.addStatistic(Statistics::kProximityTime_detections(), proximityDetectionsInTimeFound);
			statistics_.addStatistic(Statistics::kProximitySpace_detections_added_visually(), proximityDetectionsAddedVisually);
			statistics_.addStatistic(Statistics::kProximitySpace_detections_added_icp_multi(), proximityDetectionsAddedByICPMulti);
			statistics_.addStatistic(Statistics::kProximitySpace_detections_added_icp_global(), proximityDetectionsAddedByICPGlobal);
			statistics_.addStatistic(Statistics::kProximitySpace_paths(), proximitySpacePaths);
			statistics_.addStatistic(Statistics::kProximitySpace_visual_paths_checked(), localVisualPathsChecked);
			statistics_.addStatistic(Statistics::kProximitySpace_scan_paths_checked(), localScanPathsChecked);
			statistics_.addStatistic(Statistics::kProximitySpace_last_detection_id(), lastProximitySpaceClosureId);
			statistics_.setProximityDetectionId(lastProximitySpaceClosureId);
			statistics_.setProximityDetectionMapId(_memory->getMapId(lastProximitySpaceClosureId));

			int loopId = _loopClosureHypothesis.first>0?_loopClosureHypothesis.first:lastProximitySpaceClosureId;
			statistics_.addStatistic(Statistics::kLoopId(), loopId);
			statistics_.addStatistic(Statistics::kLoopMap_id(), (loopId>0 && sLoop)?sLoop->mapId():-1);

			float x,y,z,roll,pitch,yaw;
			if(_loopClosureHypothesis.first || lastProximitySpaceClosureId || (!rejectedLandmark && !landmarksDetected.empty()))
			{
				if(_loopClosureHypothesis.first || lastProximitySpaceClosureId)
				{
					// Loop closure transform
					UASSERT(sLoop);
					std::multimap<int, Link>::const_iterator loopIter =  sLoop->getLinks().find(signature->id());
					UASSERT(loopIter!=sLoop->getLinks().end());
					UINFO("Set loop closure transform = %s", loopIter->second.transform().prettyPrint().c_str());
					statistics_.setLoopClosureTransform(loopIter->second.transform());

					statistics_.addStatistic(Statistics::kLoopVisual_words(), sLoop->getWords().size());

					// if ground truth exists, compute localization error
					if(!sLoop->getGroundTruthPose().isNull() && !signature->getGroundTruthPose().isNull())
					{
						Transform transformGT = sLoop->getGroundTruthPose().inverse() * signature->getGroundTruthPose();
						Transform error = loopIter->second.transform().inverse() * transformGT;
						statistics_.addStatistic(Statistics::kGtLocalization_linear_error(), error.getNorm());
						statistics_.addStatistic(Statistics::kGtLocalization_angular_error(), error.getAngle(1,0,0)*180/M_PI);
					}
				}
				statistics_.addStatistic(Statistics::kLoopDistance_since_last_loc(), _distanceTravelledSinceLastLocalization);
				_distanceTravelledSinceLastLocalization = 0.0f;

				statistics_.addStatistic(Statistics::kLoopMapToOdom_norm(), _mapCorrection.getNorm());
				statistics_.addStatistic(Statistics::kLoopMapToOdom_angle(), _mapCorrection.getAngle()*180.0f/M_PI);
				_mapCorrection.getTranslationAndEulerAngles(x, y, z, roll, pitch, yaw);
				statistics_.addStatistic(Statistics::kLoopMapToOdom_x(), x);
				statistics_.addStatistic(Statistics::kLoopMapToOdom_y(), y);
				statistics_.addStatistic(Statistics::kLoopMapToOdom_z(), z);
				statistics_.addStatistic(Statistics::kLoopMapToOdom_roll(),  roll*180.0f/M_PI);
				statistics_.addStatistic(Statistics::kLoopMapToOdom_pitch(),  pitch*180.0f/M_PI);
				statistics_.addStatistic(Statistics::kLoopMapToOdom_yaw(), yaw*180.0f/M_PI);

				// Odom correction (actual odometry pose change), ignore correction from first localization
				if(!odomPose.isNull() && !previousMapCorrection.isNull() && !previousMapCorrection.isIdentity())
				{
					Transform odomCorrection = (previousMapCorrection*odomPose).inverse()*_mapCorrection*odomPose;
					statistics_.addStatistic(Statistics::kLoopOdom_correction_norm(), odomCorrection.getNorm());
					statistics_.addStatistic(Statistics::kLoopOdom_correction_angle(), odomCorrection.getAngle()*180.0f/M_PI);
					odomCorrection.getTranslationAndEulerAngles(x, y, z, roll, pitch, yaw);
					statistics_.addStatistic(Statistics::kLoopOdom_correction_x(), x);
					statistics_.addStatistic(Statistics::kLoopOdom_correction_y(), y);
					statistics_.addStatistic(Statistics::kLoopOdom_correction_z(), z);
					statistics_.addStatistic(Statistics::kLoopOdom_correction_roll(),  roll*180.0f/M_PI);
					statistics_.addStatistic(Statistics::kLoopOdom_correction_pitch(),  pitch*180.0f/M_PI);
					statistics_.addStatistic(Statistics::kLoopOdom_correction_yaw(), yaw*180.0f/M_PI);

					_odomCorrectionAcc[0]+=x;
					_odomCorrectionAcc[1]+=y;
					_odomCorrectionAcc[2]+=z;
					_odomCorrectionAcc[3]+=roll;
					_odomCorrectionAcc[4]+=pitch;
					_odomCorrectionAcc[5]+=yaw;
					Transform odomCorrectionAcc(
							_odomCorrectionAcc[0],
							_odomCorrectionAcc[1],
							_odomCorrectionAcc[2],
							_odomCorrectionAcc[3],
							_odomCorrectionAcc[4],
							_odomCorrectionAcc[5]);

					statistics_.addStatistic(Statistics::kLoopOdom_correction_acc_norm(), odomCorrectionAcc.getNorm());
					statistics_.addStatistic(Statistics::kLoopOdom_correction_acc_angle(), odomCorrectionAcc.getAngle()*180.0f/M_PI);
					odomCorrectionAcc.getTranslationAndEulerAngles(x, y, z, roll, pitch, yaw);
					statistics_.addStatistic(Statistics::kLoopOdom_correction_acc_x(), x);
					statistics_.addStatistic(Statistics::kLoopOdom_correction_acc_y(), y);
					statistics_.addStatistic(Statistics::kLoopOdom_correction_acc_z(), z);
					statistics_.addStatistic(Statistics::kLoopOdom_correction_acc_roll(),  roll*180.0f/M_PI);
					statistics_.addStatistic(Statistics::kLoopOdom_correction_acc_pitch(),  pitch*180.0f/M_PI);
					statistics_.addStatistic(Statistics::kLoopOdom_correction_acc_yaw(), yaw*180.0f/M_PI);
				}
			}
			if(!_lastLocalizationPose.isNull() && !_lastLocalizationPose.isIdentity())
			{
				_lastLocalizationPose.getTranslationAndEulerAngles(x, y, z, roll, pitch, yaw);
				statistics_.addStatistic(Statistics::kLoopMapToBase_x(), x);
				statistics_.addStatistic(Statistics::kLoopMapToBase_y(), y);
				statistics_.addStatistic(Statistics::kLoopMapToBase_z(), z);
				statistics_.addStatistic(Statistics::kLoopMapToBase_roll(),  roll*180.0f/M_PI);
				statistics_.addStatistic(Statistics::kLoopMapToBase_pitch(),  pitch*180.0f/M_PI);
				statistics_.addStatistic(Statistics::kLoopMapToBase_yaw(), yaw*180.0f/M_PI);
				UINFO("Localization pose = %s", _lastLocalizationPose.prettyPrint().c_str());
			}

			statistics_.setMapCorrection(_mapCorrection);
			UINFO("Set map correction = %s", _mapCorrection.prettyPrint().c_str());
			statistics_.setLocalizationCovariance(localizationCovariance);

			// timings...
			statistics_.addStatistic(Statistics::kTimingMemory_update(), timeMemoryUpdate*1000);
			statistics_.addStatistic(Statistics::kTimingNeighbor_link_refining(), timeNeighborLinkRefining*1000);
			statistics_.addStatistic(Statistics::kTimingProximity_by_time(), timeProximityByTimeDetection*1000);
			statistics_.addStatistic(Statistics::kTimingProximity_by_space_visual(), timeProximityBySpaceVisualDetection*1000);
			statistics_.addStatistic(Statistics::kTimingProximity_by_space(), timeProximityBySpaceDetection*1000);
			statistics_.addStatistic(Statistics::kTimingReactivation(), timeReactivations*1000);
			statistics_.addStatistic(Statistics::kTimingAdd_loop_closure_link(), timeAddLoopClosureLink*1000);
			statistics_.addStatistic(Statistics::kTimingMap_optimization(), timeMapOptimization*1000);
			statistics_.addStatistic(Statistics::kTimingLikelihood_computation(), timeLikelihoodCalculation*1000);
			statistics_.addStatistic(Statistics::kTimingPosterior_computation(), timePosteriorCalculation*1000);
			statistics_.addStatistic(Statistics::kTimingHypotheses_creation(), timeHypothesesCreation*1000);
			statistics_.addStatistic(Statistics::kTimingHypotheses_validation(), timeHypothesesValidation*1000);
			statistics_.addStatistic(Statistics::kTimingCleaning_neighbors(), timeCleaningNeighbors*1000);

			// retrieval
			statistics_.addStatistic(Statistics::kMemorySignatures_retrieved(), (float)signaturesRetrieved.size());

			// Feature specific parameters
			statistics_.addStatistic(Statistics::kKeypointDictionary_size(), dictionarySize);
			statistics_.addStatistic(Statistics::kKeypointCurrent_frame(), refWordsCount);
			statistics_.addStatistic(Statistics::kKeypointIndexed_words(), _memory->getVWDictionary()->getIndexedWordsCount());
			statistics_.addStatistic(Statistics::kKeypointIndex_memory_usage(), _memory->getVWDictionary()->getIndexMemoryUsed());

			//Epipolar geometry constraint
			statistics_.addStatistic(Statistics::kLoopRejectedHypothesis(), rejectedGlobalLoopClosure?1.0f:0);

			statistics_.addStatistic(Statistics::kMemorySmall_movement(), smallDisplacement?1.0f:0);
			statistics_.addStatistic(Statistics::kMemoryDistance_travelled(), _distanceTravelled);
			statistics_.addStatistic(Statistics::kMemoryFast_movement(), tooFastMovement?1.0f:0);
			if(_publishRAMUsage)
			{
				UTimer ramTimer;
				statistics_.addStatistic(Statistics::kMemoryRAM_usage(), UProcessInfo::getMemoryUsage()/(1024*1024));
				long estimatedMemoryUsage = sizeof(Rtabmap);
				estimatedMemoryUsage += _optimizedPoses.size() * (sizeof(int) + sizeof(Transform) + 12 * sizeof(float) + sizeof(std::map<int, Transform>::iterator)) + sizeof(std::map<int, Transform>);
				estimatedMemoryUsage += _constraints.size() * (sizeof(int) + sizeof(Transform) + 12 * sizeof(float) + sizeof(cv::Mat) + 36 * sizeof(double) + sizeof(std::map<int, Link>::iterator)) + sizeof(std::map<int, Link>);
				estimatedMemoryUsage += _memory->getMemoryUsed();
				estimatedMemoryUsage += _bayesFilter->getMemoryUsed();
				estimatedMemoryUsage += _parameters.size()*(sizeof(std::string)*2+sizeof(ParametersMap::iterator)) + sizeof(ParametersMap);
				statistics_.addStatistic(Statistics::kMemoryRAM_estimated(), (float)(estimatedMemoryUsage/(1024*1024)));//MB
				statistics_.addStatistic(Statistics::kTimingRAM_estimation(), ramTimer.ticks()*1000);
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

			statistics_.setLabels(_memory->getAllLabels());

			// Path
			if(_path.size())
			{
				statistics_.setLocalPath(this->getPathNextNodes());
				statistics_.setCurrentGoalId(this->getPathCurrentGoalId());
			}
		}

		timeStatsCreation = timer.ticks();
		ULOGGER_INFO("Time creating stats = %f...", timeStatsCreation);
	}

	Signature lastSignatureData(signature->id());
	Transform lastSignatureLocalizedPose;
	if(_optimizedPoses.find(signature->id()) != _optimizedPoses.end())
	{
		lastSignatureLocalizedPose = _optimizedPoses.at(signature->id());
	}
	if(_publishLastSignatureData)
	{
		lastSignatureData = *signature;
	}
	if(!_rawDataKept)
	{
		_memory->removeRawData(signature->id(), true, !_neighborLinkRefining && !_proximityBySpace, true);
	}

	// remove last signature if the memory is not incremental or is a bad signature (if bad signatures are ignored)
	int signatureRemoved = _memory->cleanup();
	if(signatureRemoved)
	{
		signaturesRemoved.push_back(signatureRemoved);
	}

	// If this option activated, add new nodes only if there are linked with a previous map.
	// Used when rtabmap is first started, it will wait a
	// global loop closure detection before starting the new map,
	// otherwise it deletes the current node.
	if(signatureRemoved != lastSignatureData.id())
	{
		if(_startNewMapOnLoopClosure &&
			_memory->isIncremental() &&              // only in mapping mode
			graph::filterLinks(signature->getLinks(), Link::kSelfRefLink).size() == 0 &&      // alone in the current map
			(landmarksDetected.empty() || rejectedLandmark) &&      // if we re not seeing a landmark from a previous map
			_memory->getWorkingMem().size()>=2)       // The working memory should not be empty (beside virtual signature)
		{
			UWARN("Ignoring location %d because a global loop closure is required before starting a new map!",
					signature->id());
			signaturesRemoved.push_back(signature->id());
			_memory->deleteLocation(signature->id());
		}
		else if(_startNewMapOnGoodSignature &&
				(signature->getLandmarks().empty() && signature->isBadSignature()) &&
				graph::filterLinks(signature->getLinks(), Link::kSelfRefLink).size() == 0)     // alone in the current map
		{
			UWARN("Ignoring location %d because a good signature (with enough features or with a landmark detected) is required before starting a new map!",
					signature->id());
			signaturesRemoved.push_back(signature->id());
			_memory->deleteLocation(signature->id());
		}
		else if((smallDisplacement || tooFastMovement) && _loopClosureHypothesis.first == 0 && lastProximitySpaceClosureId == 0)
		{
			// Don't delete the location if a loop closure is detected
			UINFO("Ignoring location %d because the displacement is too small! (d=%f a=%f)",
				  signature->id(), _rgbdLinearUpdate, _rgbdAngularUpdate);
			// If there is a too small displacement, remove the node
			signaturesRemoved.push_back(signature->id());
			_memory->deleteLocation(signature->id());
		}
		else
		{
			_memory->saveLocationData(signature->id());
		}
	}
	else if(!_memory->isIncremental() &&
			(smallDisplacement || tooFastMovement) &&
			_loopClosureHypothesis.first == 0 &&
			lastProximitySpaceClosureId == 0)
	{
		_odomCachePoses.erase(signatureRemoved);
		_odomCacheConstraints.erase(signatureRemoved);
	}

	// Pass this point signature should not be used, since it could have been transferred...
	signature = 0;

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
	if((_maxTimeAllowed != 0 && totalTime*1000>_maxTimeAllowed) ||
		(_maxMemoryAllowed != 0 && _memory->getWorkingMem().size() > _maxMemoryAllowed))
	{
		ULOGGER_INFO("Removing old signatures because time limit is reached %f>%f or memory is reached %d>%d...", totalTime*1000, _maxTimeAllowed, _memory->getWorkingMem().size(), _maxMemoryAllowed);
		immunizedLocations.insert(_lastLocalizationNodeId); // keep the latest localization in working memory
		std::list<int> transferred = _memory->forget(immunizedLocations);
		signaturesRemoved.insert(signaturesRemoved.end(), transferred.begin(), transferred.end());
		if(!_someNodesHaveBeenTransferred && transferred.size())
		{
			_someNodesHaveBeenTransferred = true; // only used to hide a warning on close nodes immunization
		}
	}
	_lastProcessTime = totalTime;

	// cleanup cached gps values
	for(std::list<int>::iterator iter=signaturesRemoved.begin(); iter!=signaturesRemoved.end() && _gpsGeocentricCache.size(); ++iter)
	{
		_gpsGeocentricCache.erase(*iter);
	}

	//Remove optimized poses from signatures transferred
	if(signaturesRemoved.size() && (_optimizedPoses.size() || _constraints.size()))
	{
		//refresh the local map because some transferred nodes may have broken the tree
		int id = 0;
		if(!_memory->isIncremental() && (_lastLocalizationNodeId > 0 || _path.size()))
		{
			if(_path.size())
			{
				// priority on node on the path
				UASSERT(_pathCurrentIndex < _path.size());
				UASSERT_MSG(uContains(_optimizedPoses, _path.at(_pathCurrentIndex).first), uFormat("id=%d", _path.at(_pathCurrentIndex).first).c_str());
				id = _path.at(_pathCurrentIndex).first;
				UDEBUG("Refresh local map from %d", id);
			}
			else
			{
				if(uContains(_optimizedPoses, _lastLocalizationNodeId))
				{
					id = _lastLocalizationNodeId;
					UDEBUG("Refresh local map from %d", id);
				}
				else
				{
					_lastLocalizationNodeId = 0;
				}
			}
		}
		else if(_memory->isIncremental() &&
				_optimizedPoses.size() &&
				_memory->getLastWorkingSignature())
		{
			id = _memory->getLastWorkingSignature()->id();
			UDEBUG("Refresh local map from %d", id);
		}
		UDEBUG("id=%d _optimizedPoses=%d", id, (int)_optimizedPoses.size());
		if(id > 0)
		{
			if(_lastLocalizationNodeId != 0)
			{
				_lastLocalizationNodeId = id;
			}
			UASSERT_MSG(_memory->getSignature(id) != 0, uFormat("id=%d", id).c_str());

			if(signaturesRemoved.size() == 1 && signaturesRemoved.front() == lastSignatureData.id())
			{
				int lastId = signaturesRemoved.front();
				UDEBUG("Detected that only last signature has been removed (lastId=%d)", lastId);
				_optimizedPoses.erase(lastId);
				for(std::multimap<int, Link>::iterator iter=_constraints.find(lastId); iter!=_constraints.end() && iter->first==lastId;++iter)
				{
					if(iter->second.to() != iter->second.from())
					{
						std::multimap<int, Link>::iterator jter = graph::findLink(_constraints, iter->second.to(), iter->second.from(), false);
						if(jter != _constraints.end())
						{
							_constraints.erase(jter);
						}
					}
				}
				_constraints.erase(lastId);
			}
			else
			{

				std::map<int, int> ids = _memory->getNeighborsId(id, 0, 0, true);
				for(std::map<int, Transform>::iterator iter=_optimizedPoses.begin(); iter!=_optimizedPoses.end();)
				{
					if(iter->first > 0 && !uContains(ids, iter->first))
					{
						UDEBUG("Removed %d from local map", iter->first);
						UASSERT(iter->first != _lastLocalizationNodeId);
						_optimizedPoses.erase(iter++);

						if(!_globalScanMap.empty())
						{
							UWARN("optimized poses have been modified, clearing global scan map...");
							_globalScanMap.clear();
							_globalScanMapPoses.clear();
						}
					}
					else
					{
						++iter;
					}
				}
				for(std::multimap<int, Link>::iterator iter=_constraints.begin(); iter!=_constraints.end();)
				{
					if(iter->first > 0 && (!uContains(ids, iter->second.from()) || !uContains(ids, iter->second.to())))
					{
						_constraints.erase(iter++);
					}
					else
					{
						++iter;
					}
				}
			}
		}
		else
		{
			if(!_optimizedPoses.empty())
				UDEBUG("Optimized poses cleared!");
			_optimizedPoses.clear();
			_constraints.clear();
		}
	}
	// just some verifications to make sure that planning path is still in the local map!
	if(_path.size())
	{
		UASSERT(_pathCurrentIndex < _path.size());
		UASSERT(_pathGoalIndex < _path.size());
		UASSERT_MSG(uContains(_optimizedPoses, _path.at(_pathCurrentIndex).first), uFormat("local map size=%d, id=%d", (int)_optimizedPoses.size(), _path.at(_pathCurrentIndex).first).c_str());
		UASSERT_MSG(uContains(_optimizedPoses, _path.at(_pathGoalIndex).first), uFormat("local map size=%d, id=%d", (int)_optimizedPoses.size(), _path.at(_pathGoalIndex).first).c_str());
	}


	timeRealTimeLimitReachedProcess = timer.ticks();
	ULOGGER_INFO("Time limit reached processing = %f...", timeRealTimeLimitReachedProcess);

	//==============================================================
	// Finalize statistics and log files
	//==============================================================
	int localGraphSize = 0;
	if(_publishStats)
	{
		statistics_.addStatistic(Statistics::kTimingStatistics_creation(), timeStatsCreation*1000);
		statistics_.addStatistic(Statistics::kTimingTotal(), totalTime*1000);
		statistics_.addStatistic(Statistics::kTimingForgetting(), timeRealTimeLimitReachedProcess*1000);
		statistics_.addStatistic(Statistics::kTimingJoining_trash(), timeJoiningTrash*1000);
		statistics_.addStatistic(Statistics::kTimingEmptying_trash(), timeEmptyingTrash*1000);
		statistics_.addStatistic(Statistics::kTimingMemory_cleanup(), timeMemoryCleanup*1000);

		// Transfer
		statistics_.addStatistic(Statistics::kMemorySignatures_removed(), signaturesRemoved.size());
		statistics_.addStatistic(Statistics::kMemoryImmunized_globally(), immunizedGlobally);
		statistics_.addStatistic(Statistics::kMemoryImmunized_locally(), immunizedLocally);
		statistics_.addStatistic(Statistics::kMemoryImmunized_locally_max(), maxLocalLocationsImmunized);

		// place after transfer because the memory/local graph may have changed
		statistics_.addStatistic(Statistics::kMemoryWorking_memory_size(), _memory->getWorkingMem().size());
		statistics_.addStatistic(Statistics::kMemoryShort_time_memory_size(), _memory->getStMem().size());
		statistics_.addStatistic(Statistics::kMemoryDatabase_memory_used(), _memory->getDatabaseMemoryUsed());

		// Set local graph
		std::map<int, Transform> poses;
		std::multimap<int, Link> constraints;
		if(!_rgbdSlamMode)
		{
			UDEBUG("");
			// no optimization on appearance-only mode, create a local graph
			std::map<int, int> ids = _memory->getNeighborsId(lastSignatureData.id(), 0, 0, true);
			_memory->getMetricConstraints(uKeysSet(ids), poses, constraints, false);
		}
		else // RGBD-SLAM mode
		{
			poses = _optimizedPoses;
			constraints = _constraints;
		}
		UDEBUG("");
		if(_publishLastSignatureData)
		{
			UINFO("Adding data %d [%d] (rgb/left=%d depth/right=%d)", lastSignatureData.id(), lastSignatureData.mapId(), lastSignatureData.sensorData().imageRaw().empty()?0:1, lastSignatureData.sensorData().depthOrRightRaw().empty()?0:1);
			statistics_.setLastSignatureData(lastSignatureData);
		}
		else
		{
			// only copy node info
			Signature nodeInfo(
					lastSignatureData.id(),
					lastSignatureData.mapId(),
					lastSignatureData.getWeight(),
					lastSignatureData.getStamp(),
					lastSignatureData.getLabel(),
					lastSignatureData.getPose(),
					lastSignatureData.getGroundTruthPose());
			const std::vector<float> & v = lastSignatureData.getVelocity();
			if(v.size() == 6)
			{
				nodeInfo.setVelocity(v[0], v[1], v[2], v[3], v[4], v[5]);
			}
			nodeInfo.sensorData().setGPS(lastSignatureData.sensorData().gps());
			nodeInfo.sensorData().setEnvSensors(lastSignatureData.sensorData().envSensors());
			statistics_.setLastSignatureData(nodeInfo);
		}
		UDEBUG("");
		localGraphSize = (int)poses.size();
		if(!lastSignatureLocalizedPose.isNull())
		{
			poses.insert(std::make_pair(lastSignatureData.id(), lastSignatureLocalizedPose)); // in case we are in localization
		}
		statistics_.setPoses(poses);
		statistics_.setConstraints(constraints);

		statistics_.addStatistic(Statistics::kMemoryLocal_graph_size(), poses.size());

		statistics_.setOdomCachePoses(_odomCachePoses);
		statistics_.setOdomCacheConstraints(_odomCacheConstraints);
		statistics_.addStatistic(Statistics::kMemoryOdom_cache_poses(), _odomCachePoses.size());
		statistics_.addStatistic(Statistics::kMemoryOdom_cache_links(), _odomCacheConstraints.size());

		if(_computeRMSE && _memory->getGroundTruths().size())
		{
			UDEBUG("Computing RMSE...");
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
					_memory->getGroundTruths(),
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

			statistics_.addStatistic(Statistics::kGtTranslational_rmse(), translational_rmse);
			statistics_.addStatistic(Statistics::kGtTranslational_mean(), translational_mean);
			statistics_.addStatistic(Statistics::kGtTranslational_median(), translational_median);
			statistics_.addStatistic(Statistics::kGtTranslational_std(), translational_std);
			statistics_.addStatistic(Statistics::kGtTranslational_min(), translational_min);
			statistics_.addStatistic(Statistics::kGtTranslational_max(), translational_max);
			statistics_.addStatistic(Statistics::kGtRotational_rmse(), rotational_rmse);
			statistics_.addStatistic(Statistics::kGtRotational_mean(), rotational_mean);
			statistics_.addStatistic(Statistics::kGtRotational_median(), rotational_median);
			statistics_.addStatistic(Statistics::kGtRotational_std(), rotational_std);
			statistics_.addStatistic(Statistics::kGtRotational_min(), rotational_min);
			statistics_.addStatistic(Statistics::kGtRotational_max(), rotational_max);
			UDEBUG("Computing RMSE...done!");
		}

		std::vector<int> ids;
		ids.reserve(_memory->getWorkingMem().size() + _memory->getStMem().size());
		for(std::set<int>::const_iterator iter=_memory->getStMem().begin(); iter!=_memory->getStMem().end(); ++iter)
		{
			ids.push_back(*iter);
		}
		for(std::map<int, double>::const_iterator iter=_memory->getWorkingMem().lower_bound(0); iter!=_memory->getWorkingMem().end(); ++iter)
		{
			ids.push_back(iter->first);
		}
		statistics_.setWmState(ids);
		UDEBUG("wmState=%d", (int)ids.size());
	}

	//Save statistics to database
	if(_memory->isIncremental() || _memory->isLocalizationDataSaved())
	{
		_memory->saveStatistics(statistics_, _saveWMState);
	}

	//Start trashing
	UDEBUG("Empty trash...");
	_memory->emptyTrash();

	// Log info...
	// TODO : use a specific class which will handle the RtabmapEvent
	if(_foutFloat && _foutInt)
	{
		UDEBUG("Logging...");
		std::string logF = uFormat("%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n",
									totalTime,
									timeMemoryUpdate,
									timeReactivations,
									timeLikelihoodCalculation,
									timePosteriorCalculation,
									timeHypothesesCreation,
									timeHypothesesValidation,
									timeRealTimeLimitReachedProcess,
									timeStatsCreation,
									_highestHypothesis.second,
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
									timeAddLoopClosureLink,
									timeMemoryCleanup,
									timeNeighborLinkRefining,
									timeProximityByTimeDetection,
									timeProximityBySpaceDetection,
									timeMapOptimization);
		std::string logI = uFormat("%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d\n",
									_loopClosureHypothesis.first,
									_highestHypothesis.first,
									(int)signaturesRemoved.size(),
									0,
									refWordsCount,
									dictionarySize,
									int(_memory->getWorkingMem().size()),
									rejectedGlobalLoopClosure?1:0,
									0,
									0,
									int(signaturesRetrieved.size()),
									lcHypothesisReactivated,
									refUniqueWordsCount,
									retrievalId,
									0,
									rehearsalMaxId,
									rehearsalMaxId>0?1:0,
									localGraphSize,
									data.id(),
									_memory->getVWDictionary()->getIndexedWordsCount(),
									_memory->getVWDictionary()->getIndexMemoryUsed());
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
	}
	timeFinalizingStatistics = timer.ticks();
	UDEBUG("End process, timeFinalizingStatistics=%fs", timeFinalizingStatistics);
	if(_publishStats)
	{
		statistics_.addStatistic(Statistics::kTimingFinalizing_statistics(), timeFinalizingStatistics*1000);
	}

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
	else if(_maxTimeAllowed > 0.0f && _maxTimeAllowed < 1.0f)
	{
		ULOGGER_WARN("Time threshold set to %fms, it is not in seconds!", _maxTimeAllowed);
	}
}

void Rtabmap::setWorkingDirectory(std::string path)
{
	path = uReplaceChar(path, '~', UDirectory::homeDir());
	if(!path.empty() && UDirectory::exists(path))
	{
		ULOGGER_DEBUG("Comparing new working directory path \"%s\" with \"%s\"", path.c_str(), _wDir.c_str());
		if(path.compare(_wDir) != 0)
		{
			if (_foutFloat || _foutInt)
			{
				UWARN("Working directory has been changed from \"%s\" with \"%s\", new log files will be created.",
					path.c_str(), _wDir.c_str());
			}
			_wDir = path;
			setupLogFiles();
		}
	}
	else if(path.empty())
	{
		_wDir.clear();
		setupLogFiles();
	}
	else
	{
		ULOGGER_ERROR("Directory \"%s\" doesn't exist!", path.c_str());
	}
}

void Rtabmap::rejectLastLoopClosure()
{
	if(_memory && _memory->getStMem().find(getLastLocationId())!=_memory->getStMem().end())
	{
		std::multimap<int, Link> links = _memory->getLinks(getLastLocationId(), false);
		bool linksRemoved = false;
		for(std::multimap<int, Link>::iterator iter = links.begin(); iter!=links.end(); ++iter)
		{
			if(iter->second.type() == Link::kGlobalClosure ||
				iter->second.type() == Link::kLocalSpaceClosure ||
				iter->second.type() == Link::kLocalTimeClosure ||
				iter->second.type() == Link::kUserClosure)
			{
				_memory->removeLink(iter->second.from(), iter->second.to());
				std::multimap<int, Link>::iterator jter = graph::findLink(_constraints, iter->second.from(), iter->second.to(), true);
				if(jter!=_constraints.end())
				{
					_constraints.erase(jter);
					// second time if link is also inverted
					jter = graph::findLink(_constraints, iter->second.from(), iter->second.to(), true);
					if(jter!=_constraints.end())
					{
						_constraints.erase(jter);
					}
				}
				linksRemoved = true;
			}
		}

		if(linksRemoved)
		{
			_loopClosureHypothesis.first = 0;

			// we have to re-optimize the graph without the rejected links
			if(_memory->isIncremental() && _optimizedPoses.size())
			{
				UINFO("Update graph");
				std::map<int, Transform> poses = _optimizedPoses;
				std::multimap<int, Link> constraints;
				cv::Mat covariance;
				optimizeCurrentMap(getLastLocationId(), false, poses, covariance, &constraints);

				if(poses.empty())
				{
					UWARN("Graph optimization failed after removing loop closure links from last location!");
				}
				else
				{
					UINFO("Updated local map (old size=%d, new size=%d)", (int)_optimizedPoses.size(), (int)poses.size());
					_optimizedPoses = poses;
					_constraints = constraints;
					_mapCorrection = _optimizedPoses.at(_memory->getLastWorkingSignature()->id()) * _memory->getLastWorkingSignature()->getPose().inverse();
				}
			}
		}
	}
}

void Rtabmap::deleteLastLocation()
{
	if(_memory && _memory->getStMem().size())
	{
		int lastId = *_memory->getStMem().rbegin();
		_memory->deleteLocation(lastId);
		// we have to re-optimize the graph without the deleted location
		if(_memory->isIncremental() && _optimizedPoses.size())
		{
			UINFO("Update graph");
			_optimizedPoses.erase(lastId);
			std::map<int, Transform> poses = _optimizedPoses;
			//remove all constraints with last localization id
			for(std::multimap<int, Link>::iterator iter=_constraints.begin(); iter!=_constraints.end();)
			{
				if(iter->second.from() == lastId || iter->second.to() == lastId)
				{
					_constraints.erase(iter++);
				}
				else
				{
					++iter;
				}
			}

			if(poses.empty())
			{
				_mapCorrection.setIdentity();
			}
			else
			{
				std::multimap<int, Link> constraints;
				cv::Mat covariance;
				optimizeCurrentMap(_memory->getLastWorkingSignature()->id(), false, poses, covariance, &constraints);

				if(poses.empty())
				{
					UWARN("Graph optimization failed after deleting the last location!");
				}
				else
				{
					_optimizedPoses = poses;
					_constraints = constraints;
					_mapCorrection = _optimizedPoses.at(_memory->getLastWorkingSignature()->id()) * _memory->getLastWorkingSignature()->getPose().inverse();
				}
			}
		}
	}
}

void Rtabmap::setOptimizedPoses(const std::map<int, Transform> & poses, const std::multimap<int, Link> & constraints)
{
	_optimizedPoses = poses;
    _constraints = constraints;
}

void Rtabmap::dumpData() const
{
	UDEBUG("");
	if(_memory)
	{
		if(this->getWorkingDir().empty())
		{
			UERROR("Working directory not set.");
		}
		else
		{
			_memory->dumpMemory(this->getWorkingDir());
		}
	}
}

// fromId must be in _memory and in _optimizedPoses
// Get poses in front of the robot, return optimized poses
std::map<int, Transform> Rtabmap::getForwardWMPoses(
		int fromId,
		int maxNearestNeighbors,
		float radius,
		int maxGraphDepth // 0 means ignore
		) const
{
	std::map<int, Transform> poses;
	if(_memory && fromId > 0)
	{
		UDEBUG("");
		const Signature * fromS = _memory->getSignature(fromId);
		UASSERT(fromS != 0);
		UASSERT(_optimizedPoses.find(fromId) != _optimizedPoses.end());

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		cloud->resize(_optimizedPoses.size());
		std::vector<int> ids(_optimizedPoses.size());
		int oi = 0;
		const std::set<int> & stm = _memory->getStMem();
		//get distances
		std::map<int, float> foundIds;
		if(_memory->isIncremental())
		{
			foundIds = _memory->getNeighborsIdRadius(fromId, radius, _optimizedPoses, maxGraphDepth);
		}
		else
		{
			foundIds = graph::findNearestNodes(fromId, _optimizedPoses, radius);
		}

		float radiusSqrd = radius * radius;
		for(std::map<int, Transform>::const_iterator iter = _optimizedPoses.begin(); iter!=_optimizedPoses.end(); ++iter)
		{
			if(iter->first != fromId)
			{
				if(stm.find(iter->first) == stm.end() &&
				   uContains(foundIds, iter->first) &&
				   (radiusSqrd==0 || foundIds.at(iter->first) <= radiusSqrd))
				{
					(*cloud)[oi] = pcl::PointXYZ(iter->second.x(), iter->second.y(), iter->second.z());
					ids[oi++] = iter->first;
				}
			}
		}

		cloud->resize(oi);
		ids.resize(oi);

		Transform fromT = _optimizedPoses.at(fromId);

		if(cloud->size())
		{
			//if(cloud->size())
			//{
			//	pcl::io::savePCDFile("radiusPoses.pcd", *cloud);
			//	UWARN("Saved radiusPoses.pcd");
			//}

			//filter poses in front of the fromId
			float x,y,z, roll,pitch,yaw;
			fromT.getTranslationAndEulerAngles(x,y,z, roll,pitch,yaw);

			pcl::CropBox<pcl::PointXYZ> cropbox;
			cropbox.setInputCloud(cloud);
			cropbox.setMin(Eigen::Vector4f(-1, -radius, -999999, 0));
			cropbox.setMax(Eigen::Vector4f(radius, radius, 999999, 0));
			cropbox.setRotation(Eigen::Vector3f(roll, pitch, yaw));
			cropbox.setTranslation(Eigen::Vector3f(x, y, z));
			cropbox.setRotation(Eigen::Vector3f(roll,pitch,yaw));
			pcl::IndicesPtr indices(new std::vector<int>());
			cropbox.filter(*indices);

			//if(indices->size())
			//{
			//	pcl::io::savePCDFile("radiusCrop.pcd", *cloud, *indices);
			//	UWARN("Saved radiusCrop.pcd");
			//}

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
			}
		}
	}
	return poses;
}

std::map<int, std::map<int, Transform> > Rtabmap::getPaths(const std::map<int, Transform> & posesIn, const Transform & target, int maxGraphDepth) const
{
	std::map<int, std::map<int, Transform> > paths;
	std::set<int> nodesSet;
	std::map<int, Transform> poses;
	for(std::map<int, Transform>::const_iterator iter=posesIn.lower_bound(1); iter!=posesIn.end(); ++iter)
	{
		nodesSet.insert(iter->first);
		poses.insert(*iter);
	}
	if(_memory && nodesSet.size() && !target.isNull())
	{
		double e0=0,e1=0,e2=0,e3=0,e4=0;
		UTimer t;
		e0 = t.ticks();
		// Segment poses connected only by neighbor links
		while(poses.size())
		{
			std::map<int, Transform> path;
			// select nearest pose and iterate neighbors from there
			int nearestId = rtabmap::graph::findNearestNode(poses, target);

			e1+=t.ticks();

			if(nearestId == 0)
			{
				UWARN("Nearest id of %s in %d poses is 0 !? Returning empty path.", target.prettyPrint().c_str(), (int)poses.size());
				break;
			}
			std::map<int, int> ids = _memory->getNeighborsId(nearestId, maxGraphDepth, 0, true, true, true, true, nodesSet);

			e2+=t.ticks();

			for(std::map<int, int>::iterator iter=ids.begin(); iter!=ids.end(); ++iter)
			{
				std::map<int, Transform>::iterator jter = poses.find(iter->first);
				if(jter != poses.end())
				{
					bool valid = path.empty();
					if(!valid)
					{
						// make sure it has a neighbor added to path
						std::multimap<int, Link> links = _memory->getNeighborLinks(iter->first);
						for(std::multimap<int, Link>::iterator kter=links.begin(); kter!=links.end() && !valid; ++kter)
						{
							valid = path.find(kter->first) != path.end();
						}
					}

					if(valid)
					{
						//UDEBUG("%d <- %d", nearestId, jter->first);
						path.insert(*jter);
						poses.erase(jter);
					}
				}
			}

			e3+=t.ticks();

			if (path.size())
			{
				if (maxGraphDepth > 0 && !_memory->isGraphReduced() && (int)path.size() > maxGraphDepth * 2 + 1)
				{
					UWARN("%s=Off but path(%d) > maxGraphDepth(%d)*2+1, nearestId=%d ids=%d. Is reduce graph activated before?",
						Parameters::kMemReduceGraph().c_str(), (int)path.size(), maxGraphDepth, nearestId, (int)ids.size());
				}
				paths.insert(std::make_pair(nearestId, path));
			}
			else
			{
				UWARN("path.size()=0!? nearestId=%d ids=%d, aborting...", nearestId, (int)ids.size());
				break;
			}

			e4+=t.ticks();
		}
		UDEBUG("e0=%fs e1=%fs e2=%fs e3=%fs e4=%fs", e0, e1, e2, e3, e4);
	}
	return paths;
}

void Rtabmap::optimizeCurrentMap(
		int id,
		bool lookInDatabase,
		std::map<int, Transform> & optimizedPoses,
		cv::Mat & covariance,
		std::multimap<int, Link> * constraints,
		double * error,
		int * iterationsDone) const
{
	//Optimize the map
	UINFO("Optimize map: around location %d (lookInDatabase=%s)", id, lookInDatabase?"true":"false");
	if(_memory && id > 0)
	{
		UTimer timer;
		std::map<int, int> ids = _memory->getNeighborsId(id, 0, lookInDatabase?-1:0, true, false);
		if(!_optimizeFromGraphEnd && ids.size() > 1)
		{
			id = ids.begin()->first;
		}
		UINFO("get %d ids time %f s", (int)ids.size(), timer.ticks());

		std::map<int, Transform> poses = Rtabmap::optimizeGraph(id, uKeysSet(ids), optimizedPoses, lookInDatabase, covariance, constraints, error, iterationsDone);
		UINFO("optimize time %f s", timer.ticks());

		if(poses.size())
		{
			optimizedPoses = poses;

			if(_memory->getSignature(id) && uContains(optimizedPoses, id))
			{
				Transform t = optimizedPoses.at(id) * _memory->getSignature(id)->getPose().inverse();
				UINFO("Correction (from node %d) %s", id, t.prettyPrint().c_str());
			}
		}
		else
		{
			UWARN("Failed to optimize the graph! returning empty optimized poses...");
			optimizedPoses.clear();
			if(constraints)
			{
				constraints->clear();
			}
		}
	}
}

std::map<int, Transform> Rtabmap::optimizeGraph(
		int fromId,
		const std::set<int> & ids,
		const std::map<int, Transform> & guessPoses,
		bool lookInDatabase,
		cv::Mat & covariance,
		std::multimap<int, Link> * constraints,
		double * error,
		int * iterationsDone) const
{
	UTimer timer;
	std::map<int, Transform> optimizedPoses;
	std::map<int, Transform> poses;
	std::multimap<int, Link> edgeConstraints;
	UDEBUG("ids=%d", (int)ids.size());
	_memory->getMetricConstraints(ids, poses, edgeConstraints, lookInDatabase, !_graphOptimizer->landmarksIgnored());
	UINFO("get constraints (ids=%d, %d poses, %d edges) time %f s", (int)ids.size(), (int)poses.size(), (int)edgeConstraints.size(), timer.ticks());

	if(_graphOptimizer->iterations() > 0)
	{
		for(std::map<int, Transform>::iterator iter=poses.begin(); iter!=poses.end(); ++iter)
		{
			// Apply guess poses (if some), ignore for rootid to avoid origin drifting
			std::map<int, Transform>::const_iterator foundGuess = guessPoses.find(iter->first);
			if(foundGuess!=guessPoses.end() && iter->first != fromId)
			{
				iter->second = foundGuess->second;
			}
		}
	}


	UASSERT(_graphOptimizer!=0);
	if(_graphOptimizer->iterations() == 0)
	{
		// Optimization disabled! Return not optimized poses.
		optimizedPoses = poses;
		if(constraints)
		{
			*constraints = edgeConstraints;
		}
	}
	else
	{
		bool hasLandmarks = edgeConstraints.begin()->first < 0;
		if(poses.size() != guessPoses.size() || hasLandmarks)
		{
			UDEBUG("recompute poses using only links (robust to multi-session)");
			std::map<int, Transform> posesOut;
			std::multimap<int, Link> edgeConstraintsOut;
			_graphOptimizer->getConnectedGraph(fromId, poses, edgeConstraints, posesOut, edgeConstraintsOut);
			optimizedPoses = _graphOptimizer->optimize(fromId, posesOut, edgeConstraintsOut, covariance, 0, error, iterationsDone);
			if(constraints)
			{
				*constraints = edgeConstraintsOut;
			}
		}
		else
		{
			UDEBUG("use input guess poses");
			optimizedPoses = _graphOptimizer->optimize(fromId, poses, edgeConstraints, covariance, 0, error, iterationsDone);
			if(constraints)
			{
				*constraints = edgeConstraints;
			}
		}

		if(!poses.empty() && optimizedPoses.empty())
		{
			UWARN("Optimization has failed (poses=%d, guess=%d, links=%d)...",
				  (int)poses.size(), (int)guessPoses.size(), (int)edgeConstraints.size());
		}
	}

	UINFO("Optimization time %f s", timer.ticks());

	return optimizedPoses;
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
	float stdDev = std::sqrt(uVariance(values, mean));


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
		if(this->getWorkingDir().empty())
		{
			UERROR("Working directory not set.");
			return;
		}
		std::list<int> signaturesToCompare;
		for(std::map<int, double>::const_iterator iter=_memory->getWorkingMem().begin();
			iter!=_memory->getWorkingMem().end();
			++iter)
		{
			if(iter->first > 0)
			{
				const Signature * s = _memory->getSignature(iter->first);
				UASSERT(s!=0);
				if(s->getWeight() != -1) // ignore intermediate nodes
				{
					signaturesToCompare.push_back(iter->first);
				}
			}
			else
			{
				// virtual signature should be added
				signaturesToCompare.push_back(iter->first);
			}
		}
		cv::Mat prediction = _bayesFilter->generatePrediction(_memory, uListToVector(signaturesToCompare));

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

Signature Rtabmap::getSignatureCopy(int id, bool images, bool scan, bool userData, bool occupancyGrid, bool withWords, bool withGlobalDescriptors) const
{
	Signature s;
	if(_memory)
	{
		Transform odomPoseLocal;
		int weight = -1;
		int mapId = -1;
		std::string label;
		double stamp = 0;
		Transform groundTruth;
		std::vector<float> velocity;
		GPS gps;
		EnvSensors sensors;
		_memory->getNodeInfo(id, odomPoseLocal, mapId, weight, label, stamp, groundTruth, velocity, gps, sensors, true);
		SensorData data;
		data.setId(id);
		if(images || scan || userData || occupancyGrid)
		{
			data = _memory->getNodeData(id, images, scan, userData, occupancyGrid);
		}
		if(!images && withWords)
		{
			std::vector<CameraModel> models;
			StereoCameraModel stereoModel;
			_memory->getNodeCalibration(id, models, stereoModel);
			data.setCameraModels(models);
			data.setStereoCameraModel(stereoModel);
		}

		s=Signature(id,
				mapId,
				weight,
				stamp,
				label,
				odomPoseLocal,
				groundTruth,
				data);

		std::multimap<int, Link> links = _memory->getLinks(id, true, true);
		for(std::multimap<int, Link>::iterator iter=links.begin(); iter!=links.end(); ++iter)
		{
			if(iter->second.type() == Link::kLandmark)
			{
				s.addLandmark(iter->second);
			}
			else
			{
				s.addLink(iter->second);
			}
		}

		if(withWords || withGlobalDescriptors)
		{
			std::multimap<int, int> words;
			std::vector<cv::KeyPoint> wordsKpts;
			std::vector<cv::Point3f> words3;
			cv::Mat wordsDescriptors;
			std::vector<rtabmap::GlobalDescriptor> globalDescriptors;
			_memory->getNodeWordsAndGlobalDescriptors(id, words, wordsKpts, words3, wordsDescriptors, globalDescriptors);
			if(withWords)
			{
				s.setWords(words, wordsKpts, words3, wordsDescriptors);
			}
			if(withGlobalDescriptors)
			{
				s.sensorData().setGlobalDescriptors(globalDescriptors);
			}
		}
		if(velocity.size()==6)
		{
			s.setVelocity(velocity[0], velocity[1], velocity[2], velocity[3], velocity[4], velocity[5]);
		}
		s.sensorData().setGPS(gps);
		s.sensorData().setEnvSensors(sensors);
	}
	return s;
}

void Rtabmap::get3DMap(
		std::map<int, Signature> & signatures,
		std::map<int, Transform> & poses,
		std::multimap<int, Link> & constraints,
		bool optimized,
		bool global) const
{
	UDEBUG("");
	return getGraph(poses, constraints, optimized, global, &signatures, true, true, true, true);
}

void Rtabmap::getGraph(
		std::map<int, Transform> & poses,
		std::multimap<int, Link> & constraints,
		bool optimized,
		bool global,
		std::map<int, Signature> * signatures,
		bool withImages,
		bool withScan,
		bool withUserData,
		bool withGrid,
		bool withWords,
		bool withGlobalDescriptors) const
{
	if(_memory && _memory->getLastWorkingSignature())
	{
		if(_rgbdSlamMode)
		{
			if(optimized)
			{
				poses = _optimizedPoses; // guess
				cv::Mat covariance;
				this->optimizeCurrentMap(_memory->getLastWorkingSignature()->id(), global, poses, covariance, &constraints);
				if(!global && !_optimizedPoses.empty())
				{
					// We send directly the already optimized poses if they are set
					UDEBUG("_optimizedPoses=%ld poses=%ld", _optimizedPoses.size(), poses.size());
					poses = _optimizedPoses;
				}
			}
			else
			{
				std::map<int, int> ids = _memory->getNeighborsId(_memory->getLastWorkingSignature()->id(), 0, global?-1:0, true);
				_memory->getMetricConstraints(uKeysSet(ids), poses, constraints, global);
			}
		}
		else
		{
			// no optimization on appearance-only mode
			std::map<int, int> ids = _memory->getNeighborsId(_memory->getLastWorkingSignature()->id(), 0, global?-1:0, true);
			_memory->getMetricConstraints(uKeysSet(ids), poses, constraints, global);
		}

		if(signatures)
		{
			// Get data
			std::set<int> ids = uKeysSet(_memory->getWorkingMem()); // WM

			//remove virtual signature
			ids.erase(Memory::kIdVirtual);

			ids.insert(_memory->getStMem().begin(), _memory->getStMem().end()); // STM + WM
			if(global)
			{
				ids = _memory->getAllSignatureIds(); // STM + WM + LTM, ignoreChildren=true
			}

			for(std::set<int>::iterator iter = ids.begin(); iter!=ids.end(); ++iter)
			{
				signatures->insert(std::make_pair(*iter, getSignatureCopy(*iter, withImages, withScan, withUserData, withGrid, withWords, withGlobalDescriptors)));
			}
		}
	}
	else if(_memory && (_memory->getStMem().size() || _memory->getWorkingMem().size() > 1))
	{
		UERROR("Last working signature is null!?");
	}
	else if(_memory == 0)
	{
		UWARN("Memory not initialized...");
	}
}

std::map<int, Transform> Rtabmap::getNodesInRadius(const Transform & pose, float radius, int k, std::map<int, float> * distsSqr)
{
	std::map<int, float> nearestNodesTmp;
	std::map<int, float> * nearestNodesPtr = distsSqr == 0? &nearestNodesTmp : distsSqr;
	*nearestNodesPtr = graph::findNearestNodes(pose, _optimizedPoses, radius<=0?_localRadius:radius, 0, k);
	std::map<int, Transform> nearestPoses;
	for(std::map<int, float>::iterator iter=nearestNodesPtr->begin(); iter!=nearestNodesPtr->end(); ++iter)
	{
		nearestPoses.insert(*_optimizedPoses.find(iter->first));
	}
	return nearestPoses;
}

std::map<int, Transform> Rtabmap::getNodesInRadius(int nodeId, float radius, int k, std::map<int, float> * distsSqr)
{
	UDEBUG("nodeId=%d, radius=%f", nodeId, radius);
	std::map<int, float> nearestNodesTmp;
	std::map<int, float> * nearestNodesPtr = distsSqr == 0? &nearestNodesTmp : distsSqr;
	if(nodeId==0 && !_lastLocalizationPose.isNull() && !_lastLocalizationPose.isIdentity())
	{
		*nearestNodesPtr = graph::findNearestNodes(_lastLocalizationPose, _optimizedPoses, radius<=0?_localRadius:radius, 0, k);
	}
	else
	{
		if(nodeId==0 && !_optimizedPoses.empty())
		{
			nodeId = _optimizedPoses.rbegin()->first;
		}

		if(_optimizedPoses.find(nodeId) != _optimizedPoses.end())
		{
			*nearestNodesPtr = graph::findNearestNodes(nodeId, _optimizedPoses, radius<=0?_localRadius:radius, 0, k);
		}
	}

	std::map<int, Transform> nearestPoses;
	for(std::map<int, float>::iterator iter=nearestNodesPtr->begin(); iter!=nearestNodesPtr->end(); ++iter)
	{
		nearestPoses.insert(*_optimizedPoses.find(iter->first));
	}

	return nearestPoses;
}

int Rtabmap::detectMoreLoopClosures(
		float clusterRadiusMax,
		float clusterAngle,
		int iterations,
		bool intraSession,
		bool interSession,
		const ProgressState * processState,
		float clusterRadiusMin)
{
	UASSERT(iterations>0);

	if(_graphOptimizer->iterations() <= 0)
	{
		UERROR("Cannot detect more loop closures if graph optimization iterations = 0");
		return -1;
	}
	if(!_rgbdSlamMode)
	{
		UERROR("Detecting more loop closures can be done only in RGBD-SLAM mode.");
		return -1;
	}
	if(!intraSession && !interSession)
	{
		UERROR("Intra and/or inter session argument should be true.");
		return -1;
	}

	std::list<Link> loopClosuresAdded;
	std::multimap<int, int> checkedLoopClosures;

	std::map<int, Transform> posesToCheckLoopClosures;
	std::map<int, Transform> poses;
	std::multimap<int, Link> links;
	std::map<int, Signature> signatures; // some signatures may be in LTM, get them all
	this->getGraph(poses, links, true, true, &signatures);

	std::map<int, int> mapIds;
	UDEBUG("remove all invalid or intermediate nodes, fill mapIds");
	for(std::map<int, Transform>::iterator iter=poses.upper_bound(0); iter!=poses.end();++iter)
	{
		if(signatures.at(iter->first).getWeight() >= 0)
		{
			posesToCheckLoopClosures.insert(*iter);
			mapIds.insert(std::make_pair(iter->first, signatures.at(iter->first).mapId()));
		}
	}

	for(int n=0; n<iterations; ++n)
	{
		UINFO("Looking for more loop closures, clustering poses... (iteration=%d/%d, radius=%f m angle=%f rad)",
				n+1, iterations, clusterRadiusMax, clusterAngle);

		std::multimap<int, int> clusters = graph::radiusPosesClustering(
				posesToCheckLoopClosures,
				clusterRadiusMax,
				clusterAngle);

		UINFO("Looking for more loop closures, clustering poses... found %d clusters.", (int)clusters.size());

		int i=0;
		std::set<int> addedLinks;
		for(std::multimap<int, int>::iterator iter=clusters.begin(); iter!= clusters.end(); ++iter, ++i)
		{
			if(processState && processState->isCanceled())
			{
				return -1;
				break;
			}

			int from = iter->first;
			int to = iter->second;
			if(from > to)
			{
				from = iter->second;
				to = iter->first;
			}

			int mapIdFrom = uValue(mapIds, from, 0);
			int mapIdTo = uValue(mapIds, to, 0);

			if((interSession && mapIdFrom != mapIdTo) ||
			   (intraSession && mapIdFrom == mapIdTo))
			{

				bool alreadyChecked = false;
				for(std::multimap<int, int>::iterator jter = checkedLoopClosures.lower_bound(from);
					!alreadyChecked && jter!=checkedLoopClosures.end() && jter->first == from;
					++jter)
				{
					if(to == jter->second)
					{
						alreadyChecked = true;
					}
				}

				if(!alreadyChecked)
				{
					// only add new links and one per cluster per iteration
					if(addedLinks.find(from) == addedLinks.end() &&
					   addedLinks.find(to) == addedLinks.end() &&
					   rtabmap::graph::findLink(links, from, to) == links.end())
					{
						// Reverify if in the bounds with the current optimized graph
						Transform delta = poses.at(from).inverse() * poses.at(to);
						if(delta.getNorm() < clusterRadiusMax &&
						   delta.getNorm() >= clusterRadiusMin)
						{
							checkedLoopClosures.insert(std::make_pair(from, to));

							UASSERT(signatures.find(from) != signatures.end());
							UASSERT(signatures.find(to) != signatures.end());

							Transform guess;
							if(_proximityOdomGuess && uContains(poses, from) && uContains(poses, to))
							{
								guess = poses.at(from).inverse() * poses.at(to);
							}

							RegistrationInfo info;
							// use signatures instead of IDs because some signatures may not be in WM
							Transform t = _memory->computeTransform(signatures.at(from), signatures.at(to), guess, &info);

							if(!t.isNull())
							{
								bool updateConstraints = true;
								if(_optimizationMaxError > 0.0f)
								{
									//optimize the graph to see if the new constraint is globally valid

									int fromId = from;
									int mapId = signatures.at(from).mapId();
									// use first node of the map containing from
									for(std::map<int, Signature>::iterator ster=signatures.begin(); ster!=signatures.end(); ++ster)
									{
										if(ster->second.mapId() == mapId)
										{
											fromId = ster->first;
											break;
										}
									}
									std::multimap<int, Link> linksIn = links;
									linksIn.insert(std::make_pair(from, Link(from, to, Link::kUserClosure, t, getInformation(info.covariance))));
									const Link * maxLinearLink = 0;
									const Link * maxAngularLink = 0;
									float maxLinearError = 0.0f;
									float maxAngularError = 0.0f;
									float maxLinearErrorRatio = 0.0f;
									float maxAngularErrorRatio = 0.0f;
									std::map<int, Transform> optimizedPoses;
									std::multimap<int, Link> links;
									UASSERT(poses.find(fromId) != poses.end());
									UASSERT_MSG(poses.find(from) != poses.end(), uFormat("id=%d poses=%d links=%d", from, (int)poses.size(), (int)links.size()).c_str());
									UASSERT_MSG(poses.find(to) != poses.end(), uFormat("id=%d poses=%d links=%d", to, (int)poses.size(), (int)links.size()).c_str());
									_graphOptimizer->getConnectedGraph(fromId, poses, linksIn, optimizedPoses, links);
									UASSERT(optimizedPoses.find(fromId) != optimizedPoses.end());
									UASSERT_MSG(optimizedPoses.find(from) != optimizedPoses.end(), uFormat("id=%d poses=%d links=%d", from, (int)optimizedPoses.size(), (int)links.size()).c_str());
									UASSERT_MSG(optimizedPoses.find(to) != optimizedPoses.end(), uFormat("id=%d poses=%d links=%d", to, (int)optimizedPoses.size(), (int)links.size()).c_str());
									UASSERT(graph::findLink(links, from, to) != links.end());
									optimizedPoses = _graphOptimizer->optimize(fromId, optimizedPoses, links);
									std::string msg;
									if(optimizedPoses.size())
									{
										graph::computeMaxGraphErrors(
												optimizedPoses,
												links,
												maxLinearErrorRatio,
												maxAngularErrorRatio,
												maxLinearError,
												maxAngularError,
												&maxLinearLink,
												&maxAngularLink);
										if(maxLinearLink)
										{
											UINFO("Max optimization linear error = %f m (link %d->%d)", maxLinearError, maxLinearLink->from(), maxLinearLink->to());
											if(maxLinearErrorRatio > _optimizationMaxError)
											{
												msg = uFormat("Rejecting edge %d->%d because "
														  "graph error is too large after optimization (%f m for edge %d->%d with ratio %f > std=%f m). "
														  "\"%s\" is %f.",
														  from,
														  to,
														  maxLinearError,
														  maxLinearLink->from(),
														  maxLinearLink->to(),
														  maxLinearErrorRatio,
														  sqrt(maxLinearLink->transVariance()),
														  Parameters::kRGBDOptimizeMaxError().c_str(),
														  _optimizationMaxError);
											}
										}
										else if(maxAngularLink)
										{
											UINFO("Max optimization angular error = %f deg (link %d->%d)", maxAngularError*180.0f/M_PI, maxAngularLink->from(), maxAngularLink->to());
											if(maxAngularErrorRatio > _optimizationMaxError)
											{
												msg = uFormat("Rejecting edge %d->%d because "
														  "graph error is too large after optimization (%f deg for edge %d->%d with ratio %f > std=%f deg). "
														  "\"%s\" is %f m.",
														  from,
														  to,
														  maxAngularError*180.0f/M_PI,
														  maxAngularLink->from(),
														  maxAngularLink->to(),
														  maxAngularErrorRatio,
														  sqrt(maxAngularLink->rotVariance()),
														  Parameters::kRGBDOptimizeMaxError().c_str(),
														  _optimizationMaxError);
											}
										}
									}
									else
									{
										msg = uFormat("Rejecting edge %d->%d because graph optimization has failed!",
												  from,
												  to);
									}
									if(!msg.empty())
									{
										UWARN("%s", msg.c_str());
										updateConstraints = false;
									}
									else
									{
										poses = optimizedPoses;
									}
								}

								if(updateConstraints)
								{
									addedLinks.insert(from);
									addedLinks.insert(to);
									cv::Mat inf = getInformation(info.covariance);
									links.insert(std::make_pair(from, Link(from, to, Link::kUserClosure, t, inf)));
									loopClosuresAdded.push_back(Link(from, to, Link::kUserClosure, t, inf));
									std::string msg = uFormat("Iteration %d/%d: Added loop closure %d->%d! (%d/%d)", n+1, iterations, from, to, i+1, (int)clusters.size());
									UINFO(msg.c_str());

									if(processState)
									{
										UINFO(msg.c_str());
										if(!processState->callback(msg))
										{
											return -1;
										}
									}
								}
							}
						}
					}
				}
			}
		}

		if(processState)
		{
			std::string msg = uFormat("Iteration %d/%d: Detected %d total loop closures!", n+1, iterations, (int)addedLinks.size()/2);
			UINFO(msg.c_str());
			if(!processState->callback(msg))
			{
				return -1;
			}
		}
		else
		{
			UINFO("Iteration %d/%d: Detected %d total loop closures!", n+1, iterations, (int)addedLinks.size()/2);
		}

		if(addedLinks.size() == 0)
		{
			break;
		}

		UINFO("Optimizing graph with new links (%d nodes, %d constraints)...",
				(int)poses.size(), (int)links.size());
		int fromId = _optimizeFromGraphEnd?poses.rbegin()->first:poses.begin()->first;
		poses = _graphOptimizer->optimize(fromId, poses, links, 0);
		if(poses.size() == 0)
		{
			UERROR("Optimization failed! Rejecting all loop closures...");
			loopClosuresAdded.clear();
			return -1;
		}
		UINFO("Optimizing graph with new links... done!");
	}
	UINFO("Total added %d loop closures.", (int)loopClosuresAdded.size());

	if(loopClosuresAdded.size())
	{
		for(std::list<Link>::iterator iter=loopClosuresAdded.begin(); iter!=loopClosuresAdded.end(); ++iter)
		{
			_memory->addLink(*iter, true);
		}
		// Update optimized poses
		for(std::map<int, Transform>::iterator iter=_optimizedPoses.begin(); iter!=_optimizedPoses.end(); ++iter)
		{
			std::map<int, Transform>::iterator jter = poses.find(iter->first);
			if(jter != poses.end())
			{
				iter->second = jter->second;
			}
		}
		std::map<int, Transform> tmp;
		// Update also the links if some have been added in WM
		_memory->getMetricConstraints(uKeysSet(_optimizedPoses), tmp, _constraints, false);
		// This will force rtabmap_ros to regenerate the global occupancy grid if there was one
		_memory->save2DMap(cv::Mat(), 0, 0, 0);
	}
	return (int)loopClosuresAdded.size();
}

bool Rtabmap::globalBundleAdjustment(
		int optimizerType,
		bool rematchFeatures,
		int iterations,
		float pixelVariance)
{
	if(!_optimizedPoses.empty() && !_constraints.empty())
	{
		int iterations = Parameters::defaultOptimizerIterations();
		float pixelVariance = Parameters::defaultg2oPixelVariance();
		ParametersMap params = _parameters;
		Parameters::parse(params, Parameters::kOptimizerIterations(), iterations);
		Parameters::parse(params, Parameters::kg2oPixelVariance(), pixelVariance);
		if(iterations > 0)
		{
			uInsert(params, ParametersPair(Parameters::kOptimizerIterations(), uNumber2Str(iterations)));
		}
		if(pixelVariance > 0.0f)
		{
			uInsert(params, ParametersPair(Parameters::kg2oPixelVariance(), uNumber2Str(pixelVariance)));
		}

		std::map<int, Signature> signatures;
		for(std::map<int, Transform>::iterator iter=_optimizedPoses.lower_bound(1); iter!=_optimizedPoses.end(); ++iter)
		{
			if(_memory->getSignature(iter->first))
			{
				signatures.insert(std::make_pair(iter->first, *_memory->getSignature(iter->first)));
			}
		}

		Optimizer * optimizer = Optimizer::create((Optimizer::Type)optimizerType, params);
		std::map<int, Transform> poses = optimizer->optimizeBA(
				_optimizeFromGraphEnd?_optimizedPoses.lower_bound(1)->first:_optimizedPoses.rbegin()->first,
				_optimizedPoses,
				_constraints,
				signatures,
				rematchFeatures);
		delete optimizer;

		if(poses.empty())
		{
			UERROR("Optimization failed!");
		}
		else
		{
			_optimizedPoses = poses;
			// This will force rtabmap_ros to regenerate the global occupancy grid if there was one
			_memory->save2DMap(cv::Mat(), 0, 0, 0);
			return true;
		}
	}
	else
	{
		UERROR("Optimized poses (%ld) or constraints (%ld) are empty!", _optimizedPoses.size(), _constraints.size());
	}
	return false;
}

int Rtabmap::cleanupLocalGrids(
		const std::map<int, Transform> & poses,
		const cv::Mat & map,
		float xMin,
		float yMin,
		float cellSize,
		int cropRadius,
		bool filterScans)
{
	if(_memory)
	{
		return _memory->cleanupLocalGrids(
				poses,
				map,
				xMin,
				yMin,
				cellSize,
				cropRadius,
				filterScans);
	}
	return -1;
}

int Rtabmap::refineLinks()
{
	if(!_rgbdSlamMode)
	{
		UERROR("Refining links can be done only in RGBD-SLAM mode.");
		return -1;
	}

	std::list<Link> linksRefined;

	std::map<int, Transform> poses;
	std::multimap<int, Link> links;
	std::map<int, Signature> signatures;
	this->getGraph(poses, links, false, true, &signatures);

	int i=0;
	for(std::multimap<int, Link>::iterator iter=links.lower_bound(1); iter!= links.end(); ++iter)
	{
		int from = iter->second.from();
		int to = iter->second.to();

		UASSERT(signatures.find(from) != signatures.end());
		UASSERT(signatures.find(to) != signatures.end());

		RegistrationInfo info;
		// use signatures instead of IDs because some signatures may not be in WM
		Transform t = _memory->computeTransform(signatures.at(from), signatures.at(to), iter->second.transform(), &info);

		if(!t.isNull())
		{
			linksRefined.push_back(Link(from, to, iter->second.type(), t, info.covariance.inv()));
			UINFO("Refined link %d->%d! (%d/%d)", from, to, ++i, (int)links.size());
		}
	}
	UINFO("Total refined %d links.", (int)linksRefined.size());

	if(linksRefined.size())
	{
		for(std::list<Link>::iterator iter=linksRefined.begin(); iter!=linksRefined.end(); ++iter)
		{
			_memory->updateLink(*iter, true);
		}
	}
	return (int)linksRefined.size();
}

bool Rtabmap::addLink(const Link & link)
{
	const Transform & t = link.transform();
	if(!_rgbdSlamMode)
	{
		UERROR("Adding new link can be done only in RGBD-SLAM mode.");
		return false;
	}
	if(!_memory)
	{
		UERROR("Memory is not initialized.");
		return false;
	}
	if(t.isNull())
	{
		UERROR("Link's transform is null!");
		return false;
	}
	if(_memory->isIncremental())
	{
		if(_memory->getSignature(link.from()) == 0)
		{
			UERROR("Link's \"from id\" %d is not in working memory", link.from());
			return false;
		}
		if(_memory->getSignature(link.to()) == 0)
		{
			UERROR("Link's \"to id\" %d is not in working memory", link.to());
			return false;
		}

		if(_optimizedPoses.find(link.from()) == _optimizedPoses.end() &&
		   _optimizedPoses.find(link.to()) == _optimizedPoses.end())
		{
			UERROR("Neither nodes %d or %d are in the local graph (size=%d). One of the 2 nodes should be in the local graph.", (int)_optimizedPoses.size(), link.from(), link.to());
			return false;
		}

		// add temporary the link
		if(!_memory->addLink(link))
		{
			UERROR("Cannot add new link %d->%d to memory", link.from(), link.to());
			return false;
		}

		// optimize with new link
		std::map<int, Transform> poses = _optimizedPoses;
		std::multimap<int, Link> links;
		cv::Mat covariance;
		optimizeCurrentMap(this->getLastLocationId(), false, poses, covariance, &links);

		if(poses.find(link.from()) == poses.end())
		{
			UERROR("Link's \"from id\" %d is not in the graph (size=%d)", link.from(), (int)poses.size());
			_memory->removeLink(link.from(), link.to());
			return false;
		}
		if(poses.find(link.to()) == poses.end())
		{
			UERROR("Link's \"to id\" %d is not in the graph (size=%d)", link.to(), (int)poses.size());
			_memory->removeLink(link.from(), link.to());
			return false;
		}

		std::string msg;
		if(poses.empty())
		{
			msg = uFormat("Rejecting edge %d->%d because graph optimization has failed!", link.from(), link.to());
		}
		else if(_optimizationMaxError > 0.0f)
		{
			float maxLinearError = 0.0f;
			float maxLinearErrorRatio = 0.0f;
			float maxAngularError = 0.0f;
			float maxAngularErrorRatio = 0.0f;
			const Link * maxLinearLink = 0;
			const Link * maxAngularLink = 0;

			graph::computeMaxGraphErrors(
					poses,
					links,
					maxLinearErrorRatio,
					maxAngularErrorRatio,
					maxLinearError,
					maxAngularError,
					&maxLinearLink,
					&maxAngularLink);
			if(maxLinearLink)
			{
				UINFO("Max optimization linear error = %f m (link %d->%d)", maxLinearError, maxLinearLink->from(), maxLinearLink->to());
				if(maxLinearErrorRatio > _optimizationMaxError)
				{
					msg = uFormat("Rejecting edge %d->%d because "
							  "graph error is too large after optimization (%f m for edge %d->%d with ratio %f > std=%f m). "
							  "\"%s\" is %f.",
							  link.from(),
							  link.to(),
							  maxLinearError,
							  maxLinearLink->from(),
							  maxLinearLink->to(),
							  maxLinearErrorRatio,
							  sqrt(maxLinearLink->transVariance()),
							  Parameters::kRGBDOptimizeMaxError().c_str(),
							  _optimizationMaxError);
				}
			}
			else if(maxAngularLink)
			{
				UINFO("Max optimization angular error = %f deg (link %d->%d)", maxAngularError*180.0f/M_PI, maxAngularLink->from(), maxAngularLink->to());
				if(maxAngularErrorRatio > _optimizationMaxError)
				{
					msg = uFormat("Rejecting edge %d->%d because "
							  "graph error is too large after optimization (%f deg for edge %d->%d with ratio %f > std=%f deg). "
							  "\"%s\" is %f m.",
							  link.from(),
							  link.to(),
							  maxAngularError*180.0f/M_PI,
							  maxAngularLink->from(),
							  maxAngularLink->to(),
							  maxAngularErrorRatio,
							  sqrt(maxAngularLink->rotVariance()),
							  Parameters::kRGBDOptimizeMaxError().c_str(),
							  _optimizationMaxError);
				}
			}
		}
		if(!msg.empty())
		{
			UERROR("%s", msg.c_str());
			_memory->removeLink(link.from(), link.to());
			return false;
		}

		// Update optimized poses
		for(std::map<int, Transform>::iterator iter=_optimizedPoses.begin(); iter!=_optimizedPoses.end(); ++iter)
		{
			std::map<int, Transform>::iterator jter = poses.find(iter->first);
			if(jter != poses.end())
			{
				iter->second = jter->second;
			}
		}
		if(!_optimizeFromGraphEnd)
		{
			_mapCorrection = _optimizedPoses.rbegin()->second * _memory->getSignature(_optimizedPoses.rbegin()->first)->getPose().inverse();
		}

		std::map<int, Transform> tmp;
		// Update also the links if some have been added in WM
		_memory->getMetricConstraints(uKeysSet(_optimizedPoses), tmp, _constraints, false);
		// This will force rtabmap_ros to regenerate the global occupancy grid if there was one
		_memory->save2DMap(cv::Mat(), 0, 0, 0);

		return true;
	}
	else // localization mode
	{
		int oldestId = link.from()>link.to()?link.to():link.from();
		int newestId = link.from()<link.to()?link.to():link.from();

		if(_memory->getSignature(oldestId) == 0)
		{
			UERROR("Link's id %d is not in working memory", oldestId);
			return false;
		}
		if(_optimizedPoses.find(oldestId) == _optimizedPoses.end())
		{
			UERROR("Link's id %d is not in the optimized graph (_optimizedPoses=%d)", oldestId, (int)_optimizedPoses.size());
			return false;
		}
		if(_optimizeFromGraphEnd)
		{
			UERROR("Adding link with %s=true in localization mode is not supported.", Parameters::kRGBDOptimizeFromGraphEnd().c_str());
			return false;
		}
		if(_odomCachePoses.find(newestId) == _odomCachePoses.end())
		{
			if(!_odomCachePoses.empty())
			{
				UERROR("Link's id %d is not in the odometry cache (oldest=%d, newest=%d, %s=%d)",
						newestId,
						_odomCachePoses.begin()->first,
						_odomCachePoses.rbegin()->first,
						Parameters::kRGBDMaxOdomCacheSize().c_str(),
						_maxOdomCacheSize);
			}
			else
			{
				UERROR("Link's id %d is not in the odometry cache (%s=%d).",
						newestId,
						Parameters::kRGBDMaxOdomCacheSize().c_str(),
						_maxOdomCacheSize);
			}
			return false;
		}

		// Verify if the new localization is valid by checking if there is
		// not too much deformation using current odometry poses
		// This will also refine localization links

		std::map<int, Transform> poses = _odomCachePoses;
		std::multimap<int, Link> constraints = _odomCacheConstraints;
		constraints.insert(std::make_pair(link.from(), link));
		for(std::multimap<int, Link>::iterator iter=constraints.begin(); iter!=constraints.end(); ++iter)
		{
			std::map<int, Transform>::iterator iterPose = _optimizedPoses.find(iter->second.to());
			if(iterPose != _optimizedPoses.end() && poses.find(iterPose->first) == poses.end())
			{
				poses.insert(*iterPose);
				// make the poses in the map fixed
				constraints.insert(std::make_pair(iterPose->first, Link(iterPose->first, iterPose->first, Link::kPosePrior, iterPose->second, cv::Mat::eye(6,6, CV_64FC1)*999999)));
			}
		}

		std::map<int, Transform> posesOut;
		std::multimap<int, Link> edgeConstraintsOut;
		bool priorsIgnored = _graphOptimizer->priorsIgnored();
		_graphOptimizer->setPriorsIgnored(false); //temporary set false to use priors above to fix nodes of the map
		_graphOptimizer->getConnectedGraph(newestId, poses, constraints, posesOut, edgeConstraintsOut);
		std::map<int, Transform> optPoses = _graphOptimizer->optimize(poses.begin()->first, posesOut, edgeConstraintsOut);
		_graphOptimizer->setPriorsIgnored(priorsIgnored); // set back

		bool rejectLocalization = false;
		if(optPoses.empty())
		{
			UWARN("Optimization failed, rejecting localization!");
			rejectLocalization = true;
		}
		else if(_optimizationMaxError > 0.0f)
		{
			UINFO("Compute max graph errors...");
			float maxLinearError = 0.0f;
			float maxLinearErrorRatio = 0.0f;
			float maxAngularError = 0.0f;
			float maxAngularErrorRatio = 0.0f;
			const Link * maxLinearLink = 0;
			const Link * maxAngularLink = 0;
			graph::computeMaxGraphErrors(
					optPoses,
					edgeConstraintsOut,
					maxLinearErrorRatio,
					maxAngularErrorRatio,
					maxLinearError,
					maxAngularError,
					&maxLinearLink,
					&maxAngularLink,
					_graphOptimizer->isSlam2d());
			if(maxLinearLink == 0 && maxAngularLink==0)
			{
				UWARN("Could not compute graph errors! Wrong loop closures could be accepted!");
			}

			if(maxLinearLink)
			{
				UINFO("Max optimization linear error = %f m (link %d->%d, var=%f, ratio error/std=%f)", maxLinearError, maxLinearLink->from(), maxLinearLink->to(), maxLinearLink->transVariance(), maxLinearError/sqrt(maxLinearLink->transVariance()));
				if(maxLinearErrorRatio > _optimizationMaxError)
				{
					UWARN("Rejecting localization (%d <-> %d) in this "
							"iteration because a wrong loop closure has been "
							"detected after graph optimization, resulting in "
							"a maximum graph error ratio of %f (edge %d->%d, type=%d, abs error=%f m, stddev=%f). The "
							"maximum error ratio parameter \"%s\" is %f of std deviation.",
							link.from(),
							link.to(),
							maxLinearErrorRatio,
							maxLinearLink->from(),
							maxLinearLink->to(),
							maxLinearLink->type(),
							maxLinearError,
							sqrt(maxLinearLink->transVariance()),
							Parameters::kRGBDOptimizeMaxError().c_str(),
							_optimizationMaxError);
					rejectLocalization = true;
				}
			}
			if(maxAngularLink)
			{
				UINFO("Max optimization angular error = %f deg (link %d->%d, var=%f, ratio error/std=%f)", maxAngularError*180.0f/CV_PI, maxAngularLink->from(), maxAngularLink->to(), maxAngularLink->rotVariance(), maxAngularError/sqrt(maxAngularLink->rotVariance()));
				if(maxAngularErrorRatio > _optimizationMaxError)
				{
					UWARN("Rejecting localization (%d <-> %d) in this "
							"iteration because a wrong loop closure has been "
							"detected after graph optimization, resulting in "
							"a maximum graph error ratio of %f (edge %d->%d, type=%d, abs error=%f deg, stddev=%f). The "
							"maximum error ratio parameter \"%s\" is %f of std deviation.",
							link.from(),
							link.to(),
							maxAngularErrorRatio,
							maxAngularLink->from(),
							maxAngularLink->to(),
							maxAngularLink->type(),
							maxAngularError*180.0f/CV_PI,
							sqrt(maxAngularLink->rotVariance()),
							Parameters::kRGBDOptimizeMaxError().c_str(),
							_optimizationMaxError);
					rejectLocalization = true;
				}
			}
		}

		if(!rejectLocalization)
		{
			Transform newOptPoseInv = optPoses.at(link.from()).inverse();
			Transform newT = newOptPoseInv * optPoses.at(link.to());
			Link linkTmp = link;
			linkTmp.setTransform(newT);
			if(oldestId == link.from())
			{
				_lastLocalizationPose = _optimizedPoses.at(link.from()) * linkTmp.transform();
				_odomCacheConstraints.insert(std::make_pair(linkTmp.to(), linkTmp.inverse()));
			}
			else
			{
				_lastLocalizationPose = _optimizedPoses.at(link.to()) * linkTmp.transform().inverse();
				_odomCacheConstraints.insert(std::make_pair(linkTmp.from(), linkTmp));
			}
			UINFO("Set _lastLocalizationPose=%s", _lastLocalizationPose.prettyPrint().c_str());
			if(_graphOptimizer->isSlam2d())
			{
				// transform constraint to 2D
				_lastLocalizationPose = _lastLocalizationPose.to3DoF();
			}
			Transform odomPose = _odomCachePoses.find(newestId)->second;
			_mapCorrection = _lastLocalizationPose * odomPose.inverse();
			_lastLocalizationNodeId = oldestId;
			return true;
		}
	}
	return false;
}

cv::Mat Rtabmap::getInformation(const cv::Mat & covariance) const
{
	cv::Mat information = covariance.inv();
	if(_loopCovLimited)
	{
		const std::vector<double> & odomMaxInf = _memory->getOdomMaxInf();
		if(odomMaxInf.size() == 6)
		{
			for(int i=0; i<6; ++i)
			{
				if(information.at<double>(i,i) > odomMaxInf[i])
				{
					information.at<double>(i,i) = odomMaxInf[i];
				}
			}
		}
	}
	return information;
}

void Rtabmap::clearPath(int status)
{
	UINFO("status=%d", status);
	_pathStatus = status;
	_path.clear();
	_pathCurrentIndex=0;
	_pathGoalIndex = 0;
	_pathTransformToGoal.setIdentity();
	_pathUnreachableNodes.clear();
	_pathStuckCount = 0;
	_pathStuckDistance = 0.0f;
	if(_memory)
	{
		_memory->removeAllVirtualLinks();
	}
}

// return true if path is updated
bool Rtabmap::computePath(int targetNode, bool global)
{
	this->clearPath(0);

	if(targetNode>0)
	{
		UINFO("Planning a path to node %d (global=%d)", targetNode, global?1:0);
	}
	else
	{
		UINFO("Planning a path to landmark %d (global=%d)", -targetNode, global?1:0);
	}

	if(!_rgbdSlamMode)
	{
		UWARN("A path can only be computed in RGBD-SLAM mode");
		return false;
	}

	UTimer totalTimer;
	UTimer timer;
	Transform transformToLandmark = Transform::getIdentity();

	// No need to optimize the graph
	if(_memory)
	{
		int currentNode = 0;
		if(_memory->isIncremental())
		{
			if(!_memory->getLastWorkingSignature())
			{
				UWARN("Working memory is empty... cannot compute a path");
				return false;
			}
			currentNode = _memory->getLastWorkingSignature()->id();
		}
		else
		{
			if(_lastLocalizationPose.isNull() || _optimizedPoses.empty())
			{
				UWARN("Last localization pose is null or optimized graph is empty... cannot compute a path");
				return false;
			}
			if(_optimizedPoses.begin()->first < 0)
			{
				std::map<int, Transform> poses(_optimizedPoses.lower_bound(1), _optimizedPoses.end());
				currentNode = graph::findNearestNode(poses, _lastLocalizationPose);
			}
			else
			{
				currentNode = graph::findNearestNode(_optimizedPoses, _lastLocalizationPose);
			}
		}
		if(currentNode && targetNode)
		{
			std::list<std::pair<int, Transform> > path = graph::computePath(
					currentNode,
					targetNode,
					_memory,
					global,
					false,
					_pathLinearVelocity,
					_pathAngularVelocity);

			//transform in current referential
			Transform t = uValue(_optimizedPoses, currentNode, Transform::getIdentity());
			_path.resize(path.size());
			int oi = 0;
			for(std::list<std::pair<int, Transform> >::iterator iter=path.begin(); iter!=path.end();++iter)
			{
				if(iter->first > 0)
				{
					// just keep nodes in the path
					_path[oi].first = iter->first;
					_path[oi++].second = t * iter->second;
				}
			}
			_path.resize(oi);
			if(!_path.empty() && !path.empty() && path.rbegin()->first < 0)
			{
				transformToLandmark = _path.back().second.inverse() * t * path.rbegin()->second;
			}
		}
		else if(currentNode == 0)
		{
			UWARN("We should be localized before planning.");
		}
	}
	UINFO("Total planning time = %fs (%d nodes, %f m long)", totalTimer.ticks(), (int)_path.size(), graph::computePathLength(_path));

	if(_path.size() == 0)
	{
		_path.clear();
		UWARN("Cannot compute a path!");
		return false;
	}
	else
	{
		UINFO("Path generated! Size=%d", (int)_path.size());
		if(ULogger::level() == ULogger::kInfo)
		{
			std::stringstream stream;
			for(unsigned int i=0; i<_path.size(); ++i)
			{
				stream << _path[i].first;
				if(i+1 < _path.size())
				{
					stream << " ";
				}
			}
			UINFO("Path = [%s]", stream.str().c_str());
		}
		if(_goalsSavedInUserData)
		{
			// set goal to latest signature
			std::string goalStr = uFormat("GOAL:%d", targetNode);

			// use label is exist
			if(_memory->getSignature(targetNode))
			{
				if(!_memory->getSignature(targetNode)->getLabel().empty())
				{
					goalStr = std::string("GOAL:")+_memory->getSignature(targetNode)->getLabel();
				}
			}
			else if(global)
			{
				std::map<int, std::string> labels = _memory->getAllLabels();
				std::map<int, std::string>::iterator iter = labels.find(targetNode);
				if(iter != labels.end() && !iter->second.empty())
				{
					goalStr = std::string("GOAL:")+labels.at(targetNode);
				}
			}
			setUserData(0, cv::Mat(1, int(goalStr.size()+1), CV_8SC1, (void *)goalStr.c_str()).clone());
		}
		_pathTransformToGoal = transformToLandmark;

		updateGoalIndex();
		return _path.size() || _pathStatus > 0;
	}

	return false;
}

bool Rtabmap::computePath(const Transform & targetPose, float tolerance)
{
	this->clearPath(0);

	UINFO("Planning a path to pose %s ", targetPose.prettyPrint().c_str());
	if(tolerance < 0.0f)
	{
		tolerance = _localRadius;
	}

	std::list<std::pair<int, Transform> > pathPoses;

	if(!_rgbdSlamMode)
	{
		UWARN("This method can only be used in RGBD-SLAM mode");
		return false;
	}

	//Find the nearest node
	UTimer timer;
	std::map<int, Transform> nodes = _optimizedPoses;
	std::multimap<int, int> links;
	for(std::map<int, Transform>::iterator iter=nodes.upper_bound(0); iter!=nodes.end(); ++iter)
	{
		const Signature * s = _memory->getSignature(iter->first);
		UASSERT(s);
		for(std::map<int, Link>::const_iterator jter=s->getLinks().begin(); jter!=s->getLinks().end(); ++jter)
		{
			// only add links for which poses are in "nodes"
			if(jter->second.from() != jter->second.to() && uContains(nodes, jter->second.to()))
			{
				links.insert(std::make_pair(jter->second.from(), jter->second.to()));
				//links.insert(std::make_pair(jter->second.to(), jter->second.from())); // <-> (commented: already added when iterating in nodes)
			}
		}
	}
	UINFO("Time getting links = %fs", timer.ticks());

	int currentNode = 0;
	if(_memory->isIncremental())
	{
		if(!_memory->getLastWorkingSignature())
		{
			UWARN("Working memory is empty... cannot compute a path");
			return false;
		}
		currentNode = _memory->getLastWorkingSignature()->id();
	}
	else
	{
		if(_lastLocalizationPose.isNull() || _optimizedPoses.empty())
		{
			UWARN("Last localization pose is null... cannot compute a path");
			return false;
		}
		if(_optimizedPoses.begin()->first < 0)
		{
			std::map<int, Transform> poses(_optimizedPoses.lower_bound(1), _optimizedPoses.end());
			currentNode = graph::findNearestNode(poses, _lastLocalizationPose);
		}
		else
		{
			currentNode = graph::findNearestNode(_optimizedPoses, _lastLocalizationPose);
		}
	}

	int nearestId;
	if(!_lastLocalizationPose.isNull() && _lastLocalizationPose.getDistance(targetPose) < tolerance)
	{
		// target can be reached from the current node
		nearestId = currentNode;
	}
	else
	{
		nearestId = rtabmap::graph::findNearestNode(nodes, targetPose);
	}
	UINFO("Nearest node found=%d ,%fs", nearestId, timer.ticks());
	if(nearestId > 0)
	{
		if(tolerance != 0.0f && targetPose.getDistance(nodes.at(nearestId)) > tolerance)
		{
			UWARN("Cannot plan farther than %f m from the graph! (distance=%f m from node %d)",
					tolerance, targetPose.getDistance(nodes.at(nearestId)), nearestId);
		}
		else
		{
			UINFO("Computing path from location %d to %d", currentNode, nearestId);
			UTimer timer;
			_path = uListToVector(rtabmap::graph::computePath(nodes, links, currentNode, nearestId));
			UINFO("A* time = %fs", timer.ticks());

			if(_path.size() == 0)
			{
				UWARN("Cannot compute a path!");
			}
			else
			{
				UINFO("Path generated! Size=%d", (int)_path.size());
				if(ULogger::level() == ULogger::kInfo)
				{
					std::stringstream stream;
					for(unsigned int i=0; i<_path.size(); ++i)
					{
						stream << _path[i].first;
						if(i+1 < _path.size())
						{
							stream << " ";
						}
					}
					UINFO("Path = [%s]", stream.str().c_str());
				}

				UASSERT(uContains(nodes, _path.back().first));
				_pathTransformToGoal = nodes.at(_path.back().first).inverse() * targetPose;

				updateGoalIndex();

				return true;
			}
		}
	}
	else
	{
		UWARN("Nearest node not found in graph (size=%d) for pose %s", (int)nodes.size(), targetPose.prettyPrint().c_str());
	}

	return false;
}

std::vector<std::pair<int, Transform> > Rtabmap::getPathNextPoses() const
{
	std::vector<std::pair<int, Transform> > poses;
	if(_path.size())
	{
		UASSERT(_pathCurrentIndex < _path.size() && _pathGoalIndex < _path.size());
		poses.resize(_pathGoalIndex-_pathCurrentIndex+1);
		int oi=0;
		for(unsigned int i=_pathCurrentIndex; i<=_pathGoalIndex; ++i)
		{
			std::map<int, Transform>::const_iterator iter = _optimizedPoses.find(_path[i].first);
			if(iter != _optimizedPoses.end())
			{
				poses[oi++] = *iter;
			}
			else
			{
				break;
			}
		}
		poses.resize(oi);
	}
	return poses;
}

std::vector<int> Rtabmap::getPathNextNodes() const
{
	std::vector<int> ids;
	if(_path.size())
	{
		UASSERT(_pathCurrentIndex < _path.size() && _pathGoalIndex < _path.size());
		ids.resize(_pathGoalIndex-_pathCurrentIndex+1);
		int oi = 0;
		for(unsigned int i=_pathCurrentIndex; i<=_pathGoalIndex; ++i)
		{
			std::map<int, Transform>::const_iterator iter = _optimizedPoses.find(_path[i].first);
			if(iter != _optimizedPoses.end())
			{
				ids[oi++] = iter->first;
			}
			else
			{
				break;
			}
		}
		ids.resize(oi);
	}
	return ids;
}

int Rtabmap::getPathCurrentGoalId() const
{
	if(_path.size())
	{
		UASSERT(_pathGoalIndex <= _path.size());
		return _path[_pathGoalIndex].first;
	}
	return 0;
}

void Rtabmap::updateGoalIndex()
{
	if(!_rgbdSlamMode)
	{
		UWARN("This method can on be used in RGBD-SLAM mode!");
		return;
	}

	if( _memory && _path.size())
	{
		// remove all previous virtual links
		for(unsigned int i=0; i<_pathCurrentIndex && i<_path.size(); ++i)
		{
			const Signature * s = _memory->getSignature(_path[i].first);
			if(s)
			{
				_memory->removeVirtualLinks(s->id());
			}
		}

		// for the current index, only keep the newest virtual link
		// This will make sure that the path is still connected even
		// if the new signature is removed (e.g., because of a small displacement)
		UASSERT(_pathCurrentIndex < _path.size());
		const Signature * currentIndexS = _memory->getSignature(_path[_pathCurrentIndex].first);
		UASSERT_MSG(currentIndexS != 0, uFormat("_path[%d].first=%d", _pathCurrentIndex, _path[_pathCurrentIndex].first).c_str());
		std::multimap<int, Link> links = currentIndexS->getLinks(); // make a copy
		bool latestVirtualLinkFound = false;
		for(std::multimap<int, Link>::reverse_iterator iter=links.rbegin(); iter!=links.rend(); ++iter)
		{
			if(iter->second.type() == Link::kVirtualClosure)
			{
				if(latestVirtualLinkFound)
				{
					_memory->removeLink(currentIndexS->id(), iter->first);
				}
				else
				{
					latestVirtualLinkFound = true;
				}
			}
		}

		// Make sure the next signatures on the path are linked together
		float distanceSoFar = 0.0f;
		for(unsigned int i=_pathCurrentIndex+1;
			i<_path.size();
			++i)
		{
			if(i>0)
			{
				if(_localRadius > 0.0f)
				{
					distanceSoFar += _path[i-1].second.getDistance(_path[i].second);
				}
				if(distanceSoFar <= _localRadius)
				{
					if(_path[i].first != _path[i-1].first)
					{
						const Signature * s = _memory->getSignature(_path[i].first);
						if(s)
						{
							if(!s->hasLink(_path[i-1].first) && _memory->getSignature(_path[i-1].first) != 0)
							{
								Transform virtualLoop = _path[i].second.inverse() * _path[i-1].second;
								_memory->addLink(Link(_path[i].first, _path[i-1].first, Link::kVirtualClosure, virtualLoop, cv::Mat::eye(6,6,CV_64FC1)*0.01)); // on the optimized path
								UINFO("Added Virtual link between %d and %d", _path[i-1].first, _path[i].first);
							}
						}
					}
				}
				else
				{
					break;
				}
			}
		}

		UDEBUG("current node = %d current goal = %d", _path[_pathCurrentIndex].first, _path[_pathGoalIndex].first);
		Transform currentPose;
		if(_memory->isIncremental())
		{
			if(_memory->getLastWorkingSignature() == 0 ||
			   !uContains(_optimizedPoses, _memory->getLastWorkingSignature()->id()))
			{
				UERROR("Last node is null in memory or not in optimized poses. Aborting the plan...");
				this->clearPath(-1);
				return;
			}
			currentPose = _optimizedPoses.at(_memory->getLastWorkingSignature()->id());
		}
		else
		{
			if(_lastLocalizationPose.isNull())
			{
				UERROR("Last localization pose is null. Aborting the plan...");
				this->clearPath(-1);
				return;
			}
			currentPose = _lastLocalizationPose;
		}

		int goalId = _path.back().first;
		if(uContains(_optimizedPoses, goalId))
		{
			//use local position to know if the goal is reached
			float d = currentPose.getDistance(_optimizedPoses.at(goalId)*_pathTransformToGoal);
			if(d < _goalReachedRadius)
			{
				UINFO("Goal %d reached!", goalId);
				this->clearPath(1);
			}
		}

		if(_path.size())
		{
			//Always check if the farthest node is accessible in local map (max to local space radius if set)
			unsigned int goalIndex = _pathCurrentIndex;
			float distanceFromCurrentNode = 0.0f;
			bool sameGoalIndex = false;
			for(unsigned int i=_pathCurrentIndex+1; i<_path.size(); ++i)
			{
				if(uContains(_optimizedPoses, _path[i].first))
				{
					if(_localRadius > 0.0f)
					{
						distanceFromCurrentNode = _path[_pathCurrentIndex].second.getDistance(_path[i].second);
					}

					if((goalIndex == _pathCurrentIndex && i == _path.size()-1) ||
					   _pathUnreachableNodes.find(i) == _pathUnreachableNodes.end())
					{
						if(distanceFromCurrentNode <= _localRadius)
						{
							goalIndex = i;
						}
						else
						{
							break;
						}
					}
				}
				else
				{
					break;
				}
			}
			UASSERT(_pathGoalIndex < _path.size() && goalIndex < _path.size());
			if(_pathGoalIndex != goalIndex)
			{
				UINFO("Updated current goal from %d to %d (%d/%d)",
						(int)_path[_pathGoalIndex].first, _path[goalIndex].first, (int)goalIndex+1, (int)_path.size());
				_pathGoalIndex = goalIndex;
			}
			else
			{
				sameGoalIndex = true;
			}

			// update nearest pose in the path
			unsigned int nearestNodeIndex = 0;
			float distance = -1.0f;
			bool sameCurrentIndex = false;
			UASSERT(_pathGoalIndex < _path.size());
			for(unsigned int i=_pathCurrentIndex; i<=_pathGoalIndex; ++i)
			{
				std::map<int, Transform>::iterator iter = _optimizedPoses.find(_path[i].first);
				if(iter != _optimizedPoses.end())
				{
					float d = currentPose.getDistanceSquared(iter->second);
					if(distance == -1.0f || distance > d)
					{
						distance = d;
						nearestNodeIndex = i;
					}
				}
			}
			if(distance < 0)
			{
				UERROR("The nearest pose on the path not found! Aborting the plan...");
				this->clearPath(-1);
			}
			else
			{
				UDEBUG("Nearest node = %d", _path[nearestNodeIndex].first);
			}
			if(distance >= 0 && nearestNodeIndex != _pathCurrentIndex)
			{
				_pathCurrentIndex = nearestNodeIndex;
				_pathUnreachableNodes.erase(nearestNodeIndex); // if we are on it, it is reachable
			}
			else
			{
				sameCurrentIndex = true;
			}

			bool isStuck = false;
			if(sameGoalIndex && sameCurrentIndex && _pathStuckIterations>0)
			{
				float distanceToCurrentGoal = 0.0f;
				std::map<int, Transform>::iterator iter = _optimizedPoses.find(_path[_pathGoalIndex].first);
				if(iter != _optimizedPoses.end())
				{
					if(_pathGoalIndex == _pathCurrentIndex &&
						_pathGoalIndex == _path.size()-1)
					{
						distanceToCurrentGoal = currentPose.getDistanceSquared(iter->second*_pathTransformToGoal);
					}
					else
					{
						distanceToCurrentGoal = currentPose.getDistanceSquared(iter->second);
					}
				}

				if(distanceToCurrentGoal > 0.0f)
				{
					if(distanceToCurrentGoal >= _pathStuckDistance)
					{
						// we are not approaching the goal
						isStuck = true;
						if(_pathStuckDistance == 0.0f)
						{
							_pathStuckDistance = distanceToCurrentGoal;
						}
					}
				}
				else
				{
					// no nodes available, cannot plan
					isStuck = true;
				}
			}

			if(isStuck && ++_pathStuckCount > _pathStuckIterations)
			{
				UWARN("Current goal %d not reached since %d iterations (\"RGBD/PlanStuckIterations\"=%d), mark that node as unreachable.",
						_path[_pathGoalIndex].first,
						_pathStuckCount,
						_pathStuckIterations);
				_pathStuckCount = 0;
				_pathStuckDistance = 0.0;
				_pathUnreachableNodes.insert(_pathGoalIndex);
				// select previous reachable one
				while(_pathUnreachableNodes.find(_pathGoalIndex) != _pathUnreachableNodes.end())
				{
					if(_pathGoalIndex == 0 || --_pathGoalIndex <= _pathCurrentIndex)
					{
						// plan failed!
						UERROR("No upcoming nodes on the path are reachable! Aborting the plan...");
						this->clearPath(-1);
						return;
					}
				}
			}
			else if(!isStuck)
			{
				_pathStuckCount = 0;
				_pathStuckDistance = 0.0;
			}
		}
	}
}

void Rtabmap::createGlobalScanMap()
{
	UDEBUG("Creating global scan map (if scans are available)");
	_globalScanMap.clear();
	_globalScanMapPoses.clear();
	std::vector<int> scanIndices;
	std::map<int, Transform> scanViewpoints;
	for(std::map<int, Transform>::iterator iter=_optimizedPoses.begin(); iter!=_optimizedPoses.end(); ++iter)
	{
		SensorData data = _memory->getNodeData(iter->first, false, true, false, false);
		if(!data.laserScanCompressed().empty())
		{
			LaserScan scan;
			data.uncompressDataConst(0, 0, &scan, 0, 0, 0, 0);
			if(!scan.empty())
			{
				UDEBUG("Adding scan %d (format=%s, points=%d)", iter->first, scan.formatName().c_str(), scan.size());
				scan = util3d::transformLaserScan(scan, iter->second*scan.localTransform());
				if(_globalScanMap.empty() || _globalScanMap.format() == scan.format())
				{
					_globalScanMap += scan;
					_globalScanMapPoses.insert(*iter);
					scanViewpoints.insert(std::make_pair(iter->first, iter->second * scan.localTransform()));
					scanIndices.resize(_globalScanMap.size(), iter->first);
				}
				else
				{
					UWARN("Incompatible scan formats (%s vs %s), cannot create global scan map.",
							_globalScanMap.formatName().c_str(),
							scan.formatName().c_str());
					_globalScanMap.clear();
					_globalScanMapPoses.clear();
					break;
				}
			}
			else
			{
				UDEBUG("Ignored %d (scan is empty), pose still added.", iter->first);
				_globalScanMapPoses.insert(*iter);
			}
		}
		else
		{
			UDEBUG("Ignored %d (no scan), pose still added.", iter->first);
			_globalScanMapPoses.insert(*iter);
		}
	}
	if(_globalScanMap.size() > 3)
	{
		float voxelSize = 0.0f;
		int normalK = 0;
		float normalRadius = 0.0f;
		Parameters::parse(_parameters, Parameters::kMemLaserScanVoxelSize(), voxelSize);
		Parameters::parse(_parameters, Parameters::kMemLaserScanNormalK(), normalK);
		Parameters::parse(_parameters, Parameters::kMemLaserScanNormalRadius(), normalRadius);

		if(voxelSize > 0.0f)
		{
			LaserScan voxelScan = util3d::commonFiltering(_globalScanMap, 1, 0, 0, voxelSize, normalK, normalRadius);
			if(voxelScan.hasNormals())
			{
				// adjust with point of views
				util3d::adjustNormalsToViewPoints(
						scanViewpoints,
						_globalScanMap,
						scanIndices,
						voxelScan);
			}
			_globalScanMap = voxelScan;
		}

		UINFO("Global scan map has been assembled (size=%d points, %d poses) "
				"for proximity detection (only in localization mode %s=false and with %s=true)",
				(int)_globalScanMap.size(),
				(int)_globalScanMapPoses.size(),
				Parameters::kMemIncrementalMemory().c_str(),
				Parameters::kRGBDProximityGlobalScanMap().c_str());

		//for debugging...
		if(!_globalScanMap.empty() && ULogger::level() == ULogger::kDebug)
		{
			if(!_wDir.empty())
			{
				UWARN("Saving %s/rtabmap_global_scan_map.pcd (only saved when logger level is debug)", _wDir.c_str());
				pcl::PCLPointCloud2::Ptr cloud2 = util3d::laserScanToPointCloud2(_globalScanMap);
				pcl::io::savePCDFile(_wDir+"/rtabmap_global_scan_map.pcd", *cloud2);
			}
			else
			{
				UWARN("%s is enabled and logger is debug, but %s is not set, cannot save global scan map for debugging.",
						Parameters::kRGBDProximityGlobalScanMap().c_str(), Parameters::kRtabmapWorkingDirectory().c_str());
			}
		}
	}
	if(!_globalScanMap.empty() && _globalScanMap.size()<100)
	{
		UWARN("Ignoring global scan map because it is too small (%d points).", (int)_globalScanMap.size());
		_globalScanMap.clear();
		_globalScanMapPoses.clear();
	}
}

} // namespace rtabmap
