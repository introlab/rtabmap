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

#include <pcl/search/kdtree.h>
#include <pcl/filters/crop_box.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>

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
	_maxTimeAllowed(Parameters::defaultRtabmapTimeThr()), // 700 ms
	_maxMemoryAllowed(Parameters::defaultRtabmapMemoryThr()), // 0=inf
	_loopThr(Parameters::defaultRtabmapLoopThr()),
	_loopRatio(Parameters::defaultRtabmapLoopRatio()),
	_maxRetrieved(Parameters::defaultRtabmapMaxRetrieved()),
	_maxLocalRetrieved(Parameters::defaultRGBDMaxLocalRetrieved()),
	_rawDataKept(Parameters::defaultMemImageKept()),
	_statisticLogsBufferedInRAM(Parameters::defaultRtabmapStatisticLogsBufferedInRAM()),
	_statisticLogged(Parameters::defaultRtabmapStatisticLogged()),
	_statisticLoggedHeaders(Parameters::defaultRtabmapStatisticLoggedHeaders()),
	_rgbdSlamMode(Parameters::defaultRGBDEnabled()),
	_rgbdLinearUpdate(Parameters::defaultRGBDLinearUpdate()),
	_rgbdAngularUpdate(Parameters::defaultRGBDAngularUpdate()),
	_newMapOdomChangeDistance(Parameters::defaultRGBDNewMapOdomChangeDistance()),
	_neighborLinkRefining(Parameters::defaultRGBDNeighborLinkRefining()),
	_proximityByTime(Parameters::defaultRGBDProximityByTime()),
	_proximityBySpace(Parameters::defaultRGBDProximityBySpace()),
	_scanMatchingIdsSavedInLinks(Parameters::defaultRGBDScanMatchingIdsSavedInLinks()),
	_localRadius(Parameters::defaultRGBDLocalRadius()),
	_localImmunizationRatio(Parameters::defaultRGBDLocalImmunizationRatio()),
	_proximityMaxGraphDepth(Parameters::defaultRGBDProximityMaxGraphDepth()),
	_proximityMaxPaths(Parameters::defaultRGBDProximityMaxPaths()),
	_proximityMaxNeighbors(Parameters::defaultRGBDProximityPathMaxNeighbors()),
	_proximityFilteringRadius(Parameters::defaultRGBDProximityPathFilteringRadius()),
	_proximityRawPosesUsed(Parameters::defaultRGBDProximityPathRawPosesUsed()),
	_proximityAngle(Parameters::defaultRGBDProximityAngle()*M_PI/180.0f),
	_databasePath(""),
	_optimizeFromGraphEnd(Parameters::defaultRGBDOptimizeFromGraphEnd()),
	_optimizationMaxLinearError(Parameters::defaultRGBDOptimizeMaxError()),
	_startNewMapOnLoopClosure(Parameters::defaultRtabmapStartNewMapOnLoopClosure()),
	_goalReachedRadius(Parameters::defaultRGBDGoalReachedRadius()),
	_goalsSavedInUserData(Parameters::defaultRGBDGoalsSavedInUserData()),
	_pathStuckIterations(Parameters::defaultRGBDPlanStuckIterations()),
	_pathLinearVelocity(Parameters::defaultRGBDPlanLinearVelocity()),
	_pathAngularVelocity(Parameters::defaultRGBDPlanAngularVelocity()),
	_loopClosureHypothesis(0,0.0f),
	_highestHypothesis(0,0.0f),
	_lastProcessTime(0.0),
	_someNodesHaveBeenTransferred(false),
	_distanceTravelled(0.0f),
	_epipolarGeometry(0),
	_bayesFilter(0),
	_graphOptimizer(0),
	_memory(0),
	_foutFloat(0),
	_foutInt(0),
	_wDir(""),
	_mapCorrection(Transform::getIdentity()),
	_lastLocalizationNodeId(0),
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

void Rtabmap::init(const ParametersMap & parameters, const std::string & databasePath)
{
	UDEBUG("path=%s", databasePath.c_str());
	ParametersMap::const_iterator iter;
	if((iter=parameters.find(Parameters::kRtabmapWorkingDirectory())) != parameters.end())
	{
		this->setWorkingDirectory(iter->second.c_str());
	}

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

	// If not exist, create a memory
	if(!_memory)
	{
		_memory = new Memory(parameters);
		_memory->init(_databasePath, false, parameters, true);
	}

	// Parse all parameters
	this->parseParameters(parameters);

	if(_databasePath.empty())
	{
		_statisticLogged = false;
	}
	setupLogFiles(newDatabase);
}

void Rtabmap::init(const std::string & configFile, const std::string & databasePath)
{
	// fill ctrl struct with values from the configuration file
	ParametersMap param;// = Parameters::defaultParameters;

	if(!configFile.empty())
	{
		ULOGGER_DEBUG("Read parameters from = %s", configFile.c_str());
		Parameters::readINI(configFile, param);
	}

	this->init(param, databasePath);
}

void Rtabmap::close(bool databaseSaved, const std::string & ouputDatabasePath)
{
	UINFO("databaseSaved=%d", databaseSaved?1:0);
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
	_distanceTravelled = 0.0f;
	this->clearPath(0);

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
		_memory->close(databaseSaved, true, ouputDatabasePath);
		delete _memory;
		_memory = 0;
	}
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
	Parameters::parse(parameters, Parameters::kRtabmapTimeThr(), _maxTimeAllowed);
	Parameters::parse(parameters, Parameters::kRtabmapMemoryThr(), _maxMemoryAllowed);
	Parameters::parse(parameters, Parameters::kRtabmapLoopThr(), _loopThr);
	Parameters::parse(parameters, Parameters::kRtabmapLoopRatio(), _loopRatio);
	Parameters::parse(parameters, Parameters::kRtabmapMaxRetrieved(), _maxRetrieved);
	Parameters::parse(parameters, Parameters::kRGBDMaxLocalRetrieved(), _maxLocalRetrieved);
	Parameters::parse(parameters, Parameters::kMemImageKept(), _rawDataKept);
	Parameters::parse(parameters, Parameters::kRGBDEnabled(), _rgbdSlamMode);
	Parameters::parse(parameters, Parameters::kRGBDLinearUpdate(), _rgbdLinearUpdate);
	Parameters::parse(parameters, Parameters::kRGBDAngularUpdate(), _rgbdAngularUpdate);
	Parameters::parse(parameters, Parameters::kRGBDNewMapOdomChangeDistance(), _newMapOdomChangeDistance);
	Parameters::parse(parameters, Parameters::kRGBDNeighborLinkRefining(), _neighborLinkRefining);
	Parameters::parse(parameters, Parameters::kRGBDProximityByTime(), _proximityByTime);
	Parameters::parse(parameters, Parameters::kRGBDProximityBySpace(), _proximityBySpace);
	Parameters::parse(parameters, Parameters::kRGBDScanMatchingIdsSavedInLinks(), _scanMatchingIdsSavedInLinks);
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
	Parameters::parse(parameters, Parameters::kRGBDOptimizeFromGraphEnd(), _optimizeFromGraphEnd);
	Parameters::parse(parameters, Parameters::kRGBDOptimizeMaxError(), _optimizationMaxLinearError);
	Parameters::parse(parameters, Parameters::kRtabmapStartNewMapOnLoopClosure(), _startNewMapOnLoopClosure);
	Parameters::parse(parameters, Parameters::kRGBDGoalReachedRadius(), _goalReachedRadius);
	Parameters::parse(parameters, Parameters::kRGBDGoalsSavedInUserData(), _goalsSavedInUserData);
	Parameters::parse(parameters, Parameters::kRGBDPlanStuckIterations(), _pathStuckIterations);
	Parameters::parse(parameters, Parameters::kRGBDPlanLinearVelocity(), _pathLinearVelocity);
	Parameters::parse(parameters, Parameters::kRGBDPlanAngularVelocity(), _pathAngularVelocity);

	UASSERT(_rgbdLinearUpdate >= 0.0f);
	UASSERT(_rgbdAngularUpdate >= 0.0f);

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

	if(_memory)
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
		_epipolarGeometry = new EpipolarGeometry(_parameters);
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

int Rtabmap::triggerNewMap()
{
	int mapId = -1;
	if(_memory)
	{
		std::map<int, int> reducedIds;
		mapId = _memory->incrementMapId(&reducedIds);
		UINFO("New map triggered, new map = %d", mapId);
		_optimizedPoses.clear();
		_constraints.clear();
		_lastLocalizationNodeId = 0;
		_mapCorrection.setIdentity();
		_mapCorrectionBackup.setNull();

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
		else if(_memory->getLastWorkingSignature())
		{
			return _memory->labelSignature(_memory->getLastWorkingSignature()->id(), label);
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
			this->optimizeCurrentMap(_memory->getLastWorkingSignature()->id(), global, poses, &constraints);
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
				_memory->getNodeInfo(iter->first, o, m, w, l, stamp, g, true);
				stamps.insert(std::make_pair(iter->first, stamp));
			}
		}

		graph::exportPoses(path, format, poses, constraints, stamps);
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
	_distanceTravelled = 0.0f;
	this->clearPath(0);

	if(_memory)
	{
		_memory->init(_databasePath, true, _parameters, true);
		if(_memory->getLastWorkingSignature())
		{
			optimizeCurrentMap(_memory->getLastWorkingSignature()->id(), false, _optimizedPoses, &_constraints);
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

//============================================================
// MAIN LOOP
//============================================================
bool Rtabmap::process(
		const SensorData & data,
		Transform odomPose,
		const cv::Mat & covariance)
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
	double timeJoiningTrash = 0;
	double timeStatsCreation = 0;

	float hypothesisRatio = 0.0f; // Only used for statistics
	bool rejectedHypothesis = false;

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
		if(!_memory->isIncremental() && !odomPose.isNull() && !_mapCorrectionBackup.isNull())
		{
			_mapCorrection = _mapCorrectionBackup;
			_mapCorrectionBackup.setNull();
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
				if(_lastLocalizationPose.isNull())
				{
					_lastLocalizationPose = Transform::getIdentity();
				}
				fakeOdom = true;
				odomPose = _mapCorrection.inverse() * _lastLocalizationPose;
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
		if(!_memory->update(data, odomPose, covariance, &statistics_))
		{
			return false;
		}
	}
	else
	{
		if(!_memory->update(data, Transform(), cv::Mat(), &statistics_))
		{
			return false;
		}
	}

	signature = _memory->getLastWorkingSignature();
	if(!signature)
	{
		UFATAL("Not supposed to be here...last signature is null?!?");
	}

	ULOGGER_INFO("Processing signature %d w=%d", signature->id(), signature->getWeight());
	timeMemoryUpdate = timer.ticks();
	ULOGGER_INFO("timeMemoryUpdate=%fs", timeMemoryUpdate);

	//============================================================
	// Metric
	//============================================================
	bool smallDisplacement = false;
	std::list<int> signaturesRemoved;
	if(_rgbdSlamMode)
	{
		statistics_.addStatistic(Statistics::kMemoryOdometry_variance_lin(), covariance.empty()?1.0f:(float)covariance.at<double>(0,0));
		statistics_.addStatistic(Statistics::kMemoryOdometry_variance_ang(), covariance.empty()?1.0f:(float)covariance.at<double>(5,5));

		//Verify if there was a rehearsal
		int rehearsedId = (int)uValue(statistics_.data(), Statistics::kMemoryRehearsal_merged(), 0.0f);
		if(rehearsedId > 0)
		{
			_optimizedPoses.erase(rehearsedId);
		}
		else if(signature->getWeight() >= 0 && _rgbdLinearUpdate > 0.0f && _rgbdAngularUpdate > 0.0f)
		{
			//============================================================
			// Minimum displacement required to add to Memory
			//============================================================
			const std::map<int, Link> & links = signature->getLinks();
			if(links.size() == 1)
			{
				// don't do this if there are intermediate nodes
				const Signature * s = _memory->getSignature(links.begin()->second.to());
				UASSERT(s!=0);
				if(s->getWeight() >= 0)
				{
					float x,y,z, roll,pitch,yaw;
					links.begin()->second.transform().getTranslationAndEulerAngles(x,y,z, roll,pitch,yaw);
					bool isMoving = fabs(x) > _rgbdLinearUpdate ||
									fabs(y) > _rgbdLinearUpdate ||
									fabs(z) > _rgbdLinearUpdate ||
									fabs(roll) > _rgbdAngularUpdate ||
									fabs(pitch) > _rgbdAngularUpdate ||
									fabs(yaw) > _rgbdAngularUpdate;
					if(!isMoving)
					{
						// This will disable global loop closure detection, only retrieval will be done.
						// The location will also be deleted at the end.
						smallDisplacement = true;
					}
				}
			}
		}

		// Update optimizedPoses with the newly added node
		Transform newPose;
		if(_neighborLinkRefining &&
			signature->getLinks().size() == 1 &&
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
						_memory->updateLink(Link(oldId, signature->id(), signature->getLinks().begin()->second.type(), guess, 0.0001, 0.0001));
					}
				}
				else
				{
					//============================================================
					// Refine neighbor links
					//============================================================
					if(!signature->sensorData().laserScanCompressed().empty())
					{
						UINFO("Odometry refining: guess = %s", guess.prettyPrint().c_str());
						RegistrationInfo info;
						Transform t = _memory->computeIcpTransform(oldId, signature->id(), guess, &info);
						if(!t.isNull())
						{
							UINFO("Odometry refining: update neighbor link (%d->%d, variance:lin=%f, ang=%f) from %s to %s",
									oldId,
									signature->id(),
									info.varianceLin,
									info.varianceAng,
									guess.prettyPrint().c_str(),
									t.prettyPrint().c_str());
							UASSERT(info.varianceLin > 0.0 && info.varianceAng > 0.0);
							_memory->updateLink(Link(oldId, signature->id(), signature->getLinks().begin()->second.type(), t, info.varianceAng, info.varianceLin));

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
							if(info.varianceLin > 0 && info.varianceAng > 0)
							{
								_memory->updateLink(Link(oldId, signature->id(), signature->getLinks().begin()->second.type(), guess, sqrt(info.varianceAng), sqrt(info.varianceLin)));
							}
						}
						statistics_.addStatistic(Statistics::kNeighborLinkRefiningAccepted(), !t.isNull()?1.0f:0);
						statistics_.addStatistic(Statistics::kNeighborLinkRefiningInliers(), info.inliers);
						statistics_.addStatistic(Statistics::kNeighborLinkRefiningInliers_ratio(), info.icpInliersRatio);
						statistics_.addStatistic(Statistics::kNeighborLinkRefiningPts(), signature->sensorData().laserScanRaw().cols);
					}
				}
				timeNeighborLinkRefining = timer.ticks();
				ULOGGER_INFO("timeOdometryRefining=%fs", timeNeighborLinkRefining);

				UASSERT(oldS->hasLink(signature->id()));
				UASSERT(uContains(_optimizedPoses, oldId));

				statistics_.addStatistic(Statistics::kNeighborLinkRefiningVariance(), oldS->getLinks().at(signature->id()).transVariance());

				newPose = _optimizedPoses.at(oldId) * oldS->getLinks().at(signature->id()).transform();
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
				UWARN("Neighbor link refining is activated but there are intermediate nodes, aborting refining...");
			}
		}
		else
		{
			newPose = _mapCorrection * signature->getPose();
		}

		UDEBUG("Added pose %s (odom=%s)", newPose.prettyPrint().c_str(), signature->getPose().prettyPrint().c_str());
		// Update Poses and Constraints
		_optimizedPoses.insert(std::make_pair(signature->id(), newPose));
		_lastLocalizationPose = newPose; // keep in cache the latest corrected pose
		if(signature->getLinks().size() == 1 &&
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
		if(_proximityByTime &&
		   rehearsedId == 0 && // don't do it if rehearsal happened
		   signature->getWords3().size() &&
		   _memory->isIncremental() && // don't do it in localization mode
		   !signature->isBadSignature() &&
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
						guess = newPose.inverse() * _optimizedPoses.at(*iter);
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
						UASSERT(info.varianceLin > 0.0 && info.varianceAng > 0.0);
						if(_memory->addLink(Link(signature->id(), *iter, Link::kLocalTimeClosure, transform, info.varianceAng, info.varianceLin)))
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
				}
			}
		}
	}

	timeProximityByTimeDetection = timer.ticks();
	UINFO("timeProximityByTimeDetection=%fs", timeProximityByTimeDetection);

	//============================================================
	// Bayes filter update
	//============================================================
	int previousId = signature->getLinks().size() == 1?signature->getLinks().begin()->first:0;
	// Not a bad signature, not an intermediate node, not a small displacement unless the previous signature didn't have a loop closure
	if(!signature->isBadSignature() && signature->getWeight()>=0 && (!smallDisplacement || _memory->getLoopClosureLinks(previousId, false).size() == 0))
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
					signature->getLinks().size() == 0 &&     // alone in the current map
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
					rejectedHypothesis = true;
					if(posterior.size() <= 2 && loopThr>0.0f)
					{
						// Ignore loop closure if there is only one loop closure hypothesis
						UDEBUG("rejected hypothesis: single hypothesis");
					}
					else if(_epipolarGeometry && !_epipolarGeometry->check(signature, _memory->getSignature(_highestHypothesis.first)))
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
						rejectedHypothesis = false;
					}

					timeHypothesesValidation = timer.ticks();
					ULOGGER_INFO("timeHypothesesValidation=%fs",timeHypothesesValidation);
				}
				else if(_highestHypothesis.second < _loopRatio*lastHighestHypothesis.second)
				{
					// Used for Precision-Recall computation.
					// When analyzing logs, it's convenient to know
					// if the hypothesis would be rejected if T_loop would be lower.
					rejectedHypothesis = true;
					UDEBUG("rejected hypothesis: under loop ratio %f < %f", _highestHypothesis.second, _loopRatio*lastHighestHypothesis.second);
				}

				//for statistic...
				hypothesisRatio = _loopClosureHypothesis.second>0?_highestHypothesis.second/_loopClosureHypothesis.second:0;
			}
		} // if(_memory->getWorkingMemSize())
	}// !isBadSignature
	else if(!signature->isBadSignature() && smallDisplacement)
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
	if(retrievalId > 0 )
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

						UDEBUG("nt=%d m=%d immunized=1", iter->first, iter->second);
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
						UDEBUG("nt=%d m=%d", iter->first, iter->second);
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

	//============================================================
	// RETRIEVAL 2/3 : Update planned path and get next nodes to retrieve
	//============================================================
	std::list<int> retrievalLocalIds;
	int maxLocalLocationsImmunized = _localImmunizationRatio * float(_memory->getWorkingMem().size());
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

		// immunize the path from the nearest local location to the current location
		if(immunizedLocally < maxLocalLocationsImmunized &&
			_memory->isIncremental()) // Can only work in mapping mode
		{
			std::map<int ,Transform> poses;
			// remove poses from STM
			for(std::map<int, Transform>::iterator iter=_optimizedPoses.begin(); iter!=_optimizedPoses.end(); ++iter)
			{
				if(!_memory->isInSTM(iter->first))
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
							UDEBUG("local node %d on path immunized=1", iter->first);
						}
					}
				}
			}
		}

		// retrieval based on the nodes close the the nearest pose in WM
		// immunize closest nodes
		std::map<int, float> nearNodes = graph::getNodesInRadius(signature->id(), _optimizedPoses, _localRadius);
		// sort by distance
		std::multimap<float, int> nearNodesByDist;
		for(std::map<int, float>::iterator iter=nearNodes.begin(); iter!=nearNodes.end(); ++iter)
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
				const std::map<int, Link> & links = s->getLinks();
				for(std::map<int, Link>::const_reverse_iterator jter=links.rbegin();
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
					UDEBUG("local node %d (%f m) immunized=1", iter->second, iter->first);
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
	}
	timeReactivations = timer.ticks();
	ULOGGER_INFO("timeReactivations=%fs", timeReactivations);

	//=============================================================
	// Update loop closure links
	// (updated: place this after retrieval to be sure that neighbors of the loop closure are in RAM)
	//=============================================================
	std::list<std::pair<int, int> > loopClosureLinksAdded;
	int loopClosureVisualInliers = 0; // for statistics
	int loopClosureVisualMatches = 0;
	if(_loopClosureHypothesis.first>0)
	{
		//Compute transform if metric data are present
		Transform transform;
		RegistrationInfo info;
		info.varianceLin = info.varianceAng = 1.0f;
		if(_rgbdSlamMode)
		{
			transform = _memory->computeTransform(_loopClosureHypothesis.first, signature->id(), Transform(), &info);
			loopClosureVisualInliers = info.inliers;
			loopClosureVisualMatches = info.matches;
			rejectedHypothesis = transform.isNull();
			if(rejectedHypothesis)
			{
				UWARN("Rejected loop closure %d -> %d: %s",
						_loopClosureHypothesis.first, signature->id(), info.rejectedMsg.c_str());
			}
			else
			{
				transform = transform.inverse();
			}
		}
		if(!rejectedHypothesis)
		{
			// Make the new one the parent of the old one
			UASSERT(info.varianceLin > 0.0 && info.varianceAng > 0.0);
			rejectedHypothesis = !_memory->addLink(Link(signature->id(), _loopClosureHypothesis.first, Link::kGlobalClosure, transform, info.varianceAng, info.varianceLin));
			if(!rejectedHypothesis)
			{
				loopClosureLinksAdded.push_back(std::make_pair(signature->id(), _loopClosureHypothesis.first));
			}
		}

		if(rejectedHypothesis)
		{
			_loopClosureHypothesis.first = 0;
		}
	}

	timeAddLoopClosureLink = timer.ticks();
	ULOGGER_INFO("timeAddLoopClosureLink=%fs", timeAddLoopClosureLink);

	int proximityDetectionsAddedVisually = 0;
	int proximityDetectionsAddedByICPOnly = 0;
	int lastProximitySpaceClosureId = 0;
	int proximitySpacePaths = 0;
	int localVisualPathsChecked = 0;
	int localScanPathsChecked = 0;
	if(_proximityBySpace &&
	   _localRadius > 0 &&
	   _rgbdSlamMode &&
	   signature->getWeight() >= 0) // not an intermediate node
	{
		if(_graphOptimizer->iterations() == 0)
		{
			UWARN("Cannot do local loop closure detection in space if graph optimization is disabled!");
		}
		else if(_memory->isIncremental() || _loopClosureHypothesis.first == 0)
		{
			// In localization mode, no need to check local loop
			// closures if we are already localized by a global closure.

			// don't do it if it is a small displacement unless the previous signature didn't have a loop closure
			if(!smallDisplacement || _memory->getLoopClosureLinks(previousId, false).size() == 0)
			{

				//============================================================
				// LOCAL LOOP CLOSURE SPACE
				//============================================================

				//
				// 1) compare visually with nearest locations
				//
				UDEBUG("Proximity detection (local loop closure in SPACE using matching images)");
				std::map<int, float> nearestIds;
				if(_memory->isIncremental())
				{
					nearestIds = _memory->getNeighborsIdRadius(signature->id(), _localRadius, _optimizedPoses, _proximityMaxGraphDepth);
				}
				else
				{
					nearestIds = graph::getNodesInRadius(signature->id(), _optimizedPoses, _localRadius);
				}
				UDEBUG("nearestIds=%d/%d", (int)nearestIds.size(), (int)_optimizedPoses.size());
				std::map<int, Transform> nearestPoses;
				for(std::map<int, float>::iterator iter=nearestIds.begin(); iter!=nearestIds.end(); ++iter)
				{
					if(_memory->getStMem().find(iter->first) == _memory->getStMem().end())
					{
						nearestPoses.insert(std::make_pair(iter->first, _optimizedPoses.at(iter->first)));
					}
				}
				UDEBUG("nearestPoses=%d", (int)nearestPoses.size());

				// segment poses by paths, only one detection per path
				std::map<int, std::map<int, Transform> > nearestPaths = getPaths(nearestPoses, _optimizedPoses.at(signature->id()), _proximityMaxGraphDepth);
				UDEBUG("nearestPaths=%d proximityMaxPaths=%d", (int)nearestPaths.size(), _proximityMaxPaths);

				for(std::map<int, std::map<int, Transform> >::const_reverse_iterator iter=nearestPaths.rbegin();
					iter!=nearestPaths.rend() &&
					(_memory->isIncremental() || lastProximitySpaceClosureId == 0) &&
					(_proximityMaxPaths <= 0 || localVisualPathsChecked < _proximityMaxPaths);
					++iter)
				{
					std::map<int, Transform> path = iter->second;
					UASSERT(path.size());

					//find the nearest pose on the path looking in the same direction
					path.insert(std::make_pair(signature->id(), _optimizedPoses.at(signature->id())));
					path = graph::getPosesInRadius(signature->id(), path, _localRadius, _proximityAngle);
					int nearestId = rtabmap::graph::findNearestNode(path, _optimizedPoses.at(signature->id()));
					if(nearestId > 0)
					{
						// nearest pose must not be linked to current location and enough close
						if(!signature->hasLink(nearestId) &&
							(_proximityFilteringRadius <= 0.0f ||
							 _optimizedPoses.at(signature->id()).getDistanceSquared(_optimizedPoses.at(nearestId)) < _proximityFilteringRadius*_proximityFilteringRadius))
						{
							++localVisualPathsChecked;
							RegistrationInfo info;
							// guess is null to make sure visual correspondences are globally computed
							Transform transform = _memory->computeTransform(nearestId, signature->id(), Transform(), &info);
							if(!transform.isNull())
							{
								transform = transform.inverse();
								if(_proximityFilteringRadius <= 0 || transform.getNormSquared() <= _proximityFilteringRadius*_proximityFilteringRadius)
								{
									UINFO("[Visual] Add local loop closure in SPACE (%d->%d) %s",
											signature->id(),
											nearestId,
											transform.prettyPrint().c_str());
									UASSERT(info.varianceLin > 0.0 && info.varianceAng > 0.0);
									_memory->addLink(Link(signature->id(), nearestId, Link::kLocalSpaceClosure, transform, info.varianceAng, info.varianceLin));
									loopClosureLinksAdded.push_back(std::make_pair(signature->id(), nearestId));

									if(loopClosureVisualInliers == 0)
									{
										loopClosureVisualInliers = info.inliers;
									}
									if(loopClosureVisualMatches == 0)
									{
										loopClosureVisualMatches = info.matches;
									}

									if(_loopClosureHypothesis.first == 0)
									{
										++proximityDetectionsAddedVisually;
										lastProximitySpaceClosureId = nearestId;
									}
								}
								else
								{
									UWARN("Ignoring local loop closure with %d because resulting "
										  "transform is to large!? (%fm > %fm)",
											nearestId, transform.getNorm(), _proximityFilteringRadius);
								}
							}
						}
					}
				}

				//
				// 2) compare locally with nearest locations by scan matching
				//
				UDEBUG("Proximity detection (local loop closure in SPACE with scan matching)");
				if( _proximityMaxNeighbors > 0 &&
					!signature->sensorData().laserScanCompressed().empty() &&
					(_memory->isIncremental() || lastProximitySpaceClosureId == 0))
				{
					// In localization mode, no need to check local loop
					// closures if we are already localized by at least one
					// local visual closure above.

					// Parse again with if different (normally, maxNeighbors would be smaller than MaxGraphDepth)
					if(_proximityMaxNeighbors != _proximityMaxGraphDepth)
					{
						nearestPaths = getPaths(nearestPoses, _optimizedPoses.at(signature->id()), _proximityMaxNeighbors);
						UDEBUG("nearestPaths=%d proximityMaxPaths=%d", (int)nearestPaths.size(), _proximityMaxPaths);
					}

					proximitySpacePaths = (int)nearestPaths.size();
					for(std::map<int, std::map<int, Transform> >::const_reverse_iterator iter=nearestPaths.rbegin();
							iter!=nearestPaths.rend() &&
							(_memory->isIncremental() || lastProximitySpaceClosureId == 0) &&
							(_proximityMaxPaths <= 0 || localScanPathsChecked < _proximityMaxPaths);
							++iter)
					{
						std::map<int, Transform> path = iter->second;
						UASSERT(path.size());

						//find the nearest pose on the path
						int nearestId = rtabmap::graph::findNearestNode(path, _optimizedPoses.at(signature->id()));
						UASSERT(nearestId > 0);
						UDEBUG("Path %d (size=%d) distance=%fm", nearestId, (int)path.size(), _optimizedPoses.at(signature->id()).getDistance(_optimizedPoses.at(nearestId)));

						// nearest pose must be close and not linked to current location
						if(!signature->hasLink(nearestId) &&
						   (_proximityFilteringRadius <= 0.0f ||
							_optimizedPoses.at(signature->id()).getDistanceSquared(_optimizedPoses.at(nearestId)) < _proximityFilteringRadius*_proximityFilteringRadius))
						{
							// Assemble scans in the path and do ICP only
							if(_proximityRawPosesUsed)
							{
								//optimize the path's poses locally
								path = optimizeGraph(nearestId, uKeysSet(path), std::map<int, Transform>(), false);
								// transform local poses in optimized graph referential
								UASSERT(uContains(path, nearestId));
								Transform t = _optimizedPoses.at(nearestId) * path.at(nearestId).inverse();
								for(std::map<int, Transform>::iterator jter=path.begin(); jter!=path.end(); ++jter)
								{
									jter->second = t * jter->second;
								}
							}
							std::map<int, Transform> filteredPath = path;
							if(path.size() > 2 && _proximityFilteringRadius > 0.0f)
							{
								// path filtering
								filteredPath = graph::radiusPosesFiltering(path, _proximityFilteringRadius, 0, true);
								// make sure the current pose is still here
								filteredPath.insert(*path.find(nearestId));
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
									Transform transform = _memory->computeIcpTransformMulti(signature->id(), nearestId, filteredPath, &info);
									if(!transform.isNull())
									{
										if(_proximityFilteringRadius <= 0 || transform.getNormSquared() <= _proximityFilteringRadius*_proximityFilteringRadius)
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
												for(std::map<int, Transform>::iterator iter=path.begin(); iter!=path.end(); ++iter)
												{
													if(iter != path.begin())
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
											UASSERT(info.varianceLin>0.0 && info.varianceAng>0.0);
											_memory->addLink(Link(signature->id(), nearestId, Link::kLocalSpaceClosure, transform, sqrt(info.varianceAng), sqrt(info.varianceLin), scanMatchingIds));
											loopClosureLinksAdded.push_back(std::make_pair(signature->id(), nearestId));

											++proximityDetectionsAddedByICPOnly;

											// no local loop closure added visually
											if(proximityDetectionsAddedVisually == 0 && _loopClosureHypothesis.first == 0)
											{
												lastProximitySpaceClosureId = nearestId;
											}
										}
										else
										{
											UWARN("Ignoring local loop closure with %d because resulting "
												  "transform is to large!? (%fm > %fm)",
													nearestId, transform.getNorm(), _proximityFilteringRadius);
										}
									}
									else
									{
										UWARN("Local scan matching rejected: %s", info.rejectedMsg.c_str());
									}
								}
							}
						}
						else
						{
							UDEBUG("Path %d ignored", nearestId);
						}
					}
				}
			}
		}
	}
	timeProximityBySpaceDetection = timer.ticks();
	ULOGGER_INFO("timeProximityBySpaceDetection=%fs", timeProximityBySpaceDetection);

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
				_memory->addLink(Link(signature->id(), _path[_pathCurrentIndex].first, Link::kVirtualClosure, virtualLoop, 100, 100)); // set high variance
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
	double optimizationError = 0.0;
	int optimizationIterations = 0;
	if(_rgbdSlamMode &&
		(_loopClosureHypothesis.first>0 ||
	     lastProximitySpaceClosureId>0 || // can be different map of the current one
	     statistics_.reducedIds().size() ||
	     proximityDetectionsInTimeFound>0 ||
		 ((_memory->isIncremental() || signature->getLinks().size()) && // In localization mode, the new node should be linked
		          signaturesRetrieved.size())))  		// can be different map of the current one
	{
		UASSERT(uContains(_optimizedPoses, signature->id()));

		//used in localization mode: filter virtual links
		std::map<int, Link> localizationLinks = graph::filterLinks(signature->getLinks(), Link::kVirtualClosure);

		// Note that in localization mode, we don't re-optimize the graph
		// if:
		//  1- there are no signatures retrieved,
		//  2- we are relocalizing on a node already in the optimized graph
		if(!_memory->isIncremental() &&
		   signaturesRetrieved.size() == 0 &&
		   localizationLinks.size() &&
		   uContains(_optimizedPoses, localizationLinks.begin()->first))
		{
			// If there are no signatures retrieved, we don't
			// need to re-optimize the graph. Just update the last
			// position if OptimizeFromGraphEnd=false or transform the
			// whole graph if OptimizeFromGraphEnd=true
			UINFO("Localization without map optimization");
			if(_optimizeFromGraphEnd)
			{
				// update all previous nodes
				// Normally _mapCorrection should be identity, but if _optimizeFromGraphEnd
				// parameters just changed state, we should put back all poses without map correction.
				Transform oldPose = _optimizedPoses.at(localizationLinks.begin()->first);
				Transform mapCorrectionInv = _mapCorrection.inverse();
				Transform u = signature->getPose() * localizationLinks.begin()->second.transform();
				Transform up = u * oldPose.inverse();
				for(std::map<int, Transform>::iterator iter=_optimizedPoses.begin(); iter!=_optimizedPoses.end(); ++iter)
				{
					iter->second = mapCorrectionInv * up * iter->second;
				}
				_optimizedPoses.at(signature->id()) = signature->getPose();
			}
			else
			{
				_optimizedPoses.at(signature->id()) = _optimizedPoses.at(localizationLinks.begin()->first) * localizationLinks.begin()->second.transform().inverse();
			}
		}
		else
		{

			UINFO("Update map correction");
			std::map<int, Transform> poses = _optimizedPoses;
			
			// if _optimizeFromGraphEnd parameter just changed state, don't use optimized poses as guess
			float normMapCorrection = _mapCorrection.getNormSquared(); // use distance for identity detection
			if((normMapCorrection > 0.001f && _optimizeFromGraphEnd) ||
				(normMapCorrection < 0.001f && !_optimizeFromGraphEnd))
			{
				poses.clear();
			}

			std::multimap<int, Link> constraints;
			optimizeCurrentMap(signature->id(), false, poses, &constraints, &optimizationError, &optimizationIterations);

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
                                rejectedHypothesis = true;
			}
			else if(_memory->isIncremental() && // FIXME: not tested in localization mode, so do it only in mapping mode
			  _optimizationMaxLinearError > 0.0f &&
			  loopClosureLinksAdded.size() &&
			  optimizationIterations > 0)
			{
				const Link * maxLinearLink = 0;
				for(std::multimap<int, Link>::iterator iter=constraints.begin(); iter!=constraints.end(); ++iter)
				{
					// ignore links with high variance
					if(iter->second.transVariance() <= 1.0)
					{
						Transform t1 = uValue(poses, iter->second.from(), Transform());
						Transform t2 = uValue(poses, iter->second.to(), Transform());
						Transform t = t1.inverse()*t2;
						float linearError = uMax3(
								fabs(iter->second.transform().x() - t.x()),
								fabs(iter->second.transform().y() - t.y()),
								fabs(iter->second.transform().z() - t.z()));
						if(linearError > maxLinearError)
						{
							maxLinearError = linearError;
							maxLinearLink = &iter->second;
						}
					}
				}
				if(maxLinearLink)
				{
					UINFO("Max optimization error = %f m (link %d->%d)", maxLinearError, maxLinearLink->from(), maxLinearLink->to());
				}

				if(maxLinearError > _optimizationMaxLinearError)
				{
					UWARN("Rejecting all added loop closures (%d) in this "
						  "iteration because a wrong loop closure has been "
						  "detected after graph optimization, resulting in "
						  "a maximum graph error of %f m (edge %d->%d, type=%d). The "
						  "maximum error parameter is %f m.",
						  (int)loopClosureLinksAdded.size(),
						  maxLinearError,
						  maxLinearLink->from(),
						  maxLinearLink->to(),
						  maxLinearLink->type(),
						  _optimizationMaxLinearError);
					for(std::list<std::pair<int, int> >::iterator iter=loopClosureLinksAdded.begin(); iter!=loopClosureLinksAdded.end(); ++iter)
					{
						_memory->removeLink(iter->first, iter->second);
						UWARN("Loop closure %d->%d rejected!", iter->first, iter->second);
					}
					updateConstraints = false;
					_loopClosureHypothesis.first = 0;
					lastProximitySpaceClosureId = 0;
					rejectedHypothesis = true;
				}
			}

			if(updateConstraints)
			{
				UINFO("Updated local map (old size=%d, new size=%d)", (int)_optimizedPoses.size(), (int)poses.size());
				_optimizedPoses = poses;
				_constraints = constraints;
			}
		}

		// Update map correction, it should be identify when optimizing from the last node
		UASSERT(_optimizedPoses.find(signature->id()) != _optimizedPoses.end());
		if(fakeOdom && _mapCorrectionBackup.isNull())
		{
			_mapCorrectionBackup = _mapCorrection;
		}
		_mapCorrection = _optimizedPoses.at(signature->id()) * signature->getPose().inverse();
		_lastLocalizationPose = _optimizedPoses.at(signature->id()); // update
		if(_mapCorrection.getNormSquared() > 0.001f && _optimizeFromGraphEnd)
		{
			UERROR("Map correction should be identity when optimizing from the last node. T=%s", _mapCorrection.prettyPrint().c_str());
		}
	}
	_lastLocalizationNodeId = _loopClosureHypothesis.first>0?_loopClosureHypothesis.first:lastProximitySpaceClosureId>0?lastProximitySpaceClosureId:_lastLocalizationNodeId;

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
		statistics_.setStamp(data.stamp());
		if(_loopClosureHypothesis.first != Memory::kIdInvalid)
		{
			statistics_.setLoopClosureId(_loopClosureHypothesis.first);
			ULOGGER_INFO("Loop closure detected! With id=%d", _loopClosureHypothesis.first);
		}
		if(_publishStats)
		{
			ULOGGER_INFO("send all stats...");
			statistics_.setExtended(1);

			statistics_.addStatistic(Statistics::kLoopAccepted_hypothesis_id(), _loopClosureHypothesis.first);
			statistics_.addStatistic(Statistics::kLoopHighest_hypothesis_id(), _highestHypothesis.first);
			statistics_.addStatistic(Statistics::kLoopHighest_hypothesis_value(), _highestHypothesis.second);
			statistics_.addStatistic(Statistics::kLoopHypothesis_reactivated(), lcHypothesisReactivated);
			statistics_.addStatistic(Statistics::kLoopVp_hypothesis(), vpHypothesis);
			statistics_.addStatistic(Statistics::kLoopReactivate_id(), retrievalId);
			statistics_.addStatistic(Statistics::kLoopHypothesis_ratio(), hypothesisRatio);
			statistics_.addStatistic(Statistics::kLoopVisual_inliers(), loopClosureVisualInliers);
			statistics_.addStatistic(Statistics::kLoopVisual_matches(), loopClosureVisualMatches);
			statistics_.addStatistic(Statistics::kLoopLast_id(), _memory->getLastGlobalLoopClosureId());
			statistics_.addStatistic(Statistics::kLoopOptimization_max_error(), maxLinearError);
			statistics_.addStatistic(Statistics::kLoopOptimization_error(), optimizationError);
			statistics_.addStatistic(Statistics::kLoopOptimization_iterations(), optimizationIterations);

			statistics_.addStatistic(Statistics::kProximityTime_detections(), proximityDetectionsInTimeFound);
			statistics_.addStatistic(Statistics::kProximitySpace_detections_added_visually(), proximityDetectionsAddedVisually);
			statistics_.addStatistic(Statistics::kProximitySpace_detections_added_icp_only(), proximityDetectionsAddedByICPOnly);
			statistics_.addStatistic(Statistics::kProximitySpace_paths(), proximitySpacePaths);
			statistics_.addStatistic(Statistics::kProximitySpace_visual_paths_checked(), localVisualPathsChecked);
			statistics_.addStatistic(Statistics::kProximitySpace_scan_paths_checked(), localScanPathsChecked);
			statistics_.addStatistic(Statistics::kProximitySpace_last_detection_id(), lastProximitySpaceClosureId);
			statistics_.setProximityDetectionId(lastProximitySpaceClosureId);
			if(_loopClosureHypothesis.first || lastProximitySpaceClosureId)
			{
				UASSERT(uContains(sLoop->getLinks(), signature->id()));
				UINFO("Set loop closure transform = %s", sLoop->getLinks().at(signature->id()).transform().prettyPrint().c_str());
				statistics_.setLoopClosureTransform(sLoop->getLinks().at(signature->id()).transform());
			}
			statistics_.setMapCorrection(_mapCorrection);
			UINFO("Set map correction = %s", _mapCorrection.prettyPrint().c_str());

			// timings...
			statistics_.addStatistic(Statistics::kTimingMemory_update(), timeMemoryUpdate*1000);
			statistics_.addStatistic(Statistics::kTimingNeighbor_link_refining(), timeNeighborLinkRefining*1000);
			statistics_.addStatistic(Statistics::kTimingProximity_by_time(), timeProximityByTimeDetection*1000);
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

			// Surf specific parameters
			statistics_.addStatistic(Statistics::kKeypointDictionary_size(), dictionarySize);
			statistics_.addStatistic(Statistics::kKeypointIndexed_words(), _memory->getVWDictionary()->getIndexedWordsCount());
			statistics_.addStatistic(Statistics::kKeypointIndex_memory_usage(), _memory->getVWDictionary()->getIndexMemoryUsed());

			//Epipolar geometry constraint
			statistics_.addStatistic(Statistics::kLoopRejectedHypothesis(), rejectedHypothesis?1.0f:0);

			statistics_.addStatistic(Statistics::kMemorySmall_movement(), smallDisplacement?1.0f:0);
			statistics_.addStatistic(Statistics::kMemoryDistance_travelled(), _distanceTravelled);

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
	if(_optimizedPoses.find(signature->id()) != _optimizedPoses.end() && signature->getLinks().size())
	{
		// only if localized set it
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
			signature->getLinks().size() == 0 &&     // alone in the current map
			_memory->getWorkingMem().size()>=2)       // The working memory should not be empty (beside virtual signature)
		{
			UWARN("Ignoring location %d because a global loop closure is required before starting a new map!",
					signature->id());
			signaturesRemoved.push_back(signature->id());
			_memory->deleteLocation(signature->id());
		}
		else if(smallDisplacement && _loopClosureHypothesis.first == 0 && lastProximitySpaceClosureId == 0)
		{
			// Don't delete the location if a loop closure is detected
			UINFO("Ignoring location %d because the displacement is too small! (d=%f a=%f)",
				  signature->id(), _rgbdLinearUpdate, _rgbdAngularUpdate);
			// If there is a too small displacement, remove the node
			signaturesRemoved.push_back(signature->id());
			_memory->deleteLocation(signature->id());
		}
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
				UASSERT_MSG(uContains(_optimizedPoses, _lastLocalizationNodeId), uFormat("id=%d isInWM?=%d", _lastLocalizationNodeId, _memory->isInWM(_lastLocalizationNodeId)?1:0).c_str());
				id = _lastLocalizationNodeId;
				UDEBUG("Refresh local map from %d", id);
			}
		}
		else if(_memory->isIncremental() &&
				_optimizedPoses.size() &&
				_memory->getLastWorkingSignature())
		{
			id = _memory->getLastWorkingSignature()->id();
			UDEBUG("Refresh local map from %d", id);
		}
		if(id > 0)
		{
			if(_lastLocalizationNodeId != 0)
			{
				_lastLocalizationNodeId = id;
			}
			UASSERT_MSG(_memory->getSignature(id) != 0, uFormat("id=%d", id).c_str());
			std::map<int, int> ids = _memory->getNeighborsId(id, 0, 0, true);
			for(std::map<int, Transform>::iterator iter=_optimizedPoses.begin(); iter!=_optimizedPoses.end();)
			{
				if(!uContains(ids, iter->first))
				{
					UDEBUG("Removed %d from local map", iter->first);
					UASSERT(iter->first != _lastLocalizationNodeId);
					_optimizedPoses.erase(iter++);
				}
				else
				{
					++iter;
				}
			}
			for(std::multimap<int, Link>::iterator iter=_constraints.begin(); iter!=_constraints.end();)
			{
				if(!uContains(ids, iter->second.from()) || !uContains(ids, iter->second.to()))
				{
					_constraints.erase(iter++);
				}
				else
				{
					++iter;
				}
			}
		}
		else
		{
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

		std::map<int, Signature> signatures;
		if(_publishLastSignatureData)
		{
			UINFO("Adding data %d (rgb/left=%d depth/right=%d)", lastSignatureData.id(), lastSignatureData.sensorData().imageRaw().empty()?0:1, lastSignatureData.sensorData().depthOrRightRaw().empty()?0:1);
			signatures.insert(std::make_pair(lastSignatureData.id(), lastSignatureData));
		}
		UDEBUG("");
		// Set local graph
		std::map<int, Transform> poses;
		std::multimap<int, Link> constraints;
		if(!_rgbdSlamMode)
		{
			// no optimization on appearance-only mode, create a local graph
			std::map<int, int> ids = _memory->getNeighborsId(lastSignatureData.id(), 0, 0, true);
			_memory->getMetricConstraints(uKeysSet(ids), poses, constraints, false);
		}
		else // RGBD-SLAM mode
		{
			poses = _optimizedPoses;
			constraints = _constraints;
		}
		UDEBUG("Get all node infos...");
		for(std::map<int, Transform>::iterator iter=poses.begin(); iter!=poses.end(); ++iter)
		{
			Transform odomPoseLocal;
			int weight = -1;
			int mapId = -1;
			std::string label;
			double stamp = 0;
			Transform groundTruth;
			std::vector<unsigned char> userData;
			_memory->getNodeInfo(iter->first, odomPoseLocal, mapId, weight, label, stamp, groundTruth, false);
			signatures.insert(std::make_pair(iter->first,
					Signature(iter->first,
							mapId,
							weight,
							stamp,
							label,
							odomPoseLocal,
							groundTruth)));
		}
		localGraphSize = (int)poses.size();
		if(!lastSignatureLocalizedPose.isNull())
		{
			poses.insert(std::make_pair(lastSignatureData.id(), lastSignatureLocalizedPose)); // in case we are in localization
		}
		statistics_.setPoses(poses);
		statistics_.setConstraints(constraints);
		statistics_.setSignatures(signatures);
		statistics_.addStatistic(Statistics::kMemoryLocal_graph_size(), poses.size());
		UDEBUG("");
	}

	//Save statistics to database
	if(_memory->isIncremental())
	{
		_memory->saveStatistics(statistics_);
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
									rejectedHypothesis?1:0,
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
	UDEBUG("End process");

	return true;
}

bool Rtabmap::process(const cv::Mat & image, int id)
{
	return this->process(SensorData(image, id), Transform());
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

void Rtabmap::rejectLoopClosure(int oldId, int newId)
{
	UDEBUG("_loopClosureHypothesis.first=%d", _loopClosureHypothesis.first);
	if(_loopClosureHypothesis.first)
	{
		_loopClosureHypothesis.first = 0;
		if(_memory)
		{
			_memory->removeLink(oldId, newId);
		}
		if(uContains(statistics_.data(), rtabmap::Statistics::kLoopRejectedHypothesis()))
		{
			statistics_.addStatistic(rtabmap::Statistics::kLoopRejectedHypothesis(), 1.0f);
		}
		statistics_.setLoopClosureId(0);
	}
}

void Rtabmap::setOptimizedPoses(const std::map<int, Transform> & poses)
{
	_optimizedPoses = poses;
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
			foundIds = graph::getNodesInRadius(fromId, _optimizedPoses, radius);
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

std::map<int, std::map<int, Transform> > Rtabmap::getPaths(std::map<int, Transform> poses, const Transform & target, int maxGraphDepth) const
{
	std::map<int, std::map<int, Transform> > paths;
	if(_memory && poses.size() && !target.isNull())
	{
		// Segment poses connected only by neighbor links
		while(poses.size())
		{
			std::map<int, Transform> path;
			// select nearest pose and iterate neighbors from there
			int nearestId = rtabmap::graph::findNearestNode(poses, target);
			std::map<int, int> ids = _memory->getNeighborsId(nearestId, maxGraphDepth, 0, true, true, true);

			for(std::map<int, int>::iterator iter=ids.begin(); iter!=ids.end(); ++iter)
			{
				std::map<int, Transform>::iterator jter = poses.find(iter->first);
				if(jter != poses.end())
				{
					bool valid = path.empty();
					if(!valid)
					{
						// make sure it has a neighbor added to path
						std::map<int, Link> links = _memory->getNeighborLinks(iter->first);
						for(std::map<int, Link>::iterator kter=links.begin(); kter!=links.end() && !valid; ++kter)
						{
							valid = path.find(kter->first) != path.end();
						}
					}

					if(valid)
					{
						UDEBUG("%d <- %d", nearestId, jter->first);
						path.insert(*jter);
						poses.erase(jter);
					}
				}
			}
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
				UWARN(uFormat("path.size()=0!? nearestId=%d ids=%d", (int)path.size(), nearestId, (int)ids.size()).c_str());
			}
		}

	}
	return paths;
}

void Rtabmap::optimizeCurrentMap(
		int id,
		bool lookInDatabase,
		std::map<int, Transform> & optimizedPoses,
		std::multimap<int, Link> * constraints,
		double * error,
		int * iterationsDone) const
{
	//Optimize the map
	UINFO("Optimize map: around location %d", id);
	if(_memory && id > 0)
	{
		UTimer timer;
		std::map<int, int> ids = _memory->getNeighborsId(id, 0, lookInDatabase?-1:0, true, false);
		if(!_optimizeFromGraphEnd && ids.size() > 1)
		{
			id = ids.begin()->first;
		}
		UINFO("get %d ids time %f s", (int)ids.size(), timer.ticks());

		std::map<int, Transform> poses = Rtabmap::optimizeGraph(id, uKeysSet(ids), optimizedPoses, lookInDatabase, constraints, error, iterationsDone);
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
			UERROR("Failed to optimize the graph! returning empty optimized poses...");
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
		std::multimap<int, Link> * constraints,
		double * error,
		int * iterationsDone) const
{
	UTimer timer;
	std::map<int, Transform> optimizedPoses;
	std::map<int, Transform> poses, posesOut;
	std::multimap<int, Link> edgeConstraints, linksOut;
	UDEBUG("ids=%d", (int)ids.size());
	_memory->getMetricConstraints(ids, poses, edgeConstraints, lookInDatabase);
	UINFO("get constraints (ids=%d, %d poses, %d edges) time %f s", (int)ids.size(), (int)poses.size(), (int)edgeConstraints.size(), timer.ticks());

	// Apply guess poses (if some)
	if(_graphOptimizer->iterations() > 0)
	{
		for(std::map<int, Transform>::const_iterator iter=guessPoses.begin(); iter!=guessPoses.end(); ++iter)
		{
			std::map<int, Transform>::iterator foundPose = poses.find(iter->first);
			if(foundPose!=poses.end())
			{
				foundPose->second = iter->second;
			}
		}
	}

	// The constraints must be all already connected! Only check in debug
	if(ULogger::level() == ULogger::kDebug)
	{
		_graphOptimizer->getConnectedGraph(fromId, poses, edgeConstraints, posesOut, linksOut);
		if(poses.size() != posesOut.size())
		{
			for(std::map<int, Transform>::iterator iter=poses.begin(); iter!=poses.end(); ++iter)
			{
				if(posesOut.find(iter->first) == posesOut.end())
				{
					UERROR("Not found %d in posesOut", iter->first);
					for(std::multimap<int, Link>::iterator jter=edgeConstraints.begin(); jter!=edgeConstraints.end(); ++jter)
					{
						if(jter->second.from() == iter->first || jter->second.to()==iter->first)
						{
							UERROR("Found link %d->%d", jter->second.from(), jter->second.to());
						}
					}
				}
			}
		}
		if(edgeConstraints.size() != linksOut.size())
		{
			for(std::multimap<int, Link>::iterator iter=edgeConstraints.begin(); iter!=edgeConstraints.end(); ++iter)
			{
				if(graph::findLink(linksOut, iter->second.from(), iter->second.to()) == linksOut.end())
				{
					UERROR("Not found link %d->%d in linksOut", iter->second.from(), iter->second.to());
				}
			}
		}
		UASSERT_MSG(poses.size() == posesOut.size() && edgeConstraints.size() == linksOut.size(),
				uFormat("nodes %d->%d, links %d->%d", poses.size(), posesOut.size(), edgeConstraints.size(), linksOut.size()).c_str());
	}

	if(constraints)
	{
		*constraints = edgeConstraints;
	}

	UASSERT(_graphOptimizer!=0);
	if(_graphOptimizer->iterations() == 0)
	{
		// Optimization desactivated! Return not optimized poses.
		optimizedPoses = poses;
	}
	else
	{
		optimizedPoses = _graphOptimizer->optimize(fromId, poses, edgeConstraints, 0, error, iterationsDone);
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

void Rtabmap::get3DMap(
		std::map<int, Signature> & signatures,
		std::map<int, Transform> & poses,
		std::multimap<int, Link> & constraints,
		bool optimized,
		bool global) const
{
	UDEBUG("");
	if(_memory && _memory->getLastWorkingSignature())
	{
		if(_rgbdSlamMode)
		{
			if(optimized)
			{
				poses = _optimizedPoses; // guess
				this->optimizeCurrentMap(_memory->getLastWorkingSignature()->id(), global, poses, &constraints);
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

		// Get data
		std::set<int> ids = uKeysSet(_memory->getWorkingMem()); // WM

		//remove virtual signature
		ids.erase(Memory::kIdVirtual);

		ids.insert(_memory->getStMem().begin(), _memory->getStMem().end()); // STM + WM
		if(global)
		{
			ids = _memory->getAllSignatureIds(); // STM + WM + LTM
		}

		for(std::set<int>::iterator iter = ids.begin(); iter!=ids.end(); ++iter)
		{
			Transform odomPoseLocal;
			int weight = -1;
			int mapId = -1;
			std::string label;
			double stamp = 0;
			Transform groundTruth;
			_memory->getNodeInfo(*iter, odomPoseLocal, mapId, weight, label, stamp, groundTruth, true);
			SensorData data = _memory->getNodeData(*iter);
			data.setId(*iter);
			std::multimap<int, cv::KeyPoint> words;
			std::multimap<int, cv::Point3f> words3;
			std::multimap<int, cv::Mat> wordsDescriptors;
			_memory->getNodeWords(*iter, words, words3, wordsDescriptors);
			signatures.insert(std::make_pair(*iter,
					Signature(*iter,
							mapId,
							weight,
							stamp,
							label,
							odomPoseLocal,
							groundTruth,
							data)));
			signatures.at(*iter).setWords(words);
			signatures.at(*iter).setWords3(words3);
			signatures.at(*iter).setWordsDescriptors(wordsDescriptors);
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

void Rtabmap::getGraph(
		std::map<int, Transform> & poses,
		std::multimap<int, Link> & constraints,
		bool optimized,
		bool global,
		std::map<int, Signature> * signatures)
{
	if(_memory && _memory->getLastWorkingSignature())
	{
		if(_rgbdSlamMode)
		{
			if(optimized)
			{
				poses = _optimizedPoses; // guess
				this->optimizeCurrentMap(_memory->getLastWorkingSignature()->id(), global, poses, &constraints);
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
			for(std::map<int, Transform>::iterator iter=poses.begin(); iter!=poses.end(); ++iter)
			{
				Transform odomPoseLocal;
				int weight = -1;
				int mapId = -1;
				std::string label;
				double stamp = 0;
				Transform groundTruth;
				_memory->getNodeInfo(iter->first, odomPoseLocal, mapId, weight, label, stamp, groundTruth, global);
				signatures->insert(std::make_pair(iter->first,
						Signature(iter->first,
							mapId,
							weight,
							stamp,
							label,
							odomPoseLocal,
							groundTruth)));

				std::multimap<int, cv::KeyPoint> words;
				std::multimap<int, cv::Point3f> words3;
				std::multimap<int, cv::Mat> wordsDescriptors;
				_memory->getNodeWords(iter->first, words, words3, wordsDescriptors);
				signatures->at(iter->first).setWords(words);
				signatures->at(iter->first).setWords3(words3);
				signatures->at(iter->first).setWordsDescriptors(wordsDescriptors);

				std::vector<CameraModel> models;
				StereoCameraModel stereoModel;
				_memory->getNodeCalibration(iter->first, models, stereoModel);
				signatures->at(iter->first).sensorData().setCameraModels(models);
				signatures->at(iter->first).sensorData().setStereoCameraModel(stereoModel);
			}
		}
	}
	else if(_memory && (_memory->getStMem().size() || _memory->getWorkingMem().size()))
	{
		UERROR("Last working signature is null!?");
	}
	else if(_memory == 0)
	{
		UWARN("Memory not initialized...");
	}
}

int Rtabmap::detectMoreLoopClosures(float clusterRadius, float clusterAngle, int iterations, const ProgressState * processState)
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

	std::list<Link> loopClosuresAdded;
	std::multimap<int, int> checkedLoopClosures;

	std::map<int, Transform> poses;
	std::multimap<int, Link> links;
	std::map<int, Signature> signatures;
	this->getGraph(poses, links, true, true, &signatures);

	//remove all invalid or intermediate nodes
	for(std::map<int, Transform>::iterator iter=poses.begin(); iter!=poses.end();)
	{
		if(signatures.at(iter->first).getWeight() < 0)
		{
			poses.erase(iter++);
		}
		else
		{
			++iter;
		}
	}

	for(int n=0; n<iterations; ++n)
	{
		UINFO("Looking for more loop closures, clustering poses... (iteration=%d/%d, radius=%f m angle=%f rad)",
				n+1, iterations, clusterRadius, clusterAngle);

		std::multimap<int, int> clusters = graph::radiusPosesClustering(
				poses,
				clusterRadius,
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
			if(iter->first < iter->second)
			{
				from = iter->second;
				to = iter->first;
			}

			if(rtabmap::graph::findLink(checkedLoopClosures, from, to) == checkedLoopClosures.end())
			{
				// only add new links and one per cluster per iteration
				if(addedLinks.find(from) == addedLinks.end() &&
				   addedLinks.find(to) == addedLinks.end() &&
				   rtabmap::graph::findLink(links, from, to) == links.end())
				{
					checkedLoopClosures.insert(std::make_pair(from, to));

					UASSERT(signatures.find(from) != signatures.end());
					UASSERT(signatures.find(to) != signatures.end());

					RegistrationInfo info;
					// use signatures instead of IDs because some signatures may not be in WM
					Transform t = _memory->computeTransform(signatures.at(from), signatures.at(to), Transform(), &info);

					if(!t.isNull())
					{
						bool updateConstraints = true;
						if(_optimizationMaxLinearError > 0.0f)
						{
							//optimize the graph to see if the new constraint is globally valid

							int fromId = from;
							int mapId = signatures.at(from).mapId();
							// use first node of the map containing from
							for(std::map<int, Signature>::iterator iter=signatures.begin(); iter!=signatures.end(); ++iter)
							{
								if(iter->second.mapId() == mapId)
								{
									fromId = iter->first;
									break;
								}
							}
							std::multimap<int, Link> linksIn = links;
							linksIn.insert(std::make_pair(from, Link(from, to, Link::kUserClosure, t, info.varianceAng, info.varianceLin)));
							const Link * maxLinearLink = 0;
							const Link * maxAngularLink = 0;
							float maxLinearError = 0.0f;
							float maxAngularError = 0.0f;
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
								for(std::multimap<int, Link>::iterator iter=links.begin(); iter!=links.end(); ++iter)
								{
									// ignore links with high variance
									if(iter->second.transVariance() <= 1.0)
									{
										UASSERT(optimizedPoses.find(iter->second.from())!=optimizedPoses.end());
										UASSERT(optimizedPoses.find(iter->second.to())!=optimizedPoses.end());
										Transform t1 = optimizedPoses.at(iter->second.from());
										Transform t2 = optimizedPoses.at(iter->second.to());
										UASSERT(!t1.isNull() && !t2.isNull());
										Transform t = t1.inverse()*t2;
										float linearError = uMax3(
												fabs(iter->second.transform().x() - t.x()),
												fabs(iter->second.transform().y() - t.y()),
												fabs(iter->second.transform().z() - t.z()));
										Eigen::Vector3f vA = t1.toEigen3f().rotation()*Eigen::Vector3f(1,0,0);
										Eigen::Vector3f vB = t2.toEigen3f().rotation()*Eigen::Vector3f(1,0,0);
										float angularError = pcl::getAngle3D(Eigen::Vector4f(vA[0], vA[1], vA[2], 0), Eigen::Vector4f(vB[0], vB[1], vB[2], 0));
										if(linearError > maxLinearError)
										{
											maxLinearError = linearError;
											maxLinearLink = &iter->second;
										}
										if(angularError > maxAngularError)
										{
											maxAngularError = angularError;
											maxAngularLink = &iter->second;
										}
									}
								}
								if(maxLinearLink)
								{
									UINFO("Max optimization linear error = %f m (link %d->%d)", maxLinearError, maxLinearLink->from(), maxLinearLink->to());
								}
								if(maxAngularLink)
								{
									UINFO("Max optimization angular error = %f deg (link %d->%d)", maxAngularError*180.0f/M_PI, maxAngularLink->from(), maxAngularLink->to());
								}

								if(maxLinearError > _optimizationMaxLinearError)
								{
									msg = uFormat("Rejecting edge %d->%d because "
											  "graph error is too large after optimization (%f m for edge %d->%d, %f deg for edge %d->%d). "
											  "\"%s\" is %f m.",
											  from,
											  to,
											  maxLinearError,
											  maxLinearLink->from(),
											  maxLinearLink->to(),
											  maxAngularError*180.0f/M_PI,
											  maxAngularLink?maxAngularLink->from():0,
											  maxAngularLink?maxAngularLink->to():0,
											  Parameters::kRGBDOptimizeMaxError().c_str(),
											  _optimizationMaxLinearError);
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
						}

						if(updateConstraints)
						{
							UINFO("Added new loop closure between %d and %d.", from, to);
							addedLinks.insert(from);
							addedLinks.insert(to);
							links.insert(std::make_pair(from, Link(from, to, Link::kUserClosure, t, info.varianceAng, info.varianceLin)));
							loopClosuresAdded.push_back(Link(from, to, Link::kUserClosure, t, info.varianceAng, info.varianceLin));
							UINFO("Detected loop closure %d->%d! (%d/%d)", from, to, i+1, (int)clusters.size());
						}
					}
				}
			}
		}

		if(processState)
		{
			std::string msg = uFormat("Iteration %d/%d: Detected %d loop closures!", n+1, iterations, (int)addedLinks.size()/2);
			UINFO(msg.c_str());
			if(!processState->callback(msg))
			{
				return -1;
			}
		}
		else
		{
			UINFO("Iteration %d/%d: Detected %d loop closures!", n+1, iterations, (int)addedLinks.size()/2);
		}

		if(addedLinks.size() == 0)
		{
			break;
		}

		if(n+1 < iterations)
		{
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
	}
	UINFO("Total added %d loop closures.", (int)loopClosuresAdded.size());

	if(loopClosuresAdded.size())
	{
		for(std::list<Link>::iterator iter=loopClosuresAdded.begin(); iter!=loopClosuresAdded.end(); ++iter)
		{
			_memory->addLink(*iter, true);
		}
	}
	return (int)loopClosuresAdded.size();
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
	for(std::multimap<int, Link>::iterator iter=links.begin(); iter!= links.end(); ++iter)
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
			linksRefined.push_back(Link(from, to, iter->second.type(), t, info.varianceAng, info.varianceLin));
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
	UINFO("Planning a path to node %d (global=%d)", targetNode, global?1:0);
	this->clearPath(0);

	if(!_rgbdSlamMode)
	{
		UWARN("A path can only be computed in RGBD-SLAM mode");
		return false;
	}

	UTimer totalTimer;
	UTimer timer;

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
			if(_lastLocalizationPose.isNull() || _optimizedPoses.size() == 0)
			{
				UWARN("Last localization pose is null... cannot compute a path");
				return false;
			}
			currentNode = graph::findNearestNode(_optimizedPoses, _lastLocalizationPose);
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
				_path[oi].first = iter->first;
				_path[oi++].second = t * iter->second;
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
		updateGoalIndex();
		return _path.size() || _pathStatus > 0;
	}

	return false;
}

bool Rtabmap::computePath(const Transform & targetPose)
{
	UINFO("Planning a path to pose %s ", targetPose.prettyPrint().c_str());

	this->clearPath(0);
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
	for(std::map<int, Transform>::iterator iter=nodes.begin(); iter!=nodes.end(); ++iter)
	{
		const Signature * s = _memory->getSignature(iter->first);
		UASSERT(s);
		for(std::map<int, Link>::const_iterator jter=s->getLinks().begin(); jter!=s->getLinks().end(); ++jter)
		{
			// only add links for which poses are in "nodes"
			if(uContains(nodes, jter->second.to()))
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
		if(_lastLocalizationPose.isNull() || _optimizedPoses.size() == 0)
		{
			UWARN("Last localization pose is null... cannot compute a path");
			return false;
		}
		currentNode = graph::findNearestNode(_optimizedPoses, _lastLocalizationPose);
	}

	int nearestId;
	if(!_lastLocalizationPose.isNull() && _lastLocalizationPose.getDistance(targetPose) < _localRadius)
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
		if(_localRadius != 0.0f && targetPose.getDistance(nodes.at(nearestId)) > _localRadius)
		{
			UWARN("Cannot plan farther than %f m from the graph! (distance=%f m from node %d)",
					_localRadius, targetPose.getDistance(nodes.at(nearestId)), nearestId);
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
		std::map<int, Link> links = currentIndexS->getLinks(); // make a copy
		bool latestVirtualLinkFound = false;
		for(std::map<int, Link>::reverse_iterator iter=links.rbegin(); iter!=links.rend(); ++iter)
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
								_memory->addLink(Link(_path[i].first, _path[i-1].first, Link::kVirtualClosure, virtualLoop, 100, 100)); // on the optimized path
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
			UASSERT(_pathGoalIndex < _path.size() && _pathGoalIndex >= 0);
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

} // namespace rtabmap
