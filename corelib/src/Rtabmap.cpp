/*
Copyright (c) 2010-2014, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
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
#include "rtabmap/core/Graph.h"
#include "rtabmap/core/Signature.h"

#include "rtabmap/core/EpipolarGeometry.h"

#include "rtabmap/core/Memory.h"
#include "rtabmap/core/VWDictionary.h"
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
	_statisticLogsBufferedInRAM(Parameters::defaultRtabmapStatisticLogsBufferedInRAM()),
	_statisticLogged(Parameters::defaultRtabmapStatisticLogged()),
	_statisticLoggedHeaders(Parameters::defaultRtabmapStatisticLoggedHeaders()),
	_rgbdSlamMode(Parameters::defaultRGBDEnabled()),
	_rgbdLinearUpdate(Parameters::defaultRGBDLinearUpdate()),
	_rgbdAngularUpdate(Parameters::defaultRGBDAngularUpdate()),
	_newMapOdomChangeDistance(Parameters::defaultRGBDNewMapOdomChangeDistance()),
	_globalLoopClosureIcpType(Parameters::defaultLccIcpType()),
	_poseScanMatching(Parameters::defaultRGBDPoseScanMatching()),
	_localLoopClosureDetectionTime(Parameters::defaultRGBDLocalLoopDetectionTime()),
	_localLoopClosureDetectionSpace(Parameters::defaultRGBDLocalLoopDetectionSpace()),
	_localRadius(Parameters::defaultRGBDLocalRadius()),
	_localImmunizationRatio(Parameters::defaultRGBDLocalImmunizationRatio()),
	_localDetectMaxGraphDepth(Parameters::defaultRGBDLocalLoopDetectionMaxGraphDepth()),
	_localPathFilteringRadius(Parameters::defaultRGBDLocalLoopDetectionPathFilteringRadius()),
	_localPathOdomPosesUsed(Parameters::defaultRGBDLocalLoopDetectionPathOdomPosesUsed()),
	_databasePath(""),
	_optimizeFromGraphEnd(Parameters::defaultRGBDOptimizeFromGraphEnd()),
	_reextractLoopClosureFeatures(Parameters::defaultLccReextractActivated()),
	_reextractNNType(Parameters::defaultLccReextractNNType()),
	_reextractNNDR(Parameters::defaultLccReextractNNDR()),
	_reextractFeatureType(Parameters::defaultLccReextractFeatureType()),
	_reextractMaxWords(Parameters::defaultLccReextractMaxWords()),
	_reextractMaxDepth(Parameters::defaultLccReextractMaxDepth()),
	_startNewMapOnLoopClosure(Parameters::defaultRtabmapStartNewMapOnLoopClosure()),
	_goalReachedRadius(Parameters::defaultRGBDGoalReachedRadius()),
	_planVirtualLinks(Parameters::defaultRGBDPlanVirtualLinks()),
	_goalsSavedInUserData(Parameters::defaultRGBDGoalsSavedInUserData()),
	_loopClosureHypothesis(0,0.0f),
	_highestHypothesis(0,0.0f),
	_lastProcessTime(0.0),
	_epipolarGeometry(0),
	_bayesFilter(0),
	_graphOptimizer(0),
	_memory(0),
	_foutFloat(0),
	_foutInt(0),
	_wDir("."),
	_mapCorrection(Transform::getIdentity()),
	_mapTransform(Transform::getIdentity()),
	_pathCurrentIndex(0),
	_pathGoalIndex(0),
	_pathTransformToGoal(Transform::getIdentity())
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
		}

		ULOGGER_DEBUG("Log file (int)=%s", (_wDir+"/"+LOG_I).c_str());
		ULOGGER_DEBUG("Log file (float)=%s", (_wDir+"/"+LOG_F).c_str());
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

void Rtabmap::init(const ParametersMap & parameters, const std::string & databasePath)
{
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
		UWARN("Using empty database. Mapping session will not be saved.");
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
		this->readParameters(configFile, param);
	}

	this->init(param, databasePath);
}

void Rtabmap::close()
{
	UINFO("");
	_highestHypothesis = std::make_pair(0,0.0f);
	_loopClosureHypothesis = std::make_pair(0,0.0f);
	_lastProcessTime = 0.0;
	_optimizedPoses.clear();
	_constraints.clear();
	_mapCorrection.setIdentity();
	_mapTransform.setIdentity();
	_lastLocalizationPose.setNull();
	this->clearPath();

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
	if(_graphOptimizer)
	{
		delete _graphOptimizer;
		_graphOptimizer = 0;
	}
	_databasePath.clear();
	parseParameters(Parameters::getDefaultParameters()); // reset to default parameters
	_modifiedParameters.clear();
}

void Rtabmap::parseParameters(const ParametersMap & parameters)
{
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
	Parameters::parse(parameters, Parameters::kRtabmapStatisticLogsBufferedInRAM(), _statisticLogsBufferedInRAM);
	Parameters::parse(parameters, Parameters::kRtabmapStatisticLogged(), _statisticLogged);
	Parameters::parse(parameters, Parameters::kRtabmapStatisticLoggedHeaders(), _statisticLoggedHeaders);
	Parameters::parse(parameters, Parameters::kRGBDEnabled(), _rgbdSlamMode);
	Parameters::parse(parameters, Parameters::kRGBDLinearUpdate(), _rgbdLinearUpdate);
	Parameters::parse(parameters, Parameters::kRGBDAngularUpdate(), _rgbdAngularUpdate);
	Parameters::parse(parameters, Parameters::kRGBDNewMapOdomChangeDistance(), _newMapOdomChangeDistance);
	Parameters::parse(parameters, Parameters::kRGBDPoseScanMatching(), _poseScanMatching);
	Parameters::parse(parameters, Parameters::kRGBDLocalLoopDetectionTime(), _localLoopClosureDetectionTime);
	Parameters::parse(parameters, Parameters::kRGBDLocalLoopDetectionSpace(), _localLoopClosureDetectionSpace);
	Parameters::parse(parameters, Parameters::kRGBDLocalRadius(), _localRadius);
	Parameters::parse(parameters, Parameters::kRGBDLocalImmunizationRatio(), _localImmunizationRatio);
	Parameters::parse(parameters, Parameters::kRGBDLocalLoopDetectionMaxGraphDepth(), _localDetectMaxGraphDepth);
	Parameters::parse(parameters, Parameters::kRGBDLocalLoopDetectionPathFilteringRadius(), _localPathFilteringRadius);
	Parameters::parse(parameters, Parameters::kRGBDLocalLoopDetectionPathOdomPosesUsed(), _localPathOdomPosesUsed);
	Parameters::parse(parameters, Parameters::kRGBDOptimizeFromGraphEnd(), _optimizeFromGraphEnd);
	Parameters::parse(parameters, Parameters::kLccReextractActivated(), _reextractLoopClosureFeatures);
	Parameters::parse(parameters, Parameters::kLccReextractNNType(), _reextractNNType);
	Parameters::parse(parameters, Parameters::kLccReextractNNDR(), _reextractNNDR);
	Parameters::parse(parameters, Parameters::kLccReextractFeatureType(), _reextractFeatureType);
	Parameters::parse(parameters, Parameters::kLccReextractMaxWords(), _reextractMaxWords);
	Parameters::parse(parameters, Parameters::kLccReextractMaxDepth(), _reextractMaxDepth);
	Parameters::parse(parameters, Parameters::kRtabmapStartNewMapOnLoopClosure(), _startNewMapOnLoopClosure);
	Parameters::parse(parameters, Parameters::kRGBDGoalReachedRadius(), _goalReachedRadius);
	Parameters::parse(parameters, Parameters::kRGBDPlanVirtualLinks(), _planVirtualLinks);
	Parameters::parse(parameters, Parameters::kRGBDGoalsSavedInUserData(), _goalsSavedInUserData);

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

	// Graph optimizer
	graph::Optimizer::Type optimizerType = graph::Optimizer::kTypeUndef;
	if((iter=parameters.find(Parameters::kRGBDOptimizeStrategy())) != parameters.end())
	{
		optimizerType = (graph::Optimizer::Type)std::atoi((*iter).second.c_str());
	}
	if(optimizerType!=graph::Optimizer::kTypeUndef)
	{
		UDEBUG("new detector strategy %d", int(optimizerType));
		if(_graphOptimizer)
		{
			delete _graphOptimizer;
			_graphOptimizer = 0;
		}

		_graphOptimizer = graph::Optimizer::create(optimizerType, parameters);
	}
	else if(_graphOptimizer)
	{
		_graphOptimizer->parseParameters(parameters);
	}
	else
	{
		optimizerType = (graph::Optimizer::Type)Parameters::defaultRGBDOptimizeStrategy();
		_graphOptimizer = graph::Optimizer::create(optimizerType, parameters);
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

	for(ParametersMap::const_iterator iter = parameters.begin(); iter!=parameters.end(); ++iter)
	{
		uInsert(_modifiedParameters, ParametersPair(iter->first, iter->second));
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
		mapId = _memory->incrementMapId();
		UINFO("New map triggered, new map = %d", mapId);
		_optimizedPoses.clear();
		_constraints.clear();
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
			_memory->getMetricConstraints(uKeysSet(ids), poses, constraints, global);
		}

		graph::TOROOptimizer::saveGraph(path, poses, constraints);
	}
}

void Rtabmap::exportPoses(const std::string & path, bool optimized, bool global)
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

		this->dumpPoses(path, poses);
	}
}

void Rtabmap::resetMemory()
{
	_highestHypothesis = std::make_pair(0,0.0f);
	_loopClosureHypothesis = std::make_pair(0,0.0f);
	_lastProcessTime = 0.0;
	_optimizedPoses.clear();
	_constraints.clear();
	_mapCorrection.setIdentity();
	_mapTransform.setIdentity();
	_lastLocalizationPose.setNull();
	this->clearPath();

	if(_memory)
	{
		_memory->init(_databasePath, true, _modifiedParameters, true);
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
		const Transform & odomPose,
		const cv::Mat & covariance)
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
	std::list<std::pair<int, float> > reactivateHypotheses;

	std::map<int, int> childCount;
	std::set<int> signaturesRetrieved;
	int localLoopClosuresInTimeFound = 0;
	bool scanMatchingSuccess = false;

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
	if(_rgbdSlamMode)
	{
		if(odomPose.isNull())
		{
			UERROR("RGB-D SLAM mode is enabled and no odometry is provided. "
				   "Image %d is ignored!", data.id());
			return false;
		}
		else
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

	ULOGGER_INFO("Processing signature %d", signature->id());
	timeMemoryUpdate = timer.ticks();
	ULOGGER_INFO("timeMemoryUpdate=%fs", timeMemoryUpdate);

	//============================================================
	// Metric
	//============================================================
	bool smallDisplacement = false;
	if(_rgbdSlamMode)
	{
		//Verify if there was a rehearsal
		int rehearsedId = (int)uValue(statistics_.data(), Statistics::kMemoryRehearsal_merged(), 0.0f);
		if(rehearsedId > 0)
		{
			_optimizedPoses.erase(rehearsedId);
		}
		else if(_rgbdLinearUpdate > 0.0f && _rgbdAngularUpdate > 0.0f)
		{
			//============================================================
			// Minimum displacement required to add to Memory
			//============================================================
			const std::map<int, Link> & links = signature->getLinks();
			if(links.size() == 1)
			{
				float x,y,z, roll,pitch,yaw;
				links.begin()->second.transform().getTranslationAndEulerAngles(x,y,z, roll,pitch,yaw);
				if((_rgbdLinearUpdate==0.0f || (
					 fabs(x) < _rgbdLinearUpdate &&
					 fabs(y) < _rgbdLinearUpdate &&
					 fabs(z) < _rgbdLinearUpdate)) &&
					(_rgbdAngularUpdate==0.0f || (
					 fabs(roll) < _rgbdAngularUpdate &&
					 fabs(pitch) < _rgbdAngularUpdate &&
					 fabs(yaw) < _rgbdAngularUpdate)))
				{
					// This will disable global loop closure detection, only retrieval will be done.
					// The location will also be deleted at the end.
					smallDisplacement = true;
				}
			}
		}

		Transform newPose = _mapCorrection * signature->getPose();
		_optimizedPoses.insert(std::make_pair(signature->id(), newPose));
		_lastLocalizationPose = newPose; // used in localization mode only (path planning)

		//============================================================
		// Scan matching
		//============================================================
		if(_poseScanMatching &&
			signature->getLinks().size() == 1 &&
			!signature->sensorData().laserScanCompressed().empty() &&
			rehearsedId == 0) // don't do it if rehearsal happened
		{
			UINFO("Odometry correction by scan matching");
			int oldId = signature->getLinks().begin()->first;
			const Signature * oldS = _memory->getSignature(oldId);
			UASSERT(oldS != 0);
			std::string rejectedMsg;
			Transform guess = signature->getLinks().begin()->second.transform();
			double variance = 1.0;
			int inliers = 0;
			float inliersRatio = 0;
			Transform t = _memory->computeIcpTransform(oldId, signature->id(), guess, false, &rejectedMsg, &inliers, &variance, &inliersRatio);
			if(!t.isNull())
			{
				scanMatchingSuccess = true;
				UINFO("Scan matching: update neighbor link (%d->%d) from %s to %s",
						signature->id(),
						oldId,
						signature->getLinks().at(oldId).transform().prettyPrint().c_str(),
						t.prettyPrint().c_str());
				_memory->updateLink(signature->id(), oldId, t, variance, variance);
			}
			else
			{
				UINFO("Scan matching rejected: %s", rejectedMsg.c_str());
			}
			statistics_.addStatistic(Statistics::kOdomCorrectionAccepted(), scanMatchingSuccess?1.0f:0);
			statistics_.addStatistic(Statistics::kOdomCorrectionInliers(), inliers);
			statistics_.addStatistic(Statistics::kOdomCorrectionInliers_ratio(), inliersRatio);
			statistics_.addStatistic(Statistics::kOdomCorrectionVariance(), variance);
		}
		timeScanMatching = timer.ticks();
		ULOGGER_INFO("timeScanMatching=%fs", timeScanMatching);

		if(signature->getLinks().size() == 1)
		{
			// link should be old to new
			UASSERT_MSG(signature->id() > signature->getLinks().begin()->second.to(),
					"Only forward links should be added.");

			Link tmp = signature->getLinks().begin()->second.inverse();

			// if the previous node is an intermediate node, remove it from the local graph
			if(_constraints.size() &&
			   _constraints.rbegin()->second.to() == signature->getLinks().begin()->second.to())
			{
				const Signature * s = _memory->getSignature(signature->getLinks().begin()->second.to());
				UASSERT(s!=0);
				if(s->getWeight() == -1)
				{
					tmp = _constraints.rbegin()->second.merge(tmp);
					_optimizedPoses.erase(s->id());
					_constraints.erase(--_constraints.end());
				}
			}
			_constraints.insert(std::make_pair(tmp.from(), tmp));
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
				   signature->getLinks().find(*iter) == signature->getLinks().end() &&
				   _memory->getSignature(*iter)->mapId() == signature->mapId())
				{
					std::string rejectedMsg;
					UDEBUG("Check local transform between %d and %d", signature->id(), *iter);
					double variance = 1.0;
					int inliers = -1;
					Transform transform = _memory->computeVisualTransform(*iter, signature->id(), &rejectedMsg, &inliers, &variance);
					if(!transform.isNull() && _globalLoopClosureIcpType > 0)
					{
						transform = _memory->computeIcpTransform(*iter, signature->id(), transform, _globalLoopClosureIcpType==1, &rejectedMsg, 0, &variance);
						variance = 1.0f; // ICP, set variance to 1
					}
					if(!transform.isNull())
					{
						UDEBUG("Add local loop closure in TIME (%d->%d) %s",
								signature->id(),
								*iter,
								transform.prettyPrint().c_str());
						// Add a loop constraint
						if(_memory->addLink(Link(signature->id(), *iter, Link::kLocalTimeClosure, transform, variance, variance)))
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
					else
					{
						UINFO("Local loop closure (time) between %d and %d rejected: %s",
								*iter, signature->id(), rejectedMsg.c_str());
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
	int previousId = signature->getLinks().size() == 1?signature->getLinks().begin()->first:0;
	// Not a bad signature, not a small displacemnt unless the previous signature didn't have a loop closure
	if(!signature->isBadSignature() && (!smallDisplacement || _memory->getLoopClosureLinks(previousId, false).size() == 0))
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
				// Loop closure Threshold
				// When _loopThr=0, accept loop closure if the hypothesis is over
				// the virtual (new) place hypothesis.
				if(_highestHypothesis.second >= _loopThr)
				{
					rejectedHypothesis = true;
					if(posterior.size() <= 2)
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
					// When analysing logs, it's convenient to know
					// if the hypothesis would be rejected if T_loop would be lower.
					rejectedHypothesis = true;
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
							if(maxLocalLocationsImmunized > 20)
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
			UASSERT(s!=0);
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
	int loopClosureVisualInliers = 0; // for statistics
	if(_loopClosureHypothesis.first>0)
	{
		//Compute transform if metric data are present
		Transform transform;
		double variance = 1;
		if(_rgbdSlamMode)
		{
			std::string rejectedMsg;
			if(_reextractLoopClosureFeatures)
			{
				ParametersMap customParameters = _modifiedParameters; // get BOW LCC parameters
				// override some parameters
				uInsert(customParameters, ParametersPair(Parameters::kMemIncrementalMemory(), "true")); // make sure it is incremental
				uInsert(customParameters, ParametersPair(Parameters::kMemRehearsalSimilarity(), "1.0")); // desactivate rehearsal
				uInsert(customParameters, ParametersPair(Parameters::kMemBinDataKept(), "false"));
				uInsert(customParameters, ParametersPair(Parameters::kMemSTMSize(), "0"));
				uInsert(customParameters, ParametersPair(Parameters::kKpIncrementalDictionary(), "true")); // make sure it is incremental
				uInsert(customParameters, ParametersPair(Parameters::kKpNewWordsComparedTogether(), "false"));
				uInsert(customParameters, ParametersPair(Parameters::kKpNNStrategy(), uNumber2Str(_reextractNNType))); // bruteforce
				uInsert(customParameters, ParametersPair(Parameters::kKpNndrRatio(), uNumber2Str(_reextractNNDR)));
				uInsert(customParameters, ParametersPair(Parameters::kKpDetectorStrategy(), uNumber2Str(_reextractFeatureType))); // FAST/BRIEF
				uInsert(customParameters, ParametersPair(Parameters::kKpWordsPerImage(), uNumber2Str(_reextractMaxWords)));
				uInsert(customParameters, ParametersPair(Parameters::kKpMaxDepth(), uNumber2Str(_reextractMaxDepth)));
				uInsert(customParameters, ParametersPair(Parameters::kKpBadSignRatio(), "0"));
				uInsert(customParameters, ParametersPair(Parameters::kKpRoiRatios(), "0.0 0.0 0.0 0.0"));
				uInsert(customParameters, ParametersPair(Parameters::kMemGenerateIds(), "false"));

				//for(ParametersMap::iterator iter = customParameters.begin(); iter!=customParameters.end(); ++iter)
				//{
				//	UDEBUG("%s=%s", iter->first.c_str(), iter->second.c_str());
				//}

				Memory memory(customParameters);

				UTimer timeT;

				// Add signatures
				SensorData dataFrom = data;
				dataFrom.setId(signature->id());
				SensorData dataTo = _memory->getNodeData(_loopClosureHypothesis.first, true);
				UDEBUG("timeTo = %fs", timeT.ticks());

				if(!dataFrom.depthOrRightRaw().empty() &&
				   !dataTo.depthOrRightRaw().empty() &&
				   dataFrom.id() != Memory::kIdInvalid &&
				   dataTo.id() != Memory::kIdInvalid)
				{
					memory.update(dataTo);
					UDEBUG("timeUpTo = %fs", timeT.ticks());
					memory.update(dataFrom);
					UDEBUG("timeUpFrom = %fs", timeT.ticks());

					transform = memory.computeVisualTransform(dataTo.id(), dataFrom.id(), &rejectedMsg, &loopClosureVisualInliers, &variance);
					UDEBUG("timeTransform = %fs", timeT.ticks());
				}
				else
				{
					// Fallback to normal way (raw data not kept in database...)
					UWARN("Loop closure: Some images not found in memory for re-extracting "
						  "features, is Mem/RawDataKept=false? Falling back with already extracted 3D features.");
					transform = _memory->computeVisualTransform(_loopClosureHypothesis.first, signature->id(), &rejectedMsg, &loopClosureVisualInliers, &variance);
				}
			}
			else
			{
				transform = _memory->computeVisualTransform(_loopClosureHypothesis.first, signature->id(), &rejectedMsg, &loopClosureVisualInliers, &variance);
			}
			if(!transform.isNull() && _globalLoopClosureIcpType > 0)
			{
				transform = _memory->computeIcpTransform(_loopClosureHypothesis.first, signature->id(), transform, _globalLoopClosureIcpType == 1, &rejectedMsg, 0, &variance);
			}
			rejectedHypothesis = transform.isNull();
			if(rejectedHypothesis)
			{
				UINFO("Rejected loop closure %d -> %d: %s",
						_loopClosureHypothesis.first, signature->id(), rejectedMsg.c_str());
			}
		}
		if(!rejectedHypothesis)
		{
			// Make the new one the parent of the old one
			rejectedHypothesis = !_memory->addLink(Link(signature->id(), _loopClosureHypothesis.first, Link::kGlobalClosure, transform, variance, variance));
		}

		if(rejectedHypothesis)
		{
			_loopClosureHypothesis.first = 0;
		}
		else
		{
			const Signature * oldS = _memory->getSignature(_loopClosureHypothesis.first);
			UASSERT(oldS != 0);
			// Old map -> new map, used for localization correction on loop closure
			_mapTransform = oldS->getPose() * transform.inverse() * signature->getPose().inverse();
		}
	}

	timeAddLoopClosureLink = timer.ticks();
	ULOGGER_INFO("timeAddLoopClosureLink=%fs", timeAddLoopClosureLink);

	int localSpaceClosuresAddedVisually = 0;
	int localSpaceClosuresAddedByICPOnly = 0;
	int lastLocalSpaceClosureId = 0;
	int localSpacePaths = 0;
	if(_localLoopClosureDetectionSpace &&
	   _localRadius > 0)
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
				float r = _localRadius;
				if(_localPathFilteringRadius > 0 && _localPathFilteringRadius<_localRadius)
				{
					r = _localPathFilteringRadius;
				}

				std::map<int, float> nearestIds;
				if(_memory->isIncremental())
				{
					nearestIds = _memory->getNeighborsIdRadius(signature->id(), r, _optimizedPoses, _localDetectMaxGraphDepth);
				}
				else
				{
					nearestIds = graph::getNodesInRadius(signature->id(), _optimizedPoses, r);
				}
				std::map<int, Transform> nearestPoses;
				for(std::map<int, float>::iterator iter=nearestIds.begin(); iter!=nearestIds.end(); ++iter)
				{
					nearestPoses.insert(std::make_pair(iter->first, _optimizedPoses.at(iter->first)));
				}
				// segment poses by paths, only one detection per path
				std::list<std::map<int, Transform> > nearestPaths = getPaths(nearestPoses);
				for(std::list<std::map<int, Transform> >::iterator iter=nearestPaths.begin();
					iter!=nearestPaths.end() && (_memory->isIncremental() || lastLocalSpaceClosureId == 0);
					++iter)
				{
					std::map<int, Transform> & path = *iter;
					UASSERT(path.size());
					//find the nearest pose on the path
					int nearestId = rtabmap::graph::findNearestNode(path, _optimizedPoses.at(signature->id()));
					UASSERT(nearestId > 0);

					// nearest pose must not be linked to current location, and not in STM
					if(!signature->hasLink(nearestId) &&
					   _memory->getStMem().find(nearestId) == _memory->getStMem().end())
					{
						double variance = 1.0;
						Transform transform;
						if(_reextractLoopClosureFeatures)
						{
							ParametersMap customParameters = _modifiedParameters; // get BOW LCC parameters
							// override some parameters
							uInsert(customParameters, ParametersPair(Parameters::kMemIncrementalMemory(), "true")); // make sure it is incremental
							uInsert(customParameters, ParametersPair(Parameters::kMemRehearsalSimilarity(), "1.0")); // desactivate rehearsal
							uInsert(customParameters, ParametersPair(Parameters::kMemBinDataKept(), "false"));
							uInsert(customParameters, ParametersPair(Parameters::kMemSTMSize(), "0"));
							uInsert(customParameters, ParametersPair(Parameters::kKpIncrementalDictionary(), "true")); // make sure it is incremental
							uInsert(customParameters, ParametersPair(Parameters::kKpNewWordsComparedTogether(), "false"));
							uInsert(customParameters, ParametersPair(Parameters::kKpNNStrategy(), uNumber2Str(_reextractNNType))); // bruteforce
							uInsert(customParameters, ParametersPair(Parameters::kKpNndrRatio(), uNumber2Str(_reextractNNDR)));
							uInsert(customParameters, ParametersPair(Parameters::kKpDetectorStrategy(), uNumber2Str(_reextractFeatureType))); // FAST/BRIEF
							uInsert(customParameters, ParametersPair(Parameters::kKpWordsPerImage(), uNumber2Str(_reextractMaxWords)));
							uInsert(customParameters, ParametersPair(Parameters::kKpMaxDepth(), uNumber2Str(_reextractMaxDepth)));
							uInsert(customParameters, ParametersPair(Parameters::kKpBadSignRatio(), "0"));
							uInsert(customParameters, ParametersPair(Parameters::kKpRoiRatios(), "0.0 0.0 0.0 0.0"));
							uInsert(customParameters, ParametersPair(Parameters::kMemGenerateIds(), "false"));

							//for(ParametersMap::iterator iter = customParameters.begin(); iter!=customParameters.end(); ++iter)
							//{
							//	UDEBUG("%s=%s", iter->first.c_str(), iter->second.c_str());
							//}

							Memory memory(customParameters);

							UTimer timeT;

							// Add signatures
							SensorData dataFrom = data;
							dataFrom.setId(signature->id());
							SensorData dataTo = _memory->getNodeData(nearestId, true);
							UDEBUG("timeTo = %fs", timeT.ticks());

							if(!dataFrom.depthOrRightRaw().empty() &&
							   !dataTo.depthOrRightRaw().empty() &&
							   dataFrom.id() != Memory::kIdInvalid &&
							   dataTo.id() != Memory::kIdInvalid)
							{
								memory.update(dataTo);
								UDEBUG("timeUpTo = %fs", timeT.ticks());
								memory.update(dataFrom);
								UDEBUG("timeUpFrom = %fs", timeT.ticks());

								transform = memory.computeVisualTransform(dataTo.id(), dataFrom.id(), 0, 0, &variance);
								UDEBUG("timeTransform = %fs", timeT.ticks());
							}
							else
							{
								// Fallback to normal way (raw data not kept in database...)
								UWARN("Loop closure: Some images not found in memory for re-extracting "
									  "features, is Mem/RawDataKept=false? Falling back with already extracted 3D features.");
								transform = _memory->computeVisualTransform(nearestId, signature->id(), 0, 0, &variance);
							}
						}
						else
						{
							transform = _memory->computeVisualTransform(nearestId, signature->id(), 0, 0, &variance);
						}
						if(!transform.isNull() && _globalLoopClosureIcpType > 0)
						{
							transform  = _memory->computeIcpTransform(nearestId, signature->id(), transform, _globalLoopClosureIcpType == 1, 0, 0, &variance);
						}
						if(!transform.isNull())
						{
							UINFO("[Visual] Add local loop closure in SPACE (%d->%d) %s",
									signature->id(),
									nearestId,
									transform.prettyPrint().c_str());
							_memory->addLink(Link(signature->id(), nearestId, Link::kLocalSpaceClosure, transform, variance, variance));

							if(_loopClosureHypothesis.first == 0)
							{
								// Old map -> new map, used for localization correction on loop closure
								const Signature * oldS = _memory->getSignature(nearestId);
								UASSERT(oldS != 0);
								_mapTransform = oldS->getPose() * transform.inverse() * signature->getPose().inverse();
								++localSpaceClosuresAddedVisually;
								lastLocalSpaceClosureId = nearestId;
							}
						}
					}
				}

				//
				// 2) compare locally with nearest locations by scan matching
				//
				if( !signature->sensorData().laserScanCompressed().empty() &&
					(_memory->isIncremental() || lastLocalSpaceClosureId == 0))
				{
					// In localization mode, no need to check local loop
					// closures if we are already localized by at least one
					// local visual closure above.

					std::map<int, Transform> forwardPoses;
					forwardPoses = this->getForwardWMPoses(
							signature->id(),
							0,
							_localRadius,
							_localDetectMaxGraphDepth);

					std::list<std::map<int, Transform> > forwardPaths = getPaths(forwardPoses);
					localSpacePaths = (int)forwardPaths.size();

					for(std::list<std::map<int, Transform> >::iterator iter=forwardPaths.begin();
							iter!=forwardPaths.end() && (_memory->isIncremental() || lastLocalSpaceClosureId == 0);
							++iter)
					{
						std::map<int, Transform> & path = *iter;
						UASSERT(path.size());

						//find the nearest pose on the path
						int nearestId = rtabmap::graph::findNearestNode(path, _optimizedPoses.at(signature->id()));
						UASSERT(nearestId > 0);

						// nearest pose must be close and not linked to current location
						if(!signature->hasLink(nearestId) &&
						   (_localPathFilteringRadius <= 0.0f ||
							_optimizedPoses.at(signature->id()).getDistanceSquared(_optimizedPoses.at(nearestId)) < _localPathFilteringRadius*_localPathFilteringRadius))
						{
							// Assemble scans in the path and do ICP only
							if(_localPathOdomPosesUsed)
							{
								//optimize the path's poses locally
								path = optimizeGraph(nearestId, uKeysSet(path), false);
								// transform local poses in optimized graph referential
								Transform t = _optimizedPoses.at(nearestId) * path.at(nearestId).inverse();
								for(std::map<int, Transform>::iterator jter=path.begin(); jter!=path.end(); ++jter)
								{
									jter->second = t * jter->second;
								}
							}
							if(_localPathFilteringRadius > 0.0f)
							{
								// path filtering
								std::map<int, Transform> filteredPath = graph::radiusPosesFiltering(path, _localPathFilteringRadius, CV_PI, true);
								// make sure the nearest and farthest poses are still here
								filteredPath.insert(*path.find(nearestId));
								filteredPath.insert(*path.begin());
								filteredPath.insert(*path.rbegin());
								path = filteredPath;
							}

							if(path.size() > 2) // more than current+nearest
							{
								// add current node to poses
								path.insert(std::make_pair(signature->id(), _optimizedPoses.at(signature->id())));
								//The nearest will be the reference for a loop closure transform
								if(signature->getLinks().find(nearestId) == signature->getLinks().end())
								{
									Transform transform = _memory->computeScanMatchingTransform(signature->id(), nearestId, path, 0, 0, 0);
									if(!transform.isNull())
									{
										UINFO("[Scan matching] Add local loop closure in SPACE (%d->%d) %s",
												signature->id(),
												nearestId,
												transform.prettyPrint().c_str());
										// set Identify covariance for laser scan matching only
										_memory->addLink(Link(signature->id(), nearestId, Link::kLocalSpaceClosure, transform, 1, 1));

										++localSpaceClosuresAddedByICPOnly;

										// no local loop closure added visually
										if(localSpaceClosuresAddedVisually == 0 && _loopClosureHypothesis.first == 0)
										{
											// Old map -> new map, used for localization correction on loop closure
											const Signature * oldS = _memory->getSignature(nearestId);
											UASSERT(oldS != 0);
											_mapTransform = oldS->getPose() * transform.inverse() * signature->getPose().inverse();
											lastLocalSpaceClosureId = nearestId;
										}
									}
								}
							}
						}
					}
				}
			}
		}
	}
	timeLocalSpaceDetection = timer.ticks();
	ULOGGER_INFO("timeLocalSpaceDetection=%fs", timeLocalSpaceDetection);

	//============================================================
	// Optimize map graph
	//============================================================
	if(_rgbdSlamMode &&
		(_loopClosureHypothesis.first>0 ||				// can be different map of the current one
		 localLoopClosuresInTimeFound>0 || 	// only same map of the current one
		 scanMatchingSuccess || 			// only same map of the current one
		 lastLocalSpaceClosureId>0 || 	    // can be different map of the current one
		 signaturesRetrieved.size()))  		// can be different map of the current one
	{
		if(_memory->isIncremental())
		{
			UINFO("Update map correction: SLAM mode");
			// SLAM mode!
			optimizeCurrentMap(signature->id(), false, _optimizedPoses, &_constraints);
			UASSERT(_optimizedPoses.find(signature->id()) != _optimizedPoses.end());

			// Update map correction, it should be identify when optimizing from the last node
			_mapCorrection = _optimizedPoses.at(signature->id()) * signature->getPose().inverse();
			_mapTransform.setIdentity(); // reset mapTransform (used for localization only)
			_lastLocalizationPose = _optimizedPoses.at(signature->id()); // update in case we switch to localization mode
			if(_mapCorrection.getNormSquared() > 0.001f && _optimizeFromGraphEnd)
			{
				UERROR("Map correction should be identity when optimizing from the last node. T=%s", _mapCorrection.prettyPrint().c_str());
			}
		}
		else if(_loopClosureHypothesis.first > 0 || lastLocalSpaceClosureId > 0 || signaturesRetrieved.size())
		{
			UINFO("Update map correction: Localization mode");
			int oldId = _loopClosureHypothesis.first>0?_loopClosureHypothesis.first:lastLocalSpaceClosureId?lastLocalSpaceClosureId:_highestHypothesis.first;
			UASSERT(oldId != 0);
			if(signaturesRetrieved.size() || _optimizedPoses.find(oldId) == _optimizedPoses.end())
			{
				// update optimized poses
				optimizeCurrentMap(oldId, false, _optimizedPoses, &_constraints);
			}
			UASSERT(_optimizedPoses.find(oldId) != _optimizedPoses.end());

			// Localization mode! only update map correction
			const Signature * oldS = _memory->getSignature(oldId);
			UASSERT(oldS != 0);
			Transform correction = _optimizedPoses.at(oldId) * oldS->getPose().inverse();
			_mapCorrection = correction * _mapTransform;
			_lastLocalizationPose = _mapCorrection * signature->getPose();
		}
		else
		{
			UERROR("Not supposed to be here!");
		}
	}

	//============================================================
	// Add virtual links if a path is activated
	//============================================================
	if(_path.size())
	{
		// Add a virtual loop closure link to keep the path linked to local map
		if( signature->id() != _path[_pathCurrentIndex].first &&
			!signature->hasLink(_path[_pathCurrentIndex].first) &&
			uContains(_optimizedPoses, _path[_pathCurrentIndex].first))
		{
			Transform virtualLoop = _optimizedPoses.at(signature->id()).inverse() * _optimizedPoses.at(_path[_pathCurrentIndex].first);
			if(_localRadius > 0.0f && virtualLoop.getNorm() < _localRadius)
			{
				_memory->addLink(Link(signature->id(), _path[_pathCurrentIndex].first, Link::kVirtualClosure, virtualLoop, 100, 100)); // set high variance
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
	sLoop = _memory->getSignature(_loopClosureHypothesis.first?_loopClosureHypothesis.first:lastLocalSpaceClosureId?lastLocalSpaceClosureId:_highestHypothesis.first);
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
		statistics_.setRefImageId(signature->id());
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
			statistics_.addStatistic(Statistics::kLoopReactivateId(), retrievalId);
			statistics_.addStatistic(Statistics::kLoopHypothesis_ratio(), hypothesisRatio);
			statistics_.addStatistic(Statistics::kLoopVisualInliers(), loopClosureVisualInliers);
			statistics_.addStatistic(Statistics::kLoopLast_id(), _memory->getLastGlobalLoopClosureId());

			statistics_.addStatistic(Statistics::kLocalLoopTime_closures(), localLoopClosuresInTimeFound);
			statistics_.addStatistic(Statistics::kLocalLoopSpace_closures_added_visually(), localSpaceClosuresAddedVisually);
			statistics_.addStatistic(Statistics::kLocalLoopSpace_closures_added_icp_only(), localSpaceClosuresAddedByICPOnly);
			statistics_.addStatistic(Statistics::kLocalLoopSpace_paths(), localSpacePaths);
			statistics_.addStatistic(Statistics::kLocalLoopSpace_last_closure_id(), lastLocalSpaceClosureId);
			statistics_.setLocalLoopClosureId(lastLocalSpaceClosureId);
			if(_loopClosureHypothesis.first || lastLocalSpaceClosureId)
			{
				UASSERT(uContains(sLoop->getLinks(), signature->id()));
				UINFO("Set loop closure transform = %s", sLoop->getLinks().at(signature->id()).transform().prettyPrint().c_str());
				statistics_.setLoopClosureTransform(sLoop->getLinks().at(signature->id()).transform());
			}
			statistics_.setMapCorrection(_mapCorrection);
			UINFO("Set map correction = %s", _mapCorrection.prettyPrint().c_str());

			// timings...
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

			// retrieval
			statistics_.addStatistic(Statistics::kMemorySignatures_retrieved(), (float)signaturesRetrieved.size());

			// Surf specific parameters
			statistics_.addStatistic(Statistics::kKeypointDictionary_size(), dictionarySize);

			//Epipolar geometry constraint
			statistics_.addStatistic(Statistics::kLoopRejectedHypothesis(), rejectedHypothesis?1.0f:0);

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
	if(_publishLastSignatureData)
	{
		lastSignatureData = *signature;
	}

	// remove last signature if the memory is not incremental or is a bad signature (if bad signatures are ignored)
	std::list<int> signaturesRemoved;
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
			_memory->getWorkingMem().size()>1)       // The working memory should not be empty
		{
			UWARN("Ignoring location %d because a global loop closure is required before starting a new map!",
					signature->id());
			signaturesRemoved.push_back(signature->id());
			_memory->deleteLocation(signature->id());
		}
		else if(smallDisplacement && _loopClosureHypothesis.first == 0 && lastLocalSpaceClosureId == 0)
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
		std::list<int> transferred = _memory->forget(immunizedLocations);
		signaturesRemoved.insert(signaturesRemoved.end(), transferred.begin(), transferred.end());
	}
	_lastProcessTime = totalTime;

	//Remove optimized poses from signatures transferred
	if(signaturesRemoved.size() && (_optimizedPoses.size() || _constraints.size()))
	{
		//refresh the local map because some transferred nodes may have broken the tree
		if(_memory->getLastWorkingSignature())
		{
			std::map<int, int> ids = _memory->getNeighborsId(_memory->getLastWorkingSignature()->id(), 0, 0, true);
			for(std::map<int, Transform>::iterator iter=_optimizedPoses.begin(); iter!=_optimizedPoses.end();)
			{
				if(!uContains(ids, iter->first))
				{
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

		std::map<int, Signature> signatures;
		if(_publishLastSignatureData)
		{
			signatures.insert(std::make_pair(lastSignatureData.id(), lastSignatureData));
		}
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
		for(std::map<int, Transform>::iterator iter=poses.begin(); iter!=poses.end(); ++iter)
		{
			Transform odomPose;
			int weight = -1;
			int mapId = -1;
			std::string label;
			double stamp = 0;
			std::vector<unsigned char> userData;
			_memory->getNodeInfo(iter->first, odomPose, mapId, weight, label, stamp, false);
			signatures.insert(std::make_pair(iter->first,
					Signature(iter->first,
							mapId,
							weight,
							stamp,
							label,
							odomPose)));
		}
		statistics_.setPoses(poses);
		statistics_.setConstraints(constraints);
		statistics_.setSignatures(signatures);
		statistics_.addStatistic(Statistics::kMemoryLocal_graph_size(), poses.size());
		localGraphSize = (int)poses.size();
	}

	//Start trashing
	_memory->emptyTrash();

	// Log info...
	// TODO : use a specific class which will handle the RtabmapEvent
	if(_foutFloat && _foutInt)
	{
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
									timeScanMatching,
									timeLocalTimeDetection,
									timeLocalSpaceDetection,
									timeMapOptimization);
		std::string logI = uFormat("%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d\n",
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
									data.id());
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

void Rtabmap::dumpData() const
{
	UDEBUG("");
	if(_memory)
	{
		_memory->dumpMemory(this->getWorkingDir());
	}
}

void Rtabmap::dumpPoses(
		const std::string & path,
		const std::map<int, Transform> & poses) const
{
	UDEBUG("");
	FILE* fout = 0;
#ifdef _MSC_VER
	fopen_s(&fout, path.c_str(), "w");
#else
	fout = fopen(path.c_str(), "w");
#endif
	if(fout)
	{
		for(std::map<int, Transform>::const_iterator iter=poses.begin(); iter!=poses.end(); ++iter)
		{
			// in camera frame
			const float * p = (const float *)(*iter).second.data();

			fprintf(fout, "%f", p[0]);
			for(int i=1; i<(*iter).second.size(); i++)
			{
				fprintf(fout, " %f", p[i]);
			}
			fprintf(fout, "\n");
		}
		fclose(fout);
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

// Get paths in front of the robot, returned optimized poses
std::list<std::map<int, Transform> > Rtabmap::getPaths(std::map<int, Transform> poses) const
{
	std::list<std::map<int, Transform> > paths;
	if(_memory && poses.size())
	{
		// Segment poses connected only by neighbor links
		while(poses.size())
		{
			std::map<int, Transform> path;
			for(std::map<int, Transform>::iterator iter=poses.begin(); iter!=poses.end();)
			{
				if(path.size() == 0 || uContains(_memory->getNeighborLinks(path.rbegin()->first), iter->first))
				{
					path.insert(*iter);
					poses.erase(iter++);
				}
				else
				{
					break;
				}
			}
			UASSERT(path.size());
			paths.push_back(path);
		}

	}
	return paths;
}

void Rtabmap::optimizeCurrentMap(
		int id,
		bool lookInDatabase,
		std::map<int, Transform> & optimizedPoses,
		std::multimap<int, Link> * constraints) const
{
	//Optimize the map
	optimizedPoses.clear();
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

		optimizedPoses = Rtabmap::optimizeGraph(id, uKeysSet(ids), lookInDatabase, constraints);

		if(_memory->getSignature(id) && uContains(optimizedPoses, id))
		{
			Transform t = optimizedPoses.at(id) * _memory->getSignature(id)->getPose().inverse();
			UINFO("Correction (from node %d) %s", id, t.prettyPrint().c_str());
		}
		UINFO("optimize time %f s", timer.ticks());
	}
}

std::map<int, Transform> Rtabmap::optimizeGraph(
		int fromId,
		const std::set<int> & ids,
		bool lookInDatabase,
		std::multimap<int, Link> * constraints) const
{
	UTimer timer;
	std::map<int, Transform> optimizedPoses;
	std::map<int, Transform> poses;
	std::multimap<int, Link> edgeConstraints;
	UDEBUG("ids=%d", (int)ids.size());
	_memory->getMetricConstraints(ids, poses, edgeConstraints, lookInDatabase);
	UINFO("get constraints (%d poses, %d edges) time %f s", (int)poses.size(), (int)edgeConstraints.size(), timer.ticks());

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
		optimizedPoses = _graphOptimizer->optimize(fromId, poses, edgeConstraints);
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
			Transform odomPose;
			int weight = -1;
			int mapId = -1;
			std::string label;
			double stamp = 0;
			_memory->getNodeInfo(*iter, odomPose, mapId, weight, label, stamp, true);
			SensorData data = _memory->getNodeData(*iter);
			data.setId(*iter);
			std::multimap<int, cv::KeyPoint> words;
			std::multimap<int, pcl::PointXYZ> words3;
			_memory->getNodeWords(*iter, words, words3);
			signatures.insert(std::make_pair(*iter,
					Signature(*iter,
							mapId,
							weight,
							stamp,
							label,
							odomPose,
							data)));
			signatures.at(*iter).setWords(words);
			signatures.at(*iter).setWords3(words3);
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
				Transform odomPose;
				int weight = -1;
				int mapId = -1;
				std::string label;
				double stamp = 0;
				_memory->getNodeInfo(iter->first, odomPose, mapId, weight, label, stamp, global);
				signatures->insert(std::make_pair(iter->first,
						Signature(iter->first,
							mapId,
							weight,
							stamp,
							label,
							odomPose)));
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

void Rtabmap::clearPath()
{
	_path.clear();
	_pathCurrentIndex=0;
	_pathGoalIndex = 0;
	_pathTransformToGoal.setIdentity();
	if(_memory)
	{
		_memory->removeAllVirtualLinks();
	}
}

// return true if path is updated
bool Rtabmap::computePath(int targetNode, bool global)
{
	UINFO("Planning a path to node %d (global=%d)", targetNode, global?1:0);
	this->clearPath();

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
					global);

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
	}
	UINFO("Total planning time = %fs (%d nodes, %f m long)", totalTimer.ticks(), (int)_path.size(), graph::computePathLength(_path));

	if(_path.size() == 0)
	{
		_path.clear();
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
		if(_goalsSavedInUserData)
		{
			// set goal to latest signature
			std::string goalStr = uFormat("GOAL:%d", targetNode);
			setUserData(0, cv::Mat(1, int(goalStr.size()+1), CV_8SC1, (void *)goalStr.c_str()).clone());
		}
		updateGoalIndex();
	}

	return _path.size()>0;
}

bool Rtabmap::computePath(const Transform & targetPose)
{
	UINFO("Planning a path to pose %s ", targetPose.prettyPrint().c_str());

	this->clearPath();
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
			links.insert(std::make_pair(jter->second.from(), jter->second.to()));
			links.insert(std::make_pair(jter->second.to(), jter->second.from())); // <->
		}
	}
	UINFO("Time getting links = %fs", timer.ticks());

	int nearestId = rtabmap::graph::findNearestNode(nodes, targetPose);
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

			// Add links between neighbor nodes in the goal radius.
			if(_planVirtualLinks)
			{
				std::multimap<int, int> clusters = rtabmap::graph::radiusPosesClustering(nodes, _goalReachedRadius, CV_PI);
				for(std::multimap<int, int>::iterator iter=clusters.begin(); iter!=clusters.end(); ++iter)
				{
					if(graph::findLink(links, iter->first, iter->second) == links.end())
					{
						links.insert(*iter);
						links.insert(std::make_pair(iter->second, iter->first)); // <->
					}
				}
			}

			UINFO("Computing path from location %d to %d", currentNode, nearestId);
			UTimer timer;
			_path = uListToVector(rtabmap::graph::computePath(nodes, links, currentNode, nearestId));
			UINFO("A* time = %fs", timer.ticks());

			if(_path.size() == 0)
			{
				_path.clear();
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
			}
		}
	}
	else
	{
		UWARN("Nearest node not found in graph (size=%d) for pose %s", (int)nodes.size(), targetPose.prettyPrint().c_str());
	}

	return _path.size()>0;
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
			if(_memory->getSignature(_path[i].first))
			{
				_memory->removeVirtualLinks(_path[i].first);
			}
		}
		// for the current index, only keep the newest virtual link
		UASSERT(_pathCurrentIndex < _path.size());
		const Signature * currentIndexS = _memory->getSignature(_path[_pathCurrentIndex].first);
		UASSERT(currentIndexS != 0);
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
		for(unsigned int i=_pathCurrentIndex;
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
					const Signature * s = _memory->getSignature(_path[i].first);
					if(s)
					{
						if(!s->hasLink(_path[i-1].first) && _memory->getSignature(_path[i-1].first) != 0)
						{
							Transform virtualLoop = _path[i].second.inverse() * _path[i-1].second;
							_memory->addLink(Link(_path[i].first, _path[i-1].first, Link::kVirtualClosure, virtualLoop, 1, 1)); // on the optimized path, set Identity variance
							UINFO("Added Virtual link between %d and %d", _path[i-1].first, _path[i].first);
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
				this->clearPath();
				return;
			}
			currentPose = _optimizedPoses.at(_memory->getLastWorkingSignature()->id());
		}
		else
		{
			if(_lastLocalizationPose.isNull())
			{
				UERROR("Last localization pose is null. Aborting the plan...");
				this->clearPath();
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
				this->clearPath();
			}
		}

		if(_path.size())
		{
			//Always check if the farthest node is accessible in local map (max to local space radius if set)
			int goalIndex = _pathCurrentIndex;
			float distanceFromCurrentNode = 0.0f;
			for(unsigned int i=_pathCurrentIndex; i<_path.size(); ++i)
			{
				if(uContains(_optimizedPoses, _path[i].first))
				{
					if(_localRadius > 0.0f)
					{
						distanceFromCurrentNode = currentPose.getDistance(_optimizedPoses.at(_path[i].first));
					}

					if(distanceFromCurrentNode <= _localRadius)
					{
						goalIndex = i;
					}
					else
					{
						break;
					}
				}
				else
				{
					break;
				}
			}
			UASSERT(_pathGoalIndex < _path.size() && goalIndex >= 0 && goalIndex < (int)_path.size());
			if((int)_pathGoalIndex != goalIndex)
			{
				UINFO("Updated current goal from %d to %d (%d/%d)",
						(int)_path[_pathGoalIndex].first, _path[goalIndex].first, goalIndex+1, (int)_path.size());
				_pathGoalIndex = goalIndex;
			}

			// update nearest pose in the path
			unsigned int nearestNodeIndex = 0;
			float distance = -1.0f;
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
				this->clearPath();
			}
			else
			{
				UDEBUG("Nearest node = %d", _path[nearestNodeIndex].first);
			}
			if(distance >= 0 && nearestNodeIndex != _pathCurrentIndex)
			{
				_pathCurrentIndex = nearestNodeIndex;
			}
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
			if(key.compare("Version") == 0)
			{
				// Compare version in ini with the current RTAB-Map version
				std::vector<std::string> version = uListToVector(uSplit((*iter).second, '.'));
				if(version.size() == 3)
				{
					if(!RTABMAP_VERSION_COMPARE(std::atoi(version[0].c_str()), std::atoi(version[1].c_str()), std::atoi(version[2].c_str())))
					{
						if(configFile.find(".rtabmap") != std::string::npos)
						{
							UWARN("Version in the config file \"%s\" is more recent (\"%s\") than "
								   "current RTAB-Map version used (\"%s\"). The config file will be upgraded "
								   "to new version.",
								   configFile.c_str(),
								   (*iter).second,
								   RTABMAP_VERSION);
						}
						else
						{
							UERROR("Version in the config file \"%s\" is more recent (\"%s\") than "
								   "current RTAB-Map version used (\"%s\"). New parameters (if there are some) will "
								   "be ignored.",
								   configFile.c_str(),
								   (*iter).second,
								   RTABMAP_VERSION);
						}
					}
				}
			}
			else
			{
				key = uReplaceChar(key, '\\', '/'); // Ini files use \ by default for separators, so replace them
				ParametersMap::iterator jter = parameters.find(key);
				if(jter != parameters.end())
				{
					parameters.erase(jter);
				}
				parameters.insert(ParametersPair(key, (*iter).second));
			}
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

	// Save current version
	ini.SetValue("Core", "Version", RTABMAP_VERSION, NULL, true);

	for(ParametersMap::const_iterator i=parameters.begin(); i!=parameters.end(); ++i)
	{
		std::string key = (*i).first;
		key = uReplaceChar(key, '/', '\\'); // Ini files use \ by default for separators, so replace the /
		ini.SetValue("Core", key.c_str(), (*i).second.c_str(), NULL, true);
	}

	ini.SaveFile(configFile.c_str());
}

} // namespace rtabmap
