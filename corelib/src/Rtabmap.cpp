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
#include "rtabmap/core/RtabmapEvent.h"
#include "rtabmap/core/Version.h"
#include "rtabmap/core/Features2d.h"

#include "rtabmap/core/Camera.h"
#include "Signature.h"

#include "rtabmap/core/EpipolarGeometry.h"

#include "rtabmap/core/Memory.h"
#include "BayesFilter.h"

#include <utilite/UtiLite.h>

#include "SimpleIni.h"

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
const char * Rtabmap::kDefaultDatabaseName = "LTM.db";

Rtabmap::Rtabmap(const std::string & workingDirectory, bool deleteMemory) :
	_publishStats(Parameters::defaultRtabmapPublishStats()),
	_publishImage(Parameters::defaultRtabmapPublishImage()),
	_publishPdf(Parameters::defaultRtabmapPublishPdf()),
	_publishLikelihood(Parameters::defaultRtabmapPublishLikelihood()),
	_publishKeypoints(Parameters::defaultKpPublishKeypoints()),
	_maxTimeAllowed(Parameters::defaultRtabmapTimeThr()), // 700 ms
	_maxMemoryAllowed(Parameters::defaultRtabmapMemoryThr()), // 0=inf
	_imageBufferMaxSize(Parameters::defaultRtabmapImageBufferSize()),
	_loopThr(Parameters::defaultRtabmapLoopThr()),
	_loopRatio(Parameters::defaultRtabmapLoopRatio()),
	_maxRetrieved(Parameters::defaultRtabmapMaxRetrieved()),
	_likelihoodNullValuesIgnored(Parameters::defaultRtabmapLikelihoodNullValuesIgnored()),
	_statisticLogsBufferedInRAM(Parameters::defaultRtabmapStatisticLogsBufferedInRAM()),
	_lcHypothesisId(0),
	_retrievedId(0),
	_lastLcHypothesisValue(0),
	_lastLoopClosureId(0),
	_lastProcessTime(0.0),
	_epipolarGeometry(0),
	_bayesFilter(0),
	_memory(0),
	_foutFloat(0),
	_foutInt(0)
{
	this->setWorkingDirectory(workingDirectory);
	if(deleteMemory)
	{
		this->resetMemory(true);
	}
}

Rtabmap::~Rtabmap() {
	UDEBUG("");

	UEventsManager::removeHandler(this);

	// Stop the thread first
	join(true);

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

	this->releaseAllStrategies();
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
	fopen_s(&_foutFloat, (_wDir+LOG_F).toStdString().c_str(), attributes.c_str());
	fopen_s(&_foutInt, (_wDir+LOG_I).toStdString().c_str(), attributes.c_str());
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

void Rtabmap::releaseAllStrategies()
{
	ULOGGER_DEBUG("");
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

void Rtabmap::pushNewState(State newState, const ParametersMap & parameters)
{
	ULOGGER_DEBUG("to %d", newState);

	_stateMutex.lock();
	{
		_state.push(newState);
		_stateParam.push(parameters);
	}
	_stateMutex.unlock();

	_imageAdded.release();
}

void Rtabmap::init(const ParametersMap & parameters)
{
	if(this->isRunning())
	{
		pushNewState(kStateChangingParameters, parameters);
	}
	else
	{
		this->parseParameters(parameters);
	}
	setupLogFiles();
}

void Rtabmap::init(const char * configFile)
{
	// fill ctrl struct with values from the configuration file
	ParametersMap param;// = Parameters::defaultParameters;

	if(configFile)
	{
		ULOGGER_DEBUG("Read parameters from = %s", configFile);
		this->readParameters(configFile, param);
	}

	this->init(param);
}

void Rtabmap::close()
{
	UEventsManager::removeHandler(this);

	// Stop the thread first
	join(true);

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

	this->releaseAllStrategies();
}

void Rtabmap::parseParameters(const ParametersMap & parameters)
{
	ULOGGER_DEBUG("");
	ParametersMap::const_iterator iter;
	if((iter=parameters.find(Parameters::kRtabmapPublishStats())) != parameters.end())
	{
		_publishStats = uStr2Bool(iter->second.c_str());
	}
	if((iter=parameters.find(Parameters::kRtabmapPublishImage())) != parameters.end())
	{
		_publishImage = uStr2Bool(iter->second.c_str());
	}
	if((iter=parameters.find(Parameters::kRtabmapPublishPdf())) != parameters.end())
	{
		_publishPdf = uStr2Bool(iter->second.c_str());
	}
	if((iter=parameters.find(Parameters::kRtabmapPublishLikelihood())) != parameters.end())
	{
		_publishLikelihood = uStr2Bool(iter->second.c_str());
	}
	if((iter=parameters.find(Parameters::kKpPublishKeypoints())) != parameters.end())
	{
		_publishKeypoints = uStr2Bool(iter->second.c_str());
	}
	if((iter=parameters.find(Parameters::kRtabmapTimeThr())) != parameters.end())
	{
		_maxTimeAllowed = std::atof(iter->second.c_str());
	}
	if((iter=parameters.find(Parameters::kRtabmapMemoryThr())) != parameters.end())
	{
		_maxMemoryAllowed = std::atoi(iter->second.c_str());
	}
	if((iter=parameters.find(Parameters::kRtabmapLoopThr())) != parameters.end())
	{
		_loopThr = std::atof(iter->second.c_str());
	}
	if((iter=parameters.find(Parameters::kRtabmapLoopRatio())) != parameters.end())
	{
		_loopRatio = std::atof(iter->second.c_str());
	}
	if((iter=parameters.find(Parameters::kRtabmapImageBufferSize())) != parameters.end())
	{
		_imageBufferMaxSize = std::atoi(iter->second.c_str());
	}
	if((iter=parameters.find(Parameters::kRtabmapWorkingDirectory())) != parameters.end())
	{
		this->setWorkingDirectory(iter->second.c_str());
	}
	if((iter=parameters.find(Parameters::kRtabmapMaxRetrieved())) != parameters.end())
	{
		_maxRetrieved = std::atoi(iter->second.c_str());
	}
	if((iter=parameters.find(Parameters::kRtabmapLikelihoodNullValuesIgnored())) != parameters.end())
	{
		_likelihoodNullValuesIgnored = uStr2Bool(iter->second.c_str());
	}
	if((iter=parameters.find(Parameters::kRtabmapStatisticLogsBufferedInRAM())) != parameters.end())
	{
		_statisticLogsBufferedInRAM = uStr2Bool(iter->second.c_str());
	}

	// By default, we create our strategies if they are not already created.
	// If they already exists, we check the parameters if a change is requested

	if(!_memory)
	{
		UEventsManager::post(new RtabmapEventInit(RtabmapEventInit::kInitializing));
		UEventsManager::post(new RtabmapEventInit("Creating memory..."));
		_memory = new Memory(parameters);
		UEventsManager::post(new RtabmapEventInit("Creating memory, done!"));
		_memory->init(_wDir + kDefaultDatabaseName, false, parameters);
		UEventsManager::post(new RtabmapEventInit(RtabmapEventInit::kInitialized));
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
	if(_memory && _memory->getLastSignature())
	{
		id = _memory->getLastSignature()->id();
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
	std::set<int> mem;
	if(_memory)
	{
		mem = _memory->getStMem();
	}
	return mem;
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
	ULOGGER_DEBUG("");
	int memSize = 0;
	if(_memory)
	{
		const Signature * s  =_memory->getLastSignature();
		if(s)
		{
			memSize = s->id();
		}
	}
	return memSize;
}

void Rtabmap::clearBufferedSensors()
{
	_imageMutex.lock();
	{
		_imageBuffer.clear();
	}
	_imageMutex.unlock();
}


void Rtabmap::mainLoopBegin()
{
	if(!_memory || !_bayesFilter)
	{
		ULOGGER_DEBUG("Rtabmap thread started without all strategies defined...");
		//this->killSafely();
	}
}

void Rtabmap::mainLoopKill()
{
	this->clearBufferedSensors();

	// this will post the newData semaphore
	_imageAdded.release();
}

void Rtabmap::mainLoop()
{
	State state = kStateDetecting;
	ParametersMap parameters;

	_stateMutex.lock();
	{
		if(!_state.empty() && !_stateParam.empty())
		{
			state = _state.top();
			_state.pop();
			parameters = _stateParam.top();
			_stateParam.pop();
		}
	}
	_stateMutex.unlock();

	switch(state)
	{
	case kStateDetecting:
		this->process();
		break;
	case kStateChangingParameters:
		this->parseParameters(parameters);
		break;
	case kStateReseting:
		this->resetMemory();
		break;
	case kStateDumpingMemory:
		if(_memory)
		{
			_memory->dumpMemory(this->getWorkingDir());
		}
		break;
	case kStateDumpingPrediction:
		if(_memory && _bayesFilter)
		{
			this->dumpPrediction();
		}
		break;
	case kStateGeneratingGraph:
		if(_memory)
		{
			_memory->joinTrashThread(); // make sure the trash is flushed
			_memory->generateGraph(parameters.at("path"));
		}
		break;
	case kStateGeneratingLocalGraph:
		if(_memory)
		{
			_memory->joinTrashThread(); // make sure the trash is flushed
			this->generateLocalGraph(parameters.at("path"), atoi(parameters.at("id").c_str()), atoi(parameters.at("margin").c_str()));
		}
		break;
	case kStateDeletingMemory:
		this->resetMemory(true);
		break;
	case kStateCleanSensorsBuffer:
		this->clearBufferedSensors();
		break;
	default:
		UFATAL("Invalid state !?!?");
		break;
	}
}

void Rtabmap::generateGraph(const std::string & path) const
{
	if(!this->isRunning() && _memory)
	{
		_memory->generateGraph(path);
	}
}

void Rtabmap::resetMemory(bool dbOverwritten)
{
	if(_memory)
	{
		UEventsManager::post(new RtabmapEventInit(RtabmapEventInit::kInitializing));
		_memory->init(_wDir + kDefaultDatabaseName, dbOverwritten);
		UEventsManager::post(new RtabmapEventInit(RtabmapEventInit::kInitialized));
		if(_bayesFilter)
		{
			_bayesFilter->reset();
		}
	}
	else if(dbOverwritten)
	{
		// FIXME May be not work with other database type.
		// May be memory should be already created here, and use init above...
		UFile::erase(_wDir + kDefaultDatabaseName);
	}
	this->clearBufferedSensors();
	_retrievedId = 0;
	_lastLcHypothesisValue = 0;
	this->setupLogFiles(dbOverwritten);
}

void Rtabmap::handleEvent(UEvent* event)
{
	if(this->isRunning() && event->getClassName().compare("CameraEvent") == 0)
	{
		CameraEvent * e = (CameraEvent*)event;
		if(e->getCode() == CameraEvent::kCodeImage || e->getCode() == CameraEvent::kCodeFeatures)
		{
			this->addImage(e->image());
		}
	}
	else if(event->getClassName().compare("RtabmapEventCmd") == 0)
	{
		RtabmapEventCmd * rtabmapEvent = (RtabmapEventCmd*)event;
		RtabmapEventCmd::Cmd cmd = rtabmapEvent->getCmd();
		if(cmd == RtabmapEventCmd::kCmdResetMemory)
		{
			if(_memory)
			{
				ULOGGER_DEBUG("CMD_RESET_MEMORY");
				pushNewState(kStateReseting);
			}
		}
		else if(cmd == RtabmapEventCmd::kCmdDumpMemory)
		{
			if(_memory)
			{
				ULOGGER_DEBUG("CMD_DUMP_MEMORY");
				pushNewState(kStateDumpingMemory);
			}
		}
		else if(cmd == RtabmapEventCmd::kCmdDumpPrediction)
		{
			if(_memory && _bayesFilter)
			{
				ULOGGER_DEBUG("CMD_DUMP_PREDICTION");
				pushNewState(kStateDumpingPrediction);
			}
		}
		else if(cmd == RtabmapEventCmd::kCmdGenerateGraph)
		{
			if(_memory && !rtabmapEvent->getStr().empty())
			{
				ULOGGER_DEBUG("CMD_GENERATE_GRAPH");
				ParametersMap param;
				param.insert(ParametersPair("path", rtabmapEvent->getStr()));
				pushNewState(kStateGeneratingGraph, param);
			}
		}
		else if(cmd == RtabmapEventCmd::kCmdGenerateLocalGraph)
		{
			std::list<std::string> values = uSplit(rtabmapEvent->getStr(), ';');
			if(_memory && values.size() == 3)
			{
				ULOGGER_DEBUG("CMD_GENERATE_LOCAL_GRAPH");
				ParametersMap param;
				param.insert(ParametersPair("path", *values.begin()));
				param.insert(ParametersPair("id", *(++values.begin())));
				param.insert(ParametersPair("margin", *values.rbegin()));
				pushNewState(kStateGeneratingLocalGraph, param);
			}
		}
		else if(cmd == RtabmapEventCmd::kCmdDeleteMemory)
		{
			ULOGGER_DEBUG("CMD_DELETE_MEMORY");
			pushNewState(kStateDeletingMemory);
		}
		else if(cmd == RtabmapEventCmd::kCmdCleanSensorsBuffer)
		{
			ULOGGER_DEBUG("CMD_CLEAN_SENSORS_BUFFER");
			pushNewState(kStateCleanSensorsBuffer);
		}
	}
	else if(event->getClassName().compare("ParamEvent") == 0)
	{
		ULOGGER_DEBUG("changing parameters");
		pushNewState(kStateChangingParameters, ((ParamEvent*)event)->getParameters());
	}
}

//============================================================
// MAIN LOOP
//============================================================
void Rtabmap::process()
{
	UDEBUG("");

	//============================================================
	// Initialization
	//============================================================
	UTimer timer;
	UTimer timerTotal;
	double timeMemoryUpdate = 0;
	double timeCleaningNeighbors = 0;
	double timeReactivations = 0;
	double timeAddLoopClosureLink = 0;
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
	std::map<std::string, float> memUpdateStats;

	int refId = Memory::kIdInvalid;
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

	const Signature * signature = 0;
	const Signature * sLoop = 0;
	Image image;

	_lcHypothesisId = 0;

	std::set<int> immunizedLocations;

	//============================================================
	// Wait for an image...
	//============================================================
	ULOGGER_INFO("getting data...");
	this->getImage(image);
	if(image.empty())
	{
		ULOGGER_INFO("image is null...");
		return;
	}

	timer.start();
	timerTotal.start();

	if(!_memory || !_bayesFilter)
	{
		UFATAL("RTAB-Map is not initialized, data received is ignored.");
	}

	//============================================================
	// Memory Update : Location creation + Add to STM + Weight Update (Rehearsal)
	//============================================================
	ULOGGER_INFO("Updating memory...");
	if(!_memory->update(image, memUpdateStats))
	{
		return;
	}
	signature = _memory->getLastSignature();
	if(!signature)
	{
		UFATAL("Not supposed to be here...last signature is null?!?");
	}
	ULOGGER_INFO("Processing signature %d", signature->id());
	refId = signature->id();
	timeMemoryUpdate = timer.ticks();
	ULOGGER_INFO("timeMemoryUpdate=%fs", timeMemoryUpdate);

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
			adjustedLikelihood = rawLikelihood;
			this->adjustLikelihood(adjustedLikelihood);

			// Apply sensory attention
			likelihood = adjustedLikelihood;

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

			//for(std::map<int,float>::iterator iter=posterior.begin(); iter!=posterior.end(); ++iter)
			//{
			//	UDEBUG("posterior (%d) = %f", iter->first, iter->second);
			//}
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
					if(_lastLcHypothesisValue && hypothesis.second >= _loopRatio*_lastLcHypothesisValue &&
					   (!_epipolarGeometry || _epipolarGeometry->check(signature, _memory->getSignature(hypothesis.first))))
					{
						_lcHypothesisId = hypothesis.first;
					}
					else
					{
						rejectedHypothesis = true;
					}

					timeHypothesesValidation = timer.ticks();
					ULOGGER_INFO("timeHypothesesValidation=%fs",timeHypothesesValidation);
				}
				else if(hypothesis.second < _loopRatio*_lastLcHypothesisValue)
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
				hypothesisRatio = _lastLcHypothesisValue>0?hypothesis.second/_lastLcHypothesisValue:0;
				_lastLcHypothesisValue = hypothesis.second;

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
		rejectedHypothesis = !_memory->addLoopClosureLink(_lcHypothesisId, signature->id());

		if(rejectedHypothesis)
		{
			_lcHypothesisId = 0;
		}
	}
	timeAddLoopClosureLink = timer.ticks();
	ULOGGER_INFO("timeAddLoopClosureLink=%fs", timeAddLoopClosureLink);

	//============================================================
	// Prepare statistics
	//============================================================
	// Data used for the statistics event and for the log files
	int dictionarySize = 0;
	int refWordsCount = 0;
	int refUniqueWordsCount = 0;
	int lcHypothesisReactivated = 0;
	float rehearsalValue = uValue(memUpdateStats, std::string("Memory/Rehearsal Max Value/"), 0.0f);
	int rehearsalMaxId = (int)uValue(memUpdateStats, std::string("Memory/Rehearsal Max Id/"), 0.0f);
	sLoop = _memory->getSignature(hypothesis.first);
	if(sLoop)
	{
		lcHypothesisReactivated = sLoop->isSaved()?1.0f:0.0f;
	}
	dictionarySize = _memory->getVWDictionarySize();
	refWordsCount = signature->getWords().size();
	refUniqueWordsCount = uUniqueKeys(signature->getWords()).size();

	// Posterior is empty if a bad signature is detected
	float vpHypothesis = posterior.size()?posterior.at(Memory::kIdVirtual):0.0f;

	// only prepare statistics if required or when there is a loop closure
	Statistics * stat = 0;
	if(_lcHypothesisId || _publishStats)
	{
		ULOGGER_INFO("sending stats...");
		stat = new Statistics();
		stat->setRefImageId(refId);
		if(_lcHypothesisId != Memory::kIdInvalid)
		{
			stat->setLoopClosureId(_lcHypothesisId);
			ULOGGER_INFO("Loop closure detected! With id=%d", _lcHypothesisId);
		}
		if(_publishStats && refId != Memory::kIdInvalid)
		{
			ULOGGER_INFO("send all stats...");
			stat->setExtended(1);

			stat->addStatistic(Statistics::kLoopHighest_hypothesis_id(), hypothesis.first);
			stat->addStatistic(Statistics::kLoopHighest_hypothesis_value(), hypothesis.second);
			stat->addStatistic(Statistics::kHypothesis_reactivated(), lcHypothesisReactivated);
			stat->addStatistic(Statistics::kLoopVp_hypothesis(), vpHypothesis);
			stat->addStatistic(Statistics::kLoopReactivateId(), _retrievedId);
			stat->addStatistic(Statistics::kLoopHypothesis_ratio(), hypothesisRatio);

			stat->addStatistic(Statistics::kMemoryWorking_memory_size(), _memory->getWorkingMem().size());
			stat->addStatistic(Statistics::kMemoryShort_time_memory_size(), _memory->getStMem().size());
			stat->addStatistic(Statistics::kMemorySignatures_retrieved(), (float)signaturesRetrieved.size());
			stat->addStatistic(Statistics::kMemoryImages_buffered(), (float)_imageBuffer.size());

			// timing...
			stat->addStatistic(Statistics::kTimingMemory_update(), timeMemoryUpdate*1000);
			stat->addStatistic(Statistics::kTimingReactivation(), timeReactivations*1000);
			stat->addStatistic(Statistics::kTimingAdd_loop_closure_link(), timeAddLoopClosureLink*1000);
			stat->addStatistic(Statistics::kTimingLikelihood_computation(), timeLikelihoodCalculation*1000);
			stat->addStatistic(Statistics::kTimingPosterior_computation(), timePosteriorCalculation*1000);
			stat->addStatistic(Statistics::kTimingHypotheses_creation(), timeHypothesesCreation*1000);
			stat->addStatistic(Statistics::kTimingHypotheses_validation(), timeHypothesesValidation*1000);
			stat->addStatistic(Statistics::kTimingCleaning_neighbors(), timeCleaningNeighbors*1000);

			// memory update timings
			for(std::map<std::string, float>::iterator iter = memUpdateStats.begin(); iter!=memUpdateStats.end(); ++iter)
			{
				stat->addStatistic(iter->first, iter->second);
			}

			// Surf specific parameters
			stat->addStatistic(Statistics::kKeypointDictionary_size(), dictionarySize);

			//Epipolar geometry constraint
			stat->addStatistic(Statistics::kLoopRejectedHypothesis(), rejectedHypothesis?1.0f:0);

			if(_publishImage)
			{
				stat->setRefImage(image.image()); // raw data
				if(sLoop)
				{
					UTimer tmpTimer;
					cv::Mat im = _memory->getImage(sLoop->id());

					if(tmpTimer.elapsed() > 0.03)
					{
						UWARN("getting image time = %fs", tmpTimer.ticks());
					}
					stat->setLoopImage(im);
				}
			}

			if(_publishLikelihood || _publishPdf)
			{
				// Child count by parent signature on the root of the memory ... for statistics
				stat->setWeights(weights);
				if(_publishPdf)
				{
					stat->setPosterior(posterior);
				}
				if(_publishLikelihood)
				{
					stat->setLikelihood(likelihood);
					stat->setRawLikelihood(rawLikelihood);
				}
			}

			if(_publishKeypoints)
			{
				//Copy keypoints
				stat->setRefWords(signature->getWords());
				if(sLoop)
				{
					//Copy keypoints
					stat->setLoopWords(sLoop->getWords());
				}
			}
		}

		timeStatsCreation = timer.ticks();
		ULOGGER_INFO("Time creating stats = %f...", timeStatsCreation);
	}

	//By default, remove all signatures with a loop closure link if they are not in reactivateIds
	//This will also remove rehearsed signatures
	int signaturesRemoved = 0;
	signaturesRemoved += _memory->cleanup();
	timeMemoryCleanup = timer.ticks();
	ULOGGER_INFO("timeMemoryCleanup = %fs...", timeMemoryCleanup);

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
		signaturesRemoved = _memory->forget(immunizedLocations);
	}
	_lastProcessTime = totalTime;

	timeRealTimeLimitReachedProcess = timer.ticks();
	ULOGGER_INFO("Time limit reached processing = %f...", timeRealTimeLimitReachedProcess);

	//Start trashing
	_memory->emptyTrash();


	//==============================================================
	// Finalize statistics and log files
	//==============================================================
	if(stat)
	{
		if(_publishStats)
		{
			stat->addStatistic(Statistics::kTimingStatistics_creation(), timeStatsCreation*1000);
			stat->addStatistic(Statistics::kTimingTotal(), totalTime*1000);
			stat->addStatistic(Statistics::kTimingForgetting(), timeRealTimeLimitReachedProcess*1000);
			stat->addStatistic(Statistics::kTimingJoining_trash(), timeJoiningTrash*1000);
			stat->addStatistic(Statistics::kTimingEmptying_trash(), timeEmptyingTrash*1000);
			stat->addStatistic(Statistics::kTimingMemory_cleanup(), timeMemoryCleanup*1000);
			stat->addStatistic(Statistics::kMemorySignatures_removed(), signaturesRemoved);
		}
		ULOGGER_DEBUG("posting stat event...");
		this->post(new RtabmapEvent(&stat)); // the stat will be automatically deleted
	}

	std::list<float> nonNulls;
	float sumLikelihoods = 0.0f;
	float maxLikelihood = 0.0f;
	float mean = 0.0f;
	float stddev = 0.0f;
	std::map<int, float>::iterator iter=likelihood.begin();
	if(iter->first == -1)
	{
		++iter;
	}
	for(; iter!=likelihood.end(); ++iter)
	{
		if(iter->second > 0)
		{
			sumLikelihoods += iter->second;
			nonNulls.push_back(iter->second);
		}
		if(iter->second > maxLikelihood)
		{
			maxLikelihood = iter->second;
		}
	}
	if(nonNulls.size())
	{
		mean = sumLikelihoods / float(nonNulls.size());
		stddev = uStdDev(nonNulls, mean);
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
								_lastLcHypothesisValue,
								0.0f,
								maxLikelihood,
								sumLikelihoods,
								mean,
								stddev,
								vpHypothesis,
								timeJoiningTrash,
								rehearsalValue,
								timeEmptyingTrash,
								timeRetrievalDbAccess,
								timeAddLoopClosureLink);
	std::string logI = uFormat("%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d\n",
								_lcHypothesisId,
								hypothesis.first,
								signaturesRemoved,
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
								int(nonNulls.size()),
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

}

void Rtabmap::addImage(const Image & image)
{
	if(image.empty())
	{
		ULOGGER_ERROR("image empty !?");
		return;
	}

	bool notify = true;
	_imageMutex.lock();
	{
		_imageBuffer.push_back(image);
		while(_imageBufferMaxSize > 0 && _imageBuffer.size() >= (unsigned int)_imageBufferMaxSize)
		{
			ULOGGER_WARN("Data buffer is full, the oldest data is removed to add the new one.");
			_imageBuffer.pop_front();
			notify = false;
		}
	}
	_imageMutex.unlock();

	if(notify)
	{
		_imageAdded.release();
	}
}

void Rtabmap::getImage(Image & image)
{
	ULOGGER_DEBUG("");

	ULOGGER_INFO("waiting for data");
	_imageAdded.acquire();
	ULOGGER_INFO("wake-up");

	_imageMutex.lock();
	{
		if(!_imageBuffer.empty())
		{
			image = _imageBuffer.front();
			_imageBuffer.pop_front();
		}
	}
	_imageMutex.unlock();
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

void Rtabmap::setDataBufferSize(int size)
{
	if(size < 0)
	{
		ULOGGER_WARN("size < 0, then setting it to 0 (inf).");
		_imageBufferMaxSize = 0;
	}
	else
	{
		_imageBufferMaxSize = size;
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
		ULOGGER_DEBUG("Comparing new path \"%s\" with \"%s\"", path.c_str(), _wDir.c_str());
		if(path.compare(_wDir) != 0)
		{
			_wDir = path;
			if(_memory)
			{
				UEventsManager::post(new RtabmapEventInit(RtabmapEventInit::kInitializing));
				_memory->init(_wDir + kDefaultDatabaseName);
				UEventsManager::post(new RtabmapEventInit(RtabmapEventInit::kInitialized));
				setupLogFiles();
				//clear all buffered images
				this->clearBufferedSensors();
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

void Rtabmap::process(const cv::Mat & image)
{
	this->process(Image(image));
}

void Rtabmap::process(const Image & image)
{
	if(!this->isRunning())
	{
		this->addImage(image);
		this->process();
	}
	else
	{
		UERROR("The core thread is running!");
	}
}

void Rtabmap::dumpData()
{
	if(_memory)
	{
		_memory->dumpMemory(this->getWorkingDir());
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
	for(std::map<int, float>::iterator iter = ++likelihood.begin(); iter!=likelihood.end(); ++iter)
	{
		if((iter->second >= 0 && !_likelihoodNullValuesIgnored) ||
		   (iter->second > 0 && _likelihoodNullValuesIgnored))
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

void Rtabmap::generateLocalGraph(const std::string & path, int id, int margin)
{
	if(_memory)
	{
		std::map<int, int> ids = _memory->getNeighborsId(id, margin, -1, false);

		if(ids.size() > 0)
		{
			ids.insert(std::pair<int,int>(id, 0));
			std::set<int> idsSet;
			for(std::map<int, int>::iterator iter = ids.begin(); iter!=ids.end(); ++iter)
			{
				idsSet.insert(idsSet.end(), iter->first);
				UINFO("Node %d", iter->first);
			}
			UINFO("idsSet=%d", idsSet.size());
			_memory->generateGraph(path, idsSet);
		}
		else
		{
			UERROR("No neighbors found for signature %d.", id);
		}
	}
}

void Rtabmap::readParameters(const char * configFile, ParametersMap & parameters)
{
	CSimpleIniA ini;
	ini.LoadFile(configFile);
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
				    "The ini file will be automatically created when this node will close.", configFile);
	}
}

void Rtabmap::writeParameters(const char * configFile, const ParametersMap & parameters)
{
	CSimpleIniA ini;
	ini.LoadFile(configFile);

	for(ParametersMap::const_iterator i=parameters.begin(); i!=parameters.end(); ++i)
	{
		std::string key = (*i).first;
		key = uReplaceChar(key, '/', '\\'); // Ini files use \ by default for separators, so replace the /
		ini.SetValue("Core", key.c_str(), (*i).second.c_str(), NULL, true);
	}

	ini.SaveFile(configFile);
}

} // namespace rtabmap
