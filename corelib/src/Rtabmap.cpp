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
#include "rtabmap/core/Sensor.h"
#include "rtabmap/core/Actuator.h"
#include "rtabmap/core/SensorimotorEvent.h"
#include "rtabmap/core/KeypointDetector.h"

#include "rtabmap/core/Camera.h"
#include "rtabmap/core/VWDictionary.h"
#include "rtabmap/core/Signature.h"
#include "rtabmap/core/Micro.h"

#include "rtabmap/core/VerifyHypotheses.h"

#include "rtabmap/core/KeypointMemory.h"
#include "rtabmap/core/SMMemory.h"
#include "rtabmap/core/BayesFilter.h"

#include <utilite/UtiLite.h>

#include "SimpleIni.h"

#include <stdlib.h>
#include <set>

#define DB_TYPE "sqlite3"

#define LOG_F "LogF.txt"
#define LOG_I "LogI.txt"

#define GRAPH_FILE_NAME "Graph.dot"

namespace rtabmap
{
const char * Rtabmap::kDefaultIniFileName = "rtabmap.ini";
const char * Rtabmap::kDefaultDatabaseName = "LTM.db";

Rtabmap::Rtabmap() :
	_publishStats(Parameters::defaultRtabmapPublishStats()),
	_publishRawData(Parameters::defaultRtabmapPublishRawData()),
	_publishPdf(Parameters::defaultRtabmapPublishPdf()),
	_publishLikelihood(Parameters::defaultRtabmapPublishLikelihood()),
	_publishKeypoints(Parameters::defaultKpPublishKeypoints()),
	_publishMasks(Parameters::defaultSMPublishMasks()),
	_maxTimeAllowed(Parameters::defaultRtabmapTimeThr()), // 700 ms
	_maxMemoryAllowed(Parameters::defaultRtabmapMemoryThr()), // 0=inf
	_sensorsBufferMaxSize(Parameters::defaultRtabmapSMStateBufferSize()),
	_loopThr(Parameters::defaultRtabmapLoopThr()),
	_loopRatio(Parameters::defaultRtabmapLoopRatio()),
	_retrievalThr(Parameters::defaultRtabmapRetrievalThr()),
	_maxRetrieved(Parameters::defaultRtabmapMaxRetrieved()),
	_selectionNeighborhoodSummationUsed(Parameters::defaultRtabmapSelectionNeighborhoodSummationUsed()),
	_selectionLikelihoodUsed(Parameters::defaultRtabmapSelectionLikelihoodUsed()),
	_actionsSentRejectHyp(Parameters::defaultRtabmapActionsSentRejectHyp()),
	_confidenceThr(Parameters::defaultRtabmapConfidenceThr()),
	_likelihoodStdDevRemoved(Parameters::defaultRtabmapLikelihoodStdDevRemoved()),
	_likelihoodNullValuesIgnored(Parameters::defaultRtabmapLikelihoodNullValuesIgnored()),
	_lcHypothesisId(0),
	_reactivateId(0),
	_lastLcHypothesisValue(0),
	_lastLoopClosureId(0),
	_vhStrategy(0),
	_bayesFilter(0),
	_memory(0),
	_foutFloat(0),
	_foutInt(0),
	_graphFileName(GRAPH_FILE_NAME)
{
	ULOGGER_DEBUG("Working directory=%s", Parameters::defaultRtabmapWorkingDirectory().c_str());
	this->setWorkingDirectory(Parameters::defaultRtabmapWorkingDirectory());
}

Rtabmap::~Rtabmap() {
	UDEBUG("");

	UEventsManager::removeHandler(this);

	// Stop the thread first
	join(true);

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

#ifdef _MSC_VER
	fopen_s(&_foutFloat, (_wDir+LOG_F).toStdString().c_str(), attributes.c_str());
	fopen_s(&_foutInt, (_wDir+LOG_I).toStdString().c_str(), attributes.c_str());
#else
	_foutFloat = fopen((_wDir+LOG_F).c_str(), attributes.c_str());
	_foutInt = fopen((_wDir+LOG_I).c_str(), attributes.c_str());
#endif

	ULOGGER_DEBUG("Log file (int)=%s", (_wDir+LOG_I).c_str());
	ULOGGER_DEBUG("Log file (float)=%s", (_wDir+LOG_F).c_str());
}

void Rtabmap::releaseAllStrategies()
{
	ULOGGER_DEBUG("");
	if(_vhStrategy)
	{
		delete _vhStrategy;
		_vhStrategy = 0;
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

	_sensorimotorAdded.release();
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

std::string Rtabmap::getIniFilePath()
{
	std::string privatePath = UDirectory::homeDir() + "/.rtabmap";
	if(!UDirectory::exists(privatePath))
	{
		UDirectory::makeDir(privatePath);
	}
	return (privatePath + "/") + kDefaultIniFileName;
}

void Rtabmap::init(const char * configFile)
{
	std::string config = this->getIniFilePath();
	if(configFile)
	{
		config = configFile;
	}
	ULOGGER_DEBUG("file path = %s", config.c_str());

	// fill ctrl struct with values from the configuration file
	ParametersMap param;// = Parameters::defaultParameters;
	this->readParameters(config.c_str(), param);

	this->init(param);
}

void Rtabmap::parseParameters(const ParametersMap & parameters)
{
	ULOGGER_DEBUG("");
	ParametersMap::const_iterator iter;
	if((iter=parameters.find(Parameters::kRtabmapPublishStats())) != parameters.end())
	{
		_publishStats = uStr2Bool(iter->second.c_str());
	}
	if((iter=parameters.find(Parameters::kRtabmapPublishRawData())) != parameters.end())
	{
		_publishRawData = uStr2Bool(iter->second.c_str());
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
	if((iter=parameters.find(Parameters::kSMPublishMasks())) != parameters.end())
	{
		_publishMasks = uStr2Bool(iter->second.c_str());
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
	if((iter=parameters.find(Parameters::kRtabmapRetrievalThr())) != parameters.end())
	{
		_retrievalThr = std::atof(iter->second.c_str());
	}
	if((iter=parameters.find(Parameters::kRtabmapSMStateBufferSize())) != parameters.end())
	{
		_sensorsBufferMaxSize = std::atoi(iter->second.c_str());
	}
	if((iter=parameters.find(Parameters::kRtabmapWorkingDirectory())) != parameters.end())
	{
		this->setWorkingDirectory(iter->second.c_str());
	}
	if((iter=parameters.find(Parameters::kRtabmapMaxRetrieved())) != parameters.end())
	{
		_maxRetrieved = std::atoi(iter->second.c_str());
	}
	if((iter=parameters.find(Parameters::kRtabmapSelectionNeighborhoodSummationUsed())) != parameters.end())
	{
		_selectionNeighborhoodSummationUsed = uStr2Bool(iter->second.c_str());
	}
	if((iter=parameters.find(Parameters::kRtabmapSelectionLikelihoodUsed())) != parameters.end())
	{
		_selectionLikelihoodUsed = uStr2Bool(iter->second.c_str());
	}
	if((iter=parameters.find(Parameters::kRtabmapActionsSentRejectHyp())) != parameters.end())
	{
		_actionsSentRejectHyp = uStr2Bool(iter->second.c_str());
	}
	if((iter=parameters.find(Parameters::kRtabmapConfidenceThr())) != parameters.end())
	{
		_confidenceThr = std::atof(iter->second.c_str());
	}
	if((iter=parameters.find(Parameters::kRtabmapLikelihoodStdDevRemoved())) != parameters.end())
	{
		_likelihoodStdDevRemoved = uStr2Bool(iter->second.c_str());
	}
	if((iter=parameters.find(Parameters::kRtabmapLikelihoodNullValuesIgnored())) != parameters.end())
	{
		_likelihoodNullValuesIgnored = uStr2Bool(iter->second.c_str());
	}
	int signatureType = -1;
	if((iter=parameters.find(Parameters::kMemSignatureType())) != parameters.end())
	{
		signatureType = std::atoi(iter->second.c_str());
	}

	// By default, we create our strategies if they are not already created.
	// If they already exists, we check the parameters if a change is requested

	if(!_memory ||
			(signatureType>=0 && ((dynamic_cast<KeypointMemory*>(_memory) && signatureType!=0) || (dynamic_cast<SMMemory*>(_memory) && signatureType!=1))))
	{
		UEventsManager::post(new RtabmapEventInit(RtabmapEventInit::kInitializing));
		UEventsManager::post(new RtabmapEventInit("Creating memory..."));
		if(_memory)
		{
			delete _memory;
			_memory = 0;
		}
		if(signatureType == 1)
		{
			_memory = new SMMemory(parameters);
		}
		else
		{
			_memory = new KeypointMemory(parameters);
		}
		UEventsManager::post(new RtabmapEventInit("Creating memory, done!"));
		_memory->init(DB_TYPE, _wDir + kDefaultDatabaseName, false, parameters);
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
	if(!_vhStrategy ||
	   (vhStrategy!=kVhUndef && ( (vhStrategy == kVhSim && !dynamic_cast<HypVerificatorSim*>(_vhStrategy)) ||
							      (vhStrategy == kVhEpipolar && !dynamic_cast<HypVerificatorEpipolarGeo*>(_vhStrategy)) ) ))
	{
		if(_vhStrategy)
		{
			delete _vhStrategy;
			_vhStrategy = 0;
		}
		switch(vhStrategy)
		{
		case kVhEpipolar:
			_vhStrategy = new HypVerificatorEpipolarGeo(parameters);
			break;
		case kVhSim:
			_vhStrategy = new HypVerificatorSim(parameters);
			break;
		default:
			_vhStrategy = new HypVerificator(parameters);
			break;
		}
	}
	else if(_vhStrategy)
	{
		_vhStrategy->parseParameters(parameters);
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

int Rtabmap::getReactivatedId() const
{
	return _reactivateId;
}

int Rtabmap::getLastSignatureId() const
{
	int id = 0;
	if(_memory && _memory->getLastSignature())
	{
		id = _memory->getLastSignature()->id();
	}
	return id;
}

std::list<int> Rtabmap::getWorkingMem() const
{
	ULOGGER_DEBUG("");
	std::list<int> mem;
	if(_memory)
	{
		mem = std::list<int>(_memory->getWorkingMem().begin(), _memory->getWorkingMem().end());
		mem.remove(-1);// Ignore the virtual signature (if here)
	}
	return mem;
}

std::map<int, int> Rtabmap::getWeights() const
{
	ULOGGER_DEBUG("");
	std::map<int, int> weights;
	if(_memory)
	{
		weights = _memory->getWeights();
		weights.erase(-1);// Ignore the virtual signature (if here)
	}
	return weights;
}

std::set<int> Rtabmap::getStMem() const
{
	ULOGGER_DEBUG("");
	std::set<int> mem;
	if(_memory)
	{
		mem = _memory->getStMem();
	}
	return mem;
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
	_sensorimotorMutex.lock();
	{
		_sensorimotorBuffer.clear();
	}
	_sensorimotorMutex.unlock();
}


void Rtabmap::mainLoopBegin()
{
	if(!_memory || !_vhStrategy || !_bayesFilter)
	{
		ULOGGER_DEBUG("Rtabmap thread started without all strategies defined...");
		//this->killSafely();
	}
}

void Rtabmap::mainLoopKill()
{
	this->clearBufferedSensors();

	// this will post the newData semaphore
	_sensorimotorAdded.release();
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
			_memory->generateGraph(this->getGraphFileName());
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

void Rtabmap::resetMemory(bool dbOverwritten)
{
	if(_memory)
	{
		UEventsManager::post(new RtabmapEventInit(RtabmapEventInit::kInitializing));
		_memory->init(DB_TYPE, _wDir + kDefaultDatabaseName, dbOverwritten);
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
	_reactivateId = 0;
	_lastLcHypothesisValue = 0;
	this->setupLogFiles(dbOverwritten);
}

void Rtabmap::handleEvent(UEvent* event)
{
	if(this->isRunning() && event->getClassName().compare("CameraEvent") == 0)
	{
		std::list<Sensor> sensors;
		CameraEvent * e = (CameraEvent*)event;
		if(e->getCode() == CameraEvent::kCodeFeatures)
		{
			sensors.push_back(Sensor(e->descriptors(), e->keypoints(), e->cameraId()));
			sensors.push_back(Sensor(e->image(), Sensor::kTypeImage, e->cameraId()));
		}
		else if(e->getCode() == CameraEvent::kCodeImage)
		{
			sensors.push_back(Sensor(e->image(), Sensor::kTypeImage, e->cameraId()));
		}
		if(sensors.size())
		{
			this->addSensorimotor(sensors, std::list<Actuator>());
		}
	}
	else if(this->isRunning() && event->getClassName().compare("MicroEvent") == 0)
	{
		std::list<Sensor> sensors;
		MicroEvent * e = (MicroEvent*)event;
		if(e->getCode() == MicroEvent::kTypeFrameFreqSqrdMagn)
		{
			sensors.push_back(Sensor(e->frame(), Sensor::kTypeAudioFreqSqrdMagn, e->microId()));
		}
		else if(e->getCode() == MicroEvent::kTypeFrameFreq)
		{
			sensors.push_back(Sensor(e->frame(), Sensor::kTypeAudioFreq, e->microId()));
		}
		else if(e->getCode() == MicroEvent::kTypeFrame)
		{
			sensors.push_back(Sensor(e->frame(), Sensor::kTypeAudio, e->microId()));
		}
		if(sensors.size())
		{
			this->addSensorimotor(sensors, std::list<Actuator>());
		}
	}
	else if(this->isRunning() && event->getClassName().compare("SensorimotorEvent") == 0)
	{
		SensorimotorEvent * e = (SensorimotorEvent*)event;
		if(e->getCode() == SensorimotorEvent::kTypeData)
		{
			this->addSensorimotor(e->sensors(), e->actuators());
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
				this->setGraphFileName(rtabmapEvent->getStr());
				ULOGGER_DEBUG("CMD_GENERATE_GRAPH");
				pushNewState(kStateGeneratingGraph);
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
	double timeRetrievalDbAccess = 0;
	double timeLikelihoodCalculation = 0;
	double timePosteriorCalculation = 0;
	double timeHypothesesCreation = 0;
	double timeHypothesesValidation = 0;
	double timeActionSelection = 0;
	double timeRealTimeLimitReachedProcess = 0;
	double timeMemoryCleanup = 0;
	double timeEmptyingTrash = 0;
	double timeJoiningTrash = 0;
	double timeStatsCreation = 0;
	std::map<std::string, float> memUpdateStats;

	int refId = Memory::kIdInvalid;
	float hypothesisRatio = 0.0f; // Only used for statistics
	bool rejectedHypothesis = false;

	std::map<int, float> likelihood;
	std::map<int, float> adjustedLikelihood;
	std::map<int, float> posterior;
	std::pair<int, float> hypothesis;
	std::list<std::pair<int, float> > reactivateHypotheses;

	std::map<int, int> childCount;
	std::set<int> signaturesRetrieved;

	const Signature * signature = 0;
	const Signature * sLoop = 0;
	std::list<Sensor> sensors;
	std::list<Actuator> actuators;

	_lcHypothesisId = 0;
	_actuators.clear();
	int neighborSelected = _reactivateId;
	int actionsChosen = 0; // for stats

	std::set<int> immunizedLocations;

	//============================================================
	// Wait for an image...
	//============================================================
	ULOGGER_INFO("getting data...");
	this->getSensorimotor(sensors, actuators);
	if(!sensors.size())
	{
		ULOGGER_INFO("sensors list is null...");
		return;
	}

	timer.start();
	timerTotal.start();

	if(!_memory || !_vhStrategy || !_bayesFilter)
	{
		UFATAL("RTAB-Map is not initialized, data received is ignored.");
	}

	//============================================================
	// Memory Update : Location creation + Rehearsal
	//============================================================
	ULOGGER_INFO("Updating memory...");
	if(!_memory->update(sensors, actuators, memUpdateStats))
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
		if(_memory->getWorkingMemSize())
		{
			//============================================================
			// Likelihood computation
			// Get the likelihood of the new signature
			// with all images contained in the working memory + reactivated.
			//============================================================
			ULOGGER_INFO("computing likelihood...");
			float maximumScore;
			const std::set<int> & wm = _memory->getWorkingMem();
			std::list<int> signaturesToCompare(wm.begin(), wm.end());
			likelihood = _memory->computeLikelihood(signature, signaturesToCompare, maximumScore);

			// Adjust the likelihood
			adjustedLikelihood = likelihood;
			this->adjustLikelihood(adjustedLikelihood);

			timeLikelihoodCalculation = timer.ticks();
			ULOGGER_INFO("timeLikelihoodCalculation=%fs",timeLikelihoodCalculation);

			//============================================================
			// Apply the Bayes filter
			//  Posterior = Likelihood x Prior
			//============================================================
			ULOGGER_INFO("getting posterior...");

			// Compute the posterior
			posterior = _bayesFilter->computePosterior(_memory, adjustedLikelihood);
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
				if(posterior.size())
				{
					hypothesis = this->selectHypothesis(posterior,
							adjustedLikelihood,
							_selectionNeighborhoodSummationUsed,
							_selectionLikelihoodUsed);
					// When using a virtual place, use sum of LC probabilities (1 - virtual place hypothesis).
					if(posterior.begin()->first == Memory::kIdVirtual)
					{
						hypothesis.second = 1-posterior.begin()->second;
					}
				}
			}
			timeHypothesesCreation = timer.ticks();
			ULOGGER_INFO("Hypothesis=%d, value=%f, timeHypothesesCreation=%fs", hypothesis.first, hypothesis.second, timeHypothesesCreation);

			if(hypothesis.first > 0)
			{
				// Loop closure Threshold
				// When _loopThr=0, accept loop closure if the hypothesis is over
				// the virtual (new) place hypothesis.
				if((_loopThr > 0 && hypothesis.second >= _loopThr) ||
				   (_loopThr == 0 && posterior.begin()->first == Memory::kIdVirtual && hypothesis.second > posterior.begin()->second))
				{
					//============================================================
					// Hypothesis verification for loop closure with geometric
					// information (like the epipolar geometry or using the local map
					// associated with the signature)
					//============================================================
					if(hypothesis.second >= _loopRatio*_lastLcHypothesisValue &&
					   _lastLcHypothesisValue &&
					   _vhStrategy->verify(signature, _memory->getSignature(hypothesis.first)))
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
				_reactivateId = hypothesis.first;

				//for statistic...
				hypothesisRatio = _lastLcHypothesisValue>0?hypothesis.second/_lastLcHypothesisValue:0;
				_lastLcHypothesisValue = hypothesis.second;

				//=============================================================
				// Update loop closure links
				//=============================================================
				// Make the new one the parent of the old one
				if(_lcHypothesisId>0)
				{
					_memory->addLoopClosureLink(_lcHypothesisId, signature->id());
				}
			}
		} // if(_memory->getWorkingMemSize())

		//============================================================
		// Select next actions
		//============================================================
		timer.start();
		if(hypothesis.first > 0)
		{
			sLoop = _memory->getSignature(hypothesis.first);
		}

		neighborSelected = _reactivateId;
		// only send actions if rejectLoopReason!=3 (decreasing hypotheses)
		if(sLoop && (_actionsSentRejectHyp || !rejectedHypothesis) && (_lastLcHypothesisValue > _confidenceThr))
		{
			// TODO to verify
			std::list<NeighborLink> neighbors = _memory->getNeighborLinks(sLoop->id(), false, false, true);
			float currentMaxSim = -1;
			UINFO("Actions: neighbors.size=%d", neighbors.size());
			for(std::list<NeighborLink>::const_reverse_iterator iter=neighbors.rbegin(); iter!=neighbors.rend() && currentMaxSim!=1.0f; ++iter)
			{
				float sim = _memory->compareOneToOne(iter->baseIds(), _memory->getLastBaseIds());
				UDEBUG("Neighbor baseIds comparison with %d = %f", iter->toId(), sim);
				if(sim > currentMaxSim)
				{
					currentMaxSim = sim;
					if(iter->actuators().size())
					{
						_actuators = iter->actuators();
					}
					neighborSelected = iter->toId();
				}
				++actionsChosen;
			}
			_reactivateId = neighborSelected;
		}
		UDEBUG("Action taken from neighbor %d", neighborSelected);
		timeActionSelection = timer.ticks();
		ULOGGER_INFO("timeActionSelection=%fs",timeActionSelection);
	}// !isBadSignature

	//============================================================
	// Before retrieval, make sure the trash has finished
	//============================================================
	_memory->joinTrashThread();
	timeEmptyingTrash = _memory->getDbSavingTime();
	timeJoiningTrash = timer.ticks();
	ULOGGER_INFO("Time emptying memory trash = %fs,  joining (actual overhead) = %fs", timeEmptyingTrash, timeJoiningTrash);

	//============================================================
	// RETRIEVAL : Loop closure neighbors reactivation
	//============================================================
	if(_reactivateId > 0 )
	{
		//Load neighbors
		ULOGGER_INFO("Retrieving locations... around id=%d", _reactivateId);
		unsigned int neighborhoodSize = _bayesFilter->getPredictionLC().size()-1;
		unsigned int margin = neighborhoodSize;//_memory->getStMemSize();
		UTimer timeGetN;
		unsigned int nbLoadedFromDb = 0;
		std::set<int> reactivatedIdsSet;
		std::list<int> reactivatedIds;
		double timeGetNeighborsTimeDb = 0.0;
		double timeGetNeighborsSpaceDb = 0.0;

		// Direct neighbors TIME
		std::map<int, int> neighbors = _memory->getNeighborsId(timeGetNeighborsTimeDb, _reactivateId, margin, _maxRetrieved, _bayesFilter->isPredictionOnNonNullActionsOnly(), true, true, true);
		unsigned int m = 0;
		//Priority to locations near in time (direct neighbor) then by space (loop closure)
		while(m < margin)
		{
			std::set<int> idsSorted;
			for(std::map<int, int>::iterator iter=neighbors.begin(); iter!=neighbors.end();)
			{
				if((unsigned int)iter->second == m)
				{
					if(reactivatedIdsSet.find(iter->first) == reactivatedIdsSet.end())
					{
						ULOGGER_INFO("nt=%d m=%d", iter->first, iter->second);
						idsSorted.insert(iter->first);
						reactivatedIdsSet.insert(iter->first);

						if(m<neighborhoodSize)
						{
							//immunized locations in the neighborhood from being transferred
							immunizedLocations.insert(iter->first);
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
			reactivatedIds.insert(reactivatedIds.end(), idsSorted.rbegin(), idsSorted.rend());
			++m;
		}
		// neighbors SPACE, already added direct neighbors will be ignored
		neighbors = _memory->getNeighborsId(timeGetNeighborsSpaceDb, _reactivateId, margin, _maxRetrieved, _bayesFilter->isPredictionOnNonNullActionsOnly(), true, true, false);
		m = 0;
		while(m < margin)
		{
			std::set<int> idsSorted;
			for(std::map<int, int>::iterator iter=neighbors.begin(); iter!=neighbors.end();)
			{
				if((unsigned int)iter->second == m)
				{
					if(reactivatedIdsSet.find(iter->first) == reactivatedIdsSet.end())
					{
						ULOGGER_INFO("ns=%d m=%d", iter->first, iter->second);
						idsSorted.insert(iter->first);
						reactivatedIdsSet.insert(iter->first);
					}
					std::map<int, int>::iterator tmp = iter++;
					neighbors.erase(tmp);
				}
				else
				{
					++iter;
				}
			}
			reactivatedIds.insert(reactivatedIds.end(), idsSorted.rbegin(), idsSorted.rend());
			++m;
		}
		ULOGGER_INFO("margin=%d, neighborhoodSize=%d, reactivatedIds.size=%d, nbLoadedFromDb=%d time=%fs", margin, neighborhoodSize, reactivatedIds.size(), (int)nbLoadedFromDb, timeGetN.ticks());

		// Max 2 signatures retrieved
		signaturesRetrieved = _memory->reactivateSignatures(reactivatedIds, _maxRetrieved, timeRetrievalDbAccess);
		timeRetrievalDbAccess += timeGetNeighborsTimeDb + timeGetNeighborsSpaceDb;
		UINFO("timeRetrievalDbAccess=%fs", timeRetrievalDbAccess);
		ULOGGER_INFO("retrieval of %d reactivatedIds=%fs", (int)signaturesRetrieved.size(), timeGetN.ticks());

		// Immunize just retrieved signatures
		immunizedLocations.insert(signaturesRetrieved.begin(), signaturesRetrieved.end());
	}
	timeReactivations = timer.ticks();
	ULOGGER_INFO("timeReactivations=%fs", timeReactivations);


	//============================================================
	// Prepare statistics
	//============================================================
	// Data used for the statistics event and for the log files
	int processMemoryUsed = UProcessInfo::getMemoryUsage()/(1024*1024); // MB
	int databaseMemoryUsed = _memory->getDatabaseMemoryUsed(); // MB
	int dictionarySize = 0;
	int refWordsCount = 0;
	int refUniqueWordsCount = 0;
	const KeypointSignature * ssRef = 0;
	const KeypointSignature * ssLoop = 0;
	int lcHypothesisReactivated = 0;
	float rehearsalValue = uValue(memUpdateStats, std::string("Memory/Rehearsal Max Value/"), 0.0f);
	KeypointMemory * kpMem = dynamic_cast<KeypointMemory *>(_memory);
	if(sLoop)
	{
		lcHypothesisReactivated = sLoop->isSaved()?1.0f:0.0f;
	}
	if(kpMem)
	{
		if(sLoop)
		{
			ssLoop = dynamic_cast<const KeypointSignature *>(sLoop);
		}
		ssRef = dynamic_cast<const KeypointSignature *>(signature);
		dictionarySize = kpMem->getVWD()->getVisualWords().size();
		if(ssRef)
		{
			refWordsCount = ssRef->getWords().size();
			refUniqueWordsCount = uUniqueKeys(ssRef->getWords()).size();
		}
		else
		{
			ULOGGER_WARN("The new signature can't be casted to a KeypointSignature while the Memory is this type ?");
		}
	}

	float vpLikelihood = 0.0f;
	if(adjustedLikelihood.size() && adjustedLikelihood.begin()->first == -1)
	{
		vpLikelihood = adjustedLikelihood.begin()->second;
	}
	float vpHypothesis = 0.0f;
	if(posterior.find(Memory::kIdVirtual) != posterior.end())
	{
		vpHypothesis = posterior.at(Memory::kIdVirtual);
	}

	// only prepare statistics if required or when there is a loop closure
	Statistics * stat = 0;
	if(_lcHypothesisId || _actuators.size() || _publishStats)
	{
		ULOGGER_INFO("sending stats...");
		stat = new Statistics();
		stat->setRefImageId(refId);
		if(_lcHypothesisId != Memory::kIdInvalid)
		{
			stat->setLoopClosureId(_lcHypothesisId);
			ULOGGER_INFO("Loop closure detected! With id=%d", _lcHypothesisId);
		}
		if(_actuators.size())
		{
			stat->setActuators(_actuators);
		}
		if(_publishStats && refId != Memory::kIdInvalid)
		{
			ULOGGER_INFO("send all stats...");
			stat->setExtended(1);

			stat->addStatistic(Statistics::kParent_id(), signature->getLoopClosureIds().size()?*signature->getLoopClosureIds().rbegin():0);

			stat->addStatistic(Statistics::kLoopHighest_hypothesis_id(), hypothesis.first);
			stat->addStatistic(Statistics::kLoopHighest_hypothesis_value(), hypothesis.second);
			stat->addStatistic(Statistics::kHypothesis_reactivated(), lcHypothesisReactivated);
			stat->addStatistic(Statistics::kLoopVp_likelihood(), vpLikelihood);
			stat->addStatistic(Statistics::kLoopVp_hypothesis(), vpHypothesis);
			stat->addStatistic(Statistics::kLoopReactivateId(), _reactivateId);
			stat->addStatistic(Statistics::kLoopHypothesis_ratio(), hypothesisRatio);
			stat->addStatistic(Statistics::kLoopActions(), (int)_actuators.size());
			stat->addStatistic(Statistics::kLoopActions_of(), neighborSelected);
			stat->addStatistic(Statistics::kLoopActions_chosen(), actionsChosen);

			stat->addStatistic(Statistics::kMemoryWorking_memory_size(), _memory->getWorkingMemSize());
			stat->addStatistic(Statistics::kMemoryShort_time_memory_size(), _memory->getStMemSize());
			stat->addStatistic(Statistics::kMemoryDatabase_size(), (float)databaseMemoryUsed);
			stat->addStatistic(Statistics::kMemoryProcess_memory_used(), (float)processMemoryUsed);
			stat->addStatistic(Statistics::kMemorySignatures_retrieved(), (float)signaturesRetrieved.size());
			stat->addStatistic(Statistics::kMemoryImages_buffered(), (float)_sensorimotorBuffer.size());
			stat->addStatistic(Statistics::kMemorySimilarities_map(), (float)_memory->getSimilaritiesMap().size());

			// timing...
			stat->addStatistic(Statistics::kTimingMemory_update(), timeMemoryUpdate*1000);
			stat->addStatistic(Statistics::kTimingReactivation(), timeReactivations*1000);
			stat->addStatistic(Statistics::kTimingLikelihood_computation(), timeLikelihoodCalculation*1000);
			stat->addStatistic(Statistics::kTimingPosterior_computation(), timePosteriorCalculation*1000);
			stat->addStatistic(Statistics::kTimingHypotheses_creation(), timeHypothesesCreation*1000);
			stat->addStatistic(Statistics::kTimingHypotheses_validation(), timeHypothesesValidation*1000);
			stat->addStatistic(Statistics::kTimingAction_selection(), timeActionSelection*1000);
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

			if(_publishRawData)
			{
				stat->setRefRawData(sensors); // raw data
				if(sLoop)
				{
					lcHypothesisReactivated = 0;
					if(sLoop->isSaved())
					{
						lcHypothesisReactivated = 1;
					}

					const std::list<Sensor> & data = sLoop->getRawData();
					if(data.empty() && _memory->isRawDataKept())
					{
						std::list<Sensor> d = _memory->getRawData(sLoop->id());
						stat->setLoopClosureRawData(d);
					}
					else if(!data.empty())
					{
						stat->setLoopClosureRawData(data);
					}
				}
			}
			if(_publishLikelihood || _publishPdf)
			{
				// Child count by parent signature on the root of the memory ... for statistics
				stat->setWeights(_memory->getWeights());
				if(_publishPdf)
				{
					stat->setPosterior(posterior);
				}
				if(_publishLikelihood)
				{
					stat->setLikelihood(adjustedLikelihood);
				}
			}
			if(_publishKeypoints)
			{
				//Copy keypoints
				if(ssRef)
				{
					stat->setRefWords(ssRef->getWords());
				}
				if(ssLoop)
				{
					//Copy keypoints
					stat->setLoopWords(ssLoop->getWords());
				}
			}
			if(_publishMasks)
			{
				// Copy mask
				UWARN("Publish motion masks TODO");
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
	// If time allowed for the detection exceed the limit of
	// real-time, move the oldest signature with less frequency
	// entry (from X oldest) from the short term memory to the
	// long term memory.
	//============================================================
	double totalTime = timerTotal.ticks();
	ULOGGER_INFO("Total time processing = %fs...", totalTime);
	timer.start();
	if((_maxTimeAllowed != 0 && totalTime*1000>_maxTimeAllowed) ||
		(_maxMemoryAllowed != 0 && _memory->getWorkingMemSize() > _maxMemoryAllowed))
	{
		ULOGGER_INFO("Removing old signatures because time limit is reached %f>%f or memory is reached %d>%d...", totalTime*1000, _maxTimeAllowed, _memory->getWorkingMemSize(), _maxMemoryAllowed);
		signaturesRemoved = _memory->forget(immunizedLocations);
	}

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
	if(_foutFloat)
	{

		fprintf(_foutFloat, "%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n",
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
				vpLikelihood,
				maxLikelihood,
				sumLikelihoods,
				mean,
				stddev,
				vpHypothesis,
				timeJoiningTrash,
				rehearsalValue,
				timeEmptyingTrash,
				timeRetrievalDbAccess);
	}

	if(_foutInt)
	{
		fprintf(_foutInt, "%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d\n",
				_lcHypothesisId,
				hypothesis.first,
				signaturesRemoved,
				0,
				refWordsCount,
				dictionarySize,
				int(_memory->getWorkingMemSize()),
				rejectedHypothesis?1:0,
				processMemoryUsed,
				databaseMemoryUsed,
				int(signaturesRetrieved.size()),
				lcHypothesisReactivated,
				refUniqueWordsCount,
				_reactivateId,
				int(nonNulls.size()));
	}
	UINFO("Time logging = %f...", timer.ticks());
	//ULogger::flush();

}

void Rtabmap::addSensorimotor(const std::list<Sensor> & sensors, const std::list<Actuator> & actuators)
{
	UDEBUG("sensors %d, actuators %d", sensors.size(), actuators.size());
	if(!sensors.size() && !actuators.size())
	{
		ULOGGER_ERROR("Sensors and actuators empty !?");
		return;
	}

	bool notify = true;
	_sensorimotorMutex.lock();
	{
		_sensorimotorBuffer.push_back(std::make_pair(sensors, actuators));
		while(_sensorsBufferMaxSize > 0 && _sensorimotorBuffer.size() >= (unsigned int)_sensorsBufferMaxSize)
		{
			ULOGGER_WARN("Data buffer is full, the oldest data is removed to add the new one.");
			_sensorimotorBuffer.pop_front();
			notify = false;
		}
	}
	_sensorimotorMutex.unlock();

	if(notify)
	{
		_sensorimotorAdded.release();
	}
}

void Rtabmap::getSensorimotor(std::list<Sensor> & sensors, std::list<Actuator> & actuators)
{
	ULOGGER_DEBUG("");
	sensors.clear();
	actuators.clear();

	ULOGGER_INFO("waiting for data");
	_sensorimotorAdded.acquire();
	ULOGGER_INFO("wake-up");

	_sensorimotorMutex.lock();
	{
		if(!_sensorimotorBuffer.empty())
		{
			sensors = _sensorimotorBuffer.front().first;
			actuators = _sensorimotorBuffer.front().second;
			_sensorimotorBuffer.pop_front();
		}
	}
	_sensorimotorMutex.unlock();
}

// SETTERS
void Rtabmap::setMaxTimeAllowed(float maxTimeAllowed)
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
		_sensorsBufferMaxSize = 0;
	}
	else
	{
		_sensorsBufferMaxSize = size;
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
				_memory->init(DB_TYPE, _wDir + kDefaultDatabaseName);
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

void Rtabmap::process(const std::list<Sensor> & data)
{
	if(!this->isRunning())
	{
		this->addSensorimotor(data, std::list<Actuator>());
		this->process();
	}
	else
	{
		UERROR("The core thread is running!");
	}
}

void Rtabmap::process(const Sensor & data)
{
	if(!this->isRunning())
	{
		std::list<Sensor> sensors;
		sensors.push_back(data);
		this->addSensorimotor(sensors, std::list<Actuator>());
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
	ULOGGER_DEBUG("");
	UTimer timer;
	timer.start();
	if(likelihood.size()==0)
	{
		return;
	}

	// remove min
	std::vector<float> allValues = uValues(likelihood);
	float min = 0.0f;
	/*float min = -1;
	if(allValues.size())
	{
		std::vector<float>::iterator iter=allValues.begin();
		if(likelihood.begin()->first == -1)
		{
			++iter;
		}
		for(; iter!=allValues.end(); ++iter)
		{
			if((min == -1 || min>*iter))
			{
				min = *iter;
			}
		}
		iter=allValues.begin();
		if(likelihood.begin()->first == -1)
		{
			++iter;
		}
		for(; iter!=allValues.end() && min != -1; ++iter)
		{
			*iter -= min;
		}
	}*/

	//for(unsigned int i=0; i<allValues.size(); ++i)
	//{
		//UDEBUG("allValues[%d]=%f", (int)i, allValues[i]);
	//}

	// Use only non-null values
	std::list<float> values;
	for(unsigned int i=0; i<allValues.size(); ++i)
	{
		if(!_likelihoodNullValuesIgnored || allValues[i])
		{
			values.push_back(allValues[i]);
		}
	}

	if(likelihood.begin()->first == -1 && likelihood.begin()->second != 0)
	{
		likelihood.begin()->second = 0;
		values.pop_front();
	}



	//Compute mean
	float mean = uMean(values);

	//Compute std dev
	float stdDev = uStdDev(values, mean);

	//Adjust likelihood with mean and standard deviation (see Angeli phd)
	float epsilon = 0.0001;
	float max = 0.0f;
	int maxId = 0;
	for(std::map<int, float>::iterator iter=likelihood.begin(); iter!= likelihood.end(); ++iter)
	{
		float value = iter->second - min;
		if(value > mean+(!_likelihoodStdDevRemoved?0:stdDev) && mean)
		{
			if(_likelihoodStdDevRemoved)
			{
				iter->second = (value-(stdDev-epsilon))/mean;
			}
			else
			{
				iter->second = value/mean;
			}
			if(value > max)
			{
				max = value;
				maxId = iter->first;
			}
		}
		else
		{
			iter->second = 1;
		}
	}

	if(likelihood.begin()->first == -1)
	{
		if(stdDev > epsilon && max)
		{
			likelihood.begin()->second = mean/stdDev + 1;
		}
		else
		{
			likelihood.begin()->second = 100; // infinity
			ULOGGER_DEBUG("Set vp likelihood to 10, time=%fs", timer.ticks());
		}
		if(likelihood.begin()->second<1.0f)
		{
			likelihood.begin()->second = 1.0f;
		}
	}

	ULOGGER_DEBUG("mean=%f, stdDev=%f, max=%f, maxId=%d, time=%fs", mean, stdDev, max, maxId, timer.ticks());
}

std::pair<int, float> Rtabmap::selectHypothesis(const std::map<int, float> & posterior,
							const std::map<int, float> & likelihood,
							bool neighborSumUsed,
							bool likelihoodUsed) const
{
	ULOGGER_DEBUG("");
	if(!_bayesFilter || !_memory)
	{
		ULOGGER_ERROR("RTAB-Map must be initialized first!");
		return std::pair<int, float>(0, 0.0f);
	}
	if(posterior.size() == 0)
	{
		ULOGGER_ERROR("Posterior parameter size = 0?");
		return std::pair<int, float>(0, 0.0f);
	}

	int id = 0;
	float value;
	float totalLoopClosure = 0.0f;
	int hypothesisId = 0;
	float hypothesisValue = 0.0f;
	UTimer timer;
	timer.start();
	UTimer timerGlobal;
	timerGlobal.start();
	for(std::map<int, float>::const_reverse_iterator iter = posterior.rbegin(); iter != posterior.rend(); ++iter)
	{
		value = (*iter).second;
		id = (*iter).first;
		if(id > 0)
		{
			totalLoopClosure += value;

			if(value > hypothesisValue)
			{
				hypothesisId = id;
				hypothesisValue = (*iter).second;
			}
		}
	}

	if((likelihoodUsed || neighborSumUsed) && hypothesisId > 0)
	{
		//select highest likelihood from the neighborhood of the highest hypothesis
		double dbAccessTime = 0.0;
		std::map<int, int> neighbors = _memory->getNeighborsId(dbAccessTime, hypothesisId, _bayesFilter->getPredictionLC().size()-1, 0);
		float oldlikelihood = likelihood.at(hypothesisId);
		int oldId = hypothesisId;
		float highestLikelihood = oldlikelihood;
		bool modified = false;
		std::set<int> loopIds, childIds;

		for(std::map<int, int>::iterator jter=neighbors.begin(); jter!=neighbors.end(); ++jter)
		{
			if(likelihoodUsed)
			{
				std::map<int, float>::const_iterator sim = likelihood.find(jter->first);
				if(sim != likelihood.end())
				{
					if(sim->second > highestLikelihood)
					{
						UDEBUG("sim->first=%d, sim->second=%f", sim->first, sim->second);
						highestLikelihood = sim->second;
						hypothesisId = sim->first;
						modified = true;
					}
				}
			}
			if(neighborSumUsed)
			{
				if(jter->first != oldId)
				{
					std::map<int, float>::const_iterator tmpIter = posterior.find(jter->first);
					if(tmpIter!=posterior.end())
					{
						hypothesisValue += tmpIter->second;
					}
				}
			}
		}
		if(modified)
		{
			UDEBUG("Lc hypothesis (%d) changed to neighbor (%d) with higher likelihood (%f->%f)", oldId, hypothesisId, oldlikelihood, highestLikelihood);
		}
	}


	ULOGGER_DEBUG("time=%fs", timerGlobal.ticks());
	return std::make_pair(hypothesisId, hypothesisValue);
}

void Rtabmap::dumpPrediction() const
{
	if(_memory && _bayesFilter)
	{
		const std::set<int> & wm = _memory->getWorkingMem();
		cv::Mat prediction(wm.size(), wm.size(), CV_32FC1);
		_bayesFilter->generatePrediction(prediction, _memory, std::vector<int>(wm.begin(), wm.end()));

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
