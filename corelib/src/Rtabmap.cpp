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
#include "rtabmap/core/CameraEvent.h"
#include "rtabmap/core/Version.h"
#include "rtabmap/core/SMState.h"
#include "rtabmap/core/KeypointDetector.h"

#include "rtabmap/core/Camera.h"
#include "VWDictionary.h"
#include "Signature.h"

#include "VerifyHypotheses.h"

#include "KeypointMemory.h"
#include "BayesFilter.h"

#include "utilite/UtiLite.h"

#include "SimpleIni.h"

#include <stdlib.h>
#include <set>

#define DB_TYPE "sqlite3"
#define DB_NAME "LTM.db"

#define LOG_F "LogF.txt"
#define LOG_I "LogI.txt"

#define GRAPH_FILE_NAME "Graph.dot"

namespace rtabmap
{
const char * Rtabmap::kDefaultIniFileName = "rtabmap.ini";

Rtabmap::Rtabmap() :
	_publishStats(Parameters::defaultRtabmapPublishStats()),
	_reactivationDisabled(Parameters::defaultRtabmapDisableReactivation()),
	_maxTimeAllowed(Parameters::defaultRtabmapTimeThr()), // 1 sec
	_smStateBufferMaxSize(Parameters::defaultRtabmapSMStateBufferSize()),
	_minMemorySizeForLoopDetection(Parameters::defaultRtabmapMinMemorySizeForLoopDetection()),
	_loopThr(Parameters::defaultRtabmapLoopThr()),
	_loopRatio(Parameters::defaultRtabmapLoopRatio()),
	_remThr(Parameters::defaultRtabmapReactivationThr()),
	_localGraphCleaned(Parameters::defaultRtabmapLocalGraphCleaned()),
	_maxRetrieved(Parameters::defaultRtabmapMaxRetrieved()),
	_actionsByTime(Parameters::defaultRtabmapActionsByTime()),
	_actionsSentRejectHyp(Parameters::defaultRtabmapActionsSentRejectHyp()),
	_lcHypothesisId(0),
	_reactivateId(0),
	_highestHypothesisValue(0),
	_spreadMargin(0),
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

	UEventsManager::addHandler(this);
}

Rtabmap::~Rtabmap() {
	ULOGGER_DEBUG("");

	UEventsManager::removeHandler(this);

	// Stop the thread first
	this->kill();

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

void Rtabmap::startInit()
{
	if(!_memory || !_vhStrategy || !_bayesFilter)
	{
		ULOGGER_DEBUG("Rtabmap thread started without all strategies defined...");
		//this->killSafely();
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

	_newSMStateSem.release();
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
	if((iter=parameters.find(Parameters::kRtabmapDisableReactivation())) != parameters.end())
	{
		_reactivationDisabled = uStr2Bool(iter->second.c_str());
	}
	if((iter=parameters.find(Parameters::kRtabmapTimeThr())) != parameters.end())
	{
		_maxTimeAllowed = std::atof(iter->second.c_str());
	}
	if((iter=parameters.find(Parameters::kRtabmapLoopThr())) != parameters.end())
	{
		_loopThr = std::atof(iter->second.c_str());
	}
	if((iter=parameters.find(Parameters::kRtabmapLoopRatio())) != parameters.end())
	{
		_loopRatio = std::atof(iter->second.c_str());
	}
	if((iter=parameters.find(Parameters::kRtabmapReactivationThr())) != parameters.end())
	{
		_remThr = std::atof(iter->second.c_str());
	}
	if((iter=parameters.find(Parameters::kRtabmapSMStateBufferSize())) != parameters.end())
	{
		_smStateBufferMaxSize = std::atoi(iter->second.c_str());
	}
	if((iter=parameters.find(Parameters::kRtabmapMinMemorySizeForLoopDetection())) != parameters.end())
	{
		_minMemorySizeForLoopDetection = std::atoi(iter->second.c_str());
	}
	if((iter=parameters.find(Parameters::kRtabmapWorkingDirectory())) != parameters.end())
	{
		this->setWorkingDirectory(iter->second.c_str());
	}
	if((iter=parameters.find(Parameters::kRtabmapLocalGraphCleaned())) != parameters.end())
	{
		_localGraphCleaned = uStr2Bool(iter->second.c_str());
	}
	if((iter=parameters.find(Parameters::kRtabmapMaxRetrieved())) != parameters.end())
	{
		_maxRetrieved = std::atoi(iter->second.c_str());
	}
	if((iter=parameters.find(Parameters::kRtabmapActionsByTime())) != parameters.end())
	{
		_actionsByTime = uStr2Bool(iter->second.c_str());
	}
	if((iter=parameters.find(Parameters::kRtabmapActionsSentRejectHyp())) != parameters.end())
	{
		_actionsSentRejectHyp = uStr2Bool(iter->second.c_str());
	}

	// By default, we create our strategies if they are not already created.
	// If they already exists, we check the parameters if a change is requested

	if(!_memory)
	{
		UEventsManager::post(new RtabmapEventInit(RtabmapEventInit::kInitializing));
		UEventsManager::post(new RtabmapEventInit("Creating memory..."));
		_memory = new KeypointMemory(parameters);
		UEventsManager::post(new RtabmapEventInit("Creating memory, done!"));
		_memory->init(DB_TYPE, _wDir + DB_NAME, false, parameters);
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
		mem = uKeysList(_memory->getWorkingMem());
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

void Rtabmap::killCleanup()
{
	_smStateBufferMutex.lock();
	{
		for(std::list<SMState *>::iterator i=_smStateBuffer.begin(); i!=_smStateBuffer.end(); ++i)
		{
			delete(*i);
		}
		_smStateBuffer.clear();
	}
	_smStateBufferMutex.unlock();

	//this->addImage(0); // this will post the newImage semaphore
	_newSMStateSem.release();
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
		if(_memory && _vhStrategy && _bayesFilter)
		{
			this->process();
		}
		else // kIdle
		{
			ULOGGER_DEBUG("RTAB-Map is not initialized... please call init() or post a STATE_CHANGING_STRATEGY RtabmapEvent...Sleeping 1 sec...");
			uSleep(1000);
		}
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
		this->deleteMemory();
		break;
	default:
		uSleep(100);
		ULOGGER_ERROR("not supposed to be here...");
		break;
	}
}

void Rtabmap::resetMemory()
{
	if(_memory)
	{
		if(_memory)
		{
			UEventsManager::post(new RtabmapEventInit(RtabmapEventInit::kInitializing));
			_memory->init(DB_TYPE, _wDir + DB_NAME);
			UEventsManager::post(new RtabmapEventInit(RtabmapEventInit::kInitialized));
		}
		if(_bayesFilter)
		{
			_bayesFilter->reset();
		}
	}
	_reactivateId = 0;
	_highestHypothesisValue = 0;
	_spreadMargin = 0;
	this->setupLogFiles();
}

void Rtabmap::deleteMemory()
{
	if(_memory)
	{
		UEventsManager::post(new RtabmapEventInit(RtabmapEventInit::kInitializing));
		_memory->init(DB_TYPE, _wDir + DB_NAME, true);
		UEventsManager::post(new RtabmapEventInit(RtabmapEventInit::kInitialized));
	}
	else
	{
		UFile::erase(_wDir + DB_NAME);
	}
	if(_bayesFilter)
	{
		_bayesFilter->reset();
	}
	_reactivateId = 0;
	_highestHypothesisValue = 0;
	_spreadMargin = 0;
	this->setupLogFiles(true);
}

void Rtabmap::handleEvent(UEvent* event)
{
	if(this->isRunning() && event->getClassName().compare("SMStateEvent") == 0)
	{
		SMStateEvent * e = (SMStateEvent*)event;
		SMState * data = e->getSMStateOwnership();
		this->addSMState(data);
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
	}
	else if(event->getClassName().compare("ParamEvent") == 0)
	{
		ULOGGER_DEBUG("changing parameters");
		pushNewState(kStateChangingParameters, ((ParamEvent*)event)->getParameters());
	}
}

void Rtabmap::process()
{
	ULOGGER_DEBUG("");

	//============================================================
	// Initialization
	//============================================================
	UTimer timer;
	UTimer timerTotal;
	double timeMemoryUpdate = 0;
	double timeCleaningNeighbors = 0;
	double timeReactivations = 0;
	double timeLikelihoodCalculation = 0;
	double timePosteriorCalculation = 0;
	double timeHypothesesCreation = 0;
	double timeHypothesesValidation = 0;
	double timeRealTimeLimitReachedProcess = 0;
	double timeMemoryCleanup = 0;
	double timeEmptyingMemoryTrash = 0;
	double timeStatsCreation = 0;
	std::map<std::string, float> memUpdateStats;

	int refId = Memory::kIdInvalid;
	float hypothesisRatio = 0.0f; // Only used for statistics
	bool rejectedHypothesis = false;

	std::map<int, float> likelihood;
	std::map<int, float> adjustedLikelihood;
	std::map<int, float> posterior;
	std::list<std::pair<int, float> > hypotheses;
	std::list<std::pair<int, float> > reactivateHypotheses;

	std::map<int, int> childCount;
	unsigned int signaturesReactivated = 0;

	const Signature * signature = 0;
	SMState * smState = 0;

	_lcHypothesisId = 0;
	_actions.clear();

	std::list<int> reactivatedIds;
	int directNeighborsNotReactivated = 0;

	// TODO Do a pre-update (build dicitonary), Reactivation parallelized with SURF, addNewWords

	//============================================================
	// Wait for an image...
	//============================================================
	ULOGGER_INFO("getting data...");
	smState =	this->getSMState();
	if(!smState)
	{
		ULOGGER_INFO("data is null...");
		return;
	}
	else if(!_state.empty())
	{
		ULOGGER_INFO("State changed while waiting.. aborting processing...");
		delete smState;
		return;
	}

	timer.start();
	timerTotal.start();

	//============================================================
	// Memory Update
	//============================================================
	ULOGGER_INFO("Updating memory...");
	if(!_memory->update(smState, memUpdateStats))
	{
		ULOGGER_ERROR("Not supposed to be here...");
		delete smState;
		return;
	}
	signature = _memory->getLastSignature();
	if(!signature)
	{
		ULOGGER_ERROR("Not supposed to be here...");
		delete smState;
		return;
	}
	ULOGGER_INFO("Processing signature %d", signature->id());
	refId = signature->id();
	timeMemoryUpdate = timer.ticks();
	ULOGGER_INFO("timeMemoryUpdate=%f", timeMemoryUpdate);

	//if(!signature->isBadSignature())
	{
		// Before the next step, make sure the trash has finished
		_memory->joinTrashThread();
		timeEmptyingMemoryTrash = timer.ticks();
		ULOGGER_INFO("Time emptying memory trash = %f...", timeEmptyingMemoryTrash);


		//============================================================
		// Loop closure neighbors reactivation
		// If a loop closure occurred the last frame, activate neighbors
		// for the likelihood
		//============================================================
		if(!_reactivationDisabled && _reactivateId > 0 )
		{
			//Load neighbors
			ULOGGER_INFO("Reactivating signatures... around id=%d", _reactivateId);
			std::list<int> signaturesToReactivate;
			//the bayesFilter is supposed to be created here...
			unsigned int margin = _bayesFilter->getPredictionLC().size()-2; // Match the neighborhood (Bayes filter)
			std::map<int, int> neighbors;
			unsigned int delta = _spreadMargin;
			UTimer timeGetN;

			if(_localGraphCleaned && margin < _memory->getStMemSize())
			{
				_memory->cleanLocalGraph(_reactivateId, margin);
				timeCleaningNeighbors = timeGetN.ticks();
				ULOGGER_DEBUG("Time cleaning local graph around %d = %fs", _reactivateId, timeCleaningNeighbors);
			}


			_memory->getNeighborsId(neighbors, _reactivateId, margin+delta);
			ULOGGER_DEBUG("margin=%d, neighbors.size=%d, time=%fs", margin+delta, neighbors.size(), timeGetN.ticks());


			// Sort by margin distance
			reactivatedIds.push_back(_reactivateId);
			int m = margin+delta;
			std::list<int> directNeighbors;
			//Priority to locations near in space
			while(m > 0)
			{
				std::list<int> idsFront;
				std::list<int> idsBack;
				for(std::map<int, int>::reverse_iterator iter=neighbors.rbegin(); iter!=neighbors.rend(); ++iter)
				{
					//Don't include signatures already in the short-time memory
					if(iter->second == m && _memory->isInSTM(iter->first) == 0)
					{
						//Priority to locations near in time
						if(iter->first <= (_reactivateId + m) && iter->first >= (_reactivateId - m))
						{
							idsFront.push_back(iter->first);
						}
						else
						{
							idsBack.push_back(iter->first);
						}
						ULOGGER_DEBUG("N id=%d, margin=%d", iter->first, iter->second);
						if(m == int(margin+delta))
						{
							directNeighbors.push_back(iter->first);
						}
					}
				}
				uAppend(reactivatedIds, idsFront);
				uAppend(reactivatedIds, idsBack);
				--m;
			}

			// Max 2 signatures retrieved
			signaturesReactivated = _memory->reactivateSignatures(reactivatedIds, _maxRetrieved, margin * 2 + 1);

			// Check if all direct neighbors are loaded (for statistics)
			for(std::list<int>::iterator iter = directNeighbors.begin(); iter != directNeighbors.end(); ++iter)
			{
				if(_memory->getSignature(*iter) == 0)
				{
					++directNeighborsNotReactivated;
				}
			}

			// The reactivatedIds is used to ignore signatures when forgetting,
			// limit the size to neighborhood size
			while(reactivatedIds.size() > margin * 2 + 1)
			{
				reactivatedIds.pop_back();
			}
		}
		timeReactivations = timer.ticks();
		ULOGGER_INFO("timeReactivations=%f", timeReactivations);

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
			likelihood = _memory->computeLikelihood(signature);

			// Adjust the likelihood
			adjustedLikelihood = likelihood;
			this->adjustLikelihood(adjustedLikelihood);

			timeLikelihoodCalculation = timer.ticks();
			ULOGGER_INFO("timeLikelihoodCalculation=%f",timeLikelihoodCalculation);

			//============================================================
			// Apply the Bayes filter
			//  Posterior = Likelihood x Prior
			//============================================================
			ULOGGER_INFO("getting posterior...");

			// Compute the posterior
			posterior = _bayesFilter->computePosterior(_memory, adjustedLikelihood);
			timePosteriorCalculation = timer.ticks();
			ULOGGER_INFO("timePosteriorCalculation=%f",timePosteriorCalculation);

			for(std::map<int,float>::iterator iter=posterior.begin(); iter!=posterior.end(); ++iter)
			{
				UDEBUG("posterior (%d) = %f", iter->first, iter->second);
			}

			//============================================================
			// Select the highest hypothesis
			//============================================================
			ULOGGER_INFO("creating hypotheses...");
			if(!signature->isBadSignature() && posterior.size())
			{
				if(posterior.size() >= _minMemorySizeForLoopDetection)
				{
					this->selectHypotheses(posterior, hypotheses, true);
				}
				else
				{
					ULOGGER_WARN("The memory size is too small (%d) to create hypotheses, "
								"min of %d is required. This warning can be safely ignored "
								"if the detector has just started from an empty memory.",
								posterior.size(), _minMemorySizeForLoopDetection);
				}
			}
			timeHypothesesCreation = timer.ticks();
			ULOGGER_INFO("timeHypothesesCreation=%f",timeHypothesesCreation);

			if(hypotheses.size())
			{
				// Loop closure Threshold
				if(hypotheses.front().second >= _loopThr)
				{
					//============================================================
					// Hypothesis verification for loop closure with geometric
					// information (like the epipolar geometry or using the local map
					// associated with the signature)
					//============================================================
					if(hypotheses.front().second >= _loopRatio*_highestHypothesisValue &&
					   _highestHypothesisValue &&
					   _vhStrategy->verify(signature, _memory->getSignature(hypotheses.front().first)) &&
					   likelihood.at(hypotheses.front().first) > 0.0f)
					{
						_lcHypothesisId = hypotheses.front().first;
					}
					else
					{
						rejectedHypothesis = true;
					}

					timeHypothesesValidation = timer.ticks();
					ULOGGER_INFO("timeHypothesesValidation=%f",timeHypothesesValidation);
				}
				else if(hypotheses.front().second < _loopRatio*_highestHypothesisValue)
				{
					rejectedHypothesis = true;
				}

				//============================================================
				// Retrieval id update
				//============================================================
				std::list<std::pair<int, float> > hyp;
				// using likelihood (only if in reactivated ids)
				this->selectHypotheses(likelihood, hyp, false);
				int lastReactivatedId = _reactivateId;
				if(std::find(reactivatedIds.begin(),reactivatedIds.end(), hyp.front().first) != reactivatedIds.end())
				{
					_reactivateId = hyp.front().first;
				}
				else
				{
					//using highest hypothesis
					this->selectHypotheses(posterior, hyp, false);
					if(likelihood.at(hyp.front().first) > _remThr)
					{
						_reactivateId = hyp.front().first;
					}
					// else don't change the reactivateId
				}

				// Spreading margin...
				if(hypotheses.front().second >= _loopThr && !rejectedHypothesis)
				{
					// We are tracking the next loop closures,
					// reset the retrieval margin
					_spreadMargin = 0;
					UDEBUG("Margin=0");
				}
				else if(std::find(reactivatedIds.begin(),reactivatedIds.end(), hypotheses.front().first) != reactivatedIds.end())
				{
					if(signaturesReactivated<_maxRetrieved || (hypotheses.front().first == lastReactivatedId && rejectedHypothesis))
					{
						// We are loosing the next loop closures (it
						// can be temporary occlusions/bad images),
						//increment the retrieval margin
						_spreadMargin+=2;
						UDEBUG("Margin++");
					}
					//else don't change the margin
				}
				else
				{
					// We lost the next loop closures (a new path is taken),
					// reset the retrieval margin
					_spreadMargin = 0;
					UDEBUG("Margin=0");
				}

				//for statistic...
				hypothesisRatio = _highestHypothesisValue>0?hypotheses.front().second/_highestHypothesisValue:0;

				_highestHypothesisValue = hypotheses.front().second;
			}

			//=============================================================
			// Update loop closure links
			//=============================================================
			// Make the new one the parent of the old one
			if(_lcHypothesisId>0)
			{
				_memory->addLoopClosureLink(_lcHypothesisId, signature->id());
			}
		} // if(_memory->getWorkingMemSize())
	} // if(!signature->isBadSignature())

	//============================================================
	// Select next actions
	//============================================================
	const Signature * sLoop = 0;
	int highestHypothesisId = 0;
	if(hypotheses.size() > 0)
	{
		highestHypothesisId = hypotheses.front().first;
	}
	if(_lcHypothesisId > 0)
	{
		sLoop = _memory->getSignature(_lcHypothesisId);
	}
	else if(highestHypothesisId > 0)
	{
		sLoop = _memory->getSignature(highestHypothesisId);
	}

	// only send actions if rejectLoopReason!=3 (decreasing hypotheses)
	if(sLoop && (_actionsSentRejectHyp || !rejectedHypothesis))
	{
		// select the actions of the neighbor with the highest
		// weight (select the more recent if some have the same weight)
		// OR
		// select the newest one (with actions)
		const NeighborsMap & neighbors =  sLoop->getNeighbors();
		float currentHyp = -1;
		for(NeighborsMap::const_reverse_iterator iter=neighbors.rbegin(); iter!=neighbors.rend(); ++iter)
		{
			if(iter->second.size())
			{
				if(_actionsByTime)
				{
					_actions = iter->second;
					break;
				}
				else // use hypothesis value
				{
					float hyp = uValue(posterior, iter->first, 0.0f);
					if(hyp > currentHyp)
					{
						currentHyp = hyp;
						_actions = iter->second;
					}
				}
			}
		}
	}

	//============================================================
	// Prepare statistics
	//============================================================
	// Data used for the statistics event and for the log files
	int processMemoryUsed = UProcessInfo::getMemoryUsage()/(1024*1024); // MB
	int databaseMemoryUsed = _memory->getDatabaseMemoryUsed(); // MB
	float responseThr = 0;
	int dictionarySize = 0;
	int refWordsCount = 0;
	int refUniqueWordsCount = 0;
	const KeypointSignature * ssRef = 0;
	const KeypointSignature * ssLoop = 0;
	int lcHypothesisReactivated = 0;
	float rehearsalValue = uValue(memUpdateStats, std::string("Memory/Rehearsal Value/"), 0.0f);
	KeypointMemory * kpMem = dynamic_cast<KeypointMemory *>(_memory);
	if(kpMem)
	{
		if(sLoop)
		{
			ssLoop = dynamic_cast<const KeypointSignature *>(sLoop);
		}
		ssRef = dynamic_cast<const KeypointSignature *>(signature);
		responseThr = (float)kpMem->getKeypointDetector()->getAdaptiveResponseThr();
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

	// only prepare statistics if required or when there is a loop closure
	Statistics * stat = 0;
	if(_lcHypothesisId || _actions.size() || _publishStats)
	{
		ULOGGER_INFO("sending stats...");
		stat = new Statistics();
		stat->setRefImageId(refId);
		if(_lcHypothesisId != Memory::kIdInvalid)
		{
			stat->addStatistic(Statistics::kLoopClosure_id(), _lcHypothesisId);
			stat->setLoopClosureId(_lcHypothesisId);
			ULOGGER_INFO("Loop closure detected! With id=%d", _lcHypothesisId);
		}
		if(_actions.size())
		{
			stat->setActions(_actions);
		}
		if(_publishStats && refId != Memory::kIdInvalid)
		{
			ULOGGER_INFO("send all stats...");
			stat->setExtended(1);

			stat->setRefImage(smState->getImage()); // The image will be released by the Statistics destructor

			stat->addStatistic(Statistics::kParent_id(), signature->getLoopClosureId());
			if(sLoop)
			{
				lcHypothesisReactivated = 0;
				if(sLoop && sLoop->isSaved())
				{
					lcHypothesisReactivated = 1;
				}

				const IplImage * img = sLoop->getImage();
				if(!img && _memory->isRawDataKept())
				{
					IplImage * image = _memory->getImage(sLoop->id());
					stat->setLoopClosureImage(&image); // The image will be released by the Statistics destructor
				}
				else if(img)
				{
					stat->setLoopClosureImage(img); // The image will be copied
				}
			}
			stat->setPosterior(posterior);
			stat->setLikelihood(adjustedLikelihood);
			stat->addStatistic(Statistics::kLoopHighest_hypothesis_id(), highestHypothesisId);
			stat->addStatistic(Statistics::kLoopHighest_hypothesis_value(), _highestHypothesisValue);
			stat->addStatistic(Statistics::kHypothesis_reactivated(), lcHypothesisReactivated);
			stat->addStatistic(Statistics::kLoopVp_likelihood(), vpLikelihood);
			stat->addStatistic(Statistics::kLoopReactivateId(), _reactivateId);
			stat->addStatistic(Statistics::kLoopHypothesis_ratio(), hypothesisRatio);
			stat->addStatistic(Statistics::kLoopRetrieval_margin(), _spreadMargin);

			// Child count by parent signature on the root of the memory ... for statistics
			stat->setWeights(_memory->getWeights());

			stat->addStatistic(Statistics::kMemoryWorking_memory_size(), _memory->getWorkingMemSize());
			stat->addStatistic(Statistics::kMemoryShort_time_memory_size(), _memory->getStMemSize());
			stat->addStatistic(Statistics::kMemoryDatabase_size(), (float)databaseMemoryUsed);
			stat->addStatistic(Statistics::kMemoryProcess_memory_used(), (float)processMemoryUsed);
			stat->addStatistic(Statistics::kMemorySignatures_reactivated(), signaturesReactivated);
			stat->addStatistic(Statistics::kMemoryImages_buffered(), (float)_smStateBuffer.size());

			// timing...
			stat->addStatistic(Statistics::kTimingMemory_update(), timeMemoryUpdate*1000);
			stat->addStatistic(Statistics::kTimingReactivation(), timeReactivations*1000);
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
			stat->addStatistic(Statistics::kKeypointResponse_threshold(), responseThr);

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

			//Epipolar geometry constraint
			stat->addStatistic(Statistics::kLoopRejectedHypothesis(), rejectedHypothesis?1.0f:0);
			timeStatsCreation = timer.ticks();
		}
		else
		{
			timeStatsCreation = timer.ticks();
		}

		ULOGGER_INFO("Time creating stats = %f...", timeStatsCreation);
	}

	//By default, remove all signatures with a loop closure link if they are not in reactivateIds
	//This will also remove rehearsed signatures
	int signaturesRemoved = 0;
	if(_reactivateId)
	{
		reactivatedIds.push_back(_reactivateId);
	}
	signaturesRemoved += _memory->cleanup(reactivatedIds);
	timeMemoryCleanup = timer.ticks();
	ULOGGER_INFO("timeMemoryCleanup = %f...", timeMemoryCleanup);

	//============================================================
	// Real time threshold
	// If time allowed for the detection exceed the limit of
	// real-time, move the oldest signature with less frequency
	// entry (from X oldest) from the short term memory to the
	// long term memory.
	//============================================================
	double totalTime = timerTotal.ticks();
	ULOGGER_INFO("Total time processing = %f...", totalTime);
	timer.start();
	if(_maxTimeAllowed != 0 && totalTime>_maxTimeAllowed)
	{
		ULOGGER_INFO("Removing old signatures because time limit is reached %f>%f...", totalTime, _maxTimeAllowed);
		signaturesRemoved = _memory->forget(reactivatedIds);
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
		stat->addStatistic(Statistics::kTimingStatistics_creation(), timeStatsCreation*1000);
		stat->addStatistic(Statistics::kTimingTotal(), totalTime*1000);
		stat->addStatistic(Statistics::kTimingForgetting(), timeRealTimeLimitReachedProcess*1000);
		stat->addStatistic(Statistics::kTimingEmptying_memory_trash(), timeEmptyingMemoryTrash*1000);
		stat->addStatistic(Statistics::kTimingMemory_cleanup(), timeMemoryCleanup*1000);
		stat->addStatistic(Statistics::kMemorySignatures_removed(), signaturesRemoved);
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

	float vpHypothesis = 0;
	std::map<int, float>::iterator vpIter = posterior.find(-1);
	if(vpIter != posterior.end())
	{
		vpHypothesis = vpIter->second;
	}

	// Log info...
	// TODO : use a specific class which will handle the RtabmapEvent
	if(_foutFloat)
	{

		fprintf(_foutFloat, "%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n",
				totalTime,
				timeMemoryUpdate,
				timeReactivations,
				timeLikelihoodCalculation,
				timePosteriorCalculation,
				timeHypothesesCreation,
				timeHypothesesValidation,
				timeRealTimeLimitReachedProcess,
				timeStatsCreation,
				_highestHypothesisValue,
				vpLikelihood,
				maxLikelihood,
				sumLikelihoods,
				mean,
				stddev,
				vpHypothesis,
				timeEmptyingMemoryTrash,
				rehearsalValue);
	}

	if(_foutInt)
	{
		fprintf(_foutInt, "%d %d %d %d %d %d %d %d %d %d %d %d %d %d %lu %d %d\n",
				_lcHypothesisId,
				highestHypothesisId,
				signaturesRemoved,
				int(responseThr),
				refWordsCount,
				dictionarySize,
				int(_memory->getWorkingMemSize()),
				rejectedHypothesis?1:0,
				processMemoryUsed,
				databaseMemoryUsed,
				signaturesReactivated,
				lcHypothesisReactivated,
				refUniqueWordsCount,
				_reactivateId,
				nonNulls.size(),
				directNeighborsNotReactivated,
				_spreadMargin);
	}
	ULOGGER_INFO("Time logging = %f...", timer.ticks());
	//ULogger::flush();
	delete smState;
}

// ownership is transferred
void Rtabmap::addSMState(SMState * data)
{
	ULOGGER_DEBUG("");
	if(!data)
	{
		ULOGGER_ERROR("Data is null?!");
		return;
	}

	bool notify = true;

	_smStateBufferMutex.lock();
	{
		while(_smStateBufferMaxSize > 0 && _smStateBuffer.size() >= (unsigned int)_smStateBufferMaxSize)
		{
			ULOGGER_WARN("Data buffer is full, the oldest data is removed to add the new one.");
			delete _smStateBuffer.front();
			_smStateBuffer.pop_front();
			notify = false;
		}
		_smStateBuffer.push_back(data);
	}
	_smStateBufferMutex.unlock();

	if(notify)
	{
		_newSMStateSem.release();
	}

}

SMState * Rtabmap::getSMState()
{
	ULOGGER_DEBUG("");
	SMState * data = 0;

	ULOGGER_INFO("waiting for data");
	_newSMStateSem.acquire();
	ULOGGER_INFO("wake-up");

	_smStateBufferMutex.lock();
	{
		if(!_smStateBuffer.empty())
		{
			data = _smStateBuffer.front();
			_smStateBuffer.pop_front();
		}
	}
	_smStateBufferMutex.unlock();
	return data;
}

// SETTERS
void Rtabmap::setReactivationDisabled(bool reactivationDisabled)
{
	_reactivationDisabled = reactivationDisabled;
}

void Rtabmap::setMaxTimeAllowed(float maxTimeAllowed)
{
	//must be positive, 0 mean inf time allowed (no time limit)
	_maxTimeAllowed = maxTimeAllowed;
	if(_maxTimeAllowed < 0)
	{
		ULOGGER_WARN("maxTimeAllowed < 0, then setting it to 0 (inf).");
		_maxTimeAllowed = 0;
	}
}

void Rtabmap::setDataBufferSize(int size)
{
	if(size < 0)
	{
		ULOGGER_WARN("size < 0, then setting it to 0 (inf).");
		_smStateBufferMaxSize = 0;
	}
	else
	{
		_smStateBufferMaxSize = size;
	}
}

void Rtabmap::setWorkingDirectory(std::string path)
{
	if(path.size() && (path.at(path.size()-1) != '\\' || path.at(path.size()-1) != '/' ))
	{
		path += "/";
	}

	if(!path.empty() && UDirectory::exists(path))
	{
		ULOGGER_DEBUG("Comparing new path \"%s\" with \"%s\"", path.c_str(), _wDir.c_str());
		if(path.compare(_wDir) != 0)
		{
			_wDir = path;
			if(_memory)
			{
				//clear all buffered images
				this->kill();
				UEventsManager::post(new RtabmapEventInit(RtabmapEventInit::kInitializing));
				_memory->init(DB_TYPE, _wDir + DB_NAME);
				UEventsManager::post(new RtabmapEventInit(RtabmapEventInit::kInitialized));
				this->kill(); // this will clean a second time the image buffer (if some images were added during the memory initialization)
				setupLogFiles();
				this->start();
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

// ownership is transferred
void Rtabmap::process(SMState * data)
{
	if(!this->isRunning())
	{
		this->addSMState(data);
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

	// Use only non-null values
	std::vector<float> allValues = uValues(likelihood);
	std::list<float> values;
	for(unsigned int i=0; i<allValues.size(); ++i)
	{
		if(allValues[i])
		{
			values.push_back(allValues[i]);
		}
	}

	if(likelihood.begin()->first == -1 && likelihood.begin()->second)
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
	for(std::map<int, float>::iterator iter=likelihood.begin(); iter!= likelihood.end(); ++iter)
	{
		if(iter->second > mean+stdDev && mean)
		{
			iter->second = (iter->second-(stdDev-epsilon))/mean;
			if(iter->second > max)
			{
				max = iter->second;
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
			likelihood.begin()->second = 2;
			ULOGGER_DEBUG("Set vp likelihood to 2, time=%fs", timer.ticks());
		}
		if(likelihood.begin()->second<1.0f)
		{
			likelihood.begin()->second = 1.0f;
		}
	}

	ULOGGER_DEBUG("mean=%f, stdDev=%f, max=%f time=%fs", mean, stdDev, max, timer.ticks());
}

void Rtabmap::selectHypotheses(const std::map<int, float> & posterior,
							std::list<std::pair<int, float> > & hypotheses,
							bool useNeighborSum) const
{
	ULOGGER_DEBUG("");
	if(!_bayesFilter || !_memory)
	{
		ULOGGER_ERROR("RTAB-Map must be initialized first!");
		return;
	}
	if(posterior.size() == 0)
	{
		ULOGGER_ERROR("Posterior parameter size = 0?");
		return;
	}

	float value;
	float valueSum;
	int id;
	unsigned int margin = _bayesFilter->getPredictionLC().size()-2;
	UTimer timer;
	timer.start();
	UTimer timerGlobal;
	timerGlobal.start();
	for(std::map<int, float>::const_iterator iter = posterior.begin(); iter != posterior.end(); ++iter)
	{
		value = (*iter).second;
		valueSum = (*iter).second;
		id = (*iter).first;

		if(id > 0)
		{
			if(useNeighborSum)
			{
				//Add neighbor values if they exist
				std::map<int, int> neighbors;
				_memory->getNeighborsId(neighbors, id, margin, false);

				for(std::map<int, int>::iterator jter=neighbors.begin(); jter!=neighbors.end(); ++jter)
				{
					if(jter->first != iter->first)
					{
						std::map<int, float>::const_iterator tmpIter = posterior.find(jter->first);
						if(tmpIter!=posterior.end())
						{
							valueSum += tmpIter->second;
							if((*tmpIter).second > value)
							{
								value = tmpIter->second;
								id = tmpIter->first;
							}
						}
					}
				}
				//ULOGGER_DEBUG("time=%fs", timer.ticks());
			}

			if(hypotheses.size() && valueSum > hypotheses.front().second)
			{
				hypotheses.push_front(std::pair<int, float>(id, valueSum));
			}
			else
			{
				hypotheses.push_back(std::pair<int, float>(id, valueSum));
			}
		}
	}
	if(hypotheses.size())
	{
		ULOGGER_DEBUG("Highest hyposthesis(%d)=%f", hypotheses.front().first, hypotheses.front().second);
	}
	ULOGGER_DEBUG("time=%fs", timerGlobal.ticks());
}

void Rtabmap::dumpPrediction() const
{
	if(_memory && _bayesFilter)
	{
		const std::map<int, int> & wm = _memory->getWorkingMem();
		CvMat * prediction = cvCreateMat(wm.size(), wm.size(), CV_32FC1);
		std::map<int, int> keys;
		std::map<int, int>::iterator insertedKeyPos = keys.begin();
		int index = 0;
		for(std::map<int, int>::const_iterator iter=wm.begin(); iter!=wm.end(); ++iter)
		{
			insertedKeyPos = keys.insert(insertedKeyPos, std::pair<int, int>(iter->first, index++));
		}
		_bayesFilter->generatePrediction(prediction, _memory, keys);

		FILE* fout = 0;
		std::string fileName = this->getWorkingDir() + "/DumpPrediction.txt";
		#ifdef _MSC_VER
			fopen_s(&fout, fileName.c_str(), "w");
		#else
			fout = fopen(fileName.c_str(), "w");
		#endif

		if(fout)
		{
			for(int i=0; i<prediction->rows; ++i)
			{
				for(int j=0; j<prediction->cols; ++j)
				{
					fprintf(fout, "%f ", prediction->data.fl[j + i*prediction->cols]);
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
