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
#include "rtabmap/core/RtabmapThread.h"
#include "rtabmap/core/RtabmapEvent.h"
#include "rtabmap/core/Camera.h"
#include "rtabmap/core/CameraEvent.h"
#include "rtabmap/core/ParamEvent.h"
#include "rtabmap/core/OdometryEvent.h"

#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UEventsManager.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/utilite/UConversion.h>

namespace rtabmap {

RtabmapThread::RtabmapThread(Rtabmap * rtabmap) :
		_dataBufferMaxSize(Parameters::defaultRtabmapImageBufferSize()),
		_rate(Parameters::defaultRtabmapDetectionRate()),
		_frameRateTimer(new UTimer()),
		_rtabmap(rtabmap),
		_paused(false)

{
	UASSERT(rtabmap != 0);
}

RtabmapThread::~RtabmapThread()
{
	UEventsManager::removeHandler(this);

	// Stop the thread first
	join(true);

	delete _frameRateTimer;
	delete _rtabmap;
}

void RtabmapThread::pushNewState(State newState, const ParametersMap & parameters)
{
	ULOGGER_DEBUG("to %d", newState);

	_stateMutex.lock();
	{
		_state.push(newState);
		_stateParam.push(parameters);
	}
	_stateMutex.unlock();

	_dataAdded.release();
}

void RtabmapThread::clearBufferedData()
{
	_dataMutex.lock();
	{
		_dataBuffer.clear();
	}
	_dataMutex.unlock();
}

void RtabmapThread::setDetectorRate(float rate)
{
	UASSERT(rate >= 0.0f);
	_rate = rate;
}

void RtabmapThread::setBufferSize(int bufferSize)
{
	UASSERT(bufferSize >= 0);
	_dataBufferMaxSize = bufferSize;
}

void RtabmapThread::publishMap(bool optimized, bool full) const
{
	std::map<int, Signature> signatures;
	std::map<int, Transform> poses;
	std::multimap<int, Link> constraints;
	std::map<int, int> mapIds;

	_rtabmap->get3DMap(signatures,
			poses,
			constraints,
			mapIds,
			optimized,
			full);

	this->post(new RtabmapEvent3DMap(signatures,
			poses,
			constraints,
			mapIds));
}

void RtabmapThread::publishTOROGraph(bool optimized, bool full) const
{
	std::map<int, Signature> signatures;
	std::map<int, Transform> poses;
	std::multimap<int, Link> constraints;
	std::map<int, int> mapIds;

	_rtabmap->getGraph(poses,
			constraints,
			mapIds,
			optimized,
			full);

	this->post(new RtabmapEvent3DMap(signatures,
			poses,
			constraints,
			mapIds));
}


void RtabmapThread::mainLoopKill()
{
	this->clearBufferedData();

	// this will post the newData semaphore
	_dataAdded.release();
}

void RtabmapThread::mainLoop()
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
	case kStateInit:
		UASSERT(!parameters.at("RtabmapThread/DatabasePath").empty());
		_rtabmap->init(parameters, parameters.at("RtabmapThread/DatabasePath"));
		break;
	case kStateChangingParameters:
		Parameters::parse(parameters, Parameters::kRtabmapImageBufferSize(), _dataBufferMaxSize);
		Parameters::parse(parameters, Parameters::kRtabmapDetectionRate(), _rate);
		UASSERT(_dataBufferMaxSize >= 0);
		UASSERT(_rate >= 0.0f);
		_rtabmap->parseParameters(parameters);
		break;
	case kStateReseting:
		_rtabmap->resetMemory();
		this->clearBufferedData();
		break;
	case kStateClose:
		if(_dataBuffer.size())
		{
			UWARN("Closing... %d data still buffered! They will be cleared.", (int)_dataBuffer.size());
			this->clearBufferedData();
		}
		_rtabmap->close();
		break;
	case kStateDumpingMemory:
		_rtabmap->dumpData();
		break;
	case kStateDumpingPrediction:
		_rtabmap->dumpPrediction();
		break;
	case kStateGeneratingDOTGraph:
		_rtabmap->generateDOTGraph(parameters.at("path"));
		break;
	case kStateGeneratingDOTLocalGraph:
		_rtabmap->generateDOTGraph(parameters.at("path"), atoi(parameters.at("id").c_str()), atoi(parameters.at("margin").c_str()));
		break;
	case kStateGeneratingTOROGraphLocal:
		_rtabmap->generateTOROGraph(parameters.at("path"), atoi(parameters.at("optimized").c_str())!=0, false);
		break;
	case kStateGeneratingTOROGraphGlobal:
		_rtabmap->generateTOROGraph(parameters.at("path"), atoi(parameters.at("optimized").c_str())!=0, true);
		break;
	case kStateCleanDataBuffer:
		this->clearBufferedData();
		break;
	case kStatePublishingMapLocal:
		this->publishMap(atoi(parameters.at("optimized").c_str())!=0, false);
		break;
	case kStatePublishingMapGlobal:
		this->publishMap(atoi(parameters.at("optimized").c_str())!=0, true);
		break;
	case kStatePublishingTOROGraphLocal:
		this->publishTOROGraph(atoi(parameters.at("optimized").c_str())!=0, false);
		break;
	case kStatePublishingTOROGraphGlobal:
		this->publishTOROGraph(atoi(parameters.at("optimized").c_str())!=0, true);
		break;
	case kStateTriggeringMap:
		_rtabmap->triggerNewMap();
		break;
	default:
		UFATAL("Invalid state !?!?");
		break;
	}
}


void RtabmapThread::handleEvent(UEvent* event)
{
	if(this->isRunning() && event->getClassName().compare("CameraEvent") == 0)
	{
		UDEBUG("CameraEvent");
		CameraEvent * e = (CameraEvent*)event;
		if(e->getCode() == CameraEvent::kCodeImage || e->getCode() == CameraEvent::kCodeImageDepth)
		{
			this->addData(e->data());
		}
	}
	else if(event->getClassName().compare("OdometryEvent") == 0)
	{
		UDEBUG("OdometryEvent");
		OdometryEvent * e = (OdometryEvent*)event;
		if(e->isValid())
		{
			this->addData(e->data());
		}
	}
	else if(event->getClassName().compare("RtabmapEventCmd") == 0)
	{
		RtabmapEventCmd * rtabmapEvent = (RtabmapEventCmd*)event;
		RtabmapEventCmd::Cmd cmd = rtabmapEvent->getCmd();
		if(cmd == RtabmapEventCmd::kCmdInit)
		{
			ULOGGER_DEBUG("CMD_INIT");
			ParametersMap parameters = ((RtabmapEventCmd*)event)->getParameters();
			UASSERT(!rtabmapEvent->getStr().empty());
			UASSERT(parameters.insert(ParametersPair("RtabmapThread/DatabasePath", rtabmapEvent->getStr())).second);
			pushNewState(kStateInit, parameters);
		}
		else if(cmd == RtabmapEventCmd::kCmdClose)
		{
			ULOGGER_DEBUG("CMD_CLOSE");
			pushNewState(kStateClose);
		}
		else if(cmd == RtabmapEventCmd::kCmdResetMemory)
		{
			ULOGGER_DEBUG("CMD_RESET_MEMORY");
			pushNewState(kStateReseting);
		}
		else if(cmd == RtabmapEventCmd::kCmdDumpMemory)
		{
			ULOGGER_DEBUG("CMD_DUMP_MEMORY");
			pushNewState(kStateDumpingMemory);
		}
		else if(cmd == RtabmapEventCmd::kCmdDumpPrediction)
		{
			ULOGGER_DEBUG("CMD_DUMP_PREDICTION");
			pushNewState(kStateDumpingPrediction);
		}
		else if(cmd == RtabmapEventCmd::kCmdGenerateDOTGraph)
		{
			UASSERT(!rtabmapEvent->getStr().empty());

			ULOGGER_DEBUG("CMD_GENERATE_DOT_GRAPH");
			ParametersMap param;
			param.insert(ParametersPair("path", rtabmapEvent->getStr()));
			pushNewState(kStateGeneratingDOTGraph, param);
		}
		else if(cmd == RtabmapEventCmd::kCmdGenerateDOTLocalGraph)
		{
			std::list<std::string> values = uSplit(rtabmapEvent->getStr(), ';');
			UASSERT(values.size() == 3);

			ULOGGER_DEBUG("CMD_GENERATE_DOT_LOCAL_GRAPH");
			ParametersMap param;
			param.insert(ParametersPair("path", *values.begin()));
			param.insert(ParametersPair("id", *(++values.begin())));
			param.insert(ParametersPair("margin", *values.rbegin()));
			pushNewState(kStateGeneratingDOTLocalGraph, param);

		}
		else if(cmd == RtabmapEventCmd::kCmdGenerateTOROGraphLocal)
		{
			UASSERT(!rtabmapEvent->getStr().empty());

			ULOGGER_DEBUG("CMD_GENERATE_TORO_GRAPH_LOCAL");
			ParametersMap param;
			param.insert(ParametersPair("path", rtabmapEvent->getStr()));
			param.insert(ParametersPair("optimized", uNumber2Str(rtabmapEvent->getInt())));
			pushNewState(kStateGeneratingTOROGraphLocal, param);

		}
		else if(cmd == RtabmapEventCmd::kCmdGenerateTOROGraphGlobal)
		{
			UASSERT(!rtabmapEvent->getStr().empty());

			ULOGGER_DEBUG("CMD_GENERATE_TORO_GRAPH_GLOBAL");
			ParametersMap param;
			param.insert(ParametersPair("path", rtabmapEvent->getStr()));
			param.insert(ParametersPair("optimized", uNumber2Str(rtabmapEvent->getInt())));
			pushNewState(kStateGeneratingTOROGraphGlobal, param);

		}
		else if(cmd == RtabmapEventCmd::kCmdCleanDataBuffer)
		{
			ULOGGER_DEBUG("CMD_CLEAN_DATA_BUFFER");
			pushNewState(kStateCleanDataBuffer);
		}
		else if(cmd == RtabmapEventCmd::kCmdPublish3DMapLocal)
		{
			ULOGGER_DEBUG("CMD_PUBLISH_MAP_LOCAL");
			ParametersMap param;
			param.insert(ParametersPair("optimized", uNumber2Str(rtabmapEvent->getInt())));
			pushNewState(kStatePublishingMapLocal, param);
		}
		else if(cmd == RtabmapEventCmd::kCmdPublish3DMapGlobal)
		{
			ULOGGER_DEBUG("CMD_PUBLISH_MAP_GLOBAL");
			ParametersMap param;
			param.insert(ParametersPair("optimized", uNumber2Str(rtabmapEvent->getInt())));
			pushNewState(kStatePublishingMapGlobal, param);
		}
		else if(cmd == RtabmapEventCmd::kCmdPublishTOROGraphLocal)
		{
			ULOGGER_DEBUG("CMD_PUBLISH_TORO_GRAPH_LOCAL");
			ParametersMap param;
			param.insert(ParametersPair("optimized", uNumber2Str(rtabmapEvent->getInt())));
			pushNewState(kStatePublishingTOROGraphLocal, param);
		}
		else if(cmd == RtabmapEventCmd::kCmdPublishTOROGraphGlobal)
		{
			ULOGGER_DEBUG("CMD_PUBLISH_TORO_GRAPH_GLOBAL");
			ParametersMap param;
			param.insert(ParametersPair("optimized", uNumber2Str(rtabmapEvent->getInt())));
			pushNewState(kStatePublishingTOROGraphGlobal, param);
		}
		else if(cmd == RtabmapEventCmd::kCmdTriggerNewMap)
		{
			ULOGGER_DEBUG("CMD_TRIGGER_NEW_MAP");
			pushNewState(kStateTriggeringMap);
		}
		else if(cmd == RtabmapEventCmd::kCmdPause)
		{
			ULOGGER_DEBUG("CMD_PAUSE");
			_paused = !_paused;
		}
		else
		{
			UWARN("Cmd %d unknown!", cmd);
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
void RtabmapThread::process()
{
	SensorData data;
	getData(data);
	if(data.isValid())
	{
		if(_rtabmap->getMemory())
		{
			if(_rtabmap->process(data))
			{
				Statistics stats = _rtabmap->getStatistics();
				stats.addStatistic(Statistics::kMemoryImages_buffered(), (float)_dataBuffer.size());
				ULOGGER_DEBUG("posting statistics_ event...");
				this->post(new RtabmapEvent(stats));
			}
		}
		else
		{
			UERROR("RTAB-Map is not initialized! Ignoring received data...");
		}
	}
}

void RtabmapThread::addData(const SensorData & sensorData)
{
	if(!_paused)
	{
		if(!sensorData.isValid())
		{
			ULOGGER_ERROR("data not valid !?");
			return;
		}

		if(_rate>0.0f)
		{
			if(_frameRateTimer->getElapsedTime() < 1.0f/_rate)
			{
				return;
			}
		}
		_frameRateTimer->start();

		bool notify = true;
		_dataMutex.lock();
		{
			_dataBuffer.push_back(sensorData);
			while(_dataBufferMaxSize > 0 && _dataBuffer.size() > (unsigned int)_dataBufferMaxSize)
			{
				ULOGGER_WARN("Data buffer is full, the oldest data is removed to add the new one.");
				_dataBuffer.pop_front();
				notify = false;
			}
		}
		_dataMutex.unlock();

		if(notify)
		{
			_dataAdded.release();
		}
	}
}

void RtabmapThread::getData(SensorData & image)
{
	ULOGGER_DEBUG("");

	ULOGGER_INFO("waiting for data");
	_dataAdded.acquire();
	ULOGGER_INFO("wake-up");

	_dataMutex.lock();
	{
		if(!_dataBuffer.empty())
		{
			image = _dataBuffer.front();
			_dataBuffer.pop_front();
		}
	}
	_dataMutex.unlock();
}

void RtabmapThread::setDataBufferSize(int size)
{
	if(size < 0)
	{
		ULOGGER_WARN("size < 0, then setting it to 0 (inf).");
		_dataBufferMaxSize = 0;
	}
	else
	{
		_dataBufferMaxSize = size;
	}
}

} /* namespace rtabmap */
