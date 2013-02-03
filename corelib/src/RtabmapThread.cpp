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
#include "rtabmap/core/RtabmapThread.h"
#include "rtabmap/core/RtabmapEvent.h"
#include "rtabmap/core/Camera.h"

#include <utilite/ULogger.h>
#include <utilite/UEventsManager.h>
#include <utilite/UStl.h>

namespace rtabmap {

RtabmapThread::RtabmapThread() :
		_imageBufferMaxSize(Parameters::defaultRtabmapImageBufferSize()),
		_rtabmap(new Rtabmap())

{
}

RtabmapThread::~RtabmapThread()
{
	UEventsManager::removeHandler(this);

	// Stop the thread first
	join(true);

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

	_imageAdded.release();
}

void RtabmapThread::setWorkingDirectory(const std::string & path)
{
	_rtabmap->setWorkingDirectory(path);
}

void RtabmapThread::clearBufferedSensors()
{
	_imageMutex.lock();
	{
		_imageBuffer.clear();
	}
	_imageMutex.unlock();
}

void RtabmapThread::mainLoopKill()
{
	this->clearBufferedSensors();

	// this will post the newData semaphore
	_imageAdded.release();
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

	ParametersMap::iterator iter;
	switch(state)
	{
	case kStateDetecting:
		this->process();
		break;
	case kStateChangingParameters:
		if((iter=parameters.find(Parameters::kRtabmapImageBufferSize())) != parameters.end())
		{
			_imageBufferMaxSize = std::atoi(iter->second.c_str());
		}
		_rtabmap->parseParameters(parameters);
		break;
	case kStateReseting:
		_rtabmap->resetMemory();
		this->clearBufferedSensors();
		break;
	case kStateDumpingMemory:
		_rtabmap->dumpData();
		break;
	case kStateDumpingPrediction:
		_rtabmap->dumpPrediction();
		break;
	case kStateGeneratingGraph:
		_rtabmap->generateGraph(parameters.at("path"));
		break;
	case kStateGeneratingLocalGraph:
		_rtabmap->generateGraph(parameters.at("path"), atoi(parameters.at("id").c_str()), atoi(parameters.at("margin").c_str()));
		break;
	case kStateDeletingMemory:
		_rtabmap->resetMemory(true);
		this->clearBufferedSensors();
		break;
	case kStateCleanSensorsBuffer:
		this->clearBufferedSensors();
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
		else if(cmd == RtabmapEventCmd::kCmdGenerateGraph)
		{
			if(!rtabmapEvent->getStr().empty())
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
			if(values.size() == 3)
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
void RtabmapThread::process()
{
	Image image;
	getImage(image);
	if(!image.empty())
	{
		_rtabmap->process(image);

		Statistics stats = _rtabmap->getStatistics();
		stats.addStatistic(Statistics::kMemoryImages_buffered(), (float)_imageBuffer.size());
		ULOGGER_DEBUG("posting statistics_ event...");
		this->post(new RtabmapEvent(stats));
	}
}

void RtabmapThread::addImage(const Image & image)
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
		while(_imageBufferMaxSize > 0 && _imageBuffer.size() > (unsigned int)_imageBufferMaxSize)
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

void RtabmapThread::getImage(Image & image)
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

void RtabmapThread::setDataBufferSize(int size)
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

} /* namespace rtabmap */
