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
#include "rtabmap/core/CameraEvent.h"
#include "rtabmap/core/ParamEvent.h"
#include "rtabmap/core/OdometryEvent.h"

#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UEventsManager.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/utilite/UConversion.h>

namespace rtabmap {

RtabmapThread::RtabmapThread() :
		_imageBufferMaxSize(Parameters::defaultRtabmapImageBufferSize()),
		_rate(Parameters::defaultRtabmapDetectionRate()),
		_frameRateTimer(new UTimer()),
		_rtabmap(new Rtabmap()),
		_paused(false)

{
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

	_imageAdded.release();
}

void RtabmapThread::clearBufferedData()
{
	_imageMutex.lock();
	{
		_imageBuffer.clear();
	}
	_imageMutex.unlock();
}

void RtabmapThread::publishMap(bool optimized, bool full) const
{
	std::map<int, std::vector<unsigned char> > images;
	std::map<int, std::vector<unsigned char> > depths;
	std::map<int, std::vector<unsigned char> > depths2d;
	std::map<int, float> depthConstants;
	std::map<int, Transform> localTransforms;
	std::map<int, Transform> poses;
	std::multimap<int, Link> constraints;

	_rtabmap->get3DMap(images,
			depths,
			depths2d,
			depthConstants,
			localTransforms,
			poses,
			constraints,
			optimized,
			full);

	this->post(new RtabmapEvent3DMap(images,
			depths,
			depths2d,
			depthConstants,
			localTransforms,
			poses,
			constraints));
}

void RtabmapThread::publishGraph(bool optimized, bool full) const
{
	std::map<int, std::vector<unsigned char> > images;
	std::map<int, std::vector<unsigned char> > depths;
	std::map<int, std::vector<unsigned char> > depths2d;
	std::map<int, float> depthConstants;
	std::map<int, Transform> localTransforms;
	std::map<int, Transform> poses;
	std::multimap<int, Link> constraints;

	_rtabmap->getGraph(poses,
			constraints,
			optimized,
			full);

	this->post(new RtabmapEvent3DMap(images,
			depths,
			depths2d,
			depthConstants,
			localTransforms,
			poses,
			constraints));
}


void RtabmapThread::mainLoopKill()
{
	this->clearBufferedData();

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

	switch(state)
	{
	case kStateDetecting:
		this->process();
		break;
	case kStateChangingParameters:
		Parameters::parse(parameters, Parameters::kRtabmapImageBufferSize(), _imageBufferMaxSize);
		Parameters::parse(parameters, Parameters::kRtabmapDetectionRate(), _rate);
		UASSERT(_imageBufferMaxSize >= 0);
		UASSERT(_rate >= 0.0f);
		_rtabmap->parseParameters(parameters);
		break;
	case kStateReseting:
		_rtabmap->resetMemory();
		this->clearBufferedData();
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
	case kStateGeneratingTOROGraph:
		_rtabmap->generateTOROGraph(parameters.at("path"), atoi(parameters.at("optimized").c_str())!=0, false);
		break;
	case kStateGeneratingTOROGraphFull:
		_rtabmap->generateTOROGraph(parameters.at("path"), atoi(parameters.at("optimized").c_str())!=0, true);
		break;
	case kStateDeletingMemory:
		if(!parameters.at("path").empty())
		{
			_rtabmap->setDatabasePath(parameters.at("path"));
		}
		_rtabmap->resetMemory(true);
		this->clearBufferedData();
		break;
	case kStateCleanDataBuffer:
		this->clearBufferedData();
		break;
	case kStatePublishingMap:
		this->publishMap(atoi(parameters.at("optimized").c_str())!=0, false);
		break;
	case kStatePublishingMapFull:
		this->publishMap(atoi(parameters.at("optimized").c_str())!=0, true);
		break;
	case kStatePublishingGraph:
		this->publishGraph(atoi(parameters.at("optimized").c_str())!=0, false);
		break;
	case kStatePublishingGraphFull:
		this->publishGraph(atoi(parameters.at("optimized").c_str())!=0, true);
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
		if(e->getCode() == CameraEvent::kCodeImage ||
		   e->getCode() == CameraEvent::kCodeFeatures ||
		   e->getCode() == CameraEvent::kCodeImageDepth)
		{
			this->addImage(e->image());
		}
	}
	else if(event->getClassName().compare("OdometryEvent") == 0)
	{
		UDEBUG("OdometryEvent");
		OdometryEvent * e = (OdometryEvent*)event;
		if(e->isValid())
		{
			this->addImage(e->data());
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
			UASSERT(!rtabmapEvent->getStr().empty());

			ULOGGER_DEBUG("CMD_GENERATE_GRAPH");
			ParametersMap param;
			param.insert(ParametersPair("path", rtabmapEvent->getStr()));
			pushNewState(kStateGeneratingGraph, param);
		}
		else if(cmd == RtabmapEventCmd::kCmdGenerateLocalGraph)
		{
			std::list<std::string> values = uSplit(rtabmapEvent->getStr(), ';');
			UASSERT(values.size() == 3);

			ULOGGER_DEBUG("CMD_GENERATE_LOCAL_GRAPH");
			ParametersMap param;
			param.insert(ParametersPair("path", *values.begin()));
			param.insert(ParametersPair("id", *(++values.begin())));
			param.insert(ParametersPair("margin", *values.rbegin()));
			pushNewState(kStateGeneratingLocalGraph, param);

		}
		else if(cmd == RtabmapEventCmd::kCmdGenerateTOROGraph)
		{
			UASSERT(!rtabmapEvent->getStr().empty());

			ULOGGER_DEBUG("CMD_GENERATE_TORO_GRAPH");
			ParametersMap param;
			param.insert(ParametersPair("path", rtabmapEvent->getStr()));
			param.insert(ParametersPair("optimized", uNumber2Str(rtabmapEvent->getInt())));
			pushNewState(kStateGeneratingTOROGraph, param);

		}
		else if(cmd == RtabmapEventCmd::kCmdGenerateTOROGraphFull)
		{
			UASSERT(!rtabmapEvent->getStr().empty());

			ULOGGER_DEBUG("CMD_GENERATE_TORO_GRAPH_FULL");
			ParametersMap param;
			param.insert(ParametersPair("path", rtabmapEvent->getStr()));
			param.insert(ParametersPair("optimized", uNumber2Str(rtabmapEvent->getInt())));
			pushNewState(kStateGeneratingTOROGraphFull, param);

		}
		else if(cmd == RtabmapEventCmd::kCmdDeleteMemory)
		{
			ULOGGER_DEBUG("CMD_DELETE_MEMORY");
			ParametersMap param;
			param.insert(ParametersPair("path", rtabmapEvent->getStr()));
			pushNewState(kStateDeletingMemory, param);
		}
		else if(cmd == RtabmapEventCmd::kCmdCleanDataBuffer)
		{
			ULOGGER_DEBUG("CMD_CLEAN_DATA_BUFFER");
			pushNewState(kStateCleanDataBuffer);
		}
		else if(cmd == RtabmapEventCmd::kCmdPublish3DMap)
		{
			ULOGGER_DEBUG("CMD_PUBLISH_MAP");
			ParametersMap param;
			param.insert(ParametersPair("optimized", uNumber2Str(rtabmapEvent->getInt())));
			pushNewState(kStatePublishingMap, param);
		}
		else if(cmd == RtabmapEventCmd::kCmdPublish3DMapFull)
		{
			ULOGGER_DEBUG("CMD_PUBLISH_MAP_FULL");
			ParametersMap param;
			param.insert(ParametersPair("optimized", uNumber2Str(rtabmapEvent->getInt())));
			pushNewState(kStatePublishingMapFull, param);
		}
		else if(cmd == RtabmapEventCmd::kCmdPublishGraph)
		{
			ULOGGER_DEBUG("CMD_PUBLISH_GRAPH");
			ParametersMap param;
			param.insert(ParametersPair("optimized", uNumber2Str(rtabmapEvent->getInt())));
			pushNewState(kStatePublishingGraph, param);
		}
		else if(cmd == RtabmapEventCmd::kCmdPublishGraphFull)
		{
			ULOGGER_DEBUG("CMD_PUBLISH_GRAPH_FULL");
			ParametersMap param;
			param.insert(ParametersPair("optimized", uNumber2Str(rtabmapEvent->getInt())));
			pushNewState(kStatePublishingGraphFull, param);
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
	if(!_paused)
	{
		if(image.empty())
		{
			ULOGGER_ERROR("image empty !?");
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
