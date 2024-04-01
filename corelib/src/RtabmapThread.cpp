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
#include "rtabmap/core/RtabmapThread.h"
#include "rtabmap/core/RtabmapEvent.h"
#include "rtabmap/core/Camera.h"
#include "rtabmap/core/CameraEvent.h"
#include "rtabmap/core/ParamEvent.h"
#include "rtabmap/core/OdometryEvent.h"
#include "rtabmap/core/UserDataEvent.h"
#include "rtabmap/core/Memory.h"

#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UEventsManager.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/utilite/UConversion.h>

namespace rtabmap {

RtabmapThread::RtabmapThread(Rtabmap * rtabmap) :
		_dataBufferMaxSize(Parameters::defaultRtabmapImageBufferSize()),
		_rate(Parameters::defaultRtabmapDetectionRate()),
		_createIntermediateNodes(Parameters::defaultRtabmapCreateIntermediateNodes()),
		_frameRateTimer(new UTimer()),
		_previousStamp(0.0),
		_rtabmap(rtabmap),
		_paused(false),
		lastPose_(Transform::getIdentity())

{
	UASSERT(rtabmap != 0);
}

RtabmapThread::~RtabmapThread()
{
	UEventsManager::removeHandler(this);

	close(true);

	delete _frameRateTimer;
}

void RtabmapThread::pushNewState(State newState, const RtabmapEventCmd & cmdEvent)
{
	ULOGGER_DEBUG("to %d", newState);

	_stateMutex.lock();
	{
		_state.push(newState);
		_stateParam.push(cmdEvent);
	}
	_stateMutex.unlock();

	_dataAdded.release();
}

void RtabmapThread::clearBufferedData()
{
	_dataMutex.lock();
	{
		_dataBuffer.clear();
		_newMapEvents.clear();
		lastPose_.setIdentity();
		covariance_ = cv::Mat();
		_previousStamp = 0;
	}
	_dataMutex.unlock();

	_userDataMutex.lock();
	{
		_userData = cv::Mat();
	}
	_userDataMutex.unlock();
}

void RtabmapThread::setDetectorRate(float rate)
{
	UASSERT(rate >= 0.0f);
	_rate = rate;
}

void RtabmapThread::setDataBufferSize(unsigned int size)
{
	_dataBufferMaxSize = size;
}

void RtabmapThread::createIntermediateNodes(bool enabled)
{
	_createIntermediateNodes = enabled;
}

void RtabmapThread::close(bool databaseSaved, const std::string & ouputDatabasePath)
{
	this->join(true);
	if(_rtabmap)
	{
		_rtabmap->close(databaseSaved, ouputDatabasePath);
		delete _rtabmap;
		_rtabmap = 0;
	}
}

void RtabmapThread::publishMap(bool optimized, bool full, bool graphOnly) const
{
	UDEBUG("optimized=%s, full=%s, graphOnly=%s", optimized?"true":"false", full?"true":"false", graphOnly?"true":"false");
	if(_rtabmap)
	{
		std::map<int, Signature> signatures;
		std::map<int, Transform> poses;
		std::multimap<int, Link> constraints;
		std::map<int, int> mapIds;
		std::map<int, double> stamps;
		std::map<int, std::string> labels;
		std::map<int, std::vector<unsigned char> > userDatas;

		_rtabmap->getGraph(poses,
				constraints,
				optimized,
				full,
				&signatures,
				!graphOnly,
				!graphOnly,
				!graphOnly,
				!graphOnly);

		this->post(new RtabmapEvent3DMap(
				signatures,
				poses,
				constraints));
	}
	else
	{
		UERROR("Rtabmap is null!");
	}
}

void RtabmapThread::mainLoopBegin()
{
	ULogger::registerCurrentThread("Rtabmap");
	if(_rtabmap == 0)
	{
		UERROR("Cannot start rtabmap thread if no rtabmap object is set! Stopping the thread...");
		this->kill();
	}
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
	RtabmapEventCmd cmdEvent(RtabmapEventCmd::kCmdUndef);

	_stateMutex.lock();
	{
		if(!_state.empty() && !_stateParam.empty())
		{
			state = _state.front();
			_state.pop();
			cmdEvent = _stateParam.front();
			_stateParam.pop();
		}
	}
	_stateMutex.unlock();

	int id = 0;
	cv::Mat userData;
	UTimer timer;
	std::string str;
	RtabmapEventCmd::Cmd cmd = cmdEvent.getCmd();
	switch(state)
	{
	case kStateDetecting:
		this->process();
		break;
	case kStateProcessCommand:
		if(cmd == RtabmapEventCmd::kCmdInit)
		{
			ULOGGER_DEBUG("CMD_INIT");
			Parameters::parse(cmdEvent.getParameters(), Parameters::kRtabmapImageBufferSize(), _dataBufferMaxSize);
			Parameters::parse(cmdEvent.getParameters(), Parameters::kRtabmapDetectionRate(), _rate);
			Parameters::parse(cmdEvent.getParameters(), Parameters::kRtabmapCreateIntermediateNodes(), _createIntermediateNodes);
			UASSERT(_rate >= 0.0f);
			_rtabmap->init(cmdEvent.getParameters(), cmdEvent.value1().toStr());
		}
		else if(cmd == RtabmapEventCmd::kCmdClose)
		{
			ULOGGER_DEBUG("CMD_CLOSE");
			if(_dataBuffer.size())
			{
				UWARN("Closing... %d data still buffered! They will be cleared.", (int)_dataBuffer.size());
				this->clearBufferedData();
			}
			_rtabmap->close(cmdEvent.value1().toBool(), cmdEvent.value2().toStr());
		}
		else if(cmd == RtabmapEventCmd::kCmdUpdateParams)
		{
			ULOGGER_DEBUG("CMD_UPDATE_PARAMS");
			Parameters::parse(cmdEvent.getParameters(), Parameters::kRtabmapImageBufferSize(), _dataBufferMaxSize);
			Parameters::parse(cmdEvent.getParameters(), Parameters::kRtabmapDetectionRate(), _rate);
			Parameters::parse(cmdEvent.getParameters(), Parameters::kRtabmapCreateIntermediateNodes(), _createIntermediateNodes);
			UASSERT(_rate >= 0.0f);
			_rtabmap->parseParameters(cmdEvent.getParameters());
			break;
		}
		else if(cmd == RtabmapEventCmd::kCmdResetMemory)
		{
			ULOGGER_DEBUG("CMD_RESET_MEMORY");
			_rtabmap->resetMemory();
			this->clearBufferedData();
		}
		else if(cmd == RtabmapEventCmd::kCmdDumpMemory)
		{
			ULOGGER_DEBUG("CMD_DUMP_MEMORY");
			_rtabmap->dumpData();
		}
		else if(cmd == RtabmapEventCmd::kCmdDumpPrediction)
		{
			ULOGGER_DEBUG("CMD_DUMP_PREDICTION");
			_rtabmap->dumpPrediction();
		}
		else if(cmd == RtabmapEventCmd::kCmdGenerateDOTGraph)
		{
			ULOGGER_DEBUG("CMD_GENERATE_DOT_GRAPH");
			_rtabmap->generateDOTGraph(
					cmdEvent.value2().toStr(),
					cmdEvent.value1().toBool()?0:cmdEvent.value3().toInt(),
					cmdEvent.value1().toBool()?0:cmdEvent.value4().toInt());
		}
		else if(cmd == RtabmapEventCmd::kCmdExportPoses)
		{
			ULOGGER_DEBUG("CMD_EXPORT_POSES");
			_rtabmap->exportPoses(
					cmdEvent.value3().toStr(),
					cmdEvent.value2().toBool(),
					cmdEvent.value1().toBool(),
					cmdEvent.value4().toInt());

		}
		else if(cmd == RtabmapEventCmd::kCmdCleanDataBuffer)
		{
			ULOGGER_DEBUG("CMD_CLEAN_DATA_BUFFER");
			this->clearBufferedData();
		}
		else if(cmd == RtabmapEventCmd::kCmdPublish3DMap)
		{
			ULOGGER_DEBUG("CMD_PUBLISH_MAP");
			this->publishMap(
					cmdEvent.value2().toBool(),
					cmdEvent.value1().toBool(),
					cmdEvent.value3().toBool());
		}
		else if(cmd == RtabmapEventCmd::kCmdTriggerNewMap)
		{
			ULOGGER_DEBUG("CMD_TRIGGER_NEW_MAP");
			_rtabmap->triggerNewMap();
		}
		else if(cmd == RtabmapEventCmd::kCmdPause)
		{
			ULOGGER_DEBUG("CMD_PAUSE");
			_paused = !_paused;
		}
		else if(cmd == RtabmapEventCmd::kCmdGoal)
		{
			ULOGGER_DEBUG("CMD_GOAL");
			if(cmdEvent.value1().isStr() && !cmdEvent.value1().toStr().empty() && _rtabmap->getMemory())
			{
				id = _rtabmap->getMemory()->getSignatureIdByLabel(cmdEvent.value1().toStr());
				if(id <= 0)
				{
					UERROR("Failed to find a node with label \"%s\".", cmdEvent.value1().toStr().c_str());
				}
			}
			else if(cmdEvent.value1().isInt() || cmdEvent.value1().isUInt())
			{
				id = cmdEvent.value1().toInt();
			}

			if(id < 0)
			{
				UERROR("Failed to set a goal. ID (%d) should be positive > 0", id);
			}
			timer.start();
			if(id > 0 && !_rtabmap->computePath(id, true))
			{
				UERROR("Failed to compute a path to goal %d.", id);
			}
			this->post(new RtabmapGlobalPathEvent(
					id,
					cmdEvent.value1().isStr()?cmdEvent.value1().toStr():"",
					_rtabmap->getPath(),
					timer.elapsed()));
			break;
		}
		else if(cmd == RtabmapEventCmd::kCmdCancelGoal)
		{
			ULOGGER_DEBUG("CMD_CANCEL_GOAL");
			_rtabmap->clearPath(0);
		}
		else if(cmd == RtabmapEventCmd::kCmdLabel)
		{
			ULOGGER_DEBUG("CMD_LABEL");
			if(!_rtabmap->labelLocation(cmdEvent.value2().toInt(), cmdEvent.value1().toStr()))
			{
				this->post(new RtabmapLabelErrorEvent(cmdEvent.value2().toInt(), cmdEvent.value1().toStr()));
			}
		}
		else if(cmd == RtabmapEventCmd::kCmdRemoveLabel)
		{
			ULOGGER_DEBUG("CMD_REMOVE_LABEL");
			id = _rtabmap->getMemory()->getSignatureIdByLabel(cmdEvent.value1().toStr(), true);
			if(id <= 0 || !_rtabmap->labelLocation(id, ""))
			{
				this->post(new RtabmapLabelErrorEvent(id, cmdEvent.value1().toStr()));
			}
		}
		else if(cmd == RtabmapEventCmd::kCmdRepublishData)
		{
			ULOGGER_DEBUG("CMD_REPUBLISH_DATA");
			_rtabmap->addNodesToRepublish(cmdEvent.value1().toIntArray());
		}
		else
		{
			UWARN("Cmd %d unknown!", cmd);
		}
		break;
	default:
		UFATAL("Invalid state !?!?");
		break;
	}
}


bool RtabmapThread::handleEvent(UEvent* event)
{
	if(this->isRunning())
	{
		if(event->getClassName().compare("IMUEvent") == 0)
		{
			// IMU events are published at high frequency, early exit
			return false;
		}
		else if(event->getClassName().compare("CameraEvent") == 0)
		{
			UDEBUG("CameraEvent");
			CameraEvent * e = (CameraEvent*)event;
			if(e->getCode() == CameraEvent::kCodeData)
			{
				if (_rtabmap->isRGBDMode())
				{
					if (!e->info().odomPose.isNull() || (_rtabmap->getMemory() && !_rtabmap->getMemory()->isIncremental()))
					{
						OdometryInfo infoCov;
						infoCov.reg.covariance = e->info().odomCovariance;
						if(e->info().odomVelocity.size() == 6)
						{
							infoCov.transform = Transform(
									e->info().odomVelocity[0],
									e->info().odomVelocity[1],
									e->info().odomVelocity[2],
									e->info().odomVelocity[3],
									e->info().odomVelocity[4],
									e->info().odomVelocity[5]);
							infoCov.interval = 1.0;
						}
						this->addData(OdometryEvent(e->data(), e->info().odomPose, infoCov));
					}
					else
					{
						lastPose_.setNull();
					}
				}
				else
				{
					OdometryInfo infoCov;
					infoCov.reg.covariance = e->info().odomCovariance;
					this->addData(OdometryEvent(e->data(), e->info().odomPose, infoCov));
				}

			}
		}
		else if(event->getClassName().compare("OdometryEvent") == 0)
		{
			UDEBUG("OdometryEvent");
			OdometryEvent * e = (OdometryEvent*)event;
			if(!e->pose().isNull() || (_rtabmap->getMemory() && !_rtabmap->getMemory()->isIncremental()))
			{
				this->addData(*e);
			}
			else
			{
				lastPose_.setNull();
			}
		}
		else if(event->getClassName().compare("UserDataEvent") == 0)
		{
			if(!_paused)
			{
				UDEBUG("UserDataEvent");
				bool updated = false;
				UserDataEvent * e = (UserDataEvent*)event;
				_userDataMutex.lock();
				if(!e->data().empty())
				{
					updated = !_userData.empty();
					_userData = e->data();
				}
				_userDataMutex.unlock();
				if(updated)
				{
					UWARN("New user data received before the last one was processed... replacing "
						"user data with this new one. Note that UserDataEvent should be used only "
						"if the rate of UserDataEvent is lower than RTAB-Map's detection rate (%f Hz).", _rate);
				}
			}
		}
		else if(event->getClassName().compare("RtabmapEventCmd") == 0)
		{
			RtabmapEventCmd * rtabmapEvent = (RtabmapEventCmd*)event;
			pushNewState(kStateProcessCommand, *rtabmapEvent);
		}
		else if(event->getClassName().compare("ParamEvent") == 0)
		{
			ULOGGER_DEBUG("changing parameters");
			pushNewState(kStateProcessCommand, RtabmapEventCmd(RtabmapEventCmd::kCmdUpdateParams, ((ParamEvent*)event)->getParameters()));
		}
	}
	return false;
}

//============================================================
// MAIN LOOP
//============================================================
void RtabmapThread::process()
{
	OdometryEvent data;
	if(_state.empty() && getData(data))
	{
		if(_rtabmap->getMemory())
		{
			bool wasPlanning = _rtabmap->getPath().size()>0;
			if(_rtabmap->process(data.data(), data.pose(), data.covariance(), data.velocity()))
			{
				Statistics stats = _rtabmap->getStatistics();
				stats.addStatistic(Statistics::kMemoryImages_buffered(), (float)_dataBuffer.size());
				ULOGGER_DEBUG("posting statistics_ event...");
				this->post(new RtabmapEvent(stats));

				if(wasPlanning && _rtabmap->getPath().size() == 0)
				{
					// Goal reached or failed
					this->post(new RtabmapGoalStatusEvent(_rtabmap->getPathStatus()));
				}
			}
		}
		else
		{
			UERROR("RTAB-Map is not initialized! Ignoring received data...");
		}
	}
}

void RtabmapThread::addData(const OdometryEvent & odomEvent)
{
	if(!_paused)
	{
		UScopeMutex scopeMutex(_dataMutex);

		bool ignoreFrame = false;
		if(_rate>0.0f)
		{
			if((_previousStamp>=0.0 && odomEvent.data().stamp()>_previousStamp && odomEvent.data().stamp() - _previousStamp < 1.0f/_rate) ||
				((_previousStamp<=0.0 || odomEvent.data().stamp()<=_previousStamp) && _frameRateTimer->getElapsedTime() < 1.0f/_rate))
			{
				ignoreFrame = true;
			}
		}
		if(!lastPose_.isIdentity() &&
						(odomEvent.pose().isIdentity() ||
						odomEvent.info().reg.covariance.at<double>(0,0)>=9999))
		{
			if(odomEvent.pose().isIdentity())
			{
				UWARN("Odometry is reset (identity pose detected). Increment map id! (stamp=%fs)", odomEvent.data().stamp());
			}
			else
			{
				UWARN("Odometry is reset (high variance (%f >=9999 detected, stamp=%fs). Increment map id!", odomEvent.info().reg.covariance.at<double>(0,0), odomEvent.data().stamp());
			}
			_newMapEvents.push_back(odomEvent.data().stamp());
			covariance_ = cv::Mat();
		}

		if(uIsFinite(odomEvent.info().reg.covariance.at<double>(0,0)) &&
			odomEvent.info().reg.covariance.at<double>(0,0) != 1.0 &&
			odomEvent.info().reg.covariance.at<double>(0,0)>0.0)
		{
			// Use largest covariance error (to be independent of the odometry frame rate)
			if(covariance_.empty() || odomEvent.info().reg.covariance.at<double>(0,0) > covariance_.at<double>(0,0))
			{
				covariance_ = odomEvent.info().reg.covariance;
			}
		}

		if(ignoreFrame && !_createIntermediateNodes)
		{
			return;
		}
		else if(!ignoreFrame)
		{
			_frameRateTimer->start();
			_previousStamp = odomEvent.data().stamp();
		}

		lastPose_ = odomEvent.pose();

		bool notify = true;

		if(covariance_.empty())
		{
			covariance_ = cv::Mat::eye(6,6,CV_64FC1);
		}
		OdometryInfo odomInfo = odomEvent.info().copyWithoutData();
		odomInfo.reg.covariance = covariance_;
		if(ignoreFrame)
		{
			// set negative id so rtabmap will detect it as an intermediate node
			SensorData tmp = odomEvent.data();
			tmp.setId(-1);
			tmp.setFeatures(std::vector<cv::KeyPoint>(), std::vector<cv::Point3f>(), cv::Mat());// remove features
			_dataBuffer.push_back(OdometryEvent(tmp, odomEvent.pose(), odomInfo));
		}
		else
		{
			_dataBuffer.push_back(OdometryEvent(odomEvent.data(), odomEvent.pose(), odomInfo));
		}
		UINFO("Added data %d", odomEvent.data().id());

		covariance_ = cv::Mat();
		while(_dataBufferMaxSize > 0 && _dataBuffer.size() > _dataBufferMaxSize)
		{
			if(_rate > 0.0f)
			{
				ULOGGER_WARN("Data buffer is full, the oldest data is removed to add the new one.");
			}
			_dataBuffer.pop_front();
			notify = false;
		}

		if(notify)
		{
			_dataAdded.release();
		}
	}
}

bool RtabmapThread::getData(OdometryEvent & data)
{
	ULOGGER_DEBUG("");

	ULOGGER_INFO("waiting for data");
	_dataAdded.acquire();
	ULOGGER_INFO("wake-up");

	bool dataFilled = false;
	bool triggerNewMap = false;
	_dataMutex.lock();
	{
		if(_state.empty() && !_dataBuffer.empty())
		{
			data = _dataBuffer.front();
			_dataBuffer.pop_front();

			_userDataMutex.lock();
			{
				if(!_userData.empty())
				{
					data.data().setUserData(_userData);
					_userData = cv::Mat();
				}
			}
			_userDataMutex.unlock();

			while(_newMapEvents.size() && _newMapEvents.front() <= data.data().stamp())
			{
				UWARN("Triggering new map %f<=%f...", _newMapEvents.front() , data.data().stamp());
				triggerNewMap = true;
				_newMapEvents.pop_front();
			}

			dataFilled = true;
		}
	}
	_dataMutex.unlock();

	if(triggerNewMap)
	{
		_rtabmap->triggerNewMap();
	}

	return dataFilled;
}

} /* namespace rtabmap */
