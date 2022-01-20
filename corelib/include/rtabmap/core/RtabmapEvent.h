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

#ifndef RTABMAPEVENT_H_
#define RTABMAPEVENT_H_



#include <rtabmap/utilite/UEvent.h>
#include <rtabmap/utilite/UVariant.h>
#include "rtabmap/core/Statistics.h"
#include "rtabmap/core/Parameters.h"

namespace rtabmap
{

////////// The RtabmapEvent class //////////////
class RtabmapEvent : public UEvent
{
public:
	RtabmapEvent(const Statistics & stats) :
		UEvent(0),
		_stats(stats) {}

	virtual ~RtabmapEvent() {}
	const Statistics & getStats() const {return _stats;}
	virtual std::string getClassName() const {return std::string("RtabmapEvent");}

private:
	Statistics _stats;
};

class RtabmapEventCmd : public UEvent
{
public:
	enum dummy {d}; // Hack, to fix Eclipse complaining about not defined Cmd enum ?!
	enum Cmd {
			kCmdInit,             // params: [string] database path + ParametersMap
			kCmdResetMemory,
			kCmdClose,            // params: [bool] database saved (default true), [string] output database path (empty=use same database to save, only work when Db/Sqlite3InMemory=true)
			kCmdDumpMemory,
			kCmdDumpPrediction,
			kCmdGenerateDOTGraph, // params: [bool] global, [string] path, if global=false: [int] id, [int] margin
			kCmdExportPoses,      // params: [bool] global, [bool] optimized, [string] path, [int] type (0=raw format, 1=RGBD-SLAM format, 2=KITTI format, 3=TORO, 4=g2o)
			kCmdCleanDataBuffer,
			kCmdPublish3DMap,     // params: [bool] global, [bool] optimized, [bool] graphOnly
			kCmdTriggerNewMap,
			kCmdPause,
			kCmdResume,
			kCmdGoal,             // params: [string] label or [int] location ID
			kCmdCancelGoal,
			kCmdLabel,            // params: [string] label, [int] location ID
			kCmdRemoveLabel       // params: [string] label
	};
public:
	RtabmapEventCmd(Cmd cmd, const ParametersMap & parameters = ParametersMap()) :
			UEvent(0),
			cmd_(cmd),
			parameters_(parameters){}
	RtabmapEventCmd(Cmd cmd, const UVariant & value1, const ParametersMap & parameters = ParametersMap()) :
			UEvent(0),
			cmd_(cmd),
			value1_(value1),
			parameters_(parameters){}
	RtabmapEventCmd(Cmd cmd, const UVariant & value1, const UVariant & value2, const ParametersMap & parameters = ParametersMap()) :
			UEvent(0),
			cmd_(cmd),
			value1_(value1),
			value2_(value2),
			parameters_(parameters){}
	RtabmapEventCmd(Cmd cmd, const UVariant & value1, const UVariant & value2, const UVariant & value3, const ParametersMap & parameters = ParametersMap()) :
			UEvent(0),
			cmd_(cmd),
			value1_(value1),
			value2_(value2),
			value3_(value3),
			parameters_(parameters){}
	RtabmapEventCmd(Cmd cmd, const UVariant & value1, const UVariant & value2, const UVariant & value3, const UVariant & value4, const ParametersMap & parameters = ParametersMap()) :
			UEvent(0),
			cmd_(cmd),
			value1_(value1),
			value2_(value2),
			value3_(value3),
			value4_(value4),
			parameters_(parameters){}

	virtual ~RtabmapEventCmd() {}
	Cmd getCmd() const {return cmd_;}

	const UVariant & value1() const {return value1_;}
	const UVariant & value2() const {return value2_;}
	const UVariant & value3() const {return value3_;}
	const UVariant & value4() const {return value4_;}

	const ParametersMap & getParameters() const {return parameters_;}

	virtual std::string getClassName() const {return std::string("RtabmapEventCmd");}

private:
	Cmd cmd_;
	UVariant value1_;
	UVariant value2_;
	UVariant value3_;
	UVariant value4_;
	ParametersMap parameters_;
};

class RtabmapEventInit : public UEvent
{
public:
	enum dummy {d}; // Hack, to fix Eclipse complaining about not defined Status enum ?!
	enum Status {
		kInitializing,
		kInitialized,
		kClosing,
		kClosed,
		kInfo,
		kError
	};

public:
	RtabmapEventInit(Status status, const std::string & info = std::string()) :
		UEvent(0),
		_status(status),
		_info(info)
	{}

	// for convenience
	RtabmapEventInit(const std::string & info) :
		UEvent(0),
		_status(kInfo),
		_info(info)
	{}

	Status getStatus() const {return _status;}
	const std::string & getInfo() const {return _info;}

	virtual ~RtabmapEventInit() {}
	virtual std::string getClassName() const {return std::string("RtabmapEventInit");}
private:
	Status _status;
	std::string _info; // "Loading signatures", "Loading words" ...
};

class RtabmapEvent3DMap : public UEvent
{
public:
	RtabmapEvent3DMap(int codeError = 0):
		UEvent(codeError){}
	RtabmapEvent3DMap(
			const std::map<int, Signature> & signatures,
			const std::map<int, Transform> & poses,
			const std::multimap<int, Link> & constraints) :
		UEvent(0),
		_signatures(signatures),
		_poses(poses),
		_constraints(constraints)
	{}

	virtual ~RtabmapEvent3DMap() {}

	const std::map<int, Signature> & getSignatures() const {return _signatures;}
	const std::map<int, Transform> & getPoses() const {return _poses;}
	const std::multimap<int, Link> & getConstraints() const {return _constraints;}

	virtual std::string getClassName() const {return std::string("RtabmapEvent3DMap");}

private:
	std::map<int, Signature> _signatures;
	std::map<int, Transform> _poses;
	std::multimap<int, Link> _constraints;
};

class RtabmapGlobalPathEvent : public UEvent
{
public:
	RtabmapGlobalPathEvent():
		UEvent(0),
		_planningTime(0.0) {}
	RtabmapGlobalPathEvent(
			int goalId,
			const std::vector<std::pair<int, Transform> > & poses,
			double planningTime) :
				UEvent(goalId),
				_poses(poses),
				_planningTime(planningTime) {}
	RtabmapGlobalPathEvent(
			int goalId,
			const std::string & goalLabel,
			const std::vector<std::pair<int, Transform> > & poses,
			double planningTime) :
				UEvent(goalId),
				_goalLabel(goalLabel),
				_poses(poses),
				_planningTime(planningTime) {}

	virtual ~RtabmapGlobalPathEvent() {}
	int getGoal() const {return this->getCode();}
	const std::string & getGoalLabel() const {return _goalLabel;}
	double getPlanningTime() const {return _planningTime;}
	const std::vector<std::pair<int, Transform> > & getPoses() const {return _poses;}
	virtual std::string getClassName() const {return std::string("RtabmapGlobalPathEvent");}

private:
	std::string _goalLabel;
	std::vector<std::pair<int, Transform> > _poses;
	double _planningTime;
};

class RtabmapLabelErrorEvent : public UEvent
{
public:
	RtabmapLabelErrorEvent(int id, const std::string & label):
		UEvent(id),
		_label(label){}

	virtual ~RtabmapLabelErrorEvent() {}
	int id() const {return this->getCode();}
	const std::string & label() const {return _label;}
	virtual std::string getClassName() const {return std::string("RtabmapLabelErrorEvent");}

private:
	std::string _label;
};

class RtabmapGoalStatusEvent : public UEvent
{
public:
	RtabmapGoalStatusEvent(int status):
		UEvent(status){}

	virtual ~RtabmapGoalStatusEvent() {}
	virtual std::string getClassName() const {return std::string("RtabmapGoalStatusEvent");}
};

} // namespace rtabmap

#endif /* RTABMAPEVENT_H_ */
