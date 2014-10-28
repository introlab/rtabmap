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

#ifndef RTABMAPEVENT_H_
#define RTABMAPEVENT_H_



#include <rtabmap/utilite/UEvent.h>
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
			kCmdInit,
			kCmdResetMemory,
			kCmdClose,
			kCmdDumpMemory,
			kCmdDumpPrediction,
			kCmdGenerateDOTGraph, // params: path
			kCmdGenerateDOTLocalGraph, // params: path, id, margin
			kCmdGenerateTOROGraphLocal, // params: path, optimized
			kCmdGenerateTOROGraphGlobal, // params: path, optimized
			kCmdCleanDataBuffer,
			kCmdPublish3DMapLocal, // params: optimized
			kCmdPublish3DMapGlobal, // params: optimized
			kCmdPublishTOROGraphGlobal, // params: optimized
			kCmdPublishTOROGraphLocal, // params: optimized
			kCmdTriggerNewMap,
			kCmdPause};
public:
	RtabmapEventCmd(Cmd cmd, const std::string & strValue = "", int intValue = 0, const ParametersMap & parameters = ParametersMap()) :
			UEvent(0),
			_cmd(cmd),
			_strValue(strValue),
			_intValue(intValue),
			_parameters(parameters){}

	virtual ~RtabmapEventCmd() {}
	Cmd getCmd() const {return _cmd;}

	void setStr(const std::string & str) {_strValue = str;}
	const std::string & getStr() const {return _strValue;}

	void setInt(int v) {_intValue = v;}
	int getInt() const {return _intValue;}

	const ParametersMap & getParameters() const {return _parameters;}

	virtual std::string getClassName() const {return std::string("RtabmapEventCmd");}

private:
	Cmd _cmd;
	std::string _strValue;
	int _intValue;
	ParametersMap _parameters;
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
			const std::multimap<int, Link> & constraints,
			const std::map<int, int> & mapIds) :
		UEvent(0),
		_signatures(signatures),
		_poses(poses),
		_constraints(constraints),
		_mapIds(mapIds)
	{}

	virtual ~RtabmapEvent3DMap() {}

	const std::map<int, Signature> & getSignatures() const {return _signatures;}
	const std::map<int, Transform> & getPoses() const {return _poses;}
	const std::multimap<int, Link> & getConstraints() const {return _constraints;}
	const std::map<int, int> & getMapIds() const {return _mapIds;}

	virtual std::string getClassName() const {return std::string("RtabmapEvent3DMap");}

private:
	std::map<int, Signature> _signatures;
	std::map<int, Transform> _poses;
	std::multimap<int, Link> _constraints;
	std::map<int, int> _mapIds;
};

} // namespace rtabmap

#endif /* RTABMAPEVENT_H_ */
