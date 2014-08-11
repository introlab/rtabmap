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
			kCmdResetMemory,
			kCmdDumpMemory,
			kCmdDumpPrediction,
			kCmdGenerateDOTGraph, // params: path
			kCmdGenerateDOTLocalGraph, // params: path, id, margin
			kCmdGenerateTOROGraphLocal, // params: path, optimized
			kCmdGenerateTOROGraphGlobal, // params: path, optimized
			kCmdDeleteMemory, // params: path [optional]
			kCmdCleanDataBuffer,
			kCmdPublish3DMapLocal, // params: optimized
			kCmdPublish3DMapGlobal, // params: optimized
			kCmdPublishTOROGraphGlobal, // params: optimized
			kCmdPublishTOROGraphLocal, // params: optimized
			kCmdTriggerNewMap,
			kCmdPause};
public:
	RtabmapEventCmd(Cmd cmd) :
		UEvent(0),
		_cmd(cmd),
		_strValue(""),
		_intValue(0){}
	RtabmapEventCmd(Cmd cmd, int value) :
			UEvent(0),
			_cmd(cmd),
			_strValue(""),
			_intValue(value){}
	RtabmapEventCmd(Cmd cmd, const std::string & value) :
			UEvent(0),
			_cmd(cmd),
			_strValue(value),
			_intValue(0){}
	RtabmapEventCmd(Cmd cmd, const std::string & strValue, int intValue) :
			UEvent(0),
			_cmd(cmd),
			_strValue(strValue),
			_intValue(intValue){}

	virtual ~RtabmapEventCmd() {}
	Cmd getCmd() const {return _cmd;}

	void setStr(const std::string & str) {_strValue = str;}
	const std::string & getStr() const {return _strValue;}

	void setInt(int v) {_intValue = v;}
	int getInt() const {return _intValue;}

	virtual std::string getClassName() const {return std::string("RtabmapEventCmd");}

private:
	Cmd _cmd;
	std::string _strValue;
	int _intValue;
};

class RtabmapEventInit : public UEvent
{
public:
	enum dummy {d}; // Hack, to fix Eclipse complaining about not defined Status enum ?!
	enum Status {
		kInitializing,
		kInitialized,
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
			const std::map<int, std::vector<unsigned char> > & images,
			const std::map<int, std::vector<unsigned char> > & depths,
			const std::map<int, std::vector<unsigned char> > & depths2d,
			const std::map<int, float> & depthFxs,
			const std::map<int, float> & depthFys,
			const std::map<int, float> & depthCxs,
			const std::map<int, float> & depthCys,
			const std::map<int, Transform> & localTransforms,
			const std::map<int, Transform> & poses,
			const std::multimap<int, Link> & constraints) :
		UEvent(0),
		_images(images),
		_depths(depths),
		_depths2d(depths2d),
		_depthFxs(depthFxs),
		_depthFys(depthFys),
		_depthCxs(depthCxs),
		_depthCys(depthCys),
		_localTransforms(localTransforms),
		_poses(poses),
		_constraints(constraints)
	{}

	virtual ~RtabmapEvent3DMap() {}

	const std::map<int, std::vector<unsigned char> > & getImages() const {return _images;}
	const std::map<int, std::vector<unsigned char> > & getDepths() const {return _depths;}
	const std::map<int, std::vector<unsigned char> > & getDepths2d() const {return _depths2d;}
	const std::map<int, float> & getDepthFxs() const {return _depthFxs;}
	const std::map<int, float> & getDepthFys() const {return _depthFys;}
	const std::map<int, float> & getDepthCxs() const {return _depthCxs;}
	const std::map<int, float> & getDepthCys() const {return _depthCys;}
	const std::map<int, Transform> & getLocalTransforms() const {return _localTransforms;}
	const std::map<int, Transform> & getPoses() const {return _poses;}
	const std::multimap<int, Link> & getConstraints() const {return _constraints;}

	virtual std::string getClassName() const {return std::string("RtabmapEvent3DMap");}

private:
	std::map<int, std::vector<unsigned char> > _images;
	std::map<int, std::vector<unsigned char> > _depths;
	std::map<int, std::vector<unsigned char> > _depths2d;
	std::map<int, float> _depthFxs;
	std::map<int, float> _depthFys;
	std::map<int, float> _depthCxs;
	std::map<int, float> _depthCys;
	std::map<int, Transform> _localTransforms;
	std::map<int, Transform> _poses;
	std::multimap<int, Link> _constraints;
};

} // namespace rtabmap

#endif /* RTABMAPEVENT_H_ */
