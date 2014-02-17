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
			const std::map<int, float> & depthConstants,
			const std::map<int, Transform> & localTransforms,
			const std::map<int, Transform> & poses,
			const std::multimap<int, Link> & constraints) :
		UEvent(0),
		_images(images),
		_depths(depths),
		_depths2d(depths2d),
		_depthConstants(depthConstants),
		_localTransforms(localTransforms),
		_poses(poses),
		_constraints(constraints)
	{}

	virtual ~RtabmapEvent3DMap() {}

	const std::map<int, std::vector<unsigned char> > & getImages() const {return _images;}
	const std::map<int, std::vector<unsigned char> > & getDepths() const {return _depths;}
	const std::map<int, std::vector<unsigned char> > & getDepths2d() const {return _depths2d;}
	const std::map<int, float> & getDepthConstants() const {return _depthConstants;}
	const std::map<int, Transform> & getLocalTransforms() const {return _localTransforms;}
	const std::map<int, Transform> & getPoses() const {return _poses;}
	const std::multimap<int, Link> & getConstraints() const {return _constraints;}

	virtual std::string getClassName() const {return std::string("RtabmapEvent3DMap");}

private:
	std::map<int, std::vector<unsigned char> > _images;
	std::map<int, std::vector<unsigned char> > _depths;
	std::map<int, std::vector<unsigned char> > _depths2d;
	std::map<int, float> _depthConstants;
	std::map<int, Transform> _localTransforms;
	std::map<int, Transform> _poses;
	std::multimap<int, Link> _constraints;
};

} // namespace rtabmap

#endif /* RTABMAPEVENT_H_ */
