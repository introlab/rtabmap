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
			kCmdGenerateGraph,
			kCmdGenerateLocalGraph,
			kCmdDeleteMemory,
			kCmdCleanSensorsBuffer};
public:
	RtabmapEventCmd(Cmd cmd) :
		UEvent(0),
		_cmd(cmd) {}

	virtual ~RtabmapEventCmd() {}
	Cmd getCmd() const {return _cmd;}
	void setStr(const std::string & str) {_str = str;}
	const std::string & getStr() const {return _str;}
	virtual std::string getClassName() const {return std::string("RtabmapEventCmd");}

private:
	Cmd _cmd;
	std::string _str;
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

} // namespace rtabmap

#endif /* RTABMAPEVENT_H_ */
