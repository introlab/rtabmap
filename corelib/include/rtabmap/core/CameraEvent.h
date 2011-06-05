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

#ifndef CAMERAEVENT_H_
#define CAMERAEVENT_H_

#include "rtabmap/core/RtabmapExp.h" // DLL export/import defines

#include "utilite/UEvent.h"
#include <opencv2/core/core.hpp>

namespace rtabmap
{

class RTABMAP_EXP CameraEvent :
	public UEvent
{
public:
	enum Code {
		kCodeCtrl,
		kCodeNoMoreImages
	};

	enum Cmd {
		kCmdUndefined,
		kCmdPause,
		kCmdChangeParam
	};

public:
	CameraEvent(Cmd command, float imageRate = -1, bool autoRestart = false) :
		UEvent(kCodeCtrl),
		_command(command),
		_imageRate(imageRate),
		_autoRestart(autoRestart)
	{
	}

	CameraEvent() :
		UEvent(kCodeNoMoreImages),
		_command(kCmdUndefined),
		_imageRate(-1)
	{

	}

	virtual ~CameraEvent() {}

	virtual std::string getClassName() const {return std::string("CameraEvent");}

	const Cmd & getCommand() const {return _command;}
	float getImageRate() const {return _imageRate;}
	bool getAutoRestart() const {return _autoRestart;}

private:
	Cmd _command;
	float _imageRate;
	bool _autoRestart;
};

} // namespace rtabmap

#endif /* CAMERAEVENT_H_ */
