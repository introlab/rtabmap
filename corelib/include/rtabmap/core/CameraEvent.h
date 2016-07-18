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

#pragma once

#include <rtabmap/utilite/UEvent.h>
#include "rtabmap/core/SensorData.h"
#include "rtabmap/core/CameraInfo.h"

namespace rtabmap
{

class CameraEvent :
	public UEvent
{
public:
	enum Code {
		kCodeData,
		kCodeNoMoreImages
	};

public:
	CameraEvent(const cv::Mat & image, int seq=0, double stamp = 0.0, const std::string & cameraName = std::string()) :
		UEvent(kCodeData),
		data_(image, seq, stamp)
	{
		cameraInfo_.cameraName = cameraName;
	}

	CameraEvent() :
		UEvent(kCodeNoMoreImages)
	{
	}

	CameraEvent(const SensorData & data) :
		UEvent(kCodeData),
		data_(data)
	{
	}

	CameraEvent(const SensorData & data, const std::string & cameraName) :
		UEvent(kCodeData),
		data_(data)
	{
		cameraInfo_.cameraName = cameraName;
	}
	CameraEvent(const SensorData & data, const CameraInfo & cameraInfo) :
		UEvent(kCodeData),
		data_(data),
		cameraInfo_(cameraInfo)
	{
	}

	// Image or descriptors
	const SensorData & data() const {return data_;}
	const std::string & cameraName() const {return cameraInfo_.cameraName;}
	const CameraInfo & info() const {return cameraInfo_;}

	virtual ~CameraEvent() {}
	virtual std::string getClassName() const {return std::string("CameraEvent");}

private:
	SensorData data_;
	CameraInfo cameraInfo_;
};

} // namespace rtabmap
