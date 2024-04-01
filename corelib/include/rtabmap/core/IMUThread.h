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

#include "rtabmap/core/rtabmap_core_export.h" // DLL export/import defines

#include <rtabmap/core/Transform.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/utilite/UThread.h>
#include <rtabmap/utilite/UEventsSender.h>
#include <rtabmap/utilite/UTimer.h>

#include <fstream>

namespace rtabmap
{

class IMUFilter;

/**
 * Class IMUThread
 *
 */
class RTABMAP_CORE_EXPORT IMUThread :
	public UThread,
	public UEventsSender
{
public:
	IMUThread(int rate, const Transform & localTransform);
	virtual ~IMUThread();

	bool init(const std::string & path);
	void setRate(int rate);
	void enableIMUFiltering(int filteringStrategy=1, const ParametersMap & parameters = ParametersMap(), bool baseFrameConversion = false);
	void disableIMUFiltering();

private:
	virtual void mainLoopBegin();
	virtual void mainLoop();

private:
	int rate_;
	Transform localTransform_;
	std::ifstream imuFile_;
	UTimer frameRateTimer_;
	double captureDelay_;
	double previousStamp_;
	IMUFilter * _imuFilter;
	bool _imuBaseFrameConversion;
};

} // namespace rtabmap
