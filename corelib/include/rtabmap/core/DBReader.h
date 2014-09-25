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

#ifndef DBREADER_H_
#define DBREADER_H_

#include "rtabmap/core/RtabmapExp.h" // DLL export/import defines

#include <rtabmap/utilite/UThreadNode.h>
#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/utilite/UEventsSender.h>
#include <rtabmap/core/Transform.h>

#include <opencv2/core/core.hpp>

#include <set>

namespace rtabmap {

class DBDriver;

class RTABMAP_EXP DBReader : public UThreadNode, public UEventsSender {
public:
	DBReader(const std::string & databasePath,
			 float frameRate = 0.0f,
			 bool odometryIgnored = false,
			 float delayToStartSec = 0.0f);
	virtual ~DBReader();

	bool init(int startIndex=0);
	void setFrameRate(float frameRate);
	void getNextImage(cv::Mat & image,
			cv::Mat & depth,
			cv::Mat & depth2d,
			float & fx, float & fy,
			float & cx, float & cy,
			Transform & localTransform,
			Transform & pose,
			int & seq);

protected:
	virtual void mainLoopBegin();
	virtual void mainLoop();

private:
	std::string _path;
	float _frameRate;
	bool _odometryIgnored;
	float _delayToStartSec;

	DBDriver * _dbDriver;
	UTimer _timer;
	std::set<int> _ids;
	std::set<int>::iterator _currentId;
};

} /* namespace rtabmap */
#endif /* DBREADER_H_ */
