/*
 * DBReader.h
 *
 *  Created on: 2012-06-13
 *      Author: mathieu
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
	void getNextImage(cv::Mat & image, cv::Mat & depth, cv::Mat & depth2d, float & depthConstant, Transform & localTransform, Transform & pose);

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
