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

#ifndef ODOMETRYEVENT_H_
#define ODOMETRYEVENT_H_

#include "rtabmap/utilite/UEvent.h"
#include "rtabmap/utilite/ULogger.h"
#include "rtabmap/utilite/UMath.h"
#include "rtabmap/core/SensorData.h"
#include "rtabmap/core/OdometryInfo.h"

namespace rtabmap {

class OdometryEvent : public UEvent
{
public:
	OdometryEvent()
	{
		_info.reg.covariance = cv::Mat::eye(6,6,CV_64FC1);
	}
	OdometryEvent(
		const SensorData & data,
		const Transform & pose,
		const OdometryInfo & info = OdometryInfo()) :
			_data(data),
			_pose(pose),
			_info(info)
	{
		if(_info.reg.covariance.empty())
		{
			_info.reg.covariance = cv::Mat::eye(6,6,CV_64FC1);
		}
		UASSERT(_info.reg.covariance.cols == 6 && _info.reg.covariance.rows == 6 && _info.reg.covariance.type() == CV_64FC1);
		UASSERT_MSG(uIsFinite(_info.reg.covariance.at<double>(0,0)) && _info.reg.covariance.at<double>(0,0)>0, "Transitional variance should not be null! (set to 1 if unknown)");
		UASSERT_MSG(uIsFinite(_info.reg.covariance.at<double>(1,1)) && _info.reg.covariance.at<double>(1,1)>0, "Transitional variance should not be null! (set to 1 if unknown)");
		UASSERT_MSG(uIsFinite(_info.reg.covariance.at<double>(2,2)) && _info.reg.covariance.at<double>(2,2)>0, "Transitional variance should not be null! (set to 1 if unknown)");
		UASSERT_MSG(uIsFinite(_info.reg.covariance.at<double>(3,3)) && _info.reg.covariance.at<double>(3,3)>0, "Rotational variance should not be null! (set to 1 if unknown)");
		UASSERT_MSG(uIsFinite(_info.reg.covariance.at<double>(4,4)) && _info.reg.covariance.at<double>(4,4)>0, "Rotational variance should not be null! (set to 1 if unknown)");
		UASSERT_MSG(uIsFinite(_info.reg.covariance.at<double>(5,5)) && _info.reg.covariance.at<double>(5,5)>0, "Rotational variance should not be null! (set to 1 if unknown)");
	}
	virtual ~OdometryEvent() {}
	virtual std::string getClassName() const {return "OdometryEvent";}

	SensorData & data() {return _data;}
	const SensorData & data() const {return _data;}
	const Transform & pose() const {return _pose;}
	const cv::Mat & covariance() const {return _info.reg.covariance;}
	std::vector<float> velocity() const {
		if(_info.interval>0.0)
		{
			std::vector<float> velocity(6,0);
			float x,y,z,roll,pitch,yaw;
			_info.transform.getTranslationAndEulerAngles(x,y,z,roll,pitch,yaw);
			velocity[0] = x/_info.interval;
			velocity[1] = y/_info.interval;
			velocity[2] = z/_info.interval;
			velocity[3] = roll/_info.interval;
			velocity[4] = pitch/_info.interval;
			velocity[5] = yaw/_info.interval;
			return velocity;
		}
		return std::vector<float>();
	}
	const OdometryInfo & info() const {return _info;}

private:
	SensorData _data;
	Transform _pose;
	OdometryInfo _info;
};

class OdometryResetEvent : public UEvent
{
public:
	OdometryResetEvent(const Transform & pose = Transform::getIdentity()){_pose = pose;}
	virtual ~OdometryResetEvent() {}
	virtual std::string getClassName() const {return "OdometryResetEvent";}
	const Transform & getPose() const {return _pose;}
private:
	Transform _pose;
};

}


#endif /* ODOMETRYEVENT_H_ */
