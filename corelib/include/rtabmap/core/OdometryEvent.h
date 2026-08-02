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

/**
 * @class OdometryEvent
 * @brief One processed frame with the pose the odometry integrated for it.
 *
 * Posted by @ref OdometryThread for every frame it processes, and consumed by
 * @ref RtabmapThread, which passes the data and the pose to
 * @ref Rtabmap::process(). A null @ref pose() means **odometry is lost** on this
 * frame; RtabmapThread reads that as a reset and starts a new map.
 *
 * The covariance is always 6x6 @c CV_64FC1 with positive finite diagonal terms
 * (identity when the front-end did not provide one, i.e. "unknown but valid").
 * A value &ge; 9999 on the first diagonal term is the convention for a lost estimate.
 *
 * @see OdometryThread
 * @see OdometryInfo
 * @see RtabmapThread
 */
class OdometryEvent : public UEvent
{
public:
	/** @brief Creates an empty event, with an identity covariance. */
	OdometryEvent()
	{
		_info.reg.covariance = cv::Mat::eye(6,6,CV_64FC1);
	}
	/**
	 * @brief Constructor.
	 * @param data The frame that was processed.
	 * @param pose Integrated odometry pose, null if odometry is lost.
	 * @param info Everything else the iteration produced.
	 *
	 * An empty covariance in @p info is replaced by identity; otherwise it must be
	 * 6x6 @c CV_64FC1 with finite, strictly positive diagonal terms.
	 */
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

	/** @return The processed frame, modifiable (e.g. to attach user data). */
	SensorData & data() {return _data;}
	/** @return The processed frame. */
	const SensorData & data() const {return _data;}
	/** @return Integrated odometry pose, null if odometry was lost on this frame. */
	const Transform & pose() const {return _pose;}
	/** @return 6x6 covariance of the motion estimate (@c CV_64FC1). */
	const cv::Mat & covariance() const {return _info.reg.covariance;}
	/**
	 * @brief Linear and angular velocity, from the motion and the frame interval.
	 * @return (vx, vy, vz, vroll, vpitch, vyaw) in m/s and rad/s, or an empty
	 *         vector when the interval is unknown.
	 */
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
	/** @return Quality indicators, timings and intermediate data of the iteration. */
	const OdometryInfo & info() const {return _info;}

private:
	SensorData _data;
	Transform _pose;
	OdometryInfo _info;
};

/**
 * @class OdometryResetEvent
 * @brief Asks @ref OdometryThread to restart the odometry from a given pose.
 *
 * The buffered frames and IMU samples are dropped, and the integration starts
 * over from @ref getPose(). Handled even before the thread is started.
 *
 * @see OdometryThread
 * @see Odometry::reset()
 */
class OdometryResetEvent : public UEvent
{
public:
	/** @param pose Pose to restart from (identity by default). */
	OdometryResetEvent(const Transform & pose = Transform::getIdentity()){_pose = pose;}
	virtual ~OdometryResetEvent() {}
	virtual std::string getClassName() const {return "OdometryResetEvent";}
	/** @return The pose the odometry should restart from. */
	const Transform & getPose() const {return _pose;}
private:
	Transform _pose;
};

}


#endif /* ODOMETRYEVENT_H_ */
