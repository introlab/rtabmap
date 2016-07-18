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
	static cv::Mat generateCovarianceMatrix(float rotVariance, float transVariance)
	{
		UASSERT(uIsFinite(rotVariance) && rotVariance>0);
		UASSERT(uIsFinite(transVariance) && transVariance>0);
		cv::Mat covariance = cv::Mat::eye(6,6,CV_64FC1);
		covariance.at<double>(0,0) = transVariance;
		covariance.at<double>(1,1) = transVariance;
		covariance.at<double>(2,2) = transVariance;
		covariance.at<double>(3,3) = rotVariance;
		covariance.at<double>(4,4) = rotVariance;
		covariance.at<double>(5,5) = rotVariance;
		return covariance;
	}
public:
	OdometryEvent() :
		_covariance(cv::Mat::eye(6,6,CV_64FC1))
	{
	}
	OdometryEvent(
		const SensorData & data,
		const Transform & pose,
		const cv::Mat & covariance = cv::Mat::eye(6,6,CV_64FC1),
		const OdometryInfo & info = OdometryInfo()) :
			_data(data),
			_pose(pose),
			_info(info)
	{
		UASSERT(covariance.cols == 6 && covariance.rows == 6 && covariance.type() == CV_64FC1);
		UASSERT_MSG(uIsFinite(covariance.at<double>(0,0)) && covariance.at<double>(0,0)>0, "Transitional variance should not be null! (set to 1 if unknown)");
		UASSERT_MSG(uIsFinite(covariance.at<double>(1,1)) && covariance.at<double>(1,1)>0, "Transitional variance should not be null! (set to 1 if unknown)");
		UASSERT_MSG(uIsFinite(covariance.at<double>(2,2)) && covariance.at<double>(2,2)>0, "Transitional variance should not be null! (set to 1 if unknown)");
		UASSERT_MSG(uIsFinite(covariance.at<double>(3,3)) && covariance.at<double>(3,3)>0, "Rotational variance should not be null! (set to 1 if unknown)");
		UASSERT_MSG(uIsFinite(covariance.at<double>(4,4)) && covariance.at<double>(4,4)>0, "Rotational variance should not be null! (set to 1 if unknown)");
		UASSERT_MSG(uIsFinite(covariance.at<double>(5,5)) && covariance.at<double>(5,5)>0, "Rotational variance should not be null! (set to 1 if unknown)");
		_covariance = covariance;
	}
	OdometryEvent(
		const SensorData & data,
		const Transform & pose,
		double rotVariance = 1.0,
		double transVariance = 1.0,
		const OdometryInfo & info = OdometryInfo()) :
			_data(data),
			_pose(pose),
			_covariance(generateCovarianceMatrix(rotVariance, transVariance)),
			_info(info)
	{
	}
	virtual ~OdometryEvent() {}
	virtual std::string getClassName() const {return "OdometryEvent";}

	SensorData & data() {return _data;}
	const SensorData & data() const {return _data;}
	const Transform & pose() const {return _pose;}
	const cv::Mat & covariance() const {return _covariance;}
	const OdometryInfo & info() const {return _info;}
	double rotVariance() const {return uMax3(_covariance.at<double>(3,3), _covariance.at<double>(4,4), _covariance.at<double>(5,5));}
	double transVariance() const {return uMax3(_covariance.at<double>(0,0), _covariance.at<double>(1,1), _covariance.at<double>(2,2));}

private:
	SensorData _data;
	Transform _pose;
	cv::Mat _covariance;
	OdometryInfo _info;
};

class OdometryResetEvent : public UEvent
{
public:
	OdometryResetEvent(){}
	virtual ~OdometryResetEvent() {}
	virtual std::string getClassName() const {return "OdometryResetEvent";}
};

}


#endif /* ODOMETRYEVENT_H_ */
