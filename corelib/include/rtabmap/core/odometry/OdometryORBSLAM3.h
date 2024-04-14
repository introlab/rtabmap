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

#ifndef ODOMETRYORBSLAM3_H_
#define ODOMETRYORBSLAM3_H_

#include <rtabmap/core/Odometry.h>

#if defined(RTABMAP_ORB_SLAM) and RTABMAP_ORB_SLAM == 3
#include <System.h>
#endif

namespace rtabmap {

class RTABMAP_CORE_EXPORT OdometryORBSLAM3 : public Odometry
{
public:
	OdometryORBSLAM3(const rtabmap::ParametersMap & parameters = rtabmap::ParametersMap());
	virtual ~OdometryORBSLAM3();

	virtual void reset(const Transform & initialPose = Transform::getIdentity());
	virtual Odometry::Type getType() {return Odometry::kTypeORBSLAM;}
	virtual bool canProcessAsyncIMU() const;

private:
	virtual Transform computeTransform(SensorData & image, const Transform & guess = Transform(), OdometryInfo * info = 0);

	bool init(const rtabmap::CameraModel & model1, const rtabmap::CameraModel & model2, double stamp, bool stereo, double baseline);
private:
#if defined(RTABMAP_ORB_SLAM) and RTABMAP_ORB_SLAM == 3
	ORB_SLAM3::System * orbslam_;
	bool firstFrame_;
	Transform originLocalTransform_;
	Transform previousPose_;
	bool useIMU_;
	Transform imuLocalTransform_;
	ParametersMap parameters_;
	std::vector<ORB_SLAM3::IMU::Point> orbslamImus_;
	double lastImuStamp_;
	double lastImageStamp_;
#endif

};

}

#endif /* ODOMETRYORBSLAM_H3_ */
