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

#ifndef ODOMETRYVISO2_H_
#define ODOMETRYVISO2_H_

#include <rtabmap/core/Odometry.h>

class VisualOdometryStereo;

namespace rtabmap {

class RTABMAP_CORE_EXPORT OdometryViso2 : public Odometry
{
public:
	OdometryViso2(const rtabmap::ParametersMap & parameters = rtabmap::ParametersMap());
	virtual ~OdometryViso2();

	virtual void reset(const Transform & initialPose = Transform::getIdentity());
	virtual Odometry::Type getType() {return Odometry::kTypeViso2;}

private:
	virtual Transform computeTransform(SensorData & image, const Transform & guess = Transform(), OdometryInfo * info = 0);

private:
#ifdef RTABMAP_VISO2
	VisualOdometryStereo * viso2_;
	int ref_frame_change_method_;       // Reference frame method (defautl 0): 0=under inliers threshold, 1=min pixel motion,
	int ref_frame_inlier_threshold_;    // method 0. Change the reference frame if the number of inliers is low
	double ref_frame_motion_threshold_; // method 1. Change the reference frame if last motion is small
	bool lost_;
	bool keep_reference_frame_;
#endif
	Transform reference_motion_;
	Transform previousLocalTransform_;
	ParametersMap viso2Parameters_;
};

}

#endif /* ODOMETRYVISO2_H_ */
