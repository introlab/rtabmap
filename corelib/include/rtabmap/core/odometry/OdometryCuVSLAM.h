/*
Copyright (c) 2025 Felix Toft
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

#ifndef ODOMETRYCUVSLAM_H_
#define ODOMETRYCUVSLAM_H_

#include <rtabmap/core/Odometry.h>
#include <memory>

#ifdef RTABMAP_CUVSLAM
#include <cuvslam.h>
#include <ground_constraint.h>
#include <cuda_runtime.h>
#endif

namespace rtabmap {

class RTABMAP_CORE_EXPORT OdometryCuVSLAM : public Odometry
{
public:
	OdometryCuVSLAM(const rtabmap::ParametersMap & parameters = rtabmap::ParametersMap());
	virtual ~OdometryCuVSLAM();

	virtual void reset(const Transform & initialPose = Transform::getIdentity());
	virtual Odometry::Type getType() {return Odometry::kTypeCuVSLAM;}

private:
	virtual Transform computeTransform(SensorData & image, const Transform & guess = Transform(), OdometryInfo * info = 0);

private:
#ifdef RTABMAP_CUVSLAM
	CUVSLAM_TrackerHandle cuvslam_handle_;
	CUVSLAM_GroundConstraintHandle ground_constraint_handle_;

	std::vector<CUVSLAM_Camera> cuvslam_cameras_;
	std::vector<std::array<float, 12>> intrinsics_;
	
	// State tracking
	bool initialized_;
	bool lost_;
	bool tracking_;
	bool planar_constraints_;
	Transform previous_pose_;
	double last_timestamp_;

	//visualization
	std::vector<CUVSLAM_Observation> observations_;
	std::vector<CUVSLAM_Landmark> landmarks_;
	
	// GPU memory management
	std::vector<uint8_t *> gpu_left_image_data_; // pointers to all gpu images
	std::vector<uint8_t *> gpu_right_image_data_;
	std::vector<size_t> gpu_left_image_sizes_; // size of one image
	std::vector<size_t> gpu_right_image_sizes_;
	cudaStream_t cuda_stream_;
#endif
};

}

#endif /* ODOMETRYCUVSLAM_H_ */