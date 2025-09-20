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

#ifndef ODOMETRYCUVSLAM_H_
#define ODOMETRYCUVSLAM_H_

#include <rtabmap/core/Odometry.h>
#include <memory>

namespace cv {
    class Mat;
}

// Forward declarations for cuVSLAM types to avoid including cuvslam.h
#ifdef RTABMAP_CUVSLAM
struct CUVSLAM_Tracker;
struct CUVSLAM_Camera;
struct CUVSLAM_CameraRig;
struct CUVSLAM_Configuration;
struct CUVSLAM_Pose;
struct CUVSLAM_Image;
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

	// Helper functions for cuVSLAM integration (internal implementation details)
	bool initializeCuVSLAM(const SensorData & data);
	bool prepareImages(const SensorData & data, std::vector<CUVSLAM_Image*> & cuvslam_images);
	Transform convertCuVSLAMPose(const CUVSLAM_Pose * cuvslam_pose);
	cv::Mat convertCuVSLAMCovariance(const float * cuvslam_covariance);
	CUVSLAM_Configuration * CreateConfiguration(const CUVSLAM_Pose * cv_base_link_pose_cv_imu);
	CUVSLAM_Pose * convertCameraPoseToCuVSLAM(const Transform & transform);
	CUVSLAM_Pose * rtabmapTransformToCuVSLAMPose(const Transform & transform);

private:
#ifdef RTABMAP_CUVSLAM
	// cuVSLAM handles and data structures (using forward-declared types for type safety)
	CUVSLAM_Tracker * cuvslam_handle_;
	std::vector<CUVSLAM_Camera*> cuvslam_cameras_;
	std::vector<CUVSLAM_Camera> * cuvslam_camera_objects_;
	std::vector<CUVSLAM_Image> * cuvslam_image_objects_;
	CUVSLAM_CameraRig * camera_rig_;
	CUVSLAM_Configuration * configuration_;
	
	// State tracking
	bool initialized_;
	bool lost_;
	Transform previous_pose_;
	double last_timestamp_;
	
	// Processed images storage (to keep them in scope)
	cv::Mat processed_left_image_;
	cv::Mat processed_right_image_;
	
	// Camera parameter storage (to keep them in scope)
	float left_camera_params_[4];
	float right_camera_params_[4];
	
	// Distortion model strings (to keep them in scope)
	const char * left_distortion_model_;
	const char * right_distortion_model_;
#endif
};

}

#endif /* ODOMETRYCUVSLAM_H_ */