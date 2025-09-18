#ifndef ODOMETRYCUVSLAM_H_
#define ODOMETRYCUVSLAM_H_

#include <rtabmap/core/Odometry.h>
#include <cuvslam/cuvslam.h>
#include <opencv2/opencv.hpp>

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
        bool initializeCuVSLAM(const SensorData & data);
        bool prepareImages(const SensorData & data, std::vector<CUVSLAM_Image> & cuvslam_images);
        void processIMUData(const SensorData & data);
        Transform convertCuVSLAMPose(const CUVSLAM_Pose & cuvslam_pose);
        CUVSLAM_Configuration CreateConfiguration(const CUVSLAM_Pose & cv_base_link_pose_cv_imu, const SensorData & data);
        
        CUVSLAM_TrackerHandle cuvslam_handle_;
        CUVSLAM_CameraRig camera_rig_;
        std::vector<CUVSLAM_Camera> cuvslam_cameras_;
        CUVSLAM_Configuration configuration_;
        bool initialized_;
        bool lost_;
        Transform previous_pose_;
        double last_timestamp_;
    #endif
};

}

#endif /* ODOMETRYCUVSLAM_H_ */