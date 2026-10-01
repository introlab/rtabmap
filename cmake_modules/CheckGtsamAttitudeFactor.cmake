# The 4.3 ROS snapshots share a numeric version but expose different APIs.
# Probe after selecting the C++ standard and inherit GTSAM's requirements.
INCLUDE(CheckCXXSourceCompiles)
INCLUDE(CMakePushCheckState)
CMAKE_PUSH_CHECK_STATE(RESET)
SET(CMAKE_REQUIRED_LIBRARIES gtsam)
UNSET(RTABMAP_GTSAM_HAS_ATTITUDE_FACTOR_TEMPLATE CACHE)
CHECK_CXX_SOURCE_COMPILES("
    #include <gtsam/navigation/AttitudeFactor.h>
    int main() {
        gtsam::AttitudeFactor<gtsam::Pose3> factor(1, gtsam::Unit3(0,0,1),
            gtsam::noiseModel::Isotropic::Sigma(2, 1.0));
        return factor.evaluateError(gtsam::Pose3()).size() != 2;
    }" RTABMAP_GTSAM_HAS_ATTITUDE_FACTOR_TEMPLATE)
CMAKE_POP_CHECK_STATE()
IF(RTABMAP_GTSAM_HAS_ATTITUDE_FACTOR_TEMPLATE)
    ADD_DEFINITIONS(-DRTABMAP_GTSAM_HAS_ATTITUDE_FACTOR_TEMPLATE)
ENDIF()
