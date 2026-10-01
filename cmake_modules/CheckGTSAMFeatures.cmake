# Detects GTSAM API variations that the version number alone can't tell
# apart. Included after FIND_PACKAGE(GTSAM) succeeded.

# Pose3AttitudeFactor has been replaced by AttitudeFactor<Pose3> in 4.3, but
# the 4.3 ROS snapshots share a numeric version (4.3.0) while exposing either
# API, so probe which one compiles. Older versions only have
# Pose3AttitudeFactor. The probe links the imported gtsam target, so it
# inherits GTSAM's usage requirements (including cxx_std_17 for 4.3) and
# doesn't depend on the C++ standard selected later in the main CMakeLists.txt.
IF(GTSAM_VERSION VERSION_GREATER_EQUAL "4.3.0")
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
ENDIF()
