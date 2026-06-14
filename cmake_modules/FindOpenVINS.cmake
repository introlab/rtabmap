# Find OpenVINS
# 
#   We search for a vins installation in ROS/ROS2 first, then fallback on 
#   ros-free library in common install paths

FIND_PACKAGE(ov_msckf QUIET)
IF(ov_msckf_FOUND)
   # On ROS2, the indirect includes and libraries
   # are not forwarded by ov_msckf target, append them manually
   FIND_PACKAGE(ov_core)
   FIND_PACKAGE(ov_init)
   IF(ov_msckf_FOUND AND ov_core_FOUND AND ov_init_FOUND)
      SET(OpenVINS_FOUND TRUE)
      SET(OpenVINS_INCLUDE_DIRS
         ${ov_msckf_INCLUDE_DIRS}
         ${ov_core_INCLUDE_DIRS}
         ${ov_init_INCLUDE_DIRS})
      SET(OpenVINS_LIBRARIES
         ${ov_msckf_LIBRARIES}
         ${ov_core_LIBRARIES}
         ${ov_init_LIBRARIES})
   ENDIF()
ELSE()
   find_path(OpenVINS_INCLUDE_DIR NAMES core/VioManager.h PATH_SUFFIXES open_vins)
   find_library(OpenVINS_LIBRARY NAMES ov_msckf_lib)
   IF (OpenVINS_INCLUDE_DIR AND OpenVINS_LIBRARY)
      SET(OpenVINS_FOUND TRUE)
      SET(OpenVINS_INCLUDE_DIRS ${OpenVINS_INCLUDE_DIR})
      SET(OpenVINS_LIBRARIES ${OpenVINS_LIBRARY})
   ENDIF()
ENDIF()

IF (OpenVINS_FOUND)
   # show which OpenVINS was found only if not quiet
   IF (NOT OpenVINS_FIND_QUIETLY)
      MESSAGE(STATUS "Found OpenVINS: ${OpenVINS_LIBRARIES} ${OpenVINS_INCLUDE_DIRS}")
   ENDIF (NOT OpenVINS_FIND_QUIETLY)
ELSE (OpenVINS_FOUND)
   # fatal error if OpenVINS is required but not found
   IF (OpenVINS_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "Could not find OpenVINS")
   ENDIF (OpenVINS_FIND_REQUIRED)
ENDIF (OpenVINS_FOUND)

