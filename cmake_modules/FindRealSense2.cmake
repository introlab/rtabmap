# - Find librealsense (https://github.com/IntelRealSense/librealsense)
#
#  RealSense2_ROOT_DIR environment variable can be set to find the library.
#
# It sets the following variables:
#  RealSense2_FOUND         - Set to false, or undefined, if RealSense2 isn't found.
#  RealSense2_INCLUDE_DIRS  - The RealSense2 include directory.
#  RealSense2_LIBRARIES     - The RealSense2 library to link against.

#RealSense library

find_path(RealSense2_INCLUDE_DIRS NAMES librealsense2/rs.hpp PATHS $ENV{RealSense2_ROOT_DIR}/include)
if(CMAKE_CL_64)
find_library(RealSense2_LIBRARY NAMES realsense2 PATHS $ENV{RealSense2_ROOT_DIR}/lib $ENV{RealSense2_ROOT_DIR}/lib/x64  $ENV{RealSense2_ROOT_DIR}/bin $ENV{RealSense2_ROOT_DIR}/bin/x64)
else()
find_library(RealSense2_LIBRARY NAMES realsense2 PATHS $ENV{RealSense2_ROOT_DIR}/lib $ENV{RealSense2_ROOT_DIR}/lib/x86  $ENV{RealSense2_ROOT_DIR}/bin $ENV{RealSense2_ROOT_DIR}/bin/x86)
endif()

IF (RealSense2_INCLUDE_DIRS AND RealSense2_LIBRARY)
   SET(RealSense2_FOUND TRUE)
ENDIF (RealSense2_INCLUDE_DIRS AND RealSense2_LIBRARY)

IF (RealSense2_FOUND)
   SET(RealSense2_LIBRARIES ${RealSense2_LIBRARY})
   
   # Compatibility with linux names
   SET(realsense2_LIBRARIES ${RealSense2_LIBRARIES})
   SET(realsense2_INCLUDE_DIRS ${RealSense2_INCLUDE_DIRS})
   SET(realsense2_FOUND ${RealSense2_FOUND})
  
   # show which RealSense was found only if not quiet
   IF (NOT RealSense2_FIND_QUIETLY)
      MESSAGE(STATUS "Found RealSense: ${RealSense2_LIBRARIES}")
   ENDIF (NOT RealSense2_FIND_QUIETLY)
ELSE (RealSense2_FOUND)
   # fatal error if RealSense is required but not found
   IF (RealSense2_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "Could not find RealSense2 (librealsense2)")
   ENDIF (RealSense2_FIND_REQUIRED)
ENDIF (RealSense2_FOUND)

