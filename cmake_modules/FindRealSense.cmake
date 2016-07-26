# - Find librealsense (https://github.com/IntelRealSense/librealsense)
#
#  RealSense_ROOT_DIR environment variable can be set to find the library.
#
# It sets the following variables:
#  RealSense_FOUND       - Set to false, or undefined, if RealSense isn't found.
#  RealSense_INCLUDE_DIRS - The RealSense include directory.
#  RealSense_LIBRARIES     - The RealSense library to link against.

#RealSense library
find_path(RealSense_INCLUDE_DIRS NAMES librealsense/rs.hpp PATHS $ENV{RealSense_ROOT_DIR}/include)
if(CMAKE_CL_64)
find_library(RealSense_LIBRARY NAMES realsense PATHS $ENV{RealSense_ROOT_DIR}/lib  $ENV{RealSense_ROOT_DIR}/bin $ENV{RealSense_ROOT_DIR}/bin/x64)
else()
find_library(RealSense_LIBRARY NAMES realsense PATHS $ENV{RealSense_ROOT_DIR}/lib  $ENV{RealSense_ROOT_DIR}/bin $ENV{RealSense_ROOT_DIR}/bin/Win32)
endif()

IF (RealSense_INCLUDE_DIRS AND RealSense_LIBRARY)
   SET(RealSense_FOUND TRUE)
ENDIF (RealSense_INCLUDE_DIRS AND RealSense_LIBRARY)

IF (RealSense_FOUND)
   # show which RealSense was found only if not quiet
   SET(RealSense_LIBRARIES ${RealSense_LIBRARY})
   IF (NOT RealSense_FIND_QUIETLY)
      MESSAGE(STATUS "Found RealSense: ${RealSense_LIBRARIES}")
   ENDIF (NOT RealSense_FIND_QUIETLY)
ELSE (RealSense_FOUND)
   # fatal error if RealSense is required but not found
   IF (RealSense_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "Could not find RealSense (librealsense)")
   ENDIF (RealSense_FIND_REQUIRED)
ENDIF (RealSense_FOUND)

