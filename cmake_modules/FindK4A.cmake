# - Find K4A 
# This module finds an kinect 4 azure SDK
#
# It sets the following variables:
#  K4A_FOUND       - Set to false, or undefined, if K4A isn't found.
#  K4A_INCLUDE_DIRS - The K4A include directory.
#  K4A_LIBRARIES     - The K4A library to link against.

find_library(K4A_LIBRARY NAMES k4a NO_DEFAULT_PATH PATHS $ENV{K4A_ROOT_DIR}/sdk/windows-desktop/amd64/release/lib)
find_library(K4ARECORD_LIBRARY NAMES k4arecord NO_DEFAULT_PATH PATHS $ENV{K4A_ROOT_DIR}/sdk/windows-desktop/amd64/release/lib)
find_path(K4A_INCLUDE_DIR NAMES k4a/k4a.h PATHS $ENV{K4A_ROOT_DIR}/sdk/include)

IF (K4A_INCLUDE_DIR AND K4A_LIBRARY AND K4ARECORD_LIBRARY)
   SET(K4A_FOUND TRUE)
   SET(K4A_INCLUDE_DIRS ${K4A_INCLUDE_DIR})
   SET(K4A_LIBRARIES ${K4A_LIBRARY} ${K4ARECORD_LIBRARY})

   # Compatibility with linux names
   SET(k4a_LIBRARIES ${K4A_LIBRARIES})
   SET(k4a_INCLUDE_DIRS ${K4A_INCLUDE_DIRS})
   SET(k4a_FOUND ${K4A_FOUND})
ENDIF (K4A_INCLUDE_DIR AND K4A_LIBRARY AND K4ARECORD_LIBRARY)

IF (K4A_FOUND)
   # show which K4A was found only if not quiet
   IF (NOT K4A_FIND_QUIETLY)
      MESSAGE(STATUS "Found K4A: ${K4A_LIBRARIES}")
   ENDIF (NOT K4A_FIND_QUIETLY)
ELSE (K4A_FOUND)
   # fatal error if K4A is required but not found
   IF (K4A_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "Could not find K4A (Kinect for Azure SDK)")
   ENDIF (K4A_FIND_REQUIRED)
ENDIF (K4A_FOUND)
