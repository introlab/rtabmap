# - Find librealsense (https://github.com/IntelRealSense/librealsense)
#
#  RealSense_ROOT_DIR environment variable can be set to find the library.
#
# It sets the following variables:
#  RealSense_FOUND       - Set to false, or undefined, if RealSense isn't found.
#  RealSenseSlam_FOUND    - Set to false, or undefined, if RealSense slam module isn't found.
#  RealSense_INCLUDE_DIRS - The RealSense include directory.
#  RealSense_LIBRARIES     - The RealSense library to link against.

# Use find_package( RealSense COMPONENTS slam ) to search for realsense slam library
if( RealSense_FIND_COMPONENTS )
   foreach( component ${RealSense_FIND_COMPONENTS} )
     string( TOUPPER ${component} _COMPONENT )
     set( REALSENSE_USE_${_COMPONENT} 1 )
   endforeach()
endif()

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

#SLAM
if(REALSENSE_USE_SLAM)
   find_path(RealSenseSlam_INCLUDE_DIRS NAMES librealsense/slam/slam.h PATHS $ENV{RealSense_ROOT_DIR}/include)
   if(CMAKE_CL_64)
      find_library(RealSenseSlam_LIBRARY NAMES realsense_slam PATHS $ENV{RealSense_ROOT_DIR}/lib  $ENV{RealSense_ROOT_DIR}/bin $ENV{RealSense_ROOT_DIR}/bin/x64)
      find_library(RealSenseImage_LIBRARY NAMES realsense_image PATHS $ENV{RealSense_ROOT_DIR}/lib  $ENV{RealSense_ROOT_DIR}/bin $ENV{RealSense_ROOT_DIR}/bin/x64)
      find_library(RealSenseSP_Core_LIBRARY NAMES SP_Core PATHS $ENV{RealSense_ROOT_DIR}/lib  $ENV{RealSense_ROOT_DIR}/bin $ENV{RealSense_ROOT_DIR}/bin/x64)
      find_library(RealSenseTracker_LIBRARY NAMES tracker PATHS $ENV{RealSense_ROOT_DIR}/lib  $ENV{RealSense_ROOT_DIR}/bin $ENV{RealSense_ROOT_DIR}/bin/x64)
   else()
      find_library(RealSenseSlam_LIBRARY NAMES realsense_slam PATHS $ENV{RealSense_ROOT_DIR}/lib  $ENV{RealSense_ROOT_DIR}/bin $ENV{RealSense_ROOT_DIR}/bin/Win32)
      find_library(RealSenseImage_LIBRARY NAMES realsense_image PATHS $ENV{RealSense_ROOT_DIR}/lib  $ENV{RealSense_ROOT_DIR}/bin $ENV{RealSense_ROOT_DIR}/bin/Win32)
      find_library(RealSenseSP_Core_LIBRARY NAMES SP_Core PATHS $ENV{RealSense_ROOT_DIR}/lib  $ENV{RealSense_ROOT_DIR}/bin $ENV{RealSense_ROOT_DIR}/bin/Win32)
      find_library(RealSenseTracker_LIBRARY NAMES tracker PATHS $ENV{RealSense_ROOT_DIR}/lib  $ENV{RealSense_ROOT_DIR}/bin $ENV{RealSense_ROOT_DIR}/bin/Win32)
   endif()
else()
   set(RealSenseSlam_INCLUDE_DIRS "")
endif()

IF (RealSense_FOUND)

   IF (RealSenseSlam_INCLUDE_DIRS AND RealSenseSlam_LIBRARY AND RealSenseImage_LIBRARY AND RealSenseSP_Core_LIBRARY AND RealSenseTracker_LIBRARY)
      SET(RealSenseSlam_FOUND TRUE)
   ENDIF (RealSenseSlam_INCLUDE_DIRS AND RealSenseSlam_LIBRARY AND RealSenseImage_LIBRARY AND RealSenseSP_Core_LIBRARY AND RealSenseTracker_LIBRARY)

   SET(RealSense_LIBRARIES ${RealSense_LIBRARY})
   IF (RealSenseSlam_FOUND)
      IF (RealSenseSlam_INCLUDE_DIRS AND RealSenseSlam_LIBRARY AND RealSenseImage_LIBRARY AND RealSenseSP_Core_LIBRARY AND RealSenseTracker_LIBRARY)
         SET(RealSenseSlam_FOUND TRUE)
      ENDIF (RealSenseSlam_INCLUDE_DIRS AND RealSenseSlam_LIBRARY AND RealSenseImage_LIBRARY AND RealSenseSP_Core_LIBRARY AND RealSenseTracker_LIBRARY)
      SET(RealSense_LIBRARIES 
            ${RealSense_LIBRARIES}
            ${RealSenseSlam_LIBRARY}
            ${RealSenseImage_LIBRARY}
            ${RealSenseSP_Core_LIBRARY}
            ${RealSenseTracker_LIBRARY})
   ENDIF(RealSenseSlam_FOUND)

   # show which RealSense was found only if not quiet
   IF (NOT RealSense_FIND_QUIETLY)
      MESSAGE(STATUS "Found RealSense: ${RealSense_LIBRARIES}")
   ENDIF (NOT RealSense_FIND_QUIETLY)
ELSE (RealSense_FOUND)
   # fatal error if RealSense is required but not found
   IF (RealSense_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "Could not find RealSense (librealsense)")
   ENDIF (RealSense_FIND_REQUIRED)
ENDIF (RealSense_FOUND)

