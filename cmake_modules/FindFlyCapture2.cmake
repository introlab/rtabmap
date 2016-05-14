# - Find FlyCapture2 
# This module finds an installed FlyCapture2+Triclops stereo camera package. (Point Grey SDK)
#
# It sets the following variables:
#  FlyCapture2_FOUND       - Set to false, or undefined, if FlyCapture2 isn't found.
#  FlyCapture2_INCLUDE_DIRS - The FlyCapture2 include directory.
#  FlyCapture2_LIBRARIES     - The FlyCapture2 library to link against.

if(CMAKE_CL_64)
set(FlyCapture2_LIBDIR $ENV{FlyCapture2_ROOT_DIR}/lib64)
else()
set(FlyCapture2_LIBDIR $ENV{FlyCapture2_ROOT_DIR}/lib)
endif()

if(CMAKE_CL_64)
set(Triclops_LIBDIR $ENV{Triclops_ROOT_DIR}/lib64)
else()
set(Triclops_LIBDIR $ENV{Triclops_ROOT_DIR}/lib)
endif()

#FlyCapture2 SDK
find_path(FlyCapture2_INCLUDE_DIR NAMES FlyCapture2.h PATHS $ENV{FlyCapture2_ROOT_DIR}/include)
find_library(FlyCapture2_LIBRARY NAMES FlyCapture2_v100 FlyCapture2 flycapture2 NO_DEFAULT_PATH PATHS ${FlyCapture2_LIBDIR})

# Triclops SDK
find_path(Triclops_INCLUDE_DIR NAMES triclops.h PATHS $ENV{Triclops_ROOT_DIR}/include)
find_library(Triclops_LIBRARY NAMES triclops triclops_v100 NO_DEFAULT_PATH PATHS ${Triclops_LIBDIR})
find_library(FlyCaptureBridge_LIBRARY NAMES flycapture2bridge flycapture2bridge_v100 NO_DEFAULT_PATH PATHS ${Triclops_LIBDIR})
find_library(pnmutils_LIBRARY NAMES pnmutils pnmutils_v100 NO_DEFAULT_PATH PATHS ${Triclops_LIBDIR})

IF (FlyCapture2_INCLUDE_DIR AND Triclops_INCLUDE_DIR AND FlyCapture2_LIBRARY AND Triclops_LIBRARY AND FlyCaptureBridge_LIBRARY AND pnmutils_LIBRARY)
   SET(FlyCapture2_FOUND TRUE)
   SET(FlyCapture2_INCLUDE_DIRS ${FlyCapture2_INCLUDE_DIR} ${Triclops_INCLUDE_DIR})
   SET(FlyCapture2_LIBRARIES ${FlyCapture2_LIBRARY} ${Triclops_LIBRARY} ${FlyCaptureBridge_LIBRARY} ${pnmutils_LIBRARY})
ENDIF (FlyCapture2_INCLUDE_DIR AND Triclops_INCLUDE_DIR AND FlyCapture2_LIBRARY AND Triclops_LIBRARY AND FlyCaptureBridge_LIBRARY AND pnmutils_LIBRARY)

IF (FlyCapture2_FOUND)
   # show which FlyCapture2 was found only if not quiet
   IF (NOT FlyCapture2_FIND_QUIETLY)
      MESSAGE(STATUS "Found FlyCapture2: ${FlyCapture2_LIBRARIES}")
   ENDIF (NOT FlyCapture2_FIND_QUIETLY)
ELSE (FlyCapture2_FOUND)
   # fatal error if FlyCapture2 is required but not found
   IF (FlyCapture2_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "Could not find FlyCapture2 (FlyCapture2 Stereo Vision SDK)")
   ENDIF (FlyCapture2_FIND_REQUIRED)
ENDIF (FlyCapture2_FOUND)

