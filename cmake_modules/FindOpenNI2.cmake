# - Find OpenNI2
# This module finds an installed OpenNI2 package.
#
# It sets the following variables:
#  OpenNI2_FOUND       - Set to false, or undefined, if OpenNI2 isn't found.
#  OpenNI2_INCLUDE_DIRS - The OpenNI2 include directory.
#  OpenNI2_LIBRARIES     - The OpenNI2 libraries to link against.
#  OpenNI2_LIBRARY       - The OpenNI2 library alone.

FIND_PATH(OpenNI2_INCLUDE_DIRS OpenNI.h HINTS $ENV{OPENNI2_INCLUDE64} $ENV{OPENNI2_INCLUDE} PATH_SUFFIXES openni2)

FIND_LIBRARY(OpenNI2_LIBRARY NAMES OpenNI2 HINTS $ENV{OPENNI2_LIB64} $ENV{OPENNI2_LIB} $ENV{OPENNI2_REDIST})

IF (OpenNI2_INCLUDE_DIRS AND OpenNI2_LIBRARY)
   SET(OpenNI2_FOUND TRUE)
ENDIF (OpenNI2_INCLUDE_DIRS AND OpenNI2_LIBRARY)

IF (OpenNI2_FOUND)
   # show which OpenNI2 was found only if not quiet
   SET(OpenNI2_LIBRARIES ${OpenNI2_LIBRARY})
   IF (NOT OpenNI2_FIND_QUIETLY)
      MESSAGE(STATUS "Found OpenNI2: ${OpenNI2_LIBRARIES}")
   ENDIF (NOT OpenNI2_FIND_QUIETLY)
ELSE (OpenNI2_FOUND)
   # fatal error if OpenNI2 is required but not found
   IF (OpenNI2_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "Could not find OpenNI2. Environment variables OPENNI2_INCLUDE (directory containing OpenNI.h) and OPENNI2_LIB (directory containing OpenNI2 library) could bet set.")
   ENDIF (OpenNI2_FIND_REQUIRED)
ENDIF (OpenNI2_FOUND)

