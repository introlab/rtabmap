# - Find ARCore
# This module finds an installed ARCore client C-API package.
#
# It sets the following variables:
#  ARCore_FOUND       - Set to false, or undefined, if ARCore isn't found.
#  ARCore_INCLUDE_DIRS - The ARCore include directory.
#  ARCore_LIBRARIES     - The ARCore library to link against.

FIND_PATH(ARCore_INCLUDE_DIR arcore_c_api.h)

FIND_LIBRARY(ARCore_c_LIBRARY NAMES arcore_sdk_c PATH_SUFFIXES ${ANDROID_ABI})
FIND_LIBRARY(ARCore_jni_LIBRARY NAMES arcore_sdk_jni PATH_SUFFIXES ${ANDROID_ABI})

IF (ARCore_INCLUDE_DIR AND ARCore_c_LIBRARY AND ARCore_jni_LIBRARY)
   SET(ARCore_FOUND TRUE)
   SET(ARCore_INCLUDE_DIRS ${ARCore_INCLUDE_DIR})
   SET(ARCore_LIBRARIES ${ARCore_c_LIBRARY} ${ARCore_jni_LIBRARY})
ENDIF (ARCore_INCLUDE_DIR AND ARCore_c_LIBRARY AND ARCore_jni_LIBRARY)

IF (ARCore_FOUND)
   # show which ARCore was found only if not quiet
   IF (NOT ARCore_FIND_QUIETLY)
      MESSAGE(STATUS "Found ARCore: ${ARCore_INCLUDE_DIRS}")
   ENDIF (NOT ARCore_FIND_QUIETLY)
ELSE (ARCore_FOUND)
   # fatal error if ARCore is required but not found
   IF (ARCore_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "Could not find ARCore (client and/or support libraries)")
   ENDIF (ARCore_FIND_REQUIRED)
ENDIF (ARCore_FOUND)

