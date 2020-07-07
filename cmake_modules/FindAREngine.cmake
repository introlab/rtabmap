# - Find AREngine
# This module finds an installed AREngine client C-API package.
#
# It sets the following variables:
#  AREngine_FOUND       - Set to false, or undefined, if AREngine isn't found.
#  AREngine_INCLUDE_DIRS - The AREngine include directory.
#  AREngine_LIBRARIES     - The AREngine library to link against.

FIND_PATH(AREngine_INCLUDE_DIR huawei_arengine_interface.h)

FIND_LIBRARY(AREngine_impl_LIBRARY NAMES huawei_arengine_impl PATH_SUFFIXES ${ANDROID_ABI})
FIND_LIBRARY(AREngine_jni_LIBRARY NAMES huawei_arengine_jni PATH_SUFFIXES ${ANDROID_ABI})
FIND_LIBRARY(AREngine_ndk_LIBRARY NAMES huawei_arengine_ndk PATH_SUFFIXES ${ANDROID_ABI})

IF (AREngine_INCLUDE_DIR AND AREngine_impl_LIBRARY AND AREngine_jni_LIBRARY AND AREngine_ndk_LIBRARY)
   SET(AREngine_FOUND TRUE)
   SET(AREngine_INCLUDE_DIRS ${AREngine_INCLUDE_DIR})
   SET(AREngine_LIBRARIES ${AREngine_impl_LIBRARY} ${AREngine_jni_LIBRARY} ${AREngine_ndk_LIBRARY})
ENDIF (AREngine_INCLUDE_DIR AND AREngine_impl_LIBRARY AND AREngine_jni_LIBRARY AND AREngine_ndk_LIBRARY)

IF (AREngine_FOUND)
   # show which AREngine was found only if not quiet
   IF (NOT AREngine_FIND_QUIETLY)
      MESSAGE(STATUS "Found AREngine: ${AREngine_INCLUDE_DIRS}")
   ENDIF (NOT AREngine_FIND_QUIETLY)
ELSE (AREngine_FOUND)
   # fatal error if AREngine is required but not found
   IF (AREngine_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "Could not find AREngine's impl, jni and ndk libraries)")
   ENDIF (AREngine_FIND_REQUIRED)
ENDIF (AREngine_FOUND)

