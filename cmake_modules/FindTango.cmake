# - Find Tango
# This module finds an installed Tango client C-API package.
#
# It sets the following variables:
#  Tango_FOUND       - Set to false, or undefined, if Tango isn't found.
#  Tango_INCLUDE_DIRS - The Tango include directory.
#  Tango_LIBRARIES     - The Tango library to link against.

FIND_PATH(Tango_INCLUDE_DIR tango_client_api.h)

FIND_LIBRARY(Tango_LIBRARY NAMES tango_client_api PATH_SUFFIXES ${ANDROID_ABI})
FIND_LIBRARY(Tango_support_LIBRARY NAMES tango_support_api PATH_SUFFIXES ${ANDROID_ABI})

IF (Tango_INCLUDE_DIR AND Tango_LIBRARY AND Tango_support_LIBRARY)
   SET(Tango_FOUND TRUE)
   SET(Tango_INCLUDE_DIRS ${Tango_INCLUDE_DIR})
   SET(Tango_LIBRARIES ${Tango_LIBRARY} ${Tango_support_LIBRARY})
ENDIF (Tango_INCLUDE_DIR AND Tango_LIBRARY AND Tango_support_LIBRARY)

IF (Tango_FOUND)
   # show which Tango was found only if not quiet
   IF (NOT Tango_FIND_QUIETLY)
      MESSAGE(STATUS "Found Tango: ${Tango_INCLUDE_DIRS}")
   ENDIF (NOT Tango_FIND_QUIETLY)
ELSE (Tango_FOUND)
   # fatal error if Tango is required but not found
   IF (Tango_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "Could not find Tango (client and/or support libraries)")
   ENDIF (Tango_FIND_REQUIRED)
ENDIF (Tango_FOUND)

