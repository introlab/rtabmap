# - Find DC1394 alias libdc1394
# This module finds an installed DC1394 package.
#
# It sets the following variables:
#  DC1394_FOUND       - Set to false, or undefined, if DC1394 isn't found.
#  DC1394_INCLUDE_DIRS - The DC1394 include directory.
#  DC1394_LIBRARIES     - The DC1394 library to link against.

find_path(DC1394_INCLUDE_DIRS NAMES dc1394.h PATH_SUFFIXES dc1394)
find_library(DC1394_LIBRARIES NAMES dc1394)

IF (DC1394_INCLUDE_DIRS AND DC1394_LIBRARIES)
   SET(DC1394_FOUND TRUE)

   #On Mac OS X
   #if(CMAKE_SYSTEM_NAME MATCHES "Darwin")
   #  set(DC1394_LIBRARIES ${DC1394_LIBRARIES} "-framework CoreServices")
   #endif(CMAKE_SYSTEM_NAME MATCHES "Darwin")

ENDIF (DC1394_INCLUDE_DIRS AND DC1394_LIBRARIES)

IF (DC1394_FOUND)
   # show which DC1394 was found only if not quiet
   IF (NOT DC1394_FIND_QUIETLY)
      MESSAGE(STATUS "Found DC1394: ${DC1394_LIBRARIES}")
   ENDIF (NOT DC1394_FIND_QUIETLY)
ELSE (DC1394_FOUND)
   # fatal error if DC1394 is required but not found
   IF (DC1394_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "Could not find DC1394 (libdc1394)")
   ENDIF (DC1394_FIND_REQUIRED)
ENDIF (DC1394_FOUND)

