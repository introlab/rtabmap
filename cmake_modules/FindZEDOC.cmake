# - Find ZED Open Capture 
# This module finds zed open capture library
#
# It sets the following variables:
#  ZEDOC_FOUND        - Set to false, or undefined, if ZEDOC isn't found.
#  ZEDOC_INCLUDE_DIRS - The ZEDOC include directory.
#  ZEDOC_LIBRARIES    - The ZEDOC library to link against.

find_library(ZEDOC_LIBRARY NAMES zed_open_capture PATHS $ENV{ZEDOC_ROOT_DIR}/lib)
find_path(ZEDOC_INCLUDE_DIR NAMES zed-open-capture/videocapture.hpp PATHS $ENV{ZEDOC_ROOT_DIR}/include)

IF (ZEDOC_INCLUDE_DIR AND ZEDOC_LIBRARY)
   SET(ZEDOC_FOUND TRUE)
   SET(ZEDOC_INCLUDE_DIRS ${ZEDOC_INCLUDE_DIR})
   SET(ZEDOC_LIBRARIES ${ZEDOC_LIBRARY})
ENDIF (ZEDOC_INCLUDE_DIR AND ZEDOC_LIBRARY)

IF (ZEDOC_FOUND)
   # show which ZEDOC was found only if not quiet
   IF (NOT ZEDOC_FIND_QUIETLY)
      MESSAGE(STATUS "Found ZEDOC: ${ZEDOC_LIBRARIES}")
   ENDIF (NOT ZEDOC_FIND_QUIETLY)
ELSE (ZEDOC_FOUND)
   # fatal error if ZEDOC is required but not found
   IF (ZEDOC_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "Could not find ZEDOC (Zed Open Capture)")
   ENDIF (ZEDOC_FIND_REQUIRED)
ENDIF (ZEDOC_FOUND)
