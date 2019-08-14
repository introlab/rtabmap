# - Find FastCV (https://developer.qualcomm.com/software/fastcv-sdk)
#
# It sets the following variables:
#  FastCV_FOUND         - Set to false, or undefined, if FastCV isn't found.
#  FastCV_INCLUDE_DIRS  - The FastCV include directory.
#  FastCV_LIBRARIES     - The FastCV library to link against.

IF(NOT WIN32 AND NOT APPLE)

   EXECUTE_PROCESS( COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE )
   MESSAGE( STATUS "Architecture: ${ARCHITECTURE}" )

   # Currently only tested on aarch64!
   IF(${ARCHITECTURE} STREQUAL "aarch64")
      find_path(FastCV_INCLUDE_DIRS NAMES fastcv.h PATH_SUFFIXES fastcv)
      find_library(FastCV_LIBRARY NAMES fastcvopt fastcv)
   ENDIF(${ARCHITECTURE} STREQUAL "aarch64")
ENDIF(NOT WIN32 AND NOT APPLE)

IF (FastCV_INCLUDE_DIRS AND FastCV_LIBRARY)
   SET(FastCV_FOUND TRUE)
ENDIF (FastCV_INCLUDE_DIRS AND FastCV_LIBRARY)

IF (FastCV_FOUND)
   SET(FastCV_LIBRARIES ${FastCV_LIBRARY})
     
   # show which RealSense was found only if not quiet
   IF (NOT FastCV_FIND_QUIETLY)
      MESSAGE(STATUS "Found FastCV: ${FastCV_LIBRARIES}")
   ENDIF (NOT FastCV_FIND_QUIETLY)
ELSE (FastCV_FOUND)
   # fatal error if RealSense is required but not found
   IF (FastCV_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "Could not find FastCV")
   ENDIF (FastCV_FIND_REQUIRED)
ENDIF (FastCV_FOUND)

