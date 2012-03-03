# - Find FFTW3F
# This module finds an installed FFTW3F package.
#
# It sets the following variables:
#  FFTW3F_FOUND       - Set to false, or undefined, if FFTW3F isn't found.
#  FFTW3F_INCLUDE_DIRS - The FFTW3F include directory.
#  FFTW3F_LIBRARIES    - The FFTW3F library to link against.
#
# 
     

     
FIND_PATH(FFTW3F_INCLUDE_DIRS 
          fftw3.h
          PATH_SUFFIXES fftw3)

FIND_LIBRARY(FFTW3F_LIBRARIES NAMES fftw3f-3 fftw3f libfftw3f-3)

IF (FFTW3F_INCLUDE_DIRS AND FFTW3F_LIBRARIES)
   SET(FFTW3F_FOUND TRUE)
ENDIF (FFTW3F_INCLUDE_DIRS AND FFTW3F_LIBRARIES)

IF (FFTW3F_FOUND)
   # show which FFTW3F was found only if not quiet
   IF (NOT FFTW3F_FIND_QUIETLY)
      MESSAGE(STATUS "Found FFTW3F: ${FFTW3F_LIBRARIES}")
   ENDIF (NOT FFTW3F_FIND_QUIETLY)
ELSE (FFTW3F_FOUND)
   # fatal error if FFTW3F is required but not found
   IF (FFTW3F_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "Could not find FFTW3F")
   ENDIF (FFTW3F_FIND_REQUIRED)
ENDIF (FFTW3F_FOUND)

