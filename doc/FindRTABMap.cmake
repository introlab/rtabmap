# - Find RTABMap
# This module finds an installed RTABMap package.
#
# It sets the following variables:
#  RTABMap_FOUND              - Set to false, or undefined, if RTABMap isn't found.
#  RTABMap_INCLUDE_DIRS        - The RTABMap include directory.
#  RTABMap_LIBRARIES            - The RTABMap library to link against.
#
# Look up for rtabmap ros package first, if not found search system wide.
# 

SET(RTABMap_ROOT)

# Add ROS RTABMap directory if ROS is installed
FIND_PROGRAM(ROSPACK_EXEC NAME rospack PATHS)  
IF(ROSPACK_EXEC)  
	EXECUTE_PROCESS(COMMAND ${ROSPACK_EXEC} find rtabmap_lib 
			   	    OUTPUT_VARIABLE RTABMap_ROS_PATH
					OUTPUT_STRIP_TRAILING_WHITESPACE
					WORKING_DIRECTORY "./"
	)
	IF(RTABMap_ROS_PATH)
	    MESSAGE(STATUS "Found RTABMap ROS pkg : ${RTABMap_ROS_PATH}")
	    SET(RTABMap_ROOT
	        ${RTABMap_ROS_PATH}/rtabmap
	        ${RTABMap_ROOT}
	    )
	ENDIF(RTABMap_ROS_PATH)
ENDIF(ROSPACK_EXEC)

IF(WIN32)
	FIND_PATH(RTABMap_INCLUDE_DIRS 
			rtabmap/core/Rtabmap.h
			PATH_SUFFIXES "../include")

	FIND_LIBRARY(RTABMap_LIBRARIES 
			NAMES rtabmap_core
		 	PATH_SUFFIXES "../lib")

ELSE()
	FIND_PATH(RTABMap_INCLUDE_DIRS 
			rtabmap/core/Rtabmap.h
			PATHS ${RTABMap_ROOT}/include)

	FIND_LIBRARY(RTABMap_LIBRARIES 
			NAMES rtabmap_core
			PATHS ${RTABMap_ROOT}/lib)
ENDIF()

	
IF (RTABMap_INCLUDE_DIRS AND RTABMap_LIBRARIES)
   SET(RTABMap_FOUND TRUE)
ENDIF (RTABMap_INCLUDE_DIRS AND RTABMap_LIBRARIES)

IF (RTABMap_FOUND)
   # show which RTABMap was found only if not quiet
   IF (NOT RTABMap_FIND_QUIETLY)
      MESSAGE(STATUS "Found RTABMap ${RTABMap_VERSION}")
   ENDIF (NOT RTABMap_FIND_QUIETLY)
ELSE ()
   # fatal error if RTABMap is required but not found
   IF (RTABMap_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "Could not find RTABMap. Verify your PATH if it is already installed or download it at http://rtabmap.googlecode.com")
   ENDIF (RTABMap_FIND_REQUIRED)
ENDIF ()
