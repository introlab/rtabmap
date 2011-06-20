# - Find ROS
# This module finds an installed ROS package.
#
# It sets the following variables:
#  ROS_FOUND       - Set to false, or undefined, if ROS isn't found.

IF(UNIX)
	FIND_PROGRAM(ROS_EXEC NAME rosrun PATHS)  
	IF(ROS_EXEC)
        SET(ROS_FOUND TRUE)
	ENDIF(ROS_EXEC)
ENDIF(UNIX)

IF (ROS_FOUND)
   # show which ROS was found only if not quiet
   IF (NOT ROS_FIND_QUIETLY)
      MESSAGE(STATUS "Found ROS")
   ENDIF (NOT ROS_FIND_QUIETLY)
ELSE (ROS_FOUND)
   # fatal error if ROS is required but not found
   IF (ROS_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "Could not find ROS")
   ENDIF (ROS_FIND_REQUIRED)
ENDIF (ROS_FOUND)

