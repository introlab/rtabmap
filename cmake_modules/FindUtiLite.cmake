# - Find UtiLite
# This module finds an installed UtiLite package.
#
# It sets the following variables:
#  UtiLite_FOUND              - Set to false, or undefined, if UtiLite isn't found.
#  UtiLite_INCLUDE_DIRS        - The UtiLite include directory.
#  UtiLite_LIBRARIES            - The UtiLite library to link against.
#  URESOURCEGENERATOR_EXEC    - The resource generator tool executable
#
# Backward compatibility:
#  UTILITE_FOUND              - Set to false, or undefined, if UtiLite isn't found.
#  UTILITE_INCLUDE_DIRS       - The UtiLite include directory.
#  UTILITE_LIBRARIES          - The UtiLite library to link against.
#  UTILITE_INCLUDE_DIR        - The UtiLite include directory.
#  UTILITE_LIBRARY            - The UtiLite library to link against.
#
# 

SET(UtiLite_VERSION_REQUIRED 0.3.0)

SET(UtiLite_ROOT)

# Add ROS UtiLite directory if ROS is installed
FIND_PROGRAM(ROSPACK_EXEC NAME rospack PATHS)  
IF(ROSPACK_EXEC)  
	EXECUTE_PROCESS(COMMAND ${ROSPACK_EXEC} find utilite 
			   	    OUTPUT_VARIABLE UtiLite_ROS_PATH
					OUTPUT_STRIP_TRAILING_WHITESPACE
					WORKING_DIRECTORY "./"
	)
	IF(UtiLite_ROS_PATH)
	    MESSAGE(STATUS "Found UtiLite ROS pkg : ${UtiLite_ROS_PATH}")
	    SET(UtiLite_ROOT
	        ${UtiLite_ROS_PATH}/utilite
	        ${UtiLite_ROOT}
	    )
	ENDIF(UtiLite_ROS_PATH)
ENDIF(ROSPACK_EXEC)

FIND_PROGRAM(URESOURCEGENERATOR_EXEC NAME uresourcegenerator PATHS ${UtiLite_ROOT}/bin)  
IF(URESOURCEGENERATOR_EXEC)  
	EXECUTE_PROCESS(COMMAND ${URESOURCEGENERATOR_EXEC} -v 
			   	    OUTPUT_VARIABLE UtiLite_VERSION
					OUTPUT_STRIP_TRAILING_WHITESPACE
					WORKING_DIRECTORY "./"
	)
	
	IF(UtiLite_VERSION VERSION_LESS UtiLite_VERSION_REQUIRED)
	    IF(UtiLite_FIND_REQUIRED)
	    	MESSAGE(FATAL_ERROR "Your version of UtiLite is too old (${UtiLite_VERSION}), UtiLite ${UtiLite_VERSION_REQUIRED} is required.")
	    ENDIF(UtiLite_FIND_REQUIRED)
	ENDIF(UtiLite_VERSION VERSION_LESS UtiLite_VERSION_REQUIRED)

	IF(WIN32)
		FIND_PATH(UtiLite_INCLUDE_DIRS 
				utilite/UEventsManager.h
				PATH_SUFFIXES "../include")
	
		FIND_LIBRARY(UtiLite_LIBRARIES 
				NAMES utilite
			 	PATH_SUFFIXES "../lib")
		FIND_LIBRARY(UtiLite_Qt 
				NAMES utilite_qt
			 	PATH_SUFFIXES "../lib")
		FIND_LIBRARY(UtiLite_Audio 
				NAMES utilite_audio
			 	PATH_SUFFIXES "../lib")
		FIND_LIBRARY(UtiLite_Cv 
				NAMES utilite_cv
			 	PATH_SUFFIXES "../lib")
		FIND_LIBRARY(UtiLite_Cvqt 
				NAMES utilite_cvqt
			 	PATH_SUFFIXES "../lib")
	
	ELSE()
		FIND_PATH(UtiLite_INCLUDE_DIRS 
				utilite/UEventsManager.h
				PATHS ${UtiLite_ROOT}/include)
	
		FIND_LIBRARY(UtiLite_LIBRARIES 
				NAMES utilite
				PATHS ${UtiLite_ROOT}/lib)
		FIND_LIBRARY(UtiLite_Qt 
				NAMES utilite_qt
				PATHS ${UtiLite_ROOT}/lib)
		FIND_LIBRARY(UtiLite_Audio 
				NAMES utilite_audio
				PATHS ${UtiLite_ROOT}/lib)
		FIND_LIBRARY(UtiLite_Cv 
				NAMES utilite_cv
				PATHS ${UtiLite_ROOT}/lib)
		FIND_LIBRARY(UtiLite_Cvqt 
				NAMES utilite_cvqt
				PATHS ${UtiLite_ROOT}/lib)
	ENDIF()
	IF(UtiLite_LIBRARIES AND UtiLite_Qt)
		SET(UtiLite_LIBRARIES ${UtiLite_LIBRARIES} ${UtiLite_Qt})
	ENDIF(UtiLite_LIBRARIES AND UtiLite_Qt)
	IF(UtiLite_LIBRARIES AND UtiLite_Audio)
		SET(UtiLite_LIBRARIES ${UtiLite_LIBRARIES} ${UtiLite_Audio})
	ENDIF(UtiLite_LIBRARIES AND UtiLite_Audio)
	IF(UtiLite_LIBRARIES AND UtiLite_Cv)
		SET(UtiLite_LIBRARIES ${UtiLite_LIBRARIES} ${UtiLite_Cv})
	ENDIF(UtiLite_LIBRARIES AND UtiLite_Cv)
	IF(UtiLite_LIBRARIES AND UtiLite_Cvqt)
		SET(UtiLite_LIBRARIES ${UtiLite_LIBRARIES} ${UtiLite_Cvqt})
	ENDIF(UtiLite_LIBRARIES AND UtiLite_Cvqt)
	
	IF (UtiLite_INCLUDE_DIRS AND UtiLite_LIBRARIES)
	   SET(UtiLite_FOUND TRUE)
	ENDIF (UtiLite_INCLUDE_DIRS AND UtiLite_LIBRARIES)
ENDIF(URESOURCEGENERATOR_EXEC)

IF (UtiLite_FOUND)
   # show which UtiLite was found only if not quiet
   IF (NOT UtiLite_FIND_QUIETLY)
      MESSAGE(STATUS "Found UtiLite ${UtiLite_VERSION}")
   ENDIF (NOT UtiLite_FIND_QUIETLY)
   # backward compatibility...
   SET(UTILITE_FOUND ${UtiLite_FOUND})
   SET(UTILITE_INCLUDE_DIRS ${UtiLite_INCLUDE_DIRS})
   SET(UTILITE_LIBRARIES ${UtiLite_LIBRARIES})
   SET(UTILITE_INCLUDE_DIR ${UtiLite_INCLUDE_DIRS})
   SET(UTILITE_LIBRARY ${UtiLite_LIBRARIES})
ELSE ()
   # fatal error if UtiLite is required but not found
   IF (UtiLite_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "Could not find UtiLite. Verify your PATH if it is already installed or download it at http://utilite.googlecode.com")
   ENDIF (UtiLite_FIND_REQUIRED)
ENDIF ()
