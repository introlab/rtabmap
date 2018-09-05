# - Find Sqlite3
# This module finds an installed Sqlite3 package.
#
# It sets the following variables:
#  Sqlite3_FOUND       - Set to false, or undefined, if Sqlite3 isn't found.
#  Sqlite3_INCLUDE_DIR - The Sqlite3 include directory.
#  Sqlite3_LIBRARY     - The Sqlite3 library to link against.

SET(Sqlite3_VERSION_REQUIRED "3.6.0")

IF(UNIX)
	FIND_PROGRAM(Sqlite3_EXEC NAME sqlite3 PATHS $ENV{Sqlite3_ROOT_DIR}/bin $ENV{Sqlite3_ROOT_DIR})  
	IF(Sqlite3_EXEC)  
        MESSAGE(STATUS "Found Sqlite3 executable : ${Sqlite3_EXEC}")
		EXECUTE_PROCESS(COMMAND ${Sqlite3_EXEC} --version 
				   	    OUTPUT_VARIABLE Sqlite3_VERSION
						OUTPUT_STRIP_TRAILING_WHITESPACE
						WORKING_DIRECTORY "./"
   		)
		IF(Sqlite3_VERSION VERSION_LESS Sqlite3_VERSION_REQUIRED)
		    MESSAGE(FATAL_ERROR "Sqlite ${Sqlite3_VERSION} found, but version ${Sqlite3_VERSION_REQUIRED} minimum is required")
		ENDIF(Sqlite3_VERSION VERSION_LESS Sqlite3_VERSION_REQUIRED)
	ELSE(Sqlite3_EXEC)
		MESSAGE(FATAL_ERROR "Could not find Sqlite3 executable")
	ENDIF(Sqlite3_EXEC)
ENDIF(UNIX)

FIND_PATH(Sqlite3_INCLUDE_DIR sqlite3.h PATHS $ENV{Sqlite3_ROOT_DIR}/include $ENV{Sqlite3_ROOT_DIR})

FIND_LIBRARY(Sqlite3_LIBRARY NAMES sqlite3 PATHS $ENV{Sqlite3_ROOT_DIR}/lib $ENV{Sqlite3_ROOT_DIR})

IF (Sqlite3_INCLUDE_DIR AND Sqlite3_LIBRARY)
   SET(Sqlite3_FOUND TRUE)
   SET(Sqlite3_INCLUDE_DIRS ${Sqlite3_INCLUDE_DIR})
   SET(Sqlite3_LIBRARIES ${Sqlite3_LIBRARY})
ENDIF (Sqlite3_INCLUDE_DIR AND Sqlite3_LIBRARY)

IF (Sqlite3_FOUND)
   # show which Sqlite3 was found only if not quiet
   IF (NOT Sqlite3_FIND_QUIETLY)
      MESSAGE(STATUS "Found Sqlite3: ${Sqlite3_INCLUDE_DIRS} ${Sqlite3_LIBRARIES}")
   ENDIF (NOT Sqlite3_FIND_QUIETLY)
ELSE (Sqlite3_FOUND)
   # fatal error if Sqlite3 is required but not found
   IF (Sqlite3_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "Could not find Sqlite3")
   ENDIF (Sqlite3_FIND_REQUIRED)
ENDIF (Sqlite3_FOUND)

