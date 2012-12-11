# - Find Sqlite3
# This module finds an installed Sqlite3 package.
#
# It sets the following variables:
#  SQLITE3_FOUND       - Set to false, or undefined, if Sqlite3 isn't found.
#  SQLITE3_INCLUDE_DIR - The Sqlite3 include directory.
#  SQLITE3_LIBRARY     - The Sqlite3 library to link against.

SET(SQLITE3_VERSION_REQUIRED "3.6.0")

IF(UNIX)
	FIND_PROGRAM(SQLITE3_EXEC NAME sqlite3 PATHS)  
	IF(SQLITE3_EXEC)  
        MESSAGE(STATUS "Found Sqlite3 executable : ${SQLITE3_EXEC}")
		EXECUTE_PROCESS(COMMAND ${SQLITE3_EXEC} --version 
				   	    OUTPUT_VARIABLE SQLITE3_VERSION
						OUTPUT_STRIP_TRAILING_WHITESPACE
						WORKING_DIRECTORY "./"
   		)
		IF(SQLITE3_VERSION VERSION_LESS SQLITE3_VERSION_REQUIRED)
		    MESSAGE(FATAL_ERROR "Sqlite ${SQLITE3_VERSION} found, but version ${SQLITE3_VERSION_REQUIRED} minimum is required")
		ENDIF(SQLITE3_VERSION VERSION_LESS SQLITE3_VERSION_REQUIRED)
	ELSE(SQLITE3_EXEC)
		MESSAGE(FATAL_ERROR "Could not find Sqlite3 executable")
	ENDIF(SQLITE3_EXEC)
ENDIF(UNIX)

FIND_PATH(SQLITE3_INCLUDE_DIR sqlite3.h)

FIND_LIBRARY(SQLITE3_LIBRARY NAMES sqlite3.dll sqlite3)

IF (SQLITE3_INCLUDE_DIR AND SQLITE3_LIBRARY)
   SET(SQLITE3_FOUND TRUE)
ENDIF (SQLITE3_INCLUDE_DIR AND SQLITE3_LIBRARY)

IF (SQLITE3_FOUND)
   # show which Sqlite3 was found only if not quiet
   IF (NOT Sqlite3_FIND_QUIETLY)
      MESSAGE(STATUS "Found Sqlite3")
   ENDIF (NOT Sqlite3_FIND_QUIETLY)
ELSE (SQLITE3_FOUND)
   # fatal error if Sqlite3 is required but not found
   IF (Sqlite3_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "Could not find Sqlite3")
   ENDIF (Sqlite3_FIND_REQUIRED)
ENDIF (SQLITE3_FOUND)

