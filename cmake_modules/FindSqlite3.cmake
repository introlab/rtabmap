# - Find Sqlite3
# This module finds an installed Sqlite3 package.
#
# It sets the following variables:
#  Sqlite3_FOUND       - Set to false, or undefined, if Sqlite3 isn't found.
#  Sqlite3_INCLUDE_DIR - The Sqlite3 include directory.
#  Sqlite3_LIBRARY     - The Sqlite3 library to link against.

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

