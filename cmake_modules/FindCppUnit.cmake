# - Find CppUnit
# This module finds an installed CppUnit package.
#
# It sets the following variables:
#  CPPUNIT_FOUND       - Set to false, or undefined, if CppUnit isn't found.
#  CPPUNIT_INCLUDE_DIR - The CppUnit include directory.
#  CPPUNIT_LIBRARY     - The CppUnit library to link against.
#  CPPUNIT_DEFINITIONS - The CppUnit definitions.

FIND_PATH(CPPUNIT_INCLUDE_DIR cppunit/Test.h)

FIND_LIBRARY(CPPUNIT_LIBRARY NAMES cppunit_dll cppunit)

IF (CPPUNIT_INCLUDE_DIR AND CPPUNIT_LIBRARY)
   SET(CPPUNIT_FOUND TRUE)
   SET(CPPUNIT_DEFINITIONS "-DCPPUNIT_DLL")
ENDIF (CPPUNIT_INCLUDE_DIR AND CPPUNIT_LIBRARY)

IF (CPPUNIT_FOUND)
   # show which CppUnit was found only if not quiet
   IF (NOT CppUnit_FIND_QUIETLY)
      MESSAGE(STATUS "Found CppUnit")
   ENDIF (NOT CppUnit_FIND_QUIETLY)
ELSE (CPPUNIT_FOUND)
   # fatal error if CppUnit is required but not found
   IF (CppUnit_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "Could not find CppUnit")
   ENDIF (CppUnit_FIND_REQUIRED)
ENDIF (CPPUNIT_FOUND)

