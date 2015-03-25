
ADD_SUBDIRECTORY( BOWMapping )

IF(TARGET rtabmap_gui)
   ADD_SUBDIRECTORY( RGBDMapping )
	
   # Only on Linux
   IF(NOT WIN32 AND NOT APPLE)
      ADD_SUBDIRECTORY( WifiMapping )
   ENDIF(NOT WIN32 AND NOT APPLE)

ELSE()
   MESSAGE(STATUS "RTAB-Map GUI lib is not built, the RGBDMapping and WifiMapping examples will not be built...")
ENDIF()


