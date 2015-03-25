
ADD_SUBDIRECTORY( BOWMapping )

IF(TARGET rtabmap_gui)
   ADD_SUBDIRECTORY( RGBDMapping )
	
   # Only on Linux and Windows
   IF(NOT APPLE)
      ADD_SUBDIRECTORY( WifiMapping )
   ENDIF(NOT APPLE)

ELSE()
   MESSAGE(STATUS "RTAB-Map GUI lib is not built, the RGBDMapping and WifiMapping examples will not be built...")
ENDIF()


