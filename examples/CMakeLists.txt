
ADD_SUBDIRECTORY( BOWMapping )

IF(TARGET rtabmap_gui)
   ADD_SUBDIRECTORY( RGBDMapping )
   ADD_SUBDIRECTORY( WifiMapping )
   ADD_SUBDIRECTORY( NoEventsExample )
   IF(PCL_VERSION VERSION_GREATER_EQUAL "1.8")
      ADD_SUBDIRECTORY( LidarMapping )
   ENDIF()
ELSE()
   MESSAGE(STATUS "RTAB-Map GUI lib is not built, the RGBDMapping and WifiMapping examples will not be built...")
ENDIF()


