
ADD_SUBDIRECTORY( BOWMapping )

IF(TARGET rtabmap_gui)
	ADD_SUBDIRECTORY( RGBDMapping )
ELSE()
   MESSAGE(STATUS "RTAB-Map GUI lib is not built, the RGBDMapping example will not be built...")
ENDIF()


