
ADD_SUBDIRECTORY( ConsoleApp )
ADD_SUBDIRECTORY( ImagesJoiner )
ADD_SUBDIRECTORY( ImagesDbExtractor )
ADD_SUBDIRECTORY( VocabularyComparison )

IF(TARGET rtabmap_guilib)
	ADD_SUBDIRECTORY( DatabaseViewer )
	ADD_SUBDIRECTORY( EpipolarGeometry )
ELSE()
   MESSAGE(STATUS "RTAB-Map GUI lib is not built, the databaseViewer and epipolarGeometry programs will not be built...")
ENDIF()