
ADD_SUBDIRECTORY( ConsoleApp )
ADD_SUBDIRECTORY( ImagesJoiner )
ADD_SUBDIRECTORY( VocabularyComparison )
ADD_SUBDIRECTORY( OdometryViewer )
ADD_SUBDIRECTORY( DataRecorder )
ADD_SUBDIRECTORY( ExtractObject )

IF(TARGET rtabmap_gui)
	ADD_SUBDIRECTORY( DatabaseViewer )
	ADD_SUBDIRECTORY( EpipolarGeometry )
ELSE()
   MESSAGE(STATUS "RTAB-Map GUI lib is not built, the databaseViewer and epipolarGeometry programs will not be built...")
ENDIF()