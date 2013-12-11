
ADD_SUBDIRECTORY( ConsoleApp )
ADD_SUBDIRECTORY( ImagesJoiner )
ADD_SUBDIRECTORY( ImagesDbExtractor )
ADD_SUBDIRECTORY( VocabularyComparison )

#On linux, we use ros package as OpenniCamera
IF(WIN32 OR APPLE)
	ADD_SUBDIRECTORY( OdometryViewer )
	ADD_SUBDIRECTORY( DataRecorder )
ENDIF(WIN32 OR APPLE)

IF(TARGET rtabmap_gui)
	ADD_SUBDIRECTORY( DatabaseViewer )
	ADD_SUBDIRECTORY( EpipolarGeometry )
ELSE()
   MESSAGE(STATUS "RTAB-Map GUI lib is not built, the databaseViewer and epipolarGeometry programs will not be built...")
ENDIF()