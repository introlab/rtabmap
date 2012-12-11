
ADD_SUBDIRECTORY( ConsoleApp )
ADD_SUBDIRECTORY( ImagesJoiner )
ADD_SUBDIRECTORY( ImagesDbExtractor )
ADD_SUBDIRECTORY( VocabularyComparison )

IF(QT4_FOUND AND QT_QTCORE_FOUND AND QT_QTGUI_FOUND)
	ADD_SUBDIRECTORY( DatabaseViewer )
	ADD_SUBDIRECTORY( EpipolarGeometry )
ELSE()
   MESSAGE(STATUS "[WARNING] Qt4 not found, the databaseViewer and epipolarGeometry programs will not be built...")
ENDIF()