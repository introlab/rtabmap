

### Qt Gui stuff ###
SET(headers_ui 
	../include/${PROJECT_PREFIX}/gui/MainWindow.h
	../include/${PROJECT_PREFIX}/gui/PreferencesDialog.h
	../include/${PROJECT_PREFIX}/gui/DatabaseViewer.h
	./AboutDialog.h
	../include/${PROJECT_PREFIX}/gui/ConsoleWidget.h
	../include/${PROJECT_PREFIX}/gui/ImageView.h
	../include/${PROJECT_PREFIX}/gui/PdfPlot.h
	../include/${PROJECT_PREFIX}/gui/StatsToolBox.h
	../include/${PROJECT_PREFIX}/gui/ProgressDialog.h
	../include/${PROJECT_PREFIX}/utilite/UPlot.h
	../include/${PROJECT_PREFIX}/utilite/UImageView.h
	../include/${PROJECT_PREFIX}/gui/CloudViewer.h
	../include/${PROJECT_PREFIX}/gui/OdometryViewer.h
	../include/${PROJECT_PREFIX}/gui/LoopClosureViewer.h
	../include/${PROJECT_PREFIX}/gui/DataRecorder.h
	../include/${PROJECT_PREFIX}/gui/CameraViewer.h
	../include/${PROJECT_PREFIX}/gui/CalibrationDialog.h
	./ExportDialog.h
	./PostProcessingDialog.h
	./ExportCloudsDialog.h
	./ExportScansDialog.h
	./MapVisibilityWidget.h
	../include/${PROJECT_PREFIX}/gui/GraphViewer.h
	./CreateSimpleCalibrationDialog.h
	./ParametersToolBox.h
)

SET(uis
    ./ui/mainWindow.ui
    ./ui/preferencesDialog.ui
    ./ui/aboutDialog.ui
    ./ui/consoleWidget.ui
    ./ui/DatabaseViewer.ui
    ./ui/loopClosureViewer.ui
    ./ui/exportDialog.ui
    ./ui/postProcessingDialog.ui
    ./ui/exportCloudsDialog.ui
    ./ui/exportScansDialog.ui
    ./ui/calibrationDialog.ui
    ./ui/createSimpleCalibrationDialog.ui
)

SET(qrc 
    ./GuiLib.qrc
)

IF(QT4_FOUND)
    # generate rules for building source files from the resources
    QT4_ADD_RESOURCES(srcs_qrc ${qrc})

    #Generate .h files from the .ui files
    QT4_WRAP_UI(moc_uis ${uis})

    #This will generate moc_* for Qt
    QT4_WRAP_CPP(moc_srcs ${headers_ui})
    ### Qt Gui stuff  end###
ELSE()
    QT5_ADD_RESOURCES(srcs_qrc ${qrc})
    QT5_WRAP_UI(moc_uis ${uis})
    QT5_WRAP_CPP(moc_srcs ${headers_ui})
ENDIF()


SET(SRC_FILES 
	./MainWindow.cpp
	./PreferencesDialog.cpp
	./KeypointItem.cpp
	./ImageView.cpp
	./PdfPlot.cpp
	./StatsToolBox.cpp
	./ProgressDialog.cpp
	./AboutDialog.cpp
	./ConsoleWidget.cpp
	./DatabaseViewer.cpp
	./utilite/UPlot.cpp
	./CloudViewer.cpp
	./OdometryViewer.cpp
	./LoopClosureViewer.cpp
	./DataRecorder.cpp
	./CameraViewer.cpp
	./CalibrationDialog.cpp
	./ExportDialog.cpp
	./PostProcessingDialog.cpp
	./ExportCloudsDialog.cpp
	./ExportScansDialog.cpp
	./MapVisibilityWidget.cpp
	./GraphViewer.cpp
	./CreateSimpleCalibrationDialog.cpp
	./ParametersToolBox.cpp
	${moc_srcs} 
	${moc_uis} 
	${srcs_qrc}
)

SET(INCLUDE_DIRS
	${PROJECT_SOURCE_DIR}/corelib/include
	${PROJECT_SOURCE_DIR}/utilite/include
	${CMAKE_CURRENT_SOURCE_DIR}/../include
	${CMAKE_CURRENT_SOURCE_DIR}
	${OpenCV_INCLUDE_DIRS}
	${CMAKE_CURRENT_BINARY_DIR} # for qt ui generated in binary dir
	${PCL_INCLUDE_DIRS}
)

IF(QT4_FOUND)
    INCLUDE(${QT_USE_FILE})
ENDIF(QT4_FOUND)

SET(LIBRARIES
    ${QT_LIBRARIES} 
	${OpenCV_LIBS} 
	${PCL_LIBRARIES}
)

IF(OCTOMAP_FOUND)
	SET(INCLUDE_DIRS
		${INCLUDE_DIRS}
		${OCTOMAP_INCLUDE_DIRS}
	)
	SET(LIBRARIES
		${LIBRARIES}
		${OCTOMAP_LIBRARIES}
	)
ENDIF(OCTOMAP_FOUND)

IF(VTK_USE_QVTK)
	SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${QVTK_INCLUDE_DIR})
	SET(LIBRARIES ${LIBRARIES} ${QVTK_LIBRARY})
ENDIF(VTK_USE_QVTK)

#include files
INCLUDE_DIRECTORIES(${INCLUDE_DIRS})

add_definitions(${PCL_DEFINITIONS})

# create a library from the source files
ADD_LIBRARY(rtabmap_gui ${SRC_FILES})
# Linking with Qt libraries

TARGET_LINK_LIBRARIES(rtabmap_gui rtabmap_core rtabmap_utilite ${LIBRARIES})
IF(Qt5_FOUND)
    QT5_USE_MODULES(rtabmap_gui Widgets Core Gui Svg PrintSupport)
ENDIF(Qt5_FOUND)

SET_TARGET_PROPERTIES(
     rtabmap_gui
   PROPERTIES
     VERSION ${RTABMAP_VERSION}
     SOVERSION ${RTABMAP_MAJOR_VERSION}.${RTABMAP_MINOR_VERSION}
)

INSTALL(TARGETS rtabmap_gui
        RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT runtime
        LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT devel
        ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT devel)
        
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../include/ DESTINATION "${INSTALL_INCLUDE_DIR}" COMPONENT devel FILES_MATCHING PATTERN "*.h" PATTERN ".svn" EXCLUDE)
