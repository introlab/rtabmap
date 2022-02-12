

### Qt Gui stuff ###
SET(headers_ui 
	../include/${PROJECT_PREFIX}/gui/MainWindow.h
	../include/${PROJECT_PREFIX}/gui/PreferencesDialog.h
	../include/${PROJECT_PREFIX}/gui/DatabaseViewer.h
	../include/${PROJECT_PREFIX}/gui/AboutDialog.h
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
	../include/${PROJECT_PREFIX}/gui/ExportDialog.h
	../include/${PROJECT_PREFIX}/gui/PostProcessingDialog.h
	../include/${PROJECT_PREFIX}/gui/ExportCloudsDialog.h
	../include/${PROJECT_PREFIX}/gui/ExportBundlerDialog.h
	../include/${PROJECT_PREFIX}/gui/MapVisibilityWidget.h
	../include/${PROJECT_PREFIX}/gui/GraphViewer.h
	../include/${PROJECT_PREFIX}/gui/CreateSimpleCalibrationDialog.h
	../include/${PROJECT_PREFIX}/gui/ParametersToolBox.h
	../include/${PROJECT_PREFIX}/gui/DepthCalibrationDialog.h
	../include/${PROJECT_PREFIX}/gui/EditConstraintDialog.h
	./3rdParty/QMultiComboBox.h
	../include/${PROJECT_PREFIX}/gui/TexturingState.h
	../include/${PROJECT_PREFIX}/gui/RecoveryState.h
	../include/${PROJECT_PREFIX}/gui/EditDepthArea.h
	../include/${PROJECT_PREFIX}/gui/EditMapArea.h
	../include/${PROJECT_PREFIX}/gui/MultiSessionLocWidget.h
	../include/${PROJECT_PREFIX}/gui/MultiSessionLocSubView.h
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
    ./ui/calibrationDialog.ui
    ./ui/createSimpleCalibrationDialog.ui
    ./ui/depthCalibrationDialog.ui
    ./ui/exportBundlerDialog.ui
    ./ui/editConstraintDialog.ui
    ./ui/multiSessionLocSubView.ui
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
	./CloudViewerCellPicker.cpp
	./CloudViewerInteractorStyle.cpp
	./OdometryViewer.cpp
	./LoopClosureViewer.cpp
	./DataRecorder.cpp
	./CameraViewer.cpp
	./CalibrationDialog.cpp
	./ExportDialog.cpp
	./ExportBundlerDialog.cpp
	./PostProcessingDialog.cpp
	./ExportCloudsDialog.cpp
	./EditConstraintDialog.cpp
	./MapVisibilityWidget.cpp
	./GraphViewer.cpp
	./EditDepthArea.cpp
	./EditMapArea.cpp
	./MultiSessionLocWidget.cpp
	./MultiSessionLocSubView.cpp
	./CreateSimpleCalibrationDialog.cpp
	./ParametersToolBox.cpp
	./DepthCalibrationDialog.cpp
	./3rdParty/QMultiComboBox.cpp
	./opencv/vtkImageMatSource.cpp
	
	${moc_srcs} 
	${moc_uis} 
	${srcs_qrc}
)

# to get includes in visual studio
IF(MSVC)
    FILE(GLOB HEADERS
        ../include/${PROJECT_PREFIX}/gui/*.h
    )
    SET(SRC_FILES ${SRC_FILES} ${HEADERS})
ENDIF(MSVC)

SET(INCLUDE_DIRS
	${PROJECT_BINARY_DIR}/corelib/include
	${PROJECT_SOURCE_DIR}/corelib/include
	${PROJECT_SOURCE_DIR}/utilite/include
	${CMAKE_CURRENT_SOURCE_DIR}/../include
	${CMAKE_CURRENT_SOURCE_DIR}
	${OpenCV_INCLUDE_DIRS}
	${CMAKE_CURRENT_BINARY_DIR} # for qt ui generated in binary dir
	${PCL_INCLUDE_DIRS}
)

# Hack as CameraRealsense2.h needs realsense2 include dir
IF(realsense2_FOUND)
	SET(INCLUDE_DIRS
		${INCLUDE_DIRS}
		${realsense2_INCLUDE_DIRS}
	)
ENDIF(realsense2_FOUND)
# Hack as CameraK4A.h needs k4a include dir
IF(k4a_FOUND)
	SET(INCLUDE_DIRS
		${INCLUDE_DIRS}
		${k4a_INCLUDE_DIRS}
	)
ENDIF(k4a_FOUND)

IF(QT4_FOUND)
    INCLUDE(${QT_USE_FILE})
ENDIF(QT4_FOUND)

SET(LIBRARIES
    ${QT_LIBRARIES} 
	${OpenCV_LIBS} 
	${PCL_LIBRARIES}
)

IF(octomap_FOUND)
	SET(INCLUDE_DIRS
		${INCLUDE_DIRS}
		${OCTOMAP_INCLUDE_DIRS}
	)
	SET(LIBRARIES
		${LIBRARIES}
		${OCTOMAP_LIBRARIES}
	)
ENDIF(octomap_FOUND)

IF(CPUTSDF_FOUND)
	SET(INCLUDE_DIRS
		${INCLUDE_DIRS}
		${CPUTSDF_INCLUDE_DIRS}
	)
	SET(LIBRARIES
		${LIBRARIES}
		${CPUTSDF_LIBRARIES}
	)
ENDIF(CPUTSDF_FOUND)

IF(open_chisel_FOUND)
	SET(INCLUDE_DIRS
		${INCLUDE_DIRS}
		${open_chisel_INCLUDE_DIRS}
	)
	SET(LIBRARIES
		${LIBRARIES}
		${open_chisel_LIBRARIES}
	)
ENDIF(open_chisel_FOUND)

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
    IF(Qt5Svg_FOUND)
        QT5_USE_MODULES(rtabmap_gui Widgets Core Gui Svg PrintSupport)
    ELSE()
        QT5_USE_MODULES(rtabmap_gui Widgets Core Gui PrintSupport)
    ENDIF()
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
