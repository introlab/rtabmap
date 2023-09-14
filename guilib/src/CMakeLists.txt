

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

SET(qrc 
    ./GuiLib.qrc
)

IF(QT4_FOUND OR Qt5_FOUND OR Qt6_FOUND)
    set(CMAKE_AUTOUIC_SEARCH_PATHS ./ui/)
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
	${qrc}
	${headers_ui}
)

# to get includes in visual studio
IF(MSVC)
    FILE(GLOB HEADERS
        ../include/${PROJECT_PREFIX}/gui/*.h
    )
    SET(SRC_FILES ${SRC_FILES} ${HEADERS})
ENDIF(MSVC)

SET(INCLUDE_DIRS
	${CMAKE_CURRENT_SOURCE_DIR}/../include
	${CMAKE_CURRENT_SOURCE_DIR}
	${CMAKE_CURRENT_BINARY_DIR} # for qt ui generated in binary dir
)

IF(QT4_FOUND)
    INCLUDE(${QT_USE_FILE})
ENDIF(QT4_FOUND)

SET(LIBRARIES "")
SET(PUBLIC_LIBRARIES "")
SET(PUBLIC_INCLUDE_DIRS "")

IF(Qt6_FOUND)
    SET(PUBLIC_LIBRARIES ${PUBLIC_LIBRARIES} Qt6::Widgets Qt6::Core Qt6::Gui Qt6::OpenGL)
    SET(LIBRARIES ${LIBRARIES} Qt6::PrintSupport)
    IF(Qt6Svg_FOUND)
        SET(LIBRARIES ${LIBRARIES} Qt6::Svg)
    ENDIF()
    SET(PUBLIC_INCLUDE_DIRS
        ${PUBLIC_INCLUDE_DIRS}
        ${Qt6Widgets_INCLUDE_DIRS}
        ${Qt6Core_INCLUDE_DIRS}
        ${Qt6Gui_INCLUDE_DIRS}
        ${Qt6OpenGL_INCLUDE_DIRS})
ELSEIF(Qt5_FOUND)
    SET(PUBLIC_LIBRARIES ${PUBLIC_LIBRARIES} Qt5::Widgets Qt5::Core Qt5::Gui Qt5::OpenGL)
    SET(LIBRARIES ${LIBRARIES} Qt5::PrintSupport)
    IF(Qt5Svg_FOUND)
        SET(LIBRARIES ${LIBRARIES} Qt5::Svg)
    ENDIF()
    SET(PUBLIC_INCLUDE_DIRS
        ${PUBLIC_INCLUDE_DIRS}
        ${Qt5Widgets_INCLUDE_DIRS}
        ${Qt5Core_INCLUDE_DIRS}
        ${Qt5Gui_INCLUDE_DIRS}
        ${Qt5OpenGL_INCLUDE_DIRS})
ELSE()
    SET(PUBLIC_LIBRARIES ${PUBLIC_LIBRARIES} ${QTCORE_LIBRARY} ${QTGUI_LIBRARY})
    IF(QTSVG_FOUND)
        SET(LIBRARIES ${LIBRARIES} ${QTSVG_LIBRARY})
    ENDIF()
    SET(PUBLIC_INCLUDE_DIRS
        ${QTCORE_INCLUDE_DIRS}
        ${QTGUI_INCLUDE_DIRS})
ENDIF()

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
ADD_LIBRARY(rtabmap::gui ALIAS rtabmap_gui)

generate_export_header(rtabmap_gui
 DEPRECATED_MACRO_NAME RTABMAP_DEPRECATED)

target_include_directories(rtabmap_gui PUBLIC 
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include;${CMAKE_CURRENT_BINARY_DIR}/include;${PUBLIC_INCLUDE_DIRS}>"
  "$<INSTALL_INTERFACE:${INSTALL_INCLUDE_DIR};${PUBLIC_INCLUDE_DIRS}>")

TARGET_LINK_LIBRARIES(rtabmap_gui 
  PUBLIC
    rtabmap_core ${PUBLIC_LIBRARIES}
  PRIVATE
    ${LIBRARIES}
)

SET_TARGET_PROPERTIES(
     rtabmap_gui
   PROPERTIES
     VERSION ${RTABMAP_VERSION}
     SOVERSION ${RTABMAP_MAJOR_VERSION}.${RTABMAP_MINOR_VERSION}
     EXPORT_NAME "gui"
)

INSTALL(TARGETS rtabmap_gui EXPORT rtabmap_guiTargets
        RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT runtime
        LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT devel
        ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT devel)

configure_file(
  ${CMAKE_CURRENT_BINARY_DIR}/rtabmap_gui_export.h
  ${CMAKE_CURRENT_BINARY_DIR}/include/${PROJECT_PREFIX}/gui/rtabmap_gui_export.h
  COPYONLY)
  
install(
  DIRECTORY
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/${PROJECT_PREFIX}
    ${CMAKE_CURRENT_BINARY_DIR}/include/${PROJECT_PREFIX}
  DESTINATION
    ${INSTALL_INCLUDE_DIR}
  COMPONENT
    devel
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)
export(EXPORT rtabmap_guiTargets
  FILE "${CMAKE_CURRENT_BINARY_DIR}/../../${PROJECT_NAME}_guiTargets.cmake"
  NAMESPACE rtabmap::
)
install(EXPORT rtabmap_guiTargets
  FILE
    ${PROJECT_NAME}_guiTargets.cmake
  DESTINATION
    ${INSTALL_CMAKE_DIR}
  NAMESPACE rtabmap::
  COMPONENT
    devel
)
