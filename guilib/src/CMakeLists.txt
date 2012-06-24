

### Qt Gui stuff ###
SET(headers_ui 
	../include/${PROJECT_PREFIX}/gui/MainWindow.h
	../include/${PROJECT_PREFIX}/gui/PreferencesDialog.h
	./AboutDialog.h
	./ConsoleWidget.h
	../include/${PROJECT_PREFIX}/gui/ImageView.h
	./PdfPlot.h
	./StatsToolBox.h
	./DetailedProgressDialog.h
	./TwistWidget.h
)

SET(uis
    ./ui/mainWindow.ui
    ./ui/preferencesDialog.ui
    ./ui/aboutDialog.ui
    ./ui/consoleWidget.ui
)

SET(qrc 
    ./GuiLib.qrc
)

# generate rules for building source files from the resources
QT4_ADD_RESOURCES(srcs_qrc ${qrc})

#Generate .h files from the .ui files
QT4_WRAP_UI(moc_uis ${uis})

#This will generate moc_* for Qt
QT4_WRAP_CPP(moc_srcs ${headers_ui})
### Qt Gui stuff  end###




SET(SRC_FILES 
	./MainWindow.cpp
	./PreferencesDialog.cpp
	./KeypointItem.cpp
	./qtipl.cpp
	./ImageView.cpp
	./PdfPlot.cpp
	./StatsToolBox.cpp
	./DetailedProgressDialog.cpp
	./AboutDialog.cpp
	./ConsoleWidget.cpp
	./TwistWidget.cpp
	${moc_srcs} 
	${moc_uis} 
	${srcs_qrc}
)

SET(INCLUDE_DIRS
	${PROJECT_SOURCE_DIR}/corelib/include
	${CMAKE_CURRENT_SOURCE_DIR}/../include
	${CMAKE_CURRENT_SOURCE_DIR}
	${UTILITE_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	${CMAKE_CURRENT_BINARY_DIR} # for qt ui generated in binary dir
)

INCLUDE(${QT_USE_FILE})

SET(LIBRARIES
    ${UTILITE_LIBRARIES}
    ${QT_LIBRARIES} 
	${OpenCV_LIBS} 
)

#include files
INCLUDE_DIRECTORIES(${INCLUDE_DIRS})

# create a library from the source files
ADD_LIBRARY(rtabmap_guilib ${SRC_FILES})
# Linking with Qt libraries
TARGET_LINK_LIBRARIES(rtabmap_guilib rtabmap_corelib ${LIBRARIES})

SET_TARGET_PROPERTIES(
rtabmap_guilib 
PROPERTIES
 OUTPUT_NAME ${PROJECT_PREFIX}_gui
 INSTALL_NAME_DIR ${CMAKE_INSTALL_PREFIX}/lib
)

INSTALL(TARGETS rtabmap_guilib
        RUNTIME DESTINATION bin COMPONENT runtime
        LIBRARY DESTINATION lib COMPONENT devel
        ARCHIVE DESTINATION lib COMPONENT devel)
        
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../include/ DESTINATION include/ COMPONENT devel FILES_MATCHING PATTERN "*.h" PATTERN ".svn" EXCLUDE)
