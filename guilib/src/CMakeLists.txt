

### Qt Gui stuff ###
SET(headers_ui 
	../include/${PROJECT_PREFIX}/gui/MainWindow.h
	../include/${PROJECT_PREFIX}/gui/PreferencesDialog.h
	./AboutDialog.h
	./ConsoleWidget.h
	./ImageView.h
	./Plot.h
	./PdfPlot.h
	./StatsToolBox.h
	./DetailedProgressDialog.h
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
	./Plot.cpp
	./PdfPlot.cpp
	./StatsToolBox.cpp
	./DetailedProgressDialog.cpp
	./AboutDialog.cpp
	./ConsoleWidget.cpp
	${moc_srcs} 
	${moc_uis} 
	${srcs_qrc}
)

SET(INCLUDE_DIRS
	${CMAKE_CURRENT_SOURCE_DIR}/../include
	${CMAKE_CURRENT_SOURCE_DIR}
	${UTILITE_INCLUDE_DIR}
	${OpenCV_INCLUDE_DIRS}
	${CMAKE_CURRENT_BINARY_DIR} # for qt ui generated in binary dir
)

INCLUDE(${QT_USE_FILE})

SET(LIBRARIES
    ${UTILITE_LIBRARY}
    ${QT_LIBRARIES} 
	${OpenCV_LIBS} 
)

#include files
INCLUDE_DIRECTORIES(${INCLUDE_DIRS})

IF(WIN32)
    IF(BUILD_SHARED_LIBS)   
	   ADD_DEFINITIONS(-DRTABMAP_EXPORTS) 
	ELSE()
	   ADD_DEFINITIONS(-DRTABMAP_EXPORTS_STATIC)
	ENDIF()
ENDIF(WIN32)

# create an executable file named "AvpdGui" from the source files
ADD_LIBRARY(guilib ${SRC_FILES})
# Linking with Qt libraries
TARGET_LINK_LIBRARIES(guilib corelib ${LIBRARIES})

SET_TARGET_PROPERTIES(
guilib 
PROPERTIES
 OUTPUT_NAME ${PROJECT_PREFIX}_gui
 INSTALL_NAME_DIR ${CMAKE_INSTALL_PREFIX}/lib
)

INSTALL(TARGETS guilib
        RUNTIME DESTINATION bin COMPONENT runtime
        LIBRARY DESTINATION lib COMPONENT devel
        ARCHIVE DESTINATION lib COMPONENT devel)
        
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../include/ DESTINATION include/ COMPONENT devel FILES_MATCHING PATTERN "*.h" PATTERN ".svn" EXCLUDE)
