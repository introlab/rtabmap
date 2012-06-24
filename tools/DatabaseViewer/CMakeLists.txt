### Qt Gui stuff ###
SET(headers_ui 
	./MainWindow.h
)

SET(uis
    ./ui/MainWindow.ui
)

#Generate .h files from the .ui files
QT4_WRAP_UI(moc_uis ${uis})

#This will generate moc_* for Qt
QT4_WRAP_CPP(moc_srcs ${headers_ui})
### Qt Gui stuff  end###


SET(SRC_FILES 
	./main.cpp
	./MainWindow.cpp
	${moc_srcs} 
	${moc_uis} 
)

SET(INCLUDE_DIRS
	${PROJECT_SOURCE_DIR}/corelib/include
	${PROJECT_SOURCE_DIR}/guilib/include
	${UTILITE_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	${CMAKE_CURRENT_BINARY_DIR} # for qt ui generated in binary dir
)

INCLUDE(${QT_USE_FILE})

SET(LIBRARIES
    ${UTILITE_LIBRARIES}
    ${QT_LIBRARIES} 
	${OpenCV_LIBRARIES} 
	#${QWT5_LIBRARY} 
)

#include files
INCLUDE_DIRECTORIES(${INCLUDE_DIRS})

# Add binary called "databaseViewer" that is built from the source file "main.cpp".
# The extension is automatically found.
ADD_EXECUTABLE(databaseViewer WIN32 ${SRC_FILES})
TARGET_LINK_LIBRARIES(databaseViewer rtabmap_corelib rtabmap_guilib ${LIBRARIES})

SET_TARGET_PROPERTIES( databaseViewer 
  PROPERTIES OUTPUT_NAME ${PROJECT_PREFIX}-databaseViewer)

INSTALL(TARGETS databaseViewer
        RUNTIME DESTINATION bin COMPONENT runtime
        LIBRARY DESTINATION lib COMPONENT devel
        ARCHIVE DESTINATION lib COMPONENT devel)