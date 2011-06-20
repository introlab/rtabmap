
ADD_DEFINITIONS(-DPLOT_WIDGET_OUT_OF_LIB) 

### Qt Gui stuff ###
SET(headers_ui 
	../src/Plot.h
	MainWindow.h
)

#This will generate moc_* for Qt
QT4_WRAP_CPP(moc_srcs ${headers_ui})
### Qt Gui stuff  end###

SET(SRC_FILES 
	../src/Plot.cpp
	main.cpp
	${moc_srcs}
)

SET(INCLUDE_DIRS
	${CMAKE_CURRENT_SOURCE_DIR}/../src
)

INCLUDE(${QT_USE_FILE})

SET(LIBRARIES
    ${QT_LIBRARIES} 
)

#include files
INCLUDE_DIRECTORIES(${INCLUDE_DIRS})

# create an executable file named "examplePlot" from the source files
ADD_EXECUTABLE(examplePlot ${SRC_FILES})
# Linking with Qt libraries
TARGET_LINK_LIBRARIES(examplePlot ${LIBRARIES})