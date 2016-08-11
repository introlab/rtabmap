cmake_minimum_required(VERSION 2.8)

IF(DEFINED PROJECT_NAME)
   set(internal TRUE)
ENDIF(DEFINED PROJECT_NAME)

if(internal)
	# inside rtabmap project (see below for external build)
	SET(RTABMap_INCLUDE_DIRS 
	    ${PROJECT_SOURCE_DIR}/utilite/include
		${PROJECT_SOURCE_DIR}/corelib/include
		${PROJECT_SOURCE_DIR}/guilib/include
	)
	SET(RTABMap_LIBRARIES 
	    rtabmap_core
		rtabmap_gui
		rtabmap_utilite
	)  
else()
	# external build
	PROJECT( MyProject )
	
	FIND_PACKAGE(RTABMap REQUIRED)
	FIND_PACKAGE(OpenCV REQUIRED)
	FIND_PACKAGE(PCL 1.7 REQUIRED)
	
	# Find Qt5 first
	FIND_PACKAGE(Qt5 COMPONENTS Widgets Core Gui Svg QUIET)
    	IF(NOT Qt5_FOUND)
       		FIND_PACKAGE(Qt4 COMPONENTS QtCore QtGui QtSvg)
    	ENDIF(NOT Qt5_FOUND)
endif()

if(POLICY CMP0020)
	cmake_policy(SET CMP0020 OLD)
endif()

SET(INCLUDE_DIRS
	${RTABMap_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

IF(QT4_FOUND)
    INCLUDE(${QT_USE_FILE})
ENDIF(QT4_FOUND)

SET(LIBRARIES
	${RTABMap_LIBRARIES}
	${OpenCV_LIBRARIES}
	${QT_LIBRARIES} 
	${PCL_LIBRARIES}
)

INCLUDE_DIRECTORIES(${INCLUDE_DIRS})

IF(QT4_FOUND)
    QT4_WRAP_CPP(moc_srcs MapBuilder.h MapBuilderWifi.h)
ELSE()
    QT5_WRAP_CPP(moc_srcs MapBuilder.h MapBuilderWifi.h)
ENDIF()

SET(srcs
    main.cpp)

IF(APPLE)
    FIND_LIBRARY(CoreWLAN_LIBRARY CoreWLAN)
    FIND_LIBRARY(Foundation_LIBRARY Foundation)
    MARK_AS_ADVANCED(CoreWLAN_LIBRARY Foundation_LIBRARY)
    SET(LIBRARIES
	    ${LIBRARIES}
	    ${CoreWLAN_LIBRARY}
	    ${Foundation_LIBRARY}
	)
	SET(srcs
	    ${srcs}
	    WifiOSX.mm
	)
ENDIF(APPLE)

ADD_EXECUTABLE(wifi_mapping ${srcs} ${moc_srcs})
TARGET_LINK_LIBRARIES(wifi_mapping ${LIBRARIES})

if(internal)
  SET_TARGET_PROPERTIES( wifi_mapping 
    PROPERTIES OUTPUT_NAME ${PROJECT_PREFIX}-wifi_mapping)
endif(internal)
