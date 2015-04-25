
SET(srcs
    main.cpp)

SET(INCLUDE_DIRS
	${PROJECT_SOURCE_DIR}/utilite/include
	${PROJECT_SOURCE_DIR}/corelib/include
	${PROJECT_SOURCE_DIR}/guilib/include
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

IF("${RTABMAP_QT_VERSION}" STREQUAL "4")
    INCLUDE(${QT_USE_FILE})
ENDIF()

SET(LIBRARIES
	${OpenCV_LIBRARIES}
	${QT_LIBRARIES} 
	${PCL_LIBRARIES}
)

INCLUDE_DIRECTORIES(${INCLUDE_DIRS})

IF("${RTABMAP_QT_VERSION}" STREQUAL "4")
    QT4_WRAP_CPP(moc_srcs ../RGBDMapping/MapBuilder.h MapBuilderWifi.h)
ELSE()
    QT5_WRAP_CPP(moc_srcs ../RGBDMapping/MapBuilder.h MapBuilderWifi.h)
ENDIF()

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
TARGET_LINK_LIBRARIES(wifi_mapping rtabmap_core rtabmap_gui rtabmap_utilite ${LIBRARIES})

SET_TARGET_PROPERTIES( wifi_mapping 
  PROPERTIES OUTPUT_NAME ${PROJECT_PREFIX}-wifi_mapping)
