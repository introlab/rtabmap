
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
    QT4_WRAP_CPP(moc_srcs MapBuilder.h)
ELSE()
    QT5_WRAP_CPP(moc_srcs MapBuilder.h)
ENDIF()

ADD_EXECUTABLE(rgbd_mapping main.cpp ${moc_srcs})
  
TARGET_LINK_LIBRARIES(rgbd_mapping rtabmap_core rtabmap_gui rtabmap_utilite ${LIBRARIES})

SET_TARGET_PROPERTIES( rgbd_mapping 
  PROPERTIES OUTPUT_NAME ${PROJECT_PREFIX}-rgbd_mapping)
