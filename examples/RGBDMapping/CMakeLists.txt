
SET(INCLUDE_DIRS
	${PROJECT_SOURCE_DIR}/utilite/include
	${PROJECT_SOURCE_DIR}/corelib/include
	${PROJECT_SOURCE_DIR}/guilib/include
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

INCLUDE(${QT_USE_FILE})

SET(LIBRARIES
	${OpenCV_LIBRARIES}
	${QT_LIBRARIES} 
	${PCL_LIBRARIES}
)

INCLUDE_DIRECTORIES(${INCLUDE_DIRS})

QT4_WRAP_CPP(moc_srcs MapBuilder.h)

ADD_EXECUTABLE(rgbd_mapping main.cpp ${moc_srcs})
  
TARGET_LINK_LIBRARIES(rgbd_mapping rtabmap_core rtabmap_gui rtabmap_utilite ${LIBRARIES})

SET_TARGET_PROPERTIES( rgbd_mapping 
  PROPERTIES OUTPUT_NAME ${PROJECT_PREFIX}-rgbd_mapping)
