
IF(NOT WITH_QT)
   # visualization module required
   FIND_PACKAGE(PCL 1.7 REQUIRED QUIET COMPONENTS common io kdtree search surface filters registration sample_consensus segmentation visualization)
ENDIF(NOT WITH_QT)

SET(INCLUDE_DIRS
	${PROJECT_SOURCE_DIR}/corelib/include
	${PROJECT_SOURCE_DIR}/utilite/include
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

SET(LIBRARIES
	${OpenCV_LIBRARIES} 
	${PCL_LIBRARIES}
)

add_definitions(${PCL_DEFINITIONS})

INCLUDE_DIRECTORIES(${INCLUDE_DIRS})

ADD_EXECUTABLE(rgbd_camera main.cpp)
TARGET_LINK_LIBRARIES(rgbd_camera rtabmap_core rtabmap_utilite ${LIBRARIES})

SET_TARGET_PROPERTIES( rgbd_camera 
  PROPERTIES OUTPUT_NAME ${PROJECT_PREFIX}-rgbd_camera)
  
INSTALL(TARGETS rgbd_camera
		RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT runtime
		BUNDLE DESTINATION "${CMAKE_BUNDLE_LOCATION}" COMPONENT runtime)