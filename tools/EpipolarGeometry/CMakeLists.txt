
SET(SRC_FILES
    main.cpp
)

SET(INCLUDE_DIRS
	${PROJECT_SOURCE_DIR}/utilite/include
	${PROJECT_SOURCE_DIR}/corelib/include
	${PROJECT_SOURCE_DIR}/guilib/include
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

IF(QT4_FOUND)
    INCLUDE(${QT_USE_FILE})
ENDIF(QT4_FOUND)

SET(LIBRARIES
	${OpenCV_LIBRARIES}
	${PCL_LIBRARIES} 
	${QT_LIBRARIES} 
)

INCLUDE_DIRECTORIES(${INCLUDE_DIRS})

IF(MINGW)
  ADD_EXECUTABLE(epipolar_geometry WIN32 ${SRC_FILES})
ELSE()
  ADD_EXECUTABLE(epipolar_geometry ${SRC_FILES})
ENDIF()
TARGET_LINK_LIBRARIES(epipolar_geometry rtabmap_core rtabmap_gui rtabmap_utilite ${LIBRARIES})

SET_TARGET_PROPERTIES( epipolar_geometry 
  PROPERTIES OUTPUT_NAME ${PROJECT_PREFIX}-epipolar_geometry)

