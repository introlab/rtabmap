
SET(INCLUDE_DIRS
	${PROJECT_SOURCE_DIR}/utilite/include
	${PROJECT_SOURCE_DIR}/corelib/include
    ${OpenCV_INCLUDE_DIRS}
)

SET(LIBRARIES
	${OpenCV_LIBRARIES}
)

INCLUDE_DIRECTORIES(${INCLUDE_DIRS})

ADD_EXECUTABLE(vocabularyComparison main.cpp)
TARGET_LINK_LIBRARIES(vocabularyComparison rtabmap_core rtabmap_utilite ${LIBRARIES})

SET_TARGET_PROPERTIES( vocabularyComparison 
  PROPERTIES OUTPUT_NAME ${PROJECT_PREFIX}-vocabularyComparison)

