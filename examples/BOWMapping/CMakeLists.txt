cmake_minimum_required(VERSION 2.8)

IF(DEFINED PROJECT_NAME)
   set(internal TRUE)
ENDIF(DEFINED PROJECT_NAME)

if(internal)
	# inside rtabmap project (see below for external build)
	SET(RTABMap_INCLUDE_DIRS 
	    ${PROJECT_SOURCE_DIR}/utilite/include
		${PROJECT_SOURCE_DIR}/corelib/include
	)
	SET(RTABMap_LIBRARIES 
	    rtabmap_core
		rtabmap_utilite
	)  
else()
	# external build
	PROJECT( MyProject )
	
	FIND_PACKAGE(RTABMap REQUIRED)
	FIND_PACKAGE(OpenCV REQUIRED)
	FIND_PACKAGE(PCL 1.7 REQUIRED)
endif()

if(POLICY CMP0020)
	cmake_policy(SET CMP0020 OLD)
endif()

SET(INCLUDE_DIRS
	${RTABMap_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

SET(LIBRARIES
    ${RTABMap_LIBRARIES}
	${OpenCV_LIBRARIES} 
	${PCL_LIBRARIES}
)

INCLUDE_DIRECTORIES(${INCLUDE_DIRS})

ADD_EXECUTABLE(bow_mapping main.cpp)
TARGET_LINK_LIBRARIES(bow_mapping ${LIBRARIES})

if(internal)
  SET_TARGET_PROPERTIES( bow_mapping 
    PROPERTIES OUTPUT_NAME ${PROJECT_PREFIX}-bow_mapping)
endif(internal)