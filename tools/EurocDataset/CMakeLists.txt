cmake_minimum_required(VERSION 2.8)

FIND_PACKAGE(yaml-cpp QUIET)
IF(yaml-cpp_FOUND)

# inside rtabmap project (see below for external build)
SET(RTABMap_INCLUDE_DIRS 
    ${PROJECT_SOURCE_DIR}/utilite/include
	${PROJECT_SOURCE_DIR}/corelib/include
)
SET(RTABMap_LIBRARIES 
    rtabmap_core
	rtabmap_utilite
)  
	
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
	${YAML_CPP_LIBRARIES}
)

INCLUDE_DIRECTORIES(${INCLUDE_DIRS})

ADD_EXECUTABLE(euroc_dataset main.cpp)
  
TARGET_LINK_LIBRARIES(euroc_dataset ${LIBRARIES})

SET_TARGET_PROPERTIES( euroc_dataset 
    PROPERTIES OUTPUT_NAME ${PROJECT_PREFIX}-euroc_dataset)
    
INSTALL(TARGETS euroc_dataset
	RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT runtime
	BUNDLE DESTINATION "${CMAKE_BUNDLE_LOCATION}" COMPONENT runtime)
	
ELSE()
  MESSAGE(STATUS "yaml-cpp not found, euroc_dataset tool won't be built...")
ENDIF()
