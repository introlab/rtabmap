cmake_minimum_required(VERSION 2.8)

IF(DEFINED PROJECT_NAME)
   set(internal TRUE)
ENDIF(DEFINED PROJECT_NAME)

if(NOT internal)
	# external build
	PROJECT( MyProject )
	
	FIND_PACKAGE(RTABMap REQUIRED)
endif()

ADD_EXECUTABLE(bow_mapping main.cpp)
TARGET_LINK_LIBRARIES(bow_mapping rtabmap)

if(internal)
  SET_TARGET_PROPERTIES( bow_mapping 
    PROPERTIES OUTPUT_NAME ${PROJECT_PREFIX}-bow_mapping)
endif(internal)