cmake_minimum_required(VERSION 2.8)

IF(DEFINED PROJECT_NAME)
   set(internal TRUE)
ENDIF(DEFINED PROJECT_NAME)

if(NOT internal)
	# external build
	PROJECT( MyProject )
	
	FIND_PACKAGE(RTABMap REQUIRED COMPONENTS gui)

endif()

IF(QT4_FOUND OR Qt5_FOUND OR Qt6_FOUND)
    SET(moc_srcs MapBuilder.h)
ENDIF()

ADD_EXECUTABLE(lidar_mapping main.cpp ${moc_srcs})
  
TARGET_LINK_LIBRARIES(lidar_mapping rtabmap::gui)

SET_TARGET_PROPERTIES(
     lidar_mapping
   PROPERTIES
     AUTOUIC ON
     AUTOMOC ON
     AUTORCC ON
)

if(internal)
  SET_TARGET_PROPERTIES( lidar_mapping 
    PROPERTIES OUTPUT_NAME ${PROJECT_PREFIX}-lidar_mapping)
endif(internal)
