cmake_minimum_required(VERSION 2.8)

IF(DEFINED PROJECT_NAME)
   set(internal TRUE)
ENDIF(DEFINED PROJECT_NAME)

if(NOT internal)
	# external build
	PROJECT( MyProject )
	
	FIND_PACKAGE(RTABMap REQUIRED COMPONENTS gui)
endif()

IF(QT4_FOUND)
    QT4_WRAP_CPP(moc_srcs MapBuilder.h)
ELSE()
    QT5_WRAP_CPP(moc_srcs MapBuilder.h)
ENDIF()

ADD_EXECUTABLE(rgbd_mapping main.cpp ${moc_srcs})
  
TARGET_LINK_LIBRARIES(rgbd_mapping rtabmap_gui)

if(internal)
  SET_TARGET_PROPERTIES( rgbd_mapping 
    PROPERTIES OUTPUT_NAME ${PROJECT_PREFIX}-rgbd_mapping)
endif(internal)
