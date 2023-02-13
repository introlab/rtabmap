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

ADD_EXECUTABLE(noEventsExample main.cpp ${moc_srcs})
  
TARGET_LINK_LIBRARIES(noEventsExample rtabmap_gui)

if(internal)
	SET_TARGET_PROPERTIES( noEventsExample 
	  PROPERTIES OUTPUT_NAME ${PROJECT_PREFIX}-noEventsExample)
endif(internal)




