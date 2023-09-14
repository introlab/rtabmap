cmake_minimum_required(VERSION 3.5)

if(POLICY CMP0020)
    cmake_policy(SET CMP0020 NEW)
endif()

IF(DEFINED PROJECT_NAME)
   set(internal TRUE)
ENDIF(DEFINED PROJECT_NAME)

if(NOT internal)
	# external build
	PROJECT( MyProject )
	
	FIND_PACKAGE(RTABMap REQUIRED)
endif()

IF(QT4_FOUND OR Qt5_FOUND OR Qt6_FOUND)
	SET(moc_srcs MapBuilder.h MapBuilderWifi.h)
ENDIF()

SET(srcs
    main.cpp)

set(LIBRARIES "")

IF(APPLE)
    FIND_LIBRARY(CoreWLAN_LIBRARY CoreWLAN)
    FIND_LIBRARY(Foundation_LIBRARY Foundation)
    MARK_AS_ADVANCED(CoreWLAN_LIBRARY Foundation_LIBRARY)
    SET(LIBRARIES
	    ${LIBRARIES}
	    ${CoreWLAN_LIBRARY}
	    ${Foundation_LIBRARY}
	)
	SET(srcs
	    ${srcs}
	    WifiOSX.mm
	)
ENDIF(APPLE)

ADD_EXECUTABLE(wifi_mapping ${srcs} ${moc_srcs})
TARGET_LINK_LIBRARIES(wifi_mapping rtabmap::gui ${LIBRARIES})

if(internal)
  SET_TARGET_PROPERTIES( wifi_mapping 
    PROPERTIES OUTPUT_NAME ${PROJECT_PREFIX}-wifi_mapping)
endif(internal)
