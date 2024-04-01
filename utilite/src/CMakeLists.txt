
SET(SRC_FILES
	UEventsManager.cpp
    UEventsHandler.cpp
    UEventsSender.cpp
    UFile.cpp
    UDirectory.cpp
    UConversion.cpp
    ULogger.cpp
    UThread.cpp
    UTimer.cpp
    UProcessInfo.cpp
    UVariant.cpp
)

ADD_LIBRARY(rtabmap_utilite ${SRC_FILES})
ADD_LIBRARY(rtabmap::utilite ALIAS rtabmap_utilite)

generate_export_header(rtabmap_utilite 
  BASE_NAME utilite)

target_include_directories(rtabmap_utilite PUBLIC 
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include;${CMAKE_CURRENT_BINARY_DIR}/include;${PTHREADS_INCLUDE_DIR}>"
  "$<INSTALL_INTERFACE:${INSTALL_INCLUDE_DIR};${PTHREADS_INCLUDE_DIR}>")

IF(MINGW)
    TARGET_LINK_LIBRARIES(rtabmap_utilite PRIVATE ${PTHREADS_LIBRARY} "-lpsapi")
ELSEIF(WIN32 OR MSVC)
	FIND_LIBRARY(PSAPI_LIBRARIES NAMES psapi libpsapi.dll.a libpsapi.a libpsapi.lib )
	TARGET_LINK_LIBRARIES(rtabmap_utilite PRIVATE ${PSAPI_LIBRARIES})
ELSE()
    TARGET_LINK_LIBRARIES(rtabmap_utilite PRIVATE ${PTHREADS_LIBRARY})
ENDIF()

SET_TARGET_PROPERTIES(
     rtabmap_utilite 
   PROPERTIES
     VERSION ${RTABMAP_VERSION}
     SOVERSION ${RTABMAP_MAJOR_VERSION}.${RTABMAP_MINOR_VERSION}
     EXPORT_NAME "utilite"
)
INSTALL(TARGETS rtabmap_utilite EXPORT rtabmap_utiliteTargets
        RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT runtime
        LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT devel
        ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT devel)

configure_file(
  ${CMAKE_CURRENT_BINARY_DIR}/utilite_export.h
  ${CMAKE_CURRENT_BINARY_DIR}/include/${PROJECT_PREFIX}/utilite/utilite_export.h
  COPYONLY)

install(
  DIRECTORY 
    ${CMAKE_CURRENT_SOURCE_DIR}/../include/${PROJECT_PREFIX}
    ${CMAKE_CURRENT_BINARY_DIR}/include/${PROJECT_PREFIX}
  DESTINATION 
    "${INSTALL_INCLUDE_DIR}" 
  COMPONENT 
    devel 
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp")

export(EXPORT rtabmap_utiliteTargets
  FILE "${CMAKE_CURRENT_BINARY_DIR}/../../${PROJECT_NAME}_utiliteTargets.cmake"
  NAMESPACE rtabmap::
)
install(EXPORT rtabmap_utiliteTargets
  FILE
    ${PROJECT_NAME}_utiliteTargets.cmake
  DESTINATION
    ${INSTALL_CMAKE_DIR}
  NAMESPACE rtabmap::
  COMPONENT
    devel
)
