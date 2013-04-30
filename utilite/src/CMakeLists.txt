
SET(SRC_FILES
	UEventsManager.cpp
    UEventsHandler.cpp
    UFile.cpp
    UDirectory.cpp
    UConversion.cpp
    ULogger.cpp
    UThread.cpp
    UTimer.cpp
    UProcessInfo.cpp
)

SET(INCLUDE_DIRS
	${CMAKE_CURRENT_SOURCE_DIR}/../include
	${PTHREADS_INCLUDE_DIR}
)

# Make sure the compiler can find include files from our library.
INCLUDE_DIRECTORIES(${INCLUDE_DIRS})

ADD_LIBRARY(rtabmap_utilite ${SRC_FILES})
IF(WIN32)
    TARGET_LINK_LIBRARIES(rtabmap_utilite ${PTHREADS_LIBRARY} ${LIBRARIES} "-lpsapi")
ELSE(WIN32)
    TARGET_LINK_LIBRARIES(rtabmap_utilite ${PTHREADS_LIBRARY} ${LIBRARIES})
ENDIF(WIN32)

SET_TARGET_PROPERTIES(
rtabmap_utilite 
PROPERTIES
 OUTPUT_NAME ${PROJECT_PREFIX}_utilite
 INSTALL_NAME_DIR ${CMAKE_INSTALL_PREFIX}/lib
)

INSTALL(TARGETS rtabmap_utilite
        RUNTIME DESTINATION bin COMPONENT runtime
        LIBRARY DESTINATION lib COMPONENT devel
        ARCHIVE DESTINATION lib COMPONENT devel)
		
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../include/ DESTINATION include/ COMPONENT devel FILES_MATCHING PATTERN "*.h" PATTERN ".svn" EXCLUDE)

