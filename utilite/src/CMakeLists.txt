
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
)

SET(INCLUDE_DIRS
	${CMAKE_CURRENT_SOURCE_DIR}/../include
	${PTHREADS_INCLUDE_DIR}
)

# Make sure the compiler can find include files from our library.
INCLUDE_DIRECTORIES(${INCLUDE_DIRS})

ADD_LIBRARY(rtabmap_utilite ${SRC_FILES})
IF(MINGW)
    TARGET_LINK_LIBRARIES(rtabmap_utilite ${PTHREADS_LIBRARY} "-lpsapi")
ELSEIF(WIN32)
	FIND_LIBRARY(PSAPI_LIBRARIES NAMES psapi libpsapi.dll.a libpsapi.a libpsapi.lib )
	TARGET_LINK_LIBRARIES(rtabmap_utilite ${PSAPI_LIBRARIES})
ELSE()
    TARGET_LINK_LIBRARIES(rtabmap_utilite ${PTHREADS_LIBRARY})
ENDIF()

INSTALL(TARGETS rtabmap_utilite
        RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT runtime
        LIBRARY DESTINATION "${INSTALL_LIB_DIR}" COMPONENT devel
        ARCHIVE DESTINATION "${INSTALL_LIB_DIR}" COMPONENT devel)
		
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../include/ DESTINATION "${INSTALL_INCLUDE_DIR}" COMPONENT devel FILES_MATCHING PATTERN "*.h" PATTERN ".svn" EXCLUDE)

