
SET(SRC_FILES
    main.cpp
)

SET(INCLUDE_DIRS
	${PROJECT_SOURCE_DIR}/corelib/include
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${UTILITE_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
	${CMAKE_CURRENT_SOURCE_DIR}/../include
)

SET(LIBRARIES
    ${UTILITE_LIBRARIES}
	${OpenCV_LIBRARIES} 
)

# Make sure the compiler can find include files from our library.
INCLUDE_DIRECTORIES(${INCLUDE_DIRS})

# Add binary called "consoleApp" that is built from the source file "main.cpp".
# The extension is automatically found.
ADD_EXECUTABLE(consoleApp ${SRC_FILES})
TARGET_LINK_LIBRARIES(consoleApp rtabmap_corelib ${LIBRARIES})

SET_TARGET_PROPERTIES( consoleApp 
  PROPERTIES OUTPUT_NAME ${PROJECT_PREFIX}-console)

INSTALL(TARGETS consoleApp
        RUNTIME DESTINATION bin COMPONENT runtime
        LIBRARY DESTINATION lib COMPONENT devel
        ARCHIVE DESTINATION lib COMPONENT devel)

