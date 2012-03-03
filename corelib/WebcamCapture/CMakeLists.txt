
SET(SRC_FILES
    main.cpp
)

SET(INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${UTILITE_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
)

SET(LIBRARIES
    ${UTILITE_LIBRARIES}
	${OpenCV_LIBS} 
)

# Make sure the compiler can find include files from our library.
INCLUDE_DIRECTORIES(${INCLUDE_DIRS})

# Add binary called "imagesJoiner" that is built from the source file "main.cpp".
# The extension is automatically found.
ADD_EXECUTABLE(webcamCapture ${SRC_FILES})
TARGET_LINK_LIBRARIES(webcamCapture corelib ${LIBRARIES})

SET_TARGET_PROPERTIES( webcamCapture 
  PROPERTIES OUTPUT_NAME ${PROJECT_PREFIX}-webcamCapture)

INSTALL(TARGETS webcamCapture
        RUNTIME DESTINATION bin COMPONENT runtime
        LIBRARY DESTINATION lib COMPONENT devel
        ARCHIVE DESTINATION lib COMPONENT devel)

