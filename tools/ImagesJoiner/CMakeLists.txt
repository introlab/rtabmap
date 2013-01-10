
SET(SRC_FILES
    main.cpp
)

SET(INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${UTILITE_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

SET(LIBRARIES
    ${UTILITE_LIBRARIES}
	${OpenCV_LIBS} 
)

# Make sure the compiler can find include files from our library.
INCLUDE_DIRECTORIES(${INCLUDE_DIRS})

# Add binary called "imagesJoiner" that is built from the source file "main.cpp".
# The extension is automatically found.
ADD_EXECUTABLE(imagesJoiner ${SRC_FILES})
TARGET_LINK_LIBRARIES(imagesJoiner ${LIBRARIES})

SET_TARGET_PROPERTIES( imagesJoiner 
  PROPERTIES OUTPUT_NAME ${PROJECT_PREFIX}-imagesJoiner)

