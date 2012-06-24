
SET(SRC_FILES
    main.cpp
    Tests.cpp
)

SET(INCLUDE_DIRS
    ${PROJECT_SOURCE_DIR}/corelib/include
    ${CPPUNIT_INCLUDE_DIR}
    ${UTILITE_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
	${SQLITE3_INCLUDE_DIR}
)

SET(LIBRARIES
    ${UTILITE_LIBRARIES}
	${OpenCV_LIBRARIES} 
	${CPPUNIT_LIBRARY}
	${SQLITE3_LIBRARY}
)

# Make sure the compiler can find include files from our library.
INCLUDE_DIRECTORIES(${INCLUDE_DIRS})

ADD_DEFINITIONS(${CPPUNIT_DEFINITIONS})

# Add binary called "testAvpdCore" that is built from the source file "main.cpp".
# The extension is automatically found.
ADD_EXECUTABLE(testCoreLib ${SRC_FILES})
TARGET_LINK_LIBRARIES(testCoreLib rtabmap_corelib ${LIBRARIES})

SET_TARGET_PROPERTIES( testCoreLib
PROPERTIES
 OUTPUT_NAME testCoreLib)

