
SET(SRC_FILES
    main.cpp
    Tests.cpp
)

SET(INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
    ${CMAKE_CURRENT_SOURCE_DIR}/../src
    ${CPPUNIT_INCLUDE_DIR}
    ${UTILITE_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
	${SQLITE3_INCLUDE_DIR}
)

SET(LIBRARIES
    ${UTILITE_LIBRARY}
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
TARGET_LINK_LIBRARIES(testCoreLib corelib ${LIBRARIES})

SET_TARGET_PROPERTIES( testCoreLib
PROPERTIES
 OUTPUT_NAME testCoreLib)

