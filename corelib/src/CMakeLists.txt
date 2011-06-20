
SET(SRC_FILES
	Rtabmap.cpp
	RtabmapEvent.cpp
	
	Memory.cpp
	KeypointMemory.cpp
	
	DBDriverFactory.cpp
	DBDriver.cpp
	DBDriverSqlite3.cpp
	
    Camera.cpp
    EpipolarGeometry.cpp
	VisualWord.cpp
	VWDictionary.cpp
	BayesFilter.cpp
	Parameters.cpp
    Signature.cpp
	KeypointDetector.cpp
	KeypointDescriptor.cpp
	VerifyHypotheses.cpp
	NearestNeighbor.cpp
)

SET(INCLUDE_DIRS
	${CMAKE_CURRENT_SOURCE_DIR}/../include
	${CMAKE_CURRENT_SOURCE_DIR}
	${CMAKE_CURRENT_BINARY_DIR}
	${UTILITE_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
	${SQLITE3_INCLUDE_DIR}
)

SET(LIBRARIES
    ${UTILITE_LIBRARY}
	${OpenCV_LIBS} 
	${SQLITE3_LIBRARY}
)

# Generate resources files
ADD_CUSTOM_COMMAND(
   OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/DatabaseSchema_sql.h
   COMMAND ${URESOURCEGENERATOR_EXEC} -n rtabmap -p ${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/resources/DatabaseSchema.sql
   COMMENT "[Creating resources]"
)

# Make sure the compiler can find include files from our library.
INCLUDE_DIRECTORIES(${INCLUDE_DIRS})

IF(WIN32)
    IF(BUILD_SHARED_LIBS)   
	   ADD_DEFINITIONS(-DRTABMAP_EXPORTS) 
	ELSE()
	   ADD_DEFINITIONS(-DRTABMAP_EXPORTS_STATIC)
	ENDIF()
ENDIF(WIN32)

# Add binary that is built from the source file "main.cpp".
# The extension is automatically found.
ADD_LIBRARY(corelib ${SRC_FILES} ${CMAKE_CURRENT_BINARY_DIR}/DatabaseSchema_sql.h)
TARGET_LINK_LIBRARIES(corelib ${LIBRARIES})

SET_TARGET_PROPERTIES(
corelib 
PROPERTIES
 OUTPUT_NAME ${PROJECT_PREFIX}_core
 INSTALL_NAME_DIR ${CMAKE_INSTALL_PREFIX}/lib
)

INSTALL(TARGETS corelib
        RUNTIME DESTINATION bin COMPONENT runtime
        LIBRARY DESTINATION lib COMPONENT devel
        ARCHIVE DESTINATION lib COMPONENT devel)
		
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../include/ DESTINATION include/ COMPONENT devel FILES_MATCHING PATTERN "*.h" PATTERN ".svn" EXCLUDE)

