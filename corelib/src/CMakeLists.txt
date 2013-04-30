
SET(SRC_FILES
	Rtabmap.cpp
	RtabmapThread.cpp
	
	Statistics.cpp
	
	Memory.cpp
	
	DBDriver.cpp
	DBDriverSqlite3.cpp
	DBReader.cpp
	
    Camera.cpp
    CameraThread.cpp
    
    EpipolarGeometry.cpp
	VisualWord.cpp
	VWDictionary.cpp
	BayesFilter.cpp
	Parameters.cpp
    Signature.cpp
	Features2d.cpp
	NearestNeighbor.cpp
)

SET(INCLUDE_DIRS
    ${PROJECT_SOURCE_DIR}/utilite/include
	${CMAKE_CURRENT_SOURCE_DIR}/../include
	${CMAKE_CURRENT_SOURCE_DIR}
	${CMAKE_CURRENT_BINARY_DIR}
    ${OpenCV_INCLUDE_DIRS}
	${SQLITE3_INCLUDE_DIR}
)

SET(LIBRARIES
	${OpenCV_LIBS} 
	${SQLITE3_LIBRARY}
)

####################################
# Generate resources files
####################################
SET(R
	${CMAKE_CURRENT_SOURCE_DIR}/resources/DatabaseSchema.sql
)

#replace semicolons by spaces
foreach(arg ${R})
   set(RESOURCES "${RESOURCES}" "${arg}")
endforeach(arg ${R})

SET(RESOURCES_HEADERS 
	${CMAKE_CURRENT_BINARY_DIR}/DatabaseSchema_sql.h 
)

ADD_CUSTOM_COMMAND(
   OUTPUT ${RESOURCES_HEADERS}
   COMMAND ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/uresourcegenerator -n rtabmap -p ${CMAKE_CURRENT_BINARY_DIR} ${RESOURCES}
   COMMENT "[Creating resources]"
   DEPENDS ${R} uresourcegenerator
)

####################################
# Generate resources files END
####################################


# Make sure the compiler can find include files from our library.
INCLUDE_DIRECTORIES(${INCLUDE_DIRS})

# Add binary that is built from the source file "main.cpp".
# The extension is automatically found.
ADD_LIBRARY(rtabmap_corelib ${SRC_FILES} ${RESOURCES_HEADERS})
TARGET_LINK_LIBRARIES(rtabmap_corelib rtabmap_utilite ${LIBRARIES})

SET_TARGET_PROPERTIES(
rtabmap_corelib 
PROPERTIES
 OUTPUT_NAME ${PROJECT_PREFIX}_core
 INSTALL_NAME_DIR ${CMAKE_INSTALL_PREFIX}/lib
)

INSTALL(TARGETS rtabmap_corelib
        RUNTIME DESTINATION bin COMPONENT runtime
        LIBRARY DESTINATION lib COMPONENT devel
        ARCHIVE DESTINATION lib COMPONENT devel)
		
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../include/ DESTINATION include/ COMPONENT devel FILES_MATCHING PATTERN "*.h" PATTERN ".svn" EXCLUDE)

