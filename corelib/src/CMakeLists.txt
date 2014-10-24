
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
    CameraRGBD.cpp
    
    EpipolarGeometry.cpp
	VisualWord.cpp
	VWDictionary.cpp
	BayesFilter.cpp
	Parameters.cpp
    Signature.cpp
	Features2d.cpp
	Transform.cpp
	
	util3d.cpp
	Odometry.cpp
	SensorData.cpp
	
	toro3d/posegraph3.cpp
	toro3d/treeoptimizer3_iteration.cpp
	toro3d/treeoptimizer3.cpp
	
	sqlite3/sqlite3.c
)

SET(INCLUDE_DIRS
    ${PROJECT_SOURCE_DIR}/utilite/include
	${CMAKE_CURRENT_SOURCE_DIR}/../include
	${CMAKE_CURRENT_SOURCE_DIR}
	${CMAKE_CURRENT_BINARY_DIR}
    ${OpenCV_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
	${ZLIB_INCLUDE_DIRS}
)

SET(LIBRARIES
	${OpenCV_LIBS} 
	${PCL_LIBRARIES} 
	${ZLIB_LIBRARIES} 
)

IF(Freenect_FOUND)
	ADD_DEFINITIONS("-DWITH_FREENECT")
	IF(Freenect_DASH_INCLUDES)
	   ADD_DEFINITIONS("-DFREENECT_DASH_INCLUDES")
	ENDIF(Freenect_DASH_INCLUDES)
	SET(INCLUDE_DIRS
		${INCLUDE_DIRS}
		${Freenect_INCLUDE_DIRS}
	)
	SET(LIBRARIES
		${LIBRARIES}
		${Freenect_LIBRARIES}
	)
ENDIF(Freenect_FOUND)

IF(OpenNI2_FOUND)
	ADD_DEFINITIONS("-DWITH_OPENNI2")
	SET(INCLUDE_DIRS
		${INCLUDE_DIRS}
		${OpenNI2_INCLUDE_DIRS}
	)
	SET(LIBRARIES
		${LIBRARIES}
		${OpenNI2_LIBRARIES}
	)
ENDIF(OpenNI2_FOUND)

####################################
# Generate resources files
####################################
CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/resources/DatabaseSchema.sql.in ${CMAKE_CURRENT_SOURCE_DIR}/resources/DatabaseSchema.sql)

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

add_definitions(${PCL_DEFINITIONS})


# Make sure the compiler can find include files from our library.
INCLUDE_DIRECTORIES(${INCLUDE_DIRS})

# Add binary that is built from the source file "main.cpp".
# The extension is automatically found.
ADD_LIBRARY(rtabmap_core ${SRC_FILES} ${RESOURCES_HEADERS})
TARGET_LINK_LIBRARIES(rtabmap_core rtabmap_utilite ${LIBRARIES})

INSTALL(TARGETS rtabmap_core
        RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT runtime
        LIBRARY DESTINATION "${INSTALL_LIB_DIR}" COMPONENT devel
        ARCHIVE DESTINATION "${INSTALL_LIB_DIR}" COMPONENT devel)
		
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../include/ 
		DESTINATION "${INSTALL_INCLUDE_DIR}"
		COMPONENT devel 
		FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp" 
		PATTERN ".svn" EXCLUDE)

