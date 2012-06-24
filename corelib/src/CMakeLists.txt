
SET(SRC_FILES
	Rtabmap.cpp
	RtabmapEvent.cpp
	
	Memory.cpp
	KeypointMemory.cpp
	SMMemory.cpp
	
	DBDriverFactory.cpp
	DBDriver.cpp
	DBDriverSqlite3.cpp
	DBReader.cpp
	
    Camera.cpp
    Micro.cpp
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
	ColorTable.cpp

)

SET(INCLUDE_DIRS
	${CMAKE_CURRENT_SOURCE_DIR}/../include
	${CMAKE_CURRENT_SOURCE_DIR}
	${CMAKE_CURRENT_BINARY_DIR}
	${UTILITE_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
	${SQLITE3_INCLUDE_DIR}
	${ZLIB_INCLUDE_DIRS}
	${FFTW3F_INCLUDE_DIRS}
)

SET(LIBRARIES
    ${UTILITE_LIBRARIES}
	${OpenCV_LIBS} 
	${SQLITE3_LIBRARY}
	${ZLIB_LIBRARIES}
	${FFTW3F_LIBRARIES}
)

####################################
# Generate resources files
####################################
ADD_CUSTOM_COMMAND(
   OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/DatabaseSchema_sql.h
   COMMAND ${URESOURCEGENERATOR_EXEC} -n rtabmap -p ${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/resources/DatabaseSchema.sql
   COMMENT "[Creating database resource]"
   DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/resources/DatabaseSchema.sql
)

ADD_CUSTOM_COMMAND(
   OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/ColorIndexes65536_bin_zip.h
   COMMAND ${URESOURCEGENERATOR_EXEC} -n rtabmap -p ${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/resources/ColorIndexes65536.bin.zip
   COMMENT "[Creating color table resource]"
   DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/resources/ColorIndexes65536.bin.zip
)
ADD_CUSTOM_COMMAND(
   OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/ColorIndexes1024_bin_zip.h
   COMMAND ${URESOURCEGENERATOR_EXEC} -n rtabmap -p ${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/resources/ColorIndexes1024.bin.zip
   COMMENT "[Creating color table resource]"
   DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/resources/ColorIndexes1024.bin.zip
)
ADD_CUSTOM_COMMAND(
   OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/ColorIndexes512_bin_zip.h
   COMMAND ${URESOURCEGENERATOR_EXEC} -n rtabmap -p ${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/resources/ColorIndexes512.bin.zip
   COMMENT "[Creating color table resource]"
   DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/resources/ColorIndexes512.bin.zip
)
ADD_CUSTOM_COMMAND(
   OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/ColorIndexes256_bin_zip.h
   COMMAND ${URESOURCEGENERATOR_EXEC} -n rtabmap -p ${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/resources/ColorIndexes256.bin.zip
   COMMENT "[Creating color table resource]"
   DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/resources/ColorIndexes256.bin.zip
)
ADD_CUSTOM_COMMAND(
   OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/ColorIndexes128_bin_zip.h
   COMMAND ${URESOURCEGENERATOR_EXEC} -n rtabmap -p ${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/resources/ColorIndexes128.bin.zip
   COMMENT "[Creating color table resource]"
   DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/resources/ColorIndexes128.bin.zip
)
ADD_CUSTOM_COMMAND(
   OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/ColorIndexes64_bin_zip.h
   COMMAND ${URESOURCEGENERATOR_EXEC} -n rtabmap -p ${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/resources/ColorIndexes64.bin.zip
   COMMENT "[Creating color table resource]"
   DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/resources/ColorIndexes64.bin.zip
)
ADD_CUSTOM_COMMAND(
   OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/ColorIndexes32_bin_zip.h
   COMMAND ${URESOURCEGENERATOR_EXEC} -n rtabmap -p ${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/resources/ColorIndexes32.bin.zip
   COMMENT "[Creating color table resource]"
   DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/resources/ColorIndexes32.bin.zip
)
ADD_CUSTOM_COMMAND(
   OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/ColorIndexes16_bin_zip.h
   COMMAND ${URESOURCEGENERATOR_EXEC} -n rtabmap -p ${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/resources/ColorIndexes16.bin.zip
   COMMENT "[Creating color table resource]"
   DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/resources/ColorIndexes16.bin.zip
)
ADD_CUSTOM_COMMAND(
   OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/ColorIndexes8_bin_zip.h
   COMMAND ${URESOURCEGENERATOR_EXEC} -n rtabmap -p ${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/resources/ColorIndexes8.bin.zip
   COMMENT "[Creating color table resource]"
   DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/resources/ColorIndexes8.bin.zip
)
SET(RESOURCES 
	${CMAKE_CURRENT_BINARY_DIR}/DatabaseSchema_sql.h 
	${CMAKE_CURRENT_BINARY_DIR}/ColorIndexes65536_bin_zip.h
	${CMAKE_CURRENT_BINARY_DIR}/ColorIndexes1024_bin_zip.h
	${CMAKE_CURRENT_BINARY_DIR}/ColorIndexes512_bin_zip.h
	${CMAKE_CURRENT_BINARY_DIR}/ColorIndexes256_bin_zip.h
	${CMAKE_CURRENT_BINARY_DIR}/ColorIndexes128_bin_zip.h
	${CMAKE_CURRENT_BINARY_DIR}/ColorIndexes64_bin_zip.h
	${CMAKE_CURRENT_BINARY_DIR}/ColorIndexes32_bin_zip.h
	${CMAKE_CURRENT_BINARY_DIR}/ColorIndexes16_bin_zip.h
	${CMAKE_CURRENT_BINARY_DIR}/ColorIndexes8_bin_zip.h
)

####################################
# Generate resources files END
####################################


# Make sure the compiler can find include files from our library.
INCLUDE_DIRECTORIES(${INCLUDE_DIRS})

# Add binary that is built from the source file "main.cpp".
# The extension is automatically found.
ADD_LIBRARY(rtabmap_corelib ${SRC_FILES} ${RESOURCES})
TARGET_LINK_LIBRARIES(rtabmap_corelib ${LIBRARIES})

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

