
SET(SRC_FILES
    main.cpp
)

SET(INCLUDE_DIRS
    ../include
)

# Make sure the compiler can find include files from our library.
INCLUDE_DIRECTORIES(${INCLUDE_DIRS})

# Add binary called "resource_tool" that is built from the source file "main.cpp".
# The extension is automatically found.
ADD_EXECUTABLE(res_tool ${SRC_FILES})
TARGET_LINK_LIBRARIES(res_tool rtabmap_utilite)
 
SET_TARGET_PROPERTIES(
res_tool 
PROPERTIES
 VERSION ${UTILITE_VERSION} 
 SOVERSION ${UTILITE_VERSION}
 OUTPUT_NAME ${PROJECT_PREFIX}-res_tool
)

INSTALL(TARGETS res_tool
		RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT runtime
		BUNDLE DESTINATION "${CMAKE_BUNDLE_LOCATION}" COMPONENT runtime)