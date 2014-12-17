
SET(SRC_FILES
    main.cpp
)

SET(INCLUDE_DIRS
	${PCL_INCLUDE_DIRS}
	${PROJECT_SOURCE_DIR}/utilite/include
)

SET(LIBRARIES
	${PCL_LIBRARIES}
)

add_definitions(${PCL_DEFINITIONS})

# Make sure the compiler can find include files from our library.
INCLUDE_DIRECTORIES(${INCLUDE_DIRS})

# Add binary called "consoleApp" that is built from the source file "main.cpp".
# The extension is automatically found.
ADD_EXECUTABLE(extractObject ${SRC_FILES})
TARGET_LINK_LIBRARIES(extractObject rtabmap_utilite ${LIBRARIES})

SET_TARGET_PROPERTIES( extractObject 
  PROPERTIES OUTPUT_NAME ${PROJECT_PREFIX}-extractObject)

INSTALL(TARGETS extractObject
		RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT runtime
		BUNDLE DESTINATION "${CMAKE_BUNDLE_LOCATION}" COMPONENT runtime)