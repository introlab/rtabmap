
if (CMAKE_CROSSCOMPILING OR ANDROID OR IOS)
    # See this page about tools being required in the build:
    # https://gitlab.kitware.com/cmake/community/-/wikis/doc/cmake/CrossCompiling#using-executables-in-the-build-created-during-the-build

    # Some ideas there were used, not all.
    # The target named 'res_tool' can be used elsewhere in all cases, when cross compiling or not.

    IF(NOT RTABMAP_RES_TOOL)
      IF (COMMAND find_host_program)
        # On android docker build
        MESSAGE(STATUS "Looking for ${PROJECT_PREFIX}-res tool in ${PROJECT_BINARY_DIR}/../bin")
        FIND_HOST_PROGRAM( RTABMAP_RES_TOOL ${PROJECT_PREFIX}-res_tool PATHS ${PROJECT_BINARY_DIR}/../bin NO_DEFAULT_PATH)
        IF(NOT RTABMAP_RES_TOOL)
          MESSAGE(STATUS "Looking for ${PROJECT_PREFIX}-res tool on host system")
          FIND_HOST_PROGRAM( RTABMAP_RES_TOOL ${PROJECT_PREFIX}-res_tool)
        ENDIF(NOT RTABMAP_RES_TOOL)
      ELSE()
        MESSAGE(STATUS "Looking for ${PROJECT_PREFIX}-res tool on host system")
        FIND_PROGRAM( RTABMAP_RES_TOOL  ${PROJECT_PREFIX}-res_tool )
      ENDIF()
      IF(NOT RTABMAP_RES_TOOL)
        MESSAGE( FATAL_ERROR "RTABMAP_RES_TOOL is not defined (it is the path to \"${PROJECT_PREFIX}-res_tool\" application created by a non-Android build)." )
      ENDIF(NOT RTABMAP_RES_TOOL)
    ENDIF(NOT RTABMAP_RES_TOOL)

    MESSAGE(STATUS "Using res_tool at ${RTABMAP_RES_TOOL}")

    ADD_EXECUTABLE(res_tool IMPORTED GLOBAL)
    SET_TARGET_PROPERTIES(res_tool PROPERTIES IMPORTED_LOCATION "${RTABMAP_RES_TOOL}")

else()

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

endif()
