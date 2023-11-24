
FIND_PACKAGE(yaml-cpp QUIET)

IF(yaml-cpp_FOUND)
  IF (TARGET yaml-cpp::yaml-cpp)
    # yaml-cpp 0.8.0 uses target yaml-cpp::yaml-cpp.
    SET(YAML_CPP_LIBRARIES yaml-cpp::yaml-cpp)
  ELSEIF (TARGET yaml-cpp)
    # yaml-cpp 0.7.0 uses target yaml-cpp (VCPKG).
    SET(YAML_CPP_LIBRARIES yaml-cpp)
  ENDIF()
ELSE()
   find_package(PkgConfig QUIET)
   IF(PKG_CONFIG_FOUND)
      pkg_check_modules(yaml_cpp QUIET yaml-cpp)
	  IF(yaml_cpp_FOUND)
         SET(YAML_CPP_LIBRARIES ${yaml_cpp_LIBRARIES})
         SET(YAML_CPP_INCLUDE_DIR ${yaml_cpp_INCLUDEDIR})
         SET(yaml-cpp_FOUND ${yaml_cpp_FOUND})
	  ENDIF(yaml_cpp_FOUND)
   ENDIF(PKG_CONFIG_FOUND)
ENDIF(yaml-cpp_FOUND)

IF(yaml-cpp_FOUND)

SET(INCLUDE_DIRS
    ${YAML_CPP_INCLUDE_DIR}
)

SET(LIBRARIES
	${YAML_CPP_LIBRARIES}
)

INCLUDE_DIRECTORIES(${INCLUDE_DIRS} yaml-cpp)

ADD_EXECUTABLE(euroc_dataset main.cpp)
  
TARGET_LINK_LIBRARIES(euroc_dataset rtabmap_core ${LIBRARIES})

SET_TARGET_PROPERTIES( euroc_dataset 
    PROPERTIES OUTPUT_NAME ${PROJECT_PREFIX}-euroc_dataset)
    
INSTALL(TARGETS euroc_dataset
	RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT runtime
	BUNDLE DESTINATION "${CMAKE_BUNDLE_LOCATION}" COMPONENT runtime)
	
ELSE()
  MESSAGE(STATUS "yaml-cpp not found, euroc_dataset tool won't be built...")
ENDIF()
