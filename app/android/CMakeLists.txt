
option(WITH_TANGO         "Include Tango support"                ON)
option(WITH_ARCORE        "Include ARCore support"               ON)
option(WITH_ARENGINE      "Include AREngine support"             ON)
option(DISABLE_LOG        "Disable Android logging (should be true in release)"    ON)
option(DEPTH_TEST         "Enable depth test on ARCore"    OFF)

IF(WITH_TANGO)
    FIND_PACKAGE(Tango QUIET)
    IF(Tango_FOUND)
       MESSAGE(STATUS "Found Tango: ${Tango_INCLUDE_DIRS}")
    ENDIF(Tango_FOUND)
ENDIF(WITH_TANGO)

IF(WITH_ARCORE)
    FIND_PACKAGE(ARCore QUIET)
    IF(ARCore_FOUND)
       MESSAGE(STATUS "Found ARCore: ${ARCore_INCLUDE_DIRS}")
    ENDIF(ARCore_FOUND)
ENDIF(WITH_ARCORE)

IF(WITH_ARENGINE)
    FIND_PACKAGE(AREngine QUIET)
    IF(AREngine_FOUND)
       MESSAGE(STATUS "Found AREngine: ${AREngine_INCLUDE_DIRS}")
    ENDIF(AREngine_FOUND)
ENDIF(WITH_ARENGINE)

IF(NOT Tango_FOUND)
   SET(TANGO "//")
ENDIF(NOT Tango_FOUND)
IF(NOT ARCore_FOUND)
   SET(ARCORE "//")
ENDIF(NOT ARCore_FOUND)
IF(NOT AREngine_FOUND)
   SET(ARENGINE "//")
ENDIF(NOT AREngine_FOUND)

CONFIGURE_FILE(CameraAvailability.h.in ${CMAKE_CURRENT_SOURCE_DIR}/jni/CameraAvailability.h)


IF(DISABLE_LOG)
  ADD_DEFINITIONS(-DDISABLE_LOG)
ENDIF(DISABLE_LOG)
IF(DEPTH_TEST)
  ADD_DEFINITIONS(-DDEPTH_TEST)
ENDIF(DEPTH_TEST)

MESSAGE(STATUS "--------------------------------------------")
MESSAGE(STATUS "Android build info:")
MESSAGE(STATUS "  DISABLE_LOG    = ${DISABLE_LOG}")
MESSAGE(STATUS "  DEPTH_TEST     = ${DEPTH_TEST}")
IF(Tango_FOUND)
MESSAGE(STATUS "  With Tango     = YES")
ELSEIF(NOT WITH_TANGO)
MESSAGE(STATUS "  With Tango     = NO (WITH_TANGO=OFF)")
ELSE()
MESSAGE(STATUS "  With Tango     = NO (tango not found)")
ENDIF()
IF(ARCore_FOUND)
MESSAGE(STATUS "  With ARCore    = YES")
ELSEIF(NOT WITH_ARCORE)
MESSAGE(STATUS "  With ARCore    = NO (WITH_ARCORE=OFF)")
ELSE()
MESSAGE(STATUS "  With ARCore    = NO (ARCore not found)")
ENDIF()
IF(AREngine_FOUND)
MESSAGE(STATUS "  With AREngine  = YES")
ELSEIF(NOT WITH_ARCORE)
MESSAGE(STATUS "  With AREngine  = NO (WITH_ARENGINE=OFF)")
ELSE()
MESSAGE(STATUS "  With AREngine  = NO (AREngine not found)")
ENDIF()

IF(DISABLE_LOG)
   SET(ANDROID_DEBUGGABLE false)
ELSE()
   SET(ANDROID_DEBUGGABLE true)
ENDIF()

add_subdirectory(jni)


######
# Packaging
######

# find android
find_host_program(ANDROID_EXECUTABLE
  NAMES android
  DOC   "The android command-line tool")
if(NOT ANDROID_EXECUTABLE)
  message(FATAL_ERROR "Can not find android command line tool: android")
endif()

configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/AndroidManifest.xml.in"
  "${CMAKE_CURRENT_SOURCE_DIR}/AndroidManifest.xml")
  
configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/AndroidManifest.xml"
  "${CMAKE_CURRENT_BINARY_DIR}/AndroidManifest.xml"
  COPYONLY)
  
configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/info.txt.in"
  "${CMAKE_CURRENT_SOURCE_DIR}/res/raw/info.txt")
  

configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/ant.properties.in"
  "${CMAKE_CURRENT_BINARY_DIR}/ant.properties"
  @ONLY)
  
configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/custom_rules.xml"
  "${CMAKE_CURRENT_BINARY_DIR}/custom_rules.xml"
  COPYONLY)

add_custom_target(NativeRTABMap-ant-configure ALL
  COMMAND "${ANDROID_EXECUTABLE}"
          update project
          --name RTABMap
          --path "${CMAKE_CURRENT_SOURCE_DIR}"
          --target "android-${ANDROID_NATIVE_API_LEVEL}"
  COMMAND "${CMAKE_COMMAND}" -E copy_if_different
          "${CMAKE_CURRENT_SOURCE_DIR}/build.xml"
          "${CMAKE_CURRENT_BINARY_DIR}/build.xml"
  COMMAND "${CMAKE_COMMAND}" -E copy_if_different
          "${CMAKE_CURRENT_SOURCE_DIR}/local.properties"
          "${CMAKE_CURRENT_BINARY_DIR}/local.properties"
  COMMAND "${CMAKE_COMMAND}" -E copy_if_different
          "${CMAKE_CURRENT_SOURCE_DIR}/project.properties"
          "${CMAKE_CURRENT_BINARY_DIR}/project.properties"
  COMMAND "${CMAKE_COMMAND}" -E copy_if_different
          "${CMAKE_CURRENT_SOURCE_DIR}/proguard-project.txt"
          "${CMAKE_CURRENT_BINARY_DIR}/proguard-project.txt"
  COMMAND "${CMAKE_COMMAND}" -E remove
          "${CMAKE_CURRENT_SOURCE_DIR}/build.xml"
          "${CMAKE_CURRENT_SOURCE_DIR}/local.properties"
          "${CMAKE_CURRENT_SOURCE_DIR}/project.properties"
          "${CMAKE_CURRENT_SOURCE_DIR}/proguard-project.txt"
  WORKING_DIRECTORY
          "${CMAKE_CURRENT_BINARY_DIR}")

add_dependencies(NativeRTABMap-ant-configure NativeRTABMap)

#find ant
find_host_program(ANT_EXECUTABLE
  NAMES ant
  DOC   "The ant build tool")
if(NOT ANT_EXECUTABLE)
  message(FATAL_ERROR "Can not find ant build tool: ant")
endif()

add_custom_target(NativeRTABMap-apk-release ALL
  COMMAND ${ANT_EXECUTABLE}
          -file "${CMAKE_CURRENT_BINARY_DIR}/build.xml"
          release)
add_dependencies(NativeRTABMap-apk-release
  NativeRTABMap-ant-configure
  NativeRTABMap)

add_custom_target(NativeRTABMap-apk-debug ALL
  COMMAND ${ANT_EXECUTABLE}
          -file "${CMAKE_CURRENT_BINARY_DIR}/build.xml"
          debug)
add_dependencies(NativeRTABMap-apk-debug
#  NativeRTABMap-apk-release
  NativeRTABMap-ant-configure
  NativeRTABMap)
