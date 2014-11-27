# Top-Level CmakeLists.txt
IF(APPLE OR WIN32)
  # BundleUtilities.cmake from 2.8.3 required on Mac and ?Windows?
  cmake_minimum_required(VERSION 2.8.3)
ELSE()
  cmake_minimum_required(VERSION 2.8.0)
ENDIF()
PROJECT( RTABMap )
SET(PROJECT_PREFIX rtabmap)

####### local cmake modules #######
SET(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake_modules")

#######################
# VERSION
#######################
SET(RTABMAP_MAJOR_VERSION 0)
SET(RTABMAP_MINOR_VERSION 7)
SET(RTABMAP_PATCH_VERSION 2)
SET(RTABMAP_VERSION
  ${RTABMAP_MAJOR_VERSION}.${RTABMAP_MINOR_VERSION}.${RTABMAP_PATCH_VERSION})
  
SET(PROJECT_VERSION "${RTABMAP_VERSION}")

SET(PROJECT_VERSION_MAJOR ${RTABMAP_MAJOR_VERSION})
SET(PROJECT_VERSION_MINOR ${RTABMAP_MINOR_VERSION})
SET(PROJECT_VERSION_PATCH ${RTABMAP_PATCH_VERSION})

SET(PROJECT_SOVERSION "${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}")

####### COMPILATION PARAMS #######
# In case of Makefiles if the user does not setup CMAKE_BUILD_TYPE, assume it's Release:
IF(${CMAKE_GENERATOR} MATCHES ".*Makefiles")
    IF("${CMAKE_BUILD_TYPE}" STREQUAL "")
        set(CMAKE_BUILD_TYPE Release)
    ENDIF("${CMAKE_BUILD_TYPE}" STREQUAL "")
ENDIF(${CMAKE_GENERATOR} MATCHES ".*Makefiles")

SET(CMAKE_DEBUG_POSTFIX "d")

IF(WIN32 AND NOT MINGW)
  	ADD_DEFINITIONS("-DNOMINMAX")
    ADD_DEFINITIONS("-wd4100 -wd4127 -wd4191 -wd4242 -wd4244 -wd4251 -wd4305 -wd4365 -wd4512 -wd4514 -wd4548 -wd4571 -wd4619 -wd4625 -wd4626 -wd4628 -wd4668 -wd4710 -wd4711 -wd4738 -wd4820 -wd4946 -wd4986")
ELSE ()
	ADD_DEFINITIONS( "-Wall" )
	ADD_DEFINITIONS("-Wno-unknown-pragmas")
ENDIF()

IF(MINGW)
    # Hide the --enable-auto-import warning
	SET(CMAKE_EXE_LINKER_FLAGS "-Wl,--enable-auto-import")  
	SET(CMAKE_MODULE_LINKER_FLAGS "-Wl,--enable-auto-import") 
	SET(CMAKE_SHARED_LINKER_FLAGS "-Wl,--enable-auto-import") 
ENDIF(MINGW)

# GCC 4 required
IF(UNIX OR MINGW)
    EXEC_PROGRAM( gcc ARGS "-dumpversion" OUTPUT_VARIABLE GCC_VERSION )
    IF(GCC_VERSION VERSION_LESS "4.0.0")
        MESSAGE(FATAL_ERROR "GCC ${GCC_VERSION} found, but version 4.x.x minimum is required")
    ENDIF(GCC_VERSION VERSION_LESS "4.0.0")
ENDIF(UNIX OR MINGW)

#The CDT Error Parser cannot handle error messages that span 
#more than one line, which is the default gcc behavior. 
#In order to force gcc to generate single line error messages with no line wrapping
IF(CMAKE_COMPILER_IS_GNUCC)
  SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fmessage-length=0")
ENDIF(CMAKE_COMPILER_IS_GNUCC)
IF(CMAKE_COMPILER_IS_GNUCXX)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fmessage-length=0")
ENDIF(CMAKE_COMPILER_IS_GNUCXX)

# [Eclipse] Automatic Discovery of Include directories (Optional, but handy) 
#SET(CMAKE_VERBOSE_MAKEFILE ON)

#Other paths...
IF(APPLE)
   # For Mac ports
   SET(CMAKE_INCLUDE_PATH "/opt/local/include;${CMAKE_INCLUDE_PATH}")
   SET(CMAKE_LIBRARY_PATH "/opt/local/lib;${CMAKE_LIBRARY_PATH}")
ENDIF()

####### Build libraries as shared or static #######
# It is a bit messed up on Windows between STATIC and SHARED, if we don't want
# to add UtiLite_DEFINITIONS in FindUtiLite.cmake... to know from 3rd party application if it needs to
# link dynamically [UTILITE_EXP __declspec( dllimport )] or statically (UTILITE_EXP "nothing").
OPTION( BUILD_SHARED_LIBS "Set to OFF to build static libraries" ON )

####### SET RPATH #########
# When RPATH is activated (supported on most UNIX systems), 
# the user doesn't need to change LD_LIBRARY_PATH

# use, i.e. don't skip the full RPATH for the build tree
SET(CMAKE_SKIP_BUILD_RPATH  FALSE)

# when building, don't use the install RPATH already
# (but later on when installing)
SET(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE) 

# the RPATH to be used when installing
SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_PREFIX}-${RTABMAP_MAJOR_VERSION}.${RTABMAP_MINOR_VERSION}")

# add the automatically determined parts of the RPATH
# which point to directories outside the build tree to the install RPATH
SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

####### OUTPUT DIR #######
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

####### INSTALL DIR #######
# Offer the user the choice of overriding the installation directories
set(INSTALL_LIB_DIR lib/${PROJECT_PREFIX}-${RTABMAP_MAJOR_VERSION}.${RTABMAP_MINOR_VERSION} CACHE PATH "Installation directory for libraries")
set(INSTALL_BIN_DIR bin CACHE PATH "Installation directory for executables")
set(INSTALL_INCLUDE_DIR include/${PROJECT_PREFIX}-${RTABMAP_MAJOR_VERSION}.${RTABMAP_MINOR_VERSION} CACHE PATH
  "Installation directory for header files")
if(WIN32 AND NOT CYGWIN)
  set(DEF_INSTALL_CMAKE_DIR CMake)
else()
  set(DEF_INSTALL_CMAKE_DIR lib/${PROJECT_PREFIX}-${RTABMAP_MAJOR_VERSION}.${RTABMAP_MINOR_VERSION})
endif()
set(INSTALL_CMAKE_DIR ${DEF_INSTALL_CMAKE_DIR} CACHE PATH
  "Installation directory for CMake files")
  
####### BUILD OPTIONS #######
IF(APPLE)
	OPTION(BUILD_AS_BUNDLE "Set to ON to build as bundle (DragNDrop)" OFF)
ENDIF(APPLE)

####### DEPENDENCIES #######
FIND_PACKAGE(OpenCV REQUIRED)
IF(OPENCV_NONFREE_FOUND)
  ADD_DEFINITIONS(-DWITH_NONFREE)
  SET(NONFREE 1)
ELSE()
  SET(NONFREE 0)
ENDIF()
FIND_PACKAGE(PCL 1.7 REQUIRED)
FIND_PACKAGE(VTK REQUIRED)
IF("${VTK_MAJOR_VERSION}" EQUAL 5)
	FIND_PACKAGE(QVTK REQUIRED) # only for VTK 5
ENDIF("${VTK_MAJOR_VERSION}" EQUAL 5)
FIND_PACKAGE(ZLIB REQUIRED)
FIND_PACKAGE(Freenect)
FIND_PACKAGE(OpenNI2)

# If Qt is here, the GUI will be built
FIND_PACKAGE(Qt4 COMPONENTS QtCore QtGui QtSvg) 

####### OSX BUNDLE CMAKE_INSTALL_PREFIX #######
IF(APPLE AND BUILD_AS_BUNDLE)
  IF(QT4_FOUND AND QT_QTCORE_FOUND AND QT_QTGUI_FOUND)
       
    # Required when packaging, and set CMAKE_INSTALL_PREFIX to "/".
    SET(CMAKE_INSTALL_PREFIX "/")
    SET(CPACK_SET_DESTDIR TRUE) 
    
    SET(CMAKE_BUNDLE_NAME "${PROJECT_NAME}")
    SET(CMAKE_BUNDLE_LOCATION "${CMAKE_INSTALL_PREFIX}")
    
    # make sure CMAKE_INSTALL_PREFIX ends in /
    STRING(LENGTH "${CMAKE_INSTALL_PREFIX}" LEN)
    MATH(EXPR LEN "${LEN} -1" )
    STRING(SUBSTRING "${CMAKE_INSTALL_PREFIX}" ${LEN} 1 ENDCH)
    IF(NOT "${ENDCH}" STREQUAL "/")
      SET(CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}/")
    ENDIF(NOT "${ENDCH}" STREQUAL "/")
    SET(CMAKE_INSTALL_PREFIX 
      "${CMAKE_INSTALL_PREFIX}${CMAKE_BUNDLE_NAME}.app/Contents")
  ELSE()
    
    #If Qt is not here, no need to build a bundle
    SET(BUILD_AS_BUNDLE OFF)
  ENDIF()
ENDIF(APPLE AND BUILD_AS_BUNDLE)


####### SOURCES (Projects) #######
CONFIGURE_FILE(Version.h.in ${PROJECT_SOURCE_DIR}/corelib/include/${PROJECT_PREFIX}/core/Version.h)

ADD_SUBDIRECTORY( utilite )
ADD_SUBDIRECTORY( corelib )

IF(QT4_FOUND AND QT_QTCORE_FOUND AND QT_QTGUI_FOUND)
   ADD_SUBDIRECTORY( guilib )
   ADD_SUBDIRECTORY( app )
ELSE()
   MESSAGE(STATUS "[WARNING] Qt4 not found, the GUI lib and the stand-alone application will not be compiled...")
ENDIF()

ADD_SUBDIRECTORY( tools )
ADD_SUBDIRECTORY( examples )

#######################
# Uninstall target, for "make uninstall"
#######################
CONFIGURE_FILE(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake_uninstall.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
  IMMEDIATE @ONLY)

ADD_CUSTOM_TARGET(uninstall
  "${CMAKE_COMMAND}" -P "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake")

####
# Setup RTABMapConfig.cmake
####
# Create the RTABMapConfig.cmake and RTABMapConfigVersion files
file(RELATIVE_PATH REL_INCLUDE_DIR "${CMAKE_INSTALL_PREFIX}/${INSTALL_CMAKE_DIR}" "${CMAKE_INSTALL_PREFIX}/${INSTALL_INCLUDE_DIR}")
file(RELATIVE_PATH REL_LIB_DIR "${CMAKE_INSTALL_PREFIX}/${INSTALL_CMAKE_DIR}" "${CMAKE_INSTALL_PREFIX}/${INSTALL_LIB_DIR}")
   
# ... for the build tree
set(CONF_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}/corelib/include" 
					  "${PROJECT_SOURCE_DIR}/guilib/include" 
					  "${PROJECT_SOURCE_DIR}/utilite/include")
set(CONF_LIB_DIR "${CMAKE_ARCHIVE_OUTPUT_DIRECTORY}")
configure_file(RTABMapConfig.cmake.in
  "${PROJECT_BINARY_DIR}/RTABMapConfig.cmake" @ONLY)
  
# ... for the install tree
set(CONF_INCLUDE_DIRS "\${RTABMap_CMAKE_DIR}/${REL_INCLUDE_DIR}")
set(CONF_LIB_DIR "\${RTABMap_CMAKE_DIR}/${REL_LIB_DIR}")
configure_file(RTABMapConfig.cmake.in
  "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/RTABMapConfig.cmake" @ONLY)
  
# ... for both
configure_file(RTABMapConfigVersion.cmake.in
  "${PROJECT_BINARY_DIR}/RTABMapConfigVersion.cmake" @ONLY)

# Install the RTABMapConfig.cmake and RTABMapConfigVersion.cmake
install(FILES
  "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/RTABMapConfig.cmake"
  "${PROJECT_BINARY_DIR}/RTABMapConfigVersion.cmake"
  DESTINATION "${INSTALL_CMAKE_DIR}" COMPONENT devel)
####

### Install package.xml for catkin
install(FILES package.xml DESTINATION "share/${PROJECT_PREFIX}")

#######################
# CPACK (Packaging)
#######################
INCLUDE(InstallRequiredSystemLibraries)

SET(CPACK_PACKAGE_NAME "${PROJECT_NAME}")
SET(CPACK_PACKAGE_VENDOR "${PROJECT_NAME} project")
SET(CPACK_PACKAGE_DESCRIPTION_SUMMARY "RTAB-MAP is a Real-Time Appearance-Based Mapping approach.")
#SET(CPACK_PACKAGE_DESCRIPTION_FILE "${CMAKE_SOURCE_DIR}/Description.txt")
#SET(CPACK_RESOURCE_FILE_README "${CMAKE_SOURCE_DIR}/README.txt")
SET(CPACK_PACKAGE_VERSION_MAJOR "${PROJECT_VERSION_MAJOR}")
SET(CPACK_PACKAGE_VERSION_MINOR "${PROJECT_VERSION_MINOR}")
SET(CPACK_PACKAGE_VERSION_PATCH "${PROJECT_VERSION_PATCH}")
#SET(CPACK_PACKAGE_INSTALL_DIRECTORY "${PROJECT_NAME}${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}")
#SET(CPACK_PACKAGE_EXECUTABLES "")
#SET(CPACK_SOURCE_PACKAGE_FILE_NAME "${PROJECT_NAME}-${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}.${CPACK_PACKAGE_VERSION_PATCH}")
SET(CPACK_PACKAGE_CONTACT "matlabbe@gmail.com")

set(CPACK_SOURCE_IGNORE_FILES 
  "\\\\.svn/" 
  "\\\\.settings/" 
  "${PROJECT_SOURCE_DIR}/build/[a-zA-Z0-9_]+" 
  "~$" 
  "${PROJECT_SOURCE_DIR}/bin/.*${PROJECT_PREFIX}"
  "${PROJECT_SOURCE_DIR}/bin/.*${PROJECT_NAME}"
  "${PROJECT_SOURCE_DIR}/bin/.*[tT]est"
  "${PROJECT_SOURCE_DIR}/bin/.*[eE]xample"
  "${PROJECT_SOURCE_DIR}/bin/.*uresourcegenerator"
  "\\\\.DS_Store"
)

IF(WIN32)
  SET(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_SOURCE_DIR}/LICENSE")
  IF(CMAKE_CL_64) 
    SET(CPACK_NSIS_INSTALL_ROOT "$PROGRAMFILES64") 
  ELSE() 
    SET(CPACK_NSIS_INSTALL_ROOT "$PROGRAMFILES") 
  ENDIF()
  SET(CPACK_GENERATOR "ZIP;NSIS")
  SET(CPACK_SOURCE_GENERATOR "ZIP")
  SET(CPACK_NSIS_PACKAGE_NAME "${PROJECT_NAME} ${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}.${CPACK_PACKAGE_VERSION_PATCH}")
  SET(ICON_PATH "${PROJECT_SOURCE_DIR}/app/src/${PROJECT_NAME}.ico")
  SET(CPACK_NSIS_MUI_ICON ${ICON_PATH})
  SET(CPACK_NSIS_MUI_UNIICON ${ICON_PATH})
  #SET(CPACK_PACKAGE_ICON ${ICON_PATH})
  #SET(CPACK_NSIS_INSTALLED_ICON_NAME ${ICON_PATH})
  #SET(CPACK_NSIS_HELP_LINK "http:\\\\\\\\www.${PROJECT_PREFIX}.googlecode.com")
  #SET(CPACK_NSIS_URL_INFO_ABOUT "http:\\\\\\\\www.${PROJECT_PREFIX}.googlecode.com")
  SET(CPACK_NSIS_DISPLAY_NAME "${PROJECT_NAME}")
  SET(CPACK_NSIS_CONTACT ${CPACK_PACKAGE_CONTACT})
  # Set the icon used for the Windows "Add or Remove Programs" tool.
  SET(CPACK_NSIS_INSTALLED_ICON_NAME bin\\\\${PROJECT_NAME}.exe)
  # Desktop link ("executableName" "linkName")
  SET(CPACK_PACKAGE_EXECUTABLES "${PROJECT_NAME}" "${PROJECT_NAME}" ${CPACK_PACKAGE_EXECUTABLES}) 
  SET(CPACK_CREATE_DESKTOP_LINKS "${PROJECT_NAME}" ${CPACK_CREATE_DESKTOP_LINKS}) 
  SET(CPACK_NSIS_MODIFY_PATH ON)
  
ELSEIF(APPLE)
  # The project is created as a bundle over the main app (see ./app/src).
  # Here we package only this bundle. Note that we set 
  # CMAKE_INSTALL_PREFIX to "/" when packaging...
  IF(BUILD_AS_BUNDLE)
    SET(CPACK_GENERATOR "DragNDrop")
  ELSE()
    SET(CPACK_GENERATOR "PackageMaker;TBZ2")
  ENDIF()
  
  SET(CPACK_SOURCE_GENERATOR "ZIP")

  SET(CPACK_PACKAGE_ICON "${PROJECT_SOURCE_DIR}/app/src/${PROJECT_NAME}.icns")

ELSE()
  SET(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_SOURCE_DIR}/LICENSE")
  SET(CPACK_GENERATOR "TBZ2")
  SET(CPACK_SOURCE_GENERATOR "ZIP")
ENDIF()

INCLUDE(CPack)

MESSAGE(STATUS "--------------------------------------------")
MESSAGE(STATUS "Info :")
MESSAGE(STATUS "  CMAKE_INSTALL_PREFIX = ${CMAKE_INSTALL_PREFIX}")
MESSAGE(STATUS "  CMAKE_BUILD_TYPE = ${CMAKE_BUILD_TYPE}")
IF(NOT WIN32)
	# see comment above for the BUILD_SHARED_LIBS option on Windows
	MESSAGE(STATUS "  BUILD_SHARED_LIBS = ${BUILD_SHARED_LIBS}")
ENDIF(NOT WIN32)
IF(APPLE)
MESSAGE(STATUS "  BUILD_AS_BUNDLE = ${BUILD_AS_BUNDLE}")
ENDIF(APPLE)

IF(OPENCV_NONFREE_FOUND)
MESSAGE(STATUS "  With OpenCV nonfree module (SIFT/SURF) = YES")
ELSE()
MESSAGE(STATUS "  With OpenCV nonfree module (SIFT/SURF) = NO (not found)")
ENDIF()

IF(Freenect_FOUND)
MESSAGE(STATUS "  With Freenect = YES")
ELSE()
MESSAGE(STATUS "  With Freenect = NO (libfreenect not found)")
ENDIF()

IF(OpenNI2_FOUND)
MESSAGE(STATUS "  With OpenNI2  = YES")
ELSE()
MESSAGE(STATUS "  With OpenNI2  = NO (OpenNI2 not found)")
ENDIF()

MESSAGE(STATUS "--------------------------------------------")
