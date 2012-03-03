# Top-Level CmakeLists.txt
IF(APPLE OR WIN32)
  # BundleUtilities.cmake from 2.8.3 required on Mac and ?Windows?
  cmake_minimum_required(VERSION 2.8.3)
ELSE()
  cmake_minimum_required(VERSION 2.8.0)
ENDIF()
PROJECT( RTAB-Map )
SET(PROJECT_PREFIX rtabmap)

####### local cmake modules #######
SET(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake_modules")

#######################
# VERSION
#######################
SET(PROJECT_VERSION "0.3.2")

STRING(REGEX MATCHALL "[0-9]" PROJECT_VERSION_PARTS "${PROJECT_VERSION}")

LIST(GET PROJECT_VERSION_PARTS 0 PROJECT_VERSION_MAJOR)
LIST(GET PROJECT_VERSION_PARTS 1 PROJECT_VERSION_MINOR)
LIST(GET PROJECT_VERSION_PARTS 2 PROJECT_VERSION_PATCH)

SET(PROJECT_SOVERSION "${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}")

CONFIGURE_FILE(Version.h.in ${PROJECT_SOURCE_DIR}/corelib/include/${PROJECT_PREFIX}/core/Version.h)

####### COMPILATION PARAMS #######
# In case of Makefiles if the user does not setup CMAKE_BUILD_TYPE, assume it's Release:
IF(${CMAKE_GENERATOR} MATCHES ".*Makefiles")
    IF("${CMAKE_BUILD_TYPE}" STREQUAL "")
        set(CMAKE_BUILD_TYPE Release)
    ENDIF("${CMAKE_BUILD_TYPE}" STREQUAL "")
ENDIF(${CMAKE_GENERATOR} MATCHES ".*Makefiles")

SET(CMAKE_DEBUG_POSTFIX "d")

ADD_DEFINITIONS( "-Wall" )
IF(WIN32 AND NOT MINGW)
    ADD_DEFINITIONS("-wd4100 -wd4512 -wd4548 -wd4619 -wd4625 -wd4626 -wd4668 -wd4710 -wd4820")
ELSE ()
	ADD_DEFINITIONS("-Wno-unknown-pragmas")
ENDIF()

IF(MINGW)
    # Hide the --enable-auto-import warning
	SET(CMAKE_EXE_LINKER_FLAGS "-enable-auto-import")  
	SET(CMAKE_MODULE_LINKER_FLAGS "-enable-auto-import") 
	SET(CMAKE_SHARED_LINKER_FLAGS "-enable-auto-import") 
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

#Find libraries included with our project first
IF(WIN32)
   SET(CMAKE_INCLUDE_PATH "${PROJECT_SOURCE_DIR}/3rdParty/include;${CMAKE_INCLUDE_PATH}")
   SET(CMAKE_LIBRARY_PATH "${PROJECT_SOURCE_DIR}/3rdParty/lib;${CMAKE_LIBRARY_PATH}")
ELSEIF(APPLE)
   # For Mac ports
   SET(CMAKE_INCLUDE_PATH "/opt/local/include;${CMAKE_INCLUDE_PATH}")
   SET(CMAKE_LIBRARY_PATH "/opt/local/lib;${CMAKE_LIBRARY_PATH}")
ENDIF()

####### Build libraries as shared or static #######
OPTION(BUILD_SHARED_LIBS "Set to OFF to build static libraries" ON)

####### SET RPATH #########
# When RPATH is activated (supported on most UNIX systems), 
# the user doesn't need to change LD_LIBRARY_PATH

# use, i.e. don't skip the full RPATH for the build tree
SET(CMAKE_SKIP_BUILD_RPATH  FALSE)

# when building, don't use the install RPATH already
# (but later on when installing)
SET(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE) 

# the RPATH to be used when installing
SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

# add the automatically determined parts of the RPATH
# which point to directories outside the build tree to the install RPATH
SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

####### OUTPUT DIR #######
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)


####### DEPENDENCIES #######
FIND_PACKAGE(UtiLite REQUIRED)
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(Sqlite3 REQUIRED)
FIND_PACKAGE(FFTW3F REQUIRED)
FIND_PACKAGE(ZLIB REQUIRED)
# If Qt is here, the GUI will be built
FIND_PACKAGE(Qt4 COMPONENTS QtCore QtGui QtSvg) 
# If CppUnit is here, the test will be built
FIND_PACKAGE(CppUnit)

####### OSX BUNDLE CMAKE_INSTALL_PREFIX #######
OPTION(BUILD_AS_BUNDLE "Set to ON to build as bundle (DragNDrop)" OFF)
IF(APPLE AND BUILD_AS_BUNDLE)
  IF(QT4_FOUND AND QT_QTCORE_FOUND AND QT_QTGUI_FOUND)
    
    #Force to Off, easiest to package into the bundle
    SET( BUILD_SHARED_LIBS OFF )
    
    # Required when packaging, and set CMAKE_INSTALL_PREFIX to "/".
    SET(CPACK_SET_DESTDIR TRUE) 
    
    SET(CMAKE_BUNDLE_NAME
      "${PROJECT_NAME} ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}")
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
ADD_SUBDIRECTORY( corelib )
INCLUDE_DIRECTORIES(corelib/include/)

IF(QT4_FOUND AND QT_QTCORE_FOUND AND QT_QTGUI_FOUND)
   ADD_SUBDIRECTORY( guilib )
   INCLUDE_DIRECTORIES(guilib/include/)
   ADD_SUBDIRECTORY( app )
ELSE()
   MESSAGE(STATUS "[WARNING] Qt4 not found, the GUI lib and the stand-alone application will not be compiled...")
ENDIF()

#######################
# Uninstall target, for "make uninstall"
#######################
CONFIGURE_FILE(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake_uninstall.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
  IMMEDIATE @ONLY)

ADD_CUSTOM_TARGET(uninstall
  "${CMAKE_COMMAND}" -P "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake")


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
  "${PROJECT_SOURCE_DIR}/build/[a-zA-Z0-9_]+" 
  "~$" 
  "${PROJECT_SOURCE_DIR}/bin/.*${PROJECT_PREFIX}"
  "${PROJECT_SOURCE_DIR}/bin/.*${PROJECT_NAME}"
  "${PROJECT_SOURCE_DIR}/bin/.*[tT]est"
  "${PROJECT_SOURCE_DIR}/bin/.*[eE]xample"
  "\\\\.DS_Store"
)

# Share files
INSTALL(FILES 
 ${PROJECT_SOURCE_DIR}/Matlab/ShowLogs/showlogs.m 
 ${PROJECT_SOURCE_DIR}/Matlab/ShowLogs/getPrecisionRecall.m 
 ${PROJECT_SOURCE_DIR}/Matlab/ShowLogs/importfile.m 
 DESTINATION share/${PROJECT_PREFIX} 
 COMPONENT runtime)

IF(WIN32)
  SET(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_SOURCE_DIR}/COPYING.txt")

  SET(CPACK_GENERATOR "NSIS;ZIP")
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
  
  SET(CPACK_SOURCE_GENERATOR "TBZ2")

  SET(CPACK_PACKAGE_ICON "${PROJECT_SOURCE_DIR}/app/src/${PROJECT_NAME}.icns")

ELSE()
  SET(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_SOURCE_DIR}/COPYING.txt")
  SET(CPACK_GENERATOR "DEB;TBZ2")
  SET(CPACK_SOURCE_GENERATOR "TBZ2")
  
  # Extern : UtiLite (>= 0.2) http://code.google.com/p/utilite/, OpenCV (>= 2.2) http://opencv.willowgarage.com/wiki/
  # ex. SET(CPACK_DEBIAN_PACKAGE_DEPENDS "libc6 (>= 2.3.1-6), libgcc1 (>= 1:3.4.2-12)")
  # have a look at GET_PROPERTY(result GLOBAL ENABLED_FEATURES), this returns the successful FIND_PACKAGE() calls, maybe this can help 
  SET(CPACK_DEBIAN_PACKAGE_DEPENDS "libc6 (>= 2.4), libstdc++6 (>= 4.4), libgcc1 (>= 4.0), libsqlite3-0 (>= 3.6)") # Most have
  SET(CPACK_DEBIAN_PACKAGE_RECOMMENDS "libqt4-dev (>= 4.6)") #Qt
  SET(CPACK_DEBIAN_PACKAGE_SUGGESTS "")
  
  SET(CPACK_DEBIAN_PACKAGE_SECTION "devel")
  SET(CPACK_DEBIAN_PACKAGE_PRIORITY "optional")
  
ENDIF()

INCLUDE(CPack)

MESSAGE(STATUS "--------------------------------------------")
MESSAGE(STATUS "Info :")
MESSAGE(STATUS "  CMAKE_INSTALL_PREFIX = ${CMAKE_INSTALL_PREFIX}")
MESSAGE(STATUS "  CMAKE_BUILD_TYPE = ${CMAKE_BUILD_TYPE}")
MESSAGE(STATUS "  BUILD_SHARED_LIBS = ${BUILD_SHARED_LIBS}")
IF(APPLE)
MESSAGE(STATUS "  BUILD_AS_BUNDLE = ${BUILD_AS_BUNDLE}")
ENDIF(APPLE)
MESSAGE(STATUS "--------------------------------------------")
