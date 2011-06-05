###########################################################
#                  Find OpenCV Library
# See http://sourceforge.net/projects/opencvlibrary/
#----------------------------------------------------------
#
## 1: Setup:
# The following variables are optionally searched for defaults
#  OpenCV_DIR:            Base directory of OpenCv tree to use.
#
## 2: Variable
# The following are set after configuration is done: 
#  
#  OpenCV_FOUND
#  OpenCV_LIBS
#  OpenCV_INCLUDE_DIR
#  OpenCV_VERSION (OpenCV_VERSION_MAJOR, OpenCV_VERSION_MINOR, OpenCV_VERSION_PATCH)
#
#
# Deprecated variable are used to maintain backward compatibility with
# the script of Jan Woetzel (2006/09): www.mip.informatik.uni-kiel.de/~jw
#  OpenCV_INCLUDE_DIRS
#  OpenCV_LIBRARIES
#  OpenCV_LINK_DIRECTORIES
# 
## 3: Version
#
# 2010/04/07 Benoit Rat, Correct a bug when OpenCVConfig.cmake is not found.
# 2010/03/24 Benoit Rat, Add compatibility for when OpenCVConfig.cmake is not found.
# 2010/03/22 Benoit Rat, Creation of the script.
#
#
# tested with:
# - OpenCV 2.1:  MinGW, MSVC2008
# - OpenCV 2.0:  MinGW, MSVC2008, GCC4
#
#
## 4: Licence:
#
# LGPL 2.1 : GNU Lesser General Public License Usage
# Alternatively, this file may be used under the terms of the GNU Lesser

# General Public License version 2.1 as published by the Free Software
# Foundation and appearing in the file LICENSE.LGPL included in the
# packaging of this file.  Please review the following information to
# ensure the GNU Lesser General Public License version 2.1 requirements
# will be met: http://www.gnu.org/licenses/old-licenses/lgpl-2.1.html.
# 
#----------------------------------------------------------

# typical root dirs of installations, exactly one of them is used
SET (OpenCV_POSSIBLE_ROOT_DIRS
  "$ENV{OpenCV_DIR}"
  /usr/local
  /usr/opt
  /usr
  )
  
# Add ROS OpenCV directory if ROS is installed
FIND_PROGRAM(ROSPACK_EXEC NAME rospack PATHS)  
IF(ROSPACK_EXEC)  
    MESSAGE(STATUS "Found rospack executable : ${ROSPACK_EXEC}")
	EXECUTE_PROCESS(COMMAND ${ROSPACK_EXEC} find opencv2 
			   	    OUTPUT_VARIABLE OPENCV_ROS_PATH
					OUTPUT_STRIP_TRAILING_WHITESPACE
					WORKING_DIRECTORY "./"
	)
	IF(OPENCV_ROS_PATH)
	    MESSAGE(STATUS "Found OpenCV ROS pkg : ${OPENCV_ROS_PATH}")
	    SET(OpenCV_POSSIBLE_ROOT_DIRS
	        ${OPENCV_ROS_PATH}/opencv
	        ${OpenCV_POSSIBLE_ROOT_DIRS}
	    )
	ENDIF(OPENCV_ROS_PATH)
ENDIF(ROSPACK_EXEC)

find_path(OpenCV_DIR "OpenCVConfig.cmake" HINTS ${OpenCV_POSSIBLE_ROOT_DIRS} DOC "Root directory of OpenCV")
if(NOT EXISTS "${OpenCV_DIR}")
  if(NOT WIN32)
     find_path(OpenCV_DIR "share/opencv/OpenCVConfig.cmake" HINTS ${OpenCV_POSSIBLE_ROOT_DIRS} DOC "Root directory of OpenCV")
     if(NOT OpenCV_DIR)  
       include(FindPkgConfig)
       if(PKG_CONFIG_FOUND)
         pkg_check_modules(OPENCV_PKGCONF opencv)
         set(OpenCV_DIR ${OPENCV_PKGCONF_PREFIX})
       endif(PKG_CONFIG_FOUND)
     endif(NOT OpenCV_DIR)
  else()
    set(OpenCV_DIR "$ENV{OpenCV_DIR}")
  endif ()
endif(NOT EXISTS "${OpenCV_DIR}")

##====================================================
## Find OpenCV libraries
##----------------------------------------------------
if(EXISTS "${OpenCV_DIR}")

  set(OpenCV_configScript "${OpenCV_DIR}/OpenCVConfig.cmake")
  if(NOT EXISTS "${OpenCV_configScript}")
    set(OpenCV_configScript "${OpenCV_DIR}/share/opencv/OpenCVConfig.cmake")
  endif(NOT EXISTS "${OpenCV_configScript}")


  #When its possible to use the Config script use it.
  if(EXISTS "${OpenCV_configScript}")

    MESSAGE(STATUS "OpenCV: using configuration file ${OpenCV_configScript}")

    ## Include the standard CMake script
    include("${OpenCV_configScript}")

    ## Search for a specific version
    if(WIN32 OR NOT PKG_CONFIG_FOUND)
      set(CVLIB_SUFFIX "${OpenCV_VERSION_MAJOR}${OpenCV_VERSION_MINOR}${OpenCV_VERSION_PATCH}")
    endif(WIN32 OR NOT PKG_CONFIG_FOUND)

    ##Boris:TODO: check for correct OpenCV version in pkg-config case !!!


    #Otherwise it try to guess it.
  else(EXISTS "${OpenCV_configScript}")


    find_path(OpenCV_INCLUDE_DIR "cv.h" PATHS "${OpenCV_DIR}" PATH_SUFFIXES "include" "include/opencv" DOC "")
    if(EXISTS  ${OpenCV_INCLUDE_DIR})
      include_directories(${OpenCV_INCLUDE_DIR})
    endif(EXISTS  ${OpenCV_INCLUDE_DIR})

    if(NOT EXISTS ${VERSION_FILE_DIR}) #may alreay be in cache

      #Find OpenCV version by looking at cvver.h or core/version.hpp
      find_path(VERSION_FILE_DIR "version.hpp" PATHS "${OpenCV_DIR}" PATH_SUFFIXES "include" "include/opencv" "include/opencv2" "include/opencv2/core" DOC "")
      if(NOT EXISTS ${VERSION_FILE_DIR})
	find_path(VERSION_FILE_DIR "cvver.h" PATHS "${OpenCV_DIR}" PATH_SUFFIXES "include" "include/opencv" DOC "")
	if(NOT EXISTS ${VERSION_FILE_DIR})
	  message(FATAL_ERROR "OpenCV version file not found")
	else(NOT EXISTS ${VERSION_FILE_DIR})
	  set(VERSION_FILE ${VERSION_FILE_DIR}/cvver.h)  
	endif(NOT EXISTS ${VERSION_FILE_DIR})

      else(NOT EXISTS ${VERSION_FILE_DIR})
	set(VERSION_FILE ${VERSION_FILE_DIR}/version.hpp)

      endif(NOT EXISTS ${VERSION_FILE_DIR})

      #file(STRINGS ${OpenCV_INCLUDE_DIR}/cvver.h OpenCV_VERSIONS_TMP REGEX "^#define CV_[A-Z]+_VERSION[ \t]+[0-9]+$")
      file(STRINGS ${VERSION_FILE} OpenCV_VERSIONS_TMP REGEX "^#define CV_[A-Z]+_VERSION[ \t]+[0-9]+$")

      string(REGEX REPLACE ".*#define CV_MAJOR_VERSION[ \t]+([0-9]+).*" "\\1" OpenCV_VERSION_MAJOR ${OpenCV_VERSIONS_TMP})
      string(REGEX REPLACE ".*#define CV_MINOR_VERSION[ \t]+([0-9]+).*" "\\1" OpenCV_VERSION_MINOR ${OpenCV_VERSIONS_TMP})
      string(REGEX REPLACE ".*#define CV_SUBMINOR_VERSION[ \t]+([0-9]+).*" "\\1" OpenCV_VERSION_PATCH ${OpenCV_VERSIONS_TMP})
      set(OpenCV_VERSION ${OpenCV_VERSION_MAJOR}.${OpenCV_VERSION_MINOR}.${OpenCV_VERSION_PATCH} CACHE STRING "" FORCE)

      if(WIN32 OR NOT PKG_CONFIG_FOUND)
	set(CVLIB_SUFFIX "${OpenCV_VERSION_MAJOR}${OpenCV_VERSION_MINOR}${OpenCV_VERSION_PATCH}")
      endif(WIN32 OR NOT PKG_CONFIG_FOUND)
      
      ##Boris:TODO: check for correct OpenCV version in pklg-config case !!!


    endif(NOT EXISTS ${VERSION_FILE_DIR})

    if(${OpenCV_VERSION} VERSION_GREATER 2.1.0)
      set(OPENCV_LIB_COMPONENTS calib3d contrib core features2d ffmpeg flann gpu highgui imgproc legacy ml objdetect ts video)

      #Add parent directory of ${OpenCV_INCLUDE_DIR} to ${OpenCV_INCLUDE_DIR} itself
      #to be able to do both
      #include <cv.h> and #include <opencv2/core/core.hpp>
      get_filename_component(PARENT_DIR ${OpenCV_INCLUDE_DIR} PATH)
      set(OpenCV_INCLUDE_DIR "${OpenCV_INCLUDE_DIR};${PARENT_DIR}")
	
    else(${OpenCV_VERSION} VERSION_GREATER 2.1.0)
      set(OPENCV_LIB_COMPONENTS cxcore cv ml highgui cvaux)
    endif(${OpenCV_VERSION} VERSION_GREATER 2.1.0)
    
  endif(EXISTS "${OpenCV_configScript}")


  if(${OpenCV_VERSION} VERSION_GREATER 2.1.0)

    set(CVLIB_SUFFIX ".${OpenCV_VERSION}")
    if(WIN32 OR NOT PKG_CONFIG_FOUND)
      set(CVLIB_SUFFIX "${OpenCV_VERSION_MAJOR}${OpenCV_VERSION_MINOR}${OpenCV_VERSION_PATCH}")
    endif(WIN32 OR NOT PKG_CONFIG_FOUND)

    ## Initiate the variable before the loop
    set(OpenCV_LIBS "")
    set(OpenCV_FOUND_TMP true)

    ## Loop over each components
    foreach(__CVLIB ${OPENCV_LIB_COMPONENTS})

      find_library(OpenCV_${__CVLIB}_LIBRARY_DEBUG NAMES "${__CVLIB}${CVLIB_SUFFIX}d" "lib${__CVLIB}${CVLIB_SUFFIX}d" PATHS "${OpenCV_DIR}/lib" NO_DEFAULT_PATH)
      find_library(OpenCV_${__CVLIB}_LIBRARY_RELEASE NAMES "${__CVLIB}${CVLIB_SUFFIX}" "lib${__CVLIB}${CVLIB_SUFFIX}" "opencv_${__CVLIB}${CVLIB_SUFFIX}" "opencv_${__CVLIB}" "${__CVLIB}" PATHS "${OpenCV_DIR}/lib" NO_DEFAULT_PATH)
	  
      #On MacOSX libraries are named:  libopencv_${__CVLIB}${CVLIB_SUFFIX}.dylib
      #On Linux libraries are named:  libopencv_${__CVLIB}.so${CVLIB_SUFFIX}
      # but with pkg-config ${OPENCV_LIB_COMPONENTS} are already prefixed with opencv_ ?

      #we add: "opencv_${__CVLIB}${CVLIB_SUFFIX}" for MacOSX
      #we add: "${__CVLIB}" for linux (but version is not checked !)
      
      
      #Remove the cache value
      set(OpenCV_${__CVLIB}_LIBRARY "" CACHE STRING "" FORCE)
      
      #both debug/release
      if(OpenCV_${__CVLIB}_LIBRARY_DEBUG AND OpenCV_${__CVLIB}_LIBRARY_RELEASE)
        set(OpenCV_${__CVLIB}_LIBRARY debug ${OpenCV_${__CVLIB}_LIBRARY_DEBUG} optimized ${OpenCV_${__CVLIB}_LIBRARY_RELEASE}  CACHE STRING "" FORCE)

        #only debug
      elseif(OpenCV_${__CVLIB}_LIBRARY_DEBUG)
        set(OpenCV_${__CVLIB}_LIBRARY ${OpenCV_${__CVLIB}_LIBRARY_DEBUG}  CACHE STRING "" FORCE)

        #only release
      elseif(OpenCV_${__CVLIB}_LIBRARY_RELEASE)
        set(OpenCV_${__CVLIB}_LIBRARY ${OpenCV_${__CVLIB}_LIBRARY_RELEASE}  CACHE STRING "" FORCE)

        #no library found
      else()
        set(OpenCV_FOUND_TMP false)

      endif()
      

      #Add to the general list
      if(OpenCV_${__CVLIB}_LIBRARY)
        set(OpenCV_LIBS ${OpenCV_LIBS} ${OpenCV_${__CVLIB}_LIBRARY})
      endif(OpenCV_${__CVLIB}_LIBRARY)

    endforeach(__CVLIB)


    set(OpenCV_FOUND ${OpenCV_FOUND_TMP} CACHE BOOL "" FORCE)


  else(${OpenCV_VERSION} VERSION_GREATER 2.1.0)


    ## Initiate the variable before the loop
    set(OpenCV_LIBS "")
    set(OpenCV_FOUND_TMP true)

    ## Loop over each components
    foreach(__CVLIB ${OPENCV_LIB_COMPONENTS})

      find_library(OpenCV_${__CVLIB}_LIBRARY_DEBUG NAMES "${__CVLIB}${CVLIB_SUFFIX}d" "lib${__CVLIB}${CVLIB_SUFFIX}d" PATHS "${OpenCV_DIR}/lib" NO_DEFAULT_PATH)
      find_library(OpenCV_${__CVLIB}_LIBRARY_RELEASE NAMES "${__CVLIB}${CVLIB_SUFFIX}" "lib${__CVLIB}${CVLIB_SUFFIX}" PATHS "${OpenCV_DIR}/lib" NO_DEFAULT_PATH)


      
      #Remove the cache value
      set(OpenCV_${__CVLIB}_LIBRARY "" CACHE STRING "" FORCE)
      
      #both debug/release
      if(OpenCV_${__CVLIB}_LIBRARY_DEBUG AND OpenCV_${__CVLIB}_LIBRARY_RELEASE)
        set(OpenCV_${__CVLIB}_LIBRARY debug ${OpenCV_${__CVLIB}_LIBRARY_DEBUG} optimized ${OpenCV_${__CVLIB}_LIBRARY_RELEASE}  CACHE STRING "" FORCE)

        #only debug
      elseif(OpenCV_${__CVLIB}_LIBRARY_DEBUG)
        set(OpenCV_${__CVLIB}_LIBRARY ${OpenCV_${__CVLIB}_LIBRARY_DEBUG}  CACHE STRING "" FORCE)

        #only release
      elseif(OpenCV_${__CVLIB}_LIBRARY_RELEASE)
        set(OpenCV_${__CVLIB}_LIBRARY ${OpenCV_${__CVLIB}_LIBRARY_RELEASE}  CACHE STRING "" FORCE)

        #no library found
      else()
        set(OpenCV_FOUND_TMP false)

      endif()
      
      #Add to the general list
      if(OpenCV_${__CVLIB}_LIBRARY)
        set(OpenCV_LIBS ${OpenCV_LIBS} ${OpenCV_${__CVLIB}_LIBRARY})
      endif(OpenCV_${__CVLIB}_LIBRARY)

      
    endforeach(__CVLIB)


    set(OpenCV_FOUND ${OpenCV_FOUND_TMP} CACHE BOOL "" FORCE)

  endif(${OpenCV_VERSION} VERSION_GREATER 2.1.0)


else(EXISTS "${OpenCV_DIR}")
  set(ERR_MSG "Please specify OpenCV directory using OpenCV_DIR env. variable")
endif(EXISTS "${OpenCV_DIR}")
##====================================================

##====================================================
## Print message
##----------------------------------------------------
if(NOT OpenCV_FOUND)
  # make FIND_PACKAGE friendly
  if(NOT OpenCV_FIND_QUIETLY)
    if(OpenCV_FIND_REQUIRED)
      message(FATAL_ERROR "OpenCV required but some headers or libs not found. ${ERR_MSG}")
    else(OpenCV_FIND_REQUIRED)
      message(STATUS "WARNING: OpenCV was not found. ${ERR_MSG}")
    endif(OpenCV_FIND_REQUIRED)
  endif(NOT OpenCV_FIND_QUIETLY)
else()
   if(NOT OpenCV_FIND_QUIETLY)
     message(STATUS "Found OpenCV ${OpenCV_VERSION}")
   endif(NOT OpenCV_FIND_QUIETLY)
endif()
##====================================================


##====================================================
## Backward compatibility
##----------------------------------------------------
if(OpenCV_FOUND)
  set(OpenCV_INCLUDE_DIRS "${OpenCV_INCLUDE_DIR}")
  #option(OpenCV_BACKWARD_COMPA "Add some variable to make this script compatible with the other version of FindOpenCV.cmake" false)
  #if(OpenCV_BACKWARD_COMPA)
  #  find_path(OpenCV_INCLUDE_DIRS "cv.h" PATHS "${OpenCV_DIR}" PATH_SUFFIXES "include" "include/opencv" DOC "Include directory") 
  #  find_path(OpenCV_INCLUDE_DIR "cv.h" PATHS "${OpenCV_DIR}" PATH_SUFFIXES "include" "include/opencv" DOC "Include directory")
  #  set(OpenCV_LIBRARIES "${OpenCV_LIBS}" CACHE STRING "" FORCE)
  #endif(OpenCV_BACKWARD_COMPA)
endif(OpenCV_FOUND)
##====================================================
