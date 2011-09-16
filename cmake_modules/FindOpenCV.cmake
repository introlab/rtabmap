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

SET(OpenCV_DIR "$ENV{OpenCV_DIR}")

if(NOT OpenCV_DIR)
	# Add ROS OpenCV directory if ROS is installed
	FIND_PROGRAM(ROSPACK_EXEC NAME rospack PATHS)  
	IF(ROSPACK_EXEC)  
	    #MESSAGE(STATUS "Found rospack executable : ${ROSPACK_EXEC}")
		EXECUTE_PROCESS(COMMAND ${ROSPACK_EXEC} find opencv2 
				   	    OUTPUT_VARIABLE OPENCV_ROS_PATH
						OUTPUT_STRIP_TRAILING_WHITESPACE
						WORKING_DIRECTORY "./"
		)
		IF(OPENCV_ROS_PATH)
		    #MESSAGE(STATUS "Found OpenCV ROS pkg : ${OPENCV_ROS_PATH}")
		    SET(OpenCV_DIR ${OPENCV_ROS_PATH}/opencv/share/opencv)
		ENDIF(OPENCV_ROS_PATH)
	ENDIF(ROSPACK_EXEC)
endif(NOT OpenCV_DIR)

if(NOT OpenCV_DIR)
	# typical root dirs of installations, exactly one of them is used
	SET (OpenCV_POSSIBLE_ROOT_DIRS
	  /usr/local
	  /usr/opt
	  /usr
	  )
    find_path(OpenCV_PATH "OpenCVConfig.cmake" HINTS ${OpenCV_POSSIBLE_ROOT_DIRS} PATH_SUFFIXES "share/opencv")
    SET(OpenCV_DIR ${OpenCV_PATH})
endif(NOT OpenCV_DIR)

##====================================================
## Find OpenCV libraries
##----------------------------------------------------
if(EXISTS "${OpenCV_DIR}")
	## Include the standard CMake script
	include("${OpenCV_DIR}/OpenCVConfig.cmake")

	## Search for a specific version
	set(CVLIB_SUFFIX "${OpenCV_VERSION_MAJOR}${OpenCV_VERSION_MINOR}${OpenCV_VERSION_PATCH}")

	if(OpenCV_INCLUDE_DIRS AND OpenCV_LIBS)
		set(OpenCV_FOUND true)
	endif(OpenCV_INCLUDE_DIRS AND OpenCV_LIBS)

else(EXISTS "${OpenCV_DIR}")
        set(ERR_MSG "Please specify OpenCV directory using OpenCV_DIR env. variable (the folder containing OpenCVConfig.cmake)")
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
     message(STATUS "Found OpenCV ${OpenCV_VERSION} : ${OpenCV_DIR}")
   endif(NOT OpenCV_FIND_QUIETLY)
endif()
##====================================================


##====================================================
## Backward compatibility	        ${OpenCV_POSSIBLE_ROOT_DIRS}

##----------------------------------------------------
if(OpenCV_FOUND)
  set(OpenCV_INCLUDE_DIRS "${OpenCV_INCLUDE_DIR}")
  set(OpenCV_LIBRARIES "${OpenCV_LIBS}")
  #option(OpenCV_BACKWARD_COMPA "Add some variable to make this script compatible with the other version of FindOpenCV.cmake" false)
  #if(OpenCV_BACKWARD_COMPA)
  #  find_path(OpenCV_INCLUDE_DIRS "cv.h" PATHS "${OpenCV_DIR}" PATH_SUFFIXES "include" "include/opencv" DOC "Include directory") 
  #  find_path(OpenCV_INCLUDE_DIR "cv.h" PATHS "${OpenCV_DIR}" PATH_SUFFIXES "include" "include/opencv" DOC "Include directory")
  #  set(OpenCV_LIBRARIES "${OpenCV_LIBS}" CACHE STRING "" FORCE)
  #endif(OpenCV_BACKWARD_COMPA)
endif(OpenCV_FOUND)
##====================================================
