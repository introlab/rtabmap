#.rst:
# FindKinectSDK2
# --------------
#
# Find Kinect for Windows SDK v2 (Kinect SDK v2) include dirs, library dirs, libraries
#
# Use this module by invoking find_package with the form::
#
#    find_package( KinectSDK2 [REQUIRED] )
#
# Results for users are reported in following variables::
#
#    KinectSDK2_FOUND                - Return "TRUE" when Kinect SDK v2 found. Otherwise, Return "FALSE".
#    KinectSDK2_INCLUDE_DIRS         - Kinect SDK v2 include directories. (${KinectSDK2_DIR}/inc)
#    KinectSDK2_LIBRARY_DIRS         - Kinect SDK v2 library directories. (${KinectSDK2_DIR}/Lib/x86 or ${KinectSDK2_DIR}/Lib/x64)
#    KinectSDK2_LIBRARIES            - Kinect SDK v2 library files. (${KinectSDK2_LIBRARY_DIRS}/Kinect20.lib (If check the box of any application festures, corresponding library will be added.))
#    KinectSDK2_COMMANDS             - Copy commands of redist files for application functions of Kinect SDK v2. (If uncheck the box of all application features, this variable has defined empty command.)
#
# This module reads hints about search locations from following environment variables::
#
#    KINECTSDK20_DIR                 - Kinect SDK v2 root directory. (This environment variable has been set by installer of Kinect SDK v2.)
#
# CMake entries::
#
#    KinectSDK2_DIR                  - Kinect SDK v2 root directory. (Default $ENV{KINECTSDK20_DIR})
#    KinectSDK2_FACE                 - Check the box when using Face or HDFace features. (Default uncheck)
#    KinectSDK2_FUSION               - Check the box when using Fusion features. (Default uncheck)
#    KinectSDK2_VGB                  - Check the box when using Visual Gesture Builder features. (Default uncheck)
#
# Example to find Kinect SDK v2::
#
#    cmake_minimum_required( VERSION 2.8 )
#
#    project( project )
#    add_executable( project main.cpp )
#    set_property( DIRECTORY PROPERTY VS_STARTUP_PROJECT "project" )
#
#    # Find package using this module.
#    find_package( KinectSDK2 REQUIRED )
#
#    if(KinectSDK2_FOUND)
#      # [C/C++]>[General]>[Additional Include Directories]
#      include_directories( ${KinectSDK2_INCLUDE_DIRS} )
#
#      # [Linker]>[General]>[Additional Library Directories]
#      link_directories( ${KinectSDK2_LIBRARY_DIRS} )
#
#      # [Linker]>[Input]>[Additional Dependencies]
#      target_link_libraries( project ${KinectSDK2_LIBRARIES} )
#
#      # [Build Events]>[Post-Build Event]>[Command Line]
#      add_custom_command( TARGET project POST_BUILD ${KinectSDK2_COMMANDS} )
#    endif()
#
# =============================================================================
#
# Copyright (c) 2016 Tsukasa SUGIURA
# Distributed under the MIT License.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
# The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
#
# =============================================================================

##### Utility #####

# Check Directory Macro
macro(CHECK_DIR _DIR)
  if(NOT EXISTS "${${_DIR}}")
    message(WARNING "Directory \"${${_DIR}}\" not found.")
    set(KinectSDK2_FOUND FALSE)
    unset(_DIR)
  endif()
endmacro()

# Check Files Macro
macro(CHECK_FILES _FILES _DIR)
  set(_MISSING_FILES)
  foreach(_FILE ${${_FILES}})
    if(NOT EXISTS "${_FILE}")
      get_filename_component(_FILE ${_FILE} NAME)
      set(_MISSING_FILES "${_MISSING_FILES}${_FILE}, ")
    endif()
  endforeach()
  if(_MISSING_FILES)
    message(WARNING "In directory \"${${_DIR}}\" not found files: ${_MISSING_FILES}")
    set(KinectSDK2_FOUND FALSE)
    unset(_FILES)
  endif()
endmacro()

# Target Platform
set(TARGET_PLATFORM)
if(NOT CMAKE_CL_64)
  set(TARGET_PLATFORM x86)
else()
  set(TARGET_PLATFORM x64)
endif()

##### Find Kinect SDK v2 #####

# Found
set(KinectSDK2_FOUND TRUE)
if(MSVC_VERSION LESS 1700)
  message(WARNING "Kinect for Windows SDK v2 supported Visual Studio 2012 or later.")
  set(KinectSDK2_FOUND FALSE)
endif()

# Options
option(KinectSDK2_FACE "Face and HDFace features" FALSE)
option(KinectSDK2_FUSION "Fusion features" FALSE)
option(KinectSDK2_VGB "Visual Gesture Builder features" FALSE)

# Root Directoty
set(KinectSDK2_DIR)
if(KinectSDK2_FOUND)
  set(KinectSDK2_DIR $ENV{KINECTSDK20_DIR} CACHE PATH "Kinect for Windows SDK v2 Install Path." FORCE)
  check_dir(KinectSDK2_DIR)
endif()

# Include Directories
set(KinectSDK2_INCLUDE_DIRS)
if(KinectSDK2_FOUND)
  set(KinectSDK2_INCLUDE_DIRS ${KinectSDK2_DIR}/inc)
  check_dir(KinectSDK2_INCLUDE_DIRS)
endif()

# Library Directories
set(KinectSDK2_LIBRARY_DIRS)
if(KinectSDK2_FOUND)
  set(KinectSDK2_LIBRARY_DIRS ${KinectSDK2_DIR}/Lib/${TARGET_PLATFORM})
  check_dir(KinectSDK2_LIBRARY_DIRS)
endif()

# Dependencies
set(KinectSDK2_LIBRARIES)
if(KinectSDK2_FOUND)
  set(KinectSDK2_LIBRARIES ${KinectSDK2_LIBRARY_DIRS}/Kinect20.lib)

  if(KinectSDK2_FACE)
    set(KinectSDK2_LIBRARIES ${KinectSDK2_LIBRARIES};${KinectSDK2_LIBRARY_DIRS}/Kinect20.Face.lib)
  endif()

  if(KinectSDK2_FUSION)
    set(KinectSDK2_LIBRARIES ${KinectSDK2_LIBRARIES};${KinectSDK2_LIBRARY_DIRS}/Kinect20.Fusion.lib)
  endif()

  if(KinectSDK2_VGB)
    set(KinectSDK2_LIBRARIES ${KinectSDK2_LIBRARIES};${KinectSDK2_LIBRARY_DIRS}/Kinect20.VisualGestureBuilder.lib)
  endif()

  check_files(KinectSDK2_LIBRARIES KinectSDK2_LIBRARY_DIRS)
endif()

# Custom Commands
set(KinectSDK2_COMMANDS)
if(KinectSDK2_FOUND)
  if(KinectSDK2_FACE)
    set(KinectSDK2_REDIST_DIR ${KinectSDK2_DIR}/Redist/Face/${TARGET_PLATFORM})
    check_dir(KinectSDK2_REDIST_DIR)
    list(APPEND KinectSDK2_COMMANDS COMMAND xcopy "${KinectSDK2_REDIST_DIR}" "$(OutDir)" /e /y /i /r > NUL)
  endif()

  if(KinectSDK2_FUSION)
    set(KinectSDK2_REDIST_DIR ${KinectSDK2_DIR}/Redist/Fusion/${TARGET_PLATFORM})
    check_dir(KinectSDK2_REDIST_DIR)
    list(APPEND KinectSDK2_COMMANDS COMMAND xcopy "${KinectSDK2_REDIST_DIR}" "$(OutDir)" /e /y /i /r > NUL)
  endif()

  if(KinectSDK2_VGB)
    set(KinectSDK2_REDIST_DIR ${KinectSDK2_DIR}/Redist/VGB/${TARGET_PLATFORM})
    check_dir(KinectSDK2_REDIST_DIR)
    list(APPEND KinectSDK2_COMMANDS COMMAND xcopy "${KinectSDK2_REDIST_DIR}" "$(OutDir)" /e /y /i /r > NUL)
  endif()

  # Empty Commands
  if(NOT KinectSDK2_COMMANDS)
    set(KinectSDK2_COMMANDS COMMAND)
  endif()
endif()

message(STATUS "KinectSDK2_FOUND : ${KinectSDK2_FOUND}")
