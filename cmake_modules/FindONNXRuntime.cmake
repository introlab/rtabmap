# - Find ONNX Runtime (https://onnxruntime.ai)
#
# Looks for the headers and library of an extracted release archive, an
# install prefix or the usual system locations, then for the CMake package of
# an installed ONNX Runtime. The headers and library are searched first since
# the CMake package shipped in the release archives references paths the
# archives don't have (lib64/, include/onnxruntime/) and aborts configuration
# when it is loaded from there.
#
# Input:
#   ONNXRUNTIME_ROOT  (CMake or environment variable) Directory of an
#                     extracted release archive (onnxruntime-<os>-<arch>-<version>)
#                     or install prefix, when ONNX Runtime is not in a
#                     standard location.
#
# Output:
#   ONNXRuntime_FOUND
#   ONNXRuntime_INCLUDE_DIRS  Directory holding onnxruntime_cxx_api.h
#   ONNXRuntime_LIBRARIES     The onnxruntime library
#   ONNXRuntime_VERSION       e.g. "1.23.2" (empty if unknown)
#   ONNXRuntime::ONNXRuntime  Imported target

if(NOT ONNXRUNTIME_ROOT AND DEFINED ENV{ONNXRUNTIME_ROOT})
  set(ONNXRUNTIME_ROOT "$ENV{ONNXRUNTIME_ROOT}")
endif()

set(_onnxruntime_hints)
if(ONNXRUNTIME_ROOT)
  list(APPEND _onnxruntime_hints "${ONNXRUNTIME_ROOT}")
endif()

# 1. Headers and library in ONNXRUNTIME_ROOT or the usual prefixes.
find_path(ONNXRuntime_INCLUDE_DIR
  NAMES onnxruntime_cxx_api.h
  HINTS ${_onnxruntime_hints}
  PATH_SUFFIXES include include/onnxruntime include/onnxruntime/core/session
  PATHS /usr/local /usr /opt/onnxruntime)
find_library(ONNXRuntime_LIBRARY
  NAMES onnxruntime
  HINTS ${_onnxruntime_hints}
  PATH_SUFFIXES lib lib64
  PATHS /usr/local /usr /opt/onnxruntime)

set(ONNXRuntime_VERSION "")
if(ONNXRuntime_INCLUDE_DIR)
  # The release archives have a VERSION_NUMBER file at their root.
  foreach(_version_file "${ONNXRuntime_INCLUDE_DIR}/../VERSION_NUMBER" "${ONNXRuntime_INCLUDE_DIR}/../../VERSION_NUMBER")
    if(EXISTS "${_version_file}")
      file(STRINGS "${_version_file}" ONNXRuntime_VERSION LIMIT_COUNT 1)
      string(STRIP "${ONNXRuntime_VERSION}" ONNXRuntime_VERSION)
      break()
    endif()
  endforeach()
  if(NOT ONNXRuntime_VERSION AND EXISTS "${ONNXRuntime_INCLUDE_DIR}/onnxruntime_c_api.h")
    # ORT_API_VERSION is the minor version of the 1.x releases.
    file(STRINGS "${ONNXRuntime_INCLUDE_DIR}/onnxruntime_c_api.h" _onnxruntime_api_version REGEX "#define ORT_API_VERSION [0-9]+")
    if(_onnxruntime_api_version)
      string(REGEX REPLACE ".*ORT_API_VERSION ([0-9]+).*" "1.\\1" ONNXRuntime_VERSION "${_onnxruntime_api_version}")
    endif()
  endif()
endif()

# 2. The CMake package of an installation (e.g. cmake --install of a source
#    build) in a location not searched above.
if(NOT ONNXRuntime_INCLUDE_DIR OR NOT ONNXRuntime_LIBRARY)
  find_package(onnxruntime CONFIG QUIET)
  if(onnxruntime_FOUND AND TARGET onnxruntime::onnxruntime)
    get_target_property(_onnxruntime_inc onnxruntime::onnxruntime INTERFACE_INCLUDE_DIRECTORIES)
    get_target_property(_onnxruntime_lib onnxruntime::onnxruntime IMPORTED_LOCATION)
    if(NOT _onnxruntime_lib)
      get_target_property(_onnxruntime_configs onnxruntime::onnxruntime IMPORTED_CONFIGURATIONS)
      foreach(_cfg ${_onnxruntime_configs})
        get_target_property(_onnxruntime_lib onnxruntime::onnxruntime IMPORTED_LOCATION_${_cfg})
        if(_onnxruntime_lib)
          break()
        endif()
      endforeach()
    endif()
    if(_onnxruntime_inc AND _onnxruntime_lib AND EXISTS "${_onnxruntime_lib}")
      foreach(_dir ${_onnxruntime_inc})
        if(EXISTS "${_dir}/onnxruntime_cxx_api.h")
          set(ONNXRuntime_INCLUDE_DIR "${_dir}" CACHE PATH "ONNX Runtime include directory" FORCE)
          set(ONNXRuntime_LIBRARY "${_onnxruntime_lib}" CACHE FILEPATH "ONNX Runtime library" FORCE)
          set(ONNXRuntime_VERSION "${onnxruntime_VERSION}")
          break()
        endif()
      endforeach()
    endif()
  endif()
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(ONNXRuntime
  REQUIRED_VARS ONNXRuntime_LIBRARY ONNXRuntime_INCLUDE_DIR
  VERSION_VAR ONNXRuntime_VERSION)

if(ONNXRuntime_FOUND)
  set(ONNXRuntime_INCLUDE_DIRS "${ONNXRuntime_INCLUDE_DIR}")
  set(ONNXRuntime_LIBRARIES "${ONNXRuntime_LIBRARY}")
  if(NOT TARGET ONNXRuntime::ONNXRuntime)
    add_library(ONNXRuntime::ONNXRuntime SHARED IMPORTED)
    set_target_properties(ONNXRuntime::ONNXRuntime PROPERTIES
      IMPORTED_LOCATION "${ONNXRuntime_LIBRARY}"
      INTERFACE_INCLUDE_DIRECTORIES "${ONNXRuntime_INCLUDE_DIR}")
  endif()
endif()

mark_as_advanced(ONNXRuntime_INCLUDE_DIR ONNXRuntime_LIBRARY)
