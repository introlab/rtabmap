# - Find cuVSLAM library (https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam)
#
#  CUVSLAM_ROOT_DIR environment variable can be set to find the library.
#
# It sets the following variables:
#  CUVSLAM_FOUND         - Set to false, or undefined, if cuVSLAM isn't found.
#  CUVSLAM_INCLUDE_DIRS  - The cuVSLAM include directory.
#  CUVSLAM_LIBRARIES     - The cuVSLAM library to link against.

# First try to find cuVSLAM using its own CMake config (if available)
find_package(CUVSLAM QUIET)

if(NOT CUVSLAM_FOUND)
    # Fallback to manual search if no CMake config is provided
    find_path(CUVSLAM_INCLUDE_DIRS 
        NAMES cuvslam/cuvslam.h
        PATHS
            /usr/include
            /usr/local/include
            /opt/cuvslam/include
            $ENV{CUVSLAM_ROOT}/include
            $ENV{CUVSLAM_ROOT_DIR}/include
    )
    
    find_library(CUVSLAM_LIBRARY 
        NAMES cuvslam
        PATHS
            /usr/lib
            /usr/local/lib
            /opt/cuvslam/lib
            $ENV{CUVSLAM_ROOT}/lib
            $ENV{CUVSLAM_ROOT_DIR}/lib
    )
    
    if(CUVSLAM_INCLUDE_DIRS AND CUVSLAM_LIBRARY)
        set(CUVSLAM_FOUND TRUE)
        set(CUVSLAM_LIBRARIES ${CUVSLAM_LIBRARY})
    endif()
endif()

# Handle the QUIET and REQUIRED arguments
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(CUVSLAM
    FOUND_VAR CUVSLAM_FOUND
    REQUIRED_VARS CUVSLAM_LIBRARIES CUVSLAM_INCLUDE_DIRS
)

if(CUVSLAM_FOUND)
    # Create imported target for modern CMake usage
    if(NOT TARGET cuvslam::cuvslam)
        add_library(cuvslam::cuvslam UNKNOWN IMPORTED)
        set_target_properties(cuvslam::cuvslam PROPERTIES
            IMPORTED_LOCATION "${CUVSLAM_LIBRARIES}"
            INTERFACE_INCLUDE_DIRECTORIES "${CUVSLAM_INCLUDE_DIRS}"
        )
    endif()
    
    # Show which cuVSLAM was found only if not quiet
    if(NOT CUVSLAM_FIND_QUIETLY)
        message(STATUS "Found cuVSLAM: ${CUVSLAM_LIBRARIES}")
    endif()
else()
    # Fatal error if cuVSLAM is required but not found
    if(CUVSLAM_FIND_REQUIRED)
        message(FATAL_ERROR "Could not find cuVSLAM library")
    endif()
endif()

# Mark as advanced
mark_as_advanced(CUVSLAM_INCLUDE_DIRS CUVSLAM_LIBRARY)
