# - Find cuVSLAM library (https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam)
#
#  CUVSLAM_ROOT_DIR environment variable can be set to find the library.
#
# It sets the following variables:
#  CUVSLAM_FOUND         - Set to false, or undefined, if cuVSLAM isn't found.
#  CUVSLAM_INCLUDE_DIRS  - The cuVSLAM include directory.
#  CUVSLAM_LIBRARIES     - The cuVSLAM library to link against.

find_package(CUDA REQUIRED)
find_package(Eigen3 REQUIRED)

find_path(CUVSLAM_INCLUDE_DIRS 
    NAMES cuvslam.h
    PATHS
        /usr/include
        /usr/local/include
        /opt/cuvslam/include
        /opt/ros/humble/share/isaac_ros_nitros/cuvslam/include
        $ENV{CUVSLAM_ROOT}/include
        $ENV{CUVSLAM_ROOT_DIR}/include
)

find_library(CUVSLAM_LIBRARY 
    NAMES cuvslam
    PATHS
        /usr/lib
        /usr/local/lib
        /opt/cuvslam/lib
        /opt/ros/humble/share/isaac_ros_nitros/cuvslam/lib
        $ENV{CUVSLAM_ROOT}/lib
        $ENV{CUVSLAM_ROOT_DIR}/lib
)

if(CUVSLAM_INCLUDE_DIRS AND CUVSLAM_LIBRARY)
    set(CUVSLAM_FOUND TRUE)
    set(CUVSLAM_LIBRARIES 
        ${CUVSLAM_LIBRARY} 
        ${CUDA_LIBRARIES}
        # Eigen3 is header-only, so we don't need to link to it
    )
    set(CUVSLAM_INCLUDE_DIRS 
        ${CUVSLAM_INCLUDE_DIRS} 
        ${CUDA_INCLUDE_DIRS} 
        ${EIGEN3_INCLUDE_DIR}
    )
endif()

# Handle the QUIET and REQUIRED arguments
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(CuVSLAM
    FOUND_VAR CUVSLAM_FOUND
    REQUIRED_VARS CUVSLAM_LIBRARIES CUVSLAM_INCLUDE_DIRS
    HANDLE_COMPONENTS
)

if(CUVSLAM_FOUND)
    # Create imported target for modern CMake usage
    if(NOT TARGET cuvslam::cuvslam)
        add_library(cuvslam::cuvslam UNKNOWN IMPORTED)
        set_target_properties(cuvslam::cuvslam PROPERTIES
            IMPORTED_LOCATION "${CUVSLAM_LIBRARY}"
            INTERFACE_INCLUDE_DIRECTORIES "${CUVSLAM_INCLUDE_DIRS}"
            INTERFACE_LINK_LIBRARIES "${CUVSLAM_LIBRARIES};Eigen3::Eigen"
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

mark_as_advanced(CUVSLAM_INCLUDE_DIRS CUVSLAM_LIBRARY)
