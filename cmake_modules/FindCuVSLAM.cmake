# - Find cuVSLAM library (https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam)
#
#  CUVSLAM_ROOT_DIR environment variable can be set to find the library.
#
# It sets the following variables:
#  CUVSLAM_FOUND         - Set to false, or undefined, if cuVSLAM isn't found.
#  CUVSLAM_VERSION       - The version of cuVSLAM found (e.g., "14.0.0").
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
    # Extract version from cuvslam.h header
    file(STRINGS "${CUVSLAM_INCLUDE_DIRS}/cuvslam.h" CUVSLAM_VERSION_MAJOR_LINE 
         REGEX "^#define CUVSLAM_API_VERSION_MAJOR")
    file(STRINGS "${CUVSLAM_INCLUDE_DIRS}/cuvslam.h" CUVSLAM_VERSION_MINOR_LINE 
         REGEX "^#define CUVSLAM_API_VERSION_MINOR")
    
    if(CUVSLAM_VERSION_MAJOR_LINE AND CUVSLAM_VERSION_MINOR_LINE)
        string(REGEX MATCH "[0-9]+" CUVSLAM_VERSION_MAJOR "${CUVSLAM_VERSION_MAJOR_LINE}")
        string(REGEX MATCH "[0-9]+" CUVSLAM_VERSION_MINOR "${CUVSLAM_VERSION_MINOR_LINE}")
        set(CUVSLAM_VERSION "${CUVSLAM_VERSION_MAJOR}.${CUVSLAM_VERSION_MINOR}.0")
    endif()
    
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

# Version compatibility check - cuVSLAM only guarantees API compatibility within the same major version
set(CUVSLAM_VERSION_MISMATCH_REASON "")
if(CuVSLAM_FIND_VERSION AND CUVSLAM_VERSION)
    string(REGEX MATCH "^[0-9]+" REQUESTED_MAJOR_VERSION "${CuVSLAM_FIND_VERSION}")
    if(NOT CUVSLAM_VERSION_MAJOR EQUAL REQUESTED_MAJOR_VERSION)
        set(CUVSLAM_VERSION_MISMATCH_REASON "Major version mismatch: found ${CUVSLAM_VERSION_MAJOR}.x but requested ${REQUESTED_MAJOR_VERSION}.x.\ncuVSLAM only guarantees API compatibility within the same major version.\nPlease install cuVSLAM ${REQUESTED_MAJOR_VERSION}.x or update CMakeLists.txt to request version ${CUVSLAM_VERSION_MAJOR}.0.0")
        
        if(CuVSLAM_FIND_REQUIRED)
            message(FATAL_ERROR 
                "cuVSLAM major version mismatch: found version ${CUVSLAM_VERSION} but version ${CuVSLAM_FIND_VERSION} is required.\n"
                "cuVSLAM only guarantees API compatibility within the same major version.\n"
                "Found major version ${CUVSLAM_VERSION_MAJOR} is not compatible with requested major version ${REQUESTED_MAJOR_VERSION}.\n"
                "Please install cuVSLAM ${REQUESTED_MAJOR_VERSION}.x or update CMakeLists.txt to request version ${CUVSLAM_VERSION_MAJOR}.x."
            )
        else()
            # Clear the found variables to indicate incompatibility
            unset(CUVSLAM_LIBRARIES)
            unset(CUVSLAM_INCLUDE_DIRS)
        endif()
    endif()
endif()

# Handle the QUIET and REQUIRED arguments
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(CuVSLAM
    FOUND_VAR CUVSLAM_FOUND
    REQUIRED_VARS CUVSLAM_LIBRARIES CUVSLAM_INCLUDE_DIRS
    VERSION_VAR CUVSLAM_VERSION
    REASON_FAILURE_MESSAGE "${CUVSLAM_VERSION_MISMATCH_REASON}"
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
endif()

mark_as_advanced(CUVSLAM_INCLUDE_DIRS CUVSLAM_LIBRARY)
