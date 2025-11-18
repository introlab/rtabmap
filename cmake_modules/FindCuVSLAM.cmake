# - Find cuVSLAM library (https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam)
#
#  CUVSLAM_ROOT_DIR environment variable can be set to find the library.
#
# It sets the following variables:
#  CUVSLAM_FOUND         - Set to false, or undefined, if cuVSLAM isn't found.
#  CuVSLAM_VERSION       - The major version of cuVSLAM found (e.g., "14").
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
    
    # Extract version from cuvslam.h header
    file(STRINGS "${CUVSLAM_INCLUDE_DIRS}/cuvslam.h" CUVSLAM_VERSION_MAJOR_LINE 
         REGEX "^#define CUVSLAM_API_VERSION_MAJOR")
    
    if(CUVSLAM_VERSION_MAJOR_LINE)
        string(REGEX MATCH "[0-9]+" CUVSLAM_VERSION_MAJOR "${CUVSLAM_VERSION_MAJOR_LINE}")
        set(CUVSLAM_VERSION "${CUVSLAM_VERSION_MAJOR}")
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

# Version checking - enforce exact version match for major version
if(CUVSLAM_FOUND AND CuVSLAM_FIND_VERSION AND CUVSLAM_VERSION)
    if(NOT CUVSLAM_VERSION VERSION_EQUAL CuVSLAM_FIND_VERSION)
        message(FATAL_ERROR 
            "cuVSLAM version mismatch: found version ${CUVSLAM_VERSION} but version ${CuVSLAM_FIND_VERSION} is required.\n"
            "cuVSLAM does not guarantee backward compatibility between major versions.\n"
            "Please install cuVSLAM version ${CuVSLAM_FIND_VERSION} or update CMakeLists.txt to request version ${CUVSLAM_VERSION}."
        )
    endif()
endif()

# Handle the QUIET and REQUIRED arguments
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(CuVSLAM
    FOUND_VAR CUVSLAM_FOUND
    REQUIRED_VARS CUVSLAM_LIBRARIES CUVSLAM_INCLUDE_DIRS
    VERSION_VAR CUVSLAM_VERSION
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
        if(CUVSLAM_VERSION)
            message(STATUS "Found cuVSLAM: ${CUVSLAM_LIBRARIES} (version: ${CUVSLAM_VERSION})")
        else()
            message(STATUS "Found cuVSLAM: ${CUVSLAM_LIBRARIES}")
        endif()
    endif()
else()
    # Fatal error if cuVSLAM is required but not found
    if(CUVSLAM_FIND_REQUIRED)
        message(FATAL_ERROR "Could not find cuVSLAM library")
    endif()
endif()

mark_as_advanced(CUVSLAM_INCLUDE_DIRS CUVSLAM_LIBRARY)
