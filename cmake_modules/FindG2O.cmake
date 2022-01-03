#Pre-requisites: Look for csparse
MESSAGE(STATUS "RTAB-Map's cmake g2o find module used for convenience (for older g2o versions)")
FIND_PATH(CSPARSE_INCLUDE_DIR NAMES cs.h PATH_SUFFIXES suitesparse csparse EXTERNAL/suitesparse EXTERNAL/csparse g2o/EXTERNAL/suitesparse g2o/EXTERNAL/csparse
  PATHS "C:\\Program Files\\g2o\\include\\EXTERNAL")
FIND_LIBRARY(CSPARSE_LIBRARY NAMES cxsparse g2o_ext_csparse 
  PATHS "C:\\Program Files\\g2o\\lib")

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(CSPARSE DEFAULT_MSG CSPARSE_INCLUDE_DIR CSPARSE_LIBRARY)

# chmold dependency
FIND_PATH(CHOLMOD_INCLUDE_DIR NAMES cholmod.h PATH_SUFFIXES suitesparse ufsparse
  PATHS $ENV{CHOLMODDIR} ${INCLUDE_INSTALL_DIR} )
FIND_LIBRARY(CHOLMOD_LIB cholmod)

# G2O: Find the header files

FIND_PATH(G2O_INCLUDE_DIR g2o/core/base_vertex.h 
  PATHS "C:\\Program Files\\g2o\\include")

FIND_FILE(G2O_CONFIG_FILE g2o/config.h 
  PATHS ${G2O_INCLUDE_DIR}
  NO_DEFAULT_PATH)

FIND_FILE(G2O_FACTORY_FILE g2o/core/factory.h 
  PATHS ${G2O_INCLUDE_DIR}
  NO_DEFAULT_PATH)

#ifdef G2O_NUMBER_FORMAT_STR
#define G2O_CPP11 // we assume that if G2O_NUMBER_FORMAT_STR is defined, this is the new g2o code with c++11 interface
#endif

# Macro to unify finding both the debug and release versions of the
# libraries; this is adapted from the rtabmap config

MACRO(FIND_G2O_LIBRARY MYLIBRARY MYLIBRARYNAME)

  FIND_LIBRARY("${MYLIBRARY}_DEBUG"
    NAMES "g2o_${MYLIBRARYNAME}_d"
	PATHS "C:\\Program Files\\g2o\\lib")
  
  FIND_LIBRARY(${MYLIBRARY}
    NAMES "g2o_${MYLIBRARYNAME}"
	PATHS "C:\\Program Files\\g2o\\lib")
  
  IF(${MYLIBRARY}_DEBUG AND ${MYLIBRARY})
    SET(${MYLIBRARY}
      debug ${${MYLIBRARY}_DEBUG}
      optimized ${${MYLIBRARY}}
    )
  ELSEIF(${MYLIBRARY}_DEBUG)
    SET(${MYLIBRARY} ${${MYLIBRARY}_DEBUG})
  ENDIF()  
  
ENDMACRO(FIND_G2O_LIBRARY LIBRARY LIBRARYNAME)

# Find the core elements
FIND_G2O_LIBRARY(G2O_STUFF_LIBRARY stuff)
FIND_G2O_LIBRARY(G2O_CORE_LIBRARY core)

# Find the CLI library
FIND_G2O_LIBRARY(G2O_CLI_LIBRARY cli)

# Find the pluggable solvers
FIND_G2O_LIBRARY(G2O_SOLVER_CHOLMOD solver_cholmod)
FIND_G2O_LIBRARY(G2O_SOLVER_CSPARSE solver_csparse)
FIND_G2O_LIBRARY(G2O_SOLVER_CSPARSE_EXTENSION csparse_extension)
FIND_G2O_LIBRARY(G2O_SOLVER_DENSE solver_dense)
FIND_G2O_LIBRARY(G2O_SOLVER_PCG solver_pcg)
FIND_G2O_LIBRARY(G2O_SOLVER_SLAM2D_LINEAR solver_slam2d_linear)
FIND_G2O_LIBRARY(G2O_SOLVER_STRUCTURE_ONLY solver_structure_only)
FIND_G2O_LIBRARY(G2O_SOLVER_EIGEN solver_eigen)

# Find the predefined types
FIND_G2O_LIBRARY(G2O_TYPES_DATA types_data)
FIND_G2O_LIBRARY(G2O_TYPES_ICP types_icp)
FIND_G2O_LIBRARY(G2O_TYPES_SBA types_sba)
FIND_G2O_LIBRARY(G2O_TYPES_SCLAM2D types_sclam2d)
FIND_G2O_LIBRARY(G2O_TYPES_SIM3 types_sim3)
FIND_G2O_LIBRARY(G2O_TYPES_SLAM2D types_slam2d)
FIND_G2O_LIBRARY(G2O_TYPES_SLAM3D types_slam3d)

# G2O solvers declared found if we found at least one solver
SET(G2O_SOLVERS_FOUND "NO")
IF(G2O_SOLVER_CHOLMOD OR G2O_SOLVER_CSPARSE OR G2O_SOLVER_DENSE OR G2O_SOLVER_PCG OR G2O_SOLVER_SLAM2D_LINEAR OR G2O_SOLVER_STRUCTURE_ONLY OR G2O_SOLVER_EIGEN)
  SET(G2O_SOLVERS_FOUND "YES")
ENDIF(G2O_SOLVER_CHOLMOD OR G2O_SOLVER_CSPARSE OR G2O_SOLVER_DENSE OR G2O_SOLVER_PCG OR G2O_SOLVER_SLAM2D_LINEAR OR G2O_SOLVER_STRUCTURE_ONLY OR G2O_SOLVER_EIGEN)

# G2O itself declared found if we found the core libraries and at least one solver
SET(G2O_FOUND "NO")
IF(G2O_STUFF_LIBRARY AND G2O_CORE_LIBRARY AND G2O_INCLUDE_DIR AND G2O_CONFIG_FILE AND G2O_FACTORY_FILE AND G2O_SOLVERS_FOUND)
  SET(G2O_INCLUDE_DIRS ${G2O_INCLUDE_DIR})
  SET(G2O_LIBRARIES 
	${G2O_CORE_LIBRARY}
	${G2O_TYPES_SLAM2D} 
	${G2O_TYPES_SLAM3D}
	${G2O_TYPES_SBA}
        ${G2O_STUFF_LIBRARY})
  
  IF(CSPARSE_FOUND)
     SET(G2O_INCLUDE_DIRS 
        ${G2O_INCLUDE_DIRS} 
        ${CSPARSE_INCLUDE_DIR})
     SET(G2O_LIBRARIES
        ${G2O_LIBRARIES} 
	${G2O_SOLVER_CSPARSE} 
	${G2O_SOLVER_CSPARSE_EXTENSION}
	${CSPARSE_LIBRARY})
  ENDIF(CSPARSE_FOUND)

  IF(G2O_SOLVER_CHOLMOD)
    SET(G2O_INCLUDE_DIRS 
        ${G2O_INCLUDE_DIRS} 
        ${CHOLMOD_INCLUDE_DIR})
    SET(G2O_LIBRARIES 
	  ${G2O_LIBRARIES}
	  ${G2O_SOLVER_CHOLMOD}
	  ${CHOLMOD_LIB})
  ENDIF(G2O_SOLVER_CHOLMOD)

  FILE(READ ${G2O_CONFIG_FILE} TMPTXT)
  STRING(FIND "${TMPTXT}" "G2O_NUMBER_FORMAT_STR" matchres)
  IF(${matchres} EQUAL -1)
    MESSAGE(STATUS "Old g2o version detected with c++03 interface (config file: ${G2O_CONFIG_FILE}).")
    SET(G2O_CPP11 0)
  ELSE()
    MESSAGE(WARNING "Latest g2o version detected with c++11 interface (config file: ${G2O_CONFIG_FILE}). Make sure g2o is built with \"-DBUILD_WITH_MARCH_NATIVE=OFF\" to avoid segmentation faults caused by Eigen.")
    FILE(READ ${G2O_FACTORY_FILE} TMPTXT)
    STRING(FIND "${TMPTXT}" "shared_ptr" matchres)
    IF(${matchres} EQUAL -1)
      MESSAGE(STATUS "Old g2o factory version detected without shared ptr (factory file: ${G2O_FACTORY_FILE}).")
      SET(G2O_CPP11 2)
    ELSE()
      MESSAGE(STATUS "Latest g2o factory version detected with shared ptr (factory file: ${G2O_FACTORY_FILE}).")
      SET(G2O_CPP11 1)
    ENDIF()
  ENDIF()

  SET(G2O_FOUND "YES")
ENDIF(G2O_STUFF_LIBRARY AND G2O_CORE_LIBRARY AND G2O_INCLUDE_DIR AND G2O_CONFIG_FILE AND G2O_FACTORY_FILE AND G2O_SOLVERS_FOUND)
