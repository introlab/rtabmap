
ADD_EXECUTABLE(lidarCameraCalibration
  main.cpp
  CalibrationProblem.cpp
  CorrectionSolver.cpp
  PatternSearchSolver.cpp
  DownhillSimplexSolver.cpp
  G2oSolver.cpp
  GtsamSolver.cpp)
  
TARGET_LINK_LIBRARIES(lidarCameraCalibration rtabmap_core)

# For the g2o solver: rtabmap_core links g2o privately.
IF(G2O_FOUND)
  IF(g2o_FOUND)
    TARGET_LINK_LIBRARIES(lidarCameraCalibration g2o::core g2o::solver_eigen)
  ELSE()
    TARGET_INCLUDE_DIRECTORIES(lidarCameraCalibration PRIVATE ${G2O_INCLUDE_DIRS})
    TARGET_LINK_LIBRARIES(lidarCameraCalibration ${G2O_LIBRARIES})
  ENDIF()
ENDIF(G2O_FOUND)

# For the GTSAM solver: rtabmap_core links GTSAM privately.
IF(GTSAM_FOUND)
  TARGET_LINK_LIBRARIES(lidarCameraCalibration gtsam)
ENDIF(GTSAM_FOUND)

SET_TARGET_PROPERTIES( lidarCameraCalibration 
  PROPERTIES OUTPUT_NAME ${PROJECT_PREFIX}-lidarCameraCalibration)

INSTALL(TARGETS lidarCameraCalibration
		RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT runtime
		BUNDLE DESTINATION "${CMAKE_BUNDLE_LOCATION}" COMPONENT runtime)
