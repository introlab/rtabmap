
SET(SRC_FILES
    Rtabmap.cpp
    RtabmapThread.cpp
    
    Statistics.cpp
    
    Memory.cpp
    
    DBDriver.cpp
    DBDriverSqlite3.cpp
    DBReader.cpp
    
    Recovery.cpp
    
    Camera.cpp
    CameraThread.cpp
    CameraModel.cpp
    
    camera/CameraFreenect.cpp
    camera/CameraFreenect2.cpp
    camera/CameraImages.cpp
    camera/CameraK4W2.cpp
    camera/CameraK4A.cpp
    camera/CameraOpenni.cpp
    camera/CameraOpenNI2.cpp
    camera/CameraOpenNICV.cpp
    camera/CameraRealSense.cpp
    camera/CameraRealSense2.cpp
    camera/CameraRGBDImages.cpp
    camera/CameraStereoDC1394.cpp
    camera/CameraStereoFlyCapture2.cpp
    camera/CameraStereoImages.cpp
    camera/CameraStereoVideo.cpp
    camera/CameraStereoZed.cpp
    camera/CameraStereoZedOC.cpp
    camera/CameraStereoTara.cpp
    camera/CameraVideo.cpp
    camera/CameraMyntEye.cpp
    camera/CameraDepthAI.cpp
    
    EpipolarGeometry.cpp
    VisualWord.cpp
    VWDictionary.cpp
    BayesFilter.cpp
    Parameters.cpp
    Signature.cpp
    Features2d.cpp
    Transform.cpp
    GeodeticCoords.cpp
    
    util2d.cpp
    
    util3d.cpp
    util3d_filtering.cpp
    util3d_mapping.cpp
    util3d_transforms.cpp
    util3d_registration.cpp
    util3d_surface.cpp
    util3d_features.cpp
    util3d_correspondences.cpp
    util3d_motion_estimation.cpp
    
    SensorData.cpp
    Graph.cpp
    Compression.cpp
    Link.cpp
    LaserScan.cpp
    
    Optimizer.cpp
    optimizer/OptimizerTORO.cpp
    optimizer/OptimizerG2O.cpp
    optimizer/OptimizerGTSAM.cpp
    optimizer/OptimizerCVSBA.cpp
    optimizer/OptimizerCeres.cpp
    
    Registration.cpp
    RegistrationIcp.cpp
    RegistrationVis.cpp
    
    Odometry.cpp
    OdometryThread.cpp
    odometry/OdometryF2M.cpp
    odometry/OdometryMono.cpp
    odometry/OdometryF2F.cpp
    odometry/OdometryFovis.cpp
    odometry/OdometryViso2.cpp
    odometry/OdometryDVO.cpp
    odometry/OdometryOkvis.cpp
    odometry/OdometryORBSLAM.cpp
    odometry/OdometryLOAM.cpp
    odometry/OdometryFLOAM.cpp
    odometry/OdometryMSCKF.cpp
    odometry/OdometryVINS.cpp
    odometry/OdometryOpenVINS.cpp
    odometry/OdometryOpen3D.cpp
    
    IMU.cpp
    IMUThread.cpp
    IMUFilter.cpp
    imufilter/ComplementaryFilter.cpp
    
    Stereo.cpp
    StereoDense.cpp
    StereoCameraModel.cpp
    stereo/StereoBM.cpp
    stereo/StereoSGBM.cpp
    
    OccupancyGrid.cpp
    
    MarkerDetector.cpp
    
    GainCompensator.cpp

    rtflann/ext/lz4.c
    rtflann/ext/lz4hc.c
    FlannIndex.cpp
    
    #clams stuff
    clams/discrete_depth_distortion_model_helpers.cpp
    clams/discrete_depth_distortion_model.cpp
    clams/frame_projector.cpp
    clams/slam_calibrator.cpp
    
    opencv/ORBextractor.cc
    opencv/solvepnp.cpp
    opencv/five-point.cpp
)

IF(OpenCV_VERSION_MAJOR EQUAL 2)
SET(SRC_FILES
    ${SRC_FILES}
    opencv/Orb.cpp
)
ENDIF(OpenCV_VERSION_MAJOR EQUAL 2)

# to get includes in visual studio
IF(MSVC)
    FILE(GLOB HEADERS
       ../include/${PROJECT_PREFIX}/core/*.h
    )
    SET(SRC_FILES ${SRC_FILES} ${HEADERS})
ENDIF(MSVC)

SET(INCLUDE_DIRS
    ${CMAKE_CURRENT_BINARY_DIR}/../include
    ${PROJECT_SOURCE_DIR}/utilite/include
	${CMAKE_CURRENT_SOURCE_DIR}/../include
	${CMAKE_CURRENT_SOURCE_DIR}
	${CMAKE_CURRENT_BINARY_DIR}
    ${OpenCV_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
	${ZLIB_INCLUDE_DIRS}
)

SET(LIBRARIES
	${OpenCV_LIBS} 
	${PCL_LIBRARIES} 
	${ZLIB_LIBRARIES} 
)

IF(Sqlite3_FOUND)
    SET(INCLUDE_DIRS
		${INCLUDE_DIRS}
		${Sqlite3_INCLUDE_DIRS}
	)
	SET(LIBRARIES
		${LIBRARIES}
		${Sqlite3_LIBRARIES}
	)
ELSE()
	SET(SRC_FILES
		${SRC_FILES}
		sqlite3/sqlite3.c
	)
	 SET(INCLUDE_DIRS
	    ${CMAKE_CURRENT_SOURCE_DIR}/sqlite3
		${INCLUDE_DIRS}
	)
ENDIF()

IF(TORCH_FOUND)
	SET(LIBRARIES
		${LIBRARIES}
		${TORCH_LIBRARIES}
	)
	SET(SRC_FILES
		${SRC_FILES}
		superpoint_torch/SuperPoint.cc
	)
	 SET(INCLUDE_DIRS
	    ${TORCH_INCLUDE_DIRS}
	    ${CMAKE_CURRENT_SOURCE_DIR}/superpoint_torch
		${INCLUDE_DIRS}
	)
ENDIF(TORCH_FOUND)

IF(WITH_PYTHON AND Python3_FOUND)
	SET(LIBRARIES
		${LIBRARIES}
		Python3::Python
		Python3::NumPy
	)
	SET(SRC_FILES
		${SRC_FILES}
		python/PythonInterface.cpp
		python/PyMatcher.cpp
		python/PyDetector.cpp
	)
	 SET(INCLUDE_DIRS
	    ${TORCH_INCLUDE_DIRS}
	    ${CMAKE_CURRENT_SOURCE_DIR}/python
		${INCLUDE_DIRS}
	)
ENDIF(WITH_PYTHON AND Python3_FOUND)



IF(Freenect_FOUND)
	IF(Freenect_DASH_INCLUDES)
	   ADD_DEFINITIONS("-DFREENECT_DASH_INCLUDES")
	ENDIF(Freenect_DASH_INCLUDES)
	SET(INCLUDE_DIRS
		${INCLUDE_DIRS}
		${Freenect_INCLUDE_DIRS}
	)
	SET(LIBRARIES
		${LIBRARIES}
		${Freenect_LIBRARIES}
	)
ENDIF(Freenect_FOUND)

IF(OpenNI2_FOUND)
	SET(INCLUDE_DIRS
		${INCLUDE_DIRS}
		${OpenNI2_INCLUDE_DIRS}
	)
	SET(LIBRARIES
		${LIBRARIES}
		${OpenNI2_LIBRARIES}
	)
ENDIF(OpenNI2_FOUND)

IF(freenect2_FOUND)
	SET(INCLUDE_DIRS
		${INCLUDE_DIRS}
		${freenect2_INCLUDE_DIRS}
	)
	SET(LIBRARIES
		${LIBRARIES}
		${freenect2_LIBRARIES}
	)
ENDIF(freenect2_FOUND)

IF(KinectSDK2_FOUND)
	SET(INCLUDE_DIRS
		${INCLUDE_DIRS}
		${KinectSDK2_INCLUDE_DIRS}
	)
	SET(LIBRARIES
		${LIBRARIES}
		${KinectSDK2_LIBRARIES}
	)
ENDIF(KinectSDK2_FOUND)

IF(k4a_FOUND)
	SET(INCLUDE_DIRS
		${INCLUDE_DIRS}
		${k4a_INCLUDE_DIRS}
	)
	IF(WIN32)
	    SET(LIBRARIES
			${LIBRARIES}
			${k4a_LIBRARIES}
		)
	ELSE()
		SET(LIBRARIES
			${LIBRARIES}
			k4a::k4a 
			k4a::k4arecord
		)
	ENDIF()
ENDIF(k4a_FOUND)

IF(RealSense_FOUND)
	SET(INCLUDE_DIRS
		${INCLUDE_DIRS}
		${RealSense_INCLUDE_DIRS}
	)
	SET(LIBRARIES
		${LIBRARIES}
		${RealSense_LIBRARIES}
	)
ENDIF(RealSense_FOUND)

IF(realsense2_FOUND)
	SET(INCLUDE_DIRS
		${INCLUDE_DIRS}
		${realsense2_INCLUDE_DIRS}
	)
	IF(WIN32)
	    SET(LIBRARIES
			${LIBRARIES}
			${RealSense2_LIBRARIES}
		)
	ELSEIF(APPLE)
	    SET(LIBRARIES
			${LIBRARIES}
			${realsense2_LIBRARIES}
		)
	ELSE()
		SET(LIBRARIES
			${LIBRARIES}
			realsense2::realsense2 # target
		)
	ENDIF()
ENDIF(realsense2_FOUND)

IF(DC1394_FOUND)
	SET(INCLUDE_DIRS
		${INCLUDE_DIRS}
		${DC1394_INCLUDE_DIRS}
	)
	SET(LIBRARIES
		${LIBRARIES}
		${DC1394_LIBRARIES}
	)
ENDIF(DC1394_FOUND)

IF(FlyCapture2_FOUND)
	SET(INCLUDE_DIRS
		${INCLUDE_DIRS}
		${FlyCapture2_INCLUDE_DIRS}
	)
	SET(LIBRARIES
		${LIBRARIES}
		${FlyCapture2_LIBRARIES}
	)
ENDIF(FlyCapture2_FOUND)

IF(mynteye_FOUND)
	SET(LIBRARIES
		${LIBRARIES}
		mynteye # target
	)
ENDIF(mynteye_FOUND)

IF(depthai_FOUND)
	SET(LIBRARIES
		${LIBRARIES}
		depthai::core
		depthai::opencv
	)
ENDIF(depthai_FOUND)

IF(TARGET OpenMP::OpenMP_CXX)
	SET(LIBRARIES
		${LIBRARIES}
		OpenMP::OpenMP_CXX
	)
ENDIF(TARGET OpenMP::OpenMP_CXX)

IF(WITH_TORO)
	SET(SRC_FILES
    	${SRC_FILES}
		optimizer/toro3d/posegraph3.cpp
		optimizer/toro3d/treeoptimizer3_iteration.cpp
		optimizer/toro3d/treeoptimizer3.cpp
	
		optimizer/toro3d/posegraph2.cpp
		optimizer/toro3d/treeoptimizer2.cpp
	)
ENDIF(WITH_TORO)

IF(G2O_FOUND)
	IF(g2o_FOUND)
	  SET(LIBRARIES ${LIBRARIES}
	     g2o::core
	     g2o::solver_eigen
	     g2o::solver_pcg
	     g2o::types_slam2d
	     g2o::types_slam3d
	     g2o::types_sba)
	  IF(TARGET g2o::solver_csparse)
	    SET(LIBRARIES ${LIBRARIES}
          g2o::solver_csparse
          g2o::csparse_extension)
      ENDIF(TARGET g2o::solver_csparse)
      IF(TARGET g2o::solver_cholmod)
        SET(LIBRARIES ${LIBRARIES} 
          g2o::solver_cholmod)
      ENDIF(TARGET g2o::solver_cholmod)
	ELSE()
	  SET(INCLUDE_DIRS 
		${INCLUDE_DIRS} 
		${G2O_INCLUDE_DIRS}
	  )
	  SET(LIBRARIES
		${LIBRARIES}
		${G2O_LIBRARIES}
	  )
	ENDIF()
	IF(WITH_VERTIGO)
		SET(SRC_FILES ${SRC_FILES}
			optimizer/vertigo/g2o/edge_se2Switchable.cpp
			optimizer/vertigo/g2o/edge_se3Switchable.cpp
			optimizer/vertigo/g2o/edge_switchPrior.cpp
			optimizer/vertigo/g2o/types_g2o_robust.cpp
			optimizer/vertigo/g2o/vertex_switchLinear.cpp
		)
	ENDIF(WITH_VERTIGO)
ENDIF(G2O_FOUND)

IF(cvsba_FOUND)
	SET(INCLUDE_DIRS 
		${INCLUDE_DIRS} 
		${cvsba_INCLUDE_DIRS}
	)
	SET(LIBRARIES
		${LIBRARIES}
		${cvsba_LIBS}
	)
ENDIF(cvsba_FOUND)

IF(CERES_FOUND)
	SET(INCLUDE_DIRS
	  ${INCLUDE_DIRS}
	  ${CERES_INCLUDE_DIRS}
	)
	SET(LIBRARIES
	  ${LIBRARIES}
	  ${CERES_LIBRARIES}
	)
ENDIF(CERES_FOUND)

IF(libpointmatcher_FOUND)
	SET(INCLUDE_DIRS 
		${INCLUDE_DIRS} 
		${libpointmatcher_INCLUDE_DIRS}
	)
	SET(LIBRARIES
		${LIBRARIES}
		${libpointmatcher_LIBRARIES}
	)
ENDIF(libpointmatcher_FOUND)

IF(CCCoreLib_FOUND)
	SET(LIBRARIES
		${LIBRARIES}
		CCCoreLib::CCCoreLib
	)
ENDIF(CCCoreLib_FOUND)

IF(Open3D_FOUND)
    SET(LIBRARIES
        ${LIBRARIES}
        Open3D::Open3D
    )
ENDIF(Open3D_FOUND)

IF(FastCV_FOUND)
	SET(INCLUDE_DIRS 
		${INCLUDE_DIRS} 
		${FastCV_INCLUDE_DIRS}
	)
	SET(LIBRARIES
		${LIBRARIES}
		${FastCV_LIBRARIES}
	)
ENDIF(FastCV_FOUND)

IF(opengv_FOUND)
    SET(LIBRARIES
        ${LIBRARIES}
        opengv
    )
ENDIF(opengv_FOUND)

IF(PDAL_FOUND)
	SET(INCLUDE_DIRS
		${INCLUDE_DIRS}
		${PDAL_INCLUDE_DIRS}
	)
	SET(LIBRARIES
		${LIBRARIES}
		${PDAL_LIBRARIES}
	)
	SET(SRC_FILES
		${SRC_FILES}
		PDALWriter.cpp
	)
	IF(PDAL_VERSION VERSION_LESS "1.7")
	    add_definitions("-DRTABMAP_PDAL_16")
	ENDIF(PDAL_VERSION VERSION_LESS "1.7")
ENDIF(PDAL_FOUND)


IF(loam_velodyne_FOUND)
	SET(INCLUDE_DIRS 
		${INCLUDE_DIRS} 
		${loam_velodyne_INCLUDE_DIRS}
	)
	SET(LIBRARIES
		${LIBRARIES}
		${loam_velodyne_LIBRARIES}
	)
ENDIF(loam_velodyne_FOUND)

IF(floam_FOUND)
    SET(INCLUDE_DIRS 
        ${INCLUDE_DIRS} 
        ${floam_INCLUDE_DIRS}
    )
    SET(LIBRARIES
        ${LIBRARIES}
        ${floam_LIBRARIES}
    )
ENDIF(floam_FOUND)

IF(ZED_FOUND)
	SET(INCLUDE_DIRS
		${INCLUDE_DIRS}
		${ZED_INCLUDE_DIRS}
	)
	SET(LIBRARIES
		${LIBRARIES}
		${ZED_LIBRARIES}
	)
	IF(CUDA_FOUND)
		SET(INCLUDE_DIRS
			${INCLUDE_DIRS}
			${CUDA_INCLUDE_DIRS}
		)
		SET(LIBRARIES
			${LIBRARIES}
			${CUDA_LIBRARIES}
		)
	ENDIF(CUDA_FOUND)
ENDIF(ZED_FOUND)

IF(ZEDOC_FOUND)
	SET(INCLUDE_DIRS
		${INCLUDE_DIRS}
		${ZEDOC_INCLUDE_DIRS}
		${HIDAPI_INCLUDE_DIRS}
	)
	SET(LIBRARIES
		${LIBRARIES}
		${ZEDOC_LIBRARIES}
		${HIDAPI_LIBRARIES}
	)
ENDIF(ZEDOC_FOUND)

IF(octomap_FOUND)
    SET(INCLUDE_DIRS
		${INCLUDE_DIRS}
		${OCTOMAP_INCLUDE_DIRS}
	)
	SET(LIBRARIES
		${LIBRARIES}
		${OCTOMAP_LIBRARIES}
	)
	SET(SRC_FILES
    	${SRC_FILES}
		OctoMap.cpp
	)
ENDIF(octomap_FOUND)

IF(AliceVision_FOUND)
  SET(LIBRARIES
    ${LIBRARIES}
    aliceVision_mesh 
    aliceVision_sfmDataIO)
ENDIF(AliceVision_FOUND)

IF(libfovis_FOUND)
    SET(INCLUDE_DIRS
		${INCLUDE_DIRS}
		${libfovis_INCLUDE_DIRS}
	)
	SET(LIBRARIES
		${LIBRARIES}
		${libfovis_LIBRARIES}
	)
ENDIF(libfovis_FOUND)

IF(libviso2_FOUND)
    SET(INCLUDE_DIRS
		${INCLUDE_DIRS}
		${libviso2_INCLUDE_DIRS}
	)
	SET(LIBRARIES
		${LIBRARIES}
		${libviso2_LIBRARIES}
	)
ENDIF(libviso2_FOUND)

IF(dvo_core_FOUND)
    SET(INCLUDE_DIRS
		${INCLUDE_DIRS}
		${dvo_core_INCLUDE_DIRS}
	)
	SET(LIBRARIES
		${LIBRARIES}
		${dvo_core_LIBRARIES}
	)
ENDIF(dvo_core_FOUND)

IF(okvis_FOUND)
    SET(INCLUDE_DIRS
		${OKVIS_INCLUDE_DIRS}
		${BRISK_INCLUDE_DIRS}
		${OPENGV_INCLUDE_DIRS}
		${CERES_INCLUDE_DIRS}
		${INCLUDE_DIRS}
	)
	SET(LIBRARIES
		${OKVIS_LIBRARIES}
		${BRISK_LIBRARIES}
		${OPENGV_LIBRARIES}
		${CERES_LIBRARIES}
		${LIBRARIES}
	)
ENDIF(okvis_FOUND)

IF(msckf_vio_FOUND)
    SET(INCLUDE_DIRS
		${msckf_vio_INCLUDE_DIRS}
		${INCLUDE_DIRS}
	)
	SET(LIBRARIES
		${msckf_vio_LIBRARIES}
		${LIBRARIES}
	)
ENDIF(msckf_vio_FOUND)

IF(vins_FOUND)
    SET(INCLUDE_DIRS
		${vins_INCLUDE_DIRS}
		${INCLUDE_DIRS}
	)
	SET(LIBRARIES
		${vins_LIBRARIES}
		${LIBRARIES}
	)
ENDIF(vins_FOUND)

IF(ov_msckf_FOUND)
    SET(INCLUDE_DIRS
		${ov_msckf_INCLUDE_DIRS}
		${INCLUDE_DIRS}
	)
	SET(LIBRARIES
		${ov_msckf_LIBRARIES}
		${LIBRARIES}
	)
ENDIF(ov_msckf_FOUND)

IF(ORB_SLAM_FOUND)
    SET(INCLUDE_DIRS
        ${ORB_SLAM_INCLUDE_DIRS} #before so that g2o includes are taken from ORB_SLAM directory before the official g2o one
		${INCLUDE_DIRS}
	)
	SET(LIBRARIES
	    ${ORB_SLAM_LIBRARIES}
		${LIBRARIES}
	)
ENDIF(ORB_SLAM_FOUND)

IF(GTSAM_FOUND)
	# Make sure GTSAM is built with system Eigen, not the included one in its package
	IF(GTSAM_INCLUDE_DIR)
		SET(INCLUDE_DIRS 
			${INCLUDE_DIRS} 
			${GTSAM_INCLUDE_DIR} 
		)
	ELSE()
		SET(INCLUDE_DIRS 
			${INCLUDE_DIRS} 
			${GTSAM_INCLUDE_DIRS}
		)
	ENDIF()
	SET(SRC_FILES
    	${SRC_FILES}
		optimizer/gtsam/GravityFactor.cpp
	)
	IF(WIN32)
		# GTSAM should be built in STATIC on Windows to avoid "error C2338: THIS_METHOD_IS_ONLY_FOR_1x1_EXPRESSIONS" when building GTSAM
		add_definitions("-DGTSAM_IMPORT_STATIC") 
	ENDIF(WIN32)
	SET(LIBRARIES
		${LIBRARIES}
		gtsam # Windows: Place static libs at the end
	)
ENDIF(GTSAM_FOUND)

IF(WITH_MADGWICK)
	SET(SRC_FILES
		${SRC_FILES}
		imufilter/MadgwickFilter.cpp
	)
ENDIF(WITH_MADGWICK)

####################################
# Generate resources files
####################################
CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/resources/DatabaseSchema.sql.in ${CMAKE_CURRENT_SOURCE_DIR}/resources/DatabaseSchema.sql)

SET(RESOURCES
	${CMAKE_CURRENT_SOURCE_DIR}/resources/DatabaseSchema.sql
	${CMAKE_CURRENT_SOURCE_DIR}/resources/backward_compatibility/DatabaseSchema_0_18_3.sql
	${CMAKE_CURRENT_SOURCE_DIR}/resources/backward_compatibility/DatabaseSchema_0_18_0.sql
	${CMAKE_CURRENT_SOURCE_DIR}/resources/backward_compatibility/DatabaseSchema_0_17_0.sql
	${CMAKE_CURRENT_SOURCE_DIR}/resources/backward_compatibility/DatabaseSchema_0_16_2.sql
	${CMAKE_CURRENT_SOURCE_DIR}/resources/backward_compatibility/DatabaseSchema_0_16_1.sql
	${CMAKE_CURRENT_SOURCE_DIR}/resources/backward_compatibility/DatabaseSchema_0_16_0.sql
)

foreach(arg ${RESOURCES})
   get_filename_component(filename ${arg} NAME)
   string(REPLACE "." "_" output ${filename})
   set(RESOURCES_HEADERS "${RESOURCES_HEADERS}" "${CMAKE_CURRENT_BINARY_DIR}/${output}.h")
endforeach(arg ${RESOURCES})

#MESSAGE(STATUS "RESOURCES = ${RESOURCES}")
#MESSAGE(STATUS "RESOURCES_HEADERS = ${RESOURCES_HEADERS}")

ADD_CUSTOM_COMMAND(
	OUTPUT ${RESOURCES_HEADERS}
	COMMAND res_tool -n rtabmap -p ${CMAKE_CURRENT_BINARY_DIR} ${RESOURCES}
	COMMENT "[Creating resources]"
	DEPENDS ${RESOURCES}
)

####################################
# Generate resources files END
####################################

add_definitions(${PCL_DEFINITIONS})


# Make sure the compiler can find include files from our library.
INCLUDE_DIRECTORIES(${INCLUDE_DIRS})

# Add binary that is built from the source file "main.cpp".
# The extension is automatically found.
ADD_LIBRARY(rtabmap_core ${SRC_FILES} ${RESOURCES_HEADERS})
TARGET_LINK_LIBRARIES(rtabmap_core rtabmap_utilite ${LIBRARIES})

SET_TARGET_PROPERTIES(
     rtabmap_core
   PROPERTIES
     VERSION ${RTABMAP_VERSION}
     SOVERSION ${RTABMAP_MAJOR_VERSION}.${RTABMAP_MINOR_VERSION}
)

INSTALL(TARGETS rtabmap_core
        RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT runtime
        LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT devel
        ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT devel)
		
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../include/ 
		DESTINATION "${INSTALL_INCLUDE_DIR}"
		COMPONENT devel 
		FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp" 
		PATTERN ".svn" EXCLUDE)
		
# For generated Version.h
install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/../include/ 
        DESTINATION "${INSTALL_INCLUDE_DIR}"
        COMPONENT devel 
        FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp" 
        PATTERN ".svn" EXCLUDE)

