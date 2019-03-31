
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
	camera/CameraStereoTara.cpp
    camera/CameraVideo.cpp
    
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
	odometry/OdometryORBSLAM2.cpp
	odometry/OdometryLOAM.cpp
	odometry/OdometryMSCKF.cpp
	odometry/OdometryVINS.cpp
	
	IMUThread.cpp
	
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
)

IF(OpenCV_VERSION_MAJOR EQUAL 2)
SET(SRC_FILES
    ${SRC_FILES}
    opencv/Orb.cpp
)
ENDIF(OpenCV_VERSION_MAJOR EQUAL 2)
SET(SRC_FILES
    ${SRC_FILES}
    opencv/solvepnp.cpp
)

# to get includes in visual studio
IF(MSVC)
    FILE(GLOB HEADERS
       ../include/${PROJECT_PREFIX}/core/*.h
    )
    SET(SRC_FILES ${SRC_FILES} ${HEADERS})
ENDIF(MSVC)

SET(INCLUDE_DIRS
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
	ELSE()
		SET(LIBRARIES
			${LIBRARIES}
			realsense2
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
	SET(INCLUDE_DIRS 
		${INCLUDE_DIRS} 
		${G2O_INCLUDE_DIRS}
	)
	SET(LIBRARIES
		${LIBRARIES}
		${G2O_LIBRARIES}
	)
	SET(SRC_FILES
    		${SRC_FILES}
		optimizer/g2o/edge_se3_xyzprior.cpp
	)
	IF(WITH_VERTIGO)
		SET(SRC_FILES
    			${SRC_FILES}
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

IF(OCTOMAP_FOUND)
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
ENDIF(OCTOMAP_FOUND)

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

IF(ORB_SLAM2_FOUND)
    SET(INCLUDE_DIRS
        ${ORB_SLAM2_INCLUDE_DIRS} #before so that g2o includes are taken from ORB_SLAM2 directory before the official g2o one
		${INCLUDE_DIRS}
	)
	SET(LIBRARIES
	    ${ORB_SLAM2_LIBRARIES}
		${LIBRARIES}
	)
ENDIF(ORB_SLAM2_FOUND)

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

####################################
# Generate resources files
####################################
CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/resources/DatabaseSchema.sql.in ${CMAKE_CURRENT_SOURCE_DIR}/resources/DatabaseSchema.sql)

SET(R
	${CMAKE_CURRENT_SOURCE_DIR}/resources/DatabaseSchema.sql
)

#replace semicolons by spaces
foreach(arg ${R})
   set(RESOURCES "${RESOURCES}" "${arg}")
endforeach(arg ${R})

SET(RESOURCES_HEADERS 
	${CMAKE_CURRENT_BINARY_DIR}/DatabaseSchema_sql.h 
)

IF(ANDROID)

   IF(NOT RTABMAP_RES_TOOL)
      find_host_program(RTABMAP_RES_TOOL rtabmap-res_tool PATHS ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
      IF(NOT RTABMAP_RES_TOOL)
         MESSAGE( FATAL_ERROR "RTABMAP_RES_TOOL is not defined (it is the path to \"rtabmap-res_tool\" application created by a non-Android build)." )
      ENDIF(NOT RTABMAP_RES_TOOL)
   ENDIF(NOT RTABMAP_RES_TOOL)

   ADD_CUSTOM_COMMAND(
      OUTPUT ${RESOURCES_HEADERS}
      COMMAND ${RTABMAP_RES_TOOL} -n rtabmap -p ${CMAKE_CURRENT_BINARY_DIR} ${RESOURCES}
      COMMENT "[Creating resources]"
      DEPENDS ${R}
   )
ELSE()
   ADD_CUSTOM_COMMAND(
      OUTPUT ${RESOURCES_HEADERS}
      COMMAND ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/rtabmap-res_tool -n rtabmap -p ${CMAKE_CURRENT_BINARY_DIR} ${RESOURCES}
      COMMENT "[Creating resources]"
      DEPENDS ${R} res_tool
   )
ENDIF()

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

