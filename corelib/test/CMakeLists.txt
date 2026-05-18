
add_definitions("-DRTABMAP_TEST_DATA_ROOT=\"${TEST_DATA_ROOT}\"")

#util2d.h
add_executable(test_util2d test_util2d.cpp)
target_link_libraries(test_util2d gtest_main rtabmap_core)
add_test(NAME test_util2d COMMAND test_util2d)

#util3d.h
add_executable(test_util3d test_util3d.cpp)
target_link_libraries(test_util3d gtest_main rtabmap_core)
add_test(NAME test_util3d COMMAND test_util3d)

#util3d_transforms.h
add_executable(test_util3d_transforms test_util3d_transforms.cpp)
target_link_libraries(test_util3d_transforms gtest_main rtabmap_core)
add_test(NAME test_util3d_transforms COMMAND test_util3d_transforms)

#util3d_filtering.h
add_executable(test_util3d_filtering test_util3d_filtering.cpp)
target_link_libraries(test_util3d_filtering gtest_main rtabmap_core)
add_test(NAME test_util3d_filtering COMMAND test_util3d_filtering)

#util3d_registration.h
add_executable(test_util3d_registration test_util3d_registration.cpp)
target_link_libraries(test_util3d_registration gtest_main rtabmap_core)
add_test(NAME test_util3d_registration COMMAND test_util3d_registration)

#util3d_features.h
add_executable(test_util3d_features test_util3d_features.cpp)
target_link_libraries(test_util3d_features gtest_main rtabmap_core)
add_test(NAME test_util3d_features COMMAND test_util3d_features)

#util3d_correspondences.h
add_executable(test_util3d_correspondences test_util3d_correspondences.cpp)
target_link_libraries(test_util3d_correspondences gtest_main rtabmap_core)
add_test(NAME test_util3d_correspondences COMMAND test_util3d_correspondences)

#util3d_mapping.h
add_executable(test_util3d_mapping test_util3d_mapping.cpp)
target_link_libraries(test_util3d_mapping gtest_main rtabmap_core)
add_test(NAME test_util3d_mapping COMMAND test_util3d_mapping)

#util3d_motion_estimation.h
add_executable(test_util3d_motion_estimation test_util3d_motion_estimation.cpp)
target_link_libraries(test_util3d_motion_estimation gtest_main rtabmap_core)
add_test(NAME test_util3d_motion_estimation COMMAND test_util3d_motion_estimation)

#util3d_surface.h
add_executable(test_util3d_surface test_util3d_surface.cpp)
target_link_libraries(test_util3d_surface gtest_main rtabmap_core)
add_test(NAME test_util3d_surface COMMAND test_util3d_surface)

#VisualWord.h
add_executable(test_visualword test_visualword.cpp)
target_link_libraries(test_visualword gtest_main rtabmap_core)
add_test(NAME test_visualword COMMAND test_visualword)

#VWDictionary.h
add_executable(test_vwdictionary test_vwdictionary.cpp)
target_link_libraries(test_vwdictionary gtest_main rtabmap_core)
add_test(NAME test_vwdictionary COMMAND test_vwdictionary)

#Transform.h
add_executable(test_transform test_transform.cpp)
target_link_libraries(test_transform gtest_main rtabmap_core)
add_test(NAME test_transform COMMAND test_transform)

#StereoDense.h (tests both BM and SGBM strategies)
add_executable(test_stereo_dense test_stereo_dense.cpp)
target_link_libraries(test_stereo_dense gtest_main rtabmap_core)
add_test(NAME test_stereo_dense COMMAND test_stereo_dense)

#Stereo.h (tests both BlockMatching and OpticalFlow strategies)
add_executable(test_stereo test_stereo.cpp)
target_link_libraries(test_stereo gtest_main rtabmap_core)
add_test(NAME test_stereo COMMAND test_stereo)

#CameraModel.h
add_executable(test_cameramodel test_cameramodel.cpp)
target_link_libraries(test_cameramodel gtest_main rtabmap_core)
add_test(NAME test_cameramodel COMMAND test_cameramodel)

#StereoCameraModel.h
add_executable(test_stereocameramodel test_stereocameramodel.cpp)
target_link_libraries(test_stereocameramodel gtest_main rtabmap_core)
add_test(NAME test_stereocameramodel COMMAND test_stereocameramodel)

#Statistics.h
add_executable(test_statistics test_statistics.cpp)
target_link_libraries(test_statistics gtest_main rtabmap_core)
add_test(NAME test_statistics COMMAND test_statistics)

#BayesFilter.h
add_executable(test_bayesfilter test_bayesfilter.cpp)
target_link_libraries(test_bayesfilter gtest_main rtabmap_core)
add_test(NAME test_bayesfilter COMMAND test_bayesfilter)

#Link.h
add_executable(test_link test_link.cpp)
target_link_libraries(test_link gtest_main rtabmap_core)
add_test(NAME test_link COMMAND test_link)

#GPS.h
add_executable(test_gps test_gps.cpp)
target_link_libraries(test_gps gtest_main rtabmap_core)
add_test(NAME test_gps COMMAND test_gps)

#IMU.h
add_executable(test_imu test_imu.cpp)
target_link_libraries(test_imu gtest_main rtabmap_core)
add_test(NAME test_imu COMMAND test_imu)

#IMUFilter.h
add_executable(test_imufilter test_imufilter.cpp)
target_link_libraries(test_imufilter gtest_main rtabmap_core)
add_test(NAME test_imufilter COMMAND test_imufilter)

#IMUThread.h
add_executable(test_imuthread test_imuthread.cpp)
target_link_libraries(test_imuthread gtest_main rtabmap_core)
add_test(NAME test_imuthread COMMAND test_imuthread)

#Landmark.h
add_executable(test_landmark test_landmark.cpp)
target_link_libraries(test_landmark gtest_main rtabmap_core)
add_test(NAME test_landmark COMMAND test_landmark)

#LocalGrid.h
add_executable(test_localgrid test_localgrid.cpp)
target_link_libraries(test_localgrid gtest_main rtabmap_core)
add_test(NAME test_localgrid COMMAND test_localgrid)

#GlobalMap.h
add_executable(test_globalmap test_globalmap.cpp)
target_link_libraries(test_globalmap gtest_main rtabmap_core)
add_test(NAME test_globalmap COMMAND test_globalmap)

#LocalGridMaker.h
add_executable(test_localgridmaker test_localgridmaker.cpp)
target_link_libraries(test_localgridmaker gtest_main rtabmap_core)
add_test(NAME test_localgridmaker COMMAND test_localgridmaker)

#LASWriter.h (optional libLAS)
IF(libLAS_FOUND)
	add_executable(test_laswriter test_laswriter.cpp)
	target_link_libraries(test_laswriter gtest_main rtabmap_core ${libLAS_LIBRARIES})
	target_include_directories(test_laswriter PRIVATE ${libLAS_INCLUDE_DIRS})
	add_test(NAME test_laswriter COMMAND test_laswriter)
ENDIF(libLAS_FOUND)

#Graph.h
add_executable(test_graph test_graph.cpp)
target_link_libraries(test_graph gtest_main rtabmap_core)
add_test(NAME test_graph COMMAND test_graph)

#GeodeticCoords.h
add_executable(test_geodeticcoords test_geodeticcoords.cpp)
target_link_libraries(test_geodeticcoords gtest_main rtabmap_core)
add_test(NAME test_geodeticcoords COMMAND test_geodeticcoords)

#Compression.h
add_executable(test_compression test_compression.cpp)
target_link_libraries(test_compression gtest_main rtabmap_core)
add_test(NAME test_compression COMMAND test_compression)

#Odometry.h
add_executable(test_odometry test_odometry.cpp)
target_link_libraries(test_odometry gtest_main rtabmap_core)
add_test(NAME test_odometry COMMAND test_odometry)

#DBDriver.h
add_executable(test_dbdriver test_dbdriver.cpp)
target_link_libraries(test_dbdriver gtest_main rtabmap_core)
add_test(NAME test_dbdriver COMMAND test_dbdriver)

#DBDriverSqlite3.h
add_executable(test_dbdriversqlite3 test_dbdriversqlite3.cpp)
target_link_libraries(test_dbdriversqlite3 gtest_main rtabmap_core)
add_test(NAME test_dbdriversqlite3 COMMAND test_dbdriversqlite3)

#Signature.h
add_executable(test_signature test_signature.cpp)
target_link_libraries(test_signature gtest_main rtabmap_core)
add_test(NAME test_signature COMMAND test_signature)

#SensorEvent.h
add_executable(test_sensorevent test_sensorevent.cpp)
target_link_libraries(test_sensorevent gtest_main rtabmap_core)
add_test(NAME test_sensorevent COMMAND test_sensorevent)

#SensorData.h
add_executable(test_sensordata test_sensordata.cpp)
target_link_libraries(test_sensordata gtest_main rtabmap_core)
add_test(NAME test_sensordata COMMAND test_sensordata)

#SensorCapture.h
add_executable(test_sensorcapture test_sensorcapture.cpp)
target_link_libraries(test_sensorcapture gtest_main rtabmap_core)
add_test(NAME test_sensorcapture COMMAND test_sensorcapture)

#SensorCaptureThread.h
add_executable(test_sensorcapturethread test_sensorcapturethread.cpp)
target_link_libraries(test_sensorcapturethread gtest_main rtabmap_core)
add_test(NAME test_sensorcapturethread COMMAND test_sensorcapturethread)
