
add_definitions("-DRTABMAP_TEST_DATA_ROOT=\"${TEST_DATA_ROOT}\"")

# ---------------------------------------------------------------------------
# One executable for the corelib unit tests, source files kept split.
#
# Every test binary that links rtabmap_core pulls in ~344 shared libraries
# (PCL, VTK, OpenCV, Qt, g2o, GTSAM, Ceres, octomap, the camera SDKs...), and
# each ctest entry is a separate process that pays that load cost again. The
# load dominates: test_util3d_transforms does ~10 ms of actual work, yet it
# reported 12.7 s on the macOS runners, where dyld has no shared cache for
# non-system dylibs and validates each one's signature per process. Roughly
# 590 s of a 2150 s macOS run was process startup.
#
# gtest registers tests through static initializers and gtest_main supplies
# main(), so the sources combine into one target with no source changes.
# ---------------------------------------------------------------------------
set(corelib_test_sources
	test_util2d.cpp                     #util2d.h
	test_util3d.cpp                     #util3d.h
	test_util3d_transforms.cpp          #util3d_transforms.h
	test_util3d_filtering.cpp           #util3d_filtering.h
	test_util3d_registration.cpp        #util3d_registration.h
	test_util3d_features.cpp            #util3d_features.h
	test_util3d_correspondences.cpp     #util3d_correspondences.h
	test_util3d_mapping.cpp             #util3d_mapping.h
	test_util3d_motion_estimation.cpp   #util3d_motion_estimation.h
	test_util3d_surface.cpp             #util3d_surface.h
	test_visualword.cpp                 #VisualWord.h
	test_vwdictionary.cpp               #VWDictionary.h
	test_flann_index.cpp                #FlannIndex.h
	test_transform.cpp                  #Transform.h
	test_stereo_dense.cpp               #StereoDense.h (BM and SGBM strategies)
	test_stereo.cpp                     #Stereo.h (BlockMatching and OpticalFlow)
	test_cameramodel.cpp                #CameraModel.h
	test_stereocameramodel.cpp          #StereoCameraModel.h
	test_statistics.cpp                 #Statistics.h
	test_bayesfilter.cpp                #BayesFilter.h
	test_memory.cpp                     #Memory.h
	test_markerdetector.cpp             #MarkerDetector.h
	test_rtabmap.cpp                    #Rtabmap.h
	test_link.cpp                       #Link.h
	test_optimizer.cpp                  #Optimizer.h
	test_gps.cpp                        #GPS.h
	test_imu.cpp                        #IMU.h
	test_imufilter.cpp                  #IMUFilter.h
	test_imuthread.cpp                  #IMUThread.h
	test_landmark.cpp                   #Landmark.h
	test_localgrid.cpp                  #LocalGrid.h
	test_features2d.cpp                 #Features2d.h
	test_globalmap.cpp                  #GlobalMap.h
	test_registration.cpp               #Registration.h
	test_registrationvis.cpp            #RegistrationVis.h
	test_registrationicp.cpp            #RegistrationIcp.h
	test_localgridmaker.cpp             #LocalGridMaker.h
	test_graph.cpp                      #Graph.h
	test_geodeticcoords.cpp             #GeodeticCoords.h
	test_compression.cpp                #Compression.h
	test_odometry.cpp                   #Odometry.h
	test_dbdriver.cpp                   #DBDriver.h
	test_dbdriversqlite3.cpp            #DBDriverSqlite3.h
	test_signature.cpp                  #Signature.h
	test_sensorevent.cpp                #SensorEvent.h
	test_sensordata.cpp                 #SensorData.h
	test_sensorcapture.cpp              #SensorCapture.h
	test_cameraimages.cpp               #camera/CameraImages.h
	test_sensorcaptureinfo.cpp          #SensorCaptureInfo.h
	test_sensorcapturethread.cpp        #SensorCaptureThread.h
)

add_executable(test_corelib ${corelib_test_sources})
target_link_libraries(test_corelib gtest_main rtabmap_core)

# test_registrationicp.cpp includes corelib/src/icp/libpointmatcher.h directly
# to test the LaserScan <-> DataPoints conversions, so it needs the library
# itself (rtabmap_core links it PRIVATE and only re-exports its include dirs).
IF(libpointmatcher_FOUND)
	target_link_libraries(test_corelib ${libpointmatcher_LIBRARIES})
ENDIF(libpointmatcher_FOUND)

# The timeouts below are sized for an optimized build; at -O0 (Debug, or
# coverage, which forces -O0) these tests run 5-40x slower.
IF(CMAKE_BUILD_TYPE MATCHES "^(Release|RelWithDebInfo|MinSizeRel)$")
	SET(_test_timeout_scale 1)
ELSE()
	SET(_test_timeout_scale 6)
ENDIF()

# Split the run across processes so `ctest -j` still overlaps work, paying the
# library load N times instead of once per source file. gtest partitions by
# test index via these two env vars; the shards are disjoint and together
# cover every test. TIMEOUT is set explicitly because a shard holds hundreds
# of tests and the TIMEOUT property has to win over the CI jobs' `ctest
# --timeout 300`, which was sized for one-test-per-process.
set(CORELIB_TEST_SHARDS 4 CACHE STRING
		"Number of processes the corelib unit tests are split across")
math(EXPR _corelib_last_shard "${CORELIB_TEST_SHARDS} - 1")
math(EXPR _corelib_shard_timeout "900 * ${_test_timeout_scale}")
foreach(shard RANGE 0 ${_corelib_last_shard})
	add_test(NAME test_corelib_${shard} COMMAND test_corelib)
	set_tests_properties(test_corelib_${shard} PROPERTIES
			TIMEOUT ${_corelib_shard_timeout}
			ENVIRONMENT "GTEST_TOTAL_SHARDS=${CORELIB_TEST_SHARDS};GTEST_SHARD_INDEX=${shard}")
endforeach()

# Comparison of the FlannIndex backends (times, memory, recall). Its own
# executable so that it can be run on demand:
#   bin/test_flann_index_perf
#   bin/test_flann_index_perf --gtest_filter=*Descriptors*
# and so that its seconds of benchmarking stay out of the unit test shards.
IF(BUILD_PERF_TESTS)
	add_executable(test_flann_index_perf perf_flann_index.cpp)
	target_link_libraries(test_flann_index_perf gtest_main rtabmap_core)

	# Labelled "performance" so CI jobs can opt out via `ctest -LE performance`,
	# and run them alone with `ctest -L performance`.
	add_test(NAME test_flann_index_perf COMMAND test_flann_index_perf)
	math(EXPR _perf_timeout "600 * ${_test_timeout_scale}")
	set_tests_properties(test_flann_index_perf PROPERTIES
			TIMEOUT ${_perf_timeout}
			LABELS  "performance")

	# Comparison of the dense and the sparse prediction x posterior multiplication of
	# BayesFilter (Bayes/SparsePrediction), against the size of the map, how connected
	# its graph is and the depth of the prediction model. Over synthetic graphs, and over
	# the graph of a real map read from data/tests/large_reduced_graph.g2o, whose link
	# types decide how much of the map a column of the prediction holds:
	#   bin/test_bayesfilter_perf
	#   bin/test_bayesfilter_perf --gtest_filter=-*LargeMap*:-*RealMap*
	# Its own executable: it spends its time benchmarking rather than asserting, and the
	# largest maps it builds allocate a gigabyte for the dense prediction matrix,
	# which in a unit test shard would look like a leak.
	add_executable(test_bayesfilter_perf perf_bayesfilter.cpp)
	target_link_libraries(test_bayesfilter_perf gtest_main rtabmap_core)

	add_test(NAME test_bayesfilter_perf COMMAND test_bayesfilter_perf)
	set_tests_properties(test_bayesfilter_perf PROPERTIES
			TIMEOUT ${_perf_timeout}
			LABELS  "performance")
ENDIF(BUILD_PERF_TESTS)

# Rtabmap end-to-end replay of sample DBs (test data fetched by
# scripts/fetch_test_data.sh into data/tests/*.db). Skips at runtime if the
# assets are absent, so the build stays green for contributors without them.
#
# Deliberately its own executable rather than part of test_corelib: it is the
# one long test (>20 min), it needs its own timeout and "long" label, and a
# crash in it stays contained instead of taking a shard of unit tests with it.
add_executable(test_rtabmap_integration test_rtabmap_integration.cpp)
target_link_libraries(test_rtabmap_integration gtest_main rtabmap_core)

# Labelled "long" so CI PR jobs can opt out via:
#  `ctest -LE long`
# while nightlies run:
#  `ctest -L long`.
add_test(NAME test_rtabmap_integration COMMAND test_rtabmap_integration)
math(EXPR _integration_timeout "1800 * ${_test_timeout_scale}")
set_tests_properties(test_rtabmap_integration PROPERTIES
        TIMEOUT ${_integration_timeout}
        LABELS  "long")

#PythonInterface / PyDetector / PyDescriptor / PyMatcher (optional Python
# support; bridges to Python feature detectors (SuperPoint), global
# descriptors (NetVLAD), and matchers (SuperGlue). Tests use numpy-only stub
# scripts so no ML weights or extra packages are needed beyond what
# WITH_PYTHON already pulls in.
#
# Kept out of test_corelib: these embed a Python interpreter, and
# Py_Initialize/Py_Finalize in a process that also runs hundreds of unrelated
# tests is a needless interaction. They also need their own link/include sets.
IF(WITH_PYTHON AND Python3_FOUND)
	# Direct tests for PythonInterface / getPythonTraceback. Links pybind11
	# and Python headers so the test can drive the Python C API directly.
	add_executable(test_python_interface test_python_interface.cpp)
	target_link_libraries(test_python_interface gtest_main rtabmap_core pybind11::embed)
	target_include_directories(test_python_interface PRIVATE ${Python3_INCLUDE_DIRS})
	add_test(NAME test_python_interface COMMAND test_python_interface)

	add_executable(test_pydetector test_pydetector.cpp)
	target_link_libraries(test_pydetector gtest_main rtabmap_core)
	add_test(NAME test_pydetector COMMAND test_pydetector)

	add_executable(test_pydescriptor test_pydescriptor.cpp)
	target_link_libraries(test_pydescriptor gtest_main rtabmap_core)
	add_test(NAME test_pydescriptor COMMAND test_pydescriptor)

	# PyMatcher's header is internal (not in the public include tree);
	# expose corelib/src so the test can `#include "python/PyMatcher.h"`.
	add_executable(test_pymatcher test_pymatcher.cpp)
	target_link_libraries(test_pymatcher gtest_main rtabmap_core)
	target_include_directories(test_pymatcher PRIVATE
			${CMAKE_SOURCE_DIR}/corelib/src)
	add_test(NAME test_pymatcher COMMAND test_pymatcher)
ENDIF(WITH_PYTHON AND Python3_FOUND)

#LASWriter.h (optional libLAS -- own target for the extra link/include set)
IF(libLAS_FOUND)
	add_executable(test_laswriter test_laswriter.cpp)
	target_link_libraries(test_laswriter gtest_main rtabmap_core ${libLAS_LIBRARIES})
	target_include_directories(test_laswriter PRIVATE ${libLAS_INCLUDE_DIRS})
	add_test(NAME test_laswriter COMMAND test_laswriter)
ENDIF(libLAS_FOUND)
