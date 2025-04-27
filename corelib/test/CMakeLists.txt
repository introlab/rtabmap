
add_definitions("-DRTABMAP_TEST_DATA_ROOT=\"${TEST_DATA_ROOT}\"")

#util2d.h
add_executable(test_util2d test_util2d.cpp)
target_link_libraries(test_util2d gtest_main rtabmap_core)
gtest_discover_tests(test_util2d)

#util3d.h
add_executable(test_util3d test_util3d.cpp)
target_link_libraries(test_util3d gtest_main rtabmap_core)
gtest_discover_tests(test_util3d)