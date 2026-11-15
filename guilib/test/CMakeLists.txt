set(guilib_test_sources
	test_ucv2qt.cpp                     #utilite/UCv2Qt.h
)

add_executable(test_guilib ${guilib_test_sources})
target_link_libraries(test_guilib GTest::gtest_main rtabmap_gui)

add_test(NAME test_guilib COMMAND test_guilib)
