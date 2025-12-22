# UConversion.h
add_executable(test_uconversion test_uconversion.cpp)
target_link_libraries(test_uconversion gtest_main rtabmap::utilite)
gtest_discover_tests(test_uconversion)

# UTimer.h
add_executable(test_utimer test_utimer.cpp)
target_link_libraries(test_utimer gtest_main rtabmap::utilite)
gtest_discover_tests(test_utimer)

# UVariant.h
add_executable(test_uvariant test_uvariant.cpp)
target_link_libraries(test_uvariant gtest_main rtabmap::utilite)
gtest_discover_tests(test_uvariant)

# UFile.h
add_executable(test_ufile test_ufile.cpp)
target_link_libraries(test_ufile gtest_main rtabmap::utilite)
gtest_discover_tests(test_ufile)

# UDirectory.h
add_executable(test_udirectory test_udirectory.cpp)
target_link_libraries(test_udirectory gtest_main rtabmap::utilite)
gtest_discover_tests(test_udirectory)

# UProcessInfo.h
add_executable(test_uprocessinfo test_uprocessinfo.cpp)
target_link_libraries(test_uprocessinfo gtest_main rtabmap::utilite)
gtest_discover_tests(test_uprocessinfo)

# UMath.h
add_executable(test_umath test_umath.cpp)
target_link_libraries(test_umath gtest_main rtabmap::utilite)
gtest_discover_tests(test_umath)

# UStl.h
add_executable(test_ustl test_ustl.cpp)
target_link_libraries(test_ustl gtest_main rtabmap::utilite)
gtest_discover_tests(test_ustl)

# UEventsHandler.h, UEventsManager.h, UEventsSender.h
add_executable(test_uevents test_uevents.cpp)
target_link_libraries(test_uevents gtest_main rtabmap::utilite)
gtest_discover_tests(test_uevents)

# UMutex.h
add_executable(test_umutex test_umutex.cpp)
target_link_libraries(test_umutex gtest_main rtabmap::utilite)
gtest_discover_tests(test_umutex)

# UThread.h
add_executable(test_uthread test_uthread.cpp)
target_link_libraries(test_uthread gtest_main rtabmap::utilite)
gtest_discover_tests(test_uthread)

# USemaphore.h
add_executable(test_usemaphore test_usemaphore.cpp)
target_link_libraries(test_usemaphore gtest_main rtabmap::utilite)
gtest_discover_tests(test_usemaphore)

# ULogger.h
add_executable(test_ulogger test_ulogger.cpp)
target_link_libraries(test_ulogger gtest_main rtabmap::utilite)
gtest_discover_tests(test_ulogger
    PROPERTIES RUN_SERIAL TRUE
)

