# UConversion.h
add_executable(test_uconversion test_uconversion.cpp)
target_link_libraries(test_uconversion gtest_main rtabmap::utilite)
add_test(NAME test_uconversion COMMAND test_uconversion)

# UTimer.h
add_executable(test_utimer test_utimer.cpp)
target_link_libraries(test_utimer gtest_main rtabmap::utilite)
add_test(NAME test_utimer COMMAND test_utimer)

# UVariant.h
add_executable(test_uvariant test_uvariant.cpp)
target_link_libraries(test_uvariant gtest_main rtabmap::utilite)
add_test(NAME test_uvariant COMMAND test_uvariant)

# UFile.h
add_executable(test_ufile test_ufile.cpp)
target_link_libraries(test_ufile gtest_main rtabmap::utilite)
add_test(NAME test_ufile COMMAND test_ufile)

# UDirectory.h
add_executable(test_udirectory test_udirectory.cpp)
target_link_libraries(test_udirectory gtest_main rtabmap::utilite)
add_test(NAME test_udirectory COMMAND test_udirectory)

# UProcessInfo.h
add_executable(test_uprocessinfo test_uprocessinfo.cpp)
target_link_libraries(test_uprocessinfo gtest_main rtabmap::utilite)
add_test(NAME test_uprocessinfo COMMAND test_uprocessinfo)

# UMath.h
add_executable(test_umath test_umath.cpp)
target_link_libraries(test_umath gtest_main rtabmap::utilite)
add_test(NAME test_umath COMMAND test_umath)

# UStl.h
add_executable(test_ustl test_ustl.cpp)
target_link_libraries(test_ustl gtest_main rtabmap::utilite)
add_test(NAME test_ustl COMMAND test_ustl)

# UEventsHandler.h, UEventsManager.h, UEventsSender.h
add_executable(test_uevents test_uevents.cpp)
target_link_libraries(test_uevents gtest_main rtabmap::utilite)
add_test(NAME test_uevents COMMAND test_uevents)

# UMutex.h
add_executable(test_umutex test_umutex.cpp)
target_link_libraries(test_umutex gtest_main rtabmap::utilite)
add_test(NAME test_umutex COMMAND test_umutex)

# UThread.h
add_executable(test_uthread test_uthread.cpp)
target_link_libraries(test_uthread gtest_main rtabmap::utilite)
add_test(NAME test_uthread COMMAND test_uthread)

# USemaphore.h
add_executable(test_usemaphore test_usemaphore.cpp)
target_link_libraries(test_usemaphore gtest_main rtabmap::utilite)
add_test(NAME test_usemaphore COMMAND test_usemaphore)

# ULogger.h
add_executable(test_ulogger test_ulogger.cpp)
target_link_libraries(test_ulogger gtest_main rtabmap::utilite)
add_test(NAME test_ulogger COMMAND test_ulogger)
set_tests_properties(test_ulogger PROPERTIES RUN_SERIAL TRUE)

