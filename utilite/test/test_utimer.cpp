#include "gtest/gtest.h"
#include "rtabmap/utilite/UTimer.h"
#include <thread>
#include <chrono>

TEST(UTimerTest, Constructor)
{
    UTimer timer;
    // Timer should be constructible
    SUCCEED();
}

TEST(UTimerTest, StartStop)
{
    UTimer timer;
    timer.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    timer.stop();
    double elapsed = timer.getElapsedTime();
    EXPECT_GE(elapsed, 0.01); // Should be at least 10 ms
    EXPECT_LT(elapsed, 0.015); // Should be less than 15 ms
}

TEST(UTimerTest, Elapsed)
{
    UTimer timer;
    timer.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    double elapsed = timer.elapsed();
    EXPECT_GE(elapsed, 0.01);
    EXPECT_LT(elapsed, 0.015);

    // Timer should still be running after elapsed()
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    double ticks2 = timer.ticks();
    EXPECT_GE(ticks2, 0.02);
    EXPECT_LT(ticks2, 0.025);
}

TEST(UTimerTest, GetElapsedTime)
{
    UTimer timer;
    timer.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    double elapsed = timer.getElapsedTime();
    EXPECT_GE(elapsed, 0.01);
    EXPECT_LT(elapsed, 0.015);

    // Timer should still be running after getElapsedTime()
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    double ticks2 = timer.ticks();
    EXPECT_GE(ticks2, 0.02);
    EXPECT_LT(ticks2, 0.025);
}

TEST(UTimerTest, Ticks)
{
    UTimer timer;
    timer.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    double ticks1 = timer.ticks();
    EXPECT_GE(ticks1, 0.01);
    EXPECT_LT(ticks1, 0.015);
    
    // Timer should have retarted and still be running after ticks()
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    double ticks2 = timer.ticks();
    EXPECT_GE(ticks2, 0.01);
    EXPECT_LT(ticks2, 0.015);
}

TEST(UTimerTest, Restart)
{
    UTimer timer;
    timer.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    double elapsed1 = timer.restart();
    EXPECT_GE(elapsed1, 0.01);
    EXPECT_LT(elapsed1, 0.02);
    
    // After restart, timer should be running again
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    double elapsed2 = timer.elapsed();
    EXPECT_GE(elapsed2, 0.01);
    EXPECT_LT(elapsed2, 0.02);
}

TEST(UTimerTest, Now)
{
    double time1 = UTimer::now();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    double time2 = UTimer::now();
    EXPECT_GE(time2-time1, 0.01);
    EXPECT_LT(time2-time1, 0.015);
}

TEST(UTimerTest, MultipleStartStop)
{
    UTimer timer;
    timer.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    timer.stop();
    double elapsed1 = timer.getElapsedTime();
    
    timer.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    timer.stop();
    double elapsed2 = timer.getElapsedTime();
    
    EXPECT_GE(elapsed1, 0.005);
    EXPECT_LT(elapsed1, 0.01);
    EXPECT_GE(elapsed2, 0.005);
    EXPECT_LT(elapsed2, 0.01);
}

