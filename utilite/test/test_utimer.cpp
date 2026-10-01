#include "gtest/gtest.h"
#include "rtabmap/utilite/UTimer.h"
#include <thread>
#include <chrono>

namespace {

// Requested sleep durations (seconds)
constexpr double kSleep10Ms = 0.01;
constexpr double kSleep20Ms = 0.02;
constexpr double kSleep5Ms = 0.005;

// Upper bounds: macOS/CI runners often wake threads much later than requested
constexpr double kMaxSlack10Ms = 0.25;
constexpr double kMaxSlack20Ms = 0.40;
constexpr double kMaxSlack5Ms = 0.15;

} // namespace

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
    EXPECT_GE(elapsed, kSleep10Ms);
    EXPECT_LT(elapsed, kMaxSlack10Ms);
}

TEST(UTimerTest, Elapsed)
{
    UTimer timer;
    timer.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    double elapsed = timer.elapsed();
    EXPECT_GE(elapsed, kSleep10Ms);
    EXPECT_LT(elapsed, kMaxSlack10Ms);

    // Timer should still be running after elapsed()
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    double ticks2 = timer.ticks();
    EXPECT_GE(ticks2, kSleep20Ms);
    EXPECT_LT(ticks2, kMaxSlack20Ms);
}

TEST(UTimerTest, GetElapsedTime)
{
    UTimer timer;
    timer.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    double elapsed = timer.getElapsedTime();
    EXPECT_GE(elapsed, kSleep10Ms);
    EXPECT_LT(elapsed, kMaxSlack10Ms);

    // Timer should still be running after getElapsedTime()
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    double ticks2 = timer.ticks();
    EXPECT_GE(ticks2, kSleep20Ms);
    EXPECT_LT(ticks2, kMaxSlack20Ms);
}

TEST(UTimerTest, Ticks)
{
    UTimer timer;
    timer.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    double ticks1 = timer.ticks();
    EXPECT_GE(ticks1, kSleep10Ms);
    EXPECT_LT(ticks1, kMaxSlack10Ms);

    // Timer should have retarted and still be running after ticks()
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    double ticks2 = timer.ticks();
    EXPECT_GE(ticks2, kSleep10Ms);
    EXPECT_LT(ticks2, kMaxSlack10Ms);
}

TEST(UTimerTest, Restart)
{
    UTimer timer;
    timer.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    double elapsed1 = timer.restart();
    EXPECT_GE(elapsed1, kSleep10Ms);
    EXPECT_LT(elapsed1, kMaxSlack10Ms);

    // After restart, timer should be running again
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    double elapsed2 = timer.elapsed();
    EXPECT_GE(elapsed2, kSleep10Ms);
    EXPECT_LT(elapsed2, kMaxSlack10Ms);
}

TEST(UTimerTest, Now)
{
    double time1 = UTimer::now();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    double time2 = UTimer::now();
    EXPECT_GE(time2-time1, kSleep10Ms);
    EXPECT_LT(time2-time1, kMaxSlack10Ms);
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
    
    EXPECT_GE(elapsed1, kSleep5Ms);
    EXPECT_LT(elapsed1, kMaxSlack5Ms);
    EXPECT_GE(elapsed2, kSleep5Ms);
    EXPECT_LT(elapsed2, kMaxSlack5Ms);
}

