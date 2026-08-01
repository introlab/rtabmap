#include "gtest/gtest.h"
#include "rtabmap/utilite/UMutex.h"
#include <thread>
#include <chrono>
#include <atomic>
#include <vector>

TEST(UMutexTest, Constructor)
{
    UMutex mutex;
    // Should be constructible
    SUCCEED();
}

TEST(UMutexTest, LockUnlock)
{
    UMutex mutex;
    // lock()/unlock() return 0 on success (an error code otherwise).
    EXPECT_EQ(mutex.lock(), 0);
    EXPECT_EQ(mutex.unlock(), 0);
}

TEST(UMutexTest, LockTry)
{
    UMutex mutex;
    
    // Try to lock when not locked: lockTry() returns 0 when it took the lock.
    EXPECT_EQ(mutex.lockTry(), 0);
    
    // Unlock
    mutex.unlock();
}

TEST(UMutexTest, LockTryWhenLocked)
{
    UMutex mutex;
    
    // Lock the mutex
    mutex.lock();
    
    // Try to lock from another thread
    std::atomic<int> tryResult(0);
    std::thread t([&mutex, &tryResult]() {
        tryResult = mutex.lockTry(); // Should fail (EBUSY or similar)
    });
    
    t.join();
    
    EXPECT_NE(tryResult, 0);
    
    mutex.unlock();
}

TEST(UMutexTest, ThreadSafety)
{
    UMutex mutex;
    std::atomic<int> counter(0);
    static constexpr int numThreads = 4;
    static constexpr int iterations = 1000;
    
    std::vector<std::thread> threads;
    
    for(int i = 0; i < numThreads; ++i)
    {
        threads.emplace_back([&mutex, &counter]() {
            for(int j = 0; j < iterations; ++j)
            {
                mutex.lock();
                int val = counter.load();
                // yield() instead of sleep_for(1us): the original sleep was
                // ~1us intended (negligible on Linux/macOS) but ~15ms on
                // Windows because of the default 15.6ms timer tick -- that
                // turned this test into a 60s test on Windows CI. yield()
                // still scrambles interleaving without the wall-clock cost.
                std::this_thread::yield();
                counter.store(val + 1);
                mutex.unlock();
            }
        });
    }
    
    for(auto& t : threads)
    {
        t.join();
    }
    
    EXPECT_EQ(counter.load(), numThreads * iterations);
}

TEST(UMutexTest, RecursiveLock)
{
    UMutex mutex;
    
    // Lock multiple times from same thread (should work on Unix)
    EXPECT_EQ(mutex.lock(), 0);
    EXPECT_EQ(mutex.lock(), 0);
    EXPECT_EQ(mutex.lock(), 0);
    
    // Unlock multiple times
    EXPECT_EQ(mutex.unlock(), 0);
    EXPECT_EQ(mutex.unlock(), 0);
    EXPECT_EQ(mutex.unlock(), 0);
}

TEST(UMutexTest, UScopeMutex)
{
    UMutex mutex;  
    {
        UScopeMutex scopeMutex(mutex);
        // Mutex should be locked here

        // Try to lock from another thread
        std::thread t([&mutex]() {
            EXPECT_NE(mutex.lockTry(), 0); // Should fail
        });
        t.join();
    }
    // Mutex should be unlocked here
    
    // Try to lock from another thread
    std::thread t([&mutex]() {
        EXPECT_EQ(mutex.lockTry(), 0); // Should succeed
        mutex.unlock();
    });
    t.join();
}

TEST(UMutexTest, UScopeMutexWithPointer)
{
    UMutex mutex;
    
    {
        UScopeMutex scopeMutex(&mutex);
        // Mutex should be locked here

        // Try to lock from another thread
        std::thread t([&mutex]() {
            EXPECT_NE(mutex.lockTry(), 0); // Should fail
        });
        t.join();
    }
    // Mutex should be unlocked here
    
    // Try to lock from another thread
    std::thread t([&mutex]() {
        EXPECT_EQ(mutex.lockTry(), 0); // Should succeed
        mutex.unlock();
    });
    t.join();
}

TEST(UMutexTest, MultipleMutexes)
{
    UMutex mutex1;
    UMutex mutex2;
    
    mutex1.lock();
    mutex2.lock();
    
    mutex1.unlock();
    mutex2.unlock();
    
    SUCCEED();
}

TEST(UMutexTest, LockUnlockSequence)
{
    UMutex mutex;
    
    for(int i = 0; i < 10; ++i)
    {
        mutex.lock();
        mutex.unlock();
    }
    
    SUCCEED();
}

