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
    EXPECT_TRUE(mutex.lock());
    EXPECT_TRUE(mutex.unlock());
}

TEST(UMutexTest, LockTry)
{
    UMutex mutex;
    
    // Try to lock when not locked
    EXPECT_TRUE(mutex.lockTry());
    
    // Unlock
    mutex.unlock();
}

TEST(UMutexTest, LockTryWhenLocked)
{
    UMutex mutex;
    
    // Lock the mutex
    mutex.lock();
    
    // Try to lock from another thread
    std::atomic<bool> tryResult(false);
    std::thread t([&mutex, &tryResult]() {
        tryResult = mutex.lockTry(); // Should fail (busy or similar)
    });
    
    t.join();
    
    EXPECT_FALSE(tryResult);
    
    mutex.unlock();
}

TEST(UMutexTest, ThreadSafety)
{
    UMutex mutex;
    std::atomic<int> counter(0);
    const int numThreads = 4;
    const int iterations = 1000;
    
    std::vector<std::thread> threads;
    
    for(int i = 0; i < numThreads; ++i)
    {
        threads.emplace_back([&mutex, &counter, iterations]() {
            for(int j = 0; j < iterations; ++j)
            {
                mutex.lock();
                int val = counter.load();
                std::this_thread::sleep_for(std::chrono::microseconds(1));
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
    EXPECT_TRUE(mutex.lock());
    EXPECT_TRUE(mutex.lock());
    EXPECT_TRUE(mutex.lock());
    
    // Unlock multiple times
    EXPECT_TRUE(mutex.unlock());
    EXPECT_TRUE(mutex.unlock());
    EXPECT_TRUE(mutex.unlock());
}

TEST(UMutexTest, UScopeMutex)
{
    UMutex mutex;  
    {
        UScopeMutex scopeMutex(mutex);
        // Mutex should be locked here

        // Try to lock from another thread
        std::thread t([&mutex]() {
            EXPECT_FALSE(mutex.lockTry()); // Should fail
        });
        t.join();
    }
    // Mutex should be unlocked here
    
    // Try to lock from another thread
    std::thread t([&mutex]() {
        EXPECT_TRUE(mutex.lockTry()); // Should succeed
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
            EXPECT_FALSE(mutex.lockTry()); // Should fail
        });
        t.join();
    }
    // Mutex should be unlocked here
    
    // Try to lock from another thread
    std::thread t([&mutex]() {
        EXPECT_TRUE(mutex.lockTry()); // Should succeed
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

