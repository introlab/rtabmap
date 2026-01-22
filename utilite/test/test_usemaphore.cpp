#include "gtest/gtest.h"
#include "rtabmap/utilite/USemaphore.h"
#include "rtabmap/utilite/UMutex.h"
#include "rtabmap/utilite/UTimer.h"
#include <thread>
#include <chrono>
#include <atomic>
#include <vector>

TEST(USemaphoreTest, ConstructorDefault)
{
    USemaphore sem;
    // Should be constructible with default value (0)
    EXPECT_EQ(sem.value(), 0);
}

TEST(USemaphoreTest, ConstructorWithValue)
{
    USemaphore sem(5);
    EXPECT_EQ(sem.value(), 5);
}

TEST(USemaphoreTest, Release)
{
    USemaphore sem(0);
    
    sem.release();
    EXPECT_EQ(sem.value(), 1);
    
    sem.release(3);
    EXPECT_EQ(sem.value(), 4);
}

TEST(USemaphoreTest, Acquire)
{
    USemaphore sem(2);
    
    bool result = sem.acquire();
    EXPECT_TRUE(result);
    EXPECT_EQ(sem.value(), 1);
    
    result = sem.acquire();
    EXPECT_TRUE(result);
    EXPECT_EQ(sem.value(), 0);
}

TEST(USemaphoreTest, AcquireMultiple)
{
    USemaphore sem(5);
    
    bool result = sem.acquire(3);
    EXPECT_TRUE(result);
    EXPECT_EQ(sem.value(), 2);
    
    result = sem.acquire(2);
    EXPECT_TRUE(result);
    EXPECT_EQ(sem.value(), 0);
}

TEST(USemaphoreTest, AcquireBlocks)
{
    USemaphore sem(0);
    std::atomic<bool> acquired(false);
    std::atomic<bool> threadStarted(false);
    
    std::thread t([&sem, &acquired, &threadStarted]() {
        threadStarted = true;
        bool result = sem.acquire();
        acquired = result;
    });
    
    // Wait for thread to start and try to acquire
    while(!threadStarted)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    
    // Thread should be blocked
    EXPECT_FALSE(acquired);
    
    // Release to unblock
    sem.release();
    
    t.join();
    
    EXPECT_TRUE(acquired);
}

TEST(USemaphoreTest, AcquireWithTimeout)
{
    USemaphore sem(0);
    
    // Try to acquire with timeout
    bool result = sem.acquire(1, 50); // 50ms timeout
    EXPECT_FALSE(result); // Should timeout
    
    // Release and try again
    sem.release();
    result = sem.acquire(1, 50);
    EXPECT_TRUE(result);
}

TEST(USemaphoreTest, AcquireTry)
{
    USemaphore sem(3);
    
    // Try to acquire when available
#ifdef _WIN32
    bool result = sem.acquireTry();
    EXPECT_TRUE(result);
    EXPECT_EQ(sem.value(), 2);
    result = sem.acquireTry();
    EXPECT_TRUE(result);
    EXPECT_EQ(sem.value(), 1);
    result = sem.acquireTry();
    EXPECT_TRUE(result);
    EXPECT_EQ(sem.value(), 0);
    result = sem.acquireTry();
    EXPECT_TRUE(result);
#else
    int result = sem.acquireTry(1);
    EXPECT_TRUE(result);
    EXPECT_EQ(sem.value(), 2);
    
    result = sem.acquireTry(2);
    EXPECT_TRUE(result);
    EXPECT_EQ(sem.value(), 0);
    
    // Try to acquire when not available
    result = sem.acquireTry(1);
    EXPECT_FALSE(result);
#endif
}

TEST(USemaphoreTest, Value)
{
    USemaphore sem(10);
    
    int value = sem.value();
    EXPECT_EQ(value, 10);
    
    sem.acquire(3);
    value = sem.value();
    EXPECT_EQ(value, 7);
    
    sem.release(2);
    value = sem.value();
    EXPECT_EQ(value, 9);
}

TEST(USemaphoreTest, ThreadSafety)
{
    USemaphore sem(0);
    std::atomic<int> counter(0);
    const int numThreads = 4;
    const int iterations = 100;
    
    std::vector<std::thread> threads;
    
    // Create threads that will wait on semaphore
    for(int i = 0; i < numThreads; ++i)
    {
        threads.emplace_back([&sem, &counter, iterations]() {
            for(int j = 0; j < iterations; ++j)
            {
                sem.acquire();
                counter++;
            }
        });
    }
    
    // Release semaphore to allow threads to proceed
    std::thread releaseThread([&sem, numThreads, iterations]() {
        for(int i = 0; i < numThreads * iterations; ++i)
        {
            sem.release();
            std::this_thread::sleep_for(std::chrono::microseconds(10));
        }
    });
    
    for(auto& t : threads)
    {
        t.join();
    }
    
    releaseThread.join();
    
    EXPECT_EQ(counter.load(), numThreads * iterations);
}

TEST(USemaphoreTest, ProducerConsumer)
{
    USemaphore full(0);
    USemaphore empty(10);
    std::vector<int> buffer;
    UMutex bufferMutex;
    const int items = 20;
    std::atomic<int> produced(0);
    std::atomic<int> consumed(0);
    
    // Producer thread
    std::thread producer([&full, &empty, &buffer, &bufferMutex, &produced, items]() {
        for(int i = 0; i < items; ++i)
        {
            empty.acquire();
            {
                UScopeMutex lock(bufferMutex);
                buffer.push_back(i);
            }
            full.release();
            produced++;
        }
    });
    
    // Consumer thread
    std::thread consumer([&full, &empty, &buffer, &bufferMutex, &consumed, items]() {
        for(int i = 0; i < items; ++i)
        {
            full.acquire();
            {
                UScopeMutex lock(bufferMutex);
                if(!buffer.empty())
                {
                    buffer.pop_back();
                }
            }
            empty.release();
            consumed++;
        }
    });
    
    producer.join();
    consumer.join();
    
    EXPECT_EQ(produced.load(), items);
    EXPECT_EQ(consumed.load(), items);
}

TEST(USemaphoreTest, MultipleAcquireRelease)
{
    USemaphore sem(0);
    
    // Release multiple times
    sem.release(5);
    EXPECT_EQ(sem.value(), 5);
    
    // Acquire multiple times
    bool result = sem.acquire(3);
    EXPECT_TRUE(result);
    EXPECT_EQ(sem.value(), 2);
    
    result = sem.acquire(2);
    EXPECT_TRUE(result);
    EXPECT_EQ(sem.value(), 0);
}

TEST(USemaphoreTest, AcquireZero)
{
    USemaphore sem(5);
    
    // Acquire 0 should not block
    bool result = sem.acquire(0);
    EXPECT_TRUE(result);
    EXPECT_EQ(sem.value(), 5);
}

TEST(USemaphoreTest, ReleaseZero)
{
    USemaphore sem(5);
    
    // Release 0 should not change value
    sem.release(0);
    EXPECT_EQ(sem.value(), 5);
}

#ifdef _WIN32
TEST(USemaphoreTest, Reset)
{
    USemaphore sem(5);
    EXPECT_EQ(sem.value(), 5);
    
    sem.reset(10);
    EXPECT_EQ(sem.value(), 10);
    
    sem.reset(0);
    EXPECT_EQ(sem.value(), 0);
}
#endif

TEST(USemaphoreTest, ConcurrentAcquireRelease)
{
    USemaphore sem(0);
    std::atomic<int> acquiredCount(0);
    const int numThreads = 5;
    
    std::vector<std::thread> acquireThreads;
    std::vector<std::thread> releaseThreads;
    
    // Create threads that acquire
    for(int i = 0; i < numThreads; ++i)
    {
        acquireThreads.emplace_back([&sem, &acquiredCount]() {
            sem.acquire();
            acquiredCount++;
        });
    }
    
    // Wait a bit
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    
    // Create threads that release
    for(int i = 0; i < numThreads; ++i)
    {
        releaseThreads.emplace_back([&sem]() {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            sem.release();
        });
    }
    
    for(auto& t : acquireThreads)
    {
        t.join();
    }
    
    for(auto& t : releaseThreads)
    {
        t.join();
    }
    
    EXPECT_EQ(acquiredCount.load(), numThreads);
}

