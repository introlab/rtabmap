#include "gtest/gtest.h"
#include "rtabmap/utilite/UThread.h"
#include "rtabmap/utilite/UTimer.h"
#include <thread>
#include <chrono>
#include <atomic>

// Simple test thread
class SimpleThread : public UThread
{
public:
    SimpleThread() : UThread(), counter_(0), started_(false) {}
    virtual ~SimpleThread() {join(true);}
    int getCounter() const { return counter_; }
    bool hasStarted() const { return started_; }
    void reset() { counter_ = 0; started_ = false; }
    
protected:
    virtual void mainLoop()
    {
        started_ = true;
        counter_++;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        kill(); // Stop after one iteration
    }
    
private:
    int counter_;
    bool started_;
};

// Thread that runs multiple iterations
class CountingThread : public UThread
{
public:
    CountingThread() : UThread(), counter_(0), maxIterations_(10) {}
    virtual ~CountingThread() {join(true);}
    void setMaxIterations(int max) { maxIterations_ = max; }
    int getCounter() const { return counter_; }
    
protected:
    virtual void mainLoop()
    {
        counter_++;
        if(counter_ >= maxIterations_)
        {
            kill();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    
private:
    int counter_;
    int maxIterations_;
};

// Thread with initialization
class InitThread : public UThread
{
public:
    InitThread() : UThread(), initialized_(false), counter_(0) {}
    virtual ~InitThread() {join(true);}
    bool isInitialized() const { return initialized_; }
    int getCounter() const { return counter_; }
    
protected:
    virtual void mainLoopBegin()
    {
        initialized_ = true;
    }
    
    virtual void mainLoop()
    {
        counter_++;
        if(counter_ >= 5)
        {
            kill();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    
private:
    bool initialized_;
    int counter_;
};

// Thread with cleanup
class CleanupThread : public UThread
{
public:
    CleanupThread() : UThread(), cleanedUp_(false), counter_(0) {}
    virtual ~CleanupThread() {join(true);}
    bool isCleanedUp() const { return cleanedUp_; }
    int getCounter() const { return counter_; }
    
protected:
    virtual void mainLoop()
    {
        counter_++;
        if(counter_ >= 3)
        {
            kill();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    
    virtual void mainLoopEnd()
    {
        cleanedUp_ = true;
    }
    
private:
    bool cleanedUp_;
    int counter_;
};

TEST(UThreadTest, Constructor)
{
    SimpleThread thread;
    EXPECT_TRUE(thread.isIdle());
    EXPECT_FALSE(thread.isRunning());
    EXPECT_FALSE(thread.isCreating());
    EXPECT_FALSE(thread.isKilled());
}

TEST(UThreadTest, Start)
{
    SimpleThread thread;
    thread.start();

    EXPECT_TRUE(thread.isRunning() || thread.isCreating());

    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    EXPECT_TRUE(thread.isRunning());
    
    // Give thread time to stop
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    
    // Thread should be running or killed (if it finished quickly)
    EXPECT_TRUE(thread.isKilled() || thread.isIdle());
    
    thread.join();
}

TEST(UThreadTest, Join)
{
    SimpleThread thread;
    UTimer timer;
    thread.start();
    thread.join();
    
    EXPECT_GT(timer.elapsed(), 0.01);
    EXPECT_TRUE(thread.isIdle());
    EXPECT_GT(thread.getCounter(), 0);
}

TEST(UThreadTest, JoinWait)
{
    CountingThread thread;
    thread.setMaxIterations(10);
    thread.start();
    
    // Join wait
    thread.join(false);
    
    EXPECT_TRUE(thread.isIdle());
    EXPECT_EQ(thread.getCounter(), 10); // Should have processed all iterations
}

TEST(UThreadTest, JoinKillFirst)
{
    CountingThread thread;
    thread.setMaxIterations(1000);
    thread.start();
    
    // Wait a bit
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    
    // Join with kill
    thread.join(true);
    
    EXPECT_TRUE(thread.isIdle());
    EXPECT_LT(thread.getCounter(), 1000); // Should have stopped early
}

TEST(UThreadTest, Kill)
{
    CountingThread thread;
    thread.setMaxIterations(1000);
    thread.start();
    
    // Wait a bit
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    
    // Kill the thread
    thread.kill();

    EXPECT_TRUE(thread.isKilled());
    
    // Wait for it to finish
    thread.join();
    
    EXPECT_TRUE(thread.isIdle());
    EXPECT_LT(thread.getCounter(), 1000); // Should have stopped early
}

TEST(UThreadTest, MultipleStart)
{
    SimpleThread thread;
    thread.start();
    
    // Try to start again (should be ignored)
    thread.start();
    
    thread.join();
    
    EXPECT_TRUE(thread.isIdle());
}

TEST(UThreadTest, MainLoopBegin)
{
    InitThread thread;
    thread.start();
    thread.join();
    
    EXPECT_TRUE(thread.isInitialized());
    EXPECT_GT(thread.getCounter(), 0);
}

TEST(UThreadTest, MainLoopEnd)
{
    CleanupThread thread;
    thread.start();
    thread.join();
    
    EXPECT_TRUE(thread.isCleanedUp());
    EXPECT_GT(thread.getCounter(), 0);
}

TEST(UThreadTest, ThreadId)
{
    SimpleThread thread;
    unsigned long mainThreadId = UThread::currentThreadId();
    
    thread.start();
    unsigned long threadId = thread.getThreadId();
    
    EXPECT_NE(threadId, 0u);
    EXPECT_NE(threadId, mainThreadId);
    
    thread.join();
}

TEST(UThreadTest, CurrentThreadId)
{
    unsigned long id1 = UThread::currentThreadId();
    unsigned long id2 = UThread::currentThreadId();
    
    EXPECT_EQ(id1, id2);
    EXPECT_NE(id1, 0u);
}

TEST(UThreadTest, SetPriority)
{
    SimpleThread thread;
    thread.setPriority(UThread::kPLow);
    thread.setPriority(UThread::kPNormal);
    thread.setPriority(UThread::kPAboveNormal);
    
    thread.start();
    thread.join();
    
    SUCCEED();
}

TEST(UThreadTest, SetAffinity)
{
    SimpleThread thread;
    thread.setAffinity(0); // No affinity
    thread.setAffinity(1); // CPU 1
    
    thread.start();
    thread.join();
    
    SUCCEED();
}

TEST(UThreadTest, ThreadHandle)
{
    SimpleThread thread;
    thread.start();
    
    UThread::Handle handle = thread.getThreadHandle();
    EXPECT_NE(handle, 0);
    
    thread.join();
}

TEST(UThreadTest, MultipleThreads)
{
    std::vector<SimpleThread*> threads;
    const int numThreads = 5;
    
    for(int i = 0; i < numThreads; ++i)
    {
        threads.push_back(new SimpleThread());
        threads[i]->start();
    }
    
    for(int i = 0; i < numThreads; ++i)
    {
        threads[i]->join();
        EXPECT_GT(threads[i]->getCounter(), 0);
        delete threads[i];
    }
}

TEST(UThreadTest, DestructorJoin)
{
    // Test that destructor properly cleans up
    {
        SimpleThread thread;
        thread.start();
        // Destructor should call join(true) if implemented
    }
    
    SUCCEED();
}

TEST(UThreadTest, KillBeforeStart)
{
    SimpleThread thread;
    thread.kill(); // Kill before start
    
    thread.start();
    thread.join();
    
    // Should still work
    SUCCEED();
}

TEST(UThreadTest, JoinBeforeStart)
{
    SimpleThread thread;
    thread.join(); // Join before start
    
    thread.start();
    thread.join();
    
    SUCCEED();
}

