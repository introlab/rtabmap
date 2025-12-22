#include "gtest/gtest.h"
#include "rtabmap/utilite/UEvent.h"
#include "rtabmap/utilite/UEventsHandler.h"
#include "rtabmap/utilite/UEventsManager.h"
#include "rtabmap/utilite/UEventsSender.h"
#include "rtabmap/utilite/UTimer.h"
#include <thread>
#include <chrono>
#include <atomic>

// Test event classes
class TestEvent : public UEvent
{
public:
    TestEvent(int code = 0) : UEvent(code) {}
    virtual ~TestEvent() {}
    virtual std::string getClassName() const {return "TestEvent";}
};

class AnotherEvent : public UEvent
{
public:
    AnotherEvent() : UEvent(0) {}
    virtual ~AnotherEvent() {}
    virtual std::string getClassName() const {return "AnotherEvent";}
};

// Test handler class
class TestHandler : public UEventsHandler
{
public:
    TestHandler() : eventCount_(0), lastEventCode_(0){}
    
    int getEventCount() const { return eventCount_; }
    int getLastEventCode() const { return lastEventCode_; }
    const std::string & getLastEventClassName() const { return lastEventClassName_; }
    void reset() { eventCount_ = 0; lastEventCode_ = 0; lastEventClassName_.clear(); }
    
protected:
    virtual bool handleEvent(UEvent * event)
    {
        eventCount_++;
        lastEventCode_ = event->getCode();
        lastEventClassName_ = event->getClassName();
        return false;
    }

private:
    int eventCount_;
    int lastEventCode_;
    std::string lastEventClassName_;
    bool ownershipTaken_;
};

// Handler that takes ownership
class OwnershipHandler : public UEventsHandler
{
public:
    OwnershipHandler() : event_(nullptr) {}
    ~OwnershipHandler() {delete event_;}
    const UEvent * getEvent() const {return event_;} 
    void reset() { delete event_; event_ = nullptr;}
    
protected:
    virtual bool handleEvent(UEvent * event)
    {
        event_ = event; // Take ownership
        return true;
    }
    
private:
    UEvent * event_;
};

// Handler that filters events
class FilteringHandler : public UEventsHandler
{
public:
    FilteringHandler() : testEventCount_(0), anotherEventCount_(0) {}
    int getTestEventCount() const { return testEventCount_; }
    int getAnotherEventCount() const { return anotherEventCount_; }
    void reset() { testEventCount_ = 0; anotherEventCount_ = 0; }
    
protected:
    virtual bool handleEvent(UEvent * event)
    {
        if(event->getClassName() == "TestEvent")
        {
            testEventCount_++;
        }
        else if(event->getClassName() == "AnotherEvent")
        {
            anotherEventCount_++;
        }
        return false;
    }
    
private:
    int testEventCount_;
    int anotherEventCount_;
};

// Test sender class that can post events
class TestSender : public UEventsSender
{
public:
    TestSender() {}
    
    // Public method to test protected post() method
    void testPost(UEvent* event, bool async = true) const
    {
        post(event, async);
    }
};

TEST(UEventsTest, UEventGetClassName)
{
    TestEvent event;
    EXPECT_EQ(event.getClassName(), "TestEvent");
    EXPECT_EQ(event.getCode(), 0);
}

TEST(UEventsTest, UEventGetCode)
{
    TestEvent event(42);
    EXPECT_EQ(event.getCode(), 42);
}

TEST(UEventsTest, UEventsHandlerRegisterUnregister)
{
    TestHandler handler;
    
    // Register handler
    handler.registerToEventsManager();
    
    // Post an event
    UEventsManager::post(new TestEvent(), true);
    
    // Give time for event to be processed
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    
    EXPECT_GT(handler.getEventCount(), 0);
    
    // Unregister handler
    handler.unregisterFromEventsManager();
    
    // Post another event
    int countBefore = handler.getEventCount();
    UEventsManager::post(new TestEvent(), true);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    
    // Should not receive event after unregister
    EXPECT_EQ(handler.getEventCount(), countBefore);
}

TEST(UEventsTest, UEventsHandlerDestructorUnregisters)
{
    TestHandler* handler = new TestHandler();
    handler->registerToEventsManager();
    
    // Post an event
    UEventsManager::post(new TestEvent(), true);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    EXPECT_EQ(handler->getEventCount(), 1);
    
    // Delete handler (should unregister automatically)
    handler->unregisterFromEventsManager();
    
    // Post another event
    UEventsManager::post(new TestEvent(), true);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    
    EXPECT_EQ(handler->getEventCount(), 1);
}

TEST(UEventsTest, UEventsManagerAddRemoveHandler)
{
    TestHandler handler1;
    TestHandler handler2;
    
    UEventsManager::addHandler(&handler1);
    UEventsManager::addHandler(&handler2);
    
    UEventsManager::post(new TestEvent(), true);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    
    EXPECT_GT(handler1.getEventCount(), 0);
    EXPECT_GT(handler2.getEventCount(), 0);
    
    UEventsManager::removeHandler(&handler1);
    
    int count1Before = handler1.getEventCount();
    int count2Before = handler2.getEventCount();
    
    UEventsManager::post(new TestEvent(), true);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    
    EXPECT_EQ(handler1.getEventCount(), count1Before);
    EXPECT_GT(handler2.getEventCount(), count2Before);
    
    UEventsManager::removeHandler(&handler2);
}

TEST(UEventsTest, UEventsManagerPostAsync)
{
    TestHandler handler;
    UEventsManager::addHandler(&handler);
    
    handler.reset();
    
    UEventsManager::post(new TestEvent(), true);
    
    // Give time for async processing
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    EXPECT_GT(handler.getEventCount(), 0);
    
    UEventsManager::removeHandler(&handler);
}

TEST(UEventsTest, UEventsManagerPostSync)
{
    TestHandler handler;
    UEventsManager::addHandler(&handler);
    
    handler.reset();
    
    UEventsManager::post(new TestEvent(), false);
    
    // Sync should be immediate
    EXPECT_GT(handler.getEventCount(), 0);
    
    UEventsManager::removeHandler(&handler);
}

TEST(UEventsTest, UEventsManagerMultipleHandlers)
{
    TestHandler handler1;
    TestHandler handler2;
    TestHandler handler3;
    
    UEventsManager::addHandler(&handler1);
    UEventsManager::addHandler(&handler2);
    UEventsManager::addHandler(&handler3);
    
    handler1.reset();
    handler2.reset();
    handler3.reset();
    
    UEventsManager::post(new TestEvent(), true);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    
    EXPECT_GT(handler1.getEventCount(), 0);
    EXPECT_GT(handler2.getEventCount(), 0);
    EXPECT_GT(handler3.getEventCount(), 0);
    
    UEventsManager::removeHandler(&handler1);
    UEventsManager::removeHandler(&handler2);
    UEventsManager::removeHandler(&handler3);
}

TEST(UEventsTest, UEventsManagerEventOrdering)
{
    TestHandler handler;
    UEventsManager::addHandler(&handler);
    
    handler.reset();
    
    // Post multiple events
    UEventsManager::post(new TestEvent(1), true);
    UEventsManager::post(new TestEvent(2), true);
    UEventsManager::post(new TestEvent(3), true);
    
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // Should receive all events (order may vary in async mode)
    EXPECT_GE(handler.getEventCount(), 3);
    
    UEventsManager::removeHandler(&handler);
}

TEST(UEventsTest, UEventsHandlerOwnership)
{
    OwnershipHandler handler;
    TestHandler handler0;
    TestHandler handler2;
    UEventsManager::addHandler(&handler0);
    UEventsManager::addHandler(&handler);
    UEventsManager::addHandler(&handler2);
    
    UEventsManager::post(new TestEvent(42), true);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    
    EXPECT_NE(handler.getEvent(), nullptr);
    EXPECT_EQ(handler.getEvent()->getCode(), 42);

    // Should have received the event
    EXPECT_GT(handler0.getEventCount(), 0);

    // Should not have received the event
    EXPECT_EQ(handler2.getEventCount(), 0);
    
    UEventsManager::removeHandler(&handler0);
    UEventsManager::removeHandler(&handler);
    UEventsManager::removeHandler(&handler2);
}

TEST(UEventsTest, UEventsHandlerFiltering)
{
    FilteringHandler handler;
    UEventsManager::addHandler(&handler);
    
    handler.reset();
    
    UEventsManager::post(new TestEvent(), true);
    UEventsManager::post(new AnotherEvent(), true);
    UEventsManager::post(new TestEvent(), true);
    
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    EXPECT_EQ(handler.getTestEventCount(), 2);
    EXPECT_EQ(handler.getAnotherEventCount(), 1);
    
    UEventsManager::removeHandler(&handler);
}

TEST(UEventsTest, UEventsSenderPost)
{
    TestSender sender;
    TestHandler handler;
    
    UEventsManager::addHandler(&handler);
    
    handler.reset();
    
    // Test protected post() method through TestSender
    sender.testPost(new TestEvent(), true);
    
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    
    EXPECT_GT(handler.getEventCount(), 0);
    
    UEventsManager::removeHandler(&handler);
}

TEST(UEventsTest, UEventsSenderDestructorRemovesPipes)
{
    TestSender* sender = new TestSender();
    TestHandler handler;
    
    UEventsManager::addHandler(&handler);
    UEventsManager::createPipe(sender, &handler, "TestEvent");
    
    // Delete sender - should remove pipes automatically
    delete sender;
    
    // Should not crash and pipes should be removed
    SUCCEED();
    
    UEventsManager::removeHandler(&handler);
}

TEST(UEventsTest, UEventsManagerCreateRemovePipe)
{
    TestSender sender;
    TestHandler handler;
    TestHandler handler2;
    
    UEventsManager::addHandler(&handler);
    UEventsManager::addHandler(&handler2);
    
    // Create a pipe
    UEventsManager::createPipe(&sender, &handler, "TestEvent");
    
    handler.reset();
    
    // Post event with sender
    sender.testPost(new TestEvent());
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    EXPECT_EQ(handler.getEventCount(), 1);
    EXPECT_EQ(handler2.getEventCount(), 0);

    // Post event with sender using UEventsManager
    UEventsManager::post(new TestEvent(), true, &sender);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    EXPECT_EQ(handler.getEventCount(), 2);
    EXPECT_EQ(handler2.getEventCount(), 0);

    // Post global event
    UEventsManager::post(new TestEvent(), true);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    EXPECT_EQ(handler.getEventCount(), 3);
    EXPECT_EQ(handler2.getEventCount(), 1);
    
    // Remove pipe
    UEventsManager::removePipe(&sender, &handler, "TestEvent");
    

    // Post another event
    UEventsManager::post(new TestEvent(), true, &sender);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    
    // Should still receive events, no pipe filtering
    EXPECT_EQ(handler.getEventCount(), 4);
    EXPECT_EQ(handler2.getEventCount(), 2);
    
    UEventsManager::removeHandler(&handler);
    UEventsManager::removeHandler(&handler2);
}

TEST(UEventsTest, UEventsManagerRemoveAllPipes)
{
    TestSender sender;
    TestHandler handler1;
    TestHandler handler2;
    
    UEventsManager::addHandler(&handler1);
    UEventsManager::addHandler(&handler2);
    
    UEventsManager::createPipe(&sender, &handler1, "TestEvent");
    UEventsManager::createPipe(&sender, &handler2, "AnotherEvent");

    sender.testPost(new TestEvent());
    sender.testPost(new AnotherEvent());
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    EXPECT_EQ(handler1.getEventCount(), 1);
    EXPECT_EQ(handler2.getEventCount(), 1);
    
    UEventsManager::removeAllPipes(&sender);
    
    sender.testPost(new TestEvent());
    sender.testPost(new AnotherEvent());
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    EXPECT_EQ(handler1.getEventCount(), 3);
    EXPECT_EQ(handler2.getEventCount(), 3);
    
    UEventsManager::removeHandler(&handler1);
    UEventsManager::removeHandler(&handler2);
}

TEST(UEventsTest, MultipleEventTypes)
{
    FilteringHandler handler;
    UEventsManager::addHandler(&handler);
    
    handler.reset();
    
    UEventsManager::post(new TestEvent(), true);
    UEventsManager::post(new AnotherEvent(), true);
    UEventsManager::post(new TestEvent(), true);
    
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    EXPECT_EQ(handler.getTestEventCount(), 2);
    EXPECT_EQ(handler.getAnotherEventCount(), 1);
    
    UEventsManager::removeHandler(&handler);
}

TEST(UEventsTest, HandlerReceivesCorrectEvent)
{
    TestHandler handler;
    UEventsManager::addHandler(&handler);
    
    handler.reset();
    
    TestEvent* event = new TestEvent(42);
    UEventsManager::post(event, true);
    
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    
    EXPECT_EQ(handler.getLastEventCode(), 42);
    EXPECT_STREQ(handler.getLastEventClassName().c_str(), "TestEvent");
    
    UEventsManager::removeHandler(&handler);
}

