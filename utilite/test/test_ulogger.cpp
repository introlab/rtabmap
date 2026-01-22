#include "gtest/gtest.h"
#include "rtabmap/utilite/ULogger.h"
#include "rtabmap/utilite/UFile.h"
#include "rtabmap/utilite/UThread.h"
#include "rtabmap/utilite/UStl.h"
#include <fstream>
#include <sstream>
#include <thread>

TEST(ULoggerTest, DefaultState)
{
    ULogger::reset();
    
    EXPECT_EQ(ULogger::type(), ULogger::kTypeNoLog);
    EXPECT_EQ(ULogger::level(), ULogger::kInfo);
    EXPECT_TRUE(ULogger::isPrintTime());
    EXPECT_TRUE(ULogger::isPrintLevel());
    EXPECT_TRUE(ULogger::isPrintEndLine());
    EXPECT_TRUE(ULogger::isPrintColored());
    EXPECT_TRUE(ULogger::isPrintWhere());
    EXPECT_FALSE(ULogger::isPrintWhereFullPath());
    EXPECT_FALSE(ULogger::isPrintThreadId());
    EXPECT_FALSE(ULogger::isBuffered());
}

TEST(ULoggerTest, SetType)
{
    ULogger::setType(ULogger::kTypeConsole);
    EXPECT_EQ(ULogger::type(), ULogger::kTypeConsole);
    
    ULogger::setType(ULogger::kTypeNoLog);
    EXPECT_EQ(ULogger::type(), ULogger::kTypeNoLog);
    
    ULogger::setType(ULogger::kTypeFile, "test_log.txt", false);
    EXPECT_EQ(ULogger::type(), ULogger::kTypeFile);
    
    // Cleanup
    UFile::erase("test_log.txt");
}

TEST(ULoggerTest, SetLevel)
{
    ULogger::setLevel(ULogger::kDebug);
    EXPECT_EQ(ULogger::level(), ULogger::kDebug);
    
    ULogger::setLevel(ULogger::kInfo);
    EXPECT_EQ(ULogger::level(), ULogger::kInfo);
    
    ULogger::setLevel(ULogger::kWarning);
    EXPECT_EQ(ULogger::level(), ULogger::kWarning);
    
    ULogger::setLevel(ULogger::kError);
    EXPECT_EQ(ULogger::level(), ULogger::kError);
    
    ULogger::setLevel(ULogger::kFatal);
    EXPECT_EQ(ULogger::level(), ULogger::kFatal);
}

TEST(ULoggerTest, LevelFiltering)
{
    ULogger::reset();
    std::string testFile = "logger_level_filtering_test.txt";
    ULogger::setType(ULogger::kTypeFile, testFile, false);

    ULogger::setLevel(ULogger::kInfo);
    
    // Debug should not be logged
    UDEBUG("This debug message should not appear");
    
    // Info and above should be logged
    UINFO("This info message should appear");
    UWARN("This warning message should appear");
    UERROR("This error message should appear");

    // Make sure the log file is closed
    ULogger::setType(ULogger::kTypeNoLog);

    std::ifstream file(testFile);
    std::string fileContent(
        (std::istreambuf_iterator<char>(file)), // Start iterator
        std::istreambuf_iterator<char>()        // End iterator
    );
    file.close();

    EXPECT_FALSE(uStrContains(fileContent, "DEBUG"));
    EXPECT_TRUE(uStrContains(fileContent, "INFO"));
    EXPECT_TRUE(uStrContains(fileContent, "WARN"));
    EXPECT_TRUE(uStrContains(fileContent, "ERROR"));
}

TEST(ULoggerTest, SetPrintTime)
{
    ULogger::reset();

    std::string testFile = "logger_print_time_test.txt";
    ULogger::setType(ULogger::kTypeFile, testFile, false);

    ULogger::setPrintTime(false);
    EXPECT_FALSE(ULogger::isPrintTime());
    UINFO("This info message should appear without time");
    
    ULogger::setPrintTime(true);
    EXPECT_TRUE(ULogger::isPrintTime());
    UINFO("This info message should appear with time");

    // Make sure the log file is closed
    ULogger::setType(ULogger::kTypeNoLog);

    std::ifstream file(testFile);
    std::string line;
    EXPECT_TRUE(file.is_open());

    // First line should not have date
    std::getline(file, line);
    EXPECT_EQ(uSplit(line, '-').size(), 1);
    // Time is format YYYY-MM-DD HH:MM:SS
    std::getline(file, line);
    EXPECT_EQ(uSplit(line, '-').size(), 3);
    file.close(); 
}

TEST(ULoggerTest, SetPrintLevel)
{
    ULogger::reset();

    std::string testFile = "logger_print_level_test.txt";
    ULogger::setType(ULogger::kTypeFile, testFile, false);

    ULogger::setPrintLevel(false);
    EXPECT_FALSE(ULogger::isPrintLevel());
    UINFO("This info message should appear without level");
    
    ULogger::setPrintLevel(true);
    EXPECT_TRUE(ULogger::isPrintLevel());
    UINFO("This info message should appear with level");

    // Make sure the log file is closed
    ULogger::setType(ULogger::kTypeNoLog);

    std::ifstream file(testFile);
    std::string line;
    EXPECT_TRUE(file.is_open());

    // First line should not have level
    std::getline(file, line);
    EXPECT_EQ(uSplit(line, ']').size(), 1);
    // Level format: [INFO] msg...
    std::getline(file, line);
    EXPECT_EQ(uSplit(line, ']').size(), 2);
    file.close(); 
}

TEST(ULoggerTest, SetPrintEndline)
{
    ULogger::reset();
    std::string testFile = "logger_print_endline_test.txt";
    ULogger::setType(ULogger::kTypeFile, testFile, false);
    
    ULogger::setPrintEndline(false);
    EXPECT_FALSE(ULogger::isPrintEndLine());
    UINFO("This info message should appear first. ");
    
    ULogger::setPrintEndline(true);
    EXPECT_TRUE(ULogger::isPrintEndLine());
    UINFO("This info message should appear on same line than previous msg.");

    // Make sure the log file is closed
    ULogger::setType(ULogger::kTypeNoLog);

    std::ifstream file(testFile);
    std::string line;
    EXPECT_TRUE(file.is_open());

    // First line should not have level
    std::getline(file, line);
    EXPECT_FALSE(line.empty());
    std::getline(file, line);
    EXPECT_TRUE(line.empty());
    file.close(); 
}

TEST(ULoggerTest, SetPrintColored)
{
    ULogger::reset();
    
    ULogger::setPrintColored(false);
    EXPECT_FALSE(ULogger::isPrintColored());
    
    ULogger::setPrintColored(true);
    EXPECT_TRUE(ULogger::isPrintColored());
}

TEST(ULoggerTest, SetPrintWhere)
{
    ULogger::reset();
    std::string testFile = "logger_print_where_test.txt";
    ULogger::setType(ULogger::kTypeFile, testFile, false);
    
    ULogger::setPrintWhere(false);
    EXPECT_FALSE(ULogger::isPrintWhere());
    UINFO("This info message should appear without where");
    
    ULogger::setPrintWhere(true);
    EXPECT_TRUE(ULogger::isPrintWhere());
    UINFO("This info message should appear with where");

    // Make sure the log file is closed
    ULogger::setType(ULogger::kTypeNoLog);

    std::ifstream file(testFile);
    std::string line;
    EXPECT_TRUE(file.is_open());

    // First line should not have where (no test_ulogger.cpp)
    std::getline(file, line);
    EXPECT_FALSE(uStrContains(line, "test_ulogger.cpp"));
    std::getline(file, line);
    EXPECT_TRUE(uStrContains(line, "test_ulogger.cpp"));
    file.close(); 
}

TEST(ULoggerTest, SetPrintWhereFullPath)
{
    ULogger::reset();
    std::string testFile = "logger_print_where_full_test.txt";
    ULogger::setType(ULogger::kTypeFile, testFile, false);
    
    ULogger::setPrintWhereFullPath(false);
    EXPECT_FALSE(ULogger::isPrintWhereFullPath());
    UINFO("This info message should appear without full path");
    
    ULogger::setPrintWhereFullPath(true);
    EXPECT_TRUE(ULogger::isPrintWhereFullPath());
    UINFO("This info message should appear with full path");

    // Make sure the log file is closed
    ULogger::setType(ULogger::kTypeNoLog);

    std::ifstream file(testFile);
    std::string line;
    EXPECT_TRUE(file.is_open());

    // First line should not have full path (without / or \\)
    std::getline(file, line);
    EXPECT_TRUE(!uStrContains(line, "/") && !uStrContains(line, "\\"));
    std::getline(file, line);
    EXPECT_TRUE(uStrContains(line, "/") || uStrContains(line, "\\"));
    file.close(); 
}

TEST(ULoggerTest, SetPrintThreadId)
{
    ULogger::reset();
    std::string testFile = "logger_print_thread_id_test.txt";
    ULogger::setType(ULogger::kTypeFile, testFile, false);
    
    ULogger::setPrintThreadId(false);
    EXPECT_FALSE(ULogger::isPrintThreadId());
    UINFO("This info message should appear without thread ID");
    
    ULogger::setPrintThreadId(true);
    EXPECT_TRUE(ULogger::isPrintThreadId());
    UINFO("This info message should appear with thread ID");

    // Make sure the log file is closed
    ULogger::setType(ULogger::kTypeNoLog);

    std::ifstream file(testFile);
    std::string line;
    EXPECT_TRUE(file.is_open());

    // First line should not have thread ID (without {})
    std::getline(file, line);
    EXPECT_TRUE(!uStrContains(line, "{") && !uStrContains(line, "}"));
    std::getline(file, line);
    EXPECT_TRUE(uStrContains(line, "{") && uStrContains(line, "}"));
    file.close(); 
}

TEST(ULoggerTest, SetBuffered)
{
    ULogger::reset();
    ULogger::setBuffered(true);
    EXPECT_TRUE(ULogger::isBuffered());
    
    ULogger::setBuffered(false);
    EXPECT_FALSE(ULogger::isBuffered());
}

TEST(ULoggerTest, FileLogging)
{
    ULogger::reset();
    std::string testFile = "test_ulogger_file.txt";
    
    // Remove file if exists
    if(UFile::exists(testFile))
    {
        UFile::erase(testFile);
    }
    
    ULogger::setType(ULogger::kTypeFile, testFile, false);
    ULogger::setLevel(ULogger::kDebug);
    
    UINFO("Test message 1");
    UWARN("Test message 2");
    UERROR("Test message 3");
    
    // Make sure everything is written to file
    ULogger::setType(ULogger::kTypeNoLog);
    
    // Check file exists and has content
    EXPECT_TRUE(UFile::exists(testFile));
    EXPECT_GT(UFile::length(testFile), 0);
}

TEST(ULoggerTest, FileLoggingAppend)
{
    ULogger::reset();
    std::string testFile = "test_ulogger_append.txt";
    
    // Remove file if exists
    if(UFile::exists(testFile))
    {
        UFile::erase(testFile);
    }
    
    ULogger::setType(ULogger::kTypeFile, testFile, false);
    ULogger::setLevel(ULogger::kInfo);
    
    UINFO("First message");
    // Make sure everything is written to file
    ULogger::setType(ULogger::kTypeNoLog);
    
    long length1 = UFile::length(testFile);
    
    // Append mode
    ULogger::setType(ULogger::kTypeFile, testFile, true);
    UINFO("Second message");
    // Make sure everything is written to file
    ULogger::setType(ULogger::kTypeNoLog);
    
    long length2 = UFile::length(testFile);
    EXPECT_GT(length2, length1);
}

TEST(ULoggerTest, BufferedLogging)
{
    ULogger::reset();
    std::string testFile = "test_ulogger_buffered.txt";
    
    // Remove file if exists
    if(UFile::exists(testFile))
    {
        UFile::erase(testFile);
    }
    
    ULogger::setType(ULogger::kTypeFile, testFile, false);
    ULogger::setBuffered(true);
    ULogger::setLevel(ULogger::kInfo);
    
    UINFO("Buffered message 1");
    UINFO("Buffered message 2");
    
    // File should be empty before flush
    EXPECT_EQ(UFile::length(testFile), 0);
    
    // Make sure everything is written to file
    ULogger::setType(ULogger::kTypeNoLog);
    
    // File should have content after flush
    EXPECT_GT(UFile::length(testFile), 0);
}

TEST(ULoggerTest, GetTime)
{
    std::string timeStr;
    int result = ULogger::getTime(timeStr);
    
    EXPECT_GT(result, 0);
    EXPECT_GT(timeStr.size(), 0u);
    
    // Time string should contain date/time format
    // Format is typically "YYYY-MM-DD HH:MM:SS" or similar
    EXPECT_NE(timeStr.find("-"), std::string::npos);
}

TEST(ULoggerTest, SetEventLevel)
{
    ULogger::reset();

    ULogger::setEventLevel(ULogger::kDebug);
    EXPECT_EQ(ULogger::eventLevel(), ULogger::kDebug);

    ULogger::setEventLevel(ULogger::kInfo);
    EXPECT_EQ(ULogger::eventLevel(), ULogger::kInfo);

    ULogger::setEventLevel(ULogger::kWarning);
    EXPECT_EQ(ULogger::eventLevel(), ULogger::kWarning);
    
    ULogger::setEventLevel(ULogger::kError);
    EXPECT_EQ(ULogger::eventLevel(), ULogger::kError);
}

TEST(ULoggerTest, RegisterCurrentThread)
{
    ULogger::reset();
    ULogger::registerCurrentThread("TestThread");
    
    std::map<std::string, unsigned long> threads = ULogger::getRegisteredThreads();
    EXPECT_GT(threads.size(), 0u);
    EXPECT_NE(threads.find("TestThread"), threads.end());
    
    ULogger::unregisterCurrentThread();
    
    threads = ULogger::getRegisteredThreads();
    EXPECT_EQ(threads.find("TestThread"), threads.end());
}

TEST(ULoggerTest, ThreadIdFilter)
{
    ULogger::reset();

    unsigned long currentId = UThread::currentThreadId();
    std::set<unsigned long> filter;
    filter.insert(currentId);
    
    ULogger::setTreadIdFilter(filter);
    
    const std::set<unsigned long>& retrieved = ULogger::getTreadIdFilter();
    EXPECT_EQ(retrieved.size(), 1u);
    EXPECT_NE(retrieved.find(currentId), retrieved.end());
}

class OtherThread : public UThread
{
public:
    OtherThread() : logOnce_(false) {}
    std::string name() {return "OtherThread";}
    void logOnce() {logOnce_ = true;}
protected:
    virtual void mainLoopBegin() {ULogger::registerCurrentThread(name());}
    virtual void mainLoopEnd() {ULogger::unregisterCurrentThread();}
    virtual void mainLoop() {
        uSleep(10);
        if(logOnce_)
        {
            UINFO("Logging from OtherThread");
            logOnce_ = false;
        }
    }
private:
    bool logOnce_;
};

TEST(ULoggerTest, ThreadIdFilterVector)
{
    ULogger::reset();

    // Both threads are registering to logger
    ULogger::registerCurrentThread("FilterThread");
    OtherThread otherThread;
    otherThread.start();
    
    std::vector<std::string> threadNames;
    threadNames.push_back(otherThread.name());
    threadNames.push_back("FilterThread");
    
    ULogger::setTreadIdFilter(threadNames);
    
    const std::set<unsigned long>& retrieved = ULogger::getTreadIdFilter();
    EXPECT_EQ(retrieved.size(), 2u);

    std::string testFile = "logger_thread_log_filtering_test.txt";
    ULogger::setType(ULogger::kTypeFile, testFile, false);
    UINFO("This log should appear");

    threadNames.pop_back();
    ULogger::setTreadIdFilter(threadNames);
    UINFO("This log should not appear");

    otherThread.logOnce();
    otherThread.join(true);

    // Make sure the log file is closed
    ULogger::setType(ULogger::kTypeNoLog);

    std::ifstream file(testFile);
    std::string line;
    EXPECT_TRUE(file.is_open());

    std::getline(file, line);
    EXPECT_TRUE(uStrContains(line, "This log should appear"));
    std::getline(file, line);
    EXPECT_TRUE(uStrContains(line, "Logging from OtherThread"));
    file.close(); 
    
    ULogger::unregisterCurrentThread();
}

TEST(ULoggerTest, WriteWithLevel)
{
    ULogger::reset();

    std::string testFile = "logger_write_level_test.txt";
    ULogger::setType(ULogger::kTypeFile, testFile, false);

    ULogger::setLevel(ULogger::kDebug);
    
    // Test write with different levels
    ULogger::write(ULogger::kDebug, __FILE__, __LINE__, __FUNCTION__, "Debug message %d", 42);
    ULogger::write(ULogger::kInfo, __FILE__, __LINE__, __FUNCTION__, "Info message %s", "test");
    ULogger::write(ULogger::kWarning, __FILE__, __LINE__, __FUNCTION__, "Warning message");
    ULogger::write(ULogger::kError, __FILE__, __LINE__, __FUNCTION__, "Error message");

    // Make sure the log file is closed
    ULogger::setType(ULogger::kTypeNoLog);

    std::ifstream file(testFile);
    std::string line;
    EXPECT_TRUE(file.is_open());

    std::getline(file, line);
    EXPECT_TRUE(uStrContains(line, "[DEBUG]"));
    EXPECT_TRUE(uStrContains(line, "Debug message 42"));
    std::getline(file, line);
    EXPECT_TRUE(uStrContains(line, "[ INFO]"));
    EXPECT_TRUE(uStrContains(line, "Info message test"));
    std::getline(file, line);
    EXPECT_TRUE(uStrContains(line, "[ WARN]"));
    EXPECT_TRUE(uStrContains(line, "Warning message"));
    std::getline(file, line);
    EXPECT_TRUE(uStrContains(line, "[ERROR]"));
    EXPECT_TRUE(uStrContains(line, "Error message"));
    file.close(); 
}

TEST(ULoggerTest, Macros)
{
    ULogger::reset();
    std::string testFile = "logger_macros_test.txt";
    ULogger::setType(ULogger::kTypeFile, testFile, false);

    ULogger::setLevel(ULogger::kDebug);
    
    UDEBUG("Debug macro message");
    UINFO("Info macro message");
    UWARN("Warning macro message");
    UERROR("Error macro message");
    
    // Make sure the log file is closed
    ULogger::setType(ULogger::kTypeNoLog);

    std::ifstream file(testFile);
    std::string line;
    EXPECT_TRUE(file.is_open());

    std::getline(file, line);
    EXPECT_TRUE(uStrContains(line, "[DEBUG]"));
    EXPECT_TRUE(uStrContains(line, "Debug macro message"));
    std::getline(file, line);
    EXPECT_TRUE(uStrContains(line, "[ INFO]"));
    EXPECT_TRUE(uStrContains(line, "Info macro message"));
    std::getline(file, line);
    EXPECT_TRUE(uStrContains(line, "[ WARN]"));
    EXPECT_TRUE(uStrContains(line, "Warning macro message"));
    std::getline(file, line);
    EXPECT_TRUE(uStrContains(line, "[ERROR]"));
    EXPECT_TRUE(uStrContains(line, "Error macro message"));
    file.close(); 
}

TEST(ULoggerTest, Reset)
{
    ULogger::reset();
    // Change some settings
    ULogger::setType(ULogger::kTypeConsole);
    ULogger::setLevel(ULogger::kDebug);
    ULogger::setPrintTime(false);
    ULogger::setPrintLevel(false);
    ULogger::setBuffered(true);
    
    // Reset
    ULogger::reset();
    
    // Should be back to defaults
    EXPECT_EQ(ULogger::type(), ULogger::kTypeNoLog);
    EXPECT_EQ(ULogger::level(), ULogger::kInfo);
    EXPECT_TRUE(ULogger::isPrintTime());
    EXPECT_TRUE(ULogger::isPrintLevel());
    EXPECT_FALSE(ULogger::isBuffered());
}

TEST(ULoggerTest, MultipleLogLevels)
{
    ULogger::reset();
    std::string testFile = "logger_multiple_levels_test.txt";
    ULogger::setType(ULogger::kTypeFile, testFile, false);
    
    // Test all levels
    ULogger::setLevel(ULogger::kDebug);
    UDEBUG("Debug");
    UINFO("Info");
    UWARN("Warning");
    UERROR("Error");
    
    ULogger::setLevel(ULogger::kInfo);
    UDEBUG("Debug - should not appear");
    UINFO("Info");
    UWARN("Warning");
    UERROR("Error");
    
    ULogger::setLevel(ULogger::kWarning);
    UDEBUG("Debug - should not appear");
    UINFO("Info - should not appear");
    UWARN("Warning");
    UERROR("Error");

    ULogger::setLevel(ULogger::kError);
    UDEBUG("Debug - should not appear");
    UINFO("Info - should not appear");
    UWARN("Warning - should not appear");
    UERROR("Error");
    
    // Make sure the log file is closed
    ULogger::setType(ULogger::kTypeNoLog);

    std::ifstream file(testFile);
    std::string line;
    EXPECT_TRUE(file.is_open());

    int count = 0;
    std::getline(file, line);
    while(!line.empty())
    {
        EXPECT_TRUE(!uStrContains(line, "should not appear"));
        ++count;
        std::getline(file, line);
    }
    EXPECT_EQ(count, 10);
    file.close(); 
}

TEST(ULoggerTest, FileContent)
{
    ULogger::reset();
    std::string testFile = "test_ulogger_content.txt";
    
    ULogger::setType(ULogger::kTypeFile, testFile, false);
    ULogger::setLevel(ULogger::kInfo);
    ULogger::setPrintTime(false);
    ULogger::setPrintLevel(true);
    ULogger::setPrintWhere(false);
    
    UINFO("Test content message");
    ULogger::setType(ULogger::kTypeNoLog);
    
    // Read file and check content
    std::ifstream file(testFile);
    std::string content((std::istreambuf_iterator<char>(file)),
                        std::istreambuf_iterator<char>());
    file.close();
    
    EXPECT_NE(content.find("Test content message"), std::string::npos);
}

TEST(ULoggerTest, ThreadSafety)
{
    ULogger::reset();
    std::string testFile = "logger_thread_safety_test.txt";
    ULogger::setType(ULogger::kTypeFile, testFile, false);

    // Disable all printing to easyly test results below
    ULogger::setPrintTime(false);
    ULogger::setPrintLevel(false);
    ULogger::setPrintWhere(false);
    ULogger::setPrintWhereFullPath(false);
    ULogger::setPrintColored(false);
    ULogger::setPrintThreadId(false);
    
    const int numThreads = 5;
    const int messagesPerThread = 9;
    
    std::vector<std::thread> threads;
    
    for(int i = 0; i < numThreads; ++i)
    {
        threads.emplace_back([i, messagesPerThread]() {
            for(int j = 0; j < messagesPerThread; ++j)
            {
                UINFO("Thread %d message %d", i, j);
            }
        });
    }
    
    for(auto& t : threads)
    {
        t.join();
    }

    // Make sure the log file is closed
    ULogger::setType(ULogger::kTypeNoLog);

    std::ifstream file(testFile);
    std::string line;
    EXPECT_TRUE(file.is_open());

    int count = 0;
    std::getline(file, line);
    while(!line.empty())
    {
        // Expect all messages on single line, no interleaved, format: "Thread X message X"
        EXPECT_EQ(line.length(), 19);
        ++count;
        std::getline(file, line);
    }
    EXPECT_EQ(count, 45);
    file.close(); 
}

TEST(ULoggerTest, DefaultLogFileName)
{
    EXPECT_EQ(ULogger::kDefaultLogFileName, "./ULog.txt");
}

TEST(ULoggerTest, AllPrintOptions)
{
    ULogger::reset();
    std::string testFile = "logger_all_print_options_test.txt";
    ULogger::setType(ULogger::kTypeFile, testFile, false);
    
    // Test with all print options enabled
    ULogger::setPrintTime(true);
    ULogger::setPrintLevel(true);
    ULogger::setPrintWhereFullPath(true);
    ULogger::setPrintColored(true);
    ULogger::setPrintThreadId(true);
    
    UINFO("Message with all options");
    
    // Test with all print options disabled
    ULogger::setPrintTime(false);
    ULogger::setPrintLevel(false);
    ULogger::setPrintWhere(false);
    ULogger::setPrintWhereFullPath(false);
    ULogger::setPrintColored(false);
    ULogger::setPrintThreadId(false);
    
    UINFO("Message with no options");
    
    // Make sure the log file is closed
    ULogger::setType(ULogger::kTypeNoLog);

    std::ifstream file(testFile);
    std::string line;
    EXPECT_TRUE(file.is_open());

    std::getline(file, line);
    EXPECT_TRUE(uStrContains(line, "["));
    EXPECT_TRUE(uStrContains(line, "{"));
    EXPECT_TRUE(uStrContains(line, "-"));
    EXPECT_TRUE(uStrContains(line, ".cpp"));
    EXPECT_TRUE(uStrContains(line, "/") || uStrContains(line, "\\"));
   
    std::getline(file, line);
    EXPECT_FALSE(uStrContains(line, "["));
    EXPECT_FALSE(uStrContains(line, "{"));
    EXPECT_FALSE(uStrContains(line, "-"));
    EXPECT_FALSE(uStrContains(line, ".cpp"));
    EXPECT_FALSE(uStrContains(line, "\\"));
    EXPECT_FALSE(uStrContains(line, "/"));

    file.close(); 
}

