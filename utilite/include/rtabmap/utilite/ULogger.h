/*
*  utilite is a cross-platform library with
*  useful utilities for fast and small developing.
*  Copyright (C) 2010  Mathieu Labbe
*
*  utilite is free library: you can redistribute it and/or modify
*  it under the terms of the GNU Lesser General Public License as published by
*  the Free Software Foundation, either version 3 of the License, or
*  (at your option) any later version.
*
*  utilite is distributed in the hope that it will be useful,
*  but WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*  GNU Lesser General Public License for more details.
*
*  You should have received a copy of the GNU Lesser General Public License
*  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef ULOGGER_H
#define ULOGGER_H

#include "rtabmap/utilite/utilite_export.h" // DLL export/import defines

#include "rtabmap/utilite/UMutex.h"
#include "rtabmap/utilite/UDestroyer.h"
#include "rtabmap/utilite/UEvent.h"
#include "rtabmap/utilite/UException.h"

#include <stdio.h>
#include <time.h>
#include <string>
#include <vector>
#include <map>
#include <set>

#include <stdarg.h>

/**
 * \file ULogger.h
 * \brief ULogger class and convenient macros
 *
 * This contains macros useful for logging a message anywhere in the
 * application. Once the ULogger is set, use these macros like a printf to
 * print debug messages.
*/

/*
 * Convenient macros for logging...
 */
#define ULOGGER_LOG(level, ...) ULogger::write(level, __FILE__, __LINE__, __FUNCTION__, __VA_ARGS__)

#define ULOGGER_DEBUG(...)   ULOGGER_LOG(ULogger::kDebug,   __VA_ARGS__)
#define ULOGGER_INFO(...)    ULOGGER_LOG(ULogger::kInfo,    __VA_ARGS__)
#define ULOGGER_WARN(...) 	 ULOGGER_LOG(ULogger::kWarning, __VA_ARGS__)
#define ULOGGER_ERROR(...)   ULOGGER_LOG(ULogger::kError,   __VA_ARGS__)
#define ULOGGER_FATAL(...)   ULOGGER_LOG(ULogger::kFatal,   __VA_ARGS__) // Throw UException

#define UDEBUG(...)   ULOGGER_DEBUG(__VA_ARGS__)
#define UINFO(...)    ULOGGER_INFO(__VA_ARGS__)
#define UWARN(...) 	  ULOGGER_WARN(__VA_ARGS__)
#define UERROR(...)   ULOGGER_ERROR(__VA_ARGS__)
#define UFATAL(...)   ULOGGER_FATAL(__VA_ARGS__) // Throw UException

// Throw UException
#define UASSERT(condition) if(!(condition)) ULogger::write(ULogger::kFatal, __FILE__, __LINE__, __FUNCTION__, "Condition (%s) not met!", #condition)
#define UASSERT_MSG(condition, msg_str) if(!(condition)) ULogger::write(ULogger::kFatal, __FILE__, __LINE__, __FUNCTION__, "Condition (%s) not met! [%s]", #condition, msg_str)

/**
 * \def UDEBUG(...)
 * Print a debug level message in the logger. Format is the same as a printf:
 * @code
 * UDEBUG("This is a debug message with the number %d", 42);
 * @endcode
 */
/**
 * \def UINFO(...)
 * Print a information level message in the logger. Format is the same as a printf:
 * @code
 * UINFO("This is a information message with the number %d", 42);
 * @endcode
 */
/**
 * \def UWARN(...)
 * Print a warning level message in the logger. Format is the same as a printf:
 * @code
 * UWARN("This is a warning message with the number %d", 42);
 * @endcode
 */
/**
 * \def UERROR(...)
 * Print an error level message in the logger. Format is the same as a printf:
 * @code
 * UERROR("This is an error message with the number %d", 42);
 * @endcode
 */
/**
 * \def UFATAL(...)
 * Print a fatal error level message in the logger. The application will exit on
 * fatal error. Format is the same as a printf:
 * @code
 * UFATAL("This is a fatal error message with the number %d", 42);
 * @endcode
 */
/**
 * \def UASSERT(condition, ...)
 * Print a fatal error level message in the logger if condition is not met. The application will exit on
 * fatal error. Format is the same as a printf:
 * @code
 * UASSERT(a!=42, "This is a fatal error message with the number %d", 42);
 * @endcode
 */



/**
 * This class is used by the ULogger to send logged messages like events. Messages with level
 * over the event level set in ULogger::setEventLevel() are sent like ULogEvent with the message and its level.
 * The default event level of ULogger is kFatal (see ULogger::Level).
 */
class ULogEvent : public UEvent
{
public:
	/**
	 * ULogEvent constructor. Note that to retrieve the message level, use UEvent::getCode().
	 * @param msg the message already formatted to a full string.
	 * @param level the severity of the message, @see ULogger::Level.
	 */
	ULogEvent(const std::string & msg, int level) :
		UEvent(level),
		msg_(msg)
	{}
	virtual ~ULogEvent() {}
	/**
	 * Get the message from the event.
	 */
	const std::string & getMsg() const {return msg_;}
	/**
	 * @return string "ULogEvent"
	 */
	virtual std::string getClassName() const {return "ULogEvent";}
private:
	std::string msg_;
};

/**
 * This class is used to log messages with time on a console, in a file 
 * and/or with an event. At the start of the application, call
 * ULogger::setType() with the type of the logger you want (see ULogger::Type, the type of the output
 * can be changed at the run-time.). To use it,
 * simply call the convenient macros UDEBUG(), UINFO(), UWARN(), UERROR(), UFATAL() depending of
 * the severity of the message. You can disable some messages by setting the logger
 * level ULogger::setLevel() to severity you want, defined by ULogger::Level. A fatal message
 * will make the application to exit, printing the message on console (whatever the logger type) and
 * posting a ULogEvent (synchronously... see UEventsManager::post()) before exiting.
 *
 * The display of the logged messages can be modified:
 * - If you don't want the level label, set ULogger::setPrintLevel() to false.
 * - If you don't want the time label, set ULogger::setPrintTime() to false.
 * - If you don't want the end of line added, set ULogger::setPrintEndline() to false.
 * - If you don't the full path of the message, set ULogger::setPrintWhereFullPath() to false.
 * - If you don't the path of the message, set ULogger::setPrintWhere() to false.
 *
 * When using a file logger (kTypeLogger), it can be useful in some
 * application to buffer messages before writing them to hard drive (avoiding
 * hard drive latencies). You can set ULogger::setBuffered() to true to do that. When the
 * buffered messages will be written to file on appllciation exit (ULogger destructor) or when
 * ULogger::flush() is called.
 *
 * If you want the application to exit on a lower severity level than kFatal,
 * you can set ULogger::setExitLevel() to any ULogger::Type you want.
 *
 * Example:
 * @code
 * #include <utilite/ULogger.h>
 * int main(int argc, char * argv[])
 * {
 *    // Set the logger type. The choices are kTypeConsole,
 *    // kTypeFile or kTypeNoLog (nothing is logged).
 *    ULogger::setType(ULogger::kTypeConsole);
 *
 *    // Set the logger severity level (kDebug, kInfo, kWarning, kError, kFatal).
 *    // All log entries under the severity level are not logged. Here,
 *    // only debug messages are not logged.
 *    ULogger::setLevel(ULogger::kInfo);
 *
 *    // Use a predefined Macro to easy logging. It can be
 *    // called anywhere in the application as the logger is
 *    // a Singleton.
 *    UDEBUG("This message won't be logged because the "
 *           "severity level of the logger is set to kInfo.");
 *
 *    UINFO("This message is logged.");
 *
 *    UWARN("A warning message...");
 *
 *    UERROR("An error message with code %d.", 42);
 *
 *    return 0;
 * }
 * @endcode
 * Output:
 * @code
 * [ INFO] (2010-09-25 18:08:20) main.cpp:18::main() This message is logged.
 * [ WARN] (2010-09-25 18:08:20) main.cpp:20::main() A warning message...
 * [ERROR] (2010-09-25 18:08:20) main.cpp:22::main() An error message with code 42.
 * @endcode
 *
 * Another useful form of the ULogger is to use it with the UTimer class. Here an example:
 * @code
 * #include <utilite/ULogger.h>
 * #include <utilite/UTimer.h>
 * ...
 * UTimer timer; // automatically starts
 * // do some works for part A
 * UINFO("Time for part A = %f s", timer.ticks());
 * // do some works for part B
 * UINFO("Time for part B = %f s", timer.ticks());
 * // do some works for part C
 * UINFO("Time for part C = %f s", timer.ticks());
 * ...
 * @endcode
 *
 * @see setType()
 * @see setLevel()
 * @see UDEBUG(), UINFO(), UWARN(), UERROR(), UFATAL()
 *
 */
class UTILITE_EXPORT ULogger
{

public:
    /**
     * The default log file name.
     */
    static const std::string kDefaultLogFileName;

    /**
     * Loggers available:
     * @code
     * kTypeNoLog, kTypeConsole, kTypeFile
     * @endcode
     */
    enum Type{kTypeNoLog, kTypeConsole, kTypeFile};

    /**
     * Logger levels, from lowest severity to highest:
     * @code
     * kDebug, kInfo, kWarning, kError, kFatal
     * @endcode
     */
    enum Level{kDebug, kInfo, kWarning, kError, kFatal};

    /**
     * Set the type of the logger. When using kTypeFile, the parameter "fileName" would be
     * changed (default is "./ULog.txt"), and optionally "append" if we want the
     * logger to append messages to file or to overwrite the file.
     * @param type the ULogger::Type of the logger.
     * @param fileName file name used with a file logger type.
     * @param append if true, the file isn't overwritten, otherwise it is.
     *
     * TODO : Can it be useful to have 2 or more types at the same time ? Print
     *         in console and file at the same time.
     */
    static void setType(Type type, const std::string &fileName = kDefaultLogFileName, bool append = true);
    static Type type() {return type_;}

    // Setters
    /**
     * Print time: default true.
     * @param printTime true to print time, otherwise set to false.
     */
    static void setPrintTime(bool printTime) {printTime_ = printTime;}
    static bool isPrintTime() {return printTime_;}

    /**
     * Print level: default true.
	 * @param printLevel true to print level, otherwise set to false.
	 */
    static void setPrintLevel(bool printLevel) {printLevel_ = printLevel;}
    static bool isPrintLevel() {return printLevel_;}

    /**
     * Print end of line: default true.
	 * @param printLevel true to print end of line, otherwise set to false.
	 */
    static void setPrintEndline(bool printEndline) {printEndline_ = printEndline;}
    static bool isPrintEndLine() {return printEndline_;}

    /**
	 * Print text with color: default true.
	 * Dark green for Debug, white for Info, yellow for Warning, red for Error and Fatal.
	 * @param printColored true to print text with color, otherwise set to false.
	 */
	static void setPrintColored(bool printColored) {printColored_ = printColored;}
	static bool isPrintColored() {return printColored_;}

    /**
     * Print where is this message in source code: default true.
	 * @param printWhere true to print where, otherwise set to false.
	 */
    static void setPrintWhere(bool printWhere) {printWhere_ = printWhere;}
    static bool isPrintWhere() {return printWhere_;}

    /**
	 * Print thread ID: default false.
	 * @param printThreadId true to print where, otherwise set to false.
	 */
	static void setPrintThreadId(bool printThreadId) {printThreadID_ = printThreadId;}
	static bool isPrintThreadId() {return printThreadID_;}

    /**
     * Print the full path: default true. ULogger::setPrintWhere() must be true to have path printed.
	 * @param printWhereFullPath true to print the full path, otherwise set to false.
	 */
    static void setPrintWhereFullPath(bool printWhereFullPath) {printWhereFullPath_ = printWhereFullPath;}
    static bool isPrintWhereFullPath() {return printWhereFullPath_;}

    /**
     * Set is the logger buffers messages, default false. When true, the messages are
     * buffered until the application is closed or ULogger::flush() is called.
     * @see ULogger::flush()
	 * @param buffered true to buffer messages, otherwise set to false.
	 */
    static void setBuffered(bool buffered);
    static bool isBuffered() {return buffered_;}

    /**
     * Set logger level: default kInfo. All messages over the severity set
     * are printed, other are ignored. The severity is from the lowest to
     * highest:
     * - kDebug
     * - kInfo
     * - kWarning
     * - kError
     * - kFatal
     * @param level the minimum level of the messages printed.
     */
    static void setLevel(ULogger::Level level) {level_ = level;}
    static ULogger::Level level() {return level_;}

	/**
	 * An ULogEvent is sent on each message logged at the specified level.
	 * Note : On message with level >= exitLevel, the event is sent synchronously (see UEventsManager::post()).
	 * @see ULogEvent
	 * @see setExitLevel()
	 */
	static void setEventLevel(ULogger::Level eventSentLevel) {eventLevel_ = eventSentLevel;}
	static ULogger::Level eventLevel() {return eventLevel_;}

	/**
	 * If not empty, only show log messages from threads included in this list.
	 */
	static void setTreadIdFilter(const std::set<unsigned long> & ids) {threadIdFilter_ = ids;}
	static void setTreadIdFilter(const std::vector<std::string> & ids);
	static const std::set<unsigned long> & getTreadIdFilter() {return threadIdFilter_;}

	/**
	 * Threads can register to this list. If name is empty, it will
	 * clear the thread in the list. Should be called from the thread itself.
	 */
	static void registerCurrentThread(const std::string & name);
	static void unregisterCurrentThread();
	static std::map<std::string, unsigned long> getRegisteredThreads();

    /**
     * Reset to default parameters.
     */
    static void reset();

    /**
	 * Flush buffered messages.
	 * @see setBuffered()
	 */
	static void flush();

    /**
     * Write a message directly to logger without level handling.
     * @param msg the message to write.
     * @param ... the variable arguments
     * @deprecated use UDEBUG(), UINFO(), UWARN(), UERROR() or UFATAL()
     */
    static void write(const char* msg, ...);

    /*
     * Write a message to logger: use UDEBUG(), UINFO(), UWARN(), UERROR() or UFATAL() instead.
     * @param level the log level of this message
     * @param file the file path
     * @param line the line in the file
     * @param function the function name in which the message is logged
     * @param msg the message to write
     * @param ... the variable arguments
     */
    static void write(ULogger::Level level,
    		const char * file,
    		int line,
    		const char *function,
    		const char* msg,
    		...);

    /**
     * Get the time in the format "2008-7-13 12:23:44".
     * @param timeStr string were the time will be copied.
     * @return the number of characters written, or 0 if an error occurred.
     */
    static int getTime(std::string &timeStr);

protected:
    /*
     * This method is used to have a reference on the 
     * Logger. When no Logger exists, one is 
     * created. There is only one instance in the application.
     * Must be protected by loggerMutex_.
     * See the Singleton pattern for further explanation.
     *
     * @return the reference on the Logger
     */
    static ULogger* getInstance();

    /*
     * Called only once in getInstance(). It can't be instanciated 
     * by the user.
     *
     * @see getInstance()
     */
    ULogger() {}

    /*
     * Only called by a Destroyer.
     * @see Destroyer
     */
    virtual ~ULogger();

    /*
	 * Flush buffered messages
	 */
	void _flush();

    /*
     * A Destroyer is used to remove a dynamicaly created 
     * Singleton. It is friend here to have access to the 
     * destructor.
     *
     * @see Destroyer
     */
    friend class UDestroyer<ULogger>;
    
    /*
     * The log file name.
     */
    static std::string logFileName_;

    /*
     * Default true, it doesn't overwrite the file.
     */
    static bool append_;
    
private:
    /*
     * Create an instance according to type. See the Abstract factory 
     * pattern for further explanation.
     * @see type_
     * @return the reference on the new logger
     */
    static ULogger* createInstance();

    /*
     * Write a message on the output with the format :
     * "A message". Inherited class
     * must override this method to output the message. It 
     * does nothing by default.
     * @param msg the message to write.
     * @param arg the variable arguments
     */
    virtual void _write(const char*, va_list) {} // Do nothing by default
    virtual void _writeStr(const char*) {} // Do nothing by default

private:
    /*
     * The Logger instance pointer.
     */
    static ULogger* instance_;

    /*
     * The Logger's destroyer
     */
    static UDestroyer<ULogger> destroyer_;

    /*
     * If the logger prints the time for each message. 
     * Default is true.
     */
    static bool printTime_;

    /*
     * If the logger prints the level for each message. 
     * Default is true.
     */
    static bool printLevel_;

    /*
     * If the logger prints the end line for each message. 
     * Default is true.
     */
    static bool printEndline_;

    /*
	 * If the logger prints text with color.
	 * Default is true.
	 */
    static bool printColored_;

    /*
	 * If the logger prints where the message is logged (fileName::function():line).
	 * Default is true.
	 */
    static bool printWhere_;

    /*
	 * If the logger prints the full path of the source file
	 * where the message is written. Only works when
	 * "printWhere_" is true.
	 * Default is false.
	 */
    static bool printWhereFullPath_;

    /*
	 * If the logger prints the thread ID.
	 * Default is false.
	 */
    static bool printThreadID_;

    /*
	 * If the logger limit the size of the "where" path to
	 * characters. If the path is over 8 characters, a "~"
	 * is added. Only works when "printWhereFullPath_" is false.
	 * Default is false.
	 */
    static bool limitWhereLength_;

    /*
     * The type of the logger.
     */
    static Type type_;

    /*
	 * The severity of the log.
	 */
    static Level level_;

    /*
	 * The severity at which the message is also sent in a ULogEvent.
	 */
	static Level eventLevel_;

    static const char * levelName_[5];

    /*
     * Mutex used when accessing public functions.
     */
    static UMutex loggerMutex_;

    /*
	 * If the logger prints messages only when ULogger::flush() is called.
	 * Default is false.
	 */
	static bool buffered_;

	static std::string bufferedMsgs_;

	static std::set<unsigned long> threadIdFilter_;
	static std::map<std::string, unsigned long> registeredThreads_;
};

#endif // ULOGGER_H
