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

#ifndef UTILITE_H
#define UTILITE_H

/** \mainpage UtiLite
  *
  * \section intro Introduction
  * <a href="http://utilite.googlecode.com">UtiLite</a> is a simple library to create small cross-platform
  * applications using <b>threads</b>, <b>events-based communication</b> and a powerful <b>logger</b>. The first three
  * sections show the core classes of the library, then last sections show some useful functions added through time.
  *
  * UtiLite provides a utility application called \ref uResourceGeneratorPage "uResourceGenerator" to generate resources to include in an executable. For example:
  * @code
  * $ ./uresourcegenerator DatabaseSchema.sql
  * @endcode
  * This will generate a HEX file "DatabaseSchema_sql.h" which can be included in source files.
  * Data of the file is global and can be accessed by the generated const char * DATABASESCHEMA_SQL.
  * @code
  * #include "DatabaseSchema_sql.h"
  * ...
  * std::string hex = DATABASESCHEMA_SQL;
  * // Assuming there are only ASCII characters, we can directly convert to a string:
  * std::string schema = uHex2Str(hex);
  * // For binary data:
  * std::vector<char> bytes = uHex2Bytes(hex);
  * @endcode
  *
  * A generated \ref findUtilitePage "FindUtiLite.cmake" is also provided for easy linking with the library.
  *
  *
  * \section logger ULogger
  * The ULogger can be used anywhere in the application to log messages (formated like a printf). The
  * logger can be set (ULogger::setType()) to print in a file or in the console (with colors depending on the severity). Convenient
  * macros are given, working like a printf(), as UDEBUG(), UINFO(), UWARN(), UERROR(), UFATAL(), UASSERT(). Small example:
  * @code
  * ...
  * UINFO("A simple message with number %d", 42);
  * UDEBUG("A debug message");
  * ...
  * @endcode
  * This will print: [Severity] (Time) File:Line:Function() "The message"
  * @code
  * [ INFO] (2010-09-25 18:08:20) main.cpp:18::main() A simple message with number 42
  * [DEBUG] (2010-09-25 18:08:20) main.cpp:18::main() A debug message
  * @endcode
  *
  * \section eventsmanager UEventsManager, UEventsHandler, UEvent
  * The events-based communication framework helps to communicate between objects/threads.
  * The UEventsManager is a singleton with which we can post events anywhere in the
  * application by calling UEventsManager::post(). All UEventsHandler will then receive the
  * event with their protected function UEventsHandler::handleEvent(). Handlers are added to UEventsManager by
  * calling UEventsManager::addHandler(). The UEvent provides an abstract class to implement any event
  * implementations. The only requirement is that the event must implements UEvent::getClassName() to know the event's type.
  * @code
  * ...
  * MyHandler handler; // MyHandler is an implementation of UEventsHandler
  * UEventsManager::addHandler(&handler);
  * UEventsManager::post(new MyEvent()); // MyEvent is an implementation of UEvent
  * // The UEventsHandler::handleEvent() of "handler" will then be called by the UEventsManager's events dispatching thread.
  * ...
  * @endcode
  * Look at the <b>full example</b> in page of UEventsHandler on how communication works with threads (UThread).
  *
  * \section thread UThread, UMutex, USemaphore
  * The multi-threading framework use a UThread as an abstract class to implement a
  * thread in object-style. Reimplement UThread::mainLoop() then call UThread::start() to
  * start the main loop of the thread. Threads can be joined by calling UThread::join() and
  * killed by calling UThread::kill(). Classes UMutex and USemaphore provide blocking mechanisms to
  * protect data between threads.
  * @code
  * ...
  * MyThread t; // MyThread is an implementation of UThread
  * t.start();
  * t.join(); // Wait the thread to finish
  * ...
  * @endcode
  *
  * \section timer UTimer
  * A useful class to compute processing time:
  *  - UTimer::start(),
  *  - UTimer::stop(),
  *  - UTimer::restart(),
  *  - UTimer::elapsed(),
  *  - UTimer::now().
  *
  * \section filedir UDirectory, UFile
  * For files and directories manipulations :
  *  - UFile::exists(),
  *  - UFile::length(),
  *  - UFile::rename(),
  *  - UFile::erase(),
  *  - UDirectory::exists(),
  *  - UDirectory::getFileNames(),
  *  - UDirectory::makeDir(),
  *  - UDirectory::removeDir(),
  *  - UDirectory::currentDir(),
  *  - UDirectory::homeDir(),
  *
  * \section stl Convenient use of STL
  * The library provides some simple wrappers of the STL like:
  *  - uUniqueKeys() to get unique keys from a std::multimap,
  *  - uKeys() to get all keys of a std::multimap or std::map,
  *  - uValues() to get all values of a std::multimap or std::map,
  *  - uValue() to get the value of a key (with a default argument if the key is not found),
  *  - uTake() to take the value of a key (with a default argument if the key is not found),
  *  - uIteratorAt() to get iterator at a specified position in a std::list,
  *  - uValueAt() to get value at a specified position in a std::list,
  *  - uContains() to know if a key/value exists in a std::multimap, std::map, std::list,
  *  - uInsert() to insert a value in a std::map and overwriting the value if the key already exists,
  *  - uListToVector(),
  *  - uVectorToList(),
  *  - uAppend() to append a list to another list,
  *  - uIndexOf() to get index of a value in a std::list,
  *  - uSplit() to split a string into a std::list of strings on the specified separator.
  *
  *
  * \section math Basic mathematic operations
  * A library of basic array manipulations:
  *  - uMax(),
  *  - uSign(),
  *  - uSum(),
  *  - uMean(),
  *  - uStdDev(),
  *  - uNorm(),
  *  - uNormalize(),
  *  - uXMatch().
  *
  * \section conversion Conversion
  * A library of convenient functions to convert some data into another like:
  * - uReplaceChar(),
  * - uToUpperCase(),
  * - uToLowerCase(),
  * - uNumber2Str(),
  * - uBool2Str(),
  * - uStr2Bool(),
  * - uBytes2Hex(),
  * - uHex2Bytes(),
  * - uHex2Bytes(),
  * - uHex2Str(),
  * - uHex2Ascii(),
  * - uAscii2Hex(),
  * - uFormatv(),
  * - uFormat().
  *
  * \section processinfo UProcessInfo
  * This class can be used to get the process memory usage: UProcessInfo::getMemoryUsage().
  *
  * \section qtLib Qt Widgets (libutilite_qt.so : OPTIONAL)
  * If Qt is found on the system, the UtiLite Qt library (libutilite_qt.so, libutilite_qt.dll) with
  * useful widgets is built. Use class UPlot to create a plot like MATLAB, and incrementally add
  * new values like a scope. USpectrogram is used to
  * show audio frequency frames.
  * - UPlot,
  * - USpectrogram,
  * - UImageView.
  * @image html UPlot.gif
  * @image html USpectrogram.png
  *
  * \section audioLib Audio stuff (libutilite_audio.so : OPTIONAL)
  * If FMOD is found on the system, the UtiLite audio
  * library is built (libutilite_audio.so, libutilite_audio.dll). It is a wrapper
  * of FMOD methods with a convenient interface to extract audio frames.
  * - UAudioCapture,
  * - UAudioCaptureFile,
  * - UAudioCaptureMic,
  * - UAudioCaptureFFT,
  * - UAudioPlayer,
  * - UAudioPlayerTone,
  * - UWav,
  * - UMp3Encoder (only if Lame is also found on the system).
  *
  * \section cvLib OpenCV stuff (libutilite_cv.so : OPTIONAL)
  * If OpenCV is found on the system, the UtiLite cv
  * library is built (libutilite_cv.so, libutilite_cv.dll). It provides
  * image capture classes used to read from a webcam, a video file
  * or a directory of images. If UtiLite is also built with Qt, a
  * convenient function uCvMat2QImage() can be used to convert a cv::Mat
  * image to a QImage.
  * - UVideoCapture,
  * - UImageFolderCapture,
  * - UColorTable,
  * - uCvMat2QImage() (only if Qt is also found on the system).
  */

/*! \page uResourceGeneratorPage uResourceGenerator
 * UtiLite provides a utility application called \ref uResourceGeneratorPage "uResourceGenerator" to generate resources to include in an executable. For example:
 * @code
 * $ ./uresourcegenerator DatabaseSchema.sql
 * @endcode
 * This will generate a HEX file "DatabaseSchema_sql.h" which can be included in source files.
 * Data of the file is global and can be accessed by the generated const char * DATABASESCHEMA_SQL.
 * @code
 * #include "DatabaseSchema_sql.h"
 * ...
 * std::string hex = DATABASESCHEMA_SQL;
 * // Assuming there are only ASCII characters, we can directly convert to a string:
 * std::string schema = uHex2Str(hex);
 * // For binary data:
 * std::vector<char> bytes = uHex2Bytes(hex);
 * @endcode
 *
 * The generator can be automated in a CMake build like:
 * @code
 * ADD_CUSTOM_COMMAND(
 *    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/DatabaseSchema_sql.h
 *    COMMAND ${URESOURCEGENERATOR_EXEC} -n my_namespace -p ${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/DatabaseSchema.sql
 *    COMMENT "[Creating database resource]"
 *    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/DatabaseSchema.sql
 * )
 * SET(RESOURCES
 *    ${CMAKE_CURRENT_BINARY_DIR}/DatabaseSchema_sql.h
 * )
 * ADD_LIBRARY(mylib ${SRC_FILES} ${RESOURCES})
 * ADD_EXECUTABLE(myexecutable ${SRC_FILES} ${RESOURCES})
 * @endcode
 * The variable URESOURCEGENERATOR_EXEC is set when FIND_PACKAGE(UtiLite) is done, you would need to add \ref findUtilitePage "FindUtiLite.cmake".
 */

#include "rtabmap/utilite/UStl.h"
#include "rtabmap/utilite/UConversion.h"
#include "rtabmap/utilite/UDirectory.h"
#include "rtabmap/utilite/UFile.h"
#include "rtabmap/utilite/ULogger.h"
#include "rtabmap/utilite/UEventsManager.h"
#include "rtabmap/utilite/UEventsHandler.h"
#include "rtabmap/utilite/UEvent.h"
#include "rtabmap/utilite/UProcessInfo.h"
#include "rtabmap/utilite/UMutex.h"
#include "rtabmap/utilite/USemaphore.h"
#include "rtabmap/utilite/UThreadNode.h"
#include "rtabmap/utilite/UTimer.h"
#include "rtabmap/utilite/UVariant.h"
#include "rtabmap/utilite/UMath.h"

#endif /* UTILITE_H */
