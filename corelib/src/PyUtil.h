/*
 * PythonUtil.h
 *
 *  Created on: Jul. 16, 2020
 *      Author: mathieu
 */

#ifndef CORELIB_SRC_PYUTIL_H_
#define CORELIB_SRC_PYUTIL_H_

#include <Python.h>
#include <rtabmap/utilite/UMutex.h>
#include <string>

namespace rtabmap {

class PyUtil
{
public:
	virtual ~PyUtil();

	static void init();
	static bool initialized();

	static std::string getTraceback();
	static PyObject* importModule(const std::string & path);

private:
	PyUtil();

	static bool initialized_;
	static UMutex mutex_;
	static PyUtil instance_;
};

}


#endif /* CORELIB_SRC_PYUTIL_H_ */
