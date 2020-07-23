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
	virtual ~PyUtil() {}

	static void acquire();
	static void release();

	static std::string getTraceback();
	static PyObject* importModule(const std::string & path);

private:
	PyUtil() {}

	static UMutex mutex_;
	static size_t references_;
};

}


#endif /* CORELIB_SRC_PYUTIL_H_ */
