/*
 * PythonInterface.h
 *
 *  Created on: Jan. 14, 2021
 *      Author: mathieu
 */

#ifndef CORELIB_SRC_PYTHON_PYTHONINTERFACE_H_
#define CORELIB_SRC_PYTHON_PYTHONINTERFACE_H_


#include <string>
#include <rtabmap/utilite/UMutex.h>
#include <Python.h>

namespace rtabmap {

class PythonInterface
{
public:
	PythonInterface();
	virtual ~PythonInterface();

protected:
	std::string getTraceback(); // should be called between lock() and unlock()
	void lock();
	void unlock();

private:
	static UMutex mutex_;
	static int refCount_;

protected:
	static PyThreadState * mainThreadState_;
	static unsigned long mainThreadID_;
	PyThreadState * threadState_;
};

}

#endif /* CORELIB_SRC_PYTHON_PYTHONINTERFACE_H_ */
