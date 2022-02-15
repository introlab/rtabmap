/*
 * PythonSingleTon.h
 *
 *  Created on: Jan. 14, 2021
 *      Author: mathieu
 */

#include <rtabmap/core/PythonInterface.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UThread.h>

namespace rtabmap {

UMutex PythonInterface::mutex_;
int PythonInterface::refCount_ = 0;
PyThreadState * PythonInterface::mainThreadState_ = 0;
unsigned long PythonInterface::mainThreadID_ = 0;

PythonInterface::PythonInterface() :
		threadState_(0)
{
	UScopeMutex lockM(mutex_);
	if(refCount_ == 0)
	{
		UINFO("Py_Initialize() with thread = %d", UThread::currentThreadId());
		// initialize Python
		Py_Initialize();

		// initialize thread support
		PyEval_InitThreads();
		Py_DECREF(PyImport_ImportModule("threading"));

		//release the GIL, store thread state, set the current thread state to NULL
		mainThreadState_ = PyEval_SaveThread();
		UASSERT(mainThreadState_);
		mainThreadID_ = UThread::currentThreadId();
	}

	++refCount_;
}

PythonInterface::~PythonInterface()
{
	UScopeMutex lock(mutex_);
	if(refCount_>0 && --refCount_==0)
	{
		// shut down the interpreter
		UINFO("Py_Finalize() with thread = %d", UThread::currentThreadId());
		PyEval_RestoreThread(mainThreadState_);
		Py_Finalize();
	}
}

void PythonInterface::lock()
{
	mutex_.lock();

	UDEBUG("Lock: Current thread=%d (main=%d)", UThread::currentThreadId(), mainThreadID_);
	if(UThread::currentThreadId() == mainThreadID_)
	{
		PyEval_RestoreThread(mainThreadState_);
	}
	else
	{
		// create a thread state object for this thread
		threadState_ = PyThreadState_New(mainThreadState_->interp);
		UASSERT(threadState_);
		PyEval_RestoreThread(threadState_);
	}
}

void PythonInterface::unlock()
{
	if(UThread::currentThreadId() == mainThreadID_)
	{
		mainThreadState_ = PyEval_SaveThread();
	}
	else
	{
		PyThreadState_Clear(threadState_);
		PyThreadState_DeleteCurrent();
	}
	UDEBUG("Unlock: Current thread=%d (main=%d)", UThread::currentThreadId(), mainThreadID_);
	mutex_.unlock();
}

std::string PythonInterface::getTraceback()
{
	// Author: https://stackoverflow.com/questions/41268061/c-c-python-exception-traceback-not-being-generated

	PyObject* type;
	PyObject* value;
	PyObject* traceback;

	PyErr_Fetch(&type, &value, &traceback);
	PyErr_NormalizeException(&type, &value, &traceback);

	std::string fcn = "";
	fcn += "def get_pretty_traceback(exc_type, exc_value, exc_tb):\n";
	fcn += "    import sys, traceback\n";
	fcn += "    lines = []\n";
	fcn += "    lines = traceback.format_exception(exc_type, exc_value, exc_tb)\n";
	fcn += "    output = '\\n'.join(lines)\n";
	fcn += "    return output\n";

	PyRun_SimpleString(fcn.c_str());
	PyObject* mod = PyImport_ImportModule("__main__");
	UASSERT(mod);
	PyObject* method = PyObject_GetAttrString(mod, "get_pretty_traceback");
	UASSERT(method);
	PyObject* outStr = PyObject_CallObject(method, Py_BuildValue("OOO", type, value, traceback));
	std::string pretty;
	if(outStr)
		pretty = PyBytes_AsString(PyUnicode_AsASCIIString(outStr));

	Py_DECREF(method);
	Py_DECREF(outStr);
	Py_DECREF(mod);

	return pretty;
}

}
