/*
 * PythonSingleTon.h
 *
 *  Created on: Jan. 14, 2021
 *      Author: mathieu
 */

#include <rtabmap/core/PythonInterface.h>
#include <rtabmap/utilite/ULogger.h>

namespace rtabmap {

UMutex PythonInterface::mutex_;
int PythonInterface::refCount_ = 0;
PyThreadState * PythonInterface::mainThreadState_ = 0;

PythonInterface::PythonInterface() :
		threadState_(0),
		interpretor_(0),
		lockedState_(0)
{
	UScopeMutex lockM(mutex_);
	if(refCount_ == 0)
	{
		// initialize Python
		Py_Initialize();

#ifdef RTABMAP_PYTHON_THREADING
		// initialize thread support
		PyEval_InitThreads();
		Py_DECREF(PyImport_ImportModule("threading"));

		//release the GIL, store thread state, set the current thread state to NULL
		mainThreadState_ = PyEval_SaveThread();
		UASSERT(mainThreadState_);
#endif
	}

#ifdef RTABMAP_PYTHON_THREADING
	PyEval_RestoreThread(mainThreadState_);
	PyThreadState * ts = PyThreadState_Swap(NULL);
	// get a reference to the PyInterpreterState
	interpretor_ = Py_NewInterpreter();
	UASSERT(interpretor_);
	PyThreadState_Swap(ts);
	mainThreadState_ = PyEval_SaveThread();
#endif

	++refCount_;
}

PythonInterface::~PythonInterface()
{
	UScopeMutex lock(mutex_);
	if(refCount_>0 && --refCount_==0)
	{
		// shut down the interpreter
#ifdef RTABMAP_PYTHON_THREADING
		PyEval_RestoreThread(mainThreadState_);
#endif
		Py_Finalize();
	}
	else
	{
#ifdef RTABMAP_PYTHON_THREADING
		PyThreadState * ts = PyThreadState_Swap(interpretor_);
		Py_EndInterpreter(interpretor_);
		PyThreadState_Swap(ts);
#endif
	}
}

void PythonInterface::lock()
{
	mutex_.lock();
#ifdef RTABMAP_PYTHON_THREADING
	UDEBUG("");
	// create a thread state object for this thread
	threadState_ = PyThreadState_New(interpretor_->interp);
	UASSERT(threadState_);
	PyEval_RestoreThread(threadState_);
	lockedState_ = PyThreadState_Swap(interpretor_);
	UDEBUG("");
#endif
}

void PythonInterface::unlock()
{
#ifdef RTABMAP_PYTHON_THREADING
	UDEBUG("");
	PyThreadState_Swap(lockedState_);
	PyThreadState_Clear(threadState_);
	PyThreadState_DeleteCurrent();
	UDEBUG("");
#endif
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
