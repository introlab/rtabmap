/*
 * PythonSingleTon.h
 *
 *  Created on: Jan. 14, 2021
 *      Author: mathieu
 */

#include <PythonInterface.h>
#include <Python.h>

namespace rtabmap {

UMutex PythonInterface::mutex_;
int PythonInterface::refCount_ = 0;

PythonInterface::PythonInterface()
{
	UScopeMutex lock(mutex_);
	if(refCount_ == 0) {
		Py_Initialize();
	}
	++refCount_;
}

PythonInterface::~PythonInterface()
{
	UScopeMutex lock(mutex_);
	if(refCount_>0 && --refCount_==0) {
		Py_Finalize();
	}
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
	PyObject* method = PyObject_GetAttrString(mod, "get_pretty_traceback");
	PyObject* outStr = PyObject_CallObject(method, Py_BuildValue("OOO", type, value, traceback));
	std::string pretty = PyBytes_AsString(PyUnicode_AsASCIIString(outStr));

	Py_DECREF(method);
	Py_DECREF(outStr);
	Py_DECREF(mod);

	return pretty;
}

}
