/*
 * PythonSingleTon.h
 *
 *  Created on: Jan. 14, 2021
 *      Author: mathieu
 */

#include <rtabmap/core/PythonInterface.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UThread.h>
#include <pybind11/embed.h>

namespace rtabmap {

PythonInterface::PythonInterface()
{
	UINFO("Initialize python interpreter");
	guard_ = new pybind11::scoped_interpreter();
	pybind11::module::import("threading");
	release_ = new pybind11::gil_scoped_release();
}

PythonInterface::~PythonInterface()
{
	UINFO("Finalize python interpreter");
	delete release_;
	delete guard_;
}

std::string getPythonTraceback()
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
