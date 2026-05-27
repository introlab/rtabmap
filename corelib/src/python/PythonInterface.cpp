/*
 * PythonSingleTon.h
 *
 *  Created on: Jan. 14, 2021
 *      Author: mathieu
 */

#include <rtabmap/core/PythonInterface.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UThread.h>
#include <rtabmap/utilite/UConversion.h>
#include <pybind11/embed.h>
#include <filesystem>
#include <thread>

namespace rtabmap {

namespace {
// Captured when librtabmap_core is loaded. The dynamic loader runs static
// initializers on the main thread before main(), so this records the main
// thread id (as long as the library isn't dlopen'd from a worker thread).
const std::thread::id g_mainThreadId = std::this_thread::get_id();
}

PythonInterface & PythonInterface::instance(const std::string & caller)
{
	// Meyers singleton: thread-safe construction in C++11, destroyed at exit.
	// The constructor asserts it runs on the main thread; the caller tag
	// from the first invocation is captured into the assertion message.
	static PythonInterface inst(caller);
	return inst;
}

PythonInterface::PythonInterface(const std::string & caller)
{
	UASSERT_MSG(std::this_thread::get_id() == g_mainThreadId,
			uFormat("PythonInterface must be created on the main thread "
					"(first construction triggered by \"%s\"). Call "
					"PythonInterface::instance() early in main() before "
					"any worker thread touches a Python-backed class.",
					caller.empty()?"<unspecified>":caller.c_str()).c_str());
	UINFO("Initialize python interpreter (triggered by \"%s\")",
			caller.empty()?"<unspecified>":caller.c_str());
	guard_ = new pybind11::scoped_interpreter();

	// Tell Python to look in this directory for DLLs
#ifdef _WIN32
	std::string exe_dir = std::filesystem::current_path().string();
    pybind11::module_ os = pybind11::module_::import("os");
    os.attr("add_dll_directory")(exe_dir);
#endif

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
