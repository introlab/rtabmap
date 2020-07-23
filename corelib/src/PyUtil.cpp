/*
 * PyUtil.cpp
 *
 *  Created on: Jul. 16, 2020
 *      Author: mathieu
 */

#include "PyUtil.h"
#include <rtabmap/utilite/UFile.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UStl.h>

namespace rtabmap {

size_t PyUtil::references_ = 0;
UMutex PyUtil::mutex_;

void PyUtil::acquire()
{
	UScopeMutex lock(mutex_);
	if(references_ == 0)
	{
		Py_Initialize();
	}
	++references_;
}
void PyUtil::release()
{
	UScopeMutex lock(mutex_);
	if(references_>0)
	{
		--references_;
		if(references_==0)
		{
			Py_Finalize();
		}
	}
}

std::string PyUtil::getTraceback()
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

PyObject * PyUtil::importModule(const std::string & path)
{
	if(!UFile::exists(path) || UFile::getExtension(path).compare("py") != 0)
	{
		UERROR("Cannot initialize Python module, the path is not valid: \"%s\"", path.c_str());
		return 0;
	}

	std::string matcherPythonDir = UDirectory::getDir(path);
	if(!matcherPythonDir.empty())
	{
		PyRun_SimpleString("import sys");
		PyRun_SimpleString(uFormat("sys.path.append(\"%s\")", matcherPythonDir.c_str()).c_str());
	}

	std::string scriptName = uSplit(UFile::getName(path), '.').front();
	PyObject * pName = PyUnicode_FromString(scriptName.c_str());
	PyObject * module = PyImport_Import(pName);
	Py_DECREF(pName);

	if(!module)
	{
		UERROR("Module \"%s\" could not be imported! (File=\"%s\")", scriptName.c_str(), path.c_str());
		UERROR("%s", getTraceback().c_str());
	}

	return module;
}

}

