/*
 * PythonInterface.h
 *
 *  Created on: Jan. 14, 2021
 *      Author: mathieu
 */

#ifndef CORELIB_SRC_PYTHON_PYTHONINTERFACE_H_
#define CORELIB_SRC_PYTHON_PYTHONINTERFACE_H_

#include "rtabmap/core/rtabmap_core_export.h" // DLL export/import defines

#include <string>
#include <rtabmap/utilite/UMutex.h>

namespace pybind11 {
class scoped_interpreter;
class gil_scoped_release;
}

namespace rtabmap {

/**
 * Process-wide singleton owning the embedded Python interpreter.
 * Call PythonInterface::instance() from the main thread (typically near
 * the top of main()) before any Python-using class is constructed.
 */
class RTABMAP_CORE_EXPORT PythonInterface
{
public:
	// Pass a caller tag (e.g. class name) so the main-thread assertion
	// can report who triggered the first construction.
	static PythonInterface & instance(const std::string & caller = "");

	PythonInterface(const PythonInterface &) = delete;
	PythonInterface & operator=(const PythonInterface &) = delete;

private:
	explicit PythonInterface(const std::string & caller);
	~PythonInterface();

	pybind11::scoped_interpreter* guard_;
	pybind11::gil_scoped_release* release_;
};

std::string RTABMAP_CORE_EXPORT getPythonTraceback();

}

#endif /* CORELIB_SRC_PYTHON_PYTHONINTERFACE_H_ */
