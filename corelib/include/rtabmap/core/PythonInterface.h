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

namespace pybind11 {
class scoped_interpreter;
class gil_scoped_release;
}

namespace rtabmap {

/**
 * Create a single PythonInterface on main thread at
 * global scope before any Python classes.
 */
class PythonInterface
{
public:
	PythonInterface();
	virtual ~PythonInterface();

private:
	pybind11::scoped_interpreter* guard_;
	pybind11::gil_scoped_release* release_;
};

std::string getPythonTraceback();

}

#endif /* CORELIB_SRC_PYTHON_PYTHONINTERFACE_H_ */
