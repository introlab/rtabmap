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

namespace rtabmap {

class PythonInterface
{
public:
	virtual ~PythonInterface();

protected:
	PythonInterface();
	std::string getTraceback();

private:
	static UMutex mutex_;
	static int refCount_;
};

}

#endif /* CORELIB_SRC_PYTHON_PYTHONINTERFACE_H_ */
