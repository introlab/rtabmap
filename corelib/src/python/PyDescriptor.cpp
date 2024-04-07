
#include <python/PyDescriptor.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UDirectory.h>
#include <rtabmap/utilite/UFile.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UTimer.h>

#include <pybind11/embed.h>

#define NPY_NO_DEPRECATED_API NPY_API_VERSION
#include <numpy/arrayobject.h>

namespace rtabmap
{

PyDescriptor::PyDescriptor(
		const ParametersMap & parameters) :
		GlobalDescriptorExtractor(parameters),
		pModule_(0),
		pFunc_(0),
		dim_(Parameters::defaultPyDescriptorDim())
{
	UDEBUG("");
	this->parseParameters(parameters);
}

PyDescriptor::~PyDescriptor()
{
	UDEBUG("");
	pybind11::gil_scoped_acquire acquire;

	if(pFunc_)
	{
		Py_DECREF(pFunc_);
	}
	if(pModule_)
	{
		Py_DECREF(pModule_);
	}
}

void PyDescriptor::parseParameters(const ParametersMap & parameters)
{
	UDEBUG("");
	std::string previousPath = path_;
	Parameters::parse(parameters, Parameters::kPyDescriptorPath(), path_);
	Parameters::parse(parameters, Parameters::kPyDescriptorDim(), dim_);
	path_ = uReplaceChar(path_, '~', UDirectory::homeDir());
	UINFO("path = %s", path_.c_str());
	UINFO("dim = %d", dim_);
	UTimer timer;

	pybind11::gil_scoped_acquire acquire;

	if(pModule_)
	{
		if(!previousPath.empty() && previousPath.compare(path_)!=0)
		{
			UDEBUG("we changed script (old=%s), we need to reload (new=%s)",
					previousPath.c_str(), path_.c_str());
			if(pFunc_)
			{
				Py_DECREF(pFunc_);
			}
			pFunc_=0;
			Py_DECREF(pModule_);
			pModule_ = 0;
		}
	}

	if(pModule_==0)
	{
		UASSERT(pFunc_ == 0);
		if(path_.empty())
		{
			return;
		}
		std::string matcherPythonDir = UDirectory::getDir(path_);
		if(!matcherPythonDir.empty())
		{
			PyRun_SimpleString("import sys");
			PyRun_SimpleString(uFormat("sys.path.append(\"%s\")", matcherPythonDir.c_str()).c_str());
		}

		_import_array();

		std::string scriptName = uSplit(UFile::getName(path_), '.').front();
		PyObject * pName = PyUnicode_FromString(scriptName.c_str());
		UDEBUG("PyImport_Import() beg");
		pModule_ = PyImport_Import(pName);
		UDEBUG("PyImport_Import() end");

		Py_DECREF(pName);

		if(!pModule_)
		{
			UERROR("Module \"%s\" could not be imported! (File=\"%s\")", scriptName.c_str(), path_.c_str());
			UERROR("%s", getPythonTraceback().c_str());
		}
		else
		{
			PyObject * pFunc = PyObject_GetAttrString(pModule_, "init");
			if(pFunc)
			{
				if(PyCallable_Check(pFunc))
				{
					PyObject * result = PyObject_CallFunction(pFunc, "i", dim_);

					if(result == NULL)
					{
						UERROR("Call to \"init(...)\" in \"%s\" failed!", path_.c_str());
						UERROR("%s", getPythonTraceback().c_str());
					}
					Py_DECREF(result);

					pFunc_ = PyObject_GetAttrString(pModule_, "extract");
					if(pFunc_ && PyCallable_Check(pFunc_))
					{
						// we are ready!
					}
					else
					{
						UERROR("Cannot find method \"extract(...)\" in %s", path_.c_str());
						UERROR("%s", getPythonTraceback().c_str());
						if(pFunc_)
						{
							Py_DECREF(pFunc_);
							pFunc_ = 0;
						}
					}
				}
				else
				{
					UERROR("Cannot call method \"init(...)\" in %s", path_.c_str());
					UERROR("%s", getPythonTraceback().c_str());
				}
				Py_DECREF(pFunc);
			}
			else
			{
				UERROR("Cannot find method \"init(...)\"");
				UERROR("%s", getPythonTraceback().c_str());
			}
		}
	}
}

GlobalDescriptor PyDescriptor::extract(
		  const SensorData & data) const
{
	UDEBUG("");
	UTimer timer;
	GlobalDescriptor descriptor;

	if(!pModule_)
	{
		UERROR("Python module not loaded!");
		return descriptor;
	}

	if(!pFunc_)
	{
		UERROR("Python function not loaded!");
		return descriptor;
	}

	pybind11::gil_scoped_acquire acquire;

	if(!data.imageRaw().empty())
	{
		std::vector<unsigned char> descriptorsQueryV(data.imageRaw().total()*data.imageRaw().channels());
		memcpy(descriptorsQueryV.data(), data.imageRaw().data, data.imageRaw().total()*data.imageRaw().channels()*sizeof(char));
		npy_intp dimsFrom[3] = {data.imageRaw().rows, data.imageRaw().cols, data.imageRaw().channels()};
		PyObject* pImageQuery = PyArray_SimpleNewFromData(3, dimsFrom, NPY_BYTE, (void*)data.imageRaw().data);
		UASSERT(pImageQuery);

		UDEBUG("Preparing data time = %fs", timer.ticks());

		PyObject *pReturn = PyObject_CallFunctionObjArgs(pFunc_, pImageQuery, NULL);
		if(pReturn == NULL)
		{
			UERROR("Failed to call extract() function!");
			UERROR("%s", getPythonTraceback().c_str());
		}
		else
		{
			UDEBUG("Python extraction time = %fs", timer.ticks());

			PyArrayObject *np_ret = reinterpret_cast<PyArrayObject*>(pReturn);

			// Convert back to C++ array and print.
			int len1 = PyArray_SHAPE(np_ret)[0];
			int dim = PyArray_SHAPE(np_ret)[1];
			int type = PyArray_TYPE(np_ret);
			UDEBUG("Descriptor array %dx%d (type=%d)", len1, dim, type);
			UASSERT(len1 == 1);
			UASSERT_MSG(type == NPY_FLOAT, uFormat("Returned descriptor should type FLOAT=11, received type=%d", type).c_str());

			float* d_out = reinterpret_cast<float*>(PyArray_DATA(np_ret));
			descriptor = GlobalDescriptor(1, cv::Mat(1, dim, CV_32FC1, d_out).clone());

			//std::cout << descriptor.data() << std::endl;

			Py_DECREF(pReturn);
		}

		Py_DECREF(pImageQuery);
	}
	else
	{
		UERROR("Invalid inputs! Missing image.");
	}
	return descriptor;
}

}
