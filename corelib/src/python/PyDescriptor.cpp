
#include <PyUtil.h>
#include <pydescriptor/PyDescriptor.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UDirectory.h>
#include <rtabmap/utilite/UFile.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UTimer.h>

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
	PyUtil::acquire();
	UASSERT(_import_array()==0);

	this->parseParameters(parameters);
}

PyDescriptor::~PyDescriptor()
{
	if(pFunc_)
	{
		Py_DECREF(pFunc_);
	}
	if(pModule_)
	{
		Py_DECREF(pModule_);
	}
	PyUtil::release();
}

void PyDescriptor::parseParameters(const ParametersMap & parameters)
{
	std::string path;
	Parameters::parse(parameters, Parameters::kPyDescriptorPath(), path);
	Parameters::parse(parameters, Parameters::kPyDescriptorDim(), dim_);
	path = uReplaceChar(path, '~', UDirectory::homeDir());
	UINFO("path = %s", path.c_str());
	UINFO("dim = %d", dim_);
	UTimer timer;

	if(pModule_)
	{
		if(!path.empty() && path.compare(path_)!=0)
		{
			UDEBUG("we changed script (old=%s), we need to reload (new=%s)",
					path_.c_str(), path.c_str());
			if(pFunc_)
			{
				Py_DECREF(pFunc_);
			}
			pFunc_=0;
			Py_DECREF(pModule_);
			pModule_ = 0;
			path_.clear();
		}
	}

	if(pModule_==0)
	{
		if(path.empty())
		{
			return;
		}
		pModule_ = PyUtil::importModule(path);

		if(pModule_)
		{
			path_ = path;
			PyObject * pFunc = PyObject_GetAttrString(pModule_, "init");
			if(pFunc)
			{
				if(PyCallable_Check(pFunc))
				{
					PyObject * result = PyObject_CallFunction(pFunc, "i", dim_);

					if(result == NULL)
					{
						UERROR("Call to \"init(...)\" in \"%s\" failed!", path_.c_str());
						UERROR("%s", PyUtil::getTraceback().c_str());
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
						UERROR("%s", PyUtil::getTraceback().c_str());
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
					UERROR("%s", PyUtil::getTraceback().c_str());
				}
				Py_DECREF(pFunc);
			}
			else
			{
				UERROR("Cannot find method \"init(...)\"");
				UERROR("%s", PyUtil::getTraceback().c_str());
			}
			UDEBUG("init time = %fs", timer.ticks());
		}
	}
}

GlobalDescriptor PyDescriptor::extract(
		  const SensorData & data) const
{
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
			UERROR("%s", PyUtil::getTraceback().c_str());
		}
		else
		{
			UDEBUG("Python extraction time = %fs", timer.ticks());

			/*
			PyArrayObject *np_ret = reinterpret_cast<PyArrayObject*>(pReturn);

			// Convert back to C++ array and print.
			int len1 = PyArray_SHAPE(np_ret)[0];
			int len2 = PyArray_SHAPE(np_ret)[1];
			int type = PyArray_TYPE(np_ret);
			UDEBUG("Matches array %dx%d (type=%d)", len1, len2, type);
			UASSERT_MSG(type == NPY_LONG || type == NPY_INT, uFormat("Returned matches should type INT=5 or LONG=7, received type=%d", type).c_str());
			if(type == NPY_LONG)
			{
				long* c_out = reinterpret_cast<long*>(PyArray_DATA(np_ret));
				for (int i = 0; i < len1*len2; i+=2)
				{
					matches.push_back(cv::DMatch(c_out[i], c_out[i+1], 0));
				}
			}
			else // INT
			{
				int* c_out = reinterpret_cast<int*>(PyArray_DATA(np_ret));
				for (int i = 0; i < len1*len2; i+=2)
				{
					matches.push_back(cv::DMatch(c_out[i], c_out[i+1], 0));
				}
			}*/
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
