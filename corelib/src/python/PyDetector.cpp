/**
 * Python interface for SuperGlue: https://github.com/magicleap/SuperGluePretrainedNetwork
 */

#include "PyDetector.h"
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

PyDetector::PyDetector(const ParametersMap & parameters) :
		pModule_(0),
		pFunc_(0),
		path_(Parameters::defaultPyDetectorPath()),
		cuda_(Parameters::defaultPyDetectorCuda())
{
	this->parseParameters(parameters);

	UDEBUG("path = %s", path_.c_str());
	if(!UFile::exists(path_) || UFile::getExtension(path_).compare("py") != 0)
	{
		UERROR("Cannot initialize Python detector, the path is not valid: \"%s\"=\"%s\"",
				Parameters::kPyDetectorPath().c_str(), path_.c_str());
		return;
	}

	pybind11::gil_scoped_acquire acquire;
	
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
}

PyDetector::~PyDetector()
{
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

void PyDetector::parseParameters(const ParametersMap & parameters)
{
	Feature2D::parseParameters(parameters);

	Parameters::parse(parameters, Parameters::kPyDetectorPath(), path_);
	Parameters::parse(parameters, Parameters::kPyDetectorCuda(), cuda_);

	path_ = uReplaceChar(path_, '~', UDirectory::homeDir());
}

std::vector<cv::KeyPoint> PyDetector::generateKeypointsImpl(const cv::Mat & image, const cv::Rect & roi, const cv::Mat & mask)
{
	UDEBUG("");
	descriptors_ = cv::Mat();
	UASSERT(!image.empty() && image.channels() == 1 && image.depth() == CV_8U);
	std::vector<cv::KeyPoint> keypoints;
	cv::Mat imgRoi(image, roi);

	UTimer timer;

	if(!pModule_)
	{
		UERROR("Python detector module not loaded!");
		return keypoints;
	}

	pybind11::gil_scoped_acquire acquire;

	if(!pFunc_)
	{
		PyObject * pFunc = PyObject_GetAttrString(pModule_, "init");
		if(pFunc)
		{
			if(PyCallable_Check(pFunc))
			{
				PyObject * result = PyObject_CallFunction(pFunc, "i", cuda_?1:0);

				if(result == NULL)
				{
					UERROR("Call to \"init(...)\" in \"%s\" failed!", path_.c_str());
					UERROR("%s", getPythonTraceback().c_str());
					return keypoints;
				}
				Py_DECREF(result);

				pFunc_ = PyObject_GetAttrString(pModule_, "detect");
				if(pFunc_ && PyCallable_Check(pFunc_))
				{
					// we are ready!
				}
				else
				{
					UERROR("Cannot find method \"detect(...)\" in %s", path_.c_str());
					UERROR("%s", getPythonTraceback().c_str());
					if(pFunc_)
					{
						Py_DECREF(pFunc_);
						pFunc_ = 0;
					}
					return keypoints;
				}
			}
			else
			{
				UERROR("Cannot call method \"init(...)\" in %s", path_.c_str());
				UERROR("%s", getPythonTraceback().c_str());
				return keypoints;
			}
			Py_DECREF(pFunc);
		}
		else
		{
			UERROR("Cannot find method \"init(...)\"");
			UERROR("%s", getPythonTraceback().c_str());
			return keypoints;
		}
		UDEBUG("init time = %fs", timer.ticks());
	}

	if(pFunc_)
	{
		npy_intp dims[2] = {imgRoi.rows, imgRoi.cols};
		PyObject* pImageBuffer = PyArray_SimpleNewFromData(2, dims, NPY_UBYTE, (void*)imgRoi.data);
		UASSERT(pImageBuffer);

		UDEBUG("Preparing data time = %fs", timer.ticks());

		PyObject *pReturn = PyObject_CallFunctionObjArgs(pFunc_, pImageBuffer, NULL);
		if(pReturn == NULL)
		{
			UERROR("Failed to call match() function!");
			UERROR("%s", getPythonTraceback().c_str());
		}
		else
		{
			UDEBUG("Python detector time = %fs", timer.ticks());

			if (PyTuple_Check(pReturn) && PyTuple_GET_SIZE(pReturn) == 2)
			{
				PyObject *kptsPtr = PyTuple_GET_ITEM(pReturn, 0);
				PyObject *descPtr = PyTuple_GET_ITEM(pReturn, 1);
				if(PyArray_Check(kptsPtr) && PyArray_Check(descPtr))
				{
					PyArrayObject *arrayPtr = reinterpret_cast<PyArrayObject*>(kptsPtr);
					int nKpts = PyArray_SHAPE(arrayPtr)[0];
					int kptSize = PyArray_SHAPE(arrayPtr)[1];
					int type = PyArray_TYPE(arrayPtr);
					UDEBUG("Kpts array %dx%d (type=%d)", nKpts, kptSize, type);
					UASSERT(kptSize == 3);
					UASSERT_MSG(type == NPY_FLOAT, uFormat("Returned matches should type FLOAT=11, received type=%d", type).c_str());

					float* c_out = reinterpret_cast<float*>(PyArray_DATA(arrayPtr));
					keypoints.reserve(nKpts);
					for (int i = 0; i < nKpts*kptSize; i+=kptSize)
					{
						cv::KeyPoint kpt(c_out[i], c_out[i+1], 8, -1, c_out[i+2]);
						keypoints.push_back(kpt);
					}

					arrayPtr = reinterpret_cast<PyArrayObject*>(descPtr);
					int nDesc = PyArray_SHAPE(arrayPtr)[0];
					UASSERT(nDesc = nKpts);
					int dim = PyArray_SHAPE(arrayPtr)[1];
					type = PyArray_TYPE(arrayPtr);
					UDEBUG("Desc array %dx%d (type=%d)", nDesc, dim, type);
					UASSERT_MSG(type == NPY_FLOAT, uFormat("Returned matches should type FLOAT=11, received type=%d", type).c_str());

					c_out = reinterpret_cast<float*>(PyArray_DATA(arrayPtr));
					for (int i = 0; i < nDesc*dim; i+=dim)
					{
						cv::Mat descriptor = cv::Mat(1, dim, CV_32FC1, &c_out[i]).clone();
						descriptors_.push_back(descriptor);
					}
				}
			}
			else
			{
				UWARN("Expected tuple (Kpts 3 x N, Descriptors dim x N), returning empty features.");
			}
			Py_DECREF(pReturn);
		}
		Py_DECREF(pImageBuffer);
	}

	return keypoints;
}

cv::Mat PyDetector::generateDescriptorsImpl(const cv::Mat & image, std::vector<cv::KeyPoint> & keypoints) const
{
	UASSERT((int)keypoints.size() == descriptors_.rows);
	return descriptors_;
}

}
