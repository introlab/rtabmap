// Tests for PythonInterface and its companion getPythonTraceback() helper.
// Mostly regression coverage for the NULL-safety fixes in getPythonTraceback
// (early-return on no exception, Py_None substitution for missing fetch
// components, NULL-safe DECREFs).
//
// The whole file is a no-op when rtabmap is built without Python -- the
// CMakeLists.txt only registers it under WITH_PYTHON AND Python3_FOUND, and
// a defensive #ifdef matches that.

#include <gtest/gtest.h>
#include <rtabmap/core/Version.h>

#ifdef RTABMAP_PYTHON

#include <rtabmap/core/PythonInterface.h>

#include <pybind11/embed.h>
#include <Python.h>

#include <string>

using namespace rtabmap;

namespace {

// All tests need a live interpreter; instance() lazily initializes it.
class PythonInterfaceTest : public ::testing::Test
{
protected:
	void SetUp() override
	{
		PythonInterface::instance("test_python_interface");
	}
};

}  // namespace

// instance() must return the same object every call and never re-initialize
// the embedded interpreter. Pinning this so any future refactor can't
// accidentally regress to "construct a new interpreter per call" (which
// pybind11's scoped_interpreter does not support in the same process).
TEST_F(PythonInterfaceTest, InstanceIsSingleton)
{
	PythonInterface & a = PythonInterface::instance("test_a");
	PythonInterface & b = PythonInterface::instance("test_b");
	EXPECT_EQ(&a, &b);
}

// No active Python exception -> the helper returns a clear sentinel instead
// of crashing inside Py_BuildValue("OOO", NULL, NULL, NULL). The sentinel
// makes it obvious in logs that there was nothing useful to report.
TEST_F(PythonInterfaceTest, TracebackEmptyWhenNoException)
{
	pybind11::gil_scoped_acquire acquire;
	PyErr_Clear();
	ASSERT_FALSE(PyErr_Occurred());

	EXPECT_EQ("<no python exception set>", getPythonTraceback());
}

// With a real exception set, the formatter returns a non-empty string that
// includes the original message. getPythonTraceback consumes the exception
// (PyErr_Fetch clears it), so PyErr_Occurred is false afterward.
TEST_F(PythonInterfaceTest, TracebackFormattedAfterException)
{
	pybind11::gil_scoped_acquire acquire;
	PyErr_Clear();
	PyErr_SetString(PyExc_RuntimeError, "test error message");

	const std::string trace = getPythonTraceback();
	EXPECT_FALSE(trace.empty());
	EXPECT_NE("<no python exception set>", trace);
	EXPECT_NE(std::string::npos, trace.find("test error message"));
	EXPECT_FALSE(PyErr_Occurred());
}

// Repeated calls on the no-exception path must stay stable. The original
// crashes (Py_DECREF on NULL outStr, Py_BuildValue on NULL slots) surfaced
// non-deterministically after several invocations -- this loop would have
// reliably tripped them.
TEST_F(PythonInterfaceTest, RepeatedCallsAreStable)
{
	pybind11::gil_scoped_acquire acquire;
	for(int i = 0; i < 25; ++i)
	{
		PyErr_Clear();
		EXPECT_EQ("<no python exception set>", getPythonTraceback());
	}

	// Same robustness check with an exception each time.
	for(int i = 0; i < 25; ++i)
	{
		PyErr_SetString(PyExc_ValueError, "boom");
		const std::string trace = getPythonTraceback();
		EXPECT_FALSE(trace.empty());
		EXPECT_NE("<no python exception set>", trace);
	}
}

#endif  // RTABMAP_PYTHON
