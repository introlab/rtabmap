#ifndef RTABMAP_CORELIB_TEST_TESTUTILS_H_
#define RTABMAP_CORELIB_TEST_TESTUTILS_H_

#include <cstdlib>
#include <string>
#include <rtabmap/utilite/UDirectory.h>
#include <rtabmap/utilite/UFile.h>

#ifdef _WIN32
#include <process.h>
#else
#include <unistd.h>
#endif

namespace rtabmap {
namespace test {

// Portable PID. MSVC has _getpid() in <process.h>; POSIX has getpid() in
// <unistd.h>. Tests typically use this to disambiguate temp file names
// across parallel runs.
inline int getPid()
{
#ifdef _WIN32
	return _getpid();
#else
	return ::getpid();
#endif
}

// Portable replacement for hardcoded "/tmp/<name>". /tmp doesn't exist on
// Windows so any test that wrote there failed to open the file. This honors
// TEMP/TMP on Windows and TMPDIR on POSIX, with an OS-appropriate fallback.
// Forward slash is accepted by Win32 APIs so we don't bother converting.
inline std::string tempPath(const std::string & name)
{
#ifdef _WIN32
	const char * dir = std::getenv("TEMP");
	if(!dir) dir = std::getenv("TMP");
	if(!dir) dir = "C:/Temp";
#else
	const char * dir = std::getenv("TMPDIR");
	if(!dir) dir = "/tmp";
#endif
	return std::string(dir) + "/" + name;
}

// Temporary directory under tempPath(), unique per process (<name>_<pid>), created
// on construction. On destruction, its files and the directory are removed (not
// recursive: sub-directories are not supported).
class TempDir
{
public:
	explicit TempDir(const std::string & name) :
		path_(tempPath(name + "_" + std::to_string(getPid())))
	{
		UDirectory::makeDir(path_);
	}
	~TempDir()
	{
		const std::list<std::string> fileNames = UDirectory(path_).getFileNames();
		for(const std::string & fileName : fileNames)
		{
			UFile::erase(path_ + "/" + fileName);
		}
		UDirectory::removeDir(path_);
	}
	TempDir(const TempDir &) = delete;
	TempDir & operator=(const TempDir &) = delete;
	const std::string & path() const {return path_;}

private:
	std::string path_;
};

} // namespace test
} // namespace rtabmap

#endif // RTABMAP_CORELIB_TEST_TESTUTILS_H_
