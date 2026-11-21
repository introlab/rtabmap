/*
Copyright (c) 2010, Mathieu Labbe
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holders nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "rtabmap/utilite/UDirectory.h"

#ifdef _WIN32
  #include <Windows.h>
  #include <direct.h>
  #include <algorithm>
  #include <conio.h>
#else
  #include <dirent.h>
  #include <sys/stat.h>
  #include <sys/types.h>
  #include <sys/param.h>
  #include <unistd.h>
  #include <stdlib.h>
  #include <string.h>
  #include <pwd.h>
#endif

#include "rtabmap/utilite/UStl.h"
#include "rtabmap/utilite/UFile.h"
#include "rtabmap/utilite/UDirectory.h"
#include "rtabmap/utilite/UConversion.h"

#include "rtabmap/utilite/ULogger.h"

#ifdef _WIN32

bool sortCallback(const std::string & a, const std::string & b)
{
	return uStrNumCmp(a,b) < 0;
}
#elif __APPLE__
int sortCallback(const struct dirent ** a, const struct dirent ** b)
{
	return uStrNumCmp((*a)->d_name, (*b)->d_name);
}
#else
int sortCallback( const dirent ** a,  const dirent ** b)
{
	return uStrNumCmp((*a)->d_name, (*b)->d_name);
}
#endif

UDirectory::UDirectory(const std::string & path, const std::string & extensions)
{
	extensions_ = uListToVector(uSplit(extensions, ' '));
	path_ = path;
	iFileName_ = fileNames_.begin();
	this->update();
}

UDirectory::UDirectory(const UDirectory & dir)
{
	*this = dir;
}

UDirectory & UDirectory::operator=(const UDirectory & dir)
{
	extensions_ = dir.extensions_;
	path_ = dir.path_;
	fileNames_ = dir.fileNames_;
	for(iFileName_=fileNames_.begin(); iFileName_!=fileNames_.end(); ++iFileName_)
	{
		if(iFileName_->compare(*dir.iFileName_) == 0)
		{
			break;
		}
	}
	return *this;
}

UDirectory::~UDirectory()
{
}

void UDirectory::setPath(const std::string & path, const std::string & extensions)
{
	extensions_ = uListToVector(uSplit(extensions, ' '));
	path_ = path;
	fileNames_.clear();
	iFileName_ = fileNames_.begin();
	this->update();
}

void UDirectory::update()
{
	if(exists(path_))
	{
		std::string lastName;
		bool endOfDir = false;
		if(iFileName_ != fileNames_.end())
		{
			//Record the last file name
			lastName = *iFileName_;
		}
		else if(fileNames_.size())
		{
			lastName = *fileNames_.rbegin();
			endOfDir = true;
		}
		fileNames_.clear();
#ifdef _WIN32
		WIN32_FIND_DATA fileInformation;
	#ifdef UNICODE
		wchar_t * pathAll = createWCharFromChar((path_+"\\*").c_str());
		HANDLE hFile  = ::FindFirstFile(pathAll, &fileInformation);
		delete [] pathAll;
	#else
		HANDLE hFile  = ::FindFirstFile((path_+"\\*").c_str(), &fileInformation);
	#endif
		if(hFile != INVALID_HANDLE_VALUE)
		{
			do
			{
	#ifdef UNICODE
				char * fileName = createCharFromWChar(fileInformation.cFileName);
				fileNames_.push_back(fileName);
				delete [] fileName;
	#else
				fileNames_.push_back(fileInformation.cFileName);
	#endif
			} while(::FindNextFile(hFile, &fileInformation) == TRUE);
			::FindClose(hFile);
			std::vector<std::string> vFileNames = uListToVector(fileNames_);
			std::sort(vFileNames.begin(), vFileNames.end(), sortCallback);
			fileNames_ = uVectorToList(vFileNames);
		}
#else
		int nameListSize;
		struct dirent ** nameList = 0;
		nameListSize =	scandir(path_.c_str(), &nameList, 0, sortCallback);
		if(nameList && nameListSize>0)
		{
			for (int i=0;i<nameListSize;++i)
			{
				fileNames_.push_back(nameList[i]->d_name);
				free(nameList[i]);
			}
			free(nameList);
		}
#endif

		//filter extensions...
		std::list<std::string>::iterator iter = fileNames_.begin();
		bool valid;
		while(iter!=fileNames_.end())
		{
			valid = false;
			if(extensions_.size() == 0 &&
			   iter->compare(".") != 0 &&
			   iter->compare("..") != 0)
			{
				valid = true;
			}
			for(unsigned int i=0; i<extensions_.size(); ++i)
			{
				if(UFile::getExtension(*iter).compare(extensions_[i]) == 0)
				{
					valid = true;
					break;
				}
			}
			if(!valid)
			{
				iter = fileNames_.erase(iter);
			}
			else
			{
				++iter;
			}
		}
		iFileName_ = fileNames_.begin();
		if(!lastName.empty())
		{
			bool found = false;
			for(std::list<std::string>::iterator iter=fileNames_.begin(); iter!=fileNames_.end(); ++iter)
			{
				if(lastName.compare(*iter) == 0)
				{
					found = true;
					iFileName_ = iter;
					break;
				}
			}
			if(endOfDir && found)
			{
				++iFileName_;
			}
			else if(endOfDir && fileNames_.size())
			{
				iFileName_ = --fileNames_.end();
			}
		}
	}
}

bool UDirectory::isValid()
{
	return exists(path_);
}

std::string UDirectory::getNextFileName()
{
	std::string fileName;
	if(iFileName_ != fileNames_.end())
	{
		fileName = *iFileName_;
		++iFileName_;
	}
	return fileName;
}

std::string UDirectory::getNextFilePath()
{
	std::string filePath;
	if(iFileName_ != fileNames_.end())
	{
		filePath = path_+separator()+*iFileName_;
		++iFileName_;
	}
	return filePath;
}

void UDirectory::rewind()
{
	iFileName_ = fileNames_.begin();
}


bool UDirectory::exists(const std::string & dirPath)
{
	bool r = false;
#ifdef _WIN32
	#ifdef UNICODE
	wchar_t * wDirPath = createWCharFromChar(dirPath.c_str());
	DWORD dwAttrib = GetFileAttributes(wDirPath);
	delete [] wDirPath;
	#else
	DWORD dwAttrib = GetFileAttributes(dirPath.c_str());
	#endif
	r = (dwAttrib != INVALID_FILE_ATTRIBUTES && (dwAttrib & FILE_ATTRIBUTE_DIRECTORY));
#else
	DIR *dp;
	if((dp  = opendir(dirPath.c_str())) != NULL)
	{
		r = true;
		closedir(dp);
	}
#endif
	return r;
}

// return the directory path of the file
std::string UDirectory::getDir(const std::string & filePath)
{
	std::string dir = filePath;
	int i=(int)dir.size()-1;
	for(; i>=0; --i)
	{
		if(dir[i] == '/' || dir[i] == '\\')
		{
			if(i == 0)
			{
				// leading /
				break;
			}
			//remove separators...
			dir[i] = 0;
			--i;
			while(i>=0 && (dir[i] == '/' || dir[i] == '\\'))
			{
				dir[i] = 0;
				--i;
			}
			break;
		}
		else
		{
			dir[i] = 0;
		}
	}

	if(i<0)
	{
		dir = ".";
	}
	else
	{
		dir.resize(i+1);
	}

	return dir;
}

std::string UDirectory::currentDir(bool trailingSeparator)
{
	std::string dir;
	char * buffer;

#ifdef _WIN32
	buffer = _getcwd(NULL, 0);
#else
	buffer = getcwd(NULL, MAXPATHLEN);
#endif

	if( buffer != NULL )
	{
		dir = buffer;
		free(buffer);
		if(trailingSeparator)
		{
			dir += separator();
		}
	}

	return dir;
}

bool UDirectory::makeDir(const std::string & dirPath)
{
	int status;
#ifdef _WIN32
	status = _mkdir(dirPath.c_str());
#else
	status = mkdir(dirPath.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
#endif
	return status==0;
}

bool UDirectory::removeDir(const std::string & dirPath)
{
	int status;
#ifdef _WIN32
	status = _rmdir(dirPath.c_str());
#else
	status = rmdir(dirPath.c_str());
#endif
	return status==0;
}

std::string UDirectory::homeDir()
{
	std::string path;
#ifdef _WIN32
	#ifdef UNICODE
	wchar_t wProfilePath[250];
	ExpandEnvironmentStrings(L"%userprofile%",wProfilePath,250);
	char * profilePath = createCharFromWChar(wProfilePath);
	path = profilePath;
	delete [] profilePath;
	#else
	char profilePath[250];
	ExpandEnvironmentStrings("%userprofile%",profilePath,250);
	path = profilePath;
	#endif
#else
	char * pathstr = getenv("HOME");
	if(pathstr)
	{
		path = pathstr;
	}
	if(path.empty())
	{
		struct passwd *pw = getpwuid(getuid());
		if(pw) {
			path = pw->pw_dir;
		}
		if(path.empty())
		{
			UFATAL("Environment variable HOME is not set, cannot get home directory! Please set HOME environment variable to a valid directory.");
		}
	}
#endif
	return path;
}

std::string UDirectory::expandHomeDir(const std::string & path)
{
	if(!path.empty() && path[0] == '~' && (path.size() == 1 || path[1] == '/' || path[1] == '\\'))
	{
		return homeDir() + path.substr(1);
	}
	return path;
}

std::string UDirectory::separator()
{
#ifdef _WIN32
	return "\\";
#else
	return "/";
#endif
}
