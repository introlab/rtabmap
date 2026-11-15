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

#ifndef FILE_H
#define FILE_H

#include "rtabmap/utilite/utilite_export.h" // DLL export/import defines

#include "rtabmap/utilite/UDirectory.h"
#include <string>

/**
 * Class UFile.
 *
 * This class can be used to modify/erase files on hard drive.
 */
class UTILITE_EXPORT UFile
{
public:
	/**
	 * Check if a file exists.
	 * @param filePath the file path
	 * @return true if the file exists, otherwise false.
	 */
	static bool exists(const std::string &filePath);

	/**
	 * Get the file length.
	 * @param filePath the file path
	 * @return long the length of the file in bytes. Return -1 if the file doesn't exist.
	 */
	static long length(const std::string &filePath);

	/**
	 * Erase a file.
	 * @param filePath the file path
	 * @return 0 if success.
	 */
	static int erase(const std::string &filePath);

	/**
	 * Rename a file.
	 * @param oldFilePath the old file path
	 * @param newFilePath the new file path
	 * @return 0 if success.
	 */
	static int rename(const std::string &oldFilePath,
						  const std::string &newFilePath);

	/**
	 * Get the file name from a file path (with extension).
	 * @param filePath the file path
	 * @return the file name.
	 */
	static std::string getName(const std::string & filePath);

	/**
	 * Get the file extension.
	 * @return the file extension
	 */
	static std::string getExtension(const std::string &filePath);

	/**
	 * Copy a file.
	 * @param from the file path
	 * @param to destination file path
	 */
	static void copy(const std::string & from, const std::string & to);

public:
	/**
	 * Create a UFile object with path initialized to an existing file .
	 * @param path the path to an existing file
	 */
	UFile(const std::string & path) : path_(path) {}
	~UFile() {}

	/**
	 * Check if the file exists. Same as exists().
	 * @return true if the path exits
	 */
	bool isValid() {return exists(path_);}

	/**
	 * Check if the file exists.
	 * @return true if the path exits
	 */
	bool exists() {return exists(path_);}

	/**
	 * Get the length of the file.
	 * @return long the length of the file in bytes. Return -1 if the file doesn't exist.
	 */
	long length() {return length(path_);}

	/**
	 * Rename the file name. The path stays the same.
	 * @param the new name
	 */
	int rename(const std::string &newName)
	{
		std::string ext = this->getExtension();
		std::string newPath = UDirectory::getDir(path_) + std::string("/") + newName;
		if(ext.size())
		{
			newPath += std::string(".") + getExtension(path_);
		}
		int result = rename(path_, newPath);
		if(result == 0)
		{
			path_ = newPath;
		}
		return result;
	}
	/**
	 * Get the file name without the path.
	 * @return the file name
	 */
	std::string getName() {return getName(path_);}
	/**
	 * Get the file extension.
	 * @return the file extension
	 */
	std::string getExtension() {return getExtension(path_);}

	/**
	 * Copy a file.
	 * @param to destination file path
	 */
	void copy(const std::string & to) {copy(path_, to);}

private:
	std::string path_;
};

#endif
