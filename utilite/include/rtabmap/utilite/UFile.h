/*
*  utilite is a cross-platform library with
*  useful utilities for fast and small developing.
*  Copyright (C) 2010  Mathieu Labbe
*
*  utilite is free library: you can redistribute it and/or modify
*  it under the terms of the GNU Lesser General Public License as published by
*  the Free Software Foundation, either version 3 of the License, or
*  (at your option) any later version.
*
*  utilite is distributed in the hope that it will be useful,
*  but WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*  GNU Lesser General Public License for more details.
*
*  You should have received a copy of the GNU Lesser General Public License
*  along with this program.  If not, see <http://www.gnu.org/licenses/>.
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
