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

#ifndef UPROCESSINFO_H
#define UPROCESSINFO_H

#include "rtabmap/utilite/utilite_export.h" // DLL export/import defines

/**
 * \file UProcessInfo.h
 * \brief UProcessInfo class for getting process information
 *
 * This class provides methods to retrieve information about the current process,
 * such as memory usage.
 */

/**
 * \class UProcessInfo
 * \brief Class for getting information about the current process
 *
 * This class provides static methods to retrieve information about the current
 * process, such as memory usage. The information is retrieved from the operating
 * system.
 *
 * Example:
 * @code
 * long int memory = UProcessInfo::getMemoryUsage();
 * std::cout << "Memory usage: " << memory << " bytes" << std::endl;
 * @endcode
 */
class UTILITE_EXPORT UProcessInfo {
public:
	/**
	 * \brief Constructor
	 */
	UProcessInfo();
	
	/**
	 * \brief Virtual destructor
	 */
	virtual ~UProcessInfo();

	/**
	 * \brief Get the memory used by the current process
	 * 
	 * This method returns the amount of memory currently used by the process.
	 * The exact meaning of "memory used" may vary between operating systems.
	 * 
	 * @return the number of bytes used by the current process, or -1 on error
	 */
	static long int getMemoryUsage();
};

#endif /* UPROCESSINFO_H */
