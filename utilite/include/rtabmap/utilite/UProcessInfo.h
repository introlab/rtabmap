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
 * This class is used to get some informations
 * about the current process.
 */
class UTILITE_EXPORT UProcessInfo {
public:
	UProcessInfo();
	virtual ~UProcessInfo();

	/**
	 * Get the memory used by the current process.
	 * @return the number of bytes used by the current process.
	 */
	static long int getMemoryUsage();
};

#endif /* UPROCESSINFO_H */
