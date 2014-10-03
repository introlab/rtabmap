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

#ifndef UTHREADC_H
#define UTHREADC_H

#include <errno.h>

/*
 * Use of StateThread is safer. The Thread class in his
 * base form is not supported.
 * @see StateThread
 */
#ifdef _WIN32
  #include "rtabmap/utilite/Win32/UThreadC.h"
#else
  #include "rtabmap/utilite/Posix/UThreadC.h"
#endif

#endif // UTHREADC_H
