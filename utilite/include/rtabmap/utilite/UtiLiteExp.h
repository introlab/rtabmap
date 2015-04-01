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

#ifndef UTILITEEXP_H
#define UTILITEEXP_H

/**
 * Used mainly on Windows for dynamic linked libraries (dll).
 */
#if defined(_WIN32)
  #if defined(rtabmap_utilite_EXPORTS)
    #define UTILITE_EXP   __declspec( dllexport )
  #else
    #define UTILITE_EXP   __declspec( dllimport )
  #endif
#else
  #define UTILITE_EXP
#endif

#ifdef __GNUC__
#define UTILITE_DEPRECATED(func) func __attribute__ ((deprecated))
#elif defined(_MSC_VER)
#define UTILITE_DEPRECATED(func) __declspec(deprecated) func
#else
#pragma message("WARNING: You need to implement DEPRECATED for this compiler")
#define UTILITE_DEPRECATED(func) func
#endif

#endif

