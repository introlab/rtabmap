/*
 * Copyright (C) 2010-2011, Mathieu Labbe and IntRoLab - Universite de Sherbrooke
 *
 * This file is part of RTAB-Map.
 *
 * RTAB-Map is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * RTAB-Map is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with RTAB-Map.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef RTABMAPEXP_H
#define RTABMAPEXP_H

#if defined(_WIN32)
  #if defined(rtabmap_core_EXPORTS)
    #define RTABMAP_EXP   __declspec( dllexport )
  #else
    #define RTABMAP_EXP   __declspec( dllimport )
  #endif
#else
  #define RTABMAP_EXP
#endif

#endif // RTABMAPEXP_H
