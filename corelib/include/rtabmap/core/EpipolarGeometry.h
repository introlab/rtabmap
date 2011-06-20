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

#pragma once

#include "rtabmap/core/RtabmapExp.h" // DLL export/import defines

#include <opencv2/core/core.hpp>

namespace rtabmap
{

//epipolar geometry
void RTABMAP_EXP findEpipolesFromF(const cv::Mat & fundamentalMatrix, cv::Vec3d & e1, cv::Vec3d & e2);
void RTABMAP_EXP findPFromF(const cv::Mat & fundamentalMatrix, cv::Mat & p2, cv::Vec3d e2 = cv::Vec3d());

} // namespace rtabmap
