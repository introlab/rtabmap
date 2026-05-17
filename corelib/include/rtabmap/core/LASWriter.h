/*
Copyright (c) 2010-2024, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Universite de Sherbrooke nor the
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

#ifndef CORELIB_INCLUDE_RTABMAP_CORE_LASWRITER_H_
#define CORELIB_INCLUDE_RTABMAP_CORE_LASWRITER_H_

#include "rtabmap/core/rtabmap_core_export.h" // DLL export/import defines
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>
#include <vector>

namespace rtabmap {

/**
 * @brief Writes a PCL point cloud to a LAS/LAZ file (requires RTAB-Map built with libLAS).
 *
 * Output uses 1 mm XYZ scale (`0.001`). The file extension (`.las` or `.laz`) selects
 * uncompressed or compressed output when libLAS LAZ support is available.
 *
 * @param filePath Output path (`.las` or `.laz`).
 * @param cloud Input point cloud.
 * @param cameraIds Optional per-point camera/signature ids (stored as point source ID);
 *                  must be empty or the same length as @p cloud.
 * @return `0` on success, `1` on error (e.g. LAZ not supported).
 */
int RTABMAP_CORE_EXPORT saveLASFile(
		const std::string & filePath,
		const pcl::PointCloud<pcl::PointXYZ> & cloud,
		const std::vector<int> & cameraIds = std::vector<int>());

/**
 * @brief Writes a colored point cloud to LAS/LAZ, with optional intensity and camera ids.
 * @param filePath Output path (`.las` or `.laz`).
 * @param cloud RGB point cloud (8-bit channels mapped to 16-bit LAS color).
 * @param cameraIds Optional per-point ids (same length as @p cloud or empty).
 * @param intensities Optional per-point intensities (same length as @p cloud or empty).
 * @return `0` on success, `1` on error.
 */
int RTABMAP_CORE_EXPORT saveLASFile(
		const std::string & filePath,
		const pcl::PointCloud<pcl::PointXYZRGB> & cloud,
		const std::vector<int> & cameraIds = std::vector<int>(),
		const std::vector<float> & intensities = std::vector<float>());

/**
 * @brief Writes a point cloud with intensity channel to LAS/LAZ.
 * @param filePath Output path (`.las` or `.laz`).
 * @param cloud XYZI point cloud (coordinates and intensity written).
 * @param cameraIds Optional per-point ids (same length as @p cloud or empty).
 * @return `0` on success, `1` on error.
 */
int RTABMAP_CORE_EXPORT saveLASFile(
		const std::string & filePath,
		const pcl::PointCloud<pcl::PointXYZI> & cloud,
		const std::vector<int> & cameraIds = std::vector<int>());

} // namespace rtabmap

#endif /* CORELIB_INCLUDE_RTABMAP_CORE_LASWRITER_H_ */
