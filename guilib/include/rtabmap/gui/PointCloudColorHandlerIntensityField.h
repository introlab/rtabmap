/*
Copyright (c) 2010-2025, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
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

#ifndef GUILIB_SRC_POINTCLOUDCOLORHANDLEINTENSITYFIELD_H_
#define GUILIB_SRC_POINTCLOUDCOLORHANDLEINTENSITYFIELD_H_

#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/pcl_config.h>
#include <rtabmap/core/util2d.h>
#include <rtabmap/utilite/UMath.h>

namespace rtabmap
{

class PointCloudColorHandlerIntensityField : public pcl::visualization::PointCloudColorHandler<pcl::PCLPointCloud2>
{
	typedef pcl::visualization::PointCloudColorHandler<pcl::PCLPointCloud2>::PointCloud PointCloud;
	typedef PointCloud::Ptr PointCloudPtr;
	typedef PointCloud::ConstPtr PointCloudConstPtr;

public:
	/** \brief Constructor. */
	PointCloudColorHandlerIntensityField(const PointCloudConstPtr &cloud, float maxAbsIntensity = 0.0f, int colorMap = 0) : pcl::visualization::PointCloudColorHandler<pcl::PCLPointCloud2>::PointCloudColorHandler(cloud),
																															maxAbsIntensity_(maxAbsIntensity),
																															colormap_(colorMap)
	{
		field_idx_ = pcl::getFieldIndex(*cloud, "intensity");
		if (field_idx_ != -1)
			capable_ = true;
		else
			capable_ = false;
	}

	/** \brief Empty destructor */
	virtual ~PointCloudColorHandlerIntensityField() {}

	/** \brief Obtain the actual color for the input dataset as vtk scalars.
	 * \param[out] scalars the output scalars containing the color for the dataset
	 * \return true if the operation was successful (the handler is capable and
	 * the input cloud was given as a valid pointer), false otherwise
	 */
#if PCL_VERSION_COMPARE(>, 1, 11, 1)
	virtual vtkSmartPointer<vtkDataArray> getColor() const
	{
		vtkSmartPointer<vtkDataArray> scalars;
		if (!capable_ || !cloud_)
			return scalars;
#else
	virtual bool getColor(vtkSmartPointer<vtkDataArray> &scalars) const
	{
		if (!capable_ || !cloud_)
			return (false);
#endif
		if (!scalars)
			scalars = vtkSmartPointer<vtkUnsignedCharArray>::New();
		scalars->SetNumberOfComponents(3);

		vtkIdType nr_points = cloud_->width * cloud_->height;
		// Allocate enough memory to hold all colors
		float *intensities = new float[nr_points];
		float intensity;
		size_t point_offset = cloud_->fields[field_idx_].offset;
		size_t j = 0;

		// If XYZ present, check if the points are invalid
		int x_idx = pcl::getFieldIndex(*cloud_, "x");
		if (x_idx != -1)
		{
			float x_data, y_data, z_data;
			size_t x_point_offset = cloud_->fields[x_idx].offset;

			// Color every point
			for (vtkIdType cp = 0; cp < nr_points; ++cp,
							point_offset += cloud_->point_step,
							x_point_offset += cloud_->point_step)
			{
				// Copy the value at the specified field
				memcpy(&intensity, &cloud_->data[point_offset], sizeof(float));

				memcpy(&x_data, &cloud_->data[x_point_offset], sizeof(float));
				memcpy(&y_data, &cloud_->data[x_point_offset + sizeof(float)], sizeof(float));
				memcpy(&z_data, &cloud_->data[x_point_offset + 2 * sizeof(float)], sizeof(float));

				if (!std::isfinite(x_data) || !std::isfinite(y_data) || !std::isfinite(z_data))
					continue;

				intensities[j++] = intensity;
			}
		}
		// No XYZ data checks
		else
		{
			// Color every point
			for (vtkIdType cp = 0; cp < nr_points; ++cp, point_offset += cloud_->point_step)
			{
				// Copy the value at the specified field
				memcpy(&intensity, &cloud_->data[point_offset], sizeof(float));

				intensities[j++] = intensity;
			}
		}
		if (j != 0)
		{
			// Allocate enough memory to hold all colors
			unsigned char *colors = new unsigned char[j * 3];
			float min, max;
			if (maxAbsIntensity_ > 0.0f)
			{
				max = maxAbsIntensity_;
			}
			else
			{
				uMinMax(intensities, j, min, max);
			}
			for (size_t k = 0; k < j; ++k)
			{
				colors[k * 3 + 0] = colors[k * 3 + 1] = colors[k * 3 + 2] = max > 0 ? (unsigned char)(std::min(intensities[k] / max * 255.0f, 255.0f)) : 255;
				if (colormap_ == 1)
				{
					colors[k * 3 + 0] = 255;
					colors[k * 3 + 2] = 0;
				}
				else if (colormap_ == 2)
				{
					float r, g, b;
					util2d::HSVtoRGB(&r, &g, &b, colors[k * 3 + 0] * 299.0f / 255.0f, 1.0f, 1.0f);
					colors[k * 3 + 0] = r * 255.0f;
					colors[k * 3 + 1] = g * 255.0f;
					colors[k * 3 + 2] = b * 255.0f;
				}
			}
			reinterpret_cast<vtkUnsignedCharArray *>(&(*scalars))->SetNumberOfTuples(j);
			reinterpret_cast<vtkUnsignedCharArray *>(&(*scalars))->SetArray(colors, j * 3, 0, vtkUnsignedCharArray::VTK_DATA_ARRAY_DELETE);
		}
		else
			reinterpret_cast<vtkUnsignedCharArray *>(&(*scalars))->SetNumberOfTuples(0);
		// delete [] colors;
		delete[] intensities;
#if PCL_VERSION_COMPARE(>, 1, 11, 1)
		return scalars;
#else
		return (true);
#endif
	}

protected:
	/** \brief Get the name of the class. */
	virtual std::string
	getName() const { return ("PointCloudColorHandlerIntensityField"); }

	/** \brief Get the name of the field used. */
	virtual std::string
	getFieldName() const { return ("intensity"); }

private:
	float maxAbsIntensity_;
	int colormap_; // 0=grayscale, 1=redYellow, 2=RainbowHSV
};

} /* namespace rtabmap */

#endif /* GUILIB_SRC_POINTCLOUDCOLORHANDLEINTENSITYFIELD_H_ */