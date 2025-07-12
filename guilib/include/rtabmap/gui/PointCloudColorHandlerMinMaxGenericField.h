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

#ifndef GUILIB_SRC_POINTCLOUDCOLORHANDLEMINMAXGENERICFIELD_H_
#define GUILIB_SRC_POINTCLOUDCOLORHANDLEMINMAXGENERICFIELD_H_

#include <limits>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/pcl_config.h>

namespace rtabmap
{

	/// Same than pcl::visualization::PointCloudColorHandlerGenericField but with min and max parameters
	class PointCloudColorHandlerMinMaxGenericField : public pcl::visualization::PointCloudColorHandler<pcl::PCLPointCloud2>
	{
		using PointCloud = typename PointCloudColorHandler<pcl::PCLPointCloud2>::PointCloud;
		using PointCloudPtr = typename PointCloud::Ptr;
		using PointCloudConstPtr = typename PointCloud::ConstPtr;

	public:
		/** \brief Constructor. */
		PointCloudColorHandlerMinMaxGenericField(const PointCloudConstPtr &cloud,
												 const std::string &field_name,
												 float min = std::numeric_limits<float>::lowest(),
												 float max = std::numeric_limits<float>::max(),
												 bool inverted_color_scale = false)
			: pcl::visualization::PointCloudColorHandler<pcl::PCLPointCloud2>(cloud),
			  field_name_(field_name),
			  min_(min),
			  max_(max),
			  inverted_color_scale_(inverted_color_scale)
		{
			setInputCloud(cloud);
		}

		/** \brief Destructor. */
		virtual ~PointCloudColorHandlerMinMaxGenericField() {}

		/** \brief Get the name of the field used. */
		virtual std::string getFieldName() const { return (field_name_); }

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
			    scalars = vtkSmartPointer<vtkFloatArray>::New ();
			scalars->SetNumberOfComponents(1);

			vtkIdType nr_points = cloud_->width * cloud_->height;
			scalars->SetNumberOfTuples(nr_points);

			float *colors = new float[nr_points];
			float field_data;
			int j = 0;
			int point_offset = cloud_->fields[field_idx_].offset;

			// If XYZ present, check if the points are invalid
			int x_idx = pcl::getFieldIndex(*cloud_, "x");
			if (x_idx != -1)
			{
				float x_data, y_data, z_data;
				int x_point_offset = cloud_->fields[x_idx].offset;

				// Color every point
				for (vtkIdType cp = 0; cp < nr_points; ++cp,
							   point_offset += cloud_->point_step,
							   x_point_offset += cloud_->point_step)
				{
					memcpy(&x_data, &cloud_->data[x_point_offset], sizeof(float));
					memcpy(&y_data, &cloud_->data[x_point_offset + sizeof(float)], sizeof(float));
					memcpy(&z_data, &cloud_->data[x_point_offset + 2 * sizeof(float)], sizeof(float));
					if (!std::isfinite(x_data) || !std::isfinite(y_data) || !std::isfinite(z_data))
						continue;

					// Copy the value at the specified field
					memcpy(&field_data, &cloud_->data[point_offset], pcl::getFieldSize(cloud_->fields[field_idx_].datatype));
					if(field_data < min_) {
						field_data = min_;
					}
					if(field_data > max_) {
						field_data = max_;
					}
					colors[j] = field_data * (inverted_color_scale_?-1.0f:1.0f);
					j++;
				}
			}
			// No XYZ data checks
			else
			{
				// Color every point
				for (vtkIdType cp = 0; cp < nr_points; ++cp, point_offset += cloud_->point_step)
				{
					// Copy the value at the specified field
					// memcpy (&field_data, &cloud_->data[point_offset], sizeof (float));
					memcpy(&field_data, &cloud_->data[point_offset], pcl::getFieldSize(cloud_->fields[field_idx_].datatype));

					if (!std::isfinite(field_data))
						continue;

					if(field_data < min_) {
						field_data = min_;
					}
					if(field_data > max_) {
						field_data = max_;
					}
					
					colors[j] = field_data * (inverted_color_scale_?-1.0f:1.0f);
					j++;
				}
			}
			reinterpret_cast<vtkFloatArray *>(&(*scalars))->SetArray(colors, j, 0, vtkFloatArray::VTK_DATA_ARRAY_DELETE);

#if PCL_VERSION_COMPARE(>, 1, 11, 1)
			return scalars;
#else
			return (true);
#endif
		}

		using PointCloudColorHandler<pcl::PCLPointCloud2>::getColor;

		/** \brief Set the input cloud to be used.
		 * \param[in] cloud the input cloud to be used by the handler
		 */
		virtual void
		setInputCloud(const PointCloudConstPtr &cloud)
		{
			PointCloudColorHandler<pcl::PCLPointCloud2>::setInputCloud(cloud);
			field_idx_ = pcl::getFieldIndex(*cloud, field_name_);
			capable_ = field_idx_ != -1;
			if (field_idx_ != -1 && cloud_->fields[field_idx_].datatype != pcl::PCLPointField::PointFieldTypes::FLOAT32)
			{
				capable_ = false;
				PCL_ERROR("[pcl::PointCloudColorHandlerGenericField] This currently only works with float32 fields, but field %s has a different type.\n", field_name_.c_str());
			}
		}

	protected:
		/** \brief Class getName method. */
		virtual std::string
		getName() const { return ("PointCloudColorHandlerMinMaxGenericField"); }

	private:
		/** \brief Name of the field used to create the color handler. */
		std::string field_name_;
		float min_;
		float max_;
		bool inverted_color_scale_;
	};

} /* namespace rtabmap */

#endif /* GUILIB_SRC_POINTCLOUDCOLORHANDLEMINMAXGENERICFIELD_H_ */
