/*
Copyright (c) 2010-2014, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
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

#include "rtabmap/core/Odometry.h"
#include "rtabmap/core/util3d_registration.h"
#include "rtabmap/core/util3d_filtering.h"
#include "rtabmap/core/util3d_surface.h"
#include "rtabmap/core/OdometryInfo.h"
#include "rtabmap/utilite/ULogger.h"
#include "rtabmap/utilite/UTimer.h"

namespace rtabmap {

OdometryICP::OdometryICP(int decimation,
		float voxelSize,
		int samples,
		float maxCorrespondenceDistance,
		int maxIterations,
		float correspondenceRatio,
		bool pointToPlane,
		const ParametersMap & odometryParameter) :
	Odometry(odometryParameter),
	_decimation(decimation),
	_voxelSize(voxelSize),
	_samples(samples),
	_maxCorrespondenceDistance(maxCorrespondenceDistance),
	_maxIterations(maxIterations),
	_correspondenceRatio(correspondenceRatio),
	_pointToPlane(pointToPlane),
	_previousCloudNormal(new pcl::PointCloud<pcl::PointNormal>),
	_previousCloud(new pcl::PointCloud<pcl::PointXYZ>)
{
}

void OdometryICP::reset(const Transform & initialPose)
{
	Odometry::reset(initialPose);
	_previousCloudNormal.reset(new pcl::PointCloud<pcl::PointNormal>);
	_previousCloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
}

// return not null transform if odometry is correctly computed
Transform OdometryICP::computeTransform(const SensorData & data, OdometryInfo * info)
{
	UTimer timer;
	Transform output;

	bool hasConverged = false;
	double variance = 0;
	unsigned int minPoints = 100;
	if(!data.depthOrRightRaw().empty())
	{
		if(data.depthOrRightRaw().type() == CV_8UC1)
		{
			UERROR("ICP 3D cannot be done on stereo images!");
			return output;
		}

		if(!(data.cameraModels().size() == 1 && data.cameraModels()[0].isValid()))
		{
			UERROR("ICP 3D cannot be done without calibration or on multi-camera!");
			return output;
		}
		const CameraModel & cameraModel = data.cameraModels()[0];

		pcl::PointCloud<pcl::PointXYZ>::Ptr newCloudXYZ = util3d::getICPReadyCloud(
						data.depthOrRightRaw(),
						cameraModel.fx(),
						cameraModel.fy(),
						cameraModel.cx(),
						cameraModel.cy(),
						_decimation,
						this->getMaxDepth(),
						_voxelSize,
						_samples,
						cameraModel.localTransform());

		if(_pointToPlane)
		{
			pcl::PointCloud<pcl::PointNormal>::Ptr newCloud = util3d::computeNormals(newCloudXYZ);

			std::vector<int> indices;
			newCloud = util3d::removeNaNNormalsFromPointCloud(newCloud);
			if(newCloudXYZ->size() != newCloud->size())
			{
				UWARN("removed nan normals...");
			}

			if(_previousCloudNormal->size() > minPoints && newCloud->size() > minPoints)
			{
				pcl::PointCloud<pcl::PointNormal>::Ptr newCloudRegistered(new pcl::PointCloud<pcl::PointNormal>);
				Transform transform = util3d::icpPointToPlane(
						newCloud,
						_previousCloudNormal,
						_maxCorrespondenceDistance,
						_maxIterations,
						hasConverged,
						*newCloudRegistered);

				int correspondences = 0;
				util3d::computeVarianceAndCorrespondences(
						newCloudRegistered,
						_previousCloudNormal,
						_maxCorrespondenceDistance,
						variance,
						correspondences);

				// verify if there are enough correspondences
				float correspondencesRatio = float(correspondences)/float(_previousCloudNormal->size()>newCloud->size()?_previousCloudNormal->size():newCloud->size());

				if(!transform.isNull() && hasConverged &&
		   	   	   correspondencesRatio >= _correspondenceRatio)
				{
					output = transform;
					_previousCloudNormal = newCloud;
				}
				else
				{
					UWARN("Transform not valid (hasConverged=%s variance = %f)",
							hasConverged?"true":"false", variance);
				}
			}
			else if(newCloud->size() > minPoints)
			{
				output.setIdentity();
				_previousCloudNormal = newCloud;
			}
		}
		else
		{
			//point to point
			if(_previousCloud->size() > minPoints && newCloudXYZ->size() > minPoints)
			{
				pcl::PointCloud<pcl::PointXYZ>::Ptr newCloudRegistered(new pcl::PointCloud<pcl::PointXYZ>);
				Transform transform = util3d::icp(
						newCloudXYZ,
						_previousCloud,
						_maxCorrespondenceDistance,
						_maxIterations,
						hasConverged,
						*newCloudRegistered);

				int correspondences = 0;
				util3d::computeVarianceAndCorrespondences(
						newCloudRegistered,
						_previousCloud,
						_maxCorrespondenceDistance,
						variance,
						correspondences);

				// verify if there are enough correspondences
				float correspondencesRatio = float(correspondences)/float(_previousCloud->size()>newCloudXYZ->size()?_previousCloud->size():newCloudXYZ->size());

				if(!transform.isNull() && hasConverged &&
				   correspondencesRatio >= _correspondenceRatio)
				{
					output = transform;
					_previousCloud = newCloudXYZ;
				}
				else
				{
					UWARN("Transform not valid (hasConverged=%s variance = %f)",
							hasConverged?"true":"false", variance);
				}
			}
			else if(newCloudXYZ->size() > minPoints)
			{
				output.setIdentity();
				_previousCloud = newCloudXYZ;
			}
		}
	}
	else
	{
		UERROR("Depth is empty?!?");
	}

	if(info)
	{
		info->variance = variance;
	}

	UINFO("Odom update time = %fs hasConverged=%s variance=%f cloud=%d",
			timer.elapsed(),
			hasConverged?"true":"false",
			variance,
			(int)(_pointToPlane?_previousCloudNormal->size():_previousCloud->size()));

	return output;
}

} // namespace rtabmap
