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


#include <rtabmap/core/RegistrationIcp.h>
#include <rtabmap/core/util3d_registration.h>
#include <rtabmap/core/util3d_surface.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/core/util3d_transforms.h>
#include <rtabmap/core/util3d_filtering.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UMath.h>
#include <pcl/io/pcd_io.h>

namespace rtabmap {

RegistrationIcp::RegistrationIcp(const ParametersMap & parameters) :
	_icpMaxTranslation(Parameters::defaultIcpMaxTranslation()),
	_icpMaxRotation(Parameters::defaultIcpMaxRotation()),
	_icp2D(Parameters::defaultIcp2D()),
	_icpVoxelSize(Parameters::defaultIcpVoxelSize()),
	_icpDownsamplingStep(Parameters::defaultIcpDownsamplingStep()),
	_icpMaxCorrespondenceDistance(Parameters::defaultIcpMaxCorrespondenceDistance()),
	_icpMaxIterations(Parameters::defaultIcpIterations()),
	_icpCorrespondenceRatio(Parameters::defaultIcpCorrespondenceRatio()),
	_icpPointToPlane(Parameters::defaultIcpPointToPlane()),
	_icpPointToPlaneNormalNeighbors(Parameters::defaultIcpPointToPlaneNormalNeighbors())
{
	this->parseParameters(parameters);
}

void RegistrationIcp::parseParameters(const ParametersMap & parameters)
{
	Registration::parseParameters(parameters);

	Parameters::parse(parameters, Parameters::kIcpMaxTranslation(), _icpMaxTranslation);
	Parameters::parse(parameters, Parameters::kIcpMaxRotation(), _icpMaxRotation);
	Parameters::parse(parameters, Parameters::kIcp2D(), _icp2D);
	Parameters::parse(parameters, Parameters::kIcpVoxelSize(), _icpVoxelSize);
	Parameters::parse(parameters, Parameters::kIcpDownsamplingStep(), _icpDownsamplingStep);
	Parameters::parse(parameters, Parameters::kIcpMaxCorrespondenceDistance(), _icpMaxCorrespondenceDistance);
	Parameters::parse(parameters, Parameters::kIcpIterations(), _icpMaxIterations);
	Parameters::parse(parameters, Parameters::kIcpCorrespondenceRatio(), _icpCorrespondenceRatio);
	Parameters::parse(parameters, Parameters::kIcpPointToPlane(), _icpPointToPlane);
	Parameters::parse(parameters, Parameters::kIcpPointToPlaneNormalNeighbors(), _icpPointToPlaneNormalNeighbors);

	UASSERT_MSG(_icpVoxelSize >= 0, uFormat("value=%d", _icpVoxelSize).c_str());
	UASSERT_MSG(_icpDownsamplingStep >= 0, uFormat("value=%d", _icpDownsamplingStep).c_str());
	UASSERT_MSG(_icpMaxCorrespondenceDistance > 0.0f, uFormat("value=%f", _icpMaxCorrespondenceDistance).c_str());
	UASSERT_MSG(_icpMaxIterations > 0, uFormat("value=%d", _icpMaxIterations).c_str());
	UASSERT_MSG(_icpCorrespondenceRatio >=0.0f && _icpCorrespondenceRatio <=1.0f, uFormat("value=%f", _icpCorrespondenceRatio).c_str());
	UASSERT_MSG(_icpPointToPlaneNormalNeighbors > 0, uFormat("value=%d", _icpPointToPlaneNormalNeighbors).c_str());
}

Transform RegistrationIcp::computeTransformation(
			const Signature & fromSignature,
			const Signature & toSignature,
			Transform guess,
			std::string * rejectedMsg,
			int * inliersOut,
			float * varianceOut,
			float * inliersRatioOut)
{
	return computeTransformation(
			fromSignature.sensorData(),
			toSignature.sensorData(),
			guess,
			rejectedMsg,
			inliersOut,
			varianceOut,
			inliersRatioOut);
}

Transform RegistrationIcp::computeTransformation(
			const SensorData & dataFrom,
			const SensorData & dataTo,
			Transform guess,
			std::string * rejectedMsg,
			int * inliersOut,
			float * varianceOut,
			float * inliersRatioOut)
{
	UDEBUG("Guess transform = %s", guess.prettyPrint().c_str());
	UDEBUG("Voxel size=%f", _icpVoxelSize);
	UDEBUG("2D=%d", _icp2D?1:0);
	UDEBUG("PointToPlane=%d", _icpPointToPlane?1:0);
	UDEBUG("Normal neighborhood=%d", _icpPointToPlaneNormalNeighbors);
	UDEBUG("Max corrrespondence distance=%f", _icpMaxCorrespondenceDistance);
	UDEBUG("Max Iterations=%d", _icpMaxIterations);
	UDEBUG("Variance from inliers count=%d", _bowVarianceFromInliersCount?1:0);
	UDEBUG("Correspondence Ratio=%f", _icpCorrespondenceRatio);
	UDEBUG("Max translation=%f", _icpMaxTranslation);
	UDEBUG("Max rotation=%f", _icpMaxRotation);
	UDEBUG("Downsampling step=%d", _icpDownsamplingStep);

	std::string msg;
	Transform transform;

	// ICP with guess transform
	if(!dataFrom.laserScanRaw().empty() && !dataTo.laserScanRaw().empty())
	{
		int maxLaserScans = dataTo.laserScanMaxPts();
		cv::Mat fromScan = dataFrom.laserScanRaw();
		cv::Mat toScan = dataTo.laserScanRaw();
		if(_icpDownsamplingStep>1)
		{
			fromScan = util3d::downsample(fromScan, _icpDownsamplingStep);
			toScan = util3d::downsample(toScan, _icpDownsamplingStep);
			maxLaserScans/=_icpDownsamplingStep;
		}

		pcl::PointCloud<pcl::PointXYZ>::Ptr fromCloud = util3d::laserScanToPointCloud(fromScan, Transform());
		pcl::PointCloud<pcl::PointXYZ>::Ptr toCloud = util3d::laserScanToPointCloud(toScan, guess);

		if(toCloud->size() && fromCloud->size())
		{
			//filtering
			pcl::PointCloud<pcl::PointXYZ>::Ptr fromCloudFiltered = fromCloud;
			pcl::PointCloud<pcl::PointXYZ>::Ptr toCloudFiltered = toCloud;
			bool filtered = false;
			if(_icpVoxelSize > 0.0f)
			{
				fromCloudFiltered = util3d::voxelize(fromCloudFiltered, _icpVoxelSize);
				toCloudFiltered = util3d::voxelize(toCloudFiltered, _icpVoxelSize);
				filtered = true;
			}

			Transform icpT;
			bool hasConverged = false;
			float correspondencesRatio = 0.0f;
			int correspondences = 0;
			double variance = 1.0;
			bool correspondencesComputed = false;
			pcl::PointCloud<pcl::PointXYZ>::Ptr newCloudRegistered(new pcl::PointCloud<pcl::PointXYZ>());
			if(!_icp2D) // 3D ICP
			{
				if(_icpPointToPlane)
				{
					pcl::PointCloud<pcl::PointNormal>::Ptr fromCloudNormals = util3d::computeNormals(fromCloudFiltered, _icpPointToPlaneNormalNeighbors);
					pcl::PointCloud<pcl::PointNormal>::Ptr toCloudNormals = util3d::computeNormals(toCloudFiltered, _icpPointToPlaneNormalNeighbors);

					std::vector<int> indices;
					toCloudNormals = util3d::removeNaNNormalsFromPointCloud(toCloudNormals);
					fromCloudNormals = util3d::removeNaNNormalsFromPointCloud(fromCloudNormals);

					if(toCloudNormals->size() && fromCloudNormals->size())
					{
						pcl::PointCloud<pcl::PointNormal>::Ptr newCloudNormalsRegistered(new pcl::PointCloud<pcl::PointNormal>());
						icpT = util3d::icpPointToPlane(
								toCloudNormals,
								fromCloudNormals,
							   _icpMaxCorrespondenceDistance,
							   _icpMaxIterations,
							   hasConverged,
							   *newCloudNormalsRegistered);
						if(!filtered &&
							!icpT.isNull() &&
							hasConverged)
						{
							util3d::computeVarianceAndCorrespondences(
								newCloudNormalsRegistered,
								fromCloudNormals,
								_icpMaxCorrespondenceDistance,
								variance,
								correspondences);
							correspondencesComputed = true;
						}
					}
				}
				else
				{
					icpT = util3d::icp(
							toCloudFiltered,
							fromCloudFiltered,
							_icpMaxCorrespondenceDistance,
							_icpMaxIterations,
							hasConverged,
							*newCloudRegistered);
				}
			}
			else // 2D ICP
			{
				icpT = util3d::icp2D(
							toCloudFiltered,
							fromCloudFiltered,
						   _icpMaxCorrespondenceDistance,
						   _icpMaxIterations,
						   hasConverged,
						   *newCloudRegistered);
			}

			/*pcl::io::savePCDFile("fromCloud.pcd", *fromCloud);
			pcl::io::savePCDFile("toCloud.pcd", *toCloud);
			UWARN("saved fromCloud.pcd and toCloud.pcd");
			if(!icpT.isNull())
			{
				pcl::PointCloud<pcl::PointXYZ>::Ptr toCloudTmp = util3d::transformPointCloud(toCloud, icpT);
				pcl::io::savePCDFile("newCloudFinal.pcd", *toCloudTmp);
				UWARN("saved toCloudFinal.pcd");
			}*/

			if(!icpT.isNull() &&
				hasConverged)
			{
				float ix,iy,iz, iroll,ipitch,iyaw;
				icpT.getTranslationAndEulerAngles(ix,iy,iz,iroll,ipitch,iyaw);
				if((_icpMaxTranslation>0.0f &&
					(fabs(ix) > _icpMaxTranslation ||
					 fabs(iy) > _icpMaxTranslation ||
					 fabs(iz) > _icpMaxTranslation))
					||
					(_icpMaxRotation>0.0f &&
					 (fabs(iroll) > _icpMaxRotation ||
					  fabs(ipitch) > _icpMaxRotation ||
					  fabs(iyaw) > _icpMaxRotation)))
				{
					msg = uFormat("Cannot compute transform (ICP correction too large -> %f m %f rad, limits=%f m, %f rad)",
							uMax3(fabs(ix), fabs(iy), fabs(iz)),
							uMax3(fabs(iroll), fabs(ipitch), fabs(iyaw)),
							_icpMaxTranslation,
							_icpMaxRotation);
					UINFO(msg.c_str());
				}
				else
				{
					if(!correspondencesComputed)
					{
						if(filtered)
						{
							fromCloud = util3d::transformPointCloud(fromCloud, icpT);
						}
						else
						{
							fromCloud = newCloudRegistered;
						}

						util3d::computeVarianceAndCorrespondences(
								toCloud,
								fromCloud,
								_icpMaxCorrespondenceDistance,
								variance,
								correspondences);
					}

					// verify if there are enough correspondences
					if(maxLaserScans)
					{
						correspondencesRatio = float(correspondences)/float(maxLaserScans);
					}
					else
					{
						UWARN("Maximum laser scans points not set for signature %d, correspondences ratio set relative instead of absolute!",
								dataTo.id());
						correspondencesRatio = float(correspondences)/float(toCloud->size()>fromCloud->size()?toCloud->size():fromCloud->size());
					}

					UDEBUG("%d->%d hasConverged=%s, variance=%f, correspondences=%d/%d (%f%%)",
							dataTo.id(), dataFrom.id(),
							hasConverged?"true":"false",
							variance,
							correspondences,
							maxLaserScans>0?maxLaserScans:dataTo.laserScanMaxPts()?dataTo.laserScanMaxPts():(int)(toCloud->size()>fromCloud->size()?toCloud->size():fromCloud->size()),
							correspondencesRatio*100.0f);

					if(_bowVarianceFromInliersCount)
					{
						variance = correspondencesRatio > 0?1.0/double(correspondencesRatio):1.0;
					}

					if(varianceOut)
					{
						*varianceOut = variance>0.0f?variance:0.0001; // epsilon if exact transform
					}
					if(inliersOut)
					{
						*inliersOut = correspondences;
					}
					if(inliersRatioOut)
					{
						*inliersRatioOut = correspondencesRatio;
					}

					if(correspondencesRatio < _icpCorrespondenceRatio)
					{
						msg = uFormat("Cannot compute transform (cor=%d corrRatio=%f/%f)",
								correspondences, correspondencesRatio, _icpCorrespondenceRatio);
						UINFO(msg.c_str());
					}
					else
					{
						transform = guess*icpT;
					}
				}
			}
			else
			{
				msg = uFormat("Cannot compute transform (converged=%s var=%f)",
						hasConverged?"true":"false", variance);
				UINFO(msg.c_str());
			}

			// still compute the variance for information
			/*if(variance == 1 && varianceOut)
			{
				util3d::computeVarianceAndCorrespondences(
					toCloudFiltered,
					fromCloudFiltered,
					_icpMaxCorrespondenceDistance,
					variance,
					correspondences);
				if(variance > 0)
				{
					*varianceOut = variance;
				}
			}*/
		}
		else
		{
			msg = "Laser scans empty ?!?";
			UWARN(msg.c_str());
		}
	}
	else
	{
		msg = uFormat("Laser scans empty?!? (new[%d]=%d old[%d]=%d)",
				dataTo.id(), dataTo.laserScanRaw().total(),
				dataFrom.id(), dataFrom.laserScanRaw().total());
		UERROR(msg.c_str());
	}


	if(rejectedMsg)
	{
		*rejectedMsg = msg;
	}

	UDEBUG("New transform = %s", transform.prettyPrint().c_str());
	return transform;
}

}
