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
	_maxTranslation(Parameters::defaultIcpMaxTranslation()),
	_maxRotation(Parameters::defaultIcpMaxRotation()),
	_icp2D(Parameters::defaultIcp2D()),
	_voxelSize(Parameters::defaultIcpVoxelSize()),
	_downsamplingStep(Parameters::defaultIcpDownsamplingStep()),
	_maxCorrespondenceDistance(Parameters::defaultIcpMaxCorrespondenceDistance()),
	_maxIterations(Parameters::defaultIcpIterations()),
	_correspondenceRatio(Parameters::defaultIcpCorrespondenceRatio()),
	_pointToPlane(Parameters::defaultIcpPointToPlane()),
	_pointToPlaneNormalNeighbors(Parameters::defaultIcpPointToPlaneNormalNeighbors())
{
	this->parseParameters(parameters);
}

void RegistrationIcp::parseParameters(const ParametersMap & parameters)
{
	Registration::parseParameters(parameters);

	Parameters::parse(parameters, Parameters::kIcpMaxTranslation(), _maxTranslation);
	Parameters::parse(parameters, Parameters::kIcpMaxRotation(), _maxRotation);
	Parameters::parse(parameters, Parameters::kIcp2D(), _icp2D);
	Parameters::parse(parameters, Parameters::kIcpVoxelSize(), _voxelSize);
	Parameters::parse(parameters, Parameters::kIcpDownsamplingStep(), _downsamplingStep);
	Parameters::parse(parameters, Parameters::kIcpMaxCorrespondenceDistance(), _maxCorrespondenceDistance);
	Parameters::parse(parameters, Parameters::kIcpIterations(), _maxIterations);
	Parameters::parse(parameters, Parameters::kIcpCorrespondenceRatio(), _correspondenceRatio);
	Parameters::parse(parameters, Parameters::kIcpPointToPlane(), _pointToPlane);
	Parameters::parse(parameters, Parameters::kIcpPointToPlaneNormalNeighbors(), _pointToPlaneNormalNeighbors);

	UASSERT_MSG(_voxelSize >= 0, uFormat("value=%d", _voxelSize).c_str());
	UASSERT_MSG(_downsamplingStep >= 0, uFormat("value=%d", _downsamplingStep).c_str());
	UASSERT_MSG(_maxCorrespondenceDistance > 0.0f, uFormat("value=%f", _maxCorrespondenceDistance).c_str());
	UASSERT_MSG(_maxIterations > 0, uFormat("value=%d", _maxIterations).c_str());
	UASSERT_MSG(_correspondenceRatio >=0.0f && _correspondenceRatio <=1.0f, uFormat("value=%f", _correspondenceRatio).c_str());
	UASSERT_MSG(_pointToPlaneNormalNeighbors > 0, uFormat("value=%d", _pointToPlaneNormalNeighbors).c_str());
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
	UDEBUG("Voxel size=%f", _voxelSize);
	UDEBUG("2D=%d", _icp2D?1:0);
	UDEBUG("PointToPlane=%d", _pointToPlane?1:0);
	UDEBUG("Normal neighborhood=%d", _pointToPlaneNormalNeighbors);
	UDEBUG("Max corrrespondence distance=%f", _maxCorrespondenceDistance);
	UDEBUG("Max Iterations=%d", _maxIterations);
	UDEBUG("Variance from inliers count=%d", _varianceFromInliersCount?1:0);
	UDEBUG("Correspondence Ratio=%f", _correspondenceRatio);
	UDEBUG("Max translation=%f", _maxTranslation);
	UDEBUG("Max rotation=%f", _maxRotation);
	UDEBUG("Downsampling step=%d", _downsamplingStep);

	std::string msg;
	Transform transform;

	// ICP with guess transform
	if(!dataFrom.laserScanRaw().empty() && !dataTo.laserScanRaw().empty())
	{
		int maxLaserScans = dataTo.laserScanMaxPts();
		cv::Mat fromScan = dataFrom.laserScanRaw();
		cv::Mat toScan = dataTo.laserScanRaw();
		if(_downsamplingStep>1)
		{
			fromScan = util3d::downsample(fromScan, _downsamplingStep);
			toScan = util3d::downsample(toScan, _downsamplingStep);
			maxLaserScans/=_downsamplingStep;
		}

		pcl::PointCloud<pcl::PointXYZ>::Ptr fromCloud = util3d::laserScanToPointCloud(fromScan, Transform());
		pcl::PointCloud<pcl::PointXYZ>::Ptr toCloud = util3d::laserScanToPointCloud(toScan, guess);

		if(toCloud->size() && fromCloud->size())
		{
			//filtering
			pcl::PointCloud<pcl::PointXYZ>::Ptr fromCloudFiltered = fromCloud;
			pcl::PointCloud<pcl::PointXYZ>::Ptr toCloudFiltered = toCloud;
			bool filtered = false;
			if(_voxelSize > 0.0f)
			{
				fromCloudFiltered = util3d::voxelize(fromCloudFiltered, _voxelSize);
				toCloudFiltered = util3d::voxelize(toCloudFiltered, _voxelSize);
				filtered = true;
			}

			Transform icpT;
			bool hasConverged = false;
			float correspondencesRatio = 0.0f;
			int correspondences = 0;
			double variance = 1.0;
			bool correspondencesComputed = false;
			pcl::PointCloud<pcl::PointXYZ>::Ptr fromCloudRegistered(new pcl::PointCloud<pcl::PointXYZ>());
			if(!_icp2D) // 3D ICP
			{
				if(_pointToPlane)
				{
					pcl::PointCloud<pcl::PointNormal>::Ptr fromCloudNormals = util3d::computeNormals(fromCloudFiltered, _pointToPlaneNormalNeighbors);
					pcl::PointCloud<pcl::PointNormal>::Ptr toCloudNormals = util3d::computeNormals(toCloudFiltered, _pointToPlaneNormalNeighbors);

					std::vector<int> indices;
					toCloudNormals = util3d::removeNaNNormalsFromPointCloud(toCloudNormals);
					fromCloudNormals = util3d::removeNaNNormalsFromPointCloud(fromCloudNormals);

					if(toCloudNormals->size() && fromCloudNormals->size())
					{
						pcl::PointCloud<pcl::PointNormal>::Ptr fromCloudNormalsRegistered(new pcl::PointCloud<pcl::PointNormal>());
						icpT = util3d::icpPointToPlane(
								fromCloudNormals,
								toCloudNormals,
							   _maxCorrespondenceDistance,
							   _maxIterations,
							   hasConverged,
							   *fromCloudNormalsRegistered);
						if(!filtered &&
							!icpT.isNull() &&
							hasConverged)
						{
							util3d::computeVarianceAndCorrespondences(
									fromCloudNormals,
									fromCloudNormalsRegistered,
									_maxCorrespondenceDistance,
									variance,
									correspondences);
							correspondencesComputed = true;
						}
					}
				}
				else
				{
					icpT = util3d::icp(
							fromCloudFiltered,
							toCloudFiltered,
							_maxCorrespondenceDistance,
							_maxIterations,
							hasConverged,
							*fromCloudRegistered);
				}
			}
			else // 2D ICP
			{
				icpT = util3d::icp2D(
						fromCloudFiltered,
						toCloudFiltered,
					   _maxCorrespondenceDistance,
					   _maxIterations,
					   hasConverged,
					   *fromCloudRegistered);
			}

			/*pcl::io::savePCDFile("fromCloud.pcd", *fromCloud);
			pcl::io::savePCDFile("toCloud.pcd", *toCloud);
			UWARN("saved fromCloud.pcd and toCloud.pcd");
			if(!icpT.isNull())
			{
				pcl::PointCloud<pcl::PointXYZ>::Ptr fromCloudTmp = util3d::transformPointCloud(fromCloud, icpT);
				pcl::io::savePCDFile("fromCloudFinal.pcd", *fromCloudTmp);
				UWARN("saved fromCloudFinal.pcd");
			}*/

			if(!icpT.isNull() &&
				hasConverged)
			{
				float ix,iy,iz, iroll,ipitch,iyaw;
				icpT.getTranslationAndEulerAngles(ix,iy,iz,iroll,ipitch,iyaw);
				if((_maxTranslation>0.0f &&
					(fabs(ix) > _maxTranslation ||
					 fabs(iy) > _maxTranslation ||
					 fabs(iz) > _maxTranslation))
					||
					(_maxRotation>0.0f &&
					 (fabs(iroll) > _maxRotation ||
					  fabs(ipitch) > _maxRotation ||
					  fabs(iyaw) > _maxRotation)))
				{
					msg = uFormat("Cannot compute transform (ICP correction too large -> %f m %f rad, limits=%f m, %f rad)",
							uMax3(fabs(ix), fabs(iy), fabs(iz)),
							uMax3(fabs(iroll), fabs(ipitch), fabs(iyaw)),
							_maxTranslation,
							_maxRotation);
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
							fromCloud = fromCloudRegistered;
						}

						util3d::computeVarianceAndCorrespondences(
								fromCloud,
								toCloud,
								_maxCorrespondenceDistance,
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

					if(_varianceFromInliersCount)
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

					if(correspondencesRatio < _correspondenceRatio)
					{
						msg = uFormat("Cannot compute transform (cor=%d corrRatio=%f/%f)",
								correspondences, correspondencesRatio, _correspondenceRatio);
						UINFO(msg.c_str());
					}
					else
					{
						transform = icpT.inverse()*guess;
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
