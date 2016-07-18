/*
Copyright (c) 2010-2016, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
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
#include <rtabmap/utilite/UTimer.h>
#include <pcl/io/pcd_io.h>

namespace rtabmap {

RegistrationIcp::RegistrationIcp(const ParametersMap & parameters, Registration * child) :
	Registration(parameters, child),
	_maxTranslation(Parameters::defaultIcpMaxTranslation()),
	_maxRotation(Parameters::defaultIcpMaxRotation()),
	_voxelSize(Parameters::defaultIcpVoxelSize()),
	_downsamplingStep(Parameters::defaultIcpDownsamplingStep()),
	_maxCorrespondenceDistance(Parameters::defaultIcpMaxCorrespondenceDistance()),
	_maxIterations(Parameters::defaultIcpIterations()),
	_epsilon(Parameters::defaultIcpEpsilon()),
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
	Parameters::parse(parameters, Parameters::kIcpVoxelSize(), _voxelSize);
	Parameters::parse(parameters, Parameters::kIcpDownsamplingStep(), _downsamplingStep);
	Parameters::parse(parameters, Parameters::kIcpMaxCorrespondenceDistance(), _maxCorrespondenceDistance);
	Parameters::parse(parameters, Parameters::kIcpIterations(), _maxIterations);
	Parameters::parse(parameters, Parameters::kIcpEpsilon(), _epsilon);
	Parameters::parse(parameters, Parameters::kIcpCorrespondenceRatio(), _correspondenceRatio);
	Parameters::parse(parameters, Parameters::kIcpPointToPlane(), _pointToPlane);
	Parameters::parse(parameters, Parameters::kIcpPointToPlaneNormalNeighbors(), _pointToPlaneNormalNeighbors);

	UASSERT_MSG(_voxelSize >= 0, uFormat("value=%d", _voxelSize).c_str());
	UASSERT_MSG(_downsamplingStep >= 0, uFormat("value=%d", _downsamplingStep).c_str());
	UASSERT_MSG(_maxCorrespondenceDistance > 0.0f, uFormat("value=%f", _maxCorrespondenceDistance).c_str());
	UASSERT_MSG(_maxIterations > 0, uFormat("value=%d", _maxIterations).c_str());
	UASSERT(_epsilon >= 0.0f);
	UASSERT_MSG(_correspondenceRatio >=0.0f && _correspondenceRatio <=1.0f, uFormat("value=%f", _correspondenceRatio).c_str());
	UASSERT_MSG(_pointToPlaneNormalNeighbors > 0, uFormat("value=%d", _pointToPlaneNormalNeighbors).c_str());
}

Transform RegistrationIcp::computeTransformationImpl(
			Signature & fromSignature,
			Signature & toSignature,
			Transform guess,
			RegistrationInfo & info) const
{
	UDEBUG("Guess transform = %s", guess.prettyPrint().c_str());
	UDEBUG("Voxel size=%f", _voxelSize);
	UDEBUG("PointToPlane=%d", _pointToPlane?1:0);
	UDEBUG("Normal neighborhood=%d", _pointToPlaneNormalNeighbors);
	UDEBUG("Max corrrespondence distance=%f", _maxCorrespondenceDistance);
	UDEBUG("Max Iterations=%d", _maxIterations);
	UDEBUG("Correspondence Ratio=%f", _correspondenceRatio);
	UDEBUG("Max translation=%f", _maxTranslation);
	UDEBUG("Max rotation=%f", _maxRotation);
	UDEBUG("Downsampling step=%d", _downsamplingStep);

	UTimer timer;
	std::string msg;
	Transform transform;

	SensorData & dataFrom = fromSignature.sensorData();
	SensorData & dataTo = toSignature.sensorData();

	UDEBUG("size from=%d (channels=%d) to=%d (channels=%d)",
			dataFrom.laserScanRaw().cols,
			dataFrom.laserScanRaw().channels(),
			dataTo.laserScanRaw().cols,
			dataTo.laserScanRaw().channels());

	if(!guess.isNull() && !dataFrom.laserScanRaw().empty() && !dataTo.laserScanRaw().empty())
	{
		// ICP with guess transform
		int maxLaserScansTo = dataTo.laserScanMaxPts();
		int maxLaserScansFrom = dataFrom.laserScanMaxPts();
		cv::Mat fromScan = dataFrom.laserScanRaw();
		cv::Mat toScan = dataTo.laserScanRaw();
		if(_downsamplingStep>1)
		{
			fromScan = util3d::downsample(fromScan, _downsamplingStep);
			toScan = util3d::downsample(toScan, _downsamplingStep);
			maxLaserScansTo/=_downsamplingStep;
			maxLaserScansFrom/=_downsamplingStep;
			UDEBUG("Downsampling time (step=%d) = %f s", _downsamplingStep, timer.ticks());
		}

		if(fromScan.cols && toScan.cols)
		{
			Transform icpT;
			bool hasConverged = false;
			float correspondencesRatio = 0.0f;
			int correspondences = 0;
			double variance = 1.0;

			if( _pointToPlane &&
				_voxelSize == 0.0f &&
				fromScan.channels() == 6 &&
				toScan.channels() == 6)
			{
				//special case if we have already normals computed and there is no filtering
				pcl::PointCloud<pcl::PointNormal>::Ptr fromCloudNormals = util3d::laserScanToPointCloudNormal(fromScan, Transform());
				pcl::PointCloud<pcl::PointNormal>::Ptr toCloudNormals = util3d::laserScanToPointCloudNormal(toScan, guess);

				UDEBUG("Conversion time = %f s", timer.ticks());
				pcl::PointCloud<pcl::PointNormal>::Ptr fromCloudNormalsRegistered(new pcl::PointCloud<pcl::PointNormal>());
				icpT = util3d::icpPointToPlane(
						fromCloudNormals,
						toCloudNormals,
					   _maxCorrespondenceDistance,
					   _maxIterations,
					   hasConverged,
					   *fromCloudNormalsRegistered,
					   _epsilon,
					   this->force3DoF());
				if(!icpT.isNull() && hasConverged)
				{
					util3d::computeVarianceAndCorrespondences(
							fromCloudNormalsRegistered,
							toCloudNormals,
							_maxCorrespondenceDistance,
							variance,
							correspondences);
					/*
					UWARN("icpT=%s", icpT.prettyPrint().c_str());
					pcl::io::savePCDFile("fromCloud.pcd", *fromCloudNormals);
					pcl::io::savePCDFile("toCloud.pcd", *toCloudNormals);
					UWARN("saved fromCloud.pcd and toCloud.pcd");
					if(!icpT.isNull())
					{
						pcl::PointCloud<pcl::PointNormal>::Ptr fromCloudTmp = util3d::transformPointCloud(fromCloudNormals, icpT);
						pcl::io::savePCDFile("fromCloudFinal.pcd", *fromCloudTmp);
						pcl::io::savePCDFile("fromCloudFinal2.pcd", *fromCloudNormalsRegistered);
						UWARN("saved fromCloudFinal.pcd");
					}
					*/

				}
			}
			else
			{
				pcl::PointCloud<pcl::PointXYZ>::Ptr fromCloud = util3d::laserScanToPointCloud(fromScan, Transform());
				pcl::PointCloud<pcl::PointXYZ>::Ptr toCloud = util3d::laserScanToPointCloud(toScan, guess);
				UDEBUG("Conversion time = %f s", timer.ticks());

				pcl::PointCloud<pcl::PointXYZ>::Ptr fromCloudFiltered = fromCloud;
				pcl::PointCloud<pcl::PointXYZ>::Ptr toCloudFiltered = toCloud;
				bool filtered = false;
				if(_voxelSize > 0.0f)
				{
					int pointsBeforeFiltering = fromCloudFiltered->size();
					fromCloudFiltered = util3d::voxelize(fromCloudFiltered, _voxelSize);
					maxLaserScansFrom = maxLaserScansFrom * fromCloudFiltered->size() / pointsBeforeFiltering;

					pointsBeforeFiltering = toCloudFiltered->size();
					toCloudFiltered = util3d::voxelize(toCloudFiltered, _voxelSize);
					maxLaserScansTo = maxLaserScansTo * toCloudFiltered->size() / pointsBeforeFiltering;

					filtered = true;
					UDEBUG("Voxel filtering time (voxel=%f m) = %f s", _voxelSize, timer.ticks());

					//Adjust maxLaserScans

				}

				bool correspondencesComputed = false;
				pcl::PointCloud<pcl::PointXYZ>::Ptr fromCloudRegistered(new pcl::PointCloud<pcl::PointXYZ>());
				if(_pointToPlane) // ICP Point To Plane, only in 3D
				{
					pcl::PointCloud<pcl::Normal>::Ptr normals;

					normals = util3d::computeNormals(fromCloudFiltered, _pointToPlaneNormalNeighbors);
					pcl::PointCloud<pcl::PointNormal>::Ptr fromCloudNormals(new pcl::PointCloud<pcl::PointNormal>);
					pcl::concatenateFields(*fromCloudFiltered, *normals, *fromCloudNormals);

					normals = util3d::computeNormals(toCloudFiltered, _pointToPlaneNormalNeighbors);
					pcl::PointCloud<pcl::PointNormal>::Ptr toCloudNormals(new pcl::PointCloud<pcl::PointNormal>);
					pcl::concatenateFields(*toCloudFiltered, *normals, *toCloudNormals);

					std::vector<int> indices;
					toCloudNormals = util3d::removeNaNNormalsFromPointCloud(toCloudNormals);
					fromCloudNormals = util3d::removeNaNNormalsFromPointCloud(fromCloudNormals);

					// update output scans
					fromSignature.sensorData().setLaserScanRaw(util3d::laserScanFromPointCloud(*fromCloudNormals), maxLaserScansFrom, fromSignature.sensorData().laserScanMaxRange());
					toSignature.sensorData().setLaserScanRaw(util3d::laserScanFromPointCloud(*toCloudNormals, guess.inverse()), maxLaserScansTo, toSignature.sensorData().laserScanMaxRange());

					UDEBUG("Compute normals time = %f s", timer.ticks());

					if(toCloudNormals->size() && fromCloudNormals->size())
					{
						pcl::PointCloud<pcl::PointNormal>::Ptr fromCloudNormalsRegistered(new pcl::PointCloud<pcl::PointNormal>());
						icpT = util3d::icpPointToPlane(
								fromCloudNormals,
								toCloudNormals,
							   _maxCorrespondenceDistance,
							   _maxIterations,
							   hasConverged,
							   *fromCloudNormalsRegistered,
							   _epsilon,
							   this->force3DoF());
						if(!filtered &&
							!icpT.isNull() &&
							hasConverged)
						{
							util3d::computeVarianceAndCorrespondences(
									fromCloudNormalsRegistered,
									toCloudNormals,
									_maxCorrespondenceDistance,
									variance,
									correspondences);
							correspondencesComputed = true;
						}
					}
				}
				else // ICP Point to Point
				{
					if(_voxelSize > 0.0f)
					{
						// update output scans
						fromSignature.sensorData().setLaserScanRaw(util3d::laserScanFromPointCloud(*fromCloudFiltered), maxLaserScansFrom, fromSignature.sensorData().laserScanMaxRange());
						toSignature.sensorData().setLaserScanRaw(util3d::laserScanFromPointCloud(*toCloudFiltered, guess.inverse()), maxLaserScansTo, toSignature.sensorData().laserScanMaxRange());
					}

					icpT = util3d::icp(
							fromCloudFiltered,
							toCloudFiltered,
						   _maxCorrespondenceDistance,
						   _maxIterations,
						   hasConverged,
						   *fromCloudRegistered,
						   _epsilon,
						   this->force3DoF()); // icp2D
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
					hasConverged &&
					!correspondencesComputed)
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
			}
			UDEBUG("ICP (iterations=%d) time = %f s", _maxIterations, timer.ticks());

			if(!icpT.isNull() &&
				hasConverged)
			{
				float ix,iy,iz, iroll,ipitch,iyaw;
				Transform icpInTargetReferential = guess.inverse() * icpT.inverse() * guess; // actual local ICP refinement
				icpInTargetReferential.getTranslationAndEulerAngles(ix,iy,iz,iroll,ipitch,iyaw);
				info.icpTranslation = uMax3(fabs(ix), fabs(iy), fabs(iz));
				info.icpRotation = uMax3(fabs(iroll), fabs(ipitch), fabs(iyaw));
				if((_maxTranslation>0.0f &&
						info.icpTranslation > _maxTranslation)
				   ||
				   (_maxRotation>0.0f &&
						info.icpRotation > _maxRotation))
				{
					msg = uFormat("Cannot compute transform (ICP correction too large -> %f m %f rad, limits=%f m, %f rad)",
							info.icpTranslation,
							info.icpRotation,
							_maxTranslation,
							_maxRotation);
					UINFO(msg.c_str());
				}
				else
				{
					// verify if there are enough correspondences
					int maxLaserScans = maxLaserScansTo?maxLaserScansTo:maxLaserScansFrom;
					if(maxLaserScans)
					{
						correspondencesRatio = float(correspondences)/float(maxLaserScans);
					}
					else
					{
						static bool warningShown = false;
						if(!warningShown)
						{
							UWARN("Maximum laser scans points not set for signature %d, correspondences ratio set relative instead of absolute! This message will only appear once.",
									dataTo.id());
							warningShown = true;
						}
						correspondencesRatio = float(correspondences)/float(toScan.cols>fromScan.cols?toScan.cols:fromScan.cols);
					}

					UDEBUG("%d->%d hasConverged=%s, variance=%f, correspondences=%d/%d (%f%%)",
							dataTo.id(), dataFrom.id(),
							hasConverged?"true":"false",
							variance,
							correspondences,
							maxLaserScans>0?maxLaserScans:(int)(toScan.cols>fromScan.cols?toScan.cols:fromScan.cols),
							correspondencesRatio*100.0f);

					info.variance = variance>0.0f?variance:0.0001; // epsilon if exact transform
					info.icpInliersRatio = correspondencesRatio;

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
		}
		else
		{
			msg = "Laser scans empty ?!?";
			UWARN(msg.c_str());
		}
	}
	else if(dataTo.isValid())
	{
		if(guess.isNull())
		{
			msg = "RegistrationIcp cannot do registration with a null guess.";
		}
		else
		{
			msg = uFormat("Laser scans empty?!? (new[%d]=%d old[%d]=%d)",
					dataTo.id(), dataTo.laserScanRaw().total(),
					dataFrom.id(), dataFrom.laserScanRaw().total());
		}
		UERROR(msg.c_str());
	}


	info.rejectedMsg = msg;

	UDEBUG("New transform = %s", transform.prettyPrint().c_str());
	return transform;
}

}
