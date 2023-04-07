/*
Copyright (c) 2010-2021, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
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
#include <rtabmap/utilite/UDirectory.h>
#include <pcl/conversions.h>
#include <pcl/common/pca.h>
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h>

#ifdef RTABMAP_CCCORELIB
#include "icp/cccorelib.h"
#endif

#ifdef RTABMAP_POINTMATCHER
#include "icp/libpointmatcher.h"
#endif

namespace rtabmap {

RegistrationIcp::RegistrationIcp(const ParametersMap & parameters, Registration * child) :
	Registration(parameters, child),
	_strategy(Parameters::defaultIcpStrategy()),
	_maxTranslation(Parameters::defaultIcpMaxTranslation()),
	_maxRotation(Parameters::defaultIcpMaxRotation()),
	_voxelSize(Parameters::defaultIcpVoxelSize()),
	_downsamplingStep(Parameters::defaultIcpDownsamplingStep()),
	_rangeMin(Parameters::defaultIcpRangeMin()),
	_rangeMax(Parameters::defaultIcpRangeMax()),
	_maxCorrespondenceDistance(Parameters::defaultIcpMaxCorrespondenceDistance()),
	_reciprocalCorrespondences(Parameters::defaultIcpReciprocalCorrespondences()),
	_maxIterations(Parameters::defaultIcpIterations()),
	_epsilon(Parameters::defaultIcpEpsilon()),
	_correspondenceRatio(Parameters::defaultIcpCorrespondenceRatio()),
	_force4DoF(Parameters::defaultIcpForce4DoF()),
	_pointToPlane(Parameters::defaultIcpPointToPlane()),
	_pointToPlaneK(Parameters::defaultIcpPointToPlaneK()),
	_pointToPlaneRadius(Parameters::defaultIcpPointToPlaneRadius()),
	_pointToPlaneGroundNormalsUp(Parameters::defaultIcpPointToPlaneGroundNormalsUp()),
	_pointToPlaneMinComplexity(Parameters::defaultIcpPointToPlaneMinComplexity()),
	_pointToPlaneLowComplexityStrategy(Parameters::defaultIcpPointToPlaneLowComplexityStrategy()),
	_libpointmatcherConfig(Parameters::defaultIcpPMConfig()),
	_libpointmatcherKnn(Parameters::defaultIcpPMMatcherKnn()),
	_libpointmatcherEpsilon(Parameters::defaultIcpPMMatcherEpsilon()),
	_libpointmatcherIntensity(Parameters::defaultIcpPMMatcherIntensity()),
	_outlierRatio(Parameters::defaultIcpOutlierRatio()),
	_ccSamplingLimit (Parameters::defaultIcpCCSamplingLimit()),
	_ccFilterOutFarthestPoints (Parameters::defaultIcpCCFilterOutFarthestPoints()),
	_ccMaxFinalRMS (Parameters::defaultIcpCCMaxFinalRMS()),
	_debugExportFormat(Parameters::defaultIcpDebugExportFormat()),
	_libpointmatcherICP(0),
	_libpointmatcherICPFilters(0)
{
	this->parseParameters(parameters);
}

RegistrationIcp::~RegistrationIcp()
{
#ifdef RTABMAP_POINTMATCHER
	delete (PM::ICP*)_libpointmatcherICP;
	delete (PM::ICP*)_libpointmatcherICPFilters;
#endif
}

void RegistrationIcp::parseParameters(const ParametersMap & parameters)
{
	Registration::parseParameters(parameters);

	Parameters::parse(parameters, Parameters::kIcpStrategy(), _strategy);
	Parameters::parse(parameters, Parameters::kIcpMaxTranslation(), _maxTranslation);
	Parameters::parse(parameters, Parameters::kIcpMaxRotation(), _maxRotation);
	Parameters::parse(parameters, Parameters::kIcpVoxelSize(), _voxelSize);
	Parameters::parse(parameters, Parameters::kIcpDownsamplingStep(), _downsamplingStep);
	Parameters::parse(parameters, Parameters::kIcpRangeMin(), _rangeMin);
	Parameters::parse(parameters, Parameters::kIcpRangeMax(), _rangeMax);
	Parameters::parse(parameters, Parameters::kIcpMaxCorrespondenceDistance(), _maxCorrespondenceDistance);
	Parameters::parse(parameters, Parameters::kIcpReciprocalCorrespondences(), _reciprocalCorrespondences);
	Parameters::parse(parameters, Parameters::kIcpIterations(), _maxIterations);
	Parameters::parse(parameters, Parameters::kIcpEpsilon(), _epsilon);
	Parameters::parse(parameters, Parameters::kIcpCorrespondenceRatio(), _correspondenceRatio);
	Parameters::parse(parameters, Parameters::kIcpForce4DoF(), _force4DoF);
	Parameters::parse(parameters, Parameters::kIcpOutlierRatio(), _outlierRatio);
	Parameters::parse(parameters, Parameters::kIcpPointToPlane(), _pointToPlane);
	Parameters::parse(parameters, Parameters::kIcpPointToPlaneK(), _pointToPlaneK);
	Parameters::parse(parameters, Parameters::kIcpPointToPlaneRadius(), _pointToPlaneRadius);
	Parameters::parse(parameters, Parameters::kIcpPointToPlaneGroundNormalsUp(), _pointToPlaneGroundNormalsUp);
	Parameters::parse(parameters, Parameters::kIcpPointToPlaneMinComplexity(), _pointToPlaneMinComplexity);
	Parameters::parse(parameters, Parameters::kIcpPointToPlaneLowComplexityStrategy(), _pointToPlaneLowComplexityStrategy);
	UASSERT(_pointToPlaneGroundNormalsUp >= 0.0f && _pointToPlaneGroundNormalsUp <= 1.0f);
	UASSERT(_pointToPlaneMinComplexity >= 0.0f && _pointToPlaneMinComplexity <= 1.0f);

	Parameters::parse(parameters, Parameters::kIcpPMConfig(), _libpointmatcherConfig);
	Parameters::parse(parameters, Parameters::kIcpPMMatcherKnn(), _libpointmatcherKnn);
	Parameters::parse(parameters, Parameters::kIcpPMMatcherEpsilon(), _libpointmatcherEpsilon);
	Parameters::parse(parameters, Parameters::kIcpPMMatcherIntensity(), _libpointmatcherIntensity);

	Parameters::parse(parameters, Parameters::kIcpCCSamplingLimit(), _ccSamplingLimit);
	Parameters::parse(parameters, Parameters::kIcpCCFilterOutFarthestPoints(), _ccFilterOutFarthestPoints);
	Parameters::parse(parameters, Parameters::kIcpCCMaxFinalRMS(), _ccMaxFinalRMS);

	Parameters::parse(parameters, Parameters::kIcpDebugExportFormat(), _debugExportFormat);
	ParametersMap::const_iterator iter;
	if((iter=parameters.find(Parameters::kRtabmapWorkingDirectory())) != parameters.end())
	{
		_workingDir = uReplaceChar(iter->second, '~', UDirectory::homeDir());
	}

	bool pointToPlane = _pointToPlane;

#ifndef RTABMAP_POINTMATCHER
	if(_strategy==1)
	{
		UWARN("Parameter %s is set to 1 but RTAB-Map has not been built with libpointmatcher support. Setting to 0.", Parameters::kIcpStrategy().c_str());
		_strategy = 0;
	}
#else
	delete (PM::ICP*)_libpointmatcherICP;
	delete (PM::ICP*)_libpointmatcherICPFilters;
	_libpointmatcherICP = 0;
	_libpointmatcherICPFilters = 0;
	if(_strategy==1)
	{
		_libpointmatcherConfig = uReplaceChar(_libpointmatcherConfig, '~', UDirectory::homeDir());
		UDEBUG("libpointmatcher enabled! config=\"%s\"", _libpointmatcherConfig.c_str());
		_libpointmatcherICP = new PM::ICP();
		PM::ICP * icp = (PM::ICP*)_libpointmatcherICP;

		bool useDefaults = true;
		if(!_libpointmatcherConfig.empty())
		{
			// load YAML config
			std::ifstream ifs(_libpointmatcherConfig.c_str());
			if (ifs.good())
			{
				try {
					icp->loadFromYaml(ifs);
					useDefaults = false;

					_libpointmatcherICPFilters = new PM::ICP();
					PM::ICP * icpFilters = (PM::ICP*)_libpointmatcherICPFilters;
					icpFilters->readingDataPointsFilters = icp->readingDataPointsFilters;
					icpFilters->referenceDataPointsFilters = icp->referenceDataPointsFilters;

					icp->readingDataPointsFilters.clear();
					icp->readingDataPointsFilters.push_back(PM::get().DataPointsFilterRegistrar.create("IdentityDataPointsFilter"));

					icp->referenceDataPointsFilters.clear();
					icp->referenceDataPointsFilters.push_back(PM::get().DataPointsFilterRegistrar.create("IdentityDataPointsFilter"));
				}
				catch (const std::exception & e)
				{
					UFATAL("Error reading libpointmatcher config file \"%s\": %s", _libpointmatcherConfig.c_str(), e.what());
				}
			}
			else
			{
				UERROR("Cannot open libpointmatcher config file \"%s\", using default values instead.", _libpointmatcherConfig.c_str());
			}
		}
		if(useDefaults)
		{
			// Create the default ICP algorithm
			// See the implementation of setDefault() to create a custom ICP algorithm
			icp->setDefault();

			icp->readingDataPointsFilters.clear();
			icp->readingDataPointsFilters.push_back(PM::get().DataPointsFilterRegistrar.create("IdentityDataPointsFilter"));

			icp->referenceDataPointsFilters.clear();
			icp->referenceDataPointsFilters.push_back(PM::get().DataPointsFilterRegistrar.create("IdentityDataPointsFilter"));

			PM::Parameters params;
			params["maxDist"] = uNumber2Str(_maxCorrespondenceDistance);
			params["knn"] = uNumber2Str(_libpointmatcherKnn);
			params["epsilon"] = uNumber2Str(_libpointmatcherEpsilon);

			if(_libpointmatcherIntensity)
			{
				icp->matcher.reset(new KDTreeMatcherIntensity<float>(params));
			}
			else
			{
#if POINTMATCHER_VERSION_INT >= 10300
				icp->matcher = PM::get().MatcherRegistrar.create("KDTreeMatcher", params);
#else
				icp->matcher.reset(PM::get().MatcherRegistrar.create("KDTreeMatcher", params));
#endif
			}
			params.clear();

			params["ratio"] = uNumber2Str(_outlierRatio);
			icp->outlierFilters.clear();
			icp->outlierFilters.push_back(PM::get().OutlierFilterRegistrar.create("TrimmedDistOutlierFilter", params));
			params.clear();
			if(_pointToPlane)
			{
				params["maxAngle"] = uNumber2Str(_maxRotation<=0.0f?M_PI:_maxRotation);
				icp->outlierFilters.push_back(PM::get().OutlierFilterRegistrar.create("SurfaceNormalOutlierFilter", params));
				params.clear();

				params["force2D"] = force3DoF()?"1":"0";

				if(!force3DoF()&&_force4DoF)
				{
					params["force4DOF"] = "1";
				}
#if POINTMATCHER_VERSION_INT >= 10300
				icp->errorMinimizer = PM::get().ErrorMinimizerRegistrar.create("PointToPlaneErrorMinimizer", params);
#else
				icp->errorMinimizer.reset(PM::get().ErrorMinimizerRegistrar.create("PointToPlaneErrorMinimizer", params));
#endif
				params.clear();
			}
			else
			{
#if POINTMATCHER_VERSION_INT >= 10300
				icp->errorMinimizer = PM::get().ErrorMinimizerRegistrar.create("PointToPointErrorMinimizer");
#else
				icp->errorMinimizer.reset(PM::get().ErrorMinimizerRegistrar.create("PointToPointErrorMinimizer"));
#endif
			}

			icp->transformationCheckers.clear();
			params["maxIterationCount"] = uNumber2Str(_maxIterations);
			icp->transformationCheckers.push_back(PM::get().TransformationCheckerRegistrar.create("CounterTransformationChecker", params));
			params.clear();

			params["minDiffRotErr"] =   uNumber2Str(_epsilon*_epsilon*100.0f);
			params["minDiffTransErr"] = uNumber2Str(_epsilon*_epsilon);
			params["smoothLength"] =    uNumber2Str(4);
			icp->transformationCheckers.push_back(PM::get().TransformationCheckerRegistrar.create("DifferentialTransformationChecker", params));
			params.clear();

			params["maxRotationNorm"] = uNumber2Str(_maxRotation<=0.0f?M_PI:_maxRotation);
			params["maxTranslationNorm"] =    uNumber2Str(_maxTranslation<=0.0f?std::numeric_limits<float>::max():_maxTranslation);
			icp->transformationCheckers.push_back(PM::get().TransformationCheckerRegistrar.create("BoundTransformationChecker", params));
			params.clear();
		}
		pointToPlane = icp->errorMinimizer->className.compare("PointToPlaneErrorMinimizer")==0;
	}
#endif

#ifndef RTABMAP_CCCORELIB
	if(_strategy==2)
	{
#ifdef RTABMAP_POINTMATCHER
		_strategy = 1;
#else
		_strategy = 0;
#endif
		UWARN("Parameter %s is set to 2 but RTAB-Map has not been built with CCCoreLib support. Setting to %d.", Parameters::kIcpStrategy().c_str(), _strategy);
	}
#else
	if(_strategy==2 && _pointToPlane)
	{
		UWARN("%s cannot be used with %s=2 (CCCoreLib), setting %s to false", Parameters::kIcpPointToPlane().c_str(), Parameters::kIcpStrategy().c_str(), Parameters::kIcpPointToPlane().c_str());
		_pointToPlane = false;
	}
#endif

	if(_force4DoF && _strategy == 0)
	{
		UWARN("%s cannot be used with %s == 0.", Parameters::kIcpForce4DoF().c_str(), Parameters::kIcpStrategy().c_str());
		_force4DoF = false;
	}

	UASSERT_MSG(_voxelSize >= 0, uFormat("value=%d", _voxelSize).c_str());
	UASSERT_MSG(_downsamplingStep >= 0, uFormat("value=%d", _downsamplingStep).c_str());
	UASSERT_MSG(_maxCorrespondenceDistance > 0.0f, uFormat("value=%f", _maxCorrespondenceDistance).c_str());
	UASSERT_MSG(_maxIterations > 0, uFormat("value=%d", _maxIterations).c_str());
	UASSERT(_epsilon >= 0.0f);
	UASSERT_MSG(_correspondenceRatio >=0.0f && _correspondenceRatio <=1.0f, uFormat("value=%f", _correspondenceRatio).c_str());
	UASSERT_MSG(!_pointToPlane || (_pointToPlane && (_pointToPlaneK > 0 || _pointToPlaneRadius > 0.0f || pointToPlane)), uFormat("_pointToPlaneK=%d _pointToPlaneRadius=%f", _pointToPlaneK, _pointToPlaneRadius).c_str());
}

Transform RegistrationIcp::computeTransformationImpl(
			Signature & fromSignature,
			Signature & toSignature,
			Transform guess,
			RegistrationInfo & info) const
{
	bool pointToPlane = _pointToPlane;
#ifdef RTABMAP_POINTMATCHER
	if(_strategy==1)
	{
		PM::ICP * icp = (PM::ICP*)_libpointmatcherICP;
		pointToPlane = icp->errorMinimizer->className.compare("PointToPlaneErrorMinimizer")==0;
	}
#endif

	UDEBUG("Guess transform = %s", guess.prettyPrint().c_str());
	UDEBUG("Voxel size=%f", _voxelSize);
	UDEBUG("PointToPlane=%d", pointToPlane?1:0);
	UDEBUG("Normal neighborhood=%d", _pointToPlaneK);
	UDEBUG("Normal radius=%f", _pointToPlaneRadius);
	UDEBUG("Max correspondence distance=%f", _maxCorrespondenceDistance);
	UDEBUG("Max Iterations=%d", _maxIterations);
	UDEBUG("Correspondence Ratio=%f", _correspondenceRatio);
	UDEBUG("Max translation=%f", _maxTranslation);
	UDEBUG("Max rotation=%f", _maxRotation);
	UDEBUG("Downsampling step=%d", _downsamplingStep);
	UDEBUG("Force 3DoF=%s", this->force3DoF()?"true":"false");
	UDEBUG("Force 4DoF=%s", _force4DoF?"true":"false");
	UDEBUG("Min Complexity=%f", _pointToPlaneMinComplexity);
	UDEBUG("libpointmatcher (knn=%d, outlier ratio=%f)", _libpointmatcherKnn, _outlierRatio);
	UDEBUG("Strategy=%d", _strategy);

	UTimer timer;
	std::string msg;
	Transform transform;

	SensorData & dataFrom = fromSignature.sensorData();
	SensorData & dataTo = toSignature.sensorData();

	UDEBUG("size before filtering, from=%d (format=%s, max pts=%d) to=%d (format=%s, max pts=%d)",
			dataFrom.laserScanRaw().size(),
			dataFrom.laserScanRaw().formatName().c_str(),
			dataFrom.laserScanRaw().maxPoints(),
			dataTo.laserScanRaw().size(),
			dataTo.laserScanRaw().formatName().c_str(),
			dataTo.laserScanRaw().maxPoints());

	// Do the filtering
	int maxLaserScansFrom = dataFrom.laserScanRaw().maxPoints()>0?dataFrom.laserScanRaw().maxPoints():dataFrom.laserScanRaw().size();
	int maxLaserScansTo = dataTo.laserScanRaw().maxPoints()>0?dataTo.laserScanRaw().maxPoints():dataTo.laserScanRaw().size();

	if(!dataFrom.laserScanRaw().empty())
	{
		int pointsBeforeFiltering = dataFrom.laserScanRaw().size();
		LaserScan fromScan = util3d::commonFiltering(dataFrom.laserScanRaw(),
				_downsamplingStep,
				_rangeMin,
				_rangeMax,
				_voxelSize,
				pointToPlane?_pointToPlaneK:0,
				pointToPlane?_pointToPlaneRadius:0.0f,
				pointToPlane?_pointToPlaneGroundNormalsUp:0.0f);
#ifdef RTABMAP_POINTMATCHER
		if(_strategy==1 && _libpointmatcherICPFilters)
		{
			PM::ICP & filters = *((PM::ICP*)_libpointmatcherICPFilters);
			UDEBUG("icp.referenceDataPointsFilters.size()=%d", (int)filters.referenceDataPointsFilters.size());
			if(filters.referenceDataPointsFilters.size()>1 ||
			   (filters.referenceDataPointsFilters.size() == 1 && filters.referenceDataPointsFilters[0]->className.compare("IdentityDataPointsFilter")!=0))
			{
				try {
					DP data = laserScanToDP(fromScan, true); // ignore local transform to make sure viewpoint is 0,0,0
					filters.referenceDataPointsFilters.apply(data);
					rtabmap::LaserScan pmFromScan = laserScanFromDP(data);
					fromScan = LaserScan(
							pmFromScan,
							fromScan.maxPoints(),
							fromScan.rangeMax(),
							fromScan.localTransform());
				}
				catch(const std::exception & e)
				{
					msg = uFormat("libpointmatcher's data filters have failed: %s", e.what());
					UERROR("%s", msg.c_str());
				}
			}
		}
#endif
		dataFrom.setLaserScan(fromScan);
		float ratio = float(dataFrom.laserScanRaw().size()) / float(pointsBeforeFiltering);
		maxLaserScansFrom = int(float(maxLaserScansFrom) * ratio);
	}
	if(!dataTo.laserScanRaw().empty())
	{
		int pointsBeforeFiltering = dataTo.laserScanRaw().size();
		LaserScan toScan = util3d::commonFiltering(dataTo.laserScanRaw(),
				_downsamplingStep,
				_rangeMin,
				_rangeMax,
				_voxelSize,
				pointToPlane?_pointToPlaneK:0,
				pointToPlane?_pointToPlaneRadius:0.0f,
				pointToPlane?_pointToPlaneGroundNormalsUp:0.0f);
#ifdef RTABMAP_POINTMATCHER
		if(_strategy == 1 && _libpointmatcherICPFilters)
		{
			PM::ICP & filters = *((PM::ICP*)_libpointmatcherICPFilters);
			UDEBUG("icp.readingDataPointsFilters.size()=%d", (int)filters.readingDataPointsFilters.size());
			if(filters.readingDataPointsFilters.size()>1 ||
			   (filters.readingDataPointsFilters.size() == 1 && filters.readingDataPointsFilters[0]->className.compare("IdentityDataPointsFilter")!=0))
			{
				try {
					DP data = laserScanToDP(toScan, true); // ignore local transform to make sure viewpoint is 0,0,0
					filters.readingDataPointsFilters.apply(data);
					rtabmap::LaserScan pmToScan = laserScanFromDP(data);
					toScan = LaserScan(
							pmToScan,
							toScan.maxPoints(),
							toScan.rangeMax(),
							toScan.localTransform());
				}
				catch(const std::exception & e)
				{
					msg = uFormat("libpointmatcher's data filters have failed: %s", e.what());
					UERROR("%s", msg.c_str());
				}
			}
		}
#endif
		dataTo.setLaserScan(toScan);
		float ratio = float(dataTo.laserScanRaw().size()) / float(pointsBeforeFiltering);
		maxLaserScansTo = int(float(maxLaserScansTo) * ratio);
	}

	UDEBUG("size after filtering, from=%d (format=%s, max pts=%d) to=%d (format=%s, max pts=%d), filtering time=%fs",
				dataFrom.laserScanRaw().size(),
				dataFrom.laserScanRaw().formatName().c_str(),
				dataFrom.laserScanRaw().maxPoints(),
				dataTo.laserScanRaw().size(),
				dataTo.laserScanRaw().formatName().c_str(),
				dataTo.laserScanRaw().maxPoints(),
				timer.ticks());


	if(!guess.isNull() && !dataFrom.laserScanRaw().isEmpty() && !dataTo.laserScanRaw().isEmpty())
	{
		// ICP with guess transform
		LaserScan fromScan = dataFrom.laserScanRaw();
		LaserScan toScan = dataTo.laserScanRaw();

		if(fromScan.size() && toScan.size())
		{
			std::string toPrefix = "rtabmap_icp_scan";
			double now = UTimer::now();
			if(!_workingDir.empty() && !_debugExportFormat.empty())
			{
				std::string fromPrefix = "rtabmap_icp_scan";
				if(ULogger::level() == ULogger::kDebug)
				{
					fromPrefix+=uReplaceChar(uFormat("_%.3f_from_%d", fromSignature.id(), now), '.', '_');
					toPrefix+=uReplaceChar(uFormat("_%.3f_to_%d", toSignature.id(), now), '.', '_');
				}
				else
				{
					fromPrefix+="_from";
					toPrefix+="_to";
				}
				if(_debugExportFormat.compare("vtk")==0)
				{
					pcl::io::saveVTKFile(_workingDir+"/"+fromPrefix+".vtk", *util3d::laserScanToPointCloud2(fromScan, fromScan.localTransform()));
					pcl::io::saveVTKFile(_workingDir+"/"+toPrefix+".vtk", *util3d::laserScanToPointCloud2(toScan, guess*toScan.localTransform()));
					UWARN("Saved %s.vtk and %s.vtk (%s=\"%s\") to working directory (%s=%s)", fromPrefix.c_str(), toPrefix.c_str(), Parameters::kIcpDebugExportFormat().c_str(), _debugExportFormat.c_str(), Parameters::kRtabmapWorkingDirectory().c_str(), _workingDir.c_str());
				}
				else if(_debugExportFormat.compare("ply")==0)
				{
					pcl::io::savePLYFile(_workingDir+"/"+fromPrefix+".ply", *util3d::laserScanToPointCloud2(fromScan, fromScan.localTransform()), Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), true);
					pcl::io::savePLYFile(_workingDir+"/"+toPrefix+".ply", *util3d::laserScanToPointCloud2(toScan, guess*toScan.localTransform()), Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), true);
					UWARN("Saved %s.ply and %s.ply (%s=\"%s\") to directory (%s=%s)", fromPrefix.c_str(), toPrefix.c_str(), Parameters::kIcpDebugExportFormat().c_str(), _debugExportFormat.c_str(), Parameters::kRtabmapWorkingDirectory().c_str(), _workingDir.c_str());
				}
				else //pcd
				{
					pcl::io::savePCDFile(_workingDir+"/"+fromPrefix+".pcd", *util3d::laserScanToPointCloud2(fromScan, fromScan.localTransform()), Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), true);
					pcl::io::savePCDFile(_workingDir+"/"+toPrefix+".pcd", *util3d::laserScanToPointCloud2(toScan, guess*toScan.localTransform()), Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), true);
					UWARN("Saved %s.pcd and %s.pcd (%s=\"%s\") to working directory (%s=%s)", fromPrefix.c_str(), toPrefix.c_str(), Parameters::kIcpDebugExportFormat().c_str(), _debugExportFormat.c_str(), Parameters::kRtabmapWorkingDirectory().c_str(), _workingDir.c_str());
				}
			}
			else if(!_debugExportFormat.empty())
			{
				UWARN("%s is enabled but %s is not, cannot export debug scans.", Parameters::kIcpDebugExportFormat().c_str(), Parameters::kRtabmapWorkingDirectory().c_str());
			}

			bool tooLowComplexityForPlaneToPlane = false;
			float secondEigenValue = 1.0f;
			cv::Mat complexityVectors;

			if(pointToPlane && ((fromScan.is2d() || toScan.is2d()) && _strategy==0))
			{
				UWARN("ICP PointToPlane ignored for 2d scans with PCL registration "
						"(some crash issues). Use libpointmatcher (%s=1) or disable %s "
						"to avoid this warning.",
						Parameters::kIcpStrategy().c_str(),
						Parameters::kIcpPointToPlane().c_str());
				pointToPlane = false;
			}

			// If we do PointToPlane, determinate if there is
			// enough structural complexity. If not, fall back to PointToPoint ICP.
			if( pointToPlane &&
				fromScan.hasNormals() &&
				toScan.hasNormals())
			{
				cv::Mat complexityVectorsFrom, complexityVectorsTo;
				cv::Mat complexityValuesFrom, complexityValuesTo;
				double fromComplexity = util3d::computeNormalsComplexity(fromScan, Transform::getIdentity(), &complexityVectorsFrom, &complexityValuesFrom);
				double toComplexity = util3d::computeNormalsComplexity(toScan, guess, &complexityVectorsTo, &complexityValuesTo);
				float complexity = fromComplexity<toComplexity?fromComplexity:toComplexity;
				info.icpStructuralComplexity = complexity;
				if(complexity < _pointToPlaneMinComplexity)
				{
					tooLowComplexityForPlaneToPlane = true;
					pointToPlane = false;
					if(complexity > 0.0f)
					{
						complexityVectors = fromComplexity<toComplexity?complexityVectorsFrom:complexityVectorsTo;

						UASSERT((complexityVectors.rows == 2 && complexityVectors.cols == 2)||
								(complexityVectors.rows == 3 && complexityVectors.cols == 3));
						secondEigenValue = complexityValuesFrom.at<float>(1,0)<complexityValuesTo.at<float>(1,0)?complexityValuesFrom.at<float>(1,0):complexityValuesTo.at<float>(1,0);
						UWARN("ICP PointToPlane ignored as structural complexity is too low (corridor-like environment): (from=%f || to=%f) < %f (%s). Second eigen value=%f. "
							  "PointToPoint is done instead, orientation is still optimized but translation will be limited to "
							  "direction of normals (%s: %s).",
							  fromComplexity, toComplexity, _pointToPlaneMinComplexity, Parameters::kIcpPointToPlaneMinComplexity().c_str(),
							  secondEigenValue,
							  fromComplexity<toComplexity?"From":"To",
							  complexityVectors.rows==2?
									  uFormat("n=%f,%f", complexityVectors.at<float>(0,0), complexityVectors.at<float>(0,1)).c_str():
									  secondEigenValue<_pointToPlaneMinComplexity?
									  uFormat("n=%f,%f,%f", complexityVectors.at<float>(0,0), complexityVectors.at<float>(0,1), complexityVectors.at<float>(0,2)).c_str():
									  uFormat("n1=%f,%f,%f n2=%f,%f,%f", complexityVectors.at<float>(0,0), complexityVectors.at<float>(0,1), complexityVectors.at<float>(0,2), complexityVectors.at<float>(1,0), complexityVectors.at<float>(1,1), complexityVectors.at<float>(1,2)).c_str());
					}
					else
					{
						UWARN("ICP PointToPlane ignored as structural complexity cannot be computed (from=%f to=%f)!? PointToPoint is done instead.", fromComplexity, toComplexity);
					}
					if(ULogger::level() == ULogger::kDebug)
					{
						std::cout << "complexityVectorsFrom = " << std::endl << complexityVectorsFrom << std::endl;
						std::cout << "complexityValuesFrom = " << std::endl << complexityValuesFrom << std::endl;
						std::cout << "complexityVectorsTo = " << std::endl << complexityVectorsTo << std::endl;
						std::cout << "complexityValuesTo = " << std::endl << complexityValuesTo << std::endl;
					}

					// If the complexity is too low and we don't want to compute transform in this case
					if(_pointToPlaneLowComplexityStrategy == 0)
					{
						msg = uFormat("Rejecting transform because too low complexity %f (%s=0)",
								info.icpStructuralComplexity,
								Parameters::kIcpPointToPlaneLowComplexityStrategy().c_str());
						UWARN(msg.c_str());
						info.rejectedMsg = msg;
						return Transform();
					}
				}
			}
			else
			{
				pointToPlane = false;
			}

			Transform icpT;
			bool hasConverged = false;
			float correspondencesRatio = 0.0f;
			int correspondences = 0;
			double variance = 1.0;

			////////////////////
			// Point To Plane
			////////////////////
			if(pointToPlane)
			{
				pcl::PointCloud<pcl::PointXYZINormal>::Ptr fromCloudNormals = util3d::laserScanToPointCloudINormal(fromScan, fromScan.localTransform());
				pcl::PointCloud<pcl::PointXYZINormal>::Ptr toCloudNormals = util3d::laserScanToPointCloudINormal(toScan, guess * toScan.localTransform());

				fromCloudNormals = util3d::removeNaNNormalsFromPointCloud(fromCloudNormals);
				toCloudNormals = util3d::removeNaNNormalsFromPointCloud(toCloudNormals);

				UDEBUG("Conversion time = %f s", timer.ticks());
				pcl::PointCloud<pcl::PointXYZINormal>::Ptr fromCloudNormalsRegistered(new pcl::PointCloud<pcl::PointXYZINormal>());
#ifdef RTABMAP_POINTMATCHER
				if(_strategy==1)
				{
					// Load point clouds
					DP data = laserScanToDP(fromScan);
					DP ref = laserScanToDP(LaserScan(toScan.data(), toScan.maxPoints(), toScan.rangeMax(), toScan.format(), guess * toScan.localTransform()));

					// Compute the transformation to express data in ref
					PM::TransformationParameters T;
					try
					{
						UASSERT(_libpointmatcherICP != 0);
						PM::ICP & icp = *((PM::ICP*)_libpointmatcherICP);
						UDEBUG("libpointmatcher icp... (if there is a seg fault here, make sure all third party libraries are built with same Eigen version.)");
						T = icp(data, ref);
						icpT = Transform::fromEigen3d(Eigen::Affine3d(Eigen::Matrix4d(eigenMatrixToDim<double>(T.template cast<double>(), 4))));
						UDEBUG("libpointmatcher icp...done! T=%s", icpT.prettyPrint().c_str());

						float matchRatio = icp.errorMinimizer->getWeightedPointUsedRatio();
						UDEBUG("match ratio: %f", matchRatio);

						if(!icpT.isNull())
						{
							fromCloudNormalsRegistered = util3d::transformPointCloud(fromCloudNormals, icpT);
							hasConverged = true;
						}
					}
					catch(const std::exception & e)
					{
						msg = uFormat("libpointmatcher has failed: %s", e.what());
					}
				}
				else
#endif
				{
					icpT = util3d::icpPointToPlane(
							fromCloudNormals,
							toCloudNormals,
						   _maxCorrespondenceDistance,
						   _maxIterations,
						   hasConverged,
						   *fromCloudNormalsRegistered,
						   _epsilon,
						   this->force3DoF());
				}

				if(!icpT.isNull() && hasConverged)
				{
					util3d::computeVarianceAndCorrespondences(
							fromCloudNormalsRegistered,
							toCloudNormals,
							_maxCorrespondenceDistance,
							_maxRotation,
							variance,
							correspondences,
							_reciprocalCorrespondences);
				}
			}
			////////////////////
			// Point To Point
			////////////////////
			else
			{
				pcl::PointCloud<pcl::PointXYZI>::Ptr fromCloud = util3d::laserScanToPointCloudI(fromScan, fromScan.localTransform());
				pcl::PointCloud<pcl::PointXYZI>::Ptr toCloud = util3d::laserScanToPointCloudI(toScan, guess * toScan.localTransform());
				UDEBUG("Conversion time = %f s", timer.ticks());

				pcl::PointCloud<pcl::PointXYZI>::Ptr fromCloudRegistered(new pcl::PointCloud<pcl::PointXYZI>());

#ifdef RTABMAP_POINTMATCHER
				if(_strategy==1)
				{
					// Load point clouds
					DP data = laserScanToDP(fromScan);
					DP ref = laserScanToDP(LaserScan(toScan.data(), toScan.maxPoints(), toScan.rangeMax(), toScan.format(), guess*toScan.localTransform()));

					// Compute the transformation to express data in ref
					PM::TransformationParameters T;
					try
					{
						UASSERT(_libpointmatcherICP != 0);
						PM::ICP & icp = *((PM::ICP*)_libpointmatcherICP);
						UDEBUG("libpointmatcher icp... (if there is a seg fault here, make sure all third party libraries are built with same Eigen version.)");
						if(tooLowComplexityForPlaneToPlane)
						{
							// temporary set PointToPointErrorMinimizer
							PM::ICP icpTmp = icp;

							PM::Parameters params;
							params["maxDist"] = uNumber2Str(_voxelSize>0?_voxelSize:_maxCorrespondenceDistance/2.0f);
							UWARN("libpointmatcher icp...temporary maxDist=%s (%s=%f, %s=%f)", params["maxDist"].c_str(),  Parameters::kIcpMaxCorrespondenceDistance().c_str(), _maxCorrespondenceDistance, Parameters::kIcpVoxelSize().c_str(), _voxelSize);
							params["knn"] = uNumber2Str(_libpointmatcherKnn);
							params["epsilon"] = uNumber2Str(_libpointmatcherEpsilon);
							if(_libpointmatcherIntensity)
							{
								icpTmp.matcher.reset(new KDTreeMatcherIntensity<float>(params));
							}
							else
							{
#if POINTMATCHER_VERSION_INT >= 10300
								icpTmp.matcher = PM::get().MatcherRegistrar.create("KDTreeMatcher", params);
#else
								icpTmp.matcher.reset(PM::get().MatcherRegistrar.create("KDTreeMatcher", params));
#endif
							}

#if POINTMATCHER_VERSION_INT >= 10300
							icpTmp.errorMinimizer = PM::get().ErrorMinimizerRegistrar.create("PointToPointErrorMinimizer");
#else
							icpTmp.errorMinimizer.reset(PM::get().ErrorMinimizerRegistrar.create("PointToPointErrorMinimizer"));
#endif
							for(PM::OutlierFilters::iterator iter=icpTmp.outlierFilters.begin(); iter!=icpTmp.outlierFilters.end();)
							{
								if((*iter)->className.compare("SurfaceNormalOutlierFilter") == 0)
								{
									iter = icpTmp.outlierFilters.erase(iter);
								}
								else
								{
									++iter;
								}
							}

							T = icpTmp(data, ref);
						}
						else
						{
							T = icp(data, ref);
						}
						UDEBUG("libpointmatcher icp...done!");
						icpT = Transform::fromEigen3d(Eigen::Affine3d(Eigen::Matrix4d(eigenMatrixToDim<double>(T.template cast<double>(), 4))));

						float matchRatio = icp.errorMinimizer->getWeightedPointUsedRatio();
						UDEBUG("match ratio: %f", matchRatio);

						if(!icpT.isNull())
						{
							hasConverged = true;
						}
					}
					catch(const std::exception & e)
					{
						msg = uFormat("libpointmatcher has failed: %s", e.what());
					}
				}
				else
#endif
				{
#ifdef RTABMAP_CCCORELIB
					if(_strategy==2)
					{
						icpT = icpCC(
								fromCloud,
								toCloud,
								_maxIterations,
								_epsilon,
								 this->force3DoF(),
								 _force4DoF,
								 _ccSamplingLimit,
								 _outlierRatio,
								 _ccFilterOutFarthestPoints,
								 _ccMaxFinalRMS,
								 &info.icpRMS,
								 &msg);
						hasConverged = !icpT.isNull();
					}
					else
#endif
					{
						icpT = util3d::icp(
								fromCloud,
								toCloud,
							   _maxCorrespondenceDistance,
							   _maxIterations,
							   hasConverged,
							   *fromCloudRegistered,
							   _epsilon,
							   this->force3DoF()); // icp2D
					}
				}

				if(!icpT.isNull() && hasConverged)
				{
					if(tooLowComplexityForPlaneToPlane)
					{
						if(_pointToPlaneLowComplexityStrategy == 1)
						{
							Transform guessInv = guess.inverse();
							Transform t = guessInv * icpT.inverse() * guess;
							Eigen::Vector3f v(t.x(), t.y(), t.z());
							if(complexityVectors.cols == 2)
							{
								// limit translation in direction of the first eigen vector
								Eigen::Vector3f n(complexityVectors.at<float>(0,0), complexityVectors.at<float>(0,1), 0.0f);
								float a = v.dot(n);
								Eigen::Vector3f vp = n*a;
								UWARN("Normals low complexity (%f): Limiting translation from (%f,%f) to (%f,%f)",
										info.icpStructuralComplexity,
										v[0], v[1], vp[0], vp[1]);
								v= vp;
							}
							else if(complexityVectors.rows == 3)
							{
								// limit translation in direction of the first and second eigen vectors
								Eigen::Vector3f n1(complexityVectors.at<float>(0,0), complexityVectors.at<float>(0,1), complexityVectors.at<float>(0,2));
								Eigen::Vector3f n2(complexityVectors.at<float>(1,0), complexityVectors.at<float>(1,1), complexityVectors.at<float>(1,2));
								float a = v.dot(n1);
								float b = v.dot(n2);
								Eigen::Vector3f vp = n1*a;
								if(secondEigenValue >= _pointToPlaneMinComplexity)
								{
									vp += n2*b;
								}
								UWARN("Normals low complexity: Limiting translation from (%f,%f,%f) to (%f,%f,%f)",
										v[0], v[1], v[2], vp[0], vp[1], vp[2]);
								v = vp;
							}
							else
							{
								UWARN("not supposed to be here!");
								v = Eigen::Vector3f(0,0,0);
							}
							float roll, pitch, yaw;
							t.getEulerAngles(roll, pitch, yaw);
							t = Transform(v[0], v[1], v[2], roll, pitch, yaw);
							icpT = guess * t.inverse() * guessInv;

							fromCloudRegistered = util3d::transformPointCloud(fromCloud, icpT);
						}
						else
						{
							UWARN("Even if complexity is low (%f), PointToPoint transformation is accepted \"as is\" (%s=2)",
									info.icpStructuralComplexity,
									Parameters::kIcpPointToPlaneLowComplexityStrategy().c_str());
							if(fromCloudRegistered->empty())
								fromCloudRegistered = util3d::transformPointCloud(fromCloud, icpT);
						}
					}
					else if(fromCloudRegistered->empty())
						fromCloudRegistered = util3d::transformPointCloud(fromCloud, icpT);

					util3d::computeVarianceAndCorrespondences(
							fromCloudRegistered,
							toCloud,
							_maxCorrespondenceDistance,
							variance,
							correspondences,
							_reciprocalCorrespondences);
				}
			} // END Registration PointToPLane to PointToPoint
			UDEBUG("ICP (iterations=%d) time = %f s", _maxIterations, timer.ticks());

			if(!icpT.isNull() && hasConverged)
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
					// verify if there are enough correspondences (using "To" by default if set, in case if "From" is merged from multiple scans)
					int maxLaserScans = maxLaserScansTo?maxLaserScansTo:maxLaserScansFrom;
					UDEBUG("Max scans=%d (from=%d, to=%d)", maxLaserScans, maxLaserScansFrom, maxLaserScansTo);

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
						correspondencesRatio = float(correspondences)/float(toScan.size()>fromScan.size()?toScan.size():fromScan.size());
					}

					variance/=10.0;

					UDEBUG("%d->%d hasConverged=%s, variance=%f, correspondences=%d/%d (%f%%), from guess: trans=%f rot=%f",
							dataTo.id(), dataFrom.id(),
							hasConverged?"true":"false",
							variance,
							correspondences,
							maxLaserScans>0?maxLaserScans:(int)(toScan.size()>fromScan.size()?toScan.size():fromScan.size()),
							correspondencesRatio*100.0f,
							info.icpTranslation,
							info.icpRotation);

					if(correspondences == 0)
					{
						UWARN("Transform is found (%s) but no correspondences has been found!? Variance is unknown!",
								icpT.prettyPrint().c_str());
					}
					else
					{
						info.covariance = cv::Mat::eye(6,6,CV_64FC1)*variance;
						info.covariance(cv::Range(3,6),cv::Range(3,6))/=10.0; //orientation error
						if(	((_strategy == 1 && pointToPlane) || _strategy==2) &&
							_force4DoF &&
							!force3DoF())
						{
							// Force4DoF: Assume roll and pitch more accurate (IMU)
							info.covariance(cv::Range(3,5),cv::Range(3,5))/=10.0;
						}
					}
					info.icpInliersRatio = correspondencesRatio;
					info.icpCorrespondences = correspondences;

					if(correspondencesRatio <= _correspondenceRatio)
					{
						msg = uFormat("Cannot compute transform (cor=%d corrRatio=%f/%f maxLaserScans=%d)",
								correspondences, correspondencesRatio, _correspondenceRatio, maxLaserScans);
						UINFO(msg.c_str());
					}
					else
					{
						transform = icpT.inverse()*guess;

						if(!_workingDir.empty() && !_debugExportFormat.empty())
						{
							toPrefix+="_registered";
							if(_debugExportFormat.compare("vtk")==0)
							{
								pcl::io::saveVTKFile(_workingDir+"/"+toPrefix+".vtk", *util3d::laserScanToPointCloud2(toScan, transform*toScan.localTransform()));
								UWARN("Saved %s/%s.vtk (%s=\"%s\")", _workingDir.c_str(), toPrefix.c_str(), Parameters::kIcpDebugExportFormat().c_str(), _debugExportFormat.c_str());
							}
							else if(_debugExportFormat.compare("ply")==0)
							{
								pcl::io::savePLYFile(_workingDir+"/"+toPrefix+".ply", *util3d::laserScanToPointCloud2(toScan, transform*toScan.localTransform()), Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), true);
								UWARN("Saved %s/%s.ply (%s=\"%s\")", _workingDir.c_str(), toPrefix.c_str(), Parameters::kIcpDebugExportFormat().c_str(), _debugExportFormat.c_str());
							}
							else //pcd
							{
								pcl::io::savePCDFile(_workingDir+"/"+toPrefix+".pcd", *util3d::laserScanToPointCloud2(toScan, transform*toScan.localTransform()), Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), true);
								UWARN("Saved %s/%s.pcd (%s=\"%s\")", _workingDir.c_str(), toPrefix.c_str(), Parameters::kIcpDebugExportFormat().c_str(), _debugExportFormat.c_str());
							}
						}
					}
				}
			}
			else
			{
				if(msg.empty())
				{
					msg = uFormat("Cannot compute transform (converged=%s var=%f)",
							hasConverged?"true":"false", variance);
				}
				UINFO(msg.c_str());
			}
		}
		else
		{
			msg = "Laser scans empty ?!?";
			UWARN(msg.c_str());
		}
	}
	else if(!dataFrom.laserScanRaw().empty() && !dataTo.laserScanRaw().empty())
	{
		msg = "RegistrationIcp cannot do registration with a null guess.";
		UERROR(msg.c_str());
	}


	info.rejectedMsg = msg;

	UDEBUG("New transform = %s", transform.prettyPrint().c_str());
	return transform;
}

}
