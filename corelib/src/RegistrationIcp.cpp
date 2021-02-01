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
#include <pcl/conversions.h>
#include <pcl/common/pca.h>
#include <pcl/common/io.h>

#ifdef RTABMAP_POINTMATCHER
#include <fstream>
#include "pointmatcher/PointMatcher.h"
#include "nabo/nabo.h"
typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;

DP pclToDP(const pcl::PointCloud<pcl::PointXYZ>::Ptr & pclCloud, bool is2D)
{
	UDEBUG("");
	typedef DP::Label Label;
	typedef DP::Labels Labels;
	typedef DP::View View;

	if (pclCloud->empty())
		return DP();

	// fill labels
	// conversions of descriptor fields from pcl
	// see http://www.ros.org/wiki/pcl/Overview
	Labels featLabels;
	Labels descLabels;
	std::vector<bool> isFeature;
	featLabels.push_back(Label("x", 1));
	isFeature.push_back(true);
	featLabels.push_back(Label("y", 1));
	isFeature.push_back(true);
	if(!is2D)
	{
		featLabels.push_back(Label("z", 1));
		isFeature.push_back(true);
	}
	featLabels.push_back(Label("pad", 1));

	// create cloud
	DP cloud(featLabels, descLabels, pclCloud->size());
	cloud.getFeatureViewByName("pad").setConstant(1);

	// fill cloud
	View view(cloud.getFeatureViewByName("x"));
	for(unsigned int i=0; i<pclCloud->size(); ++i)
	{
		view(0, i) = pclCloud->at(i).x;
		view(1, i) = pclCloud->at(i).y;
		if(!is2D)
		{
			view(2, i) = pclCloud->at(i).z;
		}
	}

	return cloud;
}

DP pclToDP(const pcl::PointCloud<pcl::PointNormal>::Ptr & pclCloud, bool is2D)
{
	UDEBUG("");
	typedef DP::Label Label;
	typedef DP::Labels Labels;
	typedef DP::View View;

	if (pclCloud->empty())
		return DP();

	// fill labels
	// conversions of descriptor fields from pcl
	// see http://www.ros.org/wiki/pcl/Overview
	Labels featLabels;
	Labels descLabels;
	std::vector<bool> isFeature;
	featLabels.push_back(Label("x", 1));
	isFeature.push_back(true);
	featLabels.push_back(Label("y", 1));
	isFeature.push_back(true);
	if(!is2D)
	{
		featLabels.push_back(Label("z", 1));
		isFeature.push_back(true);
	}

	descLabels.push_back(Label("normals", 3));
	isFeature.push_back(false);
	isFeature.push_back(false);
	isFeature.push_back(false);

	featLabels.push_back(Label("pad", 1));

	// create cloud
	DP cloud(featLabels, descLabels, pclCloud->size());
	cloud.getFeatureViewByName("pad").setConstant(1);

	// fill cloud
	View view(cloud.getFeatureViewByName("x"));
	View viewNormalX(cloud.getDescriptorRowViewByName("normals",0));
	View viewNormalY(cloud.getDescriptorRowViewByName("normals",1));
	View viewNormalZ(cloud.getDescriptorRowViewByName("normals",2));
	for(unsigned int i=0; i<pclCloud->size(); ++i)
	{
		view(0, i) = pclCloud->at(i).x;
		view(1, i) = pclCloud->at(i).y;
		if(!is2D)
		{
			view(2, i) = pclCloud->at(i).z;
		}
		viewNormalX(0, i) = pclCloud->at(i).normal_x;
		viewNormalY(0, i) = pclCloud->at(i).normal_y;
		viewNormalZ(0, i) = pclCloud->at(i).normal_z;
	}

	return cloud;
}

DP laserScanToDP(const rtabmap::LaserScan & scan)
{
	UDEBUG("");
	typedef DP::Label Label;
	typedef DP::Labels Labels;
	typedef DP::View View;

	if (scan.isEmpty())
		return DP();

	// fill labels
	// conversions of descriptor fields from pcl
	// see http://www.ros.org/wiki/pcl/Overview
	Labels featLabels;
	Labels descLabels;
	featLabels.push_back(Label("x", 1));
	featLabels.push_back(Label("y", 1));
	if(!scan.is2d())
	{
		featLabels.push_back(Label("z", 1));
	}
	featLabels.push_back(Label("pad", 1));

	if(scan.hasNormals())
	{
		descLabels.push_back(Label("normals", 3));
	}
	if(scan.hasIntensity())
	{
		descLabels.push_back(Label("intensity", 1));
	}


	// create cloud
	DP cloud(featLabels, descLabels, scan.size());
	cloud.getFeatureViewByName("pad").setConstant(1);

	// fill cloud
	int nx = scan.getNormalsOffset();
	int ny = nx+1;
	int nz = ny+1;
	int offsetI = scan.getIntensityOffset();
	bool hasLocalTransform = !scan.localTransform().isNull() && !scan.localTransform().isIdentity();
	View view(cloud.getFeatureViewByName("x"));
	View viewNormalX(nx!=-1?cloud.getDescriptorRowViewByName("normals",0):view);
	View viewNormalY(nx!=-1?cloud.getDescriptorRowViewByName("normals",1):view);
	View viewNormalZ(nx!=-1?cloud.getDescriptorRowViewByName("normals",2):view);
	View viewIntensity(offsetI!=-1?cloud.getDescriptorRowViewByName("intensity",0):view);
	int oi = 0;
	for(int i=0; i<scan.size(); ++i)
	{
		const float * ptr = scan.data().ptr<float>(0, i);

		if(uIsFinite(ptr[0]) && uIsFinite(ptr[1]) && (scan.is2d() || uIsFinite(ptr[2])))
		{
			if(hasLocalTransform)
			{
				if(nx == -1)
				{
					cv::Point3f pt(ptr[0], ptr[1], scan.is2d()?0:ptr[2]);
					pt = rtabmap::util3d::transformPoint(pt, scan.localTransform());
					view(0, oi) = pt.x;
					view(1, oi) = pt.y;
					if(!scan.is2d())
					{
						view(2, oi) = pt.z;
					}
					if(offsetI!=-1)
					{
						viewIntensity(0, oi) = ptr[offsetI];
					}
					++oi;
				}
				else if(uIsFinite(ptr[nx]) && uIsFinite(ptr[ny]) && uIsFinite(ptr[nz]))
				{
					pcl::PointNormal pt;
					pt.x=ptr[0];
					pt.y=ptr[1];
					pt.z=scan.is2d()?0:ptr[2];
					pt.normal_x=ptr[nx];
					pt.normal_y=ptr[ny];
					pt.normal_z=ptr[nz];
					pt = rtabmap::util3d::transformPoint(pt, scan.localTransform());
					view(0, oi) = pt.x;
					view(1, oi) = pt.y;
					if(!scan.is2d())
					{
						view(2, oi) = pt.z;
					}
					viewNormalX(0, oi) = pt.normal_x;
					viewNormalY(0, oi) = pt.normal_y;
					viewNormalZ(0, oi) = pt.normal_z;

					if(offsetI!=-1)
					{
						viewIntensity(0, oi) = ptr[offsetI];
					}

					++oi;
				}
				else
				{
					UWARN("Ignoring point %d with invalid data: pos=%f %f %f, normal=%f %f %f", i, ptr[0], ptr[1], scan.is2d()?0:ptr[3], ptr[nx], ptr[ny], ptr[nz]);
				}
			}
			else if(nx==-1 || (uIsFinite(ptr[nx]) && uIsFinite(ptr[ny]) && uIsFinite(ptr[nz])))
			{
				view(0, oi) = ptr[0];
				view(1, oi) = ptr[1];
				if(!scan.is2d())
				{
					view(2, oi) = ptr[2];
				}
				if(nx!=-1)
				{
					viewNormalX(0, oi) = ptr[nx];
					viewNormalY(0, oi) = ptr[ny];
					viewNormalZ(0, oi) = ptr[nz];
				}
				if(offsetI!=-1)
				{
					viewIntensity(0, oi) = ptr[offsetI];
				}
				++oi;
			}
			else
			{
				UWARN("Ignoring point %d with invalid data: pos=%f %f %f, normal=%f %f %f", i, ptr[0], ptr[1], scan.is2d()?0:ptr[3], ptr[nx], ptr[ny], ptr[nz]);
			}
		}
		else
		{
			UWARN("Ignoring point %d with invalid data: pos=%f %f %f", i, ptr[0], ptr[1], scan.is2d()?0:ptr[3]);
		}

	}
	if(oi != scan.size())
	{
		cloud.conservativeResize(oi);
	}

	return cloud;
}

void pclFromDP(const DP & cloud, pcl::PointCloud<pcl::PointXYZ> & pclCloud)
{
	UDEBUG("");
	typedef DP::ConstView ConstView;

	if (cloud.features.cols() == 0)
		return;

	pclCloud.resize(cloud.features.cols());
	pclCloud.is_dense = true;

		// fill cloud
	ConstView view(cloud.getFeatureViewByName("x"));
	bool is3D = cloud.featureExists("z");
	for(unsigned int i=0; i<pclCloud.size(); ++i)
	{
		pclCloud.at(i).x = view(0, i);
		pclCloud.at(i).y = view(1, i);
		pclCloud.at(i).z = is3D?view(2, i):0;
	}
}

void pclFromDP(const DP & cloud, pcl::PointCloud<pcl::PointNormal> & pclCloud)
{
	UDEBUG("");
	typedef DP::ConstView ConstView;

	if (cloud.features.cols() == 0)
		return;

	pclCloud.resize(cloud.features.cols());
	pclCloud.is_dense = true;

		// fill cloud
	ConstView view(cloud.getFeatureViewByName("x"));
	bool is3D = cloud.featureExists("z");
	ConstView viewNormalX(cloud.getDescriptorRowViewByName("normals",0));
	ConstView viewNormalY(cloud.getDescriptorRowViewByName("normals",1));
	ConstView viewNormalZ(cloud.getDescriptorRowViewByName("normals",2));
	for(unsigned int i=0; i<pclCloud.size(); ++i)
	{
		pclCloud.at(i).x = view(0, i);
		pclCloud.at(i).y = view(1, i);
		pclCloud.at(i).z = is3D?view(2, i):0;
		pclCloud.at(i).normal_x = viewNormalX(0, i);
		pclCloud.at(i).normal_y = viewNormalY(0, i);
		pclCloud.at(i).normal_z = viewNormalZ(0, i);
	}
}

template<typename T>
typename PointMatcher<T>::TransformationParameters eigenMatrixToDim(const typename PointMatcher<T>::TransformationParameters& matrix, int dimp1)
{
	typedef typename PointMatcher<T>::TransformationParameters M;
	assert(matrix.rows() == matrix.cols());
	assert((matrix.rows() == 3) || (matrix.rows() == 4));
	assert((dimp1 == 3) || (dimp1 == 4));

	if (matrix.rows() == dimp1)
		return matrix;

	M out(M::Identity(dimp1,dimp1));
	out.topLeftCorner(2,2) = matrix.topLeftCorner(2,2);
	out.topRightCorner(2,1) = matrix.topRightCorner(2,1);
	return out;
}

template<typename T>
struct KDTreeMatcherIntensity : public PointMatcher<T>::Matcher
{
	typedef PointMatcherSupport::Parametrizable Parametrizable;
	typedef PointMatcherSupport::Parametrizable P;
	typedef Parametrizable::Parameters Parameters;
	typedef Parametrizable::ParameterDoc ParameterDoc;
	typedef Parametrizable::ParametersDoc ParametersDoc;

	typedef typename Nabo::NearestNeighbourSearch<T> NNS;
	typedef typename NNS::SearchType NNSearchType;

	typedef typename PointMatcher<T>::DataPoints DataPoints;
	typedef typename PointMatcher<T>::Matcher Matcher;
	typedef typename PointMatcher<T>::Matches Matches;
	typedef typename PointMatcher<T>::Matrix Matrix;

	inline static const std::string description()
	{
		return "This matcher matches a point from the reading to its closest neighbors in the reference.";
	}
	inline static const ParametersDoc availableParameters()
	{
		return {
			{"knn", "number of nearest neighbors to consider it the reference", "1", "1", "2147483647", &P::Comp<unsigned>},
			{"epsilon", "approximation to use for the nearest-neighbor search", "0", "0", "inf", &P::Comp<T>},
			{"searchType", "Nabo search type. 0: brute force, check distance to every point in the data (very slow), 1: kd-tree with linear heap, good for small knn (~up to 30) and 2: kd-tree with tree heap, good for large knn (~from 30)", "1", "0", "2", &P::Comp<unsigned>},
			{"maxDist", "maximum distance to consider for neighbors", "inf", "0", "inf", &P::Comp<T>}
		};
	}

	const int knn;
	const T epsilon;
	const NNSearchType searchType;
	const T maxDist;

protected:
	std::shared_ptr<NNS> featureNNS;
	Matrix filteredReferenceIntensity;

public:
	KDTreeMatcherIntensity(const Parameters& params = Parameters()) :
		PointMatcher<T>::Matcher("KDTreeMatcherIntensity", KDTreeMatcherIntensity::availableParameters(), params),
		knn(Parametrizable::get<int>("knn")),
		epsilon(Parametrizable::get<T>("epsilon")),
		searchType(NNSearchType(Parametrizable::get<int>("searchType"))),
		maxDist(Parametrizable::get<T>("maxDist"))
	{
		UINFO("* KDTreeMatcherIntensity: initialized with knn=%d, epsilon=%f, searchType=%d and maxDist=%f", knn, epsilon, searchType, maxDist);
	}
	virtual ~KDTreeMatcherIntensity() {}
	virtual void init(const DataPoints& filteredReference)
	{
		// build and populate NNS
		if(knn>1)
		{
			filteredReferenceIntensity = filteredReference.getDescriptorCopyByName("intensity");
		}
		else
		{
			UWARN("KDTreeMatcherIntensity: knn is not over 1 (%d), intensity re-ordering will be ignored.", knn);
		}
		featureNNS.reset( NNS::create(filteredReference.features, filteredReference.features.rows() - 1, searchType, NNS::TOUCH_STATISTICS));
	}
	virtual PM::Matches findClosests(const DP& filteredReading)
	{
		const int pointsCount(filteredReading.features.cols());
		Matches matches(
			typename Matches::Dists(knn, pointsCount),
			typename Matches::Ids(knn, pointsCount)
		);

		const BOOST_AUTO(filteredReadingIntensity, filteredReading.getDescriptorViewByName("intensity"));

		static_assert(NNS::InvalidIndex == PM::Matches::InvalidId, "");
		static_assert(NNS::InvalidValue == PM::Matches::InvalidDist, "");
		this->visitCounter += featureNNS->knn(filteredReading.features, matches.ids, matches.dists, knn, epsilon, NNS::ALLOW_SELF_MATCH, maxDist);

		if(knn > 1)
		{
			Matches matchesOrderedByIntensity(
				typename Matches::Dists(1, pointsCount),
				typename Matches::Ids(1, pointsCount)
			);
			#pragma omp parallel for
			for (int i = 0; i < pointsCount; ++i)
			{
				float minDistance = std::numeric_limits<float>::max();
				for(int k=0; k<knn && k<filteredReferenceIntensity.rows(); ++k)
				{
					float distIntensity = fabs(filteredReadingIntensity(0,i) - filteredReferenceIntensity(0, matches.ids.coeff(k, i)));
					if(distIntensity < minDistance)
					{
						matchesOrderedByIntensity.ids.coeffRef(0, i) = matches.ids.coeff(k, i);
						matchesOrderedByIntensity.dists.coeffRef(0, i) = matches.dists.coeff(k, i);
						minDistance = distIntensity;
					}
				}
			}
			matches = matchesOrderedByIntensity;
		}
		return matches;
	}
};
#endif

namespace rtabmap {

RegistrationIcp::RegistrationIcp(const ParametersMap & parameters, Registration * child) :
	Registration(parameters, child),
	_maxTranslation(Parameters::defaultIcpMaxTranslation()),
	_maxRotation(Parameters::defaultIcpMaxRotation()),
	_voxelSize(Parameters::defaultIcpVoxelSize()),
	_downsamplingStep(Parameters::defaultIcpDownsamplingStep()),
	_rangeMin(Parameters::defaultIcpRangeMin()),
	_rangeMax(Parameters::defaultIcpRangeMax()),
	_maxCorrespondenceDistance(Parameters::defaultIcpMaxCorrespondenceDistance()),
	_maxIterations(Parameters::defaultIcpIterations()),
	_epsilon(Parameters::defaultIcpEpsilon()),
	_correspondenceRatio(Parameters::defaultIcpCorrespondenceRatio()),
	_pointToPlane(Parameters::defaultIcpPointToPlane()),
	_pointToPlaneK(Parameters::defaultIcpPointToPlaneK()),
	_pointToPlaneRadius(Parameters::defaultIcpPointToPlaneRadius()),
	_pointToPlaneGroundNormalsUp(Parameters::defaultIcpPointToPlaneGroundNormalsUp()),
	_pointToPlaneMinComplexity(Parameters::defaultIcpPointToPlaneMinComplexity()),
	_pointToPlaneLowComplexityStrategy(Parameters::defaultIcpPointToPlaneLowComplexityStrategy()),
	_libpointmatcher(Parameters::defaultIcpPM()),
	_libpointmatcherConfig(Parameters::defaultIcpPMConfig()),
	_libpointmatcherKnn(Parameters::defaultIcpPMMatcherKnn()),
	_libpointmatcherEpsilon(Parameters::defaultIcpPMMatcherEpsilon()),
	_libpointmatcherIntensity(Parameters::defaultIcpPMMatcherIntensity()),
	_libpointmatcherOutlierRatio(Parameters::defaultIcpPMOutlierRatio()),
	_libpointmatcherICP(0)
{
	this->parseParameters(parameters);
}

RegistrationIcp::~RegistrationIcp()
{
#ifdef RTABMAP_POINTMATCHER
	delete (PM::ICP*)_libpointmatcherICP;
#endif
}

void RegistrationIcp::parseParameters(const ParametersMap & parameters)
{
	Registration::parseParameters(parameters);

	Parameters::parse(parameters, Parameters::kIcpMaxTranslation(), _maxTranslation);
	Parameters::parse(parameters, Parameters::kIcpMaxRotation(), _maxRotation);
	Parameters::parse(parameters, Parameters::kIcpVoxelSize(), _voxelSize);
	Parameters::parse(parameters, Parameters::kIcpDownsamplingStep(), _downsamplingStep);
	Parameters::parse(parameters, Parameters::kIcpRangeMin(), _rangeMin);
	Parameters::parse(parameters, Parameters::kIcpRangeMax(), _rangeMax);
	Parameters::parse(parameters, Parameters::kIcpMaxCorrespondenceDistance(), _maxCorrespondenceDistance);
	Parameters::parse(parameters, Parameters::kIcpIterations(), _maxIterations);
	Parameters::parse(parameters, Parameters::kIcpEpsilon(), _epsilon);
	Parameters::parse(parameters, Parameters::kIcpCorrespondenceRatio(), _correspondenceRatio);
	Parameters::parse(parameters, Parameters::kIcpPointToPlane(), _pointToPlane);
	Parameters::parse(parameters, Parameters::kIcpPointToPlaneK(), _pointToPlaneK);
	Parameters::parse(parameters, Parameters::kIcpPointToPlaneRadius(), _pointToPlaneRadius);
	Parameters::parse(parameters, Parameters::kIcpPointToPlaneGroundNormalsUp(), _pointToPlaneGroundNormalsUp);
	Parameters::parse(parameters, Parameters::kIcpPointToPlaneMinComplexity(), _pointToPlaneMinComplexity);
	Parameters::parse(parameters, Parameters::kIcpPointToPlaneLowComplexityStrategy(), _pointToPlaneLowComplexityStrategy);
	UASSERT(_pointToPlaneGroundNormalsUp >= 0.0f && _pointToPlaneGroundNormalsUp <= 1.0f);
	UASSERT(_pointToPlaneMinComplexity >= 0.0f && _pointToPlaneMinComplexity <= 1.0f);

	Parameters::parse(parameters, Parameters::kIcpPM(), _libpointmatcher);
	Parameters::parse(parameters, Parameters::kIcpPMConfig(), _libpointmatcherConfig);
	Parameters::parse(parameters, Parameters::kIcpPMOutlierRatio(), _libpointmatcherOutlierRatio);
	Parameters::parse(parameters, Parameters::kIcpPMMatcherKnn(), _libpointmatcherKnn);
	Parameters::parse(parameters, Parameters::kIcpPMMatcherEpsilon(), _libpointmatcherEpsilon);
	Parameters::parse(parameters, Parameters::kIcpPMMatcherIntensity(), _libpointmatcherIntensity);

#ifndef RTABMAP_POINTMATCHER
	if(_libpointmatcher)
	{
		UWARN("Parameter %s is set to true but RTAB-Map has not been built with libpointmatcher support. Setting to false.", Parameters::kIcpPM().c_str());
		_libpointmatcher = false;
	}
#else
	if(_libpointmatcher)
	{
		UINFO("libpointmatcher enabled! config=\"%s\"", _libpointmatcherConfig.c_str());
		if(_libpointmatcherICP!=0)
		{
			delete (PM::ICP*)_libpointmatcherICP;
			_libpointmatcherICP = 0;
		}

		_libpointmatcherICP = new PM::ICP();

		PM::ICP * icp = (PM::ICP*)_libpointmatcherICP;

		bool useDefaults = true;
		if(!_libpointmatcherConfig.empty())
		{
			// load YAML config
			std::ifstream ifs(_libpointmatcherConfig.c_str());
			if (ifs.good())
			{
				icp->loadFromYaml(ifs);
				useDefaults = false;
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

			params["ratio"] = uNumber2Str(_libpointmatcherOutlierRatio);
			icp->outlierFilters.clear();
			icp->outlierFilters.push_back(PM::get().OutlierFilterRegistrar.create("TrimmedDistOutlierFilter", params));
			params.clear();
			if(_pointToPlane)
			{
				params["maxAngle"] = uNumber2Str(_maxRotation<=0.0f?M_PI:_maxRotation);
				icp->outlierFilters.push_back(PM::get().OutlierFilterRegistrar.create("SurfaceNormalOutlierFilter", params));
				params.clear();

				params["force2D"] = force3DoF()?"1":"0";
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
	}
#endif

	UASSERT_MSG(_voxelSize >= 0, uFormat("value=%d", _voxelSize).c_str());
	UASSERT_MSG(_downsamplingStep >= 0, uFormat("value=%d", _downsamplingStep).c_str());
	UASSERT_MSG(_maxCorrespondenceDistance > 0.0f, uFormat("value=%f", _maxCorrespondenceDistance).c_str());
	UASSERT_MSG(_maxIterations > 0, uFormat("value=%d", _maxIterations).c_str());
	UASSERT(_epsilon >= 0.0f);
	UASSERT_MSG(_correspondenceRatio >=0.0f && _correspondenceRatio <=1.0f, uFormat("value=%f", _correspondenceRatio).c_str());
	UASSERT_MSG(!_pointToPlane || (_pointToPlane && (_pointToPlaneK > 0 || _pointToPlaneRadius > 0.0f)), uFormat("_pointToPlaneK=%d _pointToPlaneRadius=%f", _pointToPlaneK, _pointToPlaneRadius).c_str());
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
	UDEBUG("Normal neighborhood=%d", _pointToPlaneK);
	UDEBUG("Normal radius=%f", _pointToPlaneRadius);
	UDEBUG("Max correspondence distance=%f", _maxCorrespondenceDistance);
	UDEBUG("Max Iterations=%d", _maxIterations);
	UDEBUG("Correspondence Ratio=%f", _correspondenceRatio);
	UDEBUG("Max translation=%f", _maxTranslation);
	UDEBUG("Max rotation=%f", _maxRotation);
	UDEBUG("Downsampling step=%d", _downsamplingStep);
	UDEBUG("libpointmatcher=%d (knn=%d, outlier ratio=%f)", _libpointmatcher?1:0, _libpointmatcherKnn, _libpointmatcherOutlierRatio);

	UTimer timer;
	std::string msg;
	Transform transform;

	SensorData & dataFrom = fromSignature.sensorData();
	SensorData & dataTo = toSignature.sensorData();

	UDEBUG("size from=%d (format=%d, max pts=%d) to=%d (format=%d, max pts=%d)",
			dataFrom.laserScanRaw().size(),
			(int)dataFrom.laserScanRaw().format(),
			dataFrom.laserScanRaw().maxPoints(),
			dataTo.laserScanRaw().size(),
			(int)dataTo.laserScanRaw().format(),
			dataTo.laserScanRaw().maxPoints());

	if(!guess.isNull() && !dataFrom.laserScanRaw().isEmpty() && !dataTo.laserScanRaw().isEmpty())
	{
		// ICP with guess transform
		LaserScan fromScan = dataFrom.laserScanRaw();
		LaserScan toScan = dataTo.laserScanRaw();
		if(_downsamplingStep>1 || _rangeMin >0.0f || _rangeMax > 0.0f)
		{
			fromScan = util3d::commonFiltering(fromScan, _downsamplingStep, _rangeMin, _rangeMax);
			toScan = util3d::commonFiltering(toScan, _downsamplingStep, _rangeMin, _rangeMax);
			UDEBUG("Downsampling and/or range filtering time (step=%d, min=%fm, max=%fm) = %f s", _downsamplingStep, _rangeMin, _rangeMax,  timer.ticks());
		}

		if(fromScan.size() && toScan.size())
		{
			Transform icpT;
			bool hasConverged = false;
			float correspondencesRatio = 0.0f;
			int correspondences = 0;
			double variance = 1.0;
			bool transformComputed = false;
			bool tooLowComplexityForPlaneToPlane = false;
			float secondEigenValue = 1.0f;
			cv::Mat complexityVectors;

			if( _pointToPlane &&
				_voxelSize == 0.0f &&
				fromScan.hasNormals() &&
				toScan.hasNormals() &&
				!((fromScan.is2d() || toScan.is2d()) && !_libpointmatcher)) // PCL crashes if 2D)
			{
				//special case if we have already normals computed and there is no filtering

				cv::Mat complexityVectorsFrom, complexityVectorsTo;
				cv::Mat complexityValuesFrom, complexityValuesTo;
				double fromComplexity = util3d::computeNormalsComplexity(fromScan, Transform::getIdentity(), &complexityVectorsFrom, &complexityValuesFrom);
				double toComplexity = util3d::computeNormalsComplexity(toScan, guess, &complexityVectorsTo, &complexityValuesTo);
				float complexity = fromComplexity<toComplexity?fromComplexity:toComplexity;
				info.icpStructuralComplexity = complexity;
				if(complexity < _pointToPlaneMinComplexity)
				{
					tooLowComplexityForPlaneToPlane = true;
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
				}
				else
				{
					pcl::PointCloud<pcl::PointXYZINormal>::Ptr fromCloudNormals = util3d::laserScanToPointCloudINormal(fromScan, fromScan.localTransform());
					pcl::PointCloud<pcl::PointXYZINormal>::Ptr toCloudNormals = util3d::laserScanToPointCloudINormal(toScan, guess * toScan.localTransform());

					fromCloudNormals = util3d::removeNaNNormalsFromPointCloud(fromCloudNormals);
					toCloudNormals = util3d::removeNaNNormalsFromPointCloud(toCloudNormals);

					if(fromCloudNormals->size() > 2 && toCloudNormals->size() > 2)
					{
						pcl::PCA<pcl::PointXYZINormal> pca;
						pca.setInputCloud(fromCloudNormals);
						Eigen::Vector3f valuesFrom = pca.getEigenValues();
						pca.setInputCloud(toCloudNormals);
						Eigen::Vector3f valuesTo = pca.getEigenValues();
						if(valuesFrom[0]/fromCloudNormals->size() < valuesTo[0]/toCloudNormals->size())
						{
							info.icpStructuralDistribution = sqrt(valuesFrom[0]/fromCloudNormals->size());
						}
						else
						{
							info.icpStructuralDistribution = sqrt(valuesTo[0]/toCloudNormals->size());
						}
					}

					UDEBUG("Conversion time = %f s", timer.ticks());
					pcl::PointCloud<pcl::PointXYZINormal>::Ptr fromCloudNormalsRegistered(new pcl::PointCloud<pcl::PointXYZINormal>());
#ifdef RTABMAP_POINTMATCHER
					if(_libpointmatcher)
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
								correspondences);
					}
					transformComputed = true;
				}
			}

			int maxLaserScansFrom = fromScan.maxPoints();
			int maxLaserScansTo = toScan.maxPoints();
			if(!transformComputed)
			{
				pcl::PointCloud<pcl::PointXYZI>::Ptr fromCloud = util3d::laserScanToPointCloudI(fromScan, fromScan.localTransform());
				pcl::PointCloud<pcl::PointXYZI>::Ptr toCloud = util3d::laserScanToPointCloudI(toScan, guess * toScan.localTransform());
				UDEBUG("Conversion time = %f s", timer.ticks());

				if(fromCloud->size() > 2 && toCloud->size() > 2)
				{
					pcl::PCA<pcl::PointXYZI> pca;
					pca.setInputCloud(fromCloud);
					Eigen::Vector3f valuesFrom = pca.getEigenValues();
					pca.setInputCloud(toCloud);
					Eigen::Vector3f valuesTo = pca.getEigenValues();
					if(valuesFrom[0]/fromCloud->size() < valuesTo[0]/toCloud->size())
					{
						info.icpStructuralDistribution = sqrt(valuesFrom[0]/fromCloud->size());
					}
					else
					{
						info.icpStructuralDistribution = sqrt(valuesTo[0]/toCloud->size());
					}
					UDEBUG("Computed icpStructuralDistribution %f s",timer.ticks());
				}

				pcl::PointCloud<pcl::PointXYZI>::Ptr fromCloudFiltered = fromCloud;
				pcl::PointCloud<pcl::PointXYZI>::Ptr toCloudFiltered = toCloud;
				if(_voxelSize > 0.0f)
				{
					float pointsBeforeFiltering = (float)fromCloudFiltered->size();
					fromCloudFiltered = util3d::voxelize(fromCloudFiltered, _voxelSize);
					float ratioFrom = float(fromCloudFiltered->size()) / pointsBeforeFiltering;
					maxLaserScansFrom = int(float(maxLaserScansFrom) * ratioFrom);

					pointsBeforeFiltering = (float)toCloudFiltered->size();
					toCloudFiltered = util3d::voxelize(toCloudFiltered, _voxelSize);
					float ratioTo = float(toCloudFiltered->size()) / pointsBeforeFiltering;
					maxLaserScansTo = int(float(maxLaserScansTo) * ratioTo);

					UDEBUG("Voxel filtering time (voxel=%f m, ratioFrom=%f->%d/%d ratioTo=%f->%d/%d) = %f s",
							_voxelSize,
							ratioFrom,
							(int)fromCloudFiltered->size(),
							maxLaserScansFrom,
							ratioTo,
							(int)toCloudFiltered->size(),
							maxLaserScansTo,
							timer.ticks());
				}

				pcl::PointCloud<pcl::PointXYZI>::Ptr fromCloudRegistered(new pcl::PointCloud<pcl::PointXYZI>());
				if(_pointToPlane && // ICP Point To Plane
					!tooLowComplexityForPlaneToPlane && // if previously rejected above
					!((fromScan.is2d()|| toScan.is2d()) && !_libpointmatcher)) // PCL crashes if 2D
				{
					Eigen::Vector3f viewpointFrom(fromScan.localTransform().x(), fromScan.localTransform().y(), fromScan.localTransform().z());
					pcl::PointCloud<pcl::Normal>::Ptr normalsFrom;
					if(fromScan.is2d())
					{
						if(_voxelSize > 0.0f)
						{
							normalsFrom = util3d::computeNormals2D(
									fromCloudFiltered,
									_pointToPlaneK,
									_pointToPlaneRadius,
									viewpointFrom);
						}
						else
						{
							normalsFrom = util3d::computeFastOrganizedNormals2D(
									fromCloudFiltered,
									_pointToPlaneK,
									_pointToPlaneRadius,
									viewpointFrom);
						}
					}
					else
					{
						normalsFrom = util3d::computeNormals(fromCloudFiltered, _pointToPlaneK, _pointToPlaneRadius, viewpointFrom);
					}

					Transform toT = guess * toScan.localTransform();
					Eigen::Vector3f viewpointTo(toT.x(), toT.y(), toT.z());
					pcl::PointCloud<pcl::Normal>::Ptr normalsTo;
					if(toScan.is2d())
					{
						if(_voxelSize > 0.0f)
						{
							normalsTo = util3d::computeNormals2D(
									toCloudFiltered,
									_pointToPlaneK,
									_pointToPlaneRadius,
									viewpointTo);
						}
						else
						{
							normalsTo = util3d::computeFastOrganizedNormals2D(
									toCloudFiltered,
									_pointToPlaneK,
									_pointToPlaneRadius,
									viewpointTo);
						}
					}
					else
					{
						normalsTo = util3d::computeNormals(toCloudFiltered, _pointToPlaneK, _pointToPlaneRadius, viewpointTo);
					}

					cv::Mat complexityVectorsFrom, complexityVectorsTo;
					cv::Mat complexityValuesFrom, complexityValuesTo;
					double fromComplexity = util3d::computeNormalsComplexity(*normalsFrom, Transform::getIdentity(), fromScan.is2d(), &complexityVectorsFrom, &complexityValuesFrom);
					double toComplexity = util3d::computeNormalsComplexity(*normalsTo, Transform::getIdentity(), toScan.is2d(), &complexityVectorsTo, &complexityValuesTo);
					float complexity = fromComplexity<toComplexity?fromComplexity:toComplexity;
					info.icpStructuralComplexity = complexity;
					if(complexity < _pointToPlaneMinComplexity)
					{
						tooLowComplexityForPlaneToPlane = true;
						if(complexity > 0.0f)
						{
							complexityVectors = fromComplexity<toComplexity?complexityVectorsFrom:complexityVectorsTo;
							UASSERT((complexityVectors.rows == 2 && complexityVectors.cols == 2)||
									(complexityVectors.rows == 3 && complexityVectors.cols == 3));
							UWARN("ICP PointToPlane ignored as structural complexity is too low (corridor-like environment): (from=%f || to=%f) < %f (%s). "
								  "PointToPoint is done instead, orientation is still optimized but translation will be limited to "
								  "direction of normals (%s: %s).",
								  fromComplexity, toComplexity, _pointToPlaneMinComplexity, Parameters::kIcpPointToPlaneMinComplexity().c_str(),
								  fromComplexity<toComplexity?"From":"To",
								  complexityVectors.rows==2?
										  uFormat("n=%f,%f", complexityVectors.at<float>(0,0), complexityVectors.at<float>(0,1)).c_str():
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
					}
					else
					{
						pcl::PointCloud<pcl::PointXYZINormal>::Ptr fromCloudNormals(new pcl::PointCloud<pcl::PointXYZINormal>);
						pcl::concatenateFields(*fromCloudFiltered, *normalsFrom, *fromCloudNormals);

						pcl::PointCloud<pcl::PointXYZINormal>::Ptr toCloudNormals(new pcl::PointCloud<pcl::PointXYZINormal>);
						pcl::concatenateFields(*toCloudFiltered, *normalsTo, *toCloudNormals);

						std::vector<int> indices;
						toCloudNormals = util3d::removeNaNNormalsFromPointCloud(toCloudNormals);
						fromCloudNormals = util3d::removeNaNNormalsFromPointCloud(fromCloudNormals);

						if(!fromCloudNormals->empty() && !fromScan.is2d() && _pointToPlaneGroundNormalsUp>0.0f)
						{
							util3d::adjustNormalsToViewPoint(fromCloudNormals,
									Eigen::Vector3f(fromScan.localTransform().x(),fromScan.localTransform().y(),fromScan.localTransform().z()+10),
									_pointToPlaneGroundNormalsUp);
						}
						if(!toCloudNormals->empty() && !toScan.is2d() && _pointToPlaneGroundNormalsUp>0.0f)
						{
							Transform toT = guess * toScan.localTransform();
							Eigen::Vector3f viewpointTo(toT.x(), toT.y(), toT.z()+10);
							util3d::adjustNormalsToViewPoint(toCloudNormals,
									viewpointTo,
									_pointToPlaneGroundNormalsUp);
						}

						// update output scans
						if(fromScan.is2d())
						{
							fromSignature.sensorData().setLaserScan(
									LaserScan(
											util3d::laserScan2dFromPointCloud(*fromCloudNormals, fromScan.localTransform().inverse()),
											maxLaserScansFrom,
											fromScan.rangeMax(),
											LaserScan::kXYINormal,
											fromScan.localTransform()));
						}
						else
						{
							fromSignature.sensorData().setLaserScan(
									LaserScan(
											util3d::laserScanFromPointCloud(*fromCloudNormals, fromScan.localTransform().inverse()),
											maxLaserScansFrom,
											fromScan.rangeMax(),
											LaserScan::kXYZINormal,
											fromScan.localTransform()));
						}
						if(toScan.is2d())
						{
							toSignature.sensorData().setLaserScan(
									LaserScan(
											util3d::laserScan2dFromPointCloud(*toCloudNormals, (guess*toScan.localTransform()).inverse()),
											maxLaserScansTo,
											toScan.rangeMax(),
											LaserScan::kXYINormal,
											toScan.localTransform()));
						}
						else
						{
							toSignature.sensorData().setLaserScan(
									LaserScan(
											util3d::laserScanFromPointCloud(*toCloudNormals, (guess*toScan.localTransform()).inverse()),
											maxLaserScansTo,
											toScan.rangeMax(),
											LaserScan::kXYZINormal,
											toScan.localTransform()));
						}
						UDEBUG("Compute normals (%d,%d) time = %f s", (int)fromCloudNormals->size(), (int)toCloudNormals->size(), timer.ticks());
						fromScan = fromSignature.sensorData().laserScanRaw();
						toScan = toSignature.sensorData().laserScanRaw();

						if(toCloudNormals->size() && fromCloudNormals->size())
						{
							pcl::PointCloud<pcl::PointXYZINormal>::Ptr fromCloudNormalsRegistered(new pcl::PointCloud<pcl::PointXYZINormal>());

#ifdef RTABMAP_POINTMATCHER
							if(_libpointmatcher)
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
									T = icp(data, ref);
									UDEBUG("libpointmatcher icp...done!");
									icpT = Transform::fromEigen3d(Eigen::Affine3d(Eigen::Matrix4d(eigenMatrixToDim<double>(T.template cast<double>(), 4))));

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
										correspondences);
							}
						}
						transformComputed = true;
					}
				}

				if(!transformComputed) // ICP Point to Point
				{
					if(_pointToPlane && !tooLowComplexityForPlaneToPlane && ((fromScan.is2d() || toScan.is2d()) && !_libpointmatcher))
					{
						UWARN("ICP PointToPlane ignored for 2d scans with PCL registration (some crash issues). Use libpointmatcher (%s) or disable %s to avoid this warning.", Parameters::kIcpPM().c_str(), Parameters::kIcpPointToPlane().c_str());
					}

					if(_voxelSize > 0.0f || !tooLowComplexityForPlaneToPlane)
					{
						// update output scans
						if(fromScan.is2d())
						{
							fromSignature.sensorData().setLaserScan(
									LaserScan(
											util3d::laserScan2dFromPointCloud(*fromCloudFiltered, fromScan.localTransform().inverse()),
											maxLaserScansFrom,
											fromScan.rangeMax(),
											LaserScan::kXYI,
											fromScan.localTransform()));
						}
						else
						{
							fromSignature.sensorData().setLaserScan(
									LaserScan(
											util3d::laserScanFromPointCloud(*fromCloudFiltered, fromScan.localTransform().inverse()),
											maxLaserScansFrom,
											fromScan.rangeMax(),
											LaserScan::kXYZI,
											fromScan.localTransform()));
						}
						if(toScan.is2d())
						{
							toSignature.sensorData().setLaserScan(
									LaserScan(
											util3d::laserScan2dFromPointCloud(*toCloudFiltered, (guess*toScan.localTransform()).inverse()),
											maxLaserScansTo,
											toScan.rangeMax(),
											LaserScan::kXYI,
											toScan.localTransform()));
						}
						else
						{
							toSignature.sensorData().setLaserScan(
									LaserScan(
											util3d::laserScanFromPointCloud(*toCloudFiltered, (guess*toScan.localTransform()).inverse()),
											maxLaserScansTo,
											toScan.rangeMax(),
											LaserScan::kXYZI,
											toScan.localTransform()));
						}
						fromScan = fromSignature.sensorData().laserScanRaw();
						toScan = toSignature.sensorData().laserScanRaw();
					}

#ifdef RTABMAP_POINTMATCHER
					if(_libpointmatcher)
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
							if(_pointToPlane)
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
								fromCloudRegistered = util3d::transformPointCloud(fromCloudFiltered, icpT);
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

					if(!icpT.isNull() && hasConverged)
					{
						if(tooLowComplexityForPlaneToPlane && _pointToPlaneLowComplexityStrategy<2)
						{
							if(complexityVectors.empty() || _pointToPlaneLowComplexityStrategy == 0)
							{
								msg = uFormat("Rejecting transform because too low complexity (%s=0)", Parameters::kIcpPointToPlaneLowComplexityStrategy().c_str());
								icpT.setNull();
								UWARN(msg.c_str());
							}
							else //if(_pointToPlaneLowComplexityStrategy == 1)
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
									UWARN("Normals low complexity: Limiting translation from (%f,%f) to (%f,%f)",
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

								if(fromScan.hasNormals() && toScan.hasNormals())
								{
									// we were using normals, so compute correspondences using normals
									pcl::PointCloud<pcl::PointXYZINormal>::Ptr fromCloudNormalsRegistered = util3d::laserScanToPointCloudINormal(fromScan, icpT * fromScan.localTransform());
									pcl::PointCloud<pcl::PointXYZINormal>::Ptr toCloudNormals = util3d::laserScanToPointCloudINormal(toScan, guess * toScan.localTransform());

									util3d::computeVarianceAndCorrespondences(
											fromCloudNormalsRegistered,
											toCloudNormals,
											_maxCorrespondenceDistance,
											_maxRotation,
											variance,
											correspondences);
								}
								else
								{
									util3d::computeVarianceAndCorrespondences(
											fromCloudRegistered,
											toCloudFiltered,
											_maxCorrespondenceDistance,
											variance,
											correspondences);
								}
							}
						}
						else
						{
							if(tooLowComplexityForPlaneToPlane)
							{
								UWARN("Even if complexity is low , PointToPoint transformation is accepted \"as is\" (%s=2)", Parameters::kIcpPointToPlaneLowComplexityStrategy().c_str());
							}
							util3d::computeVarianceAndCorrespondences(
									fromCloudRegistered,
									toCloudFiltered,
									_maxCorrespondenceDistance,
									variance,
									correspondences);
						}
					}
				}
			}
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
	else if(dataTo.isValid())
	{
		if(guess.isNull())
		{
			msg = "RegistrationIcp cannot do registration with a null guess.";
		}
		else
		{
			msg = uFormat("Laser scans empty?!? (new[%d]=%d old[%d]=%d)",
					dataTo.id(), dataTo.laserScanRaw().size(),
					dataFrom.id(), dataFrom.laserScanRaw().size());
		}
		UERROR(msg.c_str());
	}


	info.rejectedMsg = msg;

	UDEBUG("New transform = %s", transform.prettyPrint().c_str());
	return transform;
}

}
