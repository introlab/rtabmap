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
#include <rtabmap/utilite/UDirectory.h>
#include <pcl/conversions.h>
#include <pcl/common/pca.h>
#include <pcl/common/io.h>

#ifdef RTABMAP_CCCORELIB
#include <CCCoreLib/RegistrationTools.h>

rtabmap::Transform icpCC(
		const pcl::PointCloud<pcl::PointXYZI>::Ptr & fromCloud,
		pcl::PointCloud<pcl::PointXYZI>::Ptr & toCloud,
		int maxIterations = 150,
		double minRMSDecrease = 0.00001,
		bool force3DoF = false,
		bool force4DoF = false,
		int samplingLimit = 50000,
		double finalOverlapRatio = 0.85,
		bool filterOutFarthestPoints = false,
		double maxFinalRMS = 0.2,
		std::string * errorMsg = 0)
{
	UDEBUG("maxIterations=%d", maxIterations);
	UDEBUG("minRMSDecrease=%f", minRMSDecrease);
	UDEBUG("samplingLimit=%d", samplingLimit);
	UDEBUG("finalOverlapRatio=%f", finalOverlapRatio);
	UDEBUG("filterOutFarthestPoints=%s", filterOutFarthestPoints?"true":"false");
	UDEBUG("force 3DoF=%s 4DoF=%s", force3DoF?"true":"false", force4DoF?"true":"false");
	UDEBUG("maxFinalRMS=%f", maxFinalRMS);

	rtabmap::Transform icpTransformation;

	CCCoreLib::ICPRegistrationTools::RESULT_TYPE result;
	CCCoreLib::PointProjectionTools::Transformation transform;
	CCCoreLib::ICPRegistrationTools::Parameters params;
	{
		if(minRMSDecrease > 0.0)
		{
			params.convType = CCCoreLib::ICPRegistrationTools::MAX_ERROR_CONVERGENCE;
			params.minRMSDecrease = minRMSDecrease; //! The minimum error (RMS) reduction between two consecutive steps to continue process (ignored if convType is not MAX_ERROR_CONVERGENCE)
		}
		else
		{
			params.convType = CCCoreLib::ICPRegistrationTools::MAX_ITER_CONVERGENCE;
			params.nbMaxIterations = maxIterations; //! The maximum number of iteration (ignored if convType is not MAX_ITER_CONVERGENCE)
		}
		params.adjustScale = false;             //! Whether to release the scale parameter during the registration procedure or not
		params.filterOutFarthestPoints = filterOutFarthestPoints; //! If true, the algorithm will automatically ignore farthest points from the reference, for better convergence
		params.samplingLimit = samplingLimit; //! Maximum number of points per cloud (they are randomly resampled below this limit otherwise)
		params.finalOverlapRatio = finalOverlapRatio;  //! Theoretical overlap ratio (at each iteration, only this percentage (between 0 and 1) will be used for registration
		params.modelWeights = nullptr;          //! Weights for model points (i.e. only if the model entity is a cloud) (optional)
		params.dataWeights = nullptr;           //! Weights for data points (optional)
		params.transformationFilters = force3DoF?33:force4DoF?1:0;  //! Filters to be applied on the resulting transformation at each step (experimental) - see RegistrationTools::TRANSFORMATION_FILTERS flags
		params.maxThreadCount = 0;              //! Maximum number of threads to use (0 = max)
	}

	double finalError = 0.0;
	unsigned finalPointCount = 0;

	CCCoreLib::PointCloud toPointCloud = CCCoreLib::PointCloud();
	CCCoreLib::PointCloud fromPointCloud = CCCoreLib::PointCloud();

	fromPointCloud.reserve(fromCloud->points.size());
	for(uint nIndex=0; nIndex < fromCloud->points.size(); nIndex++)
	{
		CCVector3 P;
		P.x = fromCloud->points[nIndex].x;
		P.y = fromCloud->points[nIndex].y;
		P.z = fromCloud->points[nIndex].z;
		fromPointCloud.addPoint(P);
	}
	toPointCloud.reserve(toCloud->points.size());
	for(uint nIndex=0; nIndex < toCloud->points.size(); nIndex++)
	{
		CCVector3 P;
		P.x = toCloud->points[nIndex].x;
		P.y = toCloud->points[nIndex].y;
		P.z = toCloud->points[nIndex].z;
		toPointCloud.addPoint(P);
	}

	UDEBUG("CCCoreLib: start ICP");
	result = CCCoreLib::ICPRegistrationTools::Register(
			&fromPointCloud,
			nullptr,
			&toPointCloud,
			params,
			transform,
			finalError,
			finalPointCount);
	UDEBUG("CCCoreLib: ICP done!");

	UDEBUG("CC ICP result: %d", result);
	UDEBUG("CC Final error: %f . Finall Pointcount: %d", finalError, finalPointCount);
	UDEBUG("CC ICP success Trans: %f %f %f", transform.T.x,transform.T.y,transform.T.z);

	if(result != 1)
	{
		std::string msg = uFormat("CCCoreLib has failed: Rejecting transform as result %d !=1", result);
		UDEBUG(msg.c_str());
		if(errorMsg)
		{
			*errorMsg = msg;
		}

		icpTransformation.setNull();
		return icpTransformation;
	}
	else if(finalPointCount <10)
	{
		std::string msg = uFormat("CCCoreLib has failed: Rejecting transform as finalPointCount %d < 10 ", finalPointCount);
		UDEBUG(msg.c_str());
		if(errorMsg)
		{
			*errorMsg = msg;
		}

		icpTransformation.setNull();
		return icpTransformation;
	}
	//CC transform to EIgen4f
	Eigen::Matrix4f matrix;
	matrix.setIdentity();
	for(int i=0;i<3;i++)
	{
		for(int j=0;j<3;j++)
		{
			matrix(i,j)=transform.R.getValue(i,j);
		}
	}
	for(int i=0;i<3;i++)
	{
		matrix(i,3)=transform.T[i];
	}

	icpTransformation = rtabmap::Transform::fromEigen4f(matrix);
	icpTransformation = icpTransformation.inverse();
	UDEBUG("CC ICP result: %s", icpTransformation.prettyPrint().c_str());

	if(finalError > maxFinalRMS)
	{
		std::string msg = uFormat("CCCoreLib has failed: Rejecting transform as RMS %f > %f (%s) ", finalError, maxFinalRMS, rtabmap::Parameters::kIcpCCMaxFinalRMS().c_str());
		UDEBUG(msg.c_str());
		if(errorMsg)
		{
			*errorMsg = msg;
		}
		icpTransformation.setNull();
	}
	return icpTransformation;
}
#endif

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

DP laserScanToDP(const rtabmap::LaserScan & scan, bool ignoreLocalTransform = false)
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
	bool hasLocalTransform = !ignoreLocalTransform && !scan.localTransform().isNull() && !scan.localTransform().isIdentity();
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

rtabmap::LaserScan laserScanFromDP(const DP & cloud, const rtabmap::Transform & localTransform = rtabmap::Transform::getIdentity())
{
	UDEBUG("");
	typedef DP::ConstView ConstView;

	rtabmap::LaserScan scan;

	if (cloud.features.cols() == 0)
		return rtabmap::LaserScan();

	// fill cloud
	bool transformValid = !localTransform.isNull() && !localTransform.isIdentity();
	rtabmap::Transform localTransformInv;
	if(transformValid)
		localTransformInv = localTransform.inverse();
	bool is3D = cloud.featureExists("z");
	bool hasNormals = cloud.descriptorExists("normals");
	bool hasIntensity = cloud.descriptorExists("intensity");
	ConstView view(cloud.getFeatureViewByName("x"));
	ConstView viewNormalX(hasNormals?cloud.getDescriptorRowViewByName("normals",0):view);
	ConstView viewNormalY(hasNormals?cloud.getDescriptorRowViewByName("normals",1):view);
	ConstView viewNormalZ(hasNormals?cloud.getDescriptorRowViewByName("normals",2):view);
	ConstView viewIntensity(hasIntensity?cloud.getDescriptorRowViewByName("intensity",0):view);
	int channels = 2+(is3D?1:0) + (hasNormals?3:0) + (hasIntensity?1:0);
	cv::Mat data(1, cloud.features.cols(), CV_32FC(channels));
	for(unsigned int i=0; i<cloud.features.cols(); ++i)
	{
		pcl::PointXYZINormal pt;
		pt.x = view(0, i);
		pt.y = view(1, i);
		if(is3D)
			pt.z = view(2, i);
		if(hasIntensity)
			pt.intensity = viewIntensity(0, i);
		if(hasNormals) {
			pt.normal_x = viewNormalX(0, i);
			pt.normal_y = viewNormalY(0, i);
			pt.normal_z = viewNormalZ(0, i);
		}
		if(transformValid)
			pt = rtabmap::util3d::transformPoint(pt, localTransformInv);

		float * value = data.ptr<float>(0, i);
		int index = 0;
		value[index++] = pt.x;
		value[index++] = pt.y;
		if(is3D)
			value[index++] = pt.z;
		if(hasIntensity)
			value[index++] = pt.intensity;
		if(hasNormals) {
			value[index++] = pt.normal_x;
			value[index++] = pt.normal_y;
			value[index++] = pt.normal_z;
		}
	}

	UASSERT(data.channels() >= 2 && data.channels() <=7);
	return rtabmap::LaserScan(data, 0, 0,
			data.channels()==2?rtabmap::LaserScan::kXY:
			data.channels()==3?(hasIntensity?rtabmap::LaserScan::kXYI:rtabmap::LaserScan::kXYZ):
			data.channels()==4?rtabmap::LaserScan::kXYZI:
			data.channels()==5?rtabmap::LaserScan::kXYINormal:
			data.channels()==6?rtabmap::LaserScan::kXYZNormal:
			rtabmap::LaserScan::kXYZINormal,
			localTransform);
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
	_strategy(Parameters::defaultIcpStrategy()),
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

	bool pointToPlane = _pointToPlane;

#ifndef RTABMAP_POINTMATCHER
	if(_libpointmatcher)
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
		UWARN("Parameter %s is set to true but RTAB-Map has not been built with CCCoreLib support. Setting to 0.", Parameters::kIcpStrategy().c_str());
		_strategy = 0;
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
			Transform icpT;
			bool hasConverged = false;
			float correspondencesRatio = 0.0f;
			int correspondences = 0;
			double variance = 1.0;
			bool transformComputed = false;
			bool tooLowComplexityForPlaneToPlane = false;
			float secondEigenValue = 1.0f;
			cv::Mat complexityVectors;

			if( pointToPlane &&
				fromScan.hasNormals() &&
				toScan.hasNormals() &&
				!((fromScan.is2d() || toScan.is2d()) && _strategy==0)) // PCL crashes if 2D)
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
								correspondences);
					}
					transformComputed = true;
				}
			}

			if(!transformComputed) // ICP Point to Point
			{
				pcl::PointCloud<pcl::PointXYZI>::Ptr fromCloud = util3d::laserScanToPointCloudI(fromScan, fromScan.localTransform());
				pcl::PointCloud<pcl::PointXYZI>::Ptr toCloud = util3d::laserScanToPointCloudI(toScan, guess * toScan.localTransform());
				UDEBUG("Conversion time = %f s", timer.ticks());

				pcl::PointCloud<pcl::PointXYZI>::Ptr fromCloudRegistered(new pcl::PointCloud<pcl::PointXYZI>());

				if(pointToPlane && !tooLowComplexityForPlaneToPlane && ((fromScan.is2d() || toScan.is2d()) && _strategy==0))
				{
					UWARN("ICP PointToPlane ignored for 2d scans with PCL registration (some crash issues). Use libpointmatcher (%s) or disable %s to avoid this warning.", Parameters::kIcpStrategy().c_str(), Parameters::kIcpPointToPlane().c_str());
				}

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
						if(pointToPlane)
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
							fromCloudRegistered = util3d::transformPointCloud(fromCloud, icpT);
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
								 &msg);
						fromCloudRegistered = util3d::transformPointCloud(fromCloud, icpT);
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
										toCloud,
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
								toCloud,
								_maxCorrespondenceDistance,
								variance,
								correspondences);
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
						if(	((_strategy == 1 && pointToPlane) || _strategy==2) &&
							_force4DoF &&
							!force3DoF() &&
							!tooLowComplexityForPlaneToPlane)
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
