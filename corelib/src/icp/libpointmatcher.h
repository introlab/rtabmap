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
#ifndef CORELIB_SRC_ICP_LIBPOINTMATCHER_H_
#define CORELIB_SRC_ICP_LIBPOINTMATCHER_H_

#include <fstream>
#include "pointmatcher/PointMatcher.h"
#include "nabo/nabo.h"
typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;

namespace rtabmap {

DP pclToDP(const pcl::PointCloud<pcl::PointXYZI>::Ptr & pclCloud, bool is2D)
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
	featLabels.push_back(Label("x", 1));
	featLabels.push_back(Label("y", 1));
	if(!is2D)
	{
		featLabels.push_back(Label("z", 1));
	}
	featLabels.push_back(Label("pad", 1));

	descLabels.push_back(Label("intensity", 1));

	// create cloud
	DP cloud(featLabels, descLabels, pclCloud->size());
	cloud.getFeatureViewByName("pad").setConstant(1);

	// fill cloud
	View view(cloud.getFeatureViewByName("x"));
	View viewIntensity(cloud.getDescriptorRowViewByName("intensity",0));
	for(unsigned int i=0; i<pclCloud->size(); ++i)
	{
		view(0, i) = pclCloud->at(i).x;
		view(1, i) = pclCloud->at(i).y;
		if(!is2D)
		{
			view(2, i) = pclCloud->at(i).z;
		}
		viewIntensity(0, i) = pclCloud->at(i).intensity;
	}

	return cloud;
}

DP pclToDP(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr & pclCloud, bool is2D)
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
	featLabels.push_back(Label("x", 1));
	featLabels.push_back(Label("y", 1));
	if(!is2D)
	{
		featLabels.push_back(Label("z", 1));
	}
	featLabels.push_back(Label("pad", 1));

	descLabels.push_back(Label("normals", 3));
	descLabels.push_back(Label("intensity", 1));

	// create cloud
	DP cloud(featLabels, descLabels, pclCloud->size());
	cloud.getFeatureViewByName("pad").setConstant(1);

	// fill cloud
	View view(cloud.getFeatureViewByName("x"));
	View viewNormalX(cloud.getDescriptorRowViewByName("normals",0));
	View viewNormalY(cloud.getDescriptorRowViewByName("normals",1));
	View viewNormalZ(cloud.getDescriptorRowViewByName("normals",2));
	View viewIntensity(cloud.getDescriptorRowViewByName("intensity",0));
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
		viewIntensity(0, i) = pclCloud->at(i).intensity;
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

void pclFromDP(const DP & cloud, pcl::PointCloud<pcl::PointXYZI> & pclCloud)
{
	UDEBUG("");
	typedef DP::ConstView ConstView;

	if (cloud.features.cols() == 0)
		return;

	pclCloud.resize(cloud.features.cols());
	pclCloud.is_dense = true;

	bool hasIntensity = cloud.descriptorExists("intensity");

		// fill cloud
	ConstView view(cloud.getFeatureViewByName("x"));
	ConstView viewIntensity(hasIntensity?cloud.getDescriptorRowViewByName("intensity",0):view);
	bool is3D = cloud.featureExists("z");
	for(unsigned int i=0; i<pclCloud.size(); ++i)
	{
		pclCloud.at(i).x = view(0, i);
		pclCloud.at(i).y = view(1, i);
		pclCloud.at(i).z = is3D?view(2, i):0;
		if(hasIntensity)
			pclCloud.at(i).intensity = viewIntensity(0, i);
	}
}

void pclFromDP(const DP & cloud, pcl::PointCloud<pcl::PointXYZINormal> & pclCloud)
{
	UDEBUG("");
	typedef DP::ConstView ConstView;

	if (cloud.features.cols() == 0)
		return;

	pclCloud.resize(cloud.features.cols());
	pclCloud.is_dense = true;

	bool hasIntensity = cloud.descriptorExists("intensity");

	// fill cloud
	ConstView view(cloud.getFeatureViewByName("x"));
	bool is3D = cloud.featureExists("z");
	ConstView viewNormalX(cloud.getDescriptorRowViewByName("normals",0));
	ConstView viewNormalY(cloud.getDescriptorRowViewByName("normals",1));
	ConstView viewNormalZ(cloud.getDescriptorRowViewByName("normals",2));
	ConstView viewIntensity(hasIntensity?cloud.getDescriptorRowViewByName("intensity",0):view);
	for(unsigned int i=0; i<pclCloud.size(); ++i)
	{
		pclCloud.at(i).x = view(0, i);
		pclCloud.at(i).y = view(1, i);
		pclCloud.at(i).z = is3D?view(2, i):0;
		pclCloud.at(i).normal_x = viewNormalX(0, i);
		pclCloud.at(i).normal_y = viewNormalY(0, i);
		pclCloud.at(i).normal_z = viewNormalZ(0, i);
		if(hasIntensity)
			pclCloud.at(i).intensity = viewIntensity(0, i);
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

} // namespace rtabmap

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
				bool minDistFound = false;
				for(int k=0; k<knn && k<filteredReferenceIntensity.rows(); ++k)
				{
					int matchesIdsCoeff = matches.ids.coeff(k, i);
					if (matchesIdsCoeff!=-1)
					{
						float distIntensity = fabs(filteredReadingIntensity(0,i) - filteredReferenceIntensity(0, matchesIdsCoeff));
						if(distIntensity < minDistance)
						{
							matchesOrderedByIntensity.ids.coeffRef(0, i) = matches.ids.coeff(k, i);
							matchesOrderedByIntensity.dists.coeffRef(0, i) = matches.dists.coeff(k, i);
							minDistance = distIntensity;
							minDistFound = true;
						}
					}
				}

				if (!minDistFound)
				{
					matchesOrderedByIntensity.ids.coeffRef(0, i) = matches.ids.coeff(0, i);
					matchesOrderedByIntensity.dists.coeffRef(0, i) = matches.dists.coeff(0, i);
				}
			}
			matches = matchesOrderedByIntensity;
		}
		return matches;
	}
};


#endif /* CORELIB_SRC_ICP_LIBPOINTMATCHER_H_ */
