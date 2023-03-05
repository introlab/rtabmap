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

#pragma once

#include "rtabmap/core/rtabmap_core_export.h" // DLL export/import defines
#include "rtabmap/core/Parameters.h"
#include "rtabmap/utilite/UStl.h"
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <list>
#include <vector>

namespace rtabmap
{

class Signature;

class RTABMAP_CORE_EXPORT EpipolarGeometry
{
public:
	EpipolarGeometry(const ParametersMap & parameters = ParametersMap());
	virtual ~EpipolarGeometry();
	bool check(const Signature * ssA, const Signature * ssB);
	void parseParameters(const ParametersMap & parameters);

	int getMatchCountMinAccepted() const {return _matchCountMinAccepted;}
	double getRansacParam1() const {return _ransacParam1;}
	double getRansacParam2() const {return _ransacParam2;}

	void setMatchCountMinAccepted(int matchCountMinAccepted) {_matchCountMinAccepted = matchCountMinAccepted;}
	void setRansacParam1(double ransacParam1) {_ransacParam1 = ransacParam1;}
	void setRansacParam2(double ransacParam2) {_ransacParam2 = ransacParam2;}


	// STATIC STUFF
	//epipolar geometry
	static void findEpipolesFromF(
			const cv::Mat & fundamentalMatrix,
			cv::Vec3d & e1,
			cv::Vec3d & e2);

	static cv::Mat findPFromE(
			const cv::Mat & E,
			const cv::Mat & x,
			const cv::Mat & xp);

	// return fundamental matrix
	// status -> inliers = 1, outliers = 0
	static cv::Mat findFFromWords(
			const std::list<std::pair<int, std::pair<cv::KeyPoint, cv::KeyPoint> > > & pairs, // id, kpt1, kpt2
			std::vector<uchar> & status,
			double ransacReprojThreshold = 3.0,
			double ransacConfidence = 0.99);

	// assume a canonical camera (without K)
	static void findRTFromP(
			const cv::Mat & p,
			cv::Mat & r,
			cv::Mat & t);

	static cv::Mat findFFromCalibratedStereoCameras(double fx, double fy, double cx, double cy, double Tx, double Ty);


	/**
	 * if a=[1 2 3 4 6], b=[1 2 4 5 6], results= [(1,1) (2,2) (4,4) (6,6)]
	 * realPairsCount = 4
	 */
	template<typename T>
	static int findPairs(
			const std::map<int, T> & wordsA,
			const std::map<int, T> & wordsB,
			std::list<std::pair<int, std::pair<T, T> > > & pairs,
			bool ignoreNegativeIds = true)
	{
		int realPairsCount = 0;
		pairs.clear();
		for(typename std::map<int, T>::const_iterator i=wordsA.begin(); i!=wordsA.end(); ++i)
		{
			if(!ignoreNegativeIds || (ignoreNegativeIds && i->first>=0))
			{
				std::map<int, cv::KeyPoint>::const_iterator ptB = wordsB.find(i->first);
				if(ptB != wordsB.end())
				{
					pairs.push_back(std::pair<int, std::pair<T, T> >(i->first, std::make_pair(i->second, ptB->second)));
					++realPairsCount;
				}
			}
		}
		return realPairsCount;
	}

	/**
	 * if a=[1 2 3 4 6 6], b=[1 1 2 4 5 6 6], results= [(1,1a) (2,2) (4,4) (6a,6a) (6b,6b)]
	 * realPairsCount = 5
	 */
	template<typename T>
	static int findPairs(
			const std::multimap<int, T> & wordsA,
			const std::multimap<int, T> & wordsB,
			std::list<std::pair<int, std::pair<T, T> > > & pairs,
			bool ignoreNegativeIds = true)
	{
		const std::list<int> & ids = uUniqueKeys(wordsA);
		typename std::multimap<int, T>::const_iterator iterA;
		typename std::multimap<int, T>::const_iterator iterB;
		pairs.clear();
		int realPairsCount = 0;
		for(std::list<int>::const_iterator i=ids.begin(); i!=ids.end(); ++i)
		{
			if(!ignoreNegativeIds || (ignoreNegativeIds && *i >= 0))
			{
				iterA = wordsA.find(*i);
				iterB = wordsB.find(*i);
				while(iterA != wordsA.end() && iterB != wordsB.end() && (*iterA).first == (*iterB).first && (*iterA).first == *i)
				{
					pairs.push_back(std::pair<int, std::pair<T, T> >(*i, std::make_pair((*iterA).second, (*iterB).second)));
					++iterA;
					++iterB;
					++realPairsCount;
				}
			}
		}
		return realPairsCount;
	}

	/**
	 * if a=[1 2 3 4 6 6], b=[1 1 2 4 5 6 6], results= [(2,2) (4,4)]
	 * realPairsCount = 5
	 */
	template<typename T>
	static int findPairsUnique(
			const std::multimap<int, T> & wordsA,
			const std::multimap<int, T> & wordsB,
			std::list<std::pair<int, std::pair<T, T> > > & pairs,
			bool ignoreNegativeIds = true)
	{
		const std::list<int> & ids = uUniqueKeys(wordsA);
		int realPairsCount = 0;
		pairs.clear();
		for(std::list<int>::const_iterator i=ids.begin(); i!=ids.end(); ++i)
		{
			if(!ignoreNegativeIds || (ignoreNegativeIds && *i>=0))
			{
				std::list<T> ptsA = uValues(wordsA, *i);
				std::list<T> ptsB = uValues(wordsB, *i);
				if(ptsA.size() == 1 && ptsB.size() == 1)
				{
					pairs.push_back(std::pair<int, std::pair<T, T> >(*i, std::pair<T, T>(ptsA.front(), ptsB.front())));
					++realPairsCount;
				}
				else if(ptsA.size()>1 && ptsB.size()>1)
				{
					// just update the count
					realPairsCount += ptsA.size() > ptsB.size() ? ptsB.size() : ptsA.size();
				}
			}
		}
		return realPairsCount;
	}

	/**
	 * if a=[1 2 3 4 6 6], b=[1 1 2 4 5 6 6], results= [(1,1a) (1,1b) (2,2) (4,4) (6a,6a) (6a,6b) (6b,6a) (6b,6b)]
	 * realPairsCount = 5
	 */
	template<typename T>
	static int findPairsAll(
			const std::multimap<int, T> & wordsA,
			const std::multimap<int, T> & wordsB,
			std::list<std::pair<int, std::pair<T, T> > > & pairs,
			bool ignoreNegativeIds = true)
	{
		const std::list<int> & ids = uUniqueKeys(wordsA);
		pairs.clear();
		int realPairsCount = 0;;
		for(std::list<int>::const_iterator iter=ids.begin(); iter!=ids.end(); ++iter)
		{
			if(!ignoreNegativeIds || (ignoreNegativeIds && *iter>=0))
			{
				std::list<T> ptsA = uValues(wordsA, *iter);
				std::list<T> ptsB = uValues(wordsB, *iter);

				realPairsCount += ptsA.size() > ptsB.size() ? ptsB.size() : ptsA.size();

				for(typename std::list<T>::iterator jter=ptsA.begin(); jter!=ptsA.end(); ++jter)
				{
					for(typename std::list<T>::iterator kter=ptsB.begin(); kter!=ptsB.end(); ++kter)
					{
						pairs.push_back(std::pair<int, std::pair<T, T> >(*iter, std::pair<T, T>(*jter, *kter)));
					}
				}
			}
		}
		return realPairsCount;
	}

	static cv::Mat linearLSTriangulation(
			cv::Point3d u,    //homogenous image point (u,v,1)
			cv::Matx34d P,        //camera 1 matrix 3x4 double
			cv::Point3d u1,   //homogenous image point in 2nd camera
			cv::Matx34d P1);       //camera 2 matrix 3x4 double

	static cv::Mat iterativeLinearLSTriangulation(
			cv::Point3d u,        //homogenous image point (u,v,1)
			const cv::Matx34d & P,    //camera 1 matrix 3x4 double
			cv::Point3d u1,        //homogenous image point in 2nd camera
			const cv::Matx34d & P1);  //camera 2 matrix 3x4 double

	static double triangulatePoints(
			const cv::Mat& pt_set1, //2xN double
			const cv::Mat& pt_set2, //2xN double
			const cv::Mat& P, // 3x4 double
			const cv::Mat& P1, // 3x4 double
			pcl::PointCloud<pcl::PointXYZ>::Ptr & pointcloud,
			std::vector<double> & reproj_errors);

private:
	int _matchCountMinAccepted;
	double _ransacParam1;
	double _ransacParam2;
};

} // namespace rtabmap
