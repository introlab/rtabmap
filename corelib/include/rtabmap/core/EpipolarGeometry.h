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

#include "rtabmap/core/RtabmapExp.h" // DLL export/import defines
#include "rtabmap/core/Parameters.h"
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <list>
#include <vector>

namespace rtabmap
{

class Signature;

class RTABMAP_EXP EpipolarGeometry
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
			double ransacParam1 = 3.0,
			double ransacParam2 = 0.99);

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
	static int findPairs(
			const std::map<int, cv::KeyPoint> & wordsA,
			const std::map<int, cv::KeyPoint> & wordsB,
			std::list<std::pair<int, std::pair<cv::KeyPoint, cv::KeyPoint> > > & pairs,
			bool ignoreNegativeIds = true);

	/**
	 * if a=[1 2 3 4 6 6], b=[1 1 2 4 5 6 6], results= [(1,1a) (2,2) (4,4) (6a,6a) (6b,6b)]
	 * realPairsCount = 5
	 */
	static int findPairs(
			const std::multimap<int, cv::KeyPoint> & wordsA,
			const std::multimap<int, cv::KeyPoint> & wordsB,
			std::list<std::pair<int, std::pair<cv::KeyPoint, cv::KeyPoint> > > & pairs,
			bool ignoreNegativeIds = true);

	/**
	 * if a=[1 2 3 4 6 6], b=[1 1 2 4 5 6 6], results= [(2,2) (4,4)]
	 * realPairsCount = 5
	 */
	static int findPairsUnique(
			const std::multimap<int, cv::KeyPoint> & wordsA,
			const std::multimap<int, cv::KeyPoint> & wordsB,
			std::list<std::pair<int, std::pair<cv::KeyPoint, cv::KeyPoint> > > & pairs,
			bool ignoreNegativeIds = true);

	/**
	 * if a=[1 2 3 4 6 6], b=[1 1 2 4 5 6 6], results= [(1,1a) (1,1b) (2,2) (4,4) (6a,6a) (6a,6b) (6b,6a) (6b,6b)]
	 * realPairsCount = 5
	 */
	static int findPairsAll(
			const std::multimap<int, cv::KeyPoint> & wordsA,
			const std::multimap<int, cv::KeyPoint> & wordsB,
			std::list<std::pair<int, std::pair<cv::KeyPoint, cv::KeyPoint> > > & pairs,
			bool ignoreNegativeIds = true);

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
