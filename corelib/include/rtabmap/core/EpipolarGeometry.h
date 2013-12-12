/*
 * Copyright (C) 2010-2011, Mathieu Labbe and IntRoLab - Universite de Sherbrooke
 *
 * This file is part of RTAB-Map.
 *
 * RTAB-Map is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * RTAB-Map is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with RTAB-Map.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "rtabmap/core/RtabmapExp.h" // DLL export/import defines
#include "rtabmap/core/Parameters.h"
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
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

	static cv::Mat findPFromF(
			const cv::Mat & fundamentalMatrix,
			const cv::Mat & x1,
			const cv::Mat & x2);

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

	/**
	 * if a=[1 2 3 4 6 6], b=[1 1 2 4 5 6 6], results= [(1,1a) (2,2) (4,4) (6a,6a) (6b,6b)]
	 * realPairsCount = 5
	 */
	static int findPairs(
			const std::multimap<int, cv::KeyPoint> & wordsA,
			const std::multimap<int, cv::KeyPoint> & wordsB,
			std::list<std::pair<int, std::pair<cv::KeyPoint, cv::KeyPoint> > > & pairs);

	/**
	 * if a=[1 2 3 4 6 6], b=[1 1 2 4 5 6 6], results= [(2,2) (4,4)]
	 * realPairsCount = 5
	 */
	static int findPairsUnique(
			const std::multimap<int, cv::KeyPoint> & wordsA,
			const std::multimap<int, cv::KeyPoint> & wordsB,
			std::list<std::pair<int, std::pair<cv::KeyPoint, cv::KeyPoint> > > & pairs);

	/**
	 * if a=[1 2 3 4 6 6], b=[1 1 2 4 5 6 6], results= [(1,1a) (1,1b) (2,2) (4,4) (6a,6a) (6a,6b) (6b,6a) (6b,6b)]
	 * realPairsCount = 5
	 */
	static int findPairsAll(
			const std::multimap<int, cv::KeyPoint> & wordsA,
			const std::multimap<int, cv::KeyPoint> & wordsB,
			std::list<std::pair<int, std::pair<cv::KeyPoint, cv::KeyPoint> > > & pairs);

private:
	int _matchCountMinAccepted;
	double _ransacParam1;
	double _ransacParam2;
};

} // namespace rtabmap
