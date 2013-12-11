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

#include "rtabmap/core/EpipolarGeometry.h"
#include "rtabmap/core/Signature.h"
#include "rtabmap/utilite/ULogger.h"
#include "rtabmap/utilite/UTimer.h"
#include "rtabmap/utilite/UStl.h"
#include "rtabmap/utilite/UMath.h"

#include <opencv2/core/core.hpp>
#include <opencv2/core/core_c.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <iostream>

namespace rtabmap
{

/////////////////////////
// HypVerificatorEpipolarGeo
/////////////////////////
EpipolarGeometry::EpipolarGeometry(const ParametersMap & parameters) :
	_matchCountMinAccepted(Parameters::defaultVhEpMatchCountMin()),
	_ransacParam1(Parameters::defaultVhEpRansacParam1()),
	_ransacParam2(Parameters::defaultVhEpRansacParam2())
{
	this->parseParameters(parameters);
}

EpipolarGeometry::~EpipolarGeometry() {

}

void EpipolarGeometry::parseParameters(const ParametersMap & parameters)
{
	ParametersMap::const_iterator iter;
	if((iter=parameters.find(Parameters::kVhEpMatchCountMin())) != parameters.end())
	{
		_matchCountMinAccepted = std::atoi((*iter).second.c_str());
	}
	if((iter=parameters.find(Parameters::kVhEpRansacParam1())) != parameters.end())
	{
		_ransacParam1 = std::atof((*iter).second.c_str());
	}
	if((iter=parameters.find(Parameters::kVhEpRansacParam2())) != parameters.end())
	{
		_ransacParam2 = std::atof((*iter).second.c_str());
	}
}

bool EpipolarGeometry::check(const Signature * ssA, const Signature * ssB)
{
	if(ssA == 0 || ssB == 0)
	{
		return false;
	}
	ULOGGER_DEBUG("id(%d,%d)", ssA->id(), ssB->id());

	std::list<std::pair<int, std::pair<cv::KeyPoint, cv::KeyPoint> > > pairs;

	findPairsUnique(ssA->getWords(), ssB->getWords(), pairs);

	if((int)pairs.size()<_matchCountMinAccepted)
	{
		return false;
	}

	std::vector<uchar> status;
	cv::Mat f = findFFromWords(pairs, status, _ransacParam1, _ransacParam2);

	int inliers = uSum(status);
	if(inliers < _matchCountMinAccepted)
	{
		ULOGGER_DEBUG("Epipolar constraint failed A : not enough inliers (%d/%d), min is %d", inliers, pairs.size(), _matchCountMinAccepted);
		return false;
	}
	else
	{
		UDEBUG("inliers = %d/%d", inliers, pairs.size());
		return true;
	}
}

//STATIC STUFF
//Epipolar geometry
void EpipolarGeometry::findEpipolesFromF(const cv::Mat & fundamentalMatrix, cv::Vec3d & e1, cv::Vec3d & e2)
{
	if(fundamentalMatrix.rows != 3 || fundamentalMatrix.cols != 3)
	{
		ULOGGER_ERROR("The matrix is not the good size...");
		return;
	}

	if(fundamentalMatrix.type() != CV_64FC1)
	{
		ULOGGER_ERROR("The matrix is not the good type...");
		return;
	}


	cv::SVD svd(fundamentalMatrix);
	cv::Mat u = svd.u;
	cv::Mat v = svd.vt;
	cv::Mat w = svd.w;

	// v is for image 1
	// u is for image 2

	e1[0] = v.at<double>(0,2);// /v->data.db[2*3+2];
	e1[1] = v.at<double>(1,2);// /v->data.db[2*3+2];
	e1[2] = v.at<double>(2,2);// /v->data.db[2*3+2];

	e2[0] = u.at<double>(0,2);// /u->data.db[2*3+2];
	e2[1] = u.at<double>(1,2);// /u->data.db[2*3+2];
	e2[2] = u.at<double>(2,2);// /u->data.db[2*3+2];
}

//Assuming P0 = [eye(3) zeros(3,1)]
// x1 and x2 are 2D points
// return camera matrix P (3x4) matrix
cv::Mat EpipolarGeometry::findPFromF(const cv::Mat & fundamentalMatrix, const cv::Mat & x1, const cv::Mat & x2)
{

	if(fundamentalMatrix.rows != 3 || fundamentalMatrix.cols != 3)
	{
		ULOGGER_ERROR("Matrices are not the good size... ");
		return cv::Mat();
	}

	if(fundamentalMatrix.type() != CV_64FC1)
	{
		ULOGGER_ERROR("Matrices are not the good type...");
		return cv::Mat();
	}

	// P matrix 3x4
	cv::Mat p = cv::Mat::zeros(3, 4, CV_64FC1);

	// P0 matrix 3X4
	cv::Mat p0 = cv::Mat::zeros(3, 4, CV_64FC1);
	p0.at<double>(0,0) = 1;
	p0.at<double>(1,1) = 1;
	p0.at<double>(2,2) = 1;

	// cv::SVD doesn't five same results as cvSVD ?!? cvSVD return same values as in MatLab
	/*cv::SVD svd(fundamentalMatrix);
	cv::Mat u = svd.u;
	cv::Mat v = svd.vt;
	cv::Mat s = svd.w;
	cv::Mat e = u.col(2);*/

	CvMat F  = fundamentalMatrix;
	cv::Mat u(3,3,CV_64F);
	cv::Mat v(3,3,CV_64F);
	cv::Mat s(3,3,CV_64F);
	CvMat U  = u;
	CvMat S  = s;
	CvMat V  = v;
	cvSVD(&F, &S, &U, &V, CV_SVD_U_T|CV_SVD_V_T); // F = U D V^T
	u = u.t();
	//
	// INFO: may be required to multiply by -1 the last column of U
	// TODO: Is any way to detect when it is required to do that ? When
	//       it is wrong, triangulated points have their Z value below 1 (between 0 and 1)...
	//
	/*u.at<double>(0,2) = -u.at<double>(0,2);
	u.at<double>(1,2) = -u.at<double>(1,2);
	u.at<double>(2,2) = -u.at<double>(2,2);*/
	v = v.t();
	cv::Mat e = u.col(2);

	//std::cout << "u=" << u << std::endl;
	//std::cout << "v=" << v << std::endl;
	//std::cout << "s=" << s << std::endl;

	// skew matrix 3X3
	cv::Mat skew = cv::Mat::zeros( 3, 3, CV_64FC1);
	skew.at<double>(0,1) = -1;
	skew.at<double>(1,0) = 1;
	skew.at<double>(2,2) = 1;

	cv::Mat r;
	cv::Mat x4d;

	cv::Mat x = x1.col(0); // just take one point
	cv::Mat xp = x2.col(0); // just take one point

	// INFO: There 4 cases of P, only one have the points in
	// front of the two cameras (positive z).

	// Case 1 : P = [U*W*V' e];
	r = u*skew*v.t();
	p.at<double>(0,0) = r.at<double>(0,0);
	p.at<double>(0,1) = r.at<double>(0,1);
	p.at<double>(0,2) = r.at<double>(0,2);
	p.at<double>(1,0) = r.at<double>(1,0);
	p.at<double>(1,1) = r.at<double>(1,1);
	p.at<double>(1,2) = r.at<double>(1,2);
	p.at<double>(2,0) = r.at<double>(2,0);
	p.at<double>(2,1) = r.at<double>(2,1);
	p.at<double>(2,2) = r.at<double>(2,2);
	p.at<double>(0,3) = e.at<double>(0,0);
	p.at<double>(1,3) = e.at<double>(1,0);
	p.at<double>(2,3) = e.at<double>(2,0);

	cv::triangulatePoints(p0, p, x, xp, x4d);
	x4d.at<double>(0) = x4d.at<double>(0)/x4d.at<double>(3);
	x4d.at<double>(1) = x4d.at<double>(1)/x4d.at<double>(3);
	x4d.at<double>(2) = x4d.at<double>(2)/x4d.at<double>(3);
	x4d.at<double>(3) = x4d.at<double>(3)/x4d.at<double>(3);

	cv::Mat xt1 = p0*x4d;
	cv::Mat xt2 = p*x4d;

	if(xt1.at<double>(2,0) < 0 || xt2.at<double>(2,0) < 0)
	{
		// Case 2 : P = [U*W*V' -e];
		p.at<double>(0,3) = -e.at<double>(0,0);
		p.at<double>(1,3) = -e.at<double>(1,0);
		p.at<double>(2,3) = -e.at<double>(2,0);
		cv::triangulatePoints(p0, p, x, xp, x4d);
		x4d.at<double>(0) = x4d.at<double>(0)/x4d.at<double>(3);
		x4d.at<double>(1) = x4d.at<double>(1)/x4d.at<double>(3);
		x4d.at<double>(2) = x4d.at<double>(2)/x4d.at<double>(3);
		x4d.at<double>(3) = x4d.at<double>(3)/x4d.at<double>(3);
		xt1 = p0*x4d;
		xt2 = p*x4d;
		if(xt1.at<double>(2,0) < 0 || xt2.at<double>(2,0) < 0)
		{
			// Case 3 : P = [U*W'*V' e];
			r = u*skew.t()*v.t();
			p.at<double>(0,0) = r.at<double>(0,0);
			p.at<double>(0,1) = r.at<double>(0,1);
			p.at<double>(0,2) = r.at<double>(0,2);
			p.at<double>(1,0) = r.at<double>(1,0);
			p.at<double>(1,1) = r.at<double>(1,1);
			p.at<double>(1,2) = r.at<double>(1,2);
			p.at<double>(2,0) = r.at<double>(2,0);
			p.at<double>(2,1) = r.at<double>(2,1);
			p.at<double>(2,2) = r.at<double>(2,2);
			p.at<double>(0,3) = e.at<double>(0,0);
			p.at<double>(1,3) = e.at<double>(1,0);
			p.at<double>(2,3) = e.at<double>(2,0);
			p.col(3) = e;
			cv::triangulatePoints(p0, p, x, xp, x4d);
			x4d.at<double>(0) = x4d.at<double>(0)/x4d.at<double>(3);
			x4d.at<double>(1) = x4d.at<double>(1)/x4d.at<double>(3);
			x4d.at<double>(2) = x4d.at<double>(2)/x4d.at<double>(3);
			x4d.at<double>(3) = x4d.at<double>(3)/x4d.at<double>(3);
			xt1 = p0*x4d;
			xt2 = p*x4d;
			if(xt1.at<double>(2,0) < 0 || xt2.at<double>(2,0) < 0)
			{
				// Case 4 : P = [U*W'*V' -e];
				p.at<double>(0,3) = -e.at<double>(0,0);
				p.at<double>(1,3) = -e.at<double>(1,0);
				p.at<double>(2,3) = -e.at<double>(2,0);
				cv::triangulatePoints(p0, p, x, xp, x4d);
				x4d.at<double>(0) = x4d.at<double>(0)/x4d.at<double>(3);
				x4d.at<double>(1) = x4d.at<double>(1)/x4d.at<double>(3);
				x4d.at<double>(2) = x4d.at<double>(2)/x4d.at<double>(3);
				x4d.at<double>(3) = x4d.at<double>(3)/x4d.at<double>(3);
				xt1 = p0*x4d;
				xt2 = p*x4d;
				UDEBUG("Case 4");
			}
			else
			{
				UDEBUG("Case 3");
			}
		}
		else
		{
			UDEBUG("Case 2");
		}
	}
	else
	{
		UDEBUG("Case 1");
	}
	return p;
}

cv::Mat EpipolarGeometry::findFFromWords(
		const std::list<std::pair<int, std::pair<cv::KeyPoint, cv::KeyPoint> > > & pairs, // id, kpt1, kpt2
		std::vector<uchar> & status,
		double ransacParam1,
		double ransacParam2)
{

	status = std::vector<uchar>(pairs.size(), 0);
	//Convert Keypoints to a structure that OpenCV understands
	//3 dimensions (Homogeneous vectors)
	cv::Mat points1(1, pairs.size(), CV_32FC2);
	cv::Mat points2(1, pairs.size(), CV_32FC2);

	float * points1data = points1.ptr<float>(0);
	float * points2data = points2.ptr<float>(0);

	// Fill the points here ...
	int i=0;
	for(std::list<std::pair<int, std::pair<cv::KeyPoint, cv::KeyPoint> > >::const_iterator iter = pairs.begin();
		iter != pairs.end();
		++iter )
	{
		points1data[i*2] = (*iter).second.first.pt.x;
		points1data[i*2+1] = (*iter).second.first.pt.y;

		points2data[i*2] = (*iter).second.second.pt.x;
		points2data[i*2+1] = (*iter).second.second.pt.y;

		++i;
	}

	UTimer timer;
	timer.start();

	// Find the fundamental matrix
	cv::Mat fundamentalMatrix = cv::findFundamentalMat(
				points1,
				points2,
				status,
				cv::FM_RANSAC,
				ransacParam1,
				ransacParam2);

	ULOGGER_DEBUG("Find fundamental matrix (OpenCV) time = %fs", timer.ticks());

		// Fundamental matrix is valid ?
	bool fundMatFound = false;
	UASSERT(fundamentalMatrix.type() == CV_64FC1);
	if(fundamentalMatrix.cols==3 && fundamentalMatrix.rows==3 &&
	   (fundamentalMatrix.at<double>(0,0) != 0.0 ||
	    fundamentalMatrix.at<double>(0,1) != 0.0 ||
	    fundamentalMatrix.at<double>(0,2) != 0.0 ||
	    fundamentalMatrix.at<double>(1,0) != 0.0 ||
	    fundamentalMatrix.at<double>(1,1) != 0.0 ||
	    fundamentalMatrix.at<double>(1,2) != 0.0 ||
		fundamentalMatrix.at<double>(2,0) != 0.0 ||
		fundamentalMatrix.at<double>(2,1) != 0.0 ||
		fundamentalMatrix.at<double>(2,2) != 0.0) )

	{
		fundMatFound = true;
	}

	ULOGGER_DEBUG("fm_count=%d...", fundMatFound);

	if(fundMatFound)
	{
		// Show the fundamental matrix
		UDEBUG(
			"F = [%f %f %f;%f %f %f;%f %f %f]",
			fundamentalMatrix.ptr<double>(0)[0],
			fundamentalMatrix.ptr<double>(0)[1],
			fundamentalMatrix.ptr<double>(0)[2],
			fundamentalMatrix.ptr<double>(0)[3],
			fundamentalMatrix.ptr<double>(0)[4],
			fundamentalMatrix.ptr<double>(0)[5],
			fundamentalMatrix.ptr<double>(0)[6],
			fundamentalMatrix.ptr<double>(0)[7],
			fundamentalMatrix.ptr<double>(0)[8]);
	}
	return fundamentalMatrix;
}

void EpipolarGeometry::findRTFromP(
		const cv::Mat & p,
		cv::Mat & r,
		cv::Mat & t)
{
	UASSERT(p.cols == 4 && p.rows == 3);
	UDEBUG("");
	r = cv::Mat(p, cv::Range(0,3), cv::Range(0,3));
	UDEBUG("");
	r = -r.inv();
	UDEBUG("r=%d %d, t=%d", r.cols, r.rows, p.col(3).rows);
	t = r*p.col(3);
	UDEBUG("");
}

/**
 * if a=[1 2 3 4 6 6], b=[1 1 2 4 5 6 6], results= [(1,1a) (2,2) (4,4) (6a,6a) (6b,6b)]
 * realPairsCount = 5
 */
int EpipolarGeometry::findPairs(const std::multimap<int, cv::KeyPoint> & wordsA,
		const std::multimap<int, cv::KeyPoint> & wordsB,
		std::list<std::pair<int, std::pair<cv::KeyPoint, cv::KeyPoint> > > & pairs)
{
	const std::list<int> & ids = uUniqueKeys(wordsA);
	std::multimap<int, cv::KeyPoint>::const_iterator iterA;
	std::multimap<int, cv::KeyPoint>::const_iterator iterB;
	pairs.clear();
	int realPairsCount = 0;
	for(std::list<int>::const_iterator i=ids.begin(); i!=ids.end(); ++i)
	{
		iterA = wordsA.find(*i);
		iterB = wordsB.find(*i);
		while(iterA != wordsA.end() && iterB != wordsB.end() && (*iterA).first == (*iterB).first && (*iterA).first == *i)
		{
			pairs.push_back(std::pair<int, std::pair<cv::KeyPoint, cv::KeyPoint> >(*i, std::pair<cv::KeyPoint, cv::KeyPoint>((*iterA).second, (*iterB).second)));
			++iterA;
			++iterB;
			++realPairsCount;
		}
	}
	return realPairsCount;
}

/**
 * if a=[1 2 3 4 6 6], b=[1 1 2 4 5 6 6], results= [(2,2) (4,4)]
 * realPairsCount = 5
 */
int EpipolarGeometry::findPairsUnique(
		const std::multimap<int, cv::KeyPoint> & wordsA,
		const std::multimap<int, cv::KeyPoint> & wordsB,
		std::list<std::pair<int, std::pair<cv::KeyPoint, cv::KeyPoint> > > & pairs)
{
	const std::list<int> & ids = uUniqueKeys(wordsA);
	int realPairsCount = 0;
	pairs.clear();
	for(std::list<int>::const_iterator i=ids.begin(); i!=ids.end(); ++i)
	{
		std::list<cv::KeyPoint> ptsA = uValues(wordsA, *i);
		std::list<cv::KeyPoint> ptsB = uValues(wordsB, *i);
		if(ptsA.size() == 1 && ptsB.size() == 1)
		{
			pairs.push_back(std::pair<int, std::pair<cv::KeyPoint, cv::KeyPoint> >(*i, std::pair<cv::KeyPoint, cv::KeyPoint>(ptsA.front(), ptsB.front())));
			++realPairsCount;
		}
		else if(ptsA.size()>1 && ptsB.size()>1)
		{
			// just update the count
			realPairsCount += ptsA.size() > ptsB.size() ? ptsB.size() : ptsA.size();
		}
	}
	return realPairsCount;
}

/**
 * if a=[1 2 3 4 6 6], b=[1 1 2 4 5 6 6], results= [(1,1a) (1,1b) (2,2) (4,4) (6a,6a) (6a,6b) (6b,6a) (6b,6b)]
 * realPairsCount = 5
 */
int EpipolarGeometry::findPairsAll(const std::multimap<int, cv::KeyPoint> & wordsA,
		const std::multimap<int, cv::KeyPoint> & wordsB,
		std::list<std::pair<int, std::pair<cv::KeyPoint, cv::KeyPoint> > > & pairs)
{
	UTimer timer;
	timer.start();
	const std::list<int> & ids = uUniqueKeys(wordsA);
	pairs.clear();
	int realPairsCount = 0;;
	for(std::list<int>::const_iterator iter=ids.begin(); iter!=ids.end(); ++iter)
	{
		std::list<cv::KeyPoint> ptsA = uValues(wordsA, *iter);
		std::list<cv::KeyPoint> ptsB = uValues(wordsB, *iter);

		realPairsCount += ptsA.size() > ptsB.size() ? ptsB.size() : ptsA.size();

		for(std::list<cv::KeyPoint>::iterator jter=ptsA.begin(); jter!=ptsA.end(); ++jter)
		{
			for(std::list<cv::KeyPoint>::iterator kter=ptsB.begin(); kter!=ptsB.end(); ++kter)
			{
				pairs.push_back(std::pair<int, std::pair<cv::KeyPoint, cv::KeyPoint> >(*iter, std::pair<cv::KeyPoint, cv::KeyPoint>(*jter, *kter)));
			}
		}
	}
	ULOGGER_DEBUG("time = %f", timer.ticks());
	return realPairsCount;
}

} // namespace rtabmap
