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
	Parameters::parse(parameters, Parameters::kVhEpMatchCountMin(), _matchCountMinAccepted);
	Parameters::parse(parameters, Parameters::kVhEpRansacParam1(), _ransacParam1);
	Parameters::parse(parameters, Parameters::kVhEpRansacParam2(), _ransacParam2);
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

int inFrontOfBothCameras(const cv::Mat & x, const cv::Mat & xp, const cv::Mat & R, const cv::Mat & T)
{
	// P0 matrix 3X4
	cv::Mat p0 = cv::Mat::zeros(3, 4, CV_64FC1);
	p0.at<double>(0,0) = 1;
	p0.at<double>(1,1) = 1;
	p0.at<double>(2,2) = 1;
	cv::Mat p = cv::Mat::zeros(3, 4, CV_64FC1);
	p.at<double>(0,0) = R.at<double>(0,0);
	p.at<double>(0,1) = R.at<double>(0,1);
	p.at<double>(0,2) = R.at<double>(0,2);
	p.at<double>(1,0) = R.at<double>(1,0);
	p.at<double>(1,1) = R.at<double>(1,1);
	p.at<double>(1,2) = R.at<double>(1,2);
	p.at<double>(2,0) = R.at<double>(2,0);
	p.at<double>(2,1) = R.at<double>(2,1);
	p.at<double>(2,2) = R.at<double>(2,2);
	p.at<double>(0,3) = T.at<double>(0,0);
	p.at<double>(1,3) = T.at<double>(1,0);
	p.at<double>(2,3) = T.at<double>(2,0);

	cv::Mat pts4D;
	//std::vector<double> reprojErrors;
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
	//EpipolarGeometry::triangulatePoints(x, xp, p0, p, cloud, reprojErrors);
	cv::triangulatePoints(p0, p, x, xp, pts4D);

    //http://en.wikipedia.org/wiki/Essential_matrix#3D_points_from_corresponding_image_points
	int nValid = 0;
    for(int i=0; i<x.cols; ++i)
    {
    	// the five to ignore when all points are super close to the camera
        if(pts4D.at<double>(2,i)/pts4D.at<double>(3,i) > 5)
    	//if(cloud->at(i).z > 5)
        {
        	++nValid;
        }
    }
    UDEBUG("nValid=%d/%d", nValid,  x.cols);

    return nValid;
}

//Assuming P0 = [eye(3) zeros(3,1)]
// x1 and x2 are 2D points
// return camera matrix P (3x4) matrix
//http://www.robots.ox.ac.uk/~vgg/hzbook/hzbook2/HZepipolar.pdf
cv::Mat EpipolarGeometry::findPFromE(const cv::Mat & E,
		const cv::Mat & x,
		const cv::Mat & xp)
{
	UDEBUG("begin");
	UASSERT(E.rows == 3 && E.cols == 3);
	UASSERT(E.type() == CV_64FC1);
	UASSERT(x.rows == 2 && x.cols>0 && x.type() == CV_64FC1);
	UASSERT(xp.rows == 2 && xp.cols>0 && x.type() == CV_64FC1);

	// skew matrix 3X3
	cv::Mat w = cv::Mat::zeros( 3, 3, CV_64FC1);
	w.at<double>(0,1) = -1;
	w.at<double>(1,0) = 1;
	w.at<double>(2,2) = 1;
	//std::cout << "W=" << w << std::endl;

	cv::Mat e = E;
	cv::SVD svd(e,cv::SVD::MODIFY_A);
	cv::Mat u = svd.u;
	cv::Mat vt = svd.vt;
	cv::Mat s = svd.w;

	//std::cout << "u=" << u << std::endl;
	//std::cout << "vt=" << vt << std::endl;
	//std::cout << "s=" << s << std::endl;

	// E = u*diag(1,1,0)*vt
	cv::Mat diag = cv::Mat::eye(3,3,CV_64FC1);
	diag.at<double>(2,2) = 0;
	e = u*diag*vt;
	svd(e,cv::SVD::MODIFY_A);
	u = svd.u;
	vt = svd.vt;
	s = svd.w;

	cv::Mat r = u*w*vt;
	if(cv::determinant(r)+1.0 < 1e-09) {
		//according to http://en.wikipedia.org/wiki/Essential_matrix#Showing_that_it_is_valid
		UDEBUG("det(R) == -1 [%f]: flip E's sign", cv::determinant(r));
		e = -E;
		svd(e,cv::SVD::MODIFY_A);
		u = svd.u;
		vt = svd.vt;
		s = svd.w;
	}
	cv::Mat wt = w.t();

	// INFO: There 4 cases of P, only one have all the points in
	// front of the two cameras (positive z).

	cv::Mat r1 = u*w*vt;
	cv::Mat r2 = u*wt*vt;

	cv::Mat t1 = u.col(2);
	cv::Mat t2 = u.col(2)*-1;

	int max = 0;
	int maxIndex = 1;
	int maxTmp;
	cv::Mat R=r1,T=t1;

	// Case 1 : P = [U*W*V' t];
	max = inFrontOfBothCameras(x, xp, r1, t1);
	// Case 2 : P = [U*W*V' -t];
	maxTmp = inFrontOfBothCameras(x, xp, r1, t2);
	if(maxTmp > max)
	{
		maxIndex = 2;
		max = maxTmp;
		R=r1,T=t2;
	}
	// Case 3 : P = [U*W'*V' t];
	maxTmp = inFrontOfBothCameras(x, xp, r2, t1);
	if(maxTmp > max)
	{
		maxIndex = 3;
		max = maxTmp;
		R=r2,T=t1;
	}
	// Case 4 : P = [U*W'*V' -t];
	maxTmp = inFrontOfBothCameras(x, xp, r2, t2);
	if(maxTmp > max)
	{
		maxIndex = 4;
		max = maxTmp;
		R=r2,T=t2;
	}

	if(max > 0)
	{
		UDEBUG("Case %d", maxIndex);

		// P matrix 3x4
		cv::Mat p = cv::Mat::zeros(3, 4, CV_64FC1);
		p.at<double>(0,0) = R.at<double>(0,0);
		p.at<double>(0,1) = R.at<double>(0,1);
		p.at<double>(0,2) = R.at<double>(0,2);
		p.at<double>(1,0) = R.at<double>(1,0);
		p.at<double>(1,1) = R.at<double>(1,1);
		p.at<double>(1,2) = R.at<double>(1,2);
		p.at<double>(2,0) = R.at<double>(2,0);
		p.at<double>(2,1) = R.at<double>(2,1);
		p.at<double>(2,2) = R.at<double>(2,2);
		p.at<double>(0,3) = T.at<double>(0);
		p.at<double>(1,3) = T.at<double>(1);
		p.at<double>(2,3) = T.at<double>(2);
		return p;
	}

	return cv::Mat();
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
	r = cv::Mat(p, cv::Range(0,3), cv::Range(0,3));
	//r = -r.inv();
	//t = r*p.col(3);
	t = p.col(3);
}

cv::Mat EpipolarGeometry::findFFromCalibratedStereoCameras(double fx, double fy, double cx, double cy, double Tx, double Ty)
{
	cv::Mat R = cv::Mat::eye(3, 3, CV_64FC1);

	double Bx = Tx/-fx;
	double By = Ty/-fy;

	cv::Mat tx = (cv::Mat_<double>(3,3) <<
			0, 0, By,
			0, 0, -Bx,
			-By, Bx, 0);

	cv::Mat K = (cv::Mat_<double>(3,3) <<
			fx, 0, cx,
			0, fy, cy,
			0, 0, 1);

	cv::Mat E = tx*R;

	return K.inv().t()*E*K.inv();
}

/**
 * if a=[1 2 3 4 6], b=[1 2 4 5 6], results= [(1,1) (2,2) (4,4) (6,6)]
 * realPairsCount = 4
 */
int EpipolarGeometry::findPairs(
		const std::map<int, cv::KeyPoint> & wordsA,
		const std::map<int, cv::KeyPoint> & wordsB,
		std::list<std::pair<int, std::pair<cv::KeyPoint, cv::KeyPoint> > > & pairs,
		bool ignoreInvalidIds)
{
	int realPairsCount = 0;
	pairs.clear();
	for(std::map<int, cv::KeyPoint>::const_iterator i=wordsA.begin(); i!=wordsA.end(); ++i)
	{
		if(!ignoreInvalidIds || (ignoreInvalidIds && i->first>=0))
		{
			std::map<int, cv::KeyPoint>::const_iterator ptB = wordsB.find(i->first);
			if(ptB != wordsB.end())
			{
				pairs.push_back(std::pair<int, std::pair<cv::KeyPoint, cv::KeyPoint> >(i->first, std::pair<cv::KeyPoint, cv::KeyPoint>(i->second, ptB->second)));
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
int EpipolarGeometry::findPairs(const std::multimap<int, cv::KeyPoint> & wordsA,
		const std::multimap<int, cv::KeyPoint> & wordsB,
		std::list<std::pair<int, std::pair<cv::KeyPoint, cv::KeyPoint> > > & pairs,
		bool ignoreInvalidIds)
{
	const std::list<int> & ids = uUniqueKeys(wordsA);
	std::multimap<int, cv::KeyPoint>::const_iterator iterA;
	std::multimap<int, cv::KeyPoint>::const_iterator iterB;
	pairs.clear();
	int realPairsCount = 0;
	for(std::list<int>::const_iterator i=ids.begin(); i!=ids.end(); ++i)
	{
		if(!ignoreInvalidIds || (ignoreInvalidIds && *i >= 0))
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
		std::list<std::pair<int, std::pair<cv::KeyPoint, cv::KeyPoint> > > & pairs,
		bool ignoreInvalidIds)
{
	const std::list<int> & ids = uUniqueKeys(wordsA);
	int realPairsCount = 0;
	pairs.clear();
	for(std::list<int>::const_iterator i=ids.begin(); i!=ids.end(); ++i)
	{
		if(!ignoreInvalidIds || (ignoreInvalidIds && *i>=0))
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
	}
	return realPairsCount;
}

/**
 * if a=[1 2 3 4 6 6], b=[1 1 2 4 5 6 6], results= [(1,1a) (1,1b) (2,2) (4,4) (6a,6a) (6a,6b) (6b,6a) (6b,6b)]
 * realPairsCount = 5
 */
int EpipolarGeometry::findPairsAll(const std::multimap<int, cv::KeyPoint> & wordsA,
		const std::multimap<int, cv::KeyPoint> & wordsB,
		std::list<std::pair<int, std::pair<cv::KeyPoint, cv::KeyPoint> > > & pairs,
		bool ignoreInvalidIds)
{
	UTimer timer;
	timer.start();
	const std::list<int> & ids = uUniqueKeys(wordsA);
	pairs.clear();
	int realPairsCount = 0;;
	for(std::list<int>::const_iterator iter=ids.begin(); iter!=ids.end(); ++iter)
	{
		if(!ignoreInvalidIds || (ignoreInvalidIds && *iter>=0))
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
	}
	ULOGGER_DEBUG("time = %f", timer.ticks());
	return realPairsCount;
}



/**
 source = SfM toy library: https://github.com/royshil/SfM-Toy-Library
 From "Triangulation", Hartley, R.I. and Sturm, P., Computer vision and image understanding, 1997
 return 1x3 double
 */
cv::Mat EpipolarGeometry::linearLSTriangulation(
		cv::Point3d u,   //homogenous image point (u,v,1)
		cv::Matx34d P,       //camera 1 matrix 3x4 double
		cv::Point3d u1,  //homogenous image point in 2nd camera
		cv::Matx34d P1       //camera 2 matrix 3x4 double
                                   )
{
    //build matrix A for homogenous equation system Ax = 0
    //assume X = (x,y,z,1), for Linear-LS method
    //which turns it into a AX = B system, where A is 4x3, X is 3x1 and B is 4x1
    cv::Mat A = (cv::Mat_<double>(4,3) <<
    		u.x*P(2,0)-P(0,0),    u.x*P(2,1)-P(0,1),      u.x*P(2,2)-P(0,2),
			u.y*P(2,0)-P(1,0),    u.y*P(2,1)-P(1,1),      u.y*P(2,2)-P(1,2),
			u1.x*P1(2,0)-P1(0,0), u1.x*P1(2,1)-P1(0,1),   u1.x*P1(2,2)-P1(0,2),
			u1.y*P1(2,0)-P1(1,0), u1.y*P1(2,1)-P1(1,1),   u1.y*P1(2,2)-P1(1,2)
              );
    cv::Mat B = (cv::Mat_<double>(4,1) <<
			-(u.x*P(2,3)    -P(0,3)),
			-(u.y*P(2,3)  -P(1,3)),
			-(u1.x*P1(2,3)    -P1(0,3)),
			-(u1.y*P1(2,3)    -P1(1,3)));

    cv::Mat X;
    solve(A,B,X,cv::DECOMP_SVD);

    return X; // return 1x3 double
}

 /**
 source = SfM toy library: https://github.com/royshil/SfM-Toy-Library
 From "Triangulation", Hartley, R.I. and Sturm, P., Computer vision and image understanding, 1997
 return 4x1 double
 */
cv::Mat EpipolarGeometry::iterativeLinearLSTriangulation(
		cv::Point3d u,            //homogenous image point (u,v,1)
		const cv::Matx34d & P,   //camera 1 matrix 3x4 double
		cv::Point3d u1,           //homogenous image point in 2nd camera
		const cv::Matx34d & P1)   //camera 2 matrix 3x4 double
{
    double wi = 1, wi1 = 1;
	double EPSILON = 0.0001;

	cv::Mat X(4,1,CV_64FC1);
	cv::Mat X_ = linearLSTriangulation(u,P,u1,P1);
	X.at<double>(0) = X_.at<double>(0);
	X.at<double>(1) = X_.at<double>(1);
	X.at<double>(2) = X_.at<double>(2);
	X.at<double>(3) = 1.0;
	for (int i=0; i<10; i++)  //Hartley suggests 10 iterations at most
	{
        //recalculate weights
    	double p2x = cv::Mat(cv::Mat(P).row(2)*X).at<double>(0);
    	double p2x1 = cv::Mat(cv::Mat(P1).row(2)*X).at<double>(0);

        //breaking point
        if(fabs(wi - p2x) <= EPSILON && fabs(wi1 - p2x1) <= EPSILON) break;

        wi = p2x;
        wi1 = p2x1;

        //reweight equations and solve
        cv::Mat A = (cv::Mat_<double>(4,3) <<
        		(u.x*P(2,0)-P(0,0))/wi,       (u.x*P(2,1)-P(0,1))/wi,         (u.x*P(2,2)-P(0,2))/wi,
				(u.y*P(2,0)-P(1,0))/wi,       (u.y*P(2,1)-P(1,1))/wi,         (u.y*P(2,2)-P(1,2))/wi,
				(u1.x*P1(2,0)-P1(0,0))/wi1,   (u1.x*P1(2,1)-P1(0,1))/wi1,     (u1.x*P1(2,2)-P1(0,2))/wi1,
				(u1.y*P1(2,0)-P1(1,0))/wi1,   (u1.y*P1(2,1)-P1(1,1))/wi1,     (u1.y*P1(2,2)-P1(1,2))/wi1);
        cv::Mat B = (cv::Mat_<double>(4,1) <<
        		-(u.x*P(2,3)    -P(0,3))/wi,
				-(u.y*P(2,3)  -P(1,3))/wi,
				-(u1.x*P1(2,3)    -P1(0,3))/wi1,
				-(u1.y*P1(2,3)    -P1(1,3))/wi1);

        solve(A,B,X_,cv::DECOMP_SVD);
        X.at<double>(0) = X_.at<double>(0);
		X.at<double>(1) = X_.at<double>(1);
		X.at<double>(2) = X_.at<double>(2);
		X.at<double>(3) = 1.0;
    }
    return X; //  return 4x1 double
}

/**
 source = SfM toy library: https://github.com/royshil/SfM-Toy-Library
 */
//Triagulate points
double EpipolarGeometry::triangulatePoints(
		const cv::Mat& pt_set, //2xN double
		const cv::Mat& pt_set1, //2xN double
		const cv::Mat& P, // 3x4 double
		const cv::Mat& P1, // 3x4 double
		pcl::PointCloud<pcl::PointXYZ>::Ptr & pointcloud,
		std::vector<double> & reproj_errors)
{
	pointcloud.reset(new pcl::PointCloud<pcl::PointXYZ>);

	unsigned int pts_size = pt_set.cols;

	pointcloud->resize(pts_size);
	reproj_errors.resize(pts_size);

	for(unsigned int i=0; i<pts_size; i++)
	{
		cv::Point3d u(pt_set.at<double>(0,i),pt_set.at<double>(1,i),1.0);
		cv::Point3d u1(pt_set1.at<double>(0,i),pt_set1.at<double>(1,i),1.0);

		cv::Mat X = iterativeLinearLSTriangulation(u,P,u1,P1);

		cv::Mat x_proj = P * X;				//reproject
		x_proj = x_proj / x_proj.at<double>(2);
		cv::Point3d xPt_img_(x_proj.at<double>(0), x_proj.at<double>(1), 1.0);

		double reprj_err = norm(xPt_img_ - u);
		reproj_errors[i] = reprj_err;
		pointcloud->at(i) = pcl::PointXYZ(X.at<double>(0),X.at<double>(1),X.at<double>(2));
	}

	return cv::mean(reproj_errors)[0]; // mean reproj error
}

} // namespace rtabmap
