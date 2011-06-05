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
#include "utilite/ULogger.h"

#include <opencv2/core/core.hpp>
#include <opencv2/core/core_c.h>

namespace rtabmap
{

//Epipolar geometry
void findEpipolesFromF(const cv::Mat & fundamentalMatrix, cv::Vec3d & e1, cv::Vec3d & e2)
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

	CvMat * w = cvCreateMat(3, 3, CV_64FC1);
	CvMat * u = cvCreateMat(3, 3, CV_64FC1);
	CvMat * v = cvCreateMat(3, 3, CV_64FC1);

	CvMat f = fundamentalMatrix;
	cvSVD(&f, w, u, v);

	// v is for image 1
	// u is for image 2

	e1[0] = v->data.db[0*3+2];// /v->data.db[2*3+2];
	e1[1] = v->data.db[1*3+2];// /v->data.db[2*3+2];
	e1[2] = v->data.db[2*3+2];// /v->data.db[2*3+2];

	e2[0] = u->data.db[0*3+2];// /u->data.db[2*3+2];
	e2[1] = u->data.db[1*3+2];// /u->data.db[2*3+2];
	e2[2] = u->data.db[2*3+2];// /u->data.db[2*3+2];

	cvReleaseMat(&w);
	cvReleaseMat(&u);
	cvReleaseMat(&v);
}

// P2 = [M | t] = [[e']_x * F | e']
void findPFromF(const cv::Mat & fundamentalMatrix, cv::Mat & p2, cv::Vec3d e2)
{
	if(p2.rows != 3 || p2.cols != 4 || fundamentalMatrix.rows != 3 || fundamentalMatrix.cols != 3)
	{
		ULOGGER_ERROR("Matrices are not the good size... ");
		return;
	}

	if(p2.type()!= CV_64FC1 || fundamentalMatrix.type() != CV_64FC1)
	{
		ULOGGER_ERROR("Matrices are not the good type...");
		return;
	}

	if(e2[0] == 0 && e2[1] == 0 && e2[2] == 0)
	{
		cv::Vec3d e1;
		findEpipolesFromF(fundamentalMatrix, e1, e2);
	}

	double e2_sd[3*3] = { 0., -e2[2], e2[1],
						   e2[2], 0., -e2[0],
						  -e2[1], e2[0], 0. };
	CvMat e2_smt = cvMat( 3, 3, CV_64FC1, e2_sd );
	cv::Mat e2_sm(&e2_smt); //;

	cv::Mat m = e2_sm*fundamentalMatrix;

	p2.at<double>(0,0) = m.at<double>(0,0);
	p2.at<double>(0,1) = m.at<double>(0,1);
	p2.at<double>(0,2) = m.at<double>(0,2);
	p2.at<double>(1,0) = m.at<double>(1,0);
	p2.at<double>(1,1) = m.at<double>(1,1);
	p2.at<double>(1,2) = m.at<double>(1,2);
	p2.at<double>(2,0) = m.at<double>(2,0);
	p2.at<double>(2,1) = m.at<double>(2,1);
	p2.at<double>(2,2) = m.at<double>(2,2);

	p2.at<double>(0,3) = e2[0];
	p2.at<double>(1,3) = e2[1];
	p2.at<double>(2,3) = e2[2];
}

} // namespace rtabmap
