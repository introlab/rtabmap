/*
Copyright (c) 2010-2014, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
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

#include "rtabmap/core/util3d_motion_estimation.h"

#include "rtabmap/utilite/UStl.h"
#include "rtabmap/utilite/UMath.h"
#include "rtabmap/core/util3d_transforms.h"
#include "rtabmap/core/util3d_registration.h"
#include "rtabmap/core/util3d_correspondences.h"

namespace rtabmap
{

namespace util3d
{

Transform estimateMotion3DTo2D(
			const std::multimap<int, pcl::PointXYZ> & words3A,
			const std::multimap<int, cv::KeyPoint> & words2B,
			const CameraModel & cameraModel,
			int minInliers,
			int iterations,
			double reprojError,
			int flagsPnP,
			const Transform & guess,
			const std::multimap<int, pcl::PointXYZ> & words3B,
			double * varianceOut,
			std::vector<int> * matchesOut,
			std::vector<int> * inliersOut)
{

	Transform transform;
	std::vector<int> matches, inliers;

	if(varianceOut)
	{
		*varianceOut = 1.0;
	}

	// find correspondences
	std::vector<int> ids = uListToVector(uUniqueKeys(words2B));
	std::vector<cv::Point3f> objectPoints(ids.size());
	std::vector<cv::Point2f> imagePoints(ids.size());
	int oi=0;
	matches.resize(ids.size());
	for(unsigned int i=0; i<ids.size(); ++i)
	{
		if(words3A.count(ids[i]) == 1)
		{
			pcl::PointXYZ pt = words3A.find(ids[i])->second;
			objectPoints[oi].x = pt.x;
			objectPoints[oi].y = pt.y;
			objectPoints[oi].z = pt.z;
			imagePoints[oi] = words2B.find(ids[i])->second.pt;
			matches[oi++] = ids[i];
		}
	}

	objectPoints.resize(oi);
	imagePoints.resize(oi);
	matches.resize(oi);

	if((int)matches.size() >= minInliers)
	{
		//PnPRansac
		cv::Mat K = cameraModel.K();
		Transform guessCameraFrame = (guess * cameraModel.localTransform()).inverse();
		cv::Mat R = (cv::Mat_<double>(3,3) <<
				(double)guessCameraFrame.r11(), (double)guessCameraFrame.r12(), (double)guessCameraFrame.r13(),
				(double)guessCameraFrame.r21(), (double)guessCameraFrame.r22(), (double)guessCameraFrame.r23(),
				(double)guessCameraFrame.r31(), (double)guessCameraFrame.r32(), (double)guessCameraFrame.r33());

		cv::Mat rvec(1,3, CV_64FC1);
		cv::Rodrigues(R, rvec);
		cv::Mat tvec = (cv::Mat_<double>(1,3) <<
				(double)guessCameraFrame.x(), (double)guessCameraFrame.y(), (double)guessCameraFrame.z());

		cv::solvePnPRansac(
				objectPoints,
				imagePoints,
				K,
				cv::Mat(),
				rvec,
				tvec,
				true,
				iterations,
				reprojError,
				0,
				inliers,
				flagsPnP);

		if((int)inliers.size() >= minInliers)
		{
			cv::Rodrigues(rvec, R);
			Transform pnp(R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2), tvec.at<double>(0),
						   R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2), tvec.at<double>(1),
						   R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2), tvec.at<double>(2));

			transform = (cameraModel.localTransform() * pnp).inverse();

			// compute variance (like in PCL computeVariance() method of sac_model.h)
			if(varianceOut && words3B.size())
			{
				std::vector<float> errorSqrdDists(inliers.size());
				oi = 0;
				for(unsigned int i=0; i<inliers.size(); ++i)
				{
					std::multimap<int, pcl::PointXYZ>::const_iterator iter = words3B.find(matches[inliers[i]]);
					if(iter != words3B.end() && pcl::isFinite(iter->second))
					{
						const cv::Point3f & objPt = objectPoints[inliers[i]];
						pcl::PointXYZ newPt = util3d::transformPoint(iter->second, transform);
						errorSqrdDists[oi++] = uNormSquared(objPt.x-newPt.x, objPt.y-newPt.y, objPt.z-newPt.z);
					}
				}
				errorSqrdDists.resize(oi);
				if(errorSqrdDists.size())
				{
					std::sort(errorSqrdDists.begin(), errorSqrdDists.end());
					double median_error_sqr = (double)errorSqrdDists[errorSqrdDists.size () >> 1];
					*varianceOut = 2.1981 * median_error_sqr;
				}
			}
		}
	}

	if(matchesOut)
	{
		*matchesOut = matches;
	}
	if(inliersOut)
	{
		inliersOut->resize(inliers.size());
		for(unsigned int i=0; i<inliers.size(); ++i)
		{
			inliersOut->at(i) = matches[inliers[i]];
		}
	}

	return transform;
}

Transform estimateMotion3DTo3D(
			const std::multimap<int, pcl::PointXYZ> & words3A,
			const std::multimap<int, pcl::PointXYZ> & words3B,
			int minInliers,
			double inliersDistance,
			int iterations,
			int refineIterations,
			double * varianceOut,
			std::vector<int> * matchesOut,
			std::vector<int> * inliersOut)
{
	Transform transform;
	pcl::PointCloud<pcl::PointXYZ>::Ptr inliers1(new pcl::PointCloud<pcl::PointXYZ>); // previous
	pcl::PointCloud<pcl::PointXYZ>::Ptr inliers2(new pcl::PointCloud<pcl::PointXYZ>); // new

	std::vector<int> matches;
	util3d::findCorrespondences(
			words3A,
			words3B,
			*inliers1,
			*inliers2,
			0,
			&matches);

	if(varianceOut)
	{
		*varianceOut = 1.0;
	}

	if((int)inliers1->size() >= minInliers)
	{
		std::vector<int> inliers;
		Transform t = util3d::transformFromXYZCorrespondences(
				inliers2,
				inliers1,
				inliersDistance,
				iterations,
				refineIterations>0,
				3.0,
				refineIterations,
				&inliers,
				varianceOut);

		if(!t.isNull() && (int)inliers.size() >= minInliers)
		{
			transform = t;
		}

		if(matchesOut)
		{
			*matchesOut = matches;
		}

		if(inliersOut)
		{
			inliersOut->resize(inliers.size());
			for(unsigned int i=0; i<inliers.size(); ++i)
			{
				inliersOut->at(i) = matches[inliers[i]];
			}
		}
	}
	return transform;
}

}

}
