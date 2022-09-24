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

#include "rtabmap/core/util3d_motion_estimation.h"

#include "rtabmap/utilite/UStl.h"
#include "rtabmap/utilite/UMath.h"
#include "rtabmap/utilite/ULogger.h"
#include "rtabmap/core/util3d_transforms.h"
#include "rtabmap/core/util3d_registration.h"
#include "rtabmap/core/util3d_correspondences.h"
#include "rtabmap/core/util3d.h"

#include <pcl/common/common.h>

#include "opencv/solvepnp.h"

#ifdef RTABMAP_OPENGV
#include <opengv/absolute_pose/methods.hpp>
#include <opengv/absolute_pose/NoncentralAbsoluteAdapter.hpp>
#include <opengv/sac/Ransac.hpp>
#include <opengv/sac_problems/absolute_pose/AbsolutePoseSacProblem.hpp>
#endif

namespace rtabmap
{

namespace util3d
{

Transform estimateMotion3DTo2D(
			const std::map<int, cv::Point3f> & words3A,
			const std::map<int, cv::KeyPoint> & words2B,
			const CameraModel & cameraModel,
			int minInliers,
			int iterations,
			double reprojError,
			int flagsPnP,
			int refineIterations,
			float maxVariance,
			const Transform & guess,
			const std::map<int, cv::Point3f> & words3B,
			cv::Mat * covariance,
			std::vector<int> * matchesOut,
			std::vector<int> * inliersOut)
{
	UASSERT(cameraModel.isValidForProjection());
	UASSERT(!guess.isNull());
	Transform transform;
	std::vector<int> matches, inliers;

	if(covariance)
	{
		*covariance = cv::Mat::eye(6,6,CV_64FC1);
	}

	// find correspondences
	std::vector<int> ids = uKeys(words2B);
	std::vector<cv::Point3f> objectPoints(ids.size());
	std::vector<cv::Point2f> imagePoints(ids.size());
	int oi=0;
	matches.resize(ids.size());
	for(unsigned int i=0; i<ids.size(); ++i)
	{
		std::map<int, cv::Point3f>::const_iterator iter=words3A.find(ids[i]);
		if(iter != words3A.end() && util3d::isFinite(iter->second))
		{
			const cv::Point3f & pt = iter->second;
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

	UDEBUG("words3A=%d words2B=%d matches=%d words3B=%d guess=%s reprojError=%f iterations=%d",
			(int)words3A.size(), (int)words2B.size(), (int)matches.size(), (int)words3B.size(),
			guess.prettyPrint().c_str(), reprojError, iterations);

	if((int)matches.size() >= minInliers)
	{
		//PnPRansac
		cv::Mat K = cameraModel.K();
		cv::Mat D = cameraModel.D();
		Transform guessCameraFrame = (guess * cameraModel.localTransform()).inverse();
		cv::Mat R = (cv::Mat_<double>(3,3) <<
				(double)guessCameraFrame.r11(), (double)guessCameraFrame.r12(), (double)guessCameraFrame.r13(),
				(double)guessCameraFrame.r21(), (double)guessCameraFrame.r22(), (double)guessCameraFrame.r23(),
				(double)guessCameraFrame.r31(), (double)guessCameraFrame.r32(), (double)guessCameraFrame.r33());

		cv::Mat rvec(3,1, CV_64FC1);
		cv::Rodrigues(R, rvec);
		cv::Mat tvec = (cv::Mat_<double>(3,1) <<
				(double)guessCameraFrame.x(), (double)guessCameraFrame.y(), (double)guessCameraFrame.z());

		util3d::solvePnPRansac(
				objectPoints,
				imagePoints,
				K,
				D,
				rvec,
				tvec,
				true,
				iterations,
				reprojError,
				minInliers, // min inliers
				inliers,
				flagsPnP,
				refineIterations);

		if((int)inliers.size() >= minInliers)
		{
			cv::Rodrigues(rvec, R);
			Transform pnp(R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2), tvec.at<double>(0),
						   R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2), tvec.at<double>(1),
						   R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2), tvec.at<double>(2));

			transform = (cameraModel.localTransform() * pnp).inverse();

			// compute variance (like in PCL computeVariance() method of sac_model.h)
			if(covariance && (!words3B.empty() || cameraModel.imageSize() != cv::Size()))
			{
				std::vector<float> errorSqrdDists(inliers.size());
				std::vector<float> errorSqrdAngles(inliers.size());
				Transform localTransformInv = cameraModel.localTransform().inverse();
				Transform transformCameraFrameInv = (transform * cameraModel.localTransform()).inverse();
				for(unsigned int i=0; i<inliers.size(); ++i)
				{
					cv::Point3f objPt = objectPoints[inliers[i]];

					// Project obj point from base frame of cameraA in cameraB frame (z+ in front of the cameraB)
					objPt = util3d::transformPoint(objPt, transformCameraFrameInv);

					// Get 3D point from target in cameraB frame
					std::map<int, cv::Point3f>::const_iterator iter = words3B.find(matches[inliers[i]]);
					cv::Point3f newPt;
					if(iter!=words3B.end() && util3d::isFinite(iter->second))
					{
						newPt = util3d::transformPoint(iter->second, localTransformInv);
					}
					else
					{
						//compute from projection
						Eigen::Vector3f ray = projectDepthTo3DRay(
								cameraModel.imageSize(),
								imagePoints.at(inliers[i]).x,
								imagePoints.at(inliers[i]).y,
								cameraModel.cx(),
								cameraModel.cy(),
								cameraModel.fx(),
								cameraModel.fy());
						// transform in camera B frame
						newPt = cv::Point3f(ray.x(), ray.y(), ray.z()) * objPt.z*1.1; // Add 10 % error
					}

					errorSqrdDists[i] = uNormSquared(objPt.x-newPt.x, objPt.y-newPt.y, objPt.z-newPt.z);

					Eigen::Vector4f v1(objPt.x, objPt.y, objPt.z, 0);
					Eigen::Vector4f v2(newPt.x, newPt.y, newPt.z, 0);
					errorSqrdAngles[i] = pcl::getAngle3D(v1, v2);
				}

				std::sort(errorSqrdDists.begin(), errorSqrdDists.end());
				//divide by 4 instead of 2 to ignore very very far features (stereo)
				double median_error_sqr_lin = 2.1981 * (double)errorSqrdDists[errorSqrdDists.size () >> 2];
				UASSERT(uIsFinite(median_error_sqr_lin));
				(*covariance)(cv::Range(0,3), cv::Range(0,3)) *= median_error_sqr_lin;
				std::sort(errorSqrdAngles.begin(), errorSqrdAngles.end());
				double median_error_sqr_ang = 2.1981 * (double)errorSqrdAngles[errorSqrdAngles.size () >> 2];
				UASSERT(uIsFinite(median_error_sqr_ang));
				(*covariance)(cv::Range(3,6), cv::Range(3,6)) *= median_error_sqr_ang;

				if(maxVariance > 0 && median_error_sqr_lin > maxVariance)
				{
					UWARN("Rejected PnP transform, variance is too high! %f > %f!", median_error_sqr_lin, maxVariance);
					*covariance = cv::Mat::eye(6,6,CV_64FC1);
					transform.setNull();
				}
			}
			else if(covariance)
			{
				// compute variance, which is the rms of reprojection errors
				std::vector<cv::Point2f> imagePointsReproj;
				cv::projectPoints(objectPoints, rvec, tvec, K, cv::Mat(), imagePointsReproj);
				float err = 0.0f;
				for(unsigned int i=0; i<inliers.size(); ++i)
				{
					err += uNormSquared(imagePoints.at(inliers[i]).x - imagePointsReproj.at(inliers[i]).x, imagePoints.at(inliers[i]).y - imagePointsReproj.at(inliers[i]).y);
				}
				UASSERT(uIsFinite(err));
				*covariance *= std::sqrt(err/float(inliers.size()));
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

Transform estimateMotion3DTo2D(
			const std::map<int, cv::Point3f> & words3A,
			const std::map<int, cv::KeyPoint> & words2B,
			const std::vector<CameraModel> & cameraModels,
			int minInliers,
			int iterations,
			double reprojError,
			int flagsPnP,
			int refineIterations,
			float maxVariance,
			const Transform & guess,
			const std::map<int, cv::Point3f> & words3B,
			cv::Mat * covariance,
			std::vector<int> * matchesOut,
			std::vector<int> * inliersOut)
{
	Transform transform;
#ifndef RTABMAP_OPENGV
	UERROR("This function is only available if rtabmap is built with OpenGV dependency.");
#else
	UASSERT(!cameraModels.empty() && cameraModels[0].imageWidth() > 0);
	int subImageWidth = cameraModels[0].imageWidth();
	for(size_t i=0; i<cameraModels.size(); ++i)
	{
		UASSERT(cameraModels[i].isValidForProjection());
		UASSERT(subImageWidth  == cameraModels[i].imageWidth());
	}

	UASSERT(!guess.isNull());

	std::vector<int> matches, inliers;

	if(covariance)
	{
		*covariance = cv::Mat::eye(6,6,CV_64FC1);
	}

	// find correspondences
	std::vector<int> ids = uKeys(words2B);
	std::vector<cv::Point3f> objectPoints(ids.size());
	std::vector<cv::Point2f> imagePoints(ids.size());
	int oi=0;
	matches.resize(ids.size());
	std::vector<int> cameraIndexes(ids.size());
	for(unsigned int i=0; i<ids.size(); ++i)
	{
		std::map<int, cv::Point3f>::const_iterator iter=words3A.find(ids[i]);
		if(iter != words3A.end() && util3d::isFinite(iter->second))
		{
			const cv::Point2f & kpt = words2B.find(ids[i])->second.pt;
			int cameraIndex = int(kpt.x / subImageWidth);
			UASSERT_MSG(cameraIndex >= 0 && cameraIndex < (int)cameraModels.size(),
					uFormat("cameraIndex=%d, models=%d, kpt.x=%f, subImageWidth=%f (Camera model image width=%d)",
							cameraIndex, (int)cameraModels.size(), kpt.x, subImageWidth, cameraModels[cameraIndex].imageWidth()).c_str());

			const cv::Point3f & pt = iter->second;
			objectPoints[oi] = pt;
			imagePoints[oi] = kpt;
			// convert in image space
			imagePoints[oi].x = imagePoints[oi].x - (cameraIndex*subImageWidth);
			cameraIndexes[oi] = cameraIndex;
			matches[oi++] = ids[i];
		}
	}

	objectPoints.resize(oi);
	imagePoints.resize(oi);
	cameraIndexes.resize(oi);
	matches.resize(oi);

	UDEBUG("words3A=%d words2B=%d matches=%d words3B=%d guess=%s reprojError=%f iterations=%d",
			(int)words3A.size(), (int)words2B.size(), (int)matches.size(), (int)words3B.size(),
			guess.prettyPrint().c_str(), reprojError, iterations);

	if((int)matches.size() >= minInliers)
	{
		// convert cameras
		opengv::translations_t camOffsets;
		opengv::rotations_t camRotations;
		for(size_t i=0; i<cameraModels.size(); ++i)
		{
			camOffsets.push_back(opengv::translation_t(
					cameraModels[i].localTransform().x(),
					cameraModels[i].localTransform().y(),
					cameraModels[i].localTransform().z()));
			camRotations.push_back(cameraModels[i].localTransform().toEigen4d().block<3,3>(0, 0));
		}

		// convert 3d points
		opengv::points_t points;
		// convert 2d-3d correspondences into bearing vectors
		opengv::bearingVectors_t bearingVectors;
		opengv::absolute_pose::NoncentralAbsoluteAdapter::camCorrespondences_t camCorrespondences;
		for(size_t i=0; i<objectPoints.size(); ++i)
		{
			int cameraIndex = cameraIndexes[i];
			points.push_back(opengv::point_t(objectPoints[i].x,objectPoints[i].y,objectPoints[i].z));
			cv::Vec3f pt;
			cameraModels[cameraIndex].project(imagePoints[i].x, imagePoints[i].y, 1, pt[0], pt[1], pt[2]);
			pt = cv::normalize(pt);
			bearingVectors.push_back(opengv::bearingVector_t(pt[0], pt[1], pt[2]));
			camCorrespondences.push_back(cameraIndex);
		}

		//create a non-central absolute adapter
		opengv::absolute_pose::NoncentralAbsoluteAdapter adapter(
				bearingVectors,
				camCorrespondences,
				points,
				camOffsets,
				camRotations );

		adapter.setR(guess.toEigen4d().block<3,3>(0, 0));
		adapter.sett(opengv::translation_t(guess.x(), guess.y(), guess.z()));

		//Create a AbsolutePoseSacProblem and Ransac
		//The method is set to GP3P
		opengv::sac::Ransac<opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem> ransac;
		std::shared_ptr<opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem> absposeproblem_ptr(
				new opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem(adapter, opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem::GP3P));

		ransac.sac_model_ = absposeproblem_ptr;
		ransac.threshold_ = 1.0 - cos(atan(reprojError/cameraModels[0].fx()));
		ransac.max_iterations_ = iterations;
		UDEBUG("Ransac params: threshold = %f (reprojError=%f fx=%f), max iterations=%d", ransac.threshold_, reprojError, cameraModels[0].fx(), ransac.max_iterations_);

		//Run the experiment
		ransac.computeModel();

		Transform pnp = Transform::fromEigen3d(ransac.model_coefficients_);

		UDEBUG("Ransac result: %s", pnp.prettyPrint().c_str());
		UDEBUG("Ransac iterations done: %d", ransac.iterations_);
		inliers = ransac.inliers_;
		UDEBUG("Ransac inliers: %ld", inliers.size());

		if((int)inliers.size() >= minInliers)
		{
			transform = pnp;

			// compute variance (like in PCL computeVariance() method of sac_model.h)
			if(covariance)
			{
				std::vector<float> errorSqrdDists(inliers.size());
				std::vector<float> errorSqrdAngles(inliers.size());
				for(unsigned int i=0; i<inliers.size(); ++i)
				{
					cv::Point3f objPt = objectPoints[inliers[i]];

					int cameraIndex = cameraIndexes[inliers[i]];
					Transform transformCameraFrameInv = (transform * cameraModels[cameraIndex].localTransform()).inverse();

					// Project obj point from base frame of cameraA in cameraB frame (z+ in front of the cameraB)
					objPt = util3d::transformPoint(objPt, transformCameraFrameInv);

					// Get 3D point from target in cameraB frame
					std::map<int, cv::Point3f>::const_iterator iter = words3B.find(matches[inliers[i]]);
					cv::Point3f newPt;
					if(iter!=words3B.end() && util3d::isFinite(iter->second))
					{
						newPt = util3d::transformPoint(iter->second, cameraModels[cameraIndex].localTransform().inverse());
					}
					else
					{
						//compute from projection
						Eigen::Vector3f ray = projectDepthTo3DRay(
								cameraModels[cameraIndex].imageSize(),
								imagePoints.at(inliers[i]).x,
								imagePoints.at(inliers[i]).y,
								cameraModels[cameraIndex].cx(),
								cameraModels[cameraIndex].cy(),
								cameraModels[cameraIndex].fx(),
								cameraModels[cameraIndex].fy());
						// transform in camera B frame
						newPt = cv::Point3f(ray.x(), ray.y(), ray.z()) * objPt.z*1.1; // Add 10 % error
					}

					errorSqrdDists[i] = uNormSquared(objPt.x-newPt.x, objPt.y-newPt.y, objPt.z-newPt.z);

					Eigen::Vector4f v1(objPt.x, objPt.y, objPt.z, 0);
					Eigen::Vector4f v2(newPt.x, newPt.y, newPt.z, 0);
					errorSqrdAngles[i] = pcl::getAngle3D(v1, v2);
				}

				std::sort(errorSqrdDists.begin(), errorSqrdDists.end());
				//divide by 4 instead of 2 to ignore very very far features (stereo)
				double median_error_sqr_lin = 2.1981 * (double)errorSqrdDists[errorSqrdDists.size () >> 2];
				UASSERT(uIsFinite(median_error_sqr_lin));
				(*covariance)(cv::Range(0,3), cv::Range(0,3)) *= median_error_sqr_lin;
				std::sort(errorSqrdAngles.begin(), errorSqrdAngles.end());
				double median_error_sqr_ang = 2.1981 * (double)errorSqrdAngles[errorSqrdAngles.size () >> 2];
				UASSERT(uIsFinite(median_error_sqr_ang));
				(*covariance)(cv::Range(3,6), cv::Range(3,6)) *= median_error_sqr_ang;

				if(maxVariance > 0 && median_error_sqr_lin > maxVariance)
				{
					UWARN("Rejected PnP transform, variance is too high! %f > %f!", median_error_sqr_lin, maxVariance);
					*covariance = cv::Mat::eye(6,6,CV_64FC1);
					transform.setNull();
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
#endif
	return transform;
}

Transform estimateMotion3DTo3D(
			const std::map<int, cv::Point3f> & words3A,
			const std::map<int, cv::Point3f> & words3B,
			int minInliers,
			double inliersDistance,
			int iterations,
			int refineIterations,
			cv::Mat * covariance,
			std::vector<int> * matchesOut,
			std::vector<int> * inliersOut)
{
	Transform transform;
	std::vector<cv::Point3f> inliers1; // previous
	std::vector<cv::Point3f> inliers2; // new

	std::vector<int> matches;
	util3d::findCorrespondences(
			words3A,
			words3B,
			inliers1,
			inliers2,
			0,
			&matches);
	UASSERT(inliers1.size() == inliers2.size());
	UDEBUG("Unique correspondences = %d", (int)inliers1.size());

	if(covariance)
	{
		*covariance = cv::Mat::eye(6,6,CV_64FC1);
	}

	std::vector<int> inliers;
	if((int)inliers1.size() >= minInliers)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr inliers1cloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr inliers2cloud(new pcl::PointCloud<pcl::PointXYZ>);
		inliers1cloud->resize(inliers1.size());
		inliers2cloud->resize(inliers1.size());
		for(unsigned int i=0; i<inliers1.size(); ++i)
		{
			(*inliers1cloud)[i].x = inliers1[i].x;
			(*inliers1cloud)[i].y = inliers1[i].y;
			(*inliers1cloud)[i].z = inliers1[i].z;
			(*inliers2cloud)[i].x = inliers2[i].x;
			(*inliers2cloud)[i].y = inliers2[i].y;
			(*inliers2cloud)[i].z = inliers2[i].z;
		}
		Transform t = util3d::transformFromXYZCorrespondences(
				inliers2cloud,
				inliers1cloud,
				inliersDistance,
				iterations,
				refineIterations,
				3.0,
				&inliers,
				covariance);

		if(!t.isNull() && (int)inliers.size() >= minInliers)
		{
			transform = t;
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


std::vector<float> computeReprojErrors(
		std::vector<cv::Point3f> opoints,
		std::vector<cv::Point2f> ipoints,
		const cv::Mat & cameraMatrix,
		const cv::Mat & distCoeffs,
		const cv::Mat & rvec,
		const cv::Mat & tvec,
		float reprojErrorThreshold,
		std::vector<int> & inliers)
{
	UASSERT(opoints.size() == ipoints.size());
	int count = (int)opoints.size();

	std::vector<cv::Point2f> projpoints;
	projectPoints(opoints, rvec, tvec, cameraMatrix, distCoeffs, projpoints);

	inliers.resize(count,0);
	std::vector<float> err(count);
	int oi=0;
	for (int i = 0; i < count; ++i)
	{
		float e = (float)cv::norm( ipoints[i] - projpoints[i]);
		if(e <= reprojErrorThreshold)
		{
			inliers[oi] = i;
			err[oi++] = e;
		}
	}
	inliers.resize(oi);
	err.resize(oi);
	return err;
}

void solvePnPRansac(
		const std::vector<cv::Point3f> & objectPoints,
		const std::vector<cv::Point2f> & imagePoints,
		const cv::Mat & cameraMatrix,
		const cv::Mat & distCoeffs,
		cv::Mat & rvec,
		cv::Mat & tvec,
		bool useExtrinsicGuess,
        int iterationsCount,
        float reprojectionError,
        int minInliersCount,
        std::vector<int> & inliers,
        int flags,
        int refineIterations,
        float refineSigma)
{
	if(minInliersCount < 4)
	{
		minInliersCount = 4;
	}

	// Use OpenCV3 version of solvePnPRansac in OpenCV2.
	// FIXME: we should use this version of solvePnPRansac in newer 3.3.1 too, which seems a lot less stable!?!? Why!?
	cv3::solvePnPRansac(
			objectPoints,
			imagePoints,
			cameraMatrix,
			distCoeffs,
			rvec,
			tvec,
			useExtrinsicGuess,
			iterationsCount,
			reprojectionError,
			0.99, // confidence
			inliers,
			flags);

	float inlierThreshold = reprojectionError;
	if((int)inliers.size() >= minInliersCount && refineIterations>0)
	{
		float error_threshold = inlierThreshold;
		int refine_iterations = 0;
		bool inlier_changed = false, oscillating = false;
		std::vector<int> new_inliers, prev_inliers = inliers;
		std::vector<size_t> inliers_sizes;
		//Eigen::VectorXf new_model_coefficients = model_coefficients;
		cv::Mat new_model_rvec = rvec;
		cv::Mat new_model_tvec = tvec;

		do
		{
			// Get inliers from the current model
			std::vector<cv::Point3f> opoints_inliers(prev_inliers.size());
			std::vector<cv::Point2f> ipoints_inliers(prev_inliers.size());
			for(unsigned int i=0; i<prev_inliers.size(); ++i)
			{
				opoints_inliers[i] = objectPoints[prev_inliers[i]];
				ipoints_inliers[i] = imagePoints[prev_inliers[i]];
			}

			UDEBUG("inliers=%d refine_iterations=%d, rvec=%f,%f,%f tvec=%f,%f,%f", (int)prev_inliers.size(), refine_iterations,
					*new_model_rvec.ptr<double>(0), *new_model_rvec.ptr<double>(1), *new_model_rvec.ptr<double>(2),
					*new_model_tvec.ptr<double>(0), *new_model_tvec.ptr<double>(1), *new_model_tvec.ptr<double>(2));

			// Optimize the model coefficients
			cv::solvePnP(opoints_inliers, ipoints_inliers, cameraMatrix, distCoeffs, new_model_rvec, new_model_tvec, true, flags);
			inliers_sizes.push_back(prev_inliers.size());

			UDEBUG("rvec=%f,%f,%f tvec=%f,%f,%f",
					*new_model_rvec.ptr<double>(0), *new_model_rvec.ptr<double>(1), *new_model_rvec.ptr<double>(2),
					*new_model_tvec.ptr<double>(0), *new_model_tvec.ptr<double>(1), *new_model_tvec.ptr<double>(2));

			// Select the new inliers based on the optimized coefficients and new threshold
			std::vector<float> err = computeReprojErrors(objectPoints, imagePoints, cameraMatrix, distCoeffs, new_model_rvec, new_model_tvec, error_threshold, new_inliers);
			UDEBUG("RANSAC refineModel: Number of inliers found (before/after): %d/%d, with an error threshold of %f.",
					(int)prev_inliers.size (), (int)new_inliers.size (), error_threshold);

			if ((int)new_inliers.size() < minInliersCount)
			{
				++refine_iterations;
				if (refine_iterations >= refineIterations)
				{
					break;
				}
				continue;
			}

			// Estimate the variance and the new threshold
			float m = uMean(err.data(), err.size());
			float variance = uVariance(err.data(), err.size());
			error_threshold = std::min(inlierThreshold, refineSigma * float(sqrt(variance)));

			UDEBUG ("RANSAC refineModel: New estimated error threshold: %f (variance=%f mean=%f) on iteration %d out of %d.",
				  error_threshold, variance, m, refine_iterations, refineIterations);
			inlier_changed = false;
			std::swap (prev_inliers, new_inliers);

			// If the number of inliers changed, then we are still optimizing
			if (new_inliers.size () != prev_inliers.size ())
			{
				// Check if the number of inliers is oscillating in between two values
				if ((int)inliers_sizes.size () >= minInliersCount)
				{
					if (inliers_sizes[inliers_sizes.size () - 1] == inliers_sizes[inliers_sizes.size () - 3] &&
					inliers_sizes[inliers_sizes.size () - 2] == inliers_sizes[inliers_sizes.size () - 4])
					{
						oscillating = true;
						break;
					}
				}
				inlier_changed = true;
				continue;
			}

			// Check the values of the inlier set
			for (size_t i = 0; i < prev_inliers.size (); ++i)
			{
				// If the value of the inliers changed, then we are still optimizing
				if (prev_inliers[i] != new_inliers[i])
				{
					inlier_changed = true;
					break;
				}
			}
		}
		while (inlier_changed && ++refine_iterations < refineIterations);

		// If the new set of inliers is empty, we didn't do a good job refining
		if ((int)prev_inliers.size() < minInliersCount)
		{
			UWARN ("RANSAC refineModel: Refinement failed: got very low inliers (%d)!", (int)prev_inliers.size());
		}

		if (oscillating)
		{
			UDEBUG("RANSAC refineModel: Detected oscillations in the model refinement.");
		}

		std::swap (inliers, new_inliers);
		rvec = new_model_rvec;
		tvec = new_model_tvec;
	}

}

}

}
