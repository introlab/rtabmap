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

#include "rtabmap/core/util3d_features.h"

#include "rtabmap/core/util3d.h"
#include "rtabmap/core/util3d_transforms.h"
#include "rtabmap/core/util3d_correspondences.h"

#include "rtabmap/core/EpipolarGeometry.h"

#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UMath.h>

#include <opencv2/video/tracking.hpp>

namespace rtabmap
{

namespace util3d
{

pcl::PointCloud<pcl::PointXYZ>::Ptr generateKeypoints3DDepth(
		const std::vector<cv::KeyPoint> & keypoints,
		const cv::Mat & depth,
		const CameraModel & cameraModel)
{
	UASSERT(cameraModel.isValid());
	std::vector<CameraModel> models;
	models.push_back(cameraModel);
	return generateKeypoints3DDepth(keypoints, depth, models);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr generateKeypoints3DDepth(
		const std::vector<cv::KeyPoint> & keypoints,
		const cv::Mat & depth,
		const std::vector<CameraModel> & cameraModels)
{
	UASSERT(!depth.empty() && (depth.type() == CV_32FC1 || depth.type() == CV_16UC1));
	UASSERT(cameraModels.size());
	pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints3d(new pcl::PointCloud<pcl::PointXYZ>);
	if(!depth.empty())
	{
		UASSERT(int((depth.cols/cameraModels.size())*cameraModels.size()) == depth.cols);
		float subImageWidth = depth.cols/cameraModels.size();
		keypoints3d->resize(keypoints.size());
		for(unsigned int i=0; i!=keypoints.size(); ++i)
		{
			int cameraIndex = int(keypoints[i].pt.x / subImageWidth);
			UASSERT(cameraIndex < (int)cameraModels.size());
			pcl::PointXYZ pt = util3d::projectDepthTo3D(
					depth,
					keypoints[i].pt.x-subImageWidth*cameraIndex,
					keypoints[i].pt.y,
					cameraModels.at(cameraIndex).cx(),
					cameraModels.at(cameraIndex).cy(),
					cameraModels.at(cameraIndex).fx(),
					cameraModels.at(cameraIndex).fy(),
					true);

			if(pcl::isFinite(pt) &&
				!cameraModels.at(cameraIndex).localTransform().isNull() &&
			    !cameraModels.at(cameraIndex).localTransform().isIdentity())
			{
				pt = util3d::transformPoint(pt, cameraModels.at(cameraIndex).localTransform());
			}
			keypoints3d->at(i) = pt;
		}
	}
	return keypoints3d;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr generateKeypoints3DDisparity(
		const std::vector<cv::KeyPoint> & keypoints,
		const cv::Mat & disparity,
		const StereoCameraModel & stereoCameraModel)
{
	UASSERT(!disparity.empty() && (disparity.type() == CV_16SC1 || disparity.type() == CV_32F));
	UASSERT(stereoCameraModel.isValid());
	pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints3d(new pcl::PointCloud<pcl::PointXYZ>);
	keypoints3d->resize(keypoints.size());
	for(unsigned int i=0; i!=keypoints.size(); ++i)
	{
		pcl::PointXYZ pt = util3d::projectDisparityTo3D(
				keypoints[i].pt,
				disparity,
				stereoCameraModel.left().cx(),
				stereoCameraModel.left().cy(),
				stereoCameraModel.left().fx(),
				stereoCameraModel.baseline());

		if(pcl::isFinite(pt) &&
			!stereoCameraModel.left().localTransform().isNull() &&
			!stereoCameraModel.left().localTransform().isIdentity())
		{
			pt = util3d::transformPoint(pt, stereoCameraModel.left().localTransform());
		}
		keypoints3d->at(i) = pt;
	}
	return keypoints3d;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr generateKeypoints3DStereo(
		const std::vector<cv::KeyPoint> & keypoints,
		const cv::Mat & leftImage,
		const cv::Mat & rightImage,
		float fx,
		float baseline,
		float cx,
		float cy,
		Transform localTransform,
		int flowWinSize,
		int flowMaxLevel,
		int flowIterations,
		double flowEps,
		double maxCorrespondencesSlope)
{
	std::vector<cv::Point2f> leftCorners;
	cv::KeyPoint::convert(keypoints, leftCorners);
	return generateKeypoints3DStereo(
			leftCorners,
			leftImage,
			rightImage,
			fx,
			baseline,
			cx,
			cy,
			localTransform,
			flowWinSize,
			flowMaxLevel,
			flowIterations,
			flowEps,
			maxCorrespondencesSlope);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr generateKeypoints3DStereo(
		const std::vector<cv::Point2f> & leftCorners,
		const cv::Mat & leftImage,
		const cv::Mat & rightImage,
		float fx,
		float baseline,
		float cx,
		float cy,
		Transform localTransform,
		int flowWinSize,
		int flowMaxLevel,
		int flowIterations,
		double flowEps,
		double maxCorrespondencesSlope)
{
	UASSERT(!leftImage.empty() && !rightImage.empty() &&
			leftImage.type() == CV_8UC1 && rightImage.type() == CV_8UC1 &&
			leftImage.rows == rightImage.rows && leftImage.cols == rightImage.cols);
	UASSERT(fx > 0.0f && baseline > 0.0f);

	// Find features in the new left image
	std::vector<unsigned char> status;
	std::vector<float> err;
	std::vector<cv::Point2f> rightCorners;
	UDEBUG("cv::calcOpticalFlowPyrLK() begin");
	cv::calcOpticalFlowPyrLK(
			leftImage,
			rightImage,
			leftCorners,
			rightCorners,
			status,
			err,
			cv::Size(flowWinSize, flowWinSize), flowMaxLevel,
			cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, flowIterations, flowEps),
			cv::OPTFLOW_LK_GET_MIN_EIGENVALS, 1e-4);
	UDEBUG("cv::calcOpticalFlowPyrLK() end");

	pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints3d(new pcl::PointCloud<pcl::PointXYZ>);
	keypoints3d->resize(leftCorners.size());
	float bad_point = std::numeric_limits<float>::quiet_NaN ();
	UASSERT(status.size() == leftCorners.size());
	for(unsigned int i=0; i<status.size(); ++i)
	{
		pcl::PointXYZ pt(bad_point, bad_point, bad_point);
		if(status[i])
		{
			float disparity = leftCorners[i].x - rightCorners[i].x;
			float slope = fabs((leftCorners[i].y-rightCorners[i].y) / (leftCorners[i].x-rightCorners[i].x));
			if(disparity > 0.0f &&
			   (maxCorrespondencesSlope <=0 || fabs(leftCorners[i].y-rightCorners[i].y) <= 1.0f || slope <= maxCorrespondencesSlope))
			{
				pcl::PointXYZ tmpPt = util3d::projectDisparityTo3D(
						leftCorners[i],
						disparity,
						cx,
						cy,
						fx,
						baseline);

				if(pcl::isFinite(tmpPt))
				{
					pt = tmpPt;
					if(!localTransform.isNull() &&
					   !localTransform.isIdentity())
					{
						pt = util3d::transformPoint(pt, localTransform);
					}
				}
			}
		}

		keypoints3d->at(i) = pt;
	}
	return keypoints3d;
}

// cameraTransform, from ref to next
// return 3D points in ref referential
// If cameraTransform is not null, it will be used for triangulation instead of the camera transform computed by epipolar geometry
// when refGuess3D is passed and cameraTransform is null, scale will be estimated, returning scaled cloud and camera transform
std::multimap<int, pcl::PointXYZ> generateWords3DMono(
		const std::multimap<int, cv::KeyPoint> & refWords,
		const std::multimap<int, cv::KeyPoint> & nextWords,
		const CameraModel & cameraModel,
		Transform & cameraTransform,
		int pnpIterations,
		float pnpReprojError,
		int pnpFlags,
		float ransacParam1,
		float ransacParam2,
		const std::multimap<int, pcl::PointXYZ> & refGuess3D,
		double * varianceOut)
{
	UASSERT(cameraModel.isValid());
	std::multimap<int, pcl::PointXYZ> words3D;
	std::list<std::pair<int, std::pair<cv::KeyPoint, cv::KeyPoint> > > pairs;
	if(EpipolarGeometry::findPairsUnique(refWords, nextWords, pairs) > 8)
	{
		std::vector<unsigned char> status;
		cv::Mat F = EpipolarGeometry::findFFromWords(pairs, status, ransacParam1, ransacParam2);
		if(!F.empty())
		{
			//get inliers
			//normalize coordinates
			int oi = 0;
			UASSERT(status.size() == pairs.size());
			std::list<std::pair<int, std::pair<cv::KeyPoint, cv::KeyPoint> > >::iterator iter=pairs.begin();
			std::vector<cv::Point2f> refCorners(status.size());
			std::vector<cv::Point2f> newCorners(status.size());
			std::vector<int> indexes(status.size());
			for(unsigned int i=0; i<status.size(); ++i)
			{
				if(status[i])
				{
					refCorners[oi] = iter->second.first.pt;
					newCorners[oi] = iter->second.second.pt;
					indexes[oi] = iter->first;
					++oi;
				}
				++iter;
			}
			refCorners.resize(oi);
			newCorners.resize(oi);
			indexes.resize(oi);

			UDEBUG("inliers=%d/%d", oi, pairs.size());
			if(oi > 3)
			{
				std::vector<cv::Point2f> refCornersRefined;
				std::vector<cv::Point2f> newCornersRefined;
				cv::correctMatches(F, refCorners, newCorners, refCornersRefined, newCornersRefined);
				refCorners = refCornersRefined;
				newCorners = newCornersRefined;

				cv::Mat x(3, (int)refCorners.size(), CV_64FC1);
				cv::Mat xp(3, (int)refCorners.size(), CV_64FC1);
				for(unsigned int i=0; i<refCorners.size(); ++i)
				{
					x.at<double>(0, i) = refCorners[i].x;
					x.at<double>(1, i) = refCorners[i].y;
					x.at<double>(2, i) = 1;

					xp.at<double>(0, i) = newCorners[i].x;
					xp.at<double>(1, i) = newCorners[i].y;
					xp.at<double>(2, i) = 1;
				}

				cv::Mat K = cameraModel.K();
				cv::Mat Kinv = K.inv();
				cv::Mat E = K.t()*F*K;
				cv::Mat x_norm = Kinv * x;
				cv::Mat xp_norm = Kinv * xp;
				x_norm = x_norm.rowRange(0,2);
				xp_norm = xp_norm.rowRange(0,2);

				cv::Mat P = EpipolarGeometry::findPFromE(E, x_norm, xp_norm);
				if(!P.empty())
				{
					cv::Mat P0 = cv::Mat::zeros(3, 4, CV_64FC1);
					P0.at<double>(0,0) = 1;
					P0.at<double>(1,1) = 1;
					P0.at<double>(2,2) = 1;

					bool useCameraTransformGuess = !cameraTransform.isNull();
					//if camera transform is set, use it instead of the computed one from epipolar geometry
					if(useCameraTransformGuess)
					{
						Transform t = (cameraModel.localTransform().inverse()*cameraTransform*cameraModel.localTransform()).inverse();
						P = (cv::Mat_<double>(3,4) <<
								(double)t.r11(), (double)t.r12(), (double)t.r13(), (double)t.x(),
								(double)t.r21(), (double)t.r22(), (double)t.r23(), (double)t.y(),
								(double)t.r31(), (double)t.r32(), (double)t.r33(), (double)t.z());
					}

					// triangulate the points
					//std::vector<double> reprojErrors;
					//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
					//EpipolarGeometry::triangulatePoints(x_norm, xp_norm, P0, P, cloud, reprojErrors);
					cv::Mat pts4D;
					cv::triangulatePoints(P0, P, x_norm, xp_norm, pts4D);

					for(unsigned int i=0; i<indexes.size(); ++i)
					{
						//if(cloud->at(i).z > 0)
						//{
						//	words3D.insert(std::make_pair(indexes[i], util3d::transformPoint(cloud->at(i), localTransform)));
						//}
						pts4D.col(i) /= pts4D.at<double>(3,i);
						if(pts4D.at<double>(2,i) > 0)
						{
							words3D.insert(std::make_pair(indexes[i], util3d::transformPoint(pcl::PointXYZ(pts4D.at<double>(0,i), pts4D.at<double>(1,i), pts4D.at<double>(2,i)), cameraModel.localTransform())));
						}
					}

					if(refGuess3D.size())
					{
						// scale estimation
						pcl::PointCloud<pcl::PointXYZ>::Ptr inliersRef(new pcl::PointCloud<pcl::PointXYZ>);
						pcl::PointCloud<pcl::PointXYZ>::Ptr inliersRefGuess(new pcl::PointCloud<pcl::PointXYZ>);
						util3d::findCorrespondences(
								words3D,
								refGuess3D,
								*inliersRef,
								*inliersRefGuess,
								0);

						if(inliersRef->size())
						{
							// estimate the scale
							float scale = 1.0f;
							float variance = 1.0f;
							if(!useCameraTransformGuess)
							{
								std::multimap<float, float> scales; // <variance, scale>
								for(unsigned int i=0; i<inliersRef->size(); ++i)
								{
									// using x as depth, assuming we are in global referential
									float s = inliersRefGuess->at(i).x/inliersRef->at(i).x;
									std::vector<float> errorSqrdDists(inliersRef->size());
									for(unsigned int j=0; j<inliersRef->size(); ++j)
									{
										pcl::PointXYZ refPt = inliersRef->at(j);
										refPt.x *= s;
										refPt.y *= s;
										refPt.z *= s;
										const pcl::PointXYZ & newPt = inliersRefGuess->at(j);
										errorSqrdDists[j] = uNormSquared(refPt.x-newPt.x, refPt.y-newPt.y, refPt.z-newPt.z);
									}
									std::sort(errorSqrdDists.begin(), errorSqrdDists.end());
									double median_error_sqr = (double)errorSqrdDists[errorSqrdDists.size () >> 1];
									float var = 2.1981 * median_error_sqr;
									//UDEBUG("scale %d = %f variance = %f", (int)i, s, variance);

									scales.insert(std::make_pair(var, s));
								}
								scale = scales.begin()->second;
								variance = scales.begin()->first;;
							}
							else
							{
								//compute variance at scale=1
								std::vector<float> errorSqrdDists(inliersRef->size());
								for(unsigned int j=0; j<inliersRef->size(); ++j)
								{
									const pcl::PointXYZ & refPt = inliersRef->at(j);
									const pcl::PointXYZ & newPt = inliersRefGuess->at(j);
									errorSqrdDists[j] = uNormSquared(refPt.x-newPt.x, refPt.y-newPt.y, refPt.z-newPt.z);
								}
								std::sort(errorSqrdDists.begin(), errorSqrdDists.end());
								double median_error_sqr = (double)errorSqrdDists[errorSqrdDists.size () >> 1];
								 variance = 2.1981 * median_error_sqr;
							}

							UDEBUG("scale used = %f (variance=%f)", scale, variance);
							if(varianceOut)
							{
								*varianceOut = variance;
							}

							if(!useCameraTransformGuess)
							{
								std::vector<cv::Point3f> objectPoints(indexes.size());
								std::vector<cv::Point2f> imagePoints(indexes.size());
								int oi=0;
								for(unsigned int i=0; i<indexes.size(); ++i)
								{
									std::multimap<int, pcl::PointXYZ>::iterator iter = words3D.find(indexes[i]);
									if(pcl::isFinite(iter->second))
									{
										iter->second.x *= scale;
										iter->second.y *= scale;
										iter->second.z *= scale;
										objectPoints[oi].x = iter->second.x;
										objectPoints[oi].y = iter->second.y;
										objectPoints[oi].z = iter->second.z;
										imagePoints[oi] = newCorners[i];
										++oi;
									}
								}
								objectPoints.resize(oi);
								imagePoints.resize(oi);

								//PnPRansac
								Transform guess = cameraModel.localTransform().inverse();
								cv::Mat R = (cv::Mat_<double>(3,3) <<
										(double)guess.r11(), (double)guess.r12(), (double)guess.r13(),
										(double)guess.r21(), (double)guess.r22(), (double)guess.r23(),
										(double)guess.r31(), (double)guess.r32(), (double)guess.r33());
								cv::Mat rvec(1,3, CV_64FC1);
								cv::Rodrigues(R, rvec);
								cv::Mat tvec = (cv::Mat_<double>(1,3) << (double)guess.x(), (double)guess.y(), (double)guess.z());
								std::vector<int> inliersV;
								cv::solvePnPRansac(
										objectPoints,
										imagePoints,
										K,
										cv::Mat(),
										rvec,
										tvec,
										true,
										pnpIterations,
										pnpReprojError,
										0,
										inliersV,
										pnpFlags);

								UDEBUG("PnP inliers = %d / %d", (int)inliersV.size(), (int)objectPoints.size());

								if(inliersV.size())
								{
									cv::Rodrigues(rvec, R);
									Transform pnp(R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2), tvec.at<double>(0),
												   R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2), tvec.at<double>(1),
												   R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2), tvec.at<double>(2));

									cameraTransform = (cameraModel.localTransform() * pnp).inverse();
								}
								else
								{
									UWARN("No inliers after PnP!");
								}
							}
						}
						else
						{
							UWARN("Cannot compute the scale, no points corresponding between the generated ref words and words guess");
						}
					}
					else if(!useCameraTransformGuess)
					{
						cv::Mat R, T;
						EpipolarGeometry::findRTFromP(P, R, T);

						Transform t(R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2), T.at<double>(0),
									R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2), T.at<double>(1),
									R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2), T.at<double>(2));

						cameraTransform = (cameraModel.localTransform() * t).inverse() * cameraModel.localTransform();
					}
				}
			}
		}
	}
	UDEBUG("wordsSet=%d / %d", (int)words3D.size(), (int)refWords.size());

	return words3D;
}

std::multimap<int, cv::KeyPoint> aggregate(
		const std::list<int> & wordIds,
		const std::vector<cv::KeyPoint> & keypoints)
{
	std::multimap<int, cv::KeyPoint> words;
	std::vector<cv::KeyPoint>::const_iterator kpIter = keypoints.begin();
	for(std::list<int>::const_iterator iter=wordIds.begin(); iter!=wordIds.end(); ++iter)
	{
		words.insert(std::pair<int, cv::KeyPoint >(*iter, *kpIter));
		++kpIter;
	}
	return words;
}

}

}
