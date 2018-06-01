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

#include "rtabmap/core/util3d_features.h"

#include "rtabmap/core/util2d.h"
#include "rtabmap/core/util3d.h"
#include "rtabmap/core/util3d_transforms.h"
#include "rtabmap/core/util3d_correspondences.h"
#include "rtabmap/core/util3d_motion_estimation.h"

#include "rtabmap/core/EpipolarGeometry.h"

#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UMath.h>
#include <rtabmap/utilite/UStl.h>

#include <opencv2/video/tracking.hpp>

namespace rtabmap
{

namespace util3d
{

std::vector<cv::Point3f> generateKeypoints3DDepth(
		const std::vector<cv::KeyPoint> & keypoints,
		const cv::Mat & depth,
		const CameraModel & cameraModel,
		float minDepth,
		float maxDepth)
{
	UASSERT(cameraModel.isValidForProjection());
	std::vector<CameraModel> models;
	models.push_back(cameraModel);
	return generateKeypoints3DDepth(keypoints, depth, models, minDepth, maxDepth);
}

std::vector<cv::Point3f> generateKeypoints3DDepth(
		const std::vector<cv::KeyPoint> & keypoints,
		const cv::Mat & depth,
		const std::vector<CameraModel> & cameraModels,
		float minDepth,
		float maxDepth)
{
	UASSERT(!depth.empty() && (depth.type() == CV_32FC1 || depth.type() == CV_16UC1));
	UASSERT(cameraModels.size());
	std::vector<cv::Point3f> keypoints3d;
	if(!depth.empty())
	{
		UASSERT(int((depth.cols/cameraModels.size())*cameraModels.size()) == depth.cols);
		float subImageWidth = depth.cols/cameraModels.size();
		keypoints3d.resize(keypoints.size());
		float rgbToDepthFactorX = 1.0f/(cameraModels[0].imageWidth()>0?cameraModels[0].imageWidth()/subImageWidth:1);
		float rgbToDepthFactorY = 1.0f/(cameraModels[0].imageHeight()>0?cameraModels[0].imageHeight()/depth.rows:1);
		float bad_point = std::numeric_limits<float>::quiet_NaN ();
		for(unsigned int i=0; i<keypoints.size(); ++i)
		{
			float x = keypoints[i].pt.x*rgbToDepthFactorX;
			float y = keypoints[i].pt.y*rgbToDepthFactorY;
			int cameraIndex = int(x / subImageWidth);
			UASSERT_MSG(cameraIndex >= 0 && cameraIndex < (int)cameraModels.size(),
					uFormat("cameraIndex=%d, models=%d, kpt.x=%f, subImageWidth=%f (Camera model image width=%d)",
							cameraIndex, (int)cameraModels.size(), keypoints[i].pt.x, subImageWidth, cameraModels[0].imageWidth()).c_str());

			pcl::PointXYZ ptXYZ = util3d::projectDepthTo3D(
					cameraModels.size()==1?depth:cv::Mat(depth, cv::Range::all(), cv::Range(subImageWidth*cameraIndex,subImageWidth*(cameraIndex+1))),
					x-subImageWidth*cameraIndex,
					y,
					cameraModels.at(cameraIndex).cx()*rgbToDepthFactorX,
					cameraModels.at(cameraIndex).cy()*rgbToDepthFactorY,
					cameraModels.at(cameraIndex).fx()*rgbToDepthFactorX,
					cameraModels.at(cameraIndex).fy()*rgbToDepthFactorY,
					true);

			cv::Point3f pt(bad_point, bad_point, bad_point);
			if(pcl::isFinite(ptXYZ) &&
				(minDepth < 0.0f || ptXYZ.z > minDepth) &&
				(maxDepth <= 0.0f || ptXYZ.z <= maxDepth))
			{
				pt = cv::Point3f(ptXYZ.x, ptXYZ.y, ptXYZ.z);
				if(!cameraModels.at(cameraIndex).localTransform().isNull() &&
				   !cameraModels.at(cameraIndex).localTransform().isIdentity())
				{
					pt = util3d::transformPoint(pt, cameraModels.at(cameraIndex).localTransform());
				}
			}
			keypoints3d.at(i) = pt;
		}
	}
	return keypoints3d;
}

std::vector<cv::Point3f> generateKeypoints3DDisparity(
		const std::vector<cv::KeyPoint> & keypoints,
		const cv::Mat & disparity,
		const StereoCameraModel & stereoCameraModel,
		float minDepth,
		float maxDepth)
{
	UASSERT(!disparity.empty() && (disparity.type() == CV_16SC1 || disparity.type() == CV_32F));
	UASSERT(stereoCameraModel.isValidForProjection());
	std::vector<cv::Point3f> keypoints3d;
	keypoints3d.resize(keypoints.size());
	float bad_point = std::numeric_limits<float>::quiet_NaN ();
	for(unsigned int i=0; i!=keypoints.size(); ++i)
	{
		cv::Point3f tmpPt = util3d::projectDisparityTo3D(
				keypoints[i].pt,
				disparity,
				stereoCameraModel);

		cv::Point3f pt(bad_point, bad_point, bad_point);
		if(util3d::isFinite(tmpPt) &&
			(minDepth < 0.0f || tmpPt.z > minDepth) &&
			(maxDepth <= 0.0f || tmpPt.z <= maxDepth))
		{
			pt = tmpPt;
			if(!stereoCameraModel.left().localTransform().isNull() &&
				!stereoCameraModel.left().localTransform().isIdentity())
			{
				pt = util3d::transformPoint(pt, stereoCameraModel.left().localTransform());
			}
		}
		keypoints3d.at(i) = pt;
	}
	return keypoints3d;
}

std::vector<cv::Point3f> generateKeypoints3DStereo(
		const std::vector<cv::Point2f> & leftCorners,
		const std::vector<cv::Point2f> & rightCorners,
		const StereoCameraModel & model,
		const std::vector<unsigned char> & mask,
		float minDepth,
		float maxDepth)
{
	UASSERT(leftCorners.size() == rightCorners.size());
	UASSERT(mask.size() == 0 || leftCorners.size() == mask.size());
	UASSERT(model.left().fx()> 0.0f && model.baseline() > 0.0f);

	std::vector<cv::Point3f> keypoints3d;
	keypoints3d.resize(leftCorners.size());
	float bad_point = std::numeric_limits<float>::quiet_NaN ();
	for(unsigned int i=0; i<leftCorners.size(); ++i)
	{
		cv::Point3f pt(bad_point, bad_point, bad_point);
		if(mask.empty() || mask[i])
		{
			float disparity = leftCorners[i].x - rightCorners[i].x;
			if(disparity != 0.0f)
			{
				cv::Point3f tmpPt = util3d::projectDisparityTo3D(
						leftCorners[i],
						disparity,
						model);

				if(util3d::isFinite(tmpPt) &&
				   (minDepth < 0.0f || tmpPt.z > minDepth) &&
				   (maxDepth <= 0.0f || tmpPt.z <= maxDepth))
				{
					pt = tmpPt;
					if(!model.localTransform().isNull() &&
					   !model.localTransform().isIdentity())
					{
						pt = util3d::transformPoint(pt, model.localTransform());
					}
				}
			}
		}

		keypoints3d.at(i) = pt;
	}
	return keypoints3d;
}

// cameraTransform, from ref to next
// return 3D points in ref referential
// If cameraTransform is not null, it will be used for triangulation instead of the camera transform computed by epipolar geometry
// when refGuess3D is passed and cameraTransform is null, scale will be estimated, returning scaled cloud and camera transform
std::map<int, cv::Point3f> generateWords3DMono(
		const std::map<int, cv::KeyPoint> & refWords,
		const std::map<int, cv::KeyPoint> & nextWords,
		const CameraModel & cameraModel,
		Transform & cameraTransform,
		int pnpIterations,
		float pnpReprojError,
		int pnpFlags,
		int pnpRefineIterations,
		float ransacParam1,
		float ransacParam2,
		const std::map<int, cv::Point3f> & refGuess3D,
		double * varianceOut)
{
	UASSERT(cameraModel.isValidForProjection());
	std::map<int, cv::Point3f> words3D;
	std::list<std::pair<int, std::pair<cv::KeyPoint, cv::KeyPoint> > > pairs;
	int pairsFound = EpipolarGeometry::findPairs(refWords, nextWords, pairs);
	UDEBUG("pairsFound=%d/%d", pairsFound, int(refWords.size()>nextWords.size()?refWords.size():nextWords.size()));
	if(pairsFound > 8)
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

						if(ULogger::level() == ULogger::kDebug)
						{
							UDEBUG("Guess = %s", t.prettyPrint().c_str());
							UDEBUG("Epipolar = %s", Transform(P).prettyPrint().c_str());
							Transform PT = Transform(P);
							float scale = t.getNorm()/PT.getNorm();
							UDEBUG("Scale= %f", scale);
							PT.x()*=scale;
							PT.y()*=scale;
							PT.z()*=scale;
							UDEBUG("Epipolar scaled= %s", PT.prettyPrint().c_str());
						}

						P = (cv::Mat_<double>(3,4) <<
								(double)t.r11(), (double)t.r12(), (double)t.r13(), (double)t.x(),
								(double)t.r21(), (double)t.r22(), (double)t.r23(), (double)t.y(),
								(double)t.r31(), (double)t.r32(), (double)t.r33(), (double)t.z());
					}

					// triangulate the points
					//std::vector<double> reprojErrors;
					//std::vector<cv::Point3f> cloud;
					//EpipolarGeometry::triangulatePoints(x_norm, xp_norm, P0, P, cloud, reprojErrors);
					cv::Mat pts4D;
					cv::triangulatePoints(P0, P, x_norm, xp_norm, pts4D);

					UASSERT((int)indexes.size() == pts4D.cols && pts4D.rows == 4);
					for(unsigned int i=0; i<indexes.size(); ++i)
					{
						//if(cloud->at(i).z > 0)
						//{
						//	words3D.insert(std::make_pair(indexes[i], util3d::transformPoint(cloud->at(i), localTransform)));
						//}
						pts4D.col(i) /= pts4D.at<double>(3,i);
						if(pts4D.at<double>(2,i) > 0)
						{
							words3D.insert(std::make_pair(indexes[i], util3d::transformPoint(cv::Point3f(pts4D.at<double>(0,i), pts4D.at<double>(1,i), pts4D.at<double>(2,i)), cameraModel.localTransform())));
						}
					}

					UDEBUG("ref guess=%d", (int)refGuess3D.size());
					if(refGuess3D.size())
					{
						// scale estimation
						std::vector<cv::Point3f> inliersRef;
						std::vector<cv::Point3f> inliersRefGuess;
						util3d::findCorrespondences(
								words3D,
								refGuess3D,
								inliersRef,
								inliersRefGuess,
								0);

						if(inliersRef.size())
						{
							// estimate the scale
							float scale = 1.0f;
							float variance = 1.0f;
							if(!useCameraTransformGuess)
							{
								std::multimap<float, float> scales; // <variance, scale>
								for(unsigned int i=0; i<inliersRef.size(); ++i)
								{
									// using x as depth, assuming we are in global referential
									float s = inliersRefGuess.at(i).x/inliersRef.at(i).x;
									std::vector<float> errorSqrdDists(inliersRef.size());
									for(unsigned int j=0; j<inliersRef.size(); ++j)
									{
										cv::Point3f refPt = inliersRef.at(j);
										refPt.x *= s;
										refPt.y *= s;
										refPt.z *= s;
										const cv::Point3f & newPt = inliersRefGuess.at(j);
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
								std::vector<float> errorSqrdDists(inliersRef.size());
								for(unsigned int j=0; j<inliersRef.size(); ++j)
								{
									const cv::Point3f & refPt = inliersRef.at(j);
									const cv::Point3f & newPt = inliersRefGuess.at(j);
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
								int oi2=0;
								UASSERT(indexes.size() == newCorners.size());
								for(unsigned int i=0; i<indexes.size(); ++i)
								{
									std::map<int, cv::Point3f>::iterator iter = words3D.find(indexes[i]);
									if(iter!=words3D.end() && util3d::isFinite(iter->second))
									{
										iter->second.x *= scale;
										iter->second.y *= scale;
										iter->second.z *= scale;
										objectPoints[oi2].x = iter->second.x;
										objectPoints[oi2].y = iter->second.y;
										objectPoints[oi2].z = iter->second.z;
										imagePoints[oi2] = newCorners[i];
										++oi2;
									}
								}
								objectPoints.resize(oi2);
								imagePoints.resize(oi2);

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
								util3d::solvePnPRansac(
										objectPoints,
										imagePoints,
										K,
										cv::Mat(),
										rvec,
										tvec,
										true,
										pnpIterations,
										pnpReprojError,
										0, // min inliers
										inliersV,
										pnpFlags,
										pnpRefineIterations);

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
