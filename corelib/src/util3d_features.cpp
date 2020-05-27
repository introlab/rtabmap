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
#include "opencv/five-point.h"

#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UMath.h>
#include <rtabmap/utilite/UStl.h>

#include <pcl/common/point_tests.h>

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
		float rgbToDepthFactorX = 1.0f/(cameraModels[0].imageWidth()>0?float(cameraModels[0].imageWidth())/subImageWidth:1.0f);
		float rgbToDepthFactorY = 1.0f/(cameraModels[0].imageHeight()>0?float(cameraModels[0].imageHeight())/float(depth.rows):1.0f);
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
		float ransacReprojThreshold,
		float ransacConfidence,
		const std::map<int, cv::Point3f> & refGuess3D,
		double * varianceOut,
		std::vector<int> * matchesOut)
{
	UASSERT(cameraModel.isValidForProjection());
	std::map<int, cv::Point3f> words3D;
	std::list<std::pair<int, std::pair<cv::KeyPoint, cv::KeyPoint> > > pairs;
	int pairsFound = EpipolarGeometry::findPairs(refWords, nextWords, pairs);
	UDEBUG("pairsFound=%d/%d", pairsFound, int(refWords.size()>nextWords.size()?refWords.size():nextWords.size()));
	if(pairsFound > 8)
	{
		std::list<std::pair<int, std::pair<cv::KeyPoint, cv::KeyPoint> > >::iterator iter=pairs.begin();
		std::vector<cv::Point2f> refCorners(pairs.size());
		std::vector<cv::Point2f> newCorners(pairs.size());
		std::vector<int> indexes(pairs.size());
		for(unsigned int i=0; i<pairs.size(); ++i)
		{
			if(matchesOut)
			{
				matchesOut->push_back(iter->first);
			}

			refCorners[i] = iter->second.first.pt;
			newCorners[i] = iter->second.second.pt;
			indexes[i] = iter->first;
			++iter;
		}

		std::vector<unsigned char> status;
		cv::Mat pts4D;

		UDEBUG("Five-point algorithm");
		/**
		 * OpenCV five-point algorithm
		 * David Nistér. An efficient solution to the five-point relative pose problem. Pattern Analysis and Machine Intelligence, IEEE Transactions on, 26(6):756–770, 2004.
		 */
		cv::Mat E = cv3::findEssentialMat(refCorners, newCorners, cameraModel.K(), cv::RANSAC, ransacConfidence, ransacReprojThreshold, status);

		int essentialInliers = 0;
		for(size_t i=0; i<status.size();++i)
		{
			if(status[i])
			{
				++essentialInliers;
			}
		}
		Transform cameraTransformGuess = cameraTransform;
		if(!E.empty())
		{
			UDEBUG("essential inliers=%d/%d", essentialInliers, (int)status.size());
			cv::Mat R,t;
			cv3::recoverPose(E, refCorners, newCorners, cameraModel.K(), R, t, 50, status, pts4D);
			if(!R.empty() && !t.empty())
			{
				cv::Mat P = cv::Mat::zeros(3, 4, CV_64FC1);
				R.copyTo(cv::Mat(P, cv::Range(0,3), cv::Range(0,3)));
				P.at<double>(0,3) = t.at<double>(0);
				P.at<double>(1,3) = t.at<double>(1);
				P.at<double>(2,3) = t.at<double>(2);

				cameraTransform = Transform(R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2), t.at<double>(0),
						R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2), t.at<double>(1),
						R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2), t.at<double>(2));
				UDEBUG("t (cam frame)=%s", cameraTransform.prettyPrint().c_str());
				UDEBUG("base->cam=%s", cameraModel.localTransform().prettyPrint().c_str());
				cameraTransform = cameraModel.localTransform() * cameraTransform.inverse() * cameraModel.localTransform().inverse();
				UDEBUG("t (base frame)=%s", cameraTransform.prettyPrint().c_str());

				UASSERT((int)indexes.size() == pts4D.cols && pts4D.rows == 4 && status.size() == indexes.size());
				for(unsigned int i=0; i<indexes.size(); ++i)
				{
					if(status[i])
					{
						pts4D.col(i) /= pts4D.at<double>(3,i);
						if(pts4D.at<double>(2,i) > 0)
						{
							words3D.insert(std::make_pair(indexes[i], util3d::transformPoint(cv::Point3f(pts4D.at<double>(0,i), pts4D.at<double>(1,i), pts4D.at<double>(2,i)), cameraModel.localTransform())));
						}
					}
				}
			}
		}
		else
		{
			UDEBUG("Failed to find essential matrix");
		}

		if(!cameraTransform.isNull())
		{
			UDEBUG("words3D=%d refGuess3D=%d cameraGuess=%s", (int)words3D.size(), (int)refGuess3D.size(), cameraTransformGuess.prettyPrint().c_str());

			// estimate the scale and variance
			float scale = 1.0f;
			if(!cameraTransformGuess.isNull())
			{
				scale = cameraTransformGuess.getNorm()/cameraTransform.getNorm();
			}
			float variance = 1.0f;

			std::vector<cv::Point3f> inliersRef;
			std::vector<cv::Point3f> inliersRefGuess;
			if(!refGuess3D.empty())
			{
				util3d::findCorrespondences(
						words3D,
						refGuess3D,
						inliersRef,
						inliersRefGuess,
						0);
			}

			if(!inliersRef.empty())
			{
				UDEBUG("inliersRef=%d", (int)inliersRef.size());
				if(cameraTransformGuess.isNull())
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
						double median_error_sqr = (double)errorSqrdDists[errorSqrdDists.size () >> 2];
						float var = 2.1981 * median_error_sqr;
						//UDEBUG("scale %d = %f variance = %f", (int)i, s, variance);

						scales.insert(std::make_pair(var, s));
					}
					scale = scales.begin()->second;
					variance = scales.begin()->first;
				}
				else if(!cameraTransformGuess.isNull())
				{
					// use scale from guess
					//compute variance
					std::vector<float> errorSqrdDists(inliersRef.size());
					for(unsigned int j=0; j<inliersRef.size(); ++j)
					{
						cv::Point3f refPt = inliersRef.at(j);
						refPt.x *= scale;
						refPt.y *= scale;
						refPt.z *= scale;
						const cv::Point3f & newPt = inliersRefGuess.at(j);
						errorSqrdDists[j] = uNormSquared(refPt.x-newPt.x, refPt.y-newPt.y, refPt.z-newPt.z);
					}
					std::sort(errorSqrdDists.begin(), errorSqrdDists.end());
					double median_error_sqr = (double)errorSqrdDists[errorSqrdDists.size () >> 2];
					variance = 2.1981 * median_error_sqr;
				}
			}
			else if(!refGuess3D.empty())
			{
				UWARN("Cannot compute variance, no points corresponding between "
						"the generated ref words (%d) and words guess (%d)",
						(int)words3D.size(), (int)refGuess3D.size());
			}

			if(scale!=1.0f)
			{
				// Adjust output transform and points based on scale found
				cameraTransform.x()*=scale;
				cameraTransform.y()*=scale;
				cameraTransform.z()*=scale;

				UASSERT(indexes.size() == newCorners.size());
				for(unsigned int i=0; i<indexes.size(); ++i)
				{
					std::map<int, cv::Point3f>::iterator iter = words3D.find(indexes[i]);
					if(iter!=words3D.end() && util3d::isFinite(iter->second))
					{
						iter->second.x *= scale;
						iter->second.y *= scale;
						iter->second.z *= scale;
					}
				}
			}
			UDEBUG("scale used = %f (variance=%f)", scale, variance);
			if(varianceOut)
			{
				*varianceOut = variance;
			}
		}
	}
	UDEBUG("wordsSet=%d / %d", (int)words3D.size(), pairsFound);

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
