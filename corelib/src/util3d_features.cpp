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
#include <opencv2/features2d/features2d.hpp>
#include <iterator>
#include <limits>
#include <set>

namespace rtabmap
{

namespace util3d
{

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
		int varianceMedianRatio,
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
		// Camera motion (pose of the next camera in the ref camera frame) with unit
		// translation, from recoverPose(). Points and motion are kept in camera frame
		// up to scale until the scale is known, then cameraModel.localTransform() is
		// applied, so that its translation (e.g. a camera lever arm relative to the
		// base frame) is not scaled.
		Transform cameraMotion;
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

				cameraMotion = Transform(R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2), t.at<double>(0),
						R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2), t.at<double>(1),
						R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2), t.at<double>(2)).inverse();
				UDEBUG("t (cam frame)=%s", cameraMotion.prettyPrint().c_str());
				UDEBUG("base->cam=%s", cameraModel.localTransform().prettyPrint().c_str());
				cameraTransform = cameraModel.localTransform() * cameraMotion * cameraModel.localTransform().inverse();
				UDEBUG("t (base frame)=%s", cameraTransform.prettyPrint().c_str());

				UASSERT((int)indexes.size() == pts4D.cols && pts4D.rows == 4 && status.size() == indexes.size());
				for(unsigned int i=0; i<indexes.size(); ++i)
				{
					if(status[i])
					{
						pts4D.col(i) /= pts4D.at<double>(3,i);
						if(pts4D.at<double>(2,i) > 0)
						{
							words3D.insert(std::make_pair(indexes[i], cv::Point3f(pts4D.at<double>(0,i), pts4D.at<double>(1,i), pts4D.at<double>(2,i))));
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
			if(!cameraTransformGuess.isNull() && !cameraMotion.isNull())
			{
				// compare translations in camera frame, cameraMotion has unit translation
				Transform cameraMotionGuess = cameraModel.localTransform().inverse() * cameraTransformGuess * cameraModel.localTransform();
				scale = cameraMotionGuess.getNorm()/cameraMotion.getNorm();
			}
			float variance = 1.0f;

			std::vector<cv::Point3f> inliersRef;
			std::vector<cv::Point3f> inliersRefGuess;
			if(!refGuess3D.empty() && !cameraMotion.isNull())
			{
				// refGuess3D are in base frame, bring them in camera frame like words3D
				std::map<int, cv::Point3f> refGuess3DCam;
				Transform baseToCam = cameraModel.localTransform().inverse();
				for(std::map<int, cv::Point3f>::const_iterator iter=refGuess3D.begin(); iter!=refGuess3D.end(); ++iter)
				{
					if(util3d::isFinite(iter->second) && (iter->second.x != 0 || iter->second.y != 0 || iter->second.z != 0))
					{
						refGuess3DCam.insert(refGuess3DCam.end(), std::make_pair(iter->first, util3d::transformPoint(iter->second, baseToCam)));
					}
				}
				util3d::findCorrespondences(
						words3D,
						refGuess3DCam,
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
						// using z as depth, we are in camera frame
						float s = inliersRefGuess.at(i).z/inliersRef.at(i).z;
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
						double median_error_sqr = (double)errorSqrdDists[errorSqrdDists.size () >> varianceMedianRatio];
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
					double median_error_sqr = (double)errorSqrdDists[errorSqrdDists.size () >> varianceMedianRatio];
					variance = 2.1981 * median_error_sqr;
				}
			}
			else if(!refGuess3D.empty())
			{
				UWARN("Cannot compute variance, no points corresponding between "
						"the generated ref words (%d) and words guess (%d)",
						(int)words3D.size(), (int)refGuess3D.size());
			}

			if(!cameraMotion.isNull())
			{
				// Adjust output transform and points based on scale found, then
				// transform them in base frame
				cameraMotion.x()*=scale;
				cameraMotion.y()*=scale;
				cameraMotion.z()*=scale;
				cameraTransform = cameraModel.localTransform() * cameraMotion * cameraModel.localTransform().inverse();

				for(std::map<int, cv::Point3f>::iterator iter=words3D.begin(); iter!=words3D.end(); ++iter)
				{
					if(util3d::isFinite(iter->second))
					{
						iter->second = util3d::transformPoint(iter->second*scale, cameraModel.localTransform());
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
	UASSERT(wordIds.size() == keypoints.size());
	std::multimap<int, cv::KeyPoint> words;
	std::vector<cv::KeyPoint>::const_iterator kpIter = keypoints.begin();
	for(std::list<int>::const_iterator iter=wordIds.begin(); iter!=wordIds.end(); ++iter)
	{
		words.insert(std::pair<int, cv::KeyPoint >(*iter, *kpIter));
		++kpIter;
	}
	return words;
}

std::map<int, int> uniqueWordObservations(
		const std::multimap<int, int> & words,
		const std::vector<cv::KeyPoint> & keypoints,
		const std::vector<cv::Point3f> & words3,
		int subImageWidth,
		std::multimap<int, int> * allObservations)
{
	UASSERT(words3.empty() || words3.size() == keypoints.size());
	std::map<int, int> unique;
	if(allObservations)
	{
		allObservations->clear();
	}
	for(std::multimap<int, int>::const_iterator iter=words.begin(); iter!=words.end();)
	{
		std::multimap<int, int>::const_iterator end = words.upper_bound(iter->first);
		int best = iter->second;
		if(std::next(iter) != end)
		{
			// Observed several times: by different cameras of the rig, or ambiguous
			bool ambiguous = subImageWidth <= 0;
			std::set<int> cameras;
			float bestDistance = std::numeric_limits<float>::max();
			for(std::multimap<int, int>::const_iterator jter=iter; jter!=end && !ambiguous; ++jter)
			{
				UASSERT(jter->second >= 0 && jter->second < (int)keypoints.size());
				ambiguous = !cameras.insert(int(keypoints[jter->second].pt.x / subImageWidth)).second;
				if(!words3.empty() && util3d::isFinite(words3[jter->second]))
				{
					float distance = (float)cv::norm(words3[jter->second]);
					if(distance < bestDistance)
					{
						bestDistance = distance;
						best = jter->second;
					}
				}
			}
			if(ambiguous)
			{
				iter = end;
				continue;
			}
		}
		unique.insert(unique.end(), std::make_pair(iter->first, best));
		if(allObservations)
		{
			allObservations->insert(iter, end);
		}
		iter = end;
	}
	return unique;
}

namespace {

// Best match of each query descriptor in train, by nearest neighbor distance
// ratio (a single train descriptor is accepted as is), -1 when none.
std::vector<int> nndrMatches(const cv::Mat & query, const cv::Mat & train, cv::BFMatcher & matcher, float nndr)
{
	std::vector<int> best(query.rows, -1);
	std::vector<std::vector<cv::DMatch> > matches;
	matcher.knnMatch(query, train, matches, train.rows>1?2:1);
	UASSERT((int)matches.size() == query.rows);
	for(int i=0; i<query.rows; ++i)
	{
		if(matches[i].size() == 1 ||
		   (matches[i].size() == 2 && matches[i][0].distance < nndr * matches[i][1].distance))
		{
			best[i] = matches[i][0].trainIdx;
		}
	}
	return best;
}

} // namespace

int mergeMultiCameraObservations(
		std::multimap<int, int> & words,
		const std::vector<cv::KeyPoint> & keypoints,
		std::vector<cv::Point3f> & words3,
		const cv::Mat & descriptors,
		const std::vector<CameraModel> & models,
		const std::set<int> & knownWords,
		float nndr,
		float reprojError,
		float minParallax)
{
	if(models.size() < 2 || words.empty())
	{
		return 0;
	}
	UASSERT(descriptors.rows == (int)keypoints.size());
	UASSERT(words3.empty() || words3.size() == keypoints.size());
	UASSERT(models[0].imageWidth() > 0);
	const float subImageWidth = models[0].imageWidth();
	std::vector<Transform> baseToCamera(models.size());
	for(size_t i=0; i<models.size(); ++i)
	{
		UASSERT(models[i].isValidForProjection());
		UASSERT(models[i].imageWidth() == models[0].imageWidth());
		baseToCamera[i] = models[i].localTransform().inverse();
	}

	// The keypoints to match, per camera: the ones of the words not linked yet
	std::vector<std::vector<int> > candidates(models.size());
	std::vector<int> wordOf(keypoints.size(), 0);
	for(std::multimap<int, int>::const_iterator iter=words.begin(); iter!=words.end(); ++iter)
	{
		UASSERT(iter->second >= 0 && iter->second < (int)keypoints.size());
		wordOf[iter->second] = iter->first;
		if(knownWords.find(iter->first) == knownWords.end())
		{
			int camera = int(keypoints[iter->second].pt.x / subImageWidth);
			UASSERT(camera >= 0 && camera < (int)models.size());
			candidates[camera].push_back(iter->second);
		}
	}

	// Does a point of the base frame reproject on this keypoint?
	const float reprojErrorSqrd = reprojError * reprojError;
	auto reprojects = [&](const cv::Point3f & pt, int camera, int index)
	{
		cv::Point3f inCamera = util3d::transformPoint(pt, baseToCamera[camera]);
		if(inCamera.z <= 0.0f)
		{
			return false;
		}
		float u, v;
		models[camera].reproject(inCamera.x, inCamera.y, inCamera.z, u, v);
		float du = u + float(camera)*subImageWidth - keypoints[index].pt.x;
		float dv = v - keypoints[index].pt.y;
		return du*du + dv*dv <= reprojErrorSqrd;
	};
	// The ray of a keypoint, in the base frame
	auto ray = [&](int camera, int index, cv::Point3f & origin, cv::Point3f & direction)
	{
		float x, y, z;
		models[camera].project(keypoints[index].pt.x - float(camera)*subImageWidth, keypoints[index].pt.y, 1.0f, x, y, z);
		direction = util3d::transformPoint(cv::Point3f(x, y, z), models[camera].localTransform());
		origin = util3d::transformPoint(cv::Point3f(0, 0, 0), models[camera].localTransform());
		direction -= origin;
		direction *= 1.0f/(float)cv::norm(direction);
	};

	std::map<int, int> remap; // dropped word ID -> word ID kept
	auto resolve = [&](int id)
	{
		for(std::map<int, int>::const_iterator iter=remap.find(id); iter!=remap.end(); iter=remap.find(id))
		{
			id = iter->second;
		}
		return id;
	};

	const float nan = std::numeric_limits<float>::quiet_NaN();
	cv::BFMatcher matcher(descriptors.type()==CV_8U?cv::NORM_HAMMING:cv::NORM_L2SQR);
	int merged = 0;
	for(size_t a=0; a<models.size(); ++a)
	{
		for(size_t b=a+1; b<models.size(); ++b)
		{
			if(candidates[a].empty() || candidates[b].empty())
			{
				continue;
			}
			cv::Mat descriptorsA((int)candidates[a].size(), descriptors.cols, descriptors.type());
			for(size_t i=0; i<candidates[a].size(); ++i)
			{
				descriptors.row(candidates[a][i]).copyTo(descriptorsA.row((int)i));
			}
			cv::Mat descriptorsB((int)candidates[b].size(), descriptors.cols, descriptors.type());
			for(size_t i=0; i<candidates[b].size(); ++i)
			{
				descriptors.row(candidates[b][i]).copyTo(descriptorsB.row((int)i));
			}
			std::vector<int> bestAB = nndrMatches(descriptorsA, descriptorsB, matcher, nndr);
			std::vector<int> bestBA = nndrMatches(descriptorsB, descriptorsA, matcher, nndr);

			for(size_t i=0; i<bestAB.size(); ++i)
			{
				int j = bestAB[i];
				if(j < 0 || bestBA[j] != (int)i)
				{
					continue; // not mutual
				}
				int indexA = candidates[a][i];
				int indexB = candidates[b][j];
				int idA = resolve(wordOf[indexA]);
				int idB = resolve(wordOf[indexB]);
				if(idA == idB)
				{
					continue; // already the same word
				}

				bool finiteA = !words3.empty() && util3d::isFinite(words3[indexA]);
				bool finiteB = !words3.empty() && util3d::isFinite(words3[indexB]);
				cv::Point3f pointA(nan, nan, nan); // the depth given to A by the merge
				cv::Point3f pointB(nan, nan, nan);
				bool consistent = false;
				if(finiteA && finiteB)
				{
					consistent = reprojects(words3[indexA], (int)b, indexB) && reprojects(words3[indexB], (int)a, indexA);
				}
				else if(finiteA)
				{
					consistent = reprojects(words3[indexA], (int)b, indexB);
					pointB = words3[indexA];
				}
				else if(finiteB)
				{
					consistent = reprojects(words3[indexB], (int)a, indexA);
					pointA = words3[indexB];
				}
				else
				{
					// Triangulate: the two rays should intersect
					cv::Point3f originA, directionA, originB, directionB;
					ray((int)a, indexA, originA, directionA);
					ray((int)b, indexB, originB, directionB);
					float cosParallax = directionA.dot(directionB);
					if(cosParallax < std::cos(minParallax))
					{
						// Closest points of the two rays (unit directions, so a=c=1)
						cv::Point3f w = originA - originB;
						float d = directionA.dot(w);
						float e = directionB.dot(w);
						float denominator = 1.0f - cosParallax*cosParallax;
						float s = (cosParallax*e - d) / denominator;
						float t = (e - cosParallax*d) / denominator;
						if(s > 0.0f && t > 0.0f)
						{
							cv::Point3f pt = (originA + directionA*s + originB + directionB*t) * 0.5f;
							consistent = reprojects(pt, (int)a, indexA) && reprojects(pt, (int)b, indexB);
							pointA = pointB = pt;
						}
					}
					else
					{
						// Nearly parallel rays (feature far away): check that each ray
						// is seen along the other one, the depth stays unknown
						const float far = 100.0f; // meters
						consistent = reprojects(originA + directionA*far, (int)b, indexB) &&
									 reprojects(originB + directionB*far, (int)a, indexA);
					}
				}
				if(!consistent)
				{
					continue;
				}

				remap.insert(std::make_pair(std::max(idA, idB), std::min(idA, idB)));
				++merged;
				if(util3d::isFinite(pointA) || util3d::isFinite(pointB))
				{
					if(words3.empty())
					{
						words3.resize(keypoints.size(), cv::Point3f(nan, nan, nan));
					}
					if(util3d::isFinite(pointA))
					{
						words3[indexA] = pointA;
					}
					if(util3d::isFinite(pointB))
					{
						words3[indexB] = pointB;
					}
				}
			}
		}
	}

	if(merged)
	{
		std::multimap<int, int> updated;
		for(std::multimap<int, int>::const_iterator iter=words.begin(); iter!=words.end(); ++iter)
		{
			updated.insert(std::make_pair(resolve(iter->first), iter->second));
		}
		words = updated;
	}
	return merged;
}

}

}
