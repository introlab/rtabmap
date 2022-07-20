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
#include "rtabmap/core/Graph.h"

#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/UMath.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UTimer.h>
#include <set>

#include <rtabmap/core/optimizer/OptimizerCVSBA.h>

#ifdef RTABMAP_CVSBA
#include <cvsba/cvsba.h>
#include "rtabmap/core/util3d_motion_estimation.h"
#include "rtabmap/core/util3d_transforms.h"
#include "rtabmap/core/util3d_correspondences.h"
#endif

namespace rtabmap {

bool OptimizerCVSBA::available()
{
#ifdef RTABMAP_CVSBA
	return true;
#else
	return false;
#endif
}

std::map<int, Transform> OptimizerCVSBA::optimizeBA(
		int rootId,
		const std::map<int, Transform> & posesIn,
		const std::multimap<int, Link> & links,
		const std::map<int, std::vector<CameraModel> > & models,
		std::map<int, cv::Point3f> & points3DMap,
		const std::map<int, std::map<int, FeatureBA> > & wordReferences, // <ID words, IDs frames + keypoint/Disparity>)
		std::set<int> * outliers)
{
#ifdef RTABMAP_CVSBA
	// run sba optimization
	cvsba::Sba sba;

	std::map<int, Transform> poses(posesIn.lower_bound(1), posesIn.end());

	// change params if desired
	cvsba::Sba::Params params ;
	params.type = cvsba::Sba::MOTIONSTRUCTURE;
	params.iterations = this->iterations();
	params.minError = this->epsilon();
	params.fixedIntrinsics = 5;
	params.fixedDistortion = 5; // updated below
	params.verbose=ULogger::level() <= ULogger::kInfo;
	sba.setParams(params);

	std::vector<cv::Mat> cameraMatrix(poses.size()); //nframes
	std::vector<cv::Mat> R(poses.size()); //nframes
	std::vector<cv::Mat> T(poses.size()); //nframes
	std::vector<cv::Mat> distCoeffs(poses.size()); //nframes
	std::map<int, int> frameIdToIndex;
	int oi=0;
	for(std::map<int, Transform>::const_iterator iter=poses.begin(); iter!=poses.end(); ++iter)
	{
		// Get camera model
		std::map<int, CameraModel>::const_iterator iterModel = models.find(iter->first);
		UASSERT(iterModel != models.end() && iterModel->second.isValidForProjection());

		frameIdToIndex.insert(std::make_pair(iter->first, oi));

		cameraMatrix[oi] = iterModel->second.K();
		if(iterModel->second.D().cols != 5)
		{
			distCoeffs[oi] = cv::Mat::zeros(1, 5, CV_64FC1);
			UWARN("Camera model %d: Distortion coefficients are not 5, setting all them to 0 (assuming no distortion)", iter->first);
		}
		else
		{
			distCoeffs[oi] = iterModel->second.D();
		}

		Transform t = (iter->second * iterModel->second.localTransform()).inverse();

		R[oi] = (cv::Mat_<double>(3,3) <<
				(double)t.r11(), (double)t.r12(), (double)t.r13(),
				(double)t.r21(), (double)t.r22(), (double)t.r23(),
				(double)t.r31(), (double)t.r32(), (double)t.r33());
		T[oi] = (cv::Mat_<double>(1,3) << (double)t.x(), (double)t.y(), (double)t.z());
		++oi;

		UDEBUG("Pose %d = %s", iter->first, t.prettyPrint().c_str());
	}
	cameraMatrix.resize(oi);
	R.resize(oi);
	T.resize(oi);
	distCoeffs.resize(oi);

	UDEBUG("points=%d frames=%d", (int)points3DMap.size(), (int)poses.size());
	std::vector<cv::Point3f> points(points3DMap.size()); //npoints
	std::vector<std::vector<cv::Point2f> >  imagePoints(poses.size()); //nframes -> npoints
	std::vector<std::vector<int> > visibility(poses.size()); //nframes -> npoints
	for(unsigned int i=0; i<poses.size(); ++i)
	{
		imagePoints[i].resize(wordReferences.size(), cv::Point2f(std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN()));
		visibility[i].resize(wordReferences.size(), 0);
	}
	int i=0;
	for(std::map<int, cv::Point3f>::const_iterator kter = points3DMap.begin(); kter!=points3DMap.end(); ++kter)
	{
		points[i] = kter->second;

		std::map<int, std::map<int, FeatureBA> >::const_iterator iter = wordReferences.find(kter->first);
		if(iter != wordReferences.end())
		{
			for(std::map<int, FeatureBA>::const_iterator jter=iter->second.begin(); jter!=iter->second.end(); ++jter)
			{
				if(frameIdToIndex.find(jter->first) != frameIdToIndex.end())
				{
					imagePoints[frameIdToIndex.at(jter->first)][i] = cv::Point2f(jter->second.kpt.pt.x, jter->second.kpt.pt.y);
					visibility[frameIdToIndex.at(jter->first)][i] = 1;
				}
			}
		}
		++i;
	}

	// SBA
	try
	{
		sba.run( points, imagePoints, visibility, cameraMatrix, R, T, distCoeffs);
	}
	catch(cv::Exception & e)
	{
		UERROR("Running SBA... error! %s", e.what());
		return std::map<int, Transform>();
	}

	//update poses
	i=0;
	std::map<int, Transform> newPoses = poses;
	for(std::map<int, Transform>::iterator iter=newPoses.begin(); iter!=newPoses.end(); ++iter)
	{
		Transform t(R[i].at<double>(0,0), R[i].at<double>(0,1), R[i].at<double>(0,2), T[i].at<double>(0),
					R[i].at<double>(1,0), R[i].at<double>(1,1), R[i].at<double>(1,2), T[i].at<double>(1),
					R[i].at<double>(2,0), R[i].at<double>(2,1), R[i].at<double>(2,2), T[i].at<double>(2));

		UDEBUG("New pose %d = %s", iter->first, t.prettyPrint().c_str());

		if(this->isSlam2d())
		{
			t = (models.at(iter->first).localTransform() * t).inverse();
			t = iter->second.inverse() * t;
			iter->second *= t.to3DoF();
		}
		else
		{
			iter->second = (models.at(iter->first).localTransform() * t).inverse();
		}

		++i;
	}

	//update 3D points
	i=0;
	for(std::map<int, cv::Point3f>::iterator kter = points3DMap.begin(); kter!=points3DMap.end(); ++kter)
	{
		kter->second = points[i++];
	}

	return newPoses;

#else
	UERROR("RTAB-Map is not built with cvsba!");
	return std::map<int, Transform>();
#endif
}

} /* namespace rtabmap */
