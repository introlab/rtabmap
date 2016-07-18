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

#include <rtabmap/core/OptimizerCVSBA.h>

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
		const std::map<int, Transform> & poses,
		const std::multimap<int, Link> & links,
		const std::map<int, Signature> & signatures)
{
#ifdef RTABMAP_CVSBA
	// run sba optimization
	cvsba::Sba sba;

	// change params if desired
	cvsba::Sba::Params params ;
	params.type = cvsba::Sba::MOTIONSTRUCTURE;
	params.iterations = this->iterations();
	params.minError = this->epsilon();
	params.fixedIntrinsics = 5;
	params.fixedDistortion = 5; // updated below
	params.verbose=ULogger::level() <= ULogger::kInfo;
	sba.setParams(params);

	std::map<int, Transform> frames = poses;

	std::vector<cv::Mat> cameraMatrix(frames.size()); //nframes
	std::vector<cv::Mat> R(frames.size()); //nframes
	std::vector<cv::Mat> T(frames.size()); //nframes
	std::vector<cv::Mat> distCoeffs(frames.size()); //nframes
	std::map<int, int> frameIdToIndex;
	std::map<int, CameraModel> models;
	int oi=0;
	for(std::map<int, Transform>::iterator iter=frames.begin(); iter!=frames.end(); )
	{
		CameraModel model;
		if(uContains(signatures, iter->first))
		{
			if(signatures.at(iter->first).sensorData().cameraModels().size() == 1 && signatures.at(iter->first).sensorData().cameraModels().at(0).isValidForProjection())
			{
				model = signatures.at(iter->first).sensorData().cameraModels()[0];
			}
			else if(signatures.at(iter->first).sensorData().stereoCameraModel().isValidForProjection())
			{
				model = signatures.at(iter->first).sensorData().stereoCameraModel().left();
			}
			else
			{
				UERROR("Missing calibration for node %d", iter->first);
			}
		}
		else
		{
			UERROR("Did not find node %d in cache", iter->first);
		}

		if(model.isValidForProjection())
		{
			frameIdToIndex.insert(std::make_pair(iter->first, oi));

			cameraMatrix[oi] = model.K();
			if(model.D().cols != 5)
			{
				distCoeffs[oi] = cv::Mat::zeros(1, 5, CV_64FC1);
				UWARN("Camera model %d: Distortion coefficients are not 5, setting all them to 0 (assuming no distortion)", iter->first);
			}
			else
			{
				distCoeffs[oi] = model.D();
			}

			Transform t = (iter->second * model.localTransform()).inverse();

			R[oi] = (cv::Mat_<double>(3,3) <<
					(double)t.r11(), (double)t.r12(), (double)t.r13(),
					(double)t.r21(), (double)t.r22(), (double)t.r23(),
					(double)t.r31(), (double)t.r32(), (double)t.r33());
			T[oi] = (cv::Mat_<double>(1,3) << (double)t.x(), (double)t.y(), (double)t.z());
			++oi;

			models.insert(std::make_pair(iter->first, model));

			UDEBUG("Pose %d = %s", iter->first, t.prettyPrint().c_str());

			++iter;
		}
		else
		{
			frames.erase(iter++);
		}
	}
	cameraMatrix.resize(oi);
	R.resize(oi);
	T.resize(oi);
	distCoeffs.resize(oi);

	std::map<int, cv::Point3f> points3DMap;
	std::map<int, std::map<int, cv::Point2f> > wordReferences; // <ID words, IDs frames + keypoint>
	computeBACorrespondences(frames, links, signatures, points3DMap, wordReferences);

	UDEBUG("points=%d frames=%d", (int)wordReferences.size(), (int)frames.size());
	std::vector<cv::Point3f> points(wordReferences.size()); //npoints
	std::vector<std::vector<cv::Point2f> >  imagePoints(frames.size()); //nframes -> npoints
	std::vector<std::vector<int> > visibility(frames.size()); //nframes -> npoints
	for(unsigned int i=0; i<frames.size(); ++i)
	{
		imagePoints[i].resize(wordReferences.size(), cv::Point2f(std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN()));
		visibility[i].resize(wordReferences.size(), 0);
	}
	int i=0;
	for(std::map<int, std::map<int, cv::Point2f> >::iterator iter = wordReferences.begin(); iter!=wordReferences.end(); ++iter)
	{
		points[i] = points3DMap.at(iter->first);

		for(std::map<int, cv::Point2f>::const_iterator jter=iter->second.begin(); jter!=iter->second.end(); ++jter)
		{
			imagePoints[frameIdToIndex.at(jter->first)][i] = jter->second;
			visibility[frameIdToIndex.at(jter->first)][i] = 1;
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
	for(std::map<int, Transform>::iterator iter=frames.begin(); iter!=frames.end(); ++iter)
	{
		Transform t(R[i].at<double>(0,0), R[i].at<double>(0,1), R[i].at<double>(0,2), T[i].at<double>(0),
					R[i].at<double>(1,0), R[i].at<double>(1,1), R[i].at<double>(1,2), T[i].at<double>(1),
					R[i].at<double>(2,0), R[i].at<double>(2,1), R[i].at<double>(2,2), T[i].at<double>(2));

		UDEBUG("New pose %d = %s", iter->first, t.prettyPrint().c_str());

		iter->second = (models.at(iter->first).localTransform() * t).inverse();

		++i;
	}

	return frames;

#else
	UERROR("RTAB-Map is not built with cvsba!");
	return std::map<int, Transform>();
#endif
}

} /* namespace rtabmap */
