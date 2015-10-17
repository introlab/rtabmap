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
#include "rtabmap/utilite/ULogger.h"
#include "rtabmap/core/util3d_transforms.h"
#include "rtabmap/core/util3d_registration.h"
#include "rtabmap/core/util3d_correspondences.h"

namespace rtabmap
{

namespace util3d
{

#if CV_MAJOR_VERSION >= 3
void solvePnPRansac(cv::InputArray _opoints, cv::InputArray _ipoints,
		cv::InputArray _cameraMatrix, cv::InputArray _distCoeffs,
		cv::OutputArray _rvec, cv::OutputArray _tvec, bool useExtrinsicGuess,
        int iterationsCount, float reprojectionError, int minInliersCount,
        cv::OutputArray _inliers, int flags);
#endif

Transform estimateMotion3DTo2D(
			const std::map<int, pcl::PointXYZ> & words3A,
			const std::map<int, cv::KeyPoint> & words2B,
			const CameraModel & cameraModel,
			int minInliers,
			int iterations,
			double reprojError,
			int flagsPnP,
			const Transform & guess,
			const std::map<int, pcl::PointXYZ> & words3B,
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
	std::vector<int> ids = uKeys(words2B);
	std::vector<cv::Point3f> objectPoints(ids.size());
	std::vector<cv::Point2f> imagePoints(ids.size());
	int oi=0;
	matches.resize(ids.size());
	for(unsigned int i=0; i<ids.size(); ++i)
	{
		if(words3A.find(ids[i]) != words3A.end())
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

#if CV_MAJOR_VERSION >= 3
		solvePnPRansac(
#else
		cv::solvePnPRansac(
#endif
				objectPoints,
				imagePoints,
				K,
				cv::Mat(),
				rvec,
				tvec,
				true,
				iterations,
				reprojError,
				0, // min inliers
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
					std::map<int, pcl::PointXYZ>::const_iterator iter = words3B.find(matches[inliers[i]]);
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
			const std::map<int, pcl::PointXYZ> & words3A,
			const std::map<int, pcl::PointXYZ> & words3B,
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

// Don't know why, but the RANSAC implementation in OpenCV 3 gives me far
// more wrong results than the 2.4x implementation.
// Here is a copy of the RANSAC implementation from OpenCV 2.4.x version
#if CV_MAJOR_VERSION >= 3
namespace pnpransac
{
	const int MIN_POINTS_COUNT = 4;

	static void project3dPoints(const cv::Mat& points, const cv::Mat& rvec, const cv::Mat& tvec, cv::Mat& modif_points)
	{
		modif_points.create(1, points.cols, CV_32FC3);
		cv::Mat R(3, 3, CV_64FC1);
		cv::Rodrigues(rvec, R);
		cv::Mat transformation(3, 4, CV_64F);
		cv::Mat r = transformation.colRange(0, 3);
		R.copyTo(r);
		cv::Mat t = transformation.colRange(3, 4);
		tvec.copyTo(t);
		transform(points, modif_points, transformation);
	}

	struct CameraParameters
	{
		void init(cv::Mat _intrinsics, cv::Mat _distCoeffs)
		{
			_intrinsics.copyTo(intrinsics);
			_distCoeffs.copyTo(distortion);
		}

		cv::Mat intrinsics;
		cv::Mat distortion;
	};

	struct Parameters
	{
		int iterationsCount;
		float reprojectionError;
		int minInliersCount;
		bool useExtrinsicGuess;
		int flags;
		CameraParameters camera;
	};

	template <typename OpointType, typename IpointType>
	static void pnpTask(const int curIndex, const std::vector<char>& pointsMask, const cv::Mat& objectPoints, const cv::Mat& imagePoints,
				 const Parameters& params, std::vector<int>& inliers, int& bestIndex, cv::Mat& rvec, cv::Mat& tvec,
				 const cv::Mat& rvecInit, const cv::Mat& tvecInit, cv::Mutex& resultsMutex)
	{
		cv::Mat modelObjectPoints(1, MIN_POINTS_COUNT, CV_MAKETYPE(cv::DataDepth<OpointType>::value, 3));
		cv::Mat modelImagePoints(1, MIN_POINTS_COUNT, CV_MAKETYPE(cv::DataDepth<IpointType>::value, 2));
		for (int i = 0, colIndex = 0; i < (int)pointsMask.size(); i++)
		{
			if (pointsMask[i])
			{
				cv::Mat colModelImagePoints = modelImagePoints(cv::Rect(colIndex, 0, 1, 1));
				imagePoints.col(i).copyTo(colModelImagePoints);
				cv::Mat colModelObjectPoints = modelObjectPoints(cv::Rect(colIndex, 0, 1, 1));
				objectPoints.col(i).copyTo(colModelObjectPoints);
				colIndex = colIndex+1;
			}
		}

		//filter same 3d points, hang in solvePnP
		double eps = 1e-10;
		int num_same_points = 0;
		for (int i = 0; i < MIN_POINTS_COUNT; i++)
			for (int j = i + 1; j < MIN_POINTS_COUNT; j++)
			{
				if (norm(modelObjectPoints.at<cv::Vec<OpointType,3> >(0, i) - modelObjectPoints.at<cv::Vec<OpointType,3> >(0, j)) < eps)
					num_same_points++;
			}
		if (num_same_points > 0)
			return;

		cv::Mat localRvec, localTvec;
		rvecInit.copyTo(localRvec);
		tvecInit.copyTo(localTvec);

		// OpenCV 3
		cv::solvePnP(
				modelObjectPoints,
				modelImagePoints,
				params.camera.intrinsics,
				params.camera.distortion,
				localRvec,
				localTvec,
				 params.useExtrinsicGuess,
				 params.flags);


		std::vector<cv::Point_<OpointType> > projected_points;
		projected_points.resize(objectPoints.cols);
		projectPoints(objectPoints, localRvec, localTvec, params.camera.intrinsics, params.camera.distortion, projected_points);

		cv::Mat rotatedPoints;
		project3dPoints(objectPoints, localRvec, localTvec, rotatedPoints);

		std::vector<int> localInliers;
		for (int i = 0; i < objectPoints.cols; i++)
		{
			//Although p is a 2D point it needs the same type as the object points to enable the norm calculation
			cv::Point_<OpointType> p((OpointType)imagePoints.at<cv::Vec<IpointType,2> >(0, i)[0],
								 (OpointType)imagePoints.at<cv::Vec<IpointType,2> >(0, i)[1]);
			if ((norm(p - projected_points[i]) < params.reprojectionError)
				&& (rotatedPoints.at<cv::Vec<OpointType,3> >(0, i)[2] > 0)) //hack
			{
				localInliers.push_back(i);
			}
		}

		resultsMutex.lock();
		if ( (localInliers.size() > inliers.size()) || (localInliers.size() == inliers.size() && curIndex > bestIndex))
		{
			inliers.clear();
			inliers.resize(localInliers.size());
			memcpy(&inliers[0], &localInliers[0], sizeof(int) * localInliers.size());
			localRvec.copyTo(rvec);
			localTvec.copyTo(tvec);
			bestIndex = curIndex;
		}
		resultsMutex.unlock();
	}

	static void pnpTask(const int curIndex, const std::vector<char>& pointsMask, const cv::Mat& objectPoints, const cv::Mat& imagePoints,
		const Parameters& params, std::vector<int>& inliers, int& bestIndex, cv::Mat& rvec, cv::Mat& tvec,
		const cv::Mat& rvecInit, const cv::Mat& tvecInit, cv::Mutex& resultsMutex)
	{
		CV_Assert(objectPoints.depth() == CV_64F ||  objectPoints.depth() == CV_32F);
		CV_Assert(imagePoints.depth() == CV_64F ||  imagePoints.depth() == CV_32F);
		const bool objectDoublePrecision = objectPoints.depth() == CV_64F;
		const bool imageDoublePrecision = imagePoints.depth() == CV_64F;
		if(objectDoublePrecision)
		{
			if(imageDoublePrecision)
				pnpTask<double, double>(curIndex, pointsMask, objectPoints, imagePoints, params, inliers, bestIndex, rvec, tvec, rvecInit, tvecInit, resultsMutex);
			else
				pnpTask<double, float>(curIndex, pointsMask, objectPoints, imagePoints, params, inliers, bestIndex, rvec, tvec, rvecInit, tvecInit, resultsMutex);
		}
		else
		{
			if(imageDoublePrecision)
				pnpTask<float, double>(curIndex, pointsMask, objectPoints, imagePoints, params, inliers, bestIndex, rvec, tvec, rvecInit, tvecInit, resultsMutex);
			else
				pnpTask<float, float>(curIndex, pointsMask, objectPoints, imagePoints, params, inliers, bestIndex, rvec, tvec, rvecInit, tvecInit, resultsMutex);
		}
	}

	// TBB removed
	class PnPSolver
	{
	public:
		void operator()(int begin, int end) const
		{
			std::vector<char> pointsMask(objectPoints.cols, 0);
			for( int i=begin; i!=end; ++i )
			{
				memset(&pointsMask[0], 0, objectPoints.cols );
				memset(&pointsMask[0], 1, MIN_POINTS_COUNT );
				generateVar(pointsMask, rng_base_seed + i);
				pnpTask(i, pointsMask, objectPoints, imagePoints, parameters,
						inliers, bestIndex, rvec, tvec, initRvec, initTvec, syncMutex);
				if ((int)inliers.size() >= parameters.minInliersCount)
				{
					break;
				}
			}
		}
		PnPSolver(const cv::Mat& _objectPoints, const cv::Mat& _imagePoints, const Parameters& _parameters,
				cv::Mat& _rvec, cv::Mat& _tvec, std::vector<int>& _inliers, int& _bestIndex, uint64 _rng_base_seed):
		objectPoints(_objectPoints), imagePoints(_imagePoints), parameters(_parameters),
		rvec(_rvec), tvec(_tvec), inliers(_inliers), bestIndex(_bestIndex), rng_base_seed(_rng_base_seed)
		{
			bestIndex = -1;
			rvec.copyTo(initRvec);
			tvec.copyTo(initTvec);
		}
	private:
		PnPSolver& operator=(const PnPSolver&);

		const cv::Mat& objectPoints;
		const cv::Mat& imagePoints;
		const Parameters& parameters;
		cv::Mat &rvec, &tvec;
		std::vector<int>& inliers;
		int& bestIndex;
		const uint64 rng_base_seed;
		cv::Mat initRvec, initTvec;

		static cv::Mutex syncMutex;

	  void generateVar(std::vector<char>& mask, uint64 rng_seed) const
		{
		    cv::RNG generator(rng_seed);
			int size = (int)mask.size();
			for (int i = 0; i < size; i++)
			{
				int i1 = generator.uniform(0, size);
				int i2 = generator.uniform(0, size);
				char curr = mask[i1];
				mask[i1] = mask[i2];
				mask[i2] = curr;
			}
		}
	};

	cv::Mutex PnPSolver::syncMutex;

}

void solvePnPRansac(cv::InputArray _opoints, cv::InputArray _ipoints,
		cv::InputArray _cameraMatrix, cv::InputArray _distCoeffs,
		cv::OutputArray _rvec, cv::OutputArray _tvec, bool useExtrinsicGuess,
        int iterationsCount, float reprojectionError, int minInliersCount,
        cv::OutputArray _inliers, int flags)
{
    const int _rng_seed = 0;
    cv::Mat opoints = _opoints.getMat(), ipoints = _ipoints.getMat();
    cv::Mat cameraMatrix = _cameraMatrix.getMat(), distCoeffs = _distCoeffs.getMat();

    CV_Assert(opoints.isContinuous());
    CV_Assert(opoints.depth() == CV_32F || opoints.depth() == CV_64F);
    CV_Assert((opoints.rows == 1 && opoints.channels() == 3) || opoints.cols*opoints.channels() == 3);
    CV_Assert(ipoints.isContinuous());
    CV_Assert(ipoints.depth() == CV_32F || ipoints.depth() == CV_64F);
    CV_Assert((ipoints.rows == 1 && ipoints.channels() == 2) || ipoints.cols*ipoints.channels() == 2);

    _rvec.create(3, 1, CV_64FC1);
    _tvec.create(3, 1, CV_64FC1);
    cv::Mat rvec = _rvec.getMat();
    cv::Mat tvec = _tvec.getMat();

    cv::Mat objectPoints = opoints.reshape(3, 1), imagePoints = ipoints.reshape(2, 1);

    if (minInliersCount <= 0)
        minInliersCount = objectPoints.cols;
    pnpransac::Parameters params;
    params.iterationsCount = iterationsCount;
    params.minInliersCount = minInliersCount;
    params.reprojectionError = reprojectionError;
    params.useExtrinsicGuess = useExtrinsicGuess;
    params.camera.init(cameraMatrix, distCoeffs);
    params.flags = flags;

    std::vector<int> localInliers;
    cv::Mat localRvec, localTvec;
    rvec.copyTo(localRvec);
    tvec.copyTo(localTvec);
    int bestIndex;

    // TBB not used
    if (objectPoints.cols >= pnpransac::MIN_POINTS_COUNT)
    {
    	pnpransac::PnPSolver solver(objectPoints, imagePoints, params,
							   localRvec, localTvec, localInliers, bestIndex,
							   _rng_seed);
    	solver(0, iterationsCount);
    }

    if (localInliers.size() >= (size_t)pnpransac::MIN_POINTS_COUNT)
    {
        if (flags != CV_P3P)
        {
            int i, pointsCount = (int)localInliers.size();
            cv::Mat inlierObjectPoints(1, pointsCount, CV_MAKE_TYPE(opoints.depth(), 3)), inlierImagePoints(1, pointsCount, CV_MAKE_TYPE(ipoints.depth(), 2));
            for (i = 0; i < pointsCount; i++)
            {
                int index = localInliers[i];
                cv::Mat colInlierImagePoints = inlierImagePoints(cv::Rect(i, 0, 1, 1));
                imagePoints.col(index).copyTo(colInlierImagePoints);
                cv::Mat colInlierObjectPoints = inlierObjectPoints(cv::Rect(i, 0, 1, 1));
                objectPoints.col(index).copyTo(colInlierObjectPoints);
            }
            solvePnP(inlierObjectPoints, inlierImagePoints, params.camera.intrinsics, params.camera.distortion, localRvec, localTvec, false, flags);
        }
        localRvec.copyTo(rvec);
        localTvec.copyTo(tvec);
        if (_inliers.needed())
        	cv::Mat(localInliers).copyTo(_inliers);
    }
    else
    {
        tvec.setTo(cv::Scalar(0));
        cv::Mat R = cv::Mat::eye(3, 3, CV_64F);
        Rodrigues(R, rvec);
        if( _inliers.needed() )
            _inliers.release();
    }
    return;
}
#endif

}

}
