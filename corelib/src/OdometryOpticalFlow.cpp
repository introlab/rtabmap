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

#include "rtabmap/core/Odometry.h"
#include "rtabmap/core/OdometryInfo.h"
#include "rtabmap/core/Features2d.h"
#include "rtabmap/core/util3d_transforms.h"
#include "rtabmap/core/util3d.h"
#include "rtabmap/core/util2d.h"
#include "rtabmap/core/util3d_registration.h"
#include "rtabmap/core/util3d_features.h"
#include "rtabmap/core/Stereo.h"
#include "rtabmap/utilite/ULogger.h"
#include "rtabmap/utilite/UTimer.h"
#include "rtabmap/utilite/UConversion.h"
#include "rtabmap/utilite/UStl.h"
#include "rtabmap/utilite/UMath.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/calib3d/calib3d.hpp>

namespace rtabmap {

OdometryOpticalFlow::OdometryOpticalFlow(const ParametersMap & parameters) :
	Odometry(parameters),
	keyFrameThr_(Parameters::defaultOdomFlowKeyFrameThr()),
	flowWinSize_(Parameters::defaultOdomFlowWinSize()),
	flowIterations_(Parameters::defaultOdomFlowIterations()),
	flowEps_(Parameters::defaultOdomFlowEps()),
	flowMaxLevel_(Parameters::defaultOdomFlowMaxLevel()),
	flowGuessFromMotion_(Parameters::defaultOdomFlowGuessMotion()),
	subPixWinSize_(Parameters::defaultVisSubPixWinSize()),
	subPixIterations_(Parameters::defaultVisSubPixIterations()),
	subPixEps_(Parameters::defaultVisSubPixEps()),
	refCorners3D_(new pcl::PointCloud<pcl::PointXYZ>),
	motionSinceLastKeyFrame_(Transform::getIdentity())
{
	Parameters::parse(parameters, Parameters::kOdomFlowKeyFrameThr(), keyFrameThr_);
	Parameters::parse(parameters, Parameters::kOdomFlowWinSize(), flowWinSize_);
	Parameters::parse(parameters, Parameters::kOdomFlowIterations(), flowIterations_);
	Parameters::parse(parameters, Parameters::kOdomFlowEps(), flowEps_);
	Parameters::parse(parameters, Parameters::kOdomFlowMaxLevel(), flowMaxLevel_);
	Parameters::parse(parameters, Parameters::kOdomFlowGuessMotion(), flowGuessFromMotion_);
	Parameters::parse(parameters, Parameters::kVisSubPixWinSize(), subPixWinSize_);
	Parameters::parse(parameters, Parameters::kVisSubPixIterations(), subPixIterations_);
	Parameters::parse(parameters, Parameters::kVisSubPixEps(), subPixEps_);

	bool stereoOpticalFlow = Parameters::defaultStereoOpticalFlow();
	Parameters::parse(parameters, Parameters::kStereoOpticalFlow(), stereoOpticalFlow);
	if(stereoOpticalFlow)
	{
		stereo_ = new StereoOpticalFlow(parameters);
	}
	else
	{
		stereo_ = new Stereo(parameters);
	}

	ParametersMap::const_iterator iter;
	Feature2D::Type detectorStrategy = (Feature2D::Type)Parameters::defaultVisFeatureType();
	if((iter=parameters.find(Parameters::kVisFeatureType())) != parameters.end())
	{
		detectorStrategy = (Feature2D::Type)std::atoi((*iter).second.c_str());
	}

	ParametersMap customParameters;
	int maxFeatures = Parameters::defaultVisMaxFeatures();
	Parameters::parse(parameters, Parameters::kVisMaxFeatures(), maxFeatures);
	customParameters.insert(ParametersPair(Parameters::kKpWordsPerImage(), uNumber2Str(maxFeatures)));
	// add only feature stuff
	for(ParametersMap::const_iterator iter=parameters.begin(); iter!=parameters.end(); ++iter)
	{
		std::string group = uSplit(iter->first, '/').front();
		if(group.compare("SURF") == 0 ||
			group.compare("SIFT") == 0 ||
			group.compare("BRIEF") == 0 ||
			group.compare("FAST") == 0 ||
			group.compare("ORB") == 0 ||
			group.compare("FREAK") == 0 ||
			group.compare("GFTT") == 0 ||
			group.compare("BRISK") == 0)
		{
			customParameters.insert(*iter);
		}
	}

	feature2D_ = Feature2D::create(detectorStrategy, customParameters);
}

OdometryOpticalFlow::~OdometryOpticalFlow()
{
	delete feature2D_;
	delete stereo_;
}

void OdometryOpticalFlow::reset(const Transform & initialPose)
{
	Odometry::reset(initialPose);
	refFrame_ = cv::Mat();
	refCorners_.clear();
	refCorners3D_->clear();
	motionSinceLastKeyFrame_.setIdentity();
}

// return not null transform if odometry is correctly computed
Transform OdometryOpticalFlow::computeTransform(
		const SensorData & data,
		OdometryInfo * info)
{
	UTimer timer;
	Transform output;
	if(!data.rightRaw().empty() && !data.stereoCameraModel().isValid())
	{
		UERROR("Calibrated stereo camera required");
		return output;
	}
	if(!data.depthRaw().empty() &&
		(data.cameraModels().size() != 1 || !data.cameraModels()[0].isValid()))
	{
		UERROR("Calibrated camera required (multi-cameras not supported).");
		return output;
	}

	double variance = 0;
	int inliers = 0;
	int correspondences = 0;

	if(info)
	{
		info->type = 1;
	}

	cv::Mat newLeftFrame;
	// convert to grayscale
	if(data.imageRaw().channels() > 1)
	{
		cv::cvtColor(data.imageRaw(), newLeftFrame, cv::COLOR_BGR2GRAY);
	}
	else
	{
		newLeftFrame = data.imageRaw().clone();
	}

	std::vector<cv::Point2f> newCorners;
	UDEBUG("lastCorners_.size()=%d lastFrame_=%d depthRight=%d",
			(int)refCorners_.size(), refFrame_.empty()?0:1, data.depthOrRightRaw().empty()?0:1);
	if(!refFrame_.empty() &&
	   ((data.cameraModels().size() == 1 && data.cameraModels()[0].isValid()) || data.stereoCameraModel().isValid()) &&
	   refCorners_.size() &&
	   refCorners3D_->size())
	{
		UASSERT_MSG(refCorners_.size() == refCorners3D_->size(),
				uFormat("%d vs %d", (int)refCorners_.size(), (int)refCorners3D_->size()).c_str());

		// make guess
		cv::Mat K = data.cameraModels().size()?data.cameraModels()[0].K():data.stereoCameraModel().left().K();
		Transform localTransform = data.cameraModels().size()?data.cameraModels()[0].localTransform():data.stereoCameraModel().left().localTransform();
		Transform guess = (motionSinceLastKeyFrame_*this->previousTransform() * localTransform).inverse();
		cv::Mat R = (cv::Mat_<double>(3,3) <<
				(double)guess.r11(), (double)guess.r12(), (double)guess.r13(),
				(double)guess.r21(), (double)guess.r22(), (double)guess.r23(),
				(double)guess.r31(), (double)guess.r32(), (double)guess.r33());
		cv::Mat rvec(1,3, CV_64FC1);
		cv::Rodrigues(R, rvec);
		cv::Mat tvec = (cv::Mat_<double>(1,3) << (double)guess.x(), (double)guess.y(), (double)guess.z());
		std::vector<cv::Point3f> objectPoints(refCorners3D_->size());
		for(unsigned int i=0; i<objectPoints.size(); ++i)
		{
			objectPoints[i].x = refCorners3D_->at(i).x;
			objectPoints[i].y = refCorners3D_->at(i).y;
			objectPoints[i].z = refCorners3D_->at(i).z;
		}
		if(flowGuessFromMotion_ && !(motionSinceLastKeyFrame_*this->previousTransform()).isIdentity())
		{
			UDEBUG("project points to new image");
			cv::projectPoints(objectPoints, rvec, tvec, K, cv::Mat(), newCorners);
		}

		// Find features in the new left image
		std::vector<unsigned char> status;
		std::vector<float> err;
		UDEBUG("cv::calcOpticalFlowPyrLK() begin");
		int winSize = flowWinSize_;
		cv::calcOpticalFlowPyrLK(
				refFrame_,
				newLeftFrame,
				refCorners_,
				newCorners,
				status,
				err,
				cv::Size(winSize, winSize),
				(newCorners.size()||!flowGuessFromMotion_)?flowMaxLevel_:3,
				cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, flowIterations_, flowEps_),
				cv::OPTFLOW_LK_GET_MIN_EIGENVALS | (newCorners.size()?cv::OPTFLOW_USE_INITIAL_FLOW:0), 1e-4);
		UDEBUG("cv::calcOpticalFlowPyrLK() end");

		pcl::PointCloud<pcl::PointXYZ>::Ptr refCorners3DKept(new pcl::PointCloud<pcl::PointXYZ>);
		refCorners3DKept->resize(status.size());
		std::vector<cv::Point3f> objectPointsKept(status.size());
		std::vector<cv::Point2f> refCornersKept(status.size());
		std::vector<cv::Point2f> newCornersKept(status.size());
		int ki = 0;
		for(unsigned int i=0; i<status.size(); ++i)
		{
			if(status[i] &&
			   uIsInBounds(newCorners[i].x, 0.0f, float(data.depthOrRightRaw().cols)) &&
			   uIsInBounds(newCorners[i].y, 0.0f, float(data.depthOrRightRaw().rows)))
			{
				refCorners3DKept->at(ki) = refCorners3D_->at(i);
				objectPointsKept[ki] = objectPoints[i];
				refCornersKept[ki] = refCorners_[i];
				newCornersKept[ki] = newCorners[i];
				++ki;
			}
		}
		refCorners3DKept->resize(ki);
		objectPointsKept.resize(ki);
		refCornersKept.resize(ki);
		newCornersKept.resize(ki);

		correspondences = ki;

		if(correspondences && correspondences >= this->getMinInliers())
		{
			// get new 3D points
			pcl::PointCloud<pcl::PointXYZ>::Ptr newCorners3DKept;
			if(!isVarianceFromInliersCount() || this->getEstimationType() != 1)
			{
				// Don't compute the new 3D points if the variance is not required on PnP estimation
				if(!data.rightRaw().empty())
				{
					// stereo
					std::vector<unsigned char> stereoStatus;
					std::vector<cv::Point2f> rightCorners;
					rightCorners = stereo_->computeCorrespondences(
							newLeftFrame,
							data.rightRaw(),
							newCornersKept,
							stereoStatus);

					if(this->getMaxDepth() > 0.0f)
					{
						UASSERT(status.size() == newCornersKept.size() && status.size() == rightCorners.size());
						for(unsigned int i=0; i<status.size(); ++i)
						{
							if(status[i] != 0)
							{
								float d = data.stereoCameraModel().computeDepth(newCornersKept[i].x - rightCorners[i].x);
								if(this->getMaxDepth() > 0.0f && d > this->getMaxDepth())
								{
									status[i] = 0;
								}
							}
						}
					}

					newCorners3DKept = util3d::generateKeypoints3DStereo(
							newCornersKept,
							rightCorners,
							data.stereoCameraModel(),
							stereoStatus);
				}
				else
				{
					//depth
					std::vector<cv::KeyPoint> newCornersKeptKpt;
					cv::KeyPoint::convert(newCornersKept, newCornersKeptKpt);
					newCorners3DKept = util3d::generateKeypoints3DDepth(
							newCornersKeptKpt,
							data.depthRaw(),
							data.cameraModels());
				}
				UASSERT(newCorners3DKept.get() != 0);
			}

			std::vector<int> inliersV;
			if(this->getEstimationType() == 1) // PnP
			{
				if(this->isInfoDataFilled() && info)
				{
					info->refCorners = refCornersKept;
					info->newCorners = newCornersKept;
				}

				//PnPRansac
				cv::solvePnPRansac(
						objectPointsKept,
						newCornersKept,
						K,
						cv::Mat(),
						rvec,
						tvec,
						true,
						this->getIterations(),
						this->getPnPReprojError(),
#if CV_MAJOR_VERSION < 3
						0, // min inliers
#else
						0.99, // confidence
#endif
						inliersV,
						this->getPnPFlags());

				cv::Rodrigues(rvec, R);
				Transform pnp(R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2), tvec.at<double>(0),
							   R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2), tvec.at<double>(1),
							   R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2), tvec.at<double>(2));

				inliers = (int)inliersV.size();
				if((int)inliersV.size() >= this->getMinInliers())
				{
					// make it incremental
					output = (localTransform * pnp).inverse();

					// compute variance from 3D correspondences error
					variance = 1;
					if(!isVarianceFromInliersCount())
					{
						UASSERT(objectPointsKept.size() == newCorners3DKept->size());
						std::vector<float> errorSqrdDists(inliersV.size());
						int oi = 0;
						for(unsigned int i=0; i<inliersV.size(); ++i)
						{
							if(pcl::isFinite(newCorners3DKept->at(inliersV[i])))
							{
								const cv::Point3f & objPt = objectPointsKept[inliersV[i]];
								pcl::PointXYZ newPt = util3d::transformPoint(newCorners3DKept->at(inliersV[i]), output);
								errorSqrdDists[oi++] = uNormSquared(objPt.x-newPt.x, objPt.y-newPt.y, objPt.z-newPt.z);
							}
						}
						errorSqrdDists.resize(oi);
						if(errorSqrdDists.size())
						{
							std::sort(errorSqrdDists.begin(), errorSqrdDists.end());
							double median_error_sqr = (double)errorSqrdDists[errorSqrdDists.size () >> 1];
							variance = 2.1981 * median_error_sqr;
						}
					}
				}
				else
				{
					UWARN("PnP not enough inliers (%d < %d), rejecting the transform...", (int)inliersV.size(), this->getMinInliers());
				}

				if(this->isInfoDataFilled() && info)
				{
					info->cornerInliers = inliersV;
				}
			}
			else
			{
				// Get 3D correspondences (remove NaN)
				pcl::PointCloud<pcl::PointXYZ>::Ptr correspondencesRef(new pcl::PointCloud<pcl::PointXYZ>);
				pcl::PointCloud<pcl::PointXYZ>::Ptr correspondencesNew(new pcl::PointCloud<pcl::PointXYZ>);
				correspondencesRef->resize(newCornersKept.size());
				correspondencesNew->resize(newCornersKept.size());
				if(this->isInfoDataFilled() && info)
				{
					info->refCorners.resize(newCornersKept.size());
					info->newCorners.resize(newCornersKept.size());
				}
				int oi = 0;
				UASSERT(newCorners3DKept->size() == newCornersKept.size());
				for(unsigned int i=0; i<newCornersKept.size(); ++i)
				{
					if(pcl::isFinite(newCorners3DKept->at(i)) &&
						(this->getMaxDepth() == 0.0f || newCorners3DKept->at(i).z < this->getMaxDepth()))
					{
						//Add 3D correspondences!
						correspondencesRef->at(oi) = refCorners3DKept->at(i);
						correspondencesNew->at(oi) = newCorners3DKept->at(i);

						if(this->isInfoDataFilled() && info)
						{
							info->refCorners[oi] = refCornersKept[i];
							info->newCorners[oi] = newCornersKept[i];
						}
						++oi;
					}
				}
				correspondencesRef->resize(oi);
				correspondencesNew->resize(oi);
				if(this->isInfoDataFilled() && info)
				{
					info->refCorners.resize(oi);
					info->newCorners.resize(oi);
				}
				correspondences = oi;
				UDEBUG("Getting correspondences end, kept %d/%d", correspondences, (int)newCornersKept.size());

				if(correspondences >= this->getMinInliers())
				{
					UTimer timerRANSAC;
					Transform t = util3d::transformFromXYZCorrespondences(
							correspondencesNew,
							correspondencesRef,
							this->getInlierDistance(),
							this->getIterations(),
							this->getRefineIterations()>0, 3.0, this->getRefineIterations(),
							&inliersV,
							&variance);
					UDEBUG("time RANSAC = %fs", timerRANSAC.ticks());

					inliers = (int)inliersV.size();
					if(!t.isNull() && inliers >= this->getMinInliers())
					{
						output = t;
					}
					else
					{
						UWARN("Transform not valid (inliers = %d/%d)", inliers, correspondences);
					}
				}
				else
				{
					UWARN("Not enough correspondences (%d)", correspondences);
				}
			}

			if(this->isInfoDataFilled() && info)
			{
				info->cornerInliers = inliersV;
			}
		}
	}
	else
	{
		//return Identity
		output = Transform::getIdentity();
	}

	newCorners.clear();
	if(!output.isNull())
	{
		output = motionSinceLastKeyFrame_.inverse() * output;

		// new key-frame?
		if(keyFrameThr_ <= 0 || inliers <= keyFrameThr_)
		{
			// Copy or generate new keypoints
			if(data.keypoints().size())
			{
				cv::KeyPoint::convert(data.keypoints(), newCorners);
			}
			else
			{
				// generate kpts
				std::vector<cv::KeyPoint> newKtps;
				cv::Rect roi = Feature2D::computeRoi(newLeftFrame, this->getRoiRatios());
				newKtps = feature2D_->generateKeypoints(newLeftFrame, roi);

				if(newKtps.size())
				{
					cv::KeyPoint::convert(newKtps, newCorners);

					if(subPixWinSize_ > 0 && subPixIterations_ > 0)
					{
						UDEBUG("cv::cornerSubPix() begin");
						cv::cornerSubPix(newLeftFrame, newCorners,
							cv::Size( subPixWinSize_, subPixWinSize_ ),
							cv::Size( -1, -1 ),
							cv::TermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, subPixIterations_, subPixEps_ ) );
						UDEBUG("cv::cornerSubPix() end");
					}
				}
			}

			if((int)newCorners.size() >= this->getMinInliers())
			{
				pcl::PointCloud<pcl::PointXYZ>::Ptr newCorners3D(new pcl::PointCloud<pcl::PointXYZ>);
				newCorners3D->resize(newCorners.size());
				std::vector<cv::Point2f> newCornersFiltered(newCorners.size());
				int oi=0;
				UTimer corner3dTimer;
				if(!data.rightRaw().empty())
				{
					// stereo
					std::vector<unsigned char> stereoStatus;
					std::vector<cv::Point2f> rightCorners;
					rightCorners = stereo_->computeCorrespondences(
							newLeftFrame,
							data.rightRaw(),
							newCorners,
							stereoStatus);

					pcl::PointCloud<pcl::PointXYZ>::Ptr refCorners3DTmp = util3d::generateKeypoints3DStereo(
							newCorners,
							rightCorners,
							data.stereoCameraModel(),
							stereoStatus);

					UASSERT(refCorners3DTmp->size() == newCorners.size());
					for(unsigned int i=0; i<newCorners.size(); ++i)
					{
						if(pcl::isFinite(refCorners3DTmp->at(i)) &&
						  (this->getMaxDepth() <= 0.0f || data.stereoCameraModel().computeDepth(newCorners[i].x - rightCorners[i].x) <= this->getMaxDepth() ))
						{
							newCorners3D->at(oi) = refCorners3DTmp->at(i);
							newCornersFiltered[oi] = newCorners[i];
							++oi;
						}
					}
				}
				else
				{
					// depth
					for(unsigned int i=0; i<newCorners.size(); ++i)
					{
						if(uIsInBounds(newCorners[i].x, 0.0f, float(data.depthRaw().cols)) &&
						   uIsInBounds(newCorners[i].y, 0.0f, float(data.depthRaw().rows)))
						{
							pcl::PointXYZ pt = util3d::projectDepthTo3D(
									data.depthRaw(),
									newCorners[i].x,
									newCorners[i].y,
									data.cameraModels()[0].cx(),
									data.cameraModels()[0].cy(),
									data.cameraModels()[0].fx(),
									data.cameraModels()[0].fy(),
									true);
							if(pcl::isFinite(pt) &&
								pt.z > 0 &&
								(this->getMaxDepth() == 0.0f || pt.z < this->getMaxDepth()))
							{
								newCorners3D->at(oi) = util3d::transformPoint(pt, data.cameraModels()[0].localTransform());
								newCornersFiltered[oi] = newCorners[i];
								++oi;
							}
						}
					}
				}
				UDEBUG("Computing 3d corners = %f s", corner3dTimer.ticks());
				newCornersFiltered.resize(oi);
				newCorners3D->resize(oi);

				if((int)newCornersFiltered.size() >= this->getMinInliers())
				{
					refFrame_ = newLeftFrame;
					refCorners_ = newCornersFiltered;
					refCorners3D_ = newCorners3D;

					//reset motion
					motionSinceLastKeyFrame_.setIdentity();
				}
				else
				{
					UWARN("Too low 3D corners (%d/%d, minCorners=%d), ignoring new frame...",
							(int)newCornersFiltered.size(), (int)refCorners3D_->size(), this->getMinInliers());
					output.setNull();
				}
			}
			else
			{
				UWARN("Too low 2D corners (%d), ignoring new frame...",
						(int)newCorners.size());
				output.setNull();
			}
		}
		else
		{
			motionSinceLastKeyFrame_ *= output;
		}
	}

	if(info)
	{
		info->type = 1;
		info->variance = variance;
		info->inliers = inliers;
		info->features = (int)refCorners_.size();
		info->matches = correspondences;
	}

	UINFO("Odom update time = %fs lost=%s inliers=%d/%d, new corners=%d, transform accepted=%s",
			timer.elapsed(),
			output.isNull()?"true":"false",
			inliers,
			correspondences,
			(int)newCorners.size(),
			!output.isNull()?"true":"false");

	return output;
}

} // namespace rtabmap
