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
#include "rtabmap/core/util3d_registration.h"
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
	flowWinSize_(Parameters::defaultOdomFlowWinSize()),
	flowIterations_(Parameters::defaultOdomFlowIterations()),
	flowEps_(Parameters::defaultOdomFlowEps()),
	flowMaxLevel_(Parameters::defaultOdomFlowMaxLevel()),
	stereoWinSize_(Parameters::defaultStereoWinSize()),
	stereoIterations_(Parameters::defaultStereoIterations()),
	stereoEps_(Parameters::defaultStereoEps()),
	stereoMaxLevel_(Parameters::defaultStereoMaxLevel()),
	stereoMaxSlope_(Parameters::defaultStereoMaxSlope()),
	subPixWinSize_(Parameters::defaultOdomSubPixWinSize()),
	subPixIterations_(Parameters::defaultOdomSubPixIterations()),
	subPixEps_(Parameters::defaultOdomSubPixEps()),
	refCorners3D_(new pcl::PointCloud<pcl::PointXYZ>)
{
	Parameters::parse(parameters, Parameters::kOdomFlowWinSize(), flowWinSize_);
	Parameters::parse(parameters, Parameters::kOdomFlowIterations(), flowIterations_);
	Parameters::parse(parameters, Parameters::kOdomFlowEps(), flowEps_);
	Parameters::parse(parameters, Parameters::kOdomFlowMaxLevel(), flowMaxLevel_);
	Parameters::parse(parameters, Parameters::kStereoWinSize(), stereoWinSize_);
	Parameters::parse(parameters, Parameters::kStereoIterations(), stereoIterations_);
	Parameters::parse(parameters, Parameters::kStereoEps(), stereoEps_);
	Parameters::parse(parameters, Parameters::kStereoMaxLevel(), stereoMaxLevel_);
	Parameters::parse(parameters, Parameters::kStereoMaxSlope(), stereoMaxSlope_);
	Parameters::parse(parameters, Parameters::kOdomSubPixWinSize(), subPixWinSize_);
	Parameters::parse(parameters, Parameters::kOdomSubPixIterations(), subPixIterations_);
	Parameters::parse(parameters, Parameters::kOdomSubPixEps(), subPixEps_);

	ParametersMap::const_iterator iter;
	Feature2D::Type detectorStrategy = (Feature2D::Type)Parameters::defaultOdomFeatureType();
	if((iter=parameters.find(Parameters::kOdomFeatureType())) != parameters.end())
	{
		detectorStrategy = (Feature2D::Type)std::atoi((*iter).second.c_str());
	}

	ParametersMap customParameters;
	int maxFeatures = Parameters::defaultOdomMaxFeatures();
	Parameters::parse(parameters, Parameters::kOdomMaxFeatures(), maxFeatures);
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
}


void OdometryOpticalFlow::reset(const Transform & initialPose)
{
	Odometry::reset(initialPose);
	refFrame_ = cv::Mat();
	refCorners_.clear();
	refCorners3D_->clear();
}

// return not null transform if odometry is correctly computed
Transform OdometryOpticalFlow::computeTransform(
		const SensorData & data,
		OdometryInfo * info)
{
	UDEBUG("");

	if(info)
	{
		info->type = 1;
	}

	if(!data.rightImage().empty())
	{
		//stereo
		return computeTransformStereo(data, info);
	}
	else
	{
		//rgbd
		return computeTransformRGBD(data, info);
	}
}

Transform OdometryOpticalFlow::computeTransformStereo(
		const SensorData & data,
		OdometryInfo * info)
{
	UTimer timer;
	Transform output;

	double variance = 0;
	int inliers = 0;
	int correspondences = 0;

	cv::Mat newLeftFrame;
	// convert to grayscale
	if(data.image().channels() > 1)
	{
		cv::cvtColor(data.image(), newLeftFrame, cv::COLOR_BGR2GRAY);
	}
	else
	{
		newLeftFrame = data.image().clone();
	}
	cv::Mat newRightFrame = data.rightImage().clone();

	std::vector<cv::Point2f> newCorners;
	UDEBUG("lastCorners_.size()=%d lastFrame_=%d lastRightFrame_=%d", (int)refCorners_.size(), refFrame_.empty()?0:1, refRightFrame_.empty()?0:1);
	if(!refFrame_.empty() && !refRightFrame_.empty() && refCorners_.size())
	{
		UDEBUG("");
		// Find features in the new left image
		std::vector<unsigned char> status;
		std::vector<float> err;
		UDEBUG("cv::calcOpticalFlowPyrLK() begin");
		cv::calcOpticalFlowPyrLK(
				refFrame_,
				newLeftFrame,
				refCorners_,
				newCorners,
				status,
				err,
				cv::Size(flowWinSize_, flowWinSize_), flowMaxLevel_,
				cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, flowIterations_, flowEps_),
				cv::OPTFLOW_LK_GET_MIN_EIGENVALS, 1e-4);
		UDEBUG("cv::calcOpticalFlowPyrLK() end");

		std::vector<cv::Point2f> lastCornersKept(status.size());
		std::vector<cv::Point2f> newCornersKept(status.size());
		int ki = 0;
		for(unsigned int i=0; i<status.size(); ++i)
		{
			if(status[i])
			{
				lastCornersKept[ki] = refCorners_[i];
				newCornersKept[ki] = newCorners[i];
				++ki;
			}
		}
		lastCornersKept.resize(ki);
		newCornersKept.resize(ki);

		if(ki && ki >= this->getMinInliers())
		{
			std::vector<unsigned char> statusLast;
			std::vector<float> errLast;
			std::vector<cv::Point2f> lastCornersKeptRight;
			UDEBUG("previous stereo disparity");
			cv::calcOpticalFlowPyrLK(
						refFrame_,
						refRightFrame_,
						lastCornersKept,
						lastCornersKeptRight,
						statusLast,
						errLast,
						cv::Size(stereoWinSize_, stereoWinSize_), stereoMaxLevel_,
						cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, stereoIterations_, stereoEps_),
						cv::OPTFLOW_LK_GET_MIN_EIGENVALS, 1e-4);

			UDEBUG("new stereo disparity");
			std::vector<unsigned char> statusNew;
			std::vector<float> errNew;
			std::vector<cv::Point2f> newCornersKeptRight;
			cv::calcOpticalFlowPyrLK(
						newLeftFrame,
						newRightFrame,
						newCornersKept,
						newCornersKeptRight,
						statusNew,
						errNew,
						cv::Size(stereoWinSize_, stereoWinSize_), stereoMaxLevel_,
						cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, stereoIterations_, stereoEps_),
						cv::OPTFLOW_LK_GET_MIN_EIGENVALS, 1e-4);

			if(this->isPnPEstimationUsed())
			{
				// find correspondences
				if(this->isInfoDataFilled() && info)
				{
					info->refCorners.resize(statusLast.size());
					info->newCorners.resize(statusLast.size());
				}

				int flowInliers = 0;
				std::vector<cv::Point3f> objectPoints(statusLast.size());
				std::vector<cv::Point2f> imagePoints(statusLast.size());
				std::vector<pcl::PointXYZ> image3DPoints(statusLast.size());
				int oi=0;
				float bad_point = std::numeric_limits<float>::quiet_NaN ();
				for(unsigned int i=0; i<statusLast.size(); ++i)
				{
					if(statusLast[i])
					{
						float lastDisparity = lastCornersKept[i].x - lastCornersKeptRight[i].x;
						float lastSlope = fabs((lastCornersKept[i].y-lastCornersKeptRight[i].y) / (lastCornersKept[i].x-lastCornersKeptRight[i].x));
						float newDisparity = newCornersKept[i].x - newCornersKeptRight[i].x;
						float newSlope = fabs((newCornersKept[i].y-newCornersKeptRight[i].y) / (newCornersKept[i].x-newCornersKeptRight[i].x));
						if(lastDisparity > 0.0f && lastSlope < stereoMaxSlope_)
						{
							pcl::PointXYZ lastPt3D = util3d::projectDisparityTo3D(
									lastCornersKept[i],
									lastDisparity,
									data.cx(), data.cy(), data.fx(), data.baseline());

							if(pcl::isFinite(lastPt3D) &&
							   (this->getMaxDepth() == 0.0f || uIsInBounds(lastPt3D.z, 0.0f, this->getMaxDepth())))
							{
								//Add 3D correspondences!
								lastPt3D = util3d::transformPoint(lastPt3D, data.localTransform());
								objectPoints[oi].x = lastPt3D.x;
								objectPoints[oi].y = lastPt3D.y;
								objectPoints[oi].z = lastPt3D.z;
								imagePoints[oi] = newCornersKept.at(i);

								// new 3D points, used to compute variance
								image3DPoints[oi] = pcl::PointXYZ(bad_point, bad_point, bad_point);
								if(newDisparity > 0.0f && newSlope < stereoMaxSlope_)
								{
									pcl::PointXYZ newPt3D = util3d::projectDisparityTo3D(
											newCornersKept[i],
											newDisparity,
											data.cx(), data.cy(), data.fx(), data.baseline());
									if(pcl::isFinite(newPt3D) &&
									   (this->getMaxDepth() == 0.0f || uIsInBounds(newPt3D.z, 0.0f, this->getMaxDepth())))
									{
										image3DPoints[oi] = util3d::transformPoint(newPt3D, data.localTransform());
									}
								}

								if(this->isInfoDataFilled() && info)
								{
									info->refCorners[oi] = lastCornersKept[i];
									info->newCorners[oi] = newCornersKept[i];
								}
								++oi;
							}
						}
						++flowInliers;
					}
				}
				objectPoints.resize(oi);
				imagePoints.resize(oi);
				image3DPoints.resize(oi);
				UDEBUG("Flow inliers = %d, added inliers=%d", flowInliers, oi);

				if(this->isInfoDataFilled() && info)
				{
					info->refCorners.resize(oi);
					info->newCorners.resize(oi);
				}

				correspondences = oi;

				if(correspondences >= this->getMinInliers())
				{
					//PnPRansac
					cv::Mat K = (cv::Mat_<double>(3,3) <<
						data.fx(), 0, data.cx(),
						0, data.fx(), data.cy(),
						0, 0, 1);
					Transform guess = (data.localTransform()).inverse();
					cv::Mat R = (cv::Mat_<double>(3,3) <<
							(double)guess.r11(), (double)guess.r12(), (double)guess.r13(),
							(double)guess.r21(), (double)guess.r22(), (double)guess.r23(),
							(double)guess.r31(), (double)guess.r32(), (double)guess.r33());
					cv::Mat rvec(1,3, CV_64FC1);
					cv::Rodrigues(R, rvec);
					cv::Mat tvec = (cv::Mat_<double>(1,3) << (double)guess.x(), (double)guess.y(), (double)guess.z());
					std::vector<int> inliersV;
					cv::solvePnPRansac(objectPoints,
							imagePoints,
							K,
							cv::Mat(),
							rvec,
							tvec,
							true,
							this->getIterations(),
							this->getPnPReprojError(),
							0,
							inliersV,
							this->getPnPFlags());

					inliers = (int)inliersV.size();
					if((int)inliersV.size() >= this->getMinInliers())
					{
						cv::Rodrigues(rvec, R);
						Transform pnp(R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2), tvec.at<double>(0),
									   R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2), tvec.at<double>(1),
									   R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2), tvec.at<double>(2));

						// make it incremental
						output = (data.localTransform() * pnp).inverse();

						UDEBUG("Odom transform = %s", output.prettyPrint().c_str());

						// compute variance (like in PCL computeVariance() method of sac_model.h)
						std::vector<float> errorSqrdDists(inliersV.size());
						int ii=0;
						for(unsigned int i=0; i<inliersV.size(); ++i)
						{
							pcl::PointXYZ & newPt = image3DPoints[inliersV[i]];
							if(pcl::isFinite(newPt))
							{
								newPt = util3d::transformPoint(newPt, output);
								const cv::Point3f & objPt = objectPoints[inliersV[i]];
								errorSqrdDists[ii++] = uNormSquared(objPt.x-newPt.x, objPt.y-newPt.y, objPt.z-newPt.z);
							}
						}
						errorSqrdDists.resize(ii);
						if(errorSqrdDists.size())
						{
							std::sort(errorSqrdDists.begin(), errorSqrdDists.end());
							double median_error_sqr = (double)errorSqrdDists[errorSqrdDists.size () >> 1];
							variance = 2.1981 * median_error_sqr;
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
					UWARN("Not enough correspondences (%d < %d)", correspondences, this->getMinInliers());
				}
			}
			else
			{
				UDEBUG("Getting correspondences begin");
				// Get 3D correspondences
				pcl::PointCloud<pcl::PointXYZ>::Ptr correspondencesLast(new pcl::PointCloud<pcl::PointXYZ>);
				pcl::PointCloud<pcl::PointXYZ>::Ptr correspondencesNew(new pcl::PointCloud<pcl::PointXYZ>);
				correspondencesLast->resize(statusLast.size());
				correspondencesNew->resize(statusLast.size());
				int oi = 0;
				if(this->isInfoDataFilled() && info)
				{
					info->refCorners.resize(statusLast.size());
					info->newCorners.resize(statusLast.size());
				}
				for(unsigned int i=0; i<statusLast.size(); ++i)
				{
					if(statusLast[i] && statusNew[i])
					{
						float lastDisparity = lastCornersKept[i].x - lastCornersKeptRight[i].x;
						float newDisparity = newCornersKept[i].x - newCornersKeptRight[i].x;
						float lastSlope = fabs((lastCornersKept[i].y-lastCornersKeptRight[i].y) / (lastCornersKept[i].x-lastCornersKeptRight[i].x));
						float newSlope = fabs((newCornersKept[i].y-newCornersKeptRight[i].y) / (newCornersKept[i].x-newCornersKeptRight[i].x));
						if(lastDisparity > 0.0f && newDisparity > 0.0f &&
							lastSlope < stereoMaxSlope_ && newSlope < stereoMaxSlope_)
						{
							pcl::PointXYZ lastPt3D = util3d::projectDisparityTo3D(
									lastCornersKept[i],
									lastDisparity,
									data.cx(), data.cy(), data.fx(), data.baseline());
							pcl::PointXYZ newPt3D = util3d::projectDisparityTo3D(
									newCornersKept[i],
									newDisparity,
									data.cx(), data.cy(), data.fx(), data.baseline());

							if(pcl::isFinite(lastPt3D) && (this->getMaxDepth() == 0.0f || uIsInBounds(lastPt3D.z, 0.0f, this->getMaxDepth())) &&
							   pcl::isFinite(newPt3D) && (this->getMaxDepth() == 0.0f || uIsInBounds(newPt3D.z, 0.0f, this->getMaxDepth())))
							{
								//Add 3D correspondences!
								lastPt3D = util3d::transformPoint(lastPt3D, data.localTransform());
								newPt3D = util3d::transformPoint(newPt3D, data.localTransform());
								correspondencesLast->at(oi) = lastPt3D;
								correspondencesNew->at(oi) = newPt3D;
								if(this->isInfoDataFilled() && info)
								{
									info->refCorners[oi] = lastCornersKept[i];
									info->newCorners[oi] = newCornersKept[i];
								}
								++oi;
							}
						}
					}
				}// end loop
				correspondencesLast->resize(oi);
				correspondencesNew->resize(oi);
				if(this->isInfoDataFilled() && info)
				{
					info->refCorners.resize(oi);
					info->newCorners.resize(oi);
				}
				correspondences = oi;
				refCorners3D_ = correspondencesNew;
				UDEBUG("Getting correspondences end, kept %d/%d", correspondences, (int)statusLast.size());

				if(correspondences >= this->getMinInliers())
				{
					std::vector<int> inliersV;
					UTimer timerRANSAC;
					Transform t = util3d::transformFromXYZCorrespondences(
							correspondencesNew,
							correspondencesLast,
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

					if(this->isInfoDataFilled() && info)
					{
						info->cornerInliers = inliersV;
					}
				}
				else
				{
					UWARN("Not enough correspondences (%d)", correspondences);
				}
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
		// Copy or generate new keypoints
		if(data.keypoints().size())
		{
			newCorners.resize(data.keypoints().size());
			for(unsigned int i=0; i<data.keypoints().size(); ++i)
			{
				newCorners[i] = data.keypoints().at(i).pt;
			}
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

		if((int)newCorners.size() > this->getMinInliers())
		{
			refFrame_ = newLeftFrame;
			refRightFrame_ = newRightFrame;
			refCorners_ = newCorners;
		}
		else
		{
			UWARN("Too low 2D corners (%d), ignoring new frame...",
					(int)newCorners.size());
			output.setNull();
		}
	}

	if(info)
	{
		info->type = 1;
		info->variance = variance;
		info->inliers = inliers;
		info->features = (int)newCorners.size();
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

Transform OdometryOpticalFlow::computeTransformRGBD(
		const SensorData & data,
		OdometryInfo * info)
{
	UTimer timer;
	Transform output;

	double variance = 0;
	int inliers = 0;
	int correspondences = 0;

	cv::Mat newFrame;
	// convert to grayscale
	if(data.image().channels() > 1)
	{
		cv::cvtColor(data.image(), newFrame, cv::COLOR_BGR2GRAY);
	}
	else
	{
		newFrame = data.image().clone();
	}

	std::vector<cv::Point2f> newCorners;
	if(!refFrame_.empty() &&
		(int)refCorners_.size() >= this->getMinInliers() &&
		(int)refCorners3D_->size() >= this->getMinInliers())
	{
		std::vector<unsigned char> status;
		std::vector<float> err;
		UDEBUG("cv::calcOpticalFlowPyrLK() begin");
		cv::calcOpticalFlowPyrLK(
				refFrame_,
				newFrame,
				refCorners_,
				newCorners,
				status,
				err,
				cv::Size(flowWinSize_, flowWinSize_), flowMaxLevel_,
				cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, flowIterations_, flowEps_),
				cv::OPTFLOW_LK_GET_MIN_EIGENVALS, 1e-4);
		UDEBUG("cv::calcOpticalFlowPyrLK() end");

		if(this->isPnPEstimationUsed())
		{
			// find correspondences
			if(this->isInfoDataFilled() && info)
			{
				info->refCorners.resize(refCorners_.size());
				info->newCorners.resize(refCorners_.size());
			}

			UASSERT(refCorners_.size() == refCorners3D_->size());
			UDEBUG("lastCorners3D_ = %d", refCorners3D_->size());
			int flowInliers = 0;
			std::vector<cv::Point3f> objectPoints(refCorners_.size());
			std::vector<cv::Point2f> imagePoints(refCorners_.size());
			std::vector<pcl::PointXYZ> image3DPoints(refCorners_.size());
			int oi=0;
			float bad_point = std::numeric_limits<float>::quiet_NaN ();
			for(unsigned int i=0; i<status.size(); ++i)
			{
				if(status[i])
				{
					if(pcl::isFinite(refCorners3D_->at(i)))
					{
						objectPoints[oi].x = refCorners3D_->at(i).x;
						objectPoints[oi].y = refCorners3D_->at(i).y;
						objectPoints[oi].z = refCorners3D_->at(i).z;
						imagePoints[oi] = newCorners.at(i);

						// new 3D points, used to compute variance
						image3DPoints[oi] = pcl::PointXYZ(bad_point, bad_point, bad_point);
						if(uIsInBounds(newCorners[i].x, 0.0f, float(data.depth().cols)) &&
						   uIsInBounds(newCorners[i].y, 0.0f, float(data.depth().rows)))
						{
							pcl::PointXYZ pt = util3d::projectDepthTo3D(data.depth(), newCorners[i].x, newCorners[i].y,
									data.cx(), data.cy(), data.fx(), data.fy(), true);
							if(pcl::isFinite(pt) &&
								(this->getMaxDepth() == 0.0f || (
								uIsInBounds(pt.x, -this->getMaxDepth(), this->getMaxDepth()) &&
								uIsInBounds(pt.y, -this->getMaxDepth(), this->getMaxDepth()) &&
								uIsInBounds(pt.z, 0.0f, this->getMaxDepth()))))
							{
								image3DPoints[oi] = util3d::transformPoint(pt, data.localTransform());
							}
						}

						if(this->isInfoDataFilled() && info)
						{
							info->refCorners[oi] = refCorners_[i];
							info->newCorners[oi] = newCorners[i];
						}

						++oi;
					}
					++flowInliers;
				}
			}
			objectPoints.resize(oi);
			imagePoints.resize(oi);
			image3DPoints.resize(oi);
			UDEBUG("Flow inliers = %d, added inliers=%d", flowInliers, oi);

			if(this->isInfoDataFilled() && info)
			{
				info->refCorners.resize(oi);
				info->newCorners.resize(oi);
			}

			correspondences = oi;

			if(correspondences >= this->getMinInliers())
			{
				//PnPRansac
				cv::Mat K = (cv::Mat_<double>(3,3) <<
					data.fx(), 0, data.cx(),
					0, data.fy(), data.cy(),
					0, 0, 1);
				Transform guess = (data.localTransform()).inverse();
				cv::Mat R = (cv::Mat_<double>(3,3) <<
						(double)guess.r11(), (double)guess.r12(), (double)guess.r13(),
						(double)guess.r21(), (double)guess.r22(), (double)guess.r23(),
						(double)guess.r31(), (double)guess.r32(), (double)guess.r33());
				cv::Mat rvec(1,3, CV_64FC1);
				cv::Rodrigues(R, rvec);
				cv::Mat tvec = (cv::Mat_<double>(1,3) << (double)guess.x(), (double)guess.y(), (double)guess.z());
				std::vector<int> inliersV;
				cv::solvePnPRansac(objectPoints,
						imagePoints,
						K,
						cv::Mat(),
						rvec,
						tvec,
						true,
						this->getIterations(),
						this->getPnPReprojError(),
						0,
						inliersV,
						this->getPnPFlags());

				inliers = (int)inliersV.size();
				if((int)inliersV.size() >= this->getMinInliers())
				{
					cv::Rodrigues(rvec, R);
					Transform pnp(R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2), tvec.at<double>(0),
								   R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2), tvec.at<double>(1),
								   R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2), tvec.at<double>(2));

					// make it incremental
					output = (data.localTransform() * pnp).inverse();

					UDEBUG("Odom transform = %s", output.prettyPrint().c_str());

					// compute variance (like in PCL computeVariance() method of sac_model.h)
					std::vector<float> errorSqrdDists(inliersV.size());
					int ii=0;
					for(unsigned int i=0; i<inliersV.size(); ++i)
					{
						pcl::PointXYZ & newPt = image3DPoints[inliersV[i]];
						if(pcl::isFinite(newPt))
						{
							newPt = util3d::transformPoint(newPt, output);
							const cv::Point3f & objPt = objectPoints[inliersV[i]];
							errorSqrdDists[ii++] = uNormSquared(objPt.x-newPt.x, objPt.y-newPt.y, objPt.z-newPt.z);
						}
					}
					errorSqrdDists.resize(ii);
					if(errorSqrdDists.size())
					{
						std::sort(errorSqrdDists.begin(), errorSqrdDists.end());
						double median_error_sqr = (double)errorSqrdDists[errorSqrdDists.size () >> 1];
						variance = 2.1981 * median_error_sqr;
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
				UWARN("Not enough correspondences (%d < %d)", correspondences, this->getMinInliers());
			}
		}
		else
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr correspondencesLast(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::PointCloud<pcl::PointXYZ>::Ptr correspondencesNew(new pcl::PointCloud<pcl::PointXYZ>);
			correspondencesLast->resize(refCorners_.size());
			correspondencesNew->resize(refCorners_.size());
			int oi=0;

			if(this->isInfoDataFilled() && info)
			{
				info->refCorners.resize(refCorners_.size());
				info->newCorners.resize(refCorners_.size());
			}

			UASSERT(refCorners_.size() == refCorners3D_->size());
			UDEBUG("lastCorners3D_ = %d", refCorners3D_->size());
			int flowInliers = 0;
			for(unsigned int i=0; i<status.size(); ++i)
			{
				if(status[i] && pcl::isFinite(refCorners3D_->at(i)) &&
					uIsInBounds(newCorners[i].x, 0.0f, float(data.depth().cols)) &&
					uIsInBounds(newCorners[i].y, 0.0f, float(data.depth().rows)))
				{
					pcl::PointXYZ pt = util3d::projectDepthTo3D(data.depth(), newCorners[i].x, newCorners[i].y,
							data.cx(), data.cy(), data.fx(), data.fy(), true);
					if(pcl::isFinite(pt) &&
						(this->getMaxDepth() == 0.0f || (
						uIsInBounds(pt.x, -this->getMaxDepth(), this->getMaxDepth()) &&
						uIsInBounds(pt.y, -this->getMaxDepth(), this->getMaxDepth()) &&
						uIsInBounds(pt.z, 0.0f, this->getMaxDepth()))))
					{
						pt = util3d::transformPoint(pt, data.localTransform());
						correspondencesLast->at(oi) = refCorners3D_->at(i);
						correspondencesNew->at(oi) = pt;

						if(this->isInfoDataFilled() && info)
						{
							info->refCorners[oi] = refCorners_[i];
							info->newCorners[oi] = newCorners[i];
						}

						++oi;
					}
					++flowInliers;
				}
				else if(status[i])
				{
					++flowInliers;
				}
			}
			UDEBUG("Flow inliers = %d, added inliers=%d", flowInliers, oi);

			if(this->isInfoDataFilled() && info)
			{
				info->refCorners.resize(oi);
				info->newCorners.resize(oi);
			}
			correspondencesLast->resize(oi);
			correspondencesNew->resize(oi);
			correspondences = oi;
			if(correspondences >= this->getMinInliers())
			{
				std::vector<int> inliersV;
				UTimer timerRANSAC;
				output = util3d::transformFromXYZCorrespondences(
						correspondencesNew,
						correspondencesLast,
						this->getInlierDistance(),
						this->getIterations(),
						this->getRefineIterations()>0, 3.0, this->getRefineIterations(),
						&inliersV,
						&variance);
				UDEBUG("time RANSAC = %fs", timerRANSAC.ticks());

				inliers = (int)inliersV.size();
				if(inliers < this->getMinInliers())
				{
					output.setNull();
					UWARN("Transform not valid (inliers = %d/%d)", inliers, correspondences);
				}

				if(this->isInfoDataFilled() && info)
				{
					info->cornerInliers = inliersV;
				}
			}
			else
			{
				UWARN("Not enough correspondences (%d)", correspondences);
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
		// Copy or generate new keypoints
		if(data.keypoints().size())
		{
			newCorners.resize(data.keypoints().size());
			for(unsigned int i=0; i<data.keypoints().size(); ++i)
			{
				newCorners[i] = data.keypoints().at(i).pt;
			}
		}
		else
		{
			// generate kpts
			std::vector<cv::KeyPoint> newKtps;
			cv::Rect roi = Feature2D::computeRoi(newFrame, this->getRoiRatios());
			newKtps = feature2D_->generateKeypoints(newFrame, roi);
			Feature2D::filterKeypointsByDepth(newKtps, data.depth(), this->getMaxDepth());

			if(newKtps.size())
			{
				cv::KeyPoint::convert(newKtps, newCorners);

				if(subPixWinSize_ > 0 && subPixIterations_ > 0)
				{
					cv::cornerSubPix(newFrame, newCorners,
						cv::Size( subPixWinSize_, subPixWinSize_ ),
						cv::Size( -1, -1 ),
						cv::TermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, subPixIterations_, subPixEps_ ) );
				}
			}
		}

		if((int)newCorners.size() > this->getMinInliers())
		{
			// get 3D corners for the extracted 2D corners (not the ones refined by Optical Flow)
			pcl::PointCloud<pcl::PointXYZ>::Ptr newCorners3D(new pcl::PointCloud<pcl::PointXYZ>);
			newCorners3D->resize(newCorners.size());
			std::vector<cv::Point2f> newCornersFiltered(newCorners.size());
			int oi=0;
			for(unsigned int i=0; i<newCorners.size(); ++i)
			{
				if(uIsInBounds(newCorners[i].x, 0.0f, float(data.depth().cols)) &&
				   uIsInBounds(newCorners[i].y, 0.0f, float(data.depth().rows)))
				{
					pcl::PointXYZ pt = util3d::projectDepthTo3D(data.depth(), newCorners[i].x, newCorners[i].y,
							data.cx(), data.cy(), data.fx(), data.fy(), true);
					if(pcl::isFinite(pt) &&
						(this->getMaxDepth() == 0.0f || (
						uIsInBounds(pt.x, -this->getMaxDepth(), this->getMaxDepth()) &&
						uIsInBounds(pt.y, -this->getMaxDepth(), this->getMaxDepth()) &&
						uIsInBounds(pt.z, 0.0f, this->getMaxDepth()))))
					{
						pt = util3d::transformPoint(pt, data.localTransform());
						newCorners3D->at(oi) = pt;
						newCornersFiltered[oi] = newCorners[i];
						++oi;
					}
				}
			}
			newCornersFiltered.resize(oi);
			newCorners3D->resize(oi);
			if((int)newCornersFiltered.size() > this->getMinInliers())
			{
				refFrame_ = newFrame;
				refCorners_ = newCornersFiltered;
				refCorners3D_ = newCorners3D;
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

	if(info)
	{
		info->type = 1;
		info->variance = variance;
		info->inliers = inliers;
		info->features = (int)newCorners.size();
		info->matches = correspondences;
	}

	UINFO("Odom update time = %fs lost=%s inliers=%d/%d, variance=%f, new corners=%d",
			timer.elapsed(),
			output.isNull()?"true":"false",
			inliers,
			correspondences,
			variance,
			(int)newCorners.size());
	return output;
}

} // namespace rtabmap
