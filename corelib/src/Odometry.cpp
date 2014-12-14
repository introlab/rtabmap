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

#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UMath.h>

#include <rtabmap/core/OdometryEvent.h>
#include <rtabmap/core/CameraEvent.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/core/Features2d.h>

#include <rtabmap/core/Memory.h>
#include <rtabmap/core/VWDictionary.h>
#include "rtabmap/core/Signature.h"
#include "rtabmap/core/Features2d.h"

#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/common/distances.h>

#include <opencv2/gpu/gpu.hpp>

#if _MSC_VER
	#define ISFINITE(value) _finite(value)
#else
	#define ISFINITE(value) std::isfinite(value)
#endif

namespace rtabmap {

Odometry::Odometry(const rtabmap::ParametersMap & parameters) :
		_maxFeatures(Parameters::defaultOdomMaxFeatures()),
		_roiRatios(Parameters::defaultOdomRoiRatios()),
		_minInliers(Parameters::defaultOdomMinInliers()),
		_inlierDistance(Parameters::defaultOdomInlierDistance()),
		_iterations(Parameters::defaultOdomIterations()),
		_refineIterations(Parameters::defaultOdomRefineIterations()),
		_featuresRatio(Parameters::defaultOdomFeaturesRatio()),
		_maxDepth(Parameters::defaultOdomMaxDepth()),
		_linearUpdate(Parameters::defaultOdomLinearUpdate()),
		_angularUpdate(Parameters::defaultOdomAngularUpdate()),
		_resetCountdown(Parameters::defaultOdomResetCountdown()),
		_force2D(Parameters::defaultOdomForce2D()),
		_resetCurrentCount(0)
{
	Parameters::parse(parameters, Parameters::kOdomLinearUpdate(), _linearUpdate);
	Parameters::parse(parameters, Parameters::kOdomAngularUpdate(), _angularUpdate);
	Parameters::parse(parameters, Parameters::kOdomResetCountdown(), _resetCountdown);
	Parameters::parse(parameters, Parameters::kOdomMinInliers(), _minInliers);
	Parameters::parse(parameters, Parameters::kOdomInlierDistance(), _inlierDistance);
	Parameters::parse(parameters, Parameters::kOdomIterations(), _iterations);
	Parameters::parse(parameters, Parameters::kOdomRefineIterations(), _refineIterations);
	Parameters::parse(parameters, Parameters::kOdomFeaturesRatio(), _featuresRatio);
	Parameters::parse(parameters, Parameters::kOdomMaxDepth(), _maxDepth);
	Parameters::parse(parameters, Parameters::kOdomMaxFeatures(), _maxFeatures);
	Parameters::parse(parameters, Parameters::kOdomRoiRatios(), _roiRatios);
	Parameters::parse(parameters, Parameters::kOdomForce2D(), _force2D);
}

void Odometry::reset(const Transform & initialPose)
{
	_resetCurrentCount = 0;
	if(_force2D)
	{
		float x,y,z, roll,pitch,yaw;
		initialPose.getTranslationAndEulerAngles(x, y, z, roll, pitch, yaw);
		if(z != 0.0f || roll != 0.0f || yaw != 0.0f)
		{
			UWARN("Force2D=true and the initial pose contains z, roll or pitch values (%s). They are set to null.", initialPose.prettyPrint().c_str());
		}
		Transform pose(x, y, 0, 0, 0, yaw);
		_pose = pose;
	}
	else
	{
		_pose = initialPose;
	}
}

bool Odometry::isLargeEnoughTransform(const Transform & transform)
{
	float x,y,z, roll,pitch,yaw;
	transform.getTranslationAndEulerAngles(x,y,z, roll,pitch,yaw);
	return (_linearUpdate == 0.0f && _angularUpdate == 0.0f) ||
			fabs(x) > _linearUpdate ||
		   fabs(y) > _linearUpdate ||
	       fabs(z) > _linearUpdate ||
	       fabs(roll) > _angularUpdate ||
			fabs(pitch) > _angularUpdate ||
			fabs(yaw) > _angularUpdate;
}

Transform Odometry::process(const SensorData & data, OdometryInfo * info)
{
	UTimer time;
	if(_pose.isNull())
	{
		_pose.setIdentity(); // initialized
	}

	Transform t = this->computeTransform(data, info);

	if(info)
	{
		info->time = time.elapsed();
		info->lost = t.isNull();
	}

	if(!t.isNull())
	{
		_resetCurrentCount = _resetCountdown;

		if(_force2D)
		{
			float x,y,z, roll,pitch,yaw;
			t.getTranslationAndEulerAngles(x, y, z, roll, pitch, yaw);
			t = Transform(x,y,0, 0,0,yaw);
		}

		return _pose *= t;
	}
	else if(_resetCurrentCount > 0)
	{
		UWARN("Odometry lost! Odometry will be reset after next %d consecutive unsuccessful odometry updates...", _resetCurrentCount);

		--_resetCurrentCount;
		if(_resetCurrentCount == 0)
		{
			UWARN("Odometry automatically reset!");
			this->reset();
		}
	}
	return Transform();
}

//OdometryBOW
OdometryBOW::OdometryBOW(const ParametersMap & parameters) :
	Odometry(parameters),
	_localHistoryMaxSize(Parameters::defaultOdomBowLocalHistorySize()),
	_memory(0)
{
	Parameters::parse(parameters, Parameters::kOdomBowLocalHistorySize(), _localHistoryMaxSize);

	ParametersMap customParameters;
	customParameters.insert(ParametersPair(Parameters::kKpWordsPerImage(), uNumber2Str(this->getMaxFeatures()))); // hack
	customParameters.insert(ParametersPair(Parameters::kKpMaxDepth(), uNumber2Str(this->getMaxDepth())));
	customParameters.insert(ParametersPair(Parameters::kKpRoiRatios(), this->getRoiRatios()));
	customParameters.insert(ParametersPair(Parameters::kMemRehearsalSimilarity(), "1.0")); // desactivate rehearsal
	customParameters.insert(ParametersPair(Parameters::kMemImageKept(), "false"));
	customParameters.insert(ParametersPair(Parameters::kMemSTMSize(), "0"));
	int nn = Parameters::defaultOdomBowNNType();
	float nndr = Parameters::defaultOdomBowNNDR();
	int featureType = Parameters::defaultOdomFeatureType();
	Parameters::parse(parameters, Parameters::kOdomBowNNType(), nn);
	Parameters::parse(parameters, Parameters::kOdomBowNNDR(), nndr);
	Parameters::parse(parameters, Parameters::kOdomFeatureType(), featureType);
	customParameters.insert(ParametersPair(Parameters::kKpNNStrategy(), uNumber2Str(nn)));
	customParameters.insert(ParametersPair(Parameters::kKpNndrRatio(), uNumber2Str(nndr)));
	customParameters.insert(ParametersPair(Parameters::kKpDetectorStrategy(), uNumber2Str(featureType)));

	// Memory's stereo parameters, copy from Odometry
	int subPixWinSize_ = Parameters::defaultOdomSubPixWinSize();
	int subPixIterations_ = Parameters::defaultOdomSubPixIterations();
	double subPixEps_ = Parameters::defaultOdomSubPixEps();
	Parameters::parse(parameters, Parameters::kOdomSubPixWinSize(), subPixWinSize_);
	Parameters::parse(parameters, Parameters::kOdomSubPixIterations(), subPixIterations_);
	Parameters::parse(parameters, Parameters::kOdomSubPixEps(), subPixEps_);
	customParameters.insert(ParametersPair(Parameters::kKpSubPixWinSize(), uNumber2Str(subPixWinSize_)));
	customParameters.insert(ParametersPair(Parameters::kKpSubPixIterations(), uNumber2Str(subPixIterations_)));
	customParameters.insert(ParametersPair(Parameters::kKpSubPixEps(), uNumber2Str(subPixEps_)));

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

	_memory = new Memory(customParameters);
	if(!_memory->init("", false, ParametersMap()))
	{
		UERROR("Error initializing the memory for BOW Odometry.");
	}
}

OdometryBOW::~OdometryBOW()
{
	delete _memory;
}


void OdometryBOW::reset(const Transform & initialPose)
{
	Odometry::reset(initialPose);
	_memory->init("", false, ParametersMap());
	localMap_.clear();
}

// return not null transform if odometry is correctly computed
Transform OdometryBOW::computeTransform(
		const SensorData & data,
		OdometryInfo * info)
{
	UTimer timer;
	Transform output;

	double variance = -1;
	int inliers = 0;
	int correspondences = 0;
	int nFeatures = 0;

	const Signature * previousSignature = _memory->getLastWorkingSignature();
	if(_memory->update(data))
	{
		const Signature * newSignature = _memory->getLastWorkingSignature();
		if(newSignature)
		{
			nFeatures = (int)newSignature->getWords().size();
		}

		if(previousSignature && newSignature)
		{
			Transform transform;
			std::set<int> uniqueCorrespondences;
			if(newSignature->getWords3().size() < (unsigned int)(this->getFeaturesRatio() * float(previousSignature->getWords3().size())))
			{
				UWARN("At least %f%% keypoints of the last image required. New=%d last=%d",
						this->getFeaturesRatio()*100.0f, newSignature->getWords3().size(), previousSignature->getWords3().size());
			}
			else if(!localMap_.empty() && !newSignature->getWords3().empty())
			{
				pcl::PointCloud<pcl::PointXYZ>::Ptr inliers1(new pcl::PointCloud<pcl::PointXYZ>); // previous
				pcl::PointCloud<pcl::PointXYZ>::Ptr inliers2(new pcl::PointCloud<pcl::PointXYZ>); // new

				// No need to set max depth here, it is already applied in extractKeypointsAndDescriptors() above.
				// Also! the localMap_ have points not in camera frame anymore (in local map frame), so filtering
				// by depth here is wrong!
				util3d::findCorrespondences(
						localMap_,
						newSignature->getWords3(),
						*inliers1,
						*inliers2,
						0,
						&uniqueCorrespondences);

				UDEBUG("localMap=%d, new=%d, unique correspondences=%d", (int)localMap_.size(), (int)newSignature->getWords3().size(), (int)uniqueCorrespondences.size());

				correspondences = (int)inliers1->size();
				if((int)inliers1->size() >= this->getMinInliers())
				{
					// transform new words in local map referential
					//inliers2 = util3d::transformPointCloud<pcl::PointXYZ>(inliers2, this->getPose());

					// the transform returned is global odometry pose, not incremental one
					std::vector<int> inliersV;
					transform = util3d::transformFromXYZCorrespondences(
							inliers2,
							inliers1,
							this->getInlierDistance(),
							this->getIterations(),
							this->getRefineIterations()>0, 3.0, this->getRefineIterations(),
							&inliersV,
							&variance);

					inliers = (int)inliersV.size();
					if(!transform.isNull())
					{
						// make it incremental
						transform = this->getPose().inverse() * transform;

						UDEBUG("Odom transform = %s", transform.prettyPrint().c_str());
					}
					else
					{
						UDEBUG("Odom transform null");
						//pcl::io::savePCDFile("from.pcd", *inliers1);
						//inliers2 = util3d::transformPointCloud(inliers2, this->getPose());
						//pcl::io::savePCDFile("to.pcd", *inliers2);
						//inliers2 = util3d::transformPointCloud(inliers2, transform);
						//pcl::io::savePCDFile("to_t.pcd", *inliers2);
					}

					if(ULogger::level() == ULogger::kDebug)
					{
						float error3D = 0;
					//	pcl::PointCloud<pcl::PointXYZ>::Ptr inliers1Cloud(new pcl::PointCloud<pcl::PointXYZ>), inliers2Cloud(new pcl::PointCloud<pcl::PointXYZ>);
						for(unsigned int i=0; i<inliersV.size(); ++i)
						{
							pcl::PointXYZ pt = util3d::transformPoint(inliers2->at(inliersV[i]), this->getPose()*transform);
							error3D+=pcl::euclideanDistance(inliers1->at(inliersV[i]), pt);
						//	inliers1Cloud->push_back(inliers1->at(inliersV[i]));
						//	inliers2Cloud->push_back(pt);
						}
						error3D/=float(inliersV.size());
						UDEBUG("3D error = %f", error3D);
					}

					/*if(inliersV.size() < 30 || fabs(transform.x()) > 0.3 || fabs(transform.y()) > 0.3 || fabs(transform.z()) > 0.3)
					{
						UWARN("Saved from.pcd, to.pcd and to_t.pcd");
						pcl::io::savePCDFile("from.pcd", *inliers1);
						pcl::io::savePCDFile("to.pcd", *inliers2);
						inliers2 = util3d::transformPointCloud<pcl::PointXYZ>(inliers2, transform);
						pcl::io::savePCDFile("to_t.pcd", *inliers2);

						pcl::io::savePCDFile("inliersFrom.pcd", *inliers1Cloud);
						pcl::io::savePCDFile("inliersTo.pcd", *inliers2Cloud);
						inliers2Cloud = util3d::transformPointCloud<pcl::PointXYZ>(inliers2Cloud, transform);
						pcl::io::savePCDFile("inliersTo_t.pcd", *inliers2Cloud);
						exit(-1);
					}*/
					/*pcl::io::savePCDFile("from.pcd", *inliers1);
					inliers2 = util3d::transformPointCloud(inliers2, this->getPose());
					pcl::io::savePCDFile("to.pcd", *inliers2);
					inliers2 = util3d::transformPointCloud(inliers2, transform);
					pcl::io::savePCDFile("to_t.pcd", *inliers2);*/


					/*
					//refine ICP test
					bool hasConverged;
					double fitness;
					inliers2 = util3d::transformPointCloud(inliers2, transform);
					Transform icpT = util3d::icp(inliers1, <-- must be all correspondences, not only unique
						inliers2,
						0.02,
						100,
						hasConverged,
						fitness);

					transform = transform * icpT;
					*/

					if(inliers < this->getMinInliers())
					{
						transform.setNull();
						UWARN("Transform not valid (inliers = %d/%d)", inliers, correspondences);
					}
				}
				else
				{
					UWARN("Not enough inliers %d < %d", (int)inliers1->size(), this->getMinInliers());
				}
			}

			if(transform.isNull())
			{
				_memory->deleteLocation(newSignature->id());
			}
			else
			{
				output = transform;
				if(!isLargeEnoughTransform(transform))
				{
					// Transform not large enough, keep the old signature
					_memory->deleteLocation(newSignature->id());
				}
				else
				{
					// remove words if history max size is reached
					while(localMap_.size() && (int)localMap_.size() > _localHistoryMaxSize && _memory->getStMem().size()>1)
					{
						int nodeId = *_memory->getStMem().begin();
						std::list<int> removedPts;
						_memory->deleteLocation(nodeId, &removedPts);
						for(std::list<int>::iterator iter = removedPts.begin(); iter!=removedPts.end(); ++iter)
						{
							localMap_.erase(*iter);
						}
					}

					if(_localHistoryMaxSize == 0 && localMap_.size() > 0 && localMap_.size() > newSignature->getWords3().size())
					{
						UERROR("Local map should have only words of the last added signature here! (size=%d, max history size=%d, newWords=%d)",
								(int)localMap_.size(), _localHistoryMaxSize, (int)newSignature->getWords3().size());
					}

					// update local map
					std::list<int> uniques = uUniqueKeys(newSignature->getWords3());
					Transform t = this->getPose()*output;
					for(std::list<int>::iterator iter = uniques.begin(); iter!=uniques.end(); ++iter)
					{
						// Only add unique words not in local map
						if(newSignature->getWords3().count(*iter) == 1)
						{
							// keep old word
							if(localMap_.find(*iter) == localMap_.end())
							{
								const pcl::PointXYZ & pt = newSignature->getWords3().find(*iter)->second;
								if(pcl::isFinite(pt))
								{
									pcl::PointXYZ pt2 = util3d::transformPoint(pt, t);
									localMap_.insert(std::make_pair(*iter, pt2));
								}
							}
						}
						else
						{
							localMap_.erase(*iter);
						}
					}
				}
			}
		}
		else if(!previousSignature && newSignature)
		{
			localMap_.clear();
			output.setIdentity();

			int count = 0;
			std::list<int> uniques = uUniqueKeys(newSignature->getWords3());
			Transform t = this->getPose(); // initial pose maybe not identity...
			for(std::list<int>::iterator iter = uniques.begin(); iter!=uniques.end(); ++iter)
			{
				// Only add unique words
				if(newSignature->getWords3().count(*iter) == 1)
				{
					const pcl::PointXYZ & pt = newSignature->getWords3().find(*iter)->second;
					if(pcl::isFinite(pt))
					{
						pcl::PointXYZ pt2 = util3d::transformPoint(pt, t);
						localMap_.insert(std::make_pair(*iter, pt2));
					}
					else
					{
						++count;
					}
				}
			}
			UDEBUG("uniques=%d, pt not finite = %d", (int)uniques.size(),count);
		}

		_memory->emptyTrash();
	}

	if(info)
	{
		info->variance = variance;
		info->inliers = inliers;
		info->matches = correspondences;
		info->features = nFeatures;
		info->localMapSize = (int)localMap_.size();
	}

	UINFO("Odom update time = %fs out=[%s] features=%d inliers=%d/%d local_map=%d[%d] dict=%d nodes=%d",
			timer.elapsed(),
			output.prettyPrint().c_str(),
			nFeatures,
			inliers,
			correspondences,
			(int)uUniqueKeys(localMap_).size(),
			(int)localMap_.size(),
			(int)_memory->getVWDictionary()->getVisualWords().size(),
			(int)_memory->getStMem().size());
	return output;
}

//OdometryOpticalFlow
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
	feature2D_ = Feature2D::create(detectorStrategy, parameters);
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

	double variance = -1;
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

			UDEBUG("");
			std::vector<cv::KeyPoint> lastKpts, newKpts;
			/*cv::KeyPoint::convert(lastCornersKept, lastKpts);
			cv::KeyPoint::convert(newCornersKept, newKpts);
			std::vector<cv::DMatch> good_matches(lastKpts.size());
			for(unsigned int i=0; i<good_matches.size(); ++i)
			{
				good_matches[i].trainIdx = i;
				good_matches[i].queryIdx = i;
			}

			cv::drawMatches( lastFrame_, lastKpts, newLeftFrame, newKpts,
						   good_matches, imgMatches_, cv::Scalar::all(-1), cv::Scalar::all(-1),
						   std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
			UDEBUG("");*/

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

			UDEBUG("Getting correspondences begin");
			// Get 3D correspondences
			pcl::PointCloud<pcl::PointXYZ>::Ptr correspondencesLast(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::PointCloud<pcl::PointXYZ>::Ptr correspondencesNew(new pcl::PointCloud<pcl::PointXYZ>);
			correspondencesLast->resize(statusLast.size());
			correspondencesNew->resize(statusLast.size());
			int oi = 0;
			lastKpts.resize(statusLast.size());
			newKpts.resize(statusLast.size());
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
							lastKpts[oi].pt = lastCornersKept[i];
							newKpts[oi].pt = newCornersKept[i];
							++oi;
						}
					}
				}
			}// end loop
			correspondencesLast->resize(oi);
			correspondencesNew->resize(oi);
			lastKpts.resize(oi);
			newKpts.resize(oi);
			correspondences = oi;
			refCorners3D_ = correspondencesNew;
			UDEBUG("Getting correspondences end, kept %d/%d", correspondences, (int)statusLast.size());

			/*good_matches.resize(lastKpts.size());
			for(unsigned int i=0; i<good_matches.size(); ++i)
			{
				good_matches[i].trainIdx = i;
				good_matches[i].queryIdx = i;
			}

			cv::Mat imgInliers;
			cv::drawMatches( lastFrame_, lastKpts, newLeftFrame, newKpts,
						   good_matches, imgInliers, cv::Scalar::all(-1), cv::Scalar::all(-1),
						   std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
			imgMatches_.push_back(imgInliers);
			UDEBUG("");*/

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

				//if(correspondencesLast->size() >= 6)
				//{
				//	UWARN("saved pcd");
				//	pcl::io::savePCDFile("last.pcd", *correspondencesLast);
				//	pcl::io::savePCDFile("new.pcd", *correspondencesNew);
					//correspondencesNew = util3d::transformPointCloud(correspondencesNew, output);
					//pcl::io::savePCDFile("new2.pcd", *correspondencesNew);
				//}
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
			cv::Rect roi = Feature2D::computeRoi(newLeftFrame, this->getRoiRatios());
			newKtps = feature2D_->generateKeypoints(newLeftFrame, this->getMaxFeatures(), roi);
			Feature2D::limitKeypoints(newKtps, this->getMaxFeatures());

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

		if(refCorners_.size() && newCorners.size() < (unsigned int)(this->getFeaturesRatio() * float(refCorners_.size())))
		{
			UWARN("At least %f%% keypoints of the last image required. New=%d last=%d",
					this->getFeaturesRatio()*100.0f, newCorners.size(), refCorners_.size());
		}
		else if((int)newCorners.size() > this->getMinInliers())
		{
			refFrame_ = newLeftFrame;
			refRightFrame_ = newRightFrame;
			refCorners_ = newCorners;
		}
		else
		{
			UWARN("Too low 2D corners (%d), ignoring new frame...",
					(int)newCorners.size());
		}
	}
	else if(!output.isNull())
	{
		output.setNull();
	}

	if(info)
	{
		info->variance = variance;
		info->inliers = inliers;
		info->features = (int)newCorners.size();
		info->matches = correspondences;
	}

	UINFO("Odom update time = %fs inliers=%d/%d, new corners=%d, transform accepted=%s",
			timer.elapsed(),
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

	double variance = -1;
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

	float updatePixels = 0.0f;
	bool updateFrame = false;

	std::vector<cv::Point2f> newCorners;
	if(!refFrame_.empty() && refCorners_.size() && refCorners3D_->size())
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

		pcl::PointCloud<pcl::PointXYZ>::Ptr correspondencesLast(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr correspondencesNew(new pcl::PointCloud<pcl::PointXYZ>);
		correspondencesLast->resize(refCorners_.size());
		correspondencesNew->resize(refCorners_.size());
		int oi=0;

		std::vector<cv::KeyPoint> lastKpts(refCorners_.size());
		std::vector<cv::KeyPoint> newKpts(refCorners_.size());

		UASSERT(refCorners_.size() == refCorners3D_->size());
		UDEBUG("lastCorners3D_ = %d", refCorners3D_->size());
		float sumSqrdDistance = 0.0f;
		int flowInliers = 0;
		for(unsigned int i=0; i<status.size(); ++i)
		{
			if(status[i] && pcl::isFinite(refCorners3D_->at(i)) &&
				uIsInBounds(newCorners[i].x, 0.0f, float(data.depth().cols-1)) &&
				uIsInBounds(newCorners[i].y, 0.0f, float(data.depth().rows-1)))
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

					cv::Point2f diff = newCorners[i]-refCorners_[i];
					sumSqrdDistance += diff.x*diff.x + diff.y*diff.y;

					lastKpts[oi].pt = refCorners_[i];
					newKpts[oi].pt = newCorners[i];

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
		float meanPixel = -1;
		if(oi)
		{
			float meanPixel = sumSqrdDistance/(float)oi;
			if(meanPixel >= updatePixels*updatePixels)
			{
				updateFrame = true;
			}
		}
		UDEBUG("mean pixel distance = %f", meanPixel);

		lastKpts.resize(oi);
		newKpts.resize(oi);
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

			/*std::vector<cv::DMatch> good_matches(lastKpts.size());
			for(unsigned int i=0; i<good_matches.size(); ++i)
			{
				good_matches[i].trainIdx = i;
				good_matches[i].queryIdx = i;
			}

			cv::drawMatches( lastFrame_, lastKpts, newFrame, newKpts,
						   good_matches, imgMatches_, cv::Scalar::all(-1), cv::Scalar::all(-1),
						   std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );*/
		}
		else
		{
			UWARN("Not enough correspondences (%d)", correspondences);
		}
	}
	else
	{
		//return Identity
		output = Transform::getIdentity();
		updateFrame = true;
	}

	newCorners.clear();
	if(!output.isNull() && updateFrame)
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
			newKtps = feature2D_->generateKeypoints(newFrame, this->getMaxFeatures(), roi);
			Feature2D::filterKeypointsByDepth(newKtps, data.depth(), this->getMaxDepth());
			Feature2D::limitKeypoints(newKtps, this->getMaxFeatures());

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

		if(refCorners_.size() && newCorners.size() < (unsigned int)(this->getFeaturesRatio() * float(refCorners_.size())))
		{
			UWARN("At least %f%% keypoints of the last image required. New=%d last=%d",
					this->getFeaturesRatio()*100.0f, newCorners.size(), refCorners_.size());
		}
		else if((int)newCorners.size() > this->getMinInliers())
		{
			// get 3D corners for the extracted 2D corners (not the ones refined by Optical Flow)
			pcl::PointCloud<pcl::PointXYZ>::Ptr newCorners3D(new pcl::PointCloud<pcl::PointXYZ>);
			newCorners3D->resize(newCorners.size());
			std::vector<cv::Point2f> newCornersFiltered(newCorners.size());
			int oi=0;
			for(unsigned int i=0; i<newCorners.size(); ++i)
			{
				if(uIsInBounds(newCorners[i].x, 0.0f, float(data.depth().cols)-1.0f) &&
				   uIsInBounds(newCorners[i].y, 0.0f, float(data.depth().rows)-1.0f))
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
			}
		}
		else
		{
			UWARN("Too low 2D corners (%d), ignoring new frame...",
					(int)newCorners.size());
		}
	}
	else if(!output.isNull())
	{
		output = Transform::getIdentity();
	}

	if(info)
	{
		info->variance = variance;
		info->inliers = inliers;
		info->features = (int)newCorners.size();
		info->matches = correspondences;
	}

	UINFO("Odom update time = %fs inliers=%d/%d, new corners=%d, transform accepted=%s",
			timer.elapsed(),
			inliers,
			correspondences,
			(int)newCorners.size(),
			updateFrame||output.isNull()?"true":"false");
	return output;
}

// OdometryICP
OdometryICP::OdometryICP(int decimation,
		float voxelSize,
		int samples,
		float maxCorrespondenceDistance,
		int maxIterations,
		float correspondenceRatio,
		bool pointToPlane,
		const ParametersMap & odometryParameter) :
	Odometry(odometryParameter),
	_decimation(decimation),
	_voxelSize(voxelSize),
	_samples(samples),
	_maxCorrespondenceDistance(maxCorrespondenceDistance),
	_maxIterations(maxIterations),
	_correspondenceRatio(correspondenceRatio),
	_pointToPlane(pointToPlane),
	_previousCloudNormal(new pcl::PointCloud<pcl::PointNormal>),
	_previousCloud(new pcl::PointCloud<pcl::PointXYZ>)
{
}

void OdometryICP::reset(const Transform & initialPose)
{
	Odometry::reset(initialPose);
	_previousCloudNormal.reset(new pcl::PointCloud<pcl::PointNormal>);
	_previousCloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
}

// return not null transform if odometry is correctly computed
Transform OdometryICP::computeTransform(const SensorData & data, OdometryInfo * info)
{
	UTimer timer;
	Transform output;

	bool hasConverged = false;
	double variance = -1;
	unsigned int minPoints = 100;
	if(!data.depth().empty())
	{
		if(data.depth().type() == CV_8UC1)
		{
			UERROR("ICP 3D cannot be done on stereo images!");
			return output;
		}

		pcl::PointCloud<pcl::PointXYZ>::Ptr newCloudXYZ = util3d::getICPReadyCloud(
						data.depth(),
						data.fx(),
						data.fy(),
						data.cx(),
						data.cy(),
						_decimation,
						this->getMaxDepth(),
						_voxelSize,
						_samples,
						data.localTransform());

		if(_pointToPlane)
		{
			pcl::PointCloud<pcl::PointNormal>::Ptr newCloud = util3d::computeNormals(newCloudXYZ);

			std::vector<int> indices;
			newCloud = util3d::removeNaNNormalsFromPointCloud<pcl::PointNormal>(newCloud);
			if(newCloudXYZ->size() != newCloud->size())
			{
				UWARN("removed nan normals...");
			}

			if(_previousCloudNormal->size() > minPoints && newCloud->size() > minPoints)
			{
				int correspondences = 0;
				Transform transform = util3d::icpPointToPlane(newCloud,
						_previousCloudNormal,
						_maxCorrespondenceDistance,
						_maxIterations,
						&hasConverged,
						&variance,
						&correspondences);

				// verify if there are enough correspondences
				float correspondencesRatio = float(correspondences)/float(_previousCloudNormal->size()>newCloud->size()?_previousCloudNormal->size():newCloud->size());

				if(!transform.isNull() && hasConverged &&
		   	   	   correspondencesRatio >= _correspondenceRatio)
				{
					output = transform;
					_previousCloudNormal = newCloud;
				}
				else
				{
					UWARN("Transform not valid (hasConverged=%s variance = %f)",
							hasConverged?"true":"false", variance);
				}
			}
			else if(newCloud->size() > minPoints)
			{
				output.setIdentity();
				_previousCloudNormal = newCloud;
			}
		}
		else
		{
			//point to point
			if(_previousCloud->size() > minPoints && newCloudXYZ->size() > minPoints)
			{
				int correspondences = 0;
				Transform transform = util3d::icp(newCloudXYZ,
						_previousCloud,
						_maxCorrespondenceDistance,
						_maxIterations,
						&hasConverged,
						&variance,
						&correspondences);

				// verify if there are enough correspondences
				float correspondencesRatio = float(correspondences)/float(_previousCloud->size()>newCloudXYZ->size()?_previousCloud->size():newCloudXYZ->size());

				if(!transform.isNull() && hasConverged &&
				   correspondencesRatio >= _correspondenceRatio)
				{
					output = transform;
					_previousCloud = newCloudXYZ;
				}
				else
				{
					UWARN("Transform not valid (hasConverged=%s variance = %f)",
							hasConverged?"true":"false", variance);
				}
			}
			else if(newCloudXYZ->size() > minPoints)
			{
				output.setIdentity();
				_previousCloud = newCloudXYZ;
			}
		}
	}
	else
	{
		UERROR("Depth is empty?!?");
	}

	if(info)
	{
		info->variance = variance;
	}

	UINFO("Odom update time = %fs hasConverged=%s variance=%f cloud=%d",
			timer.elapsed(),
			hasConverged?"true":"false",
			variance,
			(int)(_pointToPlane?_previousCloudNormal->size():_previousCloud->size()));

	return output;
}

// OdometryThread
OdometryThread::OdometryThread(Odometry * odometry) :
	_odometry(odometry),
	_resetOdometry(false)
{
	UASSERT(_odometry != 0);
}

OdometryThread::~OdometryThread()
{
	this->unregisterFromEventsManager();
	this->join(true);
	if(_odometry)
	{
		delete _odometry;
	}
}

void OdometryThread::handleEvent(UEvent * event)
{
	if(this->isRunning())
	{
		if(event->getClassName().compare("CameraEvent") == 0)
		{
			CameraEvent * cameraEvent = (CameraEvent*)event;
			if(cameraEvent->getCode() == CameraEvent::kCodeImageDepth)
			{
				this->addData(cameraEvent->data());
			}
			else if(cameraEvent->getCode() == CameraEvent::kCodeNoMoreImages)
			{
				this->post(new CameraEvent()); // forward the event
			}
		}
		else if(event->getClassName().compare("OdometryResetEvent") == 0)
		{
			_resetOdometry = true;
		}
	}
}

void OdometryThread::mainLoopKill()
{
	_dataAdded.release();
}

//============================================================
// MAIN LOOP
//============================================================
void OdometryThread::mainLoop()
{
	if(_resetOdometry)
	{
		_odometry->reset();
		_resetOdometry = false;
	}

	SensorData data;
	getData(data);
	if(data.isValid())
	{
		OdometryInfo info;
		Transform pose = _odometry->process(data, &info);
		data.setPose(pose, info.variance); // a null pose notify that odometry could not be computed
		this->post(new OdometryEvent(data, info));
	}
}

void OdometryThread::addData(const SensorData & data)
{
	if(data.image().empty() || data.depth().empty() || data.fx() == 0.0f || data.fy() == 0.0f)
	{
		ULOGGER_ERROR("image empty !?");
		return;
	}

	bool notify = true;
	_dataMutex.lock();
	{
		notify = !_dataBuffer.isValid();
		_dataBuffer = data;
	}
	_dataMutex.unlock();

	if(notify)
	{
		_dataAdded.release();
	}
}

void OdometryThread::getData(SensorData & data)
{
	_dataAdded.acquire();
	_dataMutex.lock();
	{
		if(_dataBuffer.isValid())
		{
			data = _dataBuffer;
			_dataBuffer = SensorData();
		}
	}
	_dataMutex.unlock();
}

} /* namespace rtabmap */
