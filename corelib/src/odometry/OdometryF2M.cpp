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

#include "rtabmap/core/OdometryInfo.h"
#include "rtabmap/core/Memory.h"
#include "rtabmap/core/Signature.h"
#include "rtabmap/core/RegistrationVis.h"
#include "rtabmap/core/util3d.h"
#include "rtabmap/core/util3d_transforms.h"
#include "rtabmap/core/util3d_registration.h"
#include "rtabmap/core/util3d_motion_estimation.h"
#include "rtabmap/core/util3d_filtering.h"
#include "rtabmap/core/util3d_surface.h"
#include "rtabmap/core/Optimizer.h"
#include "rtabmap/core/VWDictionary.h"
#include "rtabmap/core/Graph.h"
#include "rtabmap/utilite/ULogger.h"
#include "rtabmap/utilite/UTimer.h"
#include "rtabmap/utilite/UMath.h"
#include "rtabmap/utilite/UConversion.h"
#include <opencv2/calib3d/calib3d.hpp>
#include <rtabmap/core/odometry/OdometryF2M.h>
#include <pcl/common/io.h>

#if _MSC_VER
	#define ISFINITE(value) _finite(value)
#else
	#define ISFINITE(value) std::isfinite(value)
#endif

namespace rtabmap {

OdometryF2M::OdometryF2M(const ParametersMap & parameters) :
	Odometry(parameters),
	maximumMapSize_(Parameters::defaultOdomF2MMaxSize()),
	keyFrameThr_(Parameters::defaultOdomKeyFrameThr()),
	visKeyFrameThr_(Parameters::defaultOdomVisKeyFrameThr()),
	maxNewFeatures_(Parameters::defaultOdomF2MMaxNewFeatures()),
	scanKeyFrameThr_(Parameters::defaultOdomScanKeyFrameThr()),
	scanMaximumMapSize_(Parameters::defaultOdomF2MScanMaxSize()),
	scanSubtractRadius_(Parameters::defaultOdomF2MScanSubtractRadius()),
	scanSubtractAngle_(Parameters::defaultOdomF2MScanSubtractAngle()),
	scanMapMaxRange_(Parameters::defaultOdomF2MScanRange()),
	bundleAdjustment_(Parameters::defaultOdomF2MBundleAdjustment()),
	bundleMaxFrames_(Parameters::defaultOdomF2MBundleAdjustmentMaxFrames()),
	validDepthRatio_(Parameters::defaultOdomF2MValidDepthRatio()),
	pointToPlaneK_(Parameters::defaultIcpPointToPlaneK()),
	pointToPlaneRadius_(Parameters::defaultIcpPointToPlaneRadius()),
	map_(new Signature(-1)),
	lastFrame_(new Signature(1)),
	lastFrameOldestNewId_(0),
	initGravity_(false),
	bundleSeq_(0),
	sba_(0)
{
	UDEBUG("");
	Parameters::parse(parameters, Parameters::kOdomF2MMaxSize(), maximumMapSize_);
	Parameters::parse(parameters, Parameters::kOdomKeyFrameThr(), keyFrameThr_);
	Parameters::parse(parameters, Parameters::kOdomVisKeyFrameThr(), visKeyFrameThr_);
	Parameters::parse(parameters, Parameters::kOdomF2MMaxNewFeatures(), maxNewFeatures_);
	Parameters::parse(parameters, Parameters::kOdomScanKeyFrameThr(), scanKeyFrameThr_);
	Parameters::parse(parameters, Parameters::kOdomF2MScanMaxSize(), scanMaximumMapSize_);
	Parameters::parse(parameters, Parameters::kOdomF2MScanSubtractRadius(), scanSubtractRadius_);
	if(Parameters::parse(parameters, Parameters::kOdomF2MScanSubtractAngle(), scanSubtractAngle_))
	{
		scanSubtractAngle_ *= M_PI/180.0f;
	}
	Parameters::parse(parameters, Parameters::kOdomF2MScanRange(), scanMapMaxRange_);
	Parameters::parse(parameters, Parameters::kOdomF2MBundleAdjustment(), bundleAdjustment_);
	Parameters::parse(parameters, Parameters::kOdomF2MBundleAdjustmentMaxFrames(), bundleMaxFrames_);
	Parameters::parse(parameters, Parameters::kOdomF2MValidDepthRatio(), validDepthRatio_);

	Parameters::parse(parameters, Parameters::kIcpPointToPlaneK(), pointToPlaneK_);
	Parameters::parse(parameters, Parameters::kIcpPointToPlaneRadius(), pointToPlaneRadius_);

	UASSERT(bundleMaxFrames_ >= 0);
	ParametersMap bundleParameters = parameters;
	if(bundleAdjustment_ > 0)
	{
		if((bundleAdjustment_==1 && Optimizer::isAvailable(Optimizer::kTypeG2O)) ||
		(bundleAdjustment_==2 && Optimizer::isAvailable(Optimizer::kTypeCVSBA)) ||
		(bundleAdjustment_==3 && Optimizer::isAvailable(Optimizer::kTypeCeres)))
		{
			// disable bundle in RegistrationVis as we do it already here
			uInsert(bundleParameters, ParametersPair(Parameters::kVisBundleAdjustment(), "0"));
			sba_ = Optimizer::create(bundleAdjustment_==3?Optimizer::kTypeCeres:bundleAdjustment_==2?Optimizer::kTypeCVSBA:Optimizer::kTypeG2O, bundleParameters);
		}
		else
		{
			UWARN("Selected bundle adjustment approach (\"%s\"=\"%d\") is not available, "
					"local bundle adjustment is then disabled.", Parameters::kOdomF2MBundleAdjustment().c_str(), bundleAdjustment_);
			bundleAdjustment_ = 0;
		}
	}
	UASSERT(maximumMapSize_ >= 0);
	UASSERT(keyFrameThr_ >= 0.0f && keyFrameThr_<=1.0f);
	UASSERT(visKeyFrameThr_>=0);
	UASSERT(scanKeyFrameThr_ >= 0.0f && scanKeyFrameThr_<=1.0f);
	UASSERT(maxNewFeatures_ >= 0);

	int corType = Parameters::defaultVisCorType();
	Parameters::parse(parameters, Parameters::kVisCorType(), corType);
	if(corType != 0)
	{
		UWARN("%s=%d is not supported by OdometryF2M, using Features matching approach instead (type=0).",
				Parameters::kVisCorType().c_str(),
				corType);
		corType = 0;
	}
	uInsert(bundleParameters, ParametersPair(Parameters::kVisCorType(), uNumber2Str(corType)));

	regPipeline_ = Registration::create(bundleParameters);
	if(bundleAdjustment_>0 && regPipeline_->isScanRequired())
	{
		UWARN("%s=%d cannot be used with registration not done only with images (%s=%s), disabling bundle adjustment.",
				Parameters::kOdomF2MBundleAdjustment().c_str(),
				bundleAdjustment_,
				Parameters::kRegStrategy().c_str(),
				uValue(bundleParameters, Parameters::kRegStrategy(), uNumber2Str(Parameters::defaultRegStrategy())).c_str());
		bundleAdjustment_ = 0;
	}

	parameters_ = bundleParameters;
}

OdometryF2M::~OdometryF2M()
{
	delete map_;
	delete lastFrame_;
	delete sba_;
	delete regPipeline_;
	UDEBUG("");
}


void OdometryF2M::reset(const Transform & initialPose)
{
	Odometry::reset(initialPose);
	if(!initGravity_)
	{
		UDEBUG("initialPose=%s", initialPose.prettyPrint().c_str());
		Odometry::reset(initialPose);
		*lastFrame_ = Signature(1);
		*map_ = Signature(-1);
		scansBuffer_.clear();
		bundleWordReferences_.clear();
		bundlePoses_.clear();
		bundleLinks_.clear();
		bundleModels_.clear();
		bundlePoseReferences_.clear();
		bundleSeq_ = 0;
		lastFrameOldestNewId_ = 0;
	}
	initGravity_ = false;
}

bool OdometryF2M::canProcessIMU() const
{
	return sba_ && sba_->gravitySigma() > 0.0f;
}

// return not null transform if odometry is correctly computed
Transform OdometryF2M::computeTransform(
		SensorData & data,
		const Transform & guessIn,
		OdometryInfo * info)
{
	Transform guess = guessIn;
	UTimer timer;
	Transform output;

	if(info)
	{
		info->type = 0;
	}

	Transform imuT;
	if(sba_ && sba_->gravitySigma() > 0.0f && !data.imu().empty())
	{
		if(imus().empty())
		{
			UERROR("IMU received doesn't have orientation set, it is ignored. If you are using RTAB-Map standalone, enable IMU filtering in Preferences->Source panel. On ROS, use \"imu_filter_madgwick\" or \"imu_complementary_filter\" packages to compute the orientation.");
		}
		else
		{
			imuT = Transform::getTransform(imus(), data.stamp());
			if(this->getPose().r11() == 1.0f && this->getPose().r22() == 1.0f && this->getPose().r33() == 1.0f)
			{
				if(!imuT.isNull())
				{
					Eigen::Quaterniond imuQuat = imuT.getQuaterniond();
					Transform previous = this->getPose();
					Transform newFramePose = Transform(previous.x(), previous.y(), previous.z(), imuQuat.x(), imuQuat.y(), imuQuat.z(), imuQuat.w());
					UWARN("Updated initial pose from %s to %s with IMU orientation", previous.prettyPrint().c_str(), newFramePose.prettyPrint().c_str());
					initGravity_ = true;
					this->reset(newFramePose);
				}
			}
		}

		if(data.imageRaw().empty() && data.laserScanRaw().isEmpty())
		{
			return output;
		}
	}

	RegistrationInfo regInfo;
	int nFeatures = 0;

	delete lastFrame_;
	int id = data.id();
	data.setId(++bundleSeq_); // generate our own unique ids, to make sure they are correctly set
	lastFrame_ = new Signature(data);
	data.setId(id);

	if(bundleAdjustment_ > 0 &&
	   data.cameraModels().size() > 1)
	{
		UERROR("Odometry bundle adjustment doesn't work with multi-cameras. It is disabled.");
		bundleAdjustment_ = 0;
	}
	bool addKeyFrame = false;
	int totalBundleWordReferencesUsed = 0;
	int totalBundleOutliers = 0;
	float bundleTime = 0.0f;
	bool visDepthAsMask = Parameters::defaultVisDepthAsMask();
	Parameters::parse(parameters_, Parameters::kVisDepthAsMask(), visDepthAsMask);

	// Generate keypoints from the new data
	if(lastFrame_->sensorData().isValid())
	{
		if((map_->getWords3().size() || !map_->sensorData().laserScanRaw().isEmpty()) &&
			lastFrame_->sensorData().isValid())
		{
			Signature tmpMap;
			Transform transform;
			UDEBUG("guess=%s frames=%d image required=%d", guess.prettyPrint().c_str(), this->framesProcessed(), regPipeline_->isImageRequired()?1:0);

			// bundle adjustment stuff if used
			std::map<int, cv::Point3f> points3DMap;
			std::map<int, Transform> bundlePoses;
			std::multimap<int, Link> bundleLinks;
			std::map<int, CameraModel> bundleModels;

			for(int guessIteration=0;
					guessIteration<(!guess.isNull()&&regPipeline_->isImageRequired()?2:1) && transform.isNull();
					++guessIteration)
			{
				tmpMap = *map_;
				// reset matches, but keep already extracted features in lastFrame_->sensorData()
				lastFrame_->setWords(std::multimap<int, cv::KeyPoint>());
				lastFrame_->setWords3(std::multimap<int, cv::Point3f>());
				lastFrame_->setWordsDescriptors(std::multimap<int, cv::Mat>());

				points3DMap.clear();
				bundlePoses.clear();
				bundleLinks.clear();
				bundleModels.clear();

				float maxCorrespondenceDistance = 0.0f;
				float pmOutlierRatio = 0.0f;
				if(guess.isNull() &&
					!regPipeline_->isImageRequired() &&
					regPipeline_->isScanRequired() &&
					this->framesProcessed() < 2)
				{
					// only on initialization (first frame to register), increase icp max correspondences in case the robot is already moving
					maxCorrespondenceDistance = Parameters::defaultIcpMaxCorrespondenceDistance();
					pmOutlierRatio = Parameters::defaultIcpPMOutlierRatio();
					Parameters::parse(parameters_, Parameters::kIcpMaxCorrespondenceDistance(), maxCorrespondenceDistance);
					Parameters::parse(parameters_, Parameters::kIcpPMOutlierRatio(), pmOutlierRatio);
					ParametersMap params;
					params.insert(ParametersPair(Parameters::kIcpMaxCorrespondenceDistance(), uNumber2Str(maxCorrespondenceDistance*3.0f)));
					params.insert(ParametersPair(Parameters::kIcpPMOutlierRatio(), uNumber2Str(0.95f)));
					regPipeline_->parseParameters(params);
				}

				if(guessIteration == 1)
				{
					UWARN("Failed to find a transformation with the provided guess (%s), trying again without a guess.", guess.prettyPrint().c_str());
				}

				transform = regPipeline_->computeTransformationMod(
						tmpMap,
						*lastFrame_,
						// special case for ICP-only odom, set guess to identity if we just started or reset
						guessIteration==0 && !guess.isNull()?this->getPose()*guess:!regPipeline_->isImageRequired()&&this->framesProcessed()<2?this->getPose():Transform(),
						&regInfo);

				if(maxCorrespondenceDistance>0.0f)
				{
					// set it back
					ParametersMap params;
					params.insert(ParametersPair(Parameters::kIcpMaxCorrespondenceDistance(), uNumber2Str(maxCorrespondenceDistance)));
					params.insert(ParametersPair(Parameters::kIcpPMOutlierRatio(), uNumber2Str(pmOutlierRatio)));
					regPipeline_->parseParameters(params);
				}

				data.setFeatures(lastFrame_->sensorData().keypoints(), lastFrame_->sensorData().keypoints3D(), lastFrame_->sensorData().descriptors());

				UDEBUG("Registration time = %fs", regInfo.totalTime);
				if(!transform.isNull())
				{
					// local bundle adjustment
					if(bundleAdjustment_>0 && sba_ &&
					   regPipeline_->isImageRequired() &&
					   lastFrame_->sensorData().cameraModels().size() <= 1 && // multi-cameras not supported
					   regInfo.inliersIDs.size())
					{
						UDEBUG("Local Bundle Adjustment");

						// make sure the IDs of words in the map are not modified (Optical Flow Registration issue)
						UASSERT(map_->getWords().size() && tmpMap.getWords().size());
						if(map_->getWords().size() != tmpMap.getWords().size() ||
						   map_->getWords().begin()->first != tmpMap.getWords().begin()->first ||
						   map_->getWords().rbegin()->first != tmpMap.getWords().rbegin()->first)
						{
							UERROR("Bundle Adjustment cannot be used with a registration approach recomputing features from the \"from\" signature (e.g., Optical Flow).");
							bundleAdjustment_ = 0;
						}
						else
						{
							UASSERT(bundlePoses_.size());
							UASSERT_MSG(bundlePoses_.size()-1 == bundleLinks_.size(), uFormat("poses=%d links=%d", (int)bundlePoses_.size(), (int)bundleLinks_.size()).c_str());
							UASSERT(bundlePoses_.size() == bundleModels_.size());

							bundlePoses = bundlePoses_;
							bundleLinks = bundleLinks_;
							bundleModels = bundleModels_;
							bundleLinks.insert(bundleIMUOrientations_.begin(), bundleIMUOrientations_.end());

							UASSERT_MSG(bundlePoses.find(lastFrame_->id()) == bundlePoses.end(),
									uFormat("Frame %d already added! Make sure the input frames have unique IDs!", lastFrame_->id()).c_str());
							bundleLinks.insert(std::make_pair(bundlePoses_.rbegin()->first, Link(bundlePoses_.rbegin()->first, lastFrame_->id(), Link::kNeighbor, bundlePoses_.rbegin()->second.inverse()*transform, regInfo.covariance.inv())));
							bundlePoses.insert(std::make_pair(lastFrame_->id(), transform));

							if(!imuT.isNull())
							{
								bundleLinks.insert(std::make_pair(lastFrame_->id(), Link(lastFrame_->id(), lastFrame_->id(), Link::kGravity, imuT)));
							}

							CameraModel model;
							if(lastFrame_->sensorData().cameraModels().size() == 1 && lastFrame_->sensorData().cameraModels().at(0).isValidForProjection())
							{
								model = lastFrame_->sensorData().cameraModels()[0];
							}
							else if(lastFrame_->sensorData().stereoCameraModel().isValidForProjection())
							{
								model = lastFrame_->sensorData().stereoCameraModel().left();
								// Set Tx for stereo BA
								model = CameraModel(model.fx(),
										model.fy(),
										model.cx(),
										model.cy(),
										model.localTransform(),
										-lastFrame_->sensorData().stereoCameraModel().baseline()*model.fx());
							}
							else
							{
								UFATAL("no valid camera model to do odometry bundle adjustment!");
							}
							bundleModels.insert(std::make_pair(lastFrame_->id(), model));
							Transform invLocalTransform = model.localTransform().inverse();

							UDEBUG("Fill matches (%d)", (int)regInfo.inliersIDs.size());
							std::map<int, std::map<int, FeatureBA> > wordReferences;
							for(unsigned int i=0; i<regInfo.inliersIDs.size(); ++i)
							{
								int wordId =regInfo.inliersIDs[i];

								// 3D point
								std::multimap<int, cv::Point3f>::const_iterator iter3D = tmpMap.getWords3().find(wordId);
								UASSERT(iter3D!=tmpMap.getWords3().end());
								points3DMap.insert(*iter3D);

								std::multimap<int, cv::KeyPoint>::const_iterator iter2D = lastFrame_->getWords().find(wordId);

								// all other references
								std::map<int, std::map<int, FeatureBA> >::iterator refIter = bundleWordReferences_.find(wordId);
								UASSERT_MSG(refIter != bundleWordReferences_.end(), uFormat("wordId=%d", wordId).c_str());

								std::map<int, FeatureBA> references;
								int step = bundleMaxFrames_>0?(refIter->second.size() / bundleMaxFrames_):1;
								if(step == 0)
								{
									step = 1;
								}
								int oi=0;
								for(std::map<int, FeatureBA>::iterator jter=refIter->second.begin(); jter!=refIter->second.end(); ++jter)
								{
									if(oi++ % step == 0 && bundlePoses.find(jter->first)!=bundlePoses.end())
									{
										references.insert(*jter);
										++totalBundleWordReferencesUsed;
									}
								}
								//make sure the last reference is here
								if(refIter->second.size() > 1)
								{
									if(references.insert(*refIter->second.rbegin()).second)
									{
										++totalBundleWordReferencesUsed;
									}
								}

								if(iter2D!=lastFrame_->getWords().end())
								{
									UASSERT(lastFrame_->getWords3().find(wordId) != lastFrame_->getWords3().end());
									//move back point in camera frame (to get depth along z)
									cv::Point3f pt3d = util3d::transformPoint(lastFrame_->getWords3().find(wordId)->second, invLocalTransform);
									references.insert(std::make_pair(lastFrame_->id(), FeatureBA(iter2D->second, pt3d.z)));
								}
								wordReferences.insert(std::make_pair(wordId, references));

								//UDEBUG("%d (%f,%f,%f)", iter3D->first, iter3D->second.x, iter3D->second.y, iter3D->second.z);
								//for(std::map<int, cv::Point2f>::iterator iter=inserted.first->second.begin(); iter!=inserted.first->second.end(); ++iter)
								//{
								//	UDEBUG("%d (%f,%f)", iter->first, iter->second.x, iter->second.y);
								//}
							}

							UDEBUG("sba...start");
							// set root negative to fix all other poses
							std::set<int> sbaOutliers;
							UTimer bundleTimer;
							bundlePoses = sba_->optimizeBA(-lastFrame_->id(), bundlePoses, bundleLinks, bundleModels, points3DMap, wordReferences, &sbaOutliers);
							bundleTime = bundleTimer.ticks();
							UDEBUG("sba...end");
							totalBundleOutliers = (int)sbaOutliers.size();

							UDEBUG("bundleTime=%fs (poses=%d wordRef=%d outliers=%d)", bundleTime, (int)bundlePoses.size(), (int)bundleWordReferences_.size(), (int)sbaOutliers.size());
							if(info)
							{
								info->localBundlePoses = bundlePoses;
								info->localBundleModels = bundleModels;
							}

							UDEBUG("Local Bundle Adjustment Before: %s", transform.prettyPrint().c_str());
							if(bundlePoses.size() == bundlePoses_.size()+1)
							{
								if(!bundlePoses.rbegin()->second.isNull())
								{
									if(sbaOutliers.size())
									{
										std::vector<int> newInliers(regInfo.inliersIDs.size());
										int oi=0;
										for(unsigned int i=0; i<regInfo.inliersIDs.size(); ++i)
										{
											if(sbaOutliers.find(regInfo.inliersIDs[i]) == sbaOutliers.end())
											{
												newInliers[oi++] = regInfo.inliersIDs[i];
											}
										}
										newInliers.resize(oi);
										UDEBUG("BA outliers ratio %f", float(sbaOutliers.size())/float(regInfo.inliersIDs.size()));
										regInfo.inliers = (int)newInliers.size();
										regInfo.inliersIDs = newInliers;
									}
									if(regInfo.inliers < regPipeline_->getMinVisualCorrespondences())
									{
										regInfo.rejectedMsg = uFormat("Too low inliers after bundle adjustment: %d<%d", regInfo.inliers, regPipeline_->getMinVisualCorrespondences());
										transform.setNull();
									}
									else
									{
										transform = bundlePoses.rbegin()->second;
										std::multimap<int, Link>::iterator iter = graph::findLink(bundleLinks, bundlePoses_.rbegin()->first, lastFrame_->id(), false);
										UASSERT(iter != bundleLinks.end());
										iter->second.setTransform(bundlePoses_.rbegin()->second.inverse()*transform);

										iter = graph::findLink(bundleLinks, lastFrame_->id(), lastFrame_->id(), false);
										if(info && iter!=bundleLinks.end() && iter->second.type() == Link::kGravity)
										{
											float rollImu,pitchImu,yaw;
											iter->second.transform().getEulerAngles(rollImu, pitchImu, yaw);
											float roll,pitch;
											transform.getEulerAngles(roll, pitch, yaw);
											info->gravityRollError = fabs(rollImu - roll);
											info->gravityPitchError = fabs(pitchImu - pitch);
										}
									}
								}
								UDEBUG("Local Bundle Adjustment After : %s", transform.prettyPrint().c_str());
							}
							else
							{
								UWARN("Local bundle adjustment failed! transform is not refined.");
							}
						}
					}

					if(!transform.isNull())
					{
						// make it incremental
						transform = this->getPose().inverse() * transform;
					}
				}

				if(transform.isNull())
				{
					if(guessIteration == 1)
					{
						UWARN("Trial with no guess still fail.");
					}
					if(!regInfo.rejectedMsg.empty())
					{
						if(guess.isNull())
						{
							UWARN("Registration failed: \"%s\"", regInfo.rejectedMsg.c_str());
						}
						else
						{
							UWARN("Registration failed: \"%s\" (guess=%s)", regInfo.rejectedMsg.c_str(), guess.prettyPrint().c_str());
						}
					}
					else
					{
						UWARN("Unknown registration error");
					}
				}
				else if(guessIteration == 1)
				{
					UWARN("Trial with no guess succeeded!");
				}
			}

			if(!transform.isNull())
			{
				output = transform;

				bool modified = false;
				Transform newFramePose = this->getPose()*output;

				// fields to update
				LaserScan mapScan = tmpMap.sensorData().laserScanRaw();
				std::multimap<int, cv::KeyPoint> mapWords = tmpMap.getWords();
				std::multimap<int, cv::Point3f> mapPoints = tmpMap.getWords3();
				std::multimap<int, cv::Mat> mapDescriptors = tmpMap.getWordsDescriptors();

				bool addVisualKeyFrame = regPipeline_->isImageRequired() &&
						 (keyFrameThr_ == 0.0f ||
						  visKeyFrameThr_ == 0 ||
						  float(regInfo.inliers) <= (keyFrameThr_*float(lastFrame_->getWords().size())) ||
						  regInfo.inliers <= visKeyFrameThr_);
				float minComplexity = Parameters::defaultIcpPointToPlaneMinComplexity();
				bool p2n = Parameters::defaultIcpPointToPlane();
				Parameters::parse(parameters_, Parameters::kIcpPointToPlane(), p2n);
				Parameters::parse(parameters_, Parameters::kIcpPointToPlaneMinComplexity(), minComplexity);
				bool addGeometricKeyFrame =
					regPipeline_->isScanRequired() &&
					(scanKeyFrameThr_==0 || regInfo.icpInliersRatio <= scanKeyFrameThr_) &&
					(addVisualKeyFrame || !p2n || regInfo.icpStructuralComplexity>=minComplexity);

				addKeyFrame = false;//bundleLinks.rbegin()->second.transform().getNorm() > 5.0f*0.075f;
				addKeyFrame = addKeyFrame || addVisualKeyFrame || addGeometricKeyFrame;

				UDEBUG("keyframeThr=%f visKeyFrameThr_=%d matches=%d inliers=%d features=%d mp=%d", keyFrameThr_, visKeyFrameThr_, regInfo.matches, regInfo.inliers, (int)lastFrame_->sensorData().keypoints().size(), (int)mapPoints.size());
				if(addKeyFrame)
				{
					//Visual
					int added = 0;
					int removed = 0;
					UTimer tmpTimer;

					UDEBUG("Update local map");

					// update local map
					UASSERT(mapWords.size() == mapPoints.size());
					UASSERT(mapPoints.size() == mapDescriptors.size());
					UASSERT_MSG(lastFrame_->getWordsDescriptors().size() == lastFrame_->getWords3().size(), uFormat("%d vs %d", lastFrame_->getWordsDescriptors().size(), lastFrame_->getWords3().size()).c_str());

					std::map<int, int>::iterator iterBundlePosesRef = bundlePoseReferences_.end();
					if(bundleAdjustment_>0)
					{
						bundlePoseReferences_.insert(std::make_pair(lastFrame_->id(), 0));
						std::multimap<int, Link>::iterator iter = graph::findLink(bundleLinks, bundlePoses_.rbegin()->first, lastFrame_->id(), false);
						UASSERT(iter != bundleLinks.end());
						bundleLinks_.insert(*iter);
						iter = graph::findLink(bundleLinks, lastFrame_->id(), lastFrame_->id(), false);
						if(iter != bundleLinks.end())
						{
							bundleIMUOrientations_.insert(*iter);
						}
						uInsert(bundlePoses_, bundlePoses);
						UASSERT(bundleModels.find(lastFrame_->id()) != bundleModels.end());
						bundleModels_.insert(*bundleModels.find(lastFrame_->id()));
						iterBundlePosesRef = bundlePoseReferences_.find(lastFrame_->id());

						// update local map 3D points (if bundle adjustment was done)
						for(std::map<int, cv::Point3f>::iterator iter=points3DMap.begin(); iter!=points3DMap.end(); ++iter)
						{
							UASSERT(mapPoints.count(iter->first) == 1);
							//UDEBUG("Updated %d (%f,%f,%f) -> (%f,%f,%f)", iter->first, mapPoints.find(origin)->second.x, mapPoints.find(origin)->second.y, mapPoints.find(origin)->second.z, iter->second.x, iter->second.y, iter->second.z);
							mapPoints.find(iter->first)->second = iter->second;
						}
					}

					// sort by feature response
					std::multimap<float, std::pair<int, std::pair<cv::KeyPoint, std::pair<cv::Point3f, cv::Mat> > > > newIds;
					UASSERT(lastFrame_->getWords3().size() == lastFrame_->getWords().size());
					std::multimap<int, cv::KeyPoint>::const_iterator iter2D = lastFrame_->getWords().begin();
					std::multimap<int, cv::Mat>::const_iterator iterDesc = lastFrame_->getWordsDescriptors().begin();
					UDEBUG("new frame words3=%d", (int)lastFrame_->getWords3().size());
					std::set<int> seenStatusUpdated;
					Transform invLocalTransform;
					if(bundleAdjustment_>0)
					{
						if(lastFrame_->sensorData().cameraModels().size() == 1 && lastFrame_->sensorData().cameraModels().at(0).isValidForProjection())
						{
							invLocalTransform = lastFrame_->sensorData().cameraModels()[0].localTransform().inverse();
						}
						else if(lastFrame_->sensorData().stereoCameraModel().isValidForProjection())
						{
							invLocalTransform = lastFrame_->sensorData().stereoCameraModel().left().localTransform().inverse();
						}
						else
						{
							UFATAL("no valid camera model!");
						}
					}

					// add points without depth only if the local map has reached its maximum size
					bool addPointsWithoutDepth = false;
					if(!visDepthAsMask && validDepthRatio_ < 1.0f)
					{
						int ptsWithDepth = 0;
						for (std::multimap<int, cv::Point3f>::const_iterator iter = lastFrame_->getWords3().begin();
							iter != lastFrame_->getWords3().end();
							++iter)
						{
							if(util3d::isFinite(iter->second))
							{
								++ptsWithDepth;
							}
						}
						float r = float(ptsWithDepth) / float(lastFrame_->getWords3().size());
						addPointsWithoutDepth = r > validDepthRatio_;
						if(!addPointsWithoutDepth)
						{
							UWARN("Not enough points with valid depth in current frame (%d/%d=%f < %s=%f), points without depth are not added to map.",
									ptsWithDepth, (int)lastFrame_->getWords3().size(), r, Parameters::kOdomF2MValidDepthRatio().c_str(), validDepthRatio_);
						}
					}

					for(std::multimap<int, cv::Point3f>::const_iterator iter = lastFrame_->getWords3().begin(); iter!=lastFrame_->getWords3().end(); ++iter, ++iter2D, ++iterDesc)
					{
						if(mapPoints.find(iter->first) == mapPoints.end()) // Point not in map
						{
							if(util3d::isFinite(iter->second) || addPointsWithoutDepth)
							{
								newIds.insert(
										std::make_pair(iter2D->second.response>0?1.0f/iter2D->second.response:0.0f,
												std::make_pair(iter->first,
														std::make_pair(iter2D->second,
																std::make_pair(iter->second, iterDesc->second)))));
							}
						}
						else if(bundleAdjustment_>0)
						{
							if(lastFrame_->getWords().count(iter->first) == 1)
							{
								std::multimap<int, cv::KeyPoint>::iterator iterKpts = mapWords.find(iter->first);
								if(iterKpts!=mapWords.end())
								{
									iterKpts->second.octave = iter2D->second.octave;
								}

								UASSERT(iterBundlePosesRef!=bundlePoseReferences_.end());
								iterBundlePosesRef->second += 1;

								//move back point in camera frame (to get depth along z)
								float depth = 0.0f;
								if(util3d::isFinite(iter->second))
								{
									depth = util3d::transformPoint(iter->second, invLocalTransform).z;
								}
								if(bundleWordReferences_.find(iter->first) == bundleWordReferences_.end())
								{
									std::map<int, FeatureBA> framePt;
									framePt.insert(std::make_pair(lastFrame_->id(), FeatureBA(iter2D->second, depth)));
									bundleWordReferences_.insert(std::make_pair(iter->first, framePt));
								}
								else
								{
									bundleWordReferences_.find(iter->first)->second.insert(std::make_pair(lastFrame_->id(), FeatureBA(iter2D->second, depth)));
								}
							}
						}
					}
					UDEBUG("newIds=%d", (int)newIds.size());

					int lastFrameOldestNewId = lastFrameOldestNewId_;
					lastFrameOldestNewId_ = lastFrame_->getWords().size()?lastFrame_->getWords().rbegin()->first:0;
					for(std::multimap<float, std::pair<int, std::pair<cv::KeyPoint, std::pair<cv::Point3f, cv::Mat> > > >::reverse_iterator iter=newIds.rbegin();
						iter!=newIds.rend();
						++iter)
					{
						if(maxNewFeatures_ == 0  || added < maxNewFeatures_)
						{
							if(bundleAdjustment_>0)
							{
								if(lastFrame_->getWords().count(iter->second.first) == 1)
								{
									UASSERT(iterBundlePosesRef!=bundlePoseReferences_.end());
									iterBundlePosesRef->second += 1;

									//move back point in camera frame (to get depth along z)
									float depth = 0.0f;
									if(util3d::isFinite(iter->second.second.second.first))
									{
										depth = util3d::transformPoint(iter->second.second.second.first, invLocalTransform).z;
									}
									if(bundleWordReferences_.find(iter->second.first) == bundleWordReferences_.end())
									{
										std::map<int, FeatureBA> framePt;
										framePt.insert(std::make_pair(lastFrame_->id(), FeatureBA(iter->second.second.first, depth)));
										bundleWordReferences_.insert(std::make_pair(iter->second.first, framePt));
									}
									else
									{
										bundleWordReferences_.find(iter->second.first)->second.insert(std::make_pair(lastFrame_->id(), FeatureBA(iter->second.second.first, depth)));
									}
								}
							}

							mapWords.insert(std::make_pair(iter->second.first, iter->second.second.first));
							cv::Point3f pt = iter->second.second.second.first;
							if(!util3d::isFinite(pt))
							{
								// get the ray instead
								float x = iter->second.second.first.pt.x;
								float y = iter->second.second.first.pt.y;
								float subImageWidth = lastFrame_->sensorData().imageRaw().cols;
								CameraModel model;
								if(lastFrame_->sensorData().cameraModels().size() > 1)
								{
									subImageWidth = lastFrame_->sensorData().imageRaw().cols/lastFrame_->sensorData().cameraModels().size();
									int cameraIndex = int(x / subImageWidth);
									model = lastFrame_->sensorData().cameraModels()[cameraIndex];
									x = x-subImageWidth*cameraIndex;
								}
								else if(lastFrame_->sensorData().cameraModels().size() == 1)
								{
									model = lastFrame_->sensorData().cameraModels()[0];
								}
								else
								{
									model = lastFrame_->sensorData().stereoCameraModel().left();
								}

								Eigen::Vector3f ray = util3d::projectDepthTo3DRay(
										model.imageSize(),
										x,
										y,
										model.cx(),
										model.cy(),
										model.fx(),
										model.fy());
								float scaleInf = (0.05 * model.fx()) / 0.01;
								pt = util3d::transformPoint(cv::Point3f(ray[0]*scaleInf, ray[1]*scaleInf, ray[2]*scaleInf), model.localTransform()); // in base_link frame
							}
							mapPoints.insert(std::make_pair(iter->second.first, util3d::transformPoint(pt, newFramePose)));
							mapDescriptors.insert(std::make_pair(iter->second.first, iter->second.second.second.second));
							if(lastFrameOldestNewId_ > iter->second.first)
							{
								lastFrameOldestNewId_ = iter->second.first;
							}
							++added;
						}
					}

					// remove words in map if max size is reached
					if((int)mapPoints.size() > maximumMapSize_)
					{
						// remove oldest outliers first
						std::set<int> inliers(regInfo.inliersIDs.begin(), regInfo.inliersIDs.end());
						std::vector<int> ids = regInfo.matchesIDs;
						if(regInfo.projectedIDs.size())
						{
							ids.resize(ids.size() + regInfo.projectedIDs.size());
							int oi=0;
							for(unsigned int i=0; i<regInfo.projectedIDs.size(); ++i)
							{
								if(regInfo.projectedIDs[i]>=lastFrameOldestNewId)
								{
									ids[regInfo.matchesIDs.size()+oi++] = regInfo.projectedIDs[i];
								}
							}
							ids.resize(regInfo.matchesIDs.size()+oi);
							UDEBUG("projected added=%d/%d minLastFrameId=%d", oi, (int)regInfo.projectedIDs.size(), lastFrameOldestNewId);
						}
						for(unsigned int i=0; i<ids.size() && (int)mapPoints.size() > maximumMapSize_ && mapPoints.size() >= newIds.size(); ++i)
						{
							int id = ids.at(i);
							if(inliers.find(id) == inliers.end())
							{
								std::map<int, std::map<int, FeatureBA> >::iterator iterRef = bundleWordReferences_.find(id);
								if(iterRef != bundleWordReferences_.end())
								{
									for(std::map<int, FeatureBA>::iterator iterFrame = iterRef->second.begin(); iterFrame != iterRef->second.end(); ++iterFrame)
									{
										if(bundlePoseReferences_.find(iterFrame->first) != bundlePoseReferences_.end())
										{
											bundlePoseReferences_.at(iterFrame->first) -= 1;
										}
									}
									bundleWordReferences_.erase(iterRef);
								}

								mapPoints.erase(id);
								mapDescriptors.erase(id);
								mapWords.erase(id);
								++removed;
							}
						}

						// remove oldest first
						std::multimap<int, cv::Mat>::iterator iterMapDescriptors = mapDescriptors.begin();
						std::multimap<int, cv::KeyPoint>::iterator iterMapWords = mapWords.begin();
						for(std::multimap<int, cv::Point3f>::iterator iter = mapPoints.begin();
							iter!=mapPoints.end() && (int)mapPoints.size() > maximumMapSize_ && mapPoints.size() >= newIds.size();)
						{
							if(inliers.find(iter->first) == inliers.end())
							{
								std::map<int, std::map<int, FeatureBA> >::iterator iterRef = bundleWordReferences_.find(iter->first);
								if(iterRef != bundleWordReferences_.end())
								{
									for(std::map<int, FeatureBA>::iterator iterFrame = iterRef->second.begin(); iterFrame != iterRef->second.end(); ++iterFrame)
									{
										if(bundlePoseReferences_.find(iterFrame->first) != bundlePoseReferences_.end())
										{
											bundlePoseReferences_.at(iterFrame->first) -= 1;
										}
									}
									bundleWordReferences_.erase(iterRef);
								}

								mapPoints.erase(iter++);
								mapDescriptors.erase(iterMapDescriptors++);
								mapWords.erase(iterMapWords++);
								++removed;
							}
							else
							{
								++iter;
								++iterMapDescriptors;
								++iterMapWords;
							}
						}

						Link * previousLink = 0;
						for(std::map<int, int>::iterator iter=bundlePoseReferences_.begin(); iter!=bundlePoseReferences_.end();)
						{
							if(iter->second <= 0)
							{
								if(previousLink == 0 || bundleLinks_.find(iter->first) != bundleLinks_.end())
								{
									if(previousLink)
									{
										UASSERT(previousLink->to() == iter->first);
										*previousLink = previousLink->merge(bundleLinks_.find(iter->first)->second, previousLink->type());
									}
									UASSERT(bundlePoses_.erase(iter->first) == 1);
									bundleLinks_.erase(iter->first);
									bundleModels_.erase(iter->first);
									bundleIMUOrientations_.erase(iter->first);
									bundlePoseReferences_.erase(iter++);
								}
							}
							else
							{
								previousLink=0;
								if(bundleLinks_.find(iter->first) != bundleLinks_.end())
								{
									previousLink = &bundleLinks_.find(iter->first)->second;
								}
								++iter;
							}
						}
					}

					if(added || removed)
					{
						modified = true;
					}
					UDEBUG("Update local features map = %fs", tmpTimer.ticks());

					// Geometric
					UDEBUG("scankeyframeThr=%f icpInliersRatio=%f", scanKeyFrameThr_, regInfo.icpInliersRatio);
					UINFO("Update local scan map %d (ratio=%f < %f)", lastFrame_->id(), regInfo.icpInliersRatio, scanKeyFrameThr_);

					if(lastFrame_->sensorData().laserScanRaw().size())
					{
						pcl::PointCloud<pcl::PointNormal>::Ptr mapCloudNormals = util3d::laserScanToPointCloudNormal(mapScan, tmpMap.sensorData().laserScanRaw().localTransform());
						Transform viewpoint =  newFramePose * lastFrame_->sensorData().laserScanRaw().localTransform();
						pcl::PointCloud<pcl::PointNormal>::Ptr frameCloudNormals (new pcl::PointCloud<pcl::PointNormal>());
						
						if(scanMapMaxRange_ > 0)
						{
							frameCloudNormals = util3d::laserScanToPointCloudNormal(
									lastFrame_->sensorData().laserScanRaw());
							frameCloudNormals = util3d::cropBox(frameCloudNormals,
									Eigen::Vector4f(-scanMapMaxRange_ / 2, -scanMapMaxRange_ / 2,-scanMapMaxRange_ / 2, 0),
									Eigen::Vector4f(scanMapMaxRange_ / 2,scanMapMaxRange_ / 2,scanMapMaxRange_ / 2, 0)
									);
							frameCloudNormals = util3d::transformPointCloud(frameCloudNormals, viewpoint);
						} else
						{
							frameCloudNormals = util3d::laserScanToPointCloudNormal(
									lastFrame_->sensorData().laserScanRaw(), viewpoint);
						}
						
						pcl::IndicesPtr frameCloudNormalsIndices(new std::vector<int>);
						int newPoints;
						if(mapCloudNormals->size() && scanSubtractRadius_ > 0.0f)
						{
							// remove points that overlap (the ones found in both clouds)
							frameCloudNormalsIndices = util3d::subtractFiltering(
									frameCloudNormals,
									pcl::IndicesPtr(new std::vector<int>),
									mapCloudNormals,
									pcl::IndicesPtr(new std::vector<int>),
									scanSubtractRadius_,
									lastFrame_->sensorData().laserScanRaw().hasNormals()&&mapScan.hasNormals()?scanSubtractAngle_:0.0f);
							newPoints = frameCloudNormalsIndices->size();
						}
						else
						{
							newPoints = mapCloudNormals->size();
						}

						if(newPoints)
						{
							if (scanMapMaxRange_ > 0) {
								// Copying new points to tmp cloud
								// These are the points that have no overlap between mapScan and lastFrame
								pcl::PointCloud<pcl::PointNormal> tmp;
								pcl::copyPointCloud(*frameCloudNormals, *frameCloudNormalsIndices, tmp);

								if (int(mapCloudNormals->size() + newPoints) > scanMaximumMapSize_) // 20 000 points
								{
									// Print mapSize
									UINFO("mapSize=%d newPoints=%d maxPoints=%d",
										  int(mapCloudNormals->size()),
										  newPoints,
										  scanMaximumMapSize_);

									*mapCloudNormals += tmp;
									cv::Point3f boxMin (-scanMapMaxRange_/2, -scanMapMaxRange_/2, -scanMapMaxRange_/2);
									cv::Point3f boxMax (scanMapMaxRange_/2, scanMapMaxRange_/2, scanMapMaxRange_/2);

									boxMin = util3d::transformPoint(boxMin, viewpoint.translation());
									boxMax = util3d::transformPoint(boxMax, viewpoint.translation());

									mapCloudNormals = util3d::cropBox(mapCloudNormals, Eigen::Vector4f(boxMin.x, boxMin.y, boxMin.z, 0 ), Eigen::Vector4f(boxMax.x, boxMax.y, boxMax.z, 0 ));

								} else {
									*mapCloudNormals += tmp;
								}

								mapCloudNormals = util3d::voxelize(mapCloudNormals, scanSubtractRadius_);
								pcl::PointCloud<pcl::PointXYZI>::Ptr mapCloud (new pcl::PointCloud<pcl::PointXYZI> ());
								copyPointCloud(*mapCloudNormals, *mapCloud);
								pcl::PointCloud<pcl::Normal>::Ptr normals = util3d::computeNormals(mapCloud, pointToPlaneK_, pointToPlaneRadius_, Eigen::Vector3f(viewpoint.x(), viewpoint.y(), viewpoint.z()));
								copyPointCloud(*normals, *mapCloudNormals);

							} else {
								scansBuffer_.push_back(std::make_pair(frameCloudNormals, frameCloudNormalsIndices));

								//remove points if too big
								UDEBUG("scansBuffer=%d, mapSize=%d newPoints=%d maxPoints=%d",
									   (int)scansBuffer_.size(),
									   int(mapCloudNormals->size()),
									   newPoints,
									   scanMaximumMapSize_);

								if(scansBuffer_.size() > 1 &&
								   int(mapCloudNormals->size() + newPoints) > scanMaximumMapSize_)
								{
									//regenerate the local map
									mapCloudNormals->clear();
									std::list<int> toRemove;
									int i = int(scansBuffer_.size())-1;
									for(; i>=0; --i)
									{
										int pointsToAdd = scansBuffer_[i].second->size()?scansBuffer_[i].second->size():scansBuffer_[i].first->size();
										if((int)mapCloudNormals->size() + pointsToAdd > scanMaximumMapSize_ ||
										   i == 0)
										{
											*mapCloudNormals += *scansBuffer_[i].first;
											break;
										}
										else
										{
											if(scansBuffer_[i].second->size())
											{
												pcl::PointCloud<pcl::PointNormal> tmp;
												pcl::copyPointCloud(*scansBuffer_[i].first, *scansBuffer_[i].second, tmp);
												*mapCloudNormals += tmp;
											}
											else
											{
												*mapCloudNormals += *scansBuffer_[i].first;
											}
										}
									}
									// remove old clouds
									if(i > 0)
									{
										std::vector<std::pair<pcl::PointCloud<pcl::PointNormal>::Ptr, pcl::IndicesPtr> > scansTmp(scansBuffer_.size()-i);
										int oi = 0;
										for(; i<(int)scansBuffer_.size(); ++i)
										{
											UASSERT(oi < (int)scansTmp.size());
											scansTmp[oi++] = scansBuffer_[i];
										}
										scansBuffer_ = scansTmp;
									}
								}
								else
								{
									// just append the last cloud
									if(scansBuffer_.back().second->size())
									{
										pcl::PointCloud<pcl::PointNormal> tmp;
										pcl::copyPointCloud(*scansBuffer_.back().first, *scansBuffer_.back().second, tmp);
										*mapCloudNormals += tmp;
									}
									else
									{
										*mapCloudNormals += *scansBuffer_.back().first;
									}
								}
							}

							if(mapScan.is2d())
							{
								Transform mapViewpoint(-newFramePose.x(), -newFramePose.y(),0,0,0,0);
								mapScan = LaserScan(util3d::laserScan2dFromPointCloud(*mapCloudNormals, mapViewpoint), 0, 0.0f, LaserScan::kXYNormal);
							}
							else
							{
								Transform mapViewpoint(-newFramePose.x(), -newFramePose.y(), -newFramePose.z(),0,0,0);
								mapScan = LaserScan(util3d::laserScanFromPointCloud(*mapCloudNormals, mapViewpoint), 0, 0.0f, LaserScan::kXYZNormal);
							}
							modified=true;
						}
					}
					UDEBUG("Update local scan map = %fs", tmpTimer.ticks());
				}

				if(modified)
				{
					*map_ = tmpMap;

					if(mapScan.is2d())
					{

						map_->sensorData().setLaserScan(
								LaserScan(
										mapScan.data(),
										0,
										0.0f,
										mapScan.format(),
										Transform(newFramePose.x(), newFramePose.y(), lastFrame_->sensorData().laserScanRaw().localTransform().z(),0,0,0)));
					}
					else
					{
						map_->sensorData().setLaserScan(
								LaserScan(
										mapScan.data(),
										0,
										0.0f,
										mapScan.format(),
										newFramePose.translation()));
					}

					map_->setWords(mapWords);
					map_->setWords3(mapPoints);
				 	map_->setWordsDescriptors(mapDescriptors);
				}
			}

			if(info)
			{
				// use tmpMap instead of map_ to make sure that correspondences with the new frame matches
				info->localMapSize = (int)tmpMap.getWords3().size();
				info->localScanMapSize = tmpMap.sensorData().laserScanRaw().size();
				if(this->isInfoDataFilled())
				{
					info->localMap = uMultimapToMap(tmpMap.getWords3());
					info->localScanMap = tmpMap.sensorData().laserScanRaw();
				}
			}
		}
		else
		{
			// just generate keypoints for the new signature
			if(regPipeline_->isImageRequired())
			{
				Signature dummy;
				regPipeline_->computeTransformationMod(
						*lastFrame_,
						dummy);
			}

			data.setFeatures(lastFrame_->sensorData().keypoints(), lastFrame_->sensorData().keypoints3D(), lastFrame_->sensorData().descriptors());

			// a very high variance tells that the new pose is not linked with the previous one
			regInfo.covariance = cv::Mat::eye(6,6,CV_64FC1)*9999.0;

			bool frameValid = false;
			Transform newFramePose = this->getPose(); // initial pose may be not identity...

			if(regPipeline_->isImageRequired())
			{
				int ptsWithDepth = 0;
				for (std::multimap<int, cv::Point3f>::const_iterator iter = lastFrame_->getWords3().begin();
					iter != lastFrame_->getWords3().end();
					++iter)
				{
					if(util3d::isFinite(iter->second))
					{
						++ptsWithDepth;
					}
				}

				if (ptsWithDepth >= regPipeline_->getMinVisualCorrespondences())
				{
					frameValid = true;
					// update local map
					UASSERT_MSG(lastFrame_->getWordsDescriptors().size() == lastFrame_->getWords3().size(), uFormat("%d vs %d", lastFrame_->getWordsDescriptors().size(), lastFrame_->getWords3().size()).c_str());
					UASSERT(lastFrame_->getWords3().size() == lastFrame_->getWords().size());

					std::multimap<int, cv::KeyPoint> words;
					std::multimap<int, cv::Point3f> transformedPoints;
					std::multimap<int, int> mapPointWeights;
					std::multimap<int, cv::Mat> descriptors;
					UASSERT(lastFrame_->getWords3().size() == lastFrame_->getWordsDescriptors().size());
					std::multimap<int, cv::KeyPoint>::const_iterator wordsIter = lastFrame_->getWords().begin();
					std::multimap<int, cv::Mat>::const_iterator descIter = lastFrame_->getWordsDescriptors().begin();
					for (std::multimap<int, cv::Point3f>::const_iterator iter = lastFrame_->getWords3().begin();
						iter != lastFrame_->getWords3().end();
						++iter, ++descIter, ++wordsIter)
					{
						if (util3d::isFinite(iter->second))
						{
							words.insert(*wordsIter);
							transformedPoints.insert(std::make_pair(iter->first, util3d::transformPoint(iter->second, newFramePose)));
							mapPointWeights.insert(std::make_pair(iter->first, 0));
							descriptors.insert(*descIter);
						}
					}

					if(bundleAdjustment_>0)
					{
						Transform invLocalTransform;
						if(lastFrame_->sensorData().cameraModels().size() == 1 && lastFrame_->sensorData().cameraModels().at(0).isValidForProjection())
						{
							invLocalTransform = lastFrame_->sensorData().cameraModels()[0].localTransform().inverse();
						}
						else if(lastFrame_->sensorData().stereoCameraModel().isValidForProjection())
						{
							invLocalTransform = lastFrame_->sensorData().stereoCameraModel().left().localTransform().inverse();
						}
						else
						{
							UFATAL("no valid camera model!");
						}

						// update bundleWordReferences_: used for bundle adjustment
						for(std::multimap<int, cv::KeyPoint>::const_iterator iter=words.begin(); iter!=words.end(); ++iter)
						{
							if(words.count(iter->first) == 1)
							{
								UASSERT(bundleWordReferences_.find(iter->first) == bundleWordReferences_.end());
								std::map<int, FeatureBA> framePt;

								//get depth
								float d = 0.0f;
								if(lastFrame_->getWords3().count(iter->first) == 1 &&
									util3d::isFinite(lastFrame_->getWords3().find(iter->first)->second))
								{
									//move back point in camera frame (to get depth along z)
									d = util3d::transformPoint(lastFrame_->getWords3().find(iter->first)->second, invLocalTransform).z;
								}


								framePt.insert(std::make_pair(lastFrame_->id(), FeatureBA(iter->second, d)));
								bundleWordReferences_.insert(std::make_pair(iter->first, framePt));
							}
						}

						bundlePoseReferences_.insert(std::make_pair(lastFrame_->id(), (int)bundleWordReferences_.size()));

						CameraModel model;
						if(lastFrame_->sensorData().cameraModels().size() == 1 && lastFrame_->sensorData().cameraModels().at(0).isValidForProjection())
						{
							model = lastFrame_->sensorData().cameraModels()[0];
						}
						else if(lastFrame_->sensorData().stereoCameraModel().isValidForProjection())
						{
							model = lastFrame_->sensorData().stereoCameraModel().left();
							// Set Tx for stereo BA
							model = CameraModel(model.fx(),
									model.fy(),
									model.cx(),
									model.cy(),
									model.localTransform(),
									-lastFrame_->sensorData().stereoCameraModel().baseline()*model.fx());
						}
						else
						{
							UFATAL("invalid camera model!");
						}
						bundleModels_.insert(std::make_pair(lastFrame_->id(), model));
						bundlePoses_.insert(std::make_pair(lastFrame_->id(), newFramePose));

						if(!imuT.isNull())
						{
							bundleIMUOrientations_.insert(std::make_pair(lastFrame_->id(), Link(lastFrame_->id(), lastFrame_->id(), Link::kGravity, newFramePose)));
						}
					}

					map_->setWords(words);
					map_->setWords3(transformedPoints);
					map_->setWordsDescriptors(descriptors);
					addKeyFrame = true;
				}
				else
				{
					UWARN("%d visual features required to initialize the odometry (only %d extracted).", regPipeline_->getMinVisualCorrespondences(), (int)lastFrame_->getWords3().size());
				}
			}
			if(regPipeline_->isScanRequired())
			{
				if (lastFrame_->sensorData().laserScanRaw().size())
				{
					pcl::PointCloud<pcl::PointNormal>::Ptr mapCloudNormals = util3d::laserScanToPointCloudNormal(lastFrame_->sensorData().laserScanRaw(), newFramePose * lastFrame_->sensorData().laserScanRaw().localTransform());

					double complexity = 0.0;;
					if(!frameValid)
					{
						float minComplexity = Parameters::defaultIcpPointToPlaneMinComplexity();
						bool p2n = Parameters::defaultIcpPointToPlane();
						Parameters::parse(parameters_, Parameters::kIcpPointToPlane(), p2n);
						Parameters::parse(parameters_, Parameters::kIcpPointToPlaneMinComplexity(), minComplexity);
						if(p2n && minComplexity>0.0f)
						{
							complexity = util3d::computeNormalsComplexity(*mapCloudNormals, Transform::getIdentity(), lastFrame_->sensorData().laserScanRaw().is2d());
							if(complexity > minComplexity)
							{
								frameValid = true;
							}
						}
						else
						{
							frameValid = true;
						}
					}

					if(frameValid)
					{
						if (scanMapMaxRange_ > 0 ){
							UINFO("Local map will be updated using range instead of time with range threshold set at %f", scanMapMaxRange_);
						} else {
							scansBuffer_.push_back(std::make_pair(mapCloudNormals, pcl::IndicesPtr(new std::vector<int>)));
						}
						if(lastFrame_->sensorData().laserScanRaw().is2d())
						{
							Transform mapViewpoint(-newFramePose.x(), -newFramePose.y(),0,0,0,0);
							map_->sensorData().setLaserScan(
									LaserScan(
											util3d::laserScan2dFromPointCloud(*mapCloudNormals, mapViewpoint),
											0,
											0.0f,
											LaserScan::kXYNormal,
											Transform(newFramePose.x(), newFramePose.y(), lastFrame_->sensorData().laserScanRaw().localTransform().z(),0,0,0)));
						}
						else
						{
							Transform mapViewpoint(-newFramePose.x(), -newFramePose.y(), -newFramePose.z(),0,0,0);
							map_->sensorData().setLaserScan(
									LaserScan(
											util3d::laserScanFromPointCloud(*mapCloudNormals, mapViewpoint),
											0,
											0.0f,
											LaserScan::kXYZNormal,
											newFramePose.translation()));
						}

						addKeyFrame = true;
					}
					else
					{
						UWARN("Scan complexity too low (%f) to init first keyframe.", complexity);
					}
				}
				else
				{
					UWARN("Missing scan to initialize odometry.");
				}
			}

			if (frameValid)
			{
				// We initialized the local map
				output.setIdentity();
			}

			if(info)
			{
				info->localMapSize = (int)map_->getWords3().size();
				info->localScanMapSize = map_->sensorData().laserScanRaw().size();

				if(this->isInfoDataFilled())
				{
					info->localMap = uMultimapToMap(map_->getWords3());
					info->localScanMap = map_->sensorData().laserScanRaw();
				}
			}
		}

		map_->sensorData().setFeatures(std::vector<cv::KeyPoint>(), std::vector<cv::Point3f>(), cv::Mat()); // clear sensorData features

		nFeatures = lastFrame_->getWords().size();
		if(this->isInfoDataFilled() && info)
		{
			if(regPipeline_->isImageRequired())
			{
				info->words = lastFrame_->getWords();
			}
		}
	}

	if(info)
	{
		info->features = nFeatures;
		info->localKeyFrames = (int)bundlePoses_.size();
		info->keyFrameAdded = addKeyFrame;
		info->localBundleOutliers = totalBundleOutliers;
		info->localBundleConstraints = totalBundleWordReferencesUsed;
		info->localBundleTime = bundleTime;

		if(this->isInfoDataFilled())
		{
			info->reg = regInfo;
		}
		else
		{
			info->reg = regInfo.copyWithoutData();
		}
	}

	UINFO("Odom update time = %fs lost=%s features=%d inliers=%d/%d variance:lin=%f, ang=%f local_map=%d local_scan_map=%d",
			timer.elapsed(),
			output.isNull()?"true":"false",
			nFeatures,
			regInfo.inliers,
			regInfo.matches,
			regInfo.covariance.at<double>(0,0),
			regInfo.covariance.at<double>(5,5),
			regPipeline_->isImageRequired()?(int)map_->getWords3().size():0,
			regPipeline_->isScanRequired()?(int)map_->sensorData().laserScanRaw().size():0);

	return output;
}

} // namespace rtabmap
