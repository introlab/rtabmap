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
#include "rtabmap/core/VisualWord.h"
#include "rtabmap/core/Signature.h"
#include "rtabmap/core/RegistrationVis.h"
#include "rtabmap/core/util3d_transforms.h"
#include "rtabmap/core/util3d_registration.h"
#include "rtabmap/core/util3d_correspondences.h"
#include "rtabmap/core/util3d_motion_estimation.h"
#include "rtabmap/core/util3d_filtering.h"
#include "rtabmap/core/Optimizer.h"
#include "rtabmap/core/VWDictionary.h"
#include "rtabmap/core/util3d.h"
#include "rtabmap/utilite/ULogger.h"
#include "rtabmap/utilite/UTimer.h"
#include "rtabmap/utilite/UMath.h"
#include "rtabmap/utilite/UConversion.h"
#include <opencv2/calib3d/calib3d.hpp>
#include <rtabmap/core/OdometryF2M.h>
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
	maxNewFeatures_(Parameters::defaultOdomF2MMaxNewFeatures()),
	scanKeyFrameThr_(Parameters::defaultOdomScanKeyFrameThr()),
	scanMaximumMapSize_(Parameters::defaultOdomF2MScanMaxSize()),
	scanSubtractRadius_(Parameters::defaultOdomF2MScanSubtractRadius()),
	bundleAdjustment_(Parameters::defaultOdomF2MBundleAdjustment()),
	bundleAdjustmentMaxFrames_(Parameters::defaultOdomF2MBundleAdjustmentMaxFrames()),
	regPipeline_(Registration::create(parameters)),
	map_(new Signature(-1)),
	lastFrame_(new Signature(1))
{
	UDEBUG("");
	Parameters::parse(parameters, Parameters::kOdomF2MMaxSize(), maximumMapSize_);
	Parameters::parse(parameters, Parameters::kOdomKeyFrameThr(), keyFrameThr_);
	Parameters::parse(parameters, Parameters::kOdomF2MMaxNewFeatures(), maxNewFeatures_);
	Parameters::parse(parameters, Parameters::kOdomScanKeyFrameThr(), scanKeyFrameThr_);
	Parameters::parse(parameters, Parameters::kOdomF2MScanMaxSize(), scanMaximumMapSize_);
	Parameters::parse(parameters, Parameters::kOdomF2MScanSubtractRadius(), scanSubtractRadius_);
	Parameters::parse(parameters, Parameters::kOdomF2MBundleAdjustment(), bundleAdjustment_);
	Parameters::parse(parameters, Parameters::kOdomF2MBundleAdjustmentMaxFrames(), bundleAdjustmentMaxFrames_);
	bundleParameters_ = parameters;
	UASSERT(maximumMapSize_ >= 0);
	UASSERT(keyFrameThr_ >= 0.0f && keyFrameThr_<=1.0f);
	UASSERT(scanKeyFrameThr_ >= 0.0f && scanKeyFrameThr_<=1.0f);
	UASSERT(maxNewFeatures_ >= 0);
}

OdometryF2M::~OdometryF2M()
{
	delete map_;
	delete lastFrame_;
	scansBuffer_.clear();
	bundleWordReferences_.clear();
	bundlePoses_.clear();
	bundleLinks_.clear();
	bundleModels_.clear();
	bundlePoseReferences_.clear();
	UDEBUG("");
}


void OdometryF2M::reset(const Transform & initialPose)
{
	Odometry::reset(initialPose);
	*lastFrame_ = Signature(1);
	*map_ = Signature(-1);
}

// return not null transform if odometry is correctly computed
Transform OdometryF2M::computeTransform(
		SensorData & data,
		const Transform & guess,
		OdometryInfo * info)
{
	UTimer timer;
	Transform output;

	if(info)
	{
		info->type = 0;
	}

	RegistrationInfo regInfo;
	int nFeatures = 0;

	delete lastFrame_;
	lastFrame_ = new Signature(data);

	if(bundleAdjustment_ > 0 &&
	   data.cameraModels().size() > 1)
	{
		UERROR("Odometry bundle adjustment doesn't work with multi-cameras. It is disabled.");
		bundleAdjustment_ = 0;
	}

	// Generate keypoints from the new data
	if(lastFrame_->sensorData().isValid())
	{
		if((map_->getWords3().size() || !map_->sensorData().laserScanRaw().empty()) &&
			lastFrame_->sensorData().isValid())
		{
			Signature tmpMap = *map_;
			Transform transform = regPipeline_->computeTransformationMod(
					tmpMap,
					*lastFrame_,
					// special case for ICP-only odom, set guess to identity if we just started
					!guess.isNull()?this->getPose()*guess:!regPipeline_->isImageRequired()&&this->getPose().isIdentity()?Transform::getIdentity():Transform(),
					&regInfo);

			data.setFeatures(lastFrame_->sensorData().keypoints(), lastFrame_->sensorData().descriptors());

			std::map<int, cv::Point3f> points3DMap;
			std::map<int, Transform> bundlePoses;
			std::multimap<int, Link> bundleLinks;
			std::map<int, CameraModel> bundleModels;
			if(!transform.isNull())
			{
				// local bundle adjustment
				if(bundleAdjustment_>0 &&
				   regPipeline_->isImageRequired() &&
				   ((bundleAdjustment_==1 && Optimizer::isAvailable(Optimizer::kTypeG2O)) ||
			        (bundleAdjustment_==2 && Optimizer::isAvailable(Optimizer::kTypeCVSBA))) &&
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
						UTimer bundleTime;
						Optimizer * sba = Optimizer::create(bundleAdjustment_==2?Optimizer::kTypeCVSBA:Optimizer::kTypeG2O, bundleParameters_);

						std::map<int, std::map<int, cv::Point2f> > wordReferences;
						UASSERT(bundlePoses_.size());
						UASSERT(bundlePoses_.size()-1 == bundleLinks_.size() && bundlePoses_.size() == bundleModels_.size());
						if(bundleAdjustmentMaxFrames_ > 0)
						{
							std::map<int, Transform>::reverse_iterator iter = bundlePoses_.rbegin();
							for(int i = 0; i<bundleAdjustmentMaxFrames_ && i < (int)bundlePoses_.size()-1; ++i, ++iter)
							{
								bundlePoses.insert(*iter);
								UASSERT(bundleLinks_.find(iter->first) != bundleLinks_.end());
								bundleLinks.insert(*bundleLinks_.find(iter->first));
								UASSERT(bundleModels_.find(iter->first) != bundleModels_.end());
								bundleModels.insert(*bundleModels_.find(iter->first));
							}
							//make sure the origin is there
							bundlePoses.insert(*bundlePoses_.find(0));
							bundleModels.insert(*bundleModels_.find(0));
						}
						else
						{
							bundlePoses = bundlePoses_;
							bundleLinks = bundleLinks_;
							bundleModels = bundleModels_;
						}
						bundleLinks.insert(std::make_pair(lastFrame_->id(), Link(0, lastFrame_->id(), Link::kNeighbor, transform, regInfo.variance, regInfo.variance)));
						bundlePoses.insert(std::make_pair(lastFrame_->id(), transform));
						for(unsigned int i=0; i<regInfo.inliersIDs.size(); ++i)
						{
							std::multimap<int, cv::Point3f>::const_iterator iter3D = tmpMap.getWords3().find(regInfo.inliersIDs[i]);
							UASSERT(iter3D!=tmpMap.getWords3().end());
							points3DMap.insert(*iter3D);

							std::multimap<int, cv::KeyPoint>::const_iterator iter2D = lastFrame_->getWords().find(regInfo.inliersIDs[i]);
							UASSERT(iter2D!=lastFrame_->getWords().end());

							if(wordReferences.find(iter2D->first) == wordReferences.end())
							{
								UASSERT(bundleWordReferences_.find(iter2D->first) != bundleWordReferences_.end());
								wordReferences.insert(*bundleWordReferences_.find(iter2D->first));
							}

							wordReferences.find(iter2D->first)->second.insert(std::make_pair(lastFrame_->id(), iter2D->second.pt));
						}

						CameraModel model;
						if(lastFrame_->sensorData().cameraModels().size() == 1 && lastFrame_->sensorData().cameraModels().at(0).isValidForProjection())
						{
							model = lastFrame_->sensorData().cameraModels()[0];
						}
						else if(lastFrame_->sensorData().stereoCameraModel().isValidForProjection())
						{
							model = lastFrame_->sensorData().stereoCameraModel().left();
						}
						UASSERT(model.isValidForProjection());
						bundleModels.insert(std::make_pair(lastFrame_->id(), model));

						bundlePoses = sba->optimizeBA(0, bundlePoses, bundleLinks, bundleModels, points3DMap, wordReferences);
						delete sba;

						UDEBUG("bundleTime=%fs (poses=%d wordRef=%d)", bundleTime.ticks(), (int)bundlePoses.size(), (int)bundleWordReferences_.size());

						UDEBUG("Local Bundle Adjustment Before: %s", transform.prettyPrint().c_str());
						UDEBUG("Local Bundle Adjustment After : %s", bundlePoses.rbegin()->second.prettyPrint().c_str());

						if(!bundlePoses.rbegin()->second.isNull())
						{
							transform = bundlePoses.rbegin()->second;
						}
					}
				}

				// make it incremental
				transform = this->getPose().inverse() * transform;
			}
			else if(!regInfo.rejectedMsg.empty())
			{
				UWARN("Registration failed: \"%s\"", regInfo.rejectedMsg.c_str());
			}
			else
			{
				UWARN("Unknown registration error");
			}

			if(!transform.isNull())
			{
				output = transform;

				bool modified = false;
				Transform newFramePose = this->getPose()*output;

				// fields to update
				cv::Mat mapScan = tmpMap.sensorData().laserScanRaw();
				std::multimap<int, cv::KeyPoint> mapWords = tmpMap.getWords();
				std::multimap<int, cv::Point3f> mapPoints = tmpMap.getWords3();
				std::multimap<int, cv::Mat> mapDescriptors = tmpMap.getWordsDescriptors();

				//Visual
				int added = 0;
				int removed = 0;
				UDEBUG("keyframeThr=%f matches=%d inliers=%d features=%d mp=%d", keyFrameThr_, regInfo.matches, regInfo.inliers, (int)lastFrame_->sensorData().keypoints().size(), (int)mapPoints.size());
				if(regPipeline_->isImageRequired() &&
					(keyFrameThr_==0 || float(regInfo.inliers) <= keyFrameThr_*float(lastFrame_->sensorData().keypoints().size())))
				{
					UDEBUG("Update local map (ratio=%f < %f)", float(regInfo.inliers)/float(lastFrame_->sensorData().keypoints().size()), keyFrameThr_);

					// update local map
					UASSERT(mapWords.size() == mapPoints.size());
					UASSERT(mapPoints.size() == mapDescriptors.size());
					UASSERT_MSG(lastFrame_->getWordsDescriptors().size() == lastFrame_->getWords3().size(), uFormat("%d vs %d", lastFrame_->getWordsDescriptors().size(), lastFrame_->getWords3().size()).c_str());

					std::map<int, int>::iterator iterBundlePosesRef = bundlePoseReferences_.end();
					if(bundleAdjustment_>0)
					{
						// update local map 3D points (if bundle adjustment was done)
						for(std::map<int, cv::Point3f>::iterator iter=points3DMap.begin(); iter!=points3DMap.end(); ++iter)
						{
							UASSERT(mapPoints.count(iter->first) == 1);
							mapPoints.find(iter->first)->second = iter->second;
						}
						bundlePoseReferences_.insert(std::make_pair(lastFrame_->id(), 0));
						uInsert(bundlePoses_, bundlePoses);
						UASSERT(bundleModels.find(lastFrame_->id()) != bundleModels.end());
						bundleModels_.insert(*bundleModels.find(lastFrame_->id()));
						UASSERT(bundleLinks.find(lastFrame_->id()) != bundleLinks.end());
						bundleLinks_.insert(*bundleLinks.find(lastFrame_->id()));
						iterBundlePosesRef = bundlePoseReferences_.find(lastFrame_->id());
					}

					// sort by feature response
					std::multimap<float, std::pair<int, std::pair<cv::KeyPoint, std::pair<cv::Point3f, cv::Mat> > > > newIds;
					UASSERT(lastFrame_->getWords3().size() == lastFrame_->getWords().size());
					std::multimap<int, cv::KeyPoint>::const_iterator iter2D = lastFrame_->getWords().begin();
					std::multimap<int, cv::Mat>::const_iterator iterDesc = lastFrame_->getWordsDescriptors().begin();
					for(std::multimap<int, cv::Point3f>::const_iterator iter = lastFrame_->getWords3().begin(); iter!=lastFrame_->getWords3().end(); ++iter, ++iter2D, ++iterDesc)
					{
						if(util3d::isFinite(iter->second))
						{
							if(mapPoints.find(iter->first) == mapPoints.end()) // Point not in map
							{
								newIds.insert(
										std::make_pair(iter2D->second.response>0?1.0f/iter2D->second.response:0.0f,
												std::make_pair(iter->first,
														std::make_pair(iter2D->second,
																std::make_pair(iter->second, iterDesc->second)))));
							}
							else if(bundleAdjustment_>0)
							{
								if(lastFrame_->getWords().count(iter->first) == 1)
								{
									UASSERT(iterBundlePosesRef!=bundlePoseReferences_.end());
									iterBundlePosesRef->second += 1;

									if(bundleWordReferences_.find(iter->first) == bundleWordReferences_.end())
									{
										std::map<int, cv::Point2f> framePt;
										framePt.insert(std::make_pair(lastFrame_->id(), iter2D->second.pt));
										bundleWordReferences_.insert(std::make_pair(iter->first, framePt));
									}
									else
									{
										bundleWordReferences_.find(iter->first)->second.insert(std::make_pair(lastFrame_->id(), iter2D->second.pt));
									}
								}
							}
						}
					}

					for(std::multimap<float, std::pair<int, std::pair<cv::KeyPoint, std::pair<cv::Point3f, cv::Mat> > > >::iterator iter=newIds.begin();
						iter!=newIds.end();
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

									if(bundleWordReferences_.find(iter->second.first) == bundleWordReferences_.end())
									{
										std::map<int, cv::Point2f> framePt;
										framePt.insert(std::make_pair(lastFrame_->id(), iter->second.second.first.pt));
										bundleWordReferences_.insert(std::make_pair(iter->second.first, framePt));
									}
									else
									{
										bundleWordReferences_.find(iter->second.first)->second.insert(std::make_pair(lastFrame_->id(), iter->second.second.first.pt));
									}
								}
							}

							mapWords.insert(std::make_pair(iter->second.first, iter->second.second.first));
							mapPoints.insert(std::make_pair(iter->second.first, util3d::transformPoint(iter->second.second.second.first, newFramePose)));
							mapDescriptors.insert(std::make_pair(iter->second.first, iter->second.second.second.second));
							++added;
						}
					}

					// remove words in map if max size is reached
					if((int)mapPoints.size() > maximumMapSize_)
					{
						// remove oldest first, keep matched features
						std::set<int> matches(regInfo.matchesIDs.begin(), regInfo.matchesIDs.end());
						std::multimap<int, cv::Mat>::iterator iterMapDescriptors = mapDescriptors.begin();
						std::multimap<int, cv::KeyPoint>::iterator iterMapWords = mapWords.begin();
						for(std::multimap<int, cv::Point3f>::iterator iter = mapPoints.begin();
							iter!=mapPoints.end() && (int)mapPoints.size() > maximumMapSize_ && mapPoints.size() >= newIds.size();)
						{
							if(matches.find(iter->first) == matches.end())
							{
								std::map<int, std::map<int, cv::Point2f> >::iterator iterRef = bundleWordReferences_.find(iter->first);
								if(iterRef != bundleWordReferences_.end())
								{
									for(std::map<int, cv::Point2f>::iterator iterKp = iterRef->second.begin(); iterKp != iterRef->second.end(); ++iterKp)
									{
										if(bundlePoseReferences_.find(iterKp->first) != bundlePoseReferences_.end())
										{
											bundlePoseReferences_.at(iterKp->first) -= 1;
											if(bundlePoseReferences_.at(iterKp->first) <= regPipeline_->getMinVisualCorrespondences())
											{
												bundlePoses_.erase(iterKp->first);
												bundleLinks_.erase(iterKp->first);
												bundleModels_.erase(iterKp->first);
												bundlePoseReferences_.erase(iterKp->first);
												UDEBUG("bundlePoseReferences_ erased all words from cam %d", iterKp->first);
											}
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
					}
					modified = true;
				}

				// Geometric
				UDEBUG("scankeyframeThr=%f icpInliersRatio=%f", scanKeyFrameThr_, regInfo.icpInliersRatio);
				if(regPipeline_->isScanRequired() &&
					(scanKeyFrameThr_==0 || regInfo.icpInliersRatio <= scanKeyFrameThr_))
				{
					UINFO("Update local scan map %d (ratio=%f < %f)", lastFrame_->id(), regInfo.icpInliersRatio, scanKeyFrameThr_);

					UTimer tmpTimer;
					if(lastFrame_->sensorData().laserScanRaw().cols)
					{
						pcl::PointCloud<pcl::PointNormal>::Ptr mapCloudNormals = util3d::laserScanToPointCloudNormal(mapScan);
						pcl::PointCloud<pcl::PointNormal>::Ptr frameCloudNormals = util3d::laserScanToPointCloudNormal(lastFrame_->sensorData().laserScanRaw(), newFramePose * lastFrame_->sensorData().laserScanInfo().localTransform());

						pcl::IndicesPtr frameCloudNormalsIndices(new std::vector<int>);
						int newPoints;
						if(mapCloudNormals->size() && scanSubtractRadius_ > 0.0f)
						{
							frameCloudNormalsIndices = util3d::subtractFiltering(
									frameCloudNormals,
									pcl::IndicesPtr(new std::vector<int>),
									mapCloudNormals,
									pcl::IndicesPtr(new std::vector<int>),
									scanSubtractRadius_,
									0.0f);
							newPoints = frameCloudNormalsIndices->size();
						}
						else
						{
							newPoints = mapCloudNormals->size();
						}

						if(newPoints)
						{
							scansBuffer_.push_back(std::make_pair(frameCloudNormals, frameCloudNormalsIndices));

							//remove points if too big
							UDEBUG("scansBuffer=%d, mapSize=%d newPoints=%d maxPoints=%d",
									(int)scansBuffer_.size(),
									int(mapCloudNormals->size()),
									newPoints,
									scanMaximumMapSize_);

							if(newPoints < 20)
							{
								UWARN("The number of new scan points added to local odometry "
									  "map is low (%d), you may want to decrease the parameter \"%s\" "
									  "(current value=%f and ICP inliers ratio is %f)",
										newPoints,
										Parameters::kOdomScanKeyFrameThr().c_str(),
										scanKeyFrameThr_,
										regInfo.icpInliersRatio);
							}

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
							mapScan = util3d::laserScanFromPointCloud(*mapCloudNormals);
							modified=true;
						}
					}
					UDEBUG("Update local map = %fs", tmpTimer.ticks());
				}

				if(modified)
				{
					*map_ = tmpMap;

					map_->sensorData().setLaserScanRaw(mapScan, LaserScanInfo(0, 0));
					map_->setWords(mapWords);
					map_->setWords3(mapPoints);
					map_->setWordsDescriptors(mapDescriptors);
				}
			}

			if(info)
			{
				// use tmpMap instead of map_ to make sure that correspondences with the new frame matches
				info->localMapSize = (int)tmpMap.getWords3().size();
				info->localScanMapSize = tmpMap.sensorData().laserScanRaw().cols;
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

			data.setFeatures(lastFrame_->sensorData().keypoints(), lastFrame_->sensorData().descriptors());

			// a very high variance tells that the new pose is not linked with the previous one
			regInfo.variance = 9999;

			bool frameValid = false;
			Transform newFramePose = this->getPose(); // initial pose may be not identity...
			if(regPipeline_->isImageRequired())
			{
				if ((int)lastFrame_->getWords3().size() >= regPipeline_->getMinVisualCorrespondences())
				{
					frameValid = true;
					// update local map
					UASSERT_MSG(lastFrame_->getWordsDescriptors().size() == lastFrame_->getWords3().size(), uFormat("%d vs %d", lastFrame_->getWordsDescriptors().size(), lastFrame_->getWords3().size()).c_str());
					UASSERT(lastFrame_->getWords3().size() == lastFrame_->getWords().size());

					std::multimap<int, cv::KeyPoint> words;
					std::multimap<int, cv::Point3f> transformedPoints;
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
							descriptors.insert(*descIter);
						}
					}

					if(bundleAdjustment_>0)
					{
						// update bundleWordReferences_: used for bundle adjustment
						for(std::multimap<int, cv::KeyPoint>::const_iterator iter=words.begin(); iter!=words.end(); ++iter)
						{
							if(words.count(iter->first) == 1)
							{
								UASSERT(bundleWordReferences_.find(iter->first) == bundleWordReferences_.end());
								std::map<int, cv::Point2f> framePt;
								framePt.insert(std::make_pair(lastFrame_->id(), iter->second.pt));
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
						}
						UASSERT(model.isValidForProjection());
						UASSERT_MSG(lastFrame_->id() > 0, uFormat("Input data should have ID greater than 0 when odometry bundle adjustment is enabled!").c_str());
						bundleModels_.insert(std::make_pair(lastFrame_->id(), model));
						bundlePoses_.insert(std::make_pair(lastFrame_->id(), newFramePose));
						bundleLinks_.insert(std::make_pair(lastFrame_->id(), Link(0, lastFrame_->id(), Link::kNeighbor, newFramePose, 0.000001, 0.00001)));

						//origin
						bundlePoses_.insert(std::make_pair(0, Transform::getIdentity()));
						bundleModels_.insert(std::make_pair(0, model));
					}

					map_->setWords(words);
					map_->setWords3(transformedPoints);
					map_->setWordsDescriptors(descriptors);

					map_->sensorData().setCameraModels(lastFrame_->sensorData().cameraModels());
					map_->sensorData().setStereoCameraModel(lastFrame_->sensorData().stereoCameraModel());
				}
				else
				{
					UWARN("%d visual features required to initialize the odometry (only %d extracted).", regPipeline_->getMinVisualCorrespondences(), (int)lastFrame_->getWords3().size());
				}
			}
			if(regPipeline_->isScanRequired())
			{
				if (lastFrame_->sensorData().laserScanRaw().cols)
				{
					frameValid = true;
					pcl::PointCloud<pcl::PointNormal>::Ptr mapCloudNormals = util3d::laserScanToPointCloudNormal(lastFrame_->sensorData().laserScanRaw(), newFramePose * lastFrame_->sensorData().laserScanInfo().localTransform());
					scansBuffer_.push_back(std::make_pair(mapCloudNormals, pcl::IndicesPtr(new std::vector<int>)));
					map_->sensorData().setLaserScanRaw(util3d::laserScanFromPointCloud(*mapCloudNormals), LaserScanInfo(0,0));
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
				info->localScanMapSize = map_->sensorData().laserScanRaw().cols;

				if(this->isInfoDataFilled())
				{
					info->localMap = uMultimapToMap(map_->getWords3());
					info->localScanMap = map_->sensorData().laserScanRaw();
				}
			}
		}

		map_->sensorData().setFeatures(std::vector<cv::KeyPoint>(), cv::Mat()); // clear sensorData features

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
		info->variance = regInfo.variance;
		info->inliers = regInfo.inliers;
		info->matches = regInfo.matches;
		info->icpInliersRatio = regInfo.icpInliersRatio;
		info->features = nFeatures;

		if(this->isInfoDataFilled())
		{
			info->wordMatches = regInfo.matchesIDs;
			info->wordInliers = regInfo.inliersIDs;
		}
	}

	UINFO("Odom update time = %fs lost=%s features=%d inliers=%d/%d variance=%f local_map=%d local_scan_map=%d",
			timer.elapsed(),
			output.isNull()?"true":"false",
			nFeatures,
			regInfo.inliers,
			regInfo.matches,
			regInfo.variance,
			regPipeline_->isImageRequired()?(int)map_->getWords3().size():0,
			regPipeline_->isScanRequired()?(int)map_->sensorData().laserScanRaw().cols:0);
	return output;
}

} // namespace rtabmap
