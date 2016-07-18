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
	fixedMapPath_(Parameters::defaultOdomF2MFixedMapPath()),
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
	Parameters::parse(parameters, Parameters::kOdomF2MFixedMapPath(), fixedMapPath_);
	UASSERT(maximumMapSize_ >= 0);
	UASSERT(keyFrameThr_ >= 0.0f && keyFrameThr_<=1.0f);
	UASSERT(scanKeyFrameThr_ >= 0.0f && scanKeyFrameThr_<=1.0f);
	UASSERT(maxNewFeatures_ >= 0);

	if(!fixedMapPath_.empty())
	{
		UINFO("Init odometry from a fixed database: \"%s\"", fixedMapPath_.c_str());
		// init the local map with a all 3D features contained in the database
		ParametersMap customParameters;
		customParameters.insert(ParametersPair(Parameters::kMemIncrementalMemory(), "false"));
		customParameters.insert(ParametersPair(Parameters::kMemInitWMWithAllNodes(), "true"));
		customParameters.insert(ParametersPair(Parameters::kMemSTMSize(), "0"));
		Memory memory(customParameters);
		if(!memory.init(fixedMapPath_, false, ParametersMap()))
		{
			UERROR("Error initializing the memory for BOW Odometry.");
		}
		else
		{
			// get the graph
			std::map<int, int> ids = memory.getNeighborsId(memory.getLastSignatureId(), 0, -1);
			std::map<int, Transform> poses;
			std::multimap<int, Link> links;
			memory.getMetricConstraints(uKeysSet(ids), poses, links, true);

			if(poses.size())
			{
				//optimize the graph
				Optimizer * optimizer = Optimizer::create(parameters);
				std::map<int, Transform> optimizedPoses = optimizer->optimize(poses.begin()->first, poses, links);
				delete optimizer;

				std::multimap<int, cv::Point3f> words3D;
				std::multimap<int, cv::Mat> wordsDescriptors;

				// fill the local map
				for(std::map<int, Transform>::iterator posesIter=optimizedPoses.begin();
					posesIter!=optimizedPoses.end();
					++posesIter)
				{
					const Signature * s = memory.getSignature(posesIter->first);
					if(s)
					{
						// Transform 3D points accordingly to pose and add them to local map
						for(std::multimap<int, cv::Point3f>::const_iterator pointsIter=s->getWords3().begin();
							pointsIter!=s->getWords3().end();
							++pointsIter)
						{
							if(!uContains(words3D, pointsIter->first))
							{
								words3D.insert(std::make_pair(pointsIter->first, util3d::transformPoint(pointsIter->second, posesIter->second)));

								if(s->getWordsDescriptors().size() == s->getWords3().size())
								{
									UASSERT(uContains(s->getWordsDescriptors(), pointsIter->first));
									wordsDescriptors.insert(std::make_pair(pointsIter->first, s->getWordsDescriptors().find(pointsIter->first)->second));
								}
								else // load descriptor from dictionary
								{
									UASSERT(memory.getVWDictionary()->getWord(pointsIter->first) != 0);
									wordsDescriptors.insert(std::make_pair(pointsIter->first, memory.getVWDictionary()->getWord(pointsIter->first)->getDescriptor()));
								}
							}
						}
					}
				}
				UASSERT(words3D.size() == wordsDescriptors.size());
				map_->setWords3(words3D);
				map_->setWordsDescriptors(wordsDescriptors);
			}
			else
			{
				UERROR("No pose loaded from database \"%s\"", fixedMapPath_.c_str());
			}
		}
		if((int)map_->getWords3().size() < regPipeline_->getMinVisualCorrespondences() || map_->getWords3().size() == 0)
		{
			// TODO: support geometric-only maps?
			UERROR("The loaded fixed map from \"%s\" is too small! Only %d unique features loaded. Odometry won't be computed!",
					fixedMapPath_.c_str(), (int)map_->getWords3().size());
		}
	}
}

OdometryF2M::~OdometryF2M()
{
	delete map_;
	delete lastFrame_;
	UDEBUG("");
}


void OdometryF2M::reset(const Transform & initialPose)
{
	Odometry::reset(initialPose);
	*lastFrame_ = Signature(1);

	if(fixedMapPath_.empty())
	{
		*map_ = Signature(-1);
	}
	else
	{
		UWARN("Odometry cannot be reset when a fixed local map is set.");
	}
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
					guess.isNull()?Transform():this->getPose()*guess,
					&regInfo);

			data.setFeatures(lastFrame_->sensorData().keypoints(), lastFrame_->sensorData().descriptors());

			if(!transform.isNull())
			{
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

				if(fixedMapPath_.empty())
				{
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
							}
						}

						for(std::multimap<float, std::pair<int, std::pair<cv::KeyPoint, std::pair<cv::Point3f, cv::Mat> > > >::iterator iter=newIds.begin();
							iter!=newIds.end();
							++iter)
						{
							if(maxNewFeatures_ == 0  || added < maxNewFeatures_)
							{
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
							pcl::PointCloud<pcl::PointNormal>::Ptr frameCloudNormals = util3d::laserScanToPointCloudNormal(lastFrame_->sensorData().laserScanRaw(), newFramePose);

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

						map_->sensorData().setLaserScanRaw(mapScan, 0, 0);
						map_->setWords(mapWords);
						map_->setWords3(mapPoints);
						map_->setWordsDescriptors(mapDescriptors);
					}
				}
				else
				{
					// fixed local map, don't update with the new signature
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
					if (fixedMapPath_.empty())
					{
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
						map_->setWords(words);
						map_->setWords3(transformedPoints);
						map_->setWordsDescriptors(descriptors);

						map_->sensorData().setCameraModels(lastFrame_->sensorData().cameraModels());
						map_->sensorData().setStereoCameraModel(lastFrame_->sensorData().stereoCameraModel());
					}
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
					if (fixedMapPath_.empty())
					{
						pcl::PointCloud<pcl::PointNormal>::Ptr mapCloudNormals = util3d::laserScanToPointCloudNormal(lastFrame_->sensorData().laserScanRaw(), newFramePose);
						scansBuffer_.push_back(std::make_pair(mapCloudNormals, pcl::IndicesPtr(new std::vector<int>)));
						map_->sensorData().setLaserScanRaw(util3d::laserScanFromPointCloud(*mapCloudNormals), 0,0);
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
