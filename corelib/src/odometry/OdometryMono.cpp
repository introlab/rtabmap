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

#include "rtabmap/core/odometry/OdometryMono.h"
#include "rtabmap/core/OdometryInfo.h"
#include "rtabmap/core/Memory.h"
#include "rtabmap/core/Signature.h"
#include "rtabmap/core/util3d_transforms.h"
#include "rtabmap/core/util3d_motion_estimation.h"
#include "rtabmap/core/util3d.h"
#include "rtabmap/core/util2d.h"
#include "rtabmap/core/util3d_features.h"
#include "rtabmap/core/EpipolarGeometry.h"
#include "rtabmap/core/Optimizer.h"
#include "rtabmap/core/Stereo.h"
#include "rtabmap/utilite/ULogger.h"
#include "rtabmap/utilite/UTimer.h"
#include "rtabmap/utilite/UConversion.h"
#include "rtabmap/utilite/UStl.h"
#include "rtabmap/utilite/UMath.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/video/tracking.hpp>
#include <pcl/common/centroid.h>

namespace rtabmap {

OdometryMono::OdometryMono(const rtabmap::ParametersMap & parameters) :
	Odometry(parameters),
	flowWinSize_(Parameters::defaultVisCorFlowWinSize()),
	flowIterations_(Parameters::defaultVisCorFlowIterations()),
	flowEps_(Parameters::defaultVisCorFlowEps()),
	flowMaxLevel_(Parameters::defaultVisCorFlowMaxLevel()),
	minInliers_(Parameters::defaultVisMinInliers()),
	iterations_(Parameters::defaultVisIterations()),
	pnpReprojError_(Parameters::defaultVisPnPReprojError()),
	pnpFlags_(Parameters::defaultVisPnPFlags()),
	pnpRefineIterations_(Parameters::defaultVisPnPRefineIterations()),
	localHistoryMaxSize_(Parameters::defaultOdomF2MMaxSize()),
	initMinFlow_(Parameters::defaultOdomMonoInitMinFlow()),
	initMinTranslation_(Parameters::defaultOdomMonoInitMinTranslation()),
	minTranslation_(Parameters::defaultOdomMonoMinTranslation()),
	fundMatrixReprojError_(Parameters::defaultVhEpRansacParam1()),
	fundMatrixConfidence_(Parameters::defaultVhEpRansacParam2()),
	maxVariance_(Parameters::defaultOdomMonoMaxVariance()),
	keyFrameThr_(0.95)
{
	Parameters::parse(parameters, Parameters::kVisCorFlowWinSize(), flowWinSize_);
	Parameters::parse(parameters, Parameters::kVisCorFlowIterations(), flowIterations_);
	Parameters::parse(parameters, Parameters::kVisCorFlowEps(), flowEps_);
	Parameters::parse(parameters, Parameters::kVisCorFlowMaxLevel(), flowMaxLevel_);
	Parameters::parse(parameters, Parameters::kVisMinInliers(), minInliers_);
	UASSERT(minInliers_ >= 1);
	Parameters::parse(parameters, Parameters::kVisIterations(), iterations_);
	Parameters::parse(parameters, Parameters::kVisPnPReprojError(), pnpReprojError_);
	Parameters::parse(parameters, Parameters::kVisPnPFlags(), pnpFlags_);
	Parameters::parse(parameters, Parameters::kVisPnPRefineIterations(), pnpRefineIterations_);
	Parameters::parse(parameters, Parameters::kOdomF2MMaxSize(), localHistoryMaxSize_);

	Parameters::parse(parameters, Parameters::kOdomMonoInitMinFlow(), initMinFlow_);
	Parameters::parse(parameters, Parameters::kOdomMonoInitMinTranslation(), initMinTranslation_);
	Parameters::parse(parameters, Parameters::kOdomMonoMinTranslation(), minTranslation_);
	Parameters::parse(parameters, Parameters::kOdomMonoMaxVariance(), maxVariance_);
	Parameters::parse(parameters, Parameters::kOdomKeyFrameThr(), keyFrameThr_);

	Parameters::parse(parameters, Parameters::kVhEpRansacParam1(), fundMatrixReprojError_);
	Parameters::parse(parameters, Parameters::kVhEpRansacParam2(), fundMatrixConfidence_);

	// Setup memory
	ParametersMap customParameters;
	std::string roi = Parameters::defaultVisRoiRatios();
	Parameters::parse(parameters, Parameters::kVisRoiRatios(), roi);
	customParameters.insert(ParametersPair(Parameters::kMemDepthAsMask(), "false"));
	customParameters.insert(ParametersPair(Parameters::kKpRoiRatios(), roi));
	customParameters.insert(ParametersPair(Parameters::kMemRehearsalSimilarity(), "1.0")); // desactivate rehearsal
	customParameters.insert(ParametersPair(Parameters::kMemBinDataKept(), "false"));
	customParameters.insert(ParametersPair(Parameters::kMemSTMSize(), "0"));
	customParameters.insert(ParametersPair(Parameters::kMemNotLinkedNodesKept(), "false"));
	customParameters.insert(ParametersPair(Parameters::kKpTfIdfLikelihoodUsed(), "false"));
	int nn = Parameters::defaultVisCorNNType();
	float nndr = Parameters::defaultVisCorNNDR();
	int featureType = Parameters::defaultVisFeatureType();
	int maxFeatures = Parameters::defaultVisMaxFeatures();
	Parameters::parse(parameters, Parameters::kVisCorNNType(), nn);
	Parameters::parse(parameters, Parameters::kVisCorNNDR(), nndr);
	Parameters::parse(parameters, Parameters::kVisFeatureType(), featureType);
	Parameters::parse(parameters, Parameters::kVisMaxFeatures(), maxFeatures);
	customParameters.insert(ParametersPair(Parameters::kKpNNStrategy(), uNumber2Str(nn)));
	customParameters.insert(ParametersPair(Parameters::kKpNndrRatio(), uNumber2Str(nndr)));
	customParameters.insert(ParametersPair(Parameters::kKpDetectorStrategy(), uNumber2Str(featureType)));
	customParameters.insert(ParametersPair(Parameters::kKpMaxFeatures(), uNumber2Str(maxFeatures)));

	int subPixWinSize = Parameters::defaultVisSubPixWinSize();
	int subPixIterations = Parameters::defaultVisSubPixIterations();
	double subPixEps = Parameters::defaultVisSubPixEps();
	Parameters::parse(parameters, Parameters::kVisSubPixWinSize(), subPixWinSize);
	Parameters::parse(parameters, Parameters::kVisSubPixIterations(), subPixIterations);
	Parameters::parse(parameters, Parameters::kVisSubPixEps(), subPixEps);
	customParameters.insert(ParametersPair(Parameters::kKpSubPixWinSize(), uNumber2Str(subPixWinSize)));
	customParameters.insert(ParametersPair(Parameters::kKpSubPixIterations(), uNumber2Str(subPixIterations)));
	customParameters.insert(ParametersPair(Parameters::kKpSubPixEps(), uNumber2Str(subPixEps)));

	// add only feature stuff
	for(ParametersMap::const_iterator iter=parameters.begin(); iter!=parameters.end(); ++iter)
	{
		if(Parameters::isFeatureParameter(iter->first))
		{
			customParameters.insert(*iter);
		}
	}

	memory_ = new Memory(customParameters);
	if(!memory_->init("", false, ParametersMap()))
	{
		UERROR("Error initializing the memory for Mono Odometry.");
	}

	feature2D_ = Feature2D::create(parameters);
}

OdometryMono::~OdometryMono()
{
	delete memory_;
	delete feature2D_;
}

void OdometryMono::reset(const Transform & initialPose)
{
	Odometry::reset(initialPose);
	memory_->init("", false, ParametersMap());
	localMap_.clear();
	firstFrameGuessCorners_.clear();
	keyFrameWords3D_.clear();
	keyFramePoses_.clear();
	keyFrameModels_.clear();
	keyFrameLinks_.clear();
}

Transform OdometryMono::computeTransform(SensorData & data, const Transform & guess, OdometryInfo * info)
{
	Transform output;

	if(data.imageRaw().empty())
	{
		UERROR("Image empty! Cannot compute odometry...");
		return output;
	}

	if(!(((data.cameraModels().size() == 1 && data.cameraModels()[0].isValidForProjection()) || data.stereoCameraModel().isValidForProjection())))
	{
		UERROR("Odometry cannot be done without calibration or on multi-camera!");
		return output;
	}


	CameraModel cameraModel;
	if(data.stereoCameraModel().isValidForProjection())
	{
		cameraModel = data.stereoCameraModel().left();
		// Set Tx for stereo BA
		cameraModel = CameraModel(cameraModel.fx(),
				cameraModel.fy(),
				cameraModel.cx(),
				cameraModel.cy(),
				cameraModel.localTransform(),
				-data.stereoCameraModel().baseline()*cameraModel.fx());
	}
	else
	{
		cameraModel = data.cameraModels()[0];
	}

	UTimer timer;

	int inliers = 0;
	int correspondences = 0;
	int nFeatures = 0;

	// convert to grayscale
	if(data.imageRaw().channels() > 1)
	{
		cv::Mat newFrame;
		cv::cvtColor(data.imageRaw(), newFrame, cv::COLOR_BGR2GRAY);
		if(data.stereoCameraModel().isValidForProjection())
		{
			data.setStereoImage(newFrame, data.rightRaw(), data.stereoCameraModel());
		}
		else
		{
			data.setRGBDImage(newFrame, data.depthRaw(), data.cameraModels());
		}
	}

	if(!localMap_.empty())
	{
		UDEBUG("RUNNING");
		//PnP
		UDEBUG("PnP");

		if(this->isInfoDataFilled() && info)
		{
			info->type = 0;
		}

		// generate kpts
		if(memory_->update(data))
		{
			UDEBUG("");
			bool newPtsAdded = false;
			const Signature * newS = memory_->getLastWorkingSignature();
			UDEBUG("newWords=%d", (int)newS->getWords().size());
			nFeatures = (int)newS->getWords().size();
			if((int)newS->getWords().size() > minInliers_)
			{
				cv::Mat K = cameraModel.K();
				Transform pnpGuess = ((this->getPose() * (guess.isNull()?Transform::getIdentity():guess)) * cameraModel.localTransform()).inverse();
				cv::Mat R = (cv::Mat_<double>(3,3) <<
						(double)pnpGuess.r11(), (double)pnpGuess.r12(), (double)pnpGuess.r13(),
						(double)pnpGuess.r21(), (double)pnpGuess.r22(), (double)pnpGuess.r23(),
						(double)pnpGuess.r31(), (double)pnpGuess.r32(), (double)pnpGuess.r33());
				cv::Mat rvec(1,3, CV_64FC1);
				cv::Rodrigues(R, rvec);
				cv::Mat tvec = (cv::Mat_<double>(1,3) << (double)pnpGuess.x(), (double)pnpGuess.y(), (double)pnpGuess.z());

				std::vector<cv::Point3f> objectPoints;
				std::vector<cv::Point2f> imagePoints;
				std::vector<int> matches;

				UDEBUG("compute PnP from optical flow");

				std::vector<int> ids = uKeys(localMap_);
				objectPoints = uValues(localMap_);

				// compute last projection
				UDEBUG("project points to previous image");
				std::vector<cv::Point2f> prevImagePoints;
				const Signature * prevS = memory_->getSignature(*(++memory_->getStMem().rbegin()));
				Transform prevGuess = (keyFramePoses_.at(prevS->id()) * cameraModel.localTransform()).inverse();
				cv::Mat prevR = (cv::Mat_<double>(3,3) <<
						(double)prevGuess.r11(), (double)prevGuess.r12(), (double)prevGuess.r13(),
						(double)prevGuess.r21(), (double)prevGuess.r22(), (double)prevGuess.r23(),
						(double)prevGuess.r31(), (double)prevGuess.r32(), (double)prevGuess.r33());
				cv::Mat prevRvec(1,3, CV_64FC1);
				cv::Rodrigues(prevR, prevRvec);
				cv::Mat prevTvec = (cv::Mat_<double>(1,3) << (double)prevGuess.x(), (double)prevGuess.y(), (double)prevGuess.z());
				cv::projectPoints(objectPoints, prevRvec, prevTvec, K, cv::Mat(), prevImagePoints);

				// compute current projection
				UDEBUG("project points to current image");
				cv::projectPoints(objectPoints, rvec, tvec, K, cv::Mat(), imagePoints);

				//filter points not in the image and set guess from unique correspondences
				std::vector<cv::Point3f> objectPointsTmp(objectPoints.size());
				std::vector<cv::Point2f> refCorners(objectPoints.size());
				std::vector<cv::Point2f> newCorners(objectPoints.size());
				matches.resize(objectPoints.size());
				int oi=0;
				for(unsigned int i=0; i<objectPoints.size(); ++i)
				{
					if(uIsInBounds(int(imagePoints[i].x), 0, newS->sensorData().imageRaw().cols) &&
					   uIsInBounds(int(imagePoints[i].y), 0, newS->sensorData().imageRaw().rows) &&
					   uIsInBounds(int(prevImagePoints[i].x), 0, prevS->sensorData().imageRaw().cols) &&
					   uIsInBounds(int(prevImagePoints[i].y), 0, prevS->sensorData().imageRaw().rows))
					{
						refCorners[oi] = prevImagePoints[i];
						newCorners[oi] = imagePoints[i];
						if(localMap_.count(ids[i]) == 1)
						{
							if(prevS->getWords().count(ids[i]) == 1 && !prevS->getWordsKpts().empty())
							{
								// set guess if unique
								refCorners[oi] = prevS->getWordsKpts()[prevS->getWords().find(ids[i])->second].pt;
							}
							if(newS->getWords().count(ids[i]) == 1 && !newS->getWordsKpts().empty())
							{
								// set guess if unique
								newCorners[oi] = newS->getWordsKpts()[newS->getWords().find(ids[i])->second].pt;
							}
						}
						objectPointsTmp[oi] = objectPoints[i];
						matches[oi] = ids[i];
						++oi;
					}
				}
				objectPointsTmp.resize(oi);
				refCorners.resize(oi);
				newCorners.resize(oi);
				matches.resize(oi);

				// Refine imagePoints using optical flow
				std::vector<unsigned char> statusFlowInliers;
				std::vector<float> err;
				UDEBUG("cv::calcOpticalFlowPyrLK() begin");
				cv::calcOpticalFlowPyrLK(
						prevS->sensorData().imageRaw(),
						newS->sensorData().imageRaw(),
						refCorners,
						newCorners,
						statusFlowInliers,
						err,
						cv::Size(flowWinSize_, flowWinSize_), flowMaxLevel_,
						cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, flowIterations_, flowEps_),
						cv::OPTFLOW_LK_GET_MIN_EIGENVALS | cv::OPTFLOW_USE_INITIAL_FLOW, 1e-4);
				UDEBUG("cv::calcOpticalFlowPyrLK() end");

				objectPoints.resize(statusFlowInliers.size());
				imagePoints.resize(statusFlowInliers.size());
				std::vector<int> matchesTmp(statusFlowInliers.size());
				oi = 0;
				for(unsigned int i=0; i<statusFlowInliers.size(); ++i)
				{
					if(statusFlowInliers[i] &&
					   uIsInBounds(int(newCorners[i].x), 0, newS->sensorData().imageRaw().cols) &&
					   uIsInBounds(int(newCorners[i].y), 0, newS->sensorData().imageRaw().rows))
					{
						objectPoints[oi] = objectPointsTmp[i];
						imagePoints[oi] = newCorners[i];
						matchesTmp[oi] = matches[i];
						++oi;

						if(this->isInfoDataFilled() && info)
						{
							cv::KeyPoint kpt;
							if(newS->getWords().count(matches[i]) == 1 && !newS->getWordsKpts().empty())
							{
								kpt = newS->getWordsKpts()[newS->getWords().find(matches[i])->second];
							}
							kpt.pt = newCorners[i];
							info->words.insert(std::make_pair(matches[i], kpt));
						}
					}
				}
				UDEBUG("Flow inliers= %d/%d", oi, (int)statusFlowInliers.size());
				objectPoints.resize(oi);
				imagePoints.resize(oi);
				matchesTmp.resize(oi);
				matches = matchesTmp;

				if(this->isInfoDataFilled() && info)
				{
					info->reg.matchesIDs.insert(info->reg.matchesIDs.end(), matches.begin(), matches.end());
				}
				correspondences = (int)matches.size();

				if((int)matches.size() < minInliers_)
				{
					UWARN("not enough matches (%d < %d)...", (int)matches.size(), minInliers_);
				}
				else
				{
					//PnPRansac
					std::vector<int> inliersV;
					util3d::solvePnPRansac(
							objectPoints,
							imagePoints,
							K,
							cv::Mat(),
							rvec,
							tvec,
							true,
							iterations_,
							pnpReprojError_,
							0, // min inliers
							inliersV,
							pnpFlags_,
							pnpRefineIterations_);

					UDEBUG("inliers=%d/%d", (int)inliersV.size(), (int)objectPoints.size());

					inliers = (int)inliersV.size();
					if((int)inliersV.size() < minInliers_)
					{
						UWARN("PnP not enough inliers (%d < %d), rejecting the transform...", (int)inliersV.size(), minInliers_);
					}
					else
					{
						cv::Mat R(3,3,CV_64FC1);
						cv::Rodrigues(rvec, R);
						Transform pnp = Transform(R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2), tvec.at<double>(0),
												  R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2), tvec.at<double>(1),
												  R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2), tvec.at<double>(2));
						output = this->getPose().inverse() * pnp.inverse() * cameraModel.localTransform().inverse();

						if(this->isInfoDataFilled() && info && inliersV.size())
						{
							info->reg.inliersIDs.resize(inliersV.size());
							for(unsigned int i=0; i<inliersV.size(); ++i)
							{
								info->reg.inliersIDs[i] = matches[inliersV[i]]; // index and ID should match (index starts at 0, ID starts at 1)
							}
						}

						// compute variance, which is the rms of reprojection errors
						cv::Mat covariance = cv::Mat::eye(6, 6, CV_64FC1);
						std::vector<cv::Point2f> imagePointsReproj;
						cv::projectPoints(objectPoints, rvec, tvec, K, cv::Mat(), imagePointsReproj);
						float err = 0.0f;
						for(unsigned int i=0; i<inliersV.size(); ++i)
						{
							err += uNormSquared(imagePoints.at(inliersV[i]).x - imagePointsReproj.at(inliersV[i]).x, imagePoints.at(inliersV[i]).y - imagePointsReproj.at(inliersV[i]).y);
						}
						UASSERT(uIsFinite(err));
						covariance *= std::sqrt(err/float(inliersV.size()));

						Link newLink(keyFramePoses_.rbegin()->first, newS->id(), Link::kNeighbor, output, covariance.inv());

						//bundle adjustment
						Optimizer * ba = Optimizer::create(Optimizer::kTypeG2O);
						std::map<int, Transform> poses = keyFramePoses_;
						poses.insert(std::make_pair(newS->id(), this->getPose()*output));
						if(ba->type() == Optimizer::kTypeG2O)
						{
							UWARN("Bundle adjustment: fill arguments");
							std::multimap<int, Link> links = keyFrameLinks_;
							std::map<int, CameraModel> models = keyFrameModels_;
							links.insert(std::make_pair(keyFramePoses_.rbegin()->first, newLink));
							models.insert(std::make_pair(newS->id(), cameraModel));
							std::map<int, std::map<int, FeatureBA> > wordReferences;

							for(std::set<int>::iterator iter = memory_->getStMem().begin(); iter!=memory_->getStMem().end(); ++iter)
							{
								const Signature * s = memory_->getSignature(*iter);
								for(std::multimap<int, int>::const_iterator jter=s->getWords().begin(); jter!=s->getWords().end(); ++jter)
								{
									if(s->getWords().count(jter->first) == 1 && localMap_.find(jter->first)!=localMap_.end() && !s->getWordsKpts().empty())
									{
										if(wordReferences.find(jter->first)==wordReferences.end())
										{
											wordReferences.insert(std::make_pair(jter->first, std::map<int, FeatureBA>()));
										}
										float depth = 0.0f;
										if(keyFrameWords3D_.find(s->id()) != keyFrameWords3D_.end() &&
										   keyFrameWords3D_.at(s->id()).find(jter->first) != keyFrameWords3D_.at(s->id()).end())
										{
											depth = keyFrameWords3D_.at(s->id()).at(jter->first).x;
										}
										const cv::KeyPoint & kpts = s->getWordsKpts()[jter->second];
										wordReferences.at(jter->first).insert(std::make_pair(s->id(), FeatureBA(kpts, depth, cv::Mat())));
									}
								}
							}

							std::set<int> outliers;
							UWARN("Bundle adjustment begin");
							poses = ba->optimizeBA(poses.begin()->first, poses, links, models, localMap_, wordReferences, &outliers);
							UWARN("Bundle adjustment end");
							if(!poses.empty())
							{
								output = this->getPose().inverse()*poses.at(newS->id());
							}
						}


						//Find the frame with the most similar features
						std::set<int> stMem =  memory_->getStMem();
						stMem.erase(newS->id());
						std::map<int, float> likelihood = memory_->computeLikelihood(newS, std::list<int>(stMem.begin(), stMem.end()));
						int maxLikelihoodId = -1;
						float maxLikelihood = 0;
						for(std::map<int, float>::iterator iter=likelihood.begin(); iter!=likelihood.end(); ++iter)
						{
							if(iter->second > maxLikelihood)
							{
								maxLikelihood = iter->second;
								maxLikelihoodId = iter->first;
							}
						}
						if(maxLikelihoodId == -1)
						{
							UWARN("Cannot find a keyframe similar enough to generate new 3D points!");
						}
						else if(poses.size())
						{
							// Add new points to local map
							const Signature* previousS = memory_->getSignature(maxLikelihoodId);
							UASSERT(previousS!=0);
							Transform cameraTransform = keyFramePoses_.at(previousS->id()).inverse()*this->getPose()*output;
							UDEBUG("cameraTransform guess=  %s (norm^2=%f)", cameraTransform.prettyPrint().c_str(), cameraTransform.getNormSquared());

							UINFO("Inliers= %d/%d (%f)", inliers, (int)imagePoints.size(), float(inliers)/float(imagePoints.size()));

							if(cameraTransform.getNorm() < minTranslation_)
							{
								UINFO("Translation with the nearest frame is too small (%f<%f) to add new points to local map",
										cameraTransform.getNorm(), minTranslation_);
							}
							else if(float(inliers)/float(imagePoints.size()) < keyFrameThr_)
							{
								std::map<int, int> uniqueWordsPrevious = uMultimapToMapUnique(previousS->getWords());
								std::map<int, int> uniqueWordsNew = uMultimapToMapUnique(newS->getWords());
								std::map<int, cv::KeyPoint> wordsPrevious;
								std::map<int, cv::KeyPoint> wordsNew;
								for(std::map<int, int>::iterator iter=uniqueWordsPrevious.begin(); iter!=uniqueWordsPrevious.end(); ++iter)
								{
									wordsPrevious.insert(std::make_pair(iter->first, previousS->getWordsKpts()[iter->second]));
								}
								for(std::map<int, int>::iterator iter=uniqueWordsNew.begin(); iter!=uniqueWordsNew.end(); ++iter)
								{
									wordsNew.insert(std::make_pair(iter->first, newS->getWordsKpts()[iter->second]));
								}
								std::map<int, cv::Point3f> inliers3D = util3d::generateWords3DMono(
										wordsPrevious,
										wordsNew,
										cameraModel,
										cameraTransform,
										fundMatrixReprojError_,
										fundMatrixConfidence_);

								if((int)inliers3D.size() < minInliers_)
								{
									UWARN("Epipolar geometry not enough inliers (%d < %d), rejecting the transform (%s)...",
									(int)inliers3D.size(), minInliers_, cameraTransform.prettyPrint().c_str());
								}
								else
								{
									Transform newPose = keyFramePoses_.at(previousS->id())*cameraTransform;
									UDEBUG("cameraTransform=  %s", cameraTransform.prettyPrint().c_str());

									std::multimap<int, cv::Point3f> wordsToAdd;
									for(std::map<int, cv::Point3f>::iterator iter=inliers3D.begin();
										iter != inliers3D.end();
										++iter)
									{
										// transform inliers3D in new signature referential
										iter->second = util3d::transformPoint(iter->second, cameraTransform.inverse());

										if(!uContains(localMap_, iter->first))
										{
											//UDEBUG("Add new point %d to local map", iter->first);
											cv::Point3f newPt = util3d::transformPoint(iter->second, newPose);
											wordsToAdd.insert(std::make_pair(iter->first, newPt));
										}
									}

									if((int)wordsToAdd.size())
									{
										localMap_.insert(wordsToAdd.begin(), wordsToAdd.end());
										newPtsAdded = true;
										UWARN("Added %d words", (int)wordsToAdd.size());
									}

									if(newPtsAdded)
									{
										// if we have depth guess, set it for ba
										std::vector<cv::Point3f> newCorners3;
										if(!data.depthOrRightRaw().empty())
										{
											std::vector<cv::KeyPoint> newKeypoints(imagePoints.size());
											for(size_t i=0;i<imagePoints.size(); ++i)
											{
												newKeypoints[i] = cv::KeyPoint(imagePoints[i], 3);
											}
											newCorners3 = feature2D_->generateKeypoints3D(data, newKeypoints);
											for(size_t i=0;i<newCorners3.size(); ++i)
											{
												if(util3d::isFinite(newCorners3[i]) &&
												   inliers3D.find(matches[i])!=inliers3D.end())
												{
													inliers3D.at(matches[i]) = newCorners3[i];
												}
												else
												{
													inliers3D.erase(matches[i]);
												}
											}
										}

										if(!inliers3D.empty())
										{
											UDEBUG("Added %d/%d valid 3D features", (int)inliers3D.size(), (int)wordsToAdd.size());
											keyFrameWords3D_.insert(std::make_pair(newS->id(), inliers3D));
										}
										keyFramePoses_ = poses;
										keyFrameLinks_.insert(std::make_pair(newLink.from(), newLink));
										keyFrameModels_.insert(std::make_pair(newS->id(), cameraModel));

										// keep only the two last signatures
										while(localHistoryMaxSize_ && (int)localMap_.size() > localHistoryMaxSize_ && memory_->getStMem().size()>2)
										{
											int nodeId = *memory_->getStMem().begin();
											std::list<int> removedPts;
											memory_->deleteLocation(nodeId, &removedPts);
											keyFrameWords3D_.erase(nodeId);
											keyFramePoses_.erase(nodeId);
											keyFrameLinks_.erase(nodeId);
											keyFrameModels_.erase(nodeId);
											for(std::list<int>::iterator iter = removedPts.begin(); iter!=removedPts.end(); ++iter)
											{
												localMap_.erase(*iter);
											}
										}
									}
								}
							}
						}
					}
				}
			}

			if(!newPtsAdded)
			{
				// remove new words from dictionary
				memory_->deleteLocation(newS->id());
			}
		}
	}
	else if(!firstFrameGuessCorners_.empty())
	{
		UDEBUG("INIT PART 2/2");
		//flow

		if(this->isInfoDataFilled() && info)
		{
			info->type = 1;
		}

		const Signature * refS = memory_->getLastWorkingSignature();

		std::vector<cv::Point2f> refCorners(firstFrameGuessCorners_.size());
		std::vector<cv::Point2f> refCornersGuess(firstFrameGuessCorners_.size());
		std::vector<int> cornerIds(firstFrameGuessCorners_.size());
		int ii=0;
		for(std::map<int, cv::Point2f>::iterator iter=firstFrameGuessCorners_.begin(); iter!=firstFrameGuessCorners_.end(); ++iter)
		{
			std::multimap<int, int>::const_iterator jter=refS->getWords().find(iter->first);
			UASSERT(jter != refS->getWords().end() && !refS->getWordsKpts().empty());
			refCorners[ii] = refS->getWordsKpts()[jter->second].pt;
			refCornersGuess[ii] = iter->second;
			cornerIds[ii] = iter->first;
			++ii;
		}

		UDEBUG("flow");
		// Find features in the new left image
		std::vector<unsigned char> statusFlowInliers;
		std::vector<float> err;
		UDEBUG("cv::calcOpticalFlowPyrLK() begin");
		cv::calcOpticalFlowPyrLK(
				refS->sensorData().imageRaw(),
				data.imageRaw(),
				refCorners,
				refCornersGuess,
				statusFlowInliers,
				err,
				cv::Size(flowWinSize_, flowWinSize_), flowMaxLevel_,
				cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, flowIterations_, flowEps_),
				cv::OPTFLOW_LK_GET_MIN_EIGENVALS | cv::OPTFLOW_USE_INITIAL_FLOW, 1e-4);
		UDEBUG("cv::calcOpticalFlowPyrLK() end");

		UDEBUG("Filtering optical flow outliers...");
		float flow = 0;

		if(this->isInfoDataFilled() && info)
		{
			info->refCorners = refCorners;
			info->newCorners = refCornersGuess;
		}

		int oi = 0;
		std::vector<cv::Point2f> tmpRefCorners(statusFlowInliers.size());
		std::vector<cv::Point2f> newCorners(statusFlowInliers.size());
		std::vector<int> inliersV(statusFlowInliers.size());
		std::vector<int> tmpCornersId(statusFlowInliers.size());
		UASSERT(refCornersGuess.size() == statusFlowInliers.size());
		UASSERT(refCorners.size() == statusFlowInliers.size());
		UASSERT(cornerIds.size() == statusFlowInliers.size());
		for(unsigned int i=0; i<statusFlowInliers.size(); ++i)
		{
			if(statusFlowInliers[i])
			{
				float dx = refCorners[i].x - refCornersGuess[i].x;
				float dy = refCorners[i].y - refCornersGuess[i].y;
				float tmp = std::sqrt(dx*dx + dy*dy);
				flow+=tmp;

				tmpRefCorners[oi] = refCorners[i];
				newCorners[oi] = refCornersGuess[i];

				firstFrameGuessCorners_.at(cornerIds[i]) = refCornersGuess[i];

				inliersV[oi] = i;
				tmpCornersId[oi] = cornerIds[i];

				++oi;
			}
			else
			{
				firstFrameGuessCorners_.erase(cornerIds[i]);
			}
		}
		if(oi)
		{
			flow /=float(oi);
		}
		tmpRefCorners.resize(oi);
		newCorners.resize(oi);
		inliersV.resize((oi));
		tmpCornersId.resize(oi);
		refCorners=  tmpRefCorners;
		cornerIds = tmpCornersId;

		if(this->isInfoDataFilled() && info)
		{
			// fill flow matches info
			info->cornerInliers = inliersV;
			inliers = (int)inliersV.size();
		}

		UDEBUG("Filtering optical flow outliers...done! (inliers=%d/%d)", oi, (int)statusFlowInliers.size());

		if(flow > initMinFlow_ && oi > minInliers_)
		{
			UDEBUG("flow=%f", flow);
			std::vector<cv::Point3f> refCorners3;
			if(!refS->sensorData().depthOrRightRaw().empty())
			{
				std::vector<cv::KeyPoint> refKeypoints(refCorners.size());
				for(size_t i=0;i<refCorners.size(); ++i)
				{
					refKeypoints[i] = cv::KeyPoint(refCorners[i], 3);
				}
				refCorners3 = feature2D_->generateKeypoints3D(refS->sensorData(), refKeypoints);
			}

			std::map<int, cv::KeyPoint> refWords;
			std::map<int, cv::KeyPoint> newWords;
			std::map<int, cv::Point3f> refWords3Guess;
			for(unsigned int i=0; i<cornerIds.size(); ++i)
			{
				refWords.insert(std::make_pair(cornerIds[i], cv::KeyPoint(refCorners[i], 3)));
				newWords.insert(std::make_pair(cornerIds[i], cv::KeyPoint(newCorners[i], 3)));
				if(!refCorners3.empty())
				{
					refWords3Guess.insert(std::make_pair(cornerIds[i], refCorners3[i]));
				}
			}

			Transform cameraTransform;
			std::map<int, cv::Point3f> refWords3 = util3d::generateWords3DMono(
					refWords,
					newWords,
					cameraModel,
					cameraTransform,
					fundMatrixReprojError_,
					fundMatrixConfidence_,
					refWords3Guess);  // for scale estimation

			if(cameraTransform.getNorm() < minTranslation_*5)
			{
				UWARN("Camera must be moved at least %f m for initialization (current=%f)",
						minTranslation_*5, output.getNorm());
			}
			else
			{
				localMap_ = refWords3;
				// For values that we know the depth, set them for ba
				for(std::map<int, cv::Point3f>::iterator iter=refWords3.begin(); iter!=refWords3.end();)
				{
					std::map<int, cv::Point3f>::iterator jterGuess3D = refWords3Guess.find(iter->first);
					if(jterGuess3D != refWords3Guess.end() &&
					   util3d::isFinite(jterGuess3D->second))
					{
						iter->second = jterGuess3D->second;
						++iter;
					}
					else
					{
						refWords3.erase(iter++);
					}
				}
				if(!refWords3.empty())
				{
					UDEBUG("Added %d/%d valid 3D features", (int)refWords3.size(), (int)localMap_.size());
					keyFrameWords3D_.insert(std::make_pair(memory_->getLastWorkingSignature()->id(), refWords3));
				}
				keyFramePoses_.insert(std::make_pair(memory_->getLastWorkingSignature()->id(), this->getPose()));
				keyFrameModels_.insert(std::make_pair(memory_->getLastWorkingSignature()->id(), cameraModel));
			}
		}
		else
		{
			UWARN("Flow not enough high! flow=%f ki=%d", flow, oi);
		}
	}
	else
	{
		UDEBUG("INIT PART 1/2");
		//return Identity
		output = Transform::getIdentity();
		if(info)
		{
			// a very high variance tells that the new pose is not linked with the previous one
			info->reg.covariance = cv::Mat::eye(6,6,CV_64FC1)*9999.0;
		}

			// generate kpts
		if(memory_->update(SensorData(data)))
		{
			const Signature * s = memory_->getLastWorkingSignature();
			const std::multimap<int, int> & words = s->getWords();
			if((int)words.size() > minInliers_ && !s->getWordsKpts().empty())
			{
				for(std::multimap<int, int>::const_iterator iter=words.begin(); iter!=words.end(); ++iter)
				{
					if(words.count(iter->first) == 1)
					{
						firstFrameGuessCorners_.insert(std::make_pair(iter->first, s->getWordsKpts()[iter->second].pt));
					}
				}
			}
			else
			{
				memory_->deleteLocation(memory_->getLastSignatureId());
			}
		}
		else
		{
			UERROR("Failed creating signature");
		}
	}

	memory_->emptyTrash();

	if(this->isInfoDataFilled() && info)
	{
		//info->variance = variance;
		info->reg.inliers = inliers;
		info->reg.matches = correspondences;
		info->features = nFeatures;
		info->localMapSize = (int)localMap_.size();
		info->localMap = localMap_;
	}

	UINFO("Odom update=%fs tf=[%s] inliers=%d/%d, local_map[%d]=%d, accepted=%s",
			timer.elapsed(),
			output.prettyPrint().c_str(),
			inliers,
			correspondences,
			(int)memory_->getStMem().size(),
			(int)localMap_.size(),
			!output.isNull()?"true":"false");

	return output;
}

} // namespace rtabmap
