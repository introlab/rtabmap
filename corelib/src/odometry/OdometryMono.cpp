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
	maxVariance_(Parameters::defaultOdomMonoMaxVariance())
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

	Parameters::parse(parameters, Parameters::kVhEpRansacParam1(), fundMatrixReprojError_);
	Parameters::parse(parameters, Parameters::kVhEpRansacParam2(), fundMatrixConfidence_);

	// Setup memory
	ParametersMap customParameters;
	float minDepth = Parameters::defaultVisMinDepth();
	float maxDepth = Parameters::defaultVisMaxDepth();
	std::string roi = Parameters::defaultVisRoiRatios();
	Parameters::parse(parameters, Parameters::kVisMinDepth(), minDepth);
	Parameters::parse(parameters, Parameters::kVisMaxDepth(), maxDepth);
	Parameters::parse(parameters, Parameters::kVisRoiRatios(), roi);
	customParameters.insert(ParametersPair(Parameters::kKpMinDepth(), uNumber2Str(minDepth)));
	customParameters.insert(ParametersPair(Parameters::kKpMaxDepth(), uNumber2Str(maxDepth)));
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
}

OdometryMono::~OdometryMono()
{
	delete memory_;
	delete stereo_;
}

void OdometryMono::reset(const Transform & initialPose)
{
	Odometry::reset(initialPose);
	memory_->init("", false, ParametersMap());
	localMap_.clear();
	refDepthOrRight_ = cv::Mat();
	cornersMap_.clear();
	keyFrameWords3D_.clear();
	keyFramePoses_.clear();
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


	const CameraModel & cameraModel = data.stereoCameraModel().isValidForProjection()?data.stereoCameraModel().left():data.cameraModels()[0];

	UTimer timer;

	int inliers = 0;
	int correspondences = 0;
	int nFeatures = 0;

	cv::Mat newFrame;
	// convert to grayscale
	if(data.imageRaw().channels() > 1)
	{
		cv::cvtColor(data.imageRaw(), newFrame, cv::COLOR_BGR2GRAY);
	}
	else
	{
		newFrame = data.imageRaw().clone();
	}

	if(memory_->getStMem().size() >= 1)
	{
		if(localMap_.size())
		{
			//PnP
			UDEBUG("PnP");

			if(this->isInfoDataFilled() && info)
			{
				info->type = 0;
			}

			// generate kpts
			if(memory_->update(SensorData(newFrame)))
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
					UDEBUG("project points to previous image");
					cv::projectPoints(objectPoints, rvec, tvec, K, cv::Mat(), imagePoints);

					//filter points not in the image and set guess from unique correspondences
					std::vector<cv::Point3f> objectPointsTmp(objectPoints.size());
					std::vector<cv::Point2f> refCorners(objectPoints.size());
					std::vector<cv::Point2f> newCorners(objectPoints.size());
					matches.resize(objectPoints.size());
					int oi=0;
					for(unsigned int i=0; i<objectPoints.size(); ++i)
					{
						if(uIsInBounds(int(imagePoints[i].x), 0, newFrame.cols) &&
						   uIsInBounds(int(imagePoints[i].y), 0, newFrame.rows) &&
						   uIsInBounds(int(prevImagePoints[i].x), 0, prevS->sensorData().imageRaw().cols) &&
						   uIsInBounds(int(prevImagePoints[i].y), 0, prevS->sensorData().imageRaw().rows))
						{
							refCorners[oi] = prevImagePoints[i];
							newCorners[oi] = imagePoints[i];
							if(localMap_.count(ids[i]) == 1)
							{
								if(prevS->getWords().count(ids[i]) == 1)
								{
									// set guess if unique
									refCorners[oi] = prevS->getWords().find(ids[i])->second.pt;
								}
								if(newS->getWords().count(ids[i]) == 1)
								{
									// set guess if unique
									newCorners[oi] = newS->getWords().find(ids[i])->second.pt;
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
							newFrame,
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
						if(statusFlowInliers[i])
						{
							objectPoints[oi] = objectPointsTmp[i];
							imagePoints[oi] = newCorners[i];
							matchesTmp[oi] = matches[i];
							++oi;

							if(this->isInfoDataFilled() && info)
							{
								cv::KeyPoint kpt;
								if(newS->getWords().count(matches[i]) == 1)
								{
									kpt = newS->getWords().find(matches[i])->second;
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
							UASSERT(maxLikelihoodId != -1);

							// Add new points to local map
							const Signature* previousS = memory_->getSignature(maxLikelihoodId);
							UASSERT(previousS!=0);
							Transform cameraTransform = keyFramePoses_.at(previousS->id()).inverse()*this->getPose()*output;
							UDEBUG("cameraTransform guess=  %s (norm^2=%f)", cameraTransform.prettyPrint().c_str(), cameraTransform.getNormSquared());
							if(cameraTransform.getNorm() < minTranslation_)
							{
								UINFO("Translation with the nearest frame is too small (%f<%f) to add new points to local map",
										cameraTransform.getNorm(), minTranslation_);
							}
							else
							{

								double variance = 0;
								const std::map<int, cv::Point3f> & previousGuess = keyFrameWords3D_.find(previousS->id())->second;
								std::map<int, cv::Point3f> inliers3D = util3d::generateWords3DMono(
										uMultimapToMapUnique(previousS->getWords()),
										uMultimapToMapUnique(newS->getWords()),
										cameraModel,
										cameraTransform,
										iterations_,
										pnpReprojError_,
										pnpFlags_,
										pnpRefineIterations_,
										fundMatrixReprojError_,
										fundMatrixConfidence_,
										previousGuess,
										&variance);

								if((int)inliers3D.size() < minInliers_)
								{
									UWARN("Epipolar geometry not enough inliers (%d < %d), rejecting the transform (%s)...",
									(int)inliers3D.size(), minInliers_, cameraTransform.prettyPrint().c_str());
								}
								else if(variance == 0 || variance > maxVariance_)
								{
									UWARN("Variance too high %f (max = %f)", variance, maxVariance_);
								}
								else
								{
									UDEBUG("inliers3D=%d/%d variance=  %f", inliers3D.size(), newS->getWords().size(), variance);
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
										UDEBUG("Added %d words", (int)wordsToAdd.size());
									}

									if(newPtsAdded)
									{
										keyFrameWords3D_.insert(std::make_pair(newS->id(), inliers3D));
										keyFramePoses_.insert(std::make_pair(newS->id(), newPose));

										// keep only the two last signatures
										while(localHistoryMaxSize_ && (int)localMap_.size() > localHistoryMaxSize_ && memory_->getStMem().size()>2)
										{
											int nodeId = *memory_->getStMem().begin();
											std::list<int> removedPts;
											memory_->deleteLocation(nodeId, &removedPts);
											keyFrameWords3D_.erase(nodeId);
											keyFramePoses_.erase(nodeId);
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

				if(!newPtsAdded)
				{
					// remove new words from dictionary
					memory_->deleteLocation(newS->id());
				}
			}
		}
		else if(cornersMap_.size())
		{
			//flow

			if(this->isInfoDataFilled() && info)
			{
				info->type = 1;
			}

			const Signature * refS = memory_->getLastWorkingSignature();

			std::vector<cv::Point2f> refCorners(cornersMap_.size());
			std::vector<cv::Point2f> refCornersGuess(cornersMap_.size());
			std::vector<int> cornerIds(cornersMap_.size());
			int ii=0;
			for(std::map<int, cv::Point2f>::iterator iter=cornersMap_.begin(); iter!=cornersMap_.end(); ++iter)
			{
				std::multimap<int, cv::KeyPoint>::const_iterator jter=refS->getWords().find(iter->first);
				UASSERT(jter != refS->getWords().end());
				refCorners[ii] = jter->second.pt;
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
					newFrame,
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

					inliersV[oi] = i;
					cornersMap_.at(cornerIds[i]) = refCornersGuess[i];
					tmpCornersId[oi] = cornerIds[i];

					++oi;
				}
				else
				{
					cornersMap_.erase(cornerIds[i]);
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
				// compute fundamental matrix
				UDEBUG("Find fundamental matrix");
				std::vector<unsigned char> statusFInliers;
				cv::Mat F = cv::findFundamentalMat(
						refCorners,
						newCorners,
						statusFInliers,
						cv::RANSAC,
						fundMatrixReprojError_,
						fundMatrixConfidence_);
				//std::cout << "F=" << F << std::endl;

				if(!F.empty())
				{
					UDEBUG("Filtering fundamental matrix outliers...");
					std::vector<cv::Point2f> tmpNewCorners(statusFInliers.size());
					std::vector<cv::Point2f> tmpRefCorners(statusFInliers.size());
					tmpCornersId.resize(statusFInliers.size());
					oi = 0;
					UASSERT(newCorners.size() == statusFInliers.size());
					UASSERT(refCorners.size() == statusFInliers.size());
					UASSERT(cornerIds.size() == statusFInliers.size());
					std::vector<int> tmpInliers(statusFInliers.size());
					for(unsigned int i=0; i<statusFInliers.size(); ++i)
					{
						if(statusFInliers[i])
						{
							tmpNewCorners[oi] = newCorners[i];
							tmpRefCorners[oi] = refCorners[i];
							tmpInliers[oi] = inliersV[i];
							tmpCornersId[oi] = cornerIds[i];
							++oi;
						}
					}
					tmpInliers.resize(oi);
					tmpNewCorners.resize(oi);
					tmpRefCorners.resize(oi);
					tmpCornersId.resize(oi);
					newCorners = tmpNewCorners;
					refCorners = tmpRefCorners;
					inliersV = tmpInliers;
					cornerIds = tmpCornersId;
					if(this->isInfoDataFilled() && info)
					{
						// update inliers
						info->cornerInliers = inliersV;
						inliers = (int)inliersV.size();
					}
					UDEBUG("Filtering fundamental matrix outliers...done! (inliers=%d/%d)", oi, (int)statusFInliers.size());

					if((int)refCorners.size() > minInliers_)
					{
						std::vector<cv::Point2f> refCornersRefined;
						std::vector<cv::Point2f> newCornersRefined;
						//UDEBUG("Correcting matches...");
						cv::correctMatches(F, refCorners, newCorners, refCornersRefined, newCornersRefined);
						UASSERT(refCorners.size() == refCornersRefined.size());
						UASSERT(newCorners.size() == newCornersRefined.size());
						refCorners = refCornersRefined;
						newCorners = newCornersRefined;
						//UDEBUG("Correcting matches...done!");

						UDEBUG("Computing P...");
						cv::Mat K = cameraModel.K();

						cv::Mat Kinv = K.inv();
						cv::Mat E = K.t()*F*K;

						//normalize coordinates
						cv::Mat x(3, (int)refCorners.size(), CV_64FC1);
						cv::Mat xp(3, (int)refCorners.size(), CV_64FC1);
						for(unsigned int i=0; i<refCorners.size(); ++i)
						{
							x.at<double>(0, i) = refCorners[i].x;
							x.at<double>(1, i) = refCorners[i].y;
							x.at<double>(2, i) = 1;

							xp.at<double>(0, i) = newCorners[i].x;
							xp.at<double>(1, i) = newCorners[i].y;
							xp.at<double>(2, i) = 1;
						}

						cv::Mat x_norm = Kinv * x;
						cv::Mat xp_norm = Kinv * xp;
						x_norm = x_norm.rowRange(0,2);
						xp_norm = xp_norm.rowRange(0,2);

						cv::Mat P = EpipolarGeometry::findPFromE(E, x_norm, xp_norm);
						if(!P.empty())
						{
							cv::Mat P0 = cv::Mat::zeros(3, 4, CV_64FC1);
							P0.at<double>(0,0) = 1;
							P0.at<double>(1,1) = 1;
							P0.at<double>(2,2) = 1;

							UDEBUG("Computing P...done!");
							//std::cout << "P=" << P << std::endl;

							cv::Mat R, T;
							EpipolarGeometry::findRTFromP(P, R, T);

							UDEBUG("");
							std::vector<double> reprojErrors;
							pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
							EpipolarGeometry::triangulatePoints(x_norm, xp_norm, P0, P, cloud, reprojErrors);

							std::vector<cv::Point3f> inliersRef;
							std::vector<cv::Point3f> inliersRefGuess;
							std::vector<cv::Point2f> imagePoints(cloud->size());
							inliersRef.resize(cloud->size());
							inliersRefGuess.resize(cloud->size());
							tmpCornersId.resize(cloud->size());

							oi = 0;
							UASSERT(newCorners.size() == cloud->size());

							std::vector<cv::Point3f> newCorners3D;

							if(!refDepthOrRight_.empty())
							{
								if(refDepthOrRight_.type() == CV_8UC1)
								{
									StereoCameraModel m = data.stereoCameraModel();
									m.setLocalTransform(Transform::getIdentity());
									std::vector<unsigned char> stereoStatus;
									std::vector<cv::Point2f> rightCorners;
									rightCorners = stereo_->computeCorrespondences(
											refS->sensorData().imageRaw(),
											refDepthOrRight_,
											refCorners,
											stereoStatus);

									newCorners3D = util3d::generateKeypoints3DStereo(
											refCorners,
											rightCorners,
											m,
											stereoStatus);
								}
								else if(refDepthOrRight_.type() == CV_32FC1 || refDepthOrRight_.type() == CV_16UC1)
								{
									std::vector<cv::KeyPoint> tmpKpts;
									cv::KeyPoint::convert(refCorners, tmpKpts);
									CameraModel m(cameraModel.fx(), cameraModel.fy(), cameraModel.cx(), cameraModel.cy());
									 newCorners3D = util3d::generateKeypoints3DDepth(
											 tmpKpts,
											 refDepthOrRight_,
											 m);
								}
								else
								{
									UWARN("Depth or right image type not supported: %d", refDepthOrRight_.type());
								}
							}

							for(unsigned int i=0; i<cloud->size(); ++i)
							{
								if(cloud->at(i).z>0)
								{
									imagePoints[oi] = newCorners[i];
									tmpCornersId[oi] = cornerIds[i];
									inliersRef[oi].x = cloud->at(i).x;
									inliersRef[oi].y = cloud->at(i).y;
									inliersRef[oi].z = cloud->at(i).z;
									if(!newCorners3D.empty())
									{
										inliersRefGuess[oi] = newCorners3D.at(i);
									}
									++oi;
								}
							}
							imagePoints.resize(oi);
							inliersRef.resize(oi);
							inliersRefGuess.resize(oi);
							tmpCornersId.resize(oi);
							cornerIds = tmpCornersId;

							bool reject = false;

							//estimate scale
							float scale = 1;
							std::multimap<float, float> scales; // <variance, scale>
							if(!newCorners3D.empty()) // scale known
							{
								UASSERT(inliersRefGuess.size() == inliersRef.size());
								for(unsigned int i=0; i<inliersRef.size(); ++i)
								{
									if(util3d::isFinite(inliersRefGuess.at(i)))
									{
										float s = inliersRefGuess.at(i).z/inliersRef.at(i).z;
										std::vector<float> errorSqrdDists(inliersRef.size());
										oi = 0;
										for(unsigned int j=0; j<inliersRef.size(); ++j)
										{
											if(cloud->at(j).z>0)
											{
												cv::Point3f refPt = inliersRef.at(j);
												refPt.x *= s;
												refPt.y *= s;
												refPt.z *= s;
												const cv::Point3f & guess = inliersRefGuess.at(j);
												errorSqrdDists[oi++] = uNormSquared(refPt.x-guess.x, refPt.y-guess.y, refPt.z-guess.z);
											}
										}
										errorSqrdDists.resize(oi);
										if(errorSqrdDists.size() > 2)
										{
											std::sort(errorSqrdDists.begin(), errorSqrdDists.end());
											double median_error_sqr = (double)errorSqrdDists[errorSqrdDists.size () >> 1];
											float variance = 2.1981 * median_error_sqr;
											//UDEBUG("scale %d = %f variance = %f", i, s, variance);
											if(variance > 0)
											{
												scales.insert(std::make_pair(variance, s));
											}
										}
									}
								}
								if(scales.size() == 0)
								{
									UWARN("No scales found!?");
									reject = true;
								}
								else
								{
									scale = scales.begin()->second;
									UWARN("scale used = %f (variance=%f scales=%d)", scale, scales.begin()->first, (int)scales.size());

									UDEBUG("Max noise variance = %f current variance=%f", maxVariance_, scales.begin()->first);
									if(scales.begin()->first > maxVariance_)
									{
										UWARN("Too high variance %f (should be < %f)", scales.begin()->first, maxVariance_);
										reject = true; // 20 cm for good initialization
									}
								}

							}
							else if(inliersRef.size())
							{
								// find centroid of the cloud and set it to 1 meter
								Eigen::Vector4f centroid(0,0,0,0);
								pcl::PointCloud<pcl::PointXYZ> inliersRefCloud;
								inliersRefCloud.resize(inliersRef.size());
								for(unsigned int i=0; i<inliersRef.size(); ++i)
								{
									inliersRefCloud[i].x = inliersRef[i].x;
									inliersRefCloud[i].y = inliersRef[i].y;
									inliersRefCloud[i].z = inliersRef[i].z;
								}
								pcl::compute3DCentroid(inliersRefCloud, centroid);
								scale = 1.0f / centroid[2];
							}
							else
							{
								reject = true;
							}

							if(!reject)
							{
								//PnPRansac
								std::vector<cv::Point3f> objectPoints(inliersRef.size());
								for(unsigned int i=0; i<inliersRef.size(); ++i)
								{
									objectPoints[i].x = inliersRef.at(i).x * scale;
									objectPoints[i].y = inliersRef.at(i).y * scale;
									objectPoints[i].z = inliersRef.at(i).z * scale;
								}
								cv::Mat rvec;
								cv::Mat tvec;
								std::vector<int> inliersPnP;
								util3d::solvePnPRansac(
										objectPoints, // 3D points in ref referential
										imagePoints,  // 2D points in new referential
										K,
										cv::Mat(),
										rvec,
										tvec,
										false,
										iterations_,
										pnpReprojError_,
										0, // min inliers
										inliersPnP,
										pnpFlags_,
										pnpRefineIterations_);

								UDEBUG("PnP inliers = %d / %d", (int)inliersPnP.size(), (int)objectPoints.size());

								cv::Rodrigues(rvec, R);
								Transform pnp(R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2), tvec.at<double>(0),
											  R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2), tvec.at<double>(1),
											  R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2), tvec.at<double>(2));

								output = cameraModel.localTransform() * pnp.inverse() * cameraModel.localTransform().inverse();
								if(output.getNorm() < minTranslation_*5)
								{
									reject = true;
									UWARN("Camera must be moved at least %f m for initialization (current=%f)",
											minTranslation_*5, output.getNorm());
								}

								if(!reject)
								{
									///
									std::vector<int> wordsId = uKeys(memory_->getLastWorkingSignature()->getWords());
									UASSERT(wordsId.size());
									UASSERT(cornerIds.size() == objectPoints.size());
									std::map<int, cv::Point3f> keyFrameWords3D;
									Transform t = this->getPose()*cameraModel.localTransform();
									for(unsigned int i=0; i<inliersPnP.size(); ++i)
									{
										int index  =inliersPnP.at(i);
										int id = cornerIds[index];
										UASSERT(id > 0 && id <= *wordsId.rbegin());
										cv::Point3f pt =  util3d::transformPoint(
												objectPoints.at(index),
												t);
										localMap_.insert(std::make_pair(id, cv::Point3f(pt.x, pt.y, pt.z)));
										keyFrameWords3D.insert(std::make_pair(id, pt));
									}

									keyFrameWords3D_.insert(std::make_pair(memory_->getLastWorkingSignature()->id(), keyFrameWords3D));
									keyFramePoses_.insert(std::make_pair(memory_->getLastWorkingSignature()->id(), this->getPose()));
								}
							}
						}
						else
						{
							UERROR("No valid camera matrix found!");
						}
					}
					else
					{
						UWARN("Not enough inliers %d/%d", (int)refCorners.size(), minInliers_);
					}
				}
				else
				{
					UWARN("Fundamental matrix not found!");
				}
			}
			else
			{
				UWARN("Flow not enough high! flow=%f ki=%d", flow, oi);
			}
		}
	}
	else
	{
		//return Identity
		output = Transform::getIdentity();
		if(info)
		{
			// a very high variance tells that the new pose is not linked with the previous one
			info->reg.covariance = cv::Mat::eye(6,6,CV_64FC1)*9999.0;
		}

			// generate kpts
		if(memory_->update(SensorData(newFrame)))
		{
			const std::multimap<int, cv::KeyPoint> & words = memory_->getLastWorkingSignature()->getWords();
			if((int)words.size() > minInliers_)
			{
				for(std::multimap<int, cv::KeyPoint>::const_iterator iter=words.begin(); iter!=words.end(); ++iter)
				{
					if(words.count(iter->first) == 1)
					{
						cornersMap_.insert(std::make_pair(iter->first, iter->second.pt));
					}
				}
				refDepthOrRight_ = data.depthOrRightRaw().clone();
				keyFramePoses_.insert(std::make_pair(memory_->getLastSignatureId(), Transform::getIdentity()));
			}
			else
			{
				UWARN("Too low 2D corners (%d), ignoring new frame...",
						(int)words.size());
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
