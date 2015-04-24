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
#include "rtabmap/core/Memory.h"
#include "rtabmap/core/Signature.h"
#include "rtabmap/core/util3d.h"
#include "rtabmap/core/VWDictionary.h"
#include "rtabmap/utilite/ULogger.h"
#include "rtabmap/utilite/UTimer.h"
#include "rtabmap/utilite/UConversion.h"
#include <opencv2/calib3d/calib3d.hpp>

#if _MSC_VER
	#define ISFINITE(value) _finite(value)
#else
	#define ISFINITE(value) std::isfinite(value)
#endif

namespace rtabmap {

OdometryBOW::OdometryBOW(const ParametersMap & parameters) :
	Odometry(parameters),
	_localHistoryMaxSize(Parameters::defaultOdomBowLocalHistorySize()),
	_memory(0)
{
	Parameters::parse(parameters, Parameters::kOdomBowLocalHistorySize(), _localHistoryMaxSize);

	ParametersMap customParameters;
	customParameters.insert(ParametersPair(Parameters::kKpMaxDepth(), uNumber2Str(this->getMaxDepth())));
	customParameters.insert(ParametersPair(Parameters::kKpRoiRatios(), this->getRoiRatios()));
	customParameters.insert(ParametersPair(Parameters::kMemRehearsalSimilarity(), "1.0")); // desactivate rehearsal
	customParameters.insert(ParametersPair(Parameters::kMemBinDataKept(), "false"));
	customParameters.insert(ParametersPair(Parameters::kMemSTMSize(), "0"));
	customParameters.insert(ParametersPair(Parameters::kMemNotLinkedNodesKept(), "false"));
	int nn = Parameters::defaultOdomBowNNType();
	float nndr = Parameters::defaultOdomBowNNDR();
	int featureType = Parameters::defaultOdomFeatureType();
	int maxFeatures = Parameters::defaultOdomMaxFeatures();
	Parameters::parse(parameters, Parameters::kOdomBowNNType(), nn);
	Parameters::parse(parameters, Parameters::kOdomBowNNDR(), nndr);
	Parameters::parse(parameters, Parameters::kOdomFeatureType(), featureType);
	Parameters::parse(parameters, Parameters::kOdomMaxFeatures(), maxFeatures);
	customParameters.insert(ParametersPair(Parameters::kKpNNStrategy(), uNumber2Str(nn)));
	customParameters.insert(ParametersPair(Parameters::kKpNndrRatio(), uNumber2Str(nndr)));
	customParameters.insert(ParametersPair(Parameters::kKpDetectorStrategy(), uNumber2Str(featureType)));
	customParameters.insert(ParametersPair(Parameters::kKpWordsPerImage(), uNumber2Str(maxFeatures)));

	// Memory's stereo parameters, copy from Odometry
	int subPixWinSize = Parameters::defaultOdomSubPixWinSize();
	int subPixIterations = Parameters::defaultOdomSubPixIterations();
	double subPixEps = Parameters::defaultOdomSubPixEps();
	Parameters::parse(parameters, Parameters::kOdomSubPixWinSize(), subPixWinSize);
	Parameters::parse(parameters, Parameters::kOdomSubPixIterations(), subPixIterations);
	Parameters::parse(parameters, Parameters::kOdomSubPixEps(), subPixEps);
	customParameters.insert(ParametersPair(Parameters::kKpSubPixWinSize(), uNumber2Str(subPixWinSize)));
	customParameters.insert(ParametersPair(Parameters::kKpSubPixIterations(), uNumber2Str(subPixIterations)));
	customParameters.insert(ParametersPair(Parameters::kKpSubPixEps(), uNumber2Str(subPixEps)));

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
	UDEBUG("");
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

	if(info)
	{
		info->type = 0;
	}

	double variance = 0;
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
			if(this->isInfoDataFilled() && info)
			{
				info->words = newSignature->getWords();
			}
		}

		if(previousSignature && newSignature)
		{
			Transform transform;
			if((int)localMap_.size() >= this->getMinInliers())
			{
				if(this->isPnPEstimationUsed())
				{
					if((int)newSignature->getWords().size() >= this->getMinInliers())
					{
						// find correspondences
						std::vector<int> ids = uListToVector(uUniqueKeys(newSignature->getWords()));
						std::vector<cv::Point3f> objectPoints(ids.size());
						std::vector<cv::Point2f> imagePoints(ids.size());
						int oi=0;
						std::vector<int> matches(ids.size());
						for(unsigned int i=0; i<ids.size(); ++i)
						{
							if(localMap_.count(ids[i]) == 1)
							{
								pcl::PointXYZ pt = localMap_.find(ids[i])->second;
								objectPoints[oi].x = pt.x;
								objectPoints[oi].y = pt.y;
								objectPoints[oi].z = pt.z;
								imagePoints[oi] = newSignature->getWords().find(ids[i])->second.pt;
								matches[oi++] = ids[i];
							}
						}

						objectPoints.resize(oi);
						imagePoints.resize(oi);
						matches.resize(oi);

						if(this->isInfoDataFilled() && info)
						{
							info->wordMatches.insert(info->wordMatches.end(), matches.begin(), matches.end());
						}
						correspondences = (int)matches.size();

						if((int)matches.size() >= this->getMinInliers())
						{
							//PnPRansac
							cv::Mat K = (cv::Mat_<double>(3,3) <<
								data.fx(), 0, data.cx(),
								0, data.fy()>0?data.fy():data.fx(), data.cy(),
								0, 0, 1);
							Transform guess = (this->getPose() * data.localTransform()).inverse();
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
								transform = (data.localTransform() * pnp * this->getPose()).inverse();

								UDEBUG("Odom transform = %s", transform.prettyPrint().c_str());

								// compute variance (like in PCL computeVariance() method of sac_model.h)
								std::vector<float> errorSqrdDists(inliersV.size());
								for(unsigned int i=0; i<inliersV.size(); ++i)
								{
									std::multimap<int, pcl::PointXYZ>::const_iterator iter = newSignature->getWords3().find(matches[inliersV[i]]);
									UASSERT(iter != newSignature->getWords3().end());
									const cv::Point3f & objPt = objectPoints[inliersV[i]];
									pcl::PointXYZ newPt = util3d::transformPoint(iter->second, this->getPose()*transform);
									errorSqrdDists[i] = uNormSquared(objPt.x-newPt.x, objPt.y-newPt.y, objPt.z-newPt.z);
								}
								std::sort(errorSqrdDists.begin(), errorSqrdDists.end());
								double median_error_sqr = (double)errorSqrdDists[errorSqrdDists.size () >> 1];
								variance = 2.1981 * median_error_sqr;
							}
							else
							{
								UWARN("PnP not enough inliers (%d < %d), rejecting the transform...", (int)inliersV.size(), this->getMinInliers());
							}

							if(this->isInfoDataFilled() && info && inliersV.size())
							{
								info->wordInliers.resize(inliersV.size());
								for(unsigned int i=0; i<inliersV.size(); ++i)
								{
									info->wordInliers[i] = matches[inliersV[i]];
								}
							}
						}
						else
						{
							UWARN("Not enough correspondences (%d < %d)", correspondences, this->getMinInliers());
						}
					}
					else
					{
						UWARN("Not enough features in the new image (%d < %d)", (int)newSignature->getWords().size(), this->getMinInliers());
					}
				}
				else
				{
					if((int)newSignature->getWords3().size() >= this->getMinInliers())
					{
						pcl::PointCloud<pcl::PointXYZ>::Ptr inliers1(new pcl::PointCloud<pcl::PointXYZ>); // previous
						pcl::PointCloud<pcl::PointXYZ>::Ptr inliers2(new pcl::PointCloud<pcl::PointXYZ>); // new

						// No need to set max depth here, it is already applied in extractKeypointsAndDescriptors() above.
						// Also! the localMap_ have points not in camera frame anymore (in local map frame), so filtering
						// by depth here is wrong!
						std::set<int> uniqueCorrespondences;
						util3d::findCorrespondences(
								localMap_,
								newSignature->getWords3(),
								*inliers1,
								*inliers2,
								0,
								&uniqueCorrespondences);

						UDEBUG("localMap=%d, new=%d, unique correspondences=%d", (int)localMap_.size(), (int)newSignature->getWords3().size(), (int)uniqueCorrespondences.size());

						if(this->isInfoDataFilled() && info)
						{
							info->wordMatches.insert(info->wordMatches.end(), uniqueCorrespondences.begin(), uniqueCorrespondences.end());
						}

						correspondences = (int)inliers1->size();
						if((int)inliers1->size() >= this->getMinInliers())
						{
							// the transform returned is global odometry pose, not incremental one
							std::vector<int> inliersV;
							Transform t = util3d::transformFromXYZCorrespondences(
									inliers2,
									inliers1,
									this->getInlierDistance(),
									this->getIterations(),
									this->getRefineIterations()>0, 3.0, this->getRefineIterations(),
									&inliersV,
									&variance);

							inliers = (int)inliersV.size();
							if(!t.isNull() && inliers >= this->getMinInliers())
							{
								// make it incremental
								transform = this->getPose().inverse() * t;

								UDEBUG("Odom transform = %s", transform.prettyPrint().c_str());
							}
							else
							{
								UWARN("Transform not valid (inliers = %d/%d)", inliers, correspondences);
							}

							if(this->isInfoDataFilled() && info && inliersV.size())
							{
								info->wordInliers.resize(inliersV.size());
								for(unsigned int i=0; i<inliersV.size(); ++i)
								{
									info->wordInliers[i] = info->wordMatches[inliersV[i]];
								}
							}
						}
						else
						{
							UWARN("Not enough inliers %d < %d", (int)inliers1->size(), this->getMinInliers());
						}
					}
					else
					{
						UWARN("Not enough 3D features in the new image (%d < %d)", (int)newSignature->getWords3().size(), this->getMinInliers());
					}
				}
			}
			else
			{
				UWARN("Local map too small!? (%d < %d)", (int)localMap_.size(), this->getMinInliers());
			}

			if(transform.isNull())
			{
				_memory->deleteLocation(newSignature->id());
			}
			else
			{
				output = transform;
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
		else if(!previousSignature && newSignature)
		{
			localMap_.clear();

			int count = 0;
			std::list<int> uniques = uUniqueKeys(newSignature->getWords3());
			if((int)uniques.size() >= this->getMinInliers())
			{
				output.setIdentity();

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
			}
			else
			{
				// not enough features, just delete it
				_memory->deleteLocation(newSignature->id());
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

	UINFO("Odom update time = %fs lost=%s features=%d inliers=%d/%d variance=%f local_map=%d dict=%d nodes=%d",
			timer.elapsed(),
			output.isNull()?"true":"false",
			nFeatures,
			inliers,
			correspondences,
			variance,
			(int)localMap_.size(),
			(int)_memory->getVWDictionary()->getVisualWords().size(),
			(int)_memory->getStMem().size());
	return output;
}

} // namespace rtabmap
