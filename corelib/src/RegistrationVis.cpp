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



#include <rtabmap/core/RegistrationVis.h>
#include <rtabmap/core/util3d_motion_estimation.h>
#include <rtabmap/core/util3d_features.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/core/VWDictionary.h>
#include <rtabmap/core/util2d.h>
#include <rtabmap/core/Features2d.h>
#include <rtabmap/core/VisualWord.h>
#include <rtabmap/core/Optimizer.h>
#include <rtabmap/core/util3d_transforms.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/utilite/UMath.h>

#include <rtflann/flann.hpp>

namespace rtabmap {

RegistrationVis::RegistrationVis(const ParametersMap & parameters, Registration * child) :
		Registration(parameters, child),
		_minInliers(Parameters::defaultVisMinInliers()),
		_inlierDistance(Parameters::defaultVisInlierDistance()),
		_iterations(Parameters::defaultVisIterations()),
		_refineIterations(Parameters::defaultVisRefineIterations()),
		_epipolarGeometryVar(Parameters::defaultVisEpipolarGeometryVar()),
		_estimationType(Parameters::defaultVisEstimationType()),
		_forwardEstimateOnly(Parameters::defaultVisForwardEstOnly()),
		_PnPReprojError(Parameters::defaultVisPnPReprojError()),
		_PnPFlags(Parameters::defaultVisPnPFlags()),
		_PnPRefineIterations(Parameters::defaultVisPnPRefineIterations()),
		_correspondencesApproach(Parameters::defaultVisCorType()),
		_flowWinSize(Parameters::defaultVisCorFlowWinSize()),
		_flowIterations(Parameters::defaultVisCorFlowIterations()),
		_flowEps(Parameters::defaultVisCorFlowEps()),
		_flowMaxLevel(Parameters::defaultVisCorFlowMaxLevel()),
		_nndr(Parameters::defaultVisCorNNDR()),
		_guessWinSize(Parameters::defaultVisCorGuessWinSize()),
		_guessMatchToProjection(Parameters::defaultVisCorGuessMatchToProjection()),
		_bundleAdjustment(Parameters::defaultVisBundleAdjustment()),
		_depthAsMask(Parameters::defaultVisDepthAsMask())
{
	_featureParameters = Parameters::getDefaultParameters();
	uInsert(_featureParameters, ParametersPair(Parameters::kKpNNStrategy(), _featureParameters.at(Parameters::kVisCorNNType())));
	uInsert(_featureParameters, ParametersPair(Parameters::kKpNndrRatio(), _featureParameters.at(Parameters::kVisCorNNDR())));
	uInsert(_featureParameters, ParametersPair(Parameters::kKpDetectorStrategy(), _featureParameters.at(Parameters::kVisFeatureType())));
	uInsert(_featureParameters, ParametersPair(Parameters::kKpMaxFeatures(), _featureParameters.at(Parameters::kVisMaxFeatures())));
	uInsert(_featureParameters, ParametersPair(Parameters::kKpMaxDepth(), _featureParameters.at(Parameters::kVisMaxDepth())));
	uInsert(_featureParameters, ParametersPair(Parameters::kKpMinDepth(), _featureParameters.at(Parameters::kVisMinDepth())));
	uInsert(_featureParameters, ParametersPair(Parameters::kKpRoiRatios(), _featureParameters.at(Parameters::kVisRoiRatios())));
	uInsert(_featureParameters, ParametersPair(Parameters::kKpSubPixEps(), _featureParameters.at(Parameters::kVisSubPixWinSize())));
	uInsert(_featureParameters, ParametersPair(Parameters::kKpSubPixIterations(), _featureParameters.at(Parameters::kVisSubPixIterations())));
	uInsert(_featureParameters, ParametersPair(Parameters::kKpSubPixWinSize(), _featureParameters.at(Parameters::kVisSubPixEps())));
	uInsert(_featureParameters, ParametersPair(Parameters::kKpGridRows(), _featureParameters.at(Parameters::kVisGridRows())));
	uInsert(_featureParameters, ParametersPair(Parameters::kKpGridCols(), _featureParameters.at(Parameters::kVisGridCols())));
	uInsert(_featureParameters, ParametersPair(Parameters::kKpNewWordsComparedTogether(), "false"));

	this->parseParameters(parameters);
}

void RegistrationVis::parseParameters(const ParametersMap & parameters)
{
	Registration::parseParameters(parameters);

	Parameters::parse(parameters, Parameters::kVisMinInliers(), _minInliers);
	Parameters::parse(parameters, Parameters::kVisInlierDistance(), _inlierDistance);
	Parameters::parse(parameters, Parameters::kVisIterations(), _iterations);
	Parameters::parse(parameters, Parameters::kVisRefineIterations(), _refineIterations);
	Parameters::parse(parameters, Parameters::kVisEstimationType(), _estimationType);
	Parameters::parse(parameters, Parameters::kVisForwardEstOnly(), _forwardEstimateOnly);
	Parameters::parse(parameters, Parameters::kVisEpipolarGeometryVar(), _epipolarGeometryVar);
	Parameters::parse(parameters, Parameters::kVisPnPReprojError(), _PnPReprojError);
	Parameters::parse(parameters, Parameters::kVisPnPFlags(), _PnPFlags);
	Parameters::parse(parameters, Parameters::kVisPnPRefineIterations(), _PnPRefineIterations);
	Parameters::parse(parameters, Parameters::kVisCorType(), _correspondencesApproach);
	Parameters::parse(parameters, Parameters::kVisCorFlowWinSize(), _flowWinSize);
	Parameters::parse(parameters, Parameters::kVisCorFlowIterations(), _flowIterations);
	Parameters::parse(parameters, Parameters::kVisCorFlowEps(), _flowEps);
	Parameters::parse(parameters, Parameters::kVisCorFlowMaxLevel(), _flowMaxLevel);
	Parameters::parse(parameters, Parameters::kVisCorNNDR(), _nndr);
	Parameters::parse(parameters, Parameters::kVisCorGuessWinSize(), _guessWinSize);
	Parameters::parse(parameters, Parameters::kVisCorGuessMatchToProjection(), _guessMatchToProjection);
	Parameters::parse(parameters, Parameters::kVisBundleAdjustment(), _bundleAdjustment);
	Parameters::parse(parameters, Parameters::kVisDepthAsMask(), _depthAsMask);
	uInsert(_bundleParameters, parameters);

	UASSERT_MSG(_minInliers >= 1, uFormat("value=%d", _minInliers).c_str());
	UASSERT_MSG(_inlierDistance > 0.0f, uFormat("value=%f", _inlierDistance).c_str());
	UASSERT_MSG(_iterations > 0, uFormat("value=%d", _iterations).c_str());

	// override feature parameters
	for(ParametersMap::const_iterator iter=parameters.begin(); iter!=parameters.end(); ++iter)
	{
		std::string group = uSplit(iter->first, '/').front();
		if(Parameters::isFeatureParameter(iter->first) || group.compare("Stereo") == 0)
		{
			uInsert(_featureParameters, ParametersPair(iter->first, iter->second));
		}
	}

	if(uContains(parameters, Parameters::kVisCorNNType()))
	{
		uInsert(_featureParameters, ParametersPair(Parameters::kKpNNStrategy(), parameters.at(Parameters::kVisCorNNType())));
	}
	if(uContains(parameters, Parameters::kVisCorNNDR()))
	{
		uInsert(_featureParameters, ParametersPair(Parameters::kKpNndrRatio(), parameters.at(Parameters::kVisCorNNDR())));
	}
	if(uContains(parameters, Parameters::kVisFeatureType()))
	{
		uInsert(_featureParameters, ParametersPair(Parameters::kKpDetectorStrategy(), parameters.at(Parameters::kVisFeatureType())));
	}
	if(uContains(parameters, Parameters::kVisMaxFeatures()))
	{
		uInsert(_featureParameters, ParametersPair(Parameters::kKpMaxFeatures(), parameters.at(Parameters::kVisMaxFeatures())));
	}
	if(uContains(parameters, Parameters::kVisMaxDepth()))
	{
		uInsert(_featureParameters, ParametersPair(Parameters::kKpMaxDepth(), parameters.at(Parameters::kVisMaxDepth())));
	}
	if(uContains(parameters, Parameters::kVisMinDepth()))
	{
		uInsert(_featureParameters, ParametersPair(Parameters::kKpMinDepth(), parameters.at(Parameters::kVisMinDepth())));
	}
	if(uContains(parameters, Parameters::kVisRoiRatios()))
	{
		uInsert(_featureParameters, ParametersPair(Parameters::kKpRoiRatios(), parameters.at(Parameters::kVisRoiRatios())));
	}
	if(uContains(parameters, Parameters::kVisSubPixEps()))
	{
		uInsert(_featureParameters, ParametersPair(Parameters::kKpSubPixEps(), parameters.at(Parameters::kVisSubPixEps())));
	}
	if(uContains(parameters, Parameters::kVisSubPixIterations()))
	{
		uInsert(_featureParameters, ParametersPair(Parameters::kKpSubPixIterations(), parameters.at(Parameters::kVisSubPixIterations())));
	}
	if(uContains(parameters, Parameters::kVisSubPixWinSize()))
	{
		uInsert(_featureParameters, ParametersPair(Parameters::kKpSubPixWinSize(), parameters.at(Parameters::kVisSubPixWinSize())));
	}
	if(uContains(parameters, Parameters::kVisGridRows()))
	{
		uInsert(_featureParameters, ParametersPair(Parameters::kKpGridRows(), parameters.at(Parameters::kVisGridRows())));
	}
	if(uContains(parameters, Parameters::kVisGridCols()))
	{
		uInsert(_featureParameters, ParametersPair(Parameters::kKpGridCols(), parameters.at(Parameters::kVisGridCols())));
	}
}

RegistrationVis::~RegistrationVis()
{
}

Feature2D * RegistrationVis::createFeatureDetector() const
{
	return Feature2D::create(_featureParameters);
}

Transform RegistrationVis::computeTransformationImpl(
			Signature & fromSignature,
			Signature & toSignature,
			Transform guess, // (flowMaxLevel is set to 0 when guess is used)
			RegistrationInfo & info) const
{
	UDEBUG("%s=%d", Parameters::kVisMinInliers().c_str(), _minInliers);
	UDEBUG("%s=%f", Parameters::kVisInlierDistance().c_str(), _inlierDistance);
	UDEBUG("%s=%d", Parameters::kVisIterations().c_str(), _iterations);
	UDEBUG("%s=%d", Parameters::kVisEstimationType().c_str(), _estimationType);
	UDEBUG("%s=%d", Parameters::kVisForwardEstOnly().c_str(), _forwardEstimateOnly);
	UDEBUG("%s=%f", Parameters::kVisEpipolarGeometryVar().c_str(), _epipolarGeometryVar);
	UDEBUG("%s=%f", Parameters::kVisPnPReprojError().c_str(), _PnPReprojError);
	UDEBUG("%s=%d", Parameters::kVisPnPFlags().c_str(), _PnPFlags);
	UDEBUG("%s=%d", Parameters::kVisCorType().c_str(), _correspondencesApproach);
	UDEBUG("%s=%d", Parameters::kVisCorFlowWinSize().c_str(), _flowWinSize);
	UDEBUG("%s=%d", Parameters::kVisCorFlowIterations().c_str(), _flowIterations);
	UDEBUG("%s=%f", Parameters::kVisCorFlowEps().c_str(), _flowEps);
	UDEBUG("%s=%d", Parameters::kVisCorFlowMaxLevel().c_str(), _flowMaxLevel);
	UDEBUG("guess=%s", guess.prettyPrint().c_str());

	UDEBUG("Input(%d): from=%d words, %d 3D words, %d words descriptors,  %d kpts, %d kpts3D, %d descriptors, image=%dx%d",
			fromSignature.id(),
			(int)fromSignature.getWords().size(),
			(int)fromSignature.getWords3().size(),
			(int)fromSignature.getWordsDescriptors().size(),
			(int)fromSignature.sensorData().keypoints().size(),
			(int)fromSignature.sensorData().keypoints3D().size(),
			fromSignature.sensorData().descriptors().rows,
			fromSignature.sensorData().imageRaw().cols,
			fromSignature.sensorData().imageRaw().rows);

	UDEBUG("Input(%d): to=%d words, %d 3D words, %d words descriptors, %d kpts, %d kpts3D, %d descriptors, image=%dx%d",
			toSignature.id(),
			(int)toSignature.getWords().size(),
			(int)toSignature.getWords3().size(),
			(int)toSignature.getWordsDescriptors().size(),
			(int)toSignature.sensorData().keypoints().size(),
			(int)toSignature.sensorData().keypoints3D().size(),
			toSignature.sensorData().descriptors().rows,
			toSignature.sensorData().imageRaw().cols,
			toSignature.sensorData().imageRaw().rows);

	std::string msg;
	info.projectedIDs.clear();

	////////////////////
	// Find correspondences
	////////////////////
	//recompute correspondences if descriptors are provided
	if((fromSignature.getWordsDescriptors().empty() && toSignature.getWordsDescriptors().empty()) &&
	   (_estimationType<2 || fromSignature.getWords().size()) && // required only for 2D->2D
	   (_estimationType==0 || toSignature.getWords().size()) && // required only for 3D->2D or 2D->2D
	   fromSignature.getWords3().size() && // required in all estimation approaches
	   (_estimationType==1 || toSignature.getWords3().size())) // required only for 3D->3D and 2D->2D
	{
		// no need to extract new features, we have all the data we need
		UDEBUG("Bypassing feature matching as descriptors and images are empty. We assume features are already matched.");
	}
	else
	{
		UDEBUG("");
		// just some checks to make sure that input data are ok
		UASSERT(fromSignature.getWords().empty() ||
				fromSignature.getWords3().empty() ||
				(fromSignature.getWords().size() == fromSignature.getWords3().size()));
		UASSERT((int)fromSignature.sensorData().keypoints().size() == fromSignature.sensorData().descriptors().rows ||
				fromSignature.getWords().size() == fromSignature.getWordsDescriptors().size() ||
				fromSignature.sensorData().descriptors().rows == 0 ||
				fromSignature.getWordsDescriptors().size() == 0);
		UASSERT((toSignature.getWords().empty() && toSignature.getWords3().empty())||
				(toSignature.getWords().size() && toSignature.getWords3().empty())||
				(toSignature.getWords().size() == toSignature.getWords3().size()));
		UASSERT((int)toSignature.sensorData().keypoints().size() == toSignature.sensorData().descriptors().rows ||
				toSignature.getWords().size() == toSignature.getWordsDescriptors().size() ||
				toSignature.sensorData().descriptors().rows == 0 ||
				toSignature.getWordsDescriptors().size() == 0);
		UASSERT(fromSignature.sensorData().imageRaw().empty() ||
				fromSignature.sensorData().imageRaw().type() == CV_8UC1 ||
				fromSignature.sensorData().imageRaw().type() == CV_8UC3);
		UASSERT(toSignature.sensorData().imageRaw().empty() ||
				toSignature.sensorData().imageRaw().type() == CV_8UC1 ||
				toSignature.sensorData().imageRaw().type() == CV_8UC3);

		Feature2D * detector = createFeatureDetector();
		std::vector<cv::KeyPoint> kptsFrom;
		cv::Mat imageFrom = fromSignature.sensorData().imageRaw();
		cv::Mat imageTo = toSignature.sensorData().imageRaw();

		std::vector<int> orignalWordsFromIds;
		if(fromSignature.getWords().empty())
		{
			if(fromSignature.sensorData().keypoints().empty())
			{
				if(!imageFrom.empty())
				{
					if(imageFrom.channels() > 1)
					{
						cv::Mat tmp;
						cv::cvtColor(imageFrom, tmp, cv::COLOR_BGR2GRAY);
						imageFrom = tmp;
					}

					cv::Mat depthMask;
					if(!fromSignature.sensorData().depthRaw().empty() && _depthAsMask)
					{
						if(imageFrom.rows % fromSignature.sensorData().depthRaw().rows == 0 &&
						   imageFrom.cols % fromSignature.sensorData().depthRaw().cols == 0 &&
						   imageFrom.rows/fromSignature.sensorData().depthRaw().rows == fromSignature.sensorData().imageRaw().cols/fromSignature.sensorData().depthRaw().cols)
						{
							depthMask = util2d::interpolate(fromSignature.sensorData().depthRaw(), fromSignature.sensorData().imageRaw().rows/fromSignature.sensorData().depthRaw().rows, 0.1f);
						}
					}

					kptsFrom = detector->generateKeypoints(
							imageFrom,
							depthMask);
				}
			}
			else
			{
				kptsFrom = fromSignature.sensorData().keypoints();
			}
		}
		else
		{
			kptsFrom.resize(fromSignature.getWords().size());
			orignalWordsFromIds.resize(fromSignature.getWords().size());
			int i=0;
			bool allUniques = true;
			for(std::multimap<int, cv::KeyPoint>::const_iterator iter=fromSignature.getWords().begin(); iter!=fromSignature.getWords().end(); ++iter)
			{
				kptsFrom[i] = iter->second;
				orignalWordsFromIds[i] = iter->first;
				if(i>0 && iter->first==orignalWordsFromIds[i-1])
				{
					allUniques = false;
				}
				++i;
			}
			if(!allUniques)
			{
				UDEBUG("IDs are not unique, IDs will be regenerated!");
				orignalWordsFromIds.clear();
			}
		}

		std::multimap<int, cv::KeyPoint> wordsFrom;
		std::multimap<int, cv::KeyPoint> wordsTo;
		std::multimap<int, cv::Point3f> words3From;
		std::multimap<int, cv::Point3f> words3To;
		std::multimap<int, cv::Mat> wordsDescFrom;
		std::multimap<int, cv::Mat> wordsDescTo;
		if(_correspondencesApproach == 1) //Optical Flow
		{
			UDEBUG("");
			// convert to grayscale
			if(imageFrom.channels() > 1)
			{
				cv::Mat tmp;
				cv::cvtColor(imageFrom, tmp, cv::COLOR_BGR2GRAY);
				imageFrom = tmp;
			}
			if(imageTo.channels() > 1)
			{
				cv::Mat tmp;
				cv::cvtColor(imageTo, tmp, cv::COLOR_BGR2GRAY);
				imageTo = tmp;
			}

			std::vector<cv::Point3f> kptsFrom3D;
			if(kptsFrom.size() == fromSignature.getWords3().size())
			{
				kptsFrom3D = uValues(fromSignature.getWords3());
			}
			else if(kptsFrom.size() == fromSignature.sensorData().keypoints3D().size())
			{
				kptsFrom3D = fromSignature.sensorData().keypoints3D();
			}
			else
			{
				kptsFrom3D = detector->generateKeypoints3D(fromSignature.sensorData(), kptsFrom);
			}

			if(!imageFrom.empty() && !imageTo.empty())
			{
				std::vector<cv::Point2f> cornersFrom;
				cv::KeyPoint::convert(kptsFrom, cornersFrom);
				std::vector<cv::Point2f> cornersTo;
				bool guessSet = !guess.isIdentity() && !guess.isNull();
				if(guessSet)
				{
					Transform localTransform = fromSignature.sensorData().cameraModels().size()?fromSignature.sensorData().cameraModels()[0].localTransform():fromSignature.sensorData().stereoCameraModel().left().localTransform();
					Transform guessCameraRef = (guess * localTransform).inverse();
					cv::Mat R = (cv::Mat_<double>(3,3) <<
							(double)guessCameraRef.r11(), (double)guessCameraRef.r12(), (double)guessCameraRef.r13(),
							(double)guessCameraRef.r21(), (double)guessCameraRef.r22(), (double)guessCameraRef.r23(),
							(double)guessCameraRef.r31(), (double)guessCameraRef.r32(), (double)guessCameraRef.r33());
					cv::Mat rvec(1,3, CV_64FC1);
					cv::Rodrigues(R, rvec);
					cv::Mat tvec = (cv::Mat_<double>(1,3) << (double)guessCameraRef.x(), (double)guessCameraRef.y(), (double)guessCameraRef.z());
					cv::Mat K = fromSignature.sensorData().cameraModels().size()?fromSignature.sensorData().cameraModels()[0].K():fromSignature.sensorData().stereoCameraModel().left().K();
					cv::projectPoints(kptsFrom3D, rvec, tvec, K, cv::Mat(), cornersTo);
				}

				// Find features in the new left image
				UDEBUG("guessSet = %d", guessSet?1:0);
				std::vector<unsigned char> status;
				std::vector<float> err;
				UDEBUG("cv::calcOpticalFlowPyrLK() begin");
				cv::calcOpticalFlowPyrLK(
						imageFrom,
						imageTo,
						cornersFrom,
						cornersTo,
						status,
						err,
						cv::Size(_flowWinSize, _flowWinSize),
						guessSet?0:_flowMaxLevel,
						cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, _flowIterations, _flowEps),
						cv::OPTFLOW_LK_GET_MIN_EIGENVALS | (guessSet?cv::OPTFLOW_USE_INITIAL_FLOW:0), 1e-4);
				UDEBUG("cv::calcOpticalFlowPyrLK() end");

				UASSERT(kptsFrom.size() == kptsFrom3D.size());
				std::vector<cv::KeyPoint> kptsTo(kptsFrom.size());
				std::vector<cv::Point3f> kptsFrom3DKept(kptsFrom3D.size());
				std::vector<int> orignalWordsFromIdsCpy = orignalWordsFromIds;
				int ki = 0;
				for(unsigned int i=0; i<status.size(); ++i)
				{
					if(status[i] &&
					   uIsInBounds(cornersTo[i].x, 0.0f, float(imageTo.cols)) &&
					   uIsInBounds(cornersTo[i].y, 0.0f, float(imageTo.rows)))
					{
						if(orignalWordsFromIdsCpy.size())
						{
							orignalWordsFromIds[ki] = orignalWordsFromIdsCpy[i];
						}
						kptsFrom[ki] = cv::KeyPoint(cornersFrom[i], 1);
						kptsFrom3DKept[ki] = kptsFrom3D[i];
						kptsTo[ki++] = cv::KeyPoint(cornersTo[i], 1);
					}
				}
				if(orignalWordsFromIds.size())
				{
					orignalWordsFromIds.resize(ki);
				}
				kptsFrom.resize(ki);
				kptsTo.resize(ki);
				kptsFrom3DKept.resize(ki);
				kptsFrom3D = kptsFrom3DKept;

				std::vector<cv::Point3f> kptsTo3D;
				if(_estimationType == 0 || _estimationType == 1 || !_forwardEstimateOnly)
				{
					kptsTo3D = detector->generateKeypoints3D(toSignature.sensorData(), kptsTo);
				}

				UASSERT(kptsFrom.size() == kptsFrom3DKept.size());
				UASSERT(kptsFrom.size() == kptsTo.size());
				UASSERT(kptsTo3D.size() == 0 || kptsTo.size() == kptsTo3D.size());
				for(unsigned int i=0; i< kptsFrom3DKept.size(); ++i)
				{
					int id = orignalWordsFromIds.size()?orignalWordsFromIds[i]:i;
					wordsFrom.insert(std::make_pair(id, kptsFrom[i]));
					words3From.insert(std::make_pair(id, kptsFrom3DKept[i]));
					wordsTo.insert(std::make_pair(id, kptsTo[i]));
					if(kptsTo3D.size())
					{
						words3To.insert(std::make_pair(id, kptsTo3D[i]));
					}
				}
				toSignature.sensorData().setFeatures(kptsTo, kptsTo3D, cv::Mat());
			}
			else
			{
				if(imageFrom.empty())
				{
					UERROR("Optical flow correspondences requires images in data!");
				}
				UASSERT(kptsFrom.size() == kptsFrom3D.size());
				for(unsigned int i=0; i< kptsFrom3D.size(); ++i)
				{
					if(util3d::isFinite(kptsFrom3D[i]))
					{
						int id = orignalWordsFromIds.size()?orignalWordsFromIds[i]:i;
						wordsFrom.insert(std::make_pair(id, kptsFrom[i]));
						words3From.insert(std::make_pair(id, kptsFrom3D[i]));
					}
				}
				toSignature.sensorData().setFeatures(std::vector<cv::KeyPoint>(), std::vector<cv::Point3f>(), cv::Mat());
			}

			fromSignature.sensorData().setFeatures(kptsFrom, kptsFrom3D, cv::Mat());
		}
		else // Features Matching
		{
			UDEBUG("");
			std::vector<cv::KeyPoint> kptsTo;
			if(toSignature.getWords().empty())
			{
				if(toSignature.sensorData().keypoints().empty() &&
				   !imageTo.empty())
				{
					if(imageTo.channels() > 1)
					{
						cv::Mat tmp;
						cv::cvtColor(imageTo, tmp, cv::COLOR_BGR2GRAY);
						imageTo = tmp;
					}

					cv::Mat depthMask;
					if(!toSignature.sensorData().depthRaw().empty() && _depthAsMask)
					{
						if(imageTo.rows % toSignature.sensorData().depthRaw().rows == 0 &&
						   imageTo.cols % toSignature.sensorData().depthRaw().cols == 0 &&
						   imageTo.rows/toSignature.sensorData().depthRaw().rows == imageTo.cols/toSignature.sensorData().depthRaw().cols)
						{
							depthMask = util2d::interpolate(toSignature.sensorData().depthRaw(), imageTo.rows/toSignature.sensorData().depthRaw().rows, 0.1f);
						}
					}

					kptsTo = detector->generateKeypoints(
							imageTo,
							depthMask);
				}
				else
				{
					kptsTo = toSignature.sensorData().keypoints();
				}
			}
			else
			{
				kptsTo = uValues(toSignature.getWords());
			}

			// extract descriptors
			UDEBUG("kptsFrom=%d", (int)kptsFrom.size());
			UDEBUG("kptsTo=%d", (int)kptsTo.size());
			cv::Mat descriptorsFrom;
			if(fromSignature.getWordsDescriptors().size() &&
					((kptsFrom.empty() && fromSignature.getWordsDescriptors().size()) ||
					 fromSignature.getWordsDescriptors().size() == kptsFrom.size()))
			{
				descriptorsFrom = cv::Mat(fromSignature.getWordsDescriptors().size(),
						fromSignature.getWordsDescriptors().begin()->second.cols,
						fromSignature.getWordsDescriptors().begin()->second.type());
				int i=0;
				for(std::multimap<int, cv::Mat>::const_iterator iter=fromSignature.getWordsDescriptors().begin();
					iter!=fromSignature.getWordsDescriptors().end();
					++iter, ++i)
				{
					iter->second.copyTo(descriptorsFrom.row(i));
				}
			}
			else if(fromSignature.sensorData().descriptors().rows == (int)kptsFrom.size())
			{
				descriptorsFrom = fromSignature.sensorData().descriptors();
			}
			else if(!imageFrom.empty())
			{
				if(imageFrom.channels() > 1)
				{
					cv::Mat tmp;
					cv::cvtColor(imageFrom, tmp, cv::COLOR_BGR2GRAY);
					imageFrom = tmp;
				}
				orignalWordsFromIds.clear();
				descriptorsFrom = detector->generateDescriptors(imageFrom, kptsFrom);
			}

			cv::Mat descriptorsTo;
			if(kptsTo.size())
			{
				if(toSignature.getWordsDescriptors().size() == kptsTo.size())
				{
					descriptorsTo = cv::Mat(toSignature.getWordsDescriptors().size(),
							toSignature.getWordsDescriptors().begin()->second.cols,
							toSignature.getWordsDescriptors().begin()->second.type());
					int i=0;
					for(std::multimap<int, cv::Mat>::const_iterator iter=toSignature.getWordsDescriptors().begin();
						iter!=toSignature.getWordsDescriptors().end();
						++iter, ++i)
					{
						iter->second.copyTo(descriptorsTo.row(i));
					}
				}
				else if(toSignature.sensorData().descriptors().rows == (int)kptsTo.size())
				{
					descriptorsTo = toSignature.sensorData().descriptors();
				}
				else if(!imageTo.empty())
				{
					if(imageTo.channels() > 1)
					{
						cv::Mat tmp;
						cv::cvtColor(imageTo, tmp, cv::COLOR_BGR2GRAY);
						imageTo = tmp;
					}

					descriptorsTo = detector->generateDescriptors(imageTo, kptsTo);
				}
			}

			// create 3D keypoints
			std::vector<cv::Point3f> kptsFrom3D;
			std::vector<cv::Point3f> kptsTo3D;
			if(kptsFrom.size() == fromSignature.getWords3().size())
			{
				kptsFrom3D = uValues(fromSignature.getWords3());
			}
			else if(kptsFrom.size() == fromSignature.sensorData().keypoints3D().size())
			{
				kptsFrom3D = fromSignature.sensorData().keypoints3D();
			}
			else
			{
				if(fromSignature.getWords3().size() && kptsFrom.size() != fromSignature.getWords3().size())
				{
					UWARN("kptsFrom (%d) is not the same size as fromSignature.getWords3() (%d), there "
						   "is maybe a problem with the logic above (getWords3() should be null or equal to kptsfrom). Regenerating kptsFrom3D...",
						   kptsFrom.size(),
						   fromSignature.getWords3().size());
				}
				else if(fromSignature.sensorData().keypoints3D().size() && kptsFrom.size() != fromSignature.sensorData().keypoints3D().size())
				{
					UWARN("kptsFrom (%d) is not the same size as fromSignature.sensorData().keypoints3D() (%d), there "
						   "is maybe a problem with the logic above (keypoints3D should be null or equal to kptsfrom). Regenerating kptsFrom3D...",
						   kptsFrom.size(),
						   fromSignature.sensorData().keypoints3D().size());
				}
				kptsFrom3D = detector->generateKeypoints3D(fromSignature.sensorData(), kptsFrom);
				UDEBUG("generated kptsFrom3D=%d", (int)kptsFrom3D.size());
				if(detector->getMinDepth() > 0.0f || detector->getMaxDepth() > 0.0f)
				{
					//remove all keypoints/descriptors with no valid 3D points
					UASSERT((int)kptsFrom.size() == descriptorsFrom.rows &&
							kptsFrom3D.size() == kptsFrom.size());
					std::vector<cv::KeyPoint> validKeypoints(kptsFrom.size());
					std::vector<cv::Point3f> validKeypoints3D(kptsFrom.size());
					cv::Mat validDescriptors(descriptorsFrom.size(), descriptorsFrom.type());
					std::vector<int> validKeypointsIds;
					if(orignalWordsFromIds.size())
					{
						validKeypointsIds.resize(kptsFrom.size());
					}

					int oi=0;
					for(unsigned int i=0; i<kptsFrom3D.size(); ++i)
					{
						if(util3d::isFinite(kptsFrom3D[i]))
						{
							validKeypoints[oi] = kptsFrom[i];
							validKeypoints3D[oi] = kptsFrom3D[i];
							if(orignalWordsFromIds.size())
							{
								validKeypointsIds[oi] = orignalWordsFromIds[i];
							}
							descriptorsFrom.row(i).copyTo(validDescriptors.row(oi));
							++oi;
						}
					}
					UDEBUG("Removed %d invalid 3D points", (int)kptsFrom3D.size()-oi);
					validKeypoints.resize(oi);
					validKeypoints3D.resize(oi);
					kptsFrom = validKeypoints;
					kptsFrom3D = validKeypoints3D;
					
					if(orignalWordsFromIds.size())
					{
					validKeypointsIds.resize(oi);
						orignalWordsFromIds = validKeypointsIds;
					}
					descriptorsFrom = validDescriptors.rowRange(0, oi).clone();
				}
			}

			if(kptsTo.size() == toSignature.getWords3().size())
			{
				kptsTo3D = uValues(toSignature.getWords3());
			}
			else if(kptsTo.size() == toSignature.sensorData().keypoints3D().size())
			{
				kptsTo3D = toSignature.sensorData().keypoints3D();
			}
			else
			{
				if(toSignature.getWords3().size() && kptsTo.size() != toSignature.getWords3().size())
				{
					UWARN("kptsTo (%d) is not the same size as toSignature.getWords3() (%d), there "
						   "is maybe a problem with the logic above (getWords3() should be null or equal to kptsTo). Regenerating kptsTo3D...",
						   (int)kptsTo.size(),
						   (int)toSignature.getWords3().size());
				}
				else if(toSignature.sensorData().keypoints3D().size() && kptsTo.size() != toSignature.sensorData().keypoints3D().size())
				{
					UWARN("kptsTo (%d) is not the same size as toSignature.sensorData().keypoints3D() (%d), there "
						   "is maybe a problem with the logic above (keypoints3D() should be null or equal to kptsTo). Regenerating kptsTo3D...",
						   (int)kptsTo.size(),
						   (int)toSignature.sensorData().keypoints3D().size());
				}
				kptsTo3D = detector->generateKeypoints3D(toSignature.sensorData(), kptsTo);
				if(kptsTo3D.size() && (detector->getMinDepth() > 0.0f || detector->getMaxDepth() > 0.0f))
				{
					UDEBUG("");
					//remove all keypoints/descriptors with no valid 3D points
					UASSERT((int)kptsTo.size() == descriptorsTo.rows &&
							kptsTo3D.size() == kptsTo.size());
					std::vector<cv::KeyPoint> validKeypoints(kptsTo.size());
					std::vector<cv::Point3f> validKeypoints3D(kptsTo.size());
					cv::Mat validDescriptors(descriptorsTo.size(), descriptorsTo.type());

					int oi=0;
					for(unsigned int i=0; i<kptsTo3D.size(); ++i)
					{
						if(util3d::isFinite(kptsTo3D[i]))
						{
							validKeypoints[oi] = kptsTo[i];
							validKeypoints3D[oi] = kptsTo3D[i];
							descriptorsTo.row(i).copyTo(validDescriptors.row(oi));
							++oi;
						}
					}
					UDEBUG("Removed %d invalid 3D points", (int)kptsTo3D.size()-oi);
					validKeypoints.resize(oi);
					validKeypoints3D.resize(oi);
					kptsTo = validKeypoints;
					kptsTo3D = validKeypoints3D;
					descriptorsTo = validDescriptors.rowRange(0, oi).clone();
				}
			}

			UASSERT(kptsFrom.empty() || descriptorsFrom.rows == 0 || int(kptsFrom.size()) == descriptorsFrom.rows);

			fromSignature.sensorData().setFeatures(kptsFrom, kptsFrom3D, descriptorsFrom);
			toSignature.sensorData().setFeatures(kptsTo, kptsTo3D, descriptorsTo);

			UDEBUG("descriptorsFrom=%d", descriptorsFrom.rows);
			UDEBUG("descriptorsTo=%d", descriptorsTo.rows);

			// We have all data we need here, so match!
			if(descriptorsFrom.rows > 0 && descriptorsTo.rows > 0)
			{
				cv::Size imageSize = imageTo.size();
				bool isCalibrated = false; // multiple cameras not supported.
				if(imageSize.height == 0 || imageSize.width == 0)
				{
					imageSize = toSignature.sensorData().cameraModels().size() == 1?toSignature.sensorData().cameraModels()[0].imageSize():toSignature.sensorData().stereoCameraModel().left().imageSize();
				}

				isCalibrated = imageSize.height != 0 && imageSize.width != 0 &&
						(toSignature.sensorData().cameraModels().size()==1?toSignature.sensorData().cameraModels()[0].isValidForProjection():toSignature.sensorData().stereoCameraModel().isValidForProjection());

				// If guess is set, limit the search of matches using optical flow window size
				bool guessSet = !guess.isIdentity() && !guess.isNull();
				if(guessSet && _guessWinSize > 0 && kptsFrom3D.size() &&
						isCalibrated) // needed for projection
				{
					UDEBUG("");
					UASSERT((int)kptsTo.size() == descriptorsTo.rows);
					UASSERT((int)kptsFrom3D.size() == descriptorsFrom.rows);

					// Use guess to project 3D "from" keypoints into "to" image
					if(toSignature.sensorData().cameraModels().size() > 1)
					{
						UFATAL("Guess reprojection feature matching is not supported for multiple cameras.");
					}

					Transform localTransform = toSignature.sensorData().cameraModels().size()?toSignature.sensorData().cameraModels()[0].localTransform():toSignature.sensorData().stereoCameraModel().left().localTransform();
					Transform guessCameraRef = (guess * localTransform).inverse();
					cv::Mat R = (cv::Mat_<double>(3,3) <<
							(double)guessCameraRef.r11(), (double)guessCameraRef.r12(), (double)guessCameraRef.r13(),
							(double)guessCameraRef.r21(), (double)guessCameraRef.r22(), (double)guessCameraRef.r23(),
							(double)guessCameraRef.r31(), (double)guessCameraRef.r32(), (double)guessCameraRef.r33());
					cv::Mat rvec(1,3, CV_64FC1);
					cv::Rodrigues(R, rvec);
					cv::Mat tvec = (cv::Mat_<double>(1,3) << (double)guessCameraRef.x(), (double)guessCameraRef.y(), (double)guessCameraRef.z());
					cv::Mat K = toSignature.sensorData().cameraModels().size()?toSignature.sensorData().cameraModels()[0].K():toSignature.sensorData().stereoCameraModel().left().K();
					std::vector<cv::Point2f> projected;
					cv::projectPoints(kptsFrom3D, rvec, tvec, K, cv::Mat(), projected);
					UDEBUG("Projected points=%d", (int)projected.size());
					//remove projected points outside of the image
					UASSERT((int)projected.size() == descriptorsFrom.rows);
					std::vector<cv::Point2f> cornersProjected(projected.size());
					std::vector<int> projectedIndexToDescIndex(projected.size());
					int oi=0;
					for(unsigned int i=0; i<projected.size(); ++i)
					{
						if(uIsInBounds(projected[i].x, 0.0f, float(imageSize.width-1)) &&
						   uIsInBounds(projected[i].y, 0.0f, float(imageSize.height-1)) &&
						   util3d::transformPoint(kptsFrom3D[i], guessCameraRef).z > 0.0)
						{
							projectedIndexToDescIndex[oi] = i;
							cornersProjected[oi++] = projected[i];
						}
					}
					projectedIndexToDescIndex.resize(oi);
					cornersProjected.resize(oi);
					UDEBUG("corners in frame=%d", (int)cornersProjected.size());

					// For each projected feature guess of "from" in "to", find its matching feature in
					// the radius around the projected guess.
					// TODO: do cross-check?
					UDEBUG("guessMatchToProjection=%d, cornersProjected=%d", _guessMatchToProjection?1:0, (int)cornersProjected.size());
					if(cornersProjected.size())
					{
						if(_guessMatchToProjection)
						{
							// match frame to projected
							// Create kd-tree for projected keypoints
							rtflann::Matrix<float> cornersProjectedMat((float*)cornersProjected.data(), cornersProjected.size(), 2);
							rtflann::Index<rtflann::L2_Simple<float> > index(cornersProjectedMat, rtflann::KDTreeIndexParams());
							index.buildIndex();

							std::vector< std::vector<size_t> > indices;
							std::vector<std::vector<float> > dists;
							float radius = (float)_guessWinSize; // pixels
							std::vector<cv::Point2f> pointsTo;
							cv::KeyPoint::convert(kptsTo, pointsTo);
							rtflann::Matrix<float> pointsToMat((float*)pointsTo.data(), pointsTo.size(), 2);
							index.radiusSearch(pointsToMat, indices, dists, radius*radius, rtflann::SearchParams());

							UASSERT(indices.size() == pointsToMat.rows);
							UASSERT(descriptorsFrom.cols == descriptorsTo.cols);
							UASSERT(descriptorsFrom.rows == (int)kptsFrom.size());
							UASSERT((int)pointsToMat.rows == descriptorsTo.rows);
							UASSERT(pointsToMat.rows == kptsTo.size());
							UDEBUG("radius search done for guess");

							// Process results (Nearest Neighbor Distance Ratio)
							int newToId = orignalWordsFromIds.size()?orignalWordsFromIds.back():descriptorsFrom.rows;
							std::map<int,int> addedWordsFrom; //<id, index>
							std::map<int, int> duplicates; //<fromId, toId>
							int newWords = 0;
							cv::Mat descriptors(10, descriptorsTo.cols, descriptorsTo.type());
							for(unsigned int i = 0; i < pointsToMat.rows; ++i)
							{
								// Make octave compatible with SIFT packed octave (https://github.com/opencv/opencv/issues/4554)
								int octave = kptsTo[i].octave & 255;
								octave = octave < 128 ? octave : (-128 | octave);
								int matchedIndex = -1;
								if(indices[i].size() >= 2)
								{
									std::vector<int> descriptorsIndices(indices[i].size());
									int oi=0;
									if((int)indices[i].size() > descriptors.rows)
									{
										descriptors.resize(indices[i].size());
									}
									for(unsigned int j=0; j<indices[i].size(); ++j)
									{
										int octaveFrom = kptsFrom.at(projectedIndexToDescIndex[indices[i].at(j)]).octave & 255;
										octaveFrom = octaveFrom < 128 ? octaveFrom : (-128 | octaveFrom);
										if(octaveFrom==octave)
										{
											descriptorsFrom.row(projectedIndexToDescIndex[indices[i].at(j)]).copyTo(descriptors.row(oi));
											descriptorsIndices[oi++] = indices[i].at(j);
										}
									}
									descriptorsIndices.resize(oi);
									if(oi >=2)
									{
										std::vector<std::vector<cv::DMatch> > matches;
										cv::BFMatcher matcher(descriptors.type()==CV_8U?cv::NORM_HAMMING:cv::NORM_L2SQR);
										matcher.knnMatch(descriptorsTo.row(i), cv::Mat(descriptors, cv::Range(0, oi)), matches, 2);
										UASSERT(matches.size() == 1);
										UASSERT(matches[0].size() == 2);
										if(matches[0].at(0).distance < _nndr * matches[0].at(1).distance)
										{
											matchedIndex = descriptorsIndices.at(matches[0].at(0).trainIdx);
										}
									}
									else if(oi == 1)
									{
										matchedIndex = descriptorsIndices[0];
									}
								}
								else if(indices[i].size() == 1)
								{
									int octaveFrom = kptsFrom.at(projectedIndexToDescIndex[indices[i].at(0)]).octave & 255;
									octaveFrom = octaveFrom < 128 ? octaveFrom : (-128 | octaveFrom);
									if(octaveFrom == octave)
									{
										matchedIndex = indices[i].at(0);
									}
								}

								if(matchedIndex >= 0)
								{
									matchedIndex = projectedIndexToDescIndex[matchedIndex];
									int id = orignalWordsFromIds.size()?orignalWordsFromIds[matchedIndex]:matchedIndex;

									if(addedWordsFrom.find(matchedIndex) != addedWordsFrom.end())
									{
										id = addedWordsFrom.at(matchedIndex);
										duplicates.insert(std::make_pair(matchedIndex, id));
									}
									else
									{
										addedWordsFrom.insert(std::make_pair(matchedIndex, id));

										if(kptsFrom.size())
										{
											wordsFrom.insert(std::make_pair(id, kptsFrom[matchedIndex]));
										}
										words3From.insert(std::make_pair(id, kptsFrom3D[matchedIndex]));
										wordsDescFrom.insert(std::make_pair(id, descriptorsFrom.row(matchedIndex)));
									}

									wordsTo.insert(std::make_pair(id, kptsTo[i]));
									wordsDescTo.insert(std::make_pair(id, descriptorsTo.row(i)));
									if(kptsTo3D.size())
									{
										words3To.insert(std::make_pair(id, kptsTo3D[i]));
									}
								}
								else
								{
									// gen fake ids
									wordsTo.insert(wordsTo.end(), std::make_pair(newToId, kptsTo[i]));
									wordsDescTo.insert(wordsDescTo.end(), std::make_pair(newToId, descriptorsTo.row(i)));
									if(kptsTo3D.size())
									{
										words3To.insert(words3To.end(), std::make_pair(newToId, kptsTo3D[i]));
									}

									++newToId;
									++newWords;
								}
							}
							UDEBUG("addedWordsFrom=%d/%d (duplicates=%d, newWords=%d), kptsTo=%d, wordsTo=%d, words3From=%d",
								(int)addedWordsFrom.size(), (int)cornersProjected.size(), (int)duplicates.size(), newWords,
								(int)kptsTo.size(), (int)wordsTo.size(), (int)words3From.size());

							// create fake ids for not matched words from "from"
							int addWordsFromNotMatched = 0;
							for(unsigned int i=0; i<kptsFrom3D.size(); ++i)
							{
								if(util3d::isFinite(kptsFrom3D[i]) && addedWordsFrom.find(i) == addedWordsFrom.end())
								{
									int id = orignalWordsFromIds.size()?orignalWordsFromIds[i]:i;
									wordsFrom.insert(wordsFrom.end(), std::make_pair(id, kptsFrom[i]));
									wordsDescFrom.insert(wordsDescFrom.end(), std::make_pair(id, descriptorsFrom.row(i)));
									words3From.insert(words3From.end(), std::make_pair(id, kptsFrom3D[i]));

									++addWordsFromNotMatched;
								}
							}
							UDEBUG("addWordsFromNotMatched=%d -> words3From=%d", addWordsFromNotMatched, (int)words3From.size());
						}
						else
						{
							// match projected to frame
							std::vector<cv::Point2f> pointsTo;
							cv::KeyPoint::convert(kptsTo, pointsTo);
							rtflann::Matrix<float> pointsToMat((float*)pointsTo.data(), pointsTo.size(), 2);
							rtflann::Index<rtflann::L2_Simple<float> > index(pointsToMat, rtflann::KDTreeIndexParams());
							index.buildIndex();

							std::vector< std::vector<size_t> > indices;
							std::vector<std::vector<float> > dists;
							float radius = (float)_guessWinSize; // pixels
							rtflann::Matrix<float> cornersProjectedMat((float*)cornersProjected.data(), cornersProjected.size(), 2);
							index.radiusSearch(cornersProjectedMat, indices, dists, radius*radius, rtflann::SearchParams());

							UASSERT(indices.size() == cornersProjectedMat.rows);
							UASSERT(descriptorsFrom.cols == descriptorsTo.cols);
							UASSERT(descriptorsFrom.rows == (int)kptsFrom.size());
							UASSERT((int)pointsToMat.rows == descriptorsTo.rows);
							UASSERT(pointsToMat.rows == kptsTo.size());
							UDEBUG("radius search done for guess");

							// Process results (Nearest Neighbor Distance Ratio)
							std::set<int> addedWordsTo;
							std::set<int> addedWordsFrom;
							std::set<int> indicesToIgnore;
							double bruteForceTotalTime = 0.0;
							double bruteForceDescCopy = 0.0;
							UTimer bruteForceTimer;
							cv::Mat descriptors(10, descriptorsTo.cols, descriptorsTo.type());
							for(unsigned int i = 0; i < cornersProjectedMat.rows; ++i)
							{
								int matchedIndexFrom = projectedIndexToDescIndex[i];

								if(indices[i].size())
								{
									info.projectedIDs.push_back(orignalWordsFromIds.size()?orignalWordsFromIds[matchedIndexFrom]:matchedIndexFrom);
								}

								if(util3d::isFinite(kptsFrom3D[matchedIndexFrom]))
								{
									// Make octave compatible with SIFT packed octave (https://github.com/opencv/opencv/issues/4554)
									int octaveFrom = kptsFrom.at(matchedIndexFrom).octave & 255;
									octaveFrom = octaveFrom < 128 ? octaveFrom : (-128 | octaveFrom);

									int matchedIndexTo = -1;
									if(indices[i].size() >= 2)
									{
										bruteForceTimer.restart();
										std::vector<int> descriptorsIndices(indices[i].size());
										int oi=0;
										if((int)indices[i].size() > descriptors.rows)
										{
											descriptors.resize(indices[i].size());
										}
										std::list<int> indicesToIgnoretmp;
										for(unsigned int j=0; j<indices[i].size(); ++j)
										{
											int octave = kptsTo[indices[i].at(j)].octave & 255;
											octave = octave < 128 ? octave : (-128 | octave);
											if(octaveFrom==octave)
											{
												descriptorsTo.row(indices[i].at(j)).copyTo(descriptors.row(oi));
												descriptorsIndices[oi++] = indices[i].at(j);

												indicesToIgnoretmp.push_back(indices[i].at(j));
											}
										}
										bruteForceDescCopy += bruteForceTimer.ticks();
										if(oi >=2)
										{
											std::vector<std::vector<cv::DMatch> > matches;
											cv::BFMatcher matcher(descriptors.type()==CV_8U?cv::NORM_HAMMING:cv::NORM_L2SQR);
											matcher.knnMatch(descriptorsFrom.row(matchedIndexFrom), cv::Mat(descriptors, cv::Range(0, oi)), matches, 2);
											UASSERT(matches.size() == 1);
											UASSERT(matches[0].size() == 2);
											bruteForceTotalTime+=bruteForceTimer.elapsed();
											if(matches[0].at(0).distance < _nndr * matches[0].at(1).distance)
											{
												matchedIndexTo = descriptorsIndices.at(matches[0].at(0).trainIdx);

												indicesToIgnore.insert(indicesToIgnore.begin(), indicesToIgnore.end());
											}
										}
										else if(oi == 1)
										{
											matchedIndexTo = descriptorsIndices[0];
										}
									}
									else if(indices[i].size() == 1)
									{
										int octave = kptsTo[indices[i].at(0)].octave & 255;
										octave = octave < 128 ? octave : (-128 | octave);
										if(octaveFrom == octave)
										{
											matchedIndexTo = indices[i].at(0);
										}
									}

									int id = orignalWordsFromIds.size()?orignalWordsFromIds[matchedIndexFrom]:matchedIndexFrom;
									addedWordsFrom.insert(addedWordsFrom.end(), matchedIndexFrom);

									if(kptsFrom.size())
									{
										wordsFrom.insert(wordsFrom.end(), std::make_pair(id, kptsFrom[matchedIndexFrom]));
									}
									words3From.insert(words3From.end(), std::make_pair(id, kptsFrom3D[matchedIndexFrom]));
									wordsDescFrom.insert(wordsDescFrom.end(), std::make_pair(id, descriptorsFrom.row(matchedIndexFrom)));

									if(	matchedIndexTo >= 0 &&
										addedWordsTo.find(matchedIndexTo) == addedWordsTo.end())
									{
										addedWordsTo.insert(matchedIndexTo);

										wordsTo.insert(wordsTo.end(), std::make_pair(id, kptsTo[matchedIndexTo]));
										wordsDescTo.insert(wordsDescTo.end(), std::make_pair(id, descriptorsTo.row(matchedIndexTo)));
										if(kptsTo3D.size())
										{
											words3To.insert(words3To.end(), std::make_pair(id, kptsTo3D[matchedIndexTo]));
										}
									}
								}
							}
							UDEBUG("bruteForceDescCopy=%fs, bruteForceTotalTime=%fs", bruteForceDescCopy, bruteForceTotalTime);

							// create fake ids for not matched words from "from"
							for(unsigned int i=0; i<kptsFrom3D.size(); ++i)
							{
								if(util3d::isFinite(kptsFrom3D[i]) && addedWordsFrom.find(i) == addedWordsFrom.end())
								{
									int id = orignalWordsFromIds.size()?orignalWordsFromIds[i]:i;
									wordsFrom.insert(wordsFrom.end(), std::make_pair(id, kptsFrom[i]));
									wordsDescFrom.insert(wordsDescFrom.end(), std::make_pair(id, descriptorsFrom.row(i)));
									words3From.insert(words3From.end(), std::make_pair(id, kptsFrom3D[i]));
								}
							}

							int newToId = orignalWordsFromIds.size()?orignalWordsFromIds.back():descriptorsFrom.rows;
							for(unsigned int i = 0; i < kptsTo.size(); ++i)
							{
								if(addedWordsTo.find(i) == addedWordsTo.end() && indicesToIgnore.find(i) == indicesToIgnore.end())
								{
									wordsTo.insert(wordsTo.end(), std::make_pair(newToId, kptsTo[i]));
									wordsDescTo.insert(wordsDescTo.end(), std::make_pair(newToId, descriptorsTo.row(i)));
									if(kptsTo3D.size())
									{
										words3To.insert(words3To.end(), std::make_pair(newToId, kptsTo3D[i]));
									}
									++newToId;
								}
							}
						}
					}
					else
					{
						UWARN("All projected points are outside the camera. Guess (%s) is wrong or images are not overlapping.", guess.prettyPrint().c_str());
					}
					UDEBUG("");
				}
				else
				{
					if(guessSet && _guessWinSize > 0 && kptsFrom3D.size() && !isCalibrated)
					{
						if(fromSignature.sensorData().cameraModels().size() > 1 || toSignature.sensorData().cameraModels().size() > 1)
						{
							UWARN("Finding correspondences with the guess cannot "
								  "be done with multiple cameras, global matching is "
								   "done instead. Please set \"%s\" to 0 to avoid this warning.",
								   Parameters::kVisCorGuessWinSize().c_str());
						}
						else
						{
							UWARN("Calibration not found! Finding correspondences "
								   "with the guess cannot be done, global matching is "
								   "done instead.");
						}
					}

					UDEBUG("");
					// match between all descriptors
					VWDictionary dictionary(_featureParameters);
					std::list<int> fromWordIds;
					for (int i = 0; i < descriptorsFrom.rows; ++i)
					{
						int id = orignalWordsFromIds.size() ? orignalWordsFromIds[i] : i;
						dictionary.addWord(new VisualWord(id, descriptorsFrom.row(i), 1));
						fromWordIds.push_back(id);
					}

					std::list<int> toWordIds;
					if(descriptorsTo.rows)
					{
						dictionary.update();
						toWordIds = dictionary.addNewWords(descriptorsTo, 2);
					}
					dictionary.clear(false);

					std::multiset<int> fromWordIdsSet(fromWordIds.begin(), fromWordIds.end());
					std::multiset<int> toWordIdsSet(toWordIds.begin(), toWordIds.end());

					UASSERT(kptsFrom3D.empty() || fromWordIds.size() == kptsFrom3D.size());
					UASSERT(int(fromWordIds.size()) == descriptorsFrom.rows);
					int i=0;
					for(std::list<int>::iterator iter=fromWordIds.begin(); iter!=fromWordIds.end(); ++iter)
					{
						if(fromWordIdsSet.count(*iter) == 1)
						{
							if (kptsFrom.size())
							{
								wordsFrom.insert(std::make_pair(*iter, kptsFrom[i]));
							}
							if(kptsFrom3D.size())
							{
								words3From.insert(std::make_pair(*iter, kptsFrom3D[i]));
							}
							wordsDescFrom.insert(std::make_pair(*iter, descriptorsFrom.row(i)));
						}
						++i;
					}
					UASSERT(kptsTo3D.size() == 0 || kptsTo3D.size() == kptsTo.size());
					UASSERT(toWordIds.size() == kptsTo.size());
					UASSERT(int(toWordIds.size()) == descriptorsTo.rows);
					i=0;
					for(std::list<int>::iterator iter=toWordIds.begin(); iter!=toWordIds.end(); ++iter)
					{
						if(toWordIdsSet.count(*iter) == 1)
						{
							wordsTo.insert(std::make_pair(*iter, kptsTo[i]));
							wordsDescTo.insert(std::make_pair(*iter, descriptorsTo.row(i)));
							if(kptsTo3D.size())
							{
								words3To.insert(std::make_pair(*iter, kptsTo3D[i]));
							}
						}
						++i;
					}
				}
			}
			else if(descriptorsFrom.rows)
			{
				//just create fake words
				UASSERT(kptsFrom3D.empty() || int(kptsFrom3D.size()) == descriptorsFrom.rows);
				for(int i=0; i<descriptorsFrom.rows; ++i)
				{
					wordsFrom.insert(std::make_pair(i, kptsFrom[i]));
					wordsDescFrom.insert(std::make_pair(i, descriptorsFrom.row(i)));
					if(kptsFrom3D.size())
					{
						words3From.insert(std::make_pair(i, kptsFrom3D[i]));
					}
				}
			}
		}
		fromSignature.setWords(wordsFrom);
		fromSignature.setWords3(words3From);
		fromSignature.setWordsDescriptors(wordsDescFrom);
		toSignature.setWords(wordsTo);
		toSignature.setWords3(words3To);
		toSignature.setWordsDescriptors(wordsDescTo);
		delete detector;
	}

	/////////////////////
	// Motion estimation
	/////////////////////
	Transform transform;
	cv::Mat covariance = cv::Mat::eye(6,6,CV_64FC1);
	int inliersCount = 0;
	int matchesCount = 0;
	info.inliersIDs.clear();
	info.matchesIDs.clear();
	if(toSignature.getWords().size())
	{
		Transform transforms[2];
		std::vector<int> inliers[2];
		std::vector<int> matches[2];
		cv::Mat covariances[2];
		covariances[0] = cv::Mat::eye(6,6,CV_64FC1);
		covariances[1] = cv::Mat::eye(6,6,CV_64FC1);
		for(int dir=0; dir<(!_forwardEstimateOnly?2:1); ++dir)
		{
			// A to B
			Signature * signatureA;
			Signature * signatureB;
			if(dir == 0)
			{
				signatureA = &fromSignature;
				signatureB = &toSignature;
			}
			else
			{
				signatureA = &toSignature;
				signatureB = &fromSignature;
			}
			if(_estimationType == 2) // Epipolar Geometry
			{
				UDEBUG("");
				if(!signatureB->sensorData().stereoCameraModel().isValidForProjection() &&
				   (signatureB->sensorData().cameraModels().size() != 1 ||
					!signatureB->sensorData().cameraModels()[0].isValidForProjection()))
				{
					UERROR("Calibrated camera required (multi-cameras not supported).");
				}
				else if((int)signatureA->getWords().size() >= _minInliers &&
						(int)signatureB->getWords().size() >= _minInliers)
				{
					UASSERT(signatureA->sensorData().stereoCameraModel().isValidForProjection() || (signatureA->sensorData().cameraModels().size() == 1 && signatureA->sensorData().cameraModels()[0].isValidForProjection()));
					const CameraModel & cameraModel = signatureA->sensorData().stereoCameraModel().isValidForProjection()?signatureA->sensorData().stereoCameraModel().left():signatureA->sensorData().cameraModels()[0];

					// we only need the camera transform, send guess words3 for scale estimation
					Transform cameraTransform;
					double variance = 1.0f;
					std::map<int, cv::Point3f> inliers3D = util3d::generateWords3DMono(
							uMultimapToMapUnique(signatureA->getWords()),
							uMultimapToMapUnique(signatureB->getWords()),
							cameraModel,
							cameraTransform,
							_iterations,
							_PnPReprojError,
							_PnPFlags, // cv::SOLVEPNP_ITERATIVE
							_PnPRefineIterations,
							1.0f,
							0.99f,
							uMultimapToMapUnique(signatureA->getWords3()), // for scale estimation
							&variance);
					covariances[dir] *= variance;
					inliers[dir] = uKeys(inliers3D);

					if(!cameraTransform.isNull())
					{
						if((int)inliers3D.size() >= _minInliers)
						{
							if(variance <= _epipolarGeometryVar)
							{
								if(this->force3DoF())
								{
									transforms[dir] = cameraTransform.to3DoF();
								}
								else
								{
									transforms[dir] = cameraTransform;
								}
							}
							else
							{
								msg = uFormat("Variance is too high! (max inlier distance=%f, variance=%f)", _epipolarGeometryVar, variance);
								UINFO(msg.c_str());
							}
						}
						else
						{
							msg = uFormat("Not enough inliers %d < %d", (int)inliers3D.size(), _minInliers);
							UINFO(msg.c_str());
						}
					}
					else
					{
						msg = uFormat("No camera transform found");
						UINFO(msg.c_str());
					}
				}
				else if(signatureA->getWords().size() == 0)
				{
					msg = uFormat("No enough features (%d)", (int)signatureA->getWords().size());
					UWARN(msg.c_str());
				}
				else
				{
					msg = uFormat("No camera model");
					UWARN(msg.c_str());
				}
			}
			else if(_estimationType == 1) // PnP
			{
				UDEBUG("");
				if(!signatureB->sensorData().stereoCameraModel().isValidForProjection() &&
				   (signatureB->sensorData().cameraModels().size() != 1 ||
					!signatureB->sensorData().cameraModels()[0].isValidForProjection()))
				{
					UERROR("Calibrated camera required (multi-cameras not supported). Id=%d Models=%d StereoModel=%d weight=%d",
							signatureB->id(),
							(int)signatureB->sensorData().cameraModels().size(),
							signatureB->sensorData().stereoCameraModel().isValidForProjection()?1:0,
							signatureB->getWeight());
				}
				else
				{
					UDEBUG("words from3D=%d to2D=%d", (int)signatureA->getWords3().size(), (int)signatureB->getWords().size());
					// 3D to 2D
					if((int)signatureA->getWords3().size() >= _minInliers &&
					   (int)signatureB->getWords().size() >= _minInliers)
					{
						UASSERT(signatureB->sensorData().stereoCameraModel().isValidForProjection() || (signatureB->sensorData().cameraModels().size() == 1 && signatureB->sensorData().cameraModels()[0].isValidForProjection()));
						const CameraModel & cameraModel = signatureB->sensorData().stereoCameraModel().isValidForProjection()?signatureB->sensorData().stereoCameraModel().left():signatureB->sensorData().cameraModels()[0];

						std::vector<int> inliersV;
						std::vector<int> matchesV;
						transforms[dir] = util3d::estimateMotion3DTo2D(
								uMultimapToMapUnique(signatureA->getWords3()),
								uMultimapToMapUnique(signatureB->getWords()),
								cameraModel,
								_minInliers,
								_iterations,
								_PnPReprojError,
								_PnPFlags,
								_PnPRefineIterations,
								dir==0?(!guess.isNull()?guess:Transform::getIdentity()):!transforms[0].isNull()?transforms[0].inverse():(!guess.isNull()?guess.inverse():Transform::getIdentity()),
								uMultimapToMapUnique(signatureB->getWords3()),
								&covariances[dir],
								&matchesV,
								&inliersV);
						inliers[dir] = inliersV;
						matches[dir] = matchesV;
						UDEBUG("inliers: %d/%d", (int)inliersV.size(), (int)matchesV.size());
						if(transforms[dir].isNull())
						{
							msg = uFormat("Not enough inliers %d/%d (matches=%d) between %d and %d",
									(int)inliers[dir].size(), _minInliers, (int)matches[dir].size(), signatureA->id(), signatureB->id());
							UINFO(msg.c_str());
						}
						else if(this->force3DoF())
						{
							transforms[dir] = transforms[dir].to3DoF();
						}
					}
					else
					{
						msg = uFormat("Not enough features in images (old=%d, new=%d, min=%d)",
								(int)signatureA->getWords3().size(), (int)signatureB->getWords().size(), _minInliers);
						UINFO(msg.c_str());
					}
				}

			}
			else
			{
				UDEBUG("");
				// 3D -> 3D
				if((int)signatureA->getWords3().size() >= _minInliers &&
				   (int)signatureB->getWords3().size() >= _minInliers)
				{
					std::vector<int> inliersV;
					std::vector<int> matchesV;
					transforms[dir] = util3d::estimateMotion3DTo3D(
							uMultimapToMapUnique(signatureA->getWords3()),
							uMultimapToMapUnique(signatureB->getWords3()),
							_minInliers,
							_inlierDistance,
							_iterations,
							_refineIterations,
							&covariances[dir],
							&matchesV,
							&inliersV);
					inliers[dir] = inliersV;
					matches[dir] = matchesV;
					UDEBUG("inliers: %d/%d", (int)inliersV.size(), (int)matchesV.size());
					if(transforms[dir].isNull())
					{
						msg = uFormat("Not enough inliers %d/%d (matches=%d) between %d and %d",
								(int)inliers[dir].size(), _minInliers, (int)matches[dir].size(), signatureA->id(), signatureB->id());
						UINFO(msg.c_str());
					}
					else if(this->force3DoF())
					{
						transforms[dir] = transforms[dir].to3DoF();
					}
				}
				else
				{
					msg = uFormat("Not enough 3D features in images (old=%d, new=%d, min=%d)",
							(int)signatureA->getWords3().size(), (int)signatureB->getWords3().size(), _minInliers);
					UINFO(msg.c_str());
				}
			}
		}

		if(!_forwardEstimateOnly)
		{
			UDEBUG("from->to=%s", transforms[0].prettyPrint().c_str());
			UDEBUG("to->from=%s", transforms[1].prettyPrint().c_str());
		}

		std::vector<int> allInliers = inliers[0];
		if(inliers[1].size())
		{
			std::set<int> allInliersSet(allInliers.begin(), allInliers.end());
			unsigned int oi = allInliers.size();
			allInliers.resize(allInliers.size() + inliers[1].size());
			for(unsigned int i=0; i<inliers[1].size(); ++i)
			{
				if(allInliersSet.find(inliers[1][i]) == allInliersSet.end())
				{
					allInliers[oi++] = inliers[1][i];
				}
			}
			allInliers.resize(oi);
		}
		std::vector<int> allMatches = matches[0];
		if(matches[1].size())
		{
			std::set<int> allMatchesSet(allMatches.begin(), allMatches.end());
			unsigned int oi = allMatches.size();
			allMatches.resize(allMatches.size() + matches[1].size());
			for(unsigned int i=0; i<matches[1].size(); ++i)
			{
				if(allMatchesSet.find(matches[1][i]) == allMatchesSet.end())
				{
					allMatches[oi++] = matches[1][i];
				}
			}
			allMatches.resize(oi);
		}

		if(_bundleAdjustment > 0 &&
			_estimationType < 2 &&
			!transforms[0].isNull() &&
			allInliers.size() &&
			fromSignature.getWords3().size() &&
			toSignature.getWords().size() &&
			fromSignature.sensorData().cameraModels().size() <= 1 &&
			toSignature.sensorData().cameraModels().size() <= 1)
		{
			UDEBUG("Refine with bundle adjustment");
			Optimizer * sba = Optimizer::create(_bundleAdjustment==2?Optimizer::kTypeCVSBA:Optimizer::kTypeG2O, _bundleParameters);

			std::map<int, Transform> poses;
			std::multimap<int, Link> links;
			std::map<int, cv::Point3f> points3DMap;

			poses.insert(std::make_pair(1, Transform::getIdentity()));
			poses.insert(std::make_pair(2, transforms[0]));

			for(int i=0;i<2;++i)
			{
				UASSERT(covariances[i].cols==6 && covariances[i].rows == 6 && covariances[i].type() == CV_64FC1);
				if(covariances[i].at<double>(0,0)<=COVARIANCE_EPSILON)
					covariances[i].at<double>(0,0) = COVARIANCE_EPSILON; // epsilon if exact transform
				if(covariances[i].at<double>(1,1)<=COVARIANCE_EPSILON)
					covariances[i].at<double>(1,1) = COVARIANCE_EPSILON; // epsilon if exact transform
				if(covariances[i].at<double>(2,2)<=COVARIANCE_EPSILON)
					covariances[i].at<double>(2,2) = COVARIANCE_EPSILON; // epsilon if exact transform
				if(covariances[i].at<double>(3,3)<=COVARIANCE_EPSILON)
					covariances[i].at<double>(3,3) = COVARIANCE_EPSILON; // epsilon if exact transform
				if(covariances[i].at<double>(4,4)<=COVARIANCE_EPSILON)
					covariances[i].at<double>(4,4) = COVARIANCE_EPSILON; // epsilon if exact transform
				if(covariances[i].at<double>(5,5)<=COVARIANCE_EPSILON)
					covariances[i].at<double>(5,5) = COVARIANCE_EPSILON; // epsilon if exact transform
			}

			cv::Mat cov = covariances[0].clone();

			links.insert(std::make_pair(1, Link(1, 2, Link::kNeighbor, transforms[0], cov.inv())));
			if(!transforms[1].isNull() && inliers[1].size())
			{
				cov = covariances[1].clone();
				links.insert(std::make_pair(2, Link(2, 1, Link::kNeighbor, transforms[1], cov.inv())));
			}

			std::map<int, Transform> optimizedPoses;

			UASSERT(toSignature.sensorData().stereoCameraModel().isValidForProjection() ||
					(toSignature.sensorData().cameraModels().size() == 1 && toSignature.sensorData().cameraModels()[0].isValidForProjection()));

			std::map<int, CameraModel> models;

			Transform invLocalTransformFrom;
			CameraModel cameraModelFrom;
			if(fromSignature.sensorData().stereoCameraModel().isValidForProjection())
			{
				cameraModelFrom = fromSignature.sensorData().stereoCameraModel().left();
				// Set Tx=-baseline*fx for Stereo BA
				cameraModelFrom = CameraModel(cameraModelFrom.fx(),
						cameraModelFrom.fy(),
						cameraModelFrom.cx(),
						cameraModelFrom.cy(),
						cameraModelFrom.localTransform(),
						-fromSignature.sensorData().stereoCameraModel().baseline()*cameraModelFrom.fy());
				invLocalTransformFrom = toSignature.sensorData().stereoCameraModel().localTransform().inverse();
			}
			else if(fromSignature.sensorData().cameraModels().size() == 1)
			{
				cameraModelFrom = fromSignature.sensorData().cameraModels()[0];
				invLocalTransformFrom = toSignature.sensorData().cameraModels()[0].localTransform().inverse();
			}

			Transform invLocalTransformTo = Transform::getIdentity();
			CameraModel cameraModelTo;
			if(toSignature.sensorData().stereoCameraModel().isValidForProjection())
			{
				cameraModelTo = toSignature.sensorData().stereoCameraModel().left();
				// Set Tx=-baseline*fx for Stereo BA
				cameraModelTo = CameraModel(cameraModelTo.fx(),
						cameraModelTo.fy(),
						cameraModelTo.cx(),
						cameraModelTo.cy(),
						cameraModelTo.localTransform(),
						-toSignature.sensorData().stereoCameraModel().baseline()*cameraModelTo.fy());
				invLocalTransformTo = toSignature.sensorData().stereoCameraModel().localTransform().inverse();
			}
			else if(toSignature.sensorData().cameraModels().size() == 1)
			{
				cameraModelTo = toSignature.sensorData().cameraModels()[0];
				invLocalTransformTo = toSignature.sensorData().cameraModels()[0].localTransform().inverse();
			}
			if(invLocalTransformFrom.isNull())
			{
				invLocalTransformFrom = invLocalTransformTo;
			}

			models.insert(std::make_pair(1, cameraModelFrom.isValidForProjection()?cameraModelFrom:cameraModelTo));
			models.insert(std::make_pair(2, cameraModelTo));

			std::map<int, std::map<int, FeatureBA> > wordReferences;
			for(unsigned int i=0; i<allInliers.size(); ++i)
			{
				int wordId = allInliers[i];
				const cv::Point3f & pt3D = fromSignature.getWords3().find(wordId)->second;
				points3DMap.insert(std::make_pair(wordId, pt3D));

				std::map<int, FeatureBA> ptMap;
				if(fromSignature.getWords().size() && cameraModelFrom.isValidForProjection())
				{
					float depthFrom = util3d::transformPoint(pt3D, invLocalTransformFrom).z;
					const cv::KeyPoint & kpt = fromSignature.getWords().find(wordId)->second;
					ptMap.insert(std::make_pair(1,FeatureBA(kpt, depthFrom)));
				}
				if(toSignature.getWords().size() && cameraModelTo.isValidForProjection())
				{
					float depthTo = 0.0f;
					if(toSignature.getWords3().find(wordId) != toSignature.getWords3().end())
					{
						depthTo = util3d::transformPoint(toSignature.getWords3().find(wordId)->second, invLocalTransformTo).z;
					}
					const cv::KeyPoint & kpt = toSignature.getWords().find(wordId)->second;
					ptMap.insert(std::make_pair(2,FeatureBA(kpt, depthTo)));
				}

				wordReferences.insert(std::make_pair(wordId, ptMap));

				//UDEBUG("%d (%f,%f,%f)", wordId, points3DMap.at(wordId).x, points3DMap.at(wordId).y, points3DMap.at(wordId).z);
				//for(std::map<int, cv::Point3f>::iterator iter=ptMap.begin(); iter!=ptMap.end(); ++iter)
				//{
				//	UDEBUG("%d (%f,%f) d=%f", iter->first, iter->second.x, iter->second.y, iter->second.z);
				//}
			}

			std::set<int> sbaOutliers;
			optimizedPoses = sba->optimizeBA(1, poses, links, models, points3DMap, wordReferences, &sbaOutliers);
			delete sba;

			//update transform
			if(optimizedPoses.size() == 2 &&
				!optimizedPoses.begin()->second.isNull() &&
				!optimizedPoses.rbegin()->second.isNull())
			{
				UDEBUG("Pose optimization: %s -> %s", transforms[0].prettyPrint().c_str(), optimizedPoses.rbegin()->second.prettyPrint().c_str());

				if(sbaOutliers.size())
				{
					std::vector<int> newInliers(allInliers.size());
					int oi=0;
					for(unsigned int i=0; i<allInliers.size(); ++i)
					{
						if(sbaOutliers.find(allInliers[i]) == sbaOutliers.end())
						{
							newInliers[oi++] = allInliers[i];
						}
					}
					newInliers.resize(oi);
					UDEBUG("BA outliers ratio %f", float(sbaOutliers.size())/float(allInliers.size()));
					allInliers = newInliers;
				}
				if((int)allInliers.size() < _minInliers)
				{
					msg = uFormat("Not enough inliers after bundle adjustment %d/%d (matches=%d) between %d and %d",
							(int)allInliers.size(), _minInliers, (int)allInliers.size()+sbaOutliers.size(), fromSignature.id(), toSignature.id());
					transforms[0].setNull();
				}
				else
				{
					transforms[0] = optimizedPoses.rbegin()->second;
				}
				// update 3D points, both from and to signatures
				/*std::multimap<int, cv::Point3f> cpyWordsFrom3 = fromSignature.getWords3();
				std::multimap<int, cv::Point3f> cpyWordsTo3 = toSignature.getWords3();
				Transform invT = transforms[0].inverse();
				for(std::map<int, cv::Point3f>::iterator iter=points3DMap.begin(); iter!=points3DMap.end(); ++iter)
				{
					cpyWordsFrom3.find(iter->first)->second = iter->second;
					if(cpyWordsTo3.find(iter->first) != cpyWordsTo3.end())
					{
						cpyWordsTo3.find(iter->first)->second = util3d::transformPoint(iter->second, invT);
					}
				}
				fromSignature.setWords3(cpyWordsFrom3);
				toSignature.setWords3(cpyWordsTo3);*/
			}
			else
			{
				transforms[0].setNull();
			}
			transforms[1].setNull();
		}

		info.inliersIDs = allInliers;
		info.matchesIDs = allMatches;
		inliersCount = (int)allInliers.size();
		matchesCount = (int)allMatches.size();
		if(!transforms[1].isNull())
		{
			transforms[1] = transforms[1].inverse();
			if(transforms[0].isNull())
			{
				transform = transforms[1];
				covariance = covariances[1];
			}
			else
			{
				transform = transforms[0].interpolate(0.5f, transforms[1]);
				covariance = (covariances[0]+covariances[1])/2.0f;
			}
		}
		else
		{
			transform = transforms[0];
			covariance = covariances[0];
		}
	}
	else if(toSignature.sensorData().isValid())
	{
		UWARN("Missing correspondences for registration (%d->%d). fromWords = %d fromImageEmpty=%d toWords = %d toImageEmpty=%d",
				fromSignature.id(), toSignature.id(),
				(int)fromSignature.getWords().size(), fromSignature.sensorData().imageRaw().empty()?1:0,
				(int)toSignature.getWords().size(), toSignature.sensorData().imageRaw().empty()?1:0);
	}

	info.inliers = inliersCount;
	info.matches = matchesCount;
	info.rejectedMsg = msg;
	info.covariance = covariance;

	UDEBUG("transform=%s", transform.prettyPrint().c_str());
	return transform;
}

}
