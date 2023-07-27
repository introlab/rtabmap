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
#include <opencv2/core/core_c.h>

#if defined(HAVE_OPENCV_XFEATURES2D) && (CV_MAJOR_VERSION > 3 || (CV_MAJOR_VERSION==3 && CV_MINOR_VERSION >=4 && CV_SUBMINOR_VERSION >= 1))
#include <opencv2/xfeatures2d.hpp> // For GMS matcher
#endif

#include <rtflann/flann.hpp>


#ifdef RTABMAP_PYTHON
	#include "python/PyMatcher.h"
#endif

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
		_PnPVarMedianRatio(Parameters::defaultVisPnPVarianceMedianRatio()),
		_PnPMaxVar(Parameters::defaultVisPnPMaxVariance()),
		_multiSamplingPolicy(Parameters::defaultVisPnPSamplingPolicy()),
		_correspondencesApproach(Parameters::defaultVisCorType()),
		_flowWinSize(Parameters::defaultVisCorFlowWinSize()),
		_flowIterations(Parameters::defaultVisCorFlowIterations()),
		_flowEps(Parameters::defaultVisCorFlowEps()),
		_flowMaxLevel(Parameters::defaultVisCorFlowMaxLevel()),
		_nndr(Parameters::defaultVisCorNNDR()),
		_nnType(Parameters::defaultVisCorNNType()),
		_gmsWithRotation(Parameters::defaultGMSWithRotation()),
		_gmsWithScale(Parameters::defaultGMSWithScale()),
		_gmsThresholdFactor(Parameters::defaultGMSThresholdFactor()),
		_guessWinSize(Parameters::defaultVisCorGuessWinSize()),
		_guessMatchToProjection(Parameters::defaultVisCorGuessMatchToProjection()),
		_bundleAdjustment(Parameters::defaultVisBundleAdjustment()),
		_depthAsMask(Parameters::defaultVisDepthAsMask()),
		_minInliersDistributionThr(Parameters::defaultVisMinInliersDistribution()),
		_maxInliersMeanDistance(Parameters::defaultVisMeanInliersDistance()),
		_detectorFrom(0),
		_detectorTo(0)
#ifdef RTABMAP_PYTHON
		,
		_pyMatcher(0)
#endif
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
	Parameters::parse(parameters, Parameters::kVisPnPVarianceMedianRatio(), _PnPVarMedianRatio);
	Parameters::parse(parameters, Parameters::kVisPnPMaxVariance(), _PnPMaxVar);
	Parameters::parse(parameters, Parameters::kVisPnPSamplingPolicy(), _multiSamplingPolicy);
	Parameters::parse(parameters, Parameters::kVisCorType(), _correspondencesApproach);
	Parameters::parse(parameters, Parameters::kVisCorFlowWinSize(), _flowWinSize);
	Parameters::parse(parameters, Parameters::kVisCorFlowIterations(), _flowIterations);
	Parameters::parse(parameters, Parameters::kVisCorFlowEps(), _flowEps);
	Parameters::parse(parameters, Parameters::kVisCorFlowMaxLevel(), _flowMaxLevel);
	Parameters::parse(parameters, Parameters::kVisCorNNDR(), _nndr);
	Parameters::parse(parameters, Parameters::kVisCorNNType(), _nnType);
	Parameters::parse(parameters, Parameters::kGMSWithRotation(), _gmsWithRotation);
	Parameters::parse(parameters, Parameters::kGMSWithScale(), _gmsWithScale);
	Parameters::parse(parameters, Parameters::kGMSThresholdFactor(), _gmsThresholdFactor);
	Parameters::parse(parameters, Parameters::kVisCorGuessWinSize(), _guessWinSize);
	Parameters::parse(parameters, Parameters::kVisCorGuessMatchToProjection(), _guessMatchToProjection);
	Parameters::parse(parameters, Parameters::kVisBundleAdjustment(), _bundleAdjustment);
	Parameters::parse(parameters, Parameters::kVisDepthAsMask(), _depthAsMask);
	Parameters::parse(parameters, Parameters::kVisMinInliersDistribution(), _minInliersDistributionThr);
	Parameters::parse(parameters, Parameters::kVisMeanInliersDistance(), _maxInliersMeanDistance);
	uInsert(_bundleParameters, parameters);

	if(_minInliers < 6)
	{
		UWARN("%s should be >= 6 but it is set to %d, setting to 6.", Parameters::kVisMinInliers().c_str(), _minInliers);
		_minInliers = 6;
	}
	UASSERT_MSG(_inlierDistance > 0.0f, uFormat("value=%f", _inlierDistance).c_str());
	UASSERT_MSG(_iterations > 0, uFormat("value=%d", _iterations).c_str());

	if(_nnType == 6)
	{
		// verify that we have Python3 support
#ifndef RTABMAP_PYTHON
		UWARN("%s is set to 6 but RTAB-Map is not built with Python3 support, using default %d.",
				Parameters::kVisCorNNType().c_str(), Parameters::defaultVisCorNNType());
		_nnType = Parameters::defaultVisCorNNType();
#else
		int iterations = _pyMatcher?_pyMatcher->iterations():Parameters::defaultPyMatcherIterations();
		float matchThr = _pyMatcher?_pyMatcher->matchThreshold():Parameters::defaultPyMatcherThreshold();
		std::string path = _pyMatcher?_pyMatcher->path():Parameters::defaultPyMatcherPath();
		bool cuda = _pyMatcher?_pyMatcher->cuda():Parameters::defaultPyMatcherCuda();
		std::string model = _pyMatcher?_pyMatcher->model():Parameters::defaultPyMatcherModel();
		Parameters::parse(parameters, Parameters::kPyMatcherIterations(), iterations);
		Parameters::parse(parameters, Parameters::kPyMatcherThreshold(), matchThr);
		Parameters::parse(parameters, Parameters::kPyMatcherPath(), path);
		Parameters::parse(parameters, Parameters::kPyMatcherCuda(), cuda);
		Parameters::parse(parameters, Parameters::kPyMatcherModel(), model);
		if(path.empty())
		{
			UERROR("%s parameter should be set to use Python3 matching (%s=6), using default %d.",
					Parameters::kPyMatcherPath().c_str(),
					Parameters::kVisCorNNType().c_str(),
					Parameters::defaultVisCorNNType());
			_nnType = Parameters::defaultVisCorNNType();
		}
		else
		{
			delete _pyMatcher;
			_pyMatcher = new PyMatcher(path, matchThr, iterations, cuda, model);
		}
#endif
	}
#if !defined(HAVE_OPENCV_XFEATURES2D) || (CV_MAJOR_VERSION == 3 && (CV_MINOR_VERSION<4 || CV_MINOR_VERSION==4 && CV_SUBMINOR_VERSION<1))
	else if(_nnType == 7)
	{
		UWARN("%s is set to 7 but RTAB-Map is not built with OpenCV's xfeatures2d support (OpenCV >= 3.4.1 also required), using default %d.",
				Parameters::kVisCorNNType().c_str(), Parameters::defaultVisCorNNType());
		_nnType = Parameters::defaultVisCorNNType();
	}
#endif

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
		if(_nnType<VWDictionary::kNNUndef)
		{
			uInsert(_featureParameters, ParametersPair(Parameters::kKpNNStrategy(), uNumber2Str(_nnType)));
		}
	}
	if(uContains(parameters, Parameters::kVisCorNNDR()))
	{
		uInsert(_featureParameters, ParametersPair(Parameters::kKpNndrRatio(), parameters.at(Parameters::kVisCorNNDR())));
	}
	if(uContains(parameters, Parameters::kKpByteToFloat()))
	{
		uInsert(_featureParameters, ParametersPair(Parameters::kKpByteToFloat(), parameters.at(Parameters::kKpByteToFloat())));
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

	delete _detectorFrom;
	delete _detectorTo;
	_detectorFrom = Feature2D::create(_featureParameters);
	_detectorTo = Feature2D::create(_featureParameters);
}

RegistrationVis::~RegistrationVis()
{
	delete _detectorFrom;
	delete _detectorTo;
#ifdef RTABMAP_PYTHON
	delete _pyMatcher;
#endif
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
	UDEBUG("%s=%f", Parameters::kVisPnPMaxVariance().c_str(), _PnPMaxVar);
	UDEBUG("%s=%d", Parameters::kVisCorType().c_str(), _correspondencesApproach);
	UDEBUG("%s=%d", Parameters::kVisCorFlowWinSize().c_str(), _flowWinSize);
	UDEBUG("%s=%d", Parameters::kVisCorFlowIterations().c_str(), _flowIterations);
	UDEBUG("%s=%f", Parameters::kVisCorFlowEps().c_str(), _flowEps);
	UDEBUG("%s=%d", Parameters::kVisCorFlowMaxLevel().c_str(), _flowMaxLevel);
	UDEBUG("%s=%f", Parameters::kVisCorNNDR().c_str(), _nndr);
	UDEBUG("%s=%d", Parameters::kVisCorNNType().c_str(), _nnType);
	UDEBUG("%s=%d", Parameters::kVisCorGuessWinSize().c_str(), _guessWinSize);
	UDEBUG("%s=%d", Parameters::kVisCorGuessMatchToProjection().c_str(), _guessMatchToProjection?1:0);
	UDEBUG("Feature Detector = %d", (int)_detectorFrom->getType());
	UDEBUG("guess=%s", guess.prettyPrint().c_str());

	UDEBUG("Input(%d): from=%d words, %d 3D words, %d words descriptors,  %d kpts, %d kpts3D, %d descriptors, image=%dx%d models=%d stereo=%d",
			fromSignature.id(),
			(int)fromSignature.getWords().size(),
			(int)fromSignature.getWords3().size(),
			(int)fromSignature.getWordsDescriptors().rows,
			(int)fromSignature.sensorData().keypoints().size(),
			(int)fromSignature.sensorData().keypoints3D().size(),
			fromSignature.sensorData().descriptors().rows,
			fromSignature.sensorData().imageRaw().cols,
			fromSignature.sensorData().imageRaw().rows,
			(int)fromSignature.sensorData().cameraModels().size(),
			(int)fromSignature.sensorData().stereoCameraModels().size());

	UDEBUG("Input(%d): to=%d words, %d 3D words, %d words descriptors, %d kpts, %d kpts3D, %d descriptors, image=%dx%d models=%d stereo=%d",
			toSignature.id(),
			(int)toSignature.getWords().size(),
			(int)toSignature.getWords3().size(),
			(int)toSignature.getWordsDescriptors().rows,
			(int)toSignature.sensorData().keypoints().size(),
			(int)toSignature.sensorData().keypoints3D().size(),
			toSignature.sensorData().descriptors().rows,
			toSignature.sensorData().imageRaw().cols,
			toSignature.sensorData().imageRaw().rows,
			(int)toSignature.sensorData().cameraModels().size(),
			(int)toSignature.sensorData().stereoCameraModels().size());

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
				(int)fromSignature.getWords().size() == fromSignature.getWordsDescriptors().rows ||
				fromSignature.sensorData().descriptors().empty() ||
				fromSignature.getWordsDescriptors().empty() == 0);
		UASSERT((toSignature.getWords().empty() && toSignature.getWords3().empty())||
				(toSignature.getWords().size() && toSignature.getWords3().empty())||
				(toSignature.getWords().size() == toSignature.getWords3().size()));
		UASSERT((int)toSignature.sensorData().keypoints().size() == toSignature.sensorData().descriptors().rows ||
				(int)toSignature.getWords().size() == toSignature.getWordsDescriptors().rows ||
				toSignature.sensorData().descriptors().empty() ||
				toSignature.getWordsDescriptors().empty());
		UASSERT(fromSignature.sensorData().imageRaw().empty() ||
				fromSignature.sensorData().imageRaw().type() == CV_8UC1 ||
				fromSignature.sensorData().imageRaw().type() == CV_8UC3);
		UASSERT(toSignature.sensorData().imageRaw().empty() ||
				toSignature.sensorData().imageRaw().type() == CV_8UC1 ||
				toSignature.sensorData().imageRaw().type() == CV_8UC3);

		std::vector<cv::KeyPoint> kptsFrom;
		cv::Mat imageFrom = fromSignature.sensorData().imageRaw();
		cv::Mat imageTo = toSignature.sensorData().imageRaw();

		std::vector<int> orignalWordsFromIds;
		int kptsFromSource = 0;
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
						else
						{
							UWARN("%s is true, but RGB size (%dx%d) modulo depth size (%dx%d) is not 0. Ignoring depth mask for feature detection.",
									Parameters::kVisDepthAsMask().c_str(),
									fromSignature.sensorData().imageRaw().rows, fromSignature.sensorData().imageRaw().cols,
									fromSignature.sensorData().depthRaw().rows, fromSignature.sensorData().depthRaw().cols);
						}
					}

					kptsFrom = _detectorFrom->generateKeypoints(
							imageFrom,
							depthMask);
				}
			}
			else
			{
				kptsFrom = fromSignature.sensorData().keypoints();
				kptsFromSource = 1;
			}
		}
		else
		{
			kptsFromSource = 2;
			orignalWordsFromIds.resize(fromSignature.getWords().size());
			int i=0;
			bool allUniques = true;
			int previousIdAdded = 0;
			kptsFrom = fromSignature.getWordsKpts();
			for(std::multimap<int, int>::const_iterator iter=fromSignature.getWords().begin(); iter!=fromSignature.getWords().end(); ++iter)
			{
				UASSERT(iter->second>=0 && iter->second<(int)orignalWordsFromIds.size());
				orignalWordsFromIds[iter->second] = iter->first;
				if(i>0 && iter->first==previousIdAdded)
				{
					allUniques = false;
				}
				previousIdAdded = iter->first;
				++i;
			}
			if(!allUniques)
			{
				UDEBUG("IDs are not unique, IDs will be regenerated!");
				orignalWordsFromIds.clear();
			}
		}

		std::multimap<int, int> wordsFrom;
		std::multimap<int, int> wordsTo;
		std::vector<cv::KeyPoint> wordsKptsFrom;
		std::vector<cv::KeyPoint> wordsKptsTo;
		std::vector<cv::Point3f> words3From;
		std::vector<cv::Point3f> words3To;
		cv::Mat wordsDescFrom;
		cv::Mat wordsDescTo;
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
				kptsFrom3D = fromSignature.getWords3();
			}
			else if(kptsFrom.size() == fromSignature.sensorData().keypoints3D().size())
			{
				kptsFrom3D = fromSignature.sensorData().keypoints3D();
			}
			else
			{
				kptsFrom3D = _detectorFrom->generateKeypoints3D(fromSignature.sensorData(), kptsFrom);
			}

			if(!imageFrom.empty() && !imageTo.empty())
			{
				std::vector<cv::Point2f> cornersFrom;
				cv::KeyPoint::convert(kptsFrom, cornersFrom);
				std::vector<cv::Point2f> cornersTo;
				bool guessSet = !guess.isIdentity() && !guess.isNull();
				if(guessSet)
				{
					if(toSignature.sensorData().cameraModels().size() == 1 || toSignature.sensorData().stereoCameraModels().size() == 1)
					{
						Transform localTransform = toSignature.sensorData().cameraModels().size()?toSignature.sensorData().cameraModels()[0].localTransform():toSignature.sensorData().stereoCameraModels()[0].left().localTransform();
						Transform guessCameraRef = (guess * localTransform).inverse();
						cv::Mat R = (cv::Mat_<double>(3,3) <<
								(double)guessCameraRef.r11(), (double)guessCameraRef.r12(), (double)guessCameraRef.r13(),
								(double)guessCameraRef.r21(), (double)guessCameraRef.r22(), (double)guessCameraRef.r23(),
								(double)guessCameraRef.r31(), (double)guessCameraRef.r32(), (double)guessCameraRef.r33());
						cv::Mat rvec(1,3, CV_64FC1);
						cv::Rodrigues(R, rvec);
						cv::Mat tvec = (cv::Mat_<double>(1,3) << (double)guessCameraRef.x(), (double)guessCameraRef.y(), (double)guessCameraRef.z());
						cv::Mat K = toSignature.sensorData().cameraModels().size()?toSignature.sensorData().cameraModels()[0].K():toSignature.sensorData().stereoCameraModels()[0].left().K();
						cv::projectPoints(kptsFrom3D, rvec, tvec, K, cv::Mat(), cornersTo);
					}
					else
					{
						UERROR("Optical flow guess with multi-cameras is not implemented, guess ignored...");
					}
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
					kptsTo3D = _detectorTo->generateKeypoints3D(toSignature.sensorData(), kptsTo);
				}

				UASSERT(kptsFrom.size() == kptsFrom3DKept.size());
				UASSERT(kptsFrom.size() == kptsTo.size());
				UASSERT(kptsTo3D.size() == 0 || kptsTo.size() == kptsTo3D.size());
				for(unsigned int i=0; i< kptsFrom3DKept.size(); ++i)
				{
					int id = !orignalWordsFromIds.empty()?orignalWordsFromIds[i]:i;
					wordsFrom.insert(wordsFrom.end(), std::make_pair(id, wordsFrom.size()));
					wordsKptsFrom.push_back(kptsFrom[i]);
					words3From.push_back(kptsFrom3DKept[i]);

					wordsTo.insert(wordsTo.end(), std::make_pair(id, wordsTo.size()));
					wordsKptsTo.push_back(kptsTo[i]);
					if(!kptsTo3D.empty())
					{
						words3To.push_back(kptsTo3D[i]);
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
						int id = !orignalWordsFromIds.empty()?orignalWordsFromIds[i]:i;
						wordsFrom.insert(wordsFrom.end(), std::make_pair(id, wordsFrom.size()));
						wordsKptsFrom.push_back(kptsFrom[i]);
						words3From.push_back(kptsFrom3D[i]);
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
			int kptsToSource = 0;
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
						else
						{
							UWARN("%s is true, but RGB size (%dx%d) modulo depth size (%dx%d) is not 0. Ignoring depth mask for feature detection.",
									Parameters::kVisDepthAsMask().c_str(),
									toSignature.sensorData().imageRaw().rows, toSignature.sensorData().imageRaw().cols,
									toSignature.sensorData().depthRaw().rows, toSignature.sensorData().depthRaw().cols);
						}
					}

					kptsTo = _detectorTo->generateKeypoints(
							imageTo,
							depthMask);
				}
				else
				{
					kptsTo = toSignature.sensorData().keypoints();
					kptsToSource = 1;
				}
			}
			else
			{
				kptsTo = toSignature.getWordsKpts();
				kptsToSource = 2;
			}

			// extract descriptors
			UDEBUG("kptsFrom=%d kptsFromSource=%d", (int)kptsFrom.size(), kptsFromSource);
			UDEBUG("kptsTo=%d kptsToSource=%d", (int)kptsTo.size(), kptsToSource);
			cv::Mat descriptorsFrom;
			if(kptsFromSource == 2 &&
				fromSignature.getWordsDescriptors().rows &&
				((kptsFrom.empty() && fromSignature.getWordsDescriptors().rows) ||
				 fromSignature.getWordsDescriptors().rows == (int)kptsFrom.size()))
			{
				descriptorsFrom = fromSignature.getWordsDescriptors();
			}
			else if(kptsFromSource == 1 &&
					fromSignature.sensorData().descriptors().rows == (int)kptsFrom.size())
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
				UDEBUG("cleared orignalWordsFromIds");
				orignalWordsFromIds.clear();
				descriptorsFrom = _detectorFrom->generateDescriptors(imageFrom, kptsFrom);
			}

			cv::Mat descriptorsTo;
			if(kptsTo.size())
			{
				if(kptsToSource == 2 &&
					toSignature.getWordsDescriptors().rows == (int)kptsTo.size())
				{
					descriptorsTo = toSignature.getWordsDescriptors();
				}
				else if(kptsToSource == 1 &&
						toSignature.sensorData().descriptors().rows == (int)kptsTo.size())
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

					descriptorsTo = _detectorTo->generateDescriptors(imageTo, kptsTo);
				}
			}

			// create 3D keypoints
			std::vector<cv::Point3f> kptsFrom3D;
			std::vector<cv::Point3f> kptsTo3D;
			if(kptsFromSource == 2 &&
				kptsFrom.size() == fromSignature.getWords3().size())
			{
				kptsFrom3D = fromSignature.getWords3();
			}
			else if(kptsFromSource == 1 &&
					kptsFrom.size() == fromSignature.sensorData().keypoints3D().size())
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
				kptsFrom3D = _detectorFrom->generateKeypoints3D(fromSignature.sensorData(), kptsFrom);
				UDEBUG("generated kptsFrom3D=%d", (int)kptsFrom3D.size());
			}

			if(!kptsFrom3D.empty() &&
			   (_detectorFrom->getMinDepth() > 0.0f || _detectorFrom->getMaxDepth() > 0.0f) &&
			   (!fromSignature.sensorData().cameraModels().empty() || !fromSignature.sensorData().stereoCameraModels().empty())) // Ignore local map from OdometryF2M
			{
				_detectorFrom->filterKeypointsByDepth(kptsFrom, descriptorsFrom, kptsFrom3D, _detectorFrom->getMinDepth(), _detectorFrom->getMaxDepth());
			}

			if(kptsToSource == 2 && kptsTo.size() == toSignature.getWords3().size())
			{
				kptsTo3D = toSignature.getWords3();
			}
			else if(kptsToSource == 1 &&
					kptsTo.size() == toSignature.sensorData().keypoints3D().size())
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
				kptsTo3D = _detectorTo->generateKeypoints3D(toSignature.sensorData(), kptsTo);
			}

			if(kptsTo3D.size() &&
		       (_detectorTo->getMinDepth() > 0.0f || _detectorTo->getMaxDepth() > 0.0f) &&
			   (!toSignature.sensorData().cameraModels().empty() || !toSignature.sensorData().stereoCameraModels().empty())) // Ignore local map from OdometryF2M
			{
				_detectorTo->filterKeypointsByDepth(kptsTo, descriptorsTo, kptsTo3D, _detectorTo->getMinDepth(), _detectorTo->getMaxDepth());
			}

			UASSERT(kptsFrom.empty() || descriptorsFrom.rows == 0 || int(kptsFrom.size()) == descriptorsFrom.rows);

			fromSignature.sensorData().setFeatures(kptsFrom, kptsFrom3D, descriptorsFrom);
			toSignature.sensorData().setFeatures(kptsTo, kptsTo3D, descriptorsTo);

			UDEBUG("descriptorsFrom=%d", descriptorsFrom.rows);
			UDEBUG("descriptorsTo=%d", descriptorsTo.rows);
			UDEBUG("orignalWordsFromIds=%d", (int)orignalWordsFromIds.size());

			// We have all data we need here, so match!
			if(descriptorsFrom.rows > 0 && descriptorsTo.rows > 0)
			{
				std::vector<CameraModel> models;
				if(!toSignature.sensorData().stereoCameraModels().empty())
				{
					for(size_t i=0; i<toSignature.sensorData().stereoCameraModels().size(); ++i)
					{
						models.push_back(toSignature.sensorData().stereoCameraModels()[i].left());
					}
				}
				else
				{
					models = toSignature.sensorData().cameraModels();
				}

				bool isCalibrated = !models.empty();
				for(size_t i=0; i<models.size() && isCalibrated; ++i)
				{
					isCalibrated = models[i].isValidForProjection();

					// For old database formats
					if(isCalibrated && (models[i].imageWidth()==0 || models[i].imageHeight()==0))
					{
						if(!toSignature.sensorData().imageRaw().empty())
						{
							models[i].setImageSize(cv::Size(toSignature.sensorData().imageRaw().cols/models.size(), toSignature.sensorData().imageRaw().rows));
						}
						else
						{
							isCalibrated = false;
						}
					}
				}

				// If guess is set, limit the search of matches using optical flow window size
				bool guessSet = !guess.isIdentity() && !guess.isNull();
				if(guessSet && _guessWinSize > 0 && kptsFrom3D.size() &&
						isCalibrated &&  // needed for projection
						_estimationType != 2) // To make sure we match all features for 2D->2D
				{
					// Use guess to project 3D "from" keypoints into "to" image
					UDEBUG("");
					UASSERT((int)kptsTo.size() == descriptorsTo.rows);
					UASSERT((int)kptsFrom3D.size() == descriptorsFrom.rows);

					std::vector<cv::Point2f> cornersProjected;
					std::vector<int> projectedIndexToDescIndex;
					float subImageWidth = models[0].imageWidth();
					std::set<int> added;
					int duplicates=0;
					for(size_t m=0; m<models.size(); ++m)
					{
						Transform guessCameraRef = (guess * models[m].localTransform()).inverse();
						cv::Mat R = (cv::Mat_<double>(3,3) <<
								(double)guessCameraRef.r11(), (double)guessCameraRef.r12(), (double)guessCameraRef.r13(),
								(double)guessCameraRef.r21(), (double)guessCameraRef.r22(), (double)guessCameraRef.r23(),
								(double)guessCameraRef.r31(), (double)guessCameraRef.r32(), (double)guessCameraRef.r33());
						cv::Mat rvec(1,3, CV_64FC1);
						cv::Rodrigues(R, rvec);
						cv::Mat tvec = (cv::Mat_<double>(1,3) << (double)guessCameraRef.x(), (double)guessCameraRef.y(), (double)guessCameraRef.z());
						cv::Mat K = models[m].K();
						std::vector<cv::Point2f> projected;
						cv::projectPoints(kptsFrom3D, rvec, tvec, K, cv::Mat(), projected);
						UDEBUG("Projected points=%d", (int)projected.size());

						//remove projected points outside of the image
						UASSERT((int)projected.size() == descriptorsFrom.rows);
						int cornersInFrame = 0;
						for(unsigned int i=0; i<projected.size(); ++i)
						{
							if(uIsInBounds(projected[i].x, 0.0f, float(models[m].imageWidth()-1)) &&
							   uIsInBounds(projected[i].y, 0.0f, float(models[m].imageHeight()-1)) &&
							   util3d::transformPoint(kptsFrom3D[i], guessCameraRef).z > 0.0)
							{
								if(added.find(i) != added.end())
								{
									++duplicates;
									continue;
								}

								projectedIndexToDescIndex.push_back(i);
								projected[i].x += subImageWidth*float(m); // Convert in multicam stitched image
								cornersProjected.push_back(projected[i]);
								++cornersInFrame;
								added.insert(i);
							}
						}
						UDEBUG("corners in frame=%d (camera index=%ld)", cornersInFrame, m);
					}

					// For each projected feature guess of "from" in "to", find its matching feature in
					// the radius around the projected guess.
					// TODO: do cross-check?
					UDEBUG("guessMatchToProjection=%d, cornersProjected=%d orignalWordsFromIds=%d (added=%ld, duplicates=%d)",
							_guessMatchToProjection?1:0, (int)cornersProjected.size(), (int)orignalWordsFromIds.size(),
							added.size(), duplicates);
					if(cornersProjected.size())
					{
						if(_guessMatchToProjection)
						{
							UDEBUG("match frame to projected");
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
							int newToId = !orignalWordsFromIds.empty()?fromSignature.getWords().rbegin()->first+1:descriptorsFrom.rows;
							std::map<int,int> addedWordsFrom; //<id, index>
							std::map<int, int> duplicates; //<fromId, toId>
							int newWords = 0;
							cv::Mat descriptors(10, descriptorsTo.cols, descriptorsTo.type());
							for(unsigned int i = 0; i < pointsToMat.rows; ++i)
							{
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
										descriptorsFrom.row(projectedIndexToDescIndex[indices[i].at(j)]).copyTo(descriptors.row(oi));
										descriptorsIndices[oi++] = indices[i].at(j);
									}
									descriptorsIndices.resize(oi);
									UASSERT(oi >=2);

									cv::BFMatcher matcher(descriptors.type()==CV_8U?cv::NORM_HAMMING:cv::NORM_L2SQR, _nnType == 5);
									if(_nnType == 5) // bruteforce cross check
									{
										std::vector<cv::DMatch> matches;
										matcher.match(descriptorsTo.row(i), cv::Mat(descriptors, cv::Range(0, oi)), matches);
										if(!matches.empty())
										{
											matchedIndex = descriptorsIndices.at(matches.at(0).trainIdx);
										}
									}
									else // bruteforce knn
									{
										std::vector<std::vector<cv::DMatch> > matches;
										matcher.knnMatch(descriptorsTo.row(i), cv::Mat(descriptors, cv::Range(0, oi)), matches, 2);
										UASSERT(matches.size() == 1);
										UASSERT(matches[0].size() == 2);
										if(matches[0].at(0).distance < _nndr * matches[0].at(1).distance)
										{
											matchedIndex = descriptorsIndices.at(matches[0].at(0).trainIdx);
										}
									}
								}
								else if(indices[i].size() == 1)
								{
									matchedIndex = indices[i].at(0);
								}

								if(matchedIndex >= 0)
								{
									matchedIndex = projectedIndexToDescIndex[matchedIndex];
									int id = !orignalWordsFromIds.empty()?orignalWordsFromIds[matchedIndex]:matchedIndex;

									if(addedWordsFrom.find(matchedIndex) != addedWordsFrom.end())
									{
										id = addedWordsFrom.at(matchedIndex);
										duplicates.insert(std::make_pair(matchedIndex, id));
									}
									else
									{
										addedWordsFrom.insert(std::make_pair(matchedIndex, id));

										wordsFrom.insert(wordsFrom.end(), std::make_pair(id, wordsFrom.size()));
										if(!kptsFrom.empty())
										{
											wordsKptsFrom.push_back(kptsFrom[matchedIndex]);
										}
										words3From.push_back(kptsFrom3D[matchedIndex]);
										wordsDescFrom.push_back(descriptorsFrom.row(matchedIndex));
									}

									wordsTo.insert(wordsTo.end(), std::make_pair(id, wordsTo.size()));
									wordsKptsTo.push_back(kptsTo[i]);
									wordsDescTo.push_back(descriptorsTo.row(i));
									if(!kptsTo3D.empty())
									{
										words3To.push_back(kptsTo3D[i]);
									}
								}
								else
								{
									// gen fake ids
									wordsTo.insert(wordsTo.end(), std::make_pair(newToId, wordsTo.size()));
									wordsKptsTo.push_back(kptsTo[i]);
									wordsDescTo.push_back(descriptorsTo.row(i));
									if(!kptsTo3D.empty())
									{
										words3To.push_back(kptsTo3D[i]);
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
									int id = !orignalWordsFromIds.empty()?orignalWordsFromIds[i]:i;
									wordsFrom.insert(wordsFrom.end(), std::make_pair(id, wordsFrom.size()));
									wordsKptsFrom.push_back(kptsFrom[i]);
									wordsDescFrom.push_back(descriptorsFrom.row(i));
									words3From.push_back(kptsFrom3D[i]);

									++addWordsFromNotMatched;
								}
							}
							UDEBUG("addWordsFromNotMatched=%d -> words3From=%d", addWordsFromNotMatched, (int)words3From.size());
						}
						else
						{
							UDEBUG("match projected to frame");
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
							double bruteForceTotalTime = 0.0;
							double bruteForceDescCopy = 0.0;
							UTimer bruteForceTimer;
							cv::Mat descriptors(10, descriptorsTo.cols, descriptorsTo.type());
							for(unsigned int i = 0; i < cornersProjectedMat.rows; ++i)
							{
								int matchedIndexFrom = projectedIndexToDescIndex[i];

								if(indices[i].size())
								{
									info.projectedIDs.push_back(!orignalWordsFromIds.empty()?orignalWordsFromIds[matchedIndexFrom]:matchedIndexFrom);
								}

								if(util3d::isFinite(kptsFrom3D[matchedIndexFrom]))
								{
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
										for(unsigned int j=0; j<indices[i].size(); ++j)
										{
											descriptorsTo.row(indices[i].at(j)).copyTo(descriptors.row(oi));
											descriptorsIndices[oi++] = indices[i].at(j);
										}
										bruteForceDescCopy += bruteForceTimer.ticks();
										UASSERT(oi >=2);

										cv::BFMatcher matcher(descriptors.type()==CV_8U?cv::NORM_HAMMING:cv::NORM_L2SQR, _nnType==5);
										if(_nnType==5) // bruteforce cross check
										{
											std::vector<cv::DMatch> matches;
											matcher.match(descriptorsFrom.row(matchedIndexFrom), cv::Mat(descriptors, cv::Range(0, oi)), matches);
											if(!matches.empty())
											{
												matchedIndexTo = descriptorsIndices.at(matches.at(0).trainIdx);
											}
										}
										else // bruteforce knn
										{
											std::vector<std::vector<cv::DMatch> > matches;
											matcher.knnMatch(descriptorsFrom.row(matchedIndexFrom), cv::Mat(descriptors, cv::Range(0, oi)), matches, 2);
											UASSERT(matches.size() == 1);
											UASSERT(matches[0].size() == 2);
											bruteForceTotalTime+=bruteForceTimer.elapsed();
											if(matches[0].at(0).distance < _nndr * matches[0].at(1).distance)
											{
												matchedIndexTo = descriptorsIndices.at(matches[0].at(0).trainIdx);
											}
										}
									}
									else if(indices[i].size() == 1)
									{
										matchedIndexTo = indices[i].at(0);
									}

									int id = !orignalWordsFromIds.empty()?orignalWordsFromIds[matchedIndexFrom]:matchedIndexFrom;
									addedWordsFrom.insert(addedWordsFrom.end(), matchedIndexFrom);

									wordsFrom.insert(wordsFrom.end(), std::make_pair(id, wordsFrom.size()));
									if(!kptsFrom.empty())
									{
										wordsKptsFrom.push_back(kptsFrom[matchedIndexFrom]);
									}
									words3From.push_back(kptsFrom3D[matchedIndexFrom]);
									wordsDescFrom.push_back(descriptorsFrom.row(matchedIndexFrom));

									if(	matchedIndexTo >= 0 &&
										addedWordsTo.find(matchedIndexTo) == addedWordsTo.end())
									{
										addedWordsTo.insert(matchedIndexTo);

										wordsTo.insert(wordsTo.end(), std::make_pair(id, wordsTo.size()));
										wordsKptsTo.push_back(kptsTo[matchedIndexTo]);
										wordsDescTo.push_back(descriptorsTo.row(matchedIndexTo));
										if(!kptsTo3D.empty())
										{
											words3To.push_back(kptsTo3D[matchedIndexTo]);
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
									int id = !orignalWordsFromIds.empty()?orignalWordsFromIds[i]:i;
									wordsFrom.insert(wordsFrom.end(), std::make_pair(id, wordsFrom.size()));
									wordsKptsFrom.push_back(kptsFrom[i]);
									wordsDescFrom.push_back(descriptorsFrom.row(i));
									words3From.push_back(kptsFrom3D[i]);
								}
							}

							int newToId = !orignalWordsFromIds.empty()?fromSignature.getWords().rbegin()->first+1:descriptorsFrom.rows;
							for(unsigned int i = 0; i < kptsTo.size(); ++i)
							{
								if(addedWordsTo.find(i) == addedWordsTo.end())
								{
									wordsTo.insert(wordsTo.end(), std::make_pair(newToId, wordsTo.size()));
									wordsKptsTo.push_back(kptsTo[i]);
									wordsDescTo.push_back(descriptorsTo.row(i));
									if(!kptsTo3D.empty())
									{
										words3To.push_back(kptsTo3D[i]);
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
						UWARN("Calibration not found! Finding correspondences "
							   "with the guess cannot be done, global matching is "
							   "done instead.");
					}

					UDEBUG("");
					// match between all descriptors
					std::list<int> fromWordIds;
					std::list<int> toWordIds;
#ifdef RTABMAP_PYTHON
					if(_nnType == 5 || (_nnType == 6 && _pyMatcher) || _nnType==7)
#else
					if(_nnType == 5 || _nnType == 7) // bruteforce cross check or GMS
#endif
					{
						std::vector<int> fromWordIdsV(descriptorsFrom.rows);
						for (int i = 0; i < descriptorsFrom.rows; ++i)
						{
							int id = i+1;
							if(!orignalWordsFromIds.empty())
							{
								id = orignalWordsFromIds[i];
							}
							fromWordIds.push_back(id);
							fromWordIdsV[i] = id;
						}
						if(descriptorsTo.rows)
						{
							std::vector<int> toWordIdsV(descriptorsTo.rows, 0);
							std::vector<cv::DMatch> matches;
#ifdef RTABMAP_PYTHON
							if(_nnType == 6 && _pyMatcher &&
								descriptorsTo.cols == descriptorsFrom.cols &&
								descriptorsTo.rows == (int)kptsTo.size() &&
								descriptorsTo.type() == CV_32F &&
								descriptorsFrom.type() == CV_32F &&
								descriptorsFrom.rows == (int)kptsFrom.size() &&
								models.size() == 1)
							{
								UDEBUG("Python matching");
								matches = _pyMatcher->match(descriptorsTo, descriptorsFrom, kptsTo, kptsFrom, models[0].imageSize());
							}
							else
							{
								if(_nnType == 6 && _pyMatcher)
								{
									UDEBUG("Invalid inputs for Python matching (desc type=%d, only float descriptors supported, multicam not supported), doing bruteforce matching instead.", descriptorsFrom.type());
								}
#else
							{
#endif
								bool doCrossCheck = true;
#ifdef HAVE_OPENCV_XFEATURES2D
#if CV_MAJOR_VERSION > 3 || (CV_MAJOR_VERSION==3 && CV_MINOR_VERSION >=4 && CV_SUBMINOR_VERSION >= 1)
								cv::Size imageSizeFrom;
								if(_nnType == 7)
								{
									imageSizeFrom = imageFrom.size();
									if((imageSizeFrom.height == 0 || imageSizeFrom.width == 0) && (fromSignature.sensorData().cameraModels().size() || fromSignature.sensorData().stereoCameraModels().size()))
									{
										imageSizeFrom = fromSignature.sensorData().cameraModels().size() == 1?fromSignature.sensorData().cameraModels()[0].imageSize():fromSignature.sensorData().stereoCameraModels()[0].left().imageSize();
									}
									if(!models.empty() && models[0].imageSize().height > 0 && models[0].imageSize().width > 0 &&
									   imageSizeFrom.height > 0 && imageSizeFrom.width > 0)
									{
										doCrossCheck = false;
									}
									else
									{
										UDEBUG("Invalid inputs for GMS matching, image size should be set for both inputs, doing bruteforce matching instead.");
									}
								}
#endif
#endif

								UDEBUG("BruteForce matching%s", _nnType!=7?" with crosscheck":" with GMS");
								cv::BFMatcher matcher(descriptorsFrom.type()==CV_8U?cv::NORM_HAMMING:cv::NORM_L2SQR, doCrossCheck);
								matcher.match(descriptorsTo, descriptorsFrom, matches);

#if defined(HAVE_OPENCV_XFEATURES2D) && (CV_MAJOR_VERSION > 3 || (CV_MAJOR_VERSION==3 && CV_MINOR_VERSION >=4 && CV_SUBMINOR_VERSION >= 1))
								if(!doCrossCheck)
								{
									UASSERT(!models.empty());
									std::vector<cv::DMatch> matchesGMS;
									cv::xfeatures2d::matchGMS(models[0].imageSize(), imageSizeFrom, kptsTo, kptsFrom, matches, matchesGMS, _gmsWithRotation, _gmsWithScale, _gmsThresholdFactor);
									matches = matchesGMS;
								}
#endif
							}
							for(size_t i=0; i<matches.size(); ++i)
							{
								toWordIdsV[matches[i].queryIdx] = fromWordIdsV[matches[i].trainIdx];
							}
							for(size_t i=0; i<toWordIdsV.size(); ++i)
							{
								int toId = toWordIdsV[i];
								if(toId==0)
								{
									toId = fromWordIds.back()+i+1;
								}
								toWordIds.push_back(toId);
							}
						}
					}
					else
					{
						UDEBUG("VWDictionary knn matching");
						VWDictionary dictionary(_featureParameters);
						if(orignalWordsFromIds.empty())
						{
							fromWordIds = dictionary.addNewWords(descriptorsFrom, 1);
						}
						else
						{
							for (int i = 0; i < descriptorsFrom.rows; ++i)
							{
								int id = orignalWordsFromIds[i];
								dictionary.addWord(new VisualWord(id, descriptorsFrom.row(i), 1));
								fromWordIds.push_back(id);
							}
						}

						if(descriptorsTo.rows)
						{
							dictionary.update();
							toWordIds = dictionary.addNewWords(descriptorsTo, 2);
						}
						dictionary.clear(false);
					}

					std::multiset<int> fromWordIdsSet(fromWordIds.begin(), fromWordIds.end());
					std::multiset<int> toWordIdsSet(toWordIds.begin(), toWordIds.end());

					UASSERT(kptsFrom3D.empty() || fromWordIds.size() == kptsFrom3D.size());
					UASSERT(int(fromWordIds.size()) == descriptorsFrom.rows);
					int i=0;
					for(std::list<int>::iterator iter=fromWordIds.begin(); iter!=fromWordIds.end(); ++iter)
					{
						if(fromWordIdsSet.count(*iter) == 1)
						{
							wordsFrom.insert(wordsFrom.end(), std::make_pair(*iter, wordsFrom.size()));
							if (!kptsFrom.empty())
							{
								wordsKptsFrom.push_back(kptsFrom[i]);
							}
							if(!kptsFrom3D.empty())
							{
								words3From.push_back(kptsFrom3D[i]);
							}
							wordsDescFrom.push_back(descriptorsFrom.row(i));
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
							wordsTo.insert(wordsTo.end(), std::make_pair(*iter, wordsTo.size()));
							wordsKptsTo.push_back(kptsTo[i]);
							wordsDescTo.push_back(descriptorsTo.row(i));
							if(!kptsTo3D.empty())
							{
								words3To.push_back(kptsTo3D[i]);
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
					wordsFrom.insert(wordsFrom.end(), std::make_pair(i, wordsFrom.size()));
					wordsKptsFrom.push_back(kptsFrom[i]);
					wordsDescFrom.push_back(descriptorsFrom.row(i));
					if(!kptsFrom3D.empty())
					{
						words3From.push_back(kptsFrom3D[i]);
					}
				}
			}
		}

		fromSignature.setWords(wordsFrom, wordsKptsFrom, words3From, wordsDescFrom);
		toSignature.setWords(wordsTo, wordsKptsTo, words3To, wordsDescTo);
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
				if((signatureB->sensorData().stereoCameraModels().size() != 1 ||
					!signatureB->sensorData().stereoCameraModels()[0].isValidForProjection()) &&
				   (signatureB->sensorData().cameraModels().size() != 1 ||
					!signatureB->sensorData().cameraModels()[0].isValidForProjection()))
				{
					UERROR("Calibrated camera required (multi-cameras not supported).");
				}
				else if((int)signatureA->getWords().size() >= _minInliers &&
						(int)signatureB->getWords().size() >= _minInliers)
				{
					UASSERT((signatureA->sensorData().stereoCameraModels().size() == 1 && signatureA->sensorData().stereoCameraModels()[0].isValidForProjection()) || (signatureA->sensorData().cameraModels().size() == 1 && signatureA->sensorData().cameraModels()[0].isValidForProjection()));
					const CameraModel & cameraModel = signatureA->sensorData().stereoCameraModels().size()?signatureA->sensorData().stereoCameraModels()[0].left():signatureA->sensorData().cameraModels()[0];

					// we only need the camera transform, send guess words3 for scale estimation
					Transform cameraTransform;
					double variance = 1.0f;
					std::vector<int> matchesV;
					std::map<int, int> uniqueWordsA = uMultimapToMapUnique(signatureA->getWords());
					std::map<int, int> uniqueWordsB = uMultimapToMapUnique(signatureB->getWords());
					std::map<int, cv::KeyPoint> wordsA;
					std::map<int, cv::Point3f> words3A;
					std::map<int, cv::KeyPoint> wordsB;
					for(std::map<int, int>::iterator iter=uniqueWordsA.begin(); iter!=uniqueWordsA.end(); ++iter)
					{
						wordsA.insert(std::make_pair(iter->first, signatureA->getWordsKpts()[iter->second]));
						if(!signatureA->getWords3().empty())
						{
							words3A.insert(std::make_pair(iter->first, signatureA->getWords3()[iter->second]));
						}
					}
					for(std::map<int, int>::iterator iter=uniqueWordsB.begin(); iter!=uniqueWordsB.end(); ++iter)
					{
						wordsB.insert(std::make_pair(iter->first, signatureB->getWordsKpts()[iter->second]));
					}
					std::map<int, cv::Point3f> inliers3D = util3d::generateWords3DMono(
							wordsA,
							wordsB,
							cameraModel,
							cameraTransform,
							_PnPReprojError,
							0.99f,
							words3A, // for scale estimation
							&variance,
							&matchesV);
					covariances[dir] *= variance;
					inliers[dir] = uKeys(inliers3D);
					matches[dir] = matchesV;

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
								msg = uFormat("Variance is too high! (Max %s=%f, variance=%f)", Parameters::kVisEpipolarGeometryVar().c_str(), _epipolarGeometryVar, variance);
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
				if((signatureB->sensorData().stereoCameraModels().empty() || !signatureB->sensorData().stereoCameraModels()[0].isValidForProjection()) &&
				   (signatureB->sensorData().cameraModels().empty() || !signatureB->sensorData().cameraModels()[0].isValidForProjection()))
				{
					UERROR("Calibrated camera required. Id=%d Models=%d StereoModels=%d weight=%d",
							signatureB->id(),
							(int)signatureB->sensorData().cameraModels().size(),
							signatureB->sensorData().stereoCameraModels().size(),
							signatureB->getWeight());
				}
#ifndef RTABMAP_OPENGV
				else if(signatureB->sensorData().cameraModels().size() > 1)
				{
					UERROR("Multi-camera 2D-3D PnP registration is only available if rtabmap is built "
							"with OpenGV dependency. Use 3D-3D registration approach instead for multi-camera.");
				}
#endif
				else
				{
					UDEBUG("words from3D=%d to2D=%d", (int)signatureA->getWords3().size(), (int)signatureB->getWords().size());
					// 3D to 2D
					if((int)signatureA->getWords3().size() >= _minInliers &&
					   (int)signatureB->getWords().size() >= _minInliers)
					{
						std::vector<int> inliersV;
						std::vector<int> matchesV;
						std::map<int, int> uniqueWordsA = uMultimapToMapUnique(signatureA->getWords());
						std::map<int, int> uniqueWordsB = uMultimapToMapUnique(signatureB->getWords());
						std::map<int, cv::Point3f> words3A;
						std::map<int, cv::Point3f> words3B;
						std::map<int, cv::KeyPoint> wordsB;
						for(std::map<int, int>::iterator iter=uniqueWordsA.begin(); iter!=uniqueWordsA.end(); ++iter)
						{
							words3A.insert(std::make_pair(iter->first, signatureA->getWords3()[iter->second]));
						}
						for(std::map<int, int>::iterator iter=uniqueWordsB.begin(); iter!=uniqueWordsB.end(); ++iter)
						{
							wordsB.insert(std::make_pair(iter->first, signatureB->getWordsKpts()[iter->second]));
							if(!signatureB->getWords3().empty())
							{
								words3B.insert(std::make_pair(iter->first, signatureB->getWords3()[iter->second]));
							}
						}

						std::vector<CameraModel> models;
						if(signatureB->sensorData().stereoCameraModels().size())
						{
							for(size_t i=0; i<signatureB->sensorData().stereoCameraModels().size(); ++i)
							{
								models.push_back(signatureB->sensorData().stereoCameraModels()[i].left());
							}
						}
						else
						{
							models = signatureB->sensorData().cameraModels();
						}

						if(models.size()>1)
						{
							// Multi-Camera
							UASSERT(models[0].isValidForProjection());

							transforms[dir] = util3d::estimateMotion3DTo2D(
									words3A,
									wordsB,
									models,
									_multiSamplingPolicy,
									_minInliers,
									_iterations,
									_PnPReprojError,
									_PnPFlags,
									_PnPRefineIterations,
									_PnPVarMedianRatio,
									_PnPMaxVar,
									dir==0?(!guess.isNull()?guess:Transform::getIdentity()):!transforms[0].isNull()?transforms[0].inverse():(!guess.isNull()?guess.inverse():Transform::getIdentity()),
									words3B,
									&covariances[dir],
									&matchesV,
									&inliersV);
							inliers[dir] = inliersV;
							matches[dir] = matchesV;
						}
						else
						{
							UASSERT(models.size() == 1 && models[0].isValidForProjection());

							transforms[dir] = util3d::estimateMotion3DTo2D(
									words3A,
									wordsB,
									models[0],
									_minInliers,
									_iterations,
									_PnPReprojError,
									_PnPFlags,
									_PnPRefineIterations,
									_PnPVarMedianRatio,
									_PnPMaxVar,
									dir==0?(!guess.isNull()?guess:Transform::getIdentity()):!transforms[0].isNull()?transforms[0].inverse():(!guess.isNull()?guess.inverse():Transform::getIdentity()),
									words3B,
									&covariances[dir],
									&matchesV,
									&inliersV);
							inliers[dir] = inliersV;
							matches[dir] = matchesV;
						}
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
					std::map<int, int> uniqueWordsA = uMultimapToMapUnique(signatureA->getWords());
					std::map<int, int> uniqueWordsB = uMultimapToMapUnique(signatureB->getWords());
					std::map<int, cv::Point3f> words3A;
					std::map<int, cv::Point3f> words3B;
					for(std::map<int, int>::iterator iter=uniqueWordsA.begin(); iter!=uniqueWordsA.end(); ++iter)
					{
						words3A.insert(std::make_pair(iter->first, signatureA->getWords3()[iter->second]));
					}
					for(std::map<int, int>::iterator iter=uniqueWordsB.begin(); iter!=uniqueWordsB.end(); ++iter)
					{
						words3B.insert(std::make_pair(iter->first, signatureB->getWords3()[iter->second]));
					}
					transforms[dir] = util3d::estimateMotion3DTo3D(
							words3A,
							words3B,
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
			(fromSignature.sensorData().stereoCameraModels().size() >= 1 || fromSignature.sensorData().cameraModels().size() >= 1) &&
			(toSignature.sensorData().stereoCameraModels().size() >= 1 || toSignature.sensorData().cameraModels().size() >= 1))
		{
			UDEBUG("Refine with bundle adjustment");
			Optimizer * sba = Optimizer::create(_bundleAdjustment==3?Optimizer::kTypeCeres:_bundleAdjustment==2?Optimizer::kTypeCVSBA:Optimizer::kTypeG2O, _bundleParameters);

			std::map<int, Transform> poses;
			std::multimap<int, Link> links;
			std::map<int, cv::Point3f> points3DMap;

			poses.insert(std::make_pair(1, Transform::getIdentity()));
			poses.insert(std::make_pair(2, transforms[0]));

			for(int i=0;i<2;++i)
			{
				UASSERT(covariances[i].cols==6 && covariances[i].rows == 6 && covariances[i].type() == CV_64FC1);
				if(covariances[i].at<double>(0,0)<=COVARIANCE_LINEAR_EPSILON)
					covariances[i].at<double>(0,0) = COVARIANCE_LINEAR_EPSILON; // epsilon if exact transform
				if(covariances[i].at<double>(1,1)<=COVARIANCE_LINEAR_EPSILON)
					covariances[i].at<double>(1,1) = COVARIANCE_LINEAR_EPSILON; // epsilon if exact transform
				if(covariances[i].at<double>(2,2)<=COVARIANCE_LINEAR_EPSILON)
					covariances[i].at<double>(2,2) = COVARIANCE_LINEAR_EPSILON; // epsilon if exact transform
				if(covariances[i].at<double>(3,3)<=COVARIANCE_ANGULAR_EPSILON)
					covariances[i].at<double>(3,3) = COVARIANCE_ANGULAR_EPSILON; // epsilon if exact transform
				if(covariances[i].at<double>(4,4)<=COVARIANCE_ANGULAR_EPSILON)
					covariances[i].at<double>(4,4) = COVARIANCE_ANGULAR_EPSILON; // epsilon if exact transform
				if(covariances[i].at<double>(5,5)<=COVARIANCE_ANGULAR_EPSILON)
					covariances[i].at<double>(5,5) = COVARIANCE_ANGULAR_EPSILON; // epsilon if exact transform
			}

			cv::Mat cov = covariances[0].clone();

			links.insert(std::make_pair(1, Link(1, 2, Link::kNeighbor, transforms[0], cov.inv())));
			if(!transforms[1].isNull() && inliers[1].size())
			{
				cov = covariances[1].clone();
				links.insert(std::make_pair(2, Link(2, 1, Link::kNeighbor, transforms[1], cov.inv())));
			}

			std::map<int, Transform> optimizedPoses;

			UASSERT((toSignature.sensorData().stereoCameraModels().size()  >= 1 && toSignature.sensorData().stereoCameraModels()[0].isValidForProjection()) ||
					(toSignature.sensorData().cameraModels().size() >= 1 && toSignature.sensorData().cameraModels()[0].isValidForProjection()));

			std::map<int, std::vector<CameraModel> > models;

			std::vector<CameraModel> cameraModelsFrom;
			if(fromSignature.sensorData().stereoCameraModels().size())
			{
				for(size_t i=0; i<fromSignature.sensorData().stereoCameraModels().size(); ++i)
				{
					CameraModel cameraModel = fromSignature.sensorData().stereoCameraModels()[i].left();
					// Set Tx=-baseline*fx for Stereo BA
					cameraModel = CameraModel(cameraModel.fx(),
							cameraModel.fy(),
							cameraModel.cx(),
							cameraModel.cy(),
							cameraModel.localTransform(),
							-fromSignature.sensorData().stereoCameraModels()[0].baseline()*cameraModel.fx(),
							cameraModel.imageSize());
					cameraModelsFrom.push_back(cameraModel);
				}
			}
			else
			{
				cameraModelsFrom = fromSignature.sensorData().cameraModels();
			}

			std::vector<CameraModel> cameraModelsTo;
			if(toSignature.sensorData().stereoCameraModels().size())
			{
				for(size_t i=0; i<toSignature.sensorData().stereoCameraModels().size(); ++i)
				{
					CameraModel cameraModel = toSignature.sensorData().stereoCameraModels()[i].left();
					// Set Tx=-baseline*fx for Stereo BA
					cameraModel = CameraModel(cameraModel.fx(),
							cameraModel.fy(),
							cameraModel.cx(),
							cameraModel.cy(),
							cameraModel.localTransform(),
							-toSignature.sensorData().stereoCameraModels()[0].baseline()*cameraModel.fx(),
							cameraModel.imageSize());
					cameraModelsTo.push_back(cameraModel);
				}
			}
			else
			{
				cameraModelsTo = toSignature.sensorData().cameraModels();
			}

			models.insert(std::make_pair(1, cameraModelsFrom));
			models.insert(std::make_pair(2, cameraModelsTo));

			std::map<int, std::map<int, FeatureBA> > wordReferences;
			std::set<int> sbaOutliers;
			UDEBUG("");
			for(unsigned int i=0; i<allInliers.size(); ++i)
			{
				int wordId = allInliers[i];
				int indexFrom = fromSignature.getWords().find(wordId)->second;
				const cv::Point3f & pt3D = fromSignature.getWords3()[indexFrom];
				if(!util3d::isFinite(pt3D))
				{
					UASSERT_MSG(!_forwardEstimateOnly, uFormat("3D point %d is not finite!?", wordId).c_str());
					sbaOutliers.insert(wordId);
					continue;
				}

				points3DMap.insert(std::make_pair(wordId, pt3D));

				std::map<int, FeatureBA> ptMap;
				if(!fromSignature.getWordsKpts().empty())
				{
					cv::KeyPoint kpt = fromSignature.getWordsKpts()[indexFrom];

					int cameraIndex = 0;
					const std::vector<CameraModel> & cam = models.at(1);
					if(cam.size()>1)
					{
						UASSERT(cam[0].imageWidth()>0);
						float subImageWidth = cam[0].imageWidth();
						cameraIndex = int(kpt.pt.x / subImageWidth);
						kpt.pt.x = kpt.pt.x - (subImageWidth*float(cameraIndex));
					}

					UASSERT(cam[cameraIndex].isValidForProjection());

					float depthFrom = util3d::transformPoint(pt3D, cam[cameraIndex].localTransform().inverse()).z;
					ptMap.insert(std::make_pair(1,FeatureBA(kpt, depthFrom, cv::Mat(), cameraIndex)));
				}

				if(!toSignature.getWordsKpts().empty())
				{
					int indexTo = toSignature.getWords().find(wordId)->second;
					cv::KeyPoint kpt = toSignature.getWordsKpts()[indexTo];

					int cameraIndex = 0;
					const std::vector<CameraModel> & cam = models.at(2);
					if(cam.size()>1)
					{
						UASSERT(cam[0].imageWidth()>0);
						float subImageWidth = cam[0].imageWidth();
						cameraIndex = int(kpt.pt.x / subImageWidth);
						kpt.pt.x = kpt.pt.x - (subImageWidth*float(cameraIndex));
					}

					UASSERT(cam[cameraIndex].isValidForProjection());

					float depthTo = 0.0f;
					if(!toSignature.getWords3().empty())
					{
						depthTo = util3d::transformPoint(toSignature.getWords3()[indexTo], cam[cameraIndex].localTransform().inverse()).z;
					}

					ptMap.insert(std::make_pair(2,FeatureBA(kpt, depthTo, cv::Mat(), cameraIndex)));
				}

				wordReferences.insert(std::make_pair(wordId, ptMap));

				//UDEBUG("%d (%f,%f,%f)", wordId, points3DMap.at(wordId).x, points3DMap.at(wordId).y, points3DMap.at(wordId).z);
				//for(std::map<int, cv::Point3f>::iterator iter=ptMap.begin(); iter!=ptMap.end(); ++iter)
				//{
				//	UDEBUG("%d (%f,%f) d=%f", iter->first, iter->second.x, iter->second.y, iter->second.z);
				//}
			}

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

		if(!transform.isNull() && !allInliers.empty() && (_minInliersDistributionThr>0.0f || _maxInliersMeanDistance>0.0f))
		{
			cv::Mat pcaData;
			std::vector<CameraModel> cameraModelsTo;
			if(toSignature.sensorData().stereoCameraModels().size())
			{
				for(size_t i=0; i<toSignature.sensorData().stereoCameraModels().size(); ++i)
				{
					cameraModelsTo.push_back(toSignature.sensorData().stereoCameraModels()[i].left());
				}
			}
			else
			{
				cameraModelsTo = toSignature.sensorData().cameraModels();
			}
			if(_minInliersDistributionThr > 0)
			{
				if(cameraModelsTo.size() >= 1 && cameraModelsTo[0].isValidForReprojection())
				{
					if(cameraModelsTo[0].imageWidth()>0 && cameraModelsTo[0].imageHeight()>0)
					{
						pcaData = cv::Mat(allInliers.size(), 2, CV_32FC1);
					}
					else
					{
						UERROR("Invalid calibration image size (%dx%d), cannot compute inliers distribution! (see %s=%f)", cameraModelsTo[0].imageWidth(), cameraModelsTo[0].imageHeight(), Parameters::kVisMinInliersDistribution().c_str(), _minInliersDistributionThr);
					}
				}
				else
				{
					UERROR("Calibration not valid, cannot compute inliers distribution! (see %s=%f)", Parameters::kVisMinInliersDistribution().c_str(), _minInliersDistributionThr);
				}
			}

			Transform transformInv = transform.inverse();
			std::vector<float> distances;
			if(_maxInliersMeanDistance>0.0f)
			{
				distances.reserve(allInliers.size());
			}
			for(unsigned int i=0; i<allInliers.size(); ++i)
			{
				std::multimap<int, int>::const_iterator wordsIter = toSignature.getWords().find(allInliers[i]);
				if(wordsIter != toSignature.getWords().end() && !toSignature.getWordsKpts().empty())
				{
					const cv::KeyPoint & kpt = toSignature.getWordsKpts()[wordsIter->second];
					int cameraIndex = (int)(kpt.pt.x / cameraModelsTo[0].imageWidth());
					UASSERT_MSG(cameraIndex < (int)cameraModelsTo.size(), uFormat("cameraIndex=%d (x=%f models=%d camera width = %d)", cameraIndex, kpt.pt.x, (int)cameraModelsTo.size(), cameraModelsTo[0].imageWidth()).c_str());

					if(_maxInliersMeanDistance>0.0f && !toSignature.getWords3().empty())
					{
						const cv::Point3f & pt = toSignature.getWords3()[wordsIter->second];
						if(util3d::isFinite(pt))
						{
							UASSERT(cameraModelsTo[cameraIndex].isValidForProjection());

							float depth = util3d::transformPoint(pt, cameraModelsTo[cameraIndex].localTransform().inverse()).z;
							distances.push_back(depth);
						}
					}

					if(!pcaData.empty())
					{
						float * ptr = pcaData.ptr<float>(i, 0);
						ptr[0] = (kpt.pt.x-cameraIndex*cameraModelsTo[cameraIndex].imageWidth()-cameraModelsTo[cameraIndex].cx()) / cameraModelsTo[cameraIndex].imageWidth();
						ptr[1] = (kpt.pt.y-cameraModelsTo[cameraIndex].cy()) / cameraModelsTo[cameraIndex].imageHeight();
					}
				}
			}

			if(!distances.empty())
			{
				info.inliersMeanDistance = uMean(distances);

				if(info.inliersMeanDistance > _maxInliersMeanDistance)
				{
					msg = uFormat("The mean distance of the inliers is over %s threshold (%f)",
							info.inliersMeanDistance, Parameters::kVisMeanInliersDistance().c_str(), _maxInliersMeanDistance);
					transform.setNull();
				}
			}

			if(!transform.isNull() && !pcaData.empty())
			{
				cv::Mat pcaEigenVectors, pcaEigenValues;
				cv::PCA pca_analysis(pcaData, cv::Mat(), CV_PCA_DATA_AS_ROW);
				// We take the second eigen value
				info.inliersDistribution = pca_analysis.eigenvalues.at<float>(0, 1);

				if(info.inliersDistribution < _minInliersDistributionThr)
				{
					msg = uFormat("The distribution (%f) of inliers is under %s threshold (%f)",
							info.inliersDistribution, Parameters::kVisMinInliersDistribution().c_str(), _minInliersDistributionThr);
					transform.setNull();
				}
			}
		}
	}
	else if(toSignature.sensorData().isValid())
	{
		msg = uFormat("Missing correspondences for registration (%d->%d). fromWords = %d fromImageEmpty=%d toWords = %d toImageEmpty=%d",
				fromSignature.id(), toSignature.id(),
				(int)fromSignature.getWords().size(), fromSignature.sensorData().imageRaw().empty()?1:0,
				(int)toSignature.getWords().size(), toSignature.sensorData().imageRaw().empty()?1:0);
	}

	info.inliers = inliersCount;
	info.inliersRatio = !toSignature.getWords().empty()?float(inliersCount)/float(toSignature.getWords().size()):0;
	info.matches = matchesCount;
	info.rejectedMsg = msg;
	info.covariance = covariance;

	UDEBUG("inliers=%d/%d", info.inliers, info.matches);
	UDEBUG("transform=%s", transform.prettyPrint().c_str());
	return transform;
}

}
