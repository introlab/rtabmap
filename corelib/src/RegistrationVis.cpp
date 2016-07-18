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
		_guessWinSize(Parameters::defaultVisCorGuessWinSize())
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

	UDEBUG("Input(%d): from=%d words, %d 3D words, %d words descriptors,  %d kpts, %d descriptors",
			fromSignature.id(),
			(int)fromSignature.getWords().size(),
			(int)fromSignature.getWords3().size(),
			(int)fromSignature.getWordsDescriptors().size(),
			(int)fromSignature.sensorData().keypoints().size(),
			fromSignature.sensorData().descriptors().rows);

	UDEBUG("Input(%d): to=%d words, %d 3D words, %d words descriptors, %d kpts, %d descriptors",
			toSignature.id(),
			(int)toSignature.getWords().size(),
			(int)toSignature.getWords3().size(),
			(int)toSignature.getWordsDescriptors().size(),
			(int)toSignature.sensorData().keypoints().size(),
			toSignature.sensorData().descriptors().rows);

	std::string msg;

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
		UDEBUG("");
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
					if(!fromSignature.sensorData().depthRaw().empty())
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
			kptsFrom = uValues(fromSignature.getWords());
		}

		std::multimap<int, cv::KeyPoint> wordsFrom;
		std::multimap<int, cv::KeyPoint> wordsTo;
		std::multimap<int, cv::Point3f> words3From;
		std::multimap<int, cv::Point3f> words3To;
		std::multimap<int, cv::Mat> wordsDescFrom;
		std::multimap<int, cv::Mat> wordsDescTo;
		if(_correspondencesApproach == 1 && //Optical Flow
		   !imageFrom.empty() &&
		   !imageTo.empty())
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
			if(fromSignature.getWords3().empty())
			{
				kptsFrom3D = detector->generateKeypoints3D(fromSignature.sensorData(), kptsFrom);
			}
			else
			{
				kptsFrom3D = uValues(fromSignature.getWords3());
			}

			if(!imageTo.empty())
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
				int ki = 0;
				for(unsigned int i=0; i<status.size(); ++i)
				{
					if(status[i] &&
					   uIsInBounds(cornersTo[i].x, 0.0f, float(imageTo.cols)) &&
					   uIsInBounds(cornersTo[i].y, 0.0f, float(imageTo.rows)))
					{
						kptsFrom[ki] = cv::KeyPoint(cornersFrom[i], 1);
						kptsFrom3DKept[ki] = kptsFrom3D[i];
						kptsTo[ki++] = cv::KeyPoint(cornersTo[i], 1);
					}
				}
				kptsFrom.resize(ki);
				kptsTo.resize(ki);
				kptsFrom3DKept.resize(ki);

				std::vector<cv::Point3f> kptsTo3D;
				if(_estimationType == 0 || (_estimationType == 1 && !varianceFromInliersCount()) || !_forwardEstimateOnly)
				{
					kptsTo3D = detector->generateKeypoints3D(toSignature.sensorData(), kptsTo);
				}

				UASSERT(kptsFrom.size() == kptsFrom3DKept.size());
				UASSERT(kptsFrom.size() == kptsTo.size());
				UASSERT(kptsTo3D.size() == 0 || kptsTo.size() == kptsTo3D.size());
				for(unsigned int i=0; i< kptsFrom3DKept.size(); ++i)
				{
					wordsFrom.insert(std::make_pair(i, kptsFrom[i]));
					words3From.insert(std::make_pair(i, kptsFrom3DKept[i]));
					wordsTo.insert(std::make_pair(i, kptsTo[i]));
					if(kptsTo3D.size())
					{
						words3To.insert(std::make_pair(i, kptsTo3D[i]));
					}
				}
				toSignature.sensorData().setFeatures(kptsTo, cv::Mat());
			}
			else
			{
				UASSERT(kptsFrom.size() == kptsFrom3D.size());
				for(unsigned int i=0; i< kptsFrom3D.size(); ++i)
				{
					if(util3d::isFinite(kptsFrom3D[i]))
					{
						wordsFrom.insert(std::make_pair(i, kptsFrom[i]));
						words3From.insert(std::make_pair(i, kptsFrom3D[i]));
					}
				}
				toSignature.sensorData().setFeatures(std::vector<cv::KeyPoint>(), cv::Mat());
			}

			fromSignature.sensorData().setFeatures(kptsFrom, cv::Mat());
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
					if(!toSignature.sensorData().depthRaw().empty())
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
			if((kptsFrom.empty() && fromSignature.getWordsDescriptors().size()) ||
				fromSignature.getWordsDescriptors().size() == kptsFrom.size())
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
			if(fromSignature.getWords3().empty() || kptsFrom.size() != fromSignature.getWords3().size())
			{
				if(fromSignature.getWords3().size() && kptsFrom.size() != fromSignature.getWords3().size())
				{
					UWARN("kptsFrom (%d) is not the same size as fromSignature.getWords3() (%d), there "
						   "is maybe a problem with the logic above (getWords3() should be null or equal to kptsfrom).");
				}
				kptsFrom3D = detector->generateKeypoints3D(fromSignature.sensorData(), kptsFrom);
				if(detector->getMinDepth() > 0.0f || detector->getMaxDepth() > 0.0f)
				{
					UDEBUG("");
					//remove all keypoints/descriptors with no valid 3D points
					UASSERT((int)kptsFrom.size() == descriptorsFrom.rows &&
							kptsFrom3D.size() == kptsFrom.size());
					std::vector<cv::KeyPoint> validKeypoints(kptsFrom.size());
					std::vector<cv::Point3f> validKeypoints3D(kptsFrom.size());
					cv::Mat validDescriptors(descriptorsFrom.size(), descriptorsFrom.type());

					int oi=0;
					for(unsigned int i=0; i<kptsFrom3D.size(); ++i)
					{
						if(util3d::isFinite(kptsFrom3D[i]))
						{
							validKeypoints[oi] = kptsFrom[i];
							validKeypoints3D[oi] = kptsFrom3D[i];
							descriptorsFrom.row(i).copyTo(validDescriptors.row(oi));
							++oi;
						}
					}
					UDEBUG("Removed %d invalid 3D points", (int)kptsFrom3D.size()-oi);
					validKeypoints.resize(oi);
					validKeypoints3D.resize(oi);
					kptsFrom = validKeypoints;
					kptsFrom3D = validKeypoints3D;
					descriptorsFrom = validDescriptors.rowRange(0, oi).clone();
				}
			}
			else
			{
				kptsFrom3D = uValues(fromSignature.getWords3());
			}
			if(toSignature.getWords3().empty() || kptsTo.size() != toSignature.getWords3().size())
			{
				if(toSignature.getWords3().size() && kptsTo.size() != toSignature.getWords3().size())
				{
					UWARN("kptsTo (%d) is not the same size as toSignature.getWords3() (%d), there "
						   "is maybe a problem with the logic above (getWords3() should be null or equal to kptsTo).");
				}
				kptsTo3D = detector->generateKeypoints3D(toSignature.sensorData(), kptsTo);
				if(detector->getMinDepth() > 0.0f || detector->getMaxDepth() > 0.0f)
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
			else
			{
				kptsTo3D = uValues(toSignature.getWords3());
			}

			UASSERT(kptsFrom.empty() || int(kptsFrom.size()) == descriptorsFrom.rows);

			fromSignature.sensorData().setFeatures(kptsFrom, descriptorsFrom);
			toSignature.sensorData().setFeatures(kptsTo, descriptorsTo);

			UDEBUG("descriptorsFrom=%d", descriptorsFrom.rows);
			UDEBUG("descriptorsTo=%d", descriptorsTo.rows);

			// We have all data we need here, so match!
			if(descriptorsFrom.rows > 0 && descriptorsTo.rows > 0)
			{
				cv::Size imageSize = imageTo.size();
				bool isCalibrated = false; // multiple cameras not supported.
				if(imageSize.height == 0 || imageSize.width == 0)
				{
					imageSize = fromSignature.sensorData().cameraModels().size() == 1?fromSignature.sensorData().cameraModels()[0].imageSize():fromSignature.sensorData().stereoCameraModel().left().imageSize();
				}
				isCalibrated = imageSize.height != 0 && imageSize.width != 0 && fromSignature.sensorData().cameraModels().size()==1?fromSignature.sensorData().cameraModels()[0].isValidForProjection():fromSignature.sensorData().stereoCameraModel().isValidForProjection();

				// If guess is set, limit the search of matches using optical flow window size
				bool guessSet = !guess.isIdentity() && !guess.isNull();
				if(guessSet && _guessWinSize > 0 && kptsFrom3D.size() &&
						isCalibrated) // needed for projection
				{
					UDEBUG("");
					UASSERT((int)kptsTo.size() == descriptorsTo.rows);
					UASSERT((int)kptsFrom3D.size() == descriptorsFrom.rows);

					// Use guess to project 3D "from" keypoints into "to" image
					if(fromSignature.sensorData().cameraModels().size() > 1)
					{
						UFATAL("Guess reprojection feature matching is not supported for multiple cameras.");
					}

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
					std::vector<cv::Point2f> projected;
					cv::projectPoints(kptsFrom3D, rvec, tvec, K, cv::Mat(), projected);

					//remove projected points outside of the image
					UASSERT((int)projected.size() == descriptorsFrom.rows);
					std::vector<cv::Point2f> cornersProjected(projected.size());
					std::vector<int> projectedIndexToDescIndex(projected.size());
					int oi=0;
					for(unsigned int i=0; i<projected.size(); ++i)
					{
						if(uIsInBounds(projected[i].x, 0.0f, float(imageSize.width-1)) &&
						   uIsInBounds(projected[i].y, 0.0f, float(imageSize.height-1)))
						{
							projectedIndexToDescIndex[oi] = i;
							cornersProjected[oi++] = projected[i];
						}
					}
					projectedIndexToDescIndex.resize(oi);
					cornersProjected.resize(oi);



					UDEBUG("cornersProjected=%d", (int)cornersProjected.size());

					// For each projected feature guess of "from" in "to", find its matching feature in
					// the radius around the projected guess.
					// TODO: do cross-check?
					if(cornersProjected.size())
					{

						// Create kd-tree for projected keypoints
						rtflann::Matrix<float> cornersProjectedMat((float*)cornersProjected.data(), cornersProjected.size(), 2);
						rtflann::Index<rtflann::L2<float> > index(cornersProjectedMat, rtflann::KDTreeIndexParams());
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
						UDEBUG("");

						// Process results (Nearest Neighbor Distance Ratio)
						int matchedID = descriptorsFrom.rows+descriptorsTo.rows;
						int newToId = descriptorsFrom.rows;
						int notMatchedFromId = 0;
						std::map<int,int> addedWordsFrom; //<id, index>
						std::map<int, int> duplicates; //<fromId, toId>
						int newWords = 0;
						for(unsigned int i = 0; i < pointsToMat.rows; ++i)
						{
							if(kptsTo3D.empty() || util3d::isFinite(kptsTo3D[i]))
							{
								int octave = kptsTo[i].octave;
								int matchedIndex = -1;
								if(indices[i].size() >= 2)
								{
									cv::Mat descriptors;
									std::vector<int> descriptorsIndices(indices[i].size());
									int oi=0;
									for(unsigned int j=0; j<indices[i].size(); ++j)
									{
										if(kptsFrom.at(projectedIndexToDescIndex[indices[i].at(j)]).octave>=octave-1 &&
										   kptsFrom.at(projectedIndexToDescIndex[indices[i].at(j)]).octave<=octave+1)
										{
											descriptors.push_back(descriptorsFrom.row(projectedIndexToDescIndex[indices[i].at(j)]));
											descriptorsIndices[oi++] = indices[i].at(j);
										}
									}
									descriptorsIndices.resize(oi);
									if(oi >=2)
									{
										std::vector<std::vector<cv::DMatch> > matches;
										cv::BFMatcher matcher(descriptors.type()==CV_8U?cv::NORM_HAMMING:cv::NORM_L2SQR);
										matcher.knnMatch(descriptorsTo.row(i), descriptors, matches, 2);
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
								else if(indices[i].size() == 1 &&
									    kptsFrom.at(projectedIndexToDescIndex[indices[i].at(0)]).octave >= octave-1 &&
										kptsFrom.at(projectedIndexToDescIndex[indices[i].at(0)]).octave <= octave+1)
								{
									matchedIndex = indices[i].at(0);
								}

								if(matchedIndex >= 0)
								{
									matchedIndex = projectedIndexToDescIndex[matchedIndex];
									int id = matchedID++;

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
									wordsTo.insert(std::make_pair(newToId, kptsTo[i]));
									wordsDescTo.insert(std::make_pair(newToId, descriptorsTo.row(i)));
									if(kptsTo3D.size())
									{
										words3To.insert(std::make_pair(newToId, kptsTo3D[i]));
									}

									++newToId;
									++newWords;
								}
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
								wordsFrom.insert(std::make_pair(notMatchedFromId, kptsFrom[i]));
								wordsDescFrom.insert(std::make_pair(notMatchedFromId, descriptorsFrom.row(i)));
								words3From.insert(std::make_pair(notMatchedFromId, kptsFrom3D[i]));

								++notMatchedFromId;
								++addWordsFromNotMatched;
							}
						}
						UDEBUG("addWordsFromNotMatched=%d -> words3From=%d", addWordsFromNotMatched, (int)words3From.size());

						/*std::vector<cv::KeyPoint> matches(wordsTo.size());
						int oi=0;
						for(std::multimap<int, cv::KeyPoint>::iterator iter = wordsTo.begin(); iter!=wordsTo.end(); ++iter)
						{
							if(iter->first >= descriptorsFrom.rows+descriptorsTo.rows && wordsTo.count(iter->first) <= 1)
							{
								matches[oi++] = iter->second;
							}
						}
						matches.resize(oi);
						UDEBUG("guess=%s", guess.prettyPrint().c_str());
						std::vector<cv::KeyPoint> projectedKpts;
						cv::KeyPoint::convert(projected, projectedKpts);
						cv::Mat image = toSignature.sensorData().imageRaw().clone();
						drawKeypoints(image, projectedKpts, image, cv::Scalar(255,0,0));
						drawKeypoints(image, kptsTo, image, cv::Scalar(0,0,255));
						drawKeypoints(image, matches, image, cv::Scalar(0,255,0));
						cv::imwrite("projected.bmp", image);
						UWARN("saved projected.bmp");*/

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
					std::list<int> fromWordIds = dictionary.addNewWords(descriptorsFrom, 1);
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
							wordsFrom.insert(std::make_pair(*iter, kptsFrom[i]));
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
	float variance = 1.0f;
	int inliersCount = 0;
	int matchesCount = 0;
	if(toSignature.getWords().size())
	{
		Transform transforms[2];
		std::vector<int> inliers[2];
		std::vector<int> matches[2];
		double variances[2] = {1.0f};
		for(int dir=0; dir<(!_forwardEstimateOnly?2:1); ++dir)
		{
			// A to B
			const Signature * signatureA;
			const Signature * signatureB;
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
							&variances[dir]);

					inliers[dir] = uKeys(inliers3D);

					if(!cameraTransform.isNull())
					{
						if((int)inliers3D.size() >= _minInliers)
						{
							if(variances[dir] <= _epipolarGeometryVar)
							{
								transforms[dir] = cameraTransform;
							}
							else
							{
								msg = uFormat("Variance is too high! (max inlier distance=%f, variance=%f)", _epipolarGeometryVar, variances[dir]);
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
								varianceFromInliersCount()?0:&variances[dir],
								&matchesV,
								&inliersV);
						inliers[dir] = inliersV;
						matches[dir] = matchesV;
						if(transforms[dir].isNull())
						{
							msg = uFormat("Not enough inliers %d/%d (matches=%d) between %d and %d",
									(int)inliers[dir].size(), _minInliers, (int)matches[dir].size(), signatureA->id(), signatureB->id());
							UINFO(msg.c_str());
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
							&variances[dir],
							&matchesV,
							&inliersV);
					inliers[dir] = inliersV;
					matches[dir] = matchesV;
					if(transforms[dir].isNull())
					{
						msg = uFormat("Not enough inliers %d/%d between %d and %d",
								(int)inliers[dir].size(), _minInliers, signatureA->id(), signatureB->id());
						UINFO(msg.c_str());
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

		if(!transforms[1].isNull())
		{
			transforms[1] = transforms[1].inverse();
		}

		if(!_forwardEstimateOnly)
		{
			UDEBUG("from->to=%s", transforms[0].prettyPrint().c_str());
			UDEBUG("from->from=%s", transforms[1].prettyPrint().c_str());
		}
		if(!transforms[1].isNull())
		{
			if(transforms[0].isNull())
			{
				transform = transforms[1];
				info.inliersIDs = inliers[1];
				info.matchesIDs = matches[1];

				variance = variances[1];
				inliersCount = (int)inliers[1].size();
				matchesCount = (int)matches[1].size();
			}
			else
			{
				transform = transforms[0].interpolate(0.5f, transforms[1]);
				info.inliersIDs = inliers[0];
				info.matchesIDs = matches[0];

				variance = (variances[0]+variances[1])/2.0f;
				inliersCount = (int)(inliers[0].size()+inliers[1].size())/2;
				matchesCount = (int)(matches[0].size()+matches[1].size())/2;
			}
		}
		else
		{
			transform = transforms[0];
			info.inliersIDs = inliers[0];
			info.matchesIDs = matches[0];

			variance = variances[0];
			inliersCount = (int)inliers[0].size();
			matchesCount = (int)matches[0].size();
		}
	}
	else if(toSignature.sensorData().isValid())
	{
		UWARN("Missing correspondences for registration. toWords = %d toImageEmpty=%d",
				(int)toSignature.getWords().size(), toSignature.sensorData().imageRaw().empty()?1:0);
	}

	info.inliers = inliersCount;
	info.matches = matchesCount;
	info.rejectedMsg = msg;
	info.variance = variance>0.0f?variance:0.0001f; // epsilon if exact transform

	UDEBUG("transform=%s", transform.prettyPrint().c_str());
	return transform;
}

}
