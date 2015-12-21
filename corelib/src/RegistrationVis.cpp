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



#include <rtabmap/core/RegistrationVis.h>
#include <rtabmap/core/util3d_motion_estimation.h>
#include <rtabmap/core/util3d_features.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/core/VWDictionary.h>
#include <rtabmap/core/Features2d.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/utilite/UMath.h>

namespace rtabmap {

RegistrationVis::RegistrationVis(const ParametersMap & parameters) :
		_minInliers(Parameters::defaultVisMinInliers()),
		_inlierDistance(Parameters::defaultVisInlierDistance()),
		_iterations(Parameters::defaultVisIterations()),
		_refineIterations(Parameters::defaultVisRefineIterations()),
		_force2D(Parameters::defaultVisForce2D()),
		_epipolarGeometryVar(Parameters::defaultVisEpipolarGeometryVar()),
		_estimationType(Parameters::defaultVisEstimationType()),
		_forwardEstimateOnly(Parameters::defaultVisForwardEstOnly()),
		_PnPReprojError(Parameters::defaultVisPnPReprojError()),
		_PnPFlags(Parameters::defaultVisPnPFlags()),
		_PnPOpenCV2(Parameters::defaultVisPnPOpenCV2()),
		_correspondencesApproach(Parameters::defaultVisCorType()),
		_flowWinSize(Parameters::defaultVisCorFlowWinSize()),
		_flowIterations(Parameters::defaultVisCorFlowIterations()),
		_flowEps(Parameters::defaultVisCorFlowEps()),
		_flowMaxLevel(Parameters::defaultVisCorFlowMaxLevel())
{
	_featureParameters = Parameters::getDefaultParameters();
	uInsert(_featureParameters, ParametersPair(Parameters::kMemIncrementalMemory(), "true")); // make sure it is incremental
	uInsert(_featureParameters, ParametersPair(Parameters::kMemRehearsalSimilarity(), "1.0")); // desactivate rehearsal
	uInsert(_featureParameters, ParametersPair(Parameters::kMemBinDataKept(), "false"));
	uInsert(_featureParameters, ParametersPair(Parameters::kMemSTMSize(), "0"));
	uInsert(_featureParameters, ParametersPair(Parameters::kKpIncrementalDictionary(), "true")); // make sure it is incremental
	uInsert(_featureParameters, ParametersPair(Parameters::kKpNewWordsComparedTogether(), "false"));
	uInsert(_featureParameters, ParametersPair(Parameters::kKpBadSignRatio(), "0"));
	uInsert(_featureParameters, ParametersPair(Parameters::kMemGenerateIds(), "true"));

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

	this->parseParameters(parameters);
}

void RegistrationVis::parseParameters(const ParametersMap & parameters)
{
	Registration::parseParameters(parameters);

	Parameters::parse(parameters, Parameters::kVisMinInliers(), _minInliers);
	Parameters::parse(parameters, Parameters::kVisInlierDistance(), _inlierDistance);
	Parameters::parse(parameters, Parameters::kVisIterations(), _iterations);
	Parameters::parse(parameters, Parameters::kVisRefineIterations(), _refineIterations);
	Parameters::parse(parameters, Parameters::kVisForce2D(), _force2D);
	Parameters::parse(parameters, Parameters::kVisEstimationType(), _estimationType);
	Parameters::parse(parameters, Parameters::kVisEpipolarGeometryVar(), _epipolarGeometryVar);
	Parameters::parse(parameters, Parameters::kVisPnPReprojError(), _PnPReprojError);
	Parameters::parse(parameters, Parameters::kVisPnPFlags(), _PnPFlags);
	Parameters::parse(parameters, Parameters::kVisPnPOpenCV2(), _PnPOpenCV2);
	Parameters::parse(parameters, Parameters::kVisCorType(), _correspondencesApproach);
	Parameters::parse(parameters, Parameters::kVisCorFlowWinSize(), _flowWinSize);
	Parameters::parse(parameters, Parameters::kVisCorFlowIterations(), _flowIterations);
	Parameters::parse(parameters, Parameters::kVisCorFlowEps(), _flowEps);
	Parameters::parse(parameters, Parameters::kVisCorFlowMaxLevel(), _flowMaxLevel);

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

Transform RegistrationVis::computeTransformationMod(
			Signature & fromSignature,
			Signature & toSignature,
			Transform guess, // guess is only used by Optical Flow correspondences (flowMaxLevel is set to 0 when guess is used)
			std::string * rejectedMsg,
			std::vector<int> * inliersOut,
			float * varianceOut,
			float * inliersRatioOut) const
{
	UDEBUG("%s=%d", Parameters::kVisMinInliers().c_str(), _minInliers);
	UDEBUG("%s=%f", Parameters::kVisInlierDistance().c_str(), _inlierDistance);
	UDEBUG("%s=%d", Parameters::kVisIterations().c_str(), _iterations);
	UDEBUG("%s=%d", Parameters::kVisForce2D().c_str(), _force2D?1:0);
	UDEBUG("%s=%d", Parameters::kVisEstimationType().c_str(), _estimationType);
	UDEBUG("%s=%d", Parameters::kVisForwardEstOnly().c_str(), _forwardEstimateOnly);
	UDEBUG("%s=%f", Parameters::kVisEpipolarGeometryVar().c_str(), _epipolarGeometryVar);
	UDEBUG("%s=%f", Parameters::kVisPnPReprojError().c_str(), _PnPReprojError);
	UDEBUG("%s=%d", Parameters::kVisPnPFlags().c_str(), _PnPFlags);
	UDEBUG("%s=%d", Parameters::kVisCorType().c_str(), _correspondencesApproach?1:0);
	UDEBUG("%s=%d", Parameters::kVisCorFlowWinSize().c_str(), _flowWinSize);
	UDEBUG("%s=%d", Parameters::kVisCorFlowIterations().c_str(), _flowIterations);
	UDEBUG("%s=%f", Parameters::kVisCorFlowEps().c_str(), _flowEps);
	UDEBUG("%s=%d", Parameters::kVisCorFlowMaxLevel().c_str(), _flowMaxLevel);

	UDEBUG("Input(%d): from=%d words, %d 3D words, %d kpts, %d descriptors",
			fromSignature.id(),
			(int)fromSignature.getWords().size(),
			(int)fromSignature.getWords3().size(),
			(int)fromSignature.sensorData().keypoints().size(),
			fromSignature.sensorData().descriptors().rows);

	UDEBUG("Input(%d): to=%d words, %d 3D words, %d kpts, %d descriptors",
			toSignature.id(),
			(int)toSignature.getWords().size(),
			(int)toSignature.getWords3().size(),
			(int)toSignature.sensorData().keypoints().size(),
			toSignature.sensorData().descriptors().rows);

	std::string msg;

	////////////////////
	// Find correspondences
	////////////////////
	if(fromSignature.getWords().size() && fromSignature.getWords3().size() &&
	   toSignature.getWords().size() && (_estimationType==1 || toSignature.getWords3().size()))
	{
		// no need to extract new features, we have all the data we need
		UDEBUG("");
	}
	else
	{
		UDEBUG("");
		// just some checks to make sure that input data are ok
		UASSERT((fromSignature.getWords().empty() && fromSignature.getWords3().empty())||
				(fromSignature.getWords().size() == fromSignature.getWords3().size()));
		UASSERT(fromSignature.sensorData().keypoints().size() == fromSignature.sensorData().descriptors().rows ||
				fromSignature.sensorData().descriptors().rows == 0);
		UASSERT((toSignature.getWords().empty() && toSignature.getWords3().empty())||
				(toSignature.getWords().size() == toSignature.getWords3().size()));
		UASSERT(toSignature.sensorData().keypoints().size() == toSignature.sensorData().descriptors().rows ||
				toSignature.sensorData().descriptors().rows == 0);
		UASSERT(fromSignature.sensorData().imageRaw().type() == CV_8UC1 ||
				fromSignature.sensorData().imageRaw().type() == CV_8UC3);
		UASSERT(toSignature.sensorData().imageRaw().type() == CV_8UC1 ||
				toSignature.sensorData().imageRaw().type() == CV_8UC3);

		Feature2D * detector = Feature2D::create(_featureParameters);
		std::vector<cv::KeyPoint> kptsFrom;
		if(fromSignature.getWords().empty())
		{
			if(fromSignature.sensorData().keypoints().empty())
			{
				if(fromSignature.sensorData().imageRaw().channels() > 1)
				{
					cv::Mat tmp;
					cv::cvtColor(fromSignature.sensorData().imageRaw(), tmp, cv::COLOR_BGR2GRAY);
					fromSignature.sensorData().setImageRaw(tmp);
				}

				kptsFrom = detector->generateKeypoints(fromSignature.sensorData().imageRaw());
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
		if(_correspondencesApproach == 1) //Optical Flow
		{
			UDEBUG("");
			// convert to grayscale
			if(fromSignature.sensorData().imageRaw().channels() > 1)
			{
				cv::Mat tmp;
				cv::cvtColor(fromSignature.sensorData().imageRaw(), tmp, cv::COLOR_BGR2GRAY);
				fromSignature.sensorData().setImageRaw(tmp);
			}
			if(toSignature.sensorData().imageRaw().channels() > 1)
			{
				cv::Mat tmp;
				cv::cvtColor(toSignature.sensorData().imageRaw(), tmp, cv::COLOR_BGR2GRAY);
				toSignature.sensorData().setImageRaw(tmp);
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

			if(!toSignature.sensorData().imageRaw().empty())
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
				int winSize = _flowWinSize;
				cv::calcOpticalFlowPyrLK(
						fromSignature.sensorData().imageRaw(),
						toSignature.sensorData().imageRaw(),
						cornersFrom,
						cornersTo,
						status,
						err,
						cv::Size(winSize, winSize),
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
					   uIsInBounds(cornersTo[i].x, 0.0f, float(toSignature.sensorData().depthOrRightRaw().cols)) &&
					   uIsInBounds(cornersTo[i].y, 0.0f, float(toSignature.sensorData().depthOrRightRaw().rows)))
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
				if(_estimationType == 0 || (_estimationType == 1 && !_varianceFromInliersCount) || !_forwardEstimateOnly)
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
				   !toSignature.sensorData().imageRaw().empty())
				{
					if(toSignature.sensorData().imageRaw().channels() > 1)
					{
						cv::Mat tmp;
						cv::cvtColor(toSignature.sensorData().imageRaw(), tmp, cv::COLOR_BGR2GRAY);
						toSignature.sensorData().setImageRaw(tmp);
					}
					kptsTo = detector->generateKeypoints(toSignature.sensorData().imageRaw());
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
			cv::Mat descriptorsTo;
			if(fromSignature.getWords().empty() && fromSignature.sensorData().descriptors().rows == kptsFrom.size())
			{
				descriptorsFrom = fromSignature.sensorData().descriptors();
			}
			else
			{
				descriptorsFrom = detector->generateDescriptors(fromSignature.sensorData().imageRaw(), kptsFrom);
			}
			if(toSignature.getWords().empty() && toSignature.sensorData().descriptors().rows == kptsTo.size())
			{
				descriptorsTo = toSignature.sensorData().descriptors();
			}
			else if(!toSignature.sensorData().imageRaw().empty())
			{
				descriptorsTo = detector->generateDescriptors(toSignature.sensorData().imageRaw(), kptsTo);
			}

			// create 3D keypoints
			std::vector<cv::Point3f> kptsFrom3D;
			std::vector<cv::Point3f> kptsTo3D;
			if(fromSignature.getWords3().empty())
			{
				kptsFrom3D = detector->generateKeypoints3D(fromSignature.sensorData(), kptsFrom);
			}
			else
			{
				kptsFrom3D = uValues(fromSignature.getWords3());
			}
			if((_estimationType == 0 || (_estimationType == 1 && !_varianceFromInliersCount) || !_forwardEstimateOnly) &&
			   toSignature.getWords3().empty() &&
			   !toSignature.sensorData().imageRaw().empty())
			{
				kptsTo3D = detector->generateKeypoints3D(toSignature.sensorData(), kptsTo);
			}
			else
			{
				kptsTo3D = uValues(toSignature.getWords3());
			}

			// We have all data we need here, so match using the vocabulary
			UDEBUG("descriptorsFrom=%d", descriptorsFrom.rows);
			VWDictionary dictionary(_featureParameters);
			std::list<int> fromWordIds = dictionary.addNewWords(descriptorsFrom, 1);
			std::list<int> toWordIds;
			UDEBUG("descriptorsTo=%d", descriptorsTo.rows);
			if(descriptorsTo.rows)
			{
				dictionary.update();
				toWordIds = dictionary.addNewWords(descriptorsTo, 2);
			}
			dictionary.clear(false);

			std::multiset<int> fromWordIdsSet(fromWordIds.begin(), fromWordIds.end());
			std::multiset<int> toWordIdsSet(toWordIds.begin(), toWordIds.end());

			UASSERT(kptsFrom3D.size() == kptsFrom.size());
			UASSERT(fromWordIds.size() == kptsFrom.size());
			int i=0;
			for(std::list<int>::iterator iter=fromWordIds.begin(); iter!=fromWordIds.end(); ++iter)
			{
				if(fromWordIdsSet.count(*iter) == 1)
				{
					wordsFrom.insert(std::make_pair(*iter, kptsFrom[i]));
					words3From.insert(std::make_pair(*iter, kptsFrom3D[i]));
				}
				++i;
			}
			UASSERT(kptsTo3D.size() == 0 || kptsTo3D.size() == kptsTo.size());
			UASSERT(toWordIds.size() == kptsTo.size());
			i=0;
			for(std::list<int>::iterator iter=toWordIds.begin(); iter!=toWordIds.end(); ++iter)
			{
				if(toWordIdsSet.count(*iter) == 1)
				{
					wordsTo.insert(std::make_pair(*iter, kptsTo[i]));
					if(kptsTo3D.size())
					{
						words3To.insert(std::make_pair(*iter, kptsTo3D[i]));
					}
				}
				++i;
			}
			//remove doubles
			fromSignature.sensorData().setFeatures(kptsFrom, descriptorsFrom);
			toSignature.sensorData().setFeatures(kptsTo, descriptorsTo);
		}
		fromSignature.setWords(wordsFrom);
		fromSignature.setWords3(words3From);
		toSignature.setWords(wordsTo);
		toSignature.setWords3(words3To);
		delete detector;
	}

	/////////////////////
	// Motion estimation
	/////////////////////
	Transform transform;
	float variance = 1.0f;
	if(toSignature.getWords().size() || !toSignature.sensorData().imageRaw().empty())
	{
		Transform transforms[2];
		std::vector<int> inliers[2];
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
				if(!signatureB->sensorData().stereoCameraModel().isValid() &&
				   (signatureB->sensorData().cameraModels().size() != 1 ||
					!signatureB->sensorData().cameraModels()[0].isValid()))
				{
					UERROR("Calibrated camera required (multi-cameras not supported).");
				}
				else if((int)signatureA->getWords().size() >= _minInliers &&
						(int)signatureB->getWords().size() >= _minInliers)
				{
					UASSERT(signatureA->sensorData().stereoCameraModel().isValid() || (signatureA->sensorData().cameraModels().size() == 1 && signatureA->sensorData().cameraModels()[0].isValid()));
					const CameraModel & cameraModel = signatureA->sensorData().stereoCameraModel().isValid()?signatureA->sensorData().stereoCameraModel().left():signatureA->sensorData().cameraModels()[0];

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
							_PnPOpenCV2,
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
				if(!signatureB->sensorData().stereoCameraModel().isValid() &&
				   (signatureB->sensorData().cameraModels().size() != 1 ||
					!signatureB->sensorData().cameraModels()[0].isValid()))
				{
					UERROR("Calibrated camera required (multi-cameras not supported). Id=%d Models=%d StereoModel=%d weight=%d",
							signatureB->id(),
							(int)signatureB->sensorData().cameraModels().size(),
							signatureB->sensorData().stereoCameraModel().isValid()?1:0,
							signatureB->getWeight());
				}
				else
				{
					UDEBUG("words from3D=%d to2D=%d", (int)signatureA->getWords3().size(), (int)signatureB->getWords().size());
					// 3D to 2D
					if((int)signatureA->getWords3().size() >= _minInliers &&
					   (int)signatureB->getWords().size() >= _minInliers)
					{
						UASSERT(signatureB->sensorData().stereoCameraModel().isValid() || (signatureB->sensorData().cameraModels().size() == 1 && signatureB->sensorData().cameraModels()[0].isValid()));
						const CameraModel & cameraModel = signatureB->sensorData().stereoCameraModel().isValid()?signatureB->sensorData().stereoCameraModel().left():signatureB->sensorData().cameraModels()[0];

						std::vector<int> inliersV;
						transforms[dir] = util3d::estimateMotion3DTo2D(
								uMultimapToMapUnique(signatureA->getWords3()),
								uMultimapToMapUnique(signatureB->getWords()),
								cameraModel,
								_minInliers,
								_iterations,
								_PnPReprojError,
								_PnPFlags,
								_PnPOpenCV2,
								dir==0?(!guess.isNull()?guess:Transform::getIdentity()):!transforms[0].isNull()?transforms[0].inverse():(!guess.isNull()?guess.inverse():Transform::getIdentity()),
								uMultimapToMapUnique(signatureA->getWords3()),
								_varianceFromInliersCount?0:&variances[dir],
								0,
								&inliersV);
						inliers[dir] = inliersV;
						if(transforms[dir].isNull())
						{
							msg = uFormat("Not enough inliers %d/%d between %d and %d",
									(int)inliers[dir].size(), _minInliers, signatureA->id(), signatureB->id());
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
					transforms[dir] = util3d::estimateMotion3DTo3D(
							uMultimapToMapUnique(signatureA->getWords3()),
							uMultimapToMapUnique(signatureB->getWords3()),
							_minInliers,
							_inlierDistance,
							_iterations,
							_refineIterations,
							&variances[dir],
							0,
							&inliersV);
					inliers[dir] = inliersV;
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

		UDEBUG("t1=%s", transforms[0].prettyPrint().c_str());
		UDEBUG("t2=%s", transforms[1].prettyPrint().c_str());
		if(!transforms[1].isNull())
		{
			if(transforms[0].isNull())
			{
				transform = transforms[1];
				if(inliersOut)
				{
					*inliersOut = inliers[1];
				}

				variance = variances[1];
				if(_varianceFromInliersCount)
				{
					variance = inliers[1].size() > 0?1.0f/float(inliers[1].size()):1.0f;
				}
			}
			else
			{
				transform = transforms[0].interpolate(0.5f, transforms[1]);
				if(inliersOut)
				{
					*inliersOut = inliers[0];
				}
				variance = (variances[0]+variances[1])/2.0f;
				if(_varianceFromInliersCount)
				{
					int avg = (inliers[0].size()+inliers[1].size())/2;
					variance = avg>0?1.0f/float(avg):1.0f;
				}
			}
		}
		else
		{
			transform = transforms[0];
			if(inliersOut)
			{
				*inliersOut = inliers[0];
			}

			variance = variances[0];
			if(_varianceFromInliersCount)
			{
				variance = inliers[0].size() > 0?1.0f/float(inliers[0].size()):1.0f;
			}
		}
	}

	if(!transform.isNull())
	{
		UDEBUG("");
		// verify if it is a 180 degree transform, well verify > 90
		float x,y,z, roll,pitch,yaw;
		transform.getTranslationAndEulerAngles(x,y,z, roll,pitch,yaw);
		if(fabs(roll) > CV_PI/2 ||
		   fabs(pitch) > CV_PI/2 ||
		   fabs(yaw) > CV_PI/2)
		{
			transform.setNull();
			msg = uFormat("Too large rotation detected! (roll=%f, pitch=%f, yaw=%f)",
					roll, pitch, yaw);
			UWARN(msg.c_str());
		}
		else if(_force2D)
		{
			UDEBUG("Forcing 2D...");
			transform = Transform(x,y,0, 0, 0, yaw);
		}
	}

	if(rejectedMsg)
	{
		*rejectedMsg = msg;
	}
	if(varianceOut)
	{
		*varianceOut = variance>0.0f?variance:0.0001; // epsilon if exact transform
	}
	UDEBUG("transform=%s", transform.prettyPrint().c_str());
	return transform;
}

}
