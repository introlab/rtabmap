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
#include <rtabmap/core/Memory.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/UTimer.h>

namespace rtabmap {

RegistrationVis::RegistrationVis(const ParametersMap & parameters) :
		_minInliers(Parameters::defaultVisMinInliers()),
		_inlierDistance(Parameters::defaultVisInlierDistance()),
		_iterations(Parameters::defaultVisIterations()),
		_refineIterations(Parameters::defaultVisRefineIterations()),
		_force2D(Parameters::defaultVisForce2D()),
		_epipolarGeometryVar(Parameters::defaultVisEpipolarGeometryVar()),
		_estimationType(Parameters::defaultVisEstimationType()),
		_PnPReprojError(Parameters::defaultVisPnPReprojError()),
		_PnPFlags(Parameters::defaultVisPnPFlags())
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

	uInsert(_featureParameters, ParametersPair(Parameters::kKpNNStrategy(), _featureParameters.at(Parameters::kVisNNType())));
	uInsert(_featureParameters, ParametersPair(Parameters::kKpNndrRatio(), _featureParameters.at(Parameters::kVisNNDR())));
	uInsert(_featureParameters, ParametersPair(Parameters::kKpDetectorStrategy(), _featureParameters.at(Parameters::kVisFeatureType())));
	uInsert(_featureParameters, ParametersPair(Parameters::kKpWordsPerImage(), _featureParameters.at(Parameters::kVisMaxFeatures())));
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

	UASSERT_MSG(_minInliers >= 1, uFormat("value=%d", _minInliers).c_str());
	UASSERT_MSG(_inlierDistance > 0.0f, uFormat("value=%f", _inlierDistance).c_str());
	UASSERT_MSG(_iterations > 0, uFormat("value=%d", _iterations).c_str());

	// override feature parameters
	for(ParametersMap::const_iterator iter=parameters.begin(); iter!=parameters.end(); ++iter)
	{
		std::string group = uSplit(iter->first, '/').front();
		if(group.compare("SURF") == 0 ||
			group.compare("SIFT") == 0 ||
			group.compare("ORB") == 0 ||
			group.compare("FAST") == 0 ||
			group.compare("FREAK") == 0 ||
			group.compare("BRIEF") == 0 ||
			group.compare("GFTT") == 0 ||
			group.compare("BRISK") == 0)
		{
			uInsert(_featureParameters, ParametersPair(iter->first, iter->second));
		}
	}

	if(uContains(parameters, Parameters::kVisNNType()))
	{
		uInsert(_featureParameters, ParametersPair(Parameters::kKpNNStrategy(), parameters.at(Parameters::kVisNNType())));
	}
	if(uContains(parameters, Parameters::kVisNNDR()))
	{
		uInsert(_featureParameters, ParametersPair(Parameters::kKpNndrRatio(), parameters.at(Parameters::kVisNNDR())));
	}
	if(uContains(parameters, Parameters::kVisFeatureType()))
	{
		uInsert(_featureParameters, ParametersPair(Parameters::kKpDetectorStrategy(), parameters.at(Parameters::kVisFeatureType())));
	}
	if(uContains(parameters, Parameters::kVisMaxFeatures()))
	{
		uInsert(_featureParameters, ParametersPair(Parameters::kKpWordsPerImage(), parameters.at(Parameters::kVisMaxFeatures())));
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

Transform RegistrationVis::computeTransformation(
			const Signature & fromSignature,
			const Signature & toSignature,
			Transform guess, // guess is ignored for RegistrationVis
			std::string * rejectedMsg,
			int * inliersOut,
			float * varianceOut,
			float * inliersRatioOut)
{
	Transform transform;
	std::string msg;
	// Guess transform from visual words

	int inliersCount= 0;
	double variance = 1.0;

	// Extract features?
	const std::multimap<int, cv::KeyPoint> * wordsFrom = 0;
	const std::multimap<int, cv::KeyPoint> * wordsTo = 0;
	const std::multimap<int, pcl::PointXYZ> * words3From = 0;
	const std::multimap<int, pcl::PointXYZ> * words3To = 0;
	std::multimap<int, cv::KeyPoint> extractedWordsFrom, extractedWordsTo;
	std::multimap<int, pcl::PointXYZ> extractedWords3From, extractedWords3To;
	if(fromSignature.getWords().size() == 0 && toSignature.getWords().size() == 0)
	{
		// Use the Memory class to extract features
		Memory memory(_featureParameters);

		// Add signatures
		SensorData dataFrom = fromSignature.sensorData();
		SensorData dataTo = toSignature.sensorData();
		// make sure there are no features already in the SensorData
		dataFrom.setFeatures(std::vector<cv::KeyPoint>(), cv::Mat());
		dataTo.setFeatures(std::vector<cv::KeyPoint>(), cv::Mat());

		UTimer timeT;

		memory.update(dataFrom);
		if(memory.getLastWorkingSignature() == 0)
		{
			UWARN("Failed to extract features for node %d", dataFrom.id());
		}
		else
		{
			extractedWordsFrom = memory.getLastWorkingSignature()->getWords();
			extractedWords3From = memory.getLastWorkingSignature()->getWords3();
			UDEBUG("timeTo = %fs", timeT.ticks());

			memory.update(dataTo);
			if(memory.getLastWorkingSignature() == 0)
			{
				UWARN("Failed to extract features for node %d", dataTo.id());
			}
			else
			{
				extractedWordsTo = memory.getLastWorkingSignature()->getWords();
				extractedWords3To = memory.getLastWorkingSignature()->getWords3();
				UDEBUG("timeFrom = %fs", timeT.ticks());
			}
		}

		wordsFrom = &extractedWordsFrom;
		wordsTo = &extractedWordsTo;
		words3From = &extractedWords3From;
		words3To = &extractedWords3To;
	}
	else
	{
		wordsFrom = &fromSignature.getWords();
		wordsTo = &toSignature.getWords();
		words3From = &fromSignature.getWords3();
		words3To = &toSignature.getWords3();
	}

	if(_estimationType == 2) // Epipolar Geometry
	{
		if(!toSignature.sensorData().stereoCameraModel().isValid() &&
		   (toSignature.sensorData().cameraModels().size() != 1 ||
			!toSignature.sensorData().cameraModels()[0].isValid()))
		{
			UERROR("Calibrated camera required (multi-cameras not supported).");
		}
		else if((int)wordsFrom->size() >= _minInliers &&
				(int)wordsTo->size() >= _minInliers)
		{
			UASSERT(fromSignature.sensorData().stereoCameraModel().isValid() || (fromSignature.sensorData().cameraModels().size() == 1 && fromSignature.sensorData().cameraModels()[0].isValid()));
			const CameraModel & cameraModel = fromSignature.sensorData().stereoCameraModel().isValid()?fromSignature.sensorData().stereoCameraModel().left():fromSignature.sensorData().cameraModels()[0];

			// we only need the camera transform, send guess words3 for scale estimation
			Transform cameraTransform;
			std::multimap<int, pcl::PointXYZ> inliers3D = util3d::generateWords3DMono(
					*wordsFrom,
					*wordsTo,
					cameraModel,
					cameraTransform,
					_iterations,
					_PnPReprojError,
					_PnPFlags, // cv::SOLVEPNP_ITERATIVE
					1.0f,
					0.99f,
					*words3From, // for scale estimation
					&variance);

			inliersCount = (int)inliers3D.size();

			if(!cameraTransform.isNull())
			{
				if((int)inliers3D.size() >= _minInliers)
				{
					if(variance <= _epipolarGeometryVar)
					{
						transform = cameraTransform;
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
		else if(words3From->size() == 0)
		{
			msg = uFormat("No 3D guess words found");
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
		if(!toSignature.sensorData().stereoCameraModel().isValid() &&
		   (toSignature.sensorData().cameraModels().size() != 1 ||
			!toSignature.sensorData().cameraModels()[0].isValid()))
		{
			UERROR("Calibrated camera required (multi-cameras not supported). Id=%d Models=%d StereoModel=%d weight=%d",
					toSignature.id(),
					(int)toSignature.sensorData().cameraModels().size(),
					toSignature.sensorData().stereoCameraModel().isValid()?1:0,
					toSignature.getWeight());
		}
		else
		{
			// 3D to 2D
			if((int)words3From->size() >= _minInliers &&
			   (int)wordsTo->size() >= _minInliers)
			{
				UASSERT(toSignature.sensorData().stereoCameraModel().isValid() || (toSignature.sensorData().cameraModels().size() == 1 && toSignature.sensorData().cameraModels()[0].isValid()));
				const CameraModel & cameraModel = toSignature.sensorData().stereoCameraModel().isValid()?toSignature.sensorData().stereoCameraModel().left():toSignature.sensorData().cameraModels()[0];

				std::vector<int> inliersV;
				transform = util3d::estimateMotion3DTo2D(
						uMultimapToMap(*words3From),
						uMultimapToMap(*wordsTo),
						cameraModel,
						_minInliers,
						_iterations,
						_PnPReprojError,
						_PnPFlags,
						Transform::getIdentity(),
						uMultimapToMap(*words3To),
						&variance,
						0,
						&inliersV);
				inliersCount = (int)inliersV.size();
				if(transform.isNull())
				{
					msg = uFormat("Not enough inliers %d/%d between %d and %d",
							inliersCount, _minInliers, fromSignature.id(), toSignature.id());
					UINFO(msg.c_str());
				}
			}
			else
			{
				msg = uFormat("Not enough features in images (old=%d, new=%d, min=%d)",
						(int)words3From->size(), (int)wordsTo->size(), _minInliers);
				UINFO(msg.c_str());
			}
		}

	}
	else
	{
		// 3D -> 3D
		if((int)words3From->size() >= _minInliers &&
		   (int)words3To->size() >= _minInliers)
		{
			std::vector<int> inliersV;
			transform = util3d::estimateMotion3DTo3D(
					uMultimapToMap(*words3From),
					uMultimapToMap(*words3To),
					_minInliers,
					_inlierDistance,
					_iterations,
					_refineIterations,
					&variance,
					0,
					&inliersV);
			inliersCount = (int)inliersV.size();
			if(transform.isNull())
			{
				msg = uFormat("Not enough inliers %d/%d between %d and %d",
						inliersCount, _minInliers, fromSignature.id(), toSignature.id());
				UINFO(msg.c_str());
			}
		}
		else
		{
			msg = uFormat("Not enough 3D features in images (old=%d, new=%d, min=%d)",
					(int)words3From->size(), (int)words3To->size(), _minInliers);
			UINFO(msg.c_str());
		}
	}

	if(!transform.isNull())
	{
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

	if(_varianceFromInliersCount)
	{
		variance = inliersCount > 0?1.0/double(inliersCount):1.0;
	}

	if(rejectedMsg)
	{
		*rejectedMsg = msg;
	}
	if(inliersOut)
	{
		*inliersOut = inliersCount;
	}
	if(varianceOut)
	{
		*varianceOut = variance>0.0f?variance:0.0001; // epsilon if exact transform
	}
	UDEBUG("transform=%s", transform.prettyPrint().c_str());
	return transform;
}

}
