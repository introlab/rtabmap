/*
 * Copyright (C) 2010-2011, Mathieu Labbe and IntRoLab - Universite de Sherbrooke
 *
 * This file is part of RTAB-Map.
 *
 * RTAB-Map is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * RTAB-Map is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with RTAB-Map.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "rtabmap/core/VerifyHypotheses.h"
#include "rtabmap/core/Parameters.h"
#include "rtabmap/core/Signature.h"
#include "rtabmap/core/EpipolarGeometry.h"
#include <cstdlib>


#include "utilite/UtiLite.h"

namespace rtabmap
{

HypVerificator::HypVerificator(const ParametersMap & parameters)
{
	this->parseParameters(parameters);
}

void HypVerificator::parseParameters(const ParametersMap & parameters)
{
}

bool HypVerificator::verify(const Signature * ref, const Signature * hyp)
{
	UDEBUG("");
	return ref && hyp && !ref->isBadSignature() && !hyp->isBadSignature();
}



/////////////////////////
// HypVerificatorSim
/////////////////////////
HypVerificatorSim::HypVerificatorSim(const ParametersMap & parameters) :
	HypVerificator(parameters),
	_similarity(Parameters::defaultVhSimilarity())
{
	this->parseParameters(parameters);
}

HypVerificatorSim::~HypVerificatorSim()
{
}

void HypVerificatorSim::parseParameters(const ParametersMap & parameters)
{
	ParametersMap::const_iterator iter;
	if((iter=parameters.find(Parameters::kVhSimilarity())) != parameters.end())
	{
		_similarity = std::atof((*iter).second.c_str());
	}

	HypVerificator::parseParameters(parameters);
}

bool HypVerificatorSim::verify(const Signature * ref, const Signature * hyp)
{
	UDEBUG("");
	if(ref && hyp)
	{
		return ref->compareTo(hyp) >= _similarity;
	}

	return false;
}


/////////////////////////
// HypVerificatorEpipolarGeo
/////////////////////////
HypVerificatorEpipolarGeo::HypVerificatorEpipolarGeo(const ParametersMap & parameters) :
	HypVerificator(parameters),
	_matchCountMinAccepted(Parameters::defaultVhEpMatchCountMin()),
	_ransacParam1(Parameters::defaultVhEpRansacParam1()),
	_ransacParam2(Parameters::defaultVhEpRansacParam2())
{
	this->parseParameters(parameters);
}

HypVerificatorEpipolarGeo::~HypVerificatorEpipolarGeo() {

}

void HypVerificatorEpipolarGeo::parseParameters(const ParametersMap & parameters)
{
	ParametersMap::const_iterator iter;
	if((iter=parameters.find(Parameters::kVhEpMatchCountMin())) != parameters.end())
	{
		_matchCountMinAccepted = std::atoi((*iter).second.c_str());
	}
	if((iter=parameters.find(Parameters::kVhEpRansacParam1())) != parameters.end())
	{
		_ransacParam1 = std::atof((*iter).second.c_str());
	}
	if((iter=parameters.find(Parameters::kVhEpRansacParam2())) != parameters.end())
	{
		_ransacParam2 = std::atof((*iter).second.c_str());
	}
	HypVerificator::parseParameters(parameters);
}

bool HypVerificatorEpipolarGeo::verify(const Signature * ref, const Signature * hyp)
{
	UDEBUG("");
	const KeypointSignature * ssRef = dynamic_cast<const KeypointSignature *>(ref);
	const KeypointSignature * ssHyp = dynamic_cast<const KeypointSignature *>(hyp);
	if(ssRef && ssHyp)
	{
		return doEpipolarGeometry(ssHyp, ssRef);
	}

	return false;
}

bool HypVerificatorEpipolarGeo::doEpipolarGeometry(const KeypointSignature * ssA, const KeypointSignature * ssB)
{
	if(ssA == 0 || ssB == 0)
	{
		return false;
	}
	ULOGGER_DEBUG("id(%d,%d)", ssA->id(), ssB->id());

	std::list<std::pair<int, std::pair<cv::KeyPoint, cv::KeyPoint> > > pairs;

	findPairsUnique(ssA->getWords(), ssB->getWords(), pairs);

	if((int)pairs.size()<_matchCountMinAccepted)
	{
		return false;
	}

	std::vector<uchar> status;
	cv::Mat f = findFFromWords(pairs, status, _ransacParam1, _ransacParam2);

	int inliers = uSum(status);
	if(inliers < _matchCountMinAccepted)
	{
		ULOGGER_DEBUG("Epipolar constraint failed A : not enough inliers (%d/%d), min is %d", inliers, pairs.size(), _matchCountMinAccepted);
		return false;
	}
	else
	{
		UDEBUG("inliers = %d/%d", inliers, pairs.size());
		return true;
	}
}

}
