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

#ifndef VERIFYHYPOTHESES_H_
#define VERIFYHYPOTHESES_H_

#include "rtabmap/core/RtabmapExp.h" // DLL export/import defines

#include <list>
#include "rtabmap/core/Parameters.h"
#include "utilite/UEventsHandler.h"
#include <map>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

namespace rtabmap
{

class Signature;

// return always true, i.e, there is no verification
class RTABMAP_EXP HypVerificator
{
public:
	HypVerificator(const ParametersMap & parameters = ParametersMap());
	virtual ~HypVerificator() {}
	virtual bool verify(const Signature * ref, const Signature * hyp);

	virtual void parseParameters(const ParametersMap & parameters);
};


/////////////////////////
// HypVerificatorSim
/////////////////////////
class HypVerificatorSim : public HypVerificator {
public:
	HypVerificatorSim(const ParametersMap & parameters = ParametersMap());
	virtual ~HypVerificatorSim();
	virtual bool verify(const Signature * ref, const Signature * hyp);
	virtual void parseParameters(const ParametersMap & parameters);

private:
	float _similarity;
};


/////////////////////////
// HypVerificatorEpipolarGeo
/////////////////////////
class KeypointSignature;

class RTABMAP_EXP HypVerificatorEpipolarGeo : public HypVerificator
{
public:
	static int findPairsOne(const std::multimap<int, cv::KeyPoint> & wordsA, const std::multimap<int, cv::KeyPoint> & wordsB, std::list<std::pair<cv::KeyPoint, cv::KeyPoint> > & pairs, std::list<int> & pairsId);
	static int findPairsDirect(const std::multimap<int, cv::KeyPoint> & wordsA, const std::multimap<int, cv::KeyPoint> & wordsB, std::list<std::pair<cv::KeyPoint, cv::KeyPoint> > & pairs, std::list<int> & pairsId);
	static int findPairsAll(const std::multimap<int, cv::KeyPoint> & wordsA, const std::multimap<int, cv::KeyPoint> & wordsB, std::list<std::pair<cv::KeyPoint, cv::KeyPoint> > & pairs, std::list<int> & pairsId);
	static std::list<int> findSameIds(const std::multimap<int, cv::KeyPoint> & wordsA, const std::multimap<int, cv::KeyPoint> & wordsB);
public:
	HypVerificatorEpipolarGeo(const ParametersMap & parameters = ParametersMap());
	virtual ~HypVerificatorEpipolarGeo();
	virtual bool verify(const Signature * ref, const Signature * hyp);
	virtual void parseParameters(const ParametersMap & parameters);

	int getTotalSimilarities(const std::multimap<int, cv::KeyPoint> & wordsA, const std::multimap<int, cv::KeyPoint> & wordsB);

	int getMatchCountMinAccepted() const {return _matchCountMinAccepted;}
	double getRansacParam1() const {return _ransacParam1;}
	double getRansacParam2() const {return _ransacParam2;}

	void setMatchCountMinAccepted(int matchCountMinAccepted) {_matchCountMinAccepted = matchCountMinAccepted;}
	void setRansacParam1(double ransacParam1) {_ransacParam1 = ransacParam1;}
	void setRansacParam2(double ransacParam2) {_ransacParam2 = ransacParam2;}

private:
	bool doEpipolarGeometry(const KeypointSignature * ssA, const KeypointSignature * ssB);

private:
	int _matchCountMinAccepted;
	double _ransacParam1;
	double _ransacParam2;
};

} // namespace rtabmap

#endif /* VERIFYHYPOTHESES_H_ */
