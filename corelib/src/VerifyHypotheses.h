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

class Memory;

class RTABMAP_EXP VerifyHypotheses
{
public:
	virtual ~VerifyHypotheses() {}
	virtual int verifyHypotheses(const std::list<int> & hypotheses, const Memory * mem) = 0;

	int getStatus() {return _status;}
	virtual void parseParameters(const ParametersMap & parameters);

protected:
	VerifyHypotheses(const ParametersMap & parameters = ParametersMap());
	virtual void setStatus(int status) {_status = status;}

private:
	int _status;
};


/////////////////////////
// VerifyHypothesesSimple
/////////////////////////
class VerifyHypothesesSimple : public VerifyHypotheses {
public:
	VerifyHypothesesSimple(const ParametersMap & parameters = ParametersMap());
	virtual ~VerifyHypothesesSimple();
	virtual int verifyHypotheses(const std::list<int> & hypotheses, const Memory * mem);
	virtual void parseParameters(const ParametersMap & parameters);
};


/////////////////////////
// VerifyHypothesesSignSeq
/////////////////////////
/*class VerifyHypothesesSignSeq : public VerifyHypotheses {
public:
	VerifyHypothesesSignSeq(const ParametersMap & parameters = ParametersMap());
	virtual ~VerifyHypothesesSignSeq();
	virtual int verifyHypotheses(const std::list<int> & hypotheses, const Memory * mem);
	virtual void parseParameters(const ParametersMap & parameters);

private:
	std::map<int, int> _hypotheses;
	int _seqLength;

};*/



/////////////////////////
// VerifyHypothesesEpipolarGeo
/////////////////////////
class KeypointSignature;

class RTABMAP_EXP VerifyHypothesesEpipolarGeo : public VerifyHypotheses
{
public:
	enum STATUS
	{
		UNDEFINED,
		ACCEPTED,
		NO_HYPOTHESIS,
		MEMORY_IS_NULL,
		NOT_ENOUGH_MATCHING_PAIRS,
		EPIPOLAR_CONSTRAINT_FAILED,
		NULL_MATCHING_SURF_SIGNATURES,
		FUNDAMENTAL_MATRIX_NOT_FOUND
	};

public:
	static int findPairsOne(const std::multimap<int, cv::KeyPoint> & wordsA, const std::multimap<int, cv::KeyPoint> & wordsB, std::list<std::pair<cv::KeyPoint, cv::KeyPoint> > & pairs, std::list<int> & pairsId);
	static int findPairsDirect(const std::multimap<int, cv::KeyPoint> & wordsA, const std::multimap<int, cv::KeyPoint> & wordsB, std::list<std::pair<cv::KeyPoint, cv::KeyPoint> > & pairs, std::list<int> & pairsId);
	static int findPairsAll(const std::multimap<int, cv::KeyPoint> & wordsA, const std::multimap<int, cv::KeyPoint> & wordsB, std::list<std::pair<cv::KeyPoint, cv::KeyPoint> > & pairs, std::list<int> & pairsId);
	static std::list<int> findSameIds(const std::multimap<int, cv::KeyPoint> & wordsA, const std::multimap<int, cv::KeyPoint> & wordsB);
public:
	VerifyHypothesesEpipolarGeo(const ParametersMap & parameters = ParametersMap());
	virtual ~VerifyHypothesesEpipolarGeo();
	virtual int verifyHypotheses(const std::list<int> & hypotheses, const Memory * mem);
	virtual void parseParameters(const ParametersMap & parameters);

	int getTotalSimilarities(const std::multimap<int, cv::KeyPoint> & wordsA, const std::multimap<int, cv::KeyPoint> & wordsB);

	int getMatchCountMinAccepted() const {return _matchCountMinAccepted;}
	double getRansacParam1() const {return _ransacParam1;}
	double getRansacParam2() const {return _ransacParam2;}

	void setMatchCountMinAccepted(int matchCountMinAccepted) {_matchCountMinAccepted = matchCountMinAccepted;}
	void setRansacParam1(double ransacParam1) {_ransacParam1 = ransacParam1;}
	void setRansacParam2(double ransacParam2) {_ransacParam2 = ransacParam2;}

protected:
	virtual void setStatus(int status);

private:
	bool doEpipolarGeometry(const KeypointSignature * ssA, const KeypointSignature * ssB);

private:
	int _matchCountMinAccepted;
	double _ransacParam1;
	double _ransacParam2;
};

} // namespace rtabmap

#endif /* VERIFYHYPOTHESES_H_ */
