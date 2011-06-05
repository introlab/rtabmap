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

#include "VerifyHypotheses.h"
#include "rtabmap/core/Parameters.h"
#include "Signature.h"
#include "Memory.h"
#include <cstdlib>
#include <opencv2/calib3d/calib3d.hpp>

#include "utilite/UtiLite.h"

namespace rtabmap
{

VerifyHypotheses::VerifyHypotheses(const ParametersMap & parameters) :
	_status(0)
{
	this->parseParameters(parameters);
}

void VerifyHypotheses::parseParameters(const ParametersMap & parameters)
{
}



/////////////////////////
// VerifyHypothesesSimple
/////////////////////////
VerifyHypothesesSimple::VerifyHypothesesSimple(const ParametersMap & parameters) :
	VerifyHypotheses(parameters)
{
	this->parseParameters(parameters);
}

VerifyHypothesesSimple::~VerifyHypothesesSimple()
{
}

void VerifyHypothesesSimple::parseParameters(const ParametersMap & parameters)
{
	VerifyHypotheses::parseParameters(parameters);
}

int VerifyHypothesesSimple::verifyHypotheses(const std::list<int> & hypotheses, const Memory * mem)
{
	int hypothesis = 0;

	if(!hypotheses.empty())
	{
		for(std::list<int>::const_iterator i = hypotheses.begin(); i!=hypotheses.end(); ++i)
		{
			if(*i > 0)
			{
				hypothesis = *i;
				break;
			}
		}
	}

	return hypothesis;
}


/////////////////////////
// VerifyHypothesesSignSeq
/////////////////////////
/*VerifyHypothesesSignSeq::VerifyHypothesesSignSeq(const ParametersMap & parameters) :
	VerifyHypotheses(parameters),
	_seqLength(Parameters::defaultVhEpSeqLength())
{
	this->parseParameters(parameters);
}

VerifyHypothesesSignSeq::~VerifyHypothesesSignSeq() {

}

void VerifyHypothesesSignSeq::parseParameters(const ParametersMap & parameters)
{
	ParametersMap::const_iterator iter;
	if((iter=parameters.find(Parameters::kVhEpSeqLength())) != parameters.end())
	{
		_seqLength = atoi((*iter).second.c_str());
	}
	VerifyHypotheses::parseParameters(parameters);
}

int VerifyHypothesesSignSeq::verifyHypotheses(const std::list<int> & hypotheses, const Memory * mem)
{
	int hypothesis = 0;
	std::map<int, int> hypothesesToKeep;

	// update hypotheses
	std::map<int, int>::iterator tmp;
	for(std::list<int>::const_iterator j=hypotheses.begin(); j!=hypotheses.end(); ++j)
	{
		// Add it like a new hypothesis
		hypothesesToKeep.insert(std::pair<int, int>(*j, 1)); // NOTE : It will be deleted if an updated hypothesis gives the same id.

		// If we have already this hypothesis, just keep it
		tmp = _hypotheses.find(*j);
		if(tmp != _hypotheses.end())
		{
			hypothesesToKeep.insert(std::pair<int, int>((*tmp).first, (*tmp).second));
		}

		// Forward hypothesis
		tmp = _hypotheses.find(*j-1);
		if(tmp != _hypotheses.end())
		{
			hypothesesToKeep.insert(std::pair<int, int>(*j, (*tmp).second+1));
		}

		// Backward hypothesis
		tmp = _hypotheses.find(*j+1);
		if(tmp != _hypotheses.end())
		{
			hypothesesToKeep.insert(std::pair<int, int>(*j, (*tmp).second-1));
		}
	}

	_hypotheses = hypothesesToKeep;

	// if an hypothesis has at least 3 references, return the id
	if(_hypotheses.size()>0)
	{
		for(std::map<int, int>::iterator i=_hypotheses.begin(); i!=_hypotheses.end(); ++i)
		{
			if((*i).second > _seqLength)
			{
				hypothesis = (*i).first;
				break; // return the first
			}
		}
	}

	return hypothesis;
}*/



/////////////////////////
// VerifyHypothesesEpipolarGeo
/////////////////////////
VerifyHypothesesEpipolarGeo::VerifyHypothesesEpipolarGeo(const ParametersMap & parameters) :
	VerifyHypotheses(parameters),
	_matchCountMinAccepted(Parameters::defaultVhEpMatchCountMin()),
	_ransacParam1(Parameters::defaultVhEpRansacParam1()),
	_ransacParam2(Parameters::defaultVhEpRansacParam2())
{
	this->parseParameters(parameters);
}

VerifyHypothesesEpipolarGeo::~VerifyHypothesesEpipolarGeo() {

}

void VerifyHypothesesEpipolarGeo::parseParameters(const ParametersMap & parameters)
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
	VerifyHypotheses::parseParameters(parameters);
}

void VerifyHypothesesEpipolarGeo::setStatus(int status)
{
	// Only set if the status was not set before. This will keep the
	// first error (when comparing with many signatures)
	if(status == UNDEFINED || this->getStatus() == UNDEFINED || status == ACCEPTED)
	{
		VerifyHypotheses::setStatus(status);
	}
}

int VerifyHypothesesEpipolarGeo::verifyHypotheses(const std::list<int> & hypotheses, const Memory * mem)
{
	ULOGGER_DEBUG("");
	int hypothesis = 0;
	this->setStatus(UNDEFINED);

	if(mem && !hypotheses.empty())
	{
		const KeypointSignature * ssRef = dynamic_cast<const KeypointSignature *>(mem->getLastSignature());
		if(ssRef)
		{
			unsigned int i=0;
			for(std::list<int>::const_iterator iter = hypotheses.begin(); iter!=hypotheses.end(); ++iter)
			{
				if(*iter > 0)
				{
					const KeypointSignature * ssHyp = dynamic_cast<const KeypointSignature *>(mem->getSignature(*iter));
					if(ssHyp)
					{
						if(doEpipolarGeometry(ssHyp, ssRef))
						{
							hypothesis = *iter;
							break;
						}
					}
				}
				++i;
			}
		}
	}
	else if(!mem)
	{
		this->setStatus(this->MEMORY_IS_NULL);
	}
	else if(hypotheses.empty())
	{
		this->setStatus(this->NO_HYPOTHESIS);
	}

	return hypothesis;
}

bool VerifyHypothesesEpipolarGeo::doEpipolarGeometry(const KeypointSignature * ssA, const KeypointSignature * ssB)
{
	if(ssA == 0 || ssB == 0)
	{
		this->setStatus(this->NULL_MATCHING_SURF_SIGNATURES);
		return false;
	}
	ULOGGER_DEBUG("id(%d,%d)", ssA->id(), ssB->id());

	std::list<std::pair<cv::KeyPoint, cv::KeyPoint> > pairs;
	std::list<int> pairsId;

	//bool allPairs = true;
	int realPairsCount = 0;


	realPairsCount = findPairsOne(ssA->getWords(), ssB->getWords(), pairs, pairsId);
	ULOGGER_DEBUG("%d %d", pairs.size(), pairsId.size());
	int pairsCount = pairs.size();
	ULOGGER_DEBUG("id(%d,%d) realPairsCount found=%d, pairsCount=%d...", ssA->id(), ssB->id(), realPairsCount, pairsCount);

	int similarities = this->getTotalSimilarities(ssA->getWords(), ssB->getWords());

	ULOGGER_DEBUG("realPairsCount=%d, "
			"test1=%f%%, "
			"test2=%f%%, "
			"similarities/total=%f%%, "
			"realP/similarities=%f%%, "
			"(pairs/2)/similarities=%f%%",
			realPairsCount,
			float(realPairsCount)/(float(ssA->getWords().size() + ssB->getWords().size())/2),
			float(pairs.size())/(float(ssA->getWords().size() + ssB->getWords().size())/2),
			float(similarities)/float(ssA->getWords().size() + ssB->getWords().size()),
			float(realPairsCount) / float(similarities),
			float(pairs.size()) / float(similarities));
	if(pairsCount < _matchCountMinAccepted)
	{
		this->setStatus(this->NOT_ENOUGH_MATCHING_PAIRS);
		return false;
	}

	//Convert Keypoints to a structure that OpenCV understands
	//3 dimensions (Homogeneous vectors)
	cv::Mat points1(1, pairs.size(), CV_32FC2);
	cv::Mat points2(1, pairs.size(), CV_32FC2);

	float * points1data = points1.ptr<float>(0);
	float * points2data = points2.ptr<float>(0);

	// Fill the points here ...
	int i=0;
	for(std::list<std::pair<cv::KeyPoint, cv::KeyPoint> >::const_iterator iter = pairs.begin();
		iter != pairs.end();
		++iter )
	{
		points1data[i*2] = (*iter).first.pt.x;
		points1data[i*2+1] = (*iter).first.pt.y;

		points2data[i*2] = (*iter).second.pt.x;
		points2data[i*2+1] = (*iter).second.pt.y;

		// the output of the correspondences can be easily copied in MatLab
		/*if(i==0)
		{
			ULOGGER_DEBUG("pt x=[%f;%f;1;%d];,xp=[%f;%f;1;%d];",
					(*iter).first.pt.x,
					(*iter).first.pt.y,
					Util::valueAt(pairsId,i),
					(*iter).second.pt.x,
					(*iter).second.pt.y,
					Util::valueAt(pairsId,i));
		}
		else
		{
			ULOGGER_DEBUG("pt x=[x [%f;%f;1;%d]];,xp=[xp [%f;%f;1;%d]];",
					(*iter).first.pt.x,
					(*iter).first.pt.y,
					Util::valueAt(pairsId,i),
					(*iter).second.pt.x,
					(*iter).second.pt.y,
					Util::valueAt(pairsId,i));
		}*/
		++i;
	}

	UTimer timer;
	timer.start();

	// Find the fundamental matrix
	cv::vector<uchar> status;
	cv::Mat fundamentalMatrix = cv::findFundamentalMat(
				points1,
				points2,
				status,
				CV_FM_RANSAC,
				_ransacParam1,
				_ransacParam2);

	ULOGGER_DEBUG("Find fundamental matrix (OpenCV) time = %fs", timer.ticks());

		// Fundamental matrix is valid ?
	bool fundMatFound = false;
	if(fundamentalMatrix.type() != CV_64FC1)
	{
		ULOGGER_FATAL("fundamentalMatrix.type() != CV_64FC1");
	}
	if(fundamentalMatrix.cols==3 && fundamentalMatrix.rows==3 &&
	   (fundamentalMatrix.at<double>(0,0) != 0.0 ||
	    fundamentalMatrix.at<double>(0,1) != 0.0 ||
	    fundamentalMatrix.at<double>(0,2) != 0.0 ||
	    fundamentalMatrix.at<double>(1,0) != 0.0 ||
	    fundamentalMatrix.at<double>(1,1) != 0.0 ||
	    fundamentalMatrix.at<double>(1,2) != 0.0 ||
		fundamentalMatrix.at<double>(2,0) != 0.0 ||
		fundamentalMatrix.at<double>(2,1) != 0.0 ||
		fundamentalMatrix.at<double>(2,2) != 0.0) )

	{
		fundMatFound = true;
	}

	ULOGGER_DEBUG("id(%d,%d) fm_count=%d...", ssA->id(), ssB->id(), fundMatFound);

	if(fundMatFound)
	{
		std::list<std::pair<cv::KeyPoint, cv::KeyPoint> > inliers;
		std::list<int> inliersId;

		int goodCount = 0;
		float total = 0;
		std::list<std::pair<float, float> > ptsAddedA;
		std::list<std::pair<float, float> > ptsAddedB;
		cv::Mat x(3, 1, fundamentalMatrix.type());
		cv::Mat xp(1, 3, fundamentalMatrix.type());
		int i=0;
		for(std::list<std::pair<cv::KeyPoint, cv::KeyPoint> >::iterator iter=pairs.begin(); iter!=pairs.end(); ++iter)
		{
			//if(status[i])
			{
				if(uContains(ptsAddedA, std::pair<float, float>((*iter).first.pt.x, (*iter).first.pt.y)))
				{
					ULOGGER_DEBUG("already added point [%f,%f,1]", (*iter).first.pt.x, (*iter).first.pt.y);
				}
				else if(uContains(ptsAddedB, std::pair<float, float>((*iter).second.pt.x, (*iter).second.pt.y)))
				{
					ULOGGER_DEBUG("already added point [%f,%f,1]", (*iter).second.pt.x, (*iter).second.pt.y);
				}
				else
				{
					double * xData = x.ptr<double>(0);
					double * xpData = xp.ptr<double>(0);
					xData[0] = (*iter).first.pt.x;
					xData[1] = (*iter).first.pt.y;
					xData[2] = 1;
					xpData[0] = (*iter).second.pt.x;
					xpData[1] = (*iter).second.pt.y;
					xpData[2] = 1;
					cv::Mat r = xp * (fundamentalMatrix * x);
					//if((r->data.fl[0] < 0 ? -r->data.fl[0]:r->data.fl[0]) < 1000000)
					{
						// Add only once a pair for the same id, used when a point matches with more than one...
						ptsAddedA.push_back(std::pair<float, float>((*iter).first.pt.x, (*iter).first.pt.y));
						ptsAddedB.push_back(std::pair<float, float>((*iter).second.pt.x, (*iter).second.pt.y));
						if(status[i])
						{
							inliers.push_back(*iter);
							inliersId.push_back(uValueAt(pairsId, i));
							goodCount++;
						}
						//ULOGGER_DEBUG("[%d] status=%d, r->data.fl[0]=%f, Added!", Util::valueAt(pairsId,i), status[i], r.ptr<double>(0)[0]);
					}
					/*else
					{
						ULOGGER_DEBUG("status=%d, r->data.fl[0]=%f, Not added!", status->data.ptr[i], r->data.fl[0]);
					}*/
					total+=(r.ptr<double>(0)[0] < 0 ? -r.ptr<double>(0)[0]:r.ptr<double>(0)[0]);
				}
			}
			/*else
			{
				ULOGGER_DEBUG("VHEpipolarGeo::doEpipolarGeometry() status=%d", status[i]);
			}*/
			++i;
		}

		ULOGGER_DEBUG("pairs/realPairs=%d/%d -> %d%%, goodCount=%d -> %d%%, good/real = %d%%, totalMean=%f",
				pairsCount,
				realPairsCount,
				int(float(pairsCount)/float(realPairsCount*100)),
				goodCount,
				int(float(goodCount)/float(pairsCount*100)),
				int(float(goodCount)/float(realPairsCount*100)),
				total/float(realPairsCount));

		// Show the fundamental matrix
		ULOGGER_DEBUG(
			"F = [%f %f %f;%f %f %f;%f %f %f]",
			fundamentalMatrix.ptr<double>(0)[0],
			fundamentalMatrix.ptr<double>(0)[1],
			fundamentalMatrix.ptr<double>(0)[2],
			fundamentalMatrix.ptr<double>(0)[3],
			fundamentalMatrix.ptr<double>(0)[4],
			fundamentalMatrix.ptr<double>(0)[5],
			fundamentalMatrix.ptr<double>(0)[6],
			fundamentalMatrix.ptr<double>(0)[7],
			fundamentalMatrix.ptr<double>(0)[8]);

		if(goodCount < _matchCountMinAccepted)
		{
			this->setStatus(this->EPIPOLAR_CONSTRAINT_FAILED);
			ULOGGER_DEBUG("Epipolar constraint failed A : not enough inliers (%d), min is %d", goodCount, _matchCountMinAccepted);
			return false;
		}
		else
		{
			this->setStatus(this->ACCEPTED);
			return true;
		}
	}
	this->setStatus(this->FUNDAMENTAL_MATRIX_NOT_FOUND);
	return false;
}

/**
 * if a=[1 2 3 4 6 6], b=[1 1 2 4 5 6 6], results= [(1,1a) (2,2) (4,4) (6a,6a) (6b,6b)]
 * realPairsCount = 5
 */
int VerifyHypothesesEpipolarGeo::findPairsDirect(const std::multimap<int, cv::KeyPoint> & wordsA,
		const std::multimap<int, cv::KeyPoint> & wordsB,
		std::list<std::pair<cv::KeyPoint, cv::KeyPoint> > & pairs,
		std::list<int> & pairsId)
{
	const std::list<int> & ids = uUniqueKeys(wordsA);
	std::multimap<int, cv::KeyPoint>::const_iterator iterA;
	std::multimap<int, cv::KeyPoint>::const_iterator iterB;
	pairs.clear();
	int realPairsCount = 0;
	for(std::list<int>::const_iterator i=ids.begin(); i!=ids.end(); ++i)
	{
		iterA = wordsA.find(*i);
		iterB = wordsB.find(*i);
		while(iterA != wordsA.end() && iterB != wordsB.end() && (*iterA).first == (*iterB).first && (*iterA).first == *i)
		{
			pairsId.push_back(*i);
			pairs.push_back(std::pair<cv::KeyPoint, cv::KeyPoint>((*iterA).second, (*iterB).second));
			++iterA;
			++iterB;
			++realPairsCount;
		}
	}
	return realPairsCount;
}

/**
 * if a=[1 2 3 4 6 6], b=[1 1 2 4 5 6 6], results= [(2,2) (4,4)]
 * realPairsCount = 5
 */
int VerifyHypothesesEpipolarGeo::findPairsOne(const std::multimap<int, cv::KeyPoint> & wordsA,
		const std::multimap<int, cv::KeyPoint> & wordsB,
		std::list<std::pair<cv::KeyPoint, cv::KeyPoint> > & pairs,
		std::list<int> & pairsId)
{
	const std::list<int> & ids = uUniqueKeys(wordsA);
	int realPairsCount = 0;
	pairs.clear();
	for(std::list<int>::const_iterator i=ids.begin(); i!=ids.end(); ++i)
	{
		std::list<cv::KeyPoint> ptsA = uValues(wordsA, *i);
		std::list<cv::KeyPoint> ptsB = uValues(wordsB, *i);
		if(ptsA.size() == 1 && ptsB.size() == 1)
		{
			pairs.push_back(std::pair<cv::KeyPoint, cv::KeyPoint>(ptsA.front(), ptsB.front()));
			pairsId.push_back(*i);
			++realPairsCount;
		}
		else if(ptsA.size()>1 && ptsB.size()>1)
		{
			// just update the count
			realPairsCount += ptsA.size() > ptsB.size() ? ptsB.size() : ptsA.size();
		}
	}
	return realPairsCount;
}

/**
 * if a=[1 2 3 4 6 6], b=[1 1 2 4 5 6 6], results= [(1,1a) (1,1b) (2,2) (4,4) (6a,6a) (6a,6b) (6b,6a) (6b,6b)]
 * realPairsCount = 5
 */
int VerifyHypothesesEpipolarGeo::findPairsAll(const std::multimap<int, cv::KeyPoint> & wordsA,
		const std::multimap<int, cv::KeyPoint> & wordsB,
		std::list<std::pair<cv::KeyPoint, cv::KeyPoint> > & pairs,
		std::list<int> & pairsId)
{
	UTimer timer;
	timer.start();
	const std::list<int> & ids = uUniqueKeys(wordsA);
	pairs.clear();
	int realPairsCount = 0;;
	for(std::list<int>::const_iterator iter=ids.begin(); iter!=ids.end(); ++iter)
	{
		std::list<cv::KeyPoint> ptsA = uValues(wordsA, *iter);
		std::list<cv::KeyPoint> ptsB = uValues(wordsB, *iter);

		realPairsCount += ptsA.size() > ptsB.size() ? ptsB.size() : ptsA.size();

		for(std::list<cv::KeyPoint>::iterator jter=ptsA.begin(); jter!=ptsA.end(); ++jter)
		{
			for(std::list<cv::KeyPoint>::iterator kter=ptsB.begin(); kter!=ptsB.end(); ++kter)
			{
				pairsId.push_back(*iter);
				pairs.push_back(std::pair<cv::KeyPoint, cv::KeyPoint>(*jter, *kter));
			}
		}
	}
	ULOGGER_DEBUG("time = %f", timer.ticks());
	return realPairsCount;
}

/**
 * if a=[1 2 3 4 6 6], b=[1 1 2 4 5 6 6], results= [1 2 4 6]
 * return 4
 */
std::list<int> VerifyHypothesesEpipolarGeo::findSameIds(const std::multimap<int, cv::KeyPoint> & wordsA,
		const std::multimap<int, cv::KeyPoint> & wordsB)
{
	std::list<int> sameIds;
	const std::list<int> & ids = uUniqueKeys(wordsA);
	for(std::list<int>::const_iterator i=ids.begin(); i!=ids.end(); ++i)
	{
		if(wordsB.find(*i) != wordsB.end())
		{
			sameIds.push_back(*i);
		}
	}
	return sameIds;
}

int VerifyHypothesesEpipolarGeo::getTotalSimilarities(const std::multimap<int, cv::KeyPoint> & wordsA, const std::multimap<int, cv::KeyPoint> & wordsB)
{
	const std::list<int> & ids = uUniqueKeys(wordsA);
	int total = 0;
	for(std::list<int>::const_iterator i=ids.begin(); i!=ids.end(); ++i)
	{
		total += uValues(wordsA, *i).size();
		total += uValues(wordsB, *i).size();
	}
	return total;
}

}
