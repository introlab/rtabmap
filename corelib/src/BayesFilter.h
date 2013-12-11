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

#ifndef BAYESFILTER_H_
#define BAYESFILTER_H_

#include "rtabmap/core/RtabmapExp.h" // DLL export/import defines

#include <opencv2/core/core.hpp>
#include <list>
#include <set>
#include "rtabmap/utilite/UEventsHandler.h"
#include "rtabmap/core/Parameters.h"

namespace rtabmap {

class Memory;
class Signature;

class RTABMAP_EXP BayesFilter
{
public:
	BayesFilter(const ParametersMap & parameters = ParametersMap());
	virtual ~BayesFilter();
	virtual void parseParameters(const ParametersMap & parameters);
	const std::map<int, float> & computePosterior(const Memory * memory, const std::map<int, float> & likelihood);
	void reset();

	//setters
	void setPredictionLC(const std::string & prediction);

	//getters
	const std::map<int, float> & getPosterior() const {return _posterior;}
	float getVirtualPlacePrior() const {return _virtualPlacePrior;}
	const std::vector<double> & getPredictionLC() const; // {Vp, Lc, l1, l2, l3, l4...}
	std::string getPredictionLCStr() const; // for convenience {Vp, Lc, l1, l2, l3, l4...}

	cv::Mat generatePrediction(const Memory * memory, const std::vector<int> & ids) const;

private:
	cv::Mat updatePrediction(const cv::Mat & oldPrediction,
			const Memory * memory,
			const std::vector<int> & oldIds,
			const std::vector<int> & newIds) const;
	void updatePosterior(const Memory * memory, const std::vector<int> & likelihoodIds);
	float addNeighborProb(cv::Mat & prediction,
			unsigned int col,
			const std::map<int, int> & neighbors,
			const std::map<int, int> & idToIndexMap) const;
	void normalize(cv::Mat & prediction, unsigned int index, float addedProbabilitiesSum, bool virtualPlaceUsed) const;

private:
	std::map<int, float> _posterior;
	cv::Mat _prediction;
	float _virtualPlacePrior;
	std::vector<double> _predictionLC; // {Vp, Lc, l1, l2, l3, l4...}
	bool _fullPredictionUpdate;
	float _totalPredictionLCValues;
};

} // namespace rtabmap

#endif /* BAYESFILTER_H_ */
