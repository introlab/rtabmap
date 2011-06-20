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
#include "utilite/UEventsHandler.h"
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
	void setVirtualPlacePrior(float virtualPlacePrior);
	void setPredictionLC(const std::string & prediction);

	//getters
	const std::map<int, float> & getPosterior() const {return _posterior;}
	float getVirtualPlacePrior() const {return _virtualPlacePrior;}
	const std::vector<double> & getPredictionLC() const; // {Vp, Lc, l1, l2, l3, l4...}
	std::string getPredictionLCStr() const; // for convenience {Vp, Lc, l1, l2, l3, l4...}

	bool generatePrediction(CvMat * prediction, const Memory * memory, const std::map<int, int> & likelihoodIds) const;
	float addNeighborProb(CvMat * prediction, unsigned int row,  const Memory * memory, const std::map<int, int> & likelihoodIds, const Signature * s, unsigned int level) const;

private:
	void updatePosterior(const Memory * memory, const std::vector<int> & likelihoodIds);

private:
	std::map<int, float> _posterior;
	float _virtualPlacePrior;
	std::vector<double> _predictionLC; // {Vp, Lc, l1, l2, l3, l4...}
};

} // namespace rtabmap

#endif /* BAYESFILTER_H_ */
