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

#ifndef BAYESFILTER_H_
#define BAYESFILTER_H_

#include "rtabmap/core/rtabmap_core_export.h" // DLL export/import defines

#include <opencv2/core/core.hpp>
#include <list>
#include <set>
#include "rtabmap/utilite/UEventsHandler.h"
#include "rtabmap/core/Parameters.h"

namespace rtabmap {

class Memory;
class Signature;

class RTABMAP_CORE_EXPORT BayesFilter
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

	cv::Mat generatePrediction(const Memory * memory, const std::vector<int> & ids);

	unsigned long getMemoryUsed() const;

private:
	cv::Mat updatePrediction(const cv::Mat & oldPrediction,
			const Memory * memory,
			const std::vector<int> & oldIds,
			const std::vector<int> & newIds);
	void updatePosterior(const Memory * memory, const std::vector<int> & likelihoodIds);
	void normalize(cv::Mat & prediction, unsigned int index, float addedProbabilitiesSum, bool virtualPlaceUsed) const;

private:
	std::map<int, float> _posterior;
	cv::Mat _prediction;
	float _virtualPlacePrior;
	std::vector<double> _predictionLC; // {Vp, Lc, l1, l2, l3, l4...}
	bool _fullPredictionUpdate;
	float _totalPredictionLCValues;
	float _predictionEpsilon;
	std::map<int, std::map<int, int> > _neighborsIndex;
};

} // namespace rtabmap

#endif /* BAYESFILTER_H_ */
