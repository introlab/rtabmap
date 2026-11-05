/*
Copyright (c) 2010-2026, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
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

#include "DownhillSimplexSolver.h"

#include <opencv2/core/optim.hpp>

#include <algorithm>
#include <vector>

namespace {

// The score to minimize, as a function of the solved parameters x; the others stay as
// in p.
class NegativeScore : public cv::MinProblemSolver::Function
{
public:
	NegativeScore(const CalibrationProblem & problem, const std::vector<int> & solved, const double p[6]) :
		problem_(problem), solved_(solved)
	{
		std::copy(p, p + 6, p_);
	}
	int getDims() const override {return (int)solved_.size();}
	double calc(const double * x) const override
	{
		double q[6];
		std::copy(p_, p_ + 6, q);
		for(size_t i = 0; i < solved_.size(); ++i)
		{
			q[solved_[i]] = x[i];
		}
		return -problem_.score(correctionFrom(q));
	}
private:
	const CalibrationProblem & problem_;
	std::vector<int> solved_;
	double p_[6];
};

}  // namespace

void DownhillSimplexSolver::solve(const CalibrationProblem & problem, bool estimateTranslation, double p[6]) const
{
	std::vector<int> solved;
	for(int k = estimateTranslation ? 0 : 3; k < 6; ++k)
	{
		solved.push_back(k);
	}

	cv::Ptr<cv::DownhillSolver> solver = cv::DownhillSolver::create(
			cv::makePtr<NegativeScore>(problem, solved, p),
			cv::noArray(),
			cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 5000, 1e-9));
	cv::Mat x(1, (int)solved.size(), CV_64F), step(1, (int)solved.size(), CV_64F);
	for(size_t i = 0; i < solved.size(); ++i)
	{
		x.at<double>(0, i) = p[solved[i]];
		step.at<double>(0, i) = kInitialSteps[solved[i]];
	}
	// Started again from its result: a simplex can shrink before it reaches the
	// optimum, the restart gives it its full size back.
	for(int restart = 0; restart < 2; ++restart)
	{
		solver->setInitStep(step);
		solver->minimize(x);
	}
	for(size_t i = 0; i < solved.size(); ++i)
	{
		p[solved[i]] = x.at<double>(0, i);
	}
}
