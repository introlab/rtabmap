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

#include "GtsamSolver.h"

#ifdef RTABMAP_GTSAM

#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <algorithm>
#include <cmath>

namespace {

// One lidar edge point: CalibrationProblem::edgeDistance() for it, as a function of the
// solved parameters x of p.
template<int D>
class EdgeDistanceFactor : public gtsam::NoiseModelFactor1<Eigen::Matrix<double, D, 1> >
{
	typedef Eigen::Matrix<double, D, 1> X;
public:
	EdgeDistanceFactor(gtsam::Key key, const gtsam::SharedNoiseModel & model,
			const CalibrationProblem & problem, const Frame & frame, size_t point,
			const std::vector<int> & solved, const double p[6], double cap) :
		gtsam::NoiseModelFactor1<X>(model, key),
		problem_(problem), frame_(frame), point_(point), solved_(solved), cap_(cap)
	{
		std::copy(p, p + 6, p_);
	}

	gtsam::Vector evaluateError(const X & x,
#if GTSAM_VERSION_NUMERIC >= 40300
			gtsam::OptionalMatrixType H = OptionalNone) const override
#else
			boost::optional<gtsam::Matrix &> H = boost::none) const override
#endif
	{
		if(H)
		{
			// Central differences, with steps of the precision the projection needs.
			gtsam::Matrix J(1, D);
			for(int k = 0; k < D; ++k)
			{
				const double h = solved_[k] < 3 ? 1e-4 : 1e-3;  // m, deg
				X plus = x, minus = x;
				plus[k] += h;
				minus[k] -= h;
				J(0, k) = (residual(plus) - residual(minus)) / (2.0 * h);
			}
			*H = J;
		}
		return gtsam::Vector1(residual(x));
	}

private:
	double residual(const X & x) const
	{
		double q[6];
		std::copy(p_, p_ + 6, q);
		for(int k = 0; k < D; ++k)
		{
			q[solved_[k]] = x[k];
		}
		return problem_.edgeDistance(frame_, point_, q, cap_);
	}

	const CalibrationProblem & problem_;
	const Frame & frame_;
	size_t point_;
	std::vector<int> solved_;
	double p_[6];
	double cap_;
};

}  // namespace

void GtsamSolver::solve(const CalibrationProblem & problem, bool estimateTranslation, double p[6]) const
{
	coarse_.solve(problem, estimateTranslation, p);
	if(estimateTranslation)
	{
		solveWith<6>(problem, {0, 1, 2, 3, 4, 5}, p);
	}
	else
	{
		solveWith<3>(problem, {3, 4, 5}, p);
	}
}

template<int D>
void GtsamSolver::solveWith(const CalibrationProblem & problem, const std::vector<int> & solved, double p[6]) const
{
	typedef Eigen::Matrix<double, D, 1> X;
	const gtsam::Key key = 0;

	X x;
	for(int k = 0; k < D; ++k)
	{
		x[k] = p[solved[k]];
	}
	for(double scale : {9.0, 3.0, 1.0})
	{
		const double delta = scale * sigma_;
		const double cap = 10.0 * delta;  // beyond a few times the scale, a point does not count anyway
		const gtsam::noiseModel::mEstimator::Welsch::shared_ptr welsch =
				gtsam::noiseModel::mEstimator::Welsch::Create(delta);
		gtsam::NonlinearFactorGraph graph;
		for(int k : problem.nodes())
		{
			const Frame & f = problem.frames()[k];
			for(size_t i = 0; i < f.edgePoints.size(); ++i)
			{
				// Information = the point's weight, as with g2o.
				const gtsam::SharedNoiseModel model = gtsam::noiseModel::Robust::Create(
						welsch, gtsam::noiseModel::Isotropic::Sigma(1, 1.0 / std::sqrt(f.edgeWeights[i])));
				graph.emplace_shared<EdgeDistanceFactor<D> >(key, model, problem, f, i, solved, p, cap);
			}
		}
		gtsam::Values initial;
		initial.insert(key, x);
		gtsam::LevenbergMarquardtParams params;
		params.setMaxIterations(10);  // close already, after the coarse search
		x = gtsam::LevenbergMarquardtOptimizer(graph, initial, params).optimize().template at<X>(key);
	}
	for(int k = 0; k < D; ++k)
	{
		p[solved[k]] = x[k];
	}
}

#endif
