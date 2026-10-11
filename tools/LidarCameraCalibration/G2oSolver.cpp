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

#include "G2oSolver.h"

#ifdef RTABMAP_G2O

#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>

#include <algorithm>

namespace {

// The solved parameters of p (the rotation, and the translation if estimated).
template<int D>
class VertexCorrection : public g2o::BaseVertex<D, Eigen::Matrix<double, D, 1> >
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	void setToOriginImpl() override {this->_estimate.setZero();}
	void oplusImpl(const double * update) override  // as rtabmap's own g2o vertices
	{
		for(int i = 0; i < D; ++i)
		{
			this->_estimate[i] += update[i];
		}
	}
	bool read(std::istream &) override {return false;}
	bool write(std::ostream &) const override {return true;}
};

// One lidar edge point: CalibrationProblem::edgeDistance() for it.
template<int D>
class EdgeToImageEdge : public g2o::BaseUnaryEdge<1, double, VertexCorrection<D> >
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	EdgeToImageEdge(const CalibrationProblem & problem, const Frame & frame, size_t point,
			const std::vector<int> & solved, const double p[6], double cap) :
		problem_(problem), frame_(frame), point_(point), solved_(solved), cap_(cap)
	{
		std::copy(p, p + 6, p_);
	}

	void computeError() override
	{
		this->_error[0] = residual(static_cast<const VertexCorrection<D> *>(this->_vertices[0])->estimate());
	}

	// Central differences, with steps of the precision the projection needs rather than
	// g2o's default (1e-9), lost in the image's distance map.
	void linearizeOplus() override
	{
		const Eigen::Matrix<double, D, 1> x = static_cast<const VertexCorrection<D> *>(this->_vertices[0])->estimate();
		for(int k = 0; k < D; ++k)
		{
			const double h = solved_[k] < 3 ? 1e-4 : 1e-3;  // m, deg
			Eigen::Matrix<double, D, 1> plus = x, minus = x;
			plus[k] += h;
			minus[k] -= h;
			this->_jacobianOplusXi(0, k) = (residual(plus) - residual(minus)) / (2.0 * h);
		}
	}

	bool read(std::istream &) override {return false;}
	bool write(std::ostream &) const override {return true;}

private:
	double residual(const Eigen::Matrix<double, D, 1> & x) const
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

void G2oSolver::solve(const CalibrationProblem & problem, bool estimateTranslation, double p[6]) const
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
void G2oSolver::solveWith(const CalibrationProblem & problem, const std::vector<int> & solved, double p[6]) const
{
	typedef g2o::BlockSolver<g2o::BlockSolverTraits<D, 1> > Block;
	typedef g2o::LinearSolverEigen<typename Block::PoseMatrixType> Linear;

	Eigen::Matrix<double, D, 1> x;
	for(int k = 0; k < D; ++k)
	{
		x[k] = p[solved[k]];
	}
	for(double scale : {9.0, 3.0, 1.0})
	{
		const double delta = scale * sigma_;
		g2o::SparseOptimizer optimizer;
#ifdef RTABMAP_G2O_CPP11
		optimizer.setAlgorithm(new g2o::OptimizationAlgorithmLevenberg(
				std::unique_ptr<Block>(new Block(std::unique_ptr<Linear>(new Linear())))));
#else
		optimizer.setAlgorithm(new g2o::OptimizationAlgorithmLevenberg(new Block(new Linear())));
#endif
		VertexCorrection<D> * vertex = new VertexCorrection<D>();
		vertex->setEstimate(x);
		vertex->setId(0);
		optimizer.addVertex(vertex);

		// Beyond a few times the kernel's scale, a point does not count anyway.
		const double cap = 10.0 * delta;
		int id = 1;
		for(int k : problem.nodes())
		{
			const Frame & f = problem.frames()[k];
			for(size_t i = 0; i < f.edgePoints.size(); ++i)
			{
				EdgeToImageEdge<D> * edge = new EdgeToImageEdge<D>(problem, f, i, solved, p, cap);
				edge->setId(id++);
				edge->setVertex(0, vertex);
				edge->setMeasurement(0.0);
				edge->setInformation(Eigen::Matrix<double, 1, 1>::Constant(f.edgeWeights[i]));
				g2o::RobustKernelWelsch * kernel = new g2o::RobustKernelWelsch();
				kernel->setDelta(delta);
				edge->setRobustKernel(kernel);
				optimizer.addEdge(edge);
			}
		}
		optimizer.initializeOptimization();
		optimizer.optimize(10);  // close already, after the coarse search
		x = vertex->estimate();
	}
	for(int k = 0; k < D; ++k)
	{
		p[solved[k]] = x[k];
	}
}

#endif
