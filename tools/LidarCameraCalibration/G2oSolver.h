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

#ifndef LIDARCAMERACALIBRATION_G2OSOLVER_H_
#define LIDARCAMERACALIBRATION_G2OSOLVER_H_

#include "CorrectionSolver.h"
#include "PatternSearchSolver.h"

#include <rtabmap/core/Version.h>

#ifdef RTABMAP_G2O

// Least squares with g2o: Levenberg-Marquardt on the distances of the lidar edge points
// to the images' edges (chamfer matching), weighted by the points' weights.
//
// - A redescending robust kernel (Welsch) makes a point far from any image edge count for
//   nothing, as the score does: many lidar edges have no counterpart in the image
//   (speckle, surfaces the camera does not see the same way).
// - Levenberg-Marquardt only follows the local slope, and each point is pulled toward its
//   nearest image edge, often not its own when far from the solution: from a few degrees
//   away, it stops in a local minimum. So a coarse pattern search gets close first, then
//   the kernel's scale goes from wide to narrow (9, 3, then 1 x sigma).
class G2oSolver : public CorrectionSolver
{
public:
	G2oSolver(double sigma) : sigma_(sigma), coarse_(4) {}
	const char * name() const override {return "g2o";}
	void solve(const CalibrationProblem & problem, bool estimateTranslation, double p[6]) const override;

private:
	template<int D>
	void solveWith(const CalibrationProblem & problem, const std::vector<int> & solved, double p[6]) const;

	double sigma_;
	PatternSearchSolver coarse_;  // steps of 2 down to 0.25 deg
};

#endif

#endif /* LIDARCAMERACALIBRATION_G2OSOLVER_H_ */
