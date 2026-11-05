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

#ifndef LIDARCAMERACALIBRATION_GTSAMSOLVER_H_
#define LIDARCAMERACALIBRATION_GTSAMSOLVER_H_

#include "CorrectionSolver.h"
#include "PatternSearchSolver.h"

#include <rtabmap/core/Version.h>

#ifdef RTABMAP_GTSAM

// The same least squares as G2oSolver (see there), with GTSAM: one factor per lidar edge
// point on the correction's parameters, a Welsch robust noise model, Levenberg-Marquardt
// after a coarse pattern search, with the robust scale going from wide to narrow.
class GtsamSolver : public CorrectionSolver
{
public:
	GtsamSolver(double sigma) : sigma_(sigma), coarse_(4) {}
	const char * name() const override {return "gtsam";}
	void solve(const CalibrationProblem & problem, bool estimateTranslation, double p[6]) const override;

private:
	template<int D>
	void solveWith(const CalibrationProblem & problem, const std::vector<int> & solved, double p[6]) const;

	double sigma_;
	PatternSearchSolver coarse_;  // steps of 2 down to 0.25 deg
};

#endif

#endif /* LIDARCAMERACALIBRATION_GTSAMSOLVER_H_ */
