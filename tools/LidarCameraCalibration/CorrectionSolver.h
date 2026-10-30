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

#ifndef LIDARCAMERACALIBRATION_CORRECTIONSOLVER_H_
#define LIDARCAMERACALIBRATION_CORRECTIONSOLVER_H_

#include "CalibrationProblem.h"

#include <memory>
#include <string>
#include <vector>

// Parameters: p = tx ty tz (m), rx ry rz (deg), in the camera frame (see correctionFrom()).
// Initial search steps, also the scale of each parameter.
extern const double kInitialSteps[6];

// Finds the correction best aligning the problem's lidar edges with its images' edges.
// A new solver implements this and is added to createSolver().
class CorrectionSolver
{
public:
	virtual ~CorrectionSolver() {}
	virtual const char * name() const = 0;
	// Improves p from its value. Without estimateTranslation, p's translation is left as is.
	virtual void solve(const CalibrationProblem & problem, bool estimateTranslation, double p[6]) const = 0;
};

// The solver named "pattern", "simplex", "g2o" or "gtsam" (the last two if rtabmap was
// built with them), or null. sigma: the image edges' fall off (pixels).
std::unique_ptr<CorrectionSolver> createSolver(const std::string & name, float sigma);
// The names createSolver() knows in this build.
std::vector<std::string> availableSolvers();

#endif /* LIDARCAMERACALIBRATION_CORRECTIONSOLVER_H_ */
