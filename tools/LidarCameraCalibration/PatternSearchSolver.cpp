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

#include "PatternSearchSolver.h"

#include <algorithm>

void PatternSearchSolver::solve(const CalibrationProblem & problem, bool estimateTranslation, double p[6]) const
{
	double steps[6];
	std::copy(kInitialSteps, kInitialSteps + 6, steps);
	if(!estimateTranslation)
	{
		steps[0] = steps[1] = steps[2] = 0.0;
	}
	double best = problem.score(correctionFrom(p));
	for(int level = 0; level < levels_; ++level)
	{
		bool improved = true;
		while(improved)
		{
			improved = false;
			for(int k = 0; k < 6; ++k)
			{
				if(steps[k] == 0.0)
				{
					continue;
				}
				for(int sign = -1; sign <= 1; sign += 2)
				{
					double q[6];
					std::copy(p, p + 6, q);
					q[k] += sign * steps[k];
					const double s = problem.score(correctionFrom(q));
					if(s > best + 1e-7)
					{
						best = s;
						std::copy(q, q + 6, p);
						improved = true;
					}
				}
			}
		}
		for(double & s : steps)
		{
			s /= 2.0;
		}
	}
}
