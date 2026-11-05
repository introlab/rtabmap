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

#include "CorrectionSolver.h"
#include "DownhillSimplexSolver.h"
#include "G2oSolver.h"
#include "GtsamSolver.h"
#include "PatternSearchSolver.h"

#include <rtabmap/core/Version.h>

const double kInitialSteps[6] = {0.04, 0.04, 0.04, 2.0, 2.0, 2.0};

std::unique_ptr<CorrectionSolver> createSolver(const std::string & name, float sigma)
{
	(void)sigma;  // used only by the least-squares solvers
	if(name == "pattern")
	{
		return std::unique_ptr<CorrectionSolver>(new PatternSearchSolver());
	}
	if(name == "simplex")
	{
		return std::unique_ptr<CorrectionSolver>(new DownhillSimplexSolver());
	}
#ifdef RTABMAP_G2O
	if(name == "g2o")
	{
		return std::unique_ptr<CorrectionSolver>(new G2oSolver(sigma));
	}
#endif
#ifdef RTABMAP_GTSAM
	if(name == "gtsam")
	{
		return std::unique_ptr<CorrectionSolver>(new GtsamSolver(sigma));
	}
#endif
	return std::unique_ptr<CorrectionSolver>();
}

std::vector<std::string> availableSolvers()
{
	std::vector<std::string> names = {"simplex", "pattern"};
#ifdef RTABMAP_G2O
	names.push_back("g2o");
#endif
#ifdef RTABMAP_GTSAM
	names.push_back("gtsam");
#endif
	return names;
}
