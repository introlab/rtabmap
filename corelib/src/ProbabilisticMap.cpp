/*
Copyright (c) 2010-2023, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
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

#include <rtabmap/core/ProbabilisticMap.h>
#include <rtabmap/utilite/ULogger.h>

namespace rtabmap {

ProbabilisticMap::ProbabilisticMap(const ParametersMap & parameters) :
	Map(parameters),
	occupancyThr_(Parameters::defaultGridGlobalOccupancyThr()),
	probHit_(logodds(Parameters::defaultGridGlobalProbHit())),
	probMiss_(logodds(Parameters::defaultGridGlobalProbMiss())),
	probClampingMin_(logodds(Parameters::defaultGridGlobalProbClampingMin())),
	probClampingMax_(logodds(Parameters::defaultGridGlobalProbClampingMax()))
{
	Parameters::parse(parameters, Parameters::kGridGlobalOccupancyThr(), occupancyThr_);
	if(Parameters::parse(parameters, Parameters::kGridGlobalProbHit(), probHit_))
	{
		probHit_ = logodds(probHit_);
		UASSERT_MSG(probHit_ >= 0.0f, uFormat("probHit_=%f",probHit_).c_str());
	}
	if(Parameters::parse(parameters, Parameters::kGridGlobalProbMiss(), probMiss_))
	{
		probMiss_ = logodds(probMiss_);
		UASSERT_MSG(probMiss_ <= 0.0f, uFormat("probMiss_=%f",probMiss_).c_str());
	}
	if(Parameters::parse(parameters, Parameters::kGridGlobalProbClampingMin(), probClampingMin_))
	{
		probClampingMin_ = logodds(probClampingMin_);
	}
	if(Parameters::parse(parameters, Parameters::kGridGlobalProbClampingMax(), probClampingMax_))
	{
		probClampingMax_ = logodds(probClampingMax_);
	}
	UASSERT(probClampingMax_ > probClampingMin_);
}

} // namespace rtabmap
