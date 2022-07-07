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

#ifndef OPTIMIZERCVSBA_H_
#define OPTIMIZERCVSBA_H_

#include "rtabmap/core/RtabmapExp.h" // DLL export/import defines

#include <rtabmap/core/Optimizer.h>

namespace rtabmap {

class RTABMAP_EXP OptimizerCVSBA : public Optimizer
{
public:
	static bool available();

public:
	OptimizerCVSBA(
			int iterations         = Parameters::defaultOptimizerIterations(),
			bool slam2d            = Parameters::defaultRegForce3DoF(),
			bool covarianceIgnored = Parameters::defaultOptimizerVarianceIgnored()) :
		Optimizer(iterations, slam2d, covarianceIgnored) {}
	OptimizerCVSBA(const ParametersMap & parameters) :
		Optimizer(parameters) {}
	virtual ~OptimizerCVSBA() {}

	virtual Type type() const {return kTypeCVSBA;}

	virtual std::map<int, Transform> optimizeBA(
			int rootId,
			const std::map<int, Transform> & poses,
			const std::multimap<int, Link> & links,
			const std::map<int, std::vector<CameraModel> > & models,
			std::map<int, cv::Point3f> & points3DMap,
			const std::map<int, std::map<int, FeatureBA> > & wordReferences, // <ID words, IDs frames + keypoint(x,y,depth)>
			std::set<int> * outliers = 0);
};

} /* namespace rtabmap */
#endif /* OPTIMIZERCVSBA_H_ */
