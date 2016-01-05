/*
Copyright (c) 2010-2014, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
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

#ifndef REGISTRATIONVIS_H_
#define REGISTRATIONVIS_H_

#include <rtabmap/core/Registration.h>
#include <rtabmap/core/Signature.h>

namespace rtabmap {

// Visual registration
class RegistrationVis : public Registration
{
public:
	RegistrationVis(const ParametersMap & parameters = ParametersMap());
	virtual ~RegistrationVis();

	virtual void parseParameters(const ParametersMap & parameters);

	virtual Transform computeTransformationMod(
			Signature & from,
			Signature & to,
			Transform guess = Transform::getIdentity(), // guess is ignored for RegistrationVis
			std::string * rejectedMsg = 0,
			std::vector<int> * inliersOut = 0,
			float * varianceOut = 0,
			float * inliersRatioOut = 0) const;

	float getBowInlierDistance() const {return _inlierDistance;}
	int getBowIterations() const {return _iterations;}
	int getBowMinInliers() const {return _minInliers;}
	bool getBowForce2D() const {return _force2D;}

private:
	int _minInliers;
	float _inlierDistance;
	int _iterations;
	int _refineIterations;
	bool _force2D;
	float _epipolarGeometryVar;
	int _estimationType;
	bool _forwardEstimateOnly;
	float _PnPReprojError;
	int _PnPFlags;
	int _PnPRefineIterations;
	int _correspondencesApproach;
	int _flowWinSize;
	int _flowIterations;
	float _flowEps;
	int _flowMaxLevel;

	ParametersMap _featureParameters;
};

}

#endif /* REGISTRATION_H_ */
