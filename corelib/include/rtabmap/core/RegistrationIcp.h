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

#ifndef REGISTRATIONICP_H_
#define REGISTRATIONICP_H_

#include <rtabmap/core/Registration.h>
#include <rtabmap/core/Signature.h>

namespace rtabmap {

// Geometrical registration
class RegistrationIcp : public Registration
{
public:
	RegistrationIcp(const ParametersMap & parameters = ParametersMap());
	virtual ~RegistrationIcp() {}

	virtual void parseParameters(const ParametersMap & parameters);

	virtual Transform computeTransformation(
			const Signature & from,
			const Signature & to,
			Transform guess = Transform::getIdentity(),
			std::string * rejectedMsg = 0,
			int * inliersOut = 0,
			float * varianceOut = 0,
			float * inliersRatioOut = 0);

	Transform computeTransformation(
			const SensorData & from,
			const SensorData & to,
			Transform guess = Transform::getIdentity(),
			std::string * rejectedMsg = 0,
			int * inliersOut = 0,
			float * varianceOut = 0,
			float * inliersRatioOut = 0);

private:
	float _icpMaxTranslation;
	float _icpMaxRotation;
	bool _icp2D;
	float _icpVoxelSize;
	int _icpDownsamplingStep;
	float _icpMaxCorrespondenceDistance;
	int _icpMaxIterations;
	float _icpCorrespondenceRatio;
	bool _icpPointToPlane;
	int _icpPointToPlaneNormalNeighbors;
};

}

#endif /* REGISTRATIONICP_H_ */
