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

#ifndef REGISTRATIONICP_H_
#define REGISTRATIONICP_H_

#include "rtabmap/core/rtabmap_core_export.h" // DLL export/import defines

#include <rtabmap/core/Registration.h>
#include <rtabmap/core/Signature.h>

namespace rtabmap {

// Geometrical registration
class RTABMAP_CORE_EXPORT RegistrationIcp : public Registration
{
public:
	// take ownership of child
	RegistrationIcp(const ParametersMap & parameters = ParametersMap(), Registration * child = 0);
	virtual ~RegistrationIcp();

	virtual void parseParameters(const ParametersMap & parameters);

protected:
	virtual Transform computeTransformationImpl(
			Signature & from,
			Signature & to,
			Transform guess,
			RegistrationInfo & info) const;
	virtual bool isScanRequiredImpl() const {return true;}
	virtual bool canUseGuessImpl() const {return true;}
	virtual float getMinGeometryCorrespondencesRatioImpl() const {return _correspondenceRatio;}

private:
	int _strategy;
	float _maxTranslation;
	float _maxRotation;
	float _voxelSize;
	int _downsamplingStep;
	float _rangeMin;
	float _rangeMax;
	float _maxCorrespondenceDistance;
	bool _reciprocalCorrespondences;
	int _maxIterations;
	float _epsilon;
	float _correspondenceRatio;
	bool _force4DoF;
	bool _pointToPlane;
	int _pointToPlaneK;
	float _pointToPlaneRadius;
	float _pointToPlaneGroundNormalsUp;
	float _pointToPlaneMinComplexity;
	int _pointToPlaneLowComplexityStrategy;
	std::string _libpointmatcherConfig;
	int _libpointmatcherKnn;
	float _libpointmatcherEpsilon;
	bool _libpointmatcherIntensity;
	float _outlierRatio;
	unsigned int _ccSamplingLimit;
	bool _ccFilterOutFarthestPoints;
	double _ccMaxFinalRMS;
	std::string _debugExportFormat;
	std::string _workingDir;

	void * _libpointmatcherICP;
	void * _libpointmatcherICPFilters;
};

}

#endif /* REGISTRATIONICP_H_ */
