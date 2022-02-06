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

#ifndef RTABMAP_REGISTRATION_H_
#define RTABMAP_REGISTRATION_H_

#include "rtabmap/core/RtabmapExp.h" // DLL export/import defines

#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/Signature.h>
#include <rtabmap/core/RegistrationInfo.h>

namespace rtabmap {

class RTABMAP_EXP Registration
{
public:
	enum Type {
		kTypeUndef = -1,
		kTypeVis = 0,
		kTypeIcp = 1,
		kTypeVisIcp = 2
	};
	static double COVARIANCE_LINEAR_EPSILON;
	static double COVARIANCE_ANGULAR_EPSILON;

public:
	static Registration * create(const ParametersMap & parameters);
	static Registration * create(Type & type, const ParametersMap & parameters = ParametersMap());

public:
	virtual ~Registration();
	virtual void parseParameters(const ParametersMap & parameters);

	bool isImageRequired() const;
	bool isScanRequired() const;
	bool isUserDataRequired() const;

	bool canUseGuess() const;

	int getMinVisualCorrespondences() const;
	float getMinGeometryCorrespondencesRatio() const;

	bool repeatOnce() const {return repeatOnce_;}
	bool force3DoF() const {return force3DoF_;}

	// take ownership!
	void setChildRegistration(Registration * child);

	Transform computeTransformation(
			const Signature & from,
			const Signature & to,
			Transform guess = Transform::getIdentity(),
			RegistrationInfo * info = 0) const;
	Transform computeTransformation(
			const SensorData & from,
			const SensorData & to,
			Transform guess = Transform::getIdentity(),
			RegistrationInfo * info = 0) const;

	Transform computeTransformationMod(
			Signature & from,
			Signature & to,
			Transform guess = Transform::getIdentity(),
			RegistrationInfo * info = 0) const;

protected:
	// take ownership of child
	Registration(const ParametersMap & parameters = ParametersMap(), Registration * child = 0);

	// It is safe to modify the signatures in the implementation, if so, the
	// child registration will use these modifications.
	virtual Transform computeTransformationImpl(
			Signature & from,
			Signature & to,
			Transform guess,
			RegistrationInfo & info) const = 0;

	virtual bool isImageRequiredImpl() const {return false;}
	virtual bool isScanRequiredImpl() const {return false;}
	virtual bool isUserDataRequiredImpl() const {return false;}
	virtual bool canUseGuessImpl() const {return false;}
	virtual int getMinVisualCorrespondencesImpl() const {return 0;}
	virtual float getMinGeometryCorrespondencesRatioImpl() const {return 0.0f;}

private:
	bool repeatOnce_;
	bool force3DoF_;
	Registration * child_;

};

}

#endif /* RTABMAP_REGISTRATION_H_ */
