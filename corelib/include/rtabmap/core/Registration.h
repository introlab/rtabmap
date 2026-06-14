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

#include "rtabmap/core/rtabmap_core_export.h" // DLL export/import defines

#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/Signature.h>
#include <rtabmap/core/RegistrationInfo.h>

namespace rtabmap {

/**
 * @class Registration
 * @brief Abstract base for registering two observations (visual, ICP, or both).
 *
 * Factory @ref create() builds @ref RegistrationVis, @ref RegistrationIcp, or
 * @ref RegistrationVisIcp from **Reg/Strategy** in a @ref ParametersMap.
 *
 * @ref computeTransformation() wraps signatures and delegates to
 * @ref computeTransformationMod(), which calls @ref computeTransformationImpl()
 * on the concrete strategy, optionally chains a @ref child_ registration (e.g. ICP
 * after visual), applies **Reg/Force3DoF**, and may repeat once with the first
 * result as guess when **Reg/RepeatOnce** is enabled.
 *
 * Requirement queries (@ref isImageRequired(), @ref isScanRequired(), etc.) OR-combine
 * with the child when the parent returns false. Minimum correspondence thresholds
 * take the maximum between parent and child.
 *
 * @see RegistrationVis
 * @see RegistrationIcp
 * @see RegistrationInfo
 */
class RTABMAP_CORE_EXPORT Registration
{
public:
	/** @brief Registration strategy selected by **Reg/Strategy**. */
	enum Type {
		kTypeUndef = -1,
		kTypeVis = 0,
		kTypeIcp = 1,
		kTypeVisIcp = 2
	};
	/** @brief Minimum diagonal value for linear covariance (m²). */
	static double COVARIANCE_LINEAR_EPSILON;
	/** @brief Minimum diagonal value for angular covariance (rad²). */
	static double COVARIANCE_ANGULAR_EPSILON;

public:
	/** @brief Creates a registration from **Reg/Strategy** in @p parameters. Caller owns the pointer. */
	static Registration * create(const ParametersMap & parameters);
	/** @brief Creates a registration of @p type (may be adjusted, e.g. to @ref kTypeVis). Caller owns the pointer. */
	static Registration * create(Type & type, const ParametersMap & parameters = ParametersMap());

public:
	virtual ~Registration();
	/** @brief Parses **Reg/RepeatOnce**, **Reg/Force3DoF** and forwards to @ref child_ if set. */
	virtual void parseParameters(const ParametersMap & parameters);

	/** @brief True if images are needed (parent or child). */
	bool isImageRequired() const;
	/** @brief True if laser scans are needed (parent or child). */
	bool isScanRequired() const;
	/** @brief True if user data is needed (parent or child). */
	bool isUserDataRequired() const;

	/** @brief True if the strategy can refine an initial guess (parent or child). */
	bool canUseGuess() const;

	/** @brief Minimum visual inliers required (max of parent and child thresholds). */
	int getMinVisualCorrespondences() const;
	/** @brief Minimum geometry inlier ratio required (max of parent and child thresholds). */
	float getMinGeometryCorrespondencesRatio() const;

	/** @return Value of **Reg/RepeatOnce**. */
	bool repeatOnce() const {return repeatOnce_;}
	/** @return Value of **Reg/Force3DoF**. */
	bool force3DoF() const {return force3DoF_;}

	/** @brief Replaces @ref child_; takes ownership of @p child. */
	void setChildRegistration(Registration * child);

	/** @brief Registers @p from to @p to using immutable signatures (copied internally). */
	Transform computeTransformation(
			const Signature & from,
			const Signature & to,
			Transform guess = Transform::getIdentity(),
			RegistrationInfo * info = 0) const;
	/** @brief Registers two @ref SensorData observations (wrapped as signatures). */
	Transform computeTransformation(
			const SensorData & from,
			const SensorData & to,
			Transform guess = Transform::getIdentity(),
			RegistrationInfo * info = 0) const;

	/** @brief Registers @p from to @p to; signatures may be modified by the implementation. */
	Transform computeTransformationMod(
			Signature & from,
			Signature & to,
			Transform guess = Transform::getIdentity(),
			RegistrationInfo * info = 0) const;

protected:
	/** @brief @p child is owned and deleted in the destructor. */
	Registration(const ParametersMap & parameters = ParametersMap(), Registration * child = 0);

	/**
	 * @brief Strategy-specific registration.
	 *
	 * May modify @p from and @p to; a @ref child_ registration reuses those changes.
	 */
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
