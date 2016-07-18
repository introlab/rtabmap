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


#include <rtabmap/core/RegistrationVis.h>
#include <rtabmap/core/RegistrationIcp.h>
#include <rtabmap/utilite/ULogger.h>

namespace rtabmap {

Registration * Registration::create(const ParametersMap & parameters)
{
	int regTypeInt = Parameters::defaultRegStrategy();
	Parameters::parse(parameters, Parameters::kRegStrategy(), regTypeInt);
	Registration::Type type = (Registration::Type)regTypeInt;
	return create(type, parameters);
}

Registration * Registration::create(Registration::Type & type, const ParametersMap & parameters)
{
	UDEBUG("type=%d", (int)type);
	Registration * reg = 0;
	switch(type)
	{
	case Registration::kTypeIcp:
		reg = new RegistrationIcp(parameters);
		break;
	case Registration::kTypeVisIcp:
		reg = new RegistrationVis(parameters, new RegistrationIcp(parameters));
		break;
	default: // kTypeVis
		reg = new RegistrationVis(parameters);
		type = Registration::kTypeVis;
		break;
	}
	return reg;
}

Registration::Registration(const ParametersMap & parameters, Registration * child) :
	varianceFromInliersCount_(Parameters::defaultRegVarianceFromInliersCount()),
	force3DoF_(Parameters::defaultRegForce3DoF()),
	child_(child)
{
	this->parseParameters(parameters);
}

Registration::~Registration()
{
	if(child_)
	{
		delete child_;
	}
}
void Registration::parseParameters(const ParametersMap & parameters)
{
	Parameters::parse(parameters, Parameters::kRegVarianceFromInliersCount(), varianceFromInliersCount_);
	Parameters::parse(parameters, Parameters::kRegForce3DoF(), force3DoF_);
	if(child_)
	{
		child_->parseParameters(parameters);
	}
}

bool Registration::isImageRequired() const
{
	bool val = isImageRequiredImpl();
	if(!val && child_)
	{
		val = child_->isImageRequired();
	}
	return val;
}

bool Registration::isScanRequired() const
{
	bool val = isScanRequiredImpl();
	if(!val && child_)
	{
		val = child_->isScanRequired();
	}
	return val;
}

bool Registration::isUserDataRequired() const
{
	bool val = isUserDataRequiredImpl();
	if(!val && child_)
	{
		val = child_->isUserDataRequired();
	}
	return val;
}

int Registration::getMinVisualCorrespondences() const
{
	int min = this->getMinVisualCorrespondencesImpl();
	if(child_)
	{
		int childMin = child_->getMinVisualCorrespondences();
		if(min == 0 || childMin > min)
		{
			min = childMin;
		}
	}
	return min;
}

float Registration::getMinGeometryCorrespondencesRatio() const
{
	float min = this->getMinGeometryCorrespondencesRatioImpl();
	if(child_)
	{
		float childMin = child_->getMinGeometryCorrespondencesRatio();
		if(min == 0 || childMin > min)
		{
			min = childMin;
		}
	}
	return min;
}

void Registration::setChildRegistration(Registration * child)
{
	if(child_)
	{
		delete child_;
	}
	child_ = child;
}

Transform Registration::computeTransformation(
		const Signature & from,
		const Signature & to,
		Transform guess,
		RegistrationInfo * infoOut) const
{
	Signature fromCopy(from);
	Signature toCopy(to);
	return computeTransformationMod(fromCopy, toCopy, guess, infoOut);
}

Transform Registration::computeTransformation(
		const SensorData & from,
		const SensorData & to,
		Transform guess,
		RegistrationInfo * infoOut) const
{
	Signature fromCopy(from);
	Signature toCopy(to);
	return computeTransformationMod(fromCopy, toCopy, guess, infoOut);
}

Transform Registration::computeTransformationMod(
		Signature & from,
		Signature & to,
		Transform guess,
		RegistrationInfo * infoOut) const
{
	RegistrationInfo info;
	if(infoOut)
	{
		info = *infoOut;
	}

	if(!guess.isNull() && force3DoF_)
	{
		guess = guess.to3DoF();
	}

	Transform t = computeTransformationImpl(from, to, guess, info);

	if(varianceFromInliersCount_)
	{
		if(info.icpInliersRatio)
		{
			info.variance = info.icpInliersRatio > 0?1.0/double(info.icpInliersRatio):1.0;
		}
		else
		{
			info.variance = info.inliers > 0?1.0f/float(info.inliers):1.0f;
		}
		info.variance = info.variance>0.0f?info.variance:0.0001f; // epsilon if exact transform
	}

	if(child_)
	{
		if(!t.isNull())
		{
			t = child_->computeTransformationMod(from, to, force3DoF_?t.to3DoF():t, &info);
		}
		else if(!guess.isNull())
		{
			UDEBUG("This registration approach failed, continue with the guess for the next registration");
			t = child_->computeTransformationMod(from, to, guess, &info);
		}
	}
	else if(!t.isNull() && force3DoF_)
	{
		t = t.to3DoF();
	}

	if(infoOut)
	{
		*infoOut = info;
	}
	return t;
}

}
