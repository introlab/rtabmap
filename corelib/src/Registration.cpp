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
#include <rtabmap/utilite/UTimer.h>

namespace rtabmap {

double Registration::COVARIANCE_LINEAR_EPSILON = 0.00000001;   // 0.1 mm
double Registration::COVARIANCE_ANGULAR_EPSILON = 0.00000003; // 0.01 deg

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
	repeatOnce_(Parameters::defaultRegRepeatOnce()),
	force3DoF_(Parameters::defaultRegForce3DoF()),
	child_(child)
{
	this->parseParameters(parameters);
}

Registration::~Registration()
{
	delete child_;
}
void Registration::parseParameters(const ParametersMap & parameters)
{
	Parameters::parse(parameters, Parameters::kRegRepeatOnce(), repeatOnce_);
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

bool Registration::canUseGuess() const
{
	bool val = canUseGuessImpl();
	if(!val && child_)
	{
		val = child_->canUseGuess();
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
	UTimer time;
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
	else if(repeatOnce_ && guess.isNull() && !t.isNull() && this->canUseGuess())
	{
		// redo with guess to get a more accurate transform
		t = computeTransformationImpl(from, to, t, info);

		if(!t.isNull() && force3DoF_)
		{
			t = t.to3DoF();
		}
	}
	else if(!t.isNull() && force3DoF_)
	{
		t = t.to3DoF();
	}

	if(info.covariance.empty())
	{
		info.covariance = cv::Mat::eye(6,6,CV_64FC1);
	}

	if(info.covariance.at<double>(0,0)<=COVARIANCE_LINEAR_EPSILON)
		info.covariance.at<double>(0,0) = COVARIANCE_LINEAR_EPSILON; // epsilon if exact transform
	if(info.covariance.at<double>(1,1)<=COVARIANCE_LINEAR_EPSILON)
		info.covariance.at<double>(1,1) = COVARIANCE_LINEAR_EPSILON; // epsilon if exact transform
	if(info.covariance.at<double>(2,2)<=COVARIANCE_LINEAR_EPSILON)
		info.covariance.at<double>(2,2) = COVARIANCE_LINEAR_EPSILON; // epsilon if exact transform
	if(info.covariance.at<double>(3,3)<=COVARIANCE_ANGULAR_EPSILON)
		info.covariance.at<double>(3,3) = COVARIANCE_ANGULAR_EPSILON; // epsilon if exact transform
	if(info.covariance.at<double>(4,4)<=COVARIANCE_ANGULAR_EPSILON)
		info.covariance.at<double>(4,4) = COVARIANCE_ANGULAR_EPSILON; // epsilon if exact transform
	if(info.covariance.at<double>(5,5)<=COVARIANCE_ANGULAR_EPSILON)
		info.covariance.at<double>(5,5) = COVARIANCE_ANGULAR_EPSILON; // epsilon if exact transform


	if(infoOut)
	{
		*infoOut = info;
		infoOut->totalTime = time.ticks();
	}
	return t;
}

}
