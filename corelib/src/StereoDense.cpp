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

#include <rtabmap/core/stereo/StereoBM.h>
#include <rtabmap/core/stereo/StereoSGBM.h>

namespace rtabmap {

StereoDense * StereoDense::create(const ParametersMap & parameters)
{
	int stereoTypeInt = Parameters::defaultStereoDenseStrategy();
	Parameters::parse(parameters, Parameters::kStereoDenseStrategy(), stereoTypeInt);
	return create((StereoDense::Type)stereoTypeInt, parameters);
}

StereoDense * StereoDense::create(StereoDense::Type type, const ParametersMap & parameters)
{
	StereoDense * stereo = 0;
	switch(type)
	{
	case StereoDense::kTypeSGBM:
		stereo = new StereoSGBM(parameters);
		break;
	case StereoDense::kTypeBM:
	default:
		stereo = new StereoBM(parameters);
		break;

	}
	return stereo;
}

} /* namespace rtabmap */
