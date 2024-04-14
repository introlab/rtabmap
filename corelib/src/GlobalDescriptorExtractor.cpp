/*
Copyright (c) 2010-2024, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
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


#include "rtabmap/core/GlobalDescriptorExtractor.h"

#ifdef RTABMAP_PYTHON
#include "python/PyDescriptor.h"
#endif

namespace rtabmap {

GlobalDescriptorExtractor::GlobalDescriptorExtractor(const ParametersMap & parameters)
{
}
GlobalDescriptorExtractor::~GlobalDescriptorExtractor()
{
}
GlobalDescriptorExtractor * GlobalDescriptorExtractor::create(const ParametersMap & parameters)
{
	int type = Parameters::defaultMemGlobalDescriptorStrategy();
	Parameters::parse(parameters, Parameters::kMemGlobalDescriptorStrategy(), type);
	return create((GlobalDescriptorExtractor::Type)type, parameters);
}
GlobalDescriptorExtractor * GlobalDescriptorExtractor::create(GlobalDescriptorExtractor::Type type, const ParametersMap & parameters)
{
	UDEBUG("Creating global descriptor of type %d", (int)type);
#ifndef RTABMAP_PYTHON
	if(type == GlobalDescriptorExtractor::kPyDescriptor)
	{
		UWARN("PyDescriptor cannot be used as rtabmap is not built with Python3 support.");
		type = GlobalDescriptorExtractor::kUndef;
	}
#endif

	GlobalDescriptorExtractor * GlobalDescriptorExtractor = 0;
	switch(type)
	{
#ifdef RTABMAP_PYTHON
	case GlobalDescriptorExtractor::kPyDescriptor:
		GlobalDescriptorExtractor = new PyDescriptor(parameters);
		break;
#endif
	default:
		type = GlobalDescriptorExtractor::kUndef;
		break;
	}
	return GlobalDescriptorExtractor;
}

}

