/*
 * GlobalDescriptorExtractor.cpp
 *
 *  Created on: Jul. 16, 2020
 *      Author: mathieu
 */


#include "rtabmap/core/GlobalDescriptorExtractor.h"

#ifdef RTABMAP_PYTHON3
#include "pydescriptor/PyDescriptor.h"
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
#ifndef RTABMAP_PYTHON3
	if(type == GlobalDescriptorExtractor::kPyDescriptor)
	{
		UWARN("PyDescriptor cannot be used as rtabmap is not built with Python3 support.");
		type = GlobalDescriptorExtractor::kUndef;
	}
#endif

	GlobalDescriptorExtractor * GlobalDescriptorExtractor = 0;
	switch(type)
	{
#ifdef RTABMAP_PYTHON3
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

