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

#ifndef GLOBAL_DESCRIPTOR_EXTRACTOR_H_
#define GLOBAL_DESCRIPTOR_EXTRACTOR_H_

#include "rtabmap/core/rtabmap_core_export.h" // DLL export/import defines

#include "rtabmap/core/Parameters.h"
#include "rtabmap/core/SensorData.h"


namespace rtabmap {

// Feature2D
class RTABMAP_CORE_EXPORT GlobalDescriptorExtractor {
public:
	enum Type {
		kUndef=0,
		kPyDescriptor=1};

	static std::string typeName(Type type)
	{
		switch(type){
		case kPyDescriptor:
			return "PyDescriptor";
		default:
			return "Unknown";
		}
	}

	static GlobalDescriptorExtractor * create(const ParametersMap & parameters = ParametersMap());
	static GlobalDescriptorExtractor * create(GlobalDescriptorExtractor::Type type, const ParametersMap & parameters = ParametersMap()); // for convenience

public:
	virtual ~GlobalDescriptorExtractor();

	virtual GlobalDescriptor extract(const SensorData & data) const = 0;

	virtual void parseParameters(const ParametersMap & parameters) {}
	virtual GlobalDescriptorExtractor::Type getType() const = 0;

protected:
	GlobalDescriptorExtractor(const ParametersMap & parameters = ParametersMap());
};

}

#endif /* GLOBAL_DESCRIPTOR_EXTRACTOR_H_ */
