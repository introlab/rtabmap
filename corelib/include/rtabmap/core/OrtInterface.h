#pragma once

#include "rtabmap/core/rtabmap_core_export.h"

#include <onnxruntime_cxx_api.h>

namespace rtabmap {

class RTABMAP_CORE_EXPORT OrtInterface
{
public:
	OrtInterface();
	// virtual ~OrtInterface();

	Ort::Session createSession(const std::string & model_path, const Ort::SessionOptions & options = Ort::SessionOptions());

private:
	Ort::Env env_;
};

}
