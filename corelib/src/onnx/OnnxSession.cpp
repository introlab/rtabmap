/*
Copyright (c) 2010-2026, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
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

#include "onnx/OnnxSession.h"

#include <rtabmap/core/Parameters.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UDirectory.h>
#include <rtabmap/utilite/UFile.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UTimer.h>

#include <onnxruntime_cxx_api.h>

#include <algorithm>
#include <map>
#include <mutex>
#include <sstream>

namespace rtabmap {

//////////////////////////
// OnnxTensor
//////////////////////////
size_t OnnxTensor::count() const
{
	if(shape.empty())
	{
		return 0;
	}
	size_t n = 1;
	for(size_t i=0; i<shape.size(); ++i)
	{
		if(shape[i] < 0)
		{
			return 0;
		}
		n *= (size_t)shape[i];
	}
	return n;
}

bool OnnxTensor::isStatic() const
{
	for(size_t i=0; i<shape.size(); ++i)
	{
		if(shape[i] < 0)
		{
			return false;
		}
	}
	return true;
}

std::string OnnxTensor::typeName(Type type)
{
	switch(type)
	{
	case kFloat32: return "float32";
	case kInt64: return "int64";
	case kInt32: return "int32";
	case kUInt8: return "uint8";
	default: return "unsupported";
	}
}

std::string OnnxTensor::toString() const
{
	std::stringstream ss;
	ss << name << " " << typeName(type) << " [";
	for(size_t i=0; i<shape.size(); ++i)
	{
		ss << (i?",":"") << shape[i];
	}
	ss << "]";
	return ss.str();
}

//////////////////////////
// OnnxSession
//////////////////////////
struct OnnxSession::Impl
{
	std::unique_ptr<Ort::Session> session;
	std::vector<std::string> inputNames;
	std::vector<std::string> outputNames;
};

namespace {

// The one ONNX Runtime environment of the process. It must outlive every
// session, and sessions may be held by objects destroyed at exit in any
// order, so it is leaked on purpose instead of being a function-local static.
Ort::Env & ortEnv()
{
	static std::mutex mutex;
	static Ort::Env * env = 0;
	std::lock_guard<std::mutex> lock(mutex);
	if(env == 0)
	{
		OrtLoggingLevel level = ORT_LOGGING_LEVEL_WARNING;
		switch(ULogger::level())
		{
		case ULogger::kDebug:   level = ORT_LOGGING_LEVEL_INFO; break;
		case ULogger::kInfo:    level = ORT_LOGGING_LEVEL_WARNING; break;
		case ULogger::kWarning: level = ORT_LOGGING_LEVEL_WARNING; break;
		case ULogger::kError:   level = ORT_LOGGING_LEVEL_ERROR; break;
		case ULogger::kFatal:   level = ORT_LOGGING_LEVEL_FATAL; break;
		}
		env = new Ort::Env(level, "rtabmap");
	}
	return *env;
}

OnnxTensor::Type fromOrtType(ONNXTensorElementDataType type)
{
	switch(type)
	{
	case ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT: return OnnxTensor::kFloat32;
	case ONNX_TENSOR_ELEMENT_DATA_TYPE_INT64: return OnnxTensor::kInt64;
	case ONNX_TENSOR_ELEMENT_DATA_TYPE_INT32: return OnnxTensor::kInt32;
	case ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT8: return OnnxTensor::kUInt8;
	default: return OnnxTensor::kUnsupported;
	}
}

// Name, type and shape of the model's inputs or outputs.
std::vector<OnnxTensor> describe(const Ort::Session & session, bool inputs, std::vector<std::string> & names)
{
	std::vector<OnnxTensor> tensors;
	Ort::AllocatorWithDefaultOptions allocator;
	size_t count = inputs?session.GetInputCount():session.GetOutputCount();
	for(size_t i=0; i<count; ++i)
	{
		OnnxTensor t;
		Ort::AllocatedStringPtr name = inputs?session.GetInputNameAllocated(i, allocator):session.GetOutputNameAllocated(i, allocator);
		t.name = name.get();
		Ort::TypeInfo info = inputs?session.GetInputTypeInfo(i):session.GetOutputTypeInfo(i);
		if(info.GetONNXType() == ONNX_TYPE_TENSOR)
		{
			Ort::ConstTensorTypeAndShapeInfo tensorInfo = info.GetTensorTypeAndShapeInfo();
			t.type = fromOrtType(tensorInfo.GetElementType());
			t.shape = tensorInfo.GetShape();
		}
		else
		{
			t.type = OnnxTensor::kUnsupported;
		}
		names.push_back(t.name);
		tensors.push_back(t);
	}
	return tensors;
}

// Not owning: the OnnxTensor's data must outlive the returned value.
Ort::Value toOrtValue(const Ort::MemoryInfo & memoryInfo, OnnxTensor & t)
{
	switch(t.type)
	{
	case OnnxTensor::kFloat32:
		return Ort::Value::CreateTensor<float>(memoryInfo, t.f.data(), t.f.size(), t.shape.data(), t.shape.size());
	case OnnxTensor::kInt64:
		return Ort::Value::CreateTensor<int64_t>(memoryInfo, t.i64.data(), t.i64.size(), t.shape.data(), t.shape.size());
	case OnnxTensor::kInt32:
		return Ort::Value::CreateTensor<int32_t>(memoryInfo, t.i32.data(), t.i32.size(), t.shape.data(), t.shape.size());
	case OnnxTensor::kUInt8:
		return Ort::Value::CreateTensor<uint8_t>(memoryInfo, t.u8.data(), t.u8.size(), t.shape.data(), t.shape.size());
	default:
		return Ort::Value(nullptr);
	}
}

bool fromOrtValue(const Ort::Value & value, OnnxTensor & t)
{
	if(!value.IsTensor())
	{
		UERROR("Output \"%s\" is not a tensor.", t.name.c_str());
		return false;
	}
	Ort::TensorTypeAndShapeInfo info = value.GetTensorTypeAndShapeInfo();
	t.shape = info.GetShape();
	t.type = fromOrtType(info.GetElementType());
	size_t count = info.GetElementCount();
	switch(t.type)
	{
	case OnnxTensor::kFloat32:
		t.f.assign(value.GetTensorData<float>(), value.GetTensorData<float>() + count);
		break;
	case OnnxTensor::kInt64:
		t.i64.assign(value.GetTensorData<int64_t>(), value.GetTensorData<int64_t>() + count);
		break;
	case OnnxTensor::kInt32:
		t.i32.assign(value.GetTensorData<int32_t>(), value.GetTensorData<int32_t>() + count);
		break;
	case OnnxTensor::kUInt8:
		t.u8.assign(value.GetTensorData<uint8_t>(), value.GetTensorData<uint8_t>() + count);
		break;
	default:
		UERROR("Output \"%s\" has an element type (%d) not supported by OnnxSession.", t.name.c_str(), (int)info.GetElementType());
		return false;
	}
	return true;
}

std::string ortProviderName(OnnxSession::Provider provider)
{
	switch(provider)
	{
	case OnnxSession::kProviderCUDA: return "CUDAExecutionProvider";
	case OnnxSession::kProviderTensorRT: return "TensorrtExecutionProvider";
	default: return "CPUExecutionProvider";
	}
}

// Updates a V2 provider options struct from (key, value) pairs.
template<typename OptionsT>
void updateProviderOptions(
		OptionsT * options,
		OrtStatus * (ORT_API_CALL * update)(OptionsT *, const char * const *, const char * const *, size_t),
		const std::vector<std::pair<std::string, std::string> > & keyValues)
{
	std::vector<const char *> keys;
	std::vector<const char *> values;
	for(size_t i=0; i<keyValues.size(); ++i)
	{
		keys.push_back(keyValues[i].first.c_str());
		values.push_back(keyValues[i].second.c_str());
	}
	Ort::ThrowOnError(update(options, keys.data(), values.data(), keys.size()));
}

} // namespace

std::string OnnxSession::Options::key() const
{
	return uFormat("provider=%d;device=%d;fp16=%d;cache=%s", (int)provider, deviceId, tensorRTFp16?1:0, tensorRTCachePath.c_str());
}

bool OnnxSession::isProviderAvailable(Provider provider)
{
	if(provider == kProviderCPU)
	{
		return true;
	}
	try
	{
		std::vector<std::string> providers = Ort::GetAvailableProviders();
		return std::find(providers.begin(), providers.end(), ortProviderName(provider)) != providers.end();
	}
	catch(const Ort::Exception & e)
	{
		UERROR("%s", e.what());
		return false;
	}
}

std::string OnnxSession::providerName(Provider provider)
{
	switch(provider)
	{
	case kProviderCUDA: return "CUDA";
	case kProviderTensorRT: return "TensorRT";
	default: return "CPU";
	}
}

std::string OnnxSession::runtimeVersion()
{
	return Ort::GetVersionString();
}

std::string OnnxSession::defaultTensorRTCachePath(const ParametersMap & parameters)
{
	std::string workingDir;
	Parameters::parse(parameters, Parameters::kRtabmapWorkingDirectory(), workingDir);
	if(workingDir.empty())
	{
		workingDir = Parameters::createDefaultWorkingDirectory();
	}
	std::string path = workingDir + "/tensorrt_cache";
	if(!UDirectory::exists(path))
	{
		UDirectory::makeDir(path);
	}
	return path;
}

std::shared_ptr<OnnxSession> OnnxSession::get(const std::string & modelPath, const Options & options)
{
	static std::mutex mutex;
	static std::map<std::string, std::weak_ptr<OnnxSession> > cache;

	std::lock_guard<std::mutex> lock(mutex);
	std::string key = modelPath + ";" + options.key();
	std::map<std::string, std::weak_ptr<OnnxSession> >::iterator iter = cache.find(key);
	if(iter != cache.end())
	{
		std::shared_ptr<OnnxSession> session = iter->second.lock();
		if(session.get())
		{
			UDEBUG("Reusing session of \"%s\" (%s)", modelPath.c_str(), options.key().c_str());
			return session;
		}
		cache.erase(iter);
	}

	std::shared_ptr<OnnxSession> session(new OnnxSession(modelPath, options));
	if(!session->load())
	{
		return std::shared_ptr<OnnxSession>();
	}
	cache.insert(std::make_pair(key, std::weak_ptr<OnnxSession>(session)));
	return session;
}

OnnxSession::OnnxSession(const std::string & modelPath, const Options & options) :
		impl_(new Impl),
		modelPath_(modelPath),
		options_(options),
		provider_(kProviderCPU)
{
}

OnnxSession::~OnnxSession()
{
}

bool OnnxSession::load()
{
	if(modelPath_.empty())
	{
		UERROR("The ONNX model path is empty.");
		return false;
	}
	if(!UFile::exists(modelPath_))
	{
		UERROR("The ONNX model \"%s\" doesn't exist.", modelPath_.c_str());
		return false;
	}

	UTimer timer;
	// The environment (and its logger) must exist before the provider options
	// are created: CreateTensorRTProviderOptions() fails with "Attempt to use
	// DefaultLogger but none has been registered" otherwise, which happened
	// when a TensorRT session was the first one of the process.
	Ort::Env & env = ortEnv();
	// The requested provider first, CPU if it cannot be used: a provider
	// listed by the library can still fail to load its dependencies (e.g.
	// cuDNN) or to find a device.
	Provider requested = options_.provider;
	for(int attempt = 0; attempt < 2; ++attempt)
	{
		try
		{
			Ort::SessionOptions sessionOptions;
			sessionOptions.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

			// Providers are tried from the requested one down to CPU, which is
			// always there: TensorRT runs the nodes it supports and hands the
			// others to CUDA, which hands its own leftovers to CPU.
			provider_ = kProviderCPU;
			if(requested == kProviderTensorRT)
			{
				if(isProviderAvailable(kProviderTensorRT))
				{
					OrtTensorRTProviderOptionsV2 * trtOptions = 0;
					Ort::ThrowOnError(Ort::GetApi().CreateTensorRTProviderOptions(&trtOptions));
					std::unique_ptr<OrtTensorRTProviderOptionsV2, decltype(Ort::GetApi().ReleaseTensorRTProviderOptions)> trtOptionsGuard(trtOptions, Ort::GetApi().ReleaseTensorRTProviderOptions);
					std::vector<std::pair<std::string, std::string> > keyValues;
					keyValues.push_back(std::make_pair("device_id", uNumber2Str(options_.deviceId)));
					keyValues.push_back(std::make_pair("trt_fp16_enable", options_.tensorRTFp16?"1":"0"));
					if(!options_.tensorRTCachePath.empty())
					{
						// The engine file names carry the graph hash, the precision
						// ("_fp16") and the GPU architecture, so FP32 and FP16 engines
						// of the same model live side by side in this folder.
						keyValues.push_back(std::make_pair("trt_engine_cache_enable", "1"));
						keyValues.push_back(std::make_pair("trt_engine_cache_path", options_.tensorRTCachePath));
						keyValues.push_back(std::make_pair("trt_timing_cache_enable", "1"));
						keyValues.push_back(std::make_pair("trt_timing_cache_path", options_.tensorRTCachePath));
					}
					updateProviderOptions(trtOptions, Ort::GetApi().UpdateTensorRTProviderOptions, keyValues);
					sessionOptions.AppendExecutionProvider_TensorRT_V2(*trtOptions);
					provider_ = kProviderTensorRT;
				}
				else
				{
					UWARN("TensorRT execution provider is not available in this ONNX Runtime library, falling back to %s.",
							isProviderAvailable(kProviderCUDA)?"CUDA":"CPU");
				}
			}
			if(requested == kProviderCUDA || requested == kProviderTensorRT)
			{
				if(isProviderAvailable(kProviderCUDA))
				{
					OrtCUDAProviderOptionsV2 * cudaOptions = 0;
					Ort::ThrowOnError(Ort::GetApi().CreateCUDAProviderOptions(&cudaOptions));
					std::unique_ptr<OrtCUDAProviderOptionsV2, decltype(Ort::GetApi().ReleaseCUDAProviderOptions)> cudaOptionsGuard(cudaOptions, Ort::GetApi().ReleaseCUDAProviderOptions);
					std::vector<std::pair<std::string, std::string> > keyValues;
					keyValues.push_back(std::make_pair("device_id", uNumber2Str(options_.deviceId)));
					updateProviderOptions(cudaOptions, Ort::GetApi().UpdateCUDAProviderOptions, keyValues);
					sessionOptions.AppendExecutionProvider_CUDA_V2(*cudaOptions);
					if(provider_ == kProviderCPU)
					{
						provider_ = kProviderCUDA;
					}
				}
				else if(requested == kProviderCUDA)
				{
					UWARN("CUDA execution provider is not available in this ONNX Runtime library, falling back to CPU.");
				}
			}

			impl_->session.reset(new Ort::Session(env, modelPath_.c_str(), sessionOptions));
			inputs_ = describe(*impl_->session, true, impl_->inputNames);
			outputs_ = describe(*impl_->session, false, impl_->outputNames);
			if(provider_ == kProviderTensorRT)
			{
				if(options_.tensorRTCachePath.empty())
				{
					UWARN("ONNX model \"%s\" runs with TensorRT without an engine cache directory: "
							"the engines are built again at every start, which takes seconds to minutes.",
							modelPath_.c_str());
				}
				else
				{
					UINFO("TensorRT engines of ONNX model \"%s\" are cached in \"%s\".", modelPath_.c_str(), options_.tensorRTCachePath.c_str());
				}
			}
			break;
		}
		catch(const Ort::Exception & e)
		{
			impl_->session.reset();
			inputs_.clear();
			outputs_.clear();
			if(requested != kProviderCPU)
			{
				UWARN("Failed to load ONNX model \"%s\" with %s provider (%s), falling back to CPU.",
						modelPath_.c_str(), providerName(requested).c_str(), e.what());
				requested = kProviderCPU;
				continue;
			}
			UERROR("Failed to load ONNX model \"%s\": %s", modelPath_.c_str(), e.what());
			return false;
		}
	}

	if(ULogger::level() <= ULogger::kInfo)
	{
		std::stringstream ss;
		for(size_t i=0; i<inputs_.size(); ++i)
		{
			ss << (i?", ":"") << inputs_[i].toString();
		}
		ss << " -> ";
		for(size_t i=0; i<outputs_.size(); ++i)
		{
			ss << (i?", ":"") << outputs_[i].toString();
		}
		UINFO("Loaded ONNX model \"%s\" with %s provider in %fs: %s",
				modelPath_.c_str(), providerName(provider_).c_str(), timer.ticks(), ss.str().c_str());
	}
	return true;
}

int OnnxSession::inputIndex(const std::string & name) const
{
	for(size_t i=0; i<inputs_.size(); ++i)
	{
		if(inputs_[i].name.compare(name) == 0)
		{
			return (int)i;
		}
	}
	return -1;
}

int OnnxSession::outputIndex(const std::string & name) const
{
	for(size_t i=0; i<outputs_.size(); ++i)
	{
		if(outputs_[i].name.compare(name) == 0)
		{
			return (int)i;
		}
	}
	return -1;
}

bool OnnxSession::run(const std::vector<OnnxTensor> & inputs, std::vector<OnnxTensor> & outputs) const
{
	outputs.clear();
	if(impl_->session.get() == 0)
	{
		UERROR("ONNX model \"%s\" is not loaded.", modelPath_.c_str());
		return false;
	}
	if(inputs.size() != inputs_.size())
	{
		UERROR("ONNX model \"%s\" expects %d inputs, %d provided.", modelPath_.c_str(), (int)inputs_.size(), (int)inputs.size());
		return false;
	}

	// Inputs in the model's order. CreateTensor() doesn't copy the data, so
	// they are copied here first to be sure they stay alive during Run().
	std::vector<OnnxTensor> ordered(inputs_.size());
	for(size_t i=0; i<inputs.size(); ++i)
	{
		int index = (int)i;
		if(!inputs[i].name.empty())
		{
			index = inputIndex(inputs[i].name);
			if(index < 0)
			{
				UERROR("ONNX model \"%s\" has no input named \"%s\".", modelPath_.c_str(), inputs[i].name.c_str());
				return false;
			}
		}
		if(inputs[i].count() == 0)
		{
			UERROR("Input \"%s\" of ONNX model \"%s\" has an empty or dynamic shape (%s).", inputs_[index].name.c_str(), modelPath_.c_str(), inputs[i].toString().c_str());
			return false;
		}
		if(inputs[i].type != inputs_[index].type)
		{
			UERROR("Input \"%s\" of ONNX model \"%s\" expects %s, %s provided.", inputs_[index].name.c_str(), modelPath_.c_str(),
					OnnxTensor::typeName(inputs_[index].type).c_str(), OnnxTensor::typeName(inputs[i].type).c_str());
			return false;
		}
		ordered[index] = inputs[i];
	}

	try
	{
		Ort::MemoryInfo memoryInfo = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
		std::vector<Ort::Value> values;
		std::vector<const char *> inputNames;
		std::vector<const char *> outputNames;
		for(size_t i=0; i<ordered.size(); ++i)
		{
			values.push_back(toOrtValue(memoryInfo, ordered[i]));
			inputNames.push_back(impl_->inputNames[i].c_str());
		}
		for(size_t i=0; i<impl_->outputNames.size(); ++i)
		{
			outputNames.push_back(impl_->outputNames[i].c_str());
		}

		UTimer timer;
		std::vector<Ort::Value> results = impl_->session->Run(
				Ort::RunOptions{nullptr},
				inputNames.data(), values.data(), values.size(),
				outputNames.data(), outputNames.size());
		// TensorRT builds an engine on the first inference, and again when an
		// input shape falls outside the ones seen so far (it extends the engine's
		// range): a run that takes seconds is one of these, worth knowing about.
		if(provider_ == kProviderTensorRT && timer.elapsed() > 1.0)
		{
			UWARN("TensorRT inference with ONNX model \"%s\" took %.1fs: an engine was built (first inference, or "
					"new input shapes). %s.",
					modelPath_.c_str(), timer.elapsed(),
					options_.tensorRTCachePath.empty()?
							"No cache directory is set, it will be built again at the next start":
							uFormat("It is cached in \"%s\" for the next runs", options_.tensorRTCachePath.c_str()).c_str());
		}

		outputs.resize(results.size());
		for(size_t i=0; i<results.size(); ++i)
		{
			outputs[i].name = impl_->outputNames[i];
			if(!fromOrtValue(results[i], outputs[i]))
			{
				outputs.clear();
				return false;
			}
		}
	}
	catch(const Ort::Exception & e)
	{
		UERROR("Inference with ONNX model \"%s\" failed: %s", modelPath_.c_str(), e.what());
		outputs.clear();
		return false;
	}
	return true;
}

} // namespace rtabmap
