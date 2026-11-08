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
#include <rtabmap/utilite/UStl.h>

#include <onnxruntime_cxx_api.h>

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <map>
#include <set>
#include <unordered_map>
#include <mutex>
#include <cstring>
#include <sstream>
#include <thread>

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

	// The range of each input dimension run so far. Without profile shapes,
	// TensorRT builds its engine at the first inference and again when an
	// input is outside the shapes it was built for.
	std::mutex shapesMutex;
	std::vector<std::vector<int64_t> > minShapes;
	std::vector<std::vector<int64_t> > maxShapes;

	/** @brief Extends the ranges with @p inputs, true if they were outside (or the first ones). */
	bool extendShapes(const std::vector<OnnxTensor> & inputs)
	{
		std::lock_guard<std::mutex> lock(shapesMutex);
		bool extended = minShapes.size() != inputs.size();
		if(extended)
		{
			minShapes.resize(inputs.size());
			maxShapes.resize(inputs.size());
		}
		for(size_t i=0; i<inputs.size(); ++i)
		{
			const std::vector<int64_t> & shape = inputs[i].shape;
			if(minShapes[i].size() != shape.size())
			{
				minShapes[i] = maxShapes[i] = shape;
				extended = true;
				continue;
			}
			for(size_t j=0; j<shape.size(); ++j)
			{
				if(shape[j] < minShapes[i][j] || shape[j] > maxShapes[i][j])
				{
					minShapes[i][j] = std::min(minShapes[i][j], shape[j]);
					maxShapes[i][j] = std::max(maxShapes[i][j], shape[j]);
					extended = true;
				}
			}
		}
		return extended;
	}
};

namespace {

// ONNX Runtime log messages go through ULogger at the matching level. The
// TensorRT parser's "Make sure output <name> has Int64 binding" reminder,
// relayed as a warning on every session creation for the int64 outputs, is
// demoted to INFO: ONNX Runtime binds them correctly.
void ortLog(void *, OrtLoggingLevel severity, const char * category, const char *, const char *, const char * message)
{
	const bool int64Reminder = message != 0 && strstr(message, "has Int64 binding") != 0;
	switch(severity)
	{
	case ORT_LOGGING_LEVEL_VERBOSE:
		UDEBUG("onnxruntime [%s]: %s", category, message);
		break;
	case ORT_LOGGING_LEVEL_INFO:
		UINFO("onnxruntime [%s]: %s", category, message);
		break;
	case ORT_LOGGING_LEVEL_WARNING:
		if(int64Reminder)
		{
			UINFO("onnxruntime [%s]: %s", category, message);
		}
		else
		{
			UWARN("onnxruntime [%s]: %s", category, message);
		}
		break;
	default:
		UERROR("onnxruntime [%s]: %s", category, message);
		break;
	}
}

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
		env = new Ort::Env(level, "rtabmap", &ortLog, 0);
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
	case OnnxSession::kProviderOpenVINO: return "OpenVINOExecutionProvider";
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

// The <folder> folder of the RTAB-Map working directory, created if needed.
std::string workingDirectoryCachePath(const ParametersMap & parameters, const std::string & folder)
{
	std::string workingDir;
	Parameters::parse(parameters, Parameters::kRtabmapWorkingDirectory(), workingDir);
	if(workingDir.empty())
	{
		workingDir = Parameters::createDefaultWorkingDirectory();
	}
	std::string path = workingDir + "/" + folder;
	if(!UDirectory::exists(path))
	{
		UDirectory::makeDir(path);
	}
	return path;
}

#if ORT_API_VERSION >= 22
// Execution provider plugins: libraries loaded at runtime by the official ONNX
// Runtime releases (plugin API of ONNX Runtime >= 1.22), e.g. OpenVINO's
// onnxruntime-ep-openvino. Loading them is the same for every plugin; how
// their devices are selected and their options set is specific to each one.

const char * hardwareTypeName(OrtHardwareDeviceType type)
{
	switch(type)
	{
	case OrtHardwareDeviceType_CPU: return "CPU";
	case OrtHardwareDeviceType_GPU: return "GPU";
	case OrtHardwareDeviceType_NPU: return "NPU";
	default: return "?";
	}
}

// Registers the plugin library @p path under @p registrationName, once per
// process: ONNX Runtime holds one library per registration name, so the first
// one loaded is kept. A library that fails to load is not tried again; @p error
// gets the reason.
bool registerPluginLibrary(const std::string & registrationName, const std::string & path, std::string * error = 0)
{
	static std::mutex mutex;
	static std::map<std::string, std::string> registered; // registration name -> path
	static std::map<std::string, std::string> failures;   // path -> error
	std::lock_guard<std::mutex> lock(mutex);
	std::map<std::string, std::string>::iterator iter = registered.find(registrationName);
	if(iter != registered.end())
	{
		if(path.compare(iter->second) != 0)
		{
			UDEBUG("Plugin \"%s\" already loaded from \"%s\", \"%s\" is ignored.", registrationName.c_str(), iter->second.c_str(), path.c_str());
		}
		return true;
	}
	iter = failures.find(path);
	if(iter != failures.end())
	{
		if(error) *error = iter->second;
		return false;
	}
	try
	{
		// ASCII paths only on Windows, where ORTCHAR_T is wchar_t.
		ortEnv().RegisterExecutionProviderLibrary(registrationName.c_str(), std::basic_string<ORTCHAR_T>(path.begin(), path.end()));
		registered.insert(std::make_pair(registrationName, path));
		std::list<std::string> devices;
		std::vector<Ort::ConstEpDevice> epDevices = ortEnv().GetEpDevices();
		for(size_t i=0; i<epDevices.size(); ++i)
		{
			std::string epName = epDevices[i].EpName();
			if(epName.compare(0, registrationName.size(), registrationName) == 0)
			{
				devices.push_back(uFormat("%s on %s (%s)", epName.c_str(),
						hardwareTypeName(epDevices[i].Device().Type()), epDevices[i].EpVendor()));
			}
		}
		UINFO("Loaded execution provider plugin \"%s\": %s.", path.c_str(), devices.empty()?"no devices":uJoin(devices, ", ").c_str());
		return true;
	}
	catch(const Ort::Exception & e)
	{
		failures.insert(std::make_pair(path, std::string(e.what())));
		UDEBUG("Cannot load execution provider plugin \"%s\": %s", path.c_str(), e.what());
		if(error) *error = e.what();
		return false;
	}
}

// The devices of execution provider @p epName, with the value of their
// metadata @p metadataKey (devices without it are skipped).
std::vector<std::pair<std::string, Ort::ConstEpDevice> > pluginDevices(const std::string & epName, const char * metadataKey)
{
	std::vector<std::pair<std::string, Ort::ConstEpDevice> > devices;
	std::vector<Ort::ConstEpDevice> epDevices = ortEnv().GetEpDevices();
	for(size_t i=0; i<epDevices.size(); ++i)
	{
		const char * value = epDevices[i].EpMetadata().GetValue(metadataKey);
		if(value && epName.compare(epDevices[i].EpName()) == 0)
		{
			devices.push_back(std::make_pair(std::string(value), epDevices[i]));
		}
	}
	return devices;
}
#endif

// OpenVINO comes either built in ONNX Runtime (custom or Intel builds, the
// deprecated way) or as the onnxruntime-ep-openvino plugin.

// Whether OpenVINO is built in this ONNX Runtime library. Checked once: a
// plugin registered afterwards is not listed by GetAvailableProviders().
bool hasBuiltInOpenVINO()
{
	static const bool builtIn = []()
	{
		try
		{
			std::vector<std::string> providers = Ort::GetAvailableProviders();
			return std::find(providers.begin(), providers.end(), "OpenVINOExecutionProvider") != providers.end();
		}
		catch(const Ort::Exception & e)
		{
			UERROR("%s", e.what());
			return false;
		}
	}();
	return builtIn;
}

#if ORT_API_VERSION >= 22
// Registration name of the plugin and name of its devices, and of the devices
// of its AUTO mode.
const char * kOpenVINOPluginEp = "OpenVINOExecutionProvider";
const char * kOpenVINOPluginAutoEp = "OpenVINOExecutionProvider.AUTO";

std::string defaultOpenVINOPluginLibrary()
{
#ifdef _WIN32
	return "onnxruntime_providers_openvino_plugin.dll";
#else
	return "libonnxruntime_providers_openvino_plugin.so";
#endif
}

bool registerOpenVINOPlugin(const std::string & libraryPath, std::string * error = 0)
{
	return registerPluginLibrary(kOpenVINOPluginEp, libraryPath.empty()?defaultOpenVINOPluginLibrary():libraryPath, error);
}

// "GPU" is the first GPU, which OpenVINO also names "GPU.0".
bool sameOpenVINODevice(const std::string & a, const std::string & b)
{
	return a.compare(b) == 0 ||
		(a.compare("GPU") == 0 && b.compare("GPU.0") == 0) ||
		(a.compare("GPU.0") == 0 && b.compare("GPU") == 0);
}

// The plugin's devices matching Onnx/OpenVINODevice: "CPU", "GPU", "GPU.1",
// "NPU"... (their "ov_device" metadata), or "AUTO" (all the devices of the AUTO
// mode) / "AUTO:GPU,CPU" (these ones, in this order of priority). @p available
// lists the devices that could be selected.
std::vector<Ort::ConstEpDevice> selectOpenVINODevices(const std::string & device, std::string & available)
{
	bool autoMode = device.compare(0, 4, "AUTO") == 0;
	std::list<std::string> wanted;
	if(!autoMode)
	{
		wanted.push_back(device);
	}
	else if(device.size() > 5 && device[4] == ':')
	{
		wanted = uSplit(device.substr(5), ',');
	}
	std::vector<std::pair<std::string, Ort::ConstEpDevice> > candidates =
			pluginDevices(autoMode?kOpenVINOPluginAutoEp:kOpenVINOPluginEp, "ov_device");
	std::list<std::string> names;
	for(size_t i=0; i<candidates.size(); ++i)
	{
		names.push_back(candidates[i].first);
	}
	available = uJoin(names, ", ");

	std::vector<Ort::ConstEpDevice> selected;
	if(autoMode && wanted.empty())
	{
		for(size_t i=0; i<candidates.size(); ++i)
		{
			selected.push_back(candidates[i].second);
		}
	}
	for(std::list<std::string>::iterator iter=wanted.begin(); iter!=wanted.end(); ++iter)
	{
		for(size_t i=0; i<candidates.size(); ++i)
		{
			if(sameOpenVINODevice(uToUpperCase(*iter), candidates[i].first))
			{
				selected.push_back(candidates[i].second);
				break;
			}
		}
	}
	return selected;
}

std::string jsonString(const std::string & value)
{
	std::string out = "\"";
	for(size_t i=0; i<value.size(); ++i)
	{
		if(value[i] == '"' || value[i] == '\\')
		{
			out += '\\';
		}
		out += value[i];
	}
	return out + "\"";
}

// The plugin ignores the precision and cache_dir options of the built-in
// provider: they are given as OpenVINO properties of the device in the
// load_config option instead, e.g. {"GPU":{"INFERENCE_PRECISION_HINT":"f32",
// "CACHE_DIR":"/path"}}. Empty if there is nothing to set.
std::string openVINOLoadConfig(const std::string & device, OnnxSession::OpenVINOPrecision precision, const std::string & cachePath)
{
	std::vector<std::string> properties;
	if(precision == OnnxSession::kOpenVINOPrecisionFP32)
	{
		properties.push_back(jsonString("INFERENCE_PRECISION_HINT") + ":" + jsonString("f32"));
	}
	else if(precision == OnnxSession::kOpenVINOPrecisionFP16)
	{
		properties.push_back(jsonString("INFERENCE_PRECISION_HINT") + ":" + jsonString("f16"));
	}
	else if(precision == OnnxSession::kOpenVINOPrecisionAccuracy)
	{
		properties.push_back(jsonString("EXECUTION_MODE_HINT") + ":" + jsonString("ACCURACY"));
	}
	if(!cachePath.empty())
	{
		properties.push_back(jsonString("CACHE_DIR") + ":" + jsonString(cachePath));
	}
	if(properties.empty())
	{
		return "";
	}
	// The properties are set on the device named by Onnx/OpenVINODevice ("AUTO"
	// for "AUTO:GPU,CPU").
	std::string key = device.substr(0, device.find(':'));
	std::string config = "{" + jsonString(uToUpperCase(key)) + ":{";
	for(size_t i=0; i<properties.size(); ++i)
	{
		config += (i?",":"") + properties[i];
	}
	return config + "}}";
}
#endif

// Logs a warning when the scope lasts more than delay seconds, while it still
// runs: a TensorRT engine build blocks for seconds to minutes, the warning
// explains the freeze when it happens (not when the engine was in the cache).
class SlowScopeWarning
{
public:
	SlowScopeWarning(double delay, const std::string & message) :
		done_(false),
		thread_(&SlowScopeWarning::wait, this, delay, message)
	{
	}
	~SlowScopeWarning()
	{
		{
			std::lock_guard<std::mutex> lock(mutex_);
			done_ = true;
		}
		condition_.notify_one();
		thread_.join();
	}

private:
	void wait(double delay, std::string message)
	{
		std::unique_lock<std::mutex> lock(mutex_);
		if(!condition_.wait_for(lock, std::chrono::duration<double>(delay), [this] {return done_;}))
		{
			UWARN("%s", message.c_str());
		}
	}

	bool done_;
	std::mutex mutex_;
	std::condition_variable condition_;
	std::thread thread_; // last, started once the others are initialized
};

std::string engineCacheNote(const std::string & cachePath)
{
	return cachePath.empty()?
			"No cache directory is set, it will be built again at the next start.":
			uFormat("It will be cached in \"%s\" for the next runs.", cachePath.c_str());
}

} // namespace

std::string OnnxSession::Options::key() const
{
	return uFormat("provider=%d;device=%d;fp16=%d;cache=%s;maxkpts=%d;profile=%s|%s|%s;ovdevice=%s;ovprecision=%d;ovcache=%s;ovlib=%s",
			(int)provider, deviceId, tensorRTFp16?1:0, tensorRTCachePath.c_str(), tensorRTMaxKeypoints,
			tensorRTProfileMinShapes.c_str(), tensorRTProfileOptShapes.c_str(), tensorRTProfileMaxShapes.c_str(),
			openVINODevice.c_str(), (int)openVINOPrecision, openVINOCachePath.c_str(), openVINOLibraryPath.c_str());
}

OnnxSession::Options OnnxSession::Options::defaults()
{
	Options options;
	options.provider = (Provider)Parameters::defaultOnnxProvider();
	options.tensorRTFp16 = Parameters::defaultOnnxTensorRTFp16();
	options.tensorRTCachePath = Parameters::defaultOnnxTensorRTCachePath();
	options.tensorRTMaxKeypoints = Parameters::defaultOnnxTensorRTMaxKeypoints();
	options.openVINODevice = Parameters::defaultOnnxOpenVINODevice();
	options.openVINOPrecision = (OpenVINOPrecision)Parameters::defaultOnnxOpenVINOPrecision();
	options.openVINOCachePath = Parameters::defaultOnnxOpenVINOCachePath();
	options.openVINOLibraryPath = Parameters::defaultOnnxOpenVINOLibraryPath();
	return options;
}

void OnnxSession::Options::parse(const ParametersMap & parameters)
{
	int providerValue = (int)provider;
	Parameters::parse(parameters, Parameters::kOnnxProvider(), providerValue);
	provider = (Provider)providerValue;
	Parameters::parse(parameters, Parameters::kOnnxTensorRTFp16(), tensorRTFp16);
	Parameters::parse(parameters, Parameters::kOnnxTensorRTCachePath(), tensorRTCachePath);
	tensorRTCachePath = expandTilde(tensorRTCachePath);
	if(tensorRTCachePath.empty() && provider == kProviderTensorRT)
	{
		tensorRTCachePath = defaultTensorRTCachePath(parameters);
	}
	Parameters::parse(parameters, Parameters::kOnnxTensorRTMaxKeypoints(), tensorRTMaxKeypoints);
	Parameters::parse(parameters, Parameters::kOnnxOpenVINODevice(), openVINODevice);
	// ONNX Runtime's built-in OpenVINO provider only accepts upper case devices
	openVINODevice = uToUpperCase(openVINODevice);
	int precisionValue = (int)openVINOPrecision;
	Parameters::parse(parameters, Parameters::kOnnxOpenVINOPrecision(), precisionValue);
	if(precisionValue < kOpenVINOPrecisionDefault || precisionValue > kOpenVINOPrecisionAccuracy)
	{
		UWARN("Unknown %s=%d (0=device default, 1=FP32, 2=FP16, 3=ACCURACY), the device's default is used.",
				Parameters::kOnnxOpenVINOPrecision().c_str(), precisionValue);
		precisionValue = kOpenVINOPrecisionDefault;
	}
	openVINOPrecision = (OpenVINOPrecision)precisionValue;
	Parameters::parse(parameters, Parameters::kOnnxOpenVINOCachePath(), openVINOCachePath);
	openVINOCachePath = expandTilde(openVINOCachePath);
	if(openVINOCachePath.empty() && provider == kProviderOpenVINO)
	{
		openVINOCachePath = defaultOpenVINOCachePath(parameters);
	}
	Parameters::parse(parameters, Parameters::kOnnxOpenVINOLibraryPath(), openVINOLibraryPath);
	openVINOLibraryPath = expandTilde(openVINOLibraryPath);
}

bool OnnxSession::isProviderAvailable(Provider provider, const std::string & openVINOLibraryPath)
{
	if(provider == kProviderCPU)
	{
		return true;
	}
	if(provider == kProviderOpenVINO)
	{
		if(hasBuiltInOpenVINO())
		{
			return true;
		}
#if ORT_API_VERSION >= 22
		if(registerOpenVINOPlugin(openVINOLibraryPath))
		{
			// Available if the plugin found at least one device.
			return !pluginDevices(kOpenVINOPluginEp, "ov_device").empty();
		}
#endif
		return false;
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
	case kProviderOpenVINO: return "OpenVINO";
	default: return "CPU";
	}
}

std::string OnnxSession::openVINOPrecisionName(OpenVINOPrecision precision)
{
	switch(precision)
	{
	case kOpenVINOPrecisionFP32: return "FP32";
	case kOpenVINOPrecisionFP16: return "FP16";
	case kOpenVINOPrecisionAccuracy: return "ACCURACY";
	default: return "";
	}
}

std::string OnnxSession::runtimeVersion()
{
	return Ort::GetVersionString();
}

std::string OnnxSession::defaultTensorRTCachePath(const ParametersMap & parameters)
{
	return workingDirectoryCachePath(parameters, "tensorrt_cache");
}

std::string OnnxSession::defaultOpenVINOCachePath(const ParametersMap & parameters)
{
	return workingDirectoryCachePath(parameters, "openvino_cache");
}

std::string OnnxSession::expandTilde(const std::string & path)
{
	return UDirectory::expandHomeDir(path);
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

bool OnnxSession::readModelInterface(const std::string & modelPath, std::vector<OnnxTensor> & inputs, std::vector<OnnxTensor> & outputs)
{
	inputs.clear();
	outputs.clear();
	if(!UFile::exists(modelPath))
	{
		UERROR("The ONNX model \"%s\" doesn't exist.", modelPath.c_str());
		return false;
	}
	try
	{
		Ort::SessionOptions sessionOptions;
		sessionOptions.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_DISABLE_ALL);
		Ort::Session session(ortEnv(), modelPath.c_str(), sessionOptions);
		std::vector<std::string> names;
		inputs = describe(session, true, names);
		outputs = describe(session, false, names);
		return true;
	}
	catch(const Ort::Exception & e)
	{
		UERROR("Failed to read ONNX model \"%s\": %s", modelPath.c_str(), e.what());
	}
	return false;
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
	for(int attempt = 0; attempt < 3; ++attempt)
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
					if(!options_.tensorRTProfileMaxShapes.empty())
					{
						// One engine for the whole range, built once, instead of
						// one built again each time an input exceeds the ones seen.
						keyValues.push_back(std::make_pair("trt_profile_min_shapes", options_.tensorRTProfileMinShapes));
						keyValues.push_back(std::make_pair("trt_profile_opt_shapes", options_.tensorRTProfileOptShapes));
						keyValues.push_back(std::make_pair("trt_profile_max_shapes", options_.tensorRTProfileMaxShapes));
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
			if(requested == kProviderOpenVINO)
			{
#if ORT_API_VERSION >= 22
				std::string pluginError;
#endif
				// In both cases, the nodes OpenVINO doesn't support run on the CPU provider.
				if(hasBuiltInOpenVINO())
				{
					std::unordered_map<std::string, std::string> openVINOOptions;
					openVINOOptions["device_type"] = options_.openVINODevice;
					if(options_.openVINOPrecision != kOpenVINOPrecisionDefault)
					{
						openVINOOptions["precision"] = openVINOPrecisionName(options_.openVINOPrecision);
					}
					if(!options_.openVINOCachePath.empty())
					{
						openVINOOptions["cache_dir"] = options_.openVINOCachePath;
					}
					sessionOptions.AppendExecutionProvider_OpenVINO_V2(openVINOOptions);
					provider_ = kProviderOpenVINO;
				}
#if ORT_API_VERSION >= 22
				else if(registerOpenVINOPlugin(options_.openVINOLibraryPath, &pluginError))
				{
					std::string available;
					std::vector<Ort::ConstEpDevice> devices = selectOpenVINODevices(options_.openVINODevice, available);
					if(devices.empty())
					{
						UWARN("OpenVINO device \"%s\" (%s) not found, available: %s. Falling back to CPU.",
								options_.openVINODevice.c_str(), Parameters::kOnnxOpenVINODevice().c_str(),
								available.empty()?"none":available.c_str());
					}
					else
					{
						std::unordered_map<std::string, std::string> openVINOOptions;
						std::string config = openVINOLoadConfig(options_.openVINODevice, options_.openVINOPrecision, options_.openVINOCachePath);
						if(!config.empty())
						{
							openVINOOptions["load_config"] = config;
						}
						sessionOptions.AppendExecutionProvider_V2(env, devices, openVINOOptions);
						provider_ = kProviderOpenVINO;
					}
				}
				else
				{
					UWARN("OpenVINO execution provider is not available: it is not built in this ONNX Runtime library "
							"and its plugin \"%s\" cannot be loaded (%s; set %s to the path of "
							"libonnxruntime_providers_openvino_plugin.so of onnxruntime-ep-openvino), falling back to CPU.",
							options_.openVINOLibraryPath.empty()?defaultOpenVINOPluginLibrary().c_str():options_.openVINOLibraryPath.c_str(),
							pluginError.c_str(), Parameters::kOnnxOpenVINOLibraryPath().c_str());
				}
#else
				else
				{
					UWARN("OpenVINO execution provider is not available in this ONNX Runtime library (plugin "
							"execution providers need ONNX Runtime >= 1.22), falling back to CPU.");
				}
#endif
			}

			{
				// With profile shapes or static inputs, the engine is built here.
				std::unique_ptr<SlowScopeWarning> slowWarning;
				if(provider_ == kProviderTensorRT)
				{
					slowWarning.reset(new SlowScopeWarning(2.0, uFormat(
							"TensorRT is building the engine of ONNX model \"%s\", this can take seconds to minutes... %s",
							modelPath_.c_str(), engineCacheNote(options_.tensorRTCachePath).c_str())));
				}
				impl_->session.reset(new Ort::Session(env, modelPath_.c_str(), sessionOptions));
			}
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
			else if(provider_ == kProviderOpenVINO)
			{
				UINFO("ONNX model \"%s\" runs with OpenVINO on device \"%s\" (precision: %s, cache: \"%s\").",
						modelPath_.c_str(), options_.openVINODevice.c_str(),
						options_.openVINOPrecision == kOpenVINOPrecisionDefault?"device default":openVINOPrecisionName(options_.openVINOPrecision).c_str(),
						options_.openVINOCachePath.c_str());
			}
			break;
		}
		catch(const Ort::Exception & e)
		{
			impl_->session.reset();
			inputs_.clear();
			outputs_.clear();
			if(provider_ == kProviderTensorRT && !options_.tensorRTProfileMaxShapes.empty())
			{
				// e.g. the model is split between TensorRT and CUDA: the inputs of
				// the TensorRT part are not the model's, they would need profiles too.
				UWARN("Failed to load ONNX model \"%s\" with TensorRT and the profile shapes \"%s\" (%s), "
						"trying without them (the engine is then built again each time the inputs are larger or smaller than before).",
						modelPath_.c_str(), options_.tensorRTProfileMaxShapes.c_str(), e.what());
				options_.tensorRTProfileMinShapes.clear();
				options_.tensorRTProfileOptShapes.clear();
				options_.tensorRTProfileMaxShapes.clear();
				continue;
			}
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

		// An engine may be built (seconds to minutes) when the inputs are outside
		// the shapes run so far, unless the engine was built at load for the
		// range of the profile shapes.
		std::unique_ptr<SlowScopeWarning> slowWarning;
		if(provider_ == kProviderTensorRT && options_.tensorRTProfileMaxShapes.empty() && impl_->extendShapes(ordered))
		{
			std::string shapes;
			for(size_t i=0; i<ordered.size(); ++i)
			{
				shapes += (i?", ":"") + impl_->inputNames[i] + " [";
				for(size_t j=0; j<ordered[i].shape.size(); ++j)
				{
					shapes += (j?",":"") + uNumber2Str((int)ordered[i].shape[j]);
				}
				shapes += "]";
			}
			slowWarning.reset(new SlowScopeWarning(2.0, uFormat(
					"TensorRT is building an engine for ONNX model \"%s\" (first inference, or new input shapes: %s), "
					"this can take seconds to minutes... %s",
					modelPath_.c_str(), shapes.c_str(), engineCacheNote(options_.tensorRTCachePath).c_str())));
		}
		UTimer timer;
		std::vector<Ort::Value> results = impl_->session->Run(
				Ort::RunOptions{nullptr},
				inputNames.data(), values.data(), values.size(),
				outputNames.data(), outputNames.size());
		slowWarning.reset();
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
