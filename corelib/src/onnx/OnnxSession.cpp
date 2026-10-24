#include "OnnxSession.h"

#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/utilite/UFile.h>
#include <rtabmap/utilite/UDirectory.h>

#include <onnxruntime_cxx_api.h>

#include <algorithm>
#include <map>
#include <mutex>

namespace rtabmap
{

namespace {

Ort::Env & ortEnv()
{
	// ONNX Runtime keeps a single environment per process
	static Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "rtabmap");
	return env;
}

bool providerAvailable(const std::string & name)
{
	std::vector<std::string> providers = Ort::GetAvailableProviders();
	return std::find(providers.begin(), providers.end(), name) != providers.end();
}

void appendCUDA(Ort::SessionOptions & options)
{
	const OrtApi & api = Ort::GetApi();
	OrtCUDAProviderOptionsV2 * cudaOptions = nullptr;
	Ort::ThrowOnError(api.CreateCUDAProviderOptions(&cudaOptions));
	options.AppendExecutionProvider_CUDA_V2(*cudaOptions);
	api.ReleaseCUDAProviderOptions(cudaOptions);
}

void appendTensorRT(Ort::SessionOptions & options, bool fp16, const std::string & cachePath, const std::vector<std::string> & shapes)
{
	std::vector<const char*> keys = {"trt_fp16_enable"};
	std::vector<const char*> values = {fp16?"1":"0"};
	if(shapes.size() == 3)
	{
		// Explicit optimization profile: with dynamic inputs, TensorRT would
		// otherwise rebuild its engine every time a larger input comes in.
		keys.push_back("trt_profile_min_shapes"); values.push_back(shapes[0].c_str());
		keys.push_back("trt_profile_opt_shapes"); values.push_back(shapes[1].c_str());
		keys.push_back("trt_profile_max_shapes"); values.push_back(shapes[2].c_str());
	}
	if(!cachePath.empty())
	{
		keys.push_back("trt_engine_cache_enable"); values.push_back("1");
		keys.push_back("trt_engine_cache_path"); values.push_back(cachePath.c_str());
		keys.push_back("trt_timing_cache_enable"); values.push_back("1");
		keys.push_back("trt_timing_cache_path"); values.push_back(cachePath.c_str());
	}
	const OrtApi & api = Ort::GetApi();
	OrtTensorRTProviderOptionsV2 * trtOptions = nullptr;
	Ort::ThrowOnError(api.CreateTensorRTProviderOptions(&trtOptions));
	OrtStatus * status = api.UpdateTensorRTProviderOptions(trtOptions, keys.data(), values.data(), keys.size());
	if(status)
	{
		api.ReleaseTensorRTProviderOptions(trtOptions);
		Ort::ThrowOnError(status);
	}
	options.AppendExecutionProvider_TensorRT_V2(*trtOptions);
	api.ReleaseTensorRTProviderOptions(trtOptions);
}

} // namespace

std::shared_ptr<OnnxSession> OnnxSession::get(
		const std::string & name,
		const std::string & modelPath,
		int provider,
		bool tensorRTFp16,
		const std::string & tensorRTCachePath,
		const std::vector<std::string> & tensorRTShapes)
{
	static std::mutex mutex;
	static std::map<std::string, std::weak_ptr<OnnxSession> > sessions;
	std::string key = uFormat("%s|%d|%d|%s", uReplaceChar(modelPath, '~', UDirectory::homeDir()).c_str(), provider, tensorRTFp16?1:0, tensorRTCachePath.c_str());
	for(size_t i=0; i<tensorRTShapes.size(); ++i)
	{
		key += "|" + tensorRTShapes[i];
	}
	std::lock_guard<std::mutex> lock(mutex);
	std::shared_ptr<OnnxSession> session = sessions[key].lock();
	if(session.get() == 0)
	{
		session.reset(new OnnxSession(name, modelPath, provider, tensorRTFp16, tensorRTCachePath, tensorRTShapes));
		sessions[key] = session;
	}
	else
	{
		UDEBUG("Sharing the %s ONNX session of \"%s\"", name.c_str(), session->path().c_str());
	}
	return session;
}

OnnxSession::OnnxSession(
		const std::string & name,
		const std::string & modelPath,
		int provider,
		bool tensorRTFp16,
		const std::string & tensorRTCachePath,
		const std::vector<std::string> & tensorRTShapes) :
	path_(uReplaceChar(modelPath, '~', UDirectory::homeDir())),
	activeProvider_(provider),
	tensorRTFp16_(tensorRTFp16)
{
	const std::string cachePath = uReplaceChar(tensorRTCachePath, '~', UDirectory::homeDir());
	if(!UFile::exists(path_))
	{
		UERROR("%s ONNX model \"%s\" not found!", name.c_str(), path_.c_str());
		return;
	}
	try
	{
		Ort::SessionOptions options;
		options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
		if(activeProvider_ == kTensorRT && !providerAvailable("TensorrtExecutionProvider"))
		{
			UWARN("%s: TensorRT is not available in this ONNX Runtime build, using CUDA instead.", name.c_str());
			activeProvider_ = kCUDA;
		}
		if(activeProvider_ == kCUDA && !providerAvailable("CUDAExecutionProvider"))
		{
			UWARN("%s: CUDA is not available in this ONNX Runtime build, using CPU instead.", name.c_str());
			activeProvider_ = kCPU;
		}
		if(activeProvider_ == kTensorRT)
		{
			if(!cachePath.empty() && !UDirectory::exists(cachePath))
			{
				UDirectory::makeDir(cachePath);
			}
			appendTensorRT(options, tensorRTFp16_, cachePath, tensorRTShapes);
			appendCUDA(options); // for the nodes TensorRT doesn't support
		}
		else if(activeProvider_ == kCUDA)
		{
			appendCUDA(options);
		}
		UTimer timer;
		session_.reset(new Ort::Session(ortEnv(), path_.c_str(), options));
		UINFO("%s ONNX model \"%s\" loaded (provider=%s) in %fs",
				name.c_str(), path_.c_str(), providerName().c_str(), timer.ticks());
	}
	catch(const Ort::Exception & e)
	{
		UERROR("Failed to load %s ONNX model \"%s\": %s", name.c_str(), path_.c_str(), e.what());
		session_.reset();
	}
}

OnnxSession::~OnnxSession()
{
	session_.reset();
}

std::string OnnxSession::providerName() const
{
	return activeProvider_==kTensorRT?(tensorRTFp16_?"TensorRT FP16":"TensorRT"):activeProvider_==kCUDA?"CUDA":"CPU";
}

bool OnnxSession::readShapes(
		const std::string & modelPath,
		std::vector<std::string> & inputNames,
		std::vector<std::vector<int64_t> > & inputShapes,
		std::vector<std::string> & outputNames,
		std::vector<std::vector<int64_t> > & outputShapes)
{
	const std::string path = uReplaceChar(modelPath, '~', UDirectory::homeDir());
	if(!UFile::exists(path))
	{
		return false;
	}
	try
	{
		Ort::Session probe(ortEnv(), path.c_str(), Ort::SessionOptions());
		Ort::AllocatorWithDefaultOptions allocator;
		inputNames.clear(); inputShapes.clear(); outputNames.clear(); outputShapes.clear();
		for(size_t i=0; i<probe.GetInputCount(); ++i)
		{
			inputNames.push_back(probe.GetInputNameAllocated(i, allocator).get());
			inputShapes.push_back(probe.GetInputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape());
		}
		for(size_t i=0; i<probe.GetOutputCount(); ++i)
		{
			outputNames.push_back(probe.GetOutputNameAllocated(i, allocator).get());
			outputShapes.push_back(probe.GetOutputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape());
		}
		return true;
	}
	catch(const Ort::Exception & e)
	{
		UERROR("Failed to read ONNX model \"%s\": %s", path.c_str(), e.what());
	}
	return false;
}

}
