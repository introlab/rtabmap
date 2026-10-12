#include "LightGlueOnnx.h"

#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/utilite/UFile.h>
#include <rtabmap/utilite/UDirectory.h>

#include <onnxruntime_cxx_api.h>

#include <algorithm>
#include <cmath>

namespace rtabmap
{

namespace {

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

void appendTensorRT(Ort::SessionOptions & options, bool fp16, const std::string & cachePath, int maxKeypoints, int dim)
{
	// Explicit optimization profile: with dynamic numbers of keypoints, TensorRT
	// would otherwise rebuild its engine every time a larger input comes in.
	auto shapes = [dim](int n) {
		return uFormat("kpts0:1x%dx2,kpts1:1x%dx2,desc0:1x%dx%d,desc1:1x%dx%d", n, n, n, dim, n, dim);
	};
	std::string minShapes = shapes(2);
	std::string optShapes = shapes(std::min(maxKeypoints, 1024));
	std::string maxShapes = shapes(maxKeypoints);
	std::vector<const char*> keys = {"trt_fp16_enable", "trt_profile_min_shapes", "trt_profile_opt_shapes", "trt_profile_max_shapes"};
	std::vector<const char*> values = {fp16?"1":"0", minShapes.c_str(), optShapes.c_str(), maxShapes.c_str()};
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

LightGlueOnnx::LightGlueOnnx(
		const std::string & modelPath,
		float threshold,
		int provider,
		bool tensorRTFp16,
		const std::string & tensorRTCachePath,
		int tensorRTMaxKeypoints) :
	modelPath_(modelPath),
	threshold_(threshold),
	provider_(provider),
	activeProvider_(provider),
	tensorRTFp16_(tensorRTFp16),
	tensorRTCachePath_(tensorRTCachePath),
	tensorRTMaxKeypoints_(std::max(tensorRTMaxKeypoints, 2)),
	descriptorDim_(0)
{
	const std::string path = uReplaceChar(modelPath_, '~', UDirectory::homeDir());
	const std::string cachePath = uReplaceChar(tensorRTCachePath_, '~', UDirectory::homeDir());
	UDEBUG("modelPath=%s threshold=%f provider=%d fp16=%d cache=%s maxKeypoints=%d",
			path.c_str(), threshold_, provider_, tensorRTFp16_?1:0, cachePath.c_str(), tensorRTMaxKeypoints_);
	if(!UFile::exists(path))
	{
		UERROR("LightGlue ONNX model \"%s\" not found! See corelib/src/python/rtabmap_export_lightglue_onnx.py to create one.", path.c_str());
		return;
	}

	try
	{
		env_.reset(new Ort::Env(ORT_LOGGING_LEVEL_WARNING, "rtabmap_lightglue"));
		Ort::SessionOptions options;
		options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

		// The descriptor size is needed for the TensorRT shapes, read it from the model on CPU first
		{
			Ort::Session probe(*env_, path.c_str(), Ort::SessionOptions());
			Ort::AllocatorWithDefaultOptions allocator;
			for(size_t i=0; i<probe.GetInputCount(); ++i)
			{
				if(std::string(probe.GetInputNameAllocated(i, allocator).get()) == "desc0")
				{
					std::vector<int64_t> shape = probe.GetInputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape();
					descriptorDim_ = shape.size() == 3 ? (int)shape[2] : 0;
				}
			}
		}
		if(descriptorDim_ <= 0)
		{
			UERROR("\"%s\" doesn't look like a LightGlue model exported by rtabmap_export_lightglue_onnx.py (input \"desc0\" [1,N,D] not found).", path.c_str());
			env_.reset();
			return;
		}

		if(activeProvider_ == kTensorRT && !providerAvailable("TensorrtExecutionProvider"))
		{
			UWARN("TensorRT is not available in this ONNX Runtime build, using CUDA instead.");
			activeProvider_ = kCUDA;
		}
		if(activeProvider_ == kCUDA && !providerAvailable("CUDAExecutionProvider"))
		{
			UWARN("CUDA is not available in this ONNX Runtime build, using CPU instead.");
			activeProvider_ = kCPU;
		}
		if(activeProvider_ == kTensorRT)
		{
			if(!cachePath.empty() && !UDirectory::exists(cachePath))
			{
				UDirectory::makeDir(cachePath);
			}
			appendTensorRT(options, tensorRTFp16_, cachePath, tensorRTMaxKeypoints_, descriptorDim_);
			appendCUDA(options); // for the nodes TensorRT doesn't support
		}
		else if(activeProvider_ == kCUDA)
		{
			appendCUDA(options);
		}

		UTimer timer;
		session_.reset(new Ort::Session(*env_, path.c_str(), options));
		UINFO("LightGlue ONNX model \"%s\" loaded (%d-D descriptors, provider=%s%s) in %fs",
				path.c_str(), descriptorDim_,
				activeProvider_==kTensorRT?"TensorRT":activeProvider_==kCUDA?"CUDA":"CPU",
				activeProvider_==kTensorRT&&tensorRTFp16_?" FP16":"",
				timer.ticks());
	}
	catch(const Ort::Exception & e)
	{
		UERROR("Failed to load LightGlue ONNX model \"%s\": %s", path.c_str(), e.what());
		session_.reset();
		env_.reset();
	}
}

LightGlueOnnx::~LightGlueOnnx()
{
	session_.reset();
	env_.reset();
}

std::vector<cv::DMatch> LightGlueOnnx::match(
		const cv::Mat & descriptorsQuery,
		const cv::Mat & descriptorsTrain,
		const std::vector<cv::KeyPoint> & keypointsQuery,
		const std::vector<cv::KeyPoint> & keypointsTrain,
		const cv::Size & imageSize)
{
	std::vector<cv::DMatch> matches;
	if(!isValid())
	{
		return matches;
	}
	UASSERT(descriptorsQuery.rows == (int)keypointsQuery.size());
	UASSERT(descriptorsTrain.rows == (int)keypointsTrain.size());
	UASSERT(descriptorsQuery.type() == CV_32FC1 && descriptorsTrain.type() == CV_32FC1);
	UASSERT(imageSize.width > 0 && imageSize.height > 0);
	if(descriptorsQuery.cols != descriptorDim_ || descriptorsTrain.cols != descriptorDim_)
	{
		UERROR("LightGlue model \"%s\" expects %d-D descriptors, received %d-D and %d-D ones.",
				modelPath_.c_str(), descriptorDim_, descriptorsQuery.cols, descriptorsTrain.cols);
		return matches;
	}
	const int m = (int)keypointsQuery.size();
	const int n = (int)keypointsTrain.size();
	if(m < 2 || n < 2)
	{
		return matches;
	}
	if(activeProvider_ == kTensorRT && (m > tensorRTMaxKeypoints_ || n > tensorRTMaxKeypoints_))
	{
		UWARN("%d/%d keypoints are more than the TensorRT profile's maximum (%d), increase LightGlue/TensorRTMaxKeypoints.",
				m, n, tensorRTMaxKeypoints_);
	}

	UTimer timer;
	// LightGlue's normalize_keypoints(): centered, scaled by half the largest image side
	const float shiftX = imageSize.width / 2.0f;
	const float shiftY = imageSize.height / 2.0f;
	const float scale = std::max(imageSize.width, imageSize.height) / 2.0f;
	auto keypoints = [&](const std::vector<cv::KeyPoint> & kpts) {
		std::vector<float> out(kpts.size()*2);
		for(size_t i=0; i<kpts.size(); ++i)
		{
			out[i*2] = (kpts[i].pt.x - shiftX) / scale;
			out[i*2+1] = (kpts[i].pt.y - shiftY) / scale;
		}
		return out;
	};
	// LightGlue expects L2-normalized descriptors
	auto descriptors = [&](const cv::Mat & desc) {
		std::vector<float> out(desc.rows*desc.cols);
		for(int i=0; i<desc.rows; ++i)
		{
			const float * row = desc.ptr<float>(i);
			double norm = 0.0;
			for(int j=0; j<desc.cols; ++j)
			{
				norm += double(row[j]) * row[j];
			}
			norm = std::sqrt(norm);
			float s = norm > 0.0 ? float(1.0/norm) : 0.0f;
			for(int j=0; j<desc.cols; ++j)
			{
				out[i*desc.cols+j] = row[j] * s;
			}
		}
		return out;
	};
	std::vector<float> kpts0 = keypoints(keypointsQuery);
	std::vector<float> kpts1 = keypoints(keypointsTrain);
	std::vector<float> desc0 = descriptors(descriptorsQuery);
	std::vector<float> desc1 = descriptors(descriptorsTrain);
	UDEBUG("Preparing data time = %fs", timer.ticks());

	try
	{
		Ort::MemoryInfo memory = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
		const std::vector<int64_t> kpts0Shape = {1, m, 2};
		const std::vector<int64_t> kpts1Shape = {1, n, 2};
		const std::vector<int64_t> desc0Shape = {1, m, descriptorDim_};
		const std::vector<int64_t> desc1Shape = {1, n, descriptorDim_};
		std::vector<Ort::Value> inputs;
		inputs.push_back(Ort::Value::CreateTensor<float>(memory, kpts0.data(), kpts0.size(), kpts0Shape.data(), kpts0Shape.size()));
		inputs.push_back(Ort::Value::CreateTensor<float>(memory, kpts1.data(), kpts1.size(), kpts1Shape.data(), kpts1Shape.size()));
		inputs.push_back(Ort::Value::CreateTensor<float>(memory, desc0.data(), desc0.size(), desc0Shape.data(), desc0Shape.size()));
		inputs.push_back(Ort::Value::CreateTensor<float>(memory, desc1.data(), desc1.size(), desc1Shape.data(), desc1Shape.size()));
		const char * inputNames[] = {"kpts0", "kpts1", "desc0", "desc1"};
		const char * outputNames[] = {"matches0", "mscores0"};

		std::vector<Ort::Value> outputs = session_->Run(Ort::RunOptions{nullptr}, inputNames, inputs.data(), inputs.size(), outputNames, 2);
		UDEBUG("LightGlue inference time = %fs", timer.ticks());

		const int64_t * matches0 = outputs[0].GetTensorData<int64_t>();
		const float * mscores0 = outputs[1].GetTensorData<float>();
		for(int i=0; i<m; ++i)
		{
			if(matches0[i] >= 0 && matches0[i] < n && mscores0[i] > threshold_)
			{
				matches.push_back(cv::DMatch(i, (int)matches0[i], 1.0f - mscores0[i]));
			}
		}
		UDEBUG("%d matches (%d/%d keypoints), fill matches time = %fs", (int)matches.size(), m, n, timer.ticks());
	}
	catch(const Ort::Exception & e)
	{
		UERROR("LightGlue ONNX inference failed (%d/%d keypoints): %s", m, n, e.what());
		matches.clear();
	}
	return matches;
}

}
