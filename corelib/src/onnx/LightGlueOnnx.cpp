#include "LightGlueOnnx.h"
#include "OnnxSession.h"

#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UTimer.h>

#include <onnxruntime_cxx_api.h>

#include <algorithm>
#include <cmath>

namespace rtabmap
{

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
	tensorRTFp16_(tensorRTFp16),
	tensorRTCachePath_(tensorRTCachePath),
	tensorRTMaxKeypoints_(std::max(tensorRTMaxKeypoints, 2)),
	descriptorDim_(0)
{
	UDEBUG("modelPath=%s threshold=%f provider=%d fp16=%d cache=%s maxKeypoints=%d",
			modelPath_.c_str(), threshold_, provider_, tensorRTFp16_?1:0, tensorRTCachePath_.c_str(), tensorRTMaxKeypoints_);

	// The descriptor size is needed for the TensorRT shapes, read it from the model first
	std::vector<std::string> inputNames, outputNames;
	std::vector<std::vector<int64_t> > inputShapes, outputShapes;
	if(!OnnxSession::readShapes(modelPath_, inputNames, inputShapes, outputNames, outputShapes))
	{
		UERROR("LightGlue ONNX model \"%s\" not found or invalid! See corelib/src/python/rtabmap_export_lightglue_onnx.py to create one.", modelPath_.c_str());
		return;
	}
	for(size_t i=0; i<inputNames.size(); ++i)
	{
		if(inputNames[i] == "desc0" && inputShapes[i].size() == 3)
		{
			descriptorDim_ = (int)inputShapes[i][2];
		}
	}
	if(descriptorDim_ <= 0)
	{
		UERROR("\"%s\" doesn't look like a LightGlue model exported by rtabmap_export_lightglue_onnx.py (input \"desc0\" [1,N,D] not found).", modelPath_.c_str());
		return;
	}

	auto shapes = [this](int n) {
		return uFormat("kpts0:1x%dx2,kpts1:1x%dx2,desc0:1x%dx%d,desc1:1x%dx%d", n, n, n, descriptorDim_, n, descriptorDim_);
	};
	std::vector<std::string> profile;
	profile.push_back(shapes(2));
	profile.push_back(shapes(std::min(tensorRTMaxKeypoints_, 1024)));
	profile.push_back(shapes(tensorRTMaxKeypoints_));
	session_ = OnnxSession::get("LightGlue", modelPath_, provider_, tensorRTFp16_, tensorRTCachePath_, profile);
	if(!session_->isValid())
	{
		session_.reset();
	}
	else
	{
		UINFO("LightGlue ONNX: %d-D descriptors, provider=%s", descriptorDim_, session_->providerName().c_str());
	}
}

LightGlueOnnx::~LightGlueOnnx()
{
}

int LightGlueOnnx::activeProvider() const
{
	return session_.get()?session_->activeProvider():provider_;
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
	if(activeProvider() == OnnxSession::kTensorRT && (m > tensorRTMaxKeypoints_ || n > tensorRTMaxKeypoints_))
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

		std::vector<Ort::Value> outputs = session_->session().Run(Ort::RunOptions{nullptr}, inputNames, inputs.data(), inputs.size(), outputNames, 2);
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
