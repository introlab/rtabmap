#include "AlikedOnnxDetector.h"
#include "OnnxSession.h"

#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UTimer.h>

#include <opencv2/core.hpp>

#include <onnxruntime_cxx_api.h>

namespace rtabmap
{

AlikedOnnxDetector::AlikedOnnxDetector(
		const std::string & modelPath,
		float threshold,
		int provider,
		bool tensorRTFp16,
		const std::string & tensorRTCachePath) :
	modelPath_(modelPath),
	threshold_(threshold),
	provider_(provider),
	tensorRTFp16_(tensorRTFp16),
	tensorRTCachePath_(tensorRTCachePath),
	candidates_(0),
	descriptorDim_(0)
{
	UDEBUG("modelPath=%s threshold=%f provider=%d fp16=%d cache=%s",
			modelPath_.c_str(), threshold_, provider_, tensorRTFp16_?1:0, tensorRTCachePath_.c_str());

	std::vector<std::string> inputNames, outputNames;
	std::vector<std::vector<int64_t> > inputShapes, outputShapes;
	if(!OnnxSession::readShapes(modelPath_, inputNames, inputShapes, outputNames, outputShapes))
	{
		UERROR("ALIKED ONNX model \"%s\" not found or invalid! See corelib/src/python/rtabmap_export_aliked_onnx.py to create one.", modelPath_.c_str());
		return;
	}
	// image [1,1,H,W] -> keypoints [1,K,2], scores [1,K], descriptors [1,K,D], all static
	if(inputNames.size() != 1 || inputNames[0] != "image" || inputShapes[0].size() != 4 ||
	   inputShapes[0][2] <= 0 || inputShapes[0][3] <= 0 ||
	   outputNames.size() != 3 || outputShapes[2].size() != 3 || outputShapes[2][1] <= 0 || outputShapes[2][2] <= 0)
	{
		UERROR("\"%s\" doesn't look like an ALIKED model exported by rtabmap_export_aliked_onnx.py "
				"(expected input \"image\" [1,1,H,W] and outputs keypoints, scores, descriptors with static shapes).", modelPath_.c_str());
		return;
	}
	inputSize_ = cv::Size((int)inputShapes[0][3], (int)inputShapes[0][2]);
	candidates_ = (int)outputShapes[2][1];
	descriptorDim_ = (int)outputShapes[2][2];

	if(tensorRTFp16_ && provider_ == OnnxSession::kTensorRT)
	{
		UWARN("ALIKED with TensorRT FP16 detects different keypoints than FP32 (the score map loses "
			  "precision for the non-maximum suppression); consider ALIKED/TensorRTFp16=false.");
	}
	session_ = OnnxSession::get("ALIKED", modelPath_, provider_, tensorRTFp16_, tensorRTCachePath_);
	if(!session_->isValid())
	{
		session_.reset();
	}
	else
	{
		UINFO("ALIKED ONNX: input %dx%d, %d candidates, %d-D descriptors, provider=%s",
				inputSize_.width, inputSize_.height, candidates_, descriptorDim_, session_->providerName().c_str());
	}
}

AlikedOnnxDetector::~AlikedOnnxDetector()
{
}

void AlikedOnnxDetector::detect(const cv::Mat & image, const cv::Mat & mask,
		std::vector<cv::KeyPoint> & keypoints, cv::Mat & descriptors)
{
	keypoints.clear();
	descriptors = cv::Mat();
	if(!isValid())
	{
		return;
	}
	UASSERT(!image.empty() && image.type() == CV_8UC1);
	UASSERT(mask.empty() || (mask.type() == CV_8UC1 && mask.size() == image.size()));
	if(image.rows != inputSize_.height || image.cols % inputSize_.width != 0)
	{
		UERROR("ALIKED model \"%s\" expects %dx%d images (or several side by side), received %dx%d. "
				"Export a model for this resolution with rtabmap_export_aliked_onnx.py.",
				modelPath_.c_str(), inputSize_.width, inputSize_.height, image.cols, image.rows);
		return;
	}

	UTimer timer;
	const int tiles = image.cols / inputSize_.width;
	std::vector<float> input(inputSize_.area());
	std::vector<cv::Mat> descriptorRows;
	try
	{
		Ort::MemoryInfo memory = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
		const std::vector<int64_t> inputShape = {1, 1, inputSize_.height, inputSize_.width};
		const char * inputNames[] = {"image"};
		const char * outputNames[] = {"keypoints", "scores", "descriptors"};
		for(int t=0; t<tiles; ++t)
		{
			const cv::Mat tile = image.colRange(t*inputSize_.width, (t+1)*inputSize_.width);
			for(int y=0; y<tile.rows; ++y)
			{
				const unsigned char * row = tile.ptr<unsigned char>(y);
				for(int x=0; x<tile.cols; ++x)
				{
					input[y*tile.cols + x] = row[x] / 255.0f;
				}
			}
			Ort::Value tensor = Ort::Value::CreateTensor<float>(memory, input.data(), input.size(), inputShape.data(), inputShape.size());
			std::vector<Ort::Value> outputs = session_->session().Run(Ort::RunOptions{nullptr}, inputNames, &tensor, 1, outputNames, 3);
			const float * kpts = outputs[0].GetTensorData<float>();
			const float * scores = outputs[1].GetTensorData<float>();
			const float * desc = outputs[2].GetTensorData<float>();
			for(int i=0; i<candidates_; ++i)
			{
				if(scores[i] <= threshold_)
				{
					continue;
				}
				const float x = kpts[i*2] + t*inputSize_.width;
				const float y = kpts[i*2+1];
				if(!mask.empty())
				{
					const int u = std::min(std::max(int(x+0.5f), 0), mask.cols-1);
					const int v = std::min(std::max(int(y+0.5f), 0), mask.rows-1);
					if(mask.at<unsigned char>(v, u) == 0)
					{
						continue;
					}
				}
				keypoints.push_back(cv::KeyPoint(x, y, 8.0f, -1.0f, scores[i]));
				descriptorRows.push_back(cv::Mat(1, descriptorDim_, CV_32FC1, (void*)(desc + i*descriptorDim_)).clone());
			}
		}
	}
	catch(const Ort::Exception & e)
	{
		UERROR("ALIKED ONNX inference failed: %s", e.what());
		keypoints.clear();
		return;
	}
	if(!descriptorRows.empty())
	{
		cv::vconcat(descriptorRows, descriptors);
	}
	UDEBUG("ALIKED: %d keypoints (%d image(s)) in %fs", (int)keypoints.size(), tiles, timer.ticks());
}

}
