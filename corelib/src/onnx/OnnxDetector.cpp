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

#include "onnx/OnnxDetector.h"

#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UDirectory.h>
#include <rtabmap/utilite/UTimer.h>

#include <opencv2/imgproc/imgproc.hpp>

namespace rtabmap {

namespace {

std::string expandTilde(const std::string & path)
{
	if(!path.empty() && path[0] == '~' && (path.size() == 1 || path[1] == '/' || path[1] == '\\'))
	{
		return UDirectory::homeDir() + path.substr(1);
	}
	return path;
}

// Shape without a leading batch dimension of 1.
std::vector<int64_t> squeezeBatch(const std::vector<int64_t> & shape)
{
	if(shape.size() > 1 && shape[0] == 1)
	{
		return std::vector<int64_t>(shape.begin()+1, shape.end());
	}
	return shape;
}

bool isKeypointsLike(const OnnxTensor & t)
{
	std::vector<int64_t> shape = squeezeBatch(t.shape);
	return shape.size() == 2 && (shape[1] == 2 || shape[1] == 4) &&
			(t.type == OnnxTensor::kFloat32 || t.type == OnnxTensor::kInt64 || t.type == OnnxTensor::kInt32);
}

bool isDescriptorsLike(const OnnxTensor & t)
{
	std::vector<int64_t> shape = squeezeBatch(t.shape);
	return shape.size() == 2 && shape[1] > 2 && t.type == OnnxTensor::kFloat32;
}

bool isScoresLike(const OnnxTensor & t)
{
	std::vector<int64_t> shape = squeezeBatch(t.shape);
	return shape.size() == 1 && t.type == OnnxTensor::kFloat32;
}

int findOutput(const std::vector<OnnxTensor> & outputs, const std::vector<std::string> & names, bool (*shapeTest)(const OnnxTensor &), const std::vector<int> & taken)
{
	for(size_t n=0; n<names.size(); ++n)
	{
		for(size_t i=0; i<outputs.size(); ++i)
		{
			if(outputs[i].name.compare(names[n]) == 0)
			{
				return (int)i;
			}
		}
	}
	for(size_t i=0; i<outputs.size(); ++i)
	{
		if(std::find(taken.begin(), taken.end(), (int)i) == taken.end() && shapeTest(outputs[i]))
		{
			return (int)i;
		}
	}
	return -1;
}

} // namespace

OnnxDetector::OnnxDetector(const ParametersMap & parameters) :
		modelPath_(Parameters::defaultOnnxDetectorModelPath()),
		threshold_(Parameters::defaultOnnxDetectorThreshold()),
		provider_(Parameters::defaultOnnxProvider()),
		tensorRTFp16_(Parameters::defaultOnnxTensorRTFp16()),
		tensorRTCachePath_(Parameters::defaultOnnxTensorRTCachePath()),
		lastImageData_(0),
		resizeWarned_(false)
{
	parseParameters(parameters);
}

OnnxDetector::~OnnxDetector()
{
}

void OnnxDetector::parseParameters(const ParametersMap & parameters)
{
	Feature2D::parseParameters(parameters);

	std::string previousModelPath = modelPath_;
	int previousProvider = provider_;
	bool previousFp16 = tensorRTFp16_;
	std::string previousCachePath = tensorRTCachePath_;

	Parameters::parse(parameters, Parameters::kOnnxDetectorModelPath(), modelPath_);
	Parameters::parse(parameters, Parameters::kOnnxDetectorThreshold(), threshold_);
	Parameters::parse(parameters, Parameters::kOnnxProvider(), provider_);
	Parameters::parse(parameters, Parameters::kOnnxTensorRTFp16(), tensorRTFp16_);
	Parameters::parse(parameters, Parameters::kOnnxTensorRTCachePath(), tensorRTCachePath_);
	modelPath_ = expandTilde(modelPath_);
	tensorRTCachePath_ = expandTilde(tensorRTCachePath_);
	if(tensorRTCachePath_.empty() && provider_ == (int)OnnxSession::kProviderTensorRT)
	{
		tensorRTCachePath_ = OnnxSession::defaultTensorRTCachePath(parameters);
	}

	if(session_.get() == 0 ||
	   modelPath_.compare(previousModelPath) != 0 ||
	   provider_ != previousProvider ||
	   tensorRTFp16_ != previousFp16 ||
	   tensorRTCachePath_.compare(previousCachePath) != 0)
	{
		session_.reset();
		descriptors_ = cv::Mat();
		lastImageData_ = 0;
		if(modelPath_.empty())
		{
			UERROR("Cannot initialize ONNX detector, %s is not set.", Parameters::kOnnxDetectorModelPath().c_str());
		}
		else
		{
			OnnxSession::Options options;
			options.provider = (OnnxSession::Provider)provider_;
			options.tensorRTFp16 = tensorRTFp16_;
			options.tensorRTCachePath = tensorRTCachePath_;
			session_ = OnnxSession::get(modelPath_, options);
			if(session_.get() == 0)
			{
				UERROR("Cannot initialize ONNX detector with %s=\"%s\".", Parameters::kOnnxDetectorModelPath().c_str(), modelPath_.c_str());
			}
		}
	}
}

bool OnnxDetector::isGpuAvailable() const
{
	return OnnxSession::isProviderAvailable(OnnxSession::kProviderCUDA);
}

bool OnnxDetector::prepareInputs(const cv::Mat & image, std::vector<OnnxTensor> & inputs, cv::Size & modelImageSize) const
{
	UASSERT(session_.get() != 0);
	UASSERT(!image.empty() && image.type() == CV_8UC1);

	if(session_->inputs().size() != 1)
	{
		UERROR("ONNX model \"%s\" has %d inputs, OnnxDetector expects a single image input.", modelPath_.c_str(), (int)session_->inputs().size());
		return false;
	}
	const OnnxTensor & input = session_->inputs()[0];
	if(input.shape.size() != 4)
	{
		UERROR("ONNX model \"%s\": input %s should have 4 dimensions ([1,C,H,W] or [1,H,W,C]).", modelPath_.c_str(), input.toString().c_str());
		return false;
	}
	if(input.type != OnnxTensor::kFloat32 && input.type != OnnxTensor::kUInt8)
	{
		UERROR("ONNX model \"%s\": input %s should be float32 or uint8.", modelPath_.c_str(), input.toString().c_str());
		return false;
	}
	if(input.dim(0) != 1 && input.dim(0) != -1)
	{
		UERROR("ONNX model \"%s\": input %s should have a batch size of 1.", modelPath_.c_str(), input.toString().c_str());
		return false;
	}

	// Channels-last only when the last dimension says so and the second doesn't.
	const int64_t d1 = input.dim(1);
	const int64_t d3 = input.dim(3);
	const bool channelsLast = (d3 == 1 || d3 == 3) && !(d1 == 1 || d1 == 3);
	int64_t channels = channelsLast?d3:d1;
	if(channels == -1)
	{
		channels = 1;
	}
	if(channels != 1 && channels != 3)
	{
		UERROR("ONNX model \"%s\": input %s should have 1 or 3 channels.", modelPath_.c_str(), input.toString().c_str());
		return false;
	}
	const int64_t modelHeight = channelsLast?input.dim(1):input.dim(2);
	const int64_t modelWidth = channelsLast?input.dim(2):input.dim(3);

	// A model with a fixed input size gets the image resized to it, the
	// keypoints are scaled back by the caller with modelImageSize.
	cv::Mat img = image;
	modelImageSize = image.size();
	if(modelHeight > 0 && modelWidth > 0 && (modelHeight != image.rows || modelWidth != image.cols))
	{
		modelImageSize = cv::Size((int)modelWidth, (int)modelHeight);
		if(!resizeWarned_)
		{
			UWARN("ONNX model \"%s\" has a fixed input size of %dx%d, images of %dx%d are resized to it (this warning is shown only once).",
					modelPath_.c_str(), modelImageSize.width, modelImageSize.height, image.cols, image.rows);
			resizeWarned_ = true;
		}
		cv::resize(image, img, modelImageSize, 0, 0, modelImageSize.width < image.cols?cv::INTER_AREA:cv::INTER_LINEAR);
	}

	const int H = img.rows;
	const int W = img.cols;
	const int C = (int)channels;
	OnnxTensor tensor;
	tensor.name = input.name;
	tensor.type = input.type;
	if(channelsLast)
	{
		tensor.shape = {1, H, W, C};
	}
	else
	{
		tensor.shape = {1, C, H, W};
	}
	const size_t count = (size_t)C*H*W;
	if(tensor.type == OnnxTensor::kFloat32)
	{
		tensor.f.resize(count);
	}
	else
	{
		tensor.u8.resize(count);
	}
	for(int y=0; y<H; ++y)
	{
		const unsigned char * row = img.ptr<unsigned char>(y);
		for(int x=0; x<W; ++x)
		{
			for(int c=0; c<C; ++c)
			{
				const size_t index = channelsLast?((size_t)y*W + x)*C + c:(size_t)c*H*W + (size_t)y*W + x;
				if(tensor.type == OnnxTensor::kFloat32)
				{
					tensor.f[index] = float(row[x]) / 255.0f;
				}
				else
				{
					tensor.u8[index] = row[x];
				}
			}
		}
	}
	inputs.clear();
	inputs.push_back(tensor);
	return true;
}

bool OnnxDetector::parseOutputs(const std::vector<OnnxTensor> & outputs, std::vector<cv::KeyPoint> & keypoints, cv::Mat & descriptors) const
{
	keypoints.clear();
	descriptors = cv::Mat();

	if(outputs.size() < 2)
	{
		UERROR("ONNX model \"%s\" has %d outputs, OnnxDetector expects at least keypoints and descriptors.", modelPath_.c_str(), (int)outputs.size());
		return false;
	}

	std::vector<int> taken;
	int kptsIndex = findOutput(outputs, {"keypoints", "kpts"}, &isKeypointsLike, taken);
	if(kptsIndex >= 0) taken.push_back(kptsIndex);
	int descIndex = findOutput(outputs, {"descriptors", "desc"}, &isDescriptorsLike, taken);
	if(descIndex >= 0) taken.push_back(descIndex);
	int scoresIndex = findOutput(outputs, {"scores", "keypoint_scores", "keypoints_scores"}, &isScoresLike, taken);

	if(kptsIndex < 0 || descIndex < 0)
	{
		std::string names;
		for(size_t i=0; i<outputs.size(); ++i)
		{
			names += (i?", ":"") + outputs[i].toString();
		}
		UERROR("ONNX model \"%s\": cannot find the keypoints ([1,N,2]) and descriptors ([1,N,D] float32) among the outputs: %s", modelPath_.c_str(), names.c_str());
		return false;
	}

	const OnnxTensor & kpts = outputs[kptsIndex];
	const OnnxTensor & desc = outputs[descIndex];
	const std::vector<int64_t> kptsShape = squeezeBatch(kpts.shape);
	const std::vector<int64_t> descShape = squeezeBatch(desc.shape);
	if(!isKeypointsLike(kpts) || !isDescriptorsLike(desc))
	{
		UERROR("ONNX model \"%s\": unexpected keypoints (%s, expected [1,N,2]) or descriptors (%s, expected [1,N,D] float32).",
				modelPath_.c_str(), kpts.toString().c_str(), desc.toString().c_str());
		return false;
	}
	const int n = (int)kptsShape[0];
	const int kptsColumns = (int)kptsShape[1]; // x,y or x,y,size,orientation
	const int dim = (int)descShape[1];
	if((int)descShape[0] != n)
	{
		UERROR("ONNX model \"%s\": %d keypoints but %d descriptors.", modelPath_.c_str(), n, (int)descShape[0]);
		return false;
	}
	const float * scores = 0;
	if(scoresIndex >= 0)
	{
		const OnnxTensor & s = outputs[scoresIndex];
		if(isScoresLike(s) && (int)squeezeBatch(s.shape)[0] == n)
		{
			scores = s.f.data();
		}
		else
		{
			UWARN("ONNX model \"%s\": ignoring scores output %s not matching the %d keypoints.", modelPath_.c_str(), s.toString().c_str(), n);
		}
	}
	if(n == 0)
	{
		return true;
	}

	keypoints.resize(n);
	for(int i=0; i<n; ++i)
	{
		float v[4] = {0.0f, 0.0f, 8.0f, -1.0f}; // x, y, size, angle (degrees)
		for(int c=0; c<kptsColumns; ++c)
		{
			const size_t index = (size_t)i*kptsColumns + c;
			v[c] = kpts.type == OnnxTensor::kFloat32?kpts.f[index]:
				   kpts.type == OnnxTensor::kInt64?(float)kpts.i64[index]:(float)kpts.i32[index];
		}
		if(kptsColumns == 4)
		{
			v[3] = v[3] * 180.0f / float(CV_PI); // radians to OpenCV degrees
		}
		keypoints[i] = cv::KeyPoint(v[0], v[1], v[2], v[3], scores?scores[i]:1.0f);
	}
	descriptors = cv::Mat(n, dim, CV_32FC1, const_cast<float*>(desc.f.data())).clone();
	return true;
}

std::vector<cv::KeyPoint> OnnxDetector::generateKeypointsImpl(const cv::Mat & image, const cv::Rect & roi, const cv::Mat & mask)
{
	UASSERT(!image.empty() && image.channels() == 1 && image.depth() == CV_8U);
	std::vector<cv::KeyPoint> keypoints;

	// New image, or the first cell of the grid again: start a new descriptor set.
	if(image.data != lastImageData_ || image.size() != lastImageSize_ || roi == firstRoi_)
	{
		descriptors_ = cv::Mat();
		lastImageData_ = image.data;
		lastImageSize_ = image.size();
		firstRoi_ = roi;
	}

	if(session_.get() == 0)
	{
		UERROR("ONNX detector model not loaded (%s=\"%s\")!", Parameters::kOnnxDetectorModelPath().c_str(), modelPath_.c_str());
		return keypoints;
	}

	UTimer timer;
	cv::Mat imgRoi(image, roi);
	std::vector<OnnxTensor> inputs;
	cv::Size modelImageSize;
	if(!prepareInputs(imgRoi, inputs, modelImageSize))
	{
		return keypoints;
	}
	UDEBUG("Preparing data time = %fs", timer.ticks());

	std::vector<OnnxTensor> outputs;
	if(!session_->run(inputs, outputs))
	{
		return keypoints;
	}
	UDEBUG("ONNX detector time = %fs", timer.ticks());

	std::vector<cv::KeyPoint> modelKeypoints;
	cv::Mat modelDescriptors;
	if(!parseOutputs(outputs, modelKeypoints, modelDescriptors))
	{
		return keypoints;
	}
	UASSERT(modelDescriptors.rows == (int)modelKeypoints.size());

	const float scaleX = modelImageSize.width > 0?float(imgRoi.cols)/float(modelImageSize.width):1.0f;
	const float scaleY = modelImageSize.height > 0?float(imgRoi.rows)/float(modelImageSize.height):1.0f;
	cv::Mat descriptors;
	keypoints.reserve(modelKeypoints.size());
	for(size_t i=0; i<modelKeypoints.size(); ++i)
	{
		cv::KeyPoint kpt = modelKeypoints[i];
		if(threshold_ > 0.0f && kpt.response < threshold_)
		{
			continue;
		}
		kpt.pt.x *= scaleX;
		kpt.pt.y *= scaleY;
		if(kpt.pt.x < 0 || kpt.pt.y < 0 || kpt.pt.x >= imgRoi.cols || kpt.pt.y >= imgRoi.rows)
		{
			continue;
		}
		if(!mask.empty())
		{
			// keypoints are in ROI coordinates, the mask is in image coordinates
			const int x = int(kpt.pt.x) + roi.x;
			const int y = int(kpt.pt.y) + roi.y;
			if(x < 0 || y < 0 || x >= mask.cols || y >= mask.rows || mask.at<unsigned char>(y, x) == 0)
			{
				continue;
			}
		}
		keypoints.push_back(kpt);
		descriptors.push_back(modelDescriptors.row((int)i));
	}
	UDEBUG("%d keypoints (%d from the model, threshold=%f, mask empty=%d)", (int)keypoints.size(), (int)modelKeypoints.size(), threshold_, mask.empty()?1:0);

	// Like the base class does for the other detectors, the limit is per grid cell.
	if(getMaxFeatures() > 0)
	{
		const int cells = std::max(1, getGridRows() * getGridCols());
		limitKeypoints(keypoints, descriptors, getMaxFeatures() / cells, roi.size(), getSSC());
	}
	UASSERT(descriptors.rows == (int)keypoints.size());
	descriptors_.push_back(descriptors);
	UDEBUG("Post-processing time = %fs", timer.ticks());

	return keypoints;
}

cv::Mat OnnxDetector::generateDescriptorsImpl(const cv::Mat & image, std::vector<cv::KeyPoint> & keypoints) const
{
	if((int)keypoints.size() != descriptors_.rows)
	{
		UERROR("The number of keypoints (%d) doesn't match the number of buffered "
				"descriptors (%d). OnnxDetector's descriptors extraction should "
				"be called right after keypoints detection, with same keypoints "
				"returned by the detection. Returning empty descriptors.",
				(int)keypoints.size(), descriptors_.rows);
		return cv::Mat();
	}
	return descriptors_;
}

} // namespace rtabmap
