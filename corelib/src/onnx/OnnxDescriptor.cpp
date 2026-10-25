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

#include "onnx/OnnxDescriptor.h"

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

} // namespace

OnnxDescriptor::OnnxDescriptor(const ParametersMap & parameters) :
		GlobalDescriptorExtractor(parameters),
		modelPath_(Parameters::defaultOnnxDescriptorModelPath()),
		dim_(Parameters::defaultOnnxDescriptorDim()),
		provider_(Parameters::defaultOnnxProvider()),
		tensorRTFp16_(Parameters::defaultOnnxTensorRTFp16()),
		tensorRTCachePath_(Parameters::defaultOnnxTensorRTCachePath()),
		resizeWarned_(false)
{
	parseParameters(parameters);
}

OnnxDescriptor::~OnnxDescriptor()
{
}

void OnnxDescriptor::parseParameters(const ParametersMap & parameters)
{
	std::string previousModelPath = modelPath_;
	int previousProvider = provider_;
	bool previousFp16 = tensorRTFp16_;
	std::string previousCachePath = tensorRTCachePath_;

	Parameters::parse(parameters, Parameters::kOnnxDescriptorModelPath(), modelPath_);
	Parameters::parse(parameters, Parameters::kOnnxDescriptorDim(), dim_);
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
		if(modelPath_.empty())
		{
			UERROR("Cannot initialize ONNX descriptor, %s is not set.", Parameters::kOnnxDescriptorModelPath().c_str());
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
				UERROR("Cannot initialize ONNX descriptor with %s=\"%s\".", Parameters::kOnnxDescriptorModelPath().c_str(), modelPath_.c_str());
			}
		}
	}
}

bool OnnxDescriptor::prepareInputs(const cv::Mat & image, std::vector<OnnxTensor> & inputs) const
{
	UASSERT(session_.get() != 0);
	UASSERT(!image.empty() && image.depth() == CV_8U && (image.channels() == 1 || image.channels() == 3));

	if(session_->inputs().size() != 1)
	{
		UERROR("ONNX model \"%s\" has %d inputs, OnnxDescriptor expects a single image input.", modelPath_.c_str(), (int)session_->inputs().size());
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
	const int64_t d1 = input.dim(1);
	const int64_t d3 = input.dim(3);
	const bool channelsLast = (d3 == 1 || d3 == 3) && !(d1 == 1 || d1 == 3);
	int64_t channels = channelsLast?d3:d1;
	if(channels == -1)
	{
		channels = 3;
	}
	if(channels != 1 && channels != 3)
	{
		UERROR("ONNX model \"%s\": input %s should have 1 or 3 channels.", modelPath_.c_str(), input.toString().c_str());
		return false;
	}
	const int64_t modelHeight = channelsLast?input.dim(1):input.dim(2);
	const int64_t modelWidth = channelsLast?input.dim(2):input.dim(3);

	// RGB for the model, BGR in SensorData
	cv::Mat img;
	if(channels == 3)
	{
		cv::cvtColor(image, img, image.channels() == 3?cv::COLOR_BGR2RGB:cv::COLOR_GRAY2RGB);
	}
	else if(image.channels() == 3)
	{
		cv::cvtColor(image, img, cv::COLOR_BGR2GRAY);
	}
	else
	{
		img = image;
	}
	if(modelHeight > 0 && modelWidth > 0 && (modelHeight != img.rows || modelWidth != img.cols))
	{
		if(!resizeWarned_)
		{
			UWARN("ONNX model \"%s\" has a fixed input size of %dx%d, images of %dx%d are resized to it (this warning is shown only once).",
					modelPath_.c_str(), (int)modelWidth, (int)modelHeight, img.cols, img.rows);
			resizeWarned_ = true;
		}
		cv::resize(img, img, cv::Size((int)modelWidth, (int)modelHeight), 0, 0, modelWidth < img.cols?cv::INTER_AREA:cv::INTER_LINEAR);
	}

	const int H = img.rows;
	const int W = img.cols;
	const int C = (int)channels;
	OnnxTensor tensor;
	tensor.name = input.name;
	tensor.type = input.type;
	tensor.shape = channelsLast?std::vector<int64_t>{1, H, W, C}:std::vector<int64_t>{1, C, H, W};
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
				const unsigned char value = row[x*C + c];
				if(tensor.type == OnnxTensor::kFloat32)
				{
					tensor.f[index] = float(value) / 255.0f;
				}
				else
				{
					tensor.u8[index] = value;
				}
			}
		}
	}
	inputs.clear();
	inputs.push_back(tensor);
	return true;
}

bool OnnxDescriptor::parseOutputs(const std::vector<OnnxTensor> & outputs, cv::Mat & descriptor) const
{
	descriptor = cv::Mat();
	if(outputs.empty())
	{
		UERROR("ONNX model \"%s\" has no output.", modelPath_.c_str());
		return false;
	}
	const OnnxTensor & out = outputs[0];
	if(out.type != OnnxTensor::kFloat32 || out.f.empty())
	{
		UERROR("ONNX model \"%s\": output %s should be a float32 descriptor.", modelPath_.c_str(), out.toString().c_str());
		return false;
	}
	descriptor = cv::Mat(1, (int)out.f.size(), CV_32FC1, const_cast<float*>(out.f.data())).clone();
	return true;
}

GlobalDescriptor OnnxDescriptor::extract(const SensorData & data) const
{
	GlobalDescriptor descriptor;
	if(session_.get() == 0)
	{
		UERROR("ONNX descriptor model not loaded (%s=\"%s\")!", Parameters::kOnnxDescriptorModelPath().c_str(), modelPath_.c_str());
		return descriptor;
	}
	if(data.imageRaw().empty())
	{
		UERROR("No image in the sensor data to extract a global descriptor from.");
		return descriptor;
	}
	if(data.imageRaw().depth() != CV_8U || (data.imageRaw().channels() != 1 && data.imageRaw().channels() != 3))
	{
		UERROR("OnnxDescriptor expects a CV_8UC1 or CV_8UC3 image (type=%d).", data.imageRaw().type());
		return descriptor;
	}

	UTimer timer;
	std::vector<OnnxTensor> inputs;
	if(!prepareInputs(data.imageRaw(), inputs))
	{
		return descriptor;
	}
	UDEBUG("Preparing data time = %fs", timer.ticks());

	std::vector<OnnxTensor> outputs;
	if(!session_->run(inputs, outputs))
	{
		return descriptor;
	}
	UDEBUG("ONNX descriptor time = %fs", timer.ticks());

	cv::Mat values;
	if(!parseOutputs(outputs, values))
	{
		return descriptor;
	}
	if(dim_ > 0 && dim_ < values.cols)
	{
		// the leading components, re-normalized (like PyDescriptor)
		values = values.colRange(0, dim_).clone();
		const double norm = cv::norm(values);
		if(norm > 0.0)
		{
			values /= norm;
		}
	}
	UDEBUG("Descriptor of %d values", values.cols);
	return GlobalDescriptor((int)kOnnxDescriptor, values);
}

} // namespace rtabmap
