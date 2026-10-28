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

#include "onnx/OnnxMatcher.h"

#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UDirectory.h>
#include <rtabmap/utilite/UTimer.h>

#include <algorithm>

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

std::vector<int64_t> squeezeBatch(const std::vector<int64_t> & shape)
{
	if(shape.size() > 1 && shape[0] == 1)
	{
		return std::vector<int64_t>(shape.begin()+1, shape.end());
	}
	return shape;
}

bool isIntegerType(const OnnxTensor & t)
{
	return t.type == OnnxTensor::kInt64 || t.type == OnnxTensor::kInt32;
}

int64_t intAt(const OnnxTensor & t, size_t i)
{
	return t.type == OnnxTensor::kInt64?t.i64[i]:(int64_t)t.i32[i];
}

// matches0 [1,M] or [M]: one train index (or -1) per query keypoint.
bool isPerQueryMatches(const OnnxTensor & t)
{
	return isIntegerType(t) && squeezeBatch(t.shape).size() == 1;
}

// matches [K,2] or [1,K,2]: (query, train) pairs.
bool isPairMatches(const OnnxTensor & t)
{
	std::vector<int64_t> shape = squeezeBatch(t.shape);
	return isIntegerType(t) && shape.size() == 2 && shape[1] == 2;
}

int findByName(const std::vector<OnnxTensor> & tensors, const std::vector<std::string> & names)
{
	for(size_t n=0; n<names.size(); ++n)
	{
		for(size_t i=0; i<tensors.size(); ++i)
		{
			if(tensors[i].name.compare(names[n]) == 0)
			{
				return (int)i;
			}
		}
	}
	return -1;
}

void copyDescriptors(const cv::Mat & descriptors, std::vector<float> & data)
{
	data.resize((size_t)descriptors.rows * descriptors.cols);
	for(int i=0; i<descriptors.rows; ++i)
	{
		memcpy(&data[(size_t)i*descriptors.cols], descriptors.ptr<float>(i), descriptors.cols*sizeof(float));
	}
}

} // namespace

OnnxMatcher::OnnxMatcher(
		const std::string & modelPath,
		float matchThreshold,
		int provider,
		bool tensorRTFp16,
		const std::string & tensorRTCachePath) :
				modelPath_(expandTilde(modelPath)),
				matchThreshold_(matchThreshold),
				provider_(provider),
				tensorRTFp16_(tensorRTFp16),
				tensorRTCachePath_(expandTilde(tensorRTCachePath))
{
	if(modelPath_.empty())
	{
		UERROR("Cannot initialize ONNX matcher, the model path is empty.");
		return;
	}
	OnnxSession::Options options;
	options.provider = (OnnxSession::Provider)provider_;
	options.tensorRTFp16 = tensorRTFp16_;
	options.tensorRTCachePath = tensorRTCachePath_;
	session_ = OnnxSession::get(modelPath_, options);
	if(session_.get() == 0)
	{
		UERROR("Cannot initialize ONNX matcher with model \"%s\".", modelPath_.c_str());
	}
}

OnnxMatcher::~OnnxMatcher()
{
}

void OnnxMatcher::normalizeKeypoints(const std::vector<cv::KeyPoint> & keypoints, const cv::Size & imageSize, std::vector<float> & xy)
{
	const float shiftX = float(imageSize.width) / 2.0f;
	const float shiftY = float(imageSize.height) / 2.0f;
	const float scale = float(std::max(imageSize.width, imageSize.height)) / 2.0f;
	xy.resize(keypoints.size()*2);
	for(size_t i=0; i<keypoints.size(); ++i)
	{
		xy[i*2] = (keypoints[i].pt.x - shiftX) / scale;
		xy[i*2+1] = (keypoints[i].pt.y - shiftY) / scale;
	}
}

void OnnxMatcher::packKeypoints(const std::vector<cv::KeyPoint> & keypoints, const cv::Size & imageSize, int columns, std::vector<float> & data)
{
	UASSERT(columns == 2 || columns == 4);
	if(columns == 2)
	{
		normalizeKeypoints(keypoints, imageSize, data);
		return;
	}
	std::vector<float> xy;
	normalizeKeypoints(keypoints, imageSize, xy);
	data.resize(keypoints.size()*4);
	for(size_t i=0; i<keypoints.size(); ++i)
	{
		data[i*4] = xy[i*2];
		data[i*4+1] = xy[i*2+1];
		data[i*4+2] = keypoints[i].size;
		// OpenCV: degrees, -1 when not computed
		data[i*4+3] = keypoints[i].angle < 0.0f?0.0f:keypoints[i].angle * float(CV_PI) / 180.0f;
	}
}

bool OnnxMatcher::prepareInputs(
		const cv::Mat & descriptorsQuery,
		const cv::Mat & descriptorsTrain,
		const std::vector<cv::KeyPoint> & keypointsQuery,
		const std::vector<cv::KeyPoint> & keypointsTrain,
		const cv::Size & imageSizeQuery,
		const cv::Size & imageSizeTrain,
		std::vector<OnnxTensor> & inputs) const
{
	UASSERT(session_.get() != 0);
	const std::vector<OnnxTensor> & modelInputs = session_->inputs();
	if(modelInputs.size() != 4 && modelInputs.size() != 6)
	{
		UERROR("ONNX model \"%s\" has %d inputs, OnnxMatcher expects kpts0, kpts1, desc0 and desc1 (and optionally scores0 and scores1).", modelPath_.c_str(), (int)modelInputs.size());
		return false;
	}

	// By name, by position when the names differ.
	int indices[4] = {findByName(modelInputs, {"kpts0", "keypoints0"}),
					  findByName(modelInputs, {"kpts1", "keypoints1"}),
					  findByName(modelInputs, {"desc0", "descriptors0"}),
					  findByName(modelInputs, {"desc1", "descriptors1"})};
	int scoresIndices[2] = {findByName(modelInputs, {"scores0", "keypoint_scores0"}),
							findByName(modelInputs, {"scores1", "keypoint_scores1"})};
	if(indices[0] < 0 || indices[1] < 0 || indices[2] < 0 || indices[3] < 0)
	{
		for(int i=0; i<4; ++i)
		{
			indices[i] = i;
		}
		if(modelInputs.size() == 6)
		{
			scoresIndices[0] = 4;
			scoresIndices[1] = 5;
		}
	}
	if(modelInputs.size() == 6 && (scoresIndices[0] < 0 || scoresIndices[1] < 0))
	{
		UERROR("ONNX model \"%s\" has 6 inputs but no scores0/scores1 among them.", modelPath_.c_str());
		return false;
	}
	for(int i=0; i<4; ++i)
	{
		const OnnxTensor & input = modelInputs[indices[i]];
		const std::vector<int64_t> shape = squeezeBatch(input.shape);
		if(input.type != OnnxTensor::kFloat32 || shape.size() != 2 || (i < 2 && shape[1] != 2 && shape[1] != 4 && shape[1] != -1))
		{
			UERROR("ONNX model \"%s\": input %s should be float32 [1,N,%s].", modelPath_.c_str(), input.toString().c_str(), i<2?"2] or [1,N,4":"D");
			return false;
		}
		if(i >= 2 && shape[1] > 0 && shape[1] != descriptorsQuery.cols)
		{
			UERROR("ONNX model \"%s\" expects descriptors of dimension %d (input %s), %d provided.", modelPath_.c_str(), (int)shape[1], input.toString().c_str(), descriptorsQuery.cols);
			return false;
		}
	}

	// Keypoints as x,y or as x,y,scale,orientation when the model was trained with them.
	const int columns = squeezeBatch(modelInputs[indices[0]].shape)[1] == 4?4:2;
	inputs.resize(4);
	inputs[0].name = modelInputs[indices[0]].name;
	inputs[0].shape = {1, (int64_t)keypointsQuery.size(), columns};
	packKeypoints(keypointsQuery, imageSizeQuery, columns, inputs[0].f);
	inputs[1].name = modelInputs[indices[1]].name;
	inputs[1].shape = {1, (int64_t)keypointsTrain.size(), columns};
	packKeypoints(keypointsTrain, imageSizeTrain, columns, inputs[1].f);
	inputs[2].name = modelInputs[indices[2]].name;
	inputs[2].shape = {1, (int64_t)descriptorsQuery.rows, (int64_t)descriptorsQuery.cols};
	copyDescriptors(descriptorsQuery, inputs[2].f);
	inputs[3].name = modelInputs[indices[3]].name;
	inputs[3].shape = {1, (int64_t)descriptorsTrain.rows, (int64_t)descriptorsTrain.cols};
	copyDescriptors(descriptorsTrain, inputs[3].f);
	if(scoresIndices[0] >= 0 && scoresIndices[1] >= 0)
	{
		// the keypoint detection scores, for models taking them (SuperGlue)
		inputs.resize(6);
		const std::vector<cv::KeyPoint> * sets[2] = {&keypointsQuery, &keypointsTrain};
		for(int i=0; i<2; ++i)
		{
			inputs[4+i].name = modelInputs[scoresIndices[i]].name;
			inputs[4+i].shape = {1, (int64_t)sets[i]->size()};
			inputs[4+i].f.resize(sets[i]->size());
			for(size_t k=0; k<sets[i]->size(); ++k)
			{
				inputs[4+i].f[k] = (*sets[i])[k].response;
			}
		}
	}
	return true;
}

bool OnnxMatcher::parseOutputs(const std::vector<OnnxTensor> & outputs, int queryCount, int trainCount, std::vector<cv::DMatch> & matches) const
{
	matches.clear();
	if(outputs.empty())
	{
		UERROR("ONNX model \"%s\" has no output.", modelPath_.c_str());
		return false;
	}

	int matchesIndex = findByName(outputs, {"matches0", "matches"});
	if(matchesIndex < 0 || !(isPerQueryMatches(outputs[matchesIndex]) || isPairMatches(outputs[matchesIndex])))
	{
		matchesIndex = -1;
		for(size_t i=0; i<outputs.size(); ++i)
		{
			if(isPerQueryMatches(outputs[i]) || isPairMatches(outputs[i]))
			{
				matchesIndex = (int)i;
				break;
			}
		}
	}
	if(matchesIndex < 0)
	{
		std::string names;
		for(size_t i=0; i<outputs.size(); ++i)
		{
			names += (i?", ":"") + outputs[i].toString();
		}
		UERROR("ONNX model \"%s\": cannot find the matches (int64 [1,M] or [K,2]) among the outputs: %s", modelPath_.c_str(), names.c_str());
		return false;
	}
	const OnnxTensor & m = outputs[matchesIndex];
	const bool perQuery = isPerQueryMatches(m);
	const int count = (int)squeezeBatch(m.shape)[0];

	// Scores: by name, else the float output with one value per match.
	const float * scores = 0;
	int scoresIndex = findByName(outputs, perQuery?
			std::vector<std::string>{"mscores0", "matching_scores0", "scores0"}:
			std::vector<std::string>{"mscores", "matching_scores", "scores"});
	if(scoresIndex < 0)
	{
		for(size_t i=0; i<outputs.size(); ++i)
		{
			if((int)i != matchesIndex && outputs[i].type == OnnxTensor::kFloat32 && squeezeBatch(outputs[i].shape).size() == 1 && (int)outputs[i].f.size() == count)
			{
				scoresIndex = (int)i;
				break;
			}
		}
	}
	if(scoresIndex >= 0)
	{
		if(outputs[scoresIndex].type == OnnxTensor::kFloat32 && (int)outputs[scoresIndex].f.size() == count)
		{
			scores = outputs[scoresIndex].f.data();
		}
		else
		{
			UWARN("ONNX model \"%s\": ignoring scores output %s not matching the %d matches.", modelPath_.c_str(), outputs[scoresIndex].toString().c_str(), count);
		}
	}

	if(perQuery && count != queryCount)
	{
		UERROR("ONNX model \"%s\": %d matches for %d query keypoints.", modelPath_.c_str(), count, queryCount);
		return false;
	}
	matches.reserve(count);
	for(int i=0; i<count; ++i)
	{
		int queryIdx;
		int trainIdx;
		if(perQuery)
		{
			queryIdx = i;
			trainIdx = (int)intAt(m, i);
		}
		else
		{
			queryIdx = (int)intAt(m, i*2);
			trainIdx = (int)intAt(m, i*2+1);
		}
		if(queryIdx < 0 || trainIdx < 0)
		{
			continue;
		}
		if(queryIdx >= queryCount || trainIdx >= trainCount)
		{
			UERROR("ONNX model \"%s\": match (%d,%d) out of range (%d query, %d train keypoints).", modelPath_.c_str(), queryIdx, trainIdx, queryCount, trainCount);
			matches.clear();
			return false;
		}
		float distance = 0.0f;
		if(scores)
		{
			if(scores[i] < matchThreshold_)
			{
				continue;
			}
			distance = 1.0f - scores[i];
		}
		matches.push_back(cv::DMatch(queryIdx, trainIdx, distance));
	}
	return true;
}

std::vector<cv::DMatch> OnnxMatcher::match(
		const cv::Mat & descriptorsQuery,
		const cv::Mat & descriptorsTrain,
		const std::vector<cv::KeyPoint> & keypointsQuery,
		const std::vector<cv::KeyPoint> & keypointsTrain,
		const cv::Size & imageSizeQuery,
		const cv::Size & imageSizeTrainIn) const
{
	UTimer timer;
	std::vector<cv::DMatch> matches;

	if(session_.get() == 0)
	{
		UERROR("ONNX matcher model not loaded (\"%s\")!", modelPath_.c_str());
		return matches;
	}
	const cv::Size imageSizeTrain = imageSizeTrainIn.width > 0 && imageSizeTrainIn.height > 0?imageSizeTrainIn:imageSizeQuery;
	if(descriptorsQuery.empty() || descriptorsTrain.empty())
	{
		return matches;
	}
	if(descriptorsQuery.type() != CV_32F || descriptorsTrain.type() != CV_32F ||
	   descriptorsQuery.cols != descriptorsTrain.cols ||
	   descriptorsQuery.rows != (int)keypointsQuery.size() ||
	   descriptorsTrain.rows != (int)keypointsTrain.size() ||
	   imageSizeQuery.width <= 0 || imageSizeQuery.height <= 0)
	{
		UERROR("Invalid inputs! OnnxMatcher requires float descriptors of the same dimension (query %dx%d type=%d, train %dx%d type=%d), "
				"one per keypoint (%d query, %d train) and the image size (%dx%d).",
				descriptorsQuery.rows, descriptorsQuery.cols, descriptorsQuery.type(),
				descriptorsTrain.rows, descriptorsTrain.cols, descriptorsTrain.type(),
				(int)keypointsQuery.size(), (int)keypointsTrain.size(),
				imageSizeQuery.width, imageSizeQuery.height);
		return matches;
	}

	std::vector<OnnxTensor> inputs;
	if(!prepareInputs(descriptorsQuery, descriptorsTrain, keypointsQuery, keypointsTrain, imageSizeQuery, imageSizeTrain, inputs))
	{
		return matches;
	}
	UDEBUG("Preparing data time = %fs", timer.ticks());

	std::vector<OnnxTensor> outputs;
	if(!session_->run(inputs, outputs))
	{
		return matches;
	}
	UDEBUG("ONNX matching time = %fs", timer.ticks());

	if(!parseOutputs(outputs, descriptorsQuery.rows, descriptorsTrain.rows, matches))
	{
		matches.clear();
	}
	UDEBUG("Matches %d/%d, threshold=%f, parsing time = %fs", (int)matches.size(), std::min(descriptorsQuery.rows, descriptorsTrain.rows), matchThreshold_, timer.ticks());
	return matches;
}

} // namespace rtabmap
