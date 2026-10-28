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

#ifndef RTABMAP_ONNXMATCHER_H_
#define RTABMAP_ONNXMATCHER_H_

#include "onnx/OnnxSession.h"
#include "rtabmap/core/rtabmap_core_export.h"

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <memory>
#include <string>
#include <vector>

namespace rtabmap {

/**
 * @brief Feature matcher running an ONNX model (Vis/CorNNType=10), e.g.
 * LightGlue, or any model exported with the interface below.
 *
 * Only the model path (OnnxMatcher/ModelPath) is needed when the model
 * follows this interface, which is the one of the matchers exported by
 * [LightGlue-ONNX](https://github.com/fabio-sim/LightGlue-ONNX) and by
 * corelib/src/onnx/export_lightglue_onnx.py:
 *
 * **Inputs** (found by name, or by position in this order when the names differ):
 *   - `kpts0` `float32 [1,M,2]` and `kpts1` `float32 [1,N,2]`: the keypoints of
 *     the query and train images, normalized like LightGlue does, i.e. for an
 *     image of size (w,h), `(x - w/2) / (max(w,h)/2)` and `(y - h/2) / (max(w,h)/2)`.
 *     A model trained with the keypoint scale and orientation (LightGlue for
 *     SIFT or DoG-HardNet) declares them `[1,M,4]`: the two extra columns are
 *     cv::KeyPoint::size (not normalized) and cv::KeyPoint::angle in radians;
 *   - `desc0` `float32 [1,M,D]` and `desc1` `float32 [1,N,D]`: their descriptors;
 *   - optionally `scores0` `float32 [1,M]` and `scores1` `float32 [1,N]`: the
 *     keypoint detection scores (cv::KeyPoint::response), for models taking
 *     them (SuperGlue).
 *
 * **Outputs** (found by name, or by shape when the names differ):
 *   - `matches0` `int64 [1,M]`: for each query keypoint, the index of the matched
 *     train keypoint or -1, with optionally `mscores0` `float32 [1,M]`, the
 *     confidence of each match;
 *   - or `matches` `int64 [K,2]`: (query, train) index pairs, with optionally
 *     `mscores` `float32 [K]`.
 *   Other outputs (e.g. `matches1`, `mscores1`) are ignored and the leading batch
 *   dimension is optional. When the scores are there, matches with a score
 *   below OnnxMatcher/Threshold are dropped and cv::DMatch::distance is
 *   `1 - score`, otherwise it is 0.
 *
 * A model with a different interface can be supported by overriding
 * prepareInputs() / parseOutputs() in a subclass.
 */
class RTABMAP_CORE_EXPORT OnnxMatcher
{
public:
	/**
	 * @param modelPath Path to the ONNX model.
	 * @param matchThreshold Minimum score of a match (when the model outputs scores).
	 * @param provider OnnxSession::Provider value (0=CPU, 1=CUDA, 2=TensorRT).
	 * @param tensorRTFp16 Build the TensorRT engine in FP16.
	 * @param tensorRTCachePath Directory caching TensorRT engines (empty = no cache).
	 */
	OnnxMatcher(
			const std::string & modelPath,
			float matchThreshold = 0.1f,
			int provider = 0,
			bool tensorRTFp16 = false,
			const std::string & tensorRTCachePath = "");
	virtual ~OnnxMatcher();

	const std::string & modelPath() const {return modelPath_;}
	float matchThreshold() const {return matchThreshold_;}
	int provider() const {return provider_;}
	bool tensorRTFp16() const {return tensorRTFp16_;}
	const std::string & tensorRTCachePath() const {return tensorRTCachePath_;}
	bool isModelLoaded() const {return session_.get() != 0;}
	/** @brief The session (null if the model is not loaded), e.g. to inspect its inputs and outputs. */
	const std::shared_ptr<OnnxSession> & session() const {return session_;}

	/**
	 * @brief Matches the query features to the train ones.
	 * @param descriptorsQuery CV_32FC1, one row per query keypoint.
	 * @param descriptorsTrain CV_32FC1, one row per train keypoint, same columns.
	 * @param keypointsQuery Keypoints of the query image, in pixels.
	 * @param keypointsTrain Keypoints of the train image, in pixels.
	 * @param imageSizeQuery Size of the query image (used to normalize the keypoints).
	 * @param imageSizeTrain Size of the train image, @p imageSizeQuery if not set.
	 * @return The matches (queryIdx -> trainIdx), empty on error.
	 */
	std::vector<cv::DMatch> match(
			const cv::Mat & descriptorsQuery,
			const cv::Mat & descriptorsTrain,
			const std::vector<cv::KeyPoint> & keypointsQuery,
			const std::vector<cv::KeyPoint> & keypointsTrain,
			const cv::Size & imageSizeQuery,
			const cv::Size & imageSizeTrain = cv::Size()) const;

	/** @brief LightGlue's keypoint normalization: centered on the image and divided by half the largest side. */
	static void normalizeKeypoints(const std::vector<cv::KeyPoint> & keypoints, const cv::Size & imageSize, std::vector<float> & xy);
	/**
	 * @brief Fills the keypoint input: normalized x,y with @p columns == 2, plus the
	 * keypoint size and angle in radians (0 when undefined) with @p columns == 4.
	 */
	static void packKeypoints(const std::vector<cv::KeyPoint> & keypoints, const cv::Size & imageSize, int columns, std::vector<float> & data);

protected:
	/**
	 * @brief Fills the model inputs (see the class description for the default interface).
	 * @return false if the inputs cannot be built for this model.
	 */
	virtual bool prepareInputs(
			const cv::Mat & descriptorsQuery,
			const cv::Mat & descriptorsTrain,
			const std::vector<cv::KeyPoint> & keypointsQuery,
			const std::vector<cv::KeyPoint> & keypointsTrain,
			const cv::Size & imageSizeQuery,
			const cv::Size & imageSizeTrain,
			std::vector<OnnxTensor> & inputs) const;
	/**
	 * @brief Converts the model outputs to matches (see the class description for the default interface).
	 * @param outputs The model outputs, in the model's order.
	 * @param queryCount Number of query keypoints.
	 * @param trainCount Number of train keypoints.
	 * @param matches Output, with queryIdx and trainIdx validated against the counts and the threshold applied.
	 * @return false if the outputs don't follow the expected interface.
	 */
	virtual bool parseOutputs(const std::vector<OnnxTensor> & outputs, int queryCount, int trainCount, std::vector<cv::DMatch> & matches) const;

private:
	std::shared_ptr<OnnxSession> session_;
	std::string modelPath_;
	float matchThreshold_;
	int provider_;
	bool tensorRTFp16_;
	std::string tensorRTCachePath_;
};

} // namespace rtabmap

#endif /* RTABMAP_ONNXMATCHER_H_ */
