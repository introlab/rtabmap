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

#ifndef RTABMAP_ONNXDETECTOR_H_
#define RTABMAP_ONNXDETECTOR_H_

#include <rtabmap/core/Features2d.h>
#include "onnx/OnnxSession.h"
#include "rtabmap/core/rtabmap_core_export.h"

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <memory>
#include <string>
#include <vector>

namespace rtabmap {

/**
 * @brief Local feature detector and descriptor running an ONNX model
 * (Kp/DetectorStrategy=17, Vis/FeatureType=17), e.g. SuperPoint, ALIKED,
 * DISK or any model exported with the interface below.
 *
 * Only the model path (OnnxDetector/ModelPath) is needed when the model
 * follows this interface, which is the one of the extractors exported by
 * [LightGlue-ONNX](https://github.com/fabio-sim/LightGlue-ONNX) and by
 * corelib/src/onnx/export_superpoint_onnx.py:
 *
 * **Input** (one tensor): the image
 *   - `float32 [1,1,H,W]`: grayscale, intensities scaled to [0,1];
 *   - or `uint8 [1,1,H,W]`: grayscale, raw intensities;
 *   - `[1,3,H,W]` (the grayscale image is replicated on the 3 channels) and
 *     `[1,H,W,1]` layouts are accepted too.
 *   `H` and `W` may be dynamic; when they are fixed, the image is resized to
 *   the model's size and the keypoints are scaled back.
 *
 * **Outputs** (found by name, or by shape when the names differ):
 *   - `keypoints` `[1,N,2]` (`float32` or `int64`): x,y pixel coordinates, or
 *     `[1,N,4]` with the keypoint size (pixels) and orientation (radians) too;
 *   - `scores` `[1,N]` `float32` (optional): keypoint responses;
 *   - `descriptors` `[1,N,D]` `float32`: one descriptor per keypoint.
 *   The leading batch dimension is optional.
 *
 * Non-maximum suppression and the detection threshold are expected to be in
 * the model (that is where they are fast): OnnxDetector/Threshold is only a
 * second filter on `scores`, then the keypoints are limited to Kp/MaxFeatures
 * (or Vis/MaxFeatures) by response (or SSC). Keypoints and descriptors come
 * from the same inference: generateDescriptors() should be called with the
 * keypoints returned by generateKeypoints() on the same image, like with
 * PyDetector.
 *
 * A model with a different interface can be supported by overriding
 * prepareInputs() / parseOutputs() in a subclass.
 */
class RTABMAP_CORE_EXPORT OnnxDetector : public Feature2D
{
public:
	OnnxDetector(const ParametersMap & parameters = ParametersMap());
	virtual ~OnnxDetector();

	virtual void parseParameters(const ParametersMap & parameters) override;
	virtual Feature2D::Type getType() const override {return kFeatureOnnxDetector;}
	/** @brief True when the ONNX Runtime build has the CUDA provider (what the model is run on depends on OnnxDetector/Provider). */
	virtual bool isGpuAvailable() const override;

	const std::string & modelPath() const {return modelPath_;}
	float threshold() const {return threshold_;}
	int provider() const {return provider_;}
	bool isModelLoaded() const {return session_.get() != 0;}
	/** @brief The session (null if the model is not loaded), e.g. to inspect its inputs and outputs. */
	const std::shared_ptr<OnnxSession> & session() const {return session_;}

protected:
	/**
	 * @brief Fills the model inputs from the grayscale image (CV_8UC1).
	 * @param image The (ROI of the) image to process.
	 * @param inputs Output, the tensors to feed to the model.
	 * @param modelImageSize Output, the size at which the model sees the image (keypoints are
	 *                       scaled by image.size()/modelImageSize when they differ).
	 * @return false if the image cannot be converted for this model.
	 */
	virtual bool prepareInputs(const cv::Mat & image, std::vector<OnnxTensor> & inputs, cv::Size & modelImageSize) const;
	/**
	 * @brief Converts the model outputs to keypoints and descriptors.
	 * @param outputs The model outputs, in the model's order.
	 * @param keypoints Output, with x,y in the coordinates of the image given to prepareInputs()
	 *                  (before scaling) and the model's score as response.
	 * @param descriptors Output, CV_32FC1 with one row per keypoint.
	 * @return false if the outputs don't follow the expected interface.
	 */
	virtual bool parseOutputs(const std::vector<OnnxTensor> & outputs, std::vector<cv::KeyPoint> & keypoints, cv::Mat & descriptors) const;

private:
	virtual std::vector<cv::KeyPoint> generateKeypointsImpl(const cv::Mat & image, const cv::Rect & roi, const cv::Mat & mask = cv::Mat()) override;
	virtual cv::Mat generateDescriptorsImpl(const cv::Mat & image, std::vector<cv::KeyPoint> & keypoints) const override;

private:
	std::shared_ptr<OnnxSession> session_;
	std::string modelPath_;
	float threshold_;
	int provider_;
	bool tensorRTFp16_;
	std::string tensorRTCachePath_;

	// Descriptors of the keypoints returned by the last generateKeypoints():
	// they are produced with the keypoints, so they are kept for
	// generateDescriptors(). With a grid (Kp/GridRows, Kp/GridCols), the base
	// class calls generateKeypointsImpl() once per cell on the same image:
	// the descriptors are then accumulated over the cells.
	cv::Mat descriptors_;
	const void * lastImageData_;
	cv::Size lastImageSize_;
	cv::Rect firstRoi_;
	mutable bool resizeWarned_;
};

} // namespace rtabmap

#endif /* RTABMAP_ONNXDETECTOR_H_ */
