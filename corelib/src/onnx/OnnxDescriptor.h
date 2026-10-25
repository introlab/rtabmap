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

#ifndef RTABMAP_ONNXDESCRIPTOR_H_
#define RTABMAP_ONNXDESCRIPTOR_H_

#include <rtabmap/core/GlobalDescriptorExtractor.h>
#include "onnx/OnnxSession.h"
#include "rtabmap/core/rtabmap_core_export.h"

#include <memory>
#include <string>
#include <vector>

namespace rtabmap {

/**
 * @brief Global (whole image) descriptor extractor running an ONNX model
 * (Mem/GlobalDescriptorStrategy=2), e.g. NetVLAD, or any model exported with
 * the interface below.
 *
 * Only the model path (OnnxDescriptor/ModelPath) is needed when the model
 * follows this interface, which is the one of
 * corelib/src/onnx/export_netvlad_onnx.py:
 *
 * **Input** (one tensor): the image
 *   - `float32 [1,3,H,W]`: RGB, intensities scaled to [0,1];
 *   - or `float32 [1,1,H,W]`: grayscale;
 *   - `uint8` and `[1,H,W,C]` layouts are accepted too.
 *   `H` and `W` may be dynamic; when they are fixed, the image is resized to
 *   the model's size.
 *
 * **Output** (the first one): the descriptor, `float32` with `D` values in any
 * shape (`[1,D]` usually). OnnxDescriptor/Dim keeps the first `Dim` values and
 * L2-normalizes them (0 keeps the descriptor as is), like PyDescriptor does.
 *
 * A model with a different interface can be supported by overriding
 * prepareInputs() / parseOutputs() in a subclass.
 */
class RTABMAP_CORE_EXPORT OnnxDescriptor : public GlobalDescriptorExtractor
{
public:
	OnnxDescriptor(const ParametersMap & parameters = ParametersMap());
	virtual ~OnnxDescriptor();

	virtual void parseParameters(const ParametersMap & parameters) override;
	virtual GlobalDescriptor extract(const SensorData & data) const override;
	virtual GlobalDescriptorExtractor::Type getType() const override {return kOnnxDescriptor;}

	const std::string & modelPath() const {return modelPath_;}
	int dim() const {return dim_;}
	int provider() const {return provider_;}
	bool isModelLoaded() const {return session_.get() != 0;}
	/** @brief The session (null if the model is not loaded), e.g. to inspect its inputs and outputs. */
	const std::shared_ptr<OnnxSession> & session() const {return session_;}

protected:
	/**
	 * @brief Fills the model inputs from the image (CV_8UC1 or CV_8UC3 BGR, as in SensorData).
	 * @return false if the image cannot be converted for this model.
	 */
	virtual bool prepareInputs(const cv::Mat & image, std::vector<OnnxTensor> & inputs) const;
	/**
	 * @brief Converts the model outputs to the descriptor (1xD CV_32FC1).
	 * @return false if the outputs don't follow the expected interface.
	 */
	virtual bool parseOutputs(const std::vector<OnnxTensor> & outputs, cv::Mat & descriptor) const;

private:
	std::shared_ptr<OnnxSession> session_;
	std::string modelPath_;
	int dim_;
	int provider_;
	bool tensorRTFp16_;
	std::string tensorRTCachePath_;
	mutable bool resizeWarned_;
};

} // namespace rtabmap

#endif /* RTABMAP_ONNXDESCRIPTOR_H_ */
