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

#ifndef RTABMAP_ONNXSESSION_H_
#define RTABMAP_ONNXSESSION_H_

#include "rtabmap/core/rtabmap_core_export.h"
#include <rtabmap/core/Parameters.h>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace rtabmap {

/**
 * @brief A dense tensor exchanged with an ONNX model, independent of the
 * ONNX Runtime API so that OnnxDetector / OnnxMatcher specializations don't
 * need the ONNX Runtime headers.
 *
 * For the model's input/output descriptions (OnnxSession::inputs() and
 * outputs()), @ref shape holds -1 for dynamic dimensions and @ref data is
 * empty. For values passed to or returned by OnnxSession::run(), @ref shape
 * is concrete and exactly one of the data vectors matching @ref type is filled
 * (row-major).
 */
class RTABMAP_CORE_EXPORT OnnxTensor
{
public:
	enum Type {kFloat32, kInt64, kInt32, kUInt8, kUnsupported};

	OnnxTensor() : type(kFloat32) {}

	/** @brief Number of elements described by @ref shape (0 if any dimension is dynamic). */
	size_t count() const;
	/** @brief Dimension @p i, or @p defaultValue when the tensor has fewer dimensions. */
	int64_t dim(size_t i, int64_t defaultValue = -1) const {return i < shape.size() ? shape[i] : defaultValue;}
	/** @brief True if every dimension is known. */
	bool isStatic() const;
	/** @brief Human-readable description, e.g. "image float32 [1,1,-1,-1]". */
	std::string toString() const;
	static std::string typeName(Type type);

	std::string name;
	std::vector<int64_t> shape;
	Type type;
	std::vector<float> f;      ///< data when type == kFloat32
	std::vector<int64_t> i64;  ///< data when type == kInt64
	std::vector<int32_t> i32;  ///< data when type == kInt32
	std::vector<uint8_t> u8;   ///< data when type == kUInt8
};

/**
 * @brief An ONNX Runtime inference session shared between the objects that
 * use the same model with the same options (RTAB-Map creates many Feature2D
 * and RegistrationVis instances, each of them would otherwise load its own
 * copy of the model and, with TensorRT, build its own engine).
 *
 * Sessions are obtained with get(); the one ONNX Runtime environment of the
 * process is created on first use with a logging level following ULogger's.
 * run() is thread-safe (ONNX Runtime's Session::Run() is).
 */
class RTABMAP_CORE_EXPORT OnnxSession
{
public:
	/** @brief Execution providers, values of the OnnxDetector/Provider and OnnxMatcher/Provider parameters. */
	enum Provider {kProviderCPU=0, kProviderCUDA=1, kProviderTensorRT=2};

	struct Options
	{
		Options() : provider(kProviderCPU), deviceId(0), tensorRTFp16(false) {}
		Provider provider;
		int deviceId;                   ///< GPU device for CUDA and TensorRT
		bool tensorRTFp16;              ///< Build the TensorRT engine in FP16
		std::string tensorRTCachePath;  ///< Directory caching TensorRT engines (empty = no cache)
		std::string key() const;        ///< Identifies the options in the session cache
	};

	/**
	 * @brief Returns a session for @p modelPath, shared with any other user of the
	 * same model and options still alive. Null if the model cannot be loaded (the
	 * reason is logged with UERROR). A provider not available in the ONNX Runtime
	 * build falls back to the CPU one with a warning: see provider() for the one
	 * effectively used.
	 */
	static std::shared_ptr<OnnxSession> get(const std::string & modelPath, const Options & options = Options());

	/** @brief True if the ONNX Runtime library can use @p provider. */
	static bool isProviderAvailable(Provider provider);
	static std::string providerName(Provider provider);
	/** @brief ONNX Runtime version string, e.g. "1.23.2". */
	static std::string runtimeVersion();
	/**
	 * @brief Where the TensorRT engines are cached when Onnx/TensorRTCachePath is
	 * empty: the "tensorrt_cache" folder of the RTAB-Map working directory
	 * (Rtabmap/WorkingDirectory in @p parameters, else the default one).
	 */
	static std::string defaultTensorRTCachePath(const ParametersMap & parameters);

	virtual ~OnnxSession();

	const std::string & modelPath() const {return modelPath_;}
	/** @brief Provider effectively used (after fallback). */
	Provider provider() const {return provider_;}
	/** @brief Model inputs: name, element type and shape (-1 for dynamic dimensions). */
	const std::vector<OnnxTensor> & inputs() const {return inputs_;}
	/** @brief Model outputs: name, element type and shape (-1 for dynamic dimensions). */
	const std::vector<OnnxTensor> & outputs() const {return outputs_;}
	/** @brief Index of input @p name, or -1 if the model has no input with this name. */
	int inputIndex(const std::string & name) const;
	/** @brief Index of output @p name, or -1 if the model has no output with this name. */
	int outputIndex(const std::string & name) const;

	/**
	 * @brief Runs the model.
	 * @param inputs One tensor per model input, matched by name (OnnxTensor::name)
	 *               when the names are set, by position otherwise.
	 * @param outputs All the model outputs, in the model's order.
	 * @return false on error (logged with UERROR), @p outputs is then left empty.
	 */
	bool run(const std::vector<OnnxTensor> & inputs, std::vector<OnnxTensor> & outputs) const;

private:
	OnnxSession(const std::string & modelPath, const Options & options);
	bool load();

private:
	struct Impl;
	std::unique_ptr<Impl> impl_;
	std::string modelPath_;
	Options options_;
	Provider provider_;
	std::vector<OnnxTensor> inputs_;
	std::vector<OnnxTensor> outputs_;
};

} // namespace rtabmap

#endif /* RTABMAP_ONNXSESSION_H_ */
