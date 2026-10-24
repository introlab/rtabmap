/**
 * ONNX Runtime session shared by the ONNX models (LightGlueOnnx, AlikedOnnxDetector):
 * execution provider selection (CPU, CUDA, TensorRT) with fallback to what the
 * ONNX Runtime build provides, and TensorRT FP16 / engine cache options.
 */

#ifndef ONNXSESSION_H
#define ONNXSESSION_H

#include <memory>
#include <string>
#include <vector>

namespace Ort {
struct Env;
struct Session;
}

namespace rtabmap
{

class OnnxSession
{
public:
	enum Provider {kCPU=0, kCUDA=1, kTensorRT=2};

	// Session for this model and these options, shared with the other users of
	// the same model and options (feature detectors and matchers are created
	// several times by RTAB-Map: one TensorRT engine each would waste GPU memory).
	// ONNX Runtime sessions can be run concurrently.
	// tensorRTShapes: optional TensorRT optimization profile for dynamic inputs,
	// {min, opt, max} in TensorRT EP format ("input:1x2x2,other:1x2x256"). Leave
	// empty for models with static shapes.
	static std::shared_ptr<OnnxSession> get(
			const std::string & name,
			const std::string & modelPath,
			int provider,
			bool tensorRTFp16,
			const std::string & tensorRTCachePath,
			const std::vector<std::string> & tensorRTShapes = std::vector<std::string>());

	OnnxSession(
			const std::string & name,
			const std::string & modelPath,
			int provider,
			bool tensorRTFp16,
			const std::string & tensorRTCachePath,
			const std::vector<std::string> & tensorRTShapes = std::vector<std::string>());
	virtual ~OnnxSession();

	bool isValid() const {return session_.get() != 0;}
	Ort::Session & session() {return *session_;}
	int activeProvider() const {return activeProvider_;}
	const std::string & path() const {return path_;}
	std::string providerName() const;

	// Input/output shapes as stored in the model (-1 for dynamic dimensions),
	// readable before choosing provider options.
	static bool readShapes(
			const std::string & modelPath,
			std::vector<std::string> & inputNames,
			std::vector<std::vector<int64_t> > & inputShapes,
			std::vector<std::string> & outputNames,
			std::vector<std::vector<int64_t> > & outputShapes);

private:
	std::string path_;
	int activeProvider_;
	bool tensorRTFp16_;
	std::unique_ptr<Ort::Env> env_;
	std::unique_ptr<Ort::Session> session_;
};

}

#endif
