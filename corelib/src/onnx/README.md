# ONNX Runtime backends: OnnxDetector and OnnxMatcher

RTAB-Map can run learned local features and matchers exported to [ONNX](https://onnx.ai) with [ONNX Runtime](https://onnxruntime.ai), without Python or libtorch at runtime:

* **OnnxDetector** (`Kp/DetectorStrategy=17`, `Vis/FeatureType=17`): a detector/descriptor, e.g. SuperPoint, ALIKED, DISK.
* **OnnxMatcher** (`Vis/CorNNType=10`): a matcher, e.g. LightGlue.

Both take a model path as parameter and work with any model following the interfaces below, so adding a new (or re-trained) model doesn't need C++ changes. A model with another interface can be supported by subclassing them (see [Specializing](#specializing)).

## How it works

Detection, in `OnnxDetector` (a `Feature2D`, `Kp/DetectorStrategy=17`):

```mermaid
flowchart LR
    img["grayscale image<br/>(ROI or grid cell)"] --> prep["prepareInputs()<br/>float [1,1,H,W] in [0,1]"]
    prep --> run["OnnxSession::run()"]
    run --> parse["parseOutputs()<br/>keypoints, scores,<br/>descriptors"]
    parse --> post["threshold, mask,<br/>Kp/MaxFeatures or SSC"]
    post --> out["keypoints +<br/>descriptors"]
```

Matching, in `OnnxMatcher` (used by `RegistrationVis` when `Vis/CorNNType=10`):

```mermaid
flowchart LR
    feats["keypoints + descriptors<br/>of the two images"] --> prep["prepareInputs()<br/>keypoints normalized with<br/>the image size"]
    prep --> run["OnnxSession::run()"]
    run --> parse["parseOutputs()<br/>matches0, mscores0"]
    parse --> out["cv::DMatch<br/>(threshold applied)"]
```

Both `run()` calls go through an `OnnxSession` that holds the `Ort::Session` of the model and is shared by every object using the same model and options.

* **OnnxSession** ([OnnxSession.h](OnnxSession.h)) wraps ONNX Runtime: the one `Ort::Env` of the process (its logging level follows ULogger's), an `Ort::Session` per model with the requested execution provider (CUDA, TensorRT or OpenVINO, appended when the library has them, CPU is the fallback), the description of the model's inputs/outputs (names, element types, shapes with -1 for dynamic dimensions) and `run()`. Sessions are cached by model path and options and shared through `std::shared_ptr`: RTAB-Map creates a `Feature2D` per `RegistrationVis`, `Memory`, odometry... each of them would otherwise load its own copy of the model (and build its own TensorRT engine). Tensors are exchanged as `OnnxTensor` (name, shape, type and a plain data vector), so nothing outside `OnnxSession.cpp` includes the ONNX Runtime headers.
* **OnnxDetector** ([OnnxDetector.h](OnnxDetector.h)) is a `Feature2D`: `generateKeypointsImpl()` converts the (ROI of the) grayscale image to the model's input tensor (`prepareInputs()`), runs the session, reads the keypoints/scores/descriptors (`parseOutputs()`), then applies the threshold, the mask, `Kp/MaxFeatures` (per grid cell, by response or SSC) and keeps the descriptors for `generateDescriptorsImpl()`, which returns them when called with the same keypoints (there is no separate "compute" pass in these networks: keypoints and descriptors come from the same inference, like `PyDetector`). `Features2d.cpp` creates it for type 17 and skips its own keypoint limiting for it, as for `PyDetector`.
* **OnnxMatcher** ([OnnxMatcher.h](OnnxMatcher.h)) is used by `RegistrationVis` when `Vis/CorNNType=10`, in the same branch as `PyMatcher` (6) and GMS (7): the descriptors must be float and the image size known, otherwise brute force matching is used. With a motion guess, `RegistrationVis` matches by projection in a window around the predicted positions instead (`Vis/CorGuessWinSize`, 40 by default): set it to 0 to use the matcher. When the window is kept (> 0), disable `Reg/RepeatOnce` too: it repeats a registration done without a guess (loop closures, proximity detections with the default `RGBD/ProximityOdomGuess=false`) with its result as guess, replacing the matcher's correspondences by the window ones. With a window of 0, `RegistrationVis` can't use a guess (`canUseGuess()`) and doesn't repeat. In odometry, only Frame-to-Frame (`Odom/Strategy=1`) uses it, Frame-to-Map matches against its local map. `match()` normalizes the keypoints with the image size, copies the descriptors, runs the session and turns `matches0`/`mscores0` into `cv::DMatch` (query = the "to" signature, train = the "from" one, as for the other matchers). Multi-camera signatures are supported (as with `PyMatcher`): their keypoints are in the side-by-side image, which is what the matcher gets as image size, so all the cameras are matched in one inference; keep in mind that LightGlue was trained on single images.
* The model interfaces below are the only coupling between RTAB-Map and a network: a new or re-trained detector/matcher exported with them only needs its path in `OnnxDetector/ModelPath` or `OnnxMatcher/ModelPath`.

## ONNX, ONNX Runtime and other runtimes

ONNX is only the model file format (graph and weights, vendor neutral); a runtime is needed to execute it. RTAB-Map uses ONNX Runtime, which delegates the execution to an *execution provider* selected with `Onnx/Provider`:

```
PyTorch --export--> model.onnx --> ONNX Runtime --> CPU      (default, any platform)
                                                 --> CUDA     (NVIDIA)
                                                 --> TensorRT (NVIDIA, compiled engine)
                                                 --> OpenVINO (Intel CPU, GPU, NPU)
                                                 --> CoreML   (Apple)   not supported yet
```

The same `.onnx` file works with every provider; only the speed changes (and, with `Onnx/TensorRTFp16`, the precision). Operations a provider doesn't support are run on the CPU by ONNX Runtime automatically.

* **TensorRT** doesn't run the ONNX graph as is: it compiles it into an *engine* optimized for the GPU it runs on (fused layers, kernels benchmarked per layer), which is why the first start takes seconds to minutes (a warning is logged) and why the engines are cached in `Onnx/TensorRTCachePath`. An engine is built for a range of input shapes: the matchers' engines are built at load for 1 to `Onnx/TensorRTMaxKeypoints` keypoints per image (above, only the keypoints with the highest responses are matched). With 0, TensorRT builds it for the keypoint counts seen so far, and again (seconds, freezing the odometry) each time an image has more or fewer keypoints than before. With `Onnx/TensorRTFp16`, the engine may compute in 16-bit floats instead of 32-bit: faster on GPUs with Tensor Cores and half the memory, at the cost of slightly different scores (keypoints near the threshold) and descriptors, and some models don't survive it (SuperGlue's Sinkhorn iterations overflow, see [Benchmarking](#benchmarking-the-implementations)). Compare with and without it on your data (`test_superpoint_perf`, `test_matcher_perf`, or the inlier ratio of `rtabmap-matcher`) before enabling it. The FP32 and FP16 engines are cached side by side (the file names carry `_fp16`).
* **OpenVINO** (`Onnx/Provider=3`) is Intel's inference engine (Intel CPUs, integrated and Arc GPUs, NPUs), the Intel counterpart of TensorRT. The device is selected with `Onnx/OpenVINODevice` (`CPU`, `GPU`, `GPU.1`, `NPU`, or `AUTO:GPU,CPU` to let OpenVINO choose). Like TensorRT, GPU and NPU compile the model when it is loaded (seconds), so the compiled models are cached in `Onnx/OpenVINOCachePath`. GPU and NPU compute in FP16 by default: set `Onnx/OpenVINOPrecision=1` (FP32) to get the same results as on CPU, and compare with and without it on your data as for `Onnx/TensorRTFp16`. The NPU needs fixed input sizes (`--height`/`--width` and, for the detectors that have it, `--max-keypoints` in the export scripts), GPU works best with them. Even on CPU, OpenVINO is often faster than the default CPU provider on Intel processors. OpenVINO is not in the ONNX Runtime release archives: it is loaded at runtime from its plugin library (`Onnx/OpenVINOLibraryPath`, see [Building](#building)), or used built in an ONNX Runtime built with `--use_openvino` (deprecated by ONNX Runtime in favor of the plugin).
* **iOS**: ONNX Runtime is distributed for iOS as `onnxruntime.xcframework` (CocoaPods `onnxruntime-c`, or the onnxruntime-swift-package-manager releases) with the same C++ API, so `OnnxDetector`/`OnnxMatcher` should work on the CPU provider without code changes, the `.onnx` file shipped as an app bundle resource and `OnnxDetector/ModelPath` set to its path in the bundle. `FindONNXRuntime` would need to handle the framework layout. The CoreML provider (Neural Engine/GPU) would be one more `Onnx/Provider` value (`AppendExecutionProvider("CoreML", {{"ModelFormat", "MLProgram"}})`); it works best with fixed input sizes (`--height`/`--width` and, for the detectors that have it, `--max-keypoints` in the export scripts), and the ops CoreML doesn't support (e.g. `NonZero` of the threshold) fall back to the CPU. Not tested yet.

## Building

```bash
cmake -DWITH_ONNXRUNTIME=ON ..
```

ONNX Runtime is found either installed in a standard prefix, or in an extracted [release archive](https://github.com/microsoft/onnxruntime/releases) given with `-DONNXRUNTIME_ROOT=/path/to/onnxruntime-linux-x64-1.23.2` (`onnxruntime-linux-x64-gpu-<version>` for CUDA/TensorRT support, which needs the matching CUDA and cuDNN versions, see the [CUDA provider requirements](https://onnxruntime.ai/docs/execution-providers/CUDA-ExecutionProvider.html#requirements)). The CMake summary shows `With ONNX Runtime = YES` and `rtabmap --version` shows `With ONNX Runtime: true`. The library is loaded at runtime from the same location: keep the archive where it was, or add its `lib/` directory to `LD_LIBRARY_PATH`.

### OpenVINO

OpenVINO is an execution provider *plugin* ([onnxruntime-ep-openvino](https://github.com/onnxruntime/onnxruntime-ep-openvino)): a library loaded at runtime by the official ONNX Runtime releases (>= 1.22 for the plugin API; the plugin itself documents the versions it supports), so the CUDA/TensorRT archive above works with OpenVINO too, no ONNX Runtime build needed. The easiest way to get the plugin is its Python wheel, which bundles the OpenVINO and TBB libraries it needs (all found next to the plugin):

```bash
pip install onnxruntime-ep-openvino
SRC=$(python3 -c "import onnxruntime_ep_openvino, os; print(os.path.dirname(onnxruntime_ep_openvino.__file__))")
sudo mkdir -p /opt/onnxruntime-ep-openvino
sudo cp -a $SRC/*.so* $SRC/cache.json $SRC/third-party-programs.txt /opt/onnxruntime-ep-openvino/
```

then set `Onnx/OpenVINOLibraryPath` to `/opt/onnxruntime-ep-openvino/libonnxruntime_providers_openvino_plugin.so` (empty: the library is searched by its file name, e.g. in `LD_LIBRARY_PATH`). Don't add that folder to the system library path (`ld.so.conf`): it has its own copies of TBB and hwloc, which would replace the system ones for every program. The plugin is loaded once per process; the log tells the devices it found (`Loaded execution provider plugin ...: OpenVINOExecutionProvider on CPU (Intel), ...`), and a device not found falls back to CPU with a warning listing them.

The plugin ignores the provider options of the built-in provider (`device_type`, `precision`, `cache_dir`): `OnnxSession` selects the device among the plugin's devices and passes the precision and cache directory as OpenVINO properties (`load_config`). An ONNX Runtime with OpenVINO built in (`./build.sh ... --use_openvino CPU`) is used as before when found, the plugin is then not loaded.

## Parameters

| Parameter | Default | Description |
|---|---|---|
| `OnnxDetector/ModelPath` | | [Required] Path to the ONNX model of the detector. |
| `OnnxDetector/Threshold` | 0 | Keep only keypoints with a score above this threshold (0: only the model's own threshold applies). |
| `OnnxMatcher/ModelPath` | | [Required] Path to the ONNX model of the matcher. |
| `OnnxMatcher/Threshold` | 0.1 | Minimum score of a match (when the model outputs scores). |
| `OnnxDescriptor/ModelPath` | | [Required] Path to the ONNX model of the global descriptor (`Mem/GlobalDescriptorStrategy=2`). |
| `OnnxDescriptor/Dim` | 0 | Keep the first values of the descriptor, re-normalized (0: all). |
| `Onnx/Provider` | 1 | Execution provider of all three: 0=CPU, 1=CUDA, 2=TensorRT, 3=OpenVINO. A provider not available in the ONNX Runtime library, or failing to load (missing cuDNN, no device...), falls back to CPU with a warning. |
| `Onnx/TensorRTFp16` | false | Build the TensorRT engines in FP16. |
| `Onnx/TensorRTMaxKeypoints` | 4096 | Largest number of keypoints per image the TensorRT engine of `OnnxMatcher` is built for, at load, so that it is built once. Above, only the keypoints with the highest responses are matched. 0: no limit, the engine is built again each time an image has more (or fewer) keypoints than all the previous ones. |
| `Onnx/TensorRTCachePath` | | Directory caching the TensorRT engines (building one takes seconds to minutes; a warning is logged when it happens). Empty: the `tensorrt_cache` folder of the RTAB-Map working directory. |
| `Onnx/OpenVINODevice` | CPU | OpenVINO device: `CPU`, `GPU` (`GPU.0`, `GPU.1`... with many), `NPU`, or `AUTO` / `AUTO:GPU,CPU`. |
| `Onnx/OpenVINOPrecision` | 0 | 0=device default (FP32 on CPU, FP16 on GPU and NPU), 1=FP32, 2=FP16, 3=ACCURACY (the model's own precision). |
| `Onnx/OpenVINOCachePath` | | Directory caching the models compiled by OpenVINO. Empty: the `openvino_cache` folder of the RTAB-Map working directory. |
| `Onnx/OpenVINOLibraryPath` | | The OpenVINO plugin library (`libonnxruntime_providers_openvino_plugin.so` of onnxruntime-ep-openvino), when OpenVINO is not built in ONNX Runtime. Empty: searched by its file name. |

The usual feature parameters apply: `Kp/MaxFeatures` / `Vis/MaxFeatures` (the strongest keypoints are kept, or `Kp/SSC` for a uniform distribution), the ROI, grid and depth filtering ones. The matcher works with any detector giving float descriptors, but the model must have been trained with them (e.g. SuperPoint-LightGlue with SuperPoint descriptors, 256-d).

Example with the models from the [LightGlue-ONNX](https://github.com/fabio-sim/LightGlue-ONNX) releases (`superpoint.onnx` of v1.0.0 and `superpoint_lightglue.onnx` of v0.1.3, magicleap weights: non-commercial use; `disk.onnx` and `disk_lightglue.onnx` are the Apache-2.0 pair; the unit tests use both pairs):

```bash
rtabmap-matcher --Vis/FeatureType 17 --OnnxDetector/ModelPath superpoint.onnx \
                --Vis/CorNNType 10 --OnnxMatcher/ModelPath superpoint_lightglue.onnx from.png to.png

rtabmap --Kp/DetectorStrategy 17 --Vis/FeatureType 17 --OnnxDetector/ModelPath superpoint.onnx \
        --Vis/CorNNType 10 --OnnxMatcher/ModelPath superpoint_lightglue.onnx --Vis/CorGuessWinSize 0 --Odom/Strategy 1
```

The SuperPoint-LightGlue end-to-end "pipeline" models of LightGlue-ONNX (two images in, matches out) cannot be used: RTAB-Map extracts the features of each image once and matches them later against many images.

## Detector interface

The interface is the one of the extractors exported by LightGlue-ONNX, and of [export_superpoint_onnx.py](export_superpoint_onnx.py) below.

**Input** (one tensor), the image:

* `float32 [1,1,H,W]`: grayscale, intensities scaled to [0,1];
* or `uint8 [1,1,H,W]`: grayscale, raw intensities;
* `[1,3,H,W]` (the grayscale image is replicated on the 3 channels) and `[1,H,W,1]` layouts are accepted too.

RTAB-Map always extracts features from grayscale images (`Feature2D` takes a `CV_8UC1` image and its callers convert the color frame first), so a network trained on RGB (DISK, ALIKED, XFeat...) gets the gray image on its 3 channels. This works in practice (it is how ALIKED was exported and tested in [#1796](https://github.com/introlab/rtabmap/pull/1796)), but passing color would need a color path in `Feature2D` and its callers, with a BGR to RGB swap in `prepareInputs()`.

`H` and `W` may be dynamic (preferred: the image is processed at its resolution, the one of each grid cell with `Kp/GridRows`/`Kp/GridCols`) or fixed (the image is resized to the model's size and the keypoints are scaled back).

**Outputs**, found by name, or by shape when the names differ (the leading batch dimension is optional):

* `keypoints` `[1,N,2]` (`float32` or `int64`): x,y pixel coordinates, or `[1,N,4]` with the keypoint size (pixels) and orientation (radians) too;
* `scores` `[1,N]` `float32` (optional): keypoint responses;
* `descriptors` `[1,N,D]` `float32`: one descriptor per keypoint.

Non-maximum suppression and the detection threshold are expected to be in the model, where they run fast (on the GPU with CUDA/TensorRT). `OnnxDetector/Threshold` is a second filter on `scores`, then the keypoints are limited to `Kp/MaxFeatures` by response (or SSC). Like `PyDetector`, the descriptors are computed with the keypoints: `generateDescriptors()` must be called with the keypoints returned by `generateKeypoints()` on the same image (which is what RTAB-Map does).

## Matcher interface

The interface is the one of the matchers exported by LightGlue-ONNX, and of [export_lightglue_onnx.py](export_lightglue_onnx.py) below.

**Inputs**, found by name, or by position in this order when the names differ:

* `kpts0` `float32 [1,M,2]` and `kpts1` `float32 [1,N,2]`: the keypoints of the query and train images, normalized like LightGlue does, i.e. for an image of size (w,h): `(x - w/2) / (max(w,h)/2)` and `(y - h/2) / (max(w,h)/2)`. RTAB-Map does the normalization from the pixel coordinates with the image size (the side-by-side image size with multiple cameras), the model must not do it again. A model trained with the keypoint scale and orientation (LightGlue for SIFT or DoG-HardNet) declares them `[1,M,4]`: the two extra columns are `cv::KeyPoint::size` (not normalized) and `cv::KeyPoint::angle` in radians, 0 when the detector doesn't compute it.
* `desc0` `float32 [1,M,D]` and `desc1` `float32 [1,N,D]`: their descriptors, as returned by the detector;
* optionally `scores0` `float32 [1,M]` and `scores1` `float32 [1,N]`: the keypoint detection scores (`cv::KeyPoint::response`), for the models that take them (SuperGlue).

**Outputs**, found by name, or by shape when the names differ (the leading batch dimension is optional):

* `matches0` `int64 [1,M]`: for each query keypoint, the index of the matched train keypoint or -1, with optionally `mscores0` `float32 [1,M]`, the confidence of each match;
* or `matches` `int64 [K,2]`: (query, train) index pairs, with optionally `mscores` `float32 [K]`.

Other outputs (e.g. `matches1`, `mscores1`) are ignored. When the scores are there, matches with a score below `OnnxMatcher/Threshold` are dropped and `cv::DMatch::distance` is `1 - score`.

## Global descriptor interface

`OnnxDescriptor` (`Mem/GlobalDescriptorStrategy=2`) extracts a global (whole image) descriptor, like `PyDescriptor` does with a python script. The model takes one image, `float32 [1,3,H,W]` RGB in [0,1] (or `[1,1,H,W]` grayscale; `uint8` and `[1,H,W,C]` accepted; a fixed size gets the image resized), and returns the descriptor as its first output, `float32` with `D` values (`[1,D]`). `OnnxDescriptor/Dim` keeps the first values and re-normalizes them (0: the whole descriptor), like `PyDescriptor/Dim`. See [export_netvlad_onnx.py](export_netvlad_onnx.py) below.

## Getting models

Pre-exported models following these interfaces (SuperPoint, DISK, ALIKED and LightGlue, Apache-2.0 code but check the license of each network's weights, e.g. SuperPoint's, from magicleap, are for non-commercial use) are in the [LightGlue-ONNX releases](https://github.com/fabio-sim/LightGlue-ONNX/releases): use the extractor files (`superpoint.onnx`, `disk.onnx`, ...) and the matcher files (`superpoint_lightglue.onnx`, `disk_lightglue.onnx`, ... of the releases before v2.0, later releases only have the end-to-end pipelines), not the `end2end` / `pipeline` ones.

### Export scripts

The scripts here are RTAB-Map's own (not copies of the LightGlue-ONNX ones, which export fused image-pair pipelines and keep their own copies of the networks). Each one loads the original PyTorch implementation of a network family and wraps it in a small `nn.Module` whose `forward()` has the interface above, then calls `torch.onnx.export()` with the image size (or the number of keypoints of a matcher) declared dynamic, the number of keypoints of a detector being inferred, checks the model with `onnx` and, when `onnxruntime` is installed, runs it to compare its outputs with the PyTorch ones. There is one script per network, not per detector/matcher combination:

* the detector script wraps the network's dense outputs (score map, descriptor map) with an export-friendly post-processing (softmax, depth-to-space, NMS by max-pooling, border mask, threshold with `torch.nonzero`, optional top-k, bilinear descriptor sampling) since the original post-processing uses Python loops and in-place slices that don't export;
* the matcher script takes the feature type as argument: LightGlue is one architecture with a checkpoint per descriptor type (`superpoint_lightglue`, `disk_lightglue`, `aliked_lightglue`, ...), so `aliked_lightglue.onnx` is the same script with `--features aliked`. The matcher only has to match the descriptors it was trained with, it doesn't care which detector implementation produced them.

A detector not covered here needs its own script on the same pattern: load the network, write a `forward(image)` returning `keypoints [1,N,2]`, `scores [1,N]`, `descriptors [1,N,D]`, export with the same `input_names`/`output_names`/`dynamic_shapes`. A model exported with another interface can also be used by subclassing `OnnxDetector` (see [Specializing](#specializing)) instead of changing the export.

The scripts need `torch` >= 2.9, `onnx` and `onnxscript` (`onnxruntime` to verify the exported model); they use the `torch.export`-based exporter (`dynamo=True`, opset 18) and were tested with torch 2.15. Where the exported graph would be slow or computed wrong by a runtime (`einsum`, exported as `Einsum` nodes slower than matrix products, in-place writes exported as `ScatterND`, LightGlue's interleaved q/k/v projection with TensorRT), the scripts substitute an equivalent implementation for the export only, and the verification step compares the exported model with the original code.

### SuperPoint

[export_superpoint_onnx.py](export_superpoint_onnx.py) exports the SuperPoint of [rpautrat/SuperPoint](https://github.com/rpautrat/SuperPoint) (MIT license, `superpoint_pytorch.py` + `weights/superpoint_v6_from_tf.pth`) or of [magicleap/SuperPointPretrainedNetwork](https://github.com/magicleap/SuperPointPretrainedNetwork) (non-commercial license, `demo_superpoint.py` + `superpoint_v1.pth`), with the NMS, detection threshold, border removal and optional top-k inside the model.

**Which SuperPoint goes with which LightGlue?** A LightGlue checkpoint is trained on the descriptors of one specific network, and the two SuperPoint weights above are different trainings whose descriptors don't live in the same space. cvg/LightGlue's `superpoint` checkpoint (the one in `superpoint_lightglue.onnx` from LightGlue-ONNX, and the one [export_lightglue_onnx.py](export_lightglue_onnx.py) exports) was trained with the **magicleap** weights, which is also what LightGlue-ONNX's `superpoint.onnx` was exported from. So:

* magicleap SuperPoint (release file or `export_superpoint_onnx.py` on `superpoint_v1.pth`) + `superpoint_lightglue.onnx`: consistent;
* rpautrat SuperPoint (MIT) + `superpoint_lightglue.onnx`: mismatched, expect degraded matching. Use it with the nearest neighbor matchers (`Vis/CorNNType` 0-5) or train a LightGlue on it with [glue-factory](https://github.com/cvg/glue-factory) and export that.

To check a detector/matcher pair, match an image with a shifted copy of itself: `rtabmap-matcher` prints the inliers/matches ratio, which should be close to 1 for a consistent pair and collapse for a mismatched one (this is what `test_onnx` does with `OnnxMatcher.TranslatedImageMatches`).

Exports:

```bash
git clone https://github.com/rpautrat/SuperPoint.git
python3 export_superpoint_onnx.py --model-dir SuperPoint --weights SuperPoint/weights/superpoint_v6_from_tf.pth --output superpoint_rpautrat.onnx

git clone https://github.com/magicleap/SuperPointPretrainedNetwork.git
python3 export_superpoint_onnx.py --model-dir SuperPointPretrainedNetwork --weights SuperPointPretrainedNetwork/superpoint_v1.pth --output superpoint_magicleap.onnx
```

Options: `--nms-radius 4`, `--threshold 0.005`, `--remove-borders 4`, `--max-keypoints 0` (0: all), `--height`/`--width` (fixed input size, for TensorRT or other runtimes needing static shapes).

### DISK

[export_disk_onnx.py](export_disk_onnx.py) exports [DISK](https://github.com/cvlab-epfl/disk) (Apache-2.0, 128-d descriptors) from kornia's implementation and pretrained weights, the ones cvg/LightGlue uses, so the result pairs with `disk_lightglue.onnx` (LightGlue-ONNX release or `export_lightglue_onnx.py --features disk`). The model takes an RGB image (RTAB-Map replicates its grayscale image on the 3 channels) and has the padding to a multiple of 16, the NMS, the threshold and the optional top-k inside:

```bash
pip install kornia
python3 export_disk_onnx.py --output disk.onnx
```

Options: `--weights depth|epipolar|file.pth`, `--window-size 5`, `--threshold 0`, `--max-keypoints 0`, `--height`/`--width`.

With TensorRT, prefer this export to the `disk.onnx` of the LightGlue-ONNX v1.0.0 release: its NMS is a `MaxPool` that also outputs the indices of the maxima, which the TensorRT parser rejects ("doesn't support more than 1 outputs for MaxPool nodes", logged as errors). ONNX Runtime then runs these nodes with CUDA and the rest with TensorRT, so the results are the same, but the log is noisy. This export's NMS has a plain `MaxPool`, so the whole model runs in TensorRT. Both find the same keypoints at the same speed (about 16 ms on a 640x480 image, RTX 3080). The unit tests use both: `scripts/fetch_test_data.sh` downloads the release one and, when python3 has kornia, exports this one as `disk_rtabmap.onnx` (`test_onnx` checks they find the same features, and its TensorRT test, `RTABMAP_TEST_TENSORRT=1`, uses this one).

### XFeat

[export_xfeat_onnx.py](export_xfeat_onnx.py) exports [XFeat](https://github.com/verlab/accelerated_features) (Apache-2.0, 64-d descriptors, designed for CPU speed) from a clone of its repository, which holds the weights. It reproduces XFeat's sparse `detectAndCompute()` (NMS, heatmap times reliability scores, top-k, descriptors interpolated from the 1/8 feature map) with a grayscale input, which XFeat uses internally anyway; the one difference is that an image whose size is not a multiple of 32 is zero-padded rather than resized down, so the keypoints stay in the image's coordinates. XFeat has no LightGlue checkpoint (it has its own LighterGlue, not supported here): match its descriptors with the nearest neighbor matchers (`Vis/CorNNType` 0-5):

```bash
git clone https://github.com/verlab/accelerated_features.git
python3 export_xfeat_onnx.py --model-dir accelerated_features --output xfeat.onnx
```

Options: `--threshold 0.05` (XFeat's default), `--max-keypoints 0` (all, as the other scripts: `OnnxDetector` keeps the best ones with `Kp/MaxFeatures`; XFeat's 4096 sorts all the candidates in the model, a TopK that TensorRT refuses above 3840 candidates, so the inference fails with TensorRT on textured images), `--interpolation bicubic|bilinear` (descriptor sampling; bicubic is XFeat's, bilinear is faster on GPU providers), `--height`/`--width`.

### ALIKED

[export_aliked_onnx.py](export_aliked_onnx.py) (from [#1796](https://github.com/introlab/rtabmap/pull/1796)) exports [ALIKED](https://github.com/Shiaoming/ALIKED) (BSD-3-Clause, 128-d descriptors, 64-d for `aliked-t16`) from the implementation of the `lightglue` package (installed, or a clone given with `--lightglue-dir`; it downloads the weights), so the result pairs with `aliked_lightglue.onnx` (`export_lightglue_onnx.py --features aliked`). The padding to a multiple of 32, the NMS, the threshold and the optional top-k are inside the model, with the same results as `lightglue.ALIKED`: the operations that don't export (torchvision's deformable convolution, the Python loops of the keypoint detection and of the descriptor head) are rewritten with standard ONNX operators, and the verification compares the exported model with the original code.

```bash
pip install lightglue onnx onnxscript onnxruntime
python3 export_aliked_onnx.py --output aliked.onnx
python3 export_lightglue_onnx.py --features aliked --output aliked_lightglue.onnx
rtabmap --Kp/DetectorStrategy 17 --Vis/FeatureType 17 --OnnxDetector/ModelPath aliked.onnx \
        --Vis/CorNNType 10 --OnnxMatcher/ModelPath aliked_lightglue.onnx --Vis/CorGuessWinSize 0 --Odom/Strategy 1
```

Options: `--model aliked-n16|aliked-n16rot|aliked-n32|aliked-t16`, `--threshold 0.2` (ALIKED's, few keypoints on low-texture images, lower it for more), `--max-keypoints 0`, `--height`/`--width`. `--threshold 0` with `--max-keypoints K` keeps the K strongest local maxima instead (ALIKED's top-k mode): with `--height`/`--width` too, every shape of the model is static, the number of keypoints included.

### LightGlue

[export_lightglue_onnx.py](export_lightglue_onnx.py) exports the LightGlue of [cvg/LightGlue](https://github.com/cvg/LightGlue) (Apache-2.0) for a given feature type (`superpoint`, `disk`, `aliked`, `sift`, `doghardnet`, their weights are downloaded by LightGlue itself) or for your own checkpoint (`--weights model.pth --input-dim D`, e.g. trained with [glue-factory](https://github.com/cvg/glue-factory)), with the adaptive depth and width (early stopping and point pruning, which a static graph cannot express) disabled. The `sift` and `doghardnet` ones take the keypoint scale and orientation (`kpts` with 4 columns), which RTAB-Map fills from the OpenCV keypoints, e.g. `sift_lightglue.onnx` with `Vis/FeatureType=1`:

```bash
pip install git+https://github.com/cvg/LightGlue.git  # or: git clone it and use --lightglue-dir
python3 export_lightglue_onnx.py --features superpoint --output superpoint_lightglue.onnx
```

Options: `--threshold 0.1` (match threshold inside the model, matches below it are returned as -1), `--lightglue-dir` (a clone of the repository, to avoid installing it and its dependencies), `--weights`/`--input-dim`/`--scale-ori` (a custom checkpoint, its descriptor dimension and whether it was trained with the keypoint scale and orientation).

The rows of the self-attention projections (`Wqkv`) are reordered in the exported model so that q, k and v are contiguous blocks instead of LightGlue's interleaved order: TensorRT fuses the projection with its split into q, k and v assuming contiguous blocks, and silently computed wrong matches with the interleaved order (models exported before this fix match correctly on CPU, CUDA and OpenVINO, not with TensorRT: export them again).

### NetVLAD

[export_netvlad_onnx.py](export_netvlad_onnx.py) exports NetVLAD (VGG16 + NetVLAD + PCA whitening, the Pittsburgh 30k weights of the [original release](https://github.com/Relja/netvlad) converted to a MATLAB struct by netvlad_tf_open and hosted by hloc, whose extractor the script follows) to a model of the global descriptor interface, replacing the TensorFlow-based `rtabmap_netvlad.py` of `PyDescriptor`:

```bash
pip install scipy torchvision
python3 export_netvlad_onnx.py --download --weights Pitts30K_struct.mat --output netvlad.onnx --dim 1024
```

`--model TokyoTM` downloads the Tokyo Time Machine weights instead.

The script reads the two `_struct.mat` files hosted by hloc (`Pitts30K_struct.mat`, `TokyoTM_struct.mat`): the `vd16_*_conv5_3_vlad_preL2_intra_white` models of the NetVLAD [project page](https://www.di.ens.fr/willow/research/netvlad/) converted from MATLAB class objects to plain structs with `net_class2struct` of [netvlad_tf_open](https://github.com/uzh-rpg/netvlad_tf_open), which needs MATLAB and MatConvNet. The other models of the page (no whitening, AlexNet backbone, max pooling) are not supported as is. The NetVLAD code is MIT (original release) and Apache-2.0 (hloc); the project page provides the weights "on a non-commercial basis" without an explicit license, check these terms before redistributing a model built from them. `--dim` keeps the leading PCA components (default 4096, the full whitening): the whitening matrix is 32768 x dim floats, on top of the 59 MB VGG16 backbone (596 MB at 4096, 193 MB at 1024, 126 MB at 512); `OnnxDescriptor/Dim` can truncate further at runtime. The weights file is 555 MB. Usage, as with the python version: `--Mem/GlobalDescriptorStrategy 2 --OnnxDescriptor/ModelPath netvlad.onnx --Kp/TfIdfLikelihoodUsed false --Mem/RehearsalSimilarity 1`. With `Kp/TfIdfLikelihoodUsed=false`, the loop closure likelihood is the dot product of the descriptors (`Signature::compareTo()`), rescaled to [0,1]. Note that every node then gets a non-zero likelihood, unlike with the bag-of-words, which keeps the Bayes filter hypotheses low with the default `Rtabmap/VirtualPlaceLikelihoodRatio=0`: see the test `OnnxDescriptor.NetVLADAppearanceOnlyLoopClosure` for the retrieval quality on `data/samples` and the resulting hypothesis values.

### SuperGlue

[export_superglue_onnx.py](export_superglue_onnx.py) exports [SuperGlue](https://github.com/magicleap/SuperGluePretrainedNetwork) (non-commercial license) from a clone of its repository, with the Sinkhorn iterations unrolled in the graph. SuperGlue takes the keypoint scores too (`scores0`/`scores1` inputs) and was trained with the magicleap SuperPoint: pair it with `superpoint_magicleap.onnx` or `SuperPointTorch`.

```bash
git clone https://github.com/magicleap/SuperGluePretrainedNetwork.git
python3 export_superglue_onnx.py --superglue-dir SuperGluePretrainedNetwork --weights indoor --output superglue_indoor.onnx
```

Options: `--weights indoor|outdoor`, `--iterations 20` (Sinkhorn iterations; `PyMatcher/Iterations` default, SuperGlue's own is 100), `--threshold 0.2`, `--opset 18`. Keep the opset >= 18 for OpenVINO: the Sinkhorn iterations are `ReduceLogSumExp` nodes, whose axes the OpenVINO execution provider ignores with opset 17 and lower (it reduces over all the axes, so SuperGlue finds no match), while it reads them correctly from opset 18. A SuperGlue exported before with opset 17 has to be exported again for OpenVINO. This is a bug of OpenVINO's ONNX frontend ([openvinotoolkit/openvino#38238](https://github.com/openvinotoolkit/openvino/issues/38238)), fixed in OpenVINO 2026.5 (the onnxruntime-ep-openvino 1.8.1 wheel bundles 2026.4.1). The matcher benchmark below compares it with `PyMatcher` running the same network in Python and with LightGlue.

## Benchmarking the implementations

Two performance tests (built with `BUILD_PERF_TESTS=ON`, run with `ctest -L performance` or directly from `bin/`) compare the implementations and the execution providers on the images of `data/samples`. The variants the build or the machine can't run are skipped; the assets come from `scripts/fetch_test_data.sh` (torch needed to export some models and to run the python scripts). Example timings are on the wiki page [Learned features with ONNX](https://github.com/introlab/rtabmap/wiki/Learned-Features-with-ONNX#choosing-the-execution-provider).

* `test_matcher_perf` compares, on consecutive frames with SuperPoint (magicleap) features: FLANN kd-tree with the distance ratio test (rtabmap's default, `Vis/CorNNType=1`, with `Vis/CorNNDR` 0.8 and 0.6), brute force matching with cross check, LightGlue and SuperGlue exported to ONNX (`OnnxMatcher`, on each provider) and SuperGlue run by `PyMatcher` (python, CPU/CUDA), with the time per pair, the matches and the inliers of a RANSAC fundamental matrix.
* `test_superpoint_perf` times the implementations of SuperPoint: the python scripts (`PyDetector`), libtorch (`SuperPointRpautrat` for the rpautrat weights, `SuperPointTorch` for the magicleap ones) and ONNX Runtime (`OnnxDetector`, on each provider), with the model load and first frame times, the time per frame, and whether they find the same keypoints and descriptors (cosine similarity). The magicleap python script needs a `cv2` importable by the embedded interpreter (`pip install opencv-python-headless` in the environment the build uses).

FP16 (`Onnx/TensorRTFp16`) has to be validated per model with these tests: it changes a few keypoints near the detection threshold of SuperPoint and keeps LightGlue's matches, but SuperGlue breaks (about 10% of its FP32 matches): its Sinkhorn iterations overflow in half precision, as TensorRT warns at build time about the layer normalizations after the self-attention.

**Don't mix the magicleap python script with the other magicleap implementations in a map.** The network and the keypoints are the same, but magicleap's `demo_superpoint.py` (used by `rtabmap_superpoint.py`) reads the descriptors from the network's 1/8 descriptor map with its own convention, while LightGlue's SuperPoint, which `superpoint.onnx` (`OnnxDetector`) and `SuperPointTorch` follow, reads them at the centers of the 8x8 cells: the descriptors of a same keypoint differ (cosine 0.975). On the 83 consecutive frame pairs of `data/samples` (brute force matching, ratio test, RANSAC), a map made with the python script and localized with `OnnxDetector` gets 35% fewer inliers than `OnnxDetector` on both sides, enough to drop loop closures near `Vis/MinInliers`. The python script on both sides gets 12% fewer, its sampling being a bit blurrier. The rpautrat implementations all give the same descriptors.

For backward compatibility with the maps already made with it, `rtabmap_superpoint.py` keeps the demo's sampling by default. Set `DESCRIPTOR_SAMPLING = 'lightglue'` at the top of the script (or the `RTABMAP_SUPERPOINT_SAMPLING=lightglue` environment variable) to get the descriptors of the other implementations (cosine 1.000), e.g. for a new map that may later be used with `OnnxDetector`, or with SuperPoint-LightGlue, which was trained with them. A map made with the other sampling has to be reprocessed (`rtabmap-reprocess`) to switch.

## Specializing

A model with another interface can be supported by subclassing `OnnxDetector` or `OnnxMatcher` and overriding `prepareInputs()` (model inputs from the image or features) and/or `parseOutputs()` (keypoints/descriptors or matches from the model outputs). The tensors are exchanged as `OnnxTensor` (name, shape, type and data), the subclass doesn't need the ONNX Runtime headers. The session (`OnnxSession`) is shared between the objects using the same model and options, so the many `Feature2D`/`RegistrationVis` instances RTAB-Map creates load the model once. A specialized detector also needs a new `Feature2D::Type` value, see how `kFeatureOnnxDetector` is handled in `Features2d.cpp`.
