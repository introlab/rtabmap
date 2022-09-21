/*
Copyright (c) 2010-2016, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
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

#ifndef PARAMETERS_H_
#define PARAMETERS_H_

// default parameters
#include "rtabmap/core/RtabmapExp.h" // DLL export/import defines
#include "rtabmap/core/Version.h" // DLL export/import defines
#include <rtabmap/utilite/UConversion.h>
#include <opencv2/core/version.hpp>
#include <opencv2/opencv_modules.hpp>
#include <string>
#include <map>

namespace rtabmap
{

typedef std::map<std::string, std::string> ParametersMap; // Key, value
typedef std::pair<std::string, std::string> ParametersPair;

/**
 * Macro used to create parameter's key and default value.
 * This macro must be used only in the Parameters class definition (in this file).
 * They are automatically added to the default parameters map of the class Parameters.
 * Example:
 * @code
 *         //for PARAM(Video, ImageWidth, int, 640), the output will be :
 *         public:
 *             static std::string kVideoImageWidth() {return std::string("Video/ImageWidth");}
 *             static int defaultVideoImageWidth() {return 640;}
 *         private:
 *             class DummyVideoImageWidth {
 *             public:
 *                 DummyVideoImageWidth() {parameters_.insert(ParametersPair("Video/ImageWidth", "640"));}
 *             };
 *             DummyVideoImageWidth dummyVideoImageWidth;
 * @endcode
 */
#define RTABMAP_PARAM(PREFIX, NAME, TYPE, DEFAULT_VALUE, DESCRIPTION) \
    public: \
        static std::string k##PREFIX##NAME() {return std::string(#PREFIX "/" #NAME);} \
        static TYPE default##PREFIX##NAME() {return (TYPE)DEFAULT_VALUE;} \
        static std::string type##PREFIX##NAME() {return std::string(#TYPE);} \
    private: \
        class Dummy##PREFIX##NAME { \
        public: \
            Dummy##PREFIX##NAME() {parameters_.insert(ParametersPair(#PREFIX "/" #NAME, #DEFAULT_VALUE)); \
                                   parametersType_.insert(ParametersPair(#PREFIX "/" #NAME, #TYPE)); \
                                   descriptions_.insert(ParametersPair(#PREFIX "/" #NAME, DESCRIPTION));} \
        }; \
        Dummy##PREFIX##NAME dummy##PREFIX##NAME
// end define PARAM

/**
 * It's the same as the macro PARAM but it should be used for string parameters.
 * Macro used to create parameter's key and default value.
 * This macro must be used only in the Parameters class definition (in this file).
 * They are automatically added to the default parameters map of the class Parameters.
 * Example:
 * @code
 *         //for PARAM_STR(Video, TextFileName, "Hello_world"), the output will be :
 *         public:
 *             static std::string kVideoFileName() {return std::string("Video/FileName");}
 *             static std::string defaultVideoFileName() {return "Hello_world";}
 *         private:
 *             class DummyVideoFileName {
 *             public:
 *                 DummyVideoFileName() {parameters_.insert(ParametersPair("Video/FileName", "Hello_world"));}
 *             };
 *             DummyVideoFileName dummyVideoFileName;
 * @endcode
 */
#define RTABMAP_PARAM_STR(PREFIX, NAME, DEFAULT_VALUE, DESCRIPTION) \
    public: \
        static std::string k##PREFIX##NAME() {return std::string(#PREFIX "/" #NAME);} \
        static std::string default##PREFIX##NAME() {return DEFAULT_VALUE;} \
        static std::string type##PREFIX##NAME() {return std::string("string");} \
    private: \
        class Dummy##PREFIX##NAME { \
        public: \
            Dummy##PREFIX##NAME() {parameters_.insert(ParametersPair(#PREFIX "/" #NAME, DEFAULT_VALUE)); \
                                   parametersType_.insert(ParametersPair(#PREFIX "/" #NAME, "string")); \
                                   descriptions_.insert(ParametersPair(#PREFIX "/" #NAME, DESCRIPTION));} \
        }; \
        Dummy##PREFIX##NAME dummy##PREFIX##NAME
// end define PARAM

/**
 * Macro used to create parameter's key and default value.
 * This macro must be used only in the Parameters class definition (in this file).
 * They are automatically added to the default parameters map of the class Parameters.
 * Example:
 * @code
 *         //for PARAM(Video, ImageWidth, int, 640), the output will be :
 *         public:
 *             static std::string kVideoImageWidth() {return std::string("Video/ImageWidth");}
 *             static int defaultVideoImageWidth() {return 640;}
 *         private:
 *             class DummyVideoImageWidth {
 *             public:
 *                 DummyVideoImageWidth() {parameters_.insert(ParametersPair("Video/ImageWidth", "640"));}
 *             };
 *             DummyVideoImageWidth dummyVideoImageWidth;
 * @endcode
 */
#define RTABMAP_PARAM_COND(PREFIX, NAME, TYPE, COND, DEFAULT_VALUE1, DEFAULT_VALUE2, DESCRIPTION) \
    public: \
        static std::string k##PREFIX##NAME() {return std::string(#PREFIX "/" #NAME);} \
        static TYPE default##PREFIX##NAME() {return COND?DEFAULT_VALUE1:DEFAULT_VALUE2;} \
        static std::string type##PREFIX##NAME() {return std::string(#TYPE);} \
    private: \
        class Dummy##PREFIX##NAME { \
        public: \
            Dummy##PREFIX##NAME() {parameters_.insert(ParametersPair(#PREFIX "/" #NAME, COND?#DEFAULT_VALUE1:#DEFAULT_VALUE2)); \
                                   parametersType_.insert(ParametersPair(#PREFIX "/" #NAME, #TYPE)); \
                                   descriptions_.insert(ParametersPair(#PREFIX "/" #NAME, DESCRIPTION));} \
        }; \
        Dummy##PREFIX##NAME dummy##PREFIX##NAME
// end define PARAM

/**
 * Class Parameters.
 * This class is used to manage all custom parameters
 * we want in the application. It was designed to be very easy to add
 * a new parameter (just by adding one line of code).
 * The macro PARAM(PREFIX, NAME, TYPE, DEFAULT_VALUE) is
 * used to create a parameter in this class. A parameter can be accessed after by
 * Parameters::defaultPARAMETERNAME() for the default value, Parameters::kPARAMETERNAME for his key (parameter name).
 * The class provides also a general map containing all the parameter's key and
 * default value. This map can be accessed anywhere in the application by
 * Parameters::getDefaultParameters();
 * Example:
 * @code
 *         //Defining a parameter in this class with the macro PARAM:
 *         PARAM(Video, ImageWidth, int, 640);
 *
 *         // Now from anywhere in the application (Parameters is a singleton)
 *         int width = Parameters::defaultVideoImageWidth(); // theDefaultValue = 640
 *         std::string theKey = Parameters::kVideoImageWidth(); // theKey = "Video/ImageWidth"
 *         std::string strValue = Util::value(Parameters::getDefaultParameters(), theKey); // strValue = "640"
 * @endcode
 * @see getDefaultParameters()
 * TODO Add a detailed example with simple classes
 */
class RTABMAP_EXP Parameters
{
    // Rtabmap parameters
    RTABMAP_PARAM(Rtabmap, PublishStats,                 bool, true,  "Publishing statistics.");
    RTABMAP_PARAM(Rtabmap, PublishLastSignature,         bool, true,  "Publishing last signature.");
    RTABMAP_PARAM(Rtabmap, PublishPdf,                   bool, true,  "Publishing pdf.");
    RTABMAP_PARAM(Rtabmap, PublishLikelihood,            bool, true,  "Publishing likelihood.");
    RTABMAP_PARAM(Rtabmap, PublishRAMUsage,              bool, false, "Publishing RAM usage in statistics (may add a small overhead to get info from the system).");
    RTABMAP_PARAM(Rtabmap, ComputeRMSE,                  bool, true,  "Compute root mean square error (RMSE) and publish it in statistics, if ground truth is provided.");
    RTABMAP_PARAM(Rtabmap, SaveWMState,                  bool, false, "Save working memory state after each update in statistics.");
    RTABMAP_PARAM(Rtabmap, TimeThr,                      float, 0,    "Maximum time allowed for map update (ms) (0 means infinity). When map update time exceeds this fixed time threshold, some nodes in Working Memory (WM) are transferred to Long-Term Memory to limit the size of the WM and decrease the update time.");
    RTABMAP_PARAM(Rtabmap, MemoryThr,                    int, 0,      uFormat("Maximum nodes in the Working Memory (0 means infinity). Similar to \"%s\", when the number of nodes in Working Memory (WM) exceeds this treshold, some nodes are transferred to Long-Term Memory to keep WM size fixed.", kRtabmapTimeThr().c_str()));
    RTABMAP_PARAM(Rtabmap, DetectionRate,                float, 1,    "Detection rate (Hz). RTAB-Map will filter input images to satisfy this rate.");
    RTABMAP_PARAM(Rtabmap, ImageBufferSize,          unsigned int, 1, "Data buffer size (0 min inf).");
    RTABMAP_PARAM(Rtabmap, CreateIntermediateNodes,      bool, false, uFormat("Create intermediate nodes between loop closure detection. Only used when %s>0.", kRtabmapDetectionRate().c_str()));
    RTABMAP_PARAM_STR(Rtabmap, WorkingDirectory,         "",          "Working directory.");
    RTABMAP_PARAM(Rtabmap, MaxRetrieved,             unsigned int, 2, "Maximum locations retrieved at the same time from LTM.");
    RTABMAP_PARAM(Rtabmap, StatisticLogsBufferedInRAM,   bool, true,  "Statistic logs buffered in RAM instead of written to hard drive after each iteration.");
    RTABMAP_PARAM(Rtabmap, StatisticLogged,              bool, false, "Logging enabled.");
    RTABMAP_PARAM(Rtabmap, StatisticLoggedHeaders,       bool, true,  "Add column header description to log files.");
    RTABMAP_PARAM(Rtabmap, StartNewMapOnLoopClosure,     bool, false, "Start a new map only if there is a global loop closure with a previous map.");
    RTABMAP_PARAM(Rtabmap, StartNewMapOnGoodSignature,   bool, false, uFormat("Start a new map only if the first signature is not bad (i.e., has enough features, see %s).", kKpBadSignRatio().c_str()));
    RTABMAP_PARAM(Rtabmap, ImagesAlreadyRectified,       bool, true,  "Images are already rectified. By default RTAB-Map assumes that received images are rectified. If they are not, they can be rectified by RTAB-Map if this parameter is false.");
    RTABMAP_PARAM(Rtabmap, RectifyOnlyFeatures,          bool, false,  uFormat("If \"%s\" is false and this parameter is true, the whole RGB image will not be rectified, only the features. Warning: As projection of RGB-D image to point cloud is assuming that images are rectified, the generated point cloud map will have wrong colors if this parameter is true.", kRtabmapImagesAlreadyRectified().c_str()));

    // Hypotheses selection
    RTABMAP_PARAM(Rtabmap, LoopThr,           float, 0.11,      "Loop closing threshold.");
    RTABMAP_PARAM(Rtabmap, LoopRatio,         float, 0,         "The loop closure hypothesis must be over LoopRatio x lastHypothesisValue.");
    RTABMAP_PARAM(Rtabmap, LoopGPS,           bool,  true,      uFormat("Use GPS to filter likelihood (if GPS is recorded). Only locations inside the local radius \"%s\" of the current GPS location are considered for loop closure detection.", kRGBDLocalRadius().c_str()));

    // Memory
    RTABMAP_PARAM(Mem, RehearsalSimilarity,         float, 0.6,     "Rehearsal similarity.");
    RTABMAP_PARAM(Mem, ImageKept,                   bool, false,    "Keep raw images in RAM.");
    RTABMAP_PARAM(Mem, BinDataKept,                 bool, true,     "Keep binary data in db.");
    RTABMAP_PARAM(Mem, RawDescriptorsKept,          bool, true,     "Raw descriptors kept in memory.");
    RTABMAP_PARAM(Mem, MapLabelsAdded,              bool, true,     "Create map labels. The first node of a map will be labeled as \"map#\" where # is the map ID.");
    RTABMAP_PARAM(Mem, SaveDepth16Format,           bool, false,    "Save depth image into 16 bits format to reduce memory used. Warning: values over ~65 meters are ignored (maximum 65535 millimeters).");
    RTABMAP_PARAM(Mem, NotLinkedNodesKept,          bool, true,     "Keep not linked nodes in db (rehearsed nodes and deleted nodes).");
    RTABMAP_PARAM(Mem, IntermediateNodeDataKept,    bool, false,    "Keep intermediate node data in db.");
    RTABMAP_PARAM_STR(Mem, ImageCompressionFormat,   ".jpg",        "RGB image compression format. It should be \".jpg\" or \".png\".");
    RTABMAP_PARAM(Mem, STMSize,                   unsigned int, 10, "Short-term memory size.");
    RTABMAP_PARAM(Mem, IncrementalMemory,           bool, true,     "SLAM mode, otherwise it is Localization mode.");
    RTABMAP_PARAM(Mem, LocalizationDataSaved,       bool, false,     uFormat("Save localization data during localization session (when %s=false). When enabled, the database will then also grow in localization mode. This mode would be used only for debugging purpose.", kMemIncrementalMemory().c_str()).c_str());
    RTABMAP_PARAM(Mem, ReduceGraph,                 bool, false,    "Reduce graph. Merge nodes when loop closures are added (ignoring those with user data set).");
    RTABMAP_PARAM(Mem, RecentWmRatio,               float, 0.2,     "Ratio of locations after the last loop closure in WM that cannot be transferred.");
    RTABMAP_PARAM(Mem, TransferSortingByWeightId,   bool, false,    "On transfer, signatures are sorted by weight->ID only (i.e. the oldest of the lowest weighted signatures are transferred first). If false, the signatures are sorted by weight->Age->ID (i.e. the oldest inserted in WM of the lowest weighted signatures are transferred first). Note that retrieval updates the age, not the ID.");
    RTABMAP_PARAM(Mem, RehearsalIdUpdatedToNewOne,  bool, false,    "On merge, update to new id. When false, no copy.");
    RTABMAP_PARAM(Mem, RehearsalWeightIgnoredWhileMoving, bool, false, "When the robot is moving, weights are not updated on rehearsal.");
    RTABMAP_PARAM(Mem, GenerateIds,                 bool, true,     "True=Generate location IDs, False=use input image IDs.");
    RTABMAP_PARAM(Mem, BadSignaturesIgnored,        bool, false,    "Bad signatures are ignored.");
    RTABMAP_PARAM(Mem, InitWMWithAllNodes,          bool, false,    "Initialize the Working Memory with all nodes in Long-Term Memory. When false, it is initialized with nodes of the previous session.");
    RTABMAP_PARAM(Mem, DepthAsMask,                 bool, true,     "Use depth image as mask when extracting features for vocabulary.");
    RTABMAP_PARAM(Mem, StereoFromMotion,            bool, false,    uFormat("Triangulate features without depth using stereo from motion (odometry). It would be ignored if %s is true and the feature detector used supports masking.", kMemDepthAsMask().c_str()));
    RTABMAP_PARAM(Mem, ImagePreDecimation,          unsigned int, 1, uFormat("Decimation of the RGB image before visual feature detection. If depth size is larger than decimated RGB size, depth is decimated to be always at most equal to RGB size. If %s is true and if depth is smaller than decimated RGB, depth may be interpolated to match RGB size for feature detection.",kMemDepthAsMask().c_str()));
    RTABMAP_PARAM(Mem, ImagePostDecimation,         unsigned int, 1, uFormat("Decimation of the RGB image before saving it to database. If depth size is larger than decimated RGB size, depth is decimated to be always at most equal to RGB size. Decimation is done from the original image. If set to same value than %s, data already decimated is saved (no need to re-decimate the image).", kMemImagePreDecimation().c_str()));
    RTABMAP_PARAM(Mem, CompressionParallelized,     bool, true,     "Compression of sensor data is multi-threaded.");
    RTABMAP_PARAM(Mem, LaserScanDownsampleStepSize, int, 1,         "If > 1, downsample the laser scans when creating a signature.");
    RTABMAP_PARAM(Mem, LaserScanVoxelSize,          float, 0.0,     uFormat("If > 0 m, voxel filtering is done on laser scans when creating a signature. If the laser scan had normals, they will be removed. To recompute the normals, make sure to use \"%s\" or \"%s\" parameters.", kMemLaserScanNormalK().c_str(), kMemLaserScanNormalRadius().c_str()));
    RTABMAP_PARAM(Mem, LaserScanNormalK,            int, 0,         "If > 0 and laser scans don't have normals, normals will be computed with K search neighbors when creating a signature.");
    RTABMAP_PARAM(Mem, LaserScanNormalRadius,       float, 0.0,     "If > 0 m and laser scans don't have normals, normals will be computed with radius search neighbors when creating a signature.");
    RTABMAP_PARAM(Mem, UseOdomFeatures,             bool, true,     "Use odometry features instead of regenerating them.");
    RTABMAP_PARAM(Mem, UseOdomGravity,              bool, false,    uFormat("Use odometry instead of IMU orientation to add gravity links to new nodes created. We assume that odometry is already aligned with gravity (e.g., we are using a VIO approach). Gravity constraints are used by graph optimization only if \"%s\" is not zero.", kOptimizerGravitySigma().c_str()));
    RTABMAP_PARAM(Mem, CovOffDiagIgnored,           bool, true,     "Ignore off diagonal values of the covariance matrix.");

    // KeypointMemory (Keypoint-based)
    RTABMAP_PARAM(Kp, NNStrategy,               int, 1,       "kNNFlannNaive=0, kNNFlannKdTree=1, kNNFlannLSH=2, kNNBruteForce=3, kNNBruteForceGPU=4");
    RTABMAP_PARAM(Kp, IncrementalDictionary,    bool, true,   "");
    RTABMAP_PARAM(Kp, IncrementalFlann,         bool, true,   uFormat("When using FLANN based strategy, add/remove points to its index without always rebuilding the index (the index is built only when the dictionary increases of the factor \"%s\" in size).", kKpFlannRebalancingFactor().c_str()));
    RTABMAP_PARAM(Kp, FlannRebalancingFactor,   float, 2.0,   uFormat("Factor used when rebuilding the incremental FLANN index (see \"%s\"). Set <=1 to disable.", kKpIncrementalFlann().c_str()));
    RTABMAP_PARAM(Kp, ByteToFloat,              bool, false,  uFormat("For %s=1, binary descriptors are converted to float by converting each byte to float instead of converting each bit to float. When converting bytes instead of bits, less memory is used and search is faster at the cost of slightly less accurate matching.", kKpNNStrategy().c_str()));
    RTABMAP_PARAM(Kp, MaxDepth,                 float, 0,     "Filter extracted keypoints by depth (0=inf).");
    RTABMAP_PARAM(Kp, MinDepth,                 float, 0,     "Filter extracted keypoints by depth.");
    RTABMAP_PARAM(Kp, MaxFeatures,              int, 500,     "Maximum features extracted from the images (0 means not bounded, <0 means no extraction).");
    RTABMAP_PARAM(Kp, BadSignRatio,             float, 0.5,   "Bad signature ratio (less than Ratio x AverageWordsPerImage = bad).");
    RTABMAP_PARAM(Kp, NndrRatio,                float, 0.8,   "NNDR ratio (A matching pair is detected, if its distance is closer than X times the distance of the second nearest neighbor.)");
#if CV_MAJOR_VERSION > 2 && !defined(HAVE_OPENCV_XFEATURES2D)
    // OpenCV>2 without xFeatures2D module doesn't have BRIEF
    RTABMAP_PARAM(Kp, DetectorStrategy,         int, 8,       "0=SURF 1=SIFT 2=ORB 3=FAST/FREAK 4=FAST/BRIEF 5=GFTT/FREAK 6=GFTT/BRIEF 7=BRISK 8=GFTT/ORB 9=KAZE 10=ORB-OCTREE 11=SuperPoint 12=SURF/FREAK 13=GFTT/DAISY 14=SURF/DAISY 15=PyDetector");
#else
    RTABMAP_PARAM(Kp, DetectorStrategy,         int, 6,       "0=SURF 1=SIFT 2=ORB 3=FAST/FREAK 4=FAST/BRIEF 5=GFTT/FREAK 6=GFTT/BRIEF 7=BRISK 8=GFTT/ORB 9=KAZE 10=ORB-OCTREE 11=SuperPoint 12=SURF/FREAK 13=GFTT/DAISY 14=SURF/DAISY 15=PyDetector");
#endif
    RTABMAP_PARAM(Kp, TfIdfLikelihoodUsed,      bool, true,   "Use of the td-idf strategy to compute the likelihood.");
    RTABMAP_PARAM(Kp, Parallelized,             bool, true,   "If the dictionary update and signature creation were parallelized.");
    RTABMAP_PARAM_STR(Kp, RoiRatios,       "0.0 0.0 0.0 0.0", "Region of interest ratios [left, right, top, bottom].");
    RTABMAP_PARAM_STR(Kp, DictionaryPath,       "",           "Path of the pre-computed dictionary");
    RTABMAP_PARAM(Kp, NewWordsComparedTogether, bool, true,   "When adding new words to dictionary, they are compared also with each other (to detect same words in the same signature).");
    RTABMAP_PARAM(Kp, SubPixWinSize,            int, 3,       "See cv::cornerSubPix().");
    RTABMAP_PARAM(Kp, SubPixIterations,         int, 0,       "See cv::cornerSubPix(). 0 disables sub pixel refining.");
    RTABMAP_PARAM(Kp, SubPixEps,                double, 0.02, "See cv::cornerSubPix().");
    RTABMAP_PARAM(Kp, GridRows,                 int, 1,       uFormat("Number of rows of the grid used to extract uniformly \"%s / grid cells\" features from each cell.", kKpMaxFeatures().c_str()));
    RTABMAP_PARAM(Kp, GridCols,                 int, 1,       uFormat("Number of columns of the grid used to extract uniformly \"%s / grid cells\" features from each cell.", kKpMaxFeatures().c_str()));

    //Database
    RTABMAP_PARAM(DbSqlite3, InMemory,     bool, false,      "Using database in the memory instead of a file on the hard disk.");
    RTABMAP_PARAM(DbSqlite3, CacheSize, unsigned int, 10000, "Sqlite cache size (default is 2000).");
    RTABMAP_PARAM(DbSqlite3, JournalMode,  int, 3,           "0=DELETE, 1=TRUNCATE, 2=PERSIST, 3=MEMORY, 4=OFF (see sqlite3 doc : \"PRAGMA journal_mode\")");
    RTABMAP_PARAM(DbSqlite3, Synchronous,  int, 0,           "0=OFF, 1=NORMAL, 2=FULL (see sqlite3 doc : \"PRAGMA synchronous\")");
    RTABMAP_PARAM(DbSqlite3, TempStore,    int, 2,           "0=DEFAULT, 1=FILE, 2=MEMORY (see sqlite3 doc : \"PRAGMA temp_store\")");
    RTABMAP_PARAM_STR(Db, TargetVersion,   "",               "Target database version for backward compatibility purpose. Only Major and minor versions are used and should be set (e.g., 0.19 vs 0.20 or 1.0 vs 2.0). Patch version is ignored (e.g., 0.20.1 and 0.20.3 will generate a 0.20 database).");

    // Keypoints descriptors/detectors
    RTABMAP_PARAM(SURF, Extended,          bool, false,  "Extended descriptor flag (true - use extended 128-element descriptors; false - use 64-element descriptors).");
    RTABMAP_PARAM(SURF, HessianThreshold,  float, 500,   "Threshold for hessian keypoint detector used in SURF.");
    RTABMAP_PARAM(SURF, Octaves,           int, 4,       "Number of pyramid octaves the keypoint detector will use.");
    RTABMAP_PARAM(SURF, OctaveLayers,      int, 2,       "Number of octave layers within each octave.");
    RTABMAP_PARAM(SURF, Upright,           bool, false,  "Up-right or rotated features flag (true - do not compute orientation of features; false - compute orientation).");
    RTABMAP_PARAM(SURF, GpuVersion,        bool, false,  "GPU-SURF: Use GPU version of SURF. This option is enabled only if OpenCV is built with CUDA and GPUs are detected.");
    RTABMAP_PARAM(SURF, GpuKeypointsRatio,  float, 0.01, "Used with SURF GPU.");

    RTABMAP_PARAM(SIFT, NFeatures,         int, 0,       "The number of best features to retain. The features are ranked by their scores (measured in SIFT algorithm as the local contrast).");
    RTABMAP_PARAM(SIFT, NOctaveLayers,     int, 3,       "The number of layers in each octave. 3 is the value used in D. Lowe paper. The number of octaves is computed automatically from the image resolution.");
    RTABMAP_PARAM(SIFT, ContrastThreshold, double, 0.04, "The contrast threshold used to filter out weak features in semi-uniform (low-contrast) regions. The larger the threshold, the less features are produced by the detector.");
    RTABMAP_PARAM(SIFT, EdgeThreshold,     double, 10,   "The threshold used to filter out edge-like features. Note that the its meaning is different from the contrastThreshold, i.e. the larger the edgeThreshold, the less features are filtered out (more features are retained).");
    RTABMAP_PARAM(SIFT, Sigma,             double, 1.6,  "The sigma of the Gaussian applied to the input image at the octave #0. If your image is captured with a weak camera with soft lenses, you might want to reduce the number.");
    RTABMAP_PARAM(SIFT, RootSIFT,          bool,  false, "Apply RootSIFT normalization of the descriptors.");

    RTABMAP_PARAM(BRIEF, Bytes,            int, 32,      "Bytes is a length of descriptor in bytes. It can be equal 16, 32 or 64 bytes.");

    RTABMAP_PARAM(FAST, Threshold,          int, 20,      "Threshold on difference between intensity of the central pixel and pixels of a circle around this pixel.");
    RTABMAP_PARAM(FAST, NonmaxSuppression,  bool, true,   "If true, non-maximum suppression is applied to detected corners (keypoints).");
    RTABMAP_PARAM(FAST, Gpu,                bool, false,  "GPU-FAST: Use GPU version of FAST. This option is enabled only if OpenCV is built with CUDA and GPUs are detected.");
    RTABMAP_PARAM(FAST, GpuKeypointsRatio,  double, 0.05, "Used with FAST GPU.");
    RTABMAP_PARAM(FAST, MinThreshold,       int, 7,       "Minimum threshold. Used only when FAST/GridRows and FAST/GridCols are set.");
    RTABMAP_PARAM(FAST, MaxThreshold,       int, 200,     "Maximum threshold. Used only when FAST/GridRows and FAST/GridCols are set.");
    RTABMAP_PARAM(FAST, GridRows,           int, 0,       "Grid rows (0 to disable). Adapts the detector to partition the source image into a grid and detect points in each cell.");
    RTABMAP_PARAM(FAST, GridCols,           int, 0,       "Grid cols (0 to disable). Adapts the detector to partition the source image into a grid and detect points in each cell.");
    RTABMAP_PARAM(FAST, CV,                 int, 0,       "Enable FastCV implementation if non-zero (and RTAB-Map is built with FastCV support). Values should be 9 and 10.");

    RTABMAP_PARAM(GFTT, QualityLevel,      double, 0.001, "");
    RTABMAP_PARAM(GFTT, MinDistance,       double, 7,    "");
    RTABMAP_PARAM(GFTT, BlockSize,         int, 3,       "");
    RTABMAP_PARAM(GFTT, UseHarrisDetector, bool, false,  "");
    RTABMAP_PARAM(GFTT, K,                 double, 0.04, "");

    RTABMAP_PARAM(ORB, ScaleFactor,   float, 2,  "Pyramid decimation ratio, greater than 1. scaleFactor==2 means the classical pyramid, where each next level has 4x less pixels than the previous, but such a big scale factor will degrade feature matching scores dramatically. On the other hand, too close to 1 scale factor will mean that to cover certain scale range you will need more pyramid levels and so the speed will suffer.");
    RTABMAP_PARAM(ORB, NLevels,       int, 3,      "The number of pyramid levels. The smallest level will have linear size equal to input_image_linear_size/pow(scaleFactor, nlevels).");
    RTABMAP_PARAM(ORB, EdgeThreshold, int, 19,     "This is size of the border where the features are not detected. It should roughly match the patchSize parameter.");
    RTABMAP_PARAM(ORB, FirstLevel,    int, 0,      "It should be 0 in the current implementation.");
    RTABMAP_PARAM(ORB, WTA_K,         int, 2,      "The number of points that produce each element of the oriented BRIEF descriptor. The default value 2 means the BRIEF where we take a random point pair and compare their brightnesses, so we get 0/1 response. Other possible values are 3 and 4. For example, 3 means that we take 3 random points (of course, those point coordinates are random, but they are generated from the pre-defined seed, so each element of BRIEF descriptor is computed deterministically from the pixel rectangle), find point of maximum brightness and output index of the winner (0, 1 or 2). Such output will occupy 2 bits, and therefore it will need a special variant of Hamming distance, denoted as NORM_HAMMING2 (2 bits per bin). When WTA_K=4, we take 4 random points to compute each bin (that will also occupy 2 bits with possible values 0, 1, 2 or 3).");
    RTABMAP_PARAM(ORB, ScoreType,     int, 0,      "The default HARRIS_SCORE=0 means that Harris algorithm is used to rank features (the score is written to KeyPoint::score and is used to retain best nfeatures features); FAST_SCORE=1 is alternative value of the parameter that produces slightly less stable keypoints, but it is a little faster to compute.");
    RTABMAP_PARAM(ORB, PatchSize,     int, 31,     "size of the patch used by the oriented BRIEF descriptor. Of course, on smaller pyramid layers the perceived image area covered by a feature will be larger.");
    RTABMAP_PARAM(ORB, Gpu,           bool, false, "GPU-ORB: Use GPU version of ORB. This option is enabled only if OpenCV is built with CUDA and GPUs are detected.");

    RTABMAP_PARAM(FREAK, OrientationNormalized, bool, true,   "Enable orientation normalization.");
    RTABMAP_PARAM(FREAK, ScaleNormalized,       bool, true,   "Enable scale normalization.");
    RTABMAP_PARAM(FREAK, PatternScale,          float, 22,    "Scaling of the description pattern.");
    RTABMAP_PARAM(FREAK, NOctaves,              int, 4,       "Number of octaves covered by the detected keypoints.");

    RTABMAP_PARAM(BRISK, Thresh,       int, 30, "FAST/AGAST detection threshold score.");
    RTABMAP_PARAM(BRISK, Octaves,      int, 3,  "Detection octaves. Use 0 to do single scale.");
    RTABMAP_PARAM(BRISK, PatternScale, float, 1,"Apply this scale to the pattern used for sampling the neighbourhood of a keypoint.");

    RTABMAP_PARAM(KAZE, Extended,       bool, false,   "Set to enable extraction of extended (128-byte) descriptor.");
    RTABMAP_PARAM(KAZE, Upright,        bool, false,   "Set to enable use of upright descriptors (non rotation-invariant).");
    RTABMAP_PARAM(KAZE, Threshold,      float, 0.001,  "Detector response threshold to accept keypoint.");
    RTABMAP_PARAM(KAZE, NOctaves,       int, 4,        "Maximum octave evolution of the image.");
    RTABMAP_PARAM(KAZE, NOctaveLayers,  int, 4,        "Default number of sublevels per scale level.");
    RTABMAP_PARAM(KAZE, Diffusivity,    int, 1,        "Diffusivity type: 0=DIFF_PM_G1, 1=DIFF_PM_G2, 2=DIFF_WEICKERT or 3=DIFF_CHARBONNIER.");

    RTABMAP_PARAM_STR(SuperPoint, ModelPath, "",           "[Required] Path to pre-trained weights Torch file of SuperPoint (*.pt).");
    RTABMAP_PARAM(SuperPoint, Threshold,     float, 0.010, "Detector response threshold to accept keypoint.");
    RTABMAP_PARAM(SuperPoint, NMS,           bool,  true,  "If true, non-maximum suppression is applied to detected keypoints.");
    RTABMAP_PARAM(SuperPoint, NMSRadius,     int,  4,      uFormat("[%s=true] Minimum distance (pixels) between keypoints.", kSuperPointNMS().c_str()));
    RTABMAP_PARAM(SuperPoint, Cuda,          bool, true,   "Use Cuda device for Torch, otherwise CPU device is used by default.");

    RTABMAP_PARAM_STR(PyDetector, Path,       "",           "Path to python script file (see available ones in rtabmap/corelib/src/python/*). See the header to see where the script should be copied.");
	RTABMAP_PARAM(PyDetector, Cuda,           bool, true,   "Use cuda.");

    // BayesFilter
    RTABMAP_PARAM(Bayes, VirtualPlacePriorThr, float, 0.9,  "Virtual place prior");
    RTABMAP_PARAM_STR(Bayes, PredictionLC, "0.1 0.36 0.30 0.16 0.062 0.0151 0.00255 0.000324 2.5e-05 1.3e-06 4.8e-08 1.2e-09 1.9e-11 2.2e-13 1.7e-15 8.5e-18 2.9e-20 6.9e-23", "Prediction of loop closures (Gaussian-like, here with sigma=1.6) - Format: {VirtualPlaceProb, LoopClosureProb, NeighborLvl1, NeighborLvl2, ...}.");
    RTABMAP_PARAM(Bayes, FullPredictionUpdate, bool, false, "Regenerate all the prediction matrix on each iteration (otherwise only removed/added ids are updated).");

    // Verify hypotheses
    RTABMAP_PARAM(VhEp, Enabled, bool, false,       uFormat("Verify visual loop closure hypothesis by computing a fundamental matrix. This is done prior to transformation computation when %s is enabled.", kRGBDEnabled().c_str()));
    RTABMAP_PARAM(VhEp, MatchCountMin, int, 8,      "Minimum of matching visual words pairs to accept the loop hypothesis.");
    RTABMAP_PARAM(VhEp, RansacParam1,  float, 3,    "Fundamental matrix (see cvFindFundamentalMat()): Max distance (in pixels) from the epipolar line for a point to be inlier.");
    RTABMAP_PARAM(VhEp, RansacParam2,  float, 0.99, "Fundamental matrix (see cvFindFundamentalMat()): Performance of RANSAC.");

    // RGB-D SLAM
    RTABMAP_PARAM(RGBD, Enabled,                  bool, true,  "Activate metric SLAM. If set to false, classic RTAB-Map loop closure detection is done using only images and without any metric information.");
    RTABMAP_PARAM(RGBD, LinearUpdate,             float, 0.1,  "Minimum linear displacement (m) to update the map. Rehearsal is done prior to this, so weights are still updated.");
    RTABMAP_PARAM(RGBD, AngularUpdate,            float, 0.1,  "Minimum angular displacement (rad) to update the map. Rehearsal is done prior to this, so weights are still updated.");
    RTABMAP_PARAM(RGBD, LinearSpeedUpdate,        float, 0.0,  "Maximum linear speed (m/s) to update the map (0 means not limit).");
    RTABMAP_PARAM(RGBD, AngularSpeedUpdate,       float, 0.0,  "Maximum angular speed (rad/s) to update the map (0 means not limit).");
    RTABMAP_PARAM(RGBD, NewMapOdomChangeDistance, float, 0,    "A new map is created if a change of odometry translation greater than X m is detected (0 m = disabled).");
    RTABMAP_PARAM(RGBD, OptimizeFromGraphEnd,     bool, false, "Optimize graph from the newest node. If false, the graph is optimized from the oldest node of the current graph (this adds an overhead computation to detect to oldest node of the current graph, but it can be useful to preserve the map referential from the oldest node). Warning when set to false: when some nodes are transferred, the first referential of the local map may change, resulting in momentary changes in robot/map position (which are annoying in teleoperation).");
    RTABMAP_PARAM(RGBD, OptimizeMaxError,         float, 3.0,   uFormat("Reject loop closures if optimization error ratio is greater than this value (0=disabled). Ratio is computed as absolute error over standard deviation of each link. This will help to detect when a wrong loop closure is added to the graph. Not compatible with \"%s\" if enabled.", kOptimizerRobust().c_str()));
    RTABMAP_PARAM(RGBD, MaxLoopClosureDistance,   float, 0.0,   "Reject loop closures/localizations if the distance from the map is over this distance (0=disabled).");
    RTABMAP_PARAM(RGBD, StartAtOrigin,            bool, false, uFormat("If true, rtabmap will assume the robot is starting from origin of the map. If false, rtabmap will assume the robot is restarting from the last saved localization pose from previous session (the place where it shut down previously). Used only in localization mode (%s=false).", kMemIncrementalMemory().c_str()));
    RTABMAP_PARAM(RGBD, GoalReachedRadius,        float, 0.5,  "Goal reached radius (m).");
    RTABMAP_PARAM(RGBD, PlanStuckIterations,      int, 0,      "Mark the current goal node on the path as unreachable if it is not updated after X iterations (0=disabled). If all upcoming nodes on the path are unreachabled, the plan fails.");
    RTABMAP_PARAM(RGBD, PlanLinearVelocity,       float, 0,    "Linear velocity (m/sec) used to compute path weights.");
    RTABMAP_PARAM(RGBD, PlanAngularVelocity,      float, 0,    "Angular velocity (rad/sec) used to compute path weights.");
    RTABMAP_PARAM(RGBD, GoalsSavedInUserData,     bool, false, "When a goal is received and processed with success, it is saved in user data of the location with this format: \"GOAL:#\".");
    RTABMAP_PARAM(RGBD, MaxLocalRetrieved,        unsigned int, 2, "Maximum local locations retrieved (0=disabled) near the current pose in the local map or on the current planned path (those on the planned path have priority).");
    RTABMAP_PARAM(RGBD, LocalRadius,              float, 10,   "Local radius (m) for nodes selection in the local map. This parameter is used in some approaches about the local map management.");
    RTABMAP_PARAM(RGBD, LocalImmunizationRatio,   float, 0.25, "Ratio of working memory for which local nodes are immunized from transfer.");
    RTABMAP_PARAM(RGBD, ScanMatchingIdsSavedInLinks, bool, true,    "Save scan matching IDs from one-to-many proximity detection in link's user data.");
    RTABMAP_PARAM(RGBD, NeighborLinkRefining,         bool, false,  uFormat("When a new node is added to the graph, the transformation of its neighbor link to the previous node is refined using registration approach selected (%s).", kRegStrategy().c_str()));
    RTABMAP_PARAM(RGBD, LoopClosureIdentityGuess,     bool, false,  uFormat("Use Identity matrix as guess when computing loop closure transform, otherwise no guess is used, thus assuming that registration strategy selected (%s) can deal with transformation estimation without guess.", kRegStrategy().c_str()));
    RTABMAP_PARAM(RGBD, LoopClosureReextractFeatures, bool, false,  "Extract features even if there are some already in the nodes. Raw features are not saved in database.");
    RTABMAP_PARAM(RGBD, LocalBundleOnLoopClosure,     bool, false,  "Do local bundle adjustment with neighborhood of the loop closure.");
    RTABMAP_PARAM(RGBD, InvertedReg,                  bool, false,  "On loop closure, do registration from the target to reference instead of reference to target.");
    RTABMAP_PARAM(RGBD, CreateOccupancyGrid,          bool, false,  "Create local occupancy grid maps. See \"Grid\" group for parameters.");
    RTABMAP_PARAM(RGBD, MarkerDetection,              bool, false,  "Detect static markers to be added as landmarks for graph optimization. If input data have already landmarks, this will be ignored. See \"Marker\" group for parameters.");
    RTABMAP_PARAM(RGBD, LoopCovLimited,               bool, false,  "Limit covariance of non-neighbor links to minimum covariance of neighbor links. In other words, if covariance of a loop closure link is smaller than the minimum covariance of odometry links, its covariance is set to minimum covariance of odometry links.");
    RTABMAP_PARAM(RGBD, MaxOdomCacheSize,             int,  10,      uFormat("Maximum odometry cache size. Used only in localization mode (when %s=false). This is used to get smoother localizations and to verify localization transforms (when %s!=0) to make sure we don't teleport to a location very similar to one we previously localized on. Set 0 to disable caching.", kMemIncrementalMemory().c_str(), kRGBDOptimizeMaxError().c_str()));

    // Local/Proximity loop closure detection
    RTABMAP_PARAM(RGBD, ProximityByTime,              bool, false, "Detection over all locations in STM.");
    RTABMAP_PARAM(RGBD, ProximityBySpace,             bool, true,  "Detection over locations (in Working Memory) near in space.");
    RTABMAP_PARAM(RGBD, ProximityMaxGraphDepth,       int, 50,     "Maximum depth from the current/last loop closure location and the local loop closure hypotheses. Set 0 to ignore.");
    RTABMAP_PARAM(RGBD, ProximityMaxPaths,            int, 3,      "Maximum paths compared (from the most recent) for proximity detection. 0 means no limit.");
    RTABMAP_PARAM(RGBD, ProximityPathFilteringRadius, float, 1,    "Path filtering radius to reduce the number of nodes to compare in a path in one-to-many proximity detection. The nearest node in a path should be inside that radius to be considered for one-to-one proximity detection.");
    RTABMAP_PARAM(RGBD, ProximityPathMaxNeighbors,    int, 0,      "Maximum neighbor nodes compared on each path for one-to-many proximity detection. Set to 0 to disable one-to-many proximity detection (by merging the laser scans).");
    RTABMAP_PARAM(RGBD, ProximityPathRawPosesUsed,    bool, true,  "When comparing to a local path for one-to-many proximity detection, merge the scans using the odometry poses (with neighbor link optimizations) instead of the ones in the optimized local graph.");
    RTABMAP_PARAM(RGBD, ProximityAngle,               float, 45,   "Maximum angle (degrees) for one-to-one proximity detection.");
    RTABMAP_PARAM(RGBD, ProximityOdomGuess,           bool, false, "Use odometry as motion guess for one-to-one proximity detection.");
    RTABMAP_PARAM(RGBD, ProximityGlobalScanMap,       bool, false, uFormat("Create a global assembled map from laser scans for one-to-many proximity detection, replacing the original one-to-many proximity detection (i.e., detection against local paths). Only used in localization mode (%s=false), otherwise original one-to-many proximity detection is done. Note also that if graph is modified (i.e., memory management is enabled or robot jumps from one disjoint session to another in same database), the global scan map is cleared and one-to-many proximity detection is reverted to original approach.", kMemIncrementalMemory().c_str()));
    RTABMAP_PARAM(RGBD, ProximityMergedScanCovFactor, double, 100.0, uFormat("Covariance factor for one-to-many proximity detection (when %s>0 and scans are used).", kRGBDProximityPathMaxNeighbors().c_str()));

    // Graph optimization
#ifdef RTABMAP_GTSAM
    RTABMAP_PARAM(Optimizer, Strategy,        int, 2,          "Graph optimization strategy: 0=TORO, 1=g2o, 2=GTSAM and 3=Ceres.");
    RTABMAP_PARAM(Optimizer, Iterations,      int, 20,         "Optimization iterations.");
    RTABMAP_PARAM(Optimizer, Epsilon,         double, 0.00001, "Stop optimizing when the error improvement is less than this value.");
#else
#ifdef RTABMAP_G2O
    RTABMAP_PARAM(Optimizer, Strategy,        int, 1,          "Graph optimization strategy: 0=TORO, 1=g2o, 2=GTSAM and 3=Ceres.");
    RTABMAP_PARAM(Optimizer, Iterations,      int, 20,         "Optimization iterations.");
    RTABMAP_PARAM(Optimizer, Epsilon,         double, 0.0,     "Stop optimizing when the error improvement is less than this value.");
#else
#ifdef RTABMAP_CERES
    RTABMAP_PARAM(Optimizer, Strategy,        int, 3,          "Graph optimization strategy: 0=TORO, 1=g2o, 2=GTSAM and 3=Ceres.");
    RTABMAP_PARAM(Optimizer, Iterations,      int, 20,         "Optimization iterations.");
    RTABMAP_PARAM(Optimizer, Epsilon,         double, 0.000001, "Stop optimizing when the error improvement is less than this value.");
#else
    RTABMAP_PARAM(Optimizer, Strategy,        int, 0,          "Graph optimization strategy: 0=TORO, 1=g2o, 2=GTSAM and 3=Ceres.");
    RTABMAP_PARAM(Optimizer, Iterations,      int, 100,        "Optimization iterations.");
    RTABMAP_PARAM(Optimizer, Epsilon,         double, 0.00001, "Stop optimizing when the error improvement is less than this value.");
#endif
#endif
#endif
    RTABMAP_PARAM(Optimizer, VarianceIgnored, bool, false,     "Ignore constraints' variance. If checked, identity information matrix is used for each constraint. Otherwise, an information matrix is generated from the variance saved in the links.");
    RTABMAP_PARAM(Optimizer, Robust,          bool, false,     uFormat("Robust graph optimization using Vertigo (only work for g2o and GTSAM optimization strategies). Not compatible with \"%s\" if enabled.", kRGBDOptimizeMaxError().c_str()));
    RTABMAP_PARAM(Optimizer, PriorsIgnored,   bool, true,      "Ignore prior constraints (global pose or GPS) while optimizing. Currently only g2o and gtsam optimization supports this.");
    RTABMAP_PARAM(Optimizer, LandmarksIgnored,   bool, false,  "Ignore landmark constraints while optimizing. Currently only g2o and gtsam optimization supports this.");
#if defined(RTABMAP_G2O) || defined(RTABMAP_GTSAM)
    RTABMAP_PARAM(Optimizer, GravitySigma,    float, 0.3,      uFormat("Gravity sigma value (>=0, typically between 0.1 and 0.3). Optimization is done while preserving gravity orientation of the poses. This should be used only with visual/lidar inertial odometry approaches, for which we assume that all odometry poses are aligned with gravity. Set to 0 to disable gravity constraints. Currently supported only with g2o and GTSAM optimization strategies (see %s).", kOptimizerStrategy().c_str()));
#else
    RTABMAP_PARAM(Optimizer, GravitySigma,    float, 0.0,      uFormat("Gravity sigma value (>=0, typically between 0.1 and 0.3). Optimization is done while preserving gravity orientation of the poses. This should be used only with visual/lidar inertial odometry approaches, for which we assume that all odometry poses are aligned with gravity. Set to 0 to disable gravity constraints. Currently supported only with g2o and GTSAM optimization strategies (see %s).", kOptimizerStrategy().c_str()));
#endif

#ifdef RTABMAP_ORB_SLAM
    RTABMAP_PARAM(g2o, Solver,            int, 3,          "0=csparse 1=pcg 2=cholmod 3=Eigen");
#else
    RTABMAP_PARAM(g2o, Solver,            int, 0,          "0=csparse 1=pcg 2=cholmod 3=Eigen");
#endif
    RTABMAP_PARAM(g2o, Optimizer,         int, 0,          "0=Levenberg 1=GaussNewton");
    RTABMAP_PARAM(g2o, PixelVariance,     double, 1.0,     "Pixel variance used for bundle adjustment.");
    RTABMAP_PARAM(g2o, RobustKernelDelta, double, 8,       "Robust kernel delta used for bundle adjustment (0 means don't use robust kernel). Observations with chi2 over this threshold will be ignored in the second optimization pass.");
    RTABMAP_PARAM(g2o, Baseline,          double, 0.075,   "When doing bundle adjustment with RGB-D data, we can set a fake baseline (m) to do stereo bundle adjustment (if 0, mono bundle adjustment is done). For stereo data, the baseline in the calibration is used directly.");

    RTABMAP_PARAM(GTSAM, Optimizer,       int, 1,          "0=Levenberg 1=GaussNewton 2=Dogleg");

    // Odometry
    RTABMAP_PARAM(Odom, Strategy,               int, 0,       "0=Frame-to-Map (F2M) 1=Frame-to-Frame (F2F) 2=Fovis 3=viso2 4=DVO-SLAM 5=ORB_SLAM2 6=OKVIS 7=LOAM 8=MSCKF_VIO 9=VINS-Fusion 10=OpenVINS 11=FLOAM 12=Open3D");
    RTABMAP_PARAM(Odom, ResetCountdown,         int, 0,       "Automatically reset odometry after X consecutive images on which odometry cannot be computed (value=0 disables auto-reset).");
    RTABMAP_PARAM(Odom, Holonomic,              bool, true,   "If the robot is holonomic (strafing commands can be issued). If not, y value will be estimated from x and yaw values (y=x*tan(yaw)).");
    RTABMAP_PARAM(Odom, FillInfoData,           bool, true,   "Fill info with data (inliers/outliers features).");
    RTABMAP_PARAM(Odom, ImageBufferSize,     unsigned int, 1, "Data buffer size (0 min inf).");
    RTABMAP_PARAM(Odom, FilteringStrategy,      int, 0,       "0=No filtering 1=Kalman filtering 2=Particle filtering. This filter is used to smooth the odometry output.");
    RTABMAP_PARAM(Odom, ParticleSize,      unsigned int, 400, "Number of particles of the filter.");
    RTABMAP_PARAM(Odom, ParticleNoiseT,         float, 0.002, "Noise (m) of translation components (x,y,z).");
    RTABMAP_PARAM(Odom, ParticleLambdaT,        float, 100,   "Lambda of translation components (x,y,z).");
    RTABMAP_PARAM(Odom, ParticleNoiseR,         float, 0.002, "Noise (rad) of rotational components (roll,pitch,yaw).");
    RTABMAP_PARAM(Odom, ParticleLambdaR,        float, 100,   "Lambda of rotational components (roll,pitch,yaw).");
    RTABMAP_PARAM(Odom, KalmanProcessNoise,     float, 0.001, "Process noise covariance value.");
    RTABMAP_PARAM(Odom, KalmanMeasurementNoise, float, 0.01,  "Process measurement covariance value.");
    RTABMAP_PARAM(Odom, GuessMotion,            bool, true,   "Guess next transformation from the last motion computed.");
    RTABMAP_PARAM(Odom, GuessSmoothingDelay,    float, 0,     uFormat("Guess smoothing delay (s). Estimated velocity is averaged based on last transforms up to this maximum delay. This can help to get smoother velocity prediction. Last velocity computed is used directly if \"%s\" is set or the delay is below the odometry rate.", kOdomFilteringStrategy().c_str()));
    RTABMAP_PARAM(Odom, KeyFrameThr,            float, 0.3,   "[Visual] Create a new keyframe when the number of inliers drops under this ratio of features in last frame. Setting the value to 0 means that a keyframe is created for each processed frame.");
    RTABMAP_PARAM(Odom, VisKeyFrameThr,         int, 150,     "[Visual] Create a new keyframe when the number of inliers drops under this threshold. Setting the value to 0 means that a keyframe is created for each processed frame.");
    RTABMAP_PARAM(Odom, ScanKeyFrameThr,        float, 0.9,   "[Geometry] Create a new keyframe when the number of ICP inliers drops under this ratio of points in last frame's scan. Setting the value to 0 means that a keyframe is created for each processed frame.");
    RTABMAP_PARAM(Odom, ImageDecimation,     unsigned int, 1, uFormat("Decimation of the RGB image before registration. If depth size is larger than decimated RGB size, depth is decimated to be always at most equal to RGB size. If %s is true and if depth is smaller than decimated RGB, depth may be interpolated to match RGB size for feature detection.", kVisDepthAsMask().c_str()));
    RTABMAP_PARAM(Odom, AlignWithGround,        bool, false,  "Align odometry with the ground on initialization.");

    // Odometry Frame-to-Map
    RTABMAP_PARAM(OdomF2M, MaxSize,             int, 2000,    "[Visual] Local map size: If > 0 (example 5000), the odometry will maintain a local map of X maximum words.");
    RTABMAP_PARAM(OdomF2M, MaxNewFeatures,      int, 0,       "[Visual] Maximum features (sorted by keypoint response) added to local map from a new key-frame. 0 means no limit.");
    RTABMAP_PARAM(OdomF2M, ScanMaxSize,         int, 2000,    "[Geometry] Maximum local scan map size.");
    RTABMAP_PARAM(OdomF2M, ScanSubtractRadius,  float, 0.05,  "[Geometry] Radius used to filter points of a new added scan to local map. This could match the voxel size of the scans.");
    RTABMAP_PARAM(OdomF2M, ScanSubtractAngle,   float, 45,    uFormat("[Geometry] Max angle (degrees) used to filter points of a new added scan to local map (when \"%s\">0). 0 means any angle.", kOdomF2MScanSubtractRadius().c_str()).c_str());
    RTABMAP_PARAM(OdomF2M, ScanRange,           float, 0,     "[Geometry] Distance Range used to filter points of local map (when > 0). 0 means local map is updated using time and not range.");
    RTABMAP_PARAM(OdomF2M, ValidDepthRatio,     float, 0.75,  "If a new frame has points without valid depth, they are added to local feature map only if points with valid depth on total points is over this ratio. Setting to 1 means no points without valid depth are added to local feature map.");
#if defined(RTABMAP_G2O) || defined(RTABMAP_ORB_SLAM)
    RTABMAP_PARAM(OdomF2M, BundleAdjustment,          int, 1, "Local bundle adjustment: 0=disabled, 1=g2o, 2=cvsba, 3=Ceres.");
#else
    RTABMAP_PARAM(OdomF2M, BundleAdjustment,          int, 0, "Local bundle adjustment: 0=disabled, 1=g2o, 2=cvsba, 3=Ceres.");
#endif
    RTABMAP_PARAM(OdomF2M, BundleAdjustmentMaxFrames, int, 10, "Maximum frames used for bundle adjustment (0=inf or all current frames in the local map).");

    // Odometry Mono
    RTABMAP_PARAM(OdomMono, InitMinFlow,        float, 100,  "Minimum optical flow required for the initialization step.");
    RTABMAP_PARAM(OdomMono, InitMinTranslation, float, 0.1,  "Minimum translation required for the initialization step.");
    RTABMAP_PARAM(OdomMono, MinTranslation,     float, 0.02, "Minimum translation to add new points to local map. On initialization, translation x 5 is used as the minimum.");
    RTABMAP_PARAM(OdomMono, MaxVariance,        float, 0.01, "Maximum variance to add new points to local map.");

    // Odometry Fovis
    RTABMAP_PARAM(OdomFovis, FeatureWindowSize,               int, 9,        "The size of the n x n image patch surrounding each feature, used for keypoint matching.");
    RTABMAP_PARAM(OdomFovis, MaxPyramidLevel,                 int, 3,        "The maximum Gaussian pyramid level to process the image at. Pyramid level 1 corresponds to the original image.");
    RTABMAP_PARAM(OdomFovis, MinPyramidLevel,                 int, 0,        "The minimum pyramid level.");
    RTABMAP_PARAM(OdomFovis, TargetPixelsPerFeature,          int, 250,      "Specifies the desired feature density as a ratio of input image pixels per feature detected.  This number is used to control the adaptive feature thresholding.");
    RTABMAP_PARAM(OdomFovis, FastThreshold,                   int, 20,       "FAST threshold.");
    RTABMAP_PARAM(OdomFovis, UseAdaptiveThreshold,            bool, true,    "Use FAST adaptive threshold.");
    RTABMAP_PARAM(OdomFovis, FastThresholdAdaptiveGain,       double, 0.005, "FAST threshold adaptive gain.");
    RTABMAP_PARAM(OdomFovis, UseHomographyInitialization,     bool, true,    "Use homography initialization.");

    RTABMAP_PARAM(OdomFovis, UseBucketing,                    bool, true,  "");
    RTABMAP_PARAM(OdomFovis, BucketWidth,                     int, 80,     "");
    RTABMAP_PARAM(OdomFovis, BucketHeight,                    int, 80,     "");
    RTABMAP_PARAM(OdomFovis, MaxKeypointsPerBucket,           int, 25,     "");
    RTABMAP_PARAM(OdomFovis, UseImageNormalization,           bool, false, "");

    RTABMAP_PARAM(OdomFovis, InlierMaxReprojectionError,      double, 1.5,  "The maximum image-space reprojection error (in pixels) a feature match is allowed to have and still be considered an inlier in the set of features used for motion estimation.");
    RTABMAP_PARAM(OdomFovis, CliqueInlierThreshold,           double, 0.1,  "See Howard's greedy max-clique algorithm for determining the maximum set of mutually consisten feature matches. This specifies the compatibility threshold, in meters.");
    RTABMAP_PARAM(OdomFovis, MinFeaturesForEstimate,          int, 20,      "Minimum number of features in the inlier set for the motion estimate to be considered valid.");
    RTABMAP_PARAM(OdomFovis, MaxMeanReprojectionError,        double, 10.0, "Maximum mean reprojection error over the inlier feature matches for the motion estimate to be considered valid.");
    RTABMAP_PARAM(OdomFovis, UseSubpixelRefinement,           bool, true,   "Specifies whether or not to refine feature matches to subpixel resolution.");
    RTABMAP_PARAM(OdomFovis, FeatureSearchWindow,             int, 25,      "Specifies the size of the search window to apply when searching for feature matches across time frames.  The search is conducted around the feature location predicted by the initial rotation estimate.");
    RTABMAP_PARAM(OdomFovis, UpdateTargetFeaturesWithRefined, bool, false,  "When subpixel refinement is enabled, the refined feature locations can be saved over the original feature locations.  This has a slightly negative impact on frame-to-frame visual odometry, but is likely better when using this library as part of a visual SLAM algorithm.");

    RTABMAP_PARAM(OdomFovis, StereoRequireMutualMatch,        bool, true,  "");
    RTABMAP_PARAM(OdomFovis, StereoMaxDistEpipolarLine,       double, 1.5, "");
    RTABMAP_PARAM(OdomFovis, StereoMaxRefinementDisplacement, double, 1.0, "");
    RTABMAP_PARAM(OdomFovis, StereoMaxDisparity,              int, 128,    "");

    // Odometry viso2
    RTABMAP_PARAM(OdomViso2, RansacIters,               int, 200,    "Number of RANSAC iterations.");
    RTABMAP_PARAM(OdomViso2, InlierThreshold,           double, 2.0, "Fundamental matrix inlier threshold.");
    RTABMAP_PARAM(OdomViso2, Reweighting,               bool, true,  "Lower border weights (more robust to calibration errors).");
    RTABMAP_PARAM(OdomViso2, MatchNmsN,                 int, 3,      "Non-max-suppression: min. distance between maxima (in pixels).");
    RTABMAP_PARAM(OdomViso2, MatchNmsTau,               int, 50,     "Non-max-suppression: interest point peakiness threshold.");
    RTABMAP_PARAM(OdomViso2, MatchBinsize,              int, 50,     "Matching bin width/height (affects efficiency only).");
    RTABMAP_PARAM(OdomViso2, MatchRadius,               int, 200,    "Matching radius (du/dv in pixels).");
    RTABMAP_PARAM(OdomViso2, MatchDispTolerance,        int, 2,      "Disparity tolerance for stereo matches (in pixels).");
    RTABMAP_PARAM(OdomViso2, MatchOutlierDispTolerance, int, 5,      "Outlier removal: disparity tolerance (in pixels).");
    RTABMAP_PARAM(OdomViso2, MatchOutlierFlowTolerance, int, 5,      "Outlier removal: flow tolerance (in pixels).");
    RTABMAP_PARAM(OdomViso2, MatchMultiStage,           bool, true,  "Multistage matching (denser and faster).");
    RTABMAP_PARAM(OdomViso2, MatchHalfResolution,       bool, true,  "Match at half resolution, refine at full resolution.");
    RTABMAP_PARAM(OdomViso2, MatchRefinement,           int, 1,      "Refinement (0=none,1=pixel,2=subpixel).");
    RTABMAP_PARAM(OdomViso2, BucketMaxFeatures,         int, 2,      "Maximal number of features per bucket.");
    RTABMAP_PARAM(OdomViso2, BucketWidth,               double, 50,  "Width of bucket.");
    RTABMAP_PARAM(OdomViso2, BucketHeight,              double, 50,  "Height of bucket.");

    // Odometry ORB_SLAM2
    RTABMAP_PARAM_STR(OdomORBSLAM, VocPath,             "",    "Path to ORB vocabulary (*.txt).");
    RTABMAP_PARAM(OdomORBSLAM, Bf,              double, 0.076, "Fake IR projector baseline (m) used only when stereo is not used.");
    RTABMAP_PARAM(OdomORBSLAM, ThDepth,         double, 40.0,  "Close/Far threshold. Baseline times.");
    RTABMAP_PARAM(OdomORBSLAM, Fps,             float,  0.0,   "Camera FPS.");
    RTABMAP_PARAM(OdomORBSLAM, MaxFeatures,     int,    1000,  "Maximum ORB features extracted per frame.");
    RTABMAP_PARAM(OdomORBSLAM, MapSize,         int,    3000,  "Maximum size of the feature map (0 means infinite).");

    // Odometry OKVIS
    RTABMAP_PARAM_STR(OdomOKVIS, ConfigPath,     "",  "Path of OKVIS config file.");

    // Odometry LOAM
    RTABMAP_PARAM(OdomLOAM, Sensor,     int,    2,    "Velodyne sensor: 0=VLP-16, 1=HDL-32, 2=HDL-64E");
    RTABMAP_PARAM(OdomLOAM, ScanPeriod, float,  0.1,  "Scan period (s)");
    RTABMAP_PARAM(OdomLOAM, Resolution, float,  0.2,   "Map resolution");
    RTABMAP_PARAM(OdomLOAM, LinVar,     float,  0.01,  "Linear output variance.");
    RTABMAP_PARAM(OdomLOAM, AngVar,     float,  0.01,  "Angular output variance.");
    RTABMAP_PARAM(OdomLOAM, LocalMapping, bool,  true,  "Local mapping. It adds more time to compute odometry, but accuracy is significantly improved.");

    // Odometry MSCKF_VIO
    RTABMAP_PARAM(OdomMSCKF, GridRow,           int,  4,  "");
    RTABMAP_PARAM(OdomMSCKF, GridCol,           int,  5,  "");
    RTABMAP_PARAM(OdomMSCKF, GridMinFeatureNum, int,  3,  "");
    RTABMAP_PARAM(OdomMSCKF, GridMaxFeatureNum, int,  4,  "");
    RTABMAP_PARAM(OdomMSCKF, PyramidLevels,     int,  3,  "");
    RTABMAP_PARAM(OdomMSCKF, PatchSize,         int,  15,  "");
    RTABMAP_PARAM(OdomMSCKF, FastThreshold,     int,  10,  "");
    RTABMAP_PARAM(OdomMSCKF, MaxIteration,      int,  30,  "");
    RTABMAP_PARAM(OdomMSCKF, TrackPrecision,    double,  0.01,  "");
    RTABMAP_PARAM(OdomMSCKF, RansacThreshold,   double,  3,  "");
    RTABMAP_PARAM(OdomMSCKF, StereoThreshold,   double,  5,  "");
    RTABMAP_PARAM(OdomMSCKF, PositionStdThreshold,    double,  8.0,  "");
    RTABMAP_PARAM(OdomMSCKF, RotationThreshold,       double,  0.2618,  "");
    RTABMAP_PARAM(OdomMSCKF, TranslationThreshold,    double,  0.4,  "");
    RTABMAP_PARAM(OdomMSCKF, TrackingRateThreshold,   double,  0.5,  "");
    RTABMAP_PARAM(OdomMSCKF, OptTranslationThreshold, double,  0,  "");
    RTABMAP_PARAM(OdomMSCKF, NoiseGyro,     double,  0.005,  "");
    RTABMAP_PARAM(OdomMSCKF, NoiseAcc,      double,  0.05,  "");
    RTABMAP_PARAM(OdomMSCKF, NoiseGyroBias, double,  0.001,  "");
    RTABMAP_PARAM(OdomMSCKF, NoiseAccBias,  double,  0.01,  "");
    RTABMAP_PARAM(OdomMSCKF, NoiseFeature,  double,  0.035,  "");
    RTABMAP_PARAM(OdomMSCKF, InitCovVel,    double,  0.25,  "");
    RTABMAP_PARAM(OdomMSCKF, InitCovGyroBias, double,  0.01,  "");
    RTABMAP_PARAM(OdomMSCKF, InitCovAccBias,  double,  0.01,  "");
    RTABMAP_PARAM(OdomMSCKF, InitCovExRot,    double,  0.00030462,  "");
    RTABMAP_PARAM(OdomMSCKF, InitCovExTrans,  double,  0.000025,  "");
    RTABMAP_PARAM(OdomMSCKF, MaxCamStateSize,  int,  20,  "");

    // Odometry VINS
    RTABMAP_PARAM_STR(OdomVINS, ConfigPath,     "",  "Path of VINS config file.");

    // Odometry Open3D
    RTABMAP_PARAM(OdomOpen3D, MaxDepth,         float, 3.0,  "Maximum depth.");
    RTABMAP_PARAM(OdomOpen3D, Method,           int, 0,  "Registration method: 0=PointToPlane, 1=Intensity, 2=Hybrid.");

    // Common registration parameters
    RTABMAP_PARAM(Reg, RepeatOnce,               bool, true,    "Do a second registration with the output of the first registration as guess. Only done if no guess was provided for the first registration (like on loop closure). It can be useful if the registration approach used can use a guess to get better matches.");
    RTABMAP_PARAM(Reg, Strategy,                 int, 0,        "0=Vis, 1=Icp, 2=VisIcp");
    RTABMAP_PARAM(Reg, Force3DoF,                bool, false,   "Force 3 degrees-of-freedom transform (3Dof: x,y and yaw). Parameters z, roll and pitch will be set to 0.");

    // Visual registration parameters
    RTABMAP_PARAM(Vis, EstimationType,           int, 1,        "Motion estimation approach: 0:3D->3D, 1:3D->2D (PnP), 2:2D->2D (Epipolar Geometry)");
    RTABMAP_PARAM(Vis, ForwardEstOnly,           bool, true,    "Forward estimation only (A->B). If false, a transformation is also computed in backward direction (B->A), then the two resulting transforms are merged (middle interpolation between the transforms).");
    RTABMAP_PARAM(Vis, InlierDistance,           float, 0.1,    uFormat("[%s = 0] Maximum distance for feature correspondences. Used by 3D->3D estimation approach.", kVisEstimationType().c_str()));
    RTABMAP_PARAM(Vis, RefineIterations,         int, 5,        uFormat("[%s = 0] Number of iterations used to refine the transformation found by RANSAC. 0 means that the transformation is not refined.", kVisEstimationType().c_str()));
    RTABMAP_PARAM(Vis, PnPReprojError,           float, 2,      uFormat("[%s = 1] PnP reprojection error.", kVisEstimationType().c_str()));
    RTABMAP_PARAM(Vis, PnPFlags,                 int, 0,        uFormat("[%s = 1] PnP flags: 0=Iterative, 1=EPNP, 2=P3P", kVisEstimationType().c_str()));
#if defined(RTABMAP_G2O) || defined(RTABMAP_ORB_SLAM)
    RTABMAP_PARAM(Vis, PnPRefineIterations,      int, 0,        uFormat("[%s = 1] Refine iterations. Set to 0 if \"%s\" is also used.", kVisEstimationType().c_str(), kVisBundleAdjustment().c_str()));
#else
    RTABMAP_PARAM(Vis, PnPRefineIterations,      int, 1,        uFormat("[%s = 1] Refine iterations. Set to 0 if \"%s\" is also used.", kVisEstimationType().c_str(), kVisBundleAdjustment().c_str()));
#endif
    RTABMAP_PARAM(Vis, PnPMaxVariance,           float, 0.0,    uFormat("[%s = 1] Max linear variance between 3D point correspondences after PnP. 0 means disabled.", kVisEstimationType().c_str()));

    RTABMAP_PARAM(Vis, EpipolarGeometryVar,      float, 0.1,    uFormat("[%s = 2] Epipolar geometry maximum variance to accept the transformation.", kVisEstimationType().c_str()));
    RTABMAP_PARAM(Vis, MinInliers,               int, 20,       "Minimum feature correspondences to compute/accept the transformation.");
    RTABMAP_PARAM(Vis, MeanInliersDistance,      float, 0.0,    "Maximum distance (m) of the mean distance of inliers from the camera to accept the transformation. 0 means disabled.");
    RTABMAP_PARAM(Vis, MinInliersDistribution,   float, 0.0,    "Minimum distribution value of the inliers in the image to accept the transformation. The distribution is the second eigen value of the PCA (Principal Component Analysis) on the keypoints of the normalized image [-0.5, 0.5]. The value would be between 0 and 0.5. 0 means disabled.");

    RTABMAP_PARAM(Vis, Iterations,               int, 300,      "Maximum iterations to compute the transform.");
#if CV_MAJOR_VERSION > 2 && !defined(HAVE_OPENCV_XFEATURES2D)
    // OpenCV>2 without xFeatures2D module doesn't have BRIEF
    RTABMAP_PARAM(Vis, FeatureType, int, 8, "0=SURF 1=SIFT 2=ORB 3=FAST/FREAK 4=FAST/BRIEF 5=GFTT/FREAK 6=GFTT/BRIEF 7=BRISK 8=GFTT/ORB 9=KAZE 10=ORB-OCTREE 11=SuperPoint 12=SURF/FREAK 13=GFTT/DAISY 14=SURF/DAISY 15=PyDetector");
#else
    RTABMAP_PARAM(Vis, FeatureType, int, 6, "0=SURF 1=SIFT 2=ORB 3=FAST/FREAK 4=FAST/BRIEF 5=GFTT/FREAK 6=GFTT/BRIEF 7=BRISK 8=GFTT/ORB 9=KAZE 10=ORB-OCTREE 11=SuperPoint 12=SURF/FREAK 13=GFTT/DAISY 14=SURF/DAISY 15=PyDetector");
#endif
    RTABMAP_PARAM(Vis, MaxFeatures,              int, 1000,   "0 no limits.");
    RTABMAP_PARAM(Vis, MaxDepth,                  float, 0,   "Max depth of the features (0 means no limit).");
    RTABMAP_PARAM(Vis, MinDepth,                  float, 0,   "Min depth of the features (0 means no limit).");
    RTABMAP_PARAM(Vis, DepthAsMask,               bool, true, "Use depth image as mask when extracting features.");
    RTABMAP_PARAM_STR(Vis, RoiRatios,      "0.0 0.0 0.0 0.0", "Region of interest ratios [left, right, top, bottom].");
    RTABMAP_PARAM(Vis, SubPixWinSize,            int, 3,      "See cv::cornerSubPix().");
    RTABMAP_PARAM(Vis, SubPixIterations,         int, 0,      "See cv::cornerSubPix(). 0 disables sub pixel refining.");
    RTABMAP_PARAM(Vis, SubPixEps,                float, 0.02, "See cv::cornerSubPix().");
    RTABMAP_PARAM(Vis, GridRows,                 int, 1,      uFormat("Number of rows of the grid used to extract uniformly \"%s / grid cells\" features from each cell.", kVisMaxFeatures().c_str()));
    RTABMAP_PARAM(Vis, GridCols,                 int, 1,      uFormat("Number of columns of the grid used to extract uniformly \"%s / grid cells\" features from each cell.", kVisMaxFeatures().c_str()));
    RTABMAP_PARAM(Vis, CorType,                  int, 0,      "Correspondences computation approach: 0=Features Matching, 1=Optical Flow");
    RTABMAP_PARAM(Vis, CorNNType,                int, 1,    uFormat("[%s=0] kNNFlannNaive=0, kNNFlannKdTree=1, kNNFlannLSH=2, kNNBruteForce=3, kNNBruteForceGPU=4, BruteForceCrossCheck=5, SuperGlue=6, GMS=7. Used for features matching approach.", kVisCorType().c_str()));
    RTABMAP_PARAM(Vis, CorNNDR,                  float, 0.8,  uFormat("[%s=0] NNDR: nearest neighbor distance ratio. Used for knn features matching approach.", kVisCorType().c_str()));
    RTABMAP_PARAM(Vis, CorGuessWinSize,          int, 40,     uFormat("[%s=0] Matching window size (pixels) around projected points when a guess transform is provided to find correspondences. 0 means disabled.", kVisCorType().c_str()));
    RTABMAP_PARAM(Vis, CorGuessMatchToProjection, bool, false, uFormat("[%s=0] Match frame's corners to source's projected points (when guess transform is provided) instead of projected points to frame's corners.", kVisCorType().c_str()));
    RTABMAP_PARAM(Vis, CorFlowWinSize,           int, 16,     uFormat("[%s=1] See cv::calcOpticalFlowPyrLK(). Used for optical flow approach.", kVisCorType().c_str()));
    RTABMAP_PARAM(Vis, CorFlowIterations,        int, 30,     uFormat("[%s=1] See cv::calcOpticalFlowPyrLK(). Used for optical flow approach.", kVisCorType().c_str()));
    RTABMAP_PARAM(Vis, CorFlowEps,               float, 0.01, uFormat("[%s=1] See cv::calcOpticalFlowPyrLK(). Used for optical flow approach.", kVisCorType().c_str()));
    RTABMAP_PARAM(Vis, CorFlowMaxLevel,          int, 3,      uFormat("[%s=1] See cv::calcOpticalFlowPyrLK(). Used for optical flow approach.", kVisCorType().c_str()));
#if defined(RTABMAP_G2O) || defined(RTABMAP_ORB_SLAM)
    RTABMAP_PARAM(Vis, BundleAdjustment,         int, 1,      "Optimization with bundle adjustment: 0=disabled, 1=g2o, 2=cvsba, 3=Ceres.");
#else
    RTABMAP_PARAM(Vis, BundleAdjustment,         int, 0,      "Optimization with bundle adjustment: 0=disabled, 1=g2o, 2=cvsba, 3=Ceres.");
#endif

    // Features matching approaches
    RTABMAP_PARAM_STR(PyMatcher, Path,       "",           "Path to python script file (see available ones in rtabmap/corelib/src/python/*). See the header to see where the script should be copied.");
	RTABMAP_PARAM(PyMatcher, Iterations,     int, 20,      "Sinkhorn iterations. Used by SuperGlue.");
	RTABMAP_PARAM(PyMatcher, Threshold,      float, 0.2,   "Used by SuperGlue.");
	RTABMAP_PARAM(PyMatcher, Cuda,           bool, true,   "Used by SuperGlue.");
	RTABMAP_PARAM_STR(PyMatcher, Model,        "indoor",   "For SuperGlue, set only \"indoor\" or \"outdoor\". For OANet, set path to one of the pth file (e.g., \"OANet/model/gl3d/sift-4000/model_best.pth\").");

	RTABMAP_PARAM(GMS, WithRotation,         bool, false,   "Take rotation transformation into account.");
	RTABMAP_PARAM(GMS, WithScale,            bool, false,   "Take scale transformation into account.");
	RTABMAP_PARAM(GMS, ThresholdFactor,      double, 6.0,   "The higher, the less matches.");

    // ICP registration parameters
#ifdef RTABMAP_POINTMATCHER
    RTABMAP_PARAM(Icp, Strategy,                  int, 1,       "ICP implementation: 0=Point Cloud Library, 1=libpointmatcher, 2=CCCoreLib (CloudCompare).");
#else
    RTABMAP_PARAM(Icp, Strategy,                  int, 0,       "ICP implementation: 0=Point Cloud Library, 1=libpointmatcher, 2=CCCoreLib (CloudCompare).");
#endif
    RTABMAP_PARAM(Icp, MaxTranslation,            float, 0.2,   "Maximum ICP translation correction accepted (m).");
    RTABMAP_PARAM(Icp, MaxRotation,               float, 0.78,  "Maximum ICP rotation correction accepted (rad).");
    RTABMAP_PARAM(Icp, VoxelSize,                 float, 0.05,  "Uniform sampling voxel size (0=disabled).");
    RTABMAP_PARAM(Icp, DownsamplingStep,          int, 1,       "Downsampling step size (1=no sampling). This is done before uniform sampling.");
    RTABMAP_PARAM(Icp, RangeMin,                  float, 0,     "Minimum range filtering (0=disabled).");
    RTABMAP_PARAM(Icp, RangeMax,                  float, 0,     "Maximum range filtering (0=disabled).");
#ifdef RTABMAP_POINTMATCHER
    RTABMAP_PARAM(Icp, MaxCorrespondenceDistance, float, 0.1,   "Max distance for point correspondences.");
#else
    RTABMAP_PARAM(Icp, MaxCorrespondenceDistance, float, 0.05,  "Max distance for point correspondences.");
#endif
    RTABMAP_PARAM(Icp, Iterations,                int, 30,      "Max iterations.");
    RTABMAP_PARAM(Icp, Epsilon,                   float, 0,     "Set the transformation epsilon (maximum allowable difference between two consecutive transformations) in order for an optimization to be considered as having converged to the final solution.");
    RTABMAP_PARAM(Icp, CorrespondenceRatio,       float, 0.1,   "Ratio of matching correspondences to accept the transform.");
    RTABMAP_PARAM(Icp, Force4DoF,                 bool, false,   uFormat("Limit ICP to x, y, z and yaw DoF. Available if %s > 0.", kIcpStrategy().c_str()));
#ifdef RTABMAP_POINTMATCHER
    RTABMAP_PARAM(Icp, PointToPlane,                bool, true,   "Use point to plane ICP.");
#else
    RTABMAP_PARAM(Icp, PointToPlane,                bool, false,  "Use point to plane ICP.");
#endif
    RTABMAP_PARAM(Icp, PointToPlaneK,               int, 5,       "Number of neighbors to compute normals for point to plane if the cloud doesn't have already normals.");
    RTABMAP_PARAM(Icp, PointToPlaneRadius,          float, 0.0,   "Search radius to compute normals for point to plane if the cloud doesn't have already normals.");
    RTABMAP_PARAM(Icp, PointToPlaneGroundNormalsUp, float, 0.0,   "Invert normals on ground if they are pointing down (useful for ring-like 3D LiDARs). 0 means disabled, 1 means only normals perfectly aligned with -z axis. This is only done with 3D scans.");
    RTABMAP_PARAM(Icp, PointToPlaneMinComplexity,   float, 0.02,  uFormat("Minimum structural complexity (0.0=low, 1.0=high) of the scan to do PointToPlane registration, otherwise PointToPoint registration is done instead and strategy from %s is used. This check is done only when %s=true.", kIcpPointToPlaneLowComplexityStrategy().c_str(), kIcpPointToPlane().c_str()));
    RTABMAP_PARAM(Icp, PointToPlaneLowComplexityStrategy, int, 1, uFormat("If structural complexity is below %s: set to 0 to so that the transform is automatically rejected, set to 1 to limit ICP correction in axes with most constraints (e.g., for a corridor-like environment, the resulting transform will be limited in y and yaw, x will taken from the guess), set to 2 to accept \"as is\" the transform computed by PointToPoint.", kIcpPointToPlaneMinComplexity().c_str()));
    RTABMAP_PARAM(Icp, OutlierRatio,                float, 0.85,   uFormat("Outlier ratio used with %s>0. For libpointmatcher, this parameter set TrimmedDistOutlierFilter/ratio for convenience when configuration file is not set. For CCCoreLib, this parameter set the \"finalOverlapRatio\". The value should be between 0 and 1.", kIcpStrategy().c_str()));
    RTABMAP_PARAM_STR(Icp, DebugExportFormat,       "",           "Export scans used for ICP in the specified format (a warning on terminal will be shown with the file paths used). Supported formats are \"pcd\", \"ply\" or \"vtk\". If logger level is debug, from and to scans will stamped, so previous files won't be overwritten.");

    // libpointmatcher
    RTABMAP_PARAM_STR(Icp, PMConfig,             "",            uFormat("Configuration file (*.yaml) used by libpointmatcher. Note that data filters set for libpointmatcher are done after filtering done by rtabmap (i.e., %s, %s), so make sure to disable those in rtabmap if you want to use only those from libpointmatcher. Parameters %s, %s and %s are also ignored if configuration file is set.", kIcpVoxelSize().c_str(), kIcpDownsamplingStep().c_str(), kIcpIterations().c_str(), kIcpEpsilon().c_str(), kIcpMaxCorrespondenceDistance().c_str()).c_str());
    RTABMAP_PARAM(Icp, PMMatcherKnn,             int, 1,        "KDTreeMatcher/knn: number of nearest neighbors to consider it the reference. For convenience when configuration file is not set.");
    RTABMAP_PARAM(Icp, PMMatcherEpsilon,         float, 0.0,    "KDTreeMatcher/epsilon: approximation to use for the nearest-neighbor search. For convenience when configuration file is not set.");
    RTABMAP_PARAM(Icp, PMMatcherIntensity,       bool, false,   uFormat("KDTreeMatcher:  among nearest neighbors, keep only the one with the most similar intensity. This only work with %s>1.", kIcpPMMatcherKnn().c_str()));

    RTABMAP_PARAM(Icp, CCSamplingLimit,          unsigned int, 50000, "Maximum number of points per cloud (they are randomly resampled below this limit otherwise).");
    RTABMAP_PARAM(Icp, CCFilterOutFarthestPoints, bool, false, "If true, the algorithm will automatically ignore farthest points from the reference, for better convergence.");
    RTABMAP_PARAM(Icp, CCMaxFinalRMS,            float, 0.2,   "Maximum final RMS error.");

    // Stereo disparity
    RTABMAP_PARAM(Stereo, WinWidth,              int, 15,       "Window width.");
    RTABMAP_PARAM(Stereo, WinHeight,             int, 3,        "Window height.");
    RTABMAP_PARAM(Stereo, Iterations,            int, 30,       "Maximum iterations.");
    RTABMAP_PARAM(Stereo, MaxLevel,              int, 5,        "Maximum pyramid level.");
    RTABMAP_PARAM(Stereo, MinDisparity,          float, 0.5,    "Minimum disparity.");
    RTABMAP_PARAM(Stereo, MaxDisparity,          float, 128.0,  "Maximum disparity.");
    RTABMAP_PARAM(Stereo, OpticalFlow,           bool, true,    "Use optical flow to find stereo correspondences, otherwise a simple block matching approach is used.");
    RTABMAP_PARAM(Stereo, SSD,                   bool, true,    uFormat("[%s=false] Use Sum of Squared Differences (SSD) window, otherwise Sum of Absolute Differences (SAD) window is used.", kStereoOpticalFlow().c_str()));
    RTABMAP_PARAM(Stereo, Eps,                   double, 0.01,  uFormat("[%s=true] Epsilon stop criterion.", kStereoOpticalFlow().c_str()));

    RTABMAP_PARAM(Stereo, DenseStrategy,         int, 0,  "0=cv::StereoBM, 1=cv::StereoSGBM");

    RTABMAP_PARAM(StereoBM, BlockSize,           int, 15,       "See cv::StereoBM");
    RTABMAP_PARAM(StereoBM, MinDisparity,        int, 0,        "See cv::StereoBM");
    RTABMAP_PARAM(StereoBM, NumDisparities,      int, 128,      "See cv::StereoBM");
    RTABMAP_PARAM(StereoBM, PreFilterSize,       int, 9,        "See cv::StereoBM");
    RTABMAP_PARAM(StereoBM, PreFilterCap,        int, 31,       "See cv::StereoBM");
    RTABMAP_PARAM(StereoBM, UniquenessRatio,     int, 15,       "See cv::StereoBM");
    RTABMAP_PARAM(StereoBM, TextureThreshold,    int, 10,       "See cv::StereoBM");
    RTABMAP_PARAM(StereoBM, SpeckleWindowSize,   int, 100,      "See cv::StereoBM");
    RTABMAP_PARAM(StereoBM, SpeckleRange,        int, 4,        "See cv::StereoBM");
    RTABMAP_PARAM(StereoBM, Disp12MaxDiff,       int, -1,       "See cv::StereoBM");

    RTABMAP_PARAM(StereoSGBM, BlockSize,         int, 15,       "See cv::StereoSGBM");
	RTABMAP_PARAM(StereoSGBM, MinDisparity,      int, 0,        "See cv::StereoSGBM");
	RTABMAP_PARAM(StereoSGBM, NumDisparities,    int, 128,      "See cv::StereoSGBM");
	RTABMAP_PARAM(StereoSGBM, PreFilterCap,      int, 31,       "See cv::StereoSGBM");
	RTABMAP_PARAM(StereoSGBM, UniquenessRatio,   int, 20,       "See cv::StereoSGBM");
	RTABMAP_PARAM(StereoSGBM, SpeckleWindowSize, int, 100,      "See cv::StereoSGBM");
	RTABMAP_PARAM(StereoSGBM, SpeckleRange,      int, 4,        "See cv::StereoSGBM");
	RTABMAP_PARAM(StereoSGBM, Disp12MaxDiff,     int, 1,        "See cv::StereoSGBM");
	RTABMAP_PARAM(StereoSGBM, P1,                int, 2,        "See cv::StereoSGBM");
	RTABMAP_PARAM(StereoSGBM, P2,                int, 5,        "See cv::StereoSGBM");
#if CV_MAJOR_VERSION < 3
	RTABMAP_PARAM(StereoSGBM, Mode,              int, 0,        "See cv::StereoSGBM");
#else
	RTABMAP_PARAM(StereoSGBM, Mode,              int, 2,        "See cv::StereoSGBM");
#endif

    // Occupancy Grid
    RTABMAP_PARAM(Grid, Sensor,                  int,    1,       "Create occupancy grid from selected sensor: 0=laser scan, 1=depth image(s) or 2=both laser scan and depth image(s).");
    RTABMAP_PARAM(Grid, DepthDecimation,         unsigned int,  4, uFormat("[%s=true] Decimation of the depth image before creating cloud.", kGridDepthDecimation().c_str()));
    RTABMAP_PARAM(Grid, RangeMin,                float,  0.0,     "Minimum range from sensor.");
    RTABMAP_PARAM(Grid, RangeMax,                float,  5.0,     "Maximum range from sensor. 0=inf.");
    RTABMAP_PARAM_STR(Grid, DepthRoiRatios,      "0.0 0.0 0.0 0.0", uFormat("[%s>=1] Region of interest ratios [left, right, top, bottom].", kGridSensor().c_str()));
    RTABMAP_PARAM(Grid, FootprintLength,         float,  0.0,     "Footprint length used to filter points over the footprint of the robot.");
    RTABMAP_PARAM(Grid, FootprintWidth,          float,  0.0,     "Footprint width used to filter points over the footprint of the robot. Footprint length should be set.");
    RTABMAP_PARAM(Grid, FootprintHeight,         float,  0.0,     "Footprint height used to filter points over the footprint of the robot. Footprint length and width should be set.");
    RTABMAP_PARAM(Grid, ScanDecimation,          int,    1,       uFormat("[%s=0 or 2] Decimation of the laser scan before creating cloud.", kGridSensor().c_str()));
    RTABMAP_PARAM(Grid, CellSize,                float,  0.05,    "Resolution of the occupancy grid.");
    RTABMAP_PARAM(Grid, PreVoxelFiltering,       bool,   true,    uFormat("Input cloud is downsampled by voxel filter (voxel size is \"%s\") before doing segmentation of obstacles and ground.", kGridCellSize().c_str()));
    RTABMAP_PARAM(Grid, MapFrameProjection,      bool,   false,   "Projection in map frame. On a 3D terrain and a fixed local camera transform (the cloud is created relative to ground), you may want to disable this to do the projection in robot frame instead.");
    RTABMAP_PARAM(Grid, NormalsSegmentation,     bool,   true,    "Segment ground from obstacles using point normals, otherwise a fast passthrough is used.");
    RTABMAP_PARAM(Grid, MaxObstacleHeight,       float,  0.0,     "Maximum obstacles height (0=disabled).");
    RTABMAP_PARAM(Grid, MinGroundHeight,         float,  0.0,     "Minimum ground height (0=disabled).");
    RTABMAP_PARAM(Grid, MaxGroundHeight,         float,  0.0,     uFormat("Maximum ground height (0=disabled). Should be set if \"%s\" is false.", kGridNormalsSegmentation().c_str()));
    RTABMAP_PARAM(Grid, MaxGroundAngle,          float,  45,      uFormat("[%s=true] Maximum angle (degrees) between point's normal to ground's normal to label it as ground. Points with higher angle difference are considered as obstacles.", kGridNormalsSegmentation().c_str()));
    RTABMAP_PARAM(Grid, NormalK,                 int,    20,      uFormat("[%s=true] K neighbors to compute normals.", kGridNormalsSegmentation().c_str()));
    RTABMAP_PARAM(Grid, ClusterRadius,           float,  0.1,     uFormat("[%s=true] Cluster maximum radius.", kGridNormalsSegmentation().c_str()));
    RTABMAP_PARAM(Grid, MinClusterSize,          int,    10,      uFormat("[%s=true] Minimum cluster size to project the points.", kGridNormalsSegmentation().c_str()));
    RTABMAP_PARAM(Grid, FlatObstacleDetected,    bool,   true,    uFormat("[%s=true] Flat obstacles detected.", kGridNormalsSegmentation().c_str()));
#ifdef RTABMAP_OCTOMAP
    RTABMAP_PARAM(Grid, 3D,                      bool,   true,    uFormat("A 3D occupancy grid is required if you want an OctoMap (3D ray tracing). Set to false if you want only a 2D map, the cloud will be projected on xy plane. A 2D map can be still generated if checked, but it requires more memory and time to generate it. Ignored if laser scan is 2D and \"%s\" is 0.", kGridSensor().c_str()));
#else
    RTABMAP_PARAM(Grid, 3D,                      bool,   false,   uFormat("A 3D occupancy grid is required if you want an OctoMap (3D ray tracing). Set to false if you want only a 2D map, the cloud will be projected on xy plane. A 2D map can be still generated if checked, but it requires more memory and time to generate it. Ignored if laser scan is 2D and \"%s\" is 0.", kGridSensor().c_str()));
#endif
    RTABMAP_PARAM(Grid, GroundIsObstacle,           bool,   false,   uFormat("[%s=true] Ground segmentation (%s) is ignored, all points are obstacles. Use this only if you want an OctoMap with ground identified as an obstacle (e.g., with an UAV).", kGrid3D().c_str(), kGridNormalsSegmentation().c_str()));
    RTABMAP_PARAM(Grid, NoiseFilteringRadius,       float,   0.0,    "Noise filtering radius (0=disabled). Done after segmentation.");
    RTABMAP_PARAM(Grid, NoiseFilteringMinNeighbors, int,     5,      "Noise filtering minimum neighbors.");
    RTABMAP_PARAM(Grid, Scan2dUnknownSpaceFilled,   bool,    false,  uFormat("Unknown space filled. Only used with 2D laser scans. Use %s to set maximum range if laser scan max range is to set.", kGridRangeMax().c_str()));
    RTABMAP_PARAM(Grid, RayTracing,                 bool,   false,   uFormat("Ray tracing is done for each occupied cell, filling unknown space between the sensor and occupied cells. If %s=true, RTAB-Map should be built with OctoMap support, otherwise 3D ray tracing is ignored.", kGrid3D().c_str()));

    RTABMAP_PARAM(GridGlobal, FullUpdate,           bool,   true,    "When the graph is changed, the whole map will be reconstructed instead of moving individually each cells of the map. Also, data added to cache won't be released after updating the map. This process is longer but more robust to drift that would erase some parts of the map when it should not.");
    RTABMAP_PARAM(GridGlobal, UpdateError,          float,  0.01,    "Graph changed detection error (m). Update map only if poses in new optimized graph have moved more than this value.");
    RTABMAP_PARAM(GridGlobal, FootprintRadius,      float,  0.0,     "Footprint radius (m) used to clear all obstacles under the graph.");
    RTABMAP_PARAM(GridGlobal, MinSize,              float,  0.0,     "Minimum map size (m).");
    RTABMAP_PARAM(GridGlobal, Eroded,               bool,   false,   "Erode obstacle cells.");
    RTABMAP_PARAM(GridGlobal, MaxNodes,             int,    0,       "Maximum nodes assembled in the map starting from the last node (0=unlimited).");
    RTABMAP_PARAM(GridGlobal, AltitudeDelta,        float,  0,       "Assemble only nodes that have the same altitude of +-delta meters of the current pose (0=disabled). This is used to generate 2D occupancy grid based on the current altitude (e.g., multi-floor building).");
    RTABMAP_PARAM(GridGlobal, OccupancyThr,         float,  0.5,     "Occupancy threshold (value between 0 and 1).");
    RTABMAP_PARAM(GridGlobal, ProbHit,              float,  0.7,     "Probability of a hit (value between 0.5 and 1).");
    RTABMAP_PARAM(GridGlobal, ProbMiss,             float,  0.4,     "Probability of a miss (value between 0 and 0.5).");
    RTABMAP_PARAM(GridGlobal, ProbClampingMin,      float,  0.1192,  "Probability clamping minimum (value between 0 and 1).");
    RTABMAP_PARAM(GridGlobal, ProbClampingMax,      float,  0.971,   "Probability clamping maximum (value between 0 and 1).");
    RTABMAP_PARAM(GridGlobal, FloodFillDepth,       unsigned int, 0, "Flood fill filter (0=disabled), used to remove empty cells outside the map. The flood fill is done at the specified depth (between 1 and 16) of the OctoMap.");

    RTABMAP_PARAM(Marker, Dictionary,             int,   0,     "Dictionary to use: DICT_ARUCO_4X4_50=0, DICT_ARUCO_4X4_100=1, DICT_ARUCO_4X4_250=2, DICT_ARUCO_4X4_1000=3, DICT_ARUCO_5X5_50=4, DICT_ARUCO_5X5_100=5, DICT_ARUCO_5X5_250=6, DICT_ARUCO_5X5_1000=7, DICT_ARUCO_6X6_50=8, DICT_ARUCO_6X6_100=9, DICT_ARUCO_6X6_250=10, DICT_ARUCO_6X6_1000=11, DICT_ARUCO_7X7_50=12, DICT_ARUCO_7X7_100=13, DICT_ARUCO_7X7_250=14, DICT_ARUCO_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16, DICT_APRILTAG_16h5=17, DICT_APRILTAG_25h9=18, DICT_APRILTAG_36h10=19, DICT_APRILTAG_36h11=20");
    RTABMAP_PARAM(Marker, Length,                 float, 0,     "The length (m) of the markers' side. 0 means automatic marker length estimation using the depth image (the camera should look at the marker perpendicularly for initialization).");
    RTABMAP_PARAM(Marker, MaxDepthError,          float, 0.01,  uFormat("Maximum depth error between all corners of a marker when estimating the marker length (when %s is 0). The smaller it is, the more perpendicular the camera should be toward the marker to initialize the length.", kMarkerLength().c_str()));
    RTABMAP_PARAM(Marker, VarianceLinear,         float, 0.001, "Linear variance to set on marker detections.");
    RTABMAP_PARAM(Marker, VarianceAngular,        float, 0.01,  "Angular variance to set on marker detections. Set to >=9999 to use only position (xyz) constraint in graph optimization.");
    RTABMAP_PARAM(Marker, CornerRefinementMethod, int,   0,     "Corner refinement method (0: None, 1: Subpixel, 2:contour, 3: AprilTag2). For OpenCV <3.3.0, this is \"doCornerRefinement\" parameter: set 0 for false and 1 for true.");
    RTABMAP_PARAM(Marker, MaxRange,               float, 0.0,   "Maximum range in which markers will be detected. <=0 for unlimited range.");
    RTABMAP_PARAM(Marker, MinRange,               float, 0.0,   "Miniminum range in which markers will be detected. <=0 for unlimited range.");
    RTABMAP_PARAM_STR(Marker, Priors,             "",           "World prior locations of the markers. The map will be transformed in marker's world frame when a tag is detected. Format is the marker's ID followed by its position (angles in rad), markers are separated by vertical line (\"id1 x y z roll pitch yaw|id2 x y z roll pitch yaw\"). Example:  \"1 0 0 1 0 0 0|2 1 0 1 0 0 1.57\" (marker 2 is 1 meter forward than marker 1 with 90 deg yaw rotation).");
    RTABMAP_PARAM(Marker, PriorsVarianceLinear,   float, 0.001, "Linear variance to set on marker priors.");
    RTABMAP_PARAM(Marker, PriorsVarianceAngular,  float, 0.001, "Angular variance to set on marker priors.");

    RTABMAP_PARAM(ImuFilter, MadgwickGain,                  double, 0.1,  "Gain of the filter. Higher values lead to faster convergence but more noise. Lower values lead to slower convergence but smoother signal, belongs in [0, 1].");
    RTABMAP_PARAM(ImuFilter, MadgwickZeta,                  double, 0.0,  "Gyro drift gain (approx. rad/s), belongs in [-1, 1].");

    RTABMAP_PARAM(ImuFilter, ComplementaryGainAcc,          double, 0.01, "Gain parameter for the complementary filter, belongs in [0, 1].");
    RTABMAP_PARAM(ImuFilter, ComplementaryBiasAlpha,        double, 0.01, "Bias estimation gain parameter, belongs in [0, 1].");
    RTABMAP_PARAM(ImuFilter, ComplementaryDoBiasEstimation, bool,   true, "Parameter whether to do bias estimation or not.");
    RTABMAP_PARAM(ImuFilter, ComplementaryDoAdpativeGain,   bool,   true, "Parameter whether to do adaptive gain or not.");

public:
    virtual ~Parameters();

    /**
     * Get default parameters
     *
     */
    static const ParametersMap & getDefaultParameters()
    {
        return parameters_;
    }

    /**
     * Get parameter type
     *
     */
    static std::string getType(const std::string & paramKey);

    /**
     * Get parameter description
     *
     */
    static std::string getDescription(const std::string & paramKey);

    static bool parse(const ParametersMap & parameters, const std::string & key, bool & value);
    static bool parse(const ParametersMap & parameters, const std::string & key, int & value);
    static bool parse(const ParametersMap & parameters, const std::string & key, unsigned int & value);
    static bool parse(const ParametersMap & parameters, const std::string & key, float & value);
    static bool parse(const ParametersMap & parameters, const std::string & key, double & value);
    static bool parse(const ParametersMap & parameters, const std::string & key, std::string & value);
    static void parse(const ParametersMap & parameters, ParametersMap & parametersOut);

    static const char * showUsage();
    static ParametersMap parseArguments(int argc, char * argv[], bool onlyParameters = false);

    static std::string getVersion();
    static std::string getDefaultDatabaseName();

    static std::string serialize(const ParametersMap & parameters);
    static ParametersMap deserialize(const std::string & parameters);

    static bool isFeatureParameter(const std::string & param);
    static ParametersMap getDefaultOdometryParameters(bool stereo = false, bool vis = true, bool icp = false);
    static ParametersMap getDefaultParameters(const std::string & group);
    /**
     * If remove=false: keep only parameters of the specified group.
     * If remove=true: remove parameters of the specified group.
     */
    static ParametersMap filterParameters(const ParametersMap & parameters, const std::string & group, bool remove = false);

    static void readINI(const std::string & configFile, ParametersMap & parameters, bool modifiedOnly = false);
    static void writeINI(const std::string & configFile, const ParametersMap & parameters);

    /**
     * Get removed parameters (backward compatibility)
     * <OldKeyName, <isEqual, NewKeyName> >, when isEqual=true, the old value can be safely copied to new parameter
     */
    static const std::map<std::string, std::pair<bool, std::string> > & getRemovedParameters();

    /**
     * <NewKeyName, OldKeyName>
     */
    static const ParametersMap & getBackwardCompatibilityMap();

    static std::string createDefaultWorkingDirectory();

private:
    Parameters();

private:
    static ParametersMap parameters_;
    static ParametersMap parametersType_;
    static ParametersMap descriptions_;
    static Parameters instance_;

    static std::map<std::string, std::pair<bool, std::string> > removedParameters_;
    static ParametersMap backwardCompatibilityMap_;
};

}

#endif /* PARAMETERS_H_ */
