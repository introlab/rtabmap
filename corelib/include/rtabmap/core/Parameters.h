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
 * 		//for PARAM(Video, ImageWidth, int, 640), the output will be :
 * 		public:
 * 			static std::string kVideoImageWidth() {return std::string("Video/ImageWidth");}
 * 			static int defaultVideoImageWidth() {return 640;}
 * 		private:
 * 			class DummyVideoImageWidth {
 * 			public:
 * 				DummyVideoImageWidth() {parameters_.insert(ParametersPair("Video/ImageWidth", "640"));}
 * 			};
 * 			DummyVideoImageWidth dummyVideoImageWidth;
 * @endcode
 */
#define RTABMAP_PARAM(PREFIX, NAME, TYPE, DEFAULT_VALUE, DESCRIPTION) \
	public: \
		static std::string k##PREFIX##NAME() {return std::string(#PREFIX "/" #NAME);} \
		static TYPE default##PREFIX##NAME() {return DEFAULT_VALUE;} \
		static std::string type##PREFIX##NAME() {return std::string(#TYPE);} \
	private: \
		class Dummy##PREFIX##NAME { \
		public: \
			Dummy##PREFIX##NAME() {parameters_.insert(ParametersPair(#PREFIX "/" #NAME, #DEFAULT_VALUE)); \
								   parametersType_.insert(ParametersPair(#PREFIX "/" #NAME, #TYPE)); \
								   descriptions_.insert(ParametersPair(#PREFIX "/" #NAME, DESCRIPTION));} \
		}; \
		Dummy##PREFIX##NAME dummy##PREFIX##NAME;
// end define PARAM

/**
 * It's the same as the macro PARAM but it should be used for string parameters.
 * Macro used to create parameter's key and default value.
 * This macro must be used only in the Parameters class definition (in this file).
 * They are automatically added to the default parameters map of the class Parameters.
 * Example:
 * @code
 * 		//for PARAM_STR(Video, TextFileName, "Hello_world"), the output will be :
 * 		public:
 * 			static std::string kVideoFileName() {return std::string("Video/FileName");}
 * 			static std::string defaultVideoFileName() {return "Hello_world";}
 * 		private:
 * 			class DummyVideoFileName {
 * 			public:
 * 				DummyVideoFileName() {parameters_.insert(ParametersPair("Video/FileName", "Hello_world"));}
 * 			};
 * 			DummyVideoFileName dummyVideoFileName;
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
		Dummy##PREFIX##NAME dummy##PREFIX##NAME;
// end define PARAM

/**
 * Macro used to create parameter's key and default value.
 * This macro must be used only in the Parameters class definition (in this file).
 * They are automatically added to the default parameters map of the class Parameters.
 * Example:
 * @code
 * 		//for PARAM(Video, ImageWidth, int, 640), the output will be :
 * 		public:
 * 			static std::string kVideoImageWidth() {return std::string("Video/ImageWidth");}
 * 			static int defaultVideoImageWidth() {return 640;}
 * 		private:
 * 			class DummyVideoImageWidth {
 * 			public:
 * 				DummyVideoImageWidth() {parameters_.insert(ParametersPair("Video/ImageWidth", "640"));}
 * 			};
 * 			DummyVideoImageWidth dummyVideoImageWidth;
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
		Dummy##PREFIX##NAME dummy##PREFIX##NAME;
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
 * 		//Defining a parameter in this class with the macro PARAM:
 * 		PARAM(Video, ImageWidth, int, 640);
 *
 * 		// Now from anywhere in the application (Parameters is a singleton)
 * 		int width = Parameters::defaultVideoImageWidth(); // theDefaultValue = 640
 * 		std::string theKey = Parameters::kVideoImageWidth(); // theKey = "Video/ImageWidth"
 * 		std::string strValue = Util::value(Parameters::getDefaultParameters(), theKey); // strValue = "640"
 * @endcode
 * @see getDefaultParameters()
 * TODO Add a detailed example with simple classes
 */
class RTABMAP_EXP Parameters
{
	// Rtabmap parameters
	RTABMAP_PARAM(Rtabmap, VhStrategy, 	                 int, 0,     "None 0, Similarity 1, Epipolar 2.");
	RTABMAP_PARAM(Rtabmap, PublishStats, 	             bool, true, "Publishing statistics.");
	RTABMAP_PARAM(Rtabmap, PublishLastSignature, 	     bool, true, "Publishing last signature.");
	RTABMAP_PARAM(Rtabmap, PublishPdf, 	                 bool, true, "Publishing pdf.");
	RTABMAP_PARAM(Rtabmap, PublishLikelihood, 	         bool, true, "Publishing likelihood.");
	RTABMAP_PARAM(Rtabmap, TimeThr, 		             float, 0,   "Maximum time allowed for the detector (ms) (0 means infinity).");
	RTABMAP_PARAM(Rtabmap, MemoryThr, 		             int, 0, 	 "Maximum signatures in the Working Memory (ms) (0 means infinity).");
	RTABMAP_PARAM(Rtabmap, DetectionRate,                float, 1,   "Detection rate. RTAB-Map will filter input images to satisfy this rate.");
	RTABMAP_PARAM(Rtabmap, ImageBufferSize,              unsigned int, 1, "Data buffer size (0 min inf).");
	RTABMAP_PARAM(Rtabmap, CreateIntermediateNodes,      bool, false, "Create intermediate nodes between loop closure detection. Only used when Rtabmap/DetectionRate>0.");
	RTABMAP_PARAM_STR(Rtabmap, WorkingDirectory,         "", "Working directory.");
	RTABMAP_PARAM(Rtabmap, MaxRetrieved,                 unsigned int, 2, "Maximum locations retrieved at the same time from LTM.");
	RTABMAP_PARAM(Rtabmap, StatisticLogsBufferedInRAM,   bool, true, "Statistic logs buffered in RAM instead of written to hard drive after each iteration.");
	RTABMAP_PARAM(Rtabmap, StatisticLogged,   	         bool, false, "Logging enabled.");
	RTABMAP_PARAM(Rtabmap, StatisticLoggedHeaders,   	 bool, true, "Add column header description to log files.");
	RTABMAP_PARAM(Rtabmap, StartNewMapOnLoopClosure,     bool, false, "Start a new map only if there is a global loop closure with a previous map.");

	// Hypotheses selection
	RTABMAP_PARAM(Rtabmap, LoopThr,    	     float, 0.11, 	"Loop closing threshold.");
	RTABMAP_PARAM(Rtabmap, LoopRatio,    	 float, 0, 	    "The loop closure hypothesis must be over LoopRatio x lastHypothesisValue.");

	// Memory
	RTABMAP_PARAM(Mem, RehearsalSimilarity,     float, 0.6, 	"Rehearsal similarity.");
	RTABMAP_PARAM(Mem, ImageKept, 		        bool, false, 	"Keep raw images in RAM.");
	RTABMAP_PARAM(Mem, BinDataKept, 		    bool, true, 	"Keep binary data in db.");
	RTABMAP_PARAM(Mem, RawDescriptorsKept, 		bool, true, 	"Raw descriptors kept in memory.");
	RTABMAP_PARAM(Mem, MapLabelsAdded, 		    bool, true, 	"Create map labels. The first node of a map will be labelled as \"map#\" where # is the map ID.");
	RTABMAP_PARAM(Mem, SaveDepth16Format, 		bool, false, 	"Save depth image into 16 bits format to reduce memory used. Warning: values over ~65 meters are ignored (maximum 65535 millimeters).");
	RTABMAP_PARAM(Mem, NotLinkedNodesKept, 	    bool, true, 	"Keep not linked nodes in db (rehearsed nodes and deleted nodes).");
	RTABMAP_PARAM(Mem, STMSize, 		        unsigned int, 10, "Short-term memory size.");
	RTABMAP_PARAM(Mem, IncrementalMemory,       bool, true, 	"SLAM mode, otherwise it is Localization mode.");
	RTABMAP_PARAM(Mem, ReduceGraph,             bool, false, 	"Reduce graph. Merge nodes when loop closures are added (ignoring those with user data set).");
	RTABMAP_PARAM(Mem, RecentWmRatio,           float, 0.2, 	"Ratio of locations after the last loop closure in WM that cannot be transferred.");
	RTABMAP_PARAM(Mem, TransferSortingByWeightId, bool, false,  "On transfer, signatures are sorted by weight->ID only (i.e. the oldest of the lowest weighted signatures are transferred first). If false, the signatures are sorted by weight->Age->ID (i.e. the oldest inserted in WM of the lowest weighted signatures are transferred first). Note that retrieval updates the age, not the ID.");
	RTABMAP_PARAM(Mem, RehearsalIdUpdatedToNewOne, bool, false, "On merge, update to new id. When false, no copy.");
	RTABMAP_PARAM(Mem, RehearsalWeightIgnoredWhileMoving, bool, false, "When the robot is moving, weights are not updated on rehearsal.");
	RTABMAP_PARAM(Mem, GenerateIds,             bool, true,     "True=Generate location IDs, False=use input image IDs.");
	RTABMAP_PARAM(Mem, BadSignaturesIgnored,    bool, false,     "Bad signatures are ignored.");
	RTABMAP_PARAM(Mem, InitWMWithAllNodes,      bool, false,     "Initialize the Working Memory with all nodes in Long-Term Memory. When false, it is initialized with nodes of the previous session.");
	RTABMAP_PARAM(Mem, ImagePreDecimation,      int, 1,          "Image decimation (>=1) before features extraction.");
	RTABMAP_PARAM(Mem, ImagePostDecimation,     int, 1,          "Image decimation (>=1) of saved data in created signatures (after features extraction). Decimation is done from the original image.");
	RTABMAP_PARAM(Mem, LaserScanDownsampleStepSize, int, 1,      "If > 1, downsample the laser scans when creating a signature.");
	RTABMAP_PARAM(Mem, UseOdomFeatures,             bool, false, "Use odometry features.");

	// KeypointMemory (Keypoint-based)
	RTABMAP_PARAM(Kp, NNStrategy,            int, 1,            "kNNFlannNaive=0, kNNFlannKdTree=1, kNNFlannLSH=2, kNNBruteForce=3, kNNBruteForceGPU=4");
	RTABMAP_PARAM(Kp, IncrementalDictionary, bool, true, 		"");
	RTABMAP_PARAM(Kp, IncrementalFlann,      bool, true, 		"When using FLANN based strategy, add/remove points to its index without always rebuilding the index (the index is built only when the dictionary doubles in size).");
	RTABMAP_PARAM(Kp, MaxDepth,              float, 0,   		"Filter extracted keypoints by depth (0=inf).");
	RTABMAP_PARAM(Kp, MinDepth,              float, 0,  		"Filter extracted keypoints by depth.");
	RTABMAP_PARAM(Kp, MaxFeatures,           int, 400, 			"Maximum features extracted from the images (0 means not bounded, <0 means no extraction).");
	RTABMAP_PARAM(Kp, BadSignRatio,          float, 0.5, 		"Bad signature ratio (less than Ratio x AverageWordsPerImage = bad).");
	RTABMAP_PARAM(Kp, NndrRatio, 	         float, 0.8,		"NNDR ratio (A matching pair is detected, if its distance is closer than X times the distance of the second nearest neighbor.)");
#ifdef RTABMAP_NONFREE
	RTABMAP_PARAM(Kp, DetectorStrategy,      int, 0,            "0=SURF 1=SIFT 2=ORB 3=FAST/FREAK 4=FAST/BRIEF 5=GFTT/FREAK 6=GFTT/BRIEF 7=BRISK 8=GFTT/ORB.");
#else
	RTABMAP_PARAM(Kp, DetectorStrategy,      int, 2,            "0=SURF 1=SIFT 2=ORB 3=FAST/FREAK 4=FAST/BRIEF 5=GFTT/FREAK 6=GFTT/BRIEF 7=BRISK 8=GFTT/ORB.");
#endif
	RTABMAP_PARAM(Kp, TfIdfLikelihoodUsed,   bool, true, 		"Use of the td-idf strategy to compute the likelihood.");
	RTABMAP_PARAM(Kp, Parallelized,          bool, true, 		"If the dictionary update and signature creation were parallelized.");
	RTABMAP_PARAM_STR(Kp, RoiRatios, "0.0 0.0 0.0 0.0", 		"Region of interest ratios [left, right, top, bottom].");
	RTABMAP_PARAM_STR(Kp, DictionaryPath,    "", 				"Path of the pre-computed dictionary");
	RTABMAP_PARAM(Kp, NewWordsComparedTogether, bool, true,	"When adding new words to dictionary, they are compared also with each other (to detect same words in the same signature).");
    RTABMAP_PARAM(Kp, SubPixWinSize,            int, 3,        "See cv::cornerSubPix().");
	RTABMAP_PARAM(Kp, SubPixIterations,         int, 0,        "See cv::cornerSubPix(). 0 disables sub pixel refining.");
	RTABMAP_PARAM(Kp, SubPixEps,                double, 0.02,  "See cv::cornerSubPix().");

	//Database
	RTABMAP_PARAM(DbSqlite3, InMemory, 	   bool, false, 		"Using database in the memory instead of a file on the hard disk.");
	RTABMAP_PARAM(DbSqlite3, CacheSize,    unsigned int, 10000, "Sqlite cache size (default is 2000).");
	RTABMAP_PARAM(DbSqlite3, JournalMode,  int, 3, 				"0=DELETE, 1=TRUNCATE, 2=PERSIST, 3=MEMORY, 4=OFF (see sqlite3 doc : \"PRAGMA journal_mode\")");
	RTABMAP_PARAM(DbSqlite3, Synchronous,  int, 0, 				"0=OFF, 1=NORMAL, 2=FULL (see sqlite3 doc : \"PRAGMA synchronous\")");
	RTABMAP_PARAM(DbSqlite3, TempStore,    int, 2, 				"0=DEFAULT, 1=FILE, 2=MEMORY (see sqlite3 doc : \"PRAGMA temp_store\")");

	// Keypoints descriptors/detectors
	RTABMAP_PARAM(SURF, Extended, 		  bool, false, 	    "Extended descriptor flag (true - use extended 128-element descriptors; false - use 64-element descriptors).");
	RTABMAP_PARAM(SURF, HessianThreshold, float, 500,		"Threshold for hessian keypoint detector used in SURF.");
	RTABMAP_PARAM(SURF, Octaves, 		  int, 4, 			"Number of pyramid octaves the keypoint detector will use.");
	RTABMAP_PARAM(SURF, OctaveLayers, 	  int, 2, 			"Number of octave layers within each octave.");
	RTABMAP_PARAM(SURF, Upright, 	      bool, false, 	"Up-right or rotated features flag (true - do not compute orientation of features; false - compute orientation).");
	RTABMAP_PARAM(SURF, GpuVersion, 	  bool, false, 	"GPU-SURF: Use GPU version of SURF. This option is enabled only if OpenCV is built with CUDA and GPUs are detected.");
	RTABMAP_PARAM(SURF, GpuKeypointsRatio, 	  float, 0.01, 	"Used with SURF GPU.");

	RTABMAP_PARAM(SIFT, NFeatures,          int, 0,			"The number of best features to retain. The features are ranked by their scores (measured in SIFT algorithm as the local contrast).");
	RTABMAP_PARAM(SIFT, NOctaveLayers,      int, 3, 		"The number of layers in each octave. 3 is the value used in D. Lowe paper. The number of octaves is computed automatically from the image resolution.");
	RTABMAP_PARAM(SIFT, ContrastThreshold, 	double, 0.04, 	"The contrast threshold used to filter out weak features in semi-uniform (low-contrast) regions. The larger the threshold, the less features are produced by the detector.");
	RTABMAP_PARAM(SIFT, EdgeThreshold,      double, 10, 	"The threshold used to filter out edge-like features. Note that the its meaning is different from the contrastThreshold, i.e. the larger the edgeThreshold, the less features are filtered out (more features are retained).");
	RTABMAP_PARAM(SIFT, Sigma,              double, 1.6, 	"The sigma of the Gaussian applied to the input image at the octave #0. If your image is captured with a weak camera with soft lenses, you might want to reduce the number.");

	RTABMAP_PARAM(BRIEF, Bytes,         	   int, 32, 	"Bytes is a length of descriptor in bytes. It can be equal 16, 32 or 64 bytes.");

	RTABMAP_PARAM(FAST, Threshold,          int, 10, 	    "Threshold on difference between intensity of the central pixel and pixels of a circle around this pixel.");
	RTABMAP_PARAM(FAST, NonmaxSuppression,  bool, true, 	"If true, non-maximum suppression is applied to detected corners (keypoints).");
	RTABMAP_PARAM(FAST, Gpu,                bool, false, 	"GPU-FAST: Use GPU version of FAST. This option is enabled only if OpenCV is built with CUDA and GPUs are detected.");
	RTABMAP_PARAM(FAST, GpuKeypointsRatio,  double, 0.05, 	"Used with FAST GPU.");
	RTABMAP_PARAM(FAST, MinThreshold,       int, 1, 	    "Minimum threshold. Used only when FAST/GridRows and FAST/GridCols are set.");
	RTABMAP_PARAM(FAST, MaxThreshold,       int, 200, 	    "Maximum threshold. Used only when FAST/GridRows and FAST/GridCols are set.");
	RTABMAP_PARAM(FAST, GridRows,           int, 4,         "Grid rows (0 to disable). Adapts the detector to partition the source image into a grid and detect points in each cell.");
	RTABMAP_PARAM(FAST, GridCols,           int, 4,         "Grid cols (0 to disable). Adapts the detector to partition the source image into a grid and detect points in each cell.");

	RTABMAP_PARAM(GFTT, QualityLevel, double, 0.001, "");
	RTABMAP_PARAM(GFTT, MinDistance, double, 5, "");
	RTABMAP_PARAM(GFTT, BlockSize, int, 3, "");
	RTABMAP_PARAM(GFTT, UseHarrisDetector, bool, false, "");
	RTABMAP_PARAM(GFTT, K, double, 0.04, "");

	RTABMAP_PARAM(ORB, ScaleFactor,          float,  1.2, "Pyramid decimation ratio, greater than 1. scaleFactor==2 means the classical pyramid, where each next level has 4x less pixels than the previous, but such a big scale factor will degrade feature matching scores dramatically. On the other hand, too close to 1 scale factor will mean that to cover certain scale range you will need more pyramid levels and so the speed will suffer.");
	RTABMAP_PARAM(ORB, NLevels,              int, 8,       "The number of pyramid levels. The smallest level will have linear size equal to input_image_linear_size/pow(scaleFactor, nlevels).");
	RTABMAP_PARAM(ORB, EdgeThreshold,        int, 31,      "This is size of the border where the features are not detected. It should roughly match the patchSize parameter.");
	RTABMAP_PARAM(ORB, FirstLevel,           int, 0,       "It should be 0 in the current implementation.");
	RTABMAP_PARAM(ORB, WTA_K,                int, 2,       "The number of points that produce each element of the oriented BRIEF descriptor. The default value 2 means the BRIEF where we take a random point pair and compare their brightnesses, so we get 0/1 response. Other possible values are 3 and 4. For example, 3 means that we take 3 random points (of course, those point coordinates are random, but they are generated from the pre-defined seed, so each element of BRIEF descriptor is computed deterministically from the pixel rectangle), find point of maximum brightness and output index of the winner (0, 1 or 2). Such output will occupy 2 bits, and therefore it will need a special variant of Hamming distance, denoted as NORM_HAMMING2 (2 bits per bin). When WTA_K=4, we take 4 random points to compute each bin (that will also occupy 2 bits with possible values 0, 1, 2 or 3).");
	RTABMAP_PARAM(ORB, ScoreType,            int, 0,       "The default HARRIS_SCORE=0 means that Harris algorithm is used to rank features (the score is written to KeyPoint::score and is used to retain best nfeatures features); FAST_SCORE=1 is alternative value of the parameter that produces slightly less stable keypoints, but it is a little faster to compute.");
	RTABMAP_PARAM(ORB, PatchSize,            int, 31,      "size of the patch used by the oriented BRIEF descriptor. Of course, on smaller pyramid layers the perceived image area covered by a feature will be larger.");
	RTABMAP_PARAM(ORB, Gpu,                  bool, false, "GPU-ORB: Use GPU version of ORB. This option is enabled only if OpenCV is built with CUDA and GPUs are detected.");

	RTABMAP_PARAM(FREAK, OrientationNormalized, bool, true,   "Enable orientation normalization.");
	RTABMAP_PARAM(FREAK, ScaleNormalized,       bool, true,   "Enable scale normalization.");
	RTABMAP_PARAM(FREAK, PatternScale,          float, 22,    "Scaling of the description pattern.");
	RTABMAP_PARAM(FREAK, NOctaves,              int, 4,        "Number of octaves covered by the detected keypoints.");

	RTABMAP_PARAM(BRISK, Thresh,                int, 30,      "FAST/AGAST detection threshold score.");
	RTABMAP_PARAM(BRISK, Octaves,               int, 3,       "Detection octaves. Use 0 to do single scale.");
	RTABMAP_PARAM(BRISK, PatternScale,          float, 1,    "Apply this scale to the pattern used for sampling the neighbourhood of a keypoint.");

	// BayesFilter
	RTABMAP_PARAM(Bayes, VirtualPlacePriorThr,           float, 0.9, "Virtual place prior");
	RTABMAP_PARAM_STR(Bayes, PredictionLC, "0.1 0.36 0.30 0.16 0.062 0.0151 0.00255 0.000324 2.5e-05 1.3e-06 4.8e-08 1.2e-09 1.9e-11 2.2e-13 1.7e-15 8.5e-18 2.9e-20 6.9e-23", "Prediction of loop closures (Gaussian-like, here with sigma=1.6) - Format: {VirtualPlaceProb, LoopClosureProb, NeighborLvl1, NeighborLvl2, ...}.");
	RTABMAP_PARAM(Bayes, FullPredictionUpdate,           bool, false, "Regenerate all the prediction matrix on each iteration (otherwise only removed/added ids are updated).");

	// Verify hypotheses
	RTABMAP_PARAM(VhEp, MatchCountMin, int, 8, 		"Minimum of matching visual words pairs to accept the loop hypothesis.");
	RTABMAP_PARAM(VhEp, RansacParam1,  float, 3, 	"Fundamental matrix (see cvFindFundamentalMat()): Max distance (in pixels) from the epipolar line for a point to be inlier.");
	RTABMAP_PARAM(VhEp, RansacParam2,  float, 0.99, "Fundamental matrix (see cvFindFundamentalMat()): Performance of the RANSAC.");

	// RGB-D SLAM
	RTABMAP_PARAM(RGBD, Enabled,                  bool, true,  "");
	RTABMAP_PARAM(RGBD, LinearUpdate,             float, 0.1,  "Minimum linear displacement to update the map. Rehearsal is done prior to this, so weights are still updated.");
	RTABMAP_PARAM(RGBD, AngularUpdate,            float, 0.1,  "Minimum angular displacement to update the map. Rehearsal is done prior to this, so weights are still updated.");
	RTABMAP_PARAM(RGBD, NewMapOdomChangeDistance, float, 0,    "A new map is created if a change of odometry translation greater than X m is detected (0 m = disabled).");
	RTABMAP_PARAM(RGBD, OptimizeFromGraphEnd,     bool, false, "Optimize graph from the newest node. If false, the graph is optimized from the oldest node of the current graph (this adds an overhead computation to detect to oldest mode of the current graph, but it can be useful to preserve the map referential from the oldest node). Warning when set to false: when some nodes are transferred, the first referential of the local map may change, resulting in momentary changes in robot/map position (which are annoying in teleoperation).");
	RTABMAP_PARAM(RGBD, OptimizeMaxError,         float, 1,    "Reject loop closures if optimization error is greater than this value (0=disabled). This will help to detect when a wrong loop closure is added to the graph. Not compatible with \"Optimizer/Robust\" if enabled.");
	RTABMAP_PARAM(RGBD, GoalReachedRadius,        float, 0.5,  "Goal reached radius (m).");
	RTABMAP_PARAM(RGBD, PlanStuckIterations,      int, 0,      "Mark the current goal node on the path as unreachable if it is not updated after X iterations (0=disabled). If all upcoming nodes on the path are unreachabled, the plan fails.");
	RTABMAP_PARAM(RGBD, PlanLinearVelocity,       float, 0,    "Linear velocity (m/sec) used to compute path weights.");
	RTABMAP_PARAM(RGBD, PlanAngularVelocity,      float, 0,    "Angular velocity (rad/sec) used to compute path weights.");
	RTABMAP_PARAM(RGBD, GoalsSavedInUserData,     bool, false, "When a goal is received and processed with success, it is saved in user data of the location with this format: \"GOAL:#\".");
	RTABMAP_PARAM(RGBD, MaxLocalRetrieved,        unsigned int, 2, "Maximum local locations retrieved (0=disabled) near the current pose in the local map or on the current planned path (those on the planned path have priority).");
	RTABMAP_PARAM(RGBD, LocalRadius,              float, 10,   "Local radius (m) for nodes selection in the local map. This parameter is used in some approaches about the local map management.");
	RTABMAP_PARAM(RGBD, LocalImmunizationRatio,   float, 0.25, "Ratio of working memory for which local nodes are immunized from transfer.");
	RTABMAP_PARAM(RGBD, ScanMatchingIdsSavedInLinks, bool, true,      "Save scan matching IDs in link's user data.");
	RTABMAP_PARAM(RGBD, NeighborLinkRefining,          bool, false,  "When a new node is added to the graph, the transformation of its neighbor link to the previous node is refined using ICP (laser scans required!).");
	RTABMAP_PARAM(RGBD, LoopClosureReextractFeatures,  bool, false,  "Extract features even if there are some already in the nodes.");

	// Local/Proximity loop closure detection
	RTABMAP_PARAM(RGBD, ProximityByTime,              bool, false, 	"Detection over all locations in STM.");
	RTABMAP_PARAM(RGBD, ProximityBySpace,             bool, true, 	"Detection over locations (in Working Memory or STM) near in space.");
	RTABMAP_PARAM(RGBD, ProximityMaxGraphDepth,       int, 50,      "Maximum depth from the current/last loop closure location and the local loop closure hypotheses. Set 0 to ignore.");
	RTABMAP_PARAM(RGBD, ProximityPathFilteringRadius, float, 0.5,   "Path filtering radius.");
	RTABMAP_PARAM(RGBD, ProximityPathRawPosesUsed,    bool, true,   "When comparing to a local path, merge the scan using the odometry poses (with neighbor link optimizations) instead of the ones in the optimized local graph.");
	RTABMAP_PARAM(RGBD, ProximityAngle,               float, 45,    "Maximum angle (degrees) for visual proximity detection.");

	// Graph optimization
#ifdef RTABMAP_GTSAM
	RTABMAP_PARAM(Optimizer, Strategy,          int, 2,          "Graph optimization strategy: 0=TORO, 1=g2o and 2=GTSAM.");
#else
#ifdef RTABMAP_G2O
	RTABMAP_PARAM(Optimizer, Strategy,          int, 1,          "Graph optimization strategy: 0=TORO, 1=g2o and 2=GTSAM.");
#else
	RTABMAP_PARAM(Optimizer, Strategy,          int, 0,          "Graph optimization strategy: 0=TORO, 1=g2o and 2=GTSAM.");
#endif
#endif
	RTABMAP_PARAM(Optimizer, Iterations,        int, 100,        "Optimization iterations.");
	RTABMAP_PARAM(Optimizer, Slam2D,            bool, false,     "If optimization is done only on x,y and theta (3DoF). Otherwise, it is done on full 6DoF poses.");
	RTABMAP_PARAM(Optimizer, VarianceIgnored,   bool, false,     "Ignore constraints' variance. If checked, identity information matrix is used for each constraint. Otherwise, an information matrix is generated from the variance saved in the links.");
	RTABMAP_PARAM(Optimizer, Epsilon,           double, 0.0001,  "Stop optimizing when the error improvement is less than this value.");
	RTABMAP_PARAM(Optimizer, Robust,            bool, false,      "Robust graph optimization using Vertigo (only work for g2o and GTSAM optimization strategies). Not compatible with \"RGBD/OptimizeMaxError\" if enabled.");

	RTABMAP_PARAM(g2o, Solver,                  int, 0,          "0=csparse 1=pcg 2=cholmod");
	RTABMAP_PARAM(g2o, Optimizer,               int, 0,          "0=Levenberg 1=GaussNewton");
	RTABMAP_PARAM(g2o, PixelVariance,           double, 1,       "Pixel variance used for SBA.");

	// Odometry
	RTABMAP_PARAM(Odom, Strategy,           	int, 0, 		"0=Frame-to-Map (F2M) 1=Frame-to-Frame (F2F)");
	RTABMAP_PARAM(Odom, ResetCountdown,         int, 0,         "Automatically reset odometry after X consecutive images on which odometry cannot be computed (value=0 disables auto-reset).");
	RTABMAP_PARAM(Odom, Holonomic, 		        bool, true,     "If the robot is holonomic (strafing commands can be issued). If not, y value will be estimated from x and yaw values (y=x*tan(yaw)).");
	RTABMAP_PARAM(Odom, FillInfoData, 		    bool, true,     "Fill info with data (inliers/outliers features).");
	RTABMAP_PARAM(Odom, ImageBufferSize,        unsigned int, 1, "Data buffer size (0 min inf).");
	RTABMAP_PARAM(Odom, FilteringStrategy, 		int, 0,         "0=No filtering 1=Kalman filtering 2=Particle filtering");
	RTABMAP_PARAM(Odom, ParticleSize, 		    unsigned int, 400,  "Number of particles of the filter.");
	RTABMAP_PARAM(Odom, ParticleNoiseT, 		float, 0.002,     "Noise (m) of translation components (x,y,z).");
	RTABMAP_PARAM(Odom, ParticleLambdaT, 		float, 100,       "Lambda of translation components (x,y,z).");
	RTABMAP_PARAM(Odom, ParticleNoiseR, 		float, 0.002,     "Noise (rad) of rotational components (roll,pitch,yaw).");
	RTABMAP_PARAM(Odom, ParticleLambdaR, 		float, 100,       "Lambda of rotational components (roll,pitch,yaw).");
	RTABMAP_PARAM(Odom, KalmanProcessNoise, 	float, 0.001,     "Process noise covariance value.");
	RTABMAP_PARAM(Odom, KalmanMeasurementNoise, float, 0.01,      "Process measurement covariance value.");
	RTABMAP_PARAM(Odom, GuessMotion,            bool, false,      "Guess next transformation from the last motion computed.");
	RTABMAP_PARAM(Odom, KeyFrameThr,            float, 0.3,       "[Visual] Create a new keyframe when the number of inliers drops under this ratio of features in last frame. Setting the value to 0 means that a keyframe is created for each processed frame.");
	RTABMAP_PARAM(Odom, ScanKeyFrameThr,        float, 0.7,       "[Geometry] Create a new keyframe when the number of ICP inliers drops under this ratio of points in last frame's scan. Setting the value to 0 means that a keyframe is created for each processed frame.");
	RTABMAP_PARAM(Odom, ImageDecimation,        int, 1,           "Decimation of the images before registration.");
	RTABMAP_PARAM(Odom, AlignWithGround,        bool, false,      "Align odometry with the ground on initialization.");

	// Odometry Bag-of-words
	RTABMAP_PARAM(OdomF2M, MaxSize,             int, 2000,    "[Visual] Local map size: If > 0 (example 5000), the odometry will maintain a local map of X maximum words.");
	RTABMAP_PARAM(OdomF2M, MaxNewFeatures,      int, 0,       "[Visual] Maximum features (sorted by keypoint response) added to local map from a new key-frame. 0 means no limit.");
	RTABMAP_PARAM(OdomF2M, ScanMaxSize,         int, 2000,    "[Geometry] Maximum local scan map size.");
	RTABMAP_PARAM(OdomF2M, ScanSubtractRadius,  float, 0.05,  "[Geometry] Radius used to filter points of a new added scan to local map. This could match the voxel size of the scans.");
	RTABMAP_PARAM_STR(OdomF2M, FixedMapPath,    "",           "Path to a fixed map (RTAB-Map's database) to be used for odometry. Odometry will be constraint to this map. RGB-only images can be used if odometry PnP estimation is used.")

	// Odometry Mono
	RTABMAP_PARAM(OdomMono, InitMinFlow,            float, 100,  "Minimum optical flow required for the initialization step.");
	RTABMAP_PARAM(OdomMono, InitMinTranslation,     float, 0.1,  "Minimum translation required for the initialization step.");
	RTABMAP_PARAM(OdomMono, MinTranslation,         float, 0.02, "Minimum translation to add new points to local map. On initialization, translation x 5 is used as the minimum.");
	RTABMAP_PARAM(OdomMono, MaxVariance,            float, 0.01, "Maximum variance to add new points to local map.");

	// Common registration parameters
	RTABMAP_PARAM(Reg, VarianceFromInliersCount, bool, false,   "Set variance as the inverse of the number of inliers. Otherwise, the variance is computed as the average 3D position error of the inliers.");
	RTABMAP_PARAM(Reg, Strategy,                 int, 0,        "0=Vis, 1=Icp, 2=VisIcp");
	RTABMAP_PARAM(Reg, Force3DoF,                bool, false,   "Force 3 degrees-of-freedom transform (3Dof: x,y and yaw). Parameters z, roll and pitch will be set to 0.");
	
	// Visual registration parameters
	RTABMAP_PARAM(Vis, EstimationType,           int, 0,    	"Motion estimation approach: 0:3D->3D, 1:3D->2D (PnP), 2:2D->2D (Epipolar Geometry)");
	RTABMAP_PARAM(Vis, ForwardEstOnly,           bool, true, 	"Forward estimation only (A->B). If false, a transformation is also computed in backward direction (B->A), then the two resulting transforms are merged (middle interpolation between the transforms).");
	RTABMAP_PARAM(Vis, InlierDistance,           float, 0.1,    "[Vis/EstimationType = 0] Maximum distance for feature correspondences. Used by 3D->3D estimation approach.");
	RTABMAP_PARAM(Vis, RefineIterations,         int, 5,        "[Vis/EstimationType = 0] Number of iterations used to refine the transformation found by RANSAC. 0 means that the transformation is not refined.");
	RTABMAP_PARAM(Vis, PnPReprojError, 	         float, 2,      "[Vis/EstimationType = 1] PnP reprojection error.");
	RTABMAP_PARAM(Vis, PnPFlags,                 int, 1,        "[Vis/EstimationType = 1] PnP flags: 0=Iterative, 1=EPNP, 2=P3P");
	RTABMAP_PARAM(Vis, PnPRefineIterations,      int, 1,        "[Vis/EstimationType = 1] Refine iterations.");
	RTABMAP_PARAM(Vis, EpipolarGeometryVar,      float, 0.02,   "[Vis/EstimationType = 2] Epipolar geometry maximum variance to accept the transformation.");
	RTABMAP_PARAM(Vis, MinInliers,               int, 20, 		"Minimum feature correspondences to compute/accept the transformation.");
	RTABMAP_PARAM(Vis, Iterations,               int, 100, 		"Maximum iterations to compute the transform.");
#ifndef RTABMAP_NONFREE
#ifdef RTABMAP_OPENCV3
	// OpenCV 3 without xFeatures2D module doesn't have BRIEF
	RTABMAP_PARAM(Vis, FeatureType, int, 8, "0=SURF 1=SIFT 2=ORB 3=FAST/FREAK 4=FAST/BRIEF 5=GFTT/FREAK 6=GFTT/BRIEF 7=BRISK 8=GFTT/ORB.");
#else
	RTABMAP_PARAM(Vis, FeatureType, int, 6, "0=SURF 1=SIFT 2=ORB 3=FAST/FREAK 4=FAST/BRIEF 5=GFTT/FREAK 6=GFTT/BRIEF 7=BRISK 8=GFTT/ORB.");
#endif
#else
	RTABMAP_PARAM(Vis, FeatureType, int, 6, "0=SURF 1=SIFT 2=ORB 3=FAST/FREAK 4=FAST/BRIEF 5=GFTT/FREAK 6=GFTT/BRIEF 7=BRISK 8=GFTT/ORB.");
#endif
	
	RTABMAP_PARAM(Vis, MaxFeatures,              int, 1000,     "0 no limits.");
	RTABMAP_PARAM(Vis, MaxDepth, 	             float, 0,      "Max depth of the features (0 means no limit).");
	RTABMAP_PARAM(Vis, MinDepth, 	             float, 0,      "Min depth of the features (0 means no limit).");
	RTABMAP_PARAM_STR(Vis, RoiRatios,        "0.0 0.0 0.0 0.0", "Region of interest ratios [left, right, top, bottom].");
    RTABMAP_PARAM(Vis, SubPixWinSize,            int, 3,        "See cv::cornerSubPix().");
	RTABMAP_PARAM(Vis, SubPixIterations,         int, 0,        "See cv::cornerSubPix(). 0 disables sub pixel refining.");
	RTABMAP_PARAM(Vis, SubPixEps,                float, 0.02,   "See cv::cornerSubPix().");
	RTABMAP_PARAM(Vis, CorType,                  int, 0,        "Correspondences computation approach: 0=Features Matching, 1=Optical Flow");
	RTABMAP_PARAM(Vis, CorNNType, 	             int, 1,        "[Vis/CorrespondenceType=0] kNNFlannNaive=0, kNNFlannKdTree=1, kNNFlannLSH=2, kNNBruteForce=3, kNNBruteForceGPU=4. Used for features matching approach.");
	RTABMAP_PARAM(Vis, CorNNDR,                  float, 0.8,    "[Vis/CorrespondenceType=0] NNDR: nearest neighbor distance ratio. Used for features matching approach.");
	RTABMAP_PARAM(Vis, CorGuessWinSize,          int, 50,       "[Vis/CorrespondenceType=0] Matching window size (pixels) around projected points when a guess transform is provided to find correspondences. 0 means disabled.");
	RTABMAP_PARAM(Vis, CorFlowWinSize,           int, 16,       "[Vis/CorrespondenceType=1] See cv::calcOpticalFlowPyrLK(). Used for optical flow approach.");
	RTABMAP_PARAM(Vis, CorFlowIterations,        int, 30,       "[Vis/CorrespondenceType=1] See cv::calcOpticalFlowPyrLK(). Used for optical flow approach.");
	RTABMAP_PARAM(Vis, CorFlowEps,               float, 0.01,   "[Vis/CorrespondenceType=1] See cv::calcOpticalFlowPyrLK(). Used for optical flow approach.");
	RTABMAP_PARAM(Vis, CorFlowMaxLevel,          int, 3,        "[Vis/CorrespondenceType=1] See cv::calcOpticalFlowPyrLK(). Used for optical flow approach.");

	// ICP registration parameters
	RTABMAP_PARAM(Icp, MaxTranslation,            float, 0.2,   "Maximum ICP translation correction accepted (m).");
	RTABMAP_PARAM(Icp, MaxRotation,               float, 0.78,  "Maximum ICP rotation correction accepted (rad).");
	RTABMAP_PARAM(Icp, VoxelSize,                 float, 0.025, "Uniform sampling voxel size (0=disabled).");
	RTABMAP_PARAM(Icp, DownsamplingStep,          int, 1, 	    "Downsampling step size (1=no sampling). This is done before uniform sampling.");
	RTABMAP_PARAM(Icp, MaxCorrespondenceDistance, float, 0.05,  "Max distance for point correspondences.");
	RTABMAP_PARAM(Icp, Iterations,                int, 30, 		"Max iterations.");
	RTABMAP_PARAM(Icp, Epsilon,                   float, 0,     "Set the transformation epsilon (maximum allowable difference between two consecutive transformations) in order for an optimization to be considered as having converged to the final solution.");
	RTABMAP_PARAM(Icp, CorrespondenceRatio,       float, 0.2,   "Ratio of matching correspondences to accept the transform.");
	RTABMAP_PARAM(Icp, PointToPlane,              bool, false, 	"Use point to plane ICP.");
	RTABMAP_PARAM(Icp, PointToPlaneNormalNeighbors, int, 20,    "Number of neighbors to compute normals for point to plane.");

	// Stereo disparity
	RTABMAP_PARAM(Stereo, WinWidth,              int, 15,       "Window width.");
	RTABMAP_PARAM(Stereo, WinHeight,             int, 3,        "Window height.");
	RTABMAP_PARAM(Stereo, Iterations,            int, 30,       "Maximum iterations.");
	RTABMAP_PARAM(Stereo, MaxLevel,              int, 3,        "Maximum pyramid level.");
	RTABMAP_PARAM(Stereo, MinDisparity,          int, 1,        "Minimum disparity.");
	RTABMAP_PARAM(Stereo, MaxDisparity,          int, 128,      "Maximum disparity.");
	RTABMAP_PARAM(Stereo, OpticalFlow,           bool, true,    "Use optical flow to find stereo correspondences, otherwise a simple block matching approach is used.");
	RTABMAP_PARAM(Stereo, SSD,                   bool, true,    "[Stereo/OpticalFlow = false] Use Sum of Squared Differences (SSD) window, otherwise Sum of Absolute Differences (SAD) window is used.");
	RTABMAP_PARAM(Stereo, Eps,                   double, 0.01,  "[Stereo/OpticalFlow = true] Epsilon stop criterion.");

	RTABMAP_PARAM(StereoBM, BlockSize,           int, 15,       "See cv::StereoBM");
	RTABMAP_PARAM(StereoBM, MinDisparity,        int, 0,        "See cv::StereoBM");
	RTABMAP_PARAM(StereoBM, NumDisparities,      int, 64,       "See cv::StereoBM");
	RTABMAP_PARAM(StereoBM, PreFilterSize,       int, 9,        "See cv::StereoBM");
	RTABMAP_PARAM(StereoBM, PreFilterCap,        int, 31,       "See cv::StereoBM");
	RTABMAP_PARAM(StereoBM, UniquenessRatio,     int, 15,       "See cv::StereoBM");
	RTABMAP_PARAM(StereoBM, TextureThreshold,    int, 10,       "See cv::StereoBM");
	RTABMAP_PARAM(StereoBM, SpeckleWindowSize,   int, 100,      "See cv::StereoBM");
	RTABMAP_PARAM(StereoBM, SpeckleRange,        int, 4,        "See cv::StereoBM");

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

	static void parse(const ParametersMap & parameters, const std::string & key, bool & value);
	static void parse(const ParametersMap & parameters, const std::string & key, int & value);
	static void parse(const ParametersMap & parameters, const std::string & key, unsigned int & value);
	static void parse(const ParametersMap & parameters, const std::string & key, float & value);
	static void parse(const ParametersMap & parameters, const std::string & key, double & value);
	static void parse(const ParametersMap & parameters, const std::string & key, std::string & value);
	static void parse(const ParametersMap & parameters, ParametersMap & parametersOut);

	static const char * showUsage();
	static ParametersMap parseArguments(int argc, char * argv[]);

	static std::string getVersion();
	static std::string getDefaultDatabaseName();
	
	static std::string serialize(const ParametersMap & parameters);
	static ParametersMap deserialize(const std::string & parameters);

	static bool isFeatureParameter(const std::string & param);
	static ParametersMap getDefaultOdometryParameters(bool stereo = false);
	static ParametersMap getDefaultParameters(const std::string & group);
	static ParametersMap filterParameters(const ParametersMap & parameters, const std::string & group);

	static void readINI(const std::string & configFile, ParametersMap & parameters);
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

