/*
 * Copyright (C) 2010-2011, Mathieu Labbe and IntRoLab - Universite de Sherbrooke
 *
 * This file is part of RTAB-Map.
 *
 * RTAB-Map is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * RTAB-Map is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with RTAB-Map.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef PARAMETERS_H_
#define PARAMETERS_H_

// default parameters
#include "rtabmap/core/RtabmapExp.h" // DLL export/import defines
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
	private: \
		class Dummy##PREFIX##NAME { \
		public: \
			Dummy##PREFIX##NAME() {parameters_.insert(ParametersPair(#PREFIX "/" #NAME, #DEFAULT_VALUE)); \
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
	private: \
		class Dummy##PREFIX##NAME { \
		public: \
			Dummy##PREFIX##NAME() {parameters_.insert(ParametersPair(#PREFIX "/" #NAME, DEFAULT_VALUE)); \
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
	RTABMAP_PARAM(Rtabmap, PublishImage, 	             bool, true, "Publishing image.");
	RTABMAP_PARAM(Rtabmap, PublishPdf, 	                 bool, true, "Publishing pdf.");
	RTABMAP_PARAM(Rtabmap, PublishLikelihood, 	         bool, true, "Publishing likelihood.");
	RTABMAP_PARAM(Rtabmap, TimeThr, 		             float, 0.0, "Maximum time allowed for the detector (ms) (0 means infinity).");
	RTABMAP_PARAM(Rtabmap, MemoryThr, 		             int, 0, 	 "Maximum signatures in the Working Memory (ms) (0 means infinity).");
	RTABMAP_PARAM(Rtabmap, DetectionRate,                float, 1.0, "Detection rate. RTAB-Map will filter input images to satisfy this rate.");
	RTABMAP_PARAM(Rtabmap, ImageBufferSize,              int, 1, 	 "Data buffer size (0 min inf).");
	RTABMAP_PARAM_STR(Rtabmap, WorkingDirectory, Parameters::getDefaultWorkingDirectory(), "Working directory.");
	RTABMAP_PARAM_STR(Rtabmap, DatabasePath,     Parameters::getDefaultDatabasePath(), "Database path.");
	RTABMAP_PARAM(Rtabmap, MaxRetrieved,                 unsigned int, 2, "Maximum locations retrieved at the same time from LTM.");
	RTABMAP_PARAM(Rtabmap, StatisticLogsBufferedInRAM,   bool, true, "Statistic logs buffered in RAM instead of written to hard drive after each iteration.");
	RTABMAP_PARAM(Rtabmap, StatisticLogged,   	         bool, false, "Logging enabled.");

	// Hypotheses selection
	RTABMAP_PARAM(Rtabmap, LoopThr,    	     float, 0.11, 	"Loop closing threshold.");
	RTABMAP_PARAM(Rtabmap, LoopRatio,    	 float, 0.9, 	"The loop closure hypothesis must be over LoopRatio x lastHypothesisValue.");

	// Memory
	RTABMAP_PARAM(Mem, RehearsalSimilarity,     float, 0.6, 	"Rehearsal similarity.");
	RTABMAP_PARAM(Mem, ImageKept, 		        bool, true, 	"Keep images in db.");
	RTABMAP_PARAM(Mem, RehearsedNodesKept, 	    bool, true, 	"Keep rehearsed ndoes in db.");
	RTABMAP_PARAM(Mem, STMSize, 		        unsigned int, 10, "Short-term memory size.");
	RTABMAP_PARAM(Mem, IncrementalMemory, 	    bool, true, 	"SLAM mode, othwersize it is Localization mode.");
	RTABMAP_PARAM(Mem, RecentWmRatio,           float, 0.2, 	"Ratio of locations after the last loop closure in WM that cannot be transferred.");
	RTABMAP_PARAM(Mem, RehearsalIdUpdatedToNewOne, bool, false, "On merge, update to new id. When false, no copy.");
	RTABMAP_PARAM(Mem, GenerateIds,             bool, true,     "True=Generate location Ids, False=use input image ids.");
	RTABMAP_PARAM(Mem, BadSignaturesIgnored,    bool, false,     "Bad signatures are ignored.")

	// KeypointMemory (Keypoint-based)
	RTABMAP_PARAM(Kp, PublishKeypoints,      bool, true, 		"Publishing keypoints.");
	RTABMAP_PARAM(Kp, NNStrategy,            int, 1, 	 		"kNNFlannNaive=0, kNNFlannKdTree=1, kNNFlannLSH=2, kNNBruteForce=3, kNNBruteForceGPU=4");
	RTABMAP_PARAM(Kp, IncrementalDictionary, bool, true, 		"");
	RTABMAP_PARAM(Kp, MaxDepth,              float, 0.0, 		"Filter extracted keypoints by depth (0=inf)");
	RTABMAP_PARAM(Kp, WordsPerImage,         int, 400, 			"");
	RTABMAP_PARAM(Kp, BadSignRatio,          float, 0.2, 		"Bad signature ratio (less than Ratio x AverageWordsPerImage = bad).");
	RTABMAP_PARAM(Kp, NndrRatio, 	         float, 0.8, 		"NNDR ratio (A matching pair is detected, if its distance is closer than X times the distance of the second nearest neighbor.)");
	RTABMAP_PARAM(Kp, DetectorStrategy,      int, 0, 			"0=SURF 1=SIFT 2=ORB 3=FAST/FREAK 4=FAST/BRIEF.");
	RTABMAP_PARAM(Kp, TfIdfLikelihoodUsed,   bool, false, 		"Use of the td-idf strategy to compute the likelihood.");
	RTABMAP_PARAM(Kp, Parallelized,          bool, true, 		"If the dictionary update and signature creation were parallelized.");
	RTABMAP_PARAM_STR(Kp, RoiRatios, "0.0 0.0 0.0 0.0", 		"Region of interest ratios [left, right, top, bottom].");
	RTABMAP_PARAM_STR(Kp, DictionaryPath,    "", 				"Path of the pre-computed dictionary");

	//Database
	RTABMAP_PARAM(DbSqlite3, InMemory, 	   bool, false, 		"Using database in the memory instead of a file on the hard disk.");
	RTABMAP_PARAM(DbSqlite3, CacheSize,    unsigned int, 10000, "Sqlite cache size (default is 2000).");
	RTABMAP_PARAM(DbSqlite3, JournalMode,  int, 3, 				"0=DELETE, 1=TRUNCATE, 2=PERSIST, 3=MEMORY, 4=OFF (see sqlite3 doc : \"PRAGMA journal_mode\")");
	RTABMAP_PARAM(DbSqlite3, Synchronous,  int, 0, 				"0=OFF, 1=NORMAL, 2=FULL (see sqlite3 doc : \"PRAGMA synchronous\")");
	RTABMAP_PARAM(DbSqlite3, TempStore,    int, 2, 				"0=DEFAULT, 1=FILE, 2=MEMORY (see sqlite3 doc : \"PRAGMA temp_store\")");

	// Keypoints descriptors/detectors
	RTABMAP_PARAM(SURF, Extended, 		  bool, false, 	"true=128, false=64.");
	RTABMAP_PARAM(SURF, HessianThreshold, float, 150.0,	"");
	RTABMAP_PARAM(SURF, Octaves, 		  int, 4, 		"");
	RTABMAP_PARAM(SURF, OctaveLayers, 	  int, 2, 		"");
	RTABMAP_PARAM(SURF, Upright, 	      bool, false, 	"U-SURF");
	RTABMAP_PARAM(SURF, GpuVersion, 	  bool, false, 	"");
	RTABMAP_PARAM(SURF, GpuKeypointsRatio, 	  float, 0.01, 	"");

	RTABMAP_PARAM(SIFT, NFeatures,          int, 0,		"");
	RTABMAP_PARAM(SIFT, NOctaveLayers,      int, 3, 	"");
	RTABMAP_PARAM(SIFT, ContrastThreshold, 	double, 0.04, "");
	RTABMAP_PARAM(SIFT, EdgeThreshold,      double, 10.0, "");
	RTABMAP_PARAM(SIFT, Sigma,              double, 1.6, "");

	RTABMAP_PARAM(BRIEF, Bytes,         	   int, 32, 	"Bytes is a length of descriptor in bytes. It can be equal 16, 32 or 64 bytes.");

	RTABMAP_PARAM(FAST, Threshold,          int, 30, 	   "Threshold on difference between intensity of the central pixel and pixels of a circle around this pixel.");
	RTABMAP_PARAM(FAST, NonmaxSuppression,  bool, true, 	"If true, non-maximum suppression is applied to detected corners (keypoints).");
	RTABMAP_PARAM(FAST, Gpu,                bool, false, 	"GPU-FAST: Use GPU version of FAST. This option is enabled only if OpenCV is built with CUDA and GPUs are detected.");
	RTABMAP_PARAM(FAST, GpuKeypointsRatio,  double, 0.05, 	"Used with FAST GPU.");

	RTABMAP_PARAM(ORB, NFeatures,            int, 500,     "The maximum number of features to retain.");
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
	RTABMAP_PARAM(FREAK, PatternScale,          float, 22.0,  "Scaling of the description pattern.");
	RTABMAP_PARAM(FREAK, NOctaves,              int, 4,        "Number of octaves covered by the detected keypoints.");

	// BayesFilter
	RTABMAP_PARAM(Bayes, VirtualPlacePriorThr,           float, 0.9, "Virtual place prior");
	RTABMAP_PARAM_STR(Bayes, PredictionLC, "0.1 0.36 0.30 0.16 0.062 0.0151 0.00255 0.000324 2.5e-05 1.3e-06 4.8e-08 1.2e-09 1.9e-11 2.2e-13 1.7e-15 8.5e-18 2.9e-20 6.9e-23", "Prediction of loop closures (Gaussian-like, here with sigma=1.6) - Format: {VirtualPlaceProb, LoopClosureProb, NeighborLvl1, NeighborLvl2, ...}.");
	RTABMAP_PARAM(Bayes, FullPredictionUpdate,           bool, true, "Regenerate all the prediction matrix on each iteration (otherwise only removed/added ids are updated).");

	// Verify hypotheses
	RTABMAP_PARAM(VhEp, MatchCountMin, int, 8, 		"Minimum of matching visual words pairs to accept the loop hypothesis.");
	RTABMAP_PARAM(VhEp, RansacParam1,  float, 3.0, 	"Fundamental matrix (see cvFindFundamentalMat()): Max distance (in pixels) from the epipolar line for a point to be inlier.");
	RTABMAP_PARAM(VhEp, RansacParam2,  float, 0.99, "Fundamental matrix (see cvFindFundamentalMat()): Performance of the RANSAC.");

	// RGB-D SLAM
	RTABMAP_PARAM(RGBD, Enabled,           bool, true, 	"");
	RTABMAP_PARAM(RGBD, ScanMatchingSize,  int, 0, 		"Laser scan matching history for odometry correction (laser scans are required). Set to 0 to disable odometry correction.");
	RTABMAP_PARAM(RGBD, LinearUpdate,      float, 0.0, 	"Min linear displacement to update the map. Rehearsal is done prior to this, so weights are still updated.");
	RTABMAP_PARAM(RGBD, AngularUpdate,     float, 0.0, 	"Min angular displacement to update the map. Rehearsal is done prior to this, so weights are still updated.");
	RTABMAP_PARAM(RGBD, NewMapOdomChangeDistance, float, 0, "A new map is created if a change of odometry translation greater than X m is detected (0 m = disabled).");
	RTABMAP_PARAM(RGBD, ToroIterations,    int, 100,    "TORO graph optimization iterations");
	RTABMAP_PARAM(RGBD, OptimizeFromGraphEnd, bool, true,    "Optimize graph from the newest node. If false, the graph is optimized from the oldest node of the current graph (this adds an overhead computation to detect to oldest mode of the current graph, but it can be useful to preserve the map referential from the oldest node).");

	// Local loop closure detection
	RTABMAP_PARAM(RGBD, LocalLoopDetectionTime,     bool, true, 	"Detection over all locations in STM.");
	RTABMAP_PARAM(RGBD, LocalLoopDetectionSpace,    bool, false, 	"Detection over locations (in Working Memory or STM) near in space.");
	RTABMAP_PARAM(RGBD, LocalLoopDetectionRadius,   float, 15, 		"Maximum radius for space detection.");
	RTABMAP_PARAM(RGBD, LocalLoopDetectionNeighbors,   int, 20, 	"Maximum nearest neighbor.");
	RTABMAP_PARAM(RGBD, LocalLoopDetectionMaxDiffID,   int, 0,      "Maximum ID difference between the current/last loop closure location and the local loop closure hypotheses. Set 0 to ignore.")

	// Odometry
	RTABMAP_PARAM(Odom, Type,           		int, 0, 		"0=SURF 1=SIFT 2=ORB 3=FAST/FREAK 4=FAST/BRIEF.");
	RTABMAP_PARAM(Odom, LinearUpdate,           float, 0.0, 	"Min linear displacement to update odometry.");
	RTABMAP_PARAM(Odom, AngularUpdate,          float, 0.0, 	"Min angular displacement to update odometry.");
	RTABMAP_PARAM(Odom, MaxWords,               int, 0, 		"0 no limits.");
	RTABMAP_PARAM(Odom, InlierDistance,         float, 0.01, 	"Maximum distance for visual word correspondences.");
	RTABMAP_PARAM(Odom, MinInliers,             int, 10, 		"Minimum visual word correspondences to compute geometry transform.");
	RTABMAP_PARAM(Odom, Iterations,             int, 100, 		"Maximum iterations to compute the transform from visual words.");
	RTABMAP_PARAM(Odom, MaxDepth,               float, 5.0, 	"Max depth of the words (0 means no limit).");
	RTABMAP_PARAM(Odom, WordsRatio,             float, 0.5, 	"Minmum ratio of keypoints between the current image and the last image to compute odometry.");
	RTABMAP_PARAM(Odom, ResetCountdown,         int, 0,         "Automatically reset odometry after X consecutive images on which odometry cannot be computed (value=0 disables auto-reset).");
	RTABMAP_PARAM(Odom, LocalHistory,           int, 0,         "Local history size: If > 0 (example 5000), the odometry will maintain a local map of X maximum words.");
	RTABMAP_PARAM(Odom, NearestNeighbor,        int, 1, 	    "kNNFlannNaive=0, kNNFlannKdTree=1, kNNFlannLSH=2, kNNBruteForce=3, kNNBruteForceGPU=4");
	RTABMAP_PARAM(Odom, NNDR,                   float, 0.7,     "NNDR: nearest neighbor distance ratio.");

	// Loop closure constraint
	RTABMAP_PARAM(LccIcp, Type,            int, 0, 			"0=No ICP, 1=ICP 3D, 2=ICP 2D");
	RTABMAP_PARAM(LccIcp, MaxDistance,     float, 0.2,     "Maximum ICP correction distance accepted (m).");

	RTABMAP_PARAM(LccBow, MinInliers,      int, 20, 		"Minimum visual word correspondences to compute geometry transform.");
	RTABMAP_PARAM(LccBow, InlierDistance,  float, 0.01, 	"Maximum distance for visual word correspondences.");
	RTABMAP_PARAM(LccBow, Iterations,      int, 100, 		"Maximum iterations to compute the transform from visual words.");
	RTABMAP_PARAM(LccBow, MaxDepth,        float, 5.0, 		"Max depth of the words (0 means no limit).");

	RTABMAP_PARAM(LccIcp3, Decimation,      int, 8, 		"Depth image decimation.");
	RTABMAP_PARAM(LccIcp3, MaxDepth,        float, 4.0, 	"Max cloud depth.");
	RTABMAP_PARAM(LccIcp3, VoxelSize,       float, 0.005, 	"Voxel size to be used for ICP computation.");
	RTABMAP_PARAM(LccIcp3, Samples,         int, 0, 		"Random samples to be used for ICP computation. Not used if voxelSize is set.");
	RTABMAP_PARAM(LccIcp3, MaxCorrespondenceDistance, float, 0.05, "ICP 3D: Max distance for point correspondences.");
	RTABMAP_PARAM(LccIcp3, Iterations,      int, 30, 		"ICP 3D: Max iterations.");
	RTABMAP_PARAM(LccIcp3, MaxFitness,      float, 1.0, 	"ICP 3D: Maximum fitness to accept the computed transform.");

	RTABMAP_PARAM(LccIcp2, MaxCorrespondenceDistance, float, 0.1, 	"ICP 2D: Max distance for point correspondences.");
	RTABMAP_PARAM(LccIcp2, Iterations,      int, 30, 				"ICP 2D: Max iterations.");
	RTABMAP_PARAM(LccIcp2, MaxFitness,      float, 1.0, 			"ICP 2D: Maximum fitness to accept the computed transform.");
	RTABMAP_PARAM(LccIcp2, CorrespondenceRatio, float, 0.7, 		"ICP 2D: Ratio of matching correspondences to accept the transform.");
	RTABMAP_PARAM(LccIcp2, VoxelSize,       float, 0.005, 			"Voxel size to be used for ICP computation.");

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

	static std::string getDefaultDatabaseName();

private:
	Parameters();
	static std::string getDefaultWorkingDirectory();
	static std::string getDefaultDatabasePath();

private:
	static ParametersMap parameters_;
	static ParametersMap descriptions_;
	static Parameters instance_;
};

}

#endif /* PARAMETERS_H_ */

