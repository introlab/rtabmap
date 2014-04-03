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
	RTABMAP_PARAM(Kp, NNStrategy,            int, 1, 	 		"Naive 0, kdForest 1.");
	RTABMAP_PARAM(Kp, IncrementalDictionary, bool, true, 		"");
	RTABMAP_PARAM(Kp, MaxDepth,              float, 0.0, 		"Filter extracted keypoints by depth (0=inf)");
	RTABMAP_PARAM(Kp, WordsPerImage,         int, 400, 			"");
	RTABMAP_PARAM(Kp, BadSignRatio,          float, 0.2, 		"Bad signature ratio (less than Ratio x AverageWordsPerImage = bad).");
	RTABMAP_PARAM(Kp, MinDistUsed,           bool, false, 		"The nearest neighbor must have a distance < minDist.");
	RTABMAP_PARAM(Kp, MinDist, 	             float, 0.05, 		"Matching a descriptor with a word (euclidean distance ^ 2)");
	RTABMAP_PARAM(Kp, NndrUsed, 	         bool, true, 		"If NNDR ratio is used.");
	RTABMAP_PARAM(Kp, NndrRatio, 	         float, 0.8, 		"NNDR ratio (A matching pair is detected, if its distance is closer than X times the distance of the second nearest neighbor.)");
	RTABMAP_PARAM(Kp, MaxLeafs, 	         int, 64, 			"Maximum number of leafs checked (when using kd-trees).");
	RTABMAP_PARAM(Kp, DetectorStrategy,      int, 0, 			"Surf detector 0, SIFT detector 1, undef 2.");
	RTABMAP_PARAM(Kp, TfIdfLikelihoodUsed,   bool, false, 		"Use of the td-idf strategy to compute the likelihood.");
	RTABMAP_PARAM(Kp, Parallelized,          bool, true, 		"If the dictionary update and signature creation were parallelized.");
	RTABMAP_PARAM_STR(Kp, RoiRatios, "0.0 0.0 0.0 0.0", 		"Region of interest ratios [left, right, top, bottom].");
	RTABMAP_PARAM_STR(Kp, DictionaryPath,    "", 				"Path of the pre-computed dictionary");

	//Database
	RTABMAP_PARAM(DbSqlite3, InMemory, 	   bool, true, 			"Using database in the memory instead of a file on the hard disk.");
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

	RTABMAP_PARAM(SIFT, NFeatures,          int, 0,		"");
	RTABMAP_PARAM(SIFT, NOctaveLayers,      int, 3, 	"");
	RTABMAP_PARAM(SIFT, ContrastThreshold, 	double, 0.04, "");
	RTABMAP_PARAM(SIFT, EdgeThreshold,      double, 10.0, "");
	RTABMAP_PARAM(SIFT, Sigma,              double, 1.6, "");

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
	RTABMAP_PARAM(RGBD, NewMapOdomChangeDistance, float, 1, "A new map is created if a change of odometry translation greater than X m is detected (0 m = disabled).");
	RTABMAP_PARAM(RGBD, ToroIterations,    int, 100,    "TORO graph optimization iterations")

	// Local loop closure detection
	RTABMAP_PARAM(RGBD, LocalLoopDetectionTime,     bool, true, 	"Detection over all locations in STM.");
	RTABMAP_PARAM(RGBD, LocalLoopDetectionSpace,    bool, false, 	"Detection over locations (in Working Memory or STM) near in space.");
	RTABMAP_PARAM(RGBD, LocalLoopDetectionRadius,   float, 15, 		"Maximum radius for space detection.");
	RTABMAP_PARAM(RGBD, LocalLoopDetectionNeighbors,   int, 20, 	"Maximum nearest neighbor.");
	RTABMAP_PARAM(RGBD, LocalLoopDetectionMaxDiffID,   int, 0,      "Maximum ID difference between the current/last loop closure location and the local loop closure hypotheses. Set 0 to ignore.")

	// Odometry
	RTABMAP_PARAM(Odom, Type,           		int, 0, 		"0=BOW 1=Binary.");
	RTABMAP_PARAM(Odom, LinearUpdate,           float, 0.0, 	"Min linear displacement to update odometry.");
	RTABMAP_PARAM(Odom, AngularUpdate,          float, 0.0, 	"Min angular displacement to update odometry.");
	RTABMAP_PARAM(Odom, MaxWords,               int, 0, 		"0 no limits.");
	RTABMAP_PARAM(Odom, InlierDistance,         float, 0.01, 	"Maximum distance for visual word correspondences.");
	RTABMAP_PARAM(Odom, MinInliers,             int, 10, 		"Minimum visual word correspondences to compute geometry transform.");
	RTABMAP_PARAM(Odom, Iterations,             int, 100, 		"Maximum iterations to compute the transform from visual words.");
	RTABMAP_PARAM(Odom, MaxDepth,               float, 5.0, 	"Max depth of the words (0 means no limit).");
	RTABMAP_PARAM(Odom, ResetCountdown,         int, 0,         "Automatically reset odometry after X consecutive images on which odometry cannot be computed (value=0 disables auto-reset).")

	RTABMAP_PARAM(OdomBin, BriefBytes,         	   int, 32, 	"");
	RTABMAP_PARAM(OdomBin, FastThreshold,          int, 30, 	"");
	RTABMAP_PARAM(OdomBin, FastNonmaxSuppression,  bool, true, 	"");
	RTABMAP_PARAM(OdomBin, BruteForceMatching,     bool, true, 	"If false, FLANN LSH is used.");

	RTABMAP_PARAM(OdomICP, Decimation,               int, 4, 		"");
	RTABMAP_PARAM(OdomICP, VoxelSize,                float, 0.005, 	"Voxel size to be used for ICP computation.");
	RTABMAP_PARAM(OdomICP, Samples,                  int, 0, 		"not used if voxelSize is set.");
	RTABMAP_PARAM(OdomICP, CorrespondencesDistance,  float, 0.05, 	"");
	RTABMAP_PARAM(OdomICP, Iterations,               int, 30, 		"");
	RTABMAP_PARAM(OdomICP, MaxFitness,               float, 0.01, 	"");

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

