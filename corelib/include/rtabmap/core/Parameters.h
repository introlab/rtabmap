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
#include "utilite/UEvent.h"
#include <string>
#include <map>
#include "utilite/UDestroyer.h"

namespace rtabmap
{

typedef std::map<std::string, std::string> ParametersMap; // Key, value
typedef std::pair<const std::string, std::string> ParametersPair;

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
#define RTABMAP_PARAM(PREFIX, NAME, TYPE, DEFAULT_VALUE) \
	public: \
		static std::string k##PREFIX##NAME() {return std::string(#PREFIX "/" #NAME);} \
		static TYPE default##PREFIX##NAME() {return DEFAULT_VALUE;} \
	private: \
		class Dummy##PREFIX##NAME { \
		public: \
			Dummy##PREFIX##NAME() {parameters_.insert(ParametersPair(#PREFIX "/" #NAME, #DEFAULT_VALUE));} \
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
#define RTABMAP_PARAM_STR(PREFIX, NAME, DEFAULT_VALUE) \
	public: \
		static std::string k##PREFIX##NAME() {return std::string(#PREFIX "/" #NAME);} \
		static std::string default##PREFIX##NAME() {return DEFAULT_VALUE;} \
	private: \
		class Dummy##PREFIX##NAME { \
		public: \
			Dummy##PREFIX##NAME() {parameters_.insert(ParametersPair(#PREFIX "/" #NAME, DEFAULT_VALUE));} \
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
	RTABMAP_PARAM(Rtabmap, VhStrategy, 	        int, 0);	   // Simple 0, Epipolar 1
	RTABMAP_PARAM(Rtabmap, PublishStats, 	     bool, true); // Publishing statistics
	RTABMAP_PARAM(Rtabmap, ReactivationThr,     float, 0.0); // Reactivation threshold
	RTABMAP_PARAM(Rtabmap, TimeThr, 		     float, 0.7); // Maximum time allowed for the detector (s) (0 means infinity)
	RTABMAP_PARAM(Rtabmap, DisableReactivation, bool, false); // Memory reactivation when a loop closure occurs : 0=enable, 1=disable
	RTABMAP_PARAM(Rtabmap, SMStateBufferSize,    int, 1);      // Data buffer size (0 min inf)
	RTABMAP_PARAM(Rtabmap, MinMemorySizeForLoopDetection, unsigned int, 15); //Minimum size of the memory to create loop closure hypotheses
	RTABMAP_PARAM_STR(Rtabmap, WorkingDirectory, Parameters::getDefaultWorkingDirectory());	   // Working directory
	RTABMAP_PARAM(Rtabmap, LocalGraphCleaned, bool, false);	   // Clean the neighborhood of the retrieved id
	RTABMAP_PARAM(Rtabmap, MaxRetrieved,       unsigned int, 2); // Maximum locations retrieved at the same time from LTM
	RTABMAP_PARAM(Rtabmap, ActionsByTime,       bool, true); // Select next actions based on the more recent neighbor of the current node, otherwise, weight is also used

	// Hypotheses selection
	RTABMAP_PARAM(Rtabmap, LoopThr,    	     float, 0.10); // Loop closing threshold
	RTABMAP_PARAM(Rtabmap, LoopRatio,    	 float, 0.90); // The loop closure hypothesis must be over LoopRatio x lastHypothesisValue

	// Memory
	RTABMAP_PARAM(Mem, SimilarityThr, 	       float, 0.20);  // Similarity between the last signature and neighbor
	RTABMAP_PARAM(Mem, SimilarityOnlyLast,     bool, false);	// Only compare to the last signature in STM, otherwise it compares to all signatures in STM
	RTABMAP_PARAM(Mem, RawDataKept, 		   bool, false); // Keep raw data
	RTABMAP_PARAM(Mem, MaxStMemSize, 		   unsigned int, 25); // Short-time memory size
	RTABMAP_PARAM(Mem, CommonSignatureUsed,    bool, true); // A common signature/virtual place is automatically updated with id -1
	RTABMAP_PARAM(Mem, IncrementalMemory, 	   bool, true);
	RTABMAP_PARAM(Mem, DatabaseCleaned, 	   bool, true);  // Delete old signatures in the database (the ones which can't never be reactivated)
	RTABMAP_PARAM(Mem, DelayRequired, 	       int, 10);     // Delay (in iterations) required to transfer signatures
	RTABMAP_PARAM(Mem, RecentWmRatio,          float, 0.2);  // Ratio of locations after the last loop closure in WM that cannot be transferred

	// KeypointMemory (Keypoint-based)
	RTABMAP_PARAM(Kp, NNStrategy,     	int, 2);	  // Naive 0, kdTree 1, kdForest 2
	RTABMAP_PARAM(Kp, IncrementalDictionary, bool, true);
	RTABMAP_PARAM(Kp, WordsPerImage,    int, 400);
	RTABMAP_PARAM(Kp, BadSignRatio,    float, 0.05); //Bad signature ratio (less than Ratio x AverageWordsPerImage = bad)
	RTABMAP_PARAM(Kp, MinDistUsed,     bool, false); // The nearest neighbor must have a distance < minDist
	RTABMAP_PARAM(Kp, MinDist, 	    float, 0.05); // Matching a descriptor with a word (euclidean distance ^ 2)
	RTABMAP_PARAM(Kp, NndrUsed, 	    bool, true);  // If NNDR ratio is used
	RTABMAP_PARAM(Kp, NndrRatio, 	    float, 0.8);  // NNDR ratio (A matching pair is detected, if its distance is closer than X times the distance of the second nearest neighbor.)
	RTABMAP_PARAM(Kp, MaxLeafs, 	    int, 64);  // Maximum number of leafs checked (when using kd-trees)
	RTABMAP_PARAM(Kp, DetectorStrategy, int, 0);      // Surf detector 0, Star detector 1
	RTABMAP_PARAM(Kp, DescriptorStrategy, int, 0);      // kDescriptorSurf=0, kDescriptorColorSurf, kDescriptorLaplacianSurf, kDescriptorSift, kDescriptorHueSurf, kDescriptorUndef
	RTABMAP_PARAM(Kp, UsingAdaptiveResponseThr, bool, false);
	RTABMAP_PARAM(Kp, ReactivatedWordsComparedToNewWords, bool, true); //Reactivated words are compared to the last words added in the dictionary (which are not indexed)
	RTABMAP_PARAM(Kp, TfIdfLikelihoodUsed, bool, false); // Use of the td-idf strategy to compute the likelihood
	RTABMAP_PARAM(Kp, Parallelized, bool, true); // If the dictionary update and signature creation were parallelized
	RTABMAP_PARAM(Kp, TfIdfNormalized, bool, false); // If tf-idf weighting is normalized by the words count ratio between compared signatures
	RTABMAP_PARAM_STR(Kp, RoiRatios, "0.0 0.0 0.0 0.0"); // Region of interest ratios [left, right, top, bottom]
	RTABMAP_PARAM_STR(Kp, DictionaryPath, ""); // Path of the pre-computed dictionary

	//Database
	RTABMAP_PARAM(Db, MinSignaturesToSave, int, 20);	 // Minimum signatures needed in the trash to save them (empty trash thread)
	RTABMAP_PARAM(Db, MinWordsToSave, 	   int, 4000);	 // Minimum visual words needed in the trash to save them (empty trash thread)
	RTABMAP_PARAM(DbSqlite3, InMemory, 	   bool, false);	 // Using database in the memory instead of a file on the hard disk
	RTABMAP_PARAM(DbSqlite3, CacheSize,    unsigned int, 2000);	 // Sqlite cache size (default is 2000)
	RTABMAP_PARAM(DbSqlite3, JournalMode,  int, 0);	 	 // 0=DELETE, 1=TRUNCATE, 2=PERSIST, 3=MEMORY, 4=OFF (see sqlite3 doc : "PRAGMA journal_mode")

	RTABMAP_PARAM(SURF, Extended, 		  bool, false); // true=128, false=64
	RTABMAP_PARAM(SURF, HessianThreshold, float, 100.0);
	RTABMAP_PARAM(SURF, Octaves, 		  int, 4);
	RTABMAP_PARAM(SURF, OctaveLayers, 	  int, 2);
	RTABMAP_PARAM(SURF, GpuVersion, 	  bool, false);
	RTABMAP_PARAM(SURF, Upright, 	      bool, false); // U-SURF

	RTABMAP_PARAM(SIFT, Threshold, 	double, 0.006667); // true=128, false=64
	RTABMAP_PARAM(SIFT, EdgeThreshold, double, 10.0);

	RTABMAP_PARAM(Star, MaxSize, int, 45);
	RTABMAP_PARAM(Star, ResponseThreshold, int, 30);
	RTABMAP_PARAM(Star, LineThresholdProjected, int, 10);
	RTABMAP_PARAM(Star, LineThresholdBinarized, int, 8);
	RTABMAP_PARAM(Star, SuppressNonmaxSize, int, 5);

	// BayesFilter
	RTABMAP_PARAM(Bayes, VirtualPlacePriorThr, float, 0.9);	// Virtual place prior
	RTABMAP_PARAM_STR(Bayes, PredictionLC, "0.1 0.24 0.18 0.1 0.04 0.01"); // Prediction of loop closures (Gaussian-like, must be pair size) - Format: {VirtualPlaceProb, LoopClosureProb, BackwardNeighborLvl1, ForwardNeighborLvl1, BackwardNeighborLvl2, ForwardNeighborLvl2, ...}

	// Verify hypotheses
	RTABMAP_PARAM(VhEp, MatchCountMin, int, 8);     // Minimum of matching visual words pairs to accept the loop hypothesis
	RTABMAP_PARAM(VhEp, RansacParam1,  float, 3.0);  // Fundamental matrix (see cvFindFundamentalMat()): Max distance (in pixels) from the epipolar line for a point to be inlier
	RTABMAP_PARAM(VhEp, RansacParam2,  float, 0.99); // Fundamental matrix (see cvFindFundamentalMat()): Performance of the RANSAC

public:
	virtual ~Parameters();
	static const ParametersMap & getDefaultParameters();

private:
	Parameters();
	static Parameters * getInstance();
	const ParametersMap & getParameters() const;
	void addParameter(const std::string & key, const std::string & value);
	static std::string getDefaultWorkingDirectory();

private:
	static Parameters * instance_;
	static UDestroyer<Parameters> destroyer_;
	static ParametersMap parameters_;
};

/**
 * The parameters event. This event is used to send
 * parameters across the threads.
 */
class ParamEvent : public UEvent
{
public:
	ParamEvent(const ParametersMap & parameters) : UEvent(0), parameters_(parameters) {}
	ParamEvent(const std::string & parameterKey, const std::string & parameterValue) : UEvent(0)
	{
		parameters_.insert(std::pair<std::string, std::string>(parameterKey, parameterValue));
	}
	~ParamEvent() {}
	virtual std::string getClassName() const {return "ParamEvent";}

	const ParametersMap & getParameters() const {return parameters_;}

private:
	ParametersMap parameters_; /**< The parameters map (key,value). */
};

}

#endif /* PARAMETERS_H_ */

