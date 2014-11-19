/*
Copyright (c) 2010-2014, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
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

#ifndef RTABMAP_H_
#define RTABMAP_H_

#include "rtabmap/core/RtabmapExp.h" // DLL export/import defines

#include "rtabmap/core/Parameters.h"
#include "rtabmap/core/SensorData.h"
#include "rtabmap/core/Statistics.h"
#include "rtabmap/core/Link.h"

#include <opencv2/core/core.hpp>
#include <list>
#include <stack>
#include <set>

namespace rtabmap
{

class EpipolarGeometry;
class Memory;
class BayesFilter;
class Signature;

class RTABMAP_EXP Rtabmap
{
public:
	enum VhStrategy {kVhNone, kVhEpipolar, kVhUndef};

public:
	static std::string getVersion();
	static void readParameters(const std::string & configFile, ParametersMap & parameters);
	static void writeParameters(const std::string & configFile, const ParametersMap & parameters);

public:
	Rtabmap();
	virtual ~Rtabmap();

	bool process(const cv::Mat & image, int id=0); // for convenience, an id is automatically generated if id=0
	bool process(const SensorData & data); // for convenience

	void init(const ParametersMap & parameters, const std::string & databasePath = "");
	void init(const std::string & configFile = "", const std::string & databasePath = "");

	void close();

	const std::string & getWorkingDir() const {return _wDir;}
	int getLoopClosureId() const;
	int getRetrievedId() const;
	int getLastLocationId() const;
	float getLcHypValue() const {return _lcHypothesisValue;}
	std::list<int> getWM() const; // working memory
	std::set<int> getSTM() const; // short-term memory
	int getWMSize() const; // working memory size
	int getSTMSize() const; // short-term memory size
	std::map<int, int> getWeights() const;
	int getTotalMemSize() const;
	double getLastProcessTime() const {return _lastProcessTime;};
	std::multimap<int, cv::KeyPoint> getWords(int locationId) const;
	bool isInSTM(int locationId) const;
	bool isIDsGenerated() const;
	const Statistics & getStatistics() const;
	//bool getMetricData(int locationId, cv::Mat & rgb, cv::Mat & depth, float & depthConstant, Transform & pose, Transform & localTransform) const;
	Transform getPose(int locationId) const;
	Transform getMapCorrection() const {return _mapCorrection;}
	const Memory * getMemory() const {return _memory;}

	float getTimeThreshold() const {return _maxTimeAllowed;} // in ms
	void setTimeThreshold(float maxTimeAllowed); // in ms

	void triggerNewMap();
	void generateDOTGraph(const std::string & path, int id=0, int margin=5);
	void generateTOROGraph(const std::string & path, bool optimized, bool global);
	void resetMemory();
	void dumpPrediction() const;
	void dumpData() const;
	void parseParameters(const ParametersMap & parameters);
	void setWorkingDirectory(std::string path);
	void deleteLocation(int locationId); // Only nodes in STM can be deleted
	void rejectLoopClosure(int oldId, int newId);
	void get3DMap(std::map<int, Signature> & signatures,
			std::map<int, Transform> & poses,
			std::multimap<int, Link> & constraints,
			std::map<int, int> & mapIds,
			bool optimized,
			bool global) const;
	void getGraph(std::map<int, Transform> & poses,
			std::multimap<int, Link> & constraints,
			std::map<int, int> & mapIds,
			bool optimized,
			bool global);

	std::map<int, Transform> getOptimizedWMPosesInRadius(int fromId, int maxNearestNeighbors, float radius, int maxDiffID, int & nearestId) const;
	void adjustLikelihood(std::map<int, float> & likelihood) const;
	std::pair<int, float> selectHypothesis(const std::map<int, float> & posterior,
											const std::map<int, float> & likelihood) const;

private:
	void optimizeCurrentMap(int id,
			bool lookInDatabase,
			std::map<int, Transform> & optimizedPoses,
			std::multimap<int, Link> * constraints = 0) const;

	void setupLogFiles(bool overwrite = false);
	void flushStatisticLogs();

private:
	// Modifiable parameters
	bool _publishStats;
	bool _publishLastSignature;
	bool _publishPdf;
	bool _publishLikelihood;
	float _maxTimeAllowed; // in ms
	unsigned int _maxMemoryAllowed; // signatures count in WM
	float _loopThr;
	float _loopRatio;
	unsigned int _maxRetrieved;
	bool _statisticLogsBufferedInRAM;
	bool _statisticLogged;
	bool _statisticLoggedHeaders;
	bool _rgbdSlamMode;
	float _rgbdLinearUpdate;
	float _rgbdAngularUpdate;
	float _newMapOdomChangeDistance;
	int _globalLoopClosureIcpType;
	float _globalLoopClosureIcpMaxDistance;
	bool _poseScanMatching;
	bool _localLoopClosureDetectionTime;
	bool _localLoopClosureDetectionSpace;
	float _localDetectRadius;
	float _localDetectMaxNeighbors;
	int _localDetectMaxDiffID;
	int _toroIterations;
	std::string _databasePath;
	bool _optimizeFromGraphEnd;
	bool _reextractLoopClosureFeatures;
	int _reextractNNType;
	float _reextractNNDR;
	int _reextractFeatureType;
	int _reextractMaxWords;
	bool _startNewMapOnLoopClosure;

	int _lcHypothesisId;
	float _lcHypothesisValue;
	int _retrievedId;
	double _lastProcessTime;

	// Abstract classes containing all loop closure
	// strategies for a type of signature or configuration.
	EpipolarGeometry * _epipolarGeometry;
	BayesFilter * _bayesFilter;
	ParametersMap _lastParameters;

	Memory * _memory;

	FILE* _foutFloat;
	FILE* _foutInt;
	std::list<std::string> _bufferedLogsF;
	std::list<std::string> _bufferedLogsI;

	Statistics statistics_;

	std::string _wDir;

	std::map<int, Transform> _optimizedPoses;
	std::multimap<int, Link> _constraints;
	Transform _mapCorrection;
	Transform _mapTransform; // for localization mode
};

#endif /* RTABMAP_H_ */

} // namespace rtabmap
