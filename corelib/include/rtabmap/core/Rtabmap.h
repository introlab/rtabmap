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

#ifndef RTABMAP_H_
#define RTABMAP_H_

#include "rtabmap/core/RtabmapExp.h" // DLL export/import defines

#include "rtabmap/core/Parameters.h"
#include "rtabmap/core/Image.h"
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
	bool process(const Image & image); // for convenience

	void init(const ParametersMap & param, bool deleteMemory = true);
	void init(const std::string & configFile = "", bool deleteMemory = true);

	void close();

	const std::string & getWorkingDir() const {return _wDir;}
	std::string getDatabasePath() const;
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

	void setTimeThreshold(float maxTimeAllowed); // in ms

	void triggerNewMap();
	void generateDOTGraph(const std::string & path, int id=0, int margin=5);
	void generateTOROGraph(const std::string & path, bool optimized, bool global);
	void resetMemory(bool dbOverwritten = false);
	void dumpPrediction() const;
	void dumpData() const;
	void parseParameters(const ParametersMap & parameters);
	void setWorkingDirectory(std::string path);
	void setDatabasePath(const std::string & path);
	void deleteLocation(int locationId); // Only nodes in STM can be deleted
	void rejectLoopClosure(int oldId, int newId);
	void get3DMap(std::map<int, std::vector<unsigned char> > & images,
			std::map<int, std::vector<unsigned char> > & depths,
			std::map<int, std::vector<unsigned char> > & depths2d,
			std::map<int, float> & depthConstants,
			std::map<int, Transform> & localTransforms,
			std::map<int, Transform> & poses,
			std::multimap<int, Link> & constraints,
			bool optimized,
			bool global) const;
	void getGraph(std::map<int, Transform> & poses,
			std::multimap<int, Link> & constraints,
			bool optimized,
			bool full);

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
	bool _publishImage;
	bool _publishPdf;
	bool _publishLikelihood;
	bool _publishKeypoints;
	float _maxTimeAllowed; // in ms
	unsigned int _maxMemoryAllowed; // signatures count in WM
	float _loopThr;
	float _loopRatio;
	unsigned int _maxRetrieved;
	bool _statisticLogsBufferedInRAM;
	bool _statisticLogged;
	bool _rgbdSlamMode;
	float _rgbdLinearUpdate;
	float _rgbdAngularUpdate;
	float _newMapOdomChangeDistance;
	int _globalLoopClosureIcpType;
	float _globalLoopClosureIcpMaxDistance;
	int _scanMatchingSize;
	bool _localLoopClosureDetectionTime;
	bool _localLoopClosureDetectionSpace;
	float _localDetectRadius;
	float _localDetectMaxNeighbors;
	int _localDetectMaxDiffID;
	int _toroIterations;
	std::string _databasePath;

	int _lcHypothesisId;
	float _lcHypothesisValue;
	int _retrievedId;
	double _lastProcessTime;

	// Abstract classes containing all loop closure
	// strategies for a type of signature or configuration.
	EpipolarGeometry * _epipolarGeometry;
	BayesFilter * _bayesFilter;

	Memory * _memory;

	FILE* _foutFloat;
	FILE* _foutInt;
	std::list<std::string> _bufferedLogsF;
	std::list<std::string> _bufferedLogsI;

	Statistics statistics_;

	std::string _wDir;

	std::map<int, Transform> _optimizedPoses;
	std::multimap<int, Link> _constraints;
	Transform _mapCorrection; // for localization mode
	Transform _mapTransform; // for localization mode
};

#endif /* RTABMAP_H_ */

} // namespace rtabmap
