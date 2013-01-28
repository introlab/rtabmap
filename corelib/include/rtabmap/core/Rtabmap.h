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

#include "utilite/UThreadNode.h"
#include "utilite/UEventsHandler.h"
#include "utilite/USemaphore.h"
#include "utilite/UMutex.h"
#include "utilite/UVariant.h"
#include "rtabmap/core/RtabmapEvent.h"
#include "rtabmap/core/Parameters.h"
#include "rtabmap/core/Image.h"
#include <opencv2/core/core.hpp>
#include <list>
#include <stack>
#include <set>

namespace rtabmap
{

class EpipolarGeometry;
class Memory;
class BayesFilter;

class RTABMAP_EXP Rtabmap :
	public UThreadNode,
	public UEventsHandler
{
public:
	enum State {
		kStateIdle,
		kStateDetecting,
		kStateReseting,
		kStateChangingParameters,
		kStateDumpingMemory,
		kStateDumpingPrediction,
		kStateGeneratingGraph,
		kStateGeneratingLocalGraph,
		kStateDeletingMemory,
		kStateCleanSensorsBuffer
	};

	enum VhStrategy {kVhNone, kVhEpipolar, kVhUndef};

	static const char * kDefaultDatabaseName;

public:
	static std::string getVersion();
	static void readParameters(const std::string & configFile, ParametersMap & parameters);
	static void writeParameters(const std::string & configFile, const ParametersMap & parameters);

public:
	Rtabmap();
	virtual ~Rtabmap();

	void process(const cv::Mat & image, int id=0); // for convenience, an id is automatically generated if id=0
	void process(const Image & image); // for convenience

	void init(const ParametersMap & param, bool deleteMemory = true);
	void init(const std::string & configFile = "", bool deleteMemory = true);
	void clearBufferedSensors();

	void close();

	const std::string & getWorkingDir() const {return _wDir;}
	int getLoopClosureId() const;
	int getRetrievedId() const;
	int getLastLocationId();
	float getLcHypValue() const {return _lastLcHypothesisValue;}
	std::list<int> getWM(); // working memory
	std::set<int> getSTM(); // short-term memory
	int getWMSize(); // working memory size
	int getSTMSize(); // short-term memory size
	std::map<int, int> getWeights();
	int getTotalMemSize();
	double getLastProcessTime() const {return _lastProcessTime;};
	std::multimap<int, cv::KeyPoint> getWords(int locationId);
	std::map<int, int> getNeighbors(int nodeId, int margin, bool lookInLTM = false);
	bool isInSTM(int locationId);
	Statistics getStatistics();

	void setTimeThreshold(float maxTimeAllowed); // in ms

	void generateGraph(const std::string & path);
	void resetMemory(bool dbOverwritten = false);
	void dumpPrediction();
	void dumpData();
	void updateParameters(const ParametersMap & parameters);
	void setWorkingDirectory(std::string path);
	void deleteLastLocation();

	void adjustLikelihood(std::map<int, float> & likelihood) const;
	std::pair<int, float> selectHypothesis(const std::map<int, float> & posterior,
											const std::map<int, float> & likelihood) const;

protected:
	virtual void handleEvent(UEvent * anEvent);

private:
	virtual void mainLoop();
	virtual void mainLoopKill();
	virtual void mainLoopBegin();
	void process();
	void addImage(const Image & image);
	void getImage(Image & image);
	void setupLogFiles(bool overwrite = false);
	void flushStatisticLogs();
	void releaseAllStrategies();
	void pushNewState(State newState, const ParametersMap & parameters = ParametersMap());
	void generateLocalGraph(const std::string & path, int id, int margin);
	void parseParameters(const ParametersMap & parameters);
	void setDataBufferSize(int size);

private:
	// Modifiable parameters
	bool _publishStats;
	bool _publishImage;
	bool _publishPdf;
	bool _publishLikelihood;
	bool _publishKeypoints;
	float _maxTimeAllowed; // in ms
	unsigned int _maxMemoryAllowed; // signatures count in WM
	int _imageBufferMaxSize;
	float _loopThr;
	float _loopRatio;
	unsigned int _maxRetrieved;
	bool _likelihoodNullValuesIgnored;
	bool _statisticLogsBufferedInRAM;

	int _lcHypothesisId;
	int _retrievedId;
	float _lastLcHypothesisValue;
	int _lastLoopClosureId;
	double _lastProcessTime;

	UMutex _stateMutex;
	std::stack<State> _state;
	std::stack<ParametersMap> _stateParam;

	std::list<Image> _imageBuffer;
	UMutex _imageMutex;
	USemaphore _imageAdded;

	UMutex _threadMutex;

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
};

#endif /* RTABMAP_H_ */

} // namespace rtabmap
