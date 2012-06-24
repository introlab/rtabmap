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
#include "rtabmap/core/Sensor.h"
#include "rtabmap/core/Actuator.h"
#include <opencv2/core/core.hpp>
#include <list>
#include <stack>
#include <set>

namespace rtabmap
{

class Signature;

class HypVerificator;
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
		kStateDeletingMemory,
		kStateCleanSensorsBuffer
	};

	enum VhStrategy {kVhNone, kVhSim, kVhEpipolar, kVhUndef};

	static const char * kDefaultIniFileName;
	static const char * kDefaultIniFilePath;
	static const char * kDefaultDatabaseName;

public:
	static std::string getVersion();
	static std::string getIniFilePath();
	static void readParameters(const char * configFile, ParametersMap & parameters);
	static void writeParameters(const char * configFile, const ParametersMap & parameters);

public:
	Rtabmap();
	virtual ~Rtabmap();

	void process(const std::list<Sensor> & data);
	void process(const Sensor & data); // for convenience when only one sensor is used
	void dumpData();

	void init(const ParametersMap & param);
	void init(const char * configFile = 0);
	void clearBufferedSensors();

	const std::string & getWorkingDir() const {return _wDir;}
	int getLoopClosureId() const;
	int getReactivatedId() const;
	int getLastSignatureId() const;
	const std::list<Actuator> & getActuator() const {return _actuators;}
	std::list<int> getWorkingMem() const;
	std::set<int> getStMem() const;
	std::map<int, int> getWeights() const;
	int getTotalMemSize() const;
	const std::string & getGraphFileName() const {return _graphFileName;}

	void setMaxTimeAllowed(float maxTimeAllowed); // in ms
	void setDataBufferSize(int size);
	void setWorkingDirectory(std::string path);
	void setGraphFileName(const std::string & fileName) {_graphFileName = fileName;}

	void adjustLikelihood(std::map<int, float> & likelihood) const;
	std::pair<int, float> selectHypothesis(const std::map<int, float> & posterior,
											const std::map<int, float> & likelihood,
											bool neighborSumUsed,
											bool likelihoodUsed) const;

protected:
	virtual void handleEvent(UEvent * anEvent);

private:
	virtual void mainLoop();
	virtual void mainLoopKill();
	virtual void mainLoopBegin();
	void process();
	void resetMemory(bool dbOverwritten = false);
	void addSensorimotor(const std::list<Sensor> & sensors, const std::list<Actuator> & actuators);
	void getSensorimotor(std::list<Sensor> & sensors, std::list<Actuator> & actuators);
	void setupLogFiles(bool overwrite = false);
	void releaseAllStrategies();
	void pushNewState(State newState, const ParametersMap & parameters = ParametersMap());
	void dumpPrediction() const;
	void parseParameters(const ParametersMap & parameters);

private:
	// Modifiable parameters
	bool _publishStats;
	bool _publishRawData;
	bool _publishPdf;
	bool _publishLikelihood;
	bool _publishKeypoints;
	bool _publishMasks;
	float _maxTimeAllowed; // in ms
	unsigned int _maxMemoryAllowed; // signatures count in WM
	int _sensorsBufferMaxSize;
	float _loopThr;
	float _loopRatio;
	float _retrievalThr;
	unsigned int _maxRetrieved;
	bool _selectionNeighborhoodSummationUsed;
	bool _selectionLikelihoodUsed;
	bool _actionsSentRejectHyp;
	float _confidenceThr;
	bool _likelihoodStdDevRemoved;
	bool _likelihoodNullValuesIgnored;

	int _lcHypothesisId;
	int _reactivateId;
	float _lastLcHypothesisValue;
	int _lastLoopClosureId;
	std::list<Actuator> _actuators;

	UMutex _stateMutex;
	std::stack<State> _state;
	std::stack<ParametersMap> _stateParam;

	std::list<std::pair<std::list<Sensor>, std::list<Actuator> > > _sensorimotorBuffer;
	UMutex _sensorimotorMutex;
	USemaphore _sensorimotorAdded;

	// Abstract classes containing all loop closure
	// strategies for a type of signature or configuration.
	HypVerificator * _vhStrategy;
	BayesFilter * _bayesFilter;

	Memory * _memory;

	FILE* _foutFloat;
	FILE* _foutInt;

	std::string _wDir;
	std::string _graphFileName;
};

#endif /* RTABMAP_H_ */

} // namespace rtabmap
