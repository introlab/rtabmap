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

#ifndef CTABMAP_H_
#define CTABMAP_H_

#include "rtabmap/core/RtabmapExp.h" // DLL export/import defines

#include "utilite/UThreadNode.h"
#include "utilite/UEventsHandler.h"
#include "utilite/USemaphore.h"
#include "utilite/UMutex.h"
#include "utilite/UVariant.h"
#include "rtabmap/core/RtabmapEvent.h"
#include "rtabmap/core/Parameters.h"
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
class SMState;

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
		kStateDeletingMemory
	};

	enum VhStrategy {kVhNone, kVhSim, kVhEpipolar, kVhUndef};

	static const char * kDefaultIniFileName;
	static const char * kDefaultIniFilePath;

public:
	static std::string getVersion();
	static std::string getIniFilePath();
	static void readParameters(const char * configFile, ParametersMap & parameters);
	static void writeParameters(const char * configFile, const ParametersMap & parameters);

public:
	Rtabmap();
	virtual ~Rtabmap();

	// ownership is transferred
	void process(SMState * data);
	void dumpData();

	void init(const ParametersMap & param);
	void init(const char * configFile = 0);

	const std::string & getWorkingDir() const {return _wDir;}
	int getLoopClosureId() const;
	int getLastSignatureId() const;
	const std::list<std::vector<float> > & getActions() const {return _actions;}
	std::list<int> getWorkingMem() const;
	std::set<int> getStMem() const;
	std::map<int, int> getWeights() const;
	int getTotalMemSize() const;
	const std::string & getGraphFileName() const {return _graphFileName;}

	void setReactivationDisabled(bool reactivationDisabled);
	void setMaxTimeAllowed(float maxTimeAllowed); // in sec
	void setDataBufferSize(int size);
	void setWorkingDirectory(std::string path);
	void setGraphFileName(const std::string & fileName) {_graphFileName = fileName;}

	void adjustLikelihood(std::map<int, float> & likelihood) const;
	void selectHypotheses(const std::map<int, float> & posterior,
						  std::list<std::pair<int, float> > & hypotheses,
						  bool useNeighborSum) const;

protected:
	virtual void handleEvent(UEvent * anEvent);

private:
	virtual void mainLoop();
	virtual void killCleanup();
	virtual void startInit();
	void process();
	void resetMemory();
	void deleteMemory();
	void addSMState(SMState * data); // ownership is transferred
	SMState * getSMState();
	void setupLogFiles(bool overwrite = false);
	void releaseAllStrategies();
	void pushNewState(State newState, const ParametersMap & parameters = ParametersMap());
	void dumpPrediction() const;
	void parseParameters(const ParametersMap & parameters);

private:
	// Modifiable parameters
	bool _publishStats;
	bool _reactivationDisabled;
	float _maxTimeAllowed; // in sec
	int _smStateBufferMaxSize;
	unsigned int _minMemorySizeForLoopDetection;
	float _loopThr;
	float _loopRatio;
	float _remThr;
	bool _localGraphCleaned;
	unsigned int _maxRetrieved;
	bool _actionsByTime;
	bool _actionsSentRejectHyp;
	float _confidenceThr;

	int _lcHypothesisId;
	int _reactivateId;
	float _highestHypothesisValue;
	unsigned int _spreadMargin;
	int _lastLoopClosureId;
	std::list<std::vector<float> > _actions;

	UMutex _stateMutex;
	std::stack<State> _state;
	std::stack<ParametersMap> _stateParam;

	std::list<SMState *> _smStateBuffer;
	UMutex _smStateBufferMutex;
	USemaphore _newSMStateSem;

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

#endif /* CTABMAP_H_ */

} // namespace rtabmap
