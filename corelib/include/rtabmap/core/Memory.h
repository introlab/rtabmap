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

#ifndef MEMORY_H_
#define MEMORY_H_

#include "rtabmap/core/RtabmapExp.h" // DLL export/import defines

#include "utilite/UEventsHandler.h"
#include "rtabmap/core/Parameters.h"
#include "utilite/UVariant.h"
#include <typeinfo>
#include <list>
#include <map>
#include <set>
#include "utilite/UStl.h"
#include <opencv2/core/core.hpp>
#include "rtabmap/core/Sensor.h"
#include "rtabmap/core/Actuator.h"

namespace rtabmap {

class Signature;
class NeighborLink;
class DBDriver;
class Node;

class RTABMAP_EXP Memory
{
public:
	static const int kIdStart;
	static const int kIdVirtual;
	static const int kIdInvalid;

public:
	Memory(const ParametersMap & parameters = ParametersMap());
	virtual ~Memory();

	virtual void parseParameters(const ParametersMap & parameters);
	bool update(const std::list<Sensor> & sensors,
			const std::list<Actuator> & actuators,
			std::map<std::string, float> & stats);
	virtual bool init(const std::string & dbDriverName,
			const std::string & dbUrl,
			bool dbOverwritten = false,
			const ParametersMap & parameters = ParametersMap());
	virtual std::map<int, float> computeLikelihood(const Signature * signature,
			const std::list<int> & ids,
			float & maximumScore);
	virtual int forget(const std::set<int> & ignoredIds = std::set<int>());
	virtual std::set<int> reactivateSignatures(const std::list<int> & ids,
			unsigned int maxLoaded,
			double & timeDbAccess);

	int cleanup(const std::list<int> & ignoredIds = std::list<int>());
	void emptyTrash();
	void joinTrashThread();
	bool addLoopClosureLink(int oldId, int newId);
	std::map<int, int> getNeighborsId(double & dbAccessTime,
			int signatureId,
			unsigned int margin,
			int maxCheckedInDatabase = -1,
			bool onlyWithActions = false,
			bool incrementMarginOnLoop = false,
			bool ignoreSTM = true,
			bool ignoreLoopIds = false) const;
	float compareOneToOne(const std::vector<int> & idsA, const std::vector<int> & idsB);

	//getters
	unsigned int getWorkingMemSize() const {return _workingMem.size();}
	unsigned int getStMemSize() const {return _stMem.size();};
	const std::set<int> & getWorkingMem() const {return _workingMem;}
	const std::set<int> & getStMem() const {return _stMem;}
	std::list<NeighborLink> getNeighborLinks(int signatureId,
			bool ignoreNeighborByLoopClosure = false,
			bool lookInDatabase = false,
			bool onlyWithActions = false) const;
	void getLoopClosureIds(int signatureId,
			std::set<int> & loopClosureIds,
			std::set<int> & childLoopClosureIds,
			bool lookInDatabase = false) const;
	bool isRawDataKept() const {return _rawDataKept;}
	float getSimilarityThr() const {return _similarityThreshold;}
	std::map<int, int> getWeights() const;
	int getWeight(int id) const;
	const std::vector<int> & getLastBaseIds() const {return _lastBaseIds;}
	float getSimilarityOnlyLast() const {return _similarityOnlyLast;}
	const std::map<int, std::map<int, float> > & getSimilaritiesMap() const {return _similaritiesMap;}
	const Signature * getLastSignature() const;
	int getDatabaseMemoryUsed() const; // in bytes
	double getDbSavingTime() const;
	std::list<Sensor> getRawData(int id) const;
	bool isCommonSignatureUsed() const {return _commonSignatureUsed;}
	std::set<int> getAllSignatureIds() const;
	bool memoryChanged() const {return _memoryChanged;}
	const Signature * getSignature(int id) const;
	bool isInSTM(int signatureId) const {return _stMem.find(signatureId) != _stMem.end();}
	bool isInWM(int signatureId) const {return _workingMem.find(signatureId) != _workingMem.end();}
	bool isInLTM(int signatureId) const {return !this->isInSTM(signatureId) && !this->isInWM(signatureId);}

	//setters
	void setSimilarityThreshold(float similarityThreshold);
	void setSimilarityOnlyLast(int similarityOnlyLast) {_similarityOnlyLast = similarityOnlyLast;}
	void setOldSignatureRatio(float oldSignatureRatio);
	void setMaxStMemSize(unsigned int maxStMemSize);
	void setRecentWmRatio(float recentWmRatio);
	void setCommonSignatureUsed(bool commonSignatureUsed);
	void setRawDataKept(bool rawDataKept) {_rawDataKept = rawDataKept;}

	void dumpMemoryTree(const char * fileNameTree) const;
	virtual void dumpMemory(std::string directory) const;
	virtual void dumpSignatures(const char * fileNameSign) const {}
	void generateGraph(const std::string & fileName, std::set<int> ids = std::set<int>());
	void cleanLocalGraph(int id, unsigned int margin);
	void cleanLTM(int maxDepth = 10);
	void createGraph(Node * parent,
			unsigned int maxDepth,
			const std::set<int> & endIds = std::set<int>());

protected:
	virtual void preUpdate();
	virtual void postUpdate() {}

	virtual void addSignatureToStm(Signature * signature,
			const std::list<Actuator> & actuators = std::list<Actuator>());
	virtual void clear();
	virtual void moveToTrash(Signature * s);
	virtual Signature * getSignatureLtMem(int id);

	void addSignatureToWm(Signature * signature);
	Signature * _getSignature(int id) const;
	std::list<Signature *> getRemovableSignatures(int count,
			const std::set<int> & ignoredIds = std::set<int>());
	int getNextId();
	void initCountId();
	void rehearsal(Signature * signature, std::map<std::string, float> & stats);

	const std::map<int, Signature*> & getSignatures() const {return _signatures;}

private:
	virtual void copyData(const Signature * from, Signature * to) = 0;
	virtual Signature * createSignature(int id,
			const std::list<Sensor> & sensors,
			bool keepRawData=false) = 0;

	void createVirtualSignature(Signature ** signature);
	void cleanGraph(const Node * root);
protected:
	DBDriver * _dbDriver;

private:
	// parameters
	float _similarityThreshold;
	bool _similarityOnlyLast;
	bool _rawDataKept;
	bool _incrementalMemory;
	unsigned int _maxStMemSize;
	bool _commonSignatureUsed;
	float _recentWmRatio;
	bool _dataMergedOnRehearsal;

	int _idCount;
	Signature * _lastSignature;
	int _lastLoopClosureId;
	bool _memoryChanged; // False by default, become true when Memory::update() is called.
	bool _merging;
	int _signaturesAdded;

	std::map<int, Signature *> _signatures; // TODO : check if a signature is already added? although it is not supposed to occur...
	std::set<int> _stMem; // id
	std::set<int> _workingMem; // id,age
	std::vector<int> _lastBaseIds;
	std::map<int, std::map<int, float> > _similaritiesMap;
};

} // namespace rtabmap

#endif /* MEMORY_H_ */
