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

namespace rtabmap {

class Signature;
class DBDriver;
class Node;
class SMState;

class RTABMAP_EXP Memory
{
public:
	static const int kIdStart;
	static const int kIdVirtual;
	static const int kIdInvalid;

	enum MergingStrategy{kFullMerging, kUseOnlyFromMerging, kUseOnlyDestMerging};

public:
	Memory(const ParametersMap & parameters = ParametersMap());
	virtual ~Memory();

	virtual void parseParameters(const ParametersMap & parameters);
	bool update(const SMState * rawData, std::list<std::pair<std::string, float> > & stats);
	virtual bool init(const std::string & dbDriverName, const std::string & dbUrl, bool dbOverwritten = false, const ParametersMap & parameters = ParametersMap());
	virtual std::map<int, float> computeLikelihood(const Signature * signature, const std::set<int> & signatureIds = std::set<int>()) const;
	virtual int forget(const std::list<int> & ignoredIds = std::list<int>());
	virtual int reactivateSignatures(const std::list<int> & ids, unsigned int maxLoaded, unsigned int maxTouched);

	int cleanup(const std::list<int> & ignoredIds = std::list<int>());
	void emptyTrash();
	void joinTrashThread();
	void addLoopClosureLink(int oldId, int newId, bool rehearsal = false);
	void getNeighborsId(std::map<int,int> & ids, int signatureId, unsigned int margin, bool checkInDatabase = true, int ignoredId = 0) const;

	//getters
	unsigned int getWorkingMemSize() const {return _workingMem.size();}
	unsigned int getStMemSize() const {return _stMem.size();};
	const std::map<int, int> & getWorkingMem() const {return _workingMem;}
	const std::set<int> & getStMem() const {return _stMem;}
	std::list<int> getChildrenIds(int signatureId) const;
	bool isRawDataKept() const {return _rawDataKept;}
	std::map<int, int> getWeights() const;
	int getWeight(int id) const;
	float getSimilarityOnlyLast() const {return _similarityOnlyLast;}
	const Signature * getLastSignature() const;
	int getDatabaseMemoryUsed() const; // in bytes
	double getDbSavingTime() const;
	IplImage * getImage(int id) const;
	bool isDatabaseCleaned()  const {return _databaseCleaned;}
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
	void setDelayRequired(int delayRequired);
	void setRecentWmRatio(float recentWmRatio);
	void setCommonSignatureUsed(bool commonSignatureUsed);
	void setRawDataKept(bool rawDataKept) {_rawDataKept = rawDataKept;}

	void dumpMemoryTree(const char * fileNameTree) const;
	virtual void dumpMemory(std::string directory) const;
	virtual void dumpSignatures(const char * fileNameSign) const {}
	void generateGraph(const std::string & fileName, std::set<int> ids = std::set<int>());
	void cleanLocalGraph(int id, unsigned int margin);
	void cleanLTM(int maxDepth = 10);
	void createGraph(Node * parent, unsigned int maxDepth, const std::set<int> & endIds = std::set<int>());

protected:
	virtual void preUpdate();
	virtual void postUpdate() {}
	virtual void merge(const Signature * from, Signature * to, MergingStrategy s) = 0;

	virtual void addSignatureToStm(Signature * signature, const std::list<std::vector<float> > & actions = std::list<std::vector<float> >());
	virtual void clear();
	virtual void moveToTrash(Signature * s);
	virtual Signature * getSignatureLtMem(int id);

	void addSignatureToWm(Signature * signature);
	Signature * _getSignature(int id) const;
	Signature * _getLastSignature();
	Signature * getRemovableSignature(const std::list<int> & ignoredIds = std::list<int>(), bool onlyLoopedSignatures = false);
	int getNextId();
	void initCountId();
	int rehearsal(const Signature * signature, bool onlyLast, float & similarity);
	void touch(int signatureId);
	const std::map<int, Signature*> & getSignatures() const {return _signatures;}

private:
	void createVirtualSignature(Signature ** signature);
	virtual Signature * createSignature(int id, const SMState * rawData, bool keepRawData=false) = 0;
	void cleanGraph(const Node * root);
protected:
	DBDriver * _dbDriver;

private:
	float _similarityThreshold;
	bool _similarityOnlyLast;
	bool _rawDataKept;
	int _idCount;
	Signature * _lastSignature;
	int _lastLoopClosureId;
	bool _incrementalMemory;
	unsigned int _maxStMemSize;
	bool _commonSignatureUsed;
	bool _databaseCleaned; //if true, delete old signatures in the database
	int _delayRequired;
	float _recentWmRatio;
	bool _memoryChanged; // False by default, become true when Memory::update() is called.
	bool _merging;
	int _signaturesAdded;

	std::map<int, Signature *> _signatures; // TODO : check if a signature is already added? although it is not supposed to occur...
	std::set<int> _stMem;
	std::map<int, int> _workingMem; // id, timeStamp
};

} // namespace rtabmap

#endif /* MEMORY_H_ */
