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
#include "rtabmap/core/Image.h"
#include <typeinfo>
#include <list>
#include <map>
#include <set>
#include "utilite/UStl.h"
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

namespace rtabmap {

class Signature;
class DBDriver;
class GraphNode;
class VWDictionary;
class VisualWord;
class KeypointDetector;
class KeypointDescriptor;

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
	bool update(const Image & image, std::map<std::string, float> & stats);
	bool init(const std::string & dbUrl,
			bool dbOverwritten = false,
			const ParametersMap & parameters = ParametersMap());
	std::map<int, float> computeLikelihood(const Signature * signature,
			const std::list<int> & ids);

	int forget(const std::set<int> & ignoredIds = std::set<int>());
	std::set<int> reactivateSignatures(const std::list<int> & ids, unsigned int maxLoaded, double & timeDbAccess);

	int cleanup(const std::list<int> & ignoredIds = std::list<int>());
	void emptyTrash();
	void joinTrashThread();
	bool addLoopClosureLink(int oldId, int newId);
	std::map<int, int> getNeighborsId(int signatureId,
			unsigned int margin,
			int maxCheckedInDatabase = -1,
			bool incrementMarginOnLoop = false,
			bool ignoreLoopIds = false,
			double * dbAccessTime = 0) const;
	void deleteLastLocation();
	void rejectLastLoopClosure();

	//getters
	const std::set<int> & getWorkingMem() const {return _workingMem;}
	const std::set<int> & getStMem() const {return _stMem;}
	int getMaxStMemSize() const {return _maxStMemSize;}
	std::set<int> getNeighborLinks(int signatureId,
			bool ignoreNeighborByLoopClosure = false,
			bool lookInDatabase = false) const;
	void getLoopClosureIds(int signatureId,
			std::set<int> & loopClosureIds,
			std::set<int> & childLoopClosureIds,
			bool lookInDatabase = false) const;
	bool isRawDataKept() const {return _rawDataKept;}
	float getSimilarityThreshold() const {return _similarityThreshold;}
	std::map<int, int> getWeights() const;
	float getSimilarityOnlyWithLast() const {return _rehearsalOnlyWithLast;}
	const Signature * getLastSignature() const;
	int getDatabaseMemoryUsed() const; // in bytes
	double getDbSavingTime() const;
	cv::Mat getImage(int signatureId) const;
	std::set<int> getAllSignatureIds() const;
	bool memoryChanged() const {return _memoryChanged;}
	const Signature * getSignature(int id) const;
	bool isInSTM(int signatureId) const {return _stMem.find(signatureId) != _stMem.end();}
	bool isInWM(int signatureId) const {return _workingMem.find(signatureId) != _workingMem.end();}
	bool isInLTM(int signatureId) const {return !this->isInSTM(signatureId) && !this->isInWM(signatureId);}

	//setters
	void setSimilarityThreshold(float similarity);
	void setSimilarityOnlyLast(int rehearsalOnlyWithLast) {_rehearsalOnlyWithLast = rehearsalOnlyWithLast;}
	void setOldSignatureRatio(float oldSignatureRatio);
	void setMaxStMemSize(unsigned int maxStMemSize);
	void setRecentWmRatio(float recentWmRatio);
	void setRawDataKept(bool rawDataKept) {_rawDataKept = rawDataKept;}

	void dumpMemoryTree(const char * fileNameTree) const;
	virtual void dumpMemory(std::string directory) const;
	virtual void dumpSignatures(const char * fileNameSign) const;
	void dumpDictionary(const char * fileNameRef, const char * fileNameDesc) const;

	void generateGraph(const std::string & fileName, std::set<int> ids = std::set<int>());
	void createGraph(GraphNode * parent,
			unsigned int maxDepth,
			const std::set<int> & endIds = std::set<int>());

	//keypoint stuff
	int getVWDictionarySize() const;
	std::multimap<int, cv::KeyPoint> getWords(int signatureId) const;

protected:
	void preUpdate();
	void addSignatureToStm(Signature * signature);
	void clear();
	void moveToTrash(Signature * s);

	void addSignatureToWm(Signature * signature);
	Signature * _getSignature(int id) const;
	std::list<Signature *> getRemovableSignatures(int count,
			const std::set<int> & ignoredIds = std::set<int>());
	int getNextId();
	void initCountId();
	void rehearsal(Signature * signature, std::map<std::string, float> & stats);

	const std::map<int, Signature*> & getSignatures() const {return _signatures;}

private:
	void copyData(const Signature * from, Signature * to);
	Signature * createSignature(
			const Image & image,
			bool keepRawData=false);

	//keypoint stuff
	void disableWordsRef(int signatureId);
	void enableWordsRef(const std::list<int> & signatureIds);
	void cleanUnusedWords();
	int getNi(int signatureId) const;

protected:
	DBDriver * _dbDriver;

private:
	// parameters
	float _similarityThreshold;
	bool _rehearsalOnlyWithLast;
	bool _rawDataKept;
	bool _incrementalMemory;
	int _maxStMemSize;
	float _recentWmRatio;
	bool _oldDataKeptOnRehearsal;
	bool _idUpdatedToNewOneRehearsal;

	int _idCount;
	Signature * _lastSignature;
	int _lastLoopClosureId;
	bool _memoryChanged; // False by default, become true when Memory::update() is called.
	int _signaturesAdded;
	std::vector<std::pair<int, int> > _savedLoopClosureInfo; // size 3 or 0

	std::map<int, Signature *> _signatures; // TODO : check if a signature is already added? although it is not supposed to occur...
	std::set<int> _stMem; // id
	std::set<int> _workingMem; // id,age

	//Heypoint stuff
	VWDictionary * _vwd;
	KeypointDetector * _keypointDetector;
	KeypointDescriptor * _keypointDescriptor;
	bool _reactivatedWordsComparedToNewWords;
	float _badSignRatio;;
	bool _tfIdfLikelihoodUsed;
	bool _parallelized;
};

} // namespace rtabmap

#endif /* MEMORY_H_ */
