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

#ifndef MEMORY_H_
#define MEMORY_H_

#include "rtabmap/core/RtabmapExp.h" // DLL export/import defines

#include "rtabmap/utilite/UEventsHandler.h"
#include "rtabmap/core/Parameters.h"
#include "rtabmap/core/SensorData.h"
#include "rtabmap/core/Link.h"
#include "rtabmap/core/Features2d.h"
#include <typeinfo>
#include <list>
#include <map>
#include <set>
#include "rtabmap/utilite/UStl.h"
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

namespace rtabmap {

class Signature;
class DBDriver;
class GraphNode;
class VWDictionary;
class VisualWord;
class Feature2D;
class Statistics;

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
	bool update(const SensorData & data, Statistics * stats = 0);
	bool init(const std::string & dbUrl,
			bool dbOverwritten = false,
			const ParametersMap & parameters = ParametersMap(),
			bool postInitClosingEvents = false);
	std::map<int, float> computeLikelihood(const Signature * signature,
			const std::list<int> & ids);
	int incrementMapId();

	std::list<int> forget(const std::set<int> & ignoredIds = std::set<int>());
	std::set<int> reactivateSignatures(const std::list<int> & ids, unsigned int maxLoaded, double & timeDbAccess);

	std::list<int> cleanup(const std::list<int> & ignoredIds = std::list<int>());
	void emptyTrash();
	void joinTrashThread();
	bool addLoopClosureLink(int oldId, int newId, const Transform & transform, bool global);
	void updateNeighborLink(int fromId, int toId, const Transform & transform);
	std::map<int, int> getNeighborsId(int signatureId,
			int margin,
			int maxCheckedInDatabase = -1,
			bool incrementMarginOnLoop = false,
			bool ignoreLoopIds = false,
			double * dbAccessTime = 0) const;
	void deleteLocation(int locationId, std::list<int> * deletedWords = 0);
	void rejectLoopClosure(int oldId, int newId);

	//getters
	const std::set<int> & getWorkingMem() const {return _workingMem;}
	const std::set<int> & getStMem() const {return _stMem;}
	int getMaxStMemSize() const {return _maxStMemSize;}
	void getPose(int locationId,
			Transform & pose,
			bool lookInDatabase = false) const;
	std::map<int, Transform> getNeighborLinks(int signatureId,
			bool ignoreNeighborByLoopClosure = false,
			bool lookInDatabase = false) const;
	void getLoopClosureIds(int signatureId,
			std::map<int, Transform> & loopClosureIds,
			std::map<int, Transform> & childLoopClosureIds,
			bool lookInDatabase = false) const;
	bool isRawDataKept() const {return _rawDataKept;}
	float getSimilarityThreshold() const {return _similarityThreshold;}
	std::map<int, int> getWeights() const;
	int getLastSignatureId() const;
	const Signature * getLastWorkingSignature() const;
	int getDatabaseMemoryUsed() const; // in bytes
	double getDbSavingTime() const;
	int getMapId(int signatureId) const;
	cv::Mat getImageCompressed(int signatureId) const;
	Signature getSignatureData(int locationId, bool uncompressedData = false);
	std::set<int> getAllSignatureIds() const;
	bool memoryChanged() const {return _memoryChanged;}
	bool isIncremental() const {return _incrementalMemory;}
	const Signature * getSignature(int id) const;
	bool isInSTM(int signatureId) const {return _stMem.find(signatureId) != _stMem.end();}
	bool isInWM(int signatureId) const {return _workingMem.find(signatureId) != _workingMem.end();}
	bool isInLTM(int signatureId) const {return !this->isInSTM(signatureId) && !this->isInWM(signatureId);}
	bool isIDsGenerated() const {return _generateIds;}
	int getLastGlobalLoopClosureParentId() const {return _lastGlobalLoopClosureParentId;}
	int getLastGlobalLoopClosureChildId() const {return _lastGlobalLoopClosureChildId;}
	const Feature2D * getFeature2D() const {return _feature2D;}

	void setRoi(const std::string & roi);

	void dumpMemoryTree(const char * fileNameTree) const;
	virtual void dumpMemory(std::string directory) const;
	virtual void dumpSignatures(const char * fileNameSign, bool words3D) const;
	void dumpDictionary(const char * fileNameRef, const char * fileNameDesc) const;

	void generateGraph(const std::string & fileName, std::set<int> ids = std::set<int>());
	void createGraph(GraphNode * parent,
			unsigned int maxDepth,
			const std::set<int> & endIds = std::set<int>());

	//keypoint stuff
	const VWDictionary * getVWDictionary() const;
	Feature2D::Type getFeatureType() const {return _featureType;}

	// RGB-D stuff
	void getMetricConstraints(
			const std::vector<int> & ids,
			std::map<int, Transform> & poses,
			std::multimap<int, Link> & links,
			bool lookInDatabase = false);
	float getBowInlierDistance() const {return _bowInlierDistance;}
	int getBowIterations() const {return _bowIterations;}
	int getBowMinInliers() const {return _bowMinInliers;}
	float getBowMaxDepth() const {return _bowMaxDepth;}
	bool getBowForce2D() const {return _bowForce2D;}
	Transform computeVisualTransform(int oldId, int newId, std::string * rejectedMsg = 0, int * inliers = 0) const;
	Transform computeVisualTransform(const Signature & oldS, const Signature & newS, std::string * rejectedMsg = 0, int * inliers = 0) const;
	Transform computeIcpTransform(int oldId, int newId, Transform guess, bool icp3D, std::string * rejectedMsg = 0);
	Transform computeIcpTransform(const Signature & oldS, const Signature & newS, Transform guess, bool icp3D, std::string * rejectedMsg = 0) const;
	Transform computeScanMatchingTransform(
			int newId,
			int oldId,
			const std::map<int, Transform> & poses,
			std::string * rejectedMsg = 0);

private:
	void preUpdate();
	void addSignatureToStm(Signature * signature);
	void clear();
	void moveToTrash(Signature * s, bool saveToDatabase = true, std::list<int> * deletedWords = 0);

	void addSignatureToWm(Signature * signature);
	Signature * _getSignature(int id) const;
	std::list<Signature *> getRemovableSignatures(int count,
			const std::set<int> & ignoredIds = std::set<int>());
	int getNextId();
	void initCountId();
	void rehearsal(Signature * signature, Statistics * stats = 0);
	bool rehearsalMerge(int oldId, int newId);

	const std::map<int, Signature*> & getSignatures() const {return _signatures;}

	void copyData(const Signature * from, Signature * to);
	Signature * createSignature(
			const SensorData & data,
			bool keepRawData=false,
			Statistics * stats = 0);

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
	bool _rawDataKept;
	bool _keepRehearsedNodesInDb;
	bool _incrementalMemory;
	int _maxStMemSize;
	float _recentWmRatio;
	bool _idUpdatedToNewOneRehearsal;
	bool _generateIds;
	bool _badSignaturesIgnored;

	int _idCount;
	int _idMapCount;
	Signature * _lastSignature;
	int _lastGlobalLoopClosureParentId;
	int _lastGlobalLoopClosureChildId;
	bool _memoryChanged; // False by default, become true when Memory::update() is called.
	int _signaturesAdded;
	bool _postInitClosingEvents;

	std::map<int, Signature *> _signatures; // TODO : check if a signature is already added? although it is not supposed to occur...
	std::set<int> _stMem; // id
	std::set<int> _workingMem; // id,age

	//Keypoint stuff
	VWDictionary * _vwd;
	Feature2D * _feature2D;
	Feature2D::Type _featureType;
	float _badSignRatio;;
	bool _tfIdfLikelihoodUsed;
	bool _parallelized;
	float _wordsMaxDepth; // 0=inf
	int _wordsPerImageTarget; // <0=none, 0=inf
	std::vector<float> _roiRatios; // size 4

	// RGBD-SLAM stuff
	int _bowMinInliers;
	float _bowInlierDistance;
	int _bowIterations;
	float _bowMaxDepth;
	bool _bowForce2D;
	int _icpDecimation;
	float _icpMaxDepth;
	float _icpVoxelSize;
	int _icpSamples;
	float _icpMaxCorrespondenceDistance;
	int _icpMaxIterations;
	float _icpMaxFitness;
	bool _icpPointToPlane;
	int _icpPointToPlaneNormalNeighbors;
	float _icp2MaxCorrespondenceDistance;
	int _icp2MaxIterations;
	float _icp2MaxFitness;
	float _icp2CorrespondenceRatio;
	float _icp2VoxelSize;

	// Stereo stuff
	int _stereoFlowWinSize;
	int _stereoFlowIterations;
	double _stereoFlowEpsilon;
	int _stereoFlowMaxLevel;
	float _stereoMaxSlope;

	int _subPixWinSize;
	int _subPixIterations;
	double _subPixEps;
};

} // namespace rtabmap

#endif /* MEMORY_H_ */
