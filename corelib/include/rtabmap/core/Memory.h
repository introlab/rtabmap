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

#include "rtabmap/utilite/UEventsHandler.h"
#include "rtabmap/core/Parameters.h"
#include "rtabmap/core/Image.h"
#include "rtabmap/core/Link.h"
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
class KeypointDetector;
class KeypointDescriptor;
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
	bool update(const Image & image, Statistics * stats = 0);
	bool init(const std::string & dbUrl,
			bool dbOverwritten = false,
			const ParametersMap & parameters = ParametersMap(),
			bool postInitEvents = true);
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
			unsigned int margin,
			int maxCheckedInDatabase = -1,
			bool incrementMarginOnLoop = false,
			bool ignoreLoopIds = false,
			double * dbAccessTime = 0) const;
	void deleteLocation(int locationId);
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
	std::vector<unsigned char> getImage(int signatureId) const;
	void getImageDepth(
			int locationId, std::vector<unsigned char> & rgb,
			std::vector<unsigned char> & depth,
			std::vector<unsigned char> & depth2d,
			float & depthConstant,
			Transform & localTransform) const;
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
	int getVWDictionarySize() const;
	std::multimap<int, cv::KeyPoint> getWords(int signatureId) const;
	void extractKeypointsAndDescriptors(
			const cv::Mat & image,
			const cv::Mat & depth,
			float depthConstant,
			std::vector<cv::KeyPoint> & keypoints,
			cv::Mat & descriptors);

	void getMetricConstraints(
			const std::vector<int> & ids,
			std::map<int, Transform> & poses,
			std::multimap<int, Link> & links,
			bool lookInDatabase = false);
	Transform computeVisualTransform(int oldId, int newId) const;
	Transform computeVisualTransform(const Signature & oldS, const Signature & newS) const;
	Transform computeIcpTransform(int oldId, int newId, Transform guess, bool icp3D);
	Transform computeIcpTransform(const Signature & oldS, const Signature & newS, Transform guess, bool icp3D) const;
	Transform computeScanMatchingTransform(
			int newId,
			int oldId,
			const std::map<int, Transform> & poses);

private:
	void preUpdate();
	void addSignatureToStm(Signature * signature);
	void clear();
	void moveToTrash(Signature * s, bool saveToDatabase = true);

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
			const Image & image,
			bool keepRawData=false);

	//keypoint stuff
	void disableWordsRef(int signatureId, bool saveToDatabase = true);
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

	std::map<int, Signature *> _signatures; // TODO : check if a signature is already added? although it is not supposed to occur...
	std::set<int> _stMem; // id
	std::set<int> _workingMem; // id,age

	//Keypoint stuff
	VWDictionary * _vwd;
	KeypointDetector * _keypointDetector;
	KeypointDescriptor * _keypointDescriptor;
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
	int _icpDecimation;
	float _icpMaxDepth;
	float _icpVoxelSize;
	int _icpSamples;
	float _icpMaxCorrespondenceDistance;
	int _icpMaxIterations;
	float _icpMaxFitness;
	float _icp2MaxCorrespondenceDistance;
	int _icp2MaxIterations;
	float _icp2MaxFitness;
	float _icp2CorrespondenceRatio;
	float _icp2VoxelSize;
};

} // namespace rtabmap

#endif /* MEMORY_H_ */
