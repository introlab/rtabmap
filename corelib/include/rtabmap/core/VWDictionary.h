/*
Copyright (c) 2010-2016, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
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

#pragma once

#include "rtabmap/core/rtabmap_core_export.h" // DLL export/import defines

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <list>
#include <set>
#include "rtabmap/core/Parameters.h"

namespace rtabmap
{

class DBDriver;
class VisualWord;
class FlannIndex;

class RTABMAP_CORE_EXPORT VWDictionary
{
public:
	enum NNStrategy{
		kNNFlannNaive,
		kNNFlannKdTree,
		kNNFlannLSH,
		kNNBruteForce,
		kNNBruteForceGPU,
		kNNUndef};
	static const int ID_START;
	static const int ID_INVALID;
	static std::string nnStrategyName(NNStrategy strategy)
	{
		switch(strategy) {
		case kNNFlannNaive:
			return "FLANN NAIVE";
		case kNNFlannKdTree:
			return "FLANN KD-TREE";
		case kNNFlannLSH:
			return "FLANN LSH";
		case kNNBruteForce:
			return "BRUTE FORCE";
		case kNNBruteForceGPU:
			return "BRUTE FORCE GPU";
		default:
			return "Unknown";
		}
	}

public:
	VWDictionary(const ParametersMap & parameters = ParametersMap());
	virtual ~VWDictionary();

	virtual void parseParameters(const ParametersMap & parameters);

	virtual void update();

	virtual std::list<int> addNewWords(
			const cv::Mat & descriptors,
			int signatureId);
	virtual void addWord(VisualWord * vw);

	std::vector<int> findNN(const std::list<VisualWord *> & vws) const;
	std::vector<int> findNN(const cv::Mat & descriptors) const;

	void addWordRef(int wordId, int signatureId);
	void removeAllWordRef(int wordId, int signatureId);
	const VisualWord * getWord(int id) const;
	VisualWord * getUnusedWord(int id) const;
	void setLastWordId(int id) {_lastWordId = id;}
	const std::map<int, VisualWord *> & getVisualWords() const {return _visualWords;}
	float getNndrRatio() const {return _nndrRatio;}
	unsigned int getNotIndexedWordsCount() const {return (int)_notIndexedWords.size();}
	int getLastIndexedWordId() const;
	int getTotalActiveReferences() const {return _totalActiveReferences;}
	unsigned int getIndexedWordsCount() const;
	unsigned int getIndexMemoryUsed() const; // KB
	unsigned long getMemoryUsed() const; //Bytes
	bool setNNStrategy(NNStrategy strategy); // Return true if the search tree has been re-initialized
	bool isIncremental() const {return _incrementalDictionary;}
	bool isIncrementalFlann() const {return _incrementalFlann;}
	void setIncrementalDictionary();
	void setFixedDictionary(const std::string & dictionaryPath);

	void exportDictionary(const char * fileNameReferences, const char * fileNameDescriptors) const;

	void clear(bool printWarningsIfNotEmpty = true);
	std::vector<VisualWord *> getUnusedWords() const;
	std::vector<int> getUnusedWordIds() const;
	unsigned int getUnusedWordsSize() const {return (int)_unusedWords.size();}
	void removeWords(const std::vector<VisualWord*> & words); // caller must delete the words
	void deleteUnusedWords();

public:
	static cv::Mat convertBinTo32F(const cv::Mat & descriptorsIn, bool byteToFloat = true);
	static cv::Mat convert32FToBin(const cv::Mat & descriptorsIn, bool byteToFloat = true);

protected:
	int getNextId();

protected:
	std::map<int, VisualWord *> _visualWords; //<id,VisualWord*>
	int _totalActiveReferences; // keep track of all references for updating the common signature

private:
	bool _incrementalDictionary;
	bool _incrementalFlann;
	float _rebalancingFactor;
	bool _byteToFloat;
	float _nndrRatio;
	std::string _dictionaryPath; // a pre-computed dictionary (.txt or .db)
	std::string _newDictionaryPath; // a pre-computed dictionary (.txt or .db)
	bool _newWordsComparedTogether;
	int _lastWordId;
	bool useDistanceL1_;
	FlannIndex * _flannIndex;
	cv::Mat _dataTree;
	NNStrategy _strategy;
	std::map<int ,int> _mapIndexId;
	std::map<int ,int> _mapIdIndex;
	std::map<int, VisualWord*> _unusedWords; //<id,VisualWord*>, note that these words stay in _visualWords
	std::set<int> _notIndexedWords; // Words that are not indexed in the dictionary
	std::set<int> _removedIndexedWords; // Words not anymore in the dictionary but still indexed in the dictionary
};

} // namespace rtabmap
