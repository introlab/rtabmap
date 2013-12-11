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

#pragma once

#include "rtabmap/core/RtabmapExp.h" // DLL export/import defines

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <list>
#include <set>
#include "rtabmap/core/Parameters.h"

namespace rtabmap
{

class FlannNN;
class DBDriver;
class VisualWord;

class RTABMAP_EXP VWDictionary
{
public:
	enum NNStrategy{kNNNaive, kNNFlannKdTree, kNNUndef};
	static const int ID_START;
	static const int ID_INVALID;

public:
	VWDictionary(const ParametersMap & parameters = ParametersMap());
	virtual ~VWDictionary();

	virtual void parseParameters(const ParametersMap & parameters);

	virtual void update();

	virtual std::list<int> addNewWords(
			const cv::Mat & descriptors,
			int signatureId);
	virtual void addWord(VisualWord * vw);

	virtual std::vector<int> findNN(const std::list<VisualWord *> & vws, bool searchInNewlyAddedWords = true) const;
	void naiveNNSearch(const std::list<VisualWord *> & words, const float * d, int length, std::map<float, int> & results, unsigned int k) const;

	void addWordRef(int wordId, int signatureId);
	void removeAllWordRef(int wordId, int signatureId);
	const VisualWord * getWord(int id) const;
	VisualWord * getUnusedWord(int id) const;
	void setLastWordId(int id) {_lastWordId = id;}
	void getCommonWords(unsigned int nbCommonWords, int totalSign, std::list<int> & commonWords) const;
	const std::map<int, VisualWord *> & getVisualWords() const {return _visualWords;}
	float getMinDist() const {return _minDist;}
	bool isMinDistUsed() const {return _minDistUsed;}
	void setMinDistUsed(bool used) {_minDistUsed = used;}
	void setNndrUsed(bool used) {_nndrUsed = used;}
	bool isNndrUsed() const {return _nndrUsed;}
	float getNndrRatio() {return _nndrRatio;}
	unsigned int getNotIndexedWordsCount() const {return _notIndexedWords.size();}
	int getLastIndexedWordId() const;
	int getTotalActiveReferences() const {return _totalActiveReferences;}
	void setNNStrategy(NNStrategy strategy, const ParametersMap & parameters = ParametersMap());
	NNStrategy nnStrategy() const;
	bool isIncremental() const {return _incrementalDictionary;}
	void setIncrementalDictionary(bool incrementalDictionary, const std::string & dictionaryPath);

	void exportDictionary(const char * fileNameReferences, const char * fileNameDescriptors) const;

	void clear();
	std::vector<VisualWord *> getUnusedWords() const;
	unsigned int getUnusedWordsSize() const {return _unusedWords.size();}
	void removeWords(const std::vector<VisualWord*> & words); // caller must delete the words

protected:
	int getNextId();

protected:
	std::map<int, VisualWord *> _visualWords; //<id,VisualWord*>
	int _totalActiveReferences; // keep track of all references for updating the common signature

private:
	bool _incrementalDictionary;
	bool _minDistUsed;
	float _minDist; //euclidean distance ^ 2
	bool _nndrUsed;
	float _nndrRatio;
	unsigned int _maxLeafs;
	std::string _dictionaryPath; // a pre-computed dictionary (.txt)
	int _dim;
	int _lastWordId;
	FlannNN * _nn;
	cv::Mat _dataTree;
	std::map<int ,int> _mapIndexId;
	std::map<int, VisualWord*> _unusedWords; //<id,VisualWord*>, note that these words stay in _visualWords
	std::set<int> _notIndexedWords; // Words that are not indexed in the dictionary
	std::set<int> _removedIndexedWords; // Words not anymore in the dictionary but still indexed in the dictionary
};

} // namespace rtabmap
