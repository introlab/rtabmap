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

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <map>
#include <list>
#include <vector>
#include <set>

namespace rtabmap
{

class Memory;

class RTABMAP_EXP Signature
{
public:
	Signature(int id,
			const std::multimap<int, cv::KeyPoint> & words,
			const cv::Mat & image = cv::Mat());
	virtual ~Signature();

	/**
	 * Must return a value between >=0 and <=1 (1 means 100% similarity).
	 */
	float compareTo(const Signature * signature) const;
	bool isBadSignature() const;

	int id() const {return _id;}

	void addNeighbors(const std::set<int> & neighbors);
	void addNeighbor(int neighbor);
	void removeNeighbor(int neighborId);
	void removeNeighbors();
	bool hasNeighbor(int neighborId) const {return _neighbors.find(neighborId) != _neighbors.end();}
	void setWeight(int weight) {if(_weight!=weight)_modified=true;_weight = weight;}
	void setLoopClosureIds(const std::set<int> & loopClosureIds) {_loopClosureIds = loopClosureIds;_neighborsModified=true;}
	void addLoopClosureId(int loopClosureId) {if(loopClosureId && _loopClosureIds.insert(loopClosureId).second)_neighborsModified=true;}
	void removeLoopClosureId(int loopClosureId) {if(loopClosureId && _loopClosureIds.erase(loopClosureId))_neighborsModified=true;}
	void removeChildLoopClosureId(int childLoopClosureId) {if(childLoopClosureId && _childLoopClosureIds.erase(childLoopClosureId))_neighborsModified=true;}
	bool hasLoopClosureId(int loopClosureId) const {return _loopClosureIds.find(loopClosureId) != _loopClosureIds.end();}
	void setChildLoopClosureIds(std::set<int> & childLoopClosureIds) {_childLoopClosureIds = childLoopClosureIds;_neighborsModified=true;}
	void addChildLoopClosureId(int childLoopClosureId) {if(childLoopClosureId && _childLoopClosureIds.insert(childLoopClosureId).second)_neighborsModified=true;}
	void setSaved(bool saved) {_saved = saved;}
	void setModified(bool modified) {_modified = modified; _neighborsModified = modified;}
	void changeNeighborIds(int idFrom, int idTo);

	const std::set<int> & getNeighbors() const {return _neighbors;}
	int getWeight() const {return _weight;}
	const std::set<int> & getLoopClosureIds() const {return _loopClosureIds;}
	const std::set<int> & getChildLoopClosureIds() const {return _childLoopClosureIds;}
	bool isSaved() const {return _saved;}
	bool isModified() const {return _modified || _neighborsModified;}
	bool isNeighborsModified() const {return _neighborsModified;}

	//visual words stuff
	void removeAllWords();
	void removeWord(int wordId);
	void changeWordsRef(int oldWordId, int activeWordId);
	void setWords(const std::multimap<int, cv::KeyPoint> & words) {_enabled = false;_words = words;}
	bool isEnabled() const {return _enabled;}
	void setEnabled(bool enabled) {_enabled = enabled;}
	const std::multimap<int, cv::KeyPoint> & getWords() const {return _words;}
	const std::map<int, int> & getWordsChanged() const {return _wordsChanged;}
	void setImage(const cv::Mat & image) {_image = image;}
	const cv::Mat & getImage() const {return _image;}

private:
	int _id;
	std::set<int> _neighbors; // id
	int _weight;
	std::set<int> _loopClosureIds;
	std::set<int> _childLoopClosureIds;
	bool _saved; // If it's saved to bd
	bool _modified;
	bool _neighborsModified; // Optimization when updating signatures in database

	// Contains all words (Some can be duplicates -> if a word appears 2
	// times in the signature, it will be 2 times in this list)
	// Words match with the CvSeq keypoints and descriptors
	std::multimap<int, cv::KeyPoint> _words; // word <id, keypoint>
	std::map<int, int> _wordsChanged; // <oldId, newId>
	bool _enabled;
	cv::Mat _image;
};

} // namespace rtabmap
