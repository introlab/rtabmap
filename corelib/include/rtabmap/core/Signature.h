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

//TODO : add copy constructor

namespace rtabmap
{

class RTABMAP_EXP NeighborLink
{
public:
	NeighborLink(int id, const std::list<std::vector<float> > & actions = std::list<std::vector<float> >(), const std::vector<int> & baseIds = std::vector<int>()) :
		_id(id),
		_actions(actions),
		_baseIds(baseIds)
	{}
	virtual ~NeighborLink() {}

	int id() const {return _id;}
	const std::list<std::vector<float> > & actions() const {return _actions;}
	const std::vector<int> & baseIds() const {return _baseIds;}
	bool updateIds(int idFrom, int idTo);

private:
	int _id;
	std::list<std::vector<float> > _actions;
	std::vector<int> _baseIds; // first is the nearest
};

class Memory;
typedef std::multimap<int, NeighborLink> NeighborsMultiMap;

class RTABMAP_EXP Signature
{
public:
	static CvMat * compressImage(const IplImage * image);
	static IplImage * decompressImage(const CvMat * imageCompressed);

public:
	virtual ~Signature();

	/**
	 * Must return a value between >=0 and <=1 (1 means 100% similarity)
	 */
	virtual float compareTo(const Signature * signature) const = 0;
	virtual bool isBadSignature() const = 0;
	virtual std::string signatureType() const = 0;

	const IplImage * getImage() const;
	void setImage(const IplImage * image);

	int id() const {return _id;}

	void addNeighbors(const NeighborsMultiMap & neighbors);
	void addNeighbor(const NeighborLink & neighbor);
	void removeNeighbor(int neighborId) {if(_neighbors.erase(neighborId)) _neighborsModified = true;}
	bool hasNeighbor(int neighborId) const {return _neighbors.find(neighborId) != _neighbors.end();}
	void setWeight(int weight) {if(_weight!=weight)_modified=true;_weight = weight;}
	void setLoopClosureIds(const std::set<int> & loopClosureIds) {_loopClosureIds = loopClosureIds;_modified=true;}
	void addLoopClosureId(int loopClosureId) {if(loopClosureId && _loopClosureIds.insert(loopClosureId).second)_modified=true;}
	void removeLoopClosureId(int loopClosureId) {if(loopClosureId && _loopClosureIds.erase(loopClosureId))_modified=true;}
	bool hasLoopClosureId(int loopClosureId) const {return _loopClosureIds.find(loopClosureId) != _loopClosureIds.end();}
	void setChildLoopClosureIds(std::set<int> & childLoopClosureIds) {_childLoopClosureIds = childLoopClosureIds;_modified=true;}
	void addChildLoopClosureId(int childLoopClosureId) {if(childLoopClosureId && _childLoopClosureIds.insert(childLoopClosureId).second)_modified=true;}
	void setSaved(bool saved) {_saved = saved;}
	void setModified(bool modified) {_modified = modified; _neighborsModified = modified;}
	void changeNeighborIds(int idFrom, int idTo);

	const NeighborsMultiMap & getNeighbors() const {return _neighbors;}
	int getWeight() const {return _weight;}
	const std::set<int> & getLoopClosureIds() const {return _loopClosureIds;}
	const std::set<int> & getChildLoopClosureIds() const {return _childLoopClosureIds;}
	bool isSaved() const {return _saved;}
	bool isModified() const {return _modified || _neighborsModified;}
	bool isNeighborsModified() const {return _neighborsModified;}

protected:
	Signature(int id, const IplImage * image = 0, bool keepImage = false);

private:
	int _id;
	NeighborsMultiMap _neighbors; // id, neighborLink
	int _weight;
	std::set<int> _loopClosureIds;
	std::set<int> _childLoopClosureIds;
	IplImage * _image;
	bool _saved; // If it's saved to bd
	bool _modified;
	bool _neighborsModified; // Optimization when updating signatures in database
};


class KeypointDetector;
class VWDictionary;

class RTABMAP_EXP KeypointSignature :
	public Signature
{
public:
	KeypointSignature(
			const std::multimap<int, cv::KeyPoint> & words,
			int id,
			const IplImage * image = 0,
			bool keepRawData = false);
	KeypointSignature(int id);

	virtual ~KeypointSignature();

	virtual float compareTo(const Signature * signature) const;
	virtual bool isBadSignature() const;
	virtual std::string signatureType() const {return "KeypointSignature";};

	void removeAllWords();
	void removeWord(int wordId);
	void changeWordsRef(int oldWordId, int activeWordId);

	void setWords(const std::multimap<int, cv::KeyPoint> & words) {_enabled = false;_words = words;}
	bool isEnabled() const {return _enabled;}
	void setEnabled(bool enabled) {_enabled = enabled;}
	const std::multimap<int, cv::KeyPoint> & getWords() const {return _words;}
	const std::map<int, int> & getWordsChanged() const {return _wordsChanged;}


private:
	// Contains all words (Some can be duplicates -> if a word appears 2
	// times in the signature, it will be 2 times in this list)
	// Words match with the CvSeq keypoints and descriptors
	std::multimap<int, cv::KeyPoint> _words; // word <id, keypoint>
	std::map<int, int> _wordsChanged; // <oldId, newId>
	bool _enabled;

};

class RTABMAP_EXP SMSignature :
	public Signature
{
public:
	SMSignature(
		const std::vector<int> & sensors,
		const std::vector<unsigned char> & motionMask,
		int id,
		const IplImage * image = 0,
		bool keepRawData = false);
	SMSignature(int id);

	virtual ~SMSignature();

	virtual float compareTo(const Signature * signature) const;
	virtual bool isBadSignature() const;
	virtual std::string signatureType() const {return "SMSignature";};

	void setSensors(const std::vector<int> & sensors) {_sensors= sensors;}
	const std::vector<int> & getSensors() const {return _sensors;}

	void setMotionMask(const std::vector<unsigned char> & motionMask) {_motionMask= motionMask;}
	const std::vector<unsigned char> & getMotionMask() const {return _motionMask;}
private:
	std::vector<int> _sensors;
	std::vector<unsigned char> _motionMask;
};


} // namespace rtabmap
