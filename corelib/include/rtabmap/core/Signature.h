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
#include "rtabmap/core/Sensor.h"
#include "rtabmap/core/Actuator.h"
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
	NeighborLink(int toId, const std::vector<int> & baseIds = std::vector<int>(), const std::list<Actuator> & actuators = std::list<Actuator>(), int actuatorId = 0) :
		_toId(toId),
		_actuatorId(actuatorId),
		_actuators(actuators),
		_baseIds(baseIds)
	{}
	virtual ~NeighborLink() {}

	int toId() const {return _toId;}
	int actuatorId() const {return _actuatorId;}
	const std::list<Actuator> & actuators() const {return _actuators;}
	const std::vector<int> & baseIds() const {return _baseIds;}
	bool updateIds(int idFrom, int idTo);

private:
	int _toId;
	int _actuatorId;
	std::list<Actuator> _actuators;
	std::vector<int> _baseIds; // first is the nearest
};

class Memory;
typedef std::multimap<int, NeighborLink> NeighborsMultiMap;

class RTABMAP_EXP Signature
{
public:
	virtual ~Signature();

	/**
	 * Must return a value between >=0 and <=1 (1 means 100% similarity)
	 */
	virtual float compareTo(const Signature * signature) const = 0;
	virtual bool isBadSignature() const = 0;
	virtual std::string nodeType() const = 0;

	void setRawData(const std::list<Sensor> & rawData) {_rawData = rawData;}
	const std::list<Sensor> & getRawData() const {return _rawData;}

	int id() const {return _id;}

	void addNeighbors(const NeighborsMultiMap & neighbors);
	void addNeighbor(const NeighborLink & neighbor);
	void removeNeighbor(int neighborId) {
		if(_neighbors.erase(neighborId))
			_neighborsModified = true;
		_neighborsWithActuators.erase(neighborId);
		_neighborsAll.erase(neighborId);}
	bool hasNeighbor(int neighborId) const {return _neighbors.find(neighborId) != _neighbors.end();}
	void setWeight(int weight) {if(_weight!=weight)_modified=true;_weight = weight;}
	void setLoopClosureIds(const std::set<int> & loopClosureIds) {_loopClosureIds = loopClosureIds;_neighborsModified=true;}
	void addLoopClosureId(int loopClosureId) {if(loopClosureId && _loopClosureIds.insert(loopClosureId).second)_neighborsModified=true;}
	void removeLoopClosureId(int loopClosureId) {if(loopClosureId && _loopClosureIds.erase(loopClosureId))_neighborsModified=true;}
	bool hasLoopClosureId(int loopClosureId) const {return _loopClosureIds.find(loopClosureId) != _loopClosureIds.end();}
	void setChildLoopClosureIds(std::set<int> & childLoopClosureIds) {_childLoopClosureIds = childLoopClosureIds;_neighborsModified=true;}
	void addChildLoopClosureId(int childLoopClosureId) {if(childLoopClosureId && _childLoopClosureIds.insert(childLoopClosureId).second)_neighborsModified=true;}
	void setSaved(bool saved) {_saved = saved;}
	void setModified(bool modified) {_modified = modified; _neighborsModified = modified;}
	void changeNeighborIds(int idFrom, int idTo);

	const NeighborsMultiMap & getNeighbors() const {return _neighbors;}
	const std::set<int> & getNeighborsWithActuators() const {return _neighborsWithActuators;}
	const std::set<int> & getNeighborsAll() const {return _neighborsAll;}
	int getWeight() const {return _weight;}
	const std::set<int> & getLoopClosureIds() const {return _loopClosureIds;}
	const std::set<int> & getChildLoopClosureIds() const {return _childLoopClosureIds;}
	bool isSaved() const {return _saved;}
	bool isModified() const {return _modified || _neighborsModified;}
	bool isNeighborsModified() const {return _neighborsModified;}

protected:
	Signature(int id);
	Signature(int id, const std::list<Sensor> & rawData);

private:
	int _id;
	NeighborsMultiMap _neighbors; // id, neighborLink
	std::set<int> _neighborsWithActuators; // Hack, to increase efficiency of Memory::getNeighborIds()
	std::set<int> _neighborsAll; // Hack, to increase efficiency of Memory::getNeighborIds()
	int _weight;
	std::set<int> _loopClosureIds;
	std::set<int> _childLoopClosureIds;
	std::list<Sensor> _rawData;
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
	KeypointSignature(int id);
	KeypointSignature(
			const std::multimap<int, cv::KeyPoint> & words,
			int id);
	KeypointSignature(
			const std::multimap<int, cv::KeyPoint> & words,
			int id,
			const std::list<Sensor> & sensors);

	virtual ~KeypointSignature();

	virtual float compareTo(const Signature * signature) const;
	virtual bool isBadSignature() const;
	virtual std::string nodeType() const {return "KeypointSignature";};

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
		const std::list<std::vector<int> > & data,
		int id);
	SMSignature(
		const std::list<std::vector<int> > & data,
		int id,
		const std::list<Sensor> & rawData);
	SMSignature(int id);

	virtual ~SMSignature();

	virtual float compareTo(const Signature * signature) const;
	virtual bool isBadSignature() const;
	virtual std::string nodeType() const {return "SMSignature";};

	void setSensors(const std::list<std::vector<int> > & data) {_data = data;}
	const std::list<std::vector<int> > & getData() const {return _data;}
private:
	std::list<std::vector<int> > _data;
};


} // namespace rtabmap
