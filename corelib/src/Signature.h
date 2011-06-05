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

//TODO : add copy constructor

namespace rtabmap
{

class Memory;
typedef std::map<int, std::list<std::vector<float> > > NeighborsMap;

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

	void addNeighbors(const NeighborsMap & neighbors);
	void addNeighbor(int neighborId, const std::list<std::vector<float> > & actions);
	void removeNeighbor(int neighborId);
	bool hasNeighbor(int neighborId) const {return _neighbors.find(neighborId) != _neighbors.end();}
	void setWeight(int weight) {_weight = weight;}
	void setLoopClosureId(int loopClosureId) {_loopClosureId = loopClosureId;}
	void setWidth(int width) {_width = width;}
	void setHeight(int height) {_height = height;}
	void setSaved(bool saved) {_saved = saved;}

	const NeighborsMap & getNeighbors() const {return _neighbors;}
	int getWeight() const {return _weight;}
	int getLoopClosureId() const {return _loopClosureId;}
	int getWidth() const {return _width;}
	int getHeight() const {return _height;}
	bool isSaved() const {return _saved;}

protected:
	Signature(int id, const IplImage * image = 0, bool keepImage = false);

private:
	int _id;
	NeighborsMap _neighbors; // id, [action1, action2, ...] All actions must have the same length
	int _weight;
	int _loopClosureId;
	IplImage * _image;
	bool _saved; // If it's saved to bd
	int _width; // pixels
	int _height; // pixels
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

private:
	// Contains all words (Some can be duplicates -> if a word appears 2
	// times in the signature, it will be 2 times in this list)
	// Words match with the CvSeq keypoints and descriptors
	std::multimap<int, cv::KeyPoint> _words; // word <id, keypoint>
	bool _enabled;

};


} // namespace rtabmap
