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

#include <pcl/point_types.h>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <map>
#include <list>
#include <vector>
#include <set>

#include <rtabmap/core/Transform.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/Link.h>

namespace rtabmap
{

class RTABMAP_CORE_EXPORT Signature
{

public:
	Signature();
	Signature(int id,
			int mapId = -1,
			int weight = 0,
			double stamp = 0.0,
			const std::string & label = std::string(),
			const Transform & pose = Transform(),
			const Transform & groundTruthPose = Transform(),
			const SensorData & sensorData = SensorData());
	Signature(const SensorData & data);
	virtual ~Signature();

	/**
	 * Must return a value between >=0 and <=1 (1 means 100% similarity).
	 */
	float compareTo(const Signature & signature) const;
	bool isBadSignature() const;

	int id() const {return _id;}
	int mapId() const {return _mapId;}

	void setWeight(int weight) {_modified=_weight!=weight;_weight = weight;}
	int getWeight() const {return _weight;}

	void setLabel(const std::string & label) {_modified=_label.compare(label)!=0;_label = label;}
	const std::string & getLabel() const {return _label;}

	double getStamp() const {return _stamp;}

	void addLinks(const std::list<Link> & links);
	void addLinks(const std::map<int, Link> & links);
	void addLink(const Link & link);

	bool hasLink(int idTo, Link::Type type = Link::kUndef) const;

	void changeLinkIds(int idFrom, int idTo);

	void removeLinks(bool keepSelfReferringLinks = false);
	void removeLink(int idTo);
	void removeVirtualLinks();

	void addLandmark(const Link & landmark) {_landmarks.insert(std::make_pair(landmark.to(), landmark));}
	const std::map<int, Link> & getLandmarks() const {return _landmarks;}
	void removeLandmarks() {_landmarks.clear();}

	void setSaved(bool saved) {_saved = saved;}
	void setModified(bool modified) {_modified = modified; _linksModified = modified;}

	const std::multimap<int, Link> & getLinks() const {return _links;}
	bool isSaved() const {return _saved;}
	bool isModified() const {return _modified || _linksModified;}
	bool isLinksModified() const {return _linksModified;}

	//visual words stuff
	void removeAllWords();
	void changeWordsRef(int oldWordId, int activeWordId);
	void setWords(const std::multimap<int, int> & words, const std::vector<cv::KeyPoint> & keypoints, const std::vector<cv::Point3f> & words3, const cv::Mat & descriptors);
	bool isEnabled() const {return _enabled;}
	void setEnabled(bool enabled) {_enabled = enabled;}
	const std::multimap<int, int> & getWords() const {return _words;}
	const std::vector<cv::KeyPoint> & getWordsKpts() const {return _wordsKpts;}
	int getInvalidWordsCount() const {return _invalidWordsCount;}
	const std::map<int, int> & getWordsChanged() const {return _wordsChanged;}
	const cv::Mat & getWordsDescriptors() const {return _wordsDescriptors;}
	void setWordsDescriptors(const cv::Mat & descriptors);

	//metric stuff
	void setPose(const Transform & pose) {_pose = pose;}
	void setGroundTruthPose(const Transform & pose) {_groundTruthPose = pose;}
	void setVelocity(float vx, float vy, float vz, float vroll, float vpitch, float vyaw) {
		_velocity = std::vector<float>(6,0);
		_velocity[0]=vx;
		_velocity[1]=vy;
		_velocity[2]=vz;
		_velocity[3]=vroll;
		_velocity[4]=vpitch;
		_velocity[5]=vyaw;
	}

	const std::vector<cv::Point3f> & getWords3() const {return _words3;}
	const Transform & getPose() const {return _pose;}
	cv::Mat getPoseCovariance() const;
	const Transform & getGroundTruthPose() const {return _groundTruthPose;}
	const std::vector<float> & getVelocity() const {return _velocity;}

	SensorData & sensorData() {return _sensorData;}
	const SensorData & sensorData() const {return _sensorData;}

	unsigned long getMemoryUsed(bool withSensorData=true) const; // Return memory usage in Bytes

private:
	int _id;
	int _mapId;
	double _stamp;
	std::multimap<int, Link> _links; // id, transform
	std::map<int, Link> _landmarks;
	int _weight;
	std::string _label;
	bool _saved; // If it's saved to bd
	bool _modified;
	bool _linksModified; // Optimization when updating signatures in database

	// Contains all words (Some can be duplicates -> if a word appears 2
	// times in the signature, it will be 2 times in this list)
	// Words match with the CvSeq keypoints and descriptors
	std::multimap<int, int> _words; // word <id, keypoint index>
	std::vector<cv::KeyPoint> _wordsKpts;
	std::vector<cv::Point3f> _words3; // in base_link frame (localTransform applied))
	cv::Mat _wordsDescriptors;
	std::map<int, int> _wordsChanged; // <oldId, newId>
	bool _enabled;
	int _invalidWordsCount;

	Transform _pose;
	Transform _groundTruthPose;
	std::vector<float> _velocity;

	SensorData _sensorData;
};

} // namespace rtabmap
