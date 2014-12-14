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

#pragma once

#include "rtabmap/core/RtabmapExp.h" // DLL export/import defines

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

class Memory;

class RTABMAP_EXP Signature
{

public:
	Signature();
	Signature(int id,
			int mapId,
			const std::multimap<int, cv::KeyPoint> & words,
			const std::multimap<int, pcl::PointXYZ> & words3,
			const Transform & pose = Transform(),
			const cv::Mat & laserScan = cv::Mat(),
			const cv::Mat & image = cv::Mat(),
			const cv::Mat & depth = cv::Mat(),
			float fx = 0.0f,
			float fy = 0.0f,
			float cx = 0.0f,
			float cy = 0.0f,
			const Transform & localTransform =Transform::getIdentity());
	virtual ~Signature();

	/**
	 * Must return a value between >=0 and <=1 (1 means 100% similarity).
	 */
	float compareTo(const Signature & signature) const;
	bool isBadSignature() const;

	int id() const {return _id;}
	int mapId() const {return _mapId;}

	void setWeight(int weight) {if(_weight!=weight)_modified=true;_weight = weight;}
	int getWeight() const {return _weight;}

	void addLinks(const std::list<Link> & links);
	void addLinks(const std::map<int, Link> & links);
	void addLink(const Link & link);

	bool hasLink(int idTo) const;

	void changeLinkIds(int idFrom, int idTo);

	void removeLinks();
	void removeLink(int idTo);

	void setSaved(bool saved) {_saved = saved;}
	void setModified(bool modified) {_modified = modified; _linksModified = modified;}

	const std::map<int, Link> & getLinks() const {return _links;}
	bool isSaved() const {return _saved;}
	bool isModified() const {return _modified || _linksModified;}
	bool isLinksModified() const {return _linksModified;}

	//visual words stuff
	void removeAllWords();
	void removeWord(int wordId);
	void changeWordsRef(int oldWordId, int activeWordId);
	void setWords(const std::multimap<int, cv::KeyPoint> & words) {_enabled = false;_words = words;}
	bool isEnabled() const {return _enabled;}
	void setEnabled(bool enabled) {_enabled = enabled;}
	const std::multimap<int, cv::KeyPoint> & getWords() const {return _words;}
	const std::map<int, int> & getWordsChanged() const {return _wordsChanged;}
	void setImageCompressed(const cv::Mat & bytes) {_imageCompressed = bytes;}
	const cv::Mat & getImageCompressed() const {return _imageCompressed;}
	void setImageRaw(const cv::Mat & image) {_imageRaw = image;}
	const cv::Mat & getImageRaw() const {return _imageRaw;}

	//metric stuff
	void setWords3(const std::multimap<int, pcl::PointXYZ> & words3) {_words3 = words3;}
	void setDepthCompressed(const cv::Mat & bytes, float fx, float fy, float cx, float cy);
	void setLaserScanCompressed(const cv::Mat & bytes) {_laserScanCompressed = bytes;}
	void setLocalTransform(const Transform & t) {_localTransform = t;}
	void setPose(const Transform & pose) {_pose = pose;}
	const std::multimap<int, pcl::PointXYZ> & getWords3() const {return _words3;}
	const cv::Mat & getDepthCompressed() const {return _depthCompressed;}
	const cv::Mat & getLaserScanCompressed() const {return _laserScanCompressed;}
	float getDepthFx() const {return _fx;}
	float getDepthFy() const {return _fy;}
	float getDepthCx() const {return _cx;}
	float getDepthCy() const {return _cy;}
	const Transform & getPose() const {return _pose;}
	const Transform & getLocalTransform() const {return _localTransform;}
	void setDepthRaw(const cv::Mat & depth) {_depthRaw = depth;}
	const cv::Mat & getDepthRaw() const {return _depthRaw;}
	void setLaserScanRaw(const cv::Mat & depth2D) {_laserScanRaw = depth2D;}
	const cv::Mat & getLaserScanRaw() const {return _laserScanRaw;}

	SensorData toSensorData();
	void uncompressData();
	void uncompressData(cv::Mat * imageRaw, cv::Mat * depthRaw, cv::Mat * laserScanRaw);
	void uncompressDataConst(cv::Mat * imageRaw, cv::Mat * depthRaw, cv::Mat * laserScanRaw) const;

private:
	int _id;
	int _mapId;
	std::map<int, Link> _links; // id, transform
	int _weight;
	bool _saved; // If it's saved to bd
	bool _modified;
	bool _linksModified; // Optimization when updating signatures in database

	// Contains all words (Some can be duplicates -> if a word appears 2
	// times in the signature, it will be 2 times in this list)
	// Words match with the CvSeq keypoints and descriptors
	std::multimap<int, cv::KeyPoint> _words; // word <id, keypoint>
	std::map<int, int> _wordsChanged; // <oldId, newId>
	bool _enabled;
	cv::Mat _imageCompressed; // compressed image

	cv::Mat _depthCompressed; // compressed image
	cv::Mat _laserScanCompressed; // compressed data
	float _fx;
	float _fy;
	float _cx;
	float _cy;
	Transform _pose;
	Transform _localTransform; // camera_link -> base_link
	std::multimap<int, pcl::PointXYZ> _words3; // word <id, keypoint>

	cv::Mat _imageRaw; // CV_8UC1 or CV_8UC3
	cv::Mat _depthRaw; // depth CV_16UC1 or CV_32FC1, right image CV_8UC1
	cv::Mat _laserScanRaw; // CV_32FC2
};

} // namespace rtabmap
