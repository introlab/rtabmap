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

/**
 * @class Signature
 * @brief Represents a node in RTAB-Map's pose graph
 * 
 * The Signature class is a fundamental building block of RTAB-Map's graph-based
 * mapping system. Each signature represents a unique location or pose in the map
 * and contains:
 * 
 * - **Visual words**: Bag-of-words representation using visual features (keypoints,
 *   descriptors, 3D points) for place recognition
 * - **Links**: Connections to other signatures (neighbors, loop closures, landmarks)
 * - **Pose information**: Current pose, ground truth pose, velocity
 * - **Sensor data**: Images, depth, laser scans, etc. captured at this location
 * - **Metadata**: ID, map ID, timestamp, label, weight, modification flags
 * 
 * Signatures are used for:
 * - **Place recognition**: Comparing signatures to detect loop closures
 * - **Graph construction**: Building the pose graph through links
 * - **Localization**: Matching current observations to stored signatures
 * - **Mapping**: Storing sensor data, visual features and local occupancy grid for each location
 * 
 * The class tracks modification state to optimize database updates, and provides
 * methods to manage visual words, links, landmarks, and pose information.
 * 
 * @note Signatures are typically managed by the Memory class, which handles
 *       storage, retrieval, and graph operations.
 * 
 * @see Link
 * @see SensorData
 * @see Memory
 */
class RTABMAP_CORE_EXPORT Signature
{

public:
	/**
	 * @brief Default constructor
	 * 
	 * Creates an empty signature with invalid ID (0), default map ID (-1),
	 * and all fields initialized to default values.
	 */
	Signature();
	
	/**
	 * @brief Constructor with explicit parameters
	 * 
	 * Creates a signature with the specified parameters. The sensor data ID
	 * will be automatically set to match the signature ID if not already set.
	 * 
	 * @param id Unique signature ID (must be > 0 for valid signature)
	 * @param mapId Map ID this signature belongs to (default: -1 for no map)
	 * @param weight Weight/importance of this signature (default: 0)
	 * @param stamp Timestamp in seconds (default: 0.0)
	 * @param label Optional label/name for this signature (default: empty)
	 * @param pose Current pose of the signature (default: identity null transform)
	 * @param groundTruthPose Ground truth pose for evaluation (default: null transform)
	 * @param sensorData Sensor data captured at this location (default: empty)
	 * 
	 * @note The signature is marked as modified and not saved by default.
	 */
	Signature(int id,
			int mapId = -1,
			int weight = 0,
			double stamp = 0.0,
			const std::string & label = std::string(),
			const Transform & pose = Transform(),
			const Transform & groundTruthPose = Transform(),
			const SensorData & sensorData = SensorData());
	
	/**
	 * @brief Constructor from sensor data
	 * 
	 * Creates a signature from sensor data. The signature ID is taken from
	 * the sensor data ID, and the ground truth pose is extracted from the
	 * sensor data if available.
	 * 
	 * @param data Sensor data to create signature from
	 * 
	 * @note The signature pose is initialized to identity transform.
	 */
	Signature(const SensorData & data);
	
	/**
	 * @brief Virtual destructor
	 */
	virtual ~Signature();

	/**
	 * @brief Compares this signature to another signature
	 * 
	 * Computes a similarity score between this signature and another signature
	 * based on their visual words. The comparison uses bag-of-words matching
	 * to determine how similar the two locations are.
	 * 
	 * @param signature The signature to compare against
	 * @return Similarity score between 0.0 and 1.0, where 1.0 means 100% similarity
	 * 
	 * @note This method is used for loop closure detection and place recognition.
	 */
	float compareTo(const Signature & signature) const;
	
	/**
	 * @brief Checks if this signature is considered "bad"
	 * 
	 * A signature is considered bad if it has no valid visual words (all words
	 * are invalid or the signature has no words at all). Bad signatures cannot
	 * be used for place recognition.
	 * 
	 * @return True if the signature has no valid visual words, false otherwise
	 */
	bool isBadSignature() const;

	/**
	 * @brief Returns the signature ID
	 * @return The unique signature ID (0 if invalid/uninitialized)
	 */
	int id() const {return _id;}
	
	/**
	 * @brief Returns the map ID
	 * @return The map ID this signature belongs to (-1 if no map assigned)
	 */
	int mapId() const {return _mapId;}

	/**
	 * @brief Sets the weight/importance of this signature
	 * 
	 * The weight represents the importance or priority of this signature,
	 * typically set by Rehearsal mechanism (see Memory::rehearsal).
	 * Higher weights may be used to prioritize certain signatures during
	 * memory management or loop closure detection.
	 * 
	 * @param weight The weight value
	 * 
	 * @note Automatically marks the signature as modified if the weight changes.
	 */
	void setWeight(int weight) {_modified=_weight!=weight;_weight = weight;}
	
	/**
	 * @brief Returns the weight/importance of this signature
	 * @return The weight value
	 */
	int getWeight() const {return _weight;}

	/**
	 * @brief Sets a label/name for this signature
	 * 
	 * Labels are optional text identifiers that can be used to tag or
	 * categorize signatures (e.g., "room1", "corridor", "entrance").
	 * 
	 * @param label The label string
	 * 
	 * @note Automatically marks the signature as modified if the label changes.
	 */
	void setLabel(const std::string & label) {_modified=_label.compare(label)!=0;_label = label;}
	
	/**
	 * @brief Returns the label/name of this signature
	 * @return The label string (empty if not set)
	 */
	const std::string & getLabel() const {return _label;}

	/**
	 * @brief Returns the timestamp of this signature
	 * @return Timestamp in seconds (typically seconds since epoch)
	 */
	double getStamp() const {return _stamp;}

	/**
	 * @brief Adds multiple links from a list
	 * 
	 * Adds all links from the provided list to this signature's link collection.
	 * 
	 * @param links List of links to add
	 * 
	 * @note Each link must have `from()` equal to this signature's ID.
	 * @note Automatically marks links as modified.
	 * @see addLink()
	 */
	void addLinks(const std::list<Link> & links);
	
	/**
	 * @brief Adds multiple links from a map
	 * 
	 * Adds all links from the provided map to this signature's link collection.
	 * 
	 * @param links Map of target IDs to links
	 * 
	 * @note Each link must have `from()` equal to this signature's ID.
	 * @note The map's keys are the target IDs (`to()` values of the links).
	 * @note Automatically marks links as modified.
	 * @see addLink()
	 */
	void addLinks(const std::map<int, Link> & links);
	
	/**
	 * @brief Adds a single link to this signature
	 * 
	 * Adds a link connecting this signature to another signature. The link
	 * represents a constraint in the pose graph (e.g., neighbor, loop closure).
	 * 
	 * @param link The link to add
	 * 
	 * @note The link's `from()` must equal this signature's ID.
	 * @note The link's `to()` must be different from this signature's ID
	 *       (except for self-referring links like pose priors or gravity).
	 * @note Duplicate links to the same target are not allowed (assertion failure).
	 * @note Automatically marks links as modified.
	 */
	void addLink(const Link & link);

	/**
	 * @brief Checks if this signature has a link to a target signature
	 * 
	 * @param idTo Target signature ID (0 to check for any link of the specified type)
	 * @param type Link type to check for (default: Link::kUndef to check any type)
	 * @return True if a matching link exists, false otherwise
	 */
	bool hasLink(int idTo, Link::Type type = Link::kUndef) const;

	/**
	 * @brief Changes all link target IDs from one value to another
	 * 
	 * Updates all links that point to `idFrom` to point to `idTo` instead.
	 * This is useful when merging or reorganizing the graph.
	 * 
	 * @param idFrom Original target ID
	 * @param idTo New target ID
	 * 
	 * @note Automatically marks links as modified.
	 */
	void changeLinkIds(int idFrom, int idTo);

	/**
	 * @brief Removes all links from this signature
	 * 
	 * @param keepSelfReferringLinks If true, preserves self-referring links
	 *        (pose priors, gravity) that have from() == to()
	 * 
	 * @note Automatically marks links as modified.
	 */
	void removeLinks(bool keepSelfReferringLinks = false);
	
	/**
	 * @brief Removes a specific link to a target signature
	 * 
	 * @param idTo Target signature ID of the link to remove
	 * 
	 * @note Automatically marks links as modified.
	 */
	void removeLink(int idTo);
	
	/**
	 * @brief Removes all virtual links from this signature
	 * 
	 * Virtual links are links of type Link::kVirtualClosure.
	 * 
	 * @note Automatically marks links as modified.
	 */
	void removeVirtualLinks();

	/**
	 * @brief Adds a landmark observation to this signature
	 * 
	 * Landmarks are persistent features in the environment that can be observed
	 * from multiple locations. This method adds a link representing the observation
	 * of a landmark from this signature's location.
	 * 
	 * @param landmark Link representing the landmark observation
	 * 
	 * @note Landmark IDs are typically negative to distinguish them from regular nodes.
	 */
	void addLandmark(const Link & landmark);
	
	/**
	 * @brief Returns all landmark observations
	 * @return Const reference to the map of landmark IDs to landmark links
	 */
	const std::map<int, Link> & getLandmarks() const {return _landmarks;}
	
	/**
	 * @brief Removes all landmark observations
	 */
	void removeLandmarks();
	
	/**
	 * @brief Removes a specific landmark observation
	 * @param landmarkId The landmark ID to remove
	 */
	void removeLandmark(int landmarkId);

	/**
	 * @brief Sets whether this signature has been saved to the database
	 * @param saved True if saved, false otherwise
	 */
	void setSaved(bool saved) {_saved = saved;}
	
	/**
	 * @brief Sets the modification flag for this signature
	 * 
	 * Marks the signature as modified (or unmodified). When set to true,
	 * also marks links as modified.
	 * 
	 * @param modified True if modified, false otherwise
	 */
	void setModified(bool modified) {_modified = modified; _linksModified = modified;}

	/**
	 * @brief Returns all links from this signature
	 * @return Const reference to the multimap of target IDs to links
	 */
	const std::multimap<int, Link> & getLinks() const {return _links;}
	
	/**
	 * @brief Checks if this signature has been saved to the database
	 * @return True if saved, false otherwise
	 */
	bool isSaved() const {return _saved;}
	
	/**
	 * @brief Checks if this signature has been modified
	 * 
	 * Returns true if either the signature data or links have been modified.
	 * 
	 * @return True if modified, false otherwise
	 */
	bool isModified() const {return _modified || _linksModified;}
	
	/**
	 * @brief Checks if the links have been modified
	 * @return True if links are modified, false otherwise
	 */
	bool isLinksModified() const {return _linksModified;}

	// Visual words management
	
	/**
	 * @brief Removes all visual words from this signature
	 * 
	 * Clears all visual words, keypoints, 3D points, and descriptors.
	 * The signature will be disabled after this operation.
	 */
	void removeAllWords();
	
	/**
	 * @brief Changes visual word references from one word ID to another
	 * 
	 * Updates all occurrences of `oldWordId` to `activeWordId` in the visual words.
	 * This is used when words are merged or reorganized in the dictionary.
	 * 
	 * @param oldWordId The original word ID to replace
	 * @param activeWordId The new word ID to use
	 * 
	 * @note Tracks word ID changes in `_wordsChanged` for reference.
	 */
	void changeWordsRef(int oldWordId, int activeWordId);
	
	/**
	 * @brief Sets all visual words for this signature
	 * 
	 * Sets the complete visual word representation including word IDs, keypoints,
	 * 3D points, and descriptors. All arrays must have matching sizes.
	 * 
	 * @param words Multimap of word IDs to keypoint indices (allows duplicate words). The keypoint indices match the keypoints, points and descriptors.
	 * @param keypoints Vector of 2D keypoints in image coordinates. The keypoints must be in the same order as the words.
	 * @param words3 Vector of 3D points in base_link frame (with localTransform applied). The points must be in the same order as the words.
	 * @param descriptors Feature descriptors matrix (one row per word). The descriptors must be in the same order as the words.
	 * 
	 * @note The signature is disabled after setting words (must be explicitly enabled).
	 * @note Invalid words (ID <= 0) are counted in `_invalidWordsCount`.
	 * @note All arrays must have the same size (number of words).
	 */
	void setWords(const std::multimap<int, int> & words, const std::vector<cv::KeyPoint> & keypoints, const std::vector<cv::Point3f> & words3, const cv::Mat & descriptors);
	
	/**
	 * @brief Checks if this signature is enabled
	 * 
	 * Enabled signatures can be used for place recognition and loop closure detection. 
	 * More explicitly, it means that the signature is registered to the VWDictionary (see Memory::enableWordsRef).
	 * 
	 * @return True if enabled, false otherwise
	 */
	bool isEnabled() const {return _enabled;}
	
	/**
	 * @brief Sets whether this signature is enabled
	 * @param enabled True to enable, false to disable
	 */
	void setEnabled(bool enabled) {_enabled = enabled;}
	
	/**
	 * @brief Returns the visual words map
	 * 
	 * Returns a multimap of word IDs to keypoint indices. The multimap allows
	 * duplicate word IDs (a word can appear multiple times in the signature).
	 * 
	 * @return Const reference to the words multimap
	 */
	const std::multimap<int, int> & getWords() const {return _words;}
	
	/**
	 * @brief Returns the keypoints associated with visual words
	 * @return Const reference to the vector of 2D keypoints
	 */
	const std::vector<cv::KeyPoint> & getWordsKpts() const {return _wordsKpts;}
	
	/**
	 * @brief Returns the count of invalid visual words
	 * 
	 * Invalid words are those with ID <= 0. These words cannot be used
	 * for place recognition. However, they are still used for transform
	 * estimation after loop closures are detected with the valid words.
	 * 
	 * @return Number of invalid words
	 */
	int getInvalidWordsCount() const {return _invalidWordsCount;}
	
	/**
	 * @brief Returns the word ID change mapping
	 * 
	 * Returns a map of old word IDs to new word IDs, tracking changes
	 * made through `changeWordsRef()`.
	 * 
	 * @return Const reference to the word ID change map
	 */
	const std::map<int, int> & getWordsChanged() const {return _wordsChanged;}
	
	/**
	 * @brief Returns the feature descriptors for visual words
	 * @return Const reference to the descriptors matrix (one row per word)
	 */
	const cv::Mat & getWordsDescriptors() const {return _wordsDescriptors;}
	
	/**
	 * @brief Sets the feature descriptors for visual words
	 * 
	 * Updates the descriptors matrix. The number of rows must match
	 * the number of visual words.
	 * 
	 * @param descriptors Descriptors matrix (one row per word)
	 */
	void setWordsDescriptors(const cv::Mat & descriptors);

	// Pose and metric information
	
	/**
	 * @brief Sets the pose of this signature
	 * 
	 * The pose represents the position and orientation of this signature
	 * in the odometry coordinate frame. To get the pose in map coordinate frame,
	 * use the map correction transform computed by graph optimization and apply it to this pose.
	 * 
	 * @param pose The transform representing the pose in the odometry coordinate frame.
	 * 
	 * @code
	 * // Get pose in map coordinate frame
	 * Statistics stats;
	 * Transform poseInMapFrame = stats.mapCorrection() * signature.getPose();
	 * // poseInMapFrame is the pose in map coordinate frame
	 * // signature.getPose() returns the pose in odometry coordinate frame
	 * @endcode
	 * 
	 * @see Statistics::mapCorrection()
	 * @see getPose()
	 */
	void setPose(const Transform & pose) {_pose = pose;}
	
	/**
	 * @brief Sets the ground truth pose for evaluation
	 * 
	 * Ground truth poses are used for evaluating localization accuracy
	 * and comparing against the estimated pose.
	 * 
	 * @param pose The ground truth transform
	 */
	void setGroundTruthPose(const Transform & pose) {_groundTruthPose = pose;}
	
	/**
	 * @brief Sets the velocity of this signature
	 * 
	 * Sets the 6DOF velocity (linear and angular) at the time this signature
	 * was captured.
	 * 
	 * @param vx Linear velocity in x direction (m/s)
	 * @param vy Linear velocity in y direction (m/s)
	 * @param vz Linear velocity in z direction (m/s)
	 * @param vroll Angular velocity around x axis (rad/s)
	 * @param vpitch Angular velocity around y axis (rad/s)
	 * @param vyaw Angular velocity around z axis (rad/s)
	 */
	void setVelocity(float vx, float vy, float vz, float vroll, float vpitch, float vyaw) {
		_velocity = std::vector<float>(6,0);
		_velocity[0]=vx;
		_velocity[1]=vy;
		_velocity[2]=vz;
		_velocity[3]=vroll;
		_velocity[4]=vpitch;
		_velocity[5]=vyaw;
	}

	/**
	 * @brief Returns the 3D points of visual words
	 * 
	 * Returns the 3D coordinates of visual words in the base_link frame
	 * (with localTransform applied).
	 * 
	 * @return Const reference to the vector of 3D points
	 */
	const std::vector<cv::Point3f> & getWords3() const {return _words3;}
	
	/**
	 * @brief Returns the pose of this signature
	 * @return Const reference to the pose transform in the odometry coordinate frame.
	 * @see setPose() for an example of how to get the pose in map coordinate frame.
	 */
	const Transform & getPose() const {return _pose;}
	
	/**
	 * @brief Gets the pose covariance matrix
	 * 
	 * Gets the covariance matrix representing the uncertainty
	 * between the current pose and the previous pose (e.g, from the odometry link with the previous signature).
	 * 
	 * @return 6x6 covariance matrix (3x3 for translation, 3x3 for rotation)
	 */
	cv::Mat getPoseCovariance() const;
	
	/**
	 * @brief Returns the ground truth pose
	 * @return Const reference to the ground truth transform
	 */
	const Transform & getGroundTruthPose() const {return _groundTruthPose;}
	
	/**
	 * @brief Returns the velocity of this signature
	 * @return Const reference to the velocity vector [vx, vy, vz, vroll, vpitch, vyaw]
	 */
	const std::vector<float> & getVelocity() const {return _velocity;}

	/**
	 * @brief Returns mutable access to the sensor data
	 * @return Reference to the sensor data
	 */
	SensorData & sensorData() {return _sensorData;}
	
	/**
	 * @brief Returns const access to the sensor data
	 * @return Const reference to the sensor data
	 */
	const SensorData & sensorData() const {return _sensorData;}

	/**
	 * @brief Computes the memory usage of this signature
	 * 
	 * Calculates the approximate memory footprint of this signature, including
	 * visual words, links, sensor data, and other internal structures.
	 * 
	 * @param withSensorData If true, includes sensor data in the calculation
	 * @return Memory usage in bytes
	 */
	unsigned long getMemoryUsed(bool withSensorData=true) const;

private:
	int _id; ///< Unique signature ID (0 if invalid)
	int _mapId; ///< Map ID this signature belongs to (-1 if no map)
	double _stamp; ///< Timestamp in seconds
	std::multimap<int, Link> _links; ///< Links to other signatures (target ID -> Link, allows multiple links per target)
	std::map<int, Link> _landmarks; ///< Landmark observations (landmark ID -> Link)
	int _weight; ///< Weight/importance of this signature
	std::string _label; ///< Optional label/name for this signature
	bool _saved; ///< Flag indicating if signature is saved to database
	bool _modified; ///< Flag indicating if signature data has been modified
	bool _linksModified; ///< Flag indicating if links have been modified (optimization for database updates)

	/**
	 * @brief Visual words representation
	 * 
	 * Contains all visual words for this signature. Words can be duplicates
	 * (a word can appear multiple times in the signature). Words match with
	 * the keypoints and descriptors arrays.
	 */
	std::multimap<int, int> _words; ///< Visual words: word ID -> keypoint index (multimap allows duplicates)
	std::vector<cv::KeyPoint> _wordsKpts; ///< 2D keypoints in image coordinates
	std::vector<cv::Point3f> _words3; ///< 3D points in base_link frame (with localTransform applied)
	cv::Mat _wordsDescriptors; ///< Feature descriptors matrix (one row per word)
	std::map<int, int> _wordsChanged; ///< Word ID change tracking: old ID -> new ID
	bool _enabled; ///< Flag indicating if signature is enabled for place recognition
	int _invalidWordsCount; ///< Count of invalid words (ID <= 0)

	Transform _pose; ///< Current pose in map coordinate frame
	Transform _groundTruthPose; ///< Ground truth pose for evaluation
	std::vector<float> _velocity; ///< 6DOF velocity [vx, vy, vz, vroll, vpitch, vyaw]

	SensorData _sensorData; ///< Sensor data captured at this location (images, depth, scans, etc.)
};

} // namespace rtabmap
