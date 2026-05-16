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

#ifndef LINK_H_
#define LINK_H_

#include "rtabmap/core/rtabmap_core_export.h" // DLL export/import defines

#include <rtabmap/core/Transform.h>
#include <opencv2/core/core.hpp>

namespace rtabmap {

/**
 * @class Link
 * @brief Directed constraint between two nodes in RTAB-Map's pose graph.
 *
 * A link connects signature @ref from() to signature @ref to() with a relative
 * @ref Transform and an information matrix (inverse covariance) used by graph
 * optimization. Links are stored on @ref Signature objects and persisted in the
 * database; they define odometry chains, loop closures, landmarks, and priors.
 *
 * The transform is expressed from the @p from node frame to the @p to node frame
 * (i.e. pose of @p to relative to @p from), unless @ref type() indicates a
 * special semantics (e.g. @ref kPosePrior, @ref kLandmark).
 *
 * @see Signature
 * @see Memory::addLink()
 */
class RTABMAP_CORE_EXPORT Link
{
public:
	/**
	 * @brief Link category and filter sentinels.
	 *
	 * Values @ref kSelfRefLink, @ref kAllWithLandmarks, @ref kAllWithoutLandmarks and
	 * @ref kUndef are also used as query filters when retrieving links from memory or
	 * the database (they are not stored as link types on signatures).
	 */
	enum Type {
		kNeighbor,           /**< Sequential odometry link between consecutive nodes. */
		kGlobalClosure,      /**< Global loop closure added by global loop closure detection (i.e., using bags-of-words to find loop closures with other nodes in WM without using current estimated pose). */
		kLocalSpaceClosure,  /**< Local loop closure added by proximity detection by space (i.e. using current estimated pose to find loop closures with nearby nodes in WM). */
		kLocalTimeClosure,   /**< Local loop closure added by proximity detection by time (i.e. between nodes in STM). */
		kUserClosure,        /**< User-defined loop closure constraint. */
		kVirtualClosure,     /**< Virtual link added to keep the path linked to local map. */
		kNeighborMerged,     /**< Merged neighbor link after graph reduction. */
		kPosePrior,          /**< Absolute pose prior in the world frame (@p from == @p to). */
		kLandmark,           /**< Observation of a landmark: @p from is the observer node, @p to is a negative landmark id. */
		kGravity,            /**< Gravity direction constraint on the base frame (@p from == @p to). */
		kEnd,
		kSelfRefLink = 97,   /**< Filter: links where @p from == @p to (e.g. @ref kPosePrior, @ref kGravity). */
		kAllWithLandmarks = 98,    /**< Filter: all link types including @ref kLandmark. */
		kAllWithoutLandmarks = 99, /**< Filter: all link types except @ref kLandmark. */
		kUndef = 99};              /**< Undefined type or invalid link. */

	/** @return Human-readable name for @p type (e.g. "Neighbor", "GlobalClosure"). */
	static std::string typeName(Type type);

	/** @brief Default constructor; creates an invalid link (@ref kUndef). */
	Link();
	/**
	 * @brief Constructs a link between two nodes.
	 * @param from Source signature id.
	 * @param to Target signature id (negative for landmarks when @p type is @ref kLandmark).
	 * @param type Link category.
	 * @param transform Relative transform from @p from to @p to.
	 * @param infMatrix 6x6 information matrix (inverse covariance)
	 * @param userData Optional payload; compressed automatically if not already @c CV_8UC1.
	 */
	Link(int from,
			int to,
			Type type,
			const Transform & transform,
			const cv::Mat & infMatrix = cv::Mat::eye(6,6,CV_64FC1),
			const cv::Mat & userData = cv::Mat());

	/** @return True if ids, transform, and type are valid for use in the graph. */
	bool isValid() const {return from_ != 0 && to_ != 0 && !transform_.isNull() && type_!=kUndef;}

	int from() const {return from_;}
	int to() const {return to_;}
	const Transform & transform() const {return transform_;}
	Type type() const {return type_;}
	std::string typeName() const {return typeName(type_);}
	const cv::Mat & infMatrix() const {return infMatrix_;}

	/**
	 * @brief Rotation variance derived from the information matrix diagonal (roll, pitch, yaw).
	 * @param minimum If true, returns the largest diagonal entry (most uncertain axis);
	 *                if false, returns the smallest non-zero entry.
	 */
	double rotVariance(bool minimum = true) const;
	/**
	 * @brief Translation variance derived from the information matrix diagonal (x, y, z).
	 * @param minimum If true, returns the largest diagonal entry; if false, the smallest non-zero entry.
	 */
	double transVariance(bool minimum = true) const;

	void setFrom(int from) {from_ = from;}
	void setTo(int to) {to_ = to;}
	void setTransform(const Transform & transform) {transform_ = transform;}
	void setType(Type type) {type_ = type;}
	/** @brief Sets the 6x6 information matrix (@c CV_64FC1); diagonal entries must be positive and finite. */
    void setInfMatrix(const cv::Mat & infMatrix);

	const cv::Mat & userDataRaw() const {return _userDataRaw;}
	const cv::Mat & userDataCompressed() const {return _userDataCompressed;}
	/** @brief Decompresses user data into @ref userDataRaw() if compressed data is stored. */
	void uncompressUserData();
	/** @return Uncompressed user data without modifying internal storage. */
	cv::Mat uncompressUserDataConst() const;

	/**
	 * @brief Chains this link (from → to) with @p link (to → link.to).
	 * @param link Second link; must satisfy @c this->to() == link.from().
	 * @param outputType Type of the merged link.
	 * @return Single link from @ref from() to @p link.to() with transform
	 *         \(T_{ac} = T_{ab} T_{bc}\) (or null if either input transform is null).
	 *         Information matrix handling depends on @p outputType:
	 *         - @ref kNeighborMerged: \(\Omega_{ac} = (\Omega_{ab}^{-1} + \Omega_{bc}^{-1})^{-1}\)
	 *           (covariances add when both legs are diagonal and independent).
	 *         - Other types: keeps the full information matrix of @p link unless
	 *           \(\Omega_{ab}(0,0) < \Omega_{bc}(0,0)\) (i.e. the smaller x information entry).
	 */
	Link merge(const Link & link, Type outputType) const;
	/** @return Link with swapped endpoints and inverted transform. */
	Link inverse() const;

private:
	int from_;
	int to_;
	Transform transform_;
	Type type_;
	cv::Mat infMatrix_; // Information matrix = covariance matrix ^ -1

	// user data
	cv::Mat _userDataCompressed;      // compressed data
	cv::Mat _userDataRaw;
};

}


#endif /* LINK_H_ */
