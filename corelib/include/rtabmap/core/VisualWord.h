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

#include <opencv2/core/core.hpp>
#include <map>

namespace rtabmap
{

/**
 * @class VisualWord
 * @brief Represents a visual word (feature descriptor) used in the bag-of-words (BoW) model.
 *
 * A VisualWord holds a unique identifier, its descriptor (usually a feature vector from an image),
 * and a record of where (in which signatures/images) it was observed. This is a key component in 
 * loop closure detection and visual place recognition in RTAB-Map.
 *
 * The word keeps track of references (signature IDs) where it appears, and how many times it was observed 
 * in each signature. It also tracks whether the word has been saved to a database.
 */
class RTABMAP_CORE_EXPORT VisualWord
{
public:
	/**
     * @brief Constructor.
     * @param id          Unique ID of the visual word.
     * @param descriptor  The descriptor associated with the word (e.g., ORB, SIFT).
     * @param signatureId Optional signature ID where the word is first observed.
     */
	VisualWord(int id, const cv::Mat & descriptor, int signatureId = 0);

	/**
     * @brief Destructor.
     */
	~VisualWord();

	/**
     * @brief Adds a reference to this word for the given signature ID.
     * @param signatureId ID of the signature in which this word is observed.
     */
	void addRef(int signatureId);

	/**
     * @brief Removes all references to this word for the given signature ID.
     * @param signatureId ID of the signature from which references will be removed.
     * @return Number of references removed.
     */
	int removeAllRef(int signatureId);

	/**
     * @brief Estimates the memory used by this visual word in bytes.
     * @return Size in bytes.
     */
	unsigned long getMemoryUsed() const;

	/**
     * @brief Returns the total number of references (across all signatures).
     * @return Number of total references.
     */
	int getTotalReferences() const {return _totalReferences;}

	/**
     * @brief Returns the ID of this visual word.
     * @return Word ID.
     */
	int id() const {return _id;}

	/**
     * @brief Returns the feature descriptor of this word.
     * @return The OpenCV matrix descriptor.
     */
	const cv::Mat & getDescriptor() const {return _descriptor;}

	/**
     * @brief Returns the references of this word.
     * @return A map of (signature ID, occurrence count).
     */
	const std::map<int, int> & getReferences() const {return _references;}

	/**
     * @brief Checks if the word has been saved to the database.
     * @return True if saved, false otherwise.
     */
	bool isSaved() const {return _saved;}

	/**
     * @brief Sets the saved flag for the word.
     * @param saved True if the word has been saved to the database.
     */
	void setSaved(bool saved) {_saved = saved;}

private:
	int _id;                                  ///< Unique ID of the visual word.
	cv::Mat _descriptor;                      ///< Feature descriptor (e.g., ORB, SIFT).
	bool _saved;                              ///< Whether the word is saved to the database.

	int _totalReferences;                     ///< Total reference count across all signatures.
	std::map<int, int> _references;           ///< Active references: map of (signature ID, occurrence count).
};

} // namespace rtabmap
