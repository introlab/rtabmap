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

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <list>
#include <set>
#include "rtabmap/core/Parameters.h"

namespace rtabmap
{

class DBDriver;
class VisualWord;
class FlannIndex;

/**
 * @class VWDictionary
 * @brief Manages a dictionary of visual words for visual place recognition and loop closure detection.
 *
 * The VWDictionary class maintains a collection of visual words (feature descriptors) and provides
 * efficient nearest neighbor search capabilities. It supports both incremental and fixed dictionary modes:
 * - Incremental mode: New visual words are added dynamically as new images are processed
 * - Fixed mode: A pre-computed dictionary is loaded from a file
 *
 * The class uses various nearest neighbor search strategies (FLANN, brute force, GPU-accelerated)
 * to match descriptors efficiently. It tracks word references to signatures (images) and manages
 * unused words for memory optimization.
 *
 * @note Visual words are identified by unique integer IDs starting from VWDictionary::ID_START
 */
class RTABMAP_CORE_EXPORT VWDictionary
{
public:
	/**
	 * @enum NNStrategy
	 * @brief Nearest neighbor search strategies for descriptor matching
	 */
	enum NNStrategy{
		kNNFlannNaive,      ///< FLANN naive search (exhaustive)
		kNNFlannKdTree,     ///< FLANN kd-tree index (fast for high-dimensional descriptors)
		kNNFlannLSH,        ///< FLANN Locality-Sensitive Hashing (ideal for binary descriptors)
		kNNBruteForce,      ///< Brute force CPU search
		kNNBruteForceGPU,   ///< Brute force GPU-accelerated search (requires CUDA)
		kNNUndef            ///< Undefined strategy
	};
	
	/**
	 * @brief Starting ID for visual words (typically 1)
	 */
	static const int ID_START;
	
	/**
	 * @brief Invalid visual word ID (typically 0)
	 */
	static const int ID_INVALID;
	
	/**
	 * @brief Get the name of a nearest neighbor strategy
	 * @param strategy The strategy enum value
	 * @return String representation of the strategy name
	 */
	static std::string nnStrategyName(NNStrategy strategy)
	{
		switch(strategy) {
		case kNNFlannNaive:
			return "FLANN NAIVE";
		case kNNFlannKdTree:
			return "FLANN KD-TREE";
		case kNNFlannLSH:
			return "FLANN LSH";
		case kNNBruteForce:
			return "BRUTE FORCE";
		case kNNBruteForceGPU:
			return "BRUTE FORCE GPU";
		default:
			return "Unknown";
		}
	}

public:
	/**
	 * @brief Constructor
	 * @param parameters Optional parameters map to configure the dictionary
	 */
	VWDictionary(const ParametersMap & parameters = ParametersMap());
	
	/**
	 * @brief Destructor
	 * 
	 * Clears all visual words and releases resources.
	 */
	virtual ~VWDictionary();

	/**
	 * @brief Parse and apply parameters from a parameters map
	 * @param parameters Parameters map containing configuration values
	 */
	virtual void parseParameters(const ParametersMap & parameters);

	/**
	 * @brief Update the search index with newly added words
	 * 
	 * This method rebuilds the nearest neighbor search index (FLANN, etc.)
	 * to include any words that were added but not yet indexed.
	 */
	virtual void update();

	/**
	 * @brief Add new visual words from descriptors
	 * @param descriptors Matrix of descriptors (one row per descriptor)
	 * @param signatureId ID of the signature (image) these descriptors belong to
	 * @return List of visual word IDs that were added or matched
	 * 
	 * For each descriptor, either matches it to an existing visual word
	 * (using nearest neighbor search with NNDR ratio) or creates a new visual word.
	 * 
	 * @note If the dictionary is not incremental (fixed), NNDR is not applied
	 * and the closest existing visual word ID is always returned. If the dictionary is
	 * incremental, a new visual word is created if NNDR validation
	 * passes; otherwise, a reference to an existing visual word is added.
	 */
	virtual std::list<int> addNewWords(
			const cv::Mat & descriptors,
			int signatureId);
	
	/**
	 * @brief Add an existing visual word to the dictionary
	 * @param vw Pointer to the visual word to add (ownership is transferred)
	 * 
	 * @note The dictionary takes ownership of the VisualWord object
	 */
	virtual void addWord(VisualWord * vw);

	/**
	 * @brief Find nearest neighbor visual word IDs for a list of visual words
	 * @param vws List of visual words to match
	 * @return Vector of visual word IDs (one per input visual word)
	 * 
	 * @note If the dictionary is not incremental (fixed), NNDR is not applied
	 * and the closest visual word ID is always returned. If the dictionary is
	 * incremental, a valid visual word ID is returned only if NNDR validation passes.
	 */
	std::vector<int> findNN(const std::list<VisualWord *> & vws) const;
	
	/**
	 * @brief Find nearest neighbor visual word IDs for descriptors
	 * @param descriptors Matrix of descriptors (one row per descriptor)
	 * @return Vector of visual word IDs (one per descriptor)
	 * 
	 * @note If the dictionary is not incremental (fixed), NNDR is not applied
	 * and the closest visual word ID is always returned. If the dictionary is
	 * incremental, a valid visual word ID is returned only if NNDR validation passes.
	 */
	std::vector<int> findNN(const cv::Mat & descriptors) const;

	/**
	 * @brief Add a reference from a visual word to a signature
	 * @param wordId ID of the visual word
	 * @param signatureId ID of the signature (image)
	 * 
	 * Tracks which signatures use which visual words. If the word was unused,
	 * it is removed from the unused words list.
	 */
	void addWordRef(int wordId, int signatureId);
	
	/**
	 * @brief Remove all references from a visual word to a signature
	 * @param wordId ID of the visual word
	 * @param signatureId ID of the signature (image)
	 * 
	 * If the word has no more references after this operation,
	 * it is added to the unused words list.
	 */
	void removeAllWordRef(int wordId, int signatureId);
	
	/**
	 * @brief Get a visual word by ID
	 * @param id Visual word ID
	 * @return Pointer to the visual word, or nullptr if not found
	 */
	const VisualWord * getWord(int id) const;
	
	/**
	 * @brief Get an unused visual word by ID
	 * @param id Visual word ID
	 * @return Pointer to the unused visual word, or nullptr if not found or not unused
	 * 
	 * @note Ownership of the returned visual word still belongs to the dictionary.
	 * To transfer ownership to the caller, removeWords() must be called on this word.
	 */
	VisualWord * getUnusedWord(int id) const;
	
	/**
	 * @brief Set the last word ID (used when loading from database)
	 * @param id Last word ID
	 */
	void setLastWordId(int id) {_lastWordId = id;}
	
	/**
	 * @brief Get all visual words
	 * @return Map of visual word ID to VisualWord pointer
	 */
	const std::map<int, VisualWord *> & getVisualWords() const {return _visualWords;}
	
	/**
	 * @brief Get the Nearest Neighbor Distance Ratio (NNDR) threshold
	 * @return NNDR ratio value
	 * 
	 * The NNDR ratio is used to determine if a descriptor matches an existing
	 * visual word. If the ratio of distances to the first and second nearest
	 * neighbors is below this threshold, a match is accepted.
	 * 
	 * @note The NNDR method was introduced in "Distinctive Image Features
	 * from Scale-Invariant Keypoints" by David Lowe (IJCV 2004).
	 */
	float getNndrRatio() const {return _nndrRatio;}
	
	/**
	 * @brief Get the count of words not yet indexed in the search tree
	 * @return Number of words waiting to be indexed
	 */
	unsigned int getNotIndexedWordsCount() const {return (int)_notIndexedWords.size();}
	
	/**
	 * @brief Get the ID of the last indexed word
	 * @return Last indexed word ID, or 0 if no words are indexed
	 */
	int getLastIndexedWordId() const;
	
	/**
	 * @brief Get the total number of active word-to-signature references
	 * @return Total count of active references
	 */
	int getTotalActiveReferences() const {return _totalActiveReferences;}
	
	/**
	 * @brief Get the count of words currently indexed in the search tree
	 * @return Number of indexed words
	 */
	unsigned int getIndexedWordsCount() const;
	
	/**
	 * @brief Get the memory used by the search index
	 * @return Memory usage in kilobytes
	 */
	unsigned int getIndexMemoryUsed() const; // KB
	
	/**
	 * @brief Get the total memory used by the dictionary
	 * @return Memory usage in bytes
	 */
	unsigned long getMemoryUsed() const; //Bytes
	
	/**
	 * @brief Set the nearest neighbor search strategy
	 * @param strategy The strategy to use
	 * @return true if the search tree was re-initialized (strategy changed), false otherwise
	 * 
	 * Changing the strategy will rebuild the search index if words are already indexed.
	 */
	bool setNNStrategy(NNStrategy strategy);
	
	/**
	 * @brief Get the current nearest neighbor search strategy
	 * @return The current NNStrategy
	 */
	NNStrategy getNNStrategy() const {return _strategy;}
	
	/**
	 * @brief Check if the dictionary is in incremental mode
	 * @return true if incremental, false if fixed
	 */
	bool isIncremental() const {return _incrementalDictionary;}
	
	/**
	 * @brief Check if FLANN index is updated incrementally
	 * @return true if incremental FLANN updates are enabled
	 */
	bool isIncrementalFlann() const {return _incrementalFlann;}
	
	/**
	 * @brief Set the dictionary to incremental mode
	 * 
	 * In incremental mode, new visual words can be added dynamically.
	 * This cannot be called if a fixed dictionary is already loaded.
	 */
	void setIncrementalDictionary();
	
	/**
	 * @brief Set the dictionary to fixed mode and load from file
	 * @param dictionaryPath Path to the dictionary file (.txt or .db format)
	 * 
	 * Loads a pre-computed dictionary from a file. The dictionary file format in txt format
	 * should be: one line per visual word, with word ID followed by descriptor values.
	 * This cannot be called if words are already in the dictionary.
	 */
	void setFixedDictionary(const std::string & dictionaryPath);
	
	/**
	 * @brief Check if the dictionary has been modified since last save
	 * @return true if modified, false otherwise
	 */
	bool isModified() const;

	/**
	 * @brief Serialize the search index to a byte vector
	 * @return Serialized index data
	 */
	std::vector<unsigned char> serializeIndex() const;
	
	/**
	 * @brief Deserialize the search index from a byte vector
	 * @param data Serialized index data
	 * @return true if deserialization was successful, false otherwise
	 */
	bool deserializeIndex(const std::vector<unsigned char> & data);
	
	/**
	 * @brief Deserialize the search index from raw bytes
	 * @param data Pointer to serialized index data
	 * @param size Size of the data in bytes
	 * @return true if deserialization was successful, false otherwise
	 */
	bool deserializeIndex(const unsigned char * data, size_t size);
	
	/**
	 * @brief Export the dictionary to files
	 * @param fileNameReferences Path to file for word-to-signature references
	 * @param fileNameDescriptors Path to file for visual word descriptors
	 * 
	 * Exports the dictionary in a format that can be loaded later.
	 */
	void exportDictionary(const char * fileNameReferences, const char * fileNameDescriptors) const;

	/**
	 * @brief Clear all visual words and reset the dictionary
	 * @param printWarningsIfNotEmpty If true, print warnings if dictionary is not empty
	 * 
	 * Deletes all visual words and releases all resources.
	 */
	void clear(bool printWarningsIfNotEmpty = true);
	
	/**
	 * @brief Get all unused visual words
	 * @return Vector of pointers to unused visual words
	 * 
	 * Unused words are visual words that have no references to any signatures.
	 * 
	 * @note Ownership of the returned visual words still belongs to the dictionary.
	 * To transfer ownership to the caller, removeWords() must be called on these words.
	 */
	std::vector<VisualWord *> getUnusedWords() const;
	
	/**
	 * @brief Get IDs of all unused visual words
	 * @return Vector of unused word IDs
	 */
	std::vector<int> getUnusedWordIds() const;
	
	/**
	 * @brief Get the count of unused visual words
	 * @return Number of unused words
	 */
	unsigned int getUnusedWordsSize() const {return (int)_unusedWords.size();}
	
	/**
	 * @brief Remove words from the dictionary
	 * @param words Vector of visual word pointers to remove
	 * 
	 * @note The caller is responsible for deleting the VisualWord objects
	 */
	void removeWords(const std::vector<VisualWord*> & words); // caller must delete the words
	
	/**
	 * @brief Delete all unused visual words
	 * 
	 * Removes and deletes visual words that have no references to any signatures.
	 */
	void deleteUnusedWords();

public:
	/**
	 * @brief Convert binary descriptors to 32-bit float format
	 * @param descriptorsIn Input descriptors (CV_8UC1 for binary, or CV_32FC1 for float)
	 * @param byteToFloat Conversion mode:
	 *   - If true: Simple type conversion from CV_8UC1 to CV_32FC1 using OpenCV's convertTo().
	 *     Each byte value becomes a float value (output dimensions unchanged).
	 *   - If false: Bit-by-bit expansion for binary descriptors (e.g., ORB, BRIEF).
	 *     Each input byte (8 bits) is expanded into 8 float values (0.0f or 1.0f),
	 *     one per bit. Output has 8x the number of columns (e.g., 32 bytes -> 256 floats).
	 * @return Descriptors in 32-bit float format (CV_32FC1)
	 */
	static cv::Mat convertBinTo32F(const cv::Mat & descriptorsIn, bool byteToFloat = true);
	
	/**
	 * @brief Convert 32-bit float descriptors to binary format
	 * @param descriptorsIn Input descriptors (CV_32FC1)
	 * @param byteToFloat Conversion mode:
	 *   - If true: Simple type conversion from CV_32FC1 to CV_8UC1 using OpenCV's convertTo().
	 *     Each float value becomes a byte value (output dimensions unchanged).
	 *   - If false: Bit-by-bit packing for binary descriptors.
	 *     Each group of 8 float values (0.0f or 1.0f) is packed into 1 byte (8 bits),
	 *     one bit per float. Input must have columns divisible by 8.
	 *     Output has 1/8 the number of columns (e.g., 256 floats -> 32 bytes).
	 * @return Descriptors in binary format (CV_8UC1)
	 */
	static cv::Mat convert32FToBin(const cv::Mat & descriptorsIn, bool byteToFloat = true);

protected:
	/**
	 * @brief Get the next available visual word ID
	 * @return Next unique word ID
	 */
	int getNextId();

protected:
	/**
	 * @brief Map of visual word ID to VisualWord pointer
	 * @note All visual words (used and unused) are stored here
	 */
	std::map<int, VisualWord *> _visualWords; //<id,VisualWord*>
	
	/**
	 * @brief Total count of active word-to-signature references
	 * @note Used to track all references for updating common signatures
	 */
	int _totalActiveReferences;

private:
	/**
	 * @brief Whether the dictionary is in incremental mode
	 */
	bool _incrementalDictionary;
	
	/**
	 * @brief Whether FLANN index is updated incrementally
	 */
	bool _incrementalFlann;
	
	/**
	 * @brief Rebalancing factor for FLANN index updates
	 */
	float _rebalancingFactor;
	
	/**
	 * @brief Whether to convert descriptors from byte to float format
	 */
	bool _byteToFloat;
	
	/**
	 * @brief Nearest Neighbor Distance Ratio threshold
	 * 
	 * @note The NNDR method was introduced in "Distinctive Image Features
	 * from Scale-Invariant Keypoints" by David Lowe (IJCV 2004).
	 */
	float _nndrRatio;
	
	/**
	 * @brief Path to the pre-computed dictionary file (.txt or .db)
	 */
	std::string _dictionaryPath; // a pre-computed dictionary (.txt or .db)
	
	/**
	 * @brief Path to a new dictionary file to load
	 */
	std::string _newDictionaryPath; // a pre-computed dictionary (.txt or .db)
	
	/**
	 * @brief Whether new words should be compared together before adding
	 */
	bool _newWordsComparedTogether;
	
	/**
	 * @brief Whether to include checksum when serializing index
	 */
	bool _serializeWithChecksum;
	
	/**
	 * @brief ID of the last visual word added
	 */
	int _lastWordId;
	
	/**
	 * @brief Whether to use L1 distance metric instead of L2
	 */
	bool useDistanceL1_;
	
	/**
	 * @brief FLANN index for fast nearest neighbor search
	 */
	FlannIndex * _flannIndex;
	
	/**
	 * @brief Data matrix for the search tree
	 */
	cv::Mat _dataTree;
	
	/**
	 * @brief Whether the dictionary has been modified since last save
	 */
	bool _modified;
	
	/**
	 * @brief Current nearest neighbor search strategy
	 */
	NNStrategy _strategy;
	
	/**
	 * @brief Map from search index position to visual word ID
	 */
	std::map<int ,int> _mapIndexId;
	
	/**
	 * @brief Map from visual word ID to search index position
	 */
	std::map<int ,int> _mapIdIndex;
	
	/**
	 * @brief Map of unused visual words (words with no references)
	 * @note These words remain in _visualWords but are marked as unused
	 */
	std::map<int, VisualWord*> _unusedWords; //<id,VisualWord*>
	/**
	 * @brief Set of word IDs that are not yet indexed in the search tree
	 */
	std::set<int> _notIndexedWords;
	
	/**
	 * @brief Set of word IDs that were removed from dictionary but still indexed
	 */
	std::set<int> _removedIndexedWords;
};

} // namespace rtabmap
