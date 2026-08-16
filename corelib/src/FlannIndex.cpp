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

#include <rtabmap/core/FlannIndex.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/core/Compression.h>
#include <rtabmap/core/Version.h>
#include <rtabmap/core/Parameters.h>

#include "rtflann/flann.hpp"
#include "nanoflann/NanoFlannIndex.h"
#include <boost/crc.hpp>

namespace rtabmap {

FlannIndex::FlannIndex():
		index_(0),
		nanoIndex_(0),
		nextIndex_(0),
		featuresType_(0),
		featuresDim_(0),
		useDistanceL1_(false),
		rebalancingFactor_(2.0f)
{
}
FlannIndex::~FlannIndex()
{
	this->release();
}

void FlannIndex::release()
{
	if(nanoIndex_)
	{
		UDEBUG("Clearing nanoflann index...");
		delete nanoIndex_;
		nanoIndex_ = 0;
		UDEBUG("Clearing nanoflann index... done!");
	}
	if(index_)
	{
		UDEBUG("Clearing flann index...");
		if(featuresType_ == CV_8UC1)
		{
			delete (rtflann::Index<rtflann::Hamming<unsigned char> >*)index_;
		}
		else
		{
			if(useDistanceL1_)
			{
				delete (rtflann::Index<rtflann::L1<float> >*)index_;
			}
			else if(featuresDim_ <= 3)
			{
				delete (rtflann::Index<rtflann::L2_Simple<float> >*)index_;
			}
			else
			{
				delete (rtflann::Index<rtflann::L2<float> >*)index_;
			}
		}
		index_ = 0;
		UDEBUG("Clearing flann index... done!");
	}
	nextIndex_ = 0;
	addedDescriptors_.clear();
	removedIndexes_.clear();
}

#define FLANN_INDEX_HEADER_SIZE 12

// The rebalancing factor is turned into the fraction of removed features an
// index is allowed to hold before being rebuilt. A factor of 2 used to mean
// "rebuild once the index has doubled in size", it now means "rebuild once half
// of it has been removed": growing an index doesn't degrade it enough to be
// worth a rebuild, removing from it does, as removed features are only marked
// as such and stay in the index until it is rebuilt (see
// corelib/test/test_flann_index.cpp). This is nanoflann's alpha_deleted, which
// both backends now share.
static float removedRatioThreshold(float rebalancingFactor)
{
	if(rebalancingFactor <= 1.0f)
	{
		return 1.0f; // never rebuilt, a ratio of 1 is never reached
	}
	return (rebalancingFactor-1.0f)/rebalancingFactor;
}

template<class T>
static bool needsRebuild(const T * index, float removedRatio)
{
	const size_t total = index->size() + index->removedCount();
	return removedRatio < 1.0f &&
		   total > 0 &&
		   float(index->removedCount()) > removedRatio * float(total);
}

static bool isNanoFlannAlgorithm(FlannIndex::flann_algorithm_t algorithm)
{
	return algorithm == FlannIndex::NANOFLANN_INDEX_KDTREE_SINGLE;
}

static unsigned int computeCrc(const cv::Mat & data)
{
	boost::crc_32_type result;
	result.process_bytes(data.data, data.total()*data.elemSize());
	return result.checksum();
}

// Fill the header prefixing a serialized index. Shared by both backends: the
// same fields are checked back by loadIndex() whichever one produced the index.
static void fillIndexHeader(
		int * header,
		int algorithm,
		int featuresDim,
		bool useDistanceL1,
		float rebalancingFactor, // Deprecated
		int dataRows,
		int dataCols,
		int dataType,
		unsigned int crcValue,
		int indexSize)
{
	int rebalancingFactorAsInt; // Deprecated
	memcpy(&rebalancingFactorAsInt, &rebalancingFactor, sizeof(rebalancingFactor)); // Deprecated
	int crcValueAsInt;
	memcpy(&crcValueAsInt, &crcValue, sizeof(crcValue));
	// Not checked on load: kept so that a later change of the format, adding or
	// removing a field, can tell which one it is reading.
	header[0] = RTABMAP_VERSION_MAJOR;
	header[1] = RTABMAP_VERSION_MINOR;
	header[2] = RTABMAP_VERSION_PATCH;
	header[3] = algorithm;
	header[4] = featuresDim;
	header[5] = useDistanceL1?1:0;
	header[6] = rebalancingFactorAsInt; // Deprecated
	header[7] = dataRows;
	header[8] = dataCols;
	header[9] = dataType;
	header[10] = crcValueAsInt;
	header[11] = indexSize;
	UDEBUG("Header: \"%d.%d.%d\" alg=%d dim=%d L1=%d factor=%f data(%dx%d type=%d, crc=%X) %d",
		header[0],header[1],header[2],
		header[3],
		header[4],
		header[5],
		rebalancingFactor, // Deprecated
		header[7], header[8], header[9], crcValue,
		header[11]);
}

std::vector<unsigned char> FlannIndex::serializeIndex(bool computeChecksum) const {
	if(nanoIndex_)
	{
		std::vector<unsigned char> nanoIndexData = nanoIndex_->serializeIndex();
		if(!nanoIndexData.empty())
		{
			const size_t headerSizeBytes = sizeof(int)*FLANN_INDEX_HEADER_SIZE;
			const cv::Mat dataset = nanoIndex_->indexedPoints();
			std::vector<unsigned char> indexData(headerSizeBytes + nanoIndexData.size());
			int header[FLANN_INDEX_HEADER_SIZE];
			fillIndexHeader(header,
				algorithm_,
				featuresDim_,
				useDistanceL1_,
				rebalancingFactor_,
				dataset.rows,
				dataset.cols,
				dataset.type(),
				computeChecksum?computeCrc(dataset):0,
				(int)nanoIndexData.size());
			memcpy(indexData.data(), header, headerSizeBytes);
			memcpy(indexData.data()+headerSizeBytes, nanoIndexData.data(), nanoIndexData.size());
			return indexData;
		}
		return std::vector<unsigned char>();
	}
	if(index_ && !addedDescriptors_.empty())
	{
#ifdef WIN32
		UERROR("FLANN index serialization is not yet implemented on Windows. Parameter \"%s\" cannot be used.", Parameters::kKpFlannIndexSaved().c_str());
#else
		UTimer timer;
		const int headerSizeBytes = sizeof(int)*FLANN_INDEX_HEADER_SIZE;
		std::vector<unsigned char> indexData(1024*1024*1024 + headerSizeBytes); // Max 1 GB
        FILE* indexDataPtr = fmemopen(indexData.data()+headerSizeBytes, indexData.size() - headerSizeBytes, "wb");
		long bytes_written = 0;
        if (indexDataPtr) {
			if(featuresType_ == CV_8UC1)
			{
				((rtflann::Index<rtflann::Hamming<unsigned char> >*)index_)->save(indexDataPtr);
			}
			else
			{
				if(useDistanceL1_)
				{
					((rtflann::Index<rtflann::L1<float> >*)index_)->save(indexDataPtr);;
				}
				else if(featuresDim_ <= 3)
				{
					((rtflann::Index<rtflann::L2_Simple<float> >*)index_)->save(indexDataPtr);;
				}
				else
				{
					((rtflann::Index<rtflann::L2<float> >*)index_)->save(indexDataPtr);;
				}
			}
			bytes_written = ftell(indexDataPtr);
            fclose(indexDataPtr);
        }
		if(bytes_written < long(indexData.size()-headerSizeBytes))
		{
			//Expected data size and type
			//
			// dataType is stored in the header as a raw cv::Mat type and
			// compared against features.type() on load. That value is NOT
			// stable across OpenCV major versions: OpenCV 5 changed
			// CV_CN_SHIFT from 3 to 5, so a multi-channel type serializes to
			// a different integer than under OpenCV 4 (see the encoding
			// helpers in Compression.cpp, which normalize it for the data
			// blobs stored in the database).
			//
			// It is safe here only because descriptors are always
			// single-channel (asserted CV_32FC1 or CV_8UC1 in buildKDTreeIndex()
			// and friends), and 1-channel types have the same value in both
			// versions. A mismatch would only make loadIndex() refuse the
			// index and rebuild it, never corrupt data -- but if descriptors
			// ever become multi-channel, this field needs the same
			// normalization as Compression.cpp.
			int dataRows = 0;
			int dataCols = 0;
			int dataType = -1;
			cv::Mat dataset;
			std::set<int> removedDescriptors;
			if(computeChecksum){
				removedDescriptors.insert(removedIndexes_.begin(), removedIndexes_.end());
			}
			// A descriptor header can cover more than one point (see the end of
			// buildIndex() and addPoints()), the index of its row r being
			// iter.first+r. addedDescriptors_ is sorted by index, so walking it
			// gives the points back in the order they were added.
			for(const auto & iter: addedDescriptors_)
			{
				UASSERT(!iter.second.empty());
				dataRows += iter.second.rows;
				if(dataCols <= 0) {
					dataCols = iter.second.cols;
				}
				else {
					UASSERT(dataCols == iter.second.cols);
				}
				if(dataType < 0) {
					dataType = iter.second.type();
				}
				else {
					UASSERT(dataType == iter.second.type());
				}
			}
			// Each removed index is one point, whatever the headers cover.
			dataRows -= (int)removedIndexes_.size();

			if(computeChecksum && dataRows > 0) {
				// The checksum is compared against the one of the features
				// given back to loadIndex(), so it is computed over the points
				// still indexed, in the same order.
				dataset.create(dataRows, dataCols, dataType);
				int row = 0;
				for(const auto & iter: addedDescriptors_)
				{
					for(int r=0; r<iter.second.rows; ++r)
					{
						if(removedDescriptors.find(iter.first+r) == removedDescriptors.end())
						{
							UASSERT(row < dataRows);
							iter.second.row(r).copyTo(dataset.row(row++));
						}
					}
				}
				UASSERT(row == dataRows);
			}

			indexData.resize(bytes_written+headerSizeBytes);
			indexData.shrink_to_fit();
			int header[FLANN_INDEX_HEADER_SIZE];
			fillIndexHeader(header,
				algorithm_,
				featuresDim_,
				useDistanceL1_,
				rebalancingFactor_,
				dataRows,
				dataCols,
				dataType,
				computeChecksum?computeCrc(dataset):0,
				(int)bytes_written);
			memcpy(indexData.data(), header, headerSizeBytes);
			return indexData;
		}
		else {
			UERROR("Target buffer too small to serialize index, aborting.");
		}
		UDEBUG("Flann serialization: %fs", timer.ticks());
#endif
	}
	return std::vector<unsigned char>();
}

size_t FlannIndex::indexedFeatures() const
{
	if(nanoIndex_)
	{
		return nanoIndex_->indexedFeatures();
	}
	if(!index_)
	{
		return 0;
	}
	if(featuresType_ == CV_8UC1)
	{
		return ((const rtflann::Index<rtflann::Hamming<unsigned char> >*)index_)->size();
	}
	else
	{
		if(useDistanceL1_)
		{
			return ((const rtflann::Index<rtflann::L1<float> >*)index_)->size();
		}
		else if(featuresDim_ <= 3)
		{
			return ((const rtflann::Index<rtflann::L2_Simple<float> >*)index_)->size();
		}
		else
		{
			return ((const rtflann::Index<rtflann::L2<float> >*)index_)->size();
		}
	}
}

// return Bytes
size_t FlannIndex::memoryUsed() const
{
	if(nanoIndex_)
	{
		return nanoIndex_->memoryUsed();
	}
	if(!index_)
	{
		return 0;
	}
	size_t memoryUsage = sizeof(FlannIndex);
	memoryUsage += addedDescriptors_.size() * (sizeof(int) + sizeof(cv::Mat) + sizeof(std::map<int, cv::Mat>::iterator)) + sizeof(std::map<int, cv::Mat>);
	memoryUsage += sizeof(std::list<int>) + removedIndexes_.size() * sizeof(int);
	if(featuresType_ == CV_8UC1)
	{
		memoryUsage += ((const rtflann::Index<rtflann::Hamming<unsigned char> >*)index_)->usedMemory();
	}
	else
	{
		if(useDistanceL1_)
		{
			memoryUsage += ((const rtflann::Index<rtflann::L1<float> >*)index_)->usedMemory();
		}
		else if(featuresDim_ <= 3)
		{
			memoryUsage += ((const rtflann::Index<rtflann::L2_Simple<float> >*)index_)->usedMemory();
		}
		else
		{
			memoryUsage += ((const rtflann::Index<rtflann::L2<float> >*)index_)->usedMemory();
		}
	}
	return memoryUsage;
}

void FlannIndex::buildIndex(
		flann_algorithm_t algorithm,
		const cv::Mat & features,
		bool useDistanceL1,
		float rebalancingFactor)
{
	UDEBUG("algorithm=%d", (int)algorithm);
	this->release();
	UASSERT(index_ == 0);
	UASSERT(features.type() == CV_32FC1 || features.type() == CV_8UC1);
	featuresType_ = features.type();
	featuresDim_ = features.cols;
	useDistanceL1_ = useDistanceL1;
	rebalancingFactor_ = rebalancingFactor;
	algorithm_ = algorithm;

	if(isNanoFlannAlgorithm(algorithm))
	{
		// The tree keeps its own copy of the points and rebuilds itself, so
		// addedDescriptors_ is not used here.
		nanoIndex_ = new NanoFlannIndex();
		// Nothing to rebuild for a factor of 1: the tree that cannot be added
		// to is the cheapest one, and it upgrades itself if points are added
		// after all.
		nanoIndex_->buildIndex(
			features,
			useDistanceL1_,
			rebalancingFactor_ > 1.0f,
			removedRatioThreshold(rebalancingFactor_));
		return;
	}

	rtflann::IndexParams params;

	switch (algorithm)
	{
	case FLANN_INDEX_LINEAR:
		params = rtflann::LinearIndexParams();
		break;
	case FLANN_INDEX_KDTREE:
		params = rtflann::KDTreeIndexParams(4);
		break;
	case FLANN_INDEX_KDTREE_SINGLE:
		params = rtflann::KDTreeSingleIndexParams(10, true);
		break;
	case FLANN_INDEX_LSH:
		UASSERT(features.type() == CV_8UC1);
		UASSERT_MSG(features.cols >= 8, "LSH requires a minimum of 8 dimensions to provide valid results.");
		params = rtflann::LshIndexParams(12, 20, 2);
		break;
	default:
		UFATAL("The flann algorithm type %d is not supported!", (int)algorithm);
		break;
	}

	if(featuresType_ == CV_8UC1)
	{
		rtflann::Matrix<unsigned char> dataset(features.data, features.rows, features.cols);
		index_ = new rtflann::Index<rtflann::Hamming<unsigned char> >(dataset, params);
		((rtflann::Index<rtflann::Hamming<unsigned char> >*)index_)->buildIndex();
	}
	else
	{
		rtflann::Matrix<float> dataset((float*)features.data, features.rows, features.cols);
		if(useDistanceL1_)
		{
			index_ = new rtflann::Index<rtflann::L1<float> >(dataset, params);
			((rtflann::Index<rtflann::L1<float> >*)index_)->buildIndex();
		}
		else if(featuresDim_ <=3)
		{
			index_ = new rtflann::Index<rtflann::L2_Simple<float> >(dataset, params);
			((rtflann::Index<rtflann::L2_Simple<float> >*)index_)->buildIndex();
		}
		else
		{
			index_ = new rtflann::Index<rtflann::L2<float> >(dataset, params);
			((rtflann::Index<rtflann::L2<float> >*)index_)->buildIndex();
		}
	}

	// incremental FLANN: we should add all headers separately in case we remove
	// some indexes (to keep underlying matrix data allocated)
	if(rebalancingFactor_ > 1.0f)
	{
		for(int i=0; i<features.rows; ++i)
		{
			addedDescriptors_.insert(std::make_pair(nextIndex_++, features.row(i)));
		}
	}
	else
	{
		// tree won't ever be rebalanced, so just keep only one header for the data
		addedDescriptors_.insert(std::make_pair(nextIndex_, features));
		nextIndex_ += features.rows;
	}
	UDEBUG("");
}

bool FlannIndex::loadIndex(
	const std::vector<unsigned char> & indexData,
	flann_algorithm_t algorithm,
	const cv::Mat & features,
	bool useDistanceL1,
	float rebalancingFactor,
	std::string * error)
{
	return loadIndex(
		indexData.data(),
		indexData.size(),
		algorithm,
		features,
		useDistanceL1,
		rebalancingFactor,
		error);
}
bool FlannIndex::loadIndex(
	const unsigned char * indexData,
	size_t indexDataSize,
	flann_algorithm_t algorithm,
	const cv::Mat & features,
	bool useDistanceL1,
	float rebalancingFactor,
	std::string * error)
{
	if(indexDataSize == 0) {
		UWARN("Trying to load empty index....");
		if(error) {
			*error = "Trying to load an empty index.";
		}
		return false;
	}
	UASSERT(indexData!=NULL);
#ifdef WIN32
	if(!isNanoFlannAlgorithm(algorithm)) {
		UERROR("FLANN index deserialization is not yet implemented on Windows. Index cannot be loaded from memory buffer.");
		if(error) {
			*error = "FLANN index deserialization is not yet implemented on Windows.";
		}
		return false;
	}
#endif

	// Check if the features match the expected data from the index
	size_t headerSizeBytes = sizeof(int)*FLANN_INDEX_HEADER_SIZE;
	if(indexDataSize < headerSizeBytes) {
		if(error) {
			*error = uFormat("Wrong header size detected (%ld vs expected %ld).", indexDataSize, headerSizeBytes);
		}
		return false;
	}
	const int * header = (const int *)indexData;
	
	int savedAlgorithm = header[3];
	int savedDim = header[4];
	bool savedDistanceL1 = header[5]==1;
	float savedRebalancingFactor; // Deprecated
	memcpy(&savedRebalancingFactor, &header[6], sizeof(header[6])); // Deprecated
	int savedRows = header[7];
	int savedCols = header[8];
	int savedType = header[9];
	unsigned int savedCrc;
	 memcpy(&savedCrc, &header[10], sizeof(header[10]));
	int savedIndexSize = header[11];

	UDEBUG("Header: \"%d.%d.%d\" alg=%d dim=%d L1=%d factor=%f (deprecated, using %f instead) data(%dx%d type=%d, crc=%X) index size = %d bytes", 
				header[0],header[1],header[2],
				header[3],
				header[4],
				header[5],
				savedRebalancingFactor, // Deprecated
				rebalancingFactor,
				header[7], header[8], header[9], savedCrc, 
				header[11]);

	if(savedAlgorithm != algorithm) {
		if(error) {
			*error = uFormat("Serialized flann algorithm (%d) doesn't match the expected one (%d).", savedAlgorithm, algorithm);
		}
		return false;
	}
	if(savedDim != features.cols) {
		if(error) {
			*error = uFormat("Serialized feature dimension (%d) doesn't match the expected one (%d).", savedDim, features.cols);
		}
		return false;
	}
	if(isNanoFlannAlgorithm(algorithm) && (savedRebalancingFactor > 1.0f) != (rebalancingFactor > 1.0f)) {
		// The factor is what tells the nanoflann structures apart, and they
		// don't serialize to the same thing.
		if(error) {
			*error = uFormat("Serialized index was built with a rebalancing factor of %f, which doesn't select the same structure as %f.",
				savedRebalancingFactor, rebalancingFactor);
		}
		return false;
	}
	if(savedDistanceL1 != useDistanceL1) {
		if(error) {
			*error = uFormat("Serialized \"use distance L1\" (%s) doesn't match the expected one (%s).", savedDistanceL1?"true":"false", useDistanceL1?"true":"false");
		}
		return false;
	}
	if(savedRows != features.rows) {
		if(error) {
			*error = uFormat("Serialized feature count (%d) doesn't match the expected one (%d).", savedRows, features.rows);
		}
		return false;
	}
	if(savedCols != features.cols) {
		if(error) {
			*error = uFormat("Serialized feature dimension (%d) doesn't match the expected one (%d).", savedCols, features.cols);
		}
		return false;
	}
	// Raw cv::Mat type comparison: safe only because descriptors are always
	// single-channel, whose type value is identical under OpenCV 4 and 5
	// (OpenCV 5 changed CV_CN_SHIFT, which only shifts multi-channel types).
	// See the note where the header is written in serializeIndex().
	if(savedType != features.type()) {
		if(error) {
			*error = uFormat("Serialized feature type (%d) doesn't match the expected one (%d).", savedType, features.type());
		}
		return false;
	}
	if(savedCrc != 0) {
		// Compute checksum and compare
		boost::crc_32_type result;
		result.process_bytes(features.data, features.total()*features.elemSize());
		if(savedCrc != result.checksum()) {
			if(error) {
				*error = uFormat("Serialized feature crc (%X) doesn't match the expected one (%X).", savedCrc, result.checksum());
			}
			return false;
		}
	}
	if(savedIndexSize != int(indexDataSize - headerSizeBytes)) {
		if(error) {
			*error = uFormat("Serialized flann index size (%ld) doesn't match the expected one (%ld).", (long)savedIndexSize, indexDataSize - headerSizeBytes);
		}
		return false;
	}
	if(savedIndexSize == 0) {
		if(error) {
			*error = "Serialized flann index is empty.";
		}
		return false;
	}

	this->release();
	UASSERT(index_ == 0);
	UASSERT(features.type() == CV_32FC1 || features.type() == CV_8UC1);
	featuresType_ = features.type();
	featuresDim_ = features.cols;
	useDistanceL1_ = useDistanceL1;
	rebalancingFactor_ = rebalancingFactor;
	algorithm_ = algorithm;

	UDEBUG("algorithm=%d", (int)algorithm);

	if(isNanoFlannAlgorithm(algorithm))
	{
		nanoIndex_ = new NanoFlannIndex();
		if(!nanoIndex_->loadIndex(
				features,
				useDistanceL1_,
				rebalancingFactor_ > 1.0f,
				indexData+headerSizeBytes,
				indexDataSize-headerSizeBytes,
				removedRatioThreshold(rebalancingFactor_),
				10,
				error))
		{
			this->release();
			return false;
		}
		return true;
	}

#ifdef WIN32
	return false; // rtflann deserialization is not implemented on Windows, rejected above
#else
	rtflann::IndexParams params;

	switch (algorithm)
	{
	case FLANN_INDEX_LINEAR:
		params = rtflann::LinearIndexParams();
		break;
	case FLANN_INDEX_KDTREE:
		params = rtflann::KDTreeIndexParams(4);
		break;
	case FLANN_INDEX_KDTREE_SINGLE:
		params = rtflann::KDTreeSingleIndexParams(10, true);
		break;
	case FLANN_INDEX_LSH:
		UASSERT(features.type() == CV_8UC1);
		params = rtflann::LshIndexParams(12, 20, 2);
		break;
	default:
		UFATAL("The flann algorithm type %d is not supported!", (int)algorithm);
		break;
	}

	FILE* indexDataPtr = fmemopen((void*)(indexData+headerSizeBytes), indexDataSize - headerSizeBytes, "r");

	if(featuresType_ == CV_8UC1)
	{
		rtflann::Matrix<unsigned char> dataset(features.data, features.rows, features.cols);
		index_ = new rtflann::Index<rtflann::Hamming<unsigned char> >(dataset, params);
		((rtflann::Index<rtflann::Hamming<unsigned char> >*)index_)->load_saved_index(indexDataPtr);
	}
	else
	{
		rtflann::Matrix<float> dataset((float*)features.data, features.rows, features.cols);
		if(useDistanceL1_)
		{
			index_ = new rtflann::Index<rtflann::L1<float> >(dataset, params);
			((rtflann::Index<rtflann::L1<float> >*)index_)->load_saved_index(indexDataPtr);
		}
		else if(featuresDim_ <=3)
		{
			index_ = new rtflann::Index<rtflann::L2_Simple<float> >(dataset, params);
			((rtflann::Index<rtflann::L2_Simple<float> >*)index_)->load_saved_index(indexDataPtr);
		}
		else
		{
			index_ = new rtflann::Index<rtflann::L2<float> >(dataset, params);
			((rtflann::Index<rtflann::L2<float> >*)index_)->load_saved_index(indexDataPtr);
		}
	}
	fclose(indexDataPtr);

	// incremental FLANN: we should add all headers separately in case we remove
	// some indexes (to keep underlying matrix data allocated)
	
	if(rebalancingFactor_ > 1.0f)
	{
		for(int i=0; i<features.rows; ++i)
		{
			addedDescriptors_.insert(std::make_pair(nextIndex_++, features.row(i)));
		}
	}
	else
	{
		// tree won't ever be rebalanced, so just keep only one header for the data
		addedDescriptors_.insert(std::make_pair(nextIndex_, features));
		nextIndex_ += features.rows;
	}
	return true;
#endif
}

bool FlannIndex::isBuilt()
{
	return index_!=0 || nanoIndex_!=0;
}

std::vector<unsigned int> FlannIndex::addPoints(const cv::Mat & features)
{
	if(nanoIndex_)
	{
		return nanoIndex_->addPoints(features);
	}
	if(!index_)
	{
		UERROR("Flann index not yet created!");
		return std::vector<unsigned int>();
	}
	UASSERT(features.type() == featuresType_);
	UASSERT(features.cols == featuresDim_);
	bool indexRebuilt = false;
	size_t removedPts = 0;
	const float removedRatio = removedRatioThreshold(rebalancingFactor_);
	if(featuresType_ == CV_8UC1)
	{
		rtflann::Matrix<unsigned char> points(features.data, features.rows, features.cols);
		rtflann::Index<rtflann::Hamming<unsigned char> > * index = (rtflann::Index<rtflann::Hamming<unsigned char> >*)index_;
		removedPts = index->removedCount();
		index->addPoints(points, 0);
		if(needsRebuild(index, removedRatio))
		{
			UDEBUG("Rebuilding FLANN index: %d removed of %d", (int)index->removedCount(), (int)(index->size()+index->removedCount()));
			index->buildIndex();
		}
		// if no more removed points, the index has been rebuilt
		indexRebuilt = index->removedCount() == 0 && removedPts>0;
	}
	else
	{
		rtflann::Matrix<float> points((float*)features.data, features.rows, features.cols);
		if(useDistanceL1_)
		{
			rtflann::Index<rtflann::L1<float> > * index = (rtflann::Index<rtflann::L1<float> >*)index_;
			removedPts = index->removedCount();
			index->addPoints(points, 0);
			if(needsRebuild(index, removedRatio))
			{
				UDEBUG("Rebuilding FLANN index: %d removed of %d", (int)index->removedCount(), (int)(index->size()+index->removedCount()));
				index->buildIndex();
			}
			// if no more removed points, the index has been rebuilt
			indexRebuilt = index->removedCount() == 0 && removedPts>0;
		}
		else if(featuresDim_ <= 3)
		{
			rtflann::Index<rtflann::L2_Simple<float> > * index = (rtflann::Index<rtflann::L2_Simple<float> >*)index_;
			removedPts = index->removedCount();
			index->addPoints(points, 0);
			if(needsRebuild(index, removedRatio))
			{
				UDEBUG("Rebuilding FLANN index: %d removed of %d", (int)index->removedCount(), (int)(index->size()+index->removedCount()));
				index->buildIndex();
			}
			// if no more removed points, the index has been rebuilt
			indexRebuilt = index->removedCount() == 0 && removedPts>0;
		}
		else
		{
			rtflann::Index<rtflann::L2<float> > * index = (rtflann::Index<rtflann::L2<float> >*)index_;
			removedPts = index->removedCount();
			index->addPoints(points, 0);
			if(needsRebuild(index, removedRatio))
			{
				UDEBUG("Rebuilding FLANN index: %d removed of %d", (int)index->removedCount(), (int)(index->size()+index->removedCount()));
				index->buildIndex();
			}
			// if no more removed points, the index has been rebuilt
			indexRebuilt = index->removedCount() == 0 && removedPts>0;
		}
	}

	if(indexRebuilt)
	{
		UASSERT(removedPts == removedIndexes_.size());
		// clean not used features
		for(std::list<int>::iterator iter=removedIndexes_.begin(); iter!=removedIndexes_.end(); ++iter)
		{
			addedDescriptors_.erase(*iter);
		}
		removedIndexes_.clear();
	}

	std::vector<unsigned int> indexes;
	indexes.reserve(features.rows);
	for(int i=0; i<features.rows; ++i)
	{
		indexes.push_back(nextIndex_ + i);
	}

	// incremental FLANN: we should add all headers separately in case we remove
	// some indexes (to keep underlying matrix data allocated)
	if(rebalancingFactor_ > 1.0f)
	{
		for(int i=0; i<features.rows; ++i)
		{
			addedDescriptors_.insert(std::make_pair(nextIndex_++, features.row(i)));
		}
	}
	else
	{
		// tree won't ever be rebalanced, so just keep only one header for the data
		addedDescriptors_.insert(std::make_pair(nextIndex_, features));
		nextIndex_ += features.rows;
	}

	return indexes;
}

void FlannIndex::removePoint(unsigned int index)
{
	if(nanoIndex_)
	{
		nanoIndex_->removePoint(index);
		return;
	}
	if(!index_)
	{
		UERROR("Flann index not yet created!");
		return;
	}

	// If a Segmentation fault occurs in removePoint(), verify that you have this fix in your installed "flann/algorithms/nn_index.h":
	// 707 - if (ids_[id]==id) {
	// 707 + if (id < ids_.size() && ids_[id]==id) {
	// ref: https://github.com/mariusmuja/flann/commit/23051820b2314f07cf40ba633a4067782a982ff3#diff-33762b7383f957c2df17301639af5151

	if(featuresType_ == CV_8UC1)
	{
		((rtflann::Index<rtflann::Hamming<unsigned char> >*)index_)->removePoint(index);
	}
	else if(useDistanceL1_)
	{
		((rtflann::Index<rtflann::L1<float> >*)index_)->removePoint(index);
	}
	else if(featuresDim_ <= 3)
	{
		((rtflann::Index<rtflann::L2_Simple<float> >*)index_)->removePoint(index);
	}
	else
	{
		((rtflann::Index<rtflann::L2<float> >*)index_)->removePoint(index);
	}

	removedIndexes_.push_back(index);
}

void FlannIndex::knnSearch(
		const cv::Mat & query,
		cv::Mat & indices,
		cv::Mat & dists,
		int knn,
		int checks,
		float eps,
		bool sorted) const
{
	if(nanoIndex_)
	{
		// exact search, "checks", "eps" and "sorted" don't apply
		nanoIndex_->knnSearch(query, indices, dists, knn);
		return;
	}
	if(!index_)
	{
		UERROR("Flann index not yet created!");
		return;
	}

	dists = cv::Mat(query.rows, knn, featuresType_ == CV_8UC1?CV_32S:CV_32F, cv::Scalar(-1));

	std::vector<size_t> indicesBuffer(query.rows * knn, std::numeric_limits<size_t>::max());
	rtflann::Matrix<size_t> indicesF((size_t*)indicesBuffer.data(), query.rows, knn);

	rtflann::SearchParams params = rtflann::SearchParams(checks, eps, sorted);

	if(featuresType_ == CV_8UC1)
	{
		rtflann::Matrix<unsigned int> distsF((unsigned int*)dists.data, dists.rows, dists.cols);
		rtflann::Matrix<unsigned char> queryF(query.data, query.rows, query.cols);
		((rtflann::Index<rtflann::Hamming<unsigned char> >*)index_)->knnSearch(queryF, indicesF, distsF, knn, params);
	}
	else
	{
		rtflann::Matrix<float> distsF((float*)dists.data, dists.rows, dists.cols);
		rtflann::Matrix<float> queryF((float*)query.data, query.rows, query.cols);
		if(useDistanceL1_)
		{
			((rtflann::Index<rtflann::L1<float> >*)index_)->knnSearch(queryF, indicesF, distsF, knn, params);
		}
		else if(featuresDim_ <= 3)
		{
			((rtflann::Index<rtflann::L2_Simple<float> >*)index_)->knnSearch(queryF, indicesF, distsF, knn, params);
		}
		else
		{
			((rtflann::Index<rtflann::L2<float> >*)index_)->knnSearch(queryF, indicesF, distsF, knn, params);
		}
	}

	indices.create(query.rows, knn, CV_32S);
	int * ptr = indices.ptr<int>();
	for(size_t i=0 ; i<indicesBuffer.size(); ++i)
	{
		// Note: this loop used to write two entries per iteration, which read
		// and wrote one past the end when query.rows*knn is odd (an odd knn on
		// an odd number of queries).
		ptr[i] = indicesBuffer[i] == std::numeric_limits<size_t>::max()?-1:(int)indicesBuffer[i];
	}
}

void FlannIndex::radiusSearch(
		const cv::Mat & query,
		std::vector<std::vector<size_t> > & indices,
		std::vector<std::vector<float> > & dists,
		float radius,
		int maxNeighbors,
		int checks,
		float eps,
		bool sorted) const
{
	if(nanoIndex_)
	{
		// "checks" doesn't apply
		nanoIndex_->radiusSearch(query, indices, dists, radius, maxNeighbors, eps, sorted);
		return;
	}
	if(!index_)
	{
		UERROR("Flann index not yet created!");
		return;
	}

	rtflann::SearchParams params = rtflann::SearchParams(checks, eps, sorted);
	params.max_neighbors = maxNeighbors<=0?-1:maxNeighbors; // -1 is all in radius

	if(featuresType_ == CV_8UC1)
	{
		std::vector<std::vector<unsigned int> > distsF;
		rtflann::Matrix<unsigned char> queryF(query.data, query.rows, query.cols);
		((rtflann::Index<rtflann::Hamming<unsigned char> >*)index_)->radiusSearch(queryF, indices, distsF, radius*radius, params);
		dists.resize(distsF.size());
		for(unsigned int i=0; i<dists.size(); ++i)
		{
			dists[i].resize(distsF[i].size());
			for(unsigned int j=0; j<distsF[i].size(); ++j)
			{
				dists[i][j] = (float)distsF[i][j];
			}
		}
	}
	else
	{
		rtflann::Matrix<float> queryF((float*)query.data, query.rows, query.cols);
		if(useDistanceL1_)
		{
			((rtflann::Index<rtflann::L1<float> >*)index_)->radiusSearch(queryF, indices, dists, radius*radius, params);
		}
		else if(featuresDim_ <= 3)
		{
			((rtflann::Index<rtflann::L2_Simple<float> >*)index_)->radiusSearch(queryF, indices, dists, radius*radius, params);
		}
		else
		{
			((rtflann::Index<rtflann::L2<float> >*)index_)->radiusSearch(queryF, indices, dists, radius*radius, params);
		}
	}
}

} /* namespace rtabmap */
