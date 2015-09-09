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

#include "rtabmap/core/VWDictionary.h"
#include "rtabmap/core/VisualWord.h"

#include "rtabmap/core/Signature.h"
#include "rtabmap/core/DBDriver.h"
#include "rtabmap/core/Parameters.h"

#include "rtabmap/utilite/UtiLite.h"

#include <opencv2/opencv_modules.hpp>

#if CV_MAJOR_VERSION < 3
#include <opencv2/gpu/gpu.hpp>
#else
#include <opencv2/core/cuda.hpp>
#ifdef HAVE_OPENCV_CUDAFEATURES2D
#include <opencv2/cudafeatures2d.hpp>
#endif
#endif

#include <flann/flann.hpp>

#include <fstream>
#include <string>

namespace rtabmap
{

class FlannIndex
{
public:
	FlannIndex():
		index_(0),
		nextIndex_(0),
		featuresType_(0),
		featuresDim_(0),
		isLSH_(false)
	{
	}
	virtual ~FlannIndex()
	{
		this->release();
	}

	void release()
	{
		if(index_)
		{
			if(featuresType_ == CV_8UC1)
			{
				
#ifdef WITH_FLANN18
			delete (flann::Index<flann::Hamming<unsigned char> >*)index_;
#else
			// issue with 1.7.1: we should explicitly use the corresponding index
			if(isLSH_)
			{
				delete (flann::LshIndex<flann::Hamming<unsigned char> >*)index_;
			}
			else
			{
				delete (flann::LinearIndex<flann::Hamming<unsigned char> >*)index_;
			}
#endif
			}
			else
			{
				delete (flann::Index<flann::L2<float> >*)index_;
			}
			index_ = 0;
		}
		nextIndex_ = 0;
		isLSH_ = false;
	}

	unsigned int indexedFeatures() const
	{
		if(!index_)
		{
			return 0;
		}
		if(featuresType_ == CV_8UC1)
		{
#ifdef WITH_FLANN18
			return ((const flann::Index<flann::Hamming<unsigned char> >*)index_)->size();
#else
			// issue with 1.7.1: we should explicitly use the corresponding index
			if(isLSH_)
			{
				return ((const flann::LshIndex<flann::Hamming<unsigned char> >*)index_)->size();
			}
			else
			{
				return ((const flann::LinearIndex<flann::Hamming<unsigned char> >*)index_)->size();
			}
#endif
		}
		else
		{
			return ((const flann::Index<flann::L2<float> >*)index_)->size();
		}
	}

	// return KB
	unsigned int memoryUsed() const
	{
		if(!index_)
		{
			return 0;
		}
		if(featuresType_ == CV_8UC1)
		{	
#ifdef WITH_FLANN18
			return ((const flann::Index<flann::Hamming<unsigned char> >*)index_)->usedMemory()/1000;
#else
			// issue with 1.7.1: we should explicitly use the corresponding index
			if(isLSH_)
			{
				return ((const flann::LshIndex<flann::Hamming<unsigned char> >*)index_)->usedMemory()/1000;
			}
			else
			{
				return ((const flann::LinearIndex<flann::Hamming<unsigned char> >*)index_)->usedMemory()/1000;
			}
#endif
		}
		else
		{
			return ((const flann::Index<flann::L2<float> >*)index_)->usedMemory()/1000;
		}
	}

	void build(
			const cv::Mat & features,
			const flann::IndexParams& params)
	{
		this->release();
		UASSERT(index_ == 0);
		UASSERT(features.type() == CV_32FC1 || features.type() == CV_8UC1);
		featuresType_ = features.type();
		featuresDim_ = features.cols;

		if(featuresType_ == CV_8UC1)
		{
			flann::Matrix<unsigned char> dataset(features.data, features.rows, features.cols);
#ifdef WITH_FLANN18
			index_ = new flann::Index<flann::Hamming<unsigned char> >(dataset, params);
#else
			// issue with 1.7.1: we should explicitly create the corresponding index
			flann::flann_algorithm_t algo = params.at("algorithm").cast<flann::flann_algorithm_t>();
			if(algo == flann::FLANN_INDEX_LSH)
			{
				isLSH_ = true;
				index_ = new flann::LshIndex<flann::Hamming<unsigned char> >(dataset, params);
			}
			else
			{
				index_ = new flann::LinearIndex<flann::Hamming<unsigned char> >(dataset, params);
			}
#endif
			((flann::Index<flann::Hamming<unsigned char> >*)index_)->buildIndex();
		}
		else
		{
			flann::Matrix<float> dataset((float*)features.data, features.rows, features.cols);
			index_ = new flann::Index<flann::L2<float> >(dataset, params);
			((flann::Index<flann::L2<float> >*)index_)->buildIndex();
		}
		nextIndex_ = features.rows;
	}

	bool isIncremental()
	{
#ifdef WITH_FLANN18
		return true;
#else
		return false;
#endif
	}

	bool isBuilt()
	{
		return index_!=0;
	}

	int featuresType() const {return featuresType_;}
	int featuresDim() const {return featuresDim_;}

	unsigned int addPoint(const cv::Mat & feature)
	{
#ifdef WITH_FLANN18
		if(!index_)
		{
			UERROR("Flann index not yet created!");
			return 0;
		}
		UASSERT(feature.type() == featuresType_);
		UASSERT(feature.cols == featuresDim_);
		UASSERT(feature.rows == 1);
		if(featuresType_ == CV_8UC1)
		{
			flann::Matrix<unsigned char> dataset(feature.data, feature.rows, feature.cols);
			((flann::Index<flann::Hamming<unsigned char> >*)index_)->addPoints(dataset);
		}
		else
		{
			flann::Matrix<float> dataset((float*)feature.data, feature.rows, feature.cols);
			((flann::Index<flann::L2<float> >*)index_)->addPoints(dataset);
		}
		return nextIndex_++;
#else
		UFATAL("Not built with FLANN 1.8! Only when isIncremental() returns true that you can call this method.");
		return 0;
#endif
	}

	void removePoint(unsigned int index)
	{
#ifdef WITH_FLANN18
		if(!index_)
		{
			UERROR("Flann index not yet created!");
			return;
		}
		if(featuresType_ == CV_8UC1)
		{
			((flann::Index<flann::Hamming<unsigned char> >*)index_)->removePoint(index);
		}
		else
		{
			((flann::Index<flann::L2<float> >*)index_)->removePoint(index);
		}
#else
		UFATAL("Not built with FLANN 1.8! Only when isIncremental() returns true that you can call this method.");
#endif
	}

	void knnSearch(
			const cv::Mat & query,
			cv::Mat & indices,
			cv::Mat & dists,
	        int knn,
	        const flann::SearchParams& params=flann::SearchParams())
	{
		if(!index_)
		{
			UERROR("Flann index not yet created!");
			return;
		}
		indices.create(query.rows, knn, CV_32S);
		dists.create(query.rows, knn, featuresType_ == CV_8UC1?CV_32S:CV_32F);

		cv::flann::IndexParams i;

		flann::Matrix<int> indicesF((int*)indices.data, indices.rows, indices.cols);

		if(featuresType_ == CV_8UC1)
		{
			flann::Matrix<unsigned int> distsF((unsigned int*)dists.data, dists.rows, dists.cols);
			flann::Matrix<unsigned char> queryF(query.data, query.rows, query.cols);
#ifdef WITH_FLANN18
			((flann::Index<flann::Hamming<unsigned char> >*)index_)->knnSearch(queryF, indicesF, distsF, knn, params);
#else
			// issue with 1.7.1: we should explicitly use the corresponding index
			if(isLSH_)
			{
				((flann::LshIndex<flann::Hamming<unsigned char> >*)index_)->knnSearch(queryF, indicesF, distsF, knn, params);
			}
			else
			{
				((flann::LinearIndex<flann::Hamming<unsigned char> >*)index_)->knnSearch(queryF, indicesF, distsF, knn, params);
			}
#endif
		}
		else
		{
			flann::Matrix<float> distsF((float*)dists.data, dists.rows, dists.cols);
			flann::Matrix<float> queryF((float*)query.data, query.rows, query.cols);
			((flann::Index<flann::L2<float> >*)index_)->knnSearch(queryF, indicesF, distsF, knn, params);
		}
	}

private:
	void * index_;
	unsigned int nextIndex_;
	int featuresType_;
	int featuresDim_;
	bool isLSH_;
};

const int VWDictionary::ID_START = 1;
const int VWDictionary::ID_INVALID = 0;

VWDictionary::VWDictionary(const ParametersMap & parameters) :
	_totalActiveReferences(0),
	_incrementalDictionary(Parameters::defaultKpIncrementalDictionary()),
	_incrementalFlann(Parameters::defaultKpIncrementalFlann()),
	_nndrRatio(Parameters::defaultKpNndrRatio()),
	_dictionaryPath(Parameters::defaultKpDictionaryPath()),
	_newWordsComparedTogether(Parameters::defaultKpNewWordsComparedTogether()),
	_lastWordId(0),
	_flannIndex(new FlannIndex()),
	_strategy(kNNBruteForce)
{
	this->setNNStrategy((NNStrategy)Parameters::defaultKpNNStrategy());
	this->parseParameters(parameters);
}

VWDictionary::~VWDictionary()
{
	this->clear();
	delete _flannIndex;
}

void VWDictionary::parseParameters(const ParametersMap & parameters)
{
	ParametersMap::const_iterator iter;
	Parameters::parse(parameters, Parameters::kKpNndrRatio(), _nndrRatio);
	Parameters::parse(parameters, Parameters::kKpNewWordsComparedTogether(), _newWordsComparedTogether);
	Parameters::parse(parameters, Parameters::kKpIncrementalFlann(), _incrementalFlann);

	if(_incrementalFlann && !_flannIndex->isIncremental())
	{
		UERROR("TRying to set \"KpIncrementalFlann\"=true but RTAB-Map is not built with FLANN>=1.8. Setting to false.");
		_incrementalFlann = false;
	}

	UASSERT_MSG(_nndrRatio > 0.0f, uFormat("String=%s value=%f", uContains(parameters, Parameters::kKpNndrRatio())?parameters.at(Parameters::kKpNndrRatio()).c_str():"", _nndrRatio).c_str());

	std::string dictionaryPath = _dictionaryPath;
	bool incrementalDictionary = _incrementalDictionary;
	if((iter=parameters.find(Parameters::kKpDictionaryPath())) != parameters.end())
	{
		dictionaryPath = (*iter).second.c_str();
	}
	if((iter=parameters.find(Parameters::kKpIncrementalDictionary())) != parameters.end())
	{
		incrementalDictionary = uStr2Bool((*iter).second.c_str());
	}

	// Verifying hypotheses strategy
	if((iter=parameters.find(Parameters::kKpNNStrategy())) != parameters.end())
	{
		NNStrategy nnStrategy = (NNStrategy)std::atoi((*iter).second.c_str());
		this->setNNStrategy(nnStrategy);
	}

	if(incrementalDictionary)
	{
		this->setIncrementalDictionary();
	}
	else
	{
		this->setFixedDictionary(dictionaryPath);
	}

}

void VWDictionary::setIncrementalDictionary()
{
	if(!_incrementalDictionary)
	{
		_incrementalDictionary = true;
		if(_visualWords.size())
		{
			UWARN("Incremental dictionary set: already loaded visual words (%d) from the fixed dictionary will be included in the incremental one.", _visualWords.size());
		}
	}
	_dictionaryPath = "";
}

void VWDictionary::setFixedDictionary(const std::string & dictionaryPath)
{
	if(!dictionaryPath.empty())
	{
		if((!_incrementalDictionary && _dictionaryPath.compare(dictionaryPath) != 0) ||
		   _visualWords.size() == 0)
		{
			std::ifstream file;
			file.open(dictionaryPath.c_str(), std::ifstream::in);
			if(file.good())
			{
				UDEBUG("Deleting old dictionary and loading the new one from \"%s\"", dictionaryPath.c_str());
				UTimer timer;

				// first line is the header
				std::string str;
				std::list<std::string> strList;
				std::getline(file, str);
				strList = uSplitNumChar(str);
				unsigned int dimension  = 0;
				for(std::list<std::string>::iterator iter = strList.begin(); iter != strList.end(); ++iter)
				{
					if(uIsDigit(iter->at(0)))
					{
						dimension = std::atoi(iter->c_str());
						break;
					}
				}

				if(dimension == 0 || dimension > 1000)
				{
					UERROR("Invalid dictionary file, visual word dimension (%d) is not valid, \"%s\"", dimension, dictionaryPath.c_str());
				}
				else
				{
					// Process all words
					while(file.good())
					{
						std::getline(file, str);
						strList = uSplit(str);
						if(strList.size() == dimension+1)
						{
							//first one is the visual word id
							std::list<std::string>::iterator iter = strList.begin();
							int id = std::atoi(iter->c_str());
							cv::Mat descriptor(1, dimension, CV_32F);
							++iter;
							unsigned int i=0;

							//get descriptor
							for(;i<dimension && iter != strList.end(); ++i, ++iter)
							{
								descriptor.at<float>(i) = uStr2Float(*iter);
							}
							if(i != dimension)
							{
								UERROR("");
							}

							VisualWord * vw = new VisualWord(id, descriptor, 0);
							_visualWords.insert(_visualWords.end(), std::pair<int, VisualWord*>(id, vw));
							_notIndexedWords.insert(_notIndexedWords.end(), id);
						}
						else
						{
							UWARN("Cannot parse line \"%s\"", str.c_str());
						}
					}
					this->update();
					_incrementalDictionary = false;
				}


				UDEBUG("Time changing dictionary = %fs", timer.ticks());
			}
			else
			{
				UERROR("Cannot open dictionary file \"%s\"", dictionaryPath.c_str());
			}
			file.close();
		}
		else if(!_incrementalDictionary)
		{
			UDEBUG("Dictionary \"%s\" already loaded...", dictionaryPath.c_str());
		}
		else
		{
			UERROR("Cannot change to a fixed dictionary if there are already words (%d) in the incremental one.", _visualWords.size());
		}
	}
	else if(_visualWords.size() == 0)
	{
		_incrementalDictionary = false;
	}
	else if(_incrementalDictionary)
	{
		UWARN("Cannot change to fixed dictionary, %d words already loaded as incremental", (int)_visualWords.size());
	}
	_dictionaryPath = dictionaryPath;
}

void VWDictionary::setNNStrategy(NNStrategy strategy)
{
	if(strategy!=kNNUndef)
	{
#if CV_MAJOR_VERSION < 3
		if(strategy == kNNBruteForceGPU && !cv::gpu::getCudaEnabledDeviceCount())
		{
			UERROR("Nearest neighobr strategy \"kNNBruteForceGPU\" chosen but no CUDA devices found! Doing \"kNNBruteForce\" instead.");
			strategy = kNNBruteForce;
		}
#else
		if(strategy == kNNBruteForceGPU && !cv::cuda::getCudaEnabledDeviceCount())
		{
			UERROR("Nearest neighobr strategy \"kNNBruteForceGPU\" chosen but no CUDA devices found! Doing \"kNNBruteForce\" instead.");
			strategy = kNNBruteForce;
		}
#endif
#ifndef HAVE_OPENCV_CUDAFEATURES2D
		if(strategy == kNNBruteForceGPU)
		{
			UERROR("Nearest neighobr strategy \"kNNBruteForceGPU\" chosen but OpenCV cudafeatures2d module is not found! Doing \"kNNBruteForce\" instead.");
			strategy = kNNBruteForce;
		}
#endif

		if(RTABMAP_NONFREE == 0 && strategy == kNNFlannKdTree)
		{
			UWARN("KdTree (%d) nearest neighbor is not available because RTAB-Map isn't built "
				  "with OpenCV nonfree module (KdTree only used for SURF/SIFT features). "
				  "NN strategy is not modified (current=%d).", (int)kNNFlannKdTree, (int)_strategy);
		}
		else
		{
			bool update = _strategy != strategy;
			_strategy = strategy;
			if(update)
			{
				_dataTree = cv::Mat();
				_notIndexedWords = uKeysSet(_visualWords);
				_removedIndexedWords.clear();
				this->update();
			}
		}
	}
}

int VWDictionary::getLastIndexedWordId() const
{
	if(_mapIndexId.size())
	{
		return _mapIndexId.rbegin()->second;
	}
	else
	{
		return 0;
	}
}

unsigned int VWDictionary::getIndexedWordsCount() const
{
	return _flannIndex->indexedFeatures();
}

unsigned int VWDictionary::getIndexMemoryUsed() const
{
	return _flannIndex->memoryUsed();
}

void VWDictionary::update()
{
	ULOGGER_DEBUG("");
	if(!_incrementalDictionary && !_notIndexedWords.size())
	{
		// No need to update the search index if we
		// use a fixed dictionary and the index is
		// already built
		return;
	}

	if(_notIndexedWords.size() || _visualWords.size() == 0 || _removedIndexedWords.size())
	{
		if(_incrementalFlann &&
		   _flannIndex->isIncremental() &&
		   _strategy < kNNBruteForce &&
		   _visualWords.size())
		{
			if(_notIndexedWords.size())
			{
				for(std::set<int>::iterator iter=_notIndexedWords.begin(); iter!=_notIndexedWords.end(); ++iter)
				{
					VisualWord* w = uValue(_visualWords, *iter, (VisualWord*)0);
					UASSERT(w);
					int index = 0;
					if(!_flannIndex->isBuilt())
					{
						switch(_strategy)
						{
						case kNNFlannNaive:
							_flannIndex->build(w->getDescriptor(), flann::LinearIndexParams());
							break;
						case kNNFlannKdTree:
							UASSERT_MSG(w->getDescriptor().type() == CV_32F, "To use KdTree dictionary, float descriptors are required!");
							_flannIndex->build(w->getDescriptor(), flann::KDTreeIndexParams());
							break;
						case kNNFlannLSH:
							UASSERT_MSG(w->getDescriptor().type() == CV_8U, "To use LSH dictionary, binary descriptors are required!");
							_flannIndex->build(w->getDescriptor(), flann::LshIndexParams(12, 20, 2));
							break;
						default:
							UFATAL("Not supposed to be here!");
							break;
						}
					}
					else
					{
						UASSERT(w->getDescriptor().cols == _flannIndex->featuresDim());
						UASSERT(w->getDescriptor().type() == _flannIndex->featuresType());
						index = _flannIndex->addPoint(w->getDescriptor());
					}
					_mapIndexId.insert(_mapIndexId.end(), std::pair<int, int>(index, w->id()));
					std::pair<std::map<int, int>::iterator, bool> inserted = _mapIdIndex.insert(std::pair<int, int>(w->id(), index));
					if(!inserted.second)
					{
						//update to new index
						inserted.first->second = index;
					}
				}
			}
			for(std::set<int>::iterator iter=_removedIndexedWords.begin(); iter!=_removedIndexedWords.end(); ++iter)
			{
				UASSERT(uContains(_mapIdIndex, *iter));
				_flannIndex->removePoint(_mapIdIndex.at(*iter));
			}
		}
		else if(_strategy >= kNNBruteForce &&
				_notIndexedWords.size() &&
				_removedIndexedWords.size() == 0 &&
				_visualWords.size() &&
				_dataTree.rows)
		{
			//just add not indexed words
			int i = _dataTree.rows;
			_dataTree.reserve(_dataTree.rows + _notIndexedWords.size());
			for(std::set<int>::iterator iter=_notIndexedWords.begin(); iter!=_notIndexedWords.end(); ++iter)
			{
				VisualWord* w = uValue(_visualWords, *iter, (VisualWord*)0);
				UASSERT(w);
				UASSERT(w->getDescriptor().cols == _dataTree.cols);
				UASSERT(w->getDescriptor().type() == _dataTree.type());
				_dataTree.push_back(w->getDescriptor());
				_mapIndexId.insert(_mapIndexId.end(), std::pair<int, int>(i, w->id()));
				std::pair<std::map<int, int>::iterator, bool> inserted = _mapIdIndex.insert(std::pair<int, int>(w->id(), i));
				UASSERT(inserted.second);
				++i;
			}
		}
		else
		{
			_mapIndexId.clear();
			_mapIdIndex.clear();
			_dataTree = cv::Mat();
			_flannIndex->release();

			if(_visualWords.size())
			{
				UTimer timer;
				timer.start();

				int type = _visualWords.begin()->second->getDescriptor().type();
				int dim = _visualWords.begin()->second->getDescriptor().cols;

				UASSERT(type == CV_32F || type == CV_8U);
				UASSERT(dim > 0);

				// Create the data matrix
				_dataTree = cv::Mat(_visualWords.size(), dim, type); // SURF descriptors are CV_32F
				std::map<int, VisualWord*>::const_iterator iter = _visualWords.begin();
				for(unsigned int i=0; i < _visualWords.size(); ++i, ++iter)
				{
					UASSERT(iter->second->getDescriptor().cols == dim);
					UASSERT(iter->second->getDescriptor().type() == type);

					iter->second->getDescriptor().copyTo(_dataTree.row(i));
					_mapIndexId.insert(_mapIndexId.end(), std::pair<int, int>(i, iter->second->id()));
					_mapIdIndex.insert(_mapIdIndex.end(), std::pair<int, int>(iter->second->id(), i));
				}

				ULOGGER_DEBUG("_mapIndexId.size() = %d, words.size()=%d, _dim=%d",_mapIndexId.size(), _visualWords.size(), dim);
				ULOGGER_DEBUG("copying data = %f s", timer.ticks());

				switch(_strategy)
				{
				case kNNFlannNaive:
					_flannIndex->build(_dataTree, flann::LinearIndexParams());
					break;
				case kNNFlannKdTree:
					UASSERT_MSG(type == CV_32F, "To use KdTree dictionary, float descriptors are required!");
					_flannIndex->build(_dataTree, flann::KDTreeIndexParams());
					break;
				case kNNFlannLSH:
					UASSERT_MSG(type == CV_8U, "To use LSH dictionary, binary descriptors are required!");
					_flannIndex->build(_dataTree, flann::LshIndexParams(12, 20, 2));
					break;
				default:
					break;
				}

				ULOGGER_DEBUG("Time to create kd tree = %f s", timer.ticks());
			}
		}
		UDEBUG("Dictionary updated! (size=%d added=%d removed=%d)",
				_dataTree.rows, _notIndexedWords.size(), _removedIndexedWords.size());
	}
	else
	{
		UDEBUG("Dictionary has not changed, so no need to update it! (size=%d)", _dataTree.rows);
	}
	_notIndexedWords.clear();
	_removedIndexedWords.clear();
}

void VWDictionary::clear()
{
	ULOGGER_DEBUG("");
	if(_visualWords.size() && _incrementalDictionary)
	{
		UWARN("Visual dictionary would be already empty here (%d words still in dictionary).", (int)_visualWords.size());
	}
	if(_notIndexedWords.size())
	{
		UWARN("Not indexed words should be empty here (%d words still not indexed)", (int)_notIndexedWords.size());
	}
	for(std::map<int, VisualWord *>::iterator i=_visualWords.begin(); i!=_visualWords.end(); ++i)
	{
		delete (*i).second;
	}
	_visualWords.clear();
	_notIndexedWords.clear();
	_removedIndexedWords.clear();
	_totalActiveReferences = 0;
	_lastWordId = 0;
	_dataTree = cv::Mat();
	_mapIndexId.clear();
	_mapIdIndex.clear();
	_unusedWords.clear();
	_flannIndex->release();
}

int VWDictionary::getNextId()
{
	return ++_lastWordId;
}

void VWDictionary::addWordRef(int wordId, int signatureId)
{
	if(signatureId > 0 && wordId > 0)
	{
		VisualWord * vw = 0;
		vw = uValue(_visualWords, wordId, vw);
		if(vw)
		{
			vw->addRef(signatureId);
			_totalActiveReferences += 1;

			_unusedWords.erase(vw->id());
		}
		else
		{
			UERROR("Not found word %d", wordId);
		}
	}
}

void VWDictionary::removeAllWordRef(int wordId, int signatureId)
{
	VisualWord * vw = 0;
	vw = uValue(_visualWords, wordId, vw);
	if(vw)
	{
		_totalActiveReferences -= vw->removeAllRef(signatureId);
		if(vw->getReferences().size() == 0)
		{
			_unusedWords.insert(std::pair<int, VisualWord*>(vw->id(), vw));
		}
	}
}

std::list<int> VWDictionary::addNewWords(const cv::Mat & descriptors,
							   int signatureId)
{
	UASSERT(signatureId > 0);

	UDEBUG("id=%d descriptors=%d", signatureId, descriptors.rows);
	UTimer timer;
	std::list<int> wordIds;
	if(descriptors.rows == 0 || descriptors.cols == 0)
	{
		UERROR("Descriptors size is null!");
		return wordIds;
	}
	int dim = 0;
	int type = -1;
	if(_visualWords.size())
	{
		dim = _visualWords.begin()->second->getDescriptor().cols;
		type = _visualWords.begin()->second->getDescriptor().type();
		UASSERT(type == CV_32F || type == CV_8U);
	}

	if(dim && dim != descriptors.cols)
	{
		UERROR("Descriptors (size=%d) are not the same size as already added words in dictionary(size=%d)", descriptors.cols, dim);
		return wordIds;
	}
	dim = descriptors.cols;

	if(type>=0 && type != descriptors.type())
	{
		UERROR("Descriptors (type=%d) are not the same type as already added words in dictionary(type=%d)", descriptors.type(), type);
		return wordIds;
	}
	type = descriptors.type();

	if(!_incrementalDictionary && _visualWords.empty())
	{
		UERROR("Dictionary mode is set to fixed but no words are in it!");
		return wordIds;
	}

	int dupWordsCountFromDict= 0;
	int dupWordsCountFromLast= 0;

	unsigned int k=2; // k nearest neighbors

	cv::Mat newWords;
	std::vector<int> newWordsId;

	cv::Mat results;
	cv::Mat dists;
	std::vector<std::vector<cv::DMatch> > matches;
	bool bruteForce = false;

	UTimer timerLocal;
	timerLocal.start();

	if(_flannIndex->isBuilt() || (!_dataTree.empty() && _dataTree.rows >= (int)k))
	{
		//Find nearest neighbors
		UDEBUG("newPts.total()=%d ", descriptors.rows);

		if(_strategy == kNNFlannNaive || _strategy == kNNFlannKdTree || _strategy == kNNFlannLSH)
		{
			_flannIndex->knnSearch(descriptors, results, dists, k);
		}
		else if(_strategy == kNNBruteForce)
		{
			bruteForce = true;
			cv::BFMatcher matcher(type==CV_8U?cv::NORM_HAMMING:cv::NORM_L2SQR);
			matcher.knnMatch(descriptors, _dataTree, matches, k);
		}
		else if(_strategy == kNNBruteForceGPU)
		{
			bruteForce = true;
#if CV_MAJOR_VERSION < 3
			cv::gpu::GpuMat newDescriptorsGpu(descriptors);
			cv::gpu::GpuMat lastDescriptorsGpu(_dataTree);
			if(type==CV_8U)
			{
				cv::gpu::BruteForceMatcher_GPU<cv::Hamming> gpuMatcher;
				gpuMatcher.knnMatch(newDescriptorsGpu, lastDescriptorsGpu, matches, k);
			}
			else
			{
				cv::gpu::BruteForceMatcher_GPU<cv::L2<float> > gpuMatcher;
				gpuMatcher.knnMatch(newDescriptorsGpu, lastDescriptorsGpu, matches, k);
			}
#else
#ifdef HAVE_OPENCV_CUDAFEATURES2D
			cv::cuda::GpuMat newDescriptorsGpu(descriptors);
			cv::cuda::GpuMat lastDescriptorsGpu(_dataTree);
			cv::Ptr<cv::cuda::DescriptorMatcher> gpuMatcher;
			if(type==CV_8U)
			{
				gpuMatcher = cv::cuda::DescriptorMatcher::createBFMatcher(cv::NORM_HAMMING);
				gpuMatcher->knnMatch(newDescriptorsGpu, lastDescriptorsGpu, matches, k);
			}
			else
			{
				gpuMatcher = cv::cuda::DescriptorMatcher::createBFMatcher(cv::NORM_L2);
				gpuMatcher->knnMatch(newDescriptorsGpu, lastDescriptorsGpu, matches, k);
			}
#endif
#endif
		}
		else
		{
			UFATAL("");
		}

		// In case of binary descriptors
		if(dists.type() == CV_32S)
		{
			cv::Mat temp;
			dists.convertTo(temp, CV_32F);
			dists = temp;
		}

		UDEBUG("Time to find nn = %f s", timerLocal.ticks());
	}

	// Process results
	for(int i = 0; i < descriptors.rows; ++i)
	{
		std::multimap<float, int> fullResults; // Contains results from the kd-tree search and the naive search in new words
		if(!bruteForce && dists.cols)
		{
			for(int j=0; j<dists.cols; ++j)
			{
				float d = dists.at<float>(i,j);
				int id = uValue(_mapIndexId, results.at<int>(i,j));
				if(d >= 0.0f && id > 0)
				{
					fullResults.insert(std::pair<float, int>(d, id));
				}
				else
				{
					break;
				}
			}
		}
		else if(bruteForce && matches.size())
		{
			for(unsigned int j=0; j<matches.at(i).size(); ++j)
			{
				float d = matches.at(i).at(j).distance;
				int id = uValue(_mapIndexId, matches.at(i).at(j).trainIdx);
				if(d >= 0.0f && id > 0)
				{
					fullResults.insert(std::pair<float, int>(d, id));
				}
				else
				{
					break;
				}
			}
		}

		// Check if this descriptor matches with a word from the last signature (a word not already added to the tree)
		if(_newWordsComparedTogether && newWords.rows)
		{
			FlannIndex linearSeach;
			linearSeach.build(newWords, flann::LinearIndexParams());
			cv::Mat resultsLinear;
			cv::Mat distsLinear;
			linearSeach.knnSearch(descriptors.row(i), resultsLinear, distsLinear, newWords.rows>1?2:1);
			// In case of binary descriptors
			if(distsLinear.type() == CV_32S)
			{
				cv::Mat temp;
				distsLinear.convertTo(temp, CV_32F);
				distsLinear = temp;
			}
			if(resultsLinear.cols)
			{
				for(int j=0; j<resultsLinear.cols; ++j)
				{
					float d = distsLinear.at<float>(0,j);
					if(d >= 0.0f && resultsLinear.at<int>(0,j) >= 0)
					{
						std::multimap<float, int>::iterator iter = fullResults.insert(std::pair<float, int>(d, newWordsId[resultsLinear.at<int>(0,j)]));
						UASSERT(iter->second > 0);
					}
					else
					{
						break;
					}
				}
			}
		}

		if(_incrementalDictionary)
		{
			bool badDist = false;
			if(fullResults.size() == 0)
			{
				badDist = true;
			}
			if(!badDist)
			{
				if(fullResults.size() >= 2)
				{
					// Apply NNDR
					if(fullResults.begin()->first > _nndrRatio * (++fullResults.begin())->first)
					{
						badDist = true; // Rejected
					}
				}
				else
				{
					badDist = true; // Rejected
				}
			}

			if(badDist)
			{
				VisualWord * vw = new VisualWord(getNextId(), descriptors.row(i), signatureId);
				_visualWords.insert(_visualWords.end(), std::pair<int, VisualWord *>(vw->id(), vw));
				_notIndexedWords.insert(_notIndexedWords.end(), vw->id());
				newWords.push_back(vw->getDescriptor());
				newWordsId.push_back(vw->id());
				wordIds.push_back(vw->id());
				UASSERT(vw->id()>0);
			}
			else
			{
				if(_notIndexedWords.find(fullResults.begin()->second) != _notIndexedWords.end())
				{
					++dupWordsCountFromLast;
				}
				else
				{
					++dupWordsCountFromDict;
				}

				this->addWordRef(fullResults.begin()->second, signatureId);
				wordIds.push_back(fullResults.begin()->second);
			}
		}
		else if(fullResults.size())
		{
			// If the dictionary is not incremental, just take the nearest word
			++dupWordsCountFromDict;
			this->addWordRef(fullResults.begin()->second, signatureId);
			wordIds.push_back(fullResults.begin()->second);
			UASSERT(fullResults.begin()->second>0);
		}
	}
	ULOGGER_DEBUG("naive search and add ref/words time = %f s", timerLocal.ticks());

	ULOGGER_DEBUG("%d new words added...", _notIndexedWords.size());
	ULOGGER_DEBUG("%d duplicated words added (from current image = %d)...",
			dupWordsCountFromDict+dupWordsCountFromLast, dupWordsCountFromLast);
	UDEBUG("total time %fs", timer.ticks());

	_totalActiveReferences += _notIndexedWords.size();
	return wordIds;
}

std::vector<int> VWDictionary::findNN(const std::list<VisualWord *> & vws) const
{
	UTimer timer;
	timer.start();
	std::vector<int> resultIds(vws.size(), 0);
	unsigned int k=2; // k nearest neighbor

	if(_visualWords.size() && vws.size())
	{
		int dim = _visualWords.begin()->second->getDescriptor().cols;
		int type = _visualWords.begin()->second->getDescriptor().type();

		if(dim != (*vws.begin())->getDescriptor().cols)
		{
			UERROR("Descriptors (size=%d) are not the same size as already added words in dictionary(size=%d)", (*vws.begin())->getDescriptor().cols, dim);
			return resultIds;
		}

		if(type != (*vws.begin())->getDescriptor().type())
		{
			UERROR("Descriptors (type=%d) are not the same type as already added words in dictionary(type=%d)", (*vws.begin())->getDescriptor().type(), type);
			return resultIds;
		}

		std::vector<std::vector<cv::DMatch> > matches;
		bool bruteForce = false;
		cv::Mat results;
		cv::Mat dists;

		// fill the request matrix
		int index = 0;
		VisualWord * vw;
		cv::Mat query(vws.size(), dim, type);
		for(std::list<VisualWord *>::const_iterator iter=vws.begin(); iter!=vws.end(); ++iter, ++index)
		{
			vw = *iter;
			UASSERT(vw);
			UASSERT(vw->getDescriptor().cols == dim);
			UASSERT(vw->getDescriptor().type() == type);

			vw->getDescriptor().copyTo(query.row(index));
		}
		ULOGGER_DEBUG("Preparation time = %fs", timer.ticks());

		if(_flannIndex->isBuilt() || (!_dataTree.empty() && _dataTree.rows >= (int)k))
		{
			//Find nearest neighbors
			UDEBUG("newPts.total()=%d ", query.total());

			if(_strategy == kNNFlannNaive || _strategy == kNNFlannKdTree || _strategy == kNNFlannLSH)
			{
				_flannIndex->knnSearch(query, results, dists, k);
			}
			else if(_strategy == kNNBruteForce)
			{
				bruteForce = true;
				cv::BFMatcher matcher(type==CV_8U?cv::NORM_HAMMING:cv::NORM_L2SQR);
				matcher.knnMatch(query, _dataTree, matches, k);
			}
			else if(_strategy == kNNBruteForceGPU)
			{
				bruteForce = true;
#if CV_MAJOR_VERSION < 3
				cv::gpu::GpuMat newDescriptorsGpu(query);
				cv::gpu::GpuMat lastDescriptorsGpu(_dataTree);
				if(type==CV_8U)
				{
					cv::gpu::BruteForceMatcher_GPU<cv::Hamming> gpuMatcher;
					gpuMatcher.knnMatch(newDescriptorsGpu, lastDescriptorsGpu, matches, k);
				}
				else
				{
					cv::gpu::BruteForceMatcher_GPU<cv::L2<float> > gpuMatcher;
					gpuMatcher.knnMatch(newDescriptorsGpu, lastDescriptorsGpu, matches, k);
				}
#else
#ifdef HAVE_OPENCV_CUDAFEATURES2D
				cv::cuda::GpuMat newDescriptorsGpu(query);
				cv::cuda::GpuMat lastDescriptorsGpu(_dataTree);
				cv::Ptr<cv::cuda::DescriptorMatcher> gpuMatcher;
				if(type==CV_8U)
				{
					gpuMatcher = cv::cuda::DescriptorMatcher::createBFMatcher(cv::NORM_HAMMING);
					gpuMatcher->knnMatchAsync(newDescriptorsGpu, lastDescriptorsGpu, matches, k);
				}
				else
				{
					gpuMatcher = cv::cuda::DescriptorMatcher::createBFMatcher(cv::NORM_L2);
					gpuMatcher->knnMatchAsync(newDescriptorsGpu, lastDescriptorsGpu, matches, k);
				}
#endif
#endif
			}
			else
			{
				UFATAL("");
			}

			// In case of binary descriptors
			if(dists.type() == CV_32S)
			{
				cv::Mat temp;
				dists.convertTo(temp, CV_32F);
				dists = temp;
			}
		}
		ULOGGER_DEBUG("Search dictionary time = %fs", timer.ticks());

		cv::Mat resultsNotIndexed;
		cv::Mat distsNotIndexed;
		std::map<int, int> mapIndexIdNotIndexed;
		if(_notIndexedWords.size())
		{
			cv::Mat dataNotIndexed = cv::Mat::zeros(_notIndexedWords.size(), dim, type);
			unsigned int index = 0;
			VisualWord * vw;
			for(std::set<int>::iterator iter = _notIndexedWords.begin(); iter != _notIndexedWords.end(); ++iter, ++index)
			{
				vw = _visualWords.at(*iter);
				UASSERT(vw != 0 && vw->getDescriptor().cols == dim && vw->getDescriptor().type() == type);
				vw->getDescriptor().copyTo(dataNotIndexed.row(index));
				mapIndexIdNotIndexed.insert(mapIndexIdNotIndexed.end(), std::pair<int,int>(index, vw->id()));
			}

			// Find nearest neighbor
			ULOGGER_DEBUG("Searching in words not indexed...");
			FlannIndex linearSeach;
			linearSeach.build(dataNotIndexed, flann::LinearIndexParams());
			linearSeach.knnSearch(query, resultsNotIndexed, distsNotIndexed, _notIndexedWords.size()>1?2:1);
			// In case of binary descriptors
			if(distsNotIndexed.type() == CV_32S)
			{
				cv::Mat temp;
				distsNotIndexed.convertTo(temp, CV_32F);
				distsNotIndexed = temp;
			}
		}
		ULOGGER_DEBUG("Search not yet indexed words time = %fs", timer.ticks());

		for(unsigned int i=0; i<vws.size(); ++i)
		{
			std::multimap<float, int> fullResults; // Contains results from the kd-tree search [and the naive search in new words]
			if(!bruteForce && dists.cols)
			{
				for(int j=0; j<dists.cols; ++j)
				{
					float d = dists.at<float>(i,j);
					int id = uValue(_mapIndexId, results.at<int>(i,j));
					if(d >= 0.0f && id > 0)
					{
						fullResults.insert(std::pair<float, int>(d, id));
					}
				}
			}
			else if(bruteForce && matches.size())
			{
				for(unsigned int j=0; j<matches.at(i).size(); ++j)
				{
					float d = matches.at(i).at(j).distance;
					int id = uValue(_mapIndexId, matches.at(i).at(j).trainIdx);
					if(d >= 0.0f && id > 0)
					{
						fullResults.insert(std::pair<float, int>(d, id));
					}
				}
			}

			// not indexed..
			for(int j=0; j<distsNotIndexed.cols; ++j)
			{
				float d = distsNotIndexed.at<float>(i,j);
				if(d >= 0.0f && resultsNotIndexed.at<int>(i,j) > 0)
				{
					std::multimap<float, int>::iterator iter = fullResults.insert(std::pair<float, int>(d, uValue(mapIndexIdNotIndexed, resultsNotIndexed.at<int>(i,j))));
					UASSERT(iter->second > 0);
				}
			}

			if(_incrementalDictionary)
			{
				bool badDist = false;
				if(fullResults.size() == 0)
				{
					badDist = true;
				}
				if(!badDist)
				{
					if(fullResults.size() >= 2)
					{
						// Apply NNDR
						if(fullResults.begin()->first > _nndrRatio * (++fullResults.begin())->first)
						{
							badDist = true; // Rejected
						}
					}
					else
					{
						badDist = true; // Rejected
					}
				}

				if(!badDist)
				{
					resultIds[i] = fullResults.begin()->second; // Accepted
				}
			}
			else if(fullResults.size())
			{
				//Just take the nearest if the dictionary is not incremental
				resultIds[i] = fullResults.begin()->second; // Accepted
			}
		}
		ULOGGER_DEBUG("badDist check time = %fs", timer.ticks());
	}
	return resultIds;
}

void VWDictionary::addWord(VisualWord * vw)
{
	if(vw)
	{
		_visualWords.insert(std::pair<int, VisualWord *>(vw->id(), vw));
		_notIndexedWords.insert(vw->id());
		if(vw->getReferences().size())
		{
			_totalActiveReferences += uSum(uValues(vw->getReferences()));
		}
		else
		{
			_unusedWords.insert(std::pair<int, VisualWord *>(vw->id(), vw));
		}
	}
}

const VisualWord * VWDictionary::getWord(int id) const
{
	return uValue(_visualWords, id, (VisualWord *)0);
}

VisualWord * VWDictionary::getUnusedWord(int id) const
{
	return uValue(_unusedWords, id, (VisualWord *)0);
}

std::vector<VisualWord*> VWDictionary::getUnusedWords() const
{
	if(!_incrementalDictionary)
	{
		ULOGGER_WARN("This method does nothing on a fixed dictionary");
		return std::vector<VisualWord*>();
	}
	return uValues(_unusedWords);
}

std::vector<int> VWDictionary::getUnusedWordIds() const
{
	if(!_incrementalDictionary)
	{
		ULOGGER_WARN("This method does nothing on a fixed dictionary");
		return std::vector<int>();
	}
	return uKeys(_unusedWords);
}

void VWDictionary::removeWords(const std::vector<VisualWord*> & words)
{
	for(unsigned int i=0; i<words.size(); ++i)
	{
		_visualWords.erase(words[i]->id());
		_unusedWords.erase(words[i]->id());
		if(_notIndexedWords.erase(words[i]->id()) == 0)
		{
			_removedIndexedWords.insert(words[i]->id());
		}
	}
}

void VWDictionary::deleteUnusedWords()
{
	std::vector<VisualWord*> unusedWords = uValues(_unusedWords);
	removeWords(unusedWords);
	for(unsigned int i=0; i<unusedWords.size(); ++i)
	{
		delete unusedWords[i];
	}
}

void VWDictionary::exportDictionary(const char * fileNameReferences, const char * fileNameDescriptors) const
{
    FILE* foutRef = 0;
    FILE* foutDesc = 0;
#ifdef _MSC_VER
    fopen_s(&foutRef, fileNameReferences, "w");
    fopen_s(&foutDesc, fileNameDescriptors, "w");
#else
    foutRef = fopen(fileNameReferences, "w");
    foutDesc = fopen(fileNameDescriptors, "w");
#endif

    if(foutRef)
    {
    	fprintf(foutRef, "WordID SignaturesID...\n");
    }
	if(foutDesc)
	{
		if(_visualWords.begin() == _visualWords.end())
		{
			fprintf(foutDesc, "WordID Descriptors...\n");
		}
		else
		{
			fprintf(foutDesc, "WordID Descriptors...%d\n", (*_visualWords.begin()).second->getDescriptor().cols);
		}
	}

    for(std::map<int, VisualWord *>::const_iterator iter=_visualWords.begin(); iter!=_visualWords.end(); ++iter)
    {
    	// References
    	if(foutRef)
    	{
			fprintf(foutRef, "%d ", (*iter).first);
			const std::map<int, int> ref = (*iter).second->getReferences();
			for(std::map<int, int>::const_iterator jter=ref.begin(); jter!=ref.end(); ++jter)
			{
				for(int i=0; i<(*jter).second; ++i)
				{
					fprintf(foutRef, "%d ", (*jter).first);
				}
			}
			fprintf(foutRef, "\n");
    	}

    	//Descriptors
    	if(foutDesc)
    	{
			fprintf(foutDesc, "%d ", (*iter).first);
			const float * desc = (const float *)(*iter).second->getDescriptor().data;
			int dim = (*iter).second->getDescriptor().cols;

			for(int i=0; i<dim; i++)
			{
				fprintf(foutDesc, "%f ", desc[i]);
			}
			fprintf(foutDesc, "\n");
    	}
    }

	if(foutRef)
		fclose(foutRef);
	if(foutDesc)
		fclose(foutDesc);
}

} // namespace rtabmap
