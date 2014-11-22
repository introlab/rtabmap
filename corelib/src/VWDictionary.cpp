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
#include "VisualWord.h"

#include "rtabmap/core/Signature.h"
#include "rtabmap/core/DBDriver.h"
#include "rtabmap/core/Parameters.h"

#include "rtabmap/utilite/UtiLite.h"

#include <opencv2/gpu/gpu.hpp>

#include <fstream>
#include <string>

namespace rtabmap
{

const int VWDictionary::ID_START = 1;
const int VWDictionary::ID_INVALID = 0;

VWDictionary::VWDictionary(const ParametersMap & parameters) :
	_totalActiveReferences(0),
	_incrementalDictionary(Parameters::defaultKpIncrementalDictionary()),
	_nndrRatio(Parameters::defaultKpNndrRatio()),
	_dictionaryPath(Parameters::defaultKpDictionaryPath()),
	_newWordsComparedTogether(Parameters::defaultKpNewWordsComparedTogether()),
	_lastWordId(0),
	_flannIndex(new cv::flann::Index()),
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

	UASSERT(_nndrRatio > 0.0f);

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
								descriptor.at<float>(i) = std::atof(iter->c_str());
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
	_dictionaryPath = dictionaryPath;
}

void VWDictionary::setNNStrategy(NNStrategy strategy)
{
	if(strategy!=kNNUndef)
	{
		if(strategy == kNNBruteForceGPU && !cv::gpu::getCudaEnabledDeviceCount())
		{
			UERROR("Nearest neighobr strategy \"kNNBruteForceGPU\" chosen but no CUDA devices found! Doing \"kNNBruteForce\" instead.");
			strategy = kNNBruteForce;
		}

		if(RTABMAP_NONFREE == 0 && strategy == kNNFlannKdTree)
		{
			UWARN("KdTree (%d) nearest neighbor is not available because RTAB-Map isn't built "
				  "with OpenCV nonfree module (KdTree only used for SURF/SIFT features). "
				  "NN strategy is not modified (current=%d).", (int)kNNFlannKdTree, (int)_strategy);
		}
		else
		{
			_strategy = strategy;
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
		_mapIndexId.clear();
		int oldSize = _dataTree.rows;
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
			}

			ULOGGER_DEBUG("_mapIndexId.size() = %d, words.size()=%d, _dim=%d",_mapIndexId.size(), _visualWords.size(), dim);
			ULOGGER_DEBUG("copying data = %f s", timer.ticks());

			switch(_strategy)
			{
			case kNNFlannNaive:
				_flannIndex->build(_dataTree, cv::flann::LinearIndexParams(), type == CV_32F?cvflann::FLANN_DIST_L2:cvflann::FLANN_DIST_HAMMING);
				break;
			case kNNFlannKdTree:
				UASSERT_MSG(type == CV_32F, "To use KdTree dictionary, float descriptors are required!");
				_flannIndex->build(_dataTree, cv::flann::KDTreeIndexParams(), cvflann::FLANN_DIST_L2);
				break;
			case kNNFlannLSH:
				UASSERT_MSG(type == CV_8U, "To use LSH dictionary, binary descriptors are required!");
				_flannIndex->build(_dataTree, cv::flann::LshIndexParams(12, 20, 2), cvflann::FLANN_DIST_HAMMING);
				break;
			default:
				break;
			}

			ULOGGER_DEBUG("Time to create kd tree = %f s", timer.ticks());
		}
		UDEBUG("Dictionary updated! (size=%d->%d added=%d removed=%d)",
				oldSize, _dataTree.rows, _notIndexedWords.size(), _removedIndexedWords.size());
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

	if(!_dataTree.empty() && _dataTree.rows >= (int)k)
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
			cv::BFMatcher matcher(type==CV_8U?cv::NORM_HAMMING:cv::NORM_L2);
			matcher.knnMatch(descriptors, _dataTree, matches, k);
		}
		else if(_strategy == kNNBruteForceGPU)
		{
			bruteForce = true;
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
				if(results.at<int>(i,j) >= 0)
				{
					float d = dists.at<float>(i,j);
					fullResults.insert(std::pair<float, int>(d, uValue(_mapIndexId, results.at<int>(i,j))));
				}
			}
		}
		else if(bruteForce && matches.size())
		{
			for(unsigned int j=0; j<matches.at(i).size(); ++j)
			{
				if(matches.at(i).at(j).trainIdx >= 0)
				{
					float d = matches.at(i).at(j).distance;
					fullResults.insert(std::pair<float, int>(d, uValue(_mapIndexId, matches.at(i).at(j).trainIdx)));
				}
			}
		}

		// Check if this descriptor matches with a word from the last signature (a word not already added to the tree)
		if(_newWordsComparedTogether && newWords.rows)
		{
			cv::flann::Index linearSeach;
			linearSeach.build(newWords, cv::flann::LinearIndexParams(), type == CV_32F?cvflann::FLANN_DIST_L2:cvflann::FLANN_DIST_HAMMING);
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
					if(resultsLinear.at<int>(0,j) >= 0)
					{
						 float d = distsLinear.at<float>(0,j);
						 fullResults.insert(std::pair<float, int>(d, newWordsId[resultsLinear.at<int>(0,j)]));
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
				UASSERT(fullResults.begin()->second>0);
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

		if(!_dataTree.empty() && _dataTree.rows >= (int)k)
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
				cv::BFMatcher matcher(type==CV_8U?cv::NORM_HAMMING:cv::NORM_L2);
				matcher.knnMatch(query, _dataTree, matches, k);
			}
			else if(_strategy == kNNBruteForceGPU)
			{
				bruteForce = true;
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
			cv::flann::Index linearSeach;
			linearSeach.build(dataNotIndexed, cv::flann::LinearIndexParams(), type == CV_32F?cvflann::FLANN_DIST_L2:cvflann::FLANN_DIST_HAMMING);
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
					if(results.at<int>(i,j) > 0)
					{
						float d = dists.at<float>(i,j);
						fullResults.insert(std::pair<float, int>(d, uValue(_mapIndexId, results.at<int>(i,j))));
					}
				}
			}
			else if(bruteForce && matches.size())
			{
				for(unsigned int j=0; j<matches.at(i).size(); ++j)
				{
					if(matches.at(i).at(j).trainIdx > 0)
					{
						float d = matches.at(i).at(j).distance;
						fullResults.insert(std::pair<float, int>(d, uValue(_mapIndexId, matches.at(i).at(j).trainIdx)));
					}
				}
			}

			// not indexed..
			for(int j=0; j<distsNotIndexed.cols; ++j)
			{
				if(resultsNotIndexed.at<int>(i,j) > 0)
				{
					float d = distsNotIndexed.at<float>(i,j);
					fullResults.insert(std::pair<float, int>(d, uValue(mapIndexIdNotIndexed, resultsNotIndexed.at<int>(i,j))));
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
