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

#include "rtabmap/core/VWDictionary.h"
#include "VisualWord.h"

#include "rtabmap/core/Signature.h"
#include "rtabmap/core/DBDriver.h"
#include "NearestNeighbor.h"
#include "rtabmap/core/Parameters.h"

#include "rtabmap/utilite/UtiLite.h"

#include <fstream>
#include <string>

namespace rtabmap
{

const int VWDictionary::ID_START = 1;
const int VWDictionary::ID_INVALID = 0;

VWDictionary::VWDictionary(const ParametersMap & parameters) :
	_totalActiveReferences(0),
	_incrementalDictionary(Parameters::defaultKpIncrementalDictionary()),
	_minDistUsed(Parameters::defaultKpMinDistUsed()),
	_minDist(Parameters::defaultKpMinDist()),
	_nndrUsed(Parameters::defaultKpNndrUsed()),
	_nndrRatio(Parameters::defaultKpNndrRatio()),
	_maxLeafs(Parameters::defaultKpMaxLeafs()),
	_dictionaryPath(Parameters::defaultKpDictionaryPath()),
	_dim(0),
	_lastWordId(0),
	_nn(0)
{
	this->setNNStrategy((NNStrategy)Parameters::defaultKpNNStrategy(), parameters);
	this->parseParameters(parameters);
}

VWDictionary::~VWDictionary()
{
	this->clear();
	if(_nn)
	{
		delete _nn;
	}
}

void VWDictionary::parseParameters(const ParametersMap & parameters)
{
	ParametersMap::const_iterator iter;
	Parameters::parse(parameters, Parameters::kKpMinDistUsed(), _minDistUsed);
	Parameters::parse(parameters, Parameters::kKpMinDist(), _minDist);
	Parameters::parse(parameters, Parameters::kKpNndrUsed(), _nndrUsed);
	Parameters::parse(parameters, Parameters::kKpNndrRatio(), _nndrRatio);
	Parameters::parse(parameters, Parameters::kKpMaxLeafs(), _maxLeafs);

	UASSERT(_minDist >= 0.0f);
	UASSERT(_nndrRatio >= 0.0f);

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

	NNStrategy nnStrategy = kNNUndef;
	// Verifying hypotheses strategy
	if((iter=parameters.find(Parameters::kKpNNStrategy())) != parameters.end())
	{
		nnStrategy = (NNStrategy)std::atoi((*iter).second.c_str());
	}

	NNStrategy currentNNStrategy = this->nnStrategy();

	if(!_nn || ( nnStrategy!=kNNUndef && (nnStrategy != currentNNStrategy) ) )
	{
		this->setNNStrategy(nnStrategy, parameters);
	}
	else if(_nn)
	{
		_nn->parseParameters(parameters);
	}

	this->setIncrementalDictionary(incrementalDictionary, dictionaryPath);
}

void VWDictionary::setIncrementalDictionary(bool incrementalDictionary, const std::string & dictionaryPath)
{
	if(!incrementalDictionary)
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
								std::vector<float> descriptor(dimension);
								++iter;
								unsigned int i=0;

								//get descriptor
								for(;i<dimension && iter != strList.end(); ++i, ++iter)
								{
									descriptor[i] = std::atof(iter->c_str());
								}
								if(i != dimension)
								{
									UERROR("");
								}

								// laplacian not used
								VisualWord * vw = new VisualWord(id, &(descriptor[0]), dimension, 0);
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
	}
	else if(incrementalDictionary && !_incrementalDictionary)
	{
		_incrementalDictionary = true;
		if(_visualWords.size())
		{
			UWARN("Incremental dictionary set: already loaded visual words (%d) from the fixed dictionary will be included in the incremental one.", _visualWords.size());
		}
	}
	_dictionaryPath = dictionaryPath;
}

void VWDictionary::setNNStrategy(NNStrategy strategy, const ParametersMap & parameters)
{
	if(strategy!=kNNUndef)
	{
		if(_nn)
		{
			delete _nn;
			_nn = 0;
		}
		switch(strategy)
		{
		case kNNFlannKdTree:
			_nn = new FlannNN(FlannNN::kKDTree, parameters);
			break;
		case kNNNaive:
		default:
			// do nothing... _nn must stay = 0
			break;
		}
		if(_nn && !_dataTree.empty())
		{
			_nn->setData(_dataTree);
		}
		else if(!_nn)
		{
			this->update();
			_dataTree = cv::Mat();
		}
	}
}

VWDictionary::NNStrategy VWDictionary::nnStrategy() const
{
	NNStrategy strategy = kNNUndef;
	if(_nn)
	{
		strategy = kNNFlannKdTree;
	}
	else
	{
		strategy = kNNNaive;
	}
	return strategy;
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
		_dataTree = cv::Mat();

		if(_nn && _visualWords.size())
		{
			UTimer timer;
			timer.start();

			if(!_dim)
			{
				_dim = _visualWords.begin()->second->getDim();
			}

			// Create the kd-Tree
			_dataTree = cv::Mat(_visualWords.size(), _dim, CV_32F); // SURF descriptors are CV_32F
			std::map<int, VisualWord*>::const_iterator iter = _visualWords.begin();
			for(unsigned int i=0; i < _visualWords.size(); ++i, ++iter)
			{
				float * rowFl = _dataTree.ptr<float>(i);
				if(iter->second->getDim() == _dim)
				{
					memcpy(rowFl, iter->second->getDescriptor(), _dim*sizeof(float));
					_mapIndexId.insert(_mapIndexId.end(), std::pair<int, int>(i, iter->second->id()));
				}
				else
				{
					ULOGGER_WARN("A word is not the same size than the dictionary, ignoring that word...");
					_mapIndexId.insert(_mapIndexId.end(), std::pair<int, int>(i, 0)); // set to INVALID
				}
			}

			ULOGGER_DEBUG("_mapIndexId.size() = %d, words.size()=%d, _dim=%d",_mapIndexId.size(), _visualWords.size(), _dim);
			ULOGGER_DEBUG("copying data = %f s", timer.ticks());

			// Update the nearest neighbor algorithm
			_nn->setData(_dataTree);

			ULOGGER_DEBUG("Time to create kd tree = %f s", timer.ticks());
		}
	}
	else
	{
		UINFO("Dictionary has not changed, so no need to update it!");
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
	UTimer timer;
	std::list<int> wordIds;
	ULOGGER_DEBUG("");
	if(_dim && _dim != descriptors.cols && descriptors.cols)
	{
		ULOGGER_WARN("Descriptor size has changed! (%d to %d), Nearest neighbor approaches may not work with different descriptor sizes.", _dim, descriptors.cols);
	}
	else if(!descriptors.cols)
	{
		ULOGGER_ERROR("Descriptor size is null?!?");
		return wordIds;
	}
	_dim = descriptors.cols;
	if (descriptors.empty() || !_dim)
	{
		ULOGGER_ERROR("Parameters don't fit the requirements of this method");
		return wordIds;
	}

	if(!_incrementalDictionary && _dataTree.empty())
	{
		UERROR("Dictionary mode is set to fixed but no words are in it!");
		return wordIds;
	}

	int dupWordsCount= 0;

	unsigned int k=1; // k nearest neighbors
	if(_nndrUsed)
	{
		k = 2;
	}

	if(_nn)
	{
		std::list<VisualWord *> newWords;

		cv::Mat results(descriptors.rows, k, CV_32SC1); // results index
		cv::Mat dists;
		dists = cv::Mat(descriptors.rows, k, CV_32FC1); // Distance results are CV_32FC1

		cv::Mat newPts; // SURF descriptors are CV_32F
		if(descriptors.type()!=CV_32F)
		{
			descriptors.convertTo(newPts, CV_32F); // make sure it's CV_32F
		}
		else
		{
			newPts = descriptors;
		}

		UTimer timerLocal;
		timerLocal.start();

		if(!_dataTree.empty() && _dataTree.rows >= (int)k)
		{
			//Find nearest neighbors
			UDEBUG("newPts.total()=%d ", newPts.total());
			_nn->search(newPts, results, dists, k, _maxLeafs);
			ULOGGER_DEBUG("Time to find nn = %f s", timerLocal.ticks());
		}

		//
		for(int i = 0; i < descriptors.rows; ++i)
		{
			// Check if this descriptor matches with a word from the last signature (a word not already added to the tree)
			std::map<float, int> fullResults; // Contains results from the kd-tree search and the naive search in new words
			this->naiveNNSearch(newWords, newPts.ptr<float>(i), _dim, fullResults, k);

			if(!_dataTree.empty() && _dataTree.rows >= (int)k)
			{
				for(unsigned int j=0; j<k; ++j)
				{
					float dist;
					dist = dists.at<float>(i,j);
					fullResults.insert(std::pair<float, int>(dist, uValue(_mapIndexId, results.at<int>(i,j))));
				}
			}

			if(_incrementalDictionary)
			{
				bool badDist = false;
				if(fullResults.size() == 0)
				{
					badDist = true;
				}
				if(!badDist && _minDistUsed && fullResults.begin()->first > _minDist)
				{
					badDist = true;
				}
				if(!badDist && _nndrUsed)
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
						if(!_dataTree.empty())
						{
							UWARN("Not enough nearest neighbors found! fullResults=%d (descriptor %d)", fullResults.size(), i);
						}
						badDist = true; // Rejected
					}
				}

				if(badDist)
				{
					VisualWord * vw = new VisualWord(getNextId(), newPts.ptr<float>(i), _dim, signatureId);
					_visualWords.insert(_visualWords.end(), std::pair<int, VisualWord *>(vw->id(), vw));
					_notIndexedWords.insert(_notIndexedWords.end(), vw->id());
					newWords.push_back(vw);
					wordIds.push_back(vw->id());
					UASSERT(vw->id()>0);
				}
				else
				{
					++dupWordsCount;
					this->addWordRef(fullResults.begin()->second, signatureId);
					wordIds.push_back(fullResults.begin()->second);
					UASSERT(fullResults.begin()->second>0);
				}
			}
			else if(fullResults.size())
			{
				// If the dictionary is not incremental, just take the nearest word
				++dupWordsCount;
				this->addWordRef(fullResults.begin()->second, signatureId);
				wordIds.push_back(fullResults.begin()->second);
				UASSERT(fullResults.begin()->second>0);
			}
		}
		ULOGGER_DEBUG("naive search and add ref/words time = %f s", timerLocal.ticks());
	}
	else //Naive nearest neighbor
	{
		ULOGGER_DEBUG("Naive NN");
		UTimer timer;
		timer.start();
		for(int i=0; i<descriptors.rows; ++i)
		{
			const float* d = descriptors.ptr<float>(i);

			std::map<float, int> results;
			naiveNNSearch(uValuesList(_visualWords), d, _dim, results, k);

			if(_incrementalDictionary)
			{
				bool badDist = false;
				if(results.size() == 0)
				{
					badDist = true;
				}
				if(!badDist && _minDistUsed && results.begin()->first > _minDist)
				{
					badDist = true;
				}
				if(!badDist && _nndrUsed)
				{
					if(results.size() >= 2)
					{
						// Apply NNDR
						if(results.begin()->first > _nndrRatio * (++results.begin())->first)
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
					VisualWord * vw = new VisualWord(getNextId(), d, _dim, signatureId);
					_visualWords.insert(_visualWords.end(), std::pair<int, VisualWord *>(vw->id(), vw));
					_notIndexedWords.insert(_notIndexedWords.end(), vw->id());
					wordIds.push_back(vw->id());
					UASSERT(vw->id()>0);
				}
				else
				{
					++dupWordsCount;
					this->addWordRef(results.begin()->second, signatureId);
					wordIds.push_back(results.begin()->second);
					UASSERT(results.begin()->second>0);
				}
			}
			else if(results.size())
			{
				// If the dictionary is not incremental, just take the nearest word
				++dupWordsCount;
				this->addWordRef(results.begin()->second, signatureId);
				wordIds.push_back(results.begin()->second);
				UASSERT(results.begin()->second>0);
			}
		}
		ULOGGER_DEBUG("Naive search time = %fs", timer.ticks());
	}

	ULOGGER_DEBUG("%d new words added...", _notIndexedWords.size());
	ULOGGER_DEBUG("%d duplicated words added...", dupWordsCount);
	UDEBUG("total time %fs", timer.ticks());

	_totalActiveReferences += _notIndexedWords.size();
	return wordIds;
}

std::vector<int> VWDictionary::findNN(const std::list<VisualWord *> & vws, bool searchInNewlyAddedWords) const
{
	UTimer timer;
	timer.start();
	std::vector<int> resultIds(vws.size(), 0);
	unsigned int k=1; // k nearest neighbor
	if(_nndrUsed)
	{
		k=2;
	}
	if(_nn && _dim)
	{
		if(_visualWords.size() && vws.size() && vws.front())
		{
			cv::Mat results(vws.size(), k, CV_32SC1); // results index
			cv::Mat dists;
			cv::Mat resultsNotIndexed(vws.size(), k, CV_32SC1);
			cv::Mat distsNotIndexed;
			dists = cv::Mat(vws.size(), k, CV_32FC1); // Distance results are CV_32FC1
			distsNotIndexed = cv::Mat(vws.size(), k, CV_32FC1); // Distance results are CV_32FC1
			cv::Mat newPts(vws.size(), _dim, CV_32F); // SURF descriptors are CV_32F

			// fill the request matrix
			int index = 0;
			VisualWord * vw;
			for(std::list<VisualWord *>::const_iterator iter=vws.begin(); iter!=vws.end(); ++iter, ++index)
			{
				vw = *iter;
				float * rowFl = newPts.ptr<float>(index);
				if(vw->getDim() == _dim)
				{
					memcpy(rowFl, vw->getDescriptor(), _dim*sizeof(float));
				}
				else
				{
					ULOGGER_WARN("Descriptors are not the same size! The result may be wrong...");
				}
			}
			ULOGGER_DEBUG("Preparation time = %fs", timer.ticks());

			if(!_dataTree.empty() && _dataTree.rows >= (int)k)
			{
				_nn->search(newPts, results, dists, k, _maxLeafs);
			}
			ULOGGER_DEBUG("Search dictionary time = %fs", timer.ticks());

			std::map<int, int> mapIndexIdNotIndexed;
			unsigned int unreferencedWordsCount = _visualWords.size() - _mapIndexId.size();
			if(searchInNewlyAddedWords && unreferencedWordsCount && unreferencedWordsCount >= (int)k)
			{
				cv::Mat dataNotIndexed = cv::Mat::zeros(unreferencedWordsCount, _dim, CV_32F);
				unsigned int index = 0;
				VisualWord * vw;
				for(std::map<int, VisualWord*>::const_reverse_iterator iter = _visualWords.rbegin();
					iter != _visualWords.rend() && index < unreferencedWordsCount;
					++iter, ++index)
				{
					vw = iter->second;
					float * rowFl = dataNotIndexed.ptr<float>(index);
					if(vw->getDim() == _dim)
					{
						memcpy(rowFl, vw->getDescriptor(), _dim*sizeof(float));
						mapIndexIdNotIndexed.insert(mapIndexIdNotIndexed.end(), std::pair<int,int>(index, vw->id()));
					}
					else
					{
						ULOGGER_WARN("Descriptors are not the same size! The result may be wrong...");
						mapIndexIdNotIndexed.insert(mapIndexIdNotIndexed.end(), std::pair<int,int>(index, 0)); // invalid
					}
				}

				// Find nearest neighbor
				ULOGGER_DEBUG("Searching in words not indexed...");
				_nn->search(dataNotIndexed, newPts, resultsNotIndexed, distsNotIndexed, k, _maxLeafs);
			}
			ULOGGER_DEBUG("Search not yet indexed words time = %fs", timer.ticks());

			for(unsigned int i=0; i<vws.size(); ++i)
			{
				std::map<float, int> fullResults; // Contains results from the kd-tree search [and the naive search in new words]

				for(unsigned int j=0; j<k; ++j)
				{
					float dist;
					if(!_dataTree.empty() && _dataTree.rows >= (int)k)
					{
						dist = dists.at<float>(i,j);
						fullResults.insert(std::pair<float, int>(dist, uValue(_mapIndexId, results.at<int>(i,j))));
					}
					if(searchInNewlyAddedWords && unreferencedWordsCount)
					{
						dist = distsNotIndexed.at<float>(i,j);
						fullResults.insert(std::pair<float, int>(dist, uValue(mapIndexIdNotIndexed, resultsNotIndexed.at<int>(i,j))));
					}
				}

				if(_incrementalDictionary)
				{
					bool badDist = false;
					if(fullResults.size() == 0)
					{
						badDist = true;
					}
					if(!badDist && _minDistUsed && fullResults.begin()->first > _minDist)
					{
						badDist = true;
					}
					if(!badDist && _nndrUsed)
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
	}
	else // Naive search
	{
		ULOGGER_DEBUG("Naive NN");
		UTimer timer;
		timer.start();
		int i=0;
		std::list<VisualWord *> values = uValuesList(_visualWords);
		for(std::list<VisualWord *>::const_iterator iter=vws.begin(); iter!=vws.end(); ++iter, ++i)
		{
			const float* d = (*iter)->getDescriptor();
			unsigned int dim = (*iter)->getDim();

			std::map<float, int> results;
			naiveNNSearch(values, d, dim, results, k);

			if(_incrementalDictionary)
			{
				bool badDist = false;
				if(results.size() == 0)
				{
					badDist = true;
				}
				if(!badDist && _minDistUsed && results.begin()->first > _minDist)
				{
					badDist = true;
				}
				if(!badDist && _nndrUsed)
				{
					if(results.size() >= 2)
					{
						// Apply NNDR
						if(results.begin()->first > _nndrRatio * (++results.begin())->first)
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
					resultIds[i] = results.begin()->second; // Accepted
				}
			}
			else if(results.size())
			{
				// If the dictionary is not incremental, just take the nearest word
				resultIds[i] = results.begin()->second; // Accepted
			}
		}
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

// dist = (euclidean dist)^2, "k" nearest neighbors
void VWDictionary::naiveNNSearch(const std::list<VisualWord *> & words, const float * d, int length, std::map<float, int> & results, unsigned int k) const
{
	double total_cost = 0;
	double t0, t1, t2, t3;
	const float * dw = 0;
	bool goodMatch;

	if(!words.size() || k == 0 || words.size() < k)
	{
		return;
	}

	for(std::list<VisualWord *>::const_iterator vmi=words.begin(); vmi != words.end(); ++vmi)
	{
		goodMatch = true;
		total_cost = 0;

		if((*vmi)->getDim() == length)
		{
			dw = (*vmi)->getDescriptor();

			t0 = 0;
			total_cost += t0*t0;

			// compare descriptors
			int i = 0;
			if(length>=4)
			{
				for(; i <= length-4; i += 4 )
				{
					t0 = d[i] - dw[i];
					t1 = d[i+1] - dw[i+1];
					t2 = d[i+2] - dw[i+2];
					t3 = d[i+3] - dw[i+3];
					total_cost += t0*t0 + t1*t1 + t2*t2 + t3*t3;
					if(results.size() >= k && total_cost > results.rbegin()->first)
					{
						goodMatch = false;
						break;
					}
				}
			}
			for(; goodMatch && i < length; ++i)
			{
				t0 = d[i] - dw[i];
				total_cost += t0*t0;
				if(results.size() >= k && total_cost > results.rbegin()->first)
				{
					goodMatch = false;
					break;
				}
			}
		}
		else
		{
			ULOGGER_WARN("Descriptors are not the same length");
			goodMatch = false;
		}
		if(goodMatch)
		{
			results.insert(std::pair<float, int>(total_cost, (*vmi)->id()));
			if(results.size() > k)
			{
				results.erase(--results.end());
			}
		}
	}
}

void VWDictionary::getCommonWords(unsigned int nbCommonWords, int totalSign, std::list<int> & commonWords) const
{
	UTimer timer;
	timer.start();
	ULOGGER_DEBUG("common words %d, _visualWords.size()=%d)", nbCommonWords, _visualWords.size());
	commonWords.clear();
	if(nbCommonWords && _visualWords.size() && totalSign && nbCommonWords>0)
	{
		// Sort word by reference count and take the 'nbCommonWords' at the end
		std::multimap<int, int> countMap; // <count,id>
		for(std::map<int, VisualWord *>::const_iterator iter=_visualWords.begin(); iter!=_visualWords.end(); ++iter)
		{
			// Keep at most nbCommonWords words, don't need to keep others
			if(countMap.size()>nbCommonWords)
			{
				if((*iter).second->getTotalReferences() > countMap.begin()->first)
				{
					countMap.erase(countMap.begin());
					countMap.insert(std::pair<int, int>((*iter).second->getTotalReferences(), (*iter).first));
				}
			}
			else
			{
				countMap.insert(std::pair<int, int>((*iter).second->getTotalReferences(), (*iter).first));
			}
		}

		//ULOGGER_DEBUG("sum = %d , %d, %d", Util::sum(Util::keys(countMap)), Util::sum(Util::keys(countMap)) - 13823, countMap.size());

		ULOGGER_DEBUG("time = %f s", timer.ticks());

		bool commonWordsWeighted = true; //TODO, make a variable parameter
		if(commonWordsWeighted)
		{
			// We apply the ratio between the reference counts
			// Example :
			//     <start> add the last id (id=42, totalRef=12) 	-> {42}
			//             add the next id (id=78, totalRef=10) 	-> {42, 78}
			//             // here totalRef of the next id=23 is 5, i.e. 1/2 ratio of the last one (10), we re-add previous ids
			//                 add the last id (id=42, totalRef=12) -> {42, 78, 42}
			//                 add the next id (id=78, totalRef=10) -> {42, 78, 42, 78}
			//             add the next id (id=23, totalRef=5) 		-> {42, 78, 42, 78, 23}
			//             add the next id (id=169, totalRef=4) 	-> {42, 78, 42, 78, 23, 169}
			//             ... to have a total of nbCommonWords reference ids (an id can be more than once in the list)
			std::list<int> idsAdded; // <id>
			for(std::multimap<int, int>::reverse_iterator iter=countMap.rbegin(); iter!=countMap.rend() && commonWords.size() < nbCommonWords; ++iter)
			{
				//ULOGGER_DEBUG("count=%d, wordId=%d", (*iter).first, (*iter).second);
				if((*iter).first==0) // No ref?
				{
					break;
				}

				if(iter != countMap.rbegin())
				{
					std::multimap<int, int>::reverse_iterator previous = iter;
					--previous;
					int ratio = (*previous).first / (*iter).first;
					if(ratio>1)
					{
						for(int r=1; r<ratio;++r)
						{
							for(std::list<int>::iterator j=idsAdded.begin(); j!=idsAdded.end(); ++j)
							{
								commonWords.push_back(*j);
								if(commonWords.size() >= nbCommonWords)
								{
									break;
								}
							}
							if(commonWords.size() >= nbCommonWords)
							{
								break;
							}
						}
					}
				}

				if(commonWords.size() < nbCommonWords)
				{
					commonWords.push_back((*iter).second); // <id>
					idsAdded.push_back((*iter).second);
				}
			}
		}
		else
		{
			commonWords = uValuesList(countMap);
		}
		ULOGGER_DEBUG("time = %f s", timer.ticks());
	}
	ULOGGER_DEBUG("size = %d", commonWords.size());
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
			fprintf(foutDesc, "WordID Descriptors...%d\n", (*_visualWords.begin()).second->getDim());
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
			const float * desc = (*iter).second->getDescriptor();
			int dim = (*iter).second->getDim();

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
