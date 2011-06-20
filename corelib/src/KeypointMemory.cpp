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

#include "KeypointMemory.h"
#include "VWDictionary.h"
#include "Signature.h"
#include "rtabmap/core/DBDriver.h"
#include "utilite/UtiLite.h"
#include "rtabmap/core/Parameters.h"
#include "rtabmap/core/SMState.h"
#include "rtabmap/core/KeypointDetector.h"
#include "rtabmap/core/KeypointDescriptor.h"
#include "rtabmap/core/RtabmapEvent.h"
#include "NearestNeighbor.h"
#include "VerifyHypotheses.h"
#include "utilite/UStl.h"
#include <opencv2/core/core.hpp>
#include <set>

namespace rtabmap {

KeypointMemory::KeypointMemory(const ParametersMap & parameters) :
	Memory(parameters),
	_keypointDetector(0),
	_keypointDescriptor(0),
	_reactivatedWordsComparedToNewWords(Parameters::defaultKpReactivatedWordsComparedToNewWords()),
	_badSignRatio(Parameters::defaultKpBadSignRatio()),
	_tfIdfLikelihoodUsed(Parameters::defaultKpTfIdfLikelihoodUsed()),
	_parallelized(Parameters::defaultKpParallelized()),
	_sensorStateOnly(Parameters::defaultKpSensorStateOnly()),
	_tfIdfNormalized(Parameters::defaultKpTfIdfNormalized())
{
	_vwd = new VWDictionary(parameters);
	this->parseParameters(parameters);
}

KeypointMemory::~KeypointMemory()
{
	ULOGGER_DEBUG("");
	if(this->memoryChanged())
	{
		this->clear();
	}
	if(_keypointDetector)
	{
		delete _keypointDetector;
	}
	if(_keypointDescriptor)
	{
		delete _keypointDescriptor;
	}
	if(_vwd)
	{
		delete _vwd;
	}
}

void KeypointMemory::parseParameters(const ParametersMap & parameters)
{
	ParametersMap::const_iterator iter;

	if(_vwd)
	{
		_vwd->parseParameters(parameters);
	}

	if((iter=parameters.find(Parameters::kKpReactivatedWordsComparedToNewWords())) != parameters.end())
	{
		_reactivatedWordsComparedToNewWords = uStr2Bool((*iter).second.c_str());
	}

	if((iter=parameters.find(Parameters::kKpTfIdfLikelihoodUsed())) != parameters.end())
	{
		_tfIdfLikelihoodUsed = uStr2Bool((*iter).second.c_str());
	}

	if((iter=parameters.find(Parameters::kKpParallelized())) != parameters.end())
	{
		_parallelized = uStr2Bool((*iter).second.c_str());
	}

	if((iter=parameters.find(Parameters::kKpSensorStateOnly())) != parameters.end())
	{
		_sensorStateOnly = uStr2Bool((*iter).second.c_str());
	}

	if((iter=parameters.find(Parameters::kKpTfIdfNormalized())) != parameters.end())
	{
		_tfIdfNormalized = uStr2Bool((*iter).second.c_str());
	}

	if((iter=parameters.find(Parameters::kKpBadSignRatio())) != parameters.end())
	{
		_badSignRatio = std::atof((*iter).second.c_str());
	}

	//Keypoint detector
	DetectorStrategy detectorStrategy = kDetectorUndef;
	if((iter=parameters.find(Parameters::kKpDetectorStrategy())) != parameters.end())
	{
		detectorStrategy = (DetectorStrategy)std::atoi((*iter).second.c_str());
	}
	DetectorStrategy currentDetectorStrategy = this->detectorStrategy();
	if(!_keypointDetector || ( detectorStrategy!=kDetectorUndef && (detectorStrategy != currentDetectorStrategy) ) )
	{
		ULOGGER_DEBUG("new detector strategy %d", int(detectorStrategy));
		if(_keypointDetector)
		{
			delete _keypointDetector;
			_keypointDetector = 0;
		}
		switch(detectorStrategy)
		{
		case kDetectorStar:
			_keypointDetector = new StarDetector(parameters);
			break;
		case kDetectorSift:
			_keypointDetector = new SIFTDetector(parameters);
			break;
		case kDetectorSurf:
		default:
			_keypointDetector = new SURFDetector(parameters);
			break;
		}
	}
	else if(_keypointDetector)
	{
		_keypointDetector->parseParameters(parameters);
	}

	//Keypoint descriptor
	DescriptorStrategy descriptorStrategy = kDescriptorUndef;
	if((iter=parameters.find(Parameters::kKpDescriptorStrategy())) != parameters.end())
	{
		descriptorStrategy = (DescriptorStrategy)std::atoi((*iter).second.c_str());
	}
	if(!_keypointDescriptor || descriptorStrategy!=kDescriptorUndef)
	{
		ULOGGER_DEBUG("new descriptor strategy %d", int(descriptorStrategy));
		if(_keypointDescriptor)
		{
			delete _keypointDescriptor;
			_keypointDescriptor = 0;
		}
		switch(descriptorStrategy)
		{
		case kDescriptorColorSurf:
			// see decorator pattern...
			_keypointDescriptor = new ColorDescriptor(parameters, new SURFDescriptor(parameters));
			break;
		case kDescriptorLaplacianSurf:
			// see decorator pattern...
			_keypointDescriptor = new LaplacianDescriptor(parameters, new SURFDescriptor(parameters));
			break;
		case kDescriptorSift:
			_keypointDescriptor = new SIFTDescriptor(parameters);
			break;
		case kDescriptorHueSurf:
			// see decorator pattern...
			_keypointDescriptor = new HueDescriptor(parameters, new SURFDescriptor(parameters));
			break;
		case kDescriptorSurf:
		default:
			_keypointDescriptor = new SURFDescriptor(parameters);
			break;
		}
	}
	else if(_keypointDescriptor)
	{
		_keypointDescriptor->parseParameters(parameters);
	}

	Memory::parseParameters(parameters);
}

KeypointMemory::DetectorStrategy KeypointMemory::detectorStrategy() const
{
	DetectorStrategy strategy = kDetectorUndef;
	StarDetector * star = dynamic_cast<StarDetector*>(_keypointDetector);
	SURFDetector * surf = dynamic_cast<SURFDetector*>(_keypointDetector);
	if(star)
	{
		strategy = kDetectorStar;
	}
	else if(surf)
	{
		strategy = kDetectorSurf;
	}
	return strategy;
}

bool KeypointMemory::init(const std::string & dbDriverName, const std::string & dbUrl, bool dbOverwritten, const ParametersMap & parameters)
{
	ULOGGER_DEBUG("KeypointMemory::init()");
	// This will open a connection to the database,
	// this calls also clear()
	bool success = Memory::init(dbDriverName, dbUrl, dbOverwritten, parameters);

	// Now load the dictionary if we have a connection
	if(_vwd && _dbDriver && _dbDriver->isConnected())
	{
		UEventsManager::post(new RtabmapEventInit(std::string("Loading dictionary...")));
		_dbDriver->load(_vwd);
		ULOGGER_DEBUG("%d words loaded!", _vwd->getVisualWords().size());
		UEventsManager::post(new RtabmapEventInit(std::string("Loading dictionary, done! (") + uNumber2str(int(_vwd->getVisualWords().size())) + " loaded)"));
	}

	// Enable loaded signatures
	KeypointSignature * ss;
	const std::map<int, Signature *> & signatures = this->getSignatures();
	for(std::map<int, Signature *>::const_iterator i=signatures.begin(); i!=signatures.end(); ++i)
	{
		ss = dynamic_cast<KeypointSignature *>(this->_getSignature(i->first));
		if(ss)
		{
			ss->setEnabled(true);
		}
	}

	if(_vwd)
	{
		ULOGGER_DEBUG("Total word reference added = %d", _vwd->getTotalActiveReferences());
	}

	//if(this->isCommonSignatureUsed())
	//{
	//	Memory::updateCommonSignature(); // With words loaded, update the virtual place
	//}
	return success;
}

// TODO : Use only the parent method (in Memory)
// 1- Put "setEnabled" in the abstract Signature class, so this method is accessible from Memory
void KeypointMemory::addSignatureToStm(Signature * signature, const std::list<std::vector<float> > & actions)
{
	ULOGGER_DEBUG("");
	Memory::addSignatureToStm(signature, actions);

	UTimer timer;
	KeypointSignature * ss = dynamic_cast<KeypointSignature *>(signature);
	if(ss)
	{
		if(_vwd)
		{
			ULOGGER_DEBUG("%d words ref for the signature %d", ss->getWords().size(), ss->id());
		}
		if(ss->getWords().size())
		{
			ss->setEnabled(true);
		}
	}
	UDEBUG("time = %fs", timer.ticks());
}

void KeypointMemory::clear()
{
	ULOGGER_DEBUG("");

	// Save some SURF stats to the db (Must be before Memory::clear())
	if(_dbDriver)
	{
		int size = 0;
		if(_vwd)
		{
			size = _vwd->getVisualWords().size();
		}

		_dbDriver->addStatisticsAfterRunSurf(size);
	}

	Memory::clear();

	ULOGGER_DEBUG("");

	if(_dbDriver)
	{
		_dbDriver->kill();
		cleanUnusedWords();
		_dbDriver->emptyTrashes();

		//if(_wordRefsToChange.size())
		//{
		//	ULOGGER_DEBUG("Changing old word references (%d) in the database...", _wordRefsToChange.size());
		//	_dbDriver->beginTransaction();
			// Change all words reference for
			// all signatures with the old word to the active one...
			//_dbDriver->changeWordsRef(_wordRefsToChange);
			//remove old values
			_dbDriver->deleteUnreferencedWords();
		//	_dbDriver->commit();
		//}
		ULOGGER_DEBUG("");

		_dbDriver->start();
	}
	else
	{
		cleanUnusedWords();
	}

	_commonWords.clear();
}

void KeypointMemory::preUpdate()
{
	Memory::preUpdate();
	this->cleanUnusedWords();
	if(_vwd && !_parallelized)
	{
		//When parallelized, it is done in CreateSignature
		_vwd->update();
	}
}


// TODO Really useful?
/*void KeypointMemory::postUpdate()
{
	ULOGGER_DEBUG("");
	// Detect if the last signature is a bad one. If the signature has less than 15% of
	// the average words/signature.
	KeypointSignature * ss = dynamic_cast<KeypointSignature *>(this->_getLastSignature());

	float ratio = 0;
	if(ss)
	{
		ratio = float(uUniqueKeys(ss->getWords()).size()) / float(ss->getWords().size());
	}

	int nbCommonWords = 0;
	ULOGGER_DEBUG("_workingMem.size() = %d, _stMem.size()=%d", _workingMem.size(), _stMem.size());
	int treeSize= _workingMem.size() + _stMem.size();//Don't count the virtual place
	if(treeSize > 0)
	{
		nbCommonWords = _vwd->getTotalActiveReferences() / treeSize;
	}

	ULOGGER_DEBUG("ratio=%f, treeSize=%d, nbCommonWords=%d", ratio, treeSize, nbCommonWords);

	if(//(ratio < _badSignRatio) ||
	   (nbCommonWords && ss && ss->getWords().size() < _badSignRatio * nbCommonWords))
	{
		ULOGGER_WARN("id %d is a bad signature", ss->id());
		this->disableWordsRef(ss->id());
		ss->removeAllWords();
	}
}*/

// NON class method! only used in merge()
std::multimap<int, cv::KeyPoint> getMostDescriptiveWords(const std::multimap<int, cv::KeyPoint> & words, int max, const std::set<int> & ignoredIds)
{
	std::multimap<int, cv::KeyPoint> mostDescriptiveWords;
	if(max > 0)
	{
		std::multimap<float, std::pair<int, const cv::KeyPoint *> > responseWordMap;
		//get all descriptors from features not found in the two signatures
		for(std::multimap<int, cv::KeyPoint>::const_iterator itKey = words.begin(); itKey != words.end(); ++itKey)
		{
			if(ignoredIds.find(itKey->first) == ignoredIds.end())
			{
				responseWordMap.insert(std::pair<float, std::pair<int, const cv::KeyPoint *> >(itKey->second.response, std::pair<int, const cv::KeyPoint *>(itKey->first, &(itKey->second))));
			}
		}
		int endIndex = 0;
		if(responseWordMap.size() > (unsigned int)max)
		{
			endIndex = responseWordMap.size() - max;
		}

		//add them
		int i=0;
		std::multimap<float, std::pair<int, const cv::KeyPoint *> >::reverse_iterator iter = responseWordMap.rbegin();
		for(; iter != responseWordMap.rend(); ++iter, ++i)
		{
			if(i>=max)
			{
				break;
			}
			mostDescriptiveWords.insert(std::pair<int, cv::KeyPoint>(iter->second.first, *(iter->second.second)));
		}
	}
	return mostDescriptiveWords;
}

void KeypointMemory::merge(const Signature * from, Signature * to, MergingStrategy s)
{
	// The signatures must be KeypointSignature
	const KeypointSignature * sFrom = dynamic_cast<const KeypointSignature *>(from);
	KeypointSignature * sTo = dynamic_cast<KeypointSignature *>(to);
	UTimer timer;
	timer.start();
	if(sFrom && sTo)
	{
		if(s == kUseOnlyFromMerging)
		{
			this->disableWordsRef(sTo->id());
			sTo->setWords(sFrom->getWords());

			std::list<int> id;
			id.push_back(sTo->id());
			this->enableWordsRef(id);
			// Set old image to new merged signature
			sTo->setImage(sFrom->getImage());
		}
		else if(s == kUseOnlyDestMerging)
		{
			// do nothing... already "merged"
		}
	}
	else
	{
		ULOGGER_ERROR("Can't merge the signatures because there are not same type.");
	}
	ULOGGER_DEBUG("Merging time = %fs", timer.ticks());
}

std::map<int, float> KeypointMemory::computeLikelihood(const Signature * signature, const std::set<int> & signatureIds) const
{
	//return Memory::computeLikelihood(signature, signatureIds);

	// TODO cleanup , old way...
	if(_tfIdfLikelihoodUsed)
	{
		UTimer timer;
		timer.start();
		std::map<int, float> likelihood;
		std::map<int, float> calculatedWordsRatio;

		const KeypointSignature * newSurf = dynamic_cast<const KeypointSignature *>(signature);
		if(!newSurf)
		{
			ULOGGER_ERROR("The signature is not a KeypointSignature");
			return likelihood; // Must be a KeypointSignature *
		}

		if(signatureIds.size() == 0)
		{
			const std::map<int, int> & wm = this->getWorkingMem();
			for(std::map<int, int>::const_iterator iter = wm.begin(); iter!=wm.end(); ++iter)
			{
				likelihood.insert(likelihood.end(), std::pair<int, float>(iter->first, 0));
				if(_tfIdfNormalized)
				{
					const KeypointSignature * s = dynamic_cast<const KeypointSignature *>(this->getSignature(iter->first));
					float wordsCountRatio = -1; // default invalid
					if(s)
					{
						if(s->getWords().size() > newSurf->getWords().size())
						{
							wordsCountRatio = float(newSurf->getWords().size()) / float(s->getWords().size());
						}
						else if(newSurf->getWords().size())
						{
							wordsCountRatio = float(s->getWords().size()) / float(newSurf->getWords().size());
						}
						calculatedWordsRatio.insert(std::pair<int, float>(iter->first, wordsCountRatio));
					}
					else
					{
						calculatedWordsRatio.insert(std::pair<int, float>(iter->first, wordsCountRatio));
					}
				}
			}
		}
		else
		{
			for(std::set<int>::const_iterator i=signatureIds.begin(); i != signatureIds.end(); ++i)
			{
				likelihood.insert(likelihood.end(), std::pair<int, float>(*i, 0));
				if(_tfIdfNormalized)
				{
					const KeypointSignature * s = dynamic_cast<const KeypointSignature *>(this->getSignature(*i));
					float wordsCountRatio = -1; // default invalid
					if(s)
					{
						if(s->getWords().size() > newSurf->getWords().size())
						{
							wordsCountRatio = float(newSurf->getWords().size()) / float(s->getWords().size());
						}
						else if(newSurf->getWords().size())
						{
							wordsCountRatio = float(s->getWords().size()) / float(newSurf->getWords().size());
						}
						calculatedWordsRatio.insert(std::pair<int, float>(*i, wordsCountRatio));
					}
					else
					{
						calculatedWordsRatio.insert(std::pair<int, float>(*i, wordsCountRatio));
					}
				}
			}
		}

		const std::list<int> & wordIds = uUniqueKeys(newSurf->getWords());

		float nwi; // nwi is the number of a specific word referenced by a place
		float ni; // ni is the total of words referenced by a place
		float nw; // nw is the number of places referenced by a specific word
		float N; // N is the total number of places

		float logNnw;
		const VisualWord * vw;
		float normalizationRatio;

		N = likelihood.size();

		if(N)
		{
			ULOGGER_DEBUG("processing... ");
			// Pour chaque mot dans la signature SURF
			for(std::list<int>::const_iterator i=wordIds.begin(); i!=wordIds.end(); ++i)
			{
				// "Inverted index" - Pour chaque endroit contenu dans chaque mot
				vw = _vwd->getWord(*i);
				if(vw)
				{
					const std::map<int, int> & refs = vw->getReferences();
					nw = refs.size();
					if(nw)
					{
						logNnw = log10(N/nw);
						if(logNnw)
						{
							for(std::map<int, int>::const_iterator j=refs.begin(); j!=refs.end(); ++j)
							{
								std::map<int, float>::iterator iter = likelihood.find(j->first);
								if(iter != likelihood.end())
								{
									nwi = j->second;
									ni = this->getNi(j->first);
									if(ni != 0)
									{
										//ULOGGER_DEBUG("%d, %f %f %f %f", vw->id(), logNnw, nwi, ni, ( nwi  * logNnw ) / ni);
										if(_tfIdfNormalized)
										{
											normalizationRatio = uValue(calculatedWordsRatio, iter->first, -1.0f);
											if(normalizationRatio >= 0)
											{
												iter->second += (( nwi  * logNnw ) / ni) * normalizationRatio;
												UWARN("for id=%d, normalizationRatio=%f", iter->first, normalizationRatio);
											}
											else
											{
												UWARN("not found calculatedWordsRatio for id=%d", iter->first);
												iter->second += ( nwi  * logNnw ) / ni;
											}
										}
										else
										{
											iter->second += ( nwi  * logNnw ) / ni;
										}
									}
								}
							}
						}
					}
				}
			}
		}

		ULOGGER_DEBUG("compute likelihood... %f s", timer.ticks());
		return likelihood;
	}
	else
	{
		return Memory::computeLikelihood(signature, signatureIds);
	}

}

int KeypointMemory::getNi(int signatureId) const
{
	int ni = 0;
	const Signature * s = this->getSignature(signatureId);
	if(s) // Must be a SurfSignature
	{
		ni = ((KeypointSignature *)s)->getWords().size();
	}
	else
	{
		_dbDriver->getSurfNi(signatureId, ni);
	}
	return ni;
}

class PreUpdateThread : public UThreadNode
{
public:
	PreUpdateThread(VWDictionary * vwp) : _vwp(vwp) {}
	~PreUpdateThread() {}
private:
	void mainLoop() {
		if(_vwp)
		{
			_vwp->update();
		}
		this->kill();
	}
	VWDictionary * _vwp;
};

Signature * KeypointMemory::createSignature(int id, const SMState * smState, bool keepRawData)
{
	PreUpdateThread preUpdateThread(_vwd);

	UTimer timer;
	timer.start();
	std::list<cv::KeyPoint> keypoints;
	std::list<std::vector<float> > descriptors;
	const IplImage * image = 0;

	if(smState)
	{
		int treeSize= this->getWorkingMemSize() + this->getStMemSize();
		int nbCommonWords = 0;
		if(treeSize > 0)
		{
			nbCommonWords = _vwd->getTotalActiveReferences() / treeSize;
		}

		if(_parallelized)
		{
			preUpdateThread.start();
		}

		if(smState->getSensors().empty())
		{
			image = smState->getImage();
			if(image && _keypointDetector)
			{
				keypoints = _keypointDetector->generateKeypoints(image);
				ULOGGER_DEBUG("time keypoints = %fs", timer.ticks());
			}

			ULOGGER_DEBUG("ratio=%f, treeSize=%d, nbCommonWords=%d", _badSignRatio, treeSize, nbCommonWords);

			if(keypoints.size() && keypoints.size() >= _badSignRatio * nbCommonWords)
			{
				descriptors = _keypointDescriptor->generateDescriptors(image, keypoints);
			}
		}
		else
		{
			if(smState->getSensors().size() >= _badSignRatio * nbCommonWords)
			{
				descriptors = smState->getSensors();
				keypoints = smState->getKeypoints();
			}
			image = smState->getImage();
		}
	}

	if(_parallelized)
	{
		preUpdateThread.join(); // Wait the dictionary to be updated
	}

	std::list<int> wordIds;
	if(descriptors.size())
	{
		unsigned int descriptorSize = descriptors.begin()->size();
		if(_parallelized)
		{
			ULOGGER_DEBUG("time descriptor and memory update (%d of size=%d) = %fs", (int)descriptors.size(), (int)descriptorSize, timer.ticks());
		}
		else
		{
			ULOGGER_DEBUG("time descriptor (%d of size=%d) = %fs", (int)descriptors.size(), (int)descriptorSize, timer.ticks());
		}

		//append actuators
		if(!_sensorStateOnly && smState->getActuators().size())
		{
			const std::list<std::vector<float> > & actuators = smState->getActuators();

			unsigned int actuatorSize = actuators.begin()->size();
			if(actuatorSize > descriptorSize)
			{
				UERROR("Actuator's size (%d) is larger than descriptor size (%d)", actuatorSize, descriptorSize);
			}

			for(std::list<std::vector<float> >::const_iterator iter = actuators.begin(); iter!=actuators.end(); ++iter)
			{
				std::vector<float> descriptor(descriptorSize);
				// normalize actuator values
				std::vector<float> actuatorNormalized = uNormalize(*iter);
				for(unsigned int i=0; i<descriptorSize; ++i)
				{
					if(i<actuatorSize)
					{
						descriptor[i] = actuatorNormalized[i];
					}
					else
					{
						descriptor[i] = 0;
					}
				}
				descriptors.push_back(descriptor);
			}
			ULOGGER_DEBUG("time setup actuators (%d of length %d) like descriptors %fs", (int)actuators.size(), (int)actuatorSize, timer.ticks());
		}

		wordIds = _vwd->addNewWords(descriptors, descriptorSize, id);
		ULOGGER_DEBUG("time addNewWords %fs", timer.ticks());
	}
	else
	{
		ULOGGER_WARN("id %d is a bad signature", id);
	}

	std::multimap<int, cv::KeyPoint> words;
	if(wordIds.size() > 0)
	{
		std::list<cv::KeyPoint>::iterator kpIter = keypoints.begin();
		for(std::list<int>::iterator iter=wordIds.begin(); iter!=wordIds.end(); ++iter)
		{
			if(kpIter != keypoints.end())
			{
				words.insert(std::pair<int, cv::KeyPoint >(*iter, *kpIter));
				++kpIter;
			}
			else
			{
				words.insert(std::pair<int, cv::KeyPoint >(*iter, cv::KeyPoint()));
			}
		}
	}

	KeypointSignature * ks = new KeypointSignature(words, id, image, keepRawData);
	ULOGGER_DEBUG("time new signature (id=%d) %fs", id, timer.ticks());
	if(words.size())
	{
		ks->setEnabled(true); // All references are already activated in the dictionary at this point (see _vwd->addNewWords())
	}
	return ks;
}

Signature * KeypointMemory::getSignatureLtMem(int id)
{
	Signature * s = Memory::getSignatureLtMem(id);
	if(s)
	{
		std::list<int> lid;
		lid.push_back(id);
		this->enableWordsRef(lid);
	}
	return s;
}

void KeypointMemory::disableWordsRef(int signatureId)
{
	ULOGGER_DEBUG("id=%d", signatureId);

	KeypointSignature * ss = dynamic_cast<KeypointSignature *>(this->_getSignature(signatureId));
	if(ss && ss->isEnabled())
	{
		const std::multimap<int, cv::KeyPoint> & words = ss->getWords();
		const std::list<int> & keys = uUniqueKeys(words);
		int count = _vwd->getTotalActiveReferences();
		// First remove all references
		for(std::list<int>::const_iterator i=keys.begin(); i!=keys.end(); ++i)
		{
			_vwd->removeAllWordRef(*i, signatureId);
		}

		count -= _vwd->getTotalActiveReferences();
		ss->setEnabled(false);
		ULOGGER_DEBUG("%d words total ref removed from signature %d...", count, ss->id());
	}
}

void KeypointMemory::cleanUnusedWords()
{
	ULOGGER_DEBUG("");
	if(_vwd->isIncremental())
	{
		std::vector<VisualWord*> removedWords = _vwd->getUnusedWords();

		if(removedWords.size())
		{
			// remove them from the dictionary
			_vwd->removeWords(removedWords);

			for(unsigned int i=0; i<removedWords.size(); ++i)
			{
				if(_dbDriver)
				{
					_dbDriver->asyncSave(removedWords[i]);
				}
				else
				{
					delete removedWords[i];
				}
			}
		}
		ULOGGER_DEBUG("%d words removed...", removedWords.size());
	}
}

void KeypointMemory::enableWordsRef(const std::list<int> & signatureIds)
{
	ULOGGER_DEBUG("size=%d", signatureIds.size());
	UTimer timer;
	timer.start();

	std::map<int, int> refsToChange; //<oldWordId, activeWordId>

	std::set<int> oldWordIds;
	std::list<KeypointSignature *> surfSigns;
	for(std::list<int>::const_iterator i=signatureIds.begin(); i!=signatureIds.end(); ++i)
	{
		KeypointSignature * ss = dynamic_cast<KeypointSignature *>(this->_getSignature(*i));
		if(ss && !ss->isEnabled())
		{
			surfSigns.push_back(ss);
			std::list<int> uniqueKeys = uUniqueKeys(ss->getWords());

			//Find words in the signature which they are not in the current dictionary
			for(std::list<int>::const_iterator k=uniqueKeys.begin(); k!=uniqueKeys.end(); ++k)
			{
				if(_vwd->getWord(*k) == 0)
				{
					//std::map<int,int>::iterator iter = _wordRefsToChange.find(*k);
					//if(iter != _wordRefsToChange.end())
					//{
					//	ss->changeWordsRef(iter->first, iter->second);
					//	uniqueKeys.push_back(iter->second);
					//}
					//else
					if(oldWordIds.find(*k) == oldWordIds.end())
					{
						oldWordIds.insert(oldWordIds.end(), *k);
					}
					else
					{
						//UDEBUG("*k=%d", *k);
					}
				}
			}
		}
	}

	ULOGGER_DEBUG("oldWordIds.size()=%d, getOldIds time=%fs", oldWordIds.size(), timer.ticks());

	// the words were deleted, so try to math it with an active word
	std::list<VisualWord *> vws;
	if(oldWordIds.size() && _dbDriver)
	{
		_dbDriver->loadWords(std::list<int>(oldWordIds.begin(), oldWordIds.end()), vws); // get the descriptors
	}
	ULOGGER_DEBUG("loading words(%d) time=%fs", oldWordIds.size(), timer.ticks());


	if(vws.size())
	{
		//Search in the dictionary
		std::vector<int> vwActiveIds = _vwd->findNN(vws, _reactivatedWordsComparedToNewWords);
		ULOGGER_DEBUG("find active ids (number=%d) time=%fs", vws.size(), timer.ticks());
		int i=0;
		for(std::list<VisualWord *>::iterator iterVws=vws.begin(); iterVws!=vws.end(); ++iterVws)
		{
			if(vwActiveIds[i] > 0)
			{
				//ULOGGER_DEBUG("Match found %d with %d", (*iterVws)->id(), vwActiveIds[i]);
				refsToChange.insert(refsToChange.end(), std::pair<int, int>((*iterVws)->id(), vwActiveIds[i]));
				if((*iterVws)->isSaved() || !_dbDriver)
				{
					delete (*iterVws);
				}
				else
				{
					_dbDriver->asyncSave(*iterVws);
				}
			}
			else
			{
				//add to dictionary
				_vwd->addWord(*iterVws);
			}
			++i;
		}
		ULOGGER_DEBUG("Added %d to dictionary, time=%fs", vws.size()-refsToChange.size(), timer.ticks());

		//update the global references map and update the signatures reactivated
		for(std::map<int, int>::const_iterator iter=refsToChange.begin(); iter != refsToChange.end(); ++iter)
		{
			//uInsert(_wordRefsToChange, (const std::pair<int, int>)*iter); // This will be used to change references in the database
			for(std::list<KeypointSignature *>::iterator j=surfSigns.begin(); j!=surfSigns.end(); ++j)
			{
				(*j)->changeWordsRef(iter->first, iter->second);
			}
		}
		ULOGGER_DEBUG("changing ref, total=%d, time=%fs", refsToChange.size(), timer.ticks());
	}

	int count = _vwd->getTotalActiveReferences();

	// Reactivate references and signatures
	for(std::list<KeypointSignature *>::iterator j=surfSigns.begin(); j!=surfSigns.end(); ++j)
	{
		const std::list<int> & keys = uKeys((*j)->getWords());
		// Add all references
		for(std::list<int>::const_iterator i=keys.begin(); i!=keys.end(); ++i)
		{
			_vwd->addWordRef(*i, (*j)->id());
		}
		if(keys.size())
		{
			(*j)->setEnabled(true);
		}
	}

	count = _vwd->getTotalActiveReferences() - count;
	ULOGGER_DEBUG("%d words total ref added from %d signatures, time=%fs...", count, surfSigns.size(), timer.ticks());
}

int KeypointMemory::forget(const std::list<int> & ignoredIds)
{
	ULOGGER_DEBUG("");
	int signaturesRemoved = 0;
	if(_vwd->isIncremental())
	{
		int newWords = 0;
		int wordsRemoved = 0;

		// Get how many new words added for the last run...
		newWords = _vwd->getNotIndexedWordsCount();

		// So we need to remove at least "newWords" words from the
		// dictionary to respect the limit.
		while(wordsRemoved < newWords)
		{
			KeypointSignature *  s = dynamic_cast<KeypointSignature *>(this->getRemovableSignature(ignoredIds));
			if(s)
			{
				++signaturesRemoved;
				this->moveToTrash(s);
				wordsRemoved = _vwd->getUnusedWordsSize();
			}
			else
			{
				break;
			}
		}
		ULOGGER_DEBUG("newWords=%d, wordsRemoved=%d", newWords, wordsRemoved);
	}
	else
	{
		signaturesRemoved = Memory::forget(ignoredIds) ;
	}
	ULOGGER_DEBUG("signatures removed = %d", signaturesRemoved);
	return signaturesRemoved;
}

int KeypointMemory::reactivateSignatures(const std::list<int> & ids, unsigned int maxLoaded, unsigned int maxTouched)
{
	// get the signatures, if not in the working memory, they
	// will be loaded from the database in an more efficient way
	// than how it is done in the Memory

	ULOGGER_DEBUG("");
	UTimer timer;
	std::list<int> idsToLoad;
	unsigned int touched = 0;
	std::map<int, int>::iterator wmIter;
	for(std::list<int>::const_iterator i=ids.begin(); i!=ids.end(); ++i)
	{
		if(!this->getSignature(*i) && !uContains(idsToLoad, *i))
		{
			if(!maxLoaded || idsToLoad.size() < maxLoaded)
			{
				//When loaded from the long-term memory, the signature
				// is automatically added on top of the working memory
				idsToLoad.push_back(*i);
			}
		}
		else if(touched < maxTouched)
		{
			this->touch(*i);
		}
		++touched;
	}

	ULOGGER_DEBUG("idsToLoad = %d", idsToLoad.size());

	std::list<Signature *> reactivatedSigns;
	if(_dbDriver)
	{
		_dbDriver->loadKeypointSignatures(idsToLoad, reactivatedSigns, true);
	}
	std::list<int> idsLoaded;
	for(std::list<Signature *>::iterator i=reactivatedSigns.begin(); i!=reactivatedSigns.end(); ++i)
	{
		idsLoaded.push_back((*i)->id());
		//append to working memory
		this->addSignatureToWm(*i);
	}
	this->enableWordsRef(idsLoaded);
	ULOGGER_DEBUG("time = %fs", timer.ticks());
	return reactivatedSigns.size();
}

void KeypointMemory::moveToTrash(Signature * s)
{
	if(s)
	{
		this->disableWordsRef(s->id());
	}
	Memory::moveToTrash(s);
}

void KeypointMemory::dumpMemory(std::string directory) const
{
	this->dumpDictionary((directory+"DumpMemoryWordRef.txt").c_str(), (directory+"DumpMemoryWordDesc.txt").c_str());
	Memory::dumpMemory(directory);
}

void KeypointMemory::dumpDictionary(const char * fileNameRef, const char * fileNameDesc) const
{
	if(_vwd)
	{
		_vwd->exportDictionary(fileNameRef, fileNameDesc);
	}
}

void KeypointMemory::dumpSignatures(const char * fileNameSign) const
{
	FILE* foutSign = 0;
#ifdef _MSC_VER
	fopen_s(&foutSign, fileNameSign, "w");
#else
	foutSign = fopen(fileNameSign, "w");
#endif

	if(foutSign)
	{
		fprintf(foutSign, "SignatureID WordsID...\n");
		const std::map<int, Signature *> & signatures = this->getSignatures();
		for(std::map<int, Signature *>::const_iterator iter=signatures.begin(); iter!=signatures.end(); ++iter)
		{
			fprintf(foutSign, "%d ", iter->first);
			const KeypointSignature * ss = dynamic_cast<const KeypointSignature *>(iter->second);
			if(ss)
			{
				const std::multimap<int, cv::KeyPoint> & ref = ss->getWords();
				for(std::multimap<int, cv::KeyPoint>::const_iterator jter=ref.begin(); jter!=ref.end(); ++jter)
				{
					fprintf(foutSign, "%d ", (*jter).first);
				}
			}
			fprintf(foutSign, "\n");
		}
		fclose(foutSign);
	}
}

} // namespace rtabmap
