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

#include <rtabmap/utilite/UEventsManager.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UProcessInfo.h>
#include <rtabmap/utilite/UMath.h>

#include "rtabmap/core/Memory.h"
#include "rtabmap/core/Signature.h"
#include "rtabmap/core/Parameters.h"
#include "rtabmap/core/RtabmapEvent.h"
#include "rtabmap/core/VWDictionary.h"
#include "VisualWord.h"
#include "rtabmap/core/Features2d.h"
#include "rtabmap/core/util3d.h"
#include "DBDriverSqlite3.h"
#include "rtabmap/core/util3d.h"
#include "rtabmap/core/Statistics.h"

#include <pcl/io/pcd_io.h>

namespace rtabmap {

const int Memory::kIdStart = 0;
const int Memory::kIdVirtual = -1;
const int Memory::kIdInvalid = 0;

Memory::Memory(const ParametersMap & parameters) :
	_dbDriver(0),
	_similarityThreshold(Parameters::defaultMemRehearsalSimilarity()),
	_rawDataKept(Parameters::defaultMemImageKept()),
	_keepRehearsedNodesInDb(Parameters::defaultMemRehearsedNodesKept()),
	_incrementalMemory(Parameters::defaultMemIncrementalMemory()),
	_maxStMemSize(Parameters::defaultMemSTMSize()),
	_recentWmRatio(Parameters::defaultMemRecentWmRatio()),
	_idUpdatedToNewOneRehearsal(Parameters::defaultMemRehearsalIdUpdatedToNewOne()),
	_generateIds(Parameters::defaultMemGenerateIds()),
	_badSignaturesIgnored(Parameters::defaultMemBadSignaturesIgnored()),
	_idCount(kIdStart),
	_idMapCount(kIdStart),
	_lastSignature(0),
	_lastGlobalLoopClosureParentId(0),
	_lastGlobalLoopClosureChildId(0),
	_memoryChanged(false),
	_signaturesAdded(0),

	_keypointDetector(0),
	_keypointDescriptor(0),
	_badSignRatio(Parameters::defaultKpBadSignRatio()),
	_tfIdfLikelihoodUsed(Parameters::defaultKpTfIdfLikelihoodUsed()),
	_parallelized(Parameters::defaultKpParallelized()),
	_wordsMaxDepth(Parameters::defaultKpMaxDepth()),
	_wordsPerImageTarget(Parameters::defaultKpWordsPerImage()),
	_roiRatios(std::vector<float>(4, 0.0f)),

	_bowMinInliers(Parameters::defaultLccBowMinInliers()),
	_bowInlierDistance(Parameters::defaultLccBowInlierDistance()),
	_bowIterations(Parameters::defaultLccBowIterations()),
	_bowMaxDepth(Parameters::defaultLccBowMaxDepth()),

	_icpDecimation(Parameters::defaultLccIcp3Decimation()),
	_icpMaxDepth(Parameters::defaultLccIcp3MaxDepth()),
	_icpVoxelSize(Parameters::defaultLccIcp3VoxelSize()),
	_icpSamples(Parameters::defaultLccIcp3Samples()),
	_icpMaxCorrespondenceDistance(Parameters::defaultLccIcp3MaxCorrespondenceDistance()),
	_icpMaxIterations(Parameters::defaultLccIcp3Iterations()),
	_icpMaxFitness(Parameters::defaultLccIcp3MaxFitness()),

	_icp2MaxCorrespondenceDistance(Parameters::defaultLccIcp2MaxCorrespondenceDistance()),
	_icp2MaxIterations(Parameters::defaultLccIcp2Iterations()),
	_icp2MaxFitness(Parameters::defaultLccIcp2MaxFitness()),
	_icp2CorrespondenceRatio(Parameters::defaultLccIcp2CorrespondenceRatio()),
	_icp2VoxelSize(Parameters::defaultLccIcp2VoxelSize())
{
	_vwd = new VWDictionary(parameters);
	this->parseParameters(parameters);
}

bool Memory::init(const std::string & dbUrl, bool dbOverwritten, const ParametersMap & parameters, bool postInitEvents)
{
	if(postInitEvents) UEventsManager::post(new RtabmapEventInit(RtabmapEventInit::kInitializing));

	UDEBUG("");
	this->parseParameters(parameters);

	if(postInitEvents) UEventsManager::post(new RtabmapEventInit("Clearing memory..."));
	DBDriver * tmpDriver = 0;
	if(!_memoryChanged)
	{
		if(_dbDriver)
		{
			tmpDriver = _dbDriver;
			_dbDriver = 0; // HACK for the clear() below to think that there is no db
		}
	}
	this->clear();
	if(postInitEvents) UEventsManager::post(new RtabmapEventInit("Clearing memory, done!"));

	if(tmpDriver)
	{
		_dbDriver = tmpDriver;
	}

	if(_dbDriver)
	{
		if(postInitEvents) UEventsManager::post(new RtabmapEventInit("Closing database connection..."));
		_dbDriver->closeConnection();
		if(postInitEvents) UEventsManager::post(new RtabmapEventInit("Closing database connection, done!"));
	}

	if(_dbDriver == 0 && !dbUrl.empty())
	{
		_dbDriver = new DBDriverSqlite3(parameters);
	}

	bool success = true;
	if(_dbDriver)
	{
		success = false;
		if(postInitEvents) UEventsManager::post(new RtabmapEventInit(std::string("Connecting to database ") + dbUrl + "..."));
		if(_dbDriver->openConnection(dbUrl, dbOverwritten))
		{
			success = true;
			if(postInitEvents) UEventsManager::post(new RtabmapEventInit(std::string("Connecting to database ") + dbUrl + ", done!"));

			// Load the last working memory...
			if(postInitEvents) UEventsManager::post(new RtabmapEventInit(std::string("Loading last signatures...")));
			std::list<Signature*> dbSignatures;
			_dbDriver->loadLastNodes(dbSignatures);
			for(std::list<Signature*>::reverse_iterator iter=dbSignatures.rbegin(); iter!=dbSignatures.rend(); ++iter)
			{
				// ignore bad signatures
				if(!((*iter)->isBadSignature() && _badSignaturesIgnored))
				{
					_signatures.insert(std::pair<int, Signature *>((*iter)->id(), *iter));
					if((int)_stMem.size() <= _maxStMemSize)
					{
						_stMem.insert((*iter)->id());
					}
					else
					{
						_workingMem.insert((*iter)->id());
					}
				}
				else
				{
					delete *iter;
				}
			}
			if(postInitEvents) UEventsManager::post(new RtabmapEventInit(std::string("Loading last signatures, done! (") + uNumber2Str(int(_workingMem.size() + _stMem.size())) + " loaded)"));

			// Assign the last signature
			if(_stMem.size()>0)
			{
				_lastSignature = uValue(_signatures, *_stMem.rbegin(), (Signature*)0);
			}

			// Last id
			_dbDriver->getLastNodeId(_idCount);
			_idMapCount = _lastSignature?_lastSignature->mapId():kIdStart;
		}
		else
		{
			if(postInitEvents) UEventsManager::post(new RtabmapEventInit(RtabmapEventInit::kError, std::string("Connecting to database ") + dbUrl + ", path is invalid!"));
		}
	}
	else
	{
		_idCount = kIdStart;
		_idMapCount = kIdStart;
	}

	_workingMem.insert(kIdVirtual);

	UDEBUG("ids start with %d", _idCount+1);
	UDEBUG("map ids start with %d", _idMapCount);


	// Now load the dictionary if we have a connection
	if(_dbDriver && _dbDriver->isConnected())
	{
		if(postInitEvents) UEventsManager::post(new RtabmapEventInit("Loading dictionary..."));
		_dbDriver->load(_vwd);
		UDEBUG("%d words loaded!", _vwd->getUnusedWordsSize());
		_vwd->update();
		if(postInitEvents) UEventsManager::post(new RtabmapEventInit(uFormat("Loading dictionary, done! (%d words)", (int)_vwd->getUnusedWordsSize())));
	}

	if(postInitEvents) UEventsManager::post(new RtabmapEventInit(std::string("Adding word references...")));
	// Enable loaded signatures
	Signature * ss;
	const std::map<int, Signature *> & signatures = this->getSignatures();
	for(std::map<int, Signature *>::const_iterator i=signatures.begin(); i!=signatures.end(); ++i)
	{
		ss = this->_getSignature(i->first);
		if(ss)
		{
			const std::multimap<int, cv::KeyPoint> & words = ss->getWords();
			if(words.size())
			{
				UDEBUG("node=%d, word references=%d", ss->id(), words.size());
				for(std::multimap<int, cv::KeyPoint>::const_iterator iter = words.begin(); iter!=words.end(); ++iter)
				{
					_vwd->addWordRef(iter->first, i->first);
				}
				ss->setEnabled(true);
			}
		}
	}

	if(postInitEvents) UEventsManager::post(new RtabmapEventInit(uFormat("Adding word references, done! (%d)", _vwd->getTotalActiveReferences())));

	if(_vwd->getUnusedWordsSize())
	{
		UWARN("_vwd->getUnusedWordsSize() must be empty... size=%d", _vwd->getUnusedWordsSize());
	}
	UDEBUG("Total word references added = %d", _vwd->getTotalActiveReferences());

	if(postInitEvents) UEventsManager::post(new RtabmapEventInit(RtabmapEventInit::kInitialized));
	return success;
}

Memory::~Memory()
{
	UDEBUG("");
	if(!_memoryChanged)
	{
		UDEBUG("");
		if(_dbDriver)
		{
			_dbDriver->closeConnection();
			delete _dbDriver;
			_dbDriver = 0;
		}
		this->clear();
	}
	else
	{
		UDEBUG("");
		this->clear();
		if(_dbDriver)
		{
			_dbDriver->emptyTrashes();
			_dbDriver->closeConnection();
			delete _dbDriver;
			_dbDriver = 0;
		}
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

void Memory::parseParameters(const ParametersMap & parameters)
{
	UDEBUG("");
	ParametersMap::const_iterator iter;

	Parameters::parse(parameters, Parameters::kMemImageKept(), _rawDataKept);
	Parameters::parse(parameters, Parameters::kMemRehearsedNodesKept(), _keepRehearsedNodesInDb);
	Parameters::parse(parameters, Parameters::kMemRehearsalIdUpdatedToNewOne(), _idUpdatedToNewOneRehearsal);
	Parameters::parse(parameters, Parameters::kMemGenerateIds(), _generateIds);
	Parameters::parse(parameters, Parameters::kMemBadSignaturesIgnored(), _badSignaturesIgnored);
	Parameters::parse(parameters, Parameters::kMemRehearsalSimilarity(), _similarityThreshold);
	Parameters::parse(parameters, Parameters::kMemRecentWmRatio(), _recentWmRatio);
	Parameters::parse(parameters, Parameters::kMemSTMSize(), _maxStMemSize);

	UASSERT_MSG(_maxStMemSize > 0, uFormat("value=%d", _maxStMemSize).c_str());
	UASSERT_MSG(_similarityThreshold >= 0.0f && _similarityThreshold <= 1.0f, uFormat("value=%f", _similarityThreshold).c_str());
	UASSERT_MSG(_recentWmRatio >= 0.0f && _recentWmRatio <= 1.0f, uFormat("value=%f", _recentWmRatio).c_str());

	// SLAM mode vs Localization mode
	iter = parameters.find(Parameters::kMemIncrementalMemory());
	if(iter != parameters.end())
	{
		bool value = uStr2Bool(iter->second.c_str());
		if(value == false && _incrementalMemory)
		{
			// From SLAM to localization, change map id
			this->incrementMapId();
		}
		_incrementalMemory = value;
	}

	if(_dbDriver)
	{
		_dbDriver->parseParameters(parameters);
	}

	Parameters::parse(parameters, Parameters::kLccBowMinInliers(), _bowMinInliers);
	Parameters::parse(parameters, Parameters::kLccBowInlierDistance(), _bowInlierDistance);
	Parameters::parse(parameters, Parameters::kLccBowIterations(), _bowIterations);
	Parameters::parse(parameters, Parameters::kLccBowMaxDepth(), _bowMaxDepth);
	Parameters::parse(parameters, Parameters::kLccIcp3Decimation(), _icpDecimation);
	Parameters::parse(parameters, Parameters::kLccIcp3MaxDepth(), _icpMaxDepth);
	Parameters::parse(parameters, Parameters::kLccIcp3VoxelSize(), _icpVoxelSize);
	Parameters::parse(parameters, Parameters::kLccIcp3Samples(), _icpSamples);
	Parameters::parse(parameters, Parameters::kLccIcp3MaxCorrespondenceDistance(), _icpMaxCorrespondenceDistance);
	Parameters::parse(parameters, Parameters::kLccIcp3Iterations(), _icpMaxIterations);
	Parameters::parse(parameters, Parameters::kLccIcp3MaxFitness(), _icpMaxFitness);
	Parameters::parse(parameters, Parameters::kLccIcp2MaxCorrespondenceDistance(), _icp2MaxCorrespondenceDistance);
	Parameters::parse(parameters, Parameters::kLccIcp2Iterations(), _icp2MaxIterations);
	Parameters::parse(parameters, Parameters::kLccIcp2MaxFitness(), _icp2MaxFitness);
	Parameters::parse(parameters, Parameters::kLccIcp2CorrespondenceRatio(), _icp2CorrespondenceRatio);
	Parameters::parse(parameters, Parameters::kLccIcp2VoxelSize(), _icp2VoxelSize);

	UASSERT_MSG(_bowMinInliers >= 1, uFormat("value=%d", _bowMinInliers).c_str());
	UASSERT_MSG(_bowInlierDistance > 0.0f, uFormat("value=%f", _bowInlierDistance).c_str());
	UASSERT_MSG(_bowIterations > 0, uFormat("value=%d", _bowIterations).c_str());
	UASSERT_MSG(_bowMaxDepth >= 0.0f, uFormat("value=%f", _bowMaxDepth).c_str());
	UASSERT_MSG(_icpDecimation > 0, uFormat("value=%d", _icpDecimation).c_str());
	UASSERT_MSG(_icpMaxDepth >= 0.0f, uFormat("value=%f", _icpMaxDepth).c_str());
	UASSERT_MSG(_icpVoxelSize >= 0, uFormat("value=%d", _icpVoxelSize).c_str());
	UASSERT_MSG(_icpSamples >= 0, uFormat("value=%d", _icpSamples).c_str());
	UASSERT_MSG(_icpMaxCorrespondenceDistance > 0.0f, uFormat("value=%f", _icpMaxCorrespondenceDistance).c_str());
	UASSERT_MSG(_icpMaxIterations > 0, uFormat("value=%d", _icpMaxIterations).c_str());
	UASSERT_MSG(_icpMaxFitness > 0.0f, uFormat("value=%f", _icpMaxFitness).c_str());
	UASSERT_MSG(_icp2MaxCorrespondenceDistance > 0.0f, uFormat("value=%f", _icp2MaxCorrespondenceDistance).c_str());
	UASSERT_MSG(_icp2MaxIterations > 0, uFormat("value=%d", _icp2MaxIterations).c_str());
	UASSERT_MSG(_icp2MaxFitness > 0.0f, uFormat("value=%f", _icp2MaxFitness).c_str());
	UASSERT_MSG(_icp2CorrespondenceRatio >=0.0f && _icp2CorrespondenceRatio <=1.0f, uFormat("value=%f", _icp2MaxFitness).c_str());
	UASSERT_MSG(_icp2VoxelSize >= 0, uFormat("value=%d", _icp2VoxelSize).c_str());

	// Keypoint stuff
	if(_vwd)
	{
		_vwd->parseParameters(parameters);
	}

	Parameters::parse(parameters, Parameters::kKpTfIdfLikelihoodUsed(), _tfIdfLikelihoodUsed);
	Parameters::parse(parameters, Parameters::kKpParallelized(), _parallelized);
	Parameters::parse(parameters, Parameters::kKpBadSignRatio(), _badSignRatio);
	Parameters::parse(parameters, Parameters::kKpMaxDepth(), _wordsMaxDepth);
	Parameters::parse(parameters, Parameters::kKpWordsPerImage(), _wordsPerImageTarget);

	if((iter=parameters.find(Parameters::kKpRoiRatios())) != parameters.end())
	{
		this->setRoi((*iter).second);
	}

	//Keypoint detector
	KeypointDetector::DetectorType detectorStrategy = KeypointDetector::kDetectorUndef;
	if((iter=parameters.find(Parameters::kKpDetectorStrategy())) != parameters.end())
	{
		detectorStrategy = (KeypointDetector::DetectorType)std::atoi((*iter).second.c_str());
	}
	if(!_keypointDetector ||  detectorStrategy!=KeypointDetector::kDetectorUndef)
	{
		UDEBUG("new detector strategy %d", int(detectorStrategy));
		if(_keypointDetector)
		{
			delete _keypointDetector;
			_keypointDetector = 0;
		}
		if(_keypointDescriptor)
		{
			delete _keypointDescriptor;
			_keypointDescriptor = 0;
		}
		switch(detectorStrategy)
		{
		case KeypointDetector::kDetectorSift:
			_keypointDetector = new SIFTDetector(parameters);
			_keypointDescriptor = new SIFTDescriptor(parameters);
			break;
		case KeypointDetector::kDetectorSurf:
		default:
			_keypointDetector = new SURFDetector(parameters);
			_keypointDescriptor = new SURFDescriptor(parameters);
			break;
		}
	}
	else
	{
		if(_keypointDetector)
		{
			_keypointDetector->parseParameters(parameters);
		}
		if(_keypointDescriptor)
		{
			_keypointDescriptor->parseParameters(parameters);
		}
	}
}

void Memory::preUpdate()
{
	_signaturesAdded = 0;
	this->cleanUnusedWords();
	if(_vwd && !_parallelized)
	{
		//When parallelized, it is done in CreateSignature
		_vwd->update();
	}
}

bool Memory::update(const Image & image, Statistics * stats)
{
	UDEBUG("");
	UTimer timer;
	UTimer totalTimer;
	timer.start();
	float t;

	//============================================================
	// Pre update...
	//============================================================
	UDEBUG("pre-updating...");
	this->preUpdate();
	t=timer.ticks()*1000;
	if(stats) stats->addStatistic(Statistics::kTimingMemPre_update(), t);
	UDEBUG("time preUpdate=%f ms", t);

	//============================================================
	// Create a signature with the image received.
	//============================================================
	Signature * signature = this->createSignature(image, this->isRawDataKept());
	if (signature == 0)
	{
		UERROR("Failed to create a signature...");
		return false;
	}

	t=timer.ticks()*1000;
	if(stats) stats->addStatistic(Statistics::kTimingMemSignature_creation(), t);
	UDEBUG("time creating signature=%f ms", t);

	// It will be added to the short-term memory, no need to delete it...
	this->addSignatureToStm(signature);

	_lastSignature = signature;

	//============================================================
	// Rehearsal step...
	// Compare with the X last signatures. If different, add this
	// signature like a parent to the memory tree, otherwise add
	// it as a child to the similar signature.
	//============================================================
	if(_incrementalMemory)
	{
		this->rehearsal(signature, stats);
		t=timer.ticks()*1000;
		if(stats) stats->addStatistic(Statistics::kTimingMemRehearsal(), t);
		UDEBUG("time rehearsal=%f ms", t);
	}
	else
	{
		if(_workingMem.size() <= 1)
		{
			UWARN("The working memory is empty and the memory is not "
				  "incremental (Mem/IncrementalMemory=False), no loop closure "
				  "can be detected! Please set Mem/IncrementalMemory=true to increase "
				  "the memory with new images or decrease the STM size (which is %d "
				  "including the new one added).", (int)_stMem.size());
		}
	}

	//============================================================
	// Transfer the oldest signature of the short-term memory to the working memory
	//============================================================
	while(_stMem.size() && (int)_stMem.size() > _maxStMemSize)
	{
		UDEBUG("Inserting node %d from STM in WM...", *_stMem.begin());
		_workingMem.insert(_workingMem.end(), *_stMem.begin());
		_stMem.erase(*_stMem.begin());
		++_signaturesAdded;
	}

	if(!_memoryChanged && _incrementalMemory)
	{
		_memoryChanged = true;
	}

	UDEBUG("totalTimer = %fs", totalTimer.ticks());

	if(stats) stats->addStatistic(Statistics::kLoopLast_loop_closure_parent(), _lastGlobalLoopClosureParentId);
	if(stats) stats->addStatistic(Statistics::kLoopLast_loop_closure_child(), _lastGlobalLoopClosureChildId);

	return true;
}



void Memory::setRoi(const std::string & roi)
{
	std::list<std::string> strValues = uSplit(roi, ' ');
	if(strValues.size() != 4)
	{
		ULOGGER_ERROR("The number of values must be 4 (roi=\"%s\")", roi.c_str());
	}
	else
	{
		std::vector<float> tmpValues(4);
		unsigned int i=0;
		for(std::list<std::string>::iterator iter = strValues.begin(); iter!=strValues.end(); ++iter)
		{
			tmpValues[i] = std::atof((*iter).c_str());
			++i;
		}

		if(tmpValues[0] >= 0 && tmpValues[0] < 1 && tmpValues[0] < 1.0f-tmpValues[1] &&
			tmpValues[1] >= 0 && tmpValues[1] < 1 && tmpValues[1] < 1.0f-tmpValues[0] &&
			tmpValues[2] >= 0 && tmpValues[2] < 1 && tmpValues[2] < 1.0f-tmpValues[3] &&
			tmpValues[3] >= 0 && tmpValues[3] < 1 && tmpValues[3] < 1.0f-tmpValues[2])
		{
			_roiRatios = tmpValues;
		}
		else
		{
			ULOGGER_ERROR("The roi ratios are not valid (roi=\"%s\")", roi.c_str());
		}
	}
}

void Memory::addSignatureToStm(Signature * signature)
{
	UTimer timer;
	// add signature on top of the short-term memory
	if(signature)
	{
		UDEBUG("adding %d", signature->id());
		// Update neighbors
		if(_stMem.size())
		{
			if(_signatures.at(*_stMem.rbegin())->mapId() == signature->mapId())
			{
				Transform motionEstimate;
				if(!signature->getPose().isNull() &&
				   !_signatures.at(*_stMem.rbegin())->getPose().isNull())
				{
					motionEstimate = _signatures.at(*_stMem.rbegin())->getPose().inverse() * signature->getPose();
					_signatures.at(*_stMem.rbegin())->addNeighbor(signature->id(), motionEstimate);
				}
				else
				{
					_signatures.at(*_stMem.rbegin())->addNeighbor(signature->id());
				}
				signature->addNeighbor(*_stMem.rbegin(), motionEstimate.isNull()?Transform():motionEstimate.inverse());
				UDEBUG("Min STM id = %d", *_stMem.begin());
			}
			else
			{
				UDEBUG("Ignoring neighbor link between %d and %d because they are not in the same map! (%d vs %d)",
						*_stMem.rbegin(), signature->id(),
						_signatures.at(*_stMem.rbegin())->mapId(), signature->mapId());
			}
		}

		_signatures.insert(_signatures.end(), std::pair<int, Signature *>(signature->id(), signature));
		_stMem.insert(_stMem.end(), signature->id());

		if(_vwd)
		{
			UDEBUG("%d words ref for the signature %d", signature->getWords().size(), signature->id());
		}
		if(signature->getWords().size())
		{
			signature->setEnabled(true);
		}
	}

	UDEBUG("time = %fs", timer.ticks());
}

void Memory::addSignatureToWm(Signature * signature)
{
	if(signature)
	{
		UDEBUG("Inserting node %d in WM...", signature->id());
		_workingMem.insert(signature->id());
		_signatures.insert(std::pair<int, Signature*>(signature->id(), signature));
		++_signaturesAdded;
	}
	else
	{
		UERROR("Signature is null ?!?");
	}
}

const Signature * Memory::getSignature(int id) const
{
	return _getSignature(id);
}

Signature * Memory::_getSignature(int id) const
{
	return uValue(_signatures, id, (Signature*)0);
}

int Memory::getVWDictionarySize() const
{
	return _vwd->getVisualWords().size();
}

void Memory::getPose(int locationId, Transform & pose, bool lookInDatabase) const
{
	const Signature * s = getSignature(locationId);
	int mapId = -1;
	if(s)
	{
		pose = s->getPose();
		mapId = s->mapId();

		if(pose.isNull())
		{
			UERROR("Pose of %d is null?!?", locationId);
		}
	}
	else if(lookInDatabase && _dbDriver)
	{
		_dbDriver->getPose(locationId, pose, mapId);
	}
}

std::map<int, Transform> Memory::getNeighborLinks(int signatureId, bool ignoreNeighborByLoopClosure, bool lookInDatabase) const
{
	std::map<int, Transform> links;
	Signature * sTop = uValue(_signatures, signatureId, (Signature*)0);
	if(sTop)
	{
		std::list<Signature *> loops;
		loops.push_back(sTop);
		while(loops.size())
		{
			Signature * s = *loops.begin();
			loops.pop_front();
			if(s)
			{
				const std::map<int, Transform> & neighbors = s->getNeighbors();
				links.insert(neighbors.begin(), neighbors.end());
				if(!ignoreNeighborByLoopClosure)
				{
					const std::map<int, Transform> & loopIds = s->getLoopClosureIds();
					for(std::map<int, Transform>::const_iterator iter = loopIds.begin(); iter!=loopIds.end(); ++iter)
					{
						if(iter->first > 0 && _stMem.find(iter->first) == _stMem.end())
						{
							loops.push_back(uValue(_signatures, iter->first, (Signature*)0));
						}
					}
				}
			}
		}

		if(!ignoreNeighborByLoopClosure)
		{
			// Check for child loop closure ids
			const std::map<int, Transform> & childTopIds = sTop->getChildLoopClosureIds();
			for(std::map<int, Transform>::const_iterator iter = childTopIds.begin(); iter!=childTopIds.end(); ++iter)
			{
				if(iter->first > 0 && _stMem.find(iter->first) == _stMem.end())
				{
					loops.push_back(uValue(_signatures, iter->first, (Signature*)0));
				}
			}
			while(loops.size())
			{
				Signature * s = *loops.begin();
				loops.pop_front();
				if(s)
				{
					const std::map<int, Transform> & neighbors = s->getNeighbors();
					links.insert(neighbors.begin(), neighbors.end());
					const std::map<int, Transform> & childIds = s->getChildLoopClosureIds();
					for(std::map<int, Transform>::const_iterator iter = childIds.begin(); iter!=childIds.end(); ++iter)
					{
						if(iter->first > 0 && _stMem.find(iter->first) == _stMem.end())
						{
							loops.push_back(uValue(_signatures, iter->first, (Signature*)0));
						}
					}
				}
			}
		}
	}
	else if(lookInDatabase && _dbDriver)
	{
		std::map<int, Transform> neighbors;
		_dbDriver->loadNeighbors(signatureId, neighbors);
		links.insert(neighbors.begin(), neighbors.end());
	}
	else
	{
		UWARN("Cannot find signature %d in memory", signatureId);
	}
	return links;
}

// return map<Id,Margin>, including signatureId
// maxCheckedInDatabase = -1 means no limit to check in database (default)
// maxCheckedInDatabase = 0 means don't check in database
std::map<int, int> Memory::getNeighborsId(int signatureId,
		unsigned int margin, // 0 means infinite margin
		int maxCheckedInDatabase, // default -1 (no limit)
		bool incrementMarginOnLoop, // default false
		bool ignoreLoopIds, // default false
		double * dbAccessTime
		) const
{
	//UDEBUG("signatureId=%d, neighborsMargin=%d", signatureId, margin);
	if(dbAccessTime)
	{
		*dbAccessTime = 0;
	}
	std::map<int, int> ids;
	if(signatureId<=0)
	{
		return ids;
	}
	int nbLoadedFromDb = 0;
	std::list<int> curentMarginList;
	std::set<int> currentMargin;
	std::set<int> nextMargin;
	nextMargin.insert(signatureId);
	unsigned int m = 0;
	while((margin == 0 || m < margin) && nextMargin.size())
	{
		curentMarginList = std::list<int>(nextMargin.begin(), nextMargin.end());
		nextMargin.clear();
		// first pass: count number of node in current margin in database
		for(std::list<int>::iterator jter = curentMarginList.begin(); jter!=curentMarginList.end();++jter)
		{
			if(!uContains(ids, *jter))
			{
				const Signature * s = this->getSignature(*jter);
				if(!s)
				{
					++nbLoadedFromDb;
				}
			}
		}

		for(std::list<int>::iterator jter = curentMarginList.begin(); jter!=curentMarginList.end(); ++jter)
		{
			if(ids.insert(std::pair<int, int>(*jter, m)).second)
			{
				//UDEBUG("Added %d with margin %d", *jter, m);
				// Look up in STM/WM if all ids are here, if not... load them from the database
				const Signature * s = this->getSignature(*jter);
				std::map<int, Transform> tmpNeighborIds;
				std::map<int, Transform> tmpLoopClosureIds;
				std::map<int, Transform> tmpChildLoopClosureIds;
				const std::map<int, Transform> * neighborIds = &tmpNeighborIds;
				const std::map<int, Transform> * loopClosureIds = &tmpLoopClosureIds;
				const std::map<int, Transform> * childLoopClosureIds = &tmpChildLoopClosureIds;
				if(s)
				{
					neighborIds = &s->getNeighbors();

					if(!ignoreLoopIds)
					{
						loopClosureIds = &s->getLoopClosureIds();
						childLoopClosureIds = &s->getChildLoopClosureIds();
					}
				}
				else if(maxCheckedInDatabase == -1 || (maxCheckedInDatabase > 0 && _dbDriver && nbLoadedFromDb < maxCheckedInDatabase))
				{
					UTimer timer;
					_dbDriver->loadNeighbors(*jter, tmpNeighborIds);
					if(!ignoreLoopIds)
					{
						_dbDriver->loadLoopClosures(*jter, tmpLoopClosureIds, tmpChildLoopClosureIds);
					}
					if(dbAccessTime)
					{
						*dbAccessTime += timer.getElapsedTime();
					}
				}

				// Neighbor links
				for(std::map<int, Transform>::const_iterator iter=neighborIds->begin(); iter!=neighborIds->end(); ++iter)
				{
					if( (ignoreLoopIds || (loopClosureIds->find(iter->first) == loopClosureIds->end() && childLoopClosureIds->find(iter->first) == childLoopClosureIds->end())) &&
						!uContains(ids, iter->first) &&
						nextMargin.find(iter->first) == nextMargin.end())
					{
						nextMargin.insert(iter->first);
					}
				}

				// Parent links
				for(std::map<int, Transform>::const_iterator iter=loopClosureIds->begin(); iter!=loopClosureIds->end(); ++iter)
				{
					if( iter->first && !uContains(ids, iter->first)/* && isInNeighborLimits(iter->first, limits)*/)
					{
						if(incrementMarginOnLoop)
						{
							nextMargin.insert(iter->first);
							//UDEBUG("next of %d + %d", *jter, iter->first);
						}
						else
						{
							if(currentMargin.insert(iter->first).second)
							{
								const Signature * s = this->getSignature(iter->first);
								if(!s)
								{
									// update db count because it's on current margin
									++nbLoadedFromDb;
								}
								curentMarginList.push_back(iter->first);
								//UDEBUG("current of %d + %d", *jter, iter->first);
							}
						}
					}
				}

				//Child links
				for(std::map<int, Transform>::const_iterator iter=childLoopClosureIds->begin(); iter!=childLoopClosureIds->end(); ++iter)
				{
					if( iter->first && !uContains(ids, iter->first)/* && isInNeighborLimits(iter->first, limits)*/)
					{
						if(incrementMarginOnLoop)
						{
							nextMargin.insert(iter->first);
							//UDEBUG("next of %d + %d", *jter, iter->first);
						}
						else
						{
							if(currentMargin.insert(iter->first).second)
							{
								const Signature * s = this->getSignature(iter->first);
								if(!s)
								{
									// update db count because it's on current margin
									++nbLoadedFromDb;
								}
								curentMarginList.push_back(iter->first);
								//UDEBUG("current of %d + %d", *jter, iter->first);
							}
						}
					}
				}
			}
		}
		++m;
	}
	return ids;
}

void Memory::getLoopClosureIds(int signatureId, std::map<int, Transform> & loopClosureIds, std::map<int, Transform> & childLoopClosureIds, bool lookInDatabase) const
{
	const Signature * s = this->getSignature(signatureId);
	loopClosureIds.clear();
	childLoopClosureIds.clear();
	if(s)
	{
		loopClosureIds = s->getLoopClosureIds();
		childLoopClosureIds = s->getChildLoopClosureIds();
	}
	else if(lookInDatabase && _dbDriver)
	{
		_dbDriver->loadLoopClosures(signatureId, loopClosureIds, childLoopClosureIds);
	}
}

int Memory::getNextId()
{
	return ++_idCount;
}

int Memory::incrementMapId()
{
	//don't increment if there is no location in the current map
	const Signature * s = getLastWorkingSignature();
	if(s && s->mapId() == _idMapCount)
	{
		return ++_idMapCount;
	}
	return _idMapCount;
}

int Memory::getDatabaseMemoryUsed() const
{
	int memoryUsed = 0;
	if(_dbDriver)
	{
		memoryUsed = _dbDriver->getMemoryUsed()/(1024*1024); //Byte to MB
	}
	return memoryUsed;
}

double Memory::getDbSavingTime() const
{
	return _dbDriver?_dbDriver->getEmptyTrashesTime():0;
}

std::set<int> Memory::getAllSignatureIds() const
{
	std::set<int> ids;
	if(_dbDriver)
	{
		_dbDriver->getAllNodeIds(ids);
	}
	for(std::map<int, Signature*>::const_iterator iter = _signatures.begin(); iter!=_signatures.end(); ++iter)
	{
		ids.insert(iter->first);
	}
	return ids;
}

void Memory::clear()
{
	UDEBUG("");

	this->cleanUnusedWords();

	if(_dbDriver)
	{
		_dbDriver->emptyTrashes();
		_dbDriver->join();
	}

	// Save some stats to the db, save only when the mem is not empty
	if(_dbDriver && (_stMem.size() || _workingMem.size()))
	{
		unsigned int memSize = _workingMem.size() + _stMem.size();
		if(_workingMem.size() && *_workingMem.begin() < 0)
		{
			--memSize;
		}

		// this is only a safe check...not supposed to occur.
		UASSERT_MSG(memSize == _signatures.size(),
				uFormat("The number of signatures don't match! _workingMem=%d, _stMem=%d, _signatures=%d",
						_workingMem.size(), _stMem.size(), _signatures.size()).c_str());

		UDEBUG("Adding statistics after run...");
		_dbDriver->addStatisticsAfterRun(memSize,
				_lastSignature?_lastSignature->id():0,
				UProcessInfo::getMemoryUsage(),
				_dbDriver->getMemoryUsed(),
				_vwd->getVisualWords().size());
	}
	UDEBUG("");

	//Get the tree root (parents)
	std::map<int, Signature*> mem = _signatures;
	for(std::map<int, Signature *>::iterator i=mem.begin(); i!=mem.end(); ++i)
	{
		if(i->second)
		{
			UDEBUG("deleting from the working and the short-term memory: %d", i->first);
			this->moveToTrash(i->second);
		}
	}

	if(_workingMem.size() != 0 && !(_workingMem.size() == 1 && *_workingMem.begin() == kIdVirtual))
	{
		ULOGGER_ERROR("_workingMem must be empty here, size=%d", _workingMem.size());
	}
	_workingMem.clear();
	if(_stMem.size() != 0)
	{
		ULOGGER_ERROR("_stMem must be empty here, size=%d", _stMem.size());
	}
	_stMem.clear();
	if(_signatures.size()!=0)
	{
		ULOGGER_ERROR("_signatures must be empty here, size=%d", _signatures.size());
	}
	_signatures.clear();

	UDEBUG("");
	// Wait until the db trash has finished cleaning the memory
	if(_dbDriver)
	{
		_dbDriver->emptyTrashes();
	}
	UDEBUG("");
	_lastSignature = 0;
	_lastGlobalLoopClosureParentId = 0;
	_lastGlobalLoopClosureChildId = 0;
	_idCount = kIdStart;
	_idMapCount = kIdStart;
	_memoryChanged = false;

	if(_dbDriver)
	{
		_dbDriver->join(true);
		cleanUnusedWords();
		_dbDriver->emptyTrashes();
	}
	else
	{
		cleanUnusedWords();
	}
	if(_vwd)
	{
		_vwd->clear();
	}
	UDEBUG("");
}

/**
 * Compute the likelihood of the signature with some others in the memory.
 * Important: Assuming that all other ids are under 'signature' id.
 * If an error occurs, the result is empty.
 */
std::map<int, float> Memory::computeLikelihood(const Signature * signature, const std::list<int> & ids)
{
	if(!_tfIdfLikelihoodUsed)
	{
		UTimer timer;
		timer.start();
		std::map<int, float> likelihood;

		if(!signature)
		{
			ULOGGER_ERROR("The signature is null");
			return likelihood;
		}
		else if(ids.empty())
		{
			UWARN("ids list is empty");
			return likelihood;
		}

		for(std::list<int>::const_iterator iter = ids.begin(); iter!=ids.end(); ++iter)
		{
			float sim = 0.0f;
			if(*iter > 0)
			{
				const Signature * sB = this->getSignature(*iter);
				if(!sB)
				{
					UFATAL("Signature %d not found in WM ?!?", *iter);
				}
				sim = signature->compareTo(sB);
			}

			likelihood.insert(likelihood.end(), std::pair<int, float>(*iter, sim));
		}

		UDEBUG("compute likelihood... %f s", timer.ticks());
		return likelihood;
	}
	else
	{
		// TODO cleanup , old way...
		UTimer timer;
		timer.start();
		std::map<int, float> likelihood;
		std::map<int, float> calculatedWordsRatio;

		if(!signature)
		{
			ULOGGER_ERROR("The signature is null");
			return likelihood;
		}
		else if(ids.empty())
		{
			UWARN("ids list is empty");
			return likelihood;
		}

		for(std::list<int>::const_iterator iter = ids.begin(); iter!=ids.end(); ++iter)
		{
			likelihood.insert(likelihood.end(), std::pair<int, float>(*iter, 0.0f));
		}

		const std::list<int> & wordIds = uUniqueKeys(signature->getWords());

		float nwi; // nwi is the number of a specific word referenced by a place
		float ni; // ni is the total of words referenced by a place
		float nw; // nw is the number of places referenced by a specific word
		float N; // N is the total number of places

		float logNnw;
		const VisualWord * vw;

		N = this->getSignatures().size();

		if(N)
		{
			UDEBUG("processing... ");
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
										//UDEBUG("%d, %f %f %f %f", vw->id(), logNnw, nwi, ni, ( nwi  * logNnw ) / ni);
										iter->second += ( nwi  * logNnw ) / ni;
									}
								}
							}
						}
					}
				}
			}
		}

		UDEBUG("compute likelihood %f s", timer.ticks());
		return likelihood;
	}
}

// Weights of the signatures in the working memory <signature id, weight>
std::map<int, int> Memory::getWeights() const
{
	std::map<int, int> weights;
	for(std::set<int>::const_iterator iter=_workingMem.begin(); iter!=_workingMem.end(); ++iter)
	{
		if(*iter > 0)
		{
			const Signature * s = this->getSignature(*iter);
			if(!s)
			{
				UFATAL("Location %d must exist in memory", *iter);
			}
			weights.insert(weights.end(), std::make_pair(*iter, s->getWeight()));
		}
		else
		{
			weights.insert(weights.end(), std::make_pair(*iter, -1));
		}
	}
	return weights;
}

std::list<int> Memory::forget(const std::set<int> & ignoredIds)
{
	UDEBUG("");
	std::list<int> signaturesRemoved;
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
			std::list<Signature *> signatures = this->getRemovableSignatures(1, ignoredIds);
			if(signatures.size())
			{
				Signature *  s = dynamic_cast<Signature *>(signatures.front());
				if(s)
				{
					signaturesRemoved.push_back(s->id());
					this->moveToTrash(s);
					wordsRemoved = _vwd->getUnusedWordsSize();
				}
				else
				{
					break;
				}
			}
			else
			{
				break;
			}
		}
		UDEBUG("newWords=%d, wordsRemoved=%d", newWords, wordsRemoved);
	}
	else
	{
		UDEBUG("");
		// Remove one more than total added during the iteration
		std::list<Signature *> signatures = getRemovableSignatures(_signaturesAdded+1, ignoredIds);
		for(std::list<Signature *>::iterator iter=signatures.begin(); iter!=signatures.end(); ++iter)
		{
			signaturesRemoved.push_back((*iter)->id());
			// When a signature is deleted, it notifies the memory
			// and it is removed from the memory list
			this->moveToTrash(*iter);
		}
		UDEBUG("signaturesRemoved=%d, _signaturesAdded=%d", (int)signatures.size(), _signaturesAdded);
	}
	return signaturesRemoved;
}


std::list<int> Memory::cleanup(const std::list<int> & ignoredIds)
{
	UDEBUG("");
	std::list<int> signaturesRemoved;

	// bad signature
	if((_lastSignature->isBadSignature() && _badSignaturesIgnored) || !_incrementalMemory)
	{
		if(_lastSignature->isBadSignature())
		{
			UDEBUG("Bad signature! %d", _lastSignature->id());
		}
		signaturesRemoved.push_back(_lastSignature->id());
		moveToTrash(_lastSignature, _incrementalMemory);
	}

	return signaturesRemoved;
}

void Memory::emptyTrash()
{
	if(_dbDriver)
	{
		_dbDriver->emptyTrashes(true);
	}
}

void Memory::joinTrashThread()
{
	if(_dbDriver)
	{
		UDEBUG("");
		_dbDriver->join();
		UDEBUG("");
	}
}

class WeightIdKey
{
public:
	WeightIdKey(int w, int i) :
		weight(w),
		id(i) {}
	bool operator<(const WeightIdKey & k) const
	{
		if(weight < k.weight)
		{
			return true;
		}
		else if(weight == k.weight)
		{
			if(id < k.id)
			{
				return true;
			}
		}
		return false;
	}
	int weight, id;
};
std::list<Signature *> Memory::getRemovableSignatures(int count, const std::set<int> & ignoredIds)
{
	//UDEBUG("");
	std::list<Signature *> removableSignatures;
	std::map<WeightIdKey, Signature *> signatureMap;

	// Find the last index to check...
	const std::set<int> & wm = _workingMem;
	UDEBUG("mem.size()=%d, ignoredIds.size()=%d", wm.size(), ignoredIds.size());

	if(wm.size())
	{
		int recentWmMaxSize = _recentWmRatio * float(wm.size());
		bool recentWmImmunized = false;
		// look for the position of the lastLoopClosureId in WM
		int currentRecentWmSize = 0;
		if(_lastGlobalLoopClosureParentId > 0 && _stMem.find(_lastGlobalLoopClosureParentId) == _stMem.end())
		{
			// If set, it must be in WM
			std::set<int>::const_iterator iter = _workingMem.find(_lastGlobalLoopClosureParentId);
			while(iter != _workingMem.end())
			{
				++currentRecentWmSize;
				++iter;
			}
			if(currentRecentWmSize>1 && currentRecentWmSize < recentWmMaxSize)
			{
				recentWmImmunized = true;
			}
			else if(currentRecentWmSize == 0 && _workingMem.size() > 1)
			{
				UERROR("Last loop closure id not found in WM (%d)", _lastGlobalLoopClosureParentId);
			}
			UDEBUG("currentRecentWmSize=%d, recentWmMaxSize=%d, _recentWmRatio=%f, end recent wM = %d", currentRecentWmSize, recentWmMaxSize, _recentWmRatio, _lastGlobalLoopClosureParentId);
		}

		// Ignore neighbor of the last location in STM (for neighbor links redirection issue during Rehearsal).
		Signature * lastInSTM = 0;
		if(_stMem.size())
		{
			lastInSTM = _signatures.at(*_stMem.begin());
		}

		for(std::set<int>::const_iterator memIter = wm.begin(); memIter != wm.end(); ++memIter)
		{
			if( (recentWmImmunized && *memIter > _lastGlobalLoopClosureParentId) ||
				*memIter == _lastGlobalLoopClosureParentId)
			{
				// ignore recent memory
			}
			else if(*memIter > 0 && ignoredIds.find(*memIter) == ignoredIds.end() && (!lastInSTM || !lastInSTM->hasNeighbor(*memIter)))
			{
				Signature * s = this->_getSignature(*memIter);
				if(s)
				{
					// Its loop closures must not be in STM to be removable, rehearsal issue
					bool foundInSTM = false;
					for(std::map<int, Transform>::const_iterator iter = s->getLoopClosureIds().begin(); iter!=s->getLoopClosureIds().end(); ++iter)
					{
						if(_stMem.find(iter->first) != _stMem.end())
						{
							UDEBUG("Ignored %d because it has a parent (%d) in STM", s->id(), iter->first);
							foundInSTM = true;
							break;
						}
					}
					// Its neighbors must not be in STM to be removable, rehearsal issue
					if(!foundInSTM)
					{
						for(std::set<int>::iterator iter = _stMem.begin(); iter!=_stMem.end(); ++iter)
						{
							if(s->hasNeighbor(*iter))
							{
								UDEBUG("Ignored %d because it has a neighbor (%d) in STM", s->id(), *iter);
								foundInSTM = true;
								break;
							}
						}
					}
					if(!foundInSTM)
					{
						// less weighted signature priority to be transferred
						signatureMap.insert(std::make_pair(WeightIdKey(s->getWeight(), s->id()), s));
					}
				}
				else
				{
					ULOGGER_ERROR("Not supposed to occur!!!");
				}
			}
			else
			{
				//UDEBUG("Ignoring id %d", memIter->first);
			}
		}

		int recentWmCount = 0;
		std::set<int> addedSignatures;
		// make the list of removable signatures
		// Criteria : Weight -> ID
		UDEBUG("signatureMap.size()=%d", (int)signatureMap.size());
		for(std::map<WeightIdKey, Signature*>::iterator iter=signatureMap.begin();
			iter!=signatureMap.end();
			++iter)
		{
			bool removable = true;
			if(removable)
			{
				if(!recentWmImmunized)
				{

					UDEBUG("weight=%d, id=%d, lcCount=%d, lcId=%d, childId=%d",
							iter->first.weight,
							iter->second->id(),
							int(iter->second->getLoopClosureIds().size()),
							iter->second->getLoopClosureIds().size()?iter->second->getLoopClosureIds().rbegin()->first:0,
							iter->second->getChildLoopClosureIds().size()?iter->second->getChildLoopClosureIds().rbegin()->first:0);
					removableSignatures.push_back(iter->second);
					addedSignatures.insert(iter->second->id());

					if(iter->second->id() > _lastGlobalLoopClosureParentId)
					{
						++recentWmCount;
						if(currentRecentWmSize - recentWmCount < recentWmMaxSize)
						{
							UDEBUG("switched recentWmImmunized");
							recentWmImmunized = true;
						}
					}
				}
				else if(iter->second->id() < _lastGlobalLoopClosureParentId)
				{
					UDEBUG("weight=%d, id=%d, lcCount=%d, lcId=%d, childId=%d",
							iter->first.weight,
							iter->second->id(),
							int(iter->second->getLoopClosureIds().size()),
							iter->second->getLoopClosureIds().size()?iter->second->getLoopClosureIds().rbegin()->first:0,
							iter->second->getChildLoopClosureIds().size()?iter->second->getChildLoopClosureIds().rbegin()->first:0);
					removableSignatures.push_back(iter->second);
					addedSignatures.insert(iter->second->id());
				}
				if(removableSignatures.size() >= (unsigned int)count)
				{
					break;
				}
			}
		}
	}
	else
	{
		ULOGGER_WARN("not enough signatures to get an old one...");
	}
	return removableSignatures;
}

void Memory::moveToTrash(Signature * s, bool saveToDatabase)
{
	UDEBUG("id=%d", s?s->id():0);
	if(s)
	{
		// If not saved to database or it is a bad signature (not saved), remove links!
		if(!saveToDatabase || (!s->isSaved() && s->isBadSignature() && _badSignaturesIgnored))
		{
			UASSERT_MSG(this->isInSTM(s->id()),
					uFormat("Deleting location (%d) outside the STM is not implemented!", s->id()).c_str());
			const std::map<int, Transform> & neighbors = s->getNeighbors();
			for(std::map<int, Transform>::const_iterator iter=neighbors.begin(); iter!=neighbors.end(); ++iter)
			{
				Signature * n = this->_getSignature(iter->first);
				// neighbor to s
				if(n)
				{
					if(iter->first > s->id() && (n->getNeighbors().size() > 1 || !n->hasNeighbor(s->id())))
					{
						UWARN("Neighbor %d of %d is newer, removing neighbor link may split the map!", iter->first, s->id());
					}

					n->removeNeighbor(s->id());
					if(s == _lastSignature)
					{
						_lastSignature = n;
					}
				}
				else
				{
					UERROR("neighbor %d of %d not in WM/STM?!?", iter->first, s->id());
				}
			}
			s->removeNeighbors();

			std::map<int, Transform> children = s->getChildLoopClosureIds();
			for(std::map<int, Transform>::const_iterator iter=children.begin(); iter!=children.end(); ++iter)
			{
				Signature * child = _getSignature(iter->first);
				if(child)
				{
					child->removeLoopClosureId(s->id());
					child->setWeight(child->getWeight() + s->getWeight()); // copy weight
				}
				else
				{
					UERROR("loop child %d of %d not in WM/STM?!?", iter->first, s->id());
				}
				s->removeChildLoopClosureId(iter->first);
			}
			std::map<int, Transform> parents = s->getLoopClosureIds();
			for(std::map<int, Transform>::const_iterator iter=parents.begin(); iter!=parents.end(); ++iter)
			{
				Signature * p = _getSignature(iter->first);
				if(p)
				{
					p->removeChildLoopClosureId(s->id());
				}
				else
				{
					UERROR("loop parent %d of %d not in WM/STM?!?", iter->first, s->id());
				}
				s->removeLoopClosureId(iter->first);
			}
			s->setWeight(0);
		}

		this->disableWordsRef(s->id(), saveToDatabase);

		_workingMem.erase(s->id());
		_stMem.erase(s->id());
		_signatures.erase(s->id());

		if(_lastSignature == s)
		{
			_lastSignature = 0;
			if(_stMem.size())
			{
				_lastSignature = this->_getSignature(*_stMem.rbegin());
			}
		}

		if(	saveToDatabase &&
			_dbDriver &&
			s->id()>0)
		{
			_dbDriver->asyncSave(s);
		}
		else
		{
			delete s;
		}
	}
}

int Memory::getLastSignatureId() const
{
	return _idCount;
}

const Signature * Memory::getLastWorkingSignature() const
{
	UDEBUG("");
	return _lastSignature;
}

void Memory::deleteLocation(int locationId)
{
	UINFO("Deleting location %d", locationId);
	Signature * location = _getSignature(locationId);
	if(location)
	{
		this->moveToTrash(location, false);
	}
}

void Memory::rejectLoopClosure(int oldId, int newId)
{
	Signature * oldS = this->_getSignature(oldId);
	Signature * newS = this->_getSignature(newId);
	if(oldS && newS)
	{
		UDEBUG("removing loop closure from location %d", newS->id());

		const std::map<int, Transform> & children = newS->getChildLoopClosureIds();
		for(std::map<int, Transform>::const_iterator iter=children.begin(); iter!=children.end(); ++iter)
		{
			if(iter->first == oldId)
			{
				oldS->removeLoopClosureId(newS->id());
				oldS->setWeight(oldS->getWeight()+1);

				newS->removeChildLoopClosureId(iter->first);
				newS->setWeight(newS->getWeight()>0?newS->getWeight()-1:0);
				break;
			}
		}
		if(newS->getChildLoopClosureIds().size() == 0 && newId == _lastGlobalLoopClosureParentId)
		{
			_lastGlobalLoopClosureParentId = 0;
			_lastGlobalLoopClosureChildId = 0;
		}
	}
	else
	{
		if(!newS)
		{
			UERROR("Signature %d is not in working memory... cannot remove loop closure links.", newS->id());
		}
		if(!oldS)
		{
			UERROR("Signature %d is not in working memory... cannot remove loop closure links.", oldS->id());
		}
	}
}

// compute transform newId -> oldId
Transform Memory::computeVisualTransform(int oldId, int newId) const
{
	const Signature * oldS = this->getSignature(oldId);
	const Signature * newS = this->getSignature(newId);

	Transform transform;

	if(oldS && newId)
	{
		return computeVisualTransform(*oldS, *newS);
	}
	return Transform();
}

// compute transform newId -> oldId
Transform Memory::computeVisualTransform(const Signature & oldS, const Signature & newS) const
{
	Transform transform;
	// Guess transform from visual words
	if(!oldS.getWords3().empty() && !newS.getWords3().empty())
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr inliersOld(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr inliersNew(new pcl::PointCloud<pcl::PointXYZ>);
		util3d::findCorrespondences(
				oldS.getWords3(),
				newS.getWords3(),
				*inliersOld,
				*inliersNew,
				_bowMaxDepth);

		if((int)inliersOld->size() >= _bowMinInliers)
		{
			UDEBUG("Correspondences = %d", (int)inliersOld->size());

			int inliersCount = 0;
			Transform t = util3d::transformFromXYZCorrespondences(
					inliersOld,
					inliersNew,
					_bowInlierDistance,
					_bowIterations,
					&inliersCount);
			if(!t.isNull() && inliersCount >= _bowMinInliers)
			{
				transform = t;
			}
			else if(inliersCount < _bowMinInliers)
			{
				UINFO("Not enough inliers %d/%d between %d and %d", inliersCount, _bowMinInliers, oldS.id(), newS.id());
			}
		}
		else
		{
			UDEBUG("Not enough inliers %d/%d between %d and %d", (int)inliersOld->size(), _bowMinInliers, oldS.id(), newS.id());
		}
	}
	else if(!oldS.isBadSignature() && !newS.isBadSignature())
	{
		UERROR("Words 3D empty?!?");
	}
	return transform;
}

// compute transform newId -> oldId
Transform Memory::computeIcpTransform(int oldId, int newId, Transform guess, bool icp3D)
{
	Signature * oldS = this->_getSignature(oldId);
	Signature * newS = this->_getSignature(newId);

	if(oldS && newS && _dbDriver)
	{
		std::list<Signature*> depthToLoad;
		std::set<int> added;
		if(icp3D)
		{
			//Depth required, if not in RAM, load it from LTM
			if(oldS->getDepth().empty())
			{
				depthToLoad.push_back(oldS);
				added.insert(oldS->id());
			}
			if(newS->getDepth().empty())
			{
				depthToLoad.push_back(newS);
				added.insert(newS->id());
			}
		}
		else
		{
			//Depth required, if not in RAM, load it from LTM
			if(oldS->getDepth2D().size() == 0 && added.find(oldS->id()) == added.end())
			{
				depthToLoad.push_back(oldS);
			}
			if(newS->getDepth2D().size() == 0 && added.find(newS->id()) == added.end())
			{
				depthToLoad.push_back(newS);
			}
		}
		if(depthToLoad.size())
		{
			_dbDriver->loadNodeData(depthToLoad, true);
		}
	}
	Transform t;
	if(oldS && newS)
	{
		t = computeIcpTransform(*oldS, *newS, guess, icp3D);
	}
	return t;
}

// get transform from the new to old node
Transform Memory::computeIcpTransform(const Signature & oldS, const Signature & newS, Transform guess, bool icp3D) const
{
	if(guess.isNull())
	{
		//Make a guess using odometry
		guess = oldS.getPose().inverse() * newS.getPose();
		UASSERT_MSG(oldS.mapId() == newS.mapId(), "Compute ICP from two different maps is not implemented!");
	}
	else
	{
		guess = guess.inverse(); // from pose to cloud data
	}
	UDEBUG("Guess transform = %s", guess.prettyPrint().c_str());

	Transform transform;

	// ICP with guess transform
	if(icp3D)
	{
		UDEBUG("3D ICP");
		util3d::CompressionThread ctOld(oldS.getDepth(), true);
		util3d::CompressionThread ctNew(newS.getDepth(), true);
		ctOld.start();
		ctNew.start();
		ctOld.join();
		ctNew.join();
		cv::Mat oldDepth = ctOld.getUncompressedData();
		cv::Mat newDepth = ctNew.getUncompressedData();
		if(!oldDepth.empty() && !newDepth.empty())
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr oldCloudXYZ = util3d::getICPReadyCloud(
					oldDepth,
					oldS.getDepthConstant(),
					_icpDecimation,
					_icpMaxDepth,
					_icpVoxelSize,
					_icpSamples,
					oldS.getLocalTransform());
			pcl::PointCloud<pcl::PointXYZ>::Ptr newCloudXYZ = util3d::getICPReadyCloud(
					newDepth,
					newS.getDepthConstant(),
					_icpDecimation,
					_icpMaxDepth,
					_icpVoxelSize,
					_icpSamples,
					guess * newS.getLocalTransform());

			pcl::PointCloud<pcl::PointNormal>::Ptr oldCloud = util3d::computeNormals(oldCloudXYZ);
			pcl::PointCloud<pcl::PointNormal>::Ptr newCloud = util3d::computeNormals(newCloudXYZ);

			std::vector<int> indices;
			newCloud = util3d::removeNaNNormalsFromPointCloud(newCloud);
			oldCloud = util3d::removeNaNNormalsFromPointCloud(oldCloud);

			// 3D
			double fitness = 0;
			bool hasConverged = false;
			Transform icpT;
			if(newCloud->size() && oldCloud->size())
			{
				icpT = util3d::icpPointToPlane(newCloud,
						oldCloud,
					   _icpMaxCorrespondenceDistance,
					   _icpMaxIterations,
					   hasConverged,
					   fitness);
			}
			else
			{
				UWARN("Clouds empty ?!?");
			}

			//pcl::io::savePCDFile("old.pcd", *oldCloud);
			//pcl::io::savePCDFile("newguess.pcd", *newCloud);
			//newCloud = util3d::transformPointCloud(newCloud, icpT);
			//pcl::io::savePCDFile("newicp.pcd", *newCloud);

			UDEBUG("fitness=%f", fitness);

			if(hasConverged && (_icpMaxFitness == 0 || fitness < _icpMaxFitness))
			{
				transform = icpT * guess;
				transform = transform.inverse();
			}
			else
			{
				UWARN("Cannot compute transform (hasConverged=%s fitness=%f/%f)",
						hasConverged?"true":"false", fitness, _icpMaxFitness);
			}
		}
		else
		{
			UERROR("Depths 3D empty?!?");
		}
	}
	else // icp 2D
	{
		UDEBUG("2D ICP");

		// We are 2D here, make sure the guess has only YAW rotation
		float x,y,z,r,p,yaw;
		guess.getTranslationAndEulerAngles(x,y,z, r,p,yaw);
		guess = util3d::transformFromEigen3f(pcl::getTransformation(x,y,0, 0, 0, yaw));
		if(r!=0 || p!=0)
		{
			UINFO("2D ICP: Dropping z (%f), roll (%f) and pitch (%f) rotation!", z, r, p);
		}

		util3d::CompressionThread ctOld(oldS.getDepth2D(), false);
		util3d::CompressionThread ctNew(newS.getDepth2D(), false);
		ctOld.start();
		ctNew.start();
		ctOld.join();
		ctNew.join();
		cv::Mat oldDepth2D = ctOld.getUncompressedData();
		cv::Mat newDepth2D = ctNew.getUncompressedData();
		if(!oldDepth2D.empty() && !newDepth2D.empty())
		{
			// 2D
			pcl::PointCloud<pcl::PointXYZ>::Ptr oldCloud = util3d::cvMat2Cloud(oldDepth2D);
			pcl::PointCloud<pcl::PointXYZ>::Ptr newCloud = util3d::cvMat2Cloud(newDepth2D, guess);

			//voxelize
			if(_icp2VoxelSize > 0.0f)
			{
				oldCloud = util3d::voxelize(oldCloud, _icp2VoxelSize);
				newCloud = util3d::voxelize(newCloud, _icp2VoxelSize);
			}

			double fitness = 0.0f;
			bool hasConverged = false;
			Transform icpT;
			float correspondencesRatio = -1.0f;
			if(newCloud->size() && oldCloud->size())
			{
				icpT = util3d::icp2D(newCloud,
						oldCloud,
					   _icp2MaxCorrespondenceDistance,
					   _icp2MaxIterations,
					   hasConverged,
					   fitness);

				//pcl::io::savePCDFile("lccold.pcd", *oldCloud);
				//pcl::io::savePCDFile("lccnewguess.pcd", *newCloud);
				newCloud = util3d::transformPointCloud(newCloud, icpT);
				//pcl::io::savePCDFile("lccnewicp.pcd", *newCloud);

				// verify if there are enough correspondences
				int correspondences = util3d::getCorrespondencesCount(newCloud, oldCloud, _icp2MaxCorrespondenceDistance);
				correspondencesRatio = float(correspondences)/float(oldCloud->size()>newCloud->size()?oldCloud->size():newCloud->size());

				UDEBUG("hasConverged=%s, fitness=%f, correspondences=%d/%d (%f%%)",
						hasConverged?"true":"false",
						fitness,
						correspondences,
						(int)oldCloud->size(),
						correspondencesRatio*100.0f);
			}
			else
			{
				UWARN("Clouds empty ?!?");
			}

			if(hasConverged &&
			   (_icp2MaxFitness == 0 || fitness < _icp2MaxFitness) &&
			   correspondencesRatio >= _icp2CorrespondenceRatio)
			{
				transform = icpT * guess;
				transform = transform.inverse();
			}
			else
			{
				UWARN("Cannot compute transform (hasConverged=%s fitness=%f/%f correspondencesRatio=%f/%f)",
						hasConverged?"true":"false", fitness, _icpMaxFitness, correspondencesRatio, _icp2CorrespondenceRatio);
			}
		}
		else
		{
			UERROR("Depths 2D empty?!?");
		}
	}
	return transform;
}

// poses of newId and oldId must be in "poses"
Transform Memory::computeScanMatchingTransform(
		int newId,
		int oldId,
		const std::map<int, Transform> & poses)
{
	// make sure that all depth2D are loaded
	std::list<Signature*> depthToLoad;
	for(std::map<int, Transform>::const_iterator iter = poses.begin(); iter!=poses.end(); ++iter)
	{
		Signature * s = _getSignature(iter->first);
		UASSERT(s != 0);
		if(s->getDepth2D().size() == 0)
		{
			depthToLoad.push_back(s);
		}
	}
	if(depthToLoad.size() && _dbDriver)
	{
		_dbDriver->loadNodeData(depthToLoad, true);
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr assembledOldClouds(new pcl::PointCloud<pcl::PointXYZ>);
	for(std::map<int, Transform>::const_iterator iter = poses.begin(); iter!=poses.end(); ++iter)
	{
		if(iter->first != newId)
		{
			const Signature * s = this->getSignature(iter->first);
			if(s->getDepth2D().size())
			{
				*assembledOldClouds += *util3d::cvMat2Cloud(util3d::uncompressData(s->getDepth2D()), iter->second);
			}
			else
			{
				UWARN("Depth2D not found for signature %d", iter->first);
			}
		}
	}

	//voxelize
	if(assembledOldClouds->size() && _icp2VoxelSize > 0.0f)
	{
		assembledOldClouds = util3d::voxelize(assembledOldClouds, _icp2VoxelSize);
	}

	// get the new cloud
	const Signature * newS = getSignature(newId);
	pcl::PointCloud<pcl::PointXYZ>::Ptr newCloud;
	UASSERT(uContains(poses, newId));
	newCloud = util3d::cvMat2Cloud(util3d::uncompressData(newS->getDepth2D()), poses.at(newId));

	//voxelize
	if(newCloud->size() && _icp2VoxelSize > 0.0f)
	{
		newCloud = util3d::voxelize(newCloud, _icp2VoxelSize);
	}

	//pcl::io::savePCDFile("old.pcd", *assembledOldClouds);
	//pcl::io::savePCDFile("new.pcd", *newCloud);

	Transform transform;
	if(assembledOldClouds->size() && newCloud->size())
	{
		double fitness = 0.0f;
		bool hasConverged = false;
		Transform icpT = util3d::icp2D(newCloud,
				assembledOldClouds,
			   _icp2MaxCorrespondenceDistance,
			   _icp2MaxIterations,
			   hasConverged,
			   fitness);

		UDEBUG("icpT=%s", icpT.prettyPrint().c_str());

		newCloud = util3d::transformPointCloud(newCloud, icpT);
		//pcl::io::savePCDFile("newCorrected.pcd", *newCloud);

		// verify if there enough correspondences
		int correspondences = util3d::getCorrespondencesCount(newCloud, assembledOldClouds, _icp2MaxCorrespondenceDistance);
		float correspondencesRatio = float(correspondences)/float(newCloud->size());

		UDEBUG("fitness=%f, correspondences=%d/%d (%f%%)",
				fitness,
				correspondences,
				(int)newCloud->size(),
				correspondencesRatio);

		if(hasConverged &&
		   (_icp2MaxFitness == 0 || fitness < _icp2MaxFitness) &&
		   correspondencesRatio >= _icp2CorrespondenceRatio)
		{
			transform = poses.at(newId).inverse()*icpT.inverse() * poses.at(oldId);

			//newCloud = util3d::cvMat2Cloud(util3d::uncompressData(newS->getDepth2D()), poses.at(oldId)*transform.inverse());
			//pcl::io::savePCDFile("newFinal.pcd", *newCloud);
		}
		else
		{
			UWARN("Constraints failed... hasConverged=%s, fitness=%f, correspondences=%d/%d (%f%%)",
				hasConverged?"true":"false",
				fitness,
				correspondences,
				(int)newCloud->size(),
				correspondencesRatio);
		}
	}

	return transform;
}

// Transform from new to old
bool Memory::addLoopClosureLink(int oldId, int newId, const Transform & transform, bool global)
{
	ULOGGER_INFO("old=%d, new=%d transform: %s", oldId, newId, transform.prettyPrint().c_str());
	Signature * oldS = _getSignature(oldId);
	Signature * newS = _getSignature(newId);
	if(oldS && newS)
	{
		const std::map<int, Transform> & oldLoopclosureIds = oldS->getLoopClosureIds();
		if(oldLoopclosureIds.size() && oldLoopclosureIds.find(newS->id()) != oldLoopclosureIds.end())
		{
			// do nothing, already merged
			UDEBUG("already merged, old=%d, new=%d", oldId, newId);
			return true;
		}

		UDEBUG("Add loop closure link between %d and %d", oldS->id(), newS->id());

		oldS->addLoopClosureId(newS->id(), transform.inverse());
		newS->addChildLoopClosureId(oldS->id(), transform);

		if(_incrementalMemory && global)
		{
			_lastGlobalLoopClosureParentId = newS->id();
			_lastGlobalLoopClosureChildId = oldS->id();

			// udpate weights only if the memory is incremental
			newS->setWeight(newS->getWeight() + oldS->getWeight());
			oldS->setWeight(0);
		}
		return true;
	}
	else
	{
		if(!newS)
		{
			UERROR("newId=%d, oldId=%d, Signature %d not found in working/st memories", newId, oldId, newId);
		}
		if(!oldS)
		{
			UERROR("newId=%d, oldId=%d, Signature %d not found in working/st memories", newId, oldId, oldId);
		}
	}
	return false;
}

void Memory::updateNeighborLink(int fromId, int toId, const Transform & transform)
{
	Signature * fromS = this->_getSignature(fromId);
	Signature * toS = this->_getSignature(toId);

	if(fromS->hasNeighbor(toId) && toS->hasNeighbor(fromId))
	{
		fromS->removeNeighbor(toId);
		toS->removeNeighbor(fromId);

		fromS->addNeighbor(toId, transform);
		toS->addNeighbor(fromId, transform.inverse());
	}
	else
	{
		UERROR("fromId=%d and toId=%d are not neighbors!", fromId, toId);
	}
}

void Memory::dumpMemory(std::string directory) const
{
	UINFO("Dumping memory to directory \"%s\"", directory.c_str());
	this->dumpDictionary((directory+"DumpMemoryWordRef.txt").c_str(), (directory+"DumpMemoryWordDesc.txt").c_str());
	this->dumpSignatures((directory + "DumpMemorySign.txt").c_str(), false);
	this->dumpSignatures((directory + "DumpMemorySign3.txt").c_str(), true);
	this->dumpMemoryTree((directory + "DumpMemoryTree.txt").c_str());
}

void Memory::dumpDictionary(const char * fileNameRef, const char * fileNameDesc) const
{
	if(_vwd)
	{
		_vwd->exportDictionary(fileNameRef, fileNameDesc);
	}
}

void Memory::dumpSignatures(const char * fileNameSign, bool words3D) const
{
	FILE* foutSign = 0;
#ifdef _MSC_VER
	fopen_s(&foutSign, fileNameSign, "w");
#else
	foutSign = fopen(fileNameSign, "w");
#endif

	if(foutSign)
	{
		if(words3D)
		{
			fprintf(foutSign, "SignatureID WordsID... (Max features depth=%f)\n", _bowMaxDepth);
		}
		else
		{
			fprintf(foutSign, "SignatureID WordsID...\n");
		}
		const std::map<int, Signature *> & signatures = this->getSignatures();
		for(std::map<int, Signature *>::const_iterator iter=signatures.begin(); iter!=signatures.end(); ++iter)
		{
			fprintf(foutSign, "%d ", iter->first);
			const Signature * ss = dynamic_cast<const Signature *>(iter->second);
			if(ss)
			{
				if(words3D)
				{
					const std::multimap<int, pcl::PointXYZ> & ref = ss->getWords3();
					for(std::multimap<int, pcl::PointXYZ>::const_iterator jter=ref.begin(); jter!=ref.end(); ++jter)
					{
						//show only valid point according to current parameters
						if(pcl::isFinite(jter->second) &&
						   (jter->second.x != 0 || jter->second.y != 0 || jter->second.z != 0) &&
						   (_bowMaxDepth <= 0 || jter->second.x <= _bowMaxDepth))
						{
							fprintf(foutSign, "%d ", (*jter).first);
						}
					}
				}
				else
				{
					const std::multimap<int, cv::KeyPoint> & ref = ss->getWords();
					for(std::multimap<int, cv::KeyPoint>::const_iterator jter=ref.begin(); jter!=ref.end(); ++jter)
					{
						fprintf(foutSign, "%d ", (*jter).first);
					}
				}
			}
			fprintf(foutSign, "\n");
		}
		fclose(foutSign);
	}
}

void Memory::dumpMemoryTree(const char * fileNameTree) const
{
	FILE* foutTree = 0;
	#ifdef _MSC_VER
		fopen_s(&foutTree, fileNameTree, "w");
	#else
		foutTree = fopen(fileNameTree, "w");
	#endif

	if(foutTree)
	{
		fprintf(foutTree, "SignatureID Weight NbLoopClosureIds LoopClosureIds... NbChildLoopClosureIds ChildLoopClosureIds...\n");

		for(std::map<int, Signature *>::const_iterator i=_signatures.begin(); i!=_signatures.end(); ++i)
		{
			fprintf(foutTree, "%d %d", i->first, i->second->getWeight());

			const std::map<int, Transform> & loopIds = i->second->getLoopClosureIds();
			fprintf(foutTree, " %d", (int)loopIds.size());
			for(std::map<int, Transform>::const_iterator j=loopIds.begin(); j!=loopIds.end(); ++j)
			{
				fprintf(foutTree, " %d", j->first);
			}

			const std::map<int, Transform> & childIds = i->second->getChildLoopClosureIds();
			fprintf(foutTree, " %d", (int)childIds.size());
			for(std::map<int, Transform>::const_iterator j=childIds.begin(); j!=childIds.end(); ++j)
			{
				fprintf(foutTree, " %d", j->first);
			}

			fprintf(foutTree, "\n");
		}

		fclose(foutTree);
	}

}

void Memory::rehearsal(Signature * signature, Statistics * stats)
{
	UTimer timer;
	if(signature->getNeighbors().size() != 1)
	{
		return;
	}

	//============================================================
	// Compare with the last
	//============================================================
	int id = signature->getNeighbors().begin()->first;
	UDEBUG("Comparing with last signature (%d)...", id);
	const Signature * sB = this->getSignature(id);
	if(!sB)
	{
		UFATAL("Signature %d null?!?", id);
	}
	float sim = signature->compareTo(sB);

	int merged = 0;
	if(sim >= _similarityThreshold)
	{
		if(_incrementalMemory)
		{
			if(this->rehearsalMerge(id, signature->id()))
			{
				merged = id;
			}
		}
		else
		{
			signature->setWeight(signature->getWeight() + 1 + sB->getWeight());
		}
	}

	if(stats) stats->addStatistic(Statistics::kMemoryRehearsal_merged(), merged);
	if(stats) stats->addStatistic(Statistics::kMemoryRehearsal_sim(), sim);

	UDEBUG("merged=%d, sim=%f t=%fs", merged, sim, timer.ticks());
}

bool Memory::rehearsalMerge(int oldId, int newId)
{
	ULOGGER_INFO("old=%d, new=%d", oldId, newId);
	Signature * oldS = _getSignature(oldId);
	Signature * newS = _getSignature(newId);
	if(oldS && newS && _incrementalMemory)
	{
		const std::map<int, Transform> & oldLoopclosureIds = oldS->getLoopClosureIds();
		if(oldLoopclosureIds.size() && oldLoopclosureIds.find(newS->id()) != oldLoopclosureIds.end())
		{
			// do nothing, already merged
			UWARN("already merged, old=%d, new=%d", oldId, newId);
			return false;
		}
		UASSERT(!newS->isSaved());

		UDEBUG("Rehearsal merge %d and %d", oldS->id(), newS->id());

		// During rehearsal in STM
		if(_idUpdatedToNewOneRehearsal)
		{
			// update weight
			newS->setWeight(newS->getWeight() + 1 + oldS->getWeight());

			oldS->addLoopClosureId(newS->id()); // to keep track of the merged location

			if(_lastGlobalLoopClosureParentId == oldS->id())
			{
				_lastGlobalLoopClosureParentId = newS->id();
			}
		}
		else
		{
			// update weight
			oldS->setWeight(newS->getWeight() + 1 + oldS->getWeight());

			newS->addLoopClosureId(oldS->id()); // to keep track of the merged location

			if(_lastSignature == newS)
			{
				_lastSignature = oldS;
			}
		}

		if(_idUpdatedToNewOneRehearsal)
		{
			// redirect neighbor links
			const std::map<int, Transform> & neighbors = oldS->getNeighbors();
			for(std::map<int, Transform>::const_iterator iter = neighbors.begin(); iter!=neighbors.end(); ++iter)
			{
				int link = iter->first;
				Transform t = iter->second;
				if(link != newS->id() && link != oldS->id())
				{
					Signature * s = this->_getSignature(link);
					if(s)
					{
						// modify neighbor "from"
						s->changeNeighborIds(oldS->id(), newS->id());

						if(!newS->hasNeighbor(link))
						{
							newS->addNeighbor(link, t);
						}
					}
					else
					{
						UERROR("Didn't find neighbor %d of %d in RAM...", link, oldS->id());
					}
				}
			}
			oldS->removeNeighbors();

			// redirect child loop closure links
			std::map<int, Transform> childIds = oldS->getChildLoopClosureIds();
			for(std::map<int, Transform>::const_iterator iter = childIds.begin(); iter!=childIds.end(); ++iter)
			{
				if(iter->first == newS->id())
				{
					UERROR("");
				}
				newS->addChildLoopClosureId(iter->first, iter->second);
				Signature * s = _getSignature(iter->first);
				if(s)
				{
					UDEBUG("changed child Loop closure %d from %d to %d", iter->first, oldS->id(), newS->id());
					s->changeLoopClosureId(oldS->id(), newS->id());
				}
				else
				{
					UERROR("A location (%d, child of %d) in WM/STM cannot be transferred if its loop closure id is in STM", iter->first, oldS->id());
				}
				oldS->removeChildLoopClosureId(iter->first);
			}

			// Set old image to new signature
			this->copyData(oldS, newS);
		}

		//remove mutual links
		oldS->removeNeighbor(newId);
		newS->removeNeighbor(oldId);

		// remove location
		bool saveToDb = _keepRehearsedNodesInDb;
		moveToTrash(_idUpdatedToNewOneRehearsal?oldS:newS, saveToDb);

		return true;
	}
	else
	{
		if(!newS)
		{
			UERROR("newId=%d, oldId=%d, Signature %d not found in working/st memories", newId, oldId, newId);
		}
		if(!oldS)
		{
			UERROR("newId=%d, oldId=%d, Signature %d not found in working/st memories", newId, oldId, oldId);
		}
	}
	return false;
}

std::vector<unsigned char> Memory::getImage(int signatureId) const
{
	std::vector<unsigned char> image;
	const Signature * s = this->getSignature(signatureId);
	if(s)
	{
		image = s->getImage();
	}
	if(image.empty() && this->isRawDataKept() && _dbDriver)
	{
		_dbDriver->getNodeData(signatureId, image);
	}
	return image;
}

void Memory::getImageDepth(
		int locationId,
		std::vector<unsigned char> & rgb,
		std::vector<unsigned char> & depth,
		std::vector<unsigned char> & depth2d,
		float & depthConstant,
		Transform & localTransform) const
{
	const Signature * s = this->getSignature(locationId);
	if(s)
	{
		rgb = s->getImage();
		depth = s->getDepth();
		depth2d = s->getDepth2D();
		depthConstant = s->getDepthConstant();
		localTransform = s->getLocalTransform();
	}
	if(rgb.empty() && this->isRawDataKept() && _dbDriver)
	{
		_dbDriver->getNodeData(locationId, rgb, depth, depth2d, depthConstant, localTransform);
	}
}

void Memory::generateGraph(const std::string & fileName, std::set<int> ids)
{
	if(!_dbDriver)
	{
		UERROR("A database must must loaded first...");
		return;
	}

	if(!fileName.empty())
	{
		FILE* fout = 0;
		#ifdef _MSC_VER
			fopen_s(&fout, fileName.c_str(), "w");
		#else
			fout = fopen(fileName.c_str(), "w");
		#endif

		 if (!fout)
		 {
			 UERROR("Cannot open file %s!", fileName.c_str());
			 return;
		 }

		 if(ids.size() == 0)
		 {
			 _dbDriver->getAllNodeIds(ids);
			 UDEBUG("ids.size()=%d", ids.size());
			 for(std::map<int, Signature*>::iterator iter=_signatures.begin(); iter!=_signatures.end(); ++iter)
			 {
				 ids.insert(iter->first);
			 }
		 }

		 const char * colorG = "green";
		 const char * colorP = "pink";
;		 UINFO("Generating map with %d locations", ids.size());
		 fprintf(fout, "digraph G {\n");
		 for(std::set<int>::iterator i=ids.begin(); i!=ids.end(); ++i)
		 {
			 if(_signatures.find(*i) == _signatures.end())
			 {
				 int id = *i;
				 std::map<int, Transform> loopIds;
				 std::map<int, Transform> childIds;
				 _dbDriver->loadLoopClosures(id, loopIds, childIds);

				 std::map<int, Transform> neighbors;
				 _dbDriver->loadNeighbors(id, neighbors);
				 int weight = 0;
				 _dbDriver->getWeight(id, weight);
				 for(std::map<int, Transform>::iterator iter = neighbors.begin(); iter!=neighbors.end(); ++iter)
				 {
					 if(id!=iter->first)
					 {
						 int weightNeighbor = 0;
						 if(_signatures.find(iter->first) == _signatures.end())
						 {
							 _dbDriver->getWeight(iter->first, weightNeighbor);
						 }
						 else
						 {
							 weightNeighbor = _signatures.find(iter->first)->second->getWeight();
						 }
						 //UDEBUG("Add neighbor link from %d to %d", id, iter->first);
						 fprintf(fout, "   \"%d\\n%d\" -> \"%d\\n%d\"\n",
								 id,
								 weight,
								 iter->first,
								 weightNeighbor);
					 }
				 }

				 // loop closure links...
				 for(std::map<int, Transform>::iterator iter = loopIds.begin(); iter!=loopIds.end(); ++iter)
				 {
					 int weightNeighbor = 0;
					 if(_signatures.find(iter->first) == _signatures.end())
					 {
						 _dbDriver->getWeight(iter->first, weightNeighbor);
					 }
					 else
					 {
						 weightNeighbor = _signatures.find(iter->first)->second->getWeight();
					 }
					 //UDEBUG("Add loop link from %d to %d", id, iter->first);
					 fprintf(fout, "   \"%d\\n%d\" -> \"%d\\n%d\" [label=\"L\", fontcolor=%s, fontsize=8];\n",
							 id,
							 weight,
							 iter->first,
							 weightNeighbor,
							 colorG);
				 }
				 for(std::map<int, Transform>::iterator iter = childIds.begin(); iter!=childIds.end(); ++iter)
				 {
					 int weightNeighbor = 0;
					 if(_signatures.find(iter->first) == _signatures.end())
					 {
						 _dbDriver->getWeight(iter->first, weightNeighbor);
					 }
					 else
					 {
						 weightNeighbor = _signatures.find(iter->first)->second->getWeight();
					 }
					 //UDEBUG("Add child link from %d to %d", id, iter->first);
					 fprintf(fout, "   \"%d\\n%d\" -> \"%d\\n%d\" [label=\"C\", fontcolor=%s, fontsize=8];\n",
							 id,
							 weight,
							 iter->first,
							 weightNeighbor,
							 colorP);
				 }
			 }
		 }
		 for(std::map<int, Signature*>::iterator i=_signatures.begin(); i!=_signatures.end(); ++i)
		 {
			 if(ids.find(i->first) != ids.end())
			 {
				 int id = i->second->id();
				 const std::map<int, Transform> & loopIds = i->second->getLoopClosureIds();
				 //don't show children when _loopClosuresMerged is on
				 //if(!_loopClosuresMerged || (loopIds.size() == 0))
				 {
					 const std::map<int, Transform> & neighbors = i->second->getNeighbors();
					 int weight = i->second->getWeight();
					 for(std::map<int, Transform>::const_iterator iter = neighbors.begin(); iter!=neighbors.end(); ++iter)
					 {
						 if(id != iter->first)
						 {
							 int weightNeighbor = 0;
							 const Signature * s = this->getSignature(iter->first);
							 if(s)
							 {
								 weightNeighbor = s->getWeight();
							 }
							 else
							 {
								 _dbDriver->getWeight(iter->first, weightNeighbor);
							 }
							 //UDEBUG("Add neighbor link from %d to %d", id, iter->first);
							 fprintf(fout, "   \"%d\\n%d\" -> \"%d\\n%d\";\n",
									 id,
									 weight,
									 iter->first,
									 weightNeighbor);
						 }
					 }

					 // loop closure link
					 for(std::map<int, Transform>::const_iterator iter = loopIds.begin(); iter!=loopIds.end(); ++iter)
					 {
						 int weightNeighbor = 0;
						 if(_signatures.find(iter->first) == _signatures.end())
						 {
							 _dbDriver->getWeight(iter->first, weightNeighbor);
						 }
						 else
						 {
							 weightNeighbor = _signatures.find(iter->first)->second->getWeight();
						 }
						 //UDEBUG("Add loop link from %d to %d", id, iter->first);
						 fprintf(fout, "   \"%d\\n%d\" -> \"%d\\n%d\" [label=\"L\", fontcolor=%s, fontsize=8];\n",
								 id,
								 weight,
								 iter->first,
								 weightNeighbor,
								 colorG);
					 }

					 // child loop closure link
					 const std::map<int, Transform> & childIds = i->second->getChildLoopClosureIds();
					 for(std::map<int, Transform>::const_iterator iter = childIds.begin(); iter!=childIds.end(); ++iter)
					 {
						 int weightNeighbor = 0;
						 if(_signatures.find(iter->first) == _signatures.end())
						 {
							 _dbDriver->getWeight(iter->first, weightNeighbor);
						 }
						 else
						 {
							 weightNeighbor = _signatures.find(iter->first)->second->getWeight();
						 }
						 //UDEBUG("Add child link from %d to %d", id, iter->first);
						 fprintf(fout, "   \"%d\\n%d\" -> \"%d\\n%d\" [label=\"C\", fontcolor=%s, fontsize=8];\n",
								 id,
								 weight,
								 iter->first,
								 weightNeighbor,
								 colorP);
					 }
				 }
			 }
		 }
		 fprintf(fout, "}\n");
		 fclose(fout);
		 UINFO("Graph saved to \"%s\"", fileName.c_str());
	}
}

// Only used to generate a .dot file
class GraphNode
{
public:
	GraphNode(int id, GraphNode * parent = 0) :
		_parent(parent),
		_id(id)
	{
		if(_parent)
		{
			_parent->addChild(this);
		}
	}
	virtual ~GraphNode()
	{
		//We copy the set because when a child is destroyed, it is removed from its parent.
		std::set<GraphNode*> children = _children;
		_children.clear();
		for(std::set<GraphNode*>::iterator iter=children.begin(); iter!=children.end(); ++iter)
		{
			delete *iter;
		}
		children.clear();
		if(_parent)
		{
			_parent->removeChild(this);
		}
	}
	int id() const {return _id;}
	bool isAncestor(int id) const
	{
		if(_parent)
		{
			if(_parent->id() == id)
			{
				return true;
			}
			return _parent->isAncestor(id);
		}
		return false;
	}

	void expand(std::list<std::list<int> > & paths, std::list<int> currentPath = std::list<int>()) const
	{
		currentPath.push_back(_id);
		if(_children.size() == 0)
		{
			paths.push_back(currentPath);
			return;
		}
		for(std::set<GraphNode*>::const_iterator iter=_children.begin(); iter!=_children.end(); ++iter)
		{
			(*iter)->expand(paths, currentPath);
		}
	}

private:
	void addChild(GraphNode * child)
	{
		_children.insert(child);
	}
	void removeChild(GraphNode * child)
	{
		_children.erase(child);
	}

private:
	std::set<GraphNode*> _children;
	GraphNode * _parent;
	int _id;
};

//recursive
void Memory::createGraph(GraphNode * parent, unsigned int maxDepth, const std::set<int> & endIds)
{
	if(maxDepth == 0 || !parent)
	{
		return;
	}
	std::map<int, int> neighbors = this->getNeighborsId(parent->id(), 1, -1, false);
	for(std::map<int, int>::iterator iter=neighbors.begin(); iter!=neighbors.end(); ++iter)
	{
		if(!parent->isAncestor(iter->first))
		{
			GraphNode * n = new GraphNode(iter->first, parent);
			if(endIds.find(iter->first) == endIds.end())
			{
				this->createGraph(n, maxDepth-1, endIds);
			}
		}
	}
}

// Keypoint stuff
std::multimap<int, cv::KeyPoint> Memory::getWords(int signatureId) const
{
	std::multimap<int, cv::KeyPoint> words;
	if(signatureId>0)
	{
		const Signature * s = this->getSignature(signatureId);
		if(s)
		{
			const Signature * ks = dynamic_cast<const Signature*>(s);
			if(ks)
			{
				words = ks->getWords();
			}
		}
		else if(_dbDriver)
		{
			std::list<int> ids;
			ids.push_back(signatureId);
			std::list<Signature *> signatures;
			_dbDriver->loadSignatures(ids, signatures);
			if(signatures.size())
			{
				const Signature * ks = dynamic_cast<const Signature*>(signatures.front());
				if(ks)
				{
					words = ks->getWords();
				}
			}
			for(std::list<Signature *>::iterator iter = signatures.begin(); iter!=signatures.end(); ++iter)
			{
				delete *iter;
			}
		}
	}
	return words;
}

int Memory::getNi(int signatureId) const
{
	int ni = 0;
	const Signature * s = this->getSignature(signatureId);
	if(s) // Must be a SurfSignature
	{
		ni = ((Signature *)s)->getWords().size();
	}
	else
	{
		_dbDriver->getInvertedIndexNi(signatureId, ni);
	}
	return ni;
}


void Memory::copyData(const Signature * from, Signature * to)
{
	// The signatures must be KeypointSignature
	UTimer timer;
	timer.start();
	if(from && to)
	{
		// words 2d
		this->disableWordsRef(to->id());
		to->setWords(from->getWords());
		std::list<int> id;
		id.push_back(to->id());
		this->enableWordsRef(id);

		if(from->isSaved() && _dbDriver)
		{
			std::vector<unsigned char> image;
			std::vector<unsigned char> depth;
			std::vector<unsigned char> depth2d;
			float depthConstant;
			Transform localTransform;
			_dbDriver->getNodeData(from->id(), image, depth, depth2d, depthConstant, localTransform);

			to->setImage(image);
			to->setDepth(depth, depthConstant);
			to->setDepth2D(depth2d);
			to->setLocalTransform(localTransform);

			UDEBUG("Loaded image data from database");
		}
		else
		{
			to->setImage(from->getImage());
			to->setDepth(from->getDepth(), from->getDepthConstant());
			to->setDepth2D(from->getDepth2D());
			to->setLocalTransform(from->getLocalTransform());
		}

		to->setPose(from->getPose());
		to->setWords3(from->getWords3());
	}
	else
	{
		ULOGGER_ERROR("Can't merge the signatures because there are not same type.");
	}
	UDEBUG("Merging time = %fs", timer.ticks());
}

void Memory::extractKeypointsAndDescriptors(
		const cv::Mat & image,
		const cv::Mat & depth,
		float depthConstant,
		std::vector<cv::KeyPoint> & keypoints,
		cv::Mat & descriptors)
{
	if(_wordsPerImageTarget >= 0)
	{
		UTimer timer;
		if(_keypointDetector)
		{
			cv::Rect roi = KeypointDetector::computeRoi(image, _roiRatios);
			keypoints = _keypointDetector->generateKeypoints(image, 0, roi);
			UDEBUG("time keypoints (%d) = %fs", (int)keypoints.size(), timer.ticks());

			filterKeypointsByDepth(keypoints, depth, depthConstant, _wordsMaxDepth);
			limitKeypoints(keypoints, _wordsPerImageTarget);
		}

		if(keypoints.size())
		{
			descriptors = _keypointDescriptor->generateDescriptors(image, keypoints);
			UDEBUG("time descriptors (%d) = %fs", descriptors.rows, timer.ticks());
		}
	}
	else
	{
		UDEBUG("_wordsPerImageTarget(%d)<0 so don't extract any descriptors...", _wordsPerImageTarget);
	}
}

class PreUpdateThread : public UThreadNode
{
public:
	PreUpdateThread(VWDictionary * vwp) : _vwp(vwp) {}
	virtual ~PreUpdateThread() {}
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

Signature * Memory::createSignature(const Image & image, bool keepRawData)
{
	UASSERT(image.image().empty() || image.image().type() == CV_8UC1 || image.image().type() == CV_8UC3);
	UASSERT(image.depth().empty() || image.depth().type() == CV_16UC1);
	UASSERT(image.depth2d().empty() || image.depth2d().type() == CV_32FC2);

	PreUpdateThread preUpdateThread(_vwd);

	UTimer timer;
	timer.start();
	std::vector<cv::KeyPoint> keypoints;
	cv::Mat descriptors;
	int id = image.id();
	if(_generateIds)
	{
		id = this->getNextId();
	}
	else
	{
		if(id <= 0)
		{
			UWARN("Received image ID is null. "
				  "Please set parameter Mem/GenerateIds to \"true\" or "
				  "make sure the input source provides image ids (seq).");
			return 0;
		}
		else if(id > _idCount)
		{
			_idCount = id;
		}
		else
		{
			UWARN("Id of acquired image (%d) is smaller than the last in memory (%d). "
				  "Please set parameter Mem/GenerateIds to \"true\" or "
				  "make sure the input source provides image ids (seq) over the last in "
				  "memory, which is %d.",
					id,
					_idCount,
					_idCount);
			return 0;
		}
	}

	int treeSize= _workingMem.size() + _stMem.size();
	int meanWordsPerLocation = 0;
	if(treeSize > 0)
	{
		meanWordsPerLocation = _vwd->getTotalActiveReferences() / treeSize;
	}

	if(_parallelized)
	{
		preUpdateThread.start();
	}

	if(!image.descriptors().empty())
	{
		// DESCRIPTORS
		if(image.descriptors().rows && image.descriptors().rows >= _badSignRatio * float(meanWordsPerLocation))
		{
			UASSERT(image.descriptors().type() == CV_32F);
			descriptors = image.descriptors();
			keypoints = image.keypoints();
		}
		filterKeypointsByDepth(keypoints, descriptors, image.depth(), image.depthConstant(), _wordsMaxDepth);
		limitKeypoints(keypoints, descriptors, _wordsPerImageTarget);
	}
	else
	{
		// IMAGE RAW
		this->extractKeypointsAndDescriptors(image.image(), image.depth(), image.depthConstant(), keypoints, descriptors);

		UDEBUG("ratio=%f, meanWordsPerLocation=%d", _badSignRatio, meanWordsPerLocation);
		if(descriptors.rows && descriptors.rows < _badSignRatio * float(meanWordsPerLocation))
		{
			descriptors = cv::Mat();
		}
	}

	if(_parallelized)
	{
		preUpdateThread.join(); // Wait the dictionary to be updated
	}

	std::list<int> wordIds;
	if(descriptors.rows)
	{
		if(_parallelized)
		{
			UDEBUG("time descriptor and memory update (%d of size=%d) = %fs", descriptors.rows, descriptors.cols, timer.ticks());
		}
		else
		{
			UDEBUG("time descriptor (%d of size=%d) = %fs", descriptors.rows, descriptors.cols, timer.ticks());
		}

		wordIds = _vwd->addNewWords(descriptors, id);
		UDEBUG("time addNewWords %fs", timer.ticks());
	}
	else if(id>0)
	{
		UDEBUG("id %d is a bad signature", id);
	}

	std::multimap<int, cv::KeyPoint> words;
	if(wordIds.size() > 0)
	{
		std::vector<cv::KeyPoint>::iterator kpIter = keypoints.begin();
		for(std::list<int>::iterator iter=wordIds.begin(); iter!=wordIds.end(); ++iter)
		{
			if(kpIter != keypoints.end())
			{
				words.insert(std::pair<int, cv::KeyPoint >(*iter, *kpIter));
				++kpIter;
			}
			else
			{
				if(keypoints.size())
				{
					UWARN("Words (%d) and keypoints(%d) are not the same size ?!?", (int)wordIds.size(), (int)keypoints.size());
				}
				words.insert(std::pair<int, cv::KeyPoint >(*iter, cv::KeyPoint()));
			}
		}
	}

	//3d words
	std::multimap<int, pcl::PointXYZ> words3;
	if(!image.depth().empty() && image.depthConstant())
	{
		words3 = util3d::generateWords3(words, image.depth(), image.depthConstant(), image.localTransform());
	}

	Signature * s;
	if(keepRawData)
	{
		std::vector<unsigned char> imageBytes;
		std::vector<unsigned char> depthBytes;
		util3d::CompressionThread ctImage(image.image(), std::string(".jpg"));
		util3d::CompressionThread ctDepth(image.depth(), std::string(".png"));
		ctImage.start();
		ctDepth.start();
		ctImage.join();
		ctDepth.join();
		imageBytes = ctImage.getCompressedData();
		depthBytes = ctDepth.getCompressedData();

		s = new Signature(id,
			_idMapCount,
			words,
			words3,
			image.pose(),
			util3d::compressData(image.depth2d()),
			imageBytes,
			depthBytes,
			image.depthConstant(),
			image.localTransform());
	}
	else
	{
		s = new Signature(id,
			_idMapCount,
			words,
			words3,
			image.pose(),
			util3d::compressData(image.depth2d()));
	}



	UDEBUG("time new signature (id=%d) %fs", id, timer.ticks());
	if(words.size())
	{
		s->setEnabled(true); // All references are already activated in the dictionary at this point (see _vwd->addNewWords())
	}
	return s;
}

void Memory::disableWordsRef(int signatureId, bool saveToDatabase)
{
	UDEBUG("id=%d", signatureId);

	Signature * ss = dynamic_cast<Signature *>(this->_getSignature(signatureId));
	if(ss && ss->isEnabled())
	{
		const std::multimap<int, cv::KeyPoint> & words = ss->getWords();
		const std::list<int> & keys = uUniqueKeys(words);
		int count = _vwd->getTotalActiveReferences();
		// First remove all references
		for(std::list<int>::const_iterator i=keys.begin(); i!=keys.end(); ++i)
		{
			_vwd->removeAllWordRef(*i, signatureId);
			if(!saveToDatabase)
			{
				// assume just removed word doesn't have any other references
				VisualWord * w = _vwd->getUnusedWord(*i);
				if(w)
				{
					std::vector<VisualWord*> wordToDelete;
					wordToDelete.push_back(w);
					_vwd->removeWords(wordToDelete);
					delete w;
				}
			}
		}

		count -= _vwd->getTotalActiveReferences();
		ss->setEnabled(false);
		UDEBUG("%d words total ref removed from signature %d... (total active ref = %d)", count, ss->id(), _vwd->getTotalActiveReferences());
	}
}

void Memory::cleanUnusedWords()
{
	if(_vwd->isIncremental())
	{
		std::vector<VisualWord*> removedWords = _vwd->getUnusedWords();
		UDEBUG("Removing %d words (dictionary size=%d)...", removedWords.size(), _vwd->getVisualWords().size());
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
	}
}

void Memory::enableWordsRef(const std::list<int> & signatureIds)
{
	UDEBUG("size=%d", signatureIds.size());
	UTimer timer;
	timer.start();

	std::map<int, int> refsToChange; //<oldWordId, activeWordId>

	std::set<int> oldWordIds;
	std::list<Signature *> surfSigns;
	for(std::list<int>::const_iterator i=signatureIds.begin(); i!=signatureIds.end(); ++i)
	{
		Signature * ss = dynamic_cast<Signature *>(this->_getSignature(*i));
		if(ss && !ss->isEnabled())
		{
			surfSigns.push_back(ss);
			std::list<int> uniqueKeys = uUniqueKeys(ss->getWords());

			//Find words in the signature which they are not in the current dictionary
			for(std::list<int>::const_iterator k=uniqueKeys.begin(); k!=uniqueKeys.end(); ++k)
			{
				if(_vwd->getWord(*k) == 0 && _vwd->getUnusedWord(*k) == 0)
				{
					oldWordIds.insert(oldWordIds.end(), *k);
				}
			}
		}
	}

	UDEBUG("oldWordIds.size()=%d, getOldIds time=%fs", oldWordIds.size(), timer.ticks());

	// the words were deleted, so try to math it with an active word
	std::list<VisualWord *> vws;
	if(oldWordIds.size() && _dbDriver)
	{
		// get the descriptors
		_dbDriver->loadWords(oldWordIds, vws);
	}
	UDEBUG("loading words(%d) time=%fs", oldWordIds.size(), timer.ticks());


	if(vws.size())
	{
		//Search in the dictionary
		bool reactivatedWordsComparedToNewWords = true;
		std::vector<int> vwActiveIds = _vwd->findNN(vws, reactivatedWordsComparedToNewWords);
		UDEBUG("find active ids (number=%d) time=%fs", vws.size(), timer.ticks());
		int i=0;
		for(std::list<VisualWord *>::iterator iterVws=vws.begin(); iterVws!=vws.end(); ++iterVws)
		{
			if(vwActiveIds[i] > 0)
			{
				//UDEBUG("Match found %d with %d", (*iterVws)->id(), vwActiveIds[i]);
				refsToChange.insert(refsToChange.end(), std::pair<int, int>((*iterVws)->id(), vwActiveIds[i]));
				if((*iterVws)->isSaved())
				{
					delete (*iterVws);
				}
				else if(_dbDriver)
				{
					_dbDriver->asyncSave(*iterVws);
				}
			}
			else
			{
				//add to dictionary
				_vwd->addWord(*iterVws); // take ownership
			}
			++i;
		}
		UDEBUG("Added %d to dictionary, time=%fs", vws.size()-refsToChange.size(), timer.ticks());

		//update the global references map and update the signatures reactivated
		for(std::map<int, int>::const_iterator iter=refsToChange.begin(); iter != refsToChange.end(); ++iter)
		{
			//uInsert(_wordRefsToChange, (const std::pair<int, int>)*iter); // This will be used to change references in the database
			for(std::list<Signature *>::iterator j=surfSigns.begin(); j!=surfSigns.end(); ++j)
			{
				(*j)->changeWordsRef(iter->first, iter->second);
			}
		}
		UDEBUG("changing ref, total=%d, time=%fs", refsToChange.size(), timer.ticks());
	}

	int count = _vwd->getTotalActiveReferences();

	// Reactivate references and signatures
	for(std::list<Signature *>::iterator j=surfSigns.begin(); j!=surfSigns.end(); ++j)
	{
		const std::vector<int> & keys = uKeys((*j)->getWords());
		// Add all references
		for(std::vector<int>::const_iterator i=keys.begin(); i!=keys.end(); ++i)
		{
			_vwd->addWordRef(*i, (*j)->id());
		}
		if(keys.size())
		{
			(*j)->setEnabled(true);
		}
	}

	count = _vwd->getTotalActiveReferences() - count;
	UDEBUG("%d words total ref added from %d signatures, time=%fs...", count, surfSigns.size(), timer.ticks());
}

std::set<int> Memory::reactivateSignatures(const std::list<int> & ids, unsigned int maxLoaded, double & timeDbAccess)
{
	// get the signatures, if not in the working memory, they
	// will be loaded from the database in an more efficient way
	// than how it is done in the Memory

	UDEBUG("");
	UTimer timer;
	std::list<int> idsToLoad;
	std::map<int, int>::iterator wmIter;
	for(std::list<int>::const_iterator i=ids.begin(); i!=ids.end(); ++i)
	{
		if(!this->getSignature(*i) && !uContains(idsToLoad, *i))
		{
			if(!maxLoaded || idsToLoad.size() < maxLoaded)
			{
				idsToLoad.push_back(*i);
				UINFO("Loading location %d from database...", *i);
			}
		}
	}

	UDEBUG("idsToLoad = %d", idsToLoad.size());

	std::list<Signature *> reactivatedSigns;
	if(_dbDriver)
	{
		_dbDriver->loadSignatures(idsToLoad, reactivatedSigns);
	}
	timeDbAccess = timer.getElapsedTime();
	std::list<int> idsLoaded;
	for(std::list<Signature *>::iterator i=reactivatedSigns.begin(); i!=reactivatedSigns.end(); ++i)
	{
		idsLoaded.push_back((*i)->id());
		//append to working memory
		this->addSignatureToWm(*i);
	}
	this->enableWordsRef(idsLoaded);
	UDEBUG("time = %fs", timer.ticks());
	return std::set<int>(idsToLoad.begin(), idsToLoad.end());
}

void Memory::getMetricConstraints(
		const std::vector<int> & ids,
		std::map<int, Transform> & poses,
		std::multimap<int, Link> & links,
		bool lookInDatabase)
{
	for(unsigned int i=0; i<ids.size(); ++i)
	{
		Transform pose;
		this->getPose(ids[i], pose, lookInDatabase);
		if(!pose.isNull())
		{
			poses.insert(std::make_pair(ids[i], pose));
		}
	}

	for(unsigned int i=0; i<ids.size(); ++i)
	{
		if(uContains(poses, ids[i]))
		{
			std::map<int, Transform> neighbors = this->getNeighborLinks(ids[i], true, lookInDatabase); // only direct neighbors
			for(std::map<int, Transform>::iterator jter=neighbors.begin(); jter!=neighbors.end(); ++jter)
			{
				if(!jter->second.isNull() && uContains(poses, jter->first))
				{
					bool edgeAlreadyAdded = false;
					for(std::multimap<int, Link>::iterator iter = links.lower_bound(jter->first);
							iter != links.end() && iter->first == jter->first;
							++iter)
					{
						if(iter->second.to() == ids[i])
						{
							edgeAlreadyAdded = true;
						}
					}
					if(!edgeAlreadyAdded)
					{
						links.insert(std::make_pair(ids[i], Link(ids[i], jter->first, jter->second, Link::kNeighbor)));
					}
				}
			}

			std::map<int, Transform> loops, children;
			this->getLoopClosureIds(ids[i], loops, children, lookInDatabase);
			for(std::map<int, Transform>::iterator jter=children.begin(); jter!=children.end(); ++jter)
			{
				if(!jter->second.isNull() && uContains(poses, jter->first))
				{
					links.insert(std::make_pair(ids[i], Link(ids[i], jter->first, jter->second, Link::kGlobalClosure)));
				}
			}
		}
	}
}

} // namespace rtabmap
