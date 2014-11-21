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
#include <pcl/common/common.h>

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
	_postInitClosingEvents(false),

	_featureType((Feature2D::Type)Parameters::defaultKpDetectorStrategy()),
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
	_bowForce2D(Parameters::defaultLccBowForce2D()),

	_icpDecimation(Parameters::defaultLccIcp3Decimation()),
	_icpMaxDepth(Parameters::defaultLccIcp3MaxDepth()),
	_icpVoxelSize(Parameters::defaultLccIcp3VoxelSize()),
	_icpSamples(Parameters::defaultLccIcp3Samples()),
	_icpMaxCorrespondenceDistance(Parameters::defaultLccIcp3MaxCorrespondenceDistance()),
	_icpMaxIterations(Parameters::defaultLccIcp3Iterations()),
	_icpMaxFitness(Parameters::defaultLccIcp3MaxFitness()),
	_icpPointToPlane(Parameters::defaultLccIcp3PointToPlane()),
	_icpPointToPlaneNormalNeighbors(Parameters::defaultLccIcp3PointToPlaneNormalNeighbors()),

	_icp2MaxCorrespondenceDistance(Parameters::defaultLccIcp2MaxCorrespondenceDistance()),
	_icp2MaxIterations(Parameters::defaultLccIcp2Iterations()),
	_icp2MaxFitness(Parameters::defaultLccIcp2MaxFitness()),
	_icp2CorrespondenceRatio(Parameters::defaultLccIcp2CorrespondenceRatio()),
	_icp2VoxelSize(Parameters::defaultLccIcp2VoxelSize()),

	_stereoFlowWinSize(Parameters::defaultStereoWinSize()),
	_stereoFlowIterations(Parameters::defaultStereoIterations()),
	_stereoFlowEpsilon(Parameters::defaultStereoEps()),
	_stereoFlowMaxLevel(Parameters::defaultStereoMaxLevel()),
	_stereoMaxSlope(Parameters::defaultStereoMaxSlope()),

	_subPixWinSize(Parameters::defaultKpSubPixWinSize()),
	_subPixIterations(Parameters::defaultKpSubPixIterations()),
	_subPixEps(Parameters::defaultKpSubPixEps())
{
	_feature2D = Feature2D::create(_featureType, parameters);
	_vwd = new VWDictionary(parameters);
	this->parseParameters(parameters);
}

bool Memory::init(const std::string & dbUrl, bool dbOverwritten, const ParametersMap & parameters, bool postInitClosingEvents)
{
	_postInitClosingEvents = postInitClosingEvents;
	if(_postInitClosingEvents) UEventsManager::post(new RtabmapEventInit(RtabmapEventInit::kInitializing));

	UDEBUG("");
	this->parseParameters(parameters);
	bool loadAllNodesInWM = Parameters::defaultMemInitWMWithAllNodes();
	Parameters::parse(parameters, Parameters::kMemInitWMWithAllNodes(), loadAllNodesInWM);

	if(_postInitClosingEvents) UEventsManager::post(new RtabmapEventInit("Clearing memory..."));
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
	if(_postInitClosingEvents) UEventsManager::post(new RtabmapEventInit("Clearing memory, done!"));

	if(tmpDriver)
	{
		_dbDriver = tmpDriver;
	}

	if(_dbDriver)
	{
		if(_postInitClosingEvents) UEventsManager::post(new RtabmapEventInit("Closing database connection..."));
		_dbDriver->closeConnection();
		if(_postInitClosingEvents) UEventsManager::post(new RtabmapEventInit("Closing database connection, done!"));
	}

	if(_dbDriver == 0 && !dbUrl.empty())
	{
		_dbDriver = new DBDriverSqlite3(parameters);
	}

	bool success = true;
	if(_dbDriver)
	{
		success = false;
		if(_postInitClosingEvents) UEventsManager::post(new RtabmapEventInit(std::string("Connecting to database ") + dbUrl + "..."));
		if(_dbDriver->openConnection(dbUrl, dbOverwritten))
		{
			success = true;
			if(_postInitClosingEvents) UEventsManager::post(new RtabmapEventInit(std::string("Connecting to database ") + dbUrl + ", done!"));

			// Load the last working memory...
			std::list<Signature*> dbSignatures;

			if(loadAllNodesInWM)
			{
				if(_postInitClosingEvents) UEventsManager::post(new RtabmapEventInit(std::string("Loading all nodes to WM...")));
				std::set<int> ids;
				_dbDriver->getAllNodeIds(ids, true);
				_dbDriver->loadSignatures(std::list<int>(ids.begin(), ids.end()), dbSignatures);
			}
			else
			{
				// load previous session working memory
				if(_postInitClosingEvents) UEventsManager::post(new RtabmapEventInit(std::string("Loading last nodes to WM...")));
				_dbDriver->loadLastNodes(dbSignatures);
			}
			for(std::list<Signature*>::reverse_iterator iter=dbSignatures.rbegin(); iter!=dbSignatures.rend(); ++iter)
			{
				// ignore bad signatures
				if(!((*iter)->isBadSignature() && _badSignaturesIgnored))
				{
					_signatures.insert(std::pair<int, Signature *>((*iter)->id(), *iter));
					if(_maxStMemSize == 0 || (int)_stMem.size() <= _maxStMemSize)
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
			if(_postInitClosingEvents) UEventsManager::post(new RtabmapEventInit(std::string("Loading nodes to WM, done! (") + uNumber2Str(int(_workingMem.size() + _stMem.size())) + " loaded)"));

			// Assign the last signature
			if(_stMem.size()>0)
			{
				_lastSignature = uValue(_signatures, *_stMem.rbegin(), (Signature*)0);
			}
			else if(_workingMem.size()>0)
			{
				_lastSignature = uValue(_signatures, *_workingMem.rbegin(), (Signature*)0);
			}

			// Last id
			_dbDriver->getLastNodeId(_idCount);
			_idMapCount = _lastSignature?_lastSignature->mapId()+1:kIdStart;
		}
		else
		{
			if(_postInitClosingEvents) UEventsManager::post(new RtabmapEventInit(RtabmapEventInit::kError, std::string("Connecting to database ") + dbUrl + ", path is invalid!"));
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
		if(_postInitClosingEvents) UEventsManager::post(new RtabmapEventInit("Loading dictionary..."));
		if(loadAllNodesInWM)
		{
			// load all referenced words in working memory
			std::set<int> wordIds;
			const std::map<int, Signature *> & signatures = this->getSignatures();
			for(std::map<int, Signature *>::const_iterator i=signatures.begin(); i!=signatures.end(); ++i)
			{
				const std::multimap<int, cv::KeyPoint> & words = i->second->getWords();
				std::list<int> keys = uUniqueKeys(words);
				wordIds.insert(keys.begin(), keys.end());
			}
			if(wordIds.size())
			{
				std::list<VisualWord*> words;
				_dbDriver->loadWords(wordIds, words);
				for(std::list<VisualWord*>::iterator iter = words.begin(); iter!=words.end(); ++iter)
				{
					_vwd->addWord(*iter);
				}
				// Get Last word id
				int id = 0;
				_dbDriver->getLastWordId(id);
				_vwd->setLastWordId(id);
			}
		}
		else
		{
			// load the last dictionary
			_dbDriver->load(_vwd);
		}
		UDEBUG("%d words loaded!", _vwd->getUnusedWordsSize());
		_vwd->update();
		if(_postInitClosingEvents) UEventsManager::post(new RtabmapEventInit(uFormat("Loading dictionary, done! (%d words)", (int)_vwd->getUnusedWordsSize())));
	}

	if(_postInitClosingEvents) UEventsManager::post(new RtabmapEventInit(std::string("Adding word references...")));
	// Enable loaded signatures
	const std::map<int, Signature *> & signatures = this->getSignatures();
	for(std::map<int, Signature *>::const_iterator i=signatures.begin(); i!=signatures.end(); ++i)
	{
		Signature * s = this->_getSignature(i->first);
		UASSERT(s != 0);

		const std::multimap<int, cv::KeyPoint> & words = s->getWords();
		if(words.size())
		{
			UDEBUG("node=%d, word references=%d", s->id(), words.size());
			for(std::multimap<int, cv::KeyPoint>::const_iterator iter = words.begin(); iter!=words.end(); ++iter)
			{
				_vwd->addWordRef(iter->first, i->first);
			}
			s->setEnabled(true);
		}
	}
	if(_postInitClosingEvents) UEventsManager::post(new RtabmapEventInit(uFormat("Adding word references, done! (%d)", _vwd->getTotalActiveReferences())));

	if(_vwd->getUnusedWordsSize())
	{
		UWARN("_vwd->getUnusedWordsSize() must be empty... size=%d", _vwd->getUnusedWordsSize());
	}
	UDEBUG("Total word references added = %d", _vwd->getTotalActiveReferences());

	if(_postInitClosingEvents) UEventsManager::post(new RtabmapEventInit(RtabmapEventInit::kInitialized));
	return success;
}

Memory::~Memory()
{
	if(_postInitClosingEvents) UEventsManager::post(new RtabmapEventInit(RtabmapEventInit::kClosing));
	UDEBUG("");
	if(!_memoryChanged)
	{
		UDEBUG("");
		if(_dbDriver)
		{
			if(_postInitClosingEvents) UEventsManager::post(new RtabmapEventInit(uFormat("Closing database \"%s\"...", _dbDriver->getUrl().c_str())));
			_dbDriver->closeConnection();
			delete _dbDriver;
			_dbDriver = 0;
			if(_postInitClosingEvents) UEventsManager::post(new RtabmapEventInit("Closing database, done!"));
		}
		if(_postInitClosingEvents) UEventsManager::post(new RtabmapEventInit("Clearing memory..."));
		this->clear();
		if(_postInitClosingEvents) UEventsManager::post(new RtabmapEventInit("Clearing memory, done!"));
	}
	else
	{
		UDEBUG("");
		if(_postInitClosingEvents) UEventsManager::post(new RtabmapEventInit("Saving memory..."));
		this->clear();
		if(_dbDriver)
		{
			_dbDriver->emptyTrashes();
			if(_postInitClosingEvents) UEventsManager::post(new RtabmapEventInit("Saving memory, done!"));
			if(_postInitClosingEvents) UEventsManager::post(new RtabmapEventInit(uFormat("Closing database \"%s\"...", _dbDriver->getUrl().c_str())));
			_dbDriver->closeConnection();
			delete _dbDriver;
			_dbDriver = 0;
			if(_postInitClosingEvents) UEventsManager::post(new RtabmapEventInit("Closing database, done!"));
		}
		else
		{
			if(_postInitClosingEvents) UEventsManager::post(new RtabmapEventInit("Saving memory, done!"));
		}
	}

	if(_feature2D)
	{
		delete _feature2D;
	}
	if(_vwd)
	{
		delete _vwd;
	}
	if(_postInitClosingEvents) UEventsManager::post(new RtabmapEventInit(RtabmapEventInit::kClosed));
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

	UASSERT_MSG(_maxStMemSize >= 0, uFormat("value=%d", _maxStMemSize).c_str());
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
	Parameters::parse(parameters, Parameters::kLccBowForce2D(), _bowForce2D);
	Parameters::parse(parameters, Parameters::kLccIcp3Decimation(), _icpDecimation);
	Parameters::parse(parameters, Parameters::kLccIcp3MaxDepth(), _icpMaxDepth);
	Parameters::parse(parameters, Parameters::kLccIcp3VoxelSize(), _icpVoxelSize);
	Parameters::parse(parameters, Parameters::kLccIcp3Samples(), _icpSamples);
	Parameters::parse(parameters, Parameters::kLccIcp3MaxCorrespondenceDistance(), _icpMaxCorrespondenceDistance);
	Parameters::parse(parameters, Parameters::kLccIcp3Iterations(), _icpMaxIterations);
	Parameters::parse(parameters, Parameters::kLccIcp3MaxFitness(), _icpMaxFitness);
	Parameters::parse(parameters, Parameters::kLccIcp3PointToPlane(), _icpPointToPlane);
	Parameters::parse(parameters, Parameters::kLccIcp3PointToPlaneNormalNeighbors(), _icpPointToPlaneNormalNeighbors);
	Parameters::parse(parameters, Parameters::kLccIcp2MaxCorrespondenceDistance(), _icp2MaxCorrespondenceDistance);
	Parameters::parse(parameters, Parameters::kLccIcp2Iterations(), _icp2MaxIterations);
	Parameters::parse(parameters, Parameters::kLccIcp2MaxFitness(), _icp2MaxFitness);
	Parameters::parse(parameters, Parameters::kLccIcp2CorrespondenceRatio(), _icp2CorrespondenceRatio);
	Parameters::parse(parameters, Parameters::kLccIcp2VoxelSize(), _icp2VoxelSize);

	//stereo
	Parameters::parse(parameters, Parameters::kStereoWinSize(), _stereoFlowWinSize);
	Parameters::parse(parameters, Parameters::kStereoIterations(), _stereoFlowIterations);
	Parameters::parse(parameters, Parameters::kStereoEps(), _stereoFlowEpsilon);
	Parameters::parse(parameters, Parameters::kStereoMaxLevel(), _stereoFlowMaxLevel);
	Parameters::parse(parameters, Parameters::kStereoMaxSlope(), _stereoMaxSlope);

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
	UASSERT_MSG(_icpPointToPlaneNormalNeighbors > 0, uFormat("value=%d", _icpPointToPlaneNormalNeighbors).c_str());
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

	Parameters::parse(parameters, Parameters::kKpSubPixWinSize(), _subPixWinSize);
	Parameters::parse(parameters, Parameters::kKpSubPixIterations(), _subPixIterations);
	Parameters::parse(parameters, Parameters::kKpSubPixEps(), _subPixEps);

	if((iter=parameters.find(Parameters::kKpRoiRatios())) != parameters.end())
	{
		this->setRoi((*iter).second);
	}

	//Keypoint detector
	UASSERT(_feature2D != 0);
	Feature2D::Type detectorStrategy = Feature2D::kFeatureUndef;
	if((iter=parameters.find(Parameters::kKpDetectorStrategy())) != parameters.end())
	{
		detectorStrategy = (Feature2D::Type)std::atoi((*iter).second.c_str());
	}
	if(detectorStrategy!=Feature2D::kFeatureUndef)
	{
		UDEBUG("new detector strategy %d", int(detectorStrategy));
		if(_feature2D)
		{
			delete _feature2D;
			_feature2D = 0;
			_featureType = Feature2D::kFeatureUndef;
		}

		_feature2D = Feature2D::create(detectorStrategy, parameters);
		_featureType = detectorStrategy;
	}
	else if(_feature2D)
	{
		_feature2D->parseParameters(parameters);
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

bool Memory::update(const SensorData & data, Statistics * stats)
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
	Signature * signature = this->createSignature(data, this->isRawDataKept(), stats);
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
		if(_similarityThreshold < 1.0f)
		{
			this->rehearsal(signature, stats);
		}
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
	while(_stMem.size() && _maxStMemSize>0 && (int)_stMem.size() > _maxStMemSize)
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

const VWDictionary * Memory::getVWDictionary() const
{
	return _vwd;
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
		int margin, // 0 means infinite margin
		int maxCheckedInDatabase, // default -1 (no limit)
		bool incrementMarginOnLoop, // default false
		bool ignoreLoopIds, // default false
		double * dbAccessTime
		) const
{
	UASSERT(margin >= 0);
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
	int m = 0;
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
			if(ids.find(*jter) == ids.end())
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
					ids.insert(std::pair<int, int>(*jter, m));

					neighborIds = &s->getNeighbors();

					if(!ignoreLoopIds)
					{
						loopClosureIds = &s->getLoopClosureIds();
						childLoopClosureIds = &s->getChildLoopClosureIds();
					}
				}
				else if(maxCheckedInDatabase == -1 || (maxCheckedInDatabase > 0 && _dbDriver && nbLoadedFromDb < maxCheckedInDatabase))
				{
					ids.insert(std::pair<int, int>(*jter, m));

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
				(int)_vwd->getVisualWords().size());
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
				sim = signature->compareTo(*sB);
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
	if(_lastSignature && ((_lastSignature->isBadSignature() && _badSignaturesIgnored) || !_incrementalMemory))
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

/**
 * If saveToDatabase=false, deleted words are filled in deletedWords.
 */
void Memory::moveToTrash(Signature * s, bool saveToDatabase, std::list<int> * deletedWords)
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
					if(iter->first > s->id() && (n->getNeighbors().size() > 2 || !n->hasNeighbor(s->id())))
					{
						UWARN("Neighbor %d of %d is newer, removing neighbor link may split the map!",
								iter->first, s->id());
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

		this->disableWordsRef(s->id());
		if(!saveToDatabase)
		{
			std::list<int> keys = uUniqueKeys(s->getWords());
			for(std::list<int>::const_iterator i=keys.begin(); i!=keys.end(); ++i)
			{
				// assume just removed word doesn't have any other references
				VisualWord * w = _vwd->getUnusedWord(*i);
				if(w)
				{
					std::vector<VisualWord*> wordToDelete;
					wordToDelete.push_back(w);
					_vwd->removeWords(wordToDelete);
					if(deletedWords)
					{
						deletedWords->push_back(w->id());
					}
					delete w;
				}
			}
		}

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
			else if(_workingMem.size())
			{
				_lastSignature = this->_getSignature(*_workingMem.rbegin());
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

void Memory::deleteLocation(int locationId, std::list<int> * deletedWords)
{
	UDEBUG("Deleting location %d", locationId);
	Signature * location = _getSignature(locationId);
	if(location)
	{
		this->moveToTrash(location, false, deletedWords);
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
Transform Memory::computeVisualTransform(int oldId, int newId, std::string * rejectedMsg, int * inliers) const
{
	const Signature * oldS = this->getSignature(oldId);
	const Signature * newS = this->getSignature(newId);

	Transform transform;

	if(oldS && newId)
	{
		return computeVisualTransform(*oldS, *newS, rejectedMsg, inliers);
	}
	else
	{
		std::string msg = uFormat("Did not find nodes %d and/or %d", oldId, newId);
		if(rejectedMsg)
		{
			*rejectedMsg = msg;
		}
		UWARN(msg.c_str());
	}
	return Transform();
}

// compute transform newId -> oldId
Transform Memory::computeVisualTransform(const Signature & oldS, const Signature & newS, std::string * rejectedMsg, int * inliers) const
{
	Transform transform;
	std::string msg;
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
			std::vector<int> inliersV;
			Transform t = util3d::transformFromXYZCorrespondences(
					inliersOld,
					inliersNew,
					_bowInlierDistance,
					_bowIterations,
					true, 3.0, 10,
					&inliersV);
			inliersCount = inliersV.size();
			if(!t.isNull() && inliersCount >= _bowMinInliers)
			{
				transform = t;
				if(_bowForce2D)
				{
					UDEBUG("Forcing 2D...");
					float x,y,z,r,p,yaw;
					transform.getTranslationAndEulerAngles(x,y,z, r,p,yaw);
					transform = util3d::transformFromEigen3f(pcl::getTransformation(x,y,0, 0, 0, yaw));
				}
			}
			else if(inliersCount < _bowMinInliers)
			{
				msg = uFormat("Not enough inliers (after RANSAC) %d/%d between %d and %d", inliersCount, _bowMinInliers, oldS.id(), newS.id());
				UINFO(msg.c_str());
			}
			else if(inliersCount == (int)inliersOld->size())
			{
				msg = uFormat("Rejected identity with full inliers.");
				UINFO(msg.c_str());
			}

			if(inliers)
			{
				*inliers = inliersCount;
			}
		}
		else
		{
			msg = uFormat("Not enough inliers %d/%d between %d and %d", (int)inliersOld->size(), _bowMinInliers, oldS.id(), newS.id());
			UINFO(msg.c_str());
		}
	}
	else if(!oldS.isBadSignature() && !newS.isBadSignature())
	{
		msg = "Words 3D empty?!?";
		UERROR(msg.c_str());
	}

	if(rejectedMsg)
	{
		*rejectedMsg = msg;
	}

	return transform;
}

// compute transform newId -> oldId
Transform Memory::computeIcpTransform(int oldId, int newId, Transform guess, bool icp3D, std::string * rejectedMsg)
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
			if(oldS->getDepthCompressed().empty())
			{
				depthToLoad.push_back(oldS);
				added.insert(oldS->id());
			}
			if(newS->getDepthCompressed().empty())
			{
				depthToLoad.push_back(newS);
				added.insert(newS->id());
			}
		}
		else
		{
			//Depth required, if not in RAM, load it from LTM
			if(oldS->getDepth2DCompressed().empty() && added.find(oldS->id()) == added.end())
			{
				depthToLoad.push_back(oldS);
			}
			if(newS->getDepth2DCompressed().empty() && added.find(newS->id()) == added.end())
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
		//make sure data are uncompressed
		if(icp3D)
		{
			cv::Mat tmp1, tmp2;
			oldS->uncompressData(0, &tmp1, 0);
			newS->uncompressData(0, &tmp2, 0);
		}
		else
		{
			cv::Mat tmp1, tmp2;
			oldS->uncompressData(0, 0, &tmp1);
			newS->uncompressData(0, 0, &tmp2);
		}

		t = computeIcpTransform(*oldS, *newS, guess, icp3D, rejectedMsg);
	}
	else
	{
		std::string msg = uFormat("Did not find nodes %d and/or %d", oldId, newId);
		if(rejectedMsg)
		{
			*rejectedMsg = msg;
		}
		UWARN(msg.c_str());
	}
	return t;
}

// get transform from the new to old node
Transform Memory::computeIcpTransform(const Signature & oldS, const Signature & newS, Transform guess, bool icp3D, std::string * rejectedMsg) const
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

	std::string msg;
	Transform transform;

	// ICP with guess transform
	if(icp3D)
	{
		UDEBUG("3D ICP");
		if(!oldS.getDepthRaw().empty() && !newS.getDepthRaw().empty())
		{
			if(oldS.getDepthRaw().type() == CV_8UC1 || newS.getDepthRaw().type() == CV_8UC1)
			{
				UERROR("ICP 3D cannot be done on stereo images!");
			}
			else
			{
				pcl::PointCloud<pcl::PointXYZ>::Ptr oldCloudXYZ = util3d::getICPReadyCloud(
						oldS.getDepthRaw(),
						oldS.getDepthFx(),
						oldS.getDepthFy(),
						oldS.getDepthCx(),
						oldS.getDepthCy(),
						_icpDecimation,
						_icpMaxDepth,
						_icpVoxelSize,
						_icpSamples,
						oldS.getLocalTransform());
				pcl::PointCloud<pcl::PointXYZ>::Ptr newCloudXYZ = util3d::getICPReadyCloud(
						newS.getDepthRaw(),
						newS.getDepthFx(),
						newS.getDepthFy(),
						newS.getDepthCx(),
						newS.getDepthCy(),
						_icpDecimation,
						_icpMaxDepth,
						_icpVoxelSize,
						_icpSamples,
						guess * newS.getLocalTransform());

				// 3D
				double fitness = 0;
				bool hasConverged = false;
				Transform icpT;
				if(newCloudXYZ->size() && oldCloudXYZ->size())
				{
					if(_icpPointToPlane)
					{
						pcl::PointCloud<pcl::PointNormal>::Ptr oldCloud = util3d::computeNormals(oldCloudXYZ, _icpPointToPlaneNormalNeighbors);
						pcl::PointCloud<pcl::PointNormal>::Ptr newCloud = util3d::computeNormals(newCloudXYZ, _icpPointToPlaneNormalNeighbors);

						std::vector<int> indices;
						newCloud = util3d::removeNaNNormalsFromPointCloud<pcl::PointNormal>(newCloud);
						oldCloud = util3d::removeNaNNormalsFromPointCloud<pcl::PointNormal>(oldCloud);

						if(newCloud->size() && oldCloud->size())
						{
							icpT = util3d::icpPointToPlane(newCloud,
									oldCloud,
								   _icpMaxCorrespondenceDistance,
								   _icpMaxIterations,
								   hasConverged,
								   fitness);
						}
					}
					else
					{
						transform = util3d::icp(newCloudXYZ,
								oldCloudXYZ,
								_icpMaxCorrespondenceDistance,
								_icpMaxIterations,
								hasConverged,
								fitness);
					}

					//pcl::io::savePCDFile("old.pcd", *oldCloudXYZ);
					//pcl::io::savePCDFile("newguess.pcd", *newCloudXYZ);
					//newCloudXYZ = util3d::transformPointCloud(newCloudXYZ, icpT);
					//pcl::io::savePCDFile("newicp.pcd", *newCloudXYZ);

					UDEBUG("fitness=%f", fitness);

					if(!icpT.isNull() && hasConverged && (_icpMaxFitness == 0 || fitness < _icpMaxFitness))
					{
						transform = icpT * guess;
						transform = transform.inverse();
					}
					else
					{
						msg = uFormat("Cannot compute transform (hasConverged=%s fitness=%f/%f)",
								hasConverged?"true":"false", fitness, _icpMaxFitness);
						UINFO(msg.c_str());
					}
				}
				else
				{
					msg = "Clouds empty ?!?";
					UWARN(msg.c_str());
				}
			}
		}
		else
		{
			msg = "Depths 3D empty?!?";
			UERROR(msg.c_str());
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

		if(!oldS.getDepth2DRaw().empty() && !newS.getDepth2DRaw().empty())
		{
			// 2D
			pcl::PointCloud<pcl::PointXYZ>::Ptr oldCloud = util3d::cvMat2Cloud(oldS.getDepth2DRaw());
			pcl::PointCloud<pcl::PointXYZ>::Ptr newCloud = util3d::cvMat2Cloud(newS.getDepth2DRaw(), guess);

			//voxelize
			if(_icp2VoxelSize > 0.0f)
			{
				oldCloud = util3d::voxelize<pcl::PointXYZ>(oldCloud, _icp2VoxelSize);
				newCloud = util3d::voxelize<pcl::PointXYZ>(newCloud, _icp2VoxelSize);
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

				//UWARN("saving ICP2D clouds!");
				//pcl::io::savePCDFile("lccold.pcd", *oldCloud);
				//pcl::io::savePCDFile("lccnewguess.pcd", *newCloud);
				newCloud = util3d::transformPointCloud<pcl::PointXYZ>(newCloud, icpT);
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

				if(!icpT.isNull() && hasConverged &&
				   (_icp2MaxFitness == 0 || fitness < _icp2MaxFitness) &&
				   correspondencesRatio >= _icp2CorrespondenceRatio)
				{
					transform = icpT * guess;
					transform = transform.inverse();
				}
				else
				{
					msg = uFormat("Cannot compute transform (hasConverged=%s fitness=%f/%f correspondencesRatio=%f/%f)",
							hasConverged?"true":"false", fitness, _icpMaxFitness, correspondencesRatio, _icp2CorrespondenceRatio);
					UINFO(msg.c_str());
				}
			}
			else
			{
				msg = "Clouds 2D empty ?!?";
				UWARN(msg.c_str());
			}
		}
		else
		{
			msg = "Depths 2D empty?!?";
			UERROR(msg.c_str());
		}
	}

	if(rejectedMsg)
	{
		*rejectedMsg = msg;
	}

	UDEBUG("New transform = %s", transform.prettyPrint().c_str());
	return transform;
}

// poses of newId and oldId must be in "poses"
Transform Memory::computeScanMatchingTransform(
		int newId,
		int oldId,
		const std::map<int, Transform> & poses,
		std::string * rejectedMsg)
{
	// make sure that all depth2D are loaded
	std::list<Signature*> depthToLoad;
	for(std::map<int, Transform>::const_iterator iter = poses.begin(); iter!=poses.end(); ++iter)
	{
		Signature * s = _getSignature(iter->first);
		UASSERT(s != 0);
		if(s->getDepth2DCompressed().empty())
		{
			depthToLoad.push_back(s);
		}
	}
	if(depthToLoad.size() && _dbDriver)
	{
		_dbDriver->loadNodeData(depthToLoad, true);
	}

	std::string msg;
	pcl::PointCloud<pcl::PointXYZ>::Ptr assembledOldClouds(new pcl::PointCloud<pcl::PointXYZ>);
	for(std::map<int, Transform>::const_iterator iter = poses.begin(); iter!=poses.end(); ++iter)
	{
		if(iter->first != newId)
		{
			const Signature * s = this->getSignature(iter->first);
			if(!s->getDepth2DCompressed().empty())
			{
				*assembledOldClouds += *util3d::cvMat2Cloud(util3d::uncompressData(s->getDepth2DCompressed()), iter->second);
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
		assembledOldClouds = util3d::voxelize<pcl::PointXYZ>(assembledOldClouds, _icp2VoxelSize);
	}

	// get the new cloud
	const Signature * newS = getSignature(newId);
	pcl::PointCloud<pcl::PointXYZ>::Ptr newCloud;
	UASSERT(uContains(poses, newId));
	newCloud = util3d::cvMat2Cloud(util3d::uncompressData(newS->getDepth2DCompressed()), poses.at(newId));

	//voxelize
	if(newCloud->size() && _icp2VoxelSize > 0.0f)
	{
		newCloud = util3d::voxelize<pcl::PointXYZ>(newCloud, _icp2VoxelSize);
	}

	//UWARN("local scan matching pcd saved!");
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

		newCloud = util3d::transformPointCloud<pcl::PointXYZ>(newCloud, icpT);
		//pcl::io::savePCDFile("newCorrected.pcd", *newCloud);

		// verify if there enough correspondences
		int correspondences = util3d::getCorrespondencesCount(newCloud, assembledOldClouds, _icp2MaxCorrespondenceDistance);
		float correspondencesRatio = float(correspondences)/float(newCloud->size());

		UDEBUG("fitness=%f, correspondences=%d/%d (%f%%)",
				fitness,
				correspondences,
				(int)newCloud->size(),
				correspondencesRatio);

		if(!icpT.isNull() && hasConverged &&
		   (_icp2MaxFitness == 0 || fitness < _icp2MaxFitness) &&
		   correspondencesRatio >= _icp2CorrespondenceRatio)
		{
			transform = poses.at(newId).inverse()*icpT.inverse() * poses.at(oldId);

			//newCloud = util3d::cvMat2Cloud(util3d::uncompressData(newS->getDepth2DCompressed()), poses.at(oldId)*transform.inverse());
			//pcl::io::savePCDFile("newFinal.pcd", *newCloud);
		}
		else
		{
			msg = uFormat("Constraints failed... hasConverged=%s, fitness=%f, correspondences=%d/%d (%f%%)",
				hasConverged?"true":"false",
				fitness,
				correspondences,
				(int)newCloud->size(),
				correspondencesRatio);
			UINFO(msg.c_str());
		}
	}
	else
	{
		msg = "Empty data ?!?";
		UWARN(msg.c_str());
	}

	if(rejectedMsg)
	{
		*rejectedMsg = msg;
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
		_memoryChanged = true;
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
	float sim = signature->compareTo(*sB);

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

int Memory::getMapId(int signatureId) const
{
	int mapId = 0;
	const Signature * s = this->getSignature(signatureId);
	if(s)
	{
		mapId = s->mapId();
	}
	else if(_dbDriver)
	{
		Transform pose;
		_dbDriver->getPose(signatureId, pose, mapId);
	}
	return mapId;
}

cv::Mat Memory::getImageCompressed(int signatureId) const
{
	cv::Mat image;
	const Signature * s = this->getSignature(signatureId);
	if(s)
	{
		image = s->getImageCompressed();
	}
	if(image.empty() && this->isRawDataKept() && _dbDriver)
	{
		_dbDriver->getNodeData(signatureId, image);
	}
	return image;
}

Signature Memory::getSignatureData(int locationId, bool uncompressedData)
{
	UDEBUG("");
	Signature r;
	Signature * s = this->_getSignature(locationId);
	if(s && !s->getImageCompressed().empty())
	{
		r = *s;
	}
	else if(_dbDriver)
	{
		// load from database
		if(s)
		{
			std::list<Signature*> signatures;
			signatures.push_back(s);
			_dbDriver->loadNodeData(signatures, !s->getPose().isNull());
			r = *s;
		}
		else
		{
			std::list<int> ids;
			ids.push_back(locationId);
			std::list<Signature*> signatures;
			std::set<int> loadedFromTrash;
			_dbDriver->loadSignatures(ids, signatures, &loadedFromTrash);
			if(signatures.size())
			{
				Signature * sTmp = signatures.front();
				if(sTmp->getImageCompressed().empty())
				{
					_dbDriver->loadNodeData(signatures, !sTmp->getPose().isNull());
				}
				r = *sTmp;
				if(loadedFromTrash.size())
				{
					//put it back to trash
					_dbDriver->asyncSave(sTmp);
				}
				else
				{
					delete sTmp;
				}
			}
		}
	}
	UDEBUG("");

	if(uncompressedData && r.getImageRaw().empty() && !r.getImageCompressed().empty())
	{
		//uncompress data
		if(s)
		{
			s->uncompressData();
			r.setImageRaw(s->getImageRaw());
			r.setDepthRaw(s->getDepthRaw());
			r.setDepth2DRaw(s->getDepth2DRaw());
		}
		else
		{
			r.uncompressData();
		}
	}
	UDEBUG("");

	return r;
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

int Memory::getNi(int signatureId) const
{
	int ni = 0;
	const Signature * s = this->getSignature(signatureId);
	if(s)
	{
		ni = (int)((Signature *)s)->getWords().size();
	}
	else
	{
		_dbDriver->getInvertedIndexNi(signatureId, ni);
	}
	return ni;
}


void Memory::copyData(const Signature * from, Signature * to)
{
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
			cv::Mat image;
			cv::Mat depth;
			cv::Mat depth2d;
			float fx, fy, cx, cy;
			Transform localTransform;
			_dbDriver->getNodeData(from->id(), image, depth, depth2d, fx, fy, cx, cy, localTransform);

			to->setImageCompressed(image);
			to->setDepthCompressed(depth, fx, fy, cx, cy);
			to->setDepth2DCompressed(depth2d);
			to->setLocalTransform(localTransform);

			UDEBUG("Loaded image data from database");
		}
		else
		{
			to->setImageCompressed(from->getImageCompressed());
			to->setDepthCompressed(from->getDepthCompressed(), from->getDepthFx(), from->getDepthFy(), from->getDepthCx(), from->getDepthCy());
			to->setDepth2DCompressed(from->getDepth2DCompressed());
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

Signature * Memory::createSignature(const SensorData & data, bool keepRawData, Statistics * stats)
{
	UASSERT(data.image().empty() || data.image().type() == CV_8UC1 || data.image().type() == CV_8UC3);
	UASSERT(data.depth().empty() || data.depth().type() == CV_16UC1 || data.depth().type() == CV_32FC1);
	UASSERT(data.rightImage().empty() || data.rightImage().type() == CV_8UC1);
	UASSERT(data.depth2d().empty() || data.depth2d().type() == CV_32FC2);
	UASSERT(_feature2D != 0);

	PreUpdateThread preUpdateThread(_vwd);

	UTimer timer;
	timer.start();
	float t;
	std::vector<cv::KeyPoint> keypoints;
	cv::Mat descriptors;
	int id = data.id();
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

	int treeSize= int(_workingMem.size() + _stMem.size());
	int meanWordsPerLocation = 0;
	if(treeSize > 0)
	{
		meanWordsPerLocation = _vwd->getTotalActiveReferences() / treeSize;
	}

	if(_parallelized)
	{
		preUpdateThread.start();
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints3D(new pcl::PointCloud<pcl::PointXYZ>);
	if(data.keypoints().size() == 0)
	{
		if(_wordsPerImageTarget >= 0)
		{
			// Extract features
			cv::Mat imageMono;
			// convert to grayscale
			if(data.image().channels() > 1)
			{
				cv::cvtColor(data.image(), imageMono, cv::COLOR_BGR2GRAY);
			}
			else
			{
				imageMono = data.image();
			}
			cv::Rect roi = Feature2D::computeRoi(imageMono, _roiRatios);

			if(!data.rightImage().empty())
			{
				//stereo
				cv::Mat disparity;
				bool subPixelOn = false;
				if(_subPixWinSize > 0 && _subPixIterations > 0)
				{
					subPixelOn = true;
				}
				keypoints = _feature2D->generateKeypoints(imageMono, subPixelOn?_wordsPerImageTarget:0, roi);
				t = timer.ticks();
				if(stats) stats->addStatistic(Statistics::kTimingMemKeypoints_detection(), t*1000.0f);
				UDEBUG("time keypoints (%d) = %fs", (int)keypoints.size(), t);

				if(keypoints.size())
				{
					std::vector<cv::Point2f> leftCorners;
					if(subPixelOn)
					{
						// descriptors should be extracted before subpixel
						descriptors = _feature2D->generateDescriptors(imageMono, keypoints);
						t = timer.ticks();
						if(stats) stats->addStatistic(Statistics::kTimingMemDescriptors_extraction(), t*1000.0f);
						UDEBUG("time descriptors (%d) = %fs", descriptors.rows, t);

						cv::KeyPoint::convert(keypoints, leftCorners);
						cv::cornerSubPix( imageMono, leftCorners,
								cv::Size( _subPixWinSize, _subPixWinSize ),
								cv::Size( -1, -1 ),
								cv::TermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, _subPixIterations, _subPixEps ) );

						for(unsigned int i=0;i<leftCorners.size(); ++i)
						{
							keypoints[i].pt = leftCorners[i];
						}

						t = timer.ticks();
						if(stats) stats->addStatistic(Statistics::kTimingMemStereo_subpixel(), t*1000.0f);
						UDEBUG("time subpix left kpts=%fs", t);
					}
					else
					{
						cv::KeyPoint::convert(keypoints, leftCorners);
					}

					//generate a disparity map
					disparity = util3d::disparityFromStereoImages(
							imageMono,
							data.rightImage(),
							leftCorners,
							_stereoFlowWinSize,
							_stereoFlowMaxLevel,
							_stereoFlowIterations,
							_stereoFlowEpsilon,
							_stereoMaxSlope);
					t = timer.ticks();
					if(stats) stats->addStatistic(Statistics::kTimingMemStereo_correspondences(), t*1000.0f);
					UDEBUG("generate disparity = %fs", t);

					if(_wordsMaxDepth > 0.0f)
					{
						// disparity = baseline * fx / depth;
						float minDisparity = data.baseline() * data.fx() / _wordsMaxDepth;
						Feature2D::filterKeypointsByDisparity(keypoints, descriptors, disparity, minDisparity);
						UDEBUG("filter keypoints by disparity (%d)", (int)keypoints.size());
					}

					if(_wordsPerImageTarget && (int)keypoints.size() > _wordsPerImageTarget)
					{
						Feature2D::limitKeypoints(keypoints, descriptors, _wordsPerImageTarget);
						UDEBUG("limit keypoints max (%d)", _wordsPerImageTarget);
					}
					t = timer.ticks();
					if(stats) stats->addStatistic(Statistics::kTimingMemKeypoints_filtering(), t*1000.0f);
					UDEBUG("time keypoints filtering = %fs", _wordsPerImageTarget);


					if(keypoints.size())
					{
						if(!subPixelOn)
						{
							descriptors = _feature2D->generateDescriptors(imageMono, keypoints);
							t = timer.ticks();
							if(stats) stats->addStatistic(Statistics::kTimingMemDescriptors_extraction(), t*1000.0f);
							UDEBUG("time descriptors (%d) = %fs", descriptors.rows, t);
						}

						keypoints3D = util3d::generateKeypoints3DDisparity(keypoints, disparity, data.fx(), data.baseline(), data.cx(), data.cy(), data.localTransform());
						t = timer.ticks();
						if(stats) stats->addStatistic(Statistics::kTimingMemKeypoints_3D(), t*1000.0f);
						UDEBUG("time keypoints 3D (%d) = %fs", (int)keypoints3D->size(), t);
					}
				}
			}
			else if(!data.depth().empty())
			{
				//depth
				bool subPixelOn = false;
				if(_subPixWinSize > 0 && _subPixIterations > 0)
				{
					subPixelOn = true;
				}
				keypoints = _feature2D->generateKeypoints(imageMono, subPixelOn?_wordsPerImageTarget:0, roi);
				t = timer.ticks();
				if(stats) stats->addStatistic(Statistics::kTimingMemKeypoints_detection(), t*1000.0f);
				UDEBUG("time keypoints (%d) = %fs", (int)keypoints.size(), t);

				if(keypoints.size())
				{
					if(subPixelOn)
					{
						// descriptors should be extracted before subpixel
						descriptors = _feature2D->generateDescriptors(imageMono, keypoints);
						t = timer.ticks();
						if(stats) stats->addStatistic(Statistics::kTimingMemDescriptors_extraction(), t*1000.0f);
						UDEBUG("time descriptors (%d) = %fs", descriptors.rows, t);

						std::vector<cv::Point2f> leftCorners;
						cv::KeyPoint::convert(keypoints, leftCorners);
						cv::cornerSubPix( imageMono, leftCorners,
								cv::Size( _subPixWinSize, _subPixWinSize ),
								cv::Size( -1, -1 ),
								cv::TermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, _subPixIterations, _subPixEps ) );

						for(unsigned int i=0;i<leftCorners.size(); ++i)
						{
							keypoints[i].pt = leftCorners[i];
						}

						t = timer.ticks();
						if(stats) stats->addStatistic(Statistics::kTimingMemStereo_subpixel(), t*1000.0f);
						UDEBUG("time subpix left kpts=%fs", t);
					}

					if(_wordsMaxDepth > 0.0f)
					{
						Feature2D::filterKeypointsByDepth(keypoints, descriptors, data.depth(), _wordsMaxDepth);
						UDEBUG("filter keypoints by depth (%d)", (int)keypoints.size());
					}

					if(_wordsPerImageTarget && (int)keypoints.size() > _wordsPerImageTarget)
					{
						Feature2D::limitKeypoints(keypoints, descriptors, _wordsPerImageTarget);
						UDEBUG("limit keypoints max (%d)", _wordsPerImageTarget);
					}
					t = timer.ticks();
					if(stats) stats->addStatistic(Statistics::kTimingMemKeypoints_filtering(), t*1000.0f);
					UDEBUG("time keypoints filtering = %fs", _wordsPerImageTarget);

					if(keypoints.size())
					{
						if(!subPixelOn)
						{
							descriptors = _feature2D->generateDescriptors(imageMono, keypoints);
							t = timer.ticks();
							if(stats) stats->addStatistic(Statistics::kTimingMemDescriptors_extraction(), t*1000.0f);
							UDEBUG("time descriptors (%d) = %fs", descriptors.rows, t);
						}

						keypoints3D = util3d::generateKeypoints3DDepth(keypoints, data.depth(), data.fx(), data.fy(), data.cx(), data.cy(), data.localTransform());
						t = timer.ticks();
						if(stats) stats->addStatistic(Statistics::kTimingMemKeypoints_3D(), t*1000.0f);
						UDEBUG("time keypoints 3D (%d) = %fs", (int)keypoints3D->size(), t);
					}
				}
			}
			else
			{
				//RGB only
				keypoints = _feature2D->generateKeypoints(imageMono, _wordsPerImageTarget, roi);
				t = timer.ticks();
				if(stats) stats->addStatistic(Statistics::kTimingMemKeypoints_detection(), t*1000.0f);
				UDEBUG("time keypoints (%d) = %fs", (int)keypoints.size(), t);

				if(keypoints.size())
				{
					descriptors = _feature2D->generateDescriptors(imageMono, keypoints);
					t = timer.ticks();
					if(stats) stats->addStatistic(Statistics::kTimingMemDescriptors_extraction(), t*1000.0f);
					UDEBUG("time descriptors (%d) = %fs", descriptors.rows, t);
				}
			}

			UDEBUG("ratio=%f, meanWordsPerLocation=%d", _badSignRatio, meanWordsPerLocation);
			if(descriptors.rows && descriptors.rows < _badSignRatio * float(meanWordsPerLocation))
			{
				descriptors = cv::Mat();
			}
		}
		else
		{
			UDEBUG("_wordsPerImageTarget(%d)<0 so don't extract any descriptors...", _wordsPerImageTarget);
		}
	}
	else
	{
		keypoints = data.keypoints();
		descriptors = data.descriptors().clone();

		// filter by depth
		if(!data.rightImage().empty())
		{
			//stereo
			cv::Mat imageMono;
			// convert to grayscale
			if(data.image().channels() > 1)
			{
				cv::cvtColor(data.image(), imageMono, cv::COLOR_BGR2GRAY);
			}
			else
			{
				imageMono = data.image();
			}
			//generate a disparity map
			std::vector<cv::Point2f> leftCorners;
			cv::KeyPoint::convert(keypoints, leftCorners);
			cv::Mat disparity = util3d::disparityFromStereoImages(
					imageMono,
					data.rightImage(),
					leftCorners,
					_stereoFlowWinSize,
					_stereoFlowMaxLevel,
					_stereoFlowIterations,
					_stereoFlowEpsilon,
					_stereoMaxSlope);
			t = timer.ticks();
			if(stats) stats->addStatistic(Statistics::kTimingMemStereo_correspondences(), t*1000.0f);
			UDEBUG("generate disparity = %fs", t);

			if(_wordsMaxDepth)
			{
				// disparity = baseline * fx / depth;
				float minDisparity = data.baseline() * data.fx() / _wordsMaxDepth;
				Feature2D::filterKeypointsByDisparity(keypoints, descriptors, disparity, minDisparity);
			}

			if(_wordsPerImageTarget && (int)keypoints.size() > _wordsPerImageTarget)
			{
				Feature2D::limitKeypoints(keypoints, _wordsPerImageTarget);
				UDEBUG("limit keypoints max (%d)", _wordsPerImageTarget);
			}
			t = timer.ticks();
			if(stats) stats->addStatistic(Statistics::kTimingMemKeypoints_filtering(), t*1000.0f);
			UDEBUG("time keypoints filtering=%fs", t);

			keypoints3D = util3d::generateKeypoints3DDisparity(keypoints, disparity, data.fx(), data.baseline(), data.cx(), data.cy(), data.localTransform());
			t = timer.ticks();
			if(stats) stats->addStatistic(Statistics::kTimingMemKeypoints_3D(), t*1000.0f);
			UDEBUG("time keypoints 3D (%d) = %fs", (int)keypoints3D->size(), t);
		}
		else if(!data.depth().empty())
		{
			//depth
			if(_wordsMaxDepth)
			{
				Feature2D::filterKeypointsByDepth(keypoints, descriptors, _wordsMaxDepth);
				UDEBUG("filter keypoints by depth (%d)", (int)keypoints.size());
			}

			if(_wordsPerImageTarget && (int)keypoints.size() > _wordsPerImageTarget)
			{
				Feature2D::limitKeypoints(keypoints, _wordsPerImageTarget);
				UDEBUG("limit keypoints max (%d)", _wordsPerImageTarget);
			}
			t = timer.ticks();
			if(stats) stats->addStatistic(Statistics::kTimingMemKeypoints_filtering(), t*1000.0f);
			UDEBUG("time keypoints filtering=%fs", t);

			keypoints3D = util3d::generateKeypoints3DDepth(keypoints, data.depth(), data.fx(), data.fy(), data.cx(), data.cy(), data.localTransform());
			t = timer.ticks();
			if(stats) stats->addStatistic(Statistics::kTimingMemKeypoints_3D(), t*1000.0f);
			UDEBUG("time keypoints 3D (%d) = %fs", (int)keypoints3D->size(), t);
		}
		else
		{
			// RGB only
			Feature2D::limitKeypoints(keypoints, descriptors, _wordsPerImageTarget);
			t = timer.ticks();
			if(stats) stats->addStatistic(Statistics::kTimingMemKeypoints_filtering(), t*1000.0f);
			UDEBUG("time keypoints filtering=%fs", t);
		}
	}

	if(_parallelized)
	{
		preUpdateThread.join(); // Wait the dictionary to be updated
	}

	std::list<int> wordIds;
	if(descriptors.rows)
	{
		t = timer.ticks();
		if(stats) stats->addStatistic(Statistics::kTimingMemJoining_dictionary_update(), t*1000.0f);
		if(_parallelized)
		{
			UDEBUG("time descriptor and memory update (%d of size=%d) = %fs", descriptors.rows, descriptors.cols, t);
		}
		else
		{
			UDEBUG("time descriptor (%d of size=%d) = %fs", descriptors.rows, descriptors.cols, t);
		}

		wordIds = _vwd->addNewWords(descriptors, id);
		t = timer.ticks();
		if(stats) stats->addStatistic(Statistics::kTimingMemAdd_new_words(), t*1000.0f);
		UDEBUG("time addNewWords %fs", t);
	}
	else if(id>0)
	{
		UDEBUG("id %d is a bad signature", id);
	}

	std::multimap<int, cv::KeyPoint> words;
	std::multimap<int, pcl::PointXYZ> words3D;
	if(wordIds.size() > 0)
	{
		UASSERT(wordIds.size() == keypoints.size());
		UASSERT(keypoints3D->size() == 0 || keypoints3D->size() == wordIds.size());
		unsigned int i=0;
		for(std::list<int>::iterator iter=wordIds.begin(); iter!=wordIds.end() && i < keypoints.size(); ++iter, ++i)
		{
			words.insert(std::pair<int, cv::KeyPoint>(*iter, keypoints[i]));
			if(keypoints3D->size())
			{
				words3D.insert(std::pair<int, pcl::PointXYZ>(*iter, keypoints3D->at(i)));
			}
		}
	}

	Signature * s;
	if(keepRawData)
	{
		std::vector<unsigned char> imageBytes;
		std::vector<unsigned char> depthBytes;
		if(data.depth().type() == CV_32FC1)
		{
			UWARN("Keeping raw data in database: depth type is 32FC1, use 16UC1 depth format to avoid a conversion.");
		}
		cv::Mat depthOrRightImage;
		if(!data.depth().empty())
		{
			depthOrRightImage = data.depth().type() == CV_32FC1?util3d::cvtDepthFromFloat(data.depth()):data.depth();
		}
		else if(!data.rightImage().empty())
		{
			depthOrRightImage = data.rightImage();
		}
		util3d::CompressionThread ctImage(data.image(), std::string(".jpg"));
		util3d::CompressionThread ctDepth(depthOrRightImage, std::string(".png"));
		util3d::CompressionThread ctDepth2d(data.depth2d());
		ctImage.start();
		ctDepth.start();
		ctDepth2d.start();
		ctImage.join();
		ctDepth.join();
		ctDepth2d.join();

		s = new Signature(id,
			_idMapCount,
			words,
			words3D,
			data.pose(),
			ctDepth2d.getCompressedData(),
			ctImage.getCompressedData(),
			ctDepth.getCompressedData(),
			data.fx(),
			data.fy()>0.0f?data.fy():data.baseline(),
			data.cx(),
			data.cy(),
			data.localTransform());
		s->setImageRaw(data.image());
		s->setDepthRaw(depthOrRightImage);
		s->setDepth2DRaw(data.depth2d());
	}
	else
	{
		s = new Signature(id,
			_idMapCount,
			words,
			words3D,
			data.pose(),
			util3d::compressData2(data.depth2d()));
	}


	t = timer.ticks();
	if(stats) stats->addStatistic(Statistics::kTimingMemCompressing_data(), t*1000.0f);
	UDEBUG("time compressing data (id=%d) %fs", id, t);
	if(words.size())
	{
		s->setEnabled(true); // All references are already activated in the dictionary at this point (see _vwd->addNewWords())
	}
	return s;
}

void Memory::disableWordsRef(int signatureId)
{
	UDEBUG("id=%d", signatureId);

	Signature * ss = this->_getSignature(signatureId);
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
		std::vector<int> vwActiveIds = _vwd->findNN(vws);
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
