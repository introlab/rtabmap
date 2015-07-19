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
#include <rtabmap/core/EpipolarGeometry.h>
#include "VisualWord.h"
#include "rtabmap/core/Features2d.h"
#include "DBDriverSqlite3.h"
#include "rtabmap/core/util3d_features.h"
#include "rtabmap/core/util3d_filtering.h"
#include "rtabmap/core/util3d_correspondences.h"
#include "rtabmap/core/util3d_registration.h"
#include "rtabmap/core/util3d_surface.h"
#include "rtabmap/core/util3d_transforms.h"
#include "rtabmap/core/util3d_motion_estimation.h"
#include "rtabmap/core/util3d.h"
#include "rtabmap/core/util2d.h"
#include "rtabmap/core/Statistics.h"
#include "rtabmap/core/Compression.h"
#include "rtabmap/core/Graph.h"

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
	_binDataKept(Parameters::defaultMemBinDataKept()),
	_notLinkedNodesKeptInDb(Parameters::defaultMemNotLinkedNodesKept()),
	_incrementalMemory(Parameters::defaultMemIncrementalMemory()),
	_maxStMemSize(Parameters::defaultMemSTMSize()),
	_recentWmRatio(Parameters::defaultMemRecentWmRatio()),
	_transferSortingByWeightId(Parameters::defaultMemTransferSortingByWeightId()),
	_idUpdatedToNewOneRehearsal(Parameters::defaultMemRehearsalIdUpdatedToNewOne()),
	_generateIds(Parameters::defaultMemGenerateIds()),
	_badSignaturesIgnored(Parameters::defaultMemBadSignaturesIgnored()),
	_imageDecimation(Parameters::defaultMemImageDecimation()),
	_laserScanVoxelSize(Parameters::defaultMemLaserScanVoxelSize()),
	_localSpaceLinksKeptInWM(Parameters::defaultMemLocalSpaceLinksKeptInWM()),
	_rehearsalMaxDistance(Parameters::defaultRGBDLinearUpdate()),
	_rehearsalMaxAngle(Parameters::defaultRGBDAngularUpdate()),
	_rehearsalWeightIgnoredWhileMoving(Parameters::defaultMemRehearsalWeightIgnoredWhileMoving()),
	_idCount(kIdStart),
	_idMapCount(kIdStart),
	_lastSignature(0),
	_lastGlobalLoopClosureId(0),
	_memoryChanged(false),
	_linksChanged(false),
	_signaturesAdded(0),
	_postInitClosingEvents(false),

	_featureType((Feature2D::Type)Parameters::defaultKpDetectorStrategy()),
	_badSignRatio(Parameters::defaultKpBadSignRatio()),
	_tfIdfLikelihoodUsed(Parameters::defaultKpTfIdfLikelihoodUsed()),
	_parallelized(Parameters::defaultKpParallelized()),
	_wordsMaxDepth(Parameters::defaultKpMaxDepth()),
	_roiRatios(std::vector<float>(4, 0.0f)),

	_bowMinInliers(Parameters::defaultLccBowMinInliers()),
	_bowInlierDistance(Parameters::defaultLccBowInlierDistance()),
	_bowIterations(Parameters::defaultLccBowIterations()),
	_bowRefineIterations(Parameters::defaultLccBowRefineIterations()),
	_bowForce2D(Parameters::defaultLccBowForce2D()),
	_bowEpipolarGeometryVar(Parameters::defaultLccBowEpipolarGeometryVar()),
	_bowEstimationType(Parameters::defaultLccBowEstimationType()),
	_bowPnPReprojError(Parameters::defaultLccBowPnPReprojError()),
	_bowPnPFlags(Parameters::defaultLccBowPnPFlags()),

	_icpMaxTranslation(Parameters::defaultLccIcpMaxTranslation()),
	_icpMaxRotation(Parameters::defaultLccIcpMaxRotation()),

	_icpDecimation(Parameters::defaultLccIcp3Decimation()),
	_icpMaxDepth(Parameters::defaultLccIcp3MaxDepth()),
	_icpVoxelSize(Parameters::defaultLccIcp3VoxelSize()),
	_icpSamples(Parameters::defaultLccIcp3Samples()),
	_icpMaxCorrespondenceDistance(Parameters::defaultLccIcp3MaxCorrespondenceDistance()),
	_icpMaxIterations(Parameters::defaultLccIcp3Iterations()),
	_icpCorrespondenceRatio(Parameters::defaultLccIcp3CorrespondenceRatio()),
	_icpPointToPlane(Parameters::defaultLccIcp3PointToPlane()),
	_icpPointToPlaneNormalNeighbors(Parameters::defaultLccIcp3PointToPlaneNormalNeighbors()),

	_icp2MaxCorrespondenceDistance(Parameters::defaultLccIcp2MaxCorrespondenceDistance()),
	_icp2MaxIterations(Parameters::defaultLccIcp2Iterations()),
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
	if(!_memoryChanged && !_linksChanged)
	{
		if(_dbDriver)
		{
			tmpDriver = _dbDriver;
			_dbDriver = 0; // HACK for the clear() below to think that there is no db
		}
	}
	else if(!_memoryChanged && _linksChanged)
	{
		_dbDriver->setTimestampUpdateEnabled(false); // update links only
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
		_dbDriver->setTimestampUpdateEnabled(true); // make sure that timestamp update is enabled (may be disabled above)
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
					// insert all in WM
					// Note: it doesn't make sense to keep last STM images
					//       of the last session in the new STM because they can be
					//       only linked with the ones of the current session by
					//       global loop closures.
					_signatures.insert(std::pair<int, Signature *>((*iter)->id(), *iter));
					_workingMem.insert(std::make_pair((*iter)->id(), UTimer::now()));
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
				_lastSignature = uValue(_signatures, _workingMem.rbegin()->first, (Signature*)0);
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

	_workingMem.insert(std::make_pair(kIdVirtual, 0));

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

	if(!_memoryChanged && !_linksChanged)
	{
		if(_postInitClosingEvents) UEventsManager::post(new RtabmapEventInit(uFormat("No changes added to database.")));

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
		if(!_memoryChanged && _linksChanged && _dbDriver)
		{
			// don't update the time stamps!
			UDEBUG("");
			_dbDriver->setTimestampUpdateEnabled(false);
		}
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
	Parameters::parse(parameters, Parameters::kMemBinDataKept(), _binDataKept);
	Parameters::parse(parameters, Parameters::kMemNotLinkedNodesKept(), _notLinkedNodesKeptInDb);
	Parameters::parse(parameters, Parameters::kMemRehearsalIdUpdatedToNewOne(), _idUpdatedToNewOneRehearsal);
	Parameters::parse(parameters, Parameters::kMemGenerateIds(), _generateIds);
	Parameters::parse(parameters, Parameters::kMemBadSignaturesIgnored(), _badSignaturesIgnored);
	Parameters::parse(parameters, Parameters::kMemRehearsalSimilarity(), _similarityThreshold);
	Parameters::parse(parameters, Parameters::kMemRecentWmRatio(), _recentWmRatio);
	Parameters::parse(parameters, Parameters::kMemTransferSortingByWeightId(), _transferSortingByWeightId);
	Parameters::parse(parameters, Parameters::kMemSTMSize(), _maxStMemSize);
	Parameters::parse(parameters, Parameters::kMemImageDecimation(), _imageDecimation);
	Parameters::parse(parameters, Parameters::kMemLaserScanVoxelSize(), _laserScanVoxelSize);
	Parameters::parse(parameters, Parameters::kMemLocalSpaceLinksKeptInWM(), _localSpaceLinksKeptInWM);
	Parameters::parse(parameters, Parameters::kRGBDLinearUpdate(), _rehearsalMaxDistance);
	Parameters::parse(parameters, Parameters::kRGBDAngularUpdate(), _rehearsalMaxAngle);
	Parameters::parse(parameters, Parameters::kMemRehearsalWeightIgnoredWhileMoving(), _rehearsalWeightIgnoredWhileMoving);

	UASSERT_MSG(_maxStMemSize >= 0, uFormat("value=%d", _maxStMemSize).c_str());
	UASSERT_MSG(_similarityThreshold >= 0.0f && _similarityThreshold <= 1.0f, uFormat("value=%f", _similarityThreshold).c_str());
	UASSERT_MSG(_recentWmRatio >= 0.0f && _recentWmRatio <= 1.0f, uFormat("value=%f", _recentWmRatio).c_str());
	UASSERT(_imageDecimation >= 1);

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
	Parameters::parse(parameters, Parameters::kLccBowRefineIterations(), _bowRefineIterations);
	Parameters::parse(parameters, Parameters::kLccBowForce2D(), _bowForce2D);
	Parameters::parse(parameters, Parameters::kLccBowEstimationType(), _bowEstimationType);
	Parameters::parse(parameters, Parameters::kLccBowEpipolarGeometryVar(), _bowEpipolarGeometryVar);
	Parameters::parse(parameters, Parameters::kLccBowPnPReprojError(), _bowPnPReprojError);
	Parameters::parse(parameters, Parameters::kLccBowPnPFlags(), _bowPnPFlags);
	Parameters::parse(parameters, Parameters::kLccIcpMaxTranslation(), _icpMaxTranslation);
	Parameters::parse(parameters, Parameters::kLccIcpMaxRotation(), _icpMaxRotation);
	Parameters::parse(parameters, Parameters::kLccIcp3Decimation(), _icpDecimation);
	Parameters::parse(parameters, Parameters::kLccIcp3MaxDepth(), _icpMaxDepth);
	Parameters::parse(parameters, Parameters::kLccIcp3VoxelSize(), _icpVoxelSize);
	Parameters::parse(parameters, Parameters::kLccIcp3Samples(), _icpSamples);
	Parameters::parse(parameters, Parameters::kLccIcp3MaxCorrespondenceDistance(), _icpMaxCorrespondenceDistance);
	Parameters::parse(parameters, Parameters::kLccIcp3Iterations(), _icpMaxIterations);
	Parameters::parse(parameters, Parameters::kLccIcp3CorrespondenceRatio(), _icpCorrespondenceRatio);
	Parameters::parse(parameters, Parameters::kLccIcp3PointToPlane(), _icpPointToPlane);
	Parameters::parse(parameters, Parameters::kLccIcp3PointToPlaneNormalNeighbors(), _icpPointToPlaneNormalNeighbors);
	Parameters::parse(parameters, Parameters::kLccIcp2MaxCorrespondenceDistance(), _icp2MaxCorrespondenceDistance);
	Parameters::parse(parameters, Parameters::kLccIcp2Iterations(), _icp2MaxIterations);
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
	UASSERT_MSG(_icpDecimation > 0, uFormat("value=%d", _icpDecimation).c_str());
	UASSERT_MSG(_icpMaxDepth >= 0.0f, uFormat("value=%f", _icpMaxDepth).c_str());
	UASSERT_MSG(_icpVoxelSize >= 0, uFormat("value=%d", _icpVoxelSize).c_str());
	UASSERT_MSG(_icpSamples >= 0, uFormat("value=%d", _icpSamples).c_str());
	UASSERT_MSG(_icpMaxCorrespondenceDistance > 0.0f, uFormat("value=%f", _icpMaxCorrespondenceDistance).c_str());
	UASSERT_MSG(_icpMaxIterations > 0, uFormat("value=%d", _icpMaxIterations).c_str());
	UASSERT_MSG(_icpCorrespondenceRatio >=0.0f && _icpCorrespondenceRatio <=1.0f, uFormat("value=%f", _icpCorrespondenceRatio).c_str());
	UASSERT_MSG(_icpPointToPlaneNormalNeighbors > 0, uFormat("value=%d", _icpPointToPlaneNormalNeighbors).c_str());
	UASSERT_MSG(_icp2MaxCorrespondenceDistance > 0.0f, uFormat("value=%f", _icp2MaxCorrespondenceDistance).c_str());
	UASSERT_MSG(_icp2MaxIterations > 0, uFormat("value=%d", _icp2MaxIterations).c_str());
	UASSERT_MSG(_icp2CorrespondenceRatio >=0.0f && _icp2CorrespondenceRatio <=1.0f, uFormat("value=%f", _icp2CorrespondenceRatio).c_str());
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

bool Memory::update(
		const SensorData & data,
		Statistics * stats)
{
	return update(data, Transform(), cv::Mat(), stats);
}

bool Memory::update(
		const SensorData & data,
		const Transform & pose,
		const cv::Mat & covariance,
		Statistics * stats)
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
	Signature * signature = this->createSignature(data, pose, stats);
	if (signature == 0)
	{
		UERROR("Failed to create a signature...");
		return false;
	}

	t=timer.ticks()*1000;
	if(stats) stats->addStatistic(Statistics::kTimingMemSignature_creation(), t);
	UDEBUG("time creating signature=%f ms", t);

	// It will be added to the short-term memory, no need to delete it...
	this->addSignatureToStm(signature, covariance);

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
	int validSignaturesCount = 0;
	for(std::set<int>::iterator iter=_stMem.begin(); iter!=_stMem.end(); ++iter)
	{
		const Signature * s = this->getSignature(*iter);
		UASSERT(s != 0);
		if(!s->isBadSignature())
		{
			++validSignaturesCount;
		}
	}
	while(_stMem.size() && _maxStMemSize>0 && validSignaturesCount > _maxStMemSize)
	{
		UDEBUG("Inserting node %d from STM in WM...", *_stMem.begin());
		Signature * s = this->_getSignature(*_stMem.begin());
		if(!_localSpaceLinksKeptInWM)
		{
			// remove local space links outside STM
			UASSERT(s!=0);
			std::map<int, Link> links = s->getLinks(); // get a copy because we will remove some links in "s"
			for(std::map<int, Link>::iterator iter=links.begin(); iter!=links.end(); ++iter)
			{
				if(iter->second.type() == Link::kLocalSpaceClosure)
				{
					Signature * sTo = this->_getSignature(iter->first);
					if(sTo)
					{
						sTo->removeLink(s->id());
					}
					else
					{
						UERROR("Link %d of %d not in WM/STM?!?", iter->first, s->id());
					}
					s->removeLink(iter->first);
				}
			}
		}
		if(!s->isBadSignature())
		{
			--validSignaturesCount;
		}
		_workingMem.insert(_workingMem.end(), std::make_pair(*_stMem.begin(), UTimer::now()));
		_stMem.erase(*_stMem.begin());
		++_signaturesAdded;
	}

	if(!_memoryChanged && _incrementalMemory)
	{
		_memoryChanged = true;
	}

	UDEBUG("totalTimer = %fs", totalTimer.ticks());

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
			tmpValues[i] = uStr2Float(*iter);
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

void Memory::addSignatureToStm(Signature * signature, const cv::Mat & covariance)
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
					cv::Mat infMatrix = covariance.inv();
					motionEstimate = _signatures.at(*_stMem.rbegin())->getPose().inverse() * signature->getPose();
					_signatures.at(*_stMem.rbegin())->addLink(Link(*_stMem.rbegin(), signature->id(), Link::kNeighbor, motionEstimate, infMatrix));
					signature->addLink(Link(signature->id(), *_stMem.rbegin(), Link::kNeighbor, motionEstimate.inverse(), infMatrix));
				}
				else
				{
					_signatures.at(*_stMem.rbegin())->addLink(Link(*_stMem.rbegin(), signature->id(), Link::kNeighbor, Transform()));
					signature->addLink(Link(signature->id(), *_stMem.rbegin(), Link::kNeighbor, Transform()));
				}
				UDEBUG("Min STM id = %d", *_stMem.begin());
			}
			else
			{
				UDEBUG("Ignoring neighbor link between %d and %d because they are not in the same map! (%d vs %d)",
						*_stMem.rbegin(), signature->id(),
						_signatures.at(*_stMem.rbegin())->mapId(), signature->mapId());

				//Tag the first node of the map
				std::string tag = uFormat("map%d", signature->mapId());
				if(getSignatureIdByLabel(tag, false) == 0)
				{
					UINFO("Tagging node %d with label \"%s\"", signature->id(), tag.c_str());
					signature->setLabel(tag);
				}
			}
		}
		else
		{
			//Tag the first node of the map
			std::string tag = uFormat("map%d", signature->mapId());
			if(getSignatureIdByLabel(tag, false) == 0)
			{
				UINFO("Tagging node %d with label \"%s\"", signature->id(), tag.c_str());
				signature->setLabel(tag);
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
		_workingMem.insert(std::make_pair(signature->id(), UTimer::now()));
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

std::map<int, Link> Memory::getNeighborLinks(
		int signatureId,
		bool lookInDatabase) const
{
	std::map<int, Link> links;
	Signature * s = uValue(_signatures, signatureId, (Signature*)0);
	if(s)
	{
		const std::map<int, Link> & allLinks = s->getLinks();
		for(std::map<int, Link>::const_iterator iter = allLinks.begin(); iter!=allLinks.end(); ++iter)
		{
			if(iter->second.type() == Link::kNeighbor)
			{
				links.insert(*iter);
			}
		}
	}
	else if(lookInDatabase && _dbDriver)
	{
		std::map<int, Link> neighbors;
		_dbDriver->loadLinks(signatureId, neighbors, Link::kNeighbor);
		links.insert(neighbors.begin(), neighbors.end());
	}
	else
	{
		UWARN("Cannot find signature %d in memory", signatureId);
	}
	return links;
}

std::map<int, Link> Memory::getLoopClosureLinks(
		int signatureId,
		bool lookInDatabase) const
{
	const Signature * s = this->getSignature(signatureId);
	std::map<int, Link> loopClosures;
	if(s)
	{
		const std::map<int, Link> & allLinks = s->getLinks();
		for(std::map<int, Link>::const_iterator iter = allLinks.begin(); iter!=allLinks.end(); ++iter)
		{
			if(iter->second.type() > Link::kNeighbor &&
			   iter->second.type() != Link::kUndef)
			{
				loopClosures.insert(*iter);
			}
		}
	}
	else if(lookInDatabase && _dbDriver)
	{
		_dbDriver->loadLinks(signatureId, loopClosures);
		for(std::map<int, Link>::iterator iter=loopClosures.begin(); iter!=loopClosures.end();)
		{
			if(iter->second.type() == Link::kNeighbor)
			{
				loopClosures.erase(iter++);
			}
			else
			{
				++iter;
			}
		}
	}
	return loopClosures;
}

std::map<int, Link> Memory::getLinks(
		int signatureId,
		bool lookInDatabase) const
{
	std::map<int, Link> links;
	Signature * s = uValue(_signatures, signatureId, (Signature*)0);
	if(s)
	{
		links = s->getLinks();
	}
	else if(lookInDatabase && _dbDriver)
	{
		_dbDriver->loadLinks(signatureId, links, Link::kUndef);
	}
	else
	{
		UWARN("Cannot find signature %d in memory", signatureId);
	}
	return links;
}

std::multimap<int, Link> Memory::getAllLinks(bool lookInDatabase, bool ignoreNullLinks) const
{
	std::multimap<int, Link> links;

	if(lookInDatabase && _dbDriver)
	{
		_dbDriver->getAllLinks(links, ignoreNullLinks);
	}

	for(std::map<int, Signature*>::const_iterator iter=_signatures.begin(); iter!=_signatures.end(); ++iter)
	{
		links.erase(iter->first);
		for(std::map<int, Link>::const_iterator jter=iter->second->getLinks().begin();
			jter!=iter->second->getLinks().end();
			++jter)
		{
			if(!ignoreNullLinks || jter->second.isValid())
			{
				links.insert(std::make_pair(iter->first, jter->second));
			}
		}
	}

	return links;
}


// return map<Id,Margin>, including signatureId
// maxCheckedInDatabase = -1 means no limit to check in database (default)
// maxCheckedInDatabase = 0 means don't check in database
std::map<int, int> Memory::getNeighborsId(int signatureId,
		int maxGraphDepth, // 0 means infinite margin
		int maxCheckedInDatabase, // default -1 (no limit)
		bool incrementMarginOnLoop, // default false
		bool ignoreLoopIds, // default false
		bool ignoreIntermediateNodes, // default false
		double * dbAccessTime
		) const
{
	UASSERT(maxGraphDepth >= 0);
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
	std::set<int> ignoredIds;
	while((maxGraphDepth == 0 || m < maxGraphDepth) && nextMargin.size())
	{
		// insert more recent first (priority to be loaded first from the database below if set)
		curentMarginList = std::list<int>(nextMargin.rbegin(), nextMargin.rend());
		nextMargin.clear();

		for(std::list<int>::iterator jter = curentMarginList.begin(); jter!=curentMarginList.end(); ++jter)
		{
			if(ids.find(*jter) == ids.end())
			{
				//UDEBUG("Added %d with margin %d", *jter, m);
				// Look up in STM/WM if all ids are here, if not... load them from the database
				const Signature * s = this->getSignature(*jter);
				std::map<int, Link> tmpLinks;
				const std::map<int, Link> * links = &tmpLinks;
				if(s)
				{
					if(!ignoreIntermediateNodes || s->getWeight() != -1)
					{
						ids.insert(std::pair<int, int>(*jter, m));
					}
					else
					{
						ignoredIds.insert(*jter);
					}

					links = &s->getLinks();
				}
				else if(maxCheckedInDatabase == -1 || (maxCheckedInDatabase > 0 && _dbDriver && nbLoadedFromDb < maxCheckedInDatabase))
				{
					++nbLoadedFromDb;
					ids.insert(std::pair<int, int>(*jter, m));

					UTimer timer;
					_dbDriver->loadLinks(*jter, tmpLinks);
					if(dbAccessTime)
					{
						*dbAccessTime += timer.getElapsedTime();
					}
				}

				// links
				for(std::map<int, Link>::const_iterator iter=links->begin(); iter!=links->end(); ++iter)
				{
					if( !uContains(ids, iter->first) && ignoredIds.find(iter->first) == ignoredIds.end())
					{
						UASSERT(iter->second.type() != Link::kUndef);
						if(iter->second.type() == Link::kNeighbor)
						{
							if(ignoreIntermediateNodes && s->getWeight()==-1)
							{
								// stay on the same margin
								if(currentMargin.insert(iter->first).second)
								{
									curentMarginList.push_back(iter->first);
								}
							}
							else
							{
								nextMargin.insert(iter->first);
							}
						}
						else if(!ignoreLoopIds)
						{
							if(incrementMarginOnLoop)
							{
								nextMargin.insert(iter->first);
							}
							else
							{
								if(currentMargin.insert(iter->first).second)
								{
									curentMarginList.push_back(iter->first);
								}
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

// return map<Id,sqrdDistance>, including signatureId
std::map<int, float> Memory::getNeighborsIdRadius(
		int signatureId,
		float radius, // 0 means ignore radius
		const std::map<int, Transform> & optimizedPoses,
		int maxGraphDepth // 0 means infinite margin
		) const
{
	UASSERT(maxGraphDepth >= 0);
	UASSERT(uContains(optimizedPoses, signatureId));
	UASSERT(signatureId > 0);
	std::map<int, float> ids;
	std::list<int> curentMarginList;
	std::set<int> currentMargin;
	std::set<int> nextMargin;
	nextMargin.insert(signatureId);
	int m = 0;
	Transform referential = optimizedPoses.at(signatureId);
	UASSERT(!referential.isNull());
	float radiusSqrd = radius*radius;
	std::map<int, float> savedRadius;
	savedRadius.insert(std::make_pair(signatureId, 0));
	while((maxGraphDepth == 0 || m < maxGraphDepth) && nextMargin.size())
	{
		curentMarginList = std::list<int>(nextMargin.begin(), nextMargin.end());
		nextMargin.clear();

		for(std::list<int>::iterator jter = curentMarginList.begin(); jter!=curentMarginList.end(); ++jter)
		{
			if(ids.find(*jter) == ids.end())
			{
				//UDEBUG("Added %d with margin %d", *jter, m);
				// Look up in STM/WM if all ids are here, if not... load them from the database
				const Signature * s = this->getSignature(*jter);
				std::map<int, Link> tmpLinks;
				const std::map<int, Link> * links = &tmpLinks;
				if(s)
				{
					ids.insert(std::pair<int, float>(*jter, savedRadius.at(*jter)));

					links = &s->getLinks();
				}

				// links
				for(std::map<int, Link>::const_iterator iter=links->begin(); iter!=links->end(); ++iter)
				{
					if(!uContains(ids, iter->first) &&
						uContains(optimizedPoses, iter->first))
					{
						const Transform & t = optimizedPoses.at(iter->first);
						UASSERT(!t.isNull());
						float distanceSqrd = referential.getDistanceSquared(t);
						if(radiusSqrd == 0 || distanceSqrd<radiusSqrd)
						{
							savedRadius.insert(std::make_pair(iter->first, distanceSqrd));
							nextMargin.insert(iter->first);
						}

					}
				}
			}
		}
		++m;
	}
	return ids;
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
		// New session! move all signatures from the STM to WM
		while(_stMem.size())
		{
			UDEBUG("Inserting node %d from STM in WM...", *_stMem.begin());
			if(!_localSpaceLinksKeptInWM)
			{
				// remove local space links outside STM
				Signature * s = this->_getSignature(*_stMem.begin());
				UASSERT(s!=0);
				std::map<int, Link> links = s->getLinks(); // get a copy because we will remove some links in "s"
				for(std::map<int, Link>::iterator iter=links.begin(); iter!=links.end(); ++iter)
				{
					if(iter->second.type() == Link::kLocalSpaceClosure)
					{
						Signature * sTo = this->_getSignature(iter->first);
						if(sTo)
						{
							sTo->removeLink(s->id());
						}
						else
						{
							UERROR("Link %d of %d not in WM/STM?!?", iter->first, s->id());
						}
						s->removeLink(iter->first);
					}
				}
			}
			_workingMem.insert(_workingMem.end(), std::make_pair(*_stMem.begin(), UTimer::now()));
			_stMem.erase(*_stMem.begin());
		}

		return ++_idMapCount;
	}
	return _idMapCount;
}

void Memory::updateAge(int signatureId)
{
	std::map<int, double>::iterator iter=_workingMem.find(signatureId);
	if(iter!=_workingMem.end())
	{
		iter->second = UTimer::now();
	}
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
		unsigned int memSize = (unsigned int)(_workingMem.size() + _stMem.size());
		if(_workingMem.size() && _workingMem.begin()->first < 0)
		{
			--memSize;
		}

		// this is only a safe check...not supposed to occur.
		UASSERT_MSG(memSize == _signatures.size(),
				uFormat("The number of signatures don't match! _workingMem=%d, _stMem=%d, _signatures=%d",
						_workingMem.size(), _stMem.size(), _signatures.size()).c_str());

		UDEBUG("Adding statistics after run...");
		if(_memoryChanged)
		{
			UDEBUG("");
			_dbDriver->addStatisticsAfterRun(memSize,
					_lastSignature?_lastSignature->id():0,
					UProcessInfo::getMemoryUsage(),
					_dbDriver->getMemoryUsed(),
					(int)_vwd->getVisualWords().size());
		}
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

	if(_workingMem.size() != 0 && !(_workingMem.size() == 1 && _workingMem.begin()->first == kIdVirtual))
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
	_lastGlobalLoopClosureId = 0;
	_idCount = kIdStart;
	_idMapCount = kIdStart;
	_memoryChanged = false;
	_linksChanged = false;

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

		UDEBUG("compute likelihood (similarity)... %f s", timer.ticks());
		return likelihood;
	}
	else
	{
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

		UDEBUG("compute likelihood (tf-idf) %f s", timer.ticks());
		return likelihood;
	}
}

// Weights of the signatures in the working memory <signature id, weight>
std::map<int, int> Memory::getWeights() const
{
	std::map<int, int> weights;
	for(std::map<int, double>::const_iterator iter=_workingMem.begin(); iter!=_workingMem.end(); ++iter)
	{
		if(iter->first > 0)
		{
			const Signature * s = this->getSignature(iter->first);
			if(!s)
			{
				UFATAL("Location %d must exist in memory", iter->first);
			}
			weights.insert(weights.end(), std::make_pair(iter->first, s->getWeight()));
		}
		else
		{
			weights.insert(weights.end(), std::make_pair(iter->first, -1));
		}
	}
	return weights;
}

std::list<int> Memory::forget(const std::set<int> & ignoredIds)
{
	UDEBUG("");
	std::list<int> signaturesRemoved;
	if(_vwd->isIncremental() && _vwd->getVisualWords().size())
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


int Memory::cleanup()
{
	UDEBUG("");
	int signatureRemoved = 0;

	// bad signature
	if(_lastSignature && ((_lastSignature->isBadSignature() && _badSignaturesIgnored) || !_incrementalMemory))
	{
		if(_lastSignature->isBadSignature())
		{
			UDEBUG("Bad signature! %d", _lastSignature->id());
		}
		signatureRemoved = _lastSignature->id();
		moveToTrash(_lastSignature, _incrementalMemory);
	}

	return signatureRemoved;
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

class WeightAgeIdKey
{
public:
	WeightAgeIdKey(int w, double a, int i) :
		weight(w),
		age(a),
		id(i){}
	bool operator<(const WeightAgeIdKey & k) const
	{
		if(weight < k.weight)
		{
			return true;
		}
		else if(weight == k.weight)
		{
			if(age < k.age)
			{
				return true;
			}
			else if(age == k.age)
			{
				if(id < k.id)
				{
					return true;
				}
			}
		}
		return false;
	}
	int weight, age, id;
};
std::list<Signature *> Memory::getRemovableSignatures(int count, const std::set<int> & ignoredIds)
{
	//UDEBUG("");
	std::list<Signature *> removableSignatures;
	std::map<WeightAgeIdKey, Signature *> weightAgeIdMap;

	// Find the last index to check...
	UDEBUG("mem.size()=%d, ignoredIds.size()=%d", (int)_workingMem.size(), (int)ignoredIds.size());

	if(_workingMem.size())
	{
		int recentWmMaxSize = _recentWmRatio * float(_workingMem.size());
		bool recentWmImmunized = false;
		// look for the position of the lastLoopClosureId in WM
		int currentRecentWmSize = 0;
		if(_lastGlobalLoopClosureId > 0 && _stMem.find(_lastGlobalLoopClosureId) == _stMem.end())
		{
			// If set, it must be in WM
			std::map<int, double>::const_iterator iter = _workingMem.find(_lastGlobalLoopClosureId);
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
				UERROR("Last loop closure id not found in WM (%d)", _lastGlobalLoopClosureId);
			}
			UDEBUG("currentRecentWmSize=%d, recentWmMaxSize=%d, _recentWmRatio=%f, end recent wM = %d", currentRecentWmSize, recentWmMaxSize, _recentWmRatio, _lastGlobalLoopClosureId);
		}

		// Ignore neighbor of the last location in STM (for neighbor links redirection issue during Rehearsal).
		Signature * lastInSTM = 0;
		if(_stMem.size())
		{
			lastInSTM = _signatures.at(*_stMem.begin());
		}

		for(std::map<int, double>::const_iterator memIter = _workingMem.begin(); memIter != _workingMem.end(); ++memIter)
		{
			if( (recentWmImmunized && memIter->first > _lastGlobalLoopClosureId) ||
				memIter->first == _lastGlobalLoopClosureId)
			{
				// ignore recent memory
			}
			else if(memIter->first > 0 && ignoredIds.find(memIter->first) == ignoredIds.end() && (!lastInSTM || !lastInSTM->hasLink(memIter->first)))
			{
				Signature * s = this->_getSignature(memIter->first);
				if(s)
				{
					// Links must not be in STM to be removable, rehearsal issue
					bool foundInSTM = false;
					for(std::map<int, Link>::const_iterator iter = s->getLinks().begin(); iter!=s->getLinks().end(); ++iter)
					{
						if(_stMem.find(iter->first) != _stMem.end())
						{
							UDEBUG("Ignored %d because it has a link (%d) to STM", s->id(), iter->first);
							foundInSTM = true;
							break;
						}
					}
					if(!foundInSTM)
					{
						// less weighted signature priority to be transferred
						weightAgeIdMap.insert(std::make_pair(WeightAgeIdKey(s->getWeight(), _transferSortingByWeightId?0.0:memIter->second, s->id()), s));
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
		// make the list of removable signatures
		// Criteria : Weight -> ID
		UDEBUG("signatureMap.size()=%d", (int)weightAgeIdMap.size());
		for(std::map<WeightAgeIdKey, Signature*>::iterator iter=weightAgeIdMap.begin();
			iter!=weightAgeIdMap.end();
			++iter)
		{
			bool removable = true;
			if(removable)
			{
				if(!recentWmImmunized)
				{
					UDEBUG("weight=%d, id=%d",
							iter->second->getWeight(),
							iter->second->id());
					removableSignatures.push_back(iter->second);

					if(iter->second->id() > _lastGlobalLoopClosureId)
					{
						++recentWmCount;
						if(currentRecentWmSize - recentWmCount < recentWmMaxSize)
						{
							UDEBUG("switched recentWmImmunized");
							recentWmImmunized = true;
						}
					}
				}
				else if(iter->second->id() < _lastGlobalLoopClosureId)
				{
					UDEBUG("weight=%d, id=%d",
							iter->second->getWeight(),
							iter->second->id());
					removableSignatures.push_back(iter->second);
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
void Memory::moveToTrash(Signature * s, bool keepLinkedToGraph, std::list<int> * deletedWords)
{
	UDEBUG("id=%d", s?s->id():0);
	if(s)
	{
		// If not saved to database or it is a bad signature (not saved), remove links!
		if(!keepLinkedToGraph || (!s->isSaved() && s->isBadSignature() && _badSignaturesIgnored))
		{
			UASSERT_MSG(this->isInSTM(s->id()),
					uFormat("Deleting location (%d) outside the STM is not implemented!", s->id()).c_str());
			const std::map<int, Link> & links = s->getLinks();
			for(std::map<int, Link>::const_iterator iter=links.begin(); iter!=links.end(); ++iter)
			{
				Signature * sTo = this->_getSignature(iter->first);
				// neighbor to s
				if(sTo)
				{
					if(iter->first > s->id() && (sTo->getLinks().size() == 1 || !sTo->hasLink(s->id())))
					{
						UWARN("Link %d of %d is newer, removing neighbor link may split the map!",
								iter->first, s->id());
					}

					// child
					if(iter->second.type() == Link::kGlobalClosure && s->id() > sTo->id())
					{
						sTo->setWeight(sTo->getWeight() + s->getWeight()); // copy weight
					}

					sTo->removeLink(s->id());
				}
				else
				{
					UERROR("Link %d of %d not in WM/STM?!?", iter->first, s->id());
				}
			}
			s->removeLinks(); // remove all links
			s->setWeight(0);
			s->setLabel(""); // reset label
		}
		else
		{
			// Make sure that virtual links are removed.
			// It should be called before the signature is
			// removed from _signatures below.
			removeVirtualLinks(s->id());
		}

		this->disableWordsRef(s->id());
		if(!keepLinkedToGraph)
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
				_lastSignature = this->_getSignature(_workingMem.rbegin()->first);
			}
		}

		if(	(_notLinkedNodesKeptInDb || keepLinkedToGraph) &&
			_dbDriver &&
			s->id()>0 &&
			(_incrementalMemory || s->isSaved()))
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

int Memory::getSignatureIdByLabel(const std::string & label, bool lookInDatabase) const
{
	UDEBUG("label=%s", label.c_str());
	int id = 0;
	if(label.size())
	{
		for(std::map<int, Signature*>::const_iterator iter=_signatures.begin(); iter!=_signatures.end(); ++iter)
		{
			UASSERT(iter->second != 0);
			if(iter->second->getLabel().compare(label) == 0)
			{
				id = iter->second->id();
				break;
			}
		}
		if(id == 0 && _dbDriver && lookInDatabase)
		{
			_dbDriver->getNodeIdByLabel(label, id);
		}
	}
	return id;
}

bool Memory::labelSignature(int id, const std::string & label)
{
	// verify that this label is not used
	int idFound=getSignatureIdByLabel(label);
	if(idFound == 0 || idFound == id)
	{
		Signature * s  = this->_getSignature(id);
		if(s)
		{
			s->setLabel(label);
			return true;
		}
		else if(_dbDriver)
		{
			std::list<int> ids;
			ids.push_back(id);
			std::list<Signature *> signatures;
			_dbDriver->loadSignatures(ids,signatures);
			if(signatures.size())
			{
				signatures.front()->setLabel(label);
				_dbDriver->asyncSave(signatures.front()); // move it again to trash
				return true;
			}
		}
		else
		{
			UERROR("Node %d not found, failed to set label \"%s\"!", id, label.c_str());
		}
	}
	else if(idFound)
	{
		UWARN("Node %d has already label \"%s\"", idFound, label.c_str());
	}
	return false;
}

std::map<int, std::string> Memory::getAllLabels() const
{
	std::map<int, std::string> labels;
	for(std::map<int, Signature*>::const_iterator iter = _signatures.begin(); iter!=_signatures.end(); ++iter)
	{
		if(!iter->second->getLabel().empty())
		{
			labels.insert(std::make_pair(iter->first, iter->second->getLabel()));
		}
	}
	if(_dbDriver)
	{
		_dbDriver->getAllLabels(labels);
	}
	return labels;
}

bool Memory::setUserData(int id, const cv::Mat & data)
{
	Signature * s  = this->_getSignature(id);
	if(s)
	{
		s->sensorData().setUserData(data);
		return true;
	}
	else
	{
		UERROR("Node %d not found in RAM, failed to set user data (size=%d)!", id, data.total());
	}
	return false;
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

void Memory::removeLink(int oldId, int newId)
{
	//this method assumes receiving oldId < newId, if not switch them
	Signature * oldS = this->_getSignature(oldId<newId?oldId:newId);
	Signature * newS = this->_getSignature(oldId<newId?newId:oldId);
	if(oldS && newS)
	{
		UINFO("removing link between location %d and %d", oldS->id(), newS->id());

		if(oldS->hasLink(newS->id()) && newS->hasLink(oldS->id()))
		{
			Link::Type type = oldS->getLinks().at(newS->id()).type();
			if(type == Link::kGlobalClosure && newS->getWeight() > 0)
			{
				// adjust the weight
				oldS->setWeight(oldS->getWeight()+1);
				newS->setWeight(newS->getWeight()>0?newS->getWeight()-1:0);
			}


			oldS->removeLink(newS->id());
			newS->removeLink(oldS->id());

			if(type!=Link::kVirtualClosure)
			{
				_linksChanged = true;
			}

			bool noChildrenAnymore = true;
			for(std::map<int, Link>::const_iterator iter=newS->getLinks().begin(); iter!=newS->getLinks().end(); ++iter)
			{
				if(iter->second.type() > Link::kNeighbor && iter->first < newS->id())
				{
					noChildrenAnymore = false;
					break;
				}
			}
			if(noChildrenAnymore && newS->id() == _lastGlobalLoopClosureId)
			{
				_lastGlobalLoopClosureId = 0;
			}
		}
		else
		{
			UERROR("Signatures %d and %d don't have bidirectional link!", oldS->id(), newS->id());
		}
	}
	else
	{
		if(!newS)
		{
			UERROR("Signature %d is not in working memory... cannot remove link.", newS->id());
		}
		if(!oldS)
		{
			UERROR("Signature %d is not in working memory... cannot remove link.", oldS->id());
		}
	}
}

// compute transform newId -> oldId
Transform Memory::computeVisualTransform(
		int oldId,
		int newId,
		std::string * rejectedMsg,
		int * inliers,
		double * variance) const
{
	const Signature * oldS = this->getSignature(oldId);
	const Signature * newS = this->getSignature(newId);

	Transform transform;

	if(oldS && newId)
	{
		return computeVisualTransform(*oldS, *newS, rejectedMsg, inliers, variance);
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
Transform Memory::computeVisualTransform(
		const Signature & oldS,
		const Signature & newS,
		std::string * rejectedMsg,
		int * inliersOut,
		double * varianceOut) const
{
	Transform transform;
	std::string msg;
	// Guess transform from visual words

	int inliersCount= 0;
	double variance = 1.0;

	if(_bowEstimationType == 2) // Epipolar Geometry
	{
		if(!newS.sensorData().stereoCameraModel().isValid() &&
		   (newS.sensorData().cameraModels().size() != 1 ||
			!newS.sensorData().cameraModels()[0].isValid()))
		{
			UERROR("Calibrated camera required (multi-cameras not supported).");
		}
		else if((int)oldS.getWords().size() >= _bowMinInliers &&
				(int)newS.getWords().size() >= _bowMinInliers)
		{
			UASSERT(oldS.sensorData().stereoCameraModel().isValid() || (oldS.sensorData().cameraModels().size() == 1 && oldS.sensorData().cameraModels()[0].isValid()));
			const CameraModel & cameraModel = oldS.sensorData().stereoCameraModel().isValid()?oldS.sensorData().stereoCameraModel().left():oldS.sensorData().cameraModels()[0];

			// we only need the camera transform, send guess words3 for scale estimation
			Transform cameraTransform;
			std::multimap<int, pcl::PointXYZ> inliers3D = util3d::generateWords3DMono(
					oldS.getWords(),
					newS.getWords(),
					cameraModel,
					cameraTransform,
					_bowIterations,
					_bowPnPReprojError,
					_bowPnPFlags, // cv::SOLVEPNP_ITERATIVE
					1.0f,
					0.99f,
					oldS.getWords3(), // for scale estimation
					&variance);

			inliersCount = (int)inliers3D.size();

			if(!cameraTransform.isNull())
			{
				if((int)inliers3D.size() >= _bowMinInliers)
				{
					if(variance <= _bowEpipolarGeometryVar)
					{
						transform = cameraTransform.inverse();
					}
					else
					{
						msg = uFormat("Variance is too high! (max inlier distance=%f, variance=%f)", _bowEpipolarGeometryVar, variance);
						UINFO(msg.c_str());
					}
				}
				else
				{
					msg = uFormat("Not enough inliers %d < %d", (int)inliers3D.size(), _bowMinInliers);
					UINFO(msg.c_str());
				}
			}
			else
			{
				msg = uFormat("No camera transform found");
				UINFO(msg.c_str());
			}
		}
		else if(oldS.getWords3().size() == 0)
		{
			msg = uFormat("No 3D guess words found");
			UWARN(msg.c_str());
		}
		else
		{
			msg = uFormat("No camera model");
			UWARN(msg.c_str());
		}
	}
	else if(_bowEstimationType == 1) // PnP
	{
		if(!newS.sensorData().stereoCameraModel().isValid() &&
		   (newS.sensorData().cameraModels().size() != 1 ||
			!newS.sensorData().cameraModels()[0].isValid()))
		{
			UERROR("Calibrated camera required (multi-cameras not supported).");
		}
		else
		{
			// 3D to 2D
			if((int)oldS.getWords3().size() >= _bowMinInliers &&
			   (int)newS.getWords().size() >= _bowMinInliers)
			{
				UASSERT(newS.sensorData().stereoCameraModel().isValid() || (newS.sensorData().cameraModels().size() == 1 && newS.sensorData().cameraModels()[0].isValid()));
				const CameraModel & cameraModel = newS.sensorData().stereoCameraModel().isValid()?newS.sensorData().stereoCameraModel().left():newS.sensorData().cameraModels()[0];

				std::vector<int> inliersV;
				transform = util3d::estimateMotion3DTo2D(
						oldS.getWords3(),
						newS.getWords(),
						cameraModel,
						_bowMinInliers,
						_bowIterations,
						_bowPnPReprojError,
						_bowPnPFlags,
						Transform::getIdentity(),
						newS.getWords3(),
						&variance,
						0,
						&inliersV);
				inliersCount = (int)inliersV.size();
				if(transform.isNull())
				{
					msg = uFormat("Not enough inliers %d/%d between %d and %d",
							inliersCount, _bowMinInliers, oldS.id(), newS.id());
					UINFO(msg.c_str());
				}
				else
				{
					transform = transform.inverse();
				}
			}
			else
			{
				msg = uFormat("Not enough features in images (old=%d, new=%d, min=%d)",
						(int)oldS.getWords3().size(), (int)newS.getWords().size(), _bowMinInliers);
				UINFO(msg.c_str());
			}
		}

	}
	else
	{
		// 3D -> 3D
		if((int)oldS.getWords3().size() >= _bowMinInliers &&
		   (int)newS.getWords3().size() >= _bowMinInliers)
		{
			std::vector<int> inliersV;
			transform = util3d::estimateMotion3DTo3D(
					oldS.getWords3(),
					newS.getWords3(),
					_bowMinInliers,
					_bowInlierDistance,
					_bowIterations,
					_bowRefineIterations,
					&variance,
					0,
					&inliersV);
			inliersCount = (int)inliersV.size();
			if(transform.isNull())
			{
				msg = uFormat("Not enough inliers %d/%d between %d and %d",
						inliersCount, _bowMinInliers, oldS.id(), newS.id());
				UINFO(msg.c_str());
			}
			else
			{
				transform = transform.inverse();
			}
		}
		else
		{
			msg = uFormat("Not enough 3D features in images (old=%d, new=%d, min=%d)",
					(int)oldS.getWords3().size(), (int)newS.getWords3().size(), _bowMinInliers);
			UINFO(msg.c_str());
		}
	}

	if(!transform.isNull())
	{
		// verify if it is a 180 degree transform, well verify > 90
		float x,y,z, roll,pitch,yaw;
		transform.getTranslationAndEulerAngles(x,y,z, roll,pitch,yaw);
		if(fabs(roll) > CV_PI/2 ||
		   fabs(pitch) > CV_PI/2 ||
		   fabs(yaw) > CV_PI/2)
		{
			transform.setNull();
			msg = uFormat("Too large rotation detected! (roll=%f, pitch=%f, yaw=%f)",
					roll, pitch, yaw);
			UWARN(msg.c_str());
		}
		else if(_bowForce2D)
		{
			UDEBUG("Forcing 2D...");
			transform = Transform(x,y,0, 0, 0, yaw);
		}
	}

	if(rejectedMsg)
	{
		*rejectedMsg = msg;
	}
	if(inliersOut)
	{
		*inliersOut = inliersCount;
	}
	if(varianceOut)
	{
		*varianceOut = variance;
	}
	UDEBUG("transform=%s", transform.prettyPrint().c_str());
	return transform;
}

// compute transform newId -> oldId
Transform Memory::computeIcpTransform(
		int oldId,
		int newId,
		Transform guess,
		bool icp3D,
		std::string * rejectedMsg,
		int * inliers,
		double * variance,
		float * inliersRatio)
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
			if(oldS->sensorData().depthOrRightCompressed().empty())
			{
				depthToLoad.push_back(oldS);
				added.insert(oldS->id());
			}
			if(newS->sensorData().depthOrRightCompressed().empty())
			{
				depthToLoad.push_back(newS);
				added.insert(newS->id());
			}
		}
		else
		{
			//Depth required, if not in RAM, load it from LTM
			if(oldS->sensorData().laserScanCompressed().empty() && added.find(oldS->id()) == added.end())
			{
				depthToLoad.push_back(oldS);
			}
			if(newS->sensorData().laserScanCompressed().empty() && added.find(newS->id()) == added.end())
			{
				depthToLoad.push_back(newS);
			}
		}
		if(depthToLoad.size())
		{
			_dbDriver->loadNodeData(depthToLoad);
		}
	}

	Transform t;
	if(oldS && newS)
	{
		//make sure data are uncompressed
		if(icp3D)
		{
			cv::Mat tmp1, tmp2;
			oldS->sensorData().uncompressData(0, &tmp1, 0);
			newS->sensorData().uncompressData(0, &tmp2, 0);
		}
		else
		{
			cv::Mat tmp1, tmp2;
			oldS->sensorData().uncompressData(0, 0, &tmp1);
			newS->sensorData().uncompressData(0, 0, &tmp2);
		}

		t = computeIcpTransform(*oldS, *newS, guess, icp3D, rejectedMsg, inliers, variance, inliersRatio);
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
Transform Memory::computeIcpTransform(
		const Signature & oldS,
		const Signature & newS,
		Transform guess,
		bool icp3D,
		std::string * rejectedMsg,
		int * correspondencesOut,
		double * varianceOut,
		float * correspondencesRatioOut) const
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
		if(!oldS.sensorData().depthOrRightRaw().empty() &&
			!newS.sensorData().depthOrRightRaw().empty() &&
			(oldS.sensorData().cameraModels().size() || oldS.sensorData().stereoCameraModel().isValid()) &&
			(newS.sensorData().cameraModels().size() || newS.sensorData().stereoCameraModel().isValid()))
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr oldCloudXYZ = util3d::cloudFromSensorData(
					oldS.sensorData(),
					_icpDecimation,
					_icpMaxDepth,
					_icpVoxelSize,
					_icpSamples);
			pcl::PointCloud<pcl::PointXYZ>::Ptr newCloudXYZ = util3d::cloudFromSensorData(
					newS.sensorData(),
					_icpDecimation,
					_icpMaxDepth,
					_icpVoxelSize,
					_icpSamples);

			// 3D
			if(newCloudXYZ->size() && oldCloudXYZ->size())
			{
				newCloudXYZ = util3d::transformPointCloud(newCloudXYZ, guess);

				bool hasConverged = false;
				Transform icpT;
				int correspondences = 0;
				float correspondencesRatio = -1.0f;
				double variance = 1;
				if(_icpPointToPlane)
				{
					pcl::PointCloud<pcl::PointNormal>::Ptr oldCloud = util3d::computeNormals(oldCloudXYZ, _icpPointToPlaneNormalNeighbors);
					pcl::PointCloud<pcl::PointNormal>::Ptr newCloud = util3d::computeNormals(newCloudXYZ, _icpPointToPlaneNormalNeighbors);

					std::vector<int> indices;
					newCloud = util3d::removeNaNNormalsFromPointCloud(newCloud);
					oldCloud = util3d::removeNaNNormalsFromPointCloud(oldCloud);

					if(newCloud->size() && oldCloud->size())
					{
						pcl::PointCloud<pcl::PointNormal>::Ptr newCloudRegistered(new pcl::PointCloud<pcl::PointNormal>);
						icpT = util3d::icpPointToPlane(
								newCloud,
								oldCloud,
							   _icpMaxCorrespondenceDistance,
							   _icpMaxIterations,
							   hasConverged,
							   *newCloudRegistered);

						util3d::computeVarianceAndCorrespondences(
								newCloudRegistered,
								oldCloud,
								_icpMaxCorrespondenceDistance,
								variance,
								correspondences);
					}
				}
				else
				{
					pcl::PointCloud<pcl::PointXYZ>::Ptr newCloudRegistered(new pcl::PointCloud<pcl::PointXYZ>);
					icpT = util3d::icp(
							newCloudXYZ,
							oldCloudXYZ,
							_icpMaxCorrespondenceDistance,
							_icpMaxIterations,
							hasConverged,
							*newCloudRegistered);

					util3d::computeVarianceAndCorrespondences(
							newCloudRegistered,
							oldCloudXYZ,
							_icpMaxCorrespondenceDistance,
							variance,
							correspondences);
				}

				// verify if there are enough correspondences
				correspondencesRatio = float(correspondences)/float(newCloudXYZ->size()>oldCloudXYZ->size()?newCloudXYZ->size():oldCloudXYZ->size());

				UDEBUG("%d->%d hasConverged=%s, variance=%f, correspondences=%d/%d (%f%%)",
						hasConverged?"true":"false",
						variance,
						correspondences,
						(int)(oldCloudXYZ->size()>newCloudXYZ->size()?oldCloudXYZ->size():newCloudXYZ->size()),
						correspondencesRatio*100.0f);

				if(varianceOut)
				{
					*varianceOut = variance;
				}
				if(correspondencesOut)
				{
					*correspondencesOut = correspondences;
				}
				if(correspondencesRatioOut)
				{
					*correspondencesRatioOut = correspondencesRatio;
				}

				if(!icpT.isNull() && hasConverged &&
				   correspondencesRatio >= _icpCorrespondenceRatio)
				{
					float x,y,z, roll,pitch,yaw;
					icpT.getTranslationAndEulerAngles(x,y,z,roll,pitch,yaw);
					if((_icpMaxTranslation>0.0f &&
						(fabs(x) > _icpMaxTranslation ||
						 fabs(y) > _icpMaxTranslation ||
						 fabs(z) > _icpMaxTranslation))
						||
						(_icpMaxRotation>0.0f &&
						 (fabs(roll) > _icpMaxRotation ||
						  fabs(pitch) > _icpMaxRotation ||
						  fabs(yaw) > _icpMaxRotation)))
					{
						msg = uFormat("Cannot compute transform (ICP correction too large)");
						UINFO(msg.c_str());
					}
					else
					{
						transform = icpT * guess;
						transform = transform.inverse();
					}
				}
				else
				{
					msg = uFormat("Cannot compute transform (converged=%s var=%f corr=%d corrRatio=%f/%f)",
							hasConverged?"true":"false", variance, correspondences, correspondencesRatio, _icpCorrespondenceRatio);
					UINFO(msg.c_str());
				}
			}
			else
			{
				msg = "Clouds empty ?!?";
				UWARN(msg.c_str());
			}
		}
		else
		{
			msg = uFormat("Depths 3D empty?!? (new[%d]=%d old[%d]=%d)",
					newS.id(), newS.sensorData().depthOrRightRaw().total(),
					oldS.id(), oldS.sensorData().depthOrRightRaw().total());
			UERROR(msg.c_str());
		}
	}
	else // icp 2D
	{
		UDEBUG("2D ICP");

		// We are 2D here, make sure the guess has only YAW rotation
		float x,y,z,r,p,yaw;
		guess.getTranslationAndEulerAngles(x,y,z, r,p,yaw);
		guess = Transform(x,y,0, 0, 0, yaw);
		if(r!=0 || p!=0)
		{
			UINFO("2D ICP: Dropping z (%f), roll (%f) and pitch (%f) rotation!", z, r, p);
		}

		if(!oldS.sensorData().laserScanRaw().empty() && !newS.sensorData().laserScanRaw().empty())
		{
			// 2D
			pcl::PointCloud<pcl::PointXYZ>::Ptr oldCloud = util3d::cvMat2Cloud(oldS.sensorData().laserScanRaw());
			pcl::PointCloud<pcl::PointXYZ>::Ptr newCloud = util3d::cvMat2Cloud(newS.sensorData().laserScanRaw(), guess);

			//voxelize
			pcl::PointCloud<pcl::PointXYZ>::Ptr oldCloudVoxelized = oldCloud;
			pcl::PointCloud<pcl::PointXYZ>::Ptr newCloudVoxelized = newCloud;
			if(_icp2VoxelSize > _laserScanVoxelSize)
			{
				oldCloudVoxelized = util3d::voxelize(oldCloud, _icp2VoxelSize);
				newCloudVoxelized = util3d::voxelize(newCloud, _icp2VoxelSize);
			}

			if(newCloud->size() && oldCloud->size())
			{
				Transform icpT;
				bool hasConverged = false;
				float correspondencesRatio = -1.0f;
				int correspondences = 0;
				double variance = 1;
				pcl::PointCloud<pcl::PointXYZ>::Ptr newCloudRegistered(new pcl::PointCloud<pcl::PointXYZ>());
				icpT = util3d::icp2D(
						newCloudVoxelized,
						oldCloudVoxelized,
					   _icp2MaxCorrespondenceDistance,
					   _icp2MaxIterations,
					   hasConverged,
					   *newCloudRegistered);

				//pcl::io::savePCDFile("oldCloud.pcd", *oldCloud);
				//pcl::io::savePCDFile("newCloud.pcd", *newCloud);
				//UWARN("saved oldCloud.pcd and newCloud.pcd");
				//if(!icpT.isNull())
				//{
				//	newCloud = util3d::transformPointCloud<pcl::PointXYZ>(newCloud, icpT);
				//	pcl::io::savePCDFile("newCloudFinal.pcd", *newCloud);
				//	UWARN("saved newCloudFinal.pcd");
				//}

				if(!icpT.isNull() &&
					hasConverged)
				{
					float ix,iy,iz, iroll,ipitch,iyaw;
					icpT.getTranslationAndEulerAngles(ix,iy,iz,iroll,ipitch,iyaw);
					if((_icpMaxTranslation>0.0f &&
						(fabs(ix) > _icpMaxTranslation ||
						 fabs(iy) > _icpMaxTranslation ||
						 fabs(iz) > _icpMaxTranslation))
						||
						(_icpMaxRotation>0.0f &&
						 (fabs(iroll) > _icpMaxRotation ||
						  fabs(ipitch) > _icpMaxRotation ||
						  fabs(iyaw) > _icpMaxRotation)))
					{
						msg = uFormat("Cannot compute transform (ICP correction too large)");
						UINFO(msg.c_str());
					}
					else
					{
						if(_icp2VoxelSize <= _laserScanVoxelSize)
						{
							newCloud = util3d::transformPointCloud(newCloud, icpT);
						}
						else
						{
							newCloud = newCloudRegistered;
						}

						pcl::PointCloud<pcl::PointXYZ>::Ptr newCloudRegistered(new pcl::PointCloud<pcl::PointXYZ>);
						util3d::computeVarianceAndCorrespondences(
								newCloud,
								oldCloud,
								_icpMaxCorrespondenceDistance,
								variance,
								correspondences);

						// verify if there are enough correspondences
						if(newS.sensorData().laserScanMaxPts())
						{
							correspondencesRatio = float(correspondences)/float(newS.sensorData().laserScanMaxPts());
						}
						else
						{
							UWARN("Maximum laser scans points not set for signature %d, correspondences ratio set to 0!",
									newS.id());
						}

						UDEBUG("%d->%d hasConverged=%s, variance=%f, correspondences=%d/%d (%f%%)",
								newS.id(), oldS.id(),
								hasConverged?"true":"false",
								variance,
								correspondences,
								(int)(newS.sensorData().laserScanMaxPts()),
								correspondencesRatio*100.0f);

						if(varianceOut)
						{
							*varianceOut = variance;
						}
						if(correspondencesOut)
						{
							*correspondencesOut = correspondences;
						}
						if(correspondencesRatioOut)
						{
							*correspondencesRatioOut = correspondencesRatio;
						}

						if(correspondencesRatio < _icp2CorrespondenceRatio)
						{
							msg = uFormat("Cannot compute transform (cor=%d corrRatio=%f/%f)",
									correspondences, correspondencesRatio, _icp2CorrespondenceRatio);
							UINFO(msg.c_str());
						}
						else
						{
							transform = icpT * guess;
							transform = transform.inverse();
						}
					}
				}
				else
				{
					msg = uFormat("Cannot compute transform (converged=%s var=%f)",
							hasConverged?"true":"false", variance);
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
			msg = uFormat("Depths 2D empty?!? (new[%d]=%d old[%d]=%d)",
					newS.id(), newS.sensorData().laserScanRaw().total(),
					oldS.id(), oldS.sensorData().laserScanRaw().total());
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
		std::string * rejectedMsg,
		int * inliers,
		double * variance)
{
	UASSERT(uContains(poses, newId) && uContains(_signatures, newId));
	UASSERT(uContains(poses, oldId) && uContains(_signatures, oldId));

	// make sure that all depth2D are loaded
	std::list<Signature*> depthToLoad;
	for(std::map<int, Transform>::const_iterator iter = poses.begin(); iter!=poses.end(); ++iter)
	{
		Signature * s = _getSignature(iter->first);
		UASSERT(s != 0);
		if(s->sensorData().laserScanCompressed().empty())
		{
			depthToLoad.push_back(s);
		}
	}
	if(depthToLoad.size() && _dbDriver)
	{
		_dbDriver->loadNodeData(depthToLoad);
	}

	std::string msg;
	pcl::PointCloud<pcl::PointXYZ>::Ptr assembledOldClouds(new pcl::PointCloud<pcl::PointXYZ>);
	for(std::map<int, Transform>::const_iterator iter = poses.begin(); iter!=poses.end(); ++iter)
	{
		if(iter->first != newId)
		{
			Signature * s = this->_getSignature(iter->first);
			if(!s->sensorData().laserScanCompressed().empty())
			{
				cv::Mat scan;
				s->sensorData().uncompressData(0, 0, &scan);
				*assembledOldClouds += *util3d::cvMat2Cloud(scan, iter->second);
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
	Signature * newS = _getSignature(newId);
	pcl::PointCloud<pcl::PointXYZ>::Ptr newCloud;
	cv::Mat newScan;
	newS->sensorData().uncompressData(0, 0, &newScan);
	newCloud = util3d::cvMat2Cloud(newScan, poses.at(newId));

	//voxelize
	pcl::PointCloud<pcl::PointXYZ>::Ptr newCloudVoxelized = newCloud;
	if(newCloud->size() && _icp2VoxelSize > _laserScanVoxelSize)
	{
		newCloudVoxelized = util3d::voxelize(newCloud, _icp2VoxelSize);
	}

	Transform transform;
	if(assembledOldClouds->size() && newCloudVoxelized->size())
	{
		int correspondences = 0;
		bool hasConverged = false;
		pcl::PointCloud<pcl::PointXYZ>::Ptr newCloudRegistered(new pcl::PointCloud<pcl::PointXYZ>);
		Transform icpT = util3d::icp2D(
				newCloudVoxelized,
				assembledOldClouds,
			   _icp2MaxCorrespondenceDistance,
			   _icp2MaxIterations,
			   hasConverged,
			   *newCloudRegistered);

		UDEBUG("icpT=%s", icpT.prettyPrint().c_str());

		//pcl::io::savePCDFile("old.pcd", *assembledOldClouds, true);
		//pcl::io::savePCDFile("new.pcd", *newCloud, true);
		//UWARN("local scan matching old.pcd, new.pcd saved!");
		//if(!icpT.isNull())
		//{
		//	newCloud = util3d::transformPointCloud<pcl::PointXYZ>(newCloud, icpT);
		//	pcl::io::savePCDFile("newFinal.pcd", *newCloud, true);
		//	UWARN("local scan matching newFinal.pcd saved!");
		//}

		if(!icpT.isNull() && hasConverged)
		{
			if(_icp2VoxelSize <= _laserScanVoxelSize)
			{
				newCloud = util3d::transformPointCloud(newCloud, icpT);
			}
			else
			{
				newCloud = newCloudRegistered;
			}

			pcl::PointCloud<pcl::PointXYZ>::Ptr newCloudRegistered(new pcl::PointCloud<pcl::PointXYZ>);
			double v = 1;
			util3d::computeVarianceAndCorrespondences(
					newCloud,
					assembledOldClouds,
					_icpMaxCorrespondenceDistance,
					v,
					correspondences);
			if(variance)
			{
				*variance = v;
			}

			// verify if there enough correspondences
			float correspondencesRatio = 0.0f;
			if(newS->sensorData().laserScanMaxPts())
			{
				correspondencesRatio = float(correspondences)/float(newS->sensorData().laserScanMaxPts());
			}
			else
			{
				UWARN("Maximum laser scans points not set for signature %d, correspondences ratio set to 0!",
						newS->id());
			}

			UDEBUG("variance=%f, correspondences=%d/%d (%f%%) %f",
					variance?*variance:-1,
					correspondences,
					(int)newCloud->size(),
					correspondencesRatio*100.0f);

			if(inliers)
			{
				*inliers = correspondences;
			}

			if(correspondencesRatio >= _icp2CorrespondenceRatio)
			{
				transform = poses.at(newId).inverse()*icpT.inverse() * poses.at(oldId);
			}
			else
			{
				msg = uFormat("Constraints failed... variance=%f, correspondences=%d/%d (%f%%)",
					variance?*variance:-1,
					correspondences,
					(int)newCloud->size(),
					correspondencesRatio);
				UINFO(msg.c_str());
			}
		}
		else
		{
			msg = uFormat("Constraints failed... hasConverged=%s",
				hasConverged?"true":"false");
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

bool Memory::addLink(const Link & link)
{
	UASSERT(link.type() > Link::kNeighbor && link.type() != Link::kUndef);

	ULOGGER_INFO("to=%d, from=%d transform: %s", link.to(), link.from(), link.transform().prettyPrint().c_str());
	Signature * toS = _getSignature(link.to());
	Signature * fromS = _getSignature(link.from());
	if(toS && fromS)
	{
		if(toS->hasLink(link.from()))
		{
			// do nothing, already merged
			UINFO("already linked! to=%d, from=%d", link.to(), link.from());
			return true;
		}

		UDEBUG("Add link between %d and %d", toS->id(), fromS->id());

		toS->addLink(Link(link.to(), link.from(), link.type(), link.transform().inverse(), link.infMatrix()));
		fromS->addLink(link);

		if(_incrementalMemory)
		{
			if(link.type()!=Link::kVirtualClosure)
			{
				_linksChanged = true;
			}

			if(link.type() == Link::kGlobalClosure)
			{
				_lastGlobalLoopClosureId = fromS->id()>toS->id()?fromS->id():toS->id();

				// update weights only if the memory is incremental
				UASSERT(fromS->getWeight() >= 0 && toS->getWeight() >=0);
				if(fromS->id() > toS->id())
				{
					fromS->setWeight(fromS->getWeight() + toS->getWeight());
					toS->setWeight(0);
				}
				else
				{
					toS->setWeight(toS->getWeight() + fromS->getWeight());
					fromS->setWeight(0);
				}
			}
		}
		return true;
	}
	else
	{
		if(!fromS)
		{
			UERROR("from=%d, to=%d, Signature %d not found in working/st memories", link.from(), link.to(), link.from());
		}
		if(!toS)
		{
			UERROR("from=%d, to=%d, Signature %d not found in working/st memories", link.from(), link.to(), link.to());
		}
	}
	return false;
}

void Memory::updateLink(int fromId, int toId, const Transform & transform, float rotVariance, float transVariance)
{
	Signature * fromS = this->_getSignature(fromId);
	Signature * toS = this->_getSignature(toId);

	if(fromS->hasLink(toId) && toS->hasLink(fromId))
	{
		Link::Type type = fromS->getLinks().at(toId).type();
		fromS->removeLink(toId);
		toS->removeLink(fromId);

		fromS->addLink(Link(fromId, toId, type, transform, rotVariance, transVariance));
		toS->addLink(Link(toId, fromId, type, transform.inverse(), rotVariance, transVariance));

		if(type!=Link::kVirtualClosure)
		{
			_linksChanged = true;
		}
	}
	else
	{
		UERROR("fromId=%d and toId=%d are not linked!", fromId, toId);
	}
}

void Memory::updateLink(int fromId, int toId, const Transform & transform, const cv::Mat & covariance)
{
	Signature * fromS = this->_getSignature(fromId);
	Signature * toS = this->_getSignature(toId);

	if(fromS->hasLink(toId) && toS->hasLink(fromId))
	{
		Link::Type type = fromS->getLinks().at(toId).type();
		fromS->removeLink(toId);
		toS->removeLink(fromId);

		cv::Mat infMatrix  = covariance.inv();
		fromS->addLink(Link(fromId, toId, type, transform, infMatrix));
		toS->addLink(Link(toId, fromId, type, transform.inverse(), infMatrix));

		if(type!=Link::kVirtualClosure)
		{
			_linksChanged = true;
		}
	}
	else
	{
		UERROR("fromId=%d and toId=%d are not linked!", fromId, toId);
	}
}

void Memory::removeAllVirtualLinks()
{
	UDEBUG("");
	for(std::map<int, Signature*>::iterator iter=_signatures.begin(); iter!=_signatures.end(); ++iter)
	{
		iter->second->removeVirtualLinks();
	}
}

void Memory::removeVirtualLinks(int signatureId)
{
	UDEBUG("");
	Signature * s = this->_getSignature(signatureId);
	if(s)
	{
		const std::map<int, Link> & links = s->getLinks();
		for(std::map<int, Link>::const_iterator iter=links.begin(); iter!=links.end(); ++iter)
		{
			if(iter->second.type() == Link::kVirtualClosure)
			{
				Signature * sTo = this->_getSignature(iter->first);
				if(sTo)
				{
					sTo->removeLink(s->id());
				}
				else
				{
					UERROR("Link %d of %d not in WM/STM?!?", iter->first, s->id());
				}
			}
		}
		s->removeVirtualLinks();
	}
	else
	{
		UERROR("Signature %d not in WM/STM?!?", signatureId);
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
		fprintf(foutSign, "SignatureID WordsID...\n");
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
						   (jter->second.x != 0 || jter->second.y != 0 || jter->second.z != 0))
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

			std::map<int, Link> loopIds, childIds;

			for(std::map<int, Link>::const_iterator iter = i->second->getLinks().begin();
					iter!=i->second->getLinks().end();
					++iter)
			{
				if(iter->second.type() > Link::kNeighbor)
				{
					if(iter->first < i->first)
					{
						childIds.insert(*iter);
					}
					else
					{
						loopIds.insert(*iter);
					}
				}
			}

			fprintf(foutTree, " %d", (int)loopIds.size());
			for(std::map<int, Link>::const_iterator j=loopIds.begin(); j!=loopIds.end(); ++j)
			{
				fprintf(foutTree, " %d", j->first);
			}

			fprintf(foutTree, " %d", (int)childIds.size());
			for(std::map<int, Link>::const_iterator j=childIds.begin(); j!=childIds.end(); ++j)
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
	if(signature->getLinks().size() != 1)
	{
		return;
	}

	//============================================================
	// Compare with the last (not null)
	//============================================================
	Signature * sB = 0;
	for(std::set<int>::reverse_iterator iter=_stMem.rbegin(); iter!=_stMem.rend(); ++iter)
	{
		Signature * s = this->_getSignature(*iter);
		UASSERT(s!=0);
		if(s->getWeight() >= 0 && s->id() != signature->id())
		{
			sB = s;
			break;
		}
	}
	if(sB)
	{
		int id = sB->id();
		UDEBUG("Comparing with signature (%d)...", id);

		float sim = signature->compareTo(*sB);

		int merged = 0;
		if(sim >= _similarityThreshold)
		{
			if(_incrementalMemory)
			{
				if(signature->hasLink(id))
				{
					if(signature->getLinks().begin()->second.transform().isNull())
					{
						if(this->rehearsalMerge(id, signature->id()))
						{
							merged = id;
						}
					}
					else
					{
						float x,y,z, roll,pitch,yaw;
						signature->getLinks().begin()->second.transform().getTranslationAndEulerAngles(x,y,z, roll,pitch,yaw);
						if((_rehearsalMaxDistance>0.0f && (
								fabs(x) > _rehearsalMaxDistance ||
								fabs(y) > _rehearsalMaxDistance ||
								fabs(z) > _rehearsalMaxDistance)) ||
							(_rehearsalMaxAngle>0.0f && (
								fabs(roll) > _rehearsalMaxAngle ||
								fabs(pitch) > _rehearsalMaxAngle ||
								fabs(yaw) > _rehearsalMaxAngle)))
						{
							if(_rehearsalWeightIgnoredWhileMoving)
							{
								UINFO("Rehearsal ignored because the robot has moved more than %f m or %f rad",
										_rehearsalMaxDistance, _rehearsalMaxAngle);
							}
							else
							{
								// if the robot has moved, increase only weight of the new one
								signature->setWeight(sB->getWeight() + signature->getWeight() + 1);
								sB->setWeight(0);
								UINFO("Only updated weight to %d of %d (old=%d) because the robot has moved. (d=%f a=%f)",
										signature->getWeight(), signature->id(), sB->id(), _rehearsalMaxDistance, _rehearsalMaxAngle);
							}
						}
						else if(this->rehearsalMerge(id, signature->id()))
						{
							merged = id;
						}
					}
				}
				else
				{
					// cannot merge not neighbor signatures, just update weight
					signature->setWeight(sB->getWeight() + signature->getWeight() + 1);
					sB->setWeight(0);
					UINFO("Only updated weight to %d of %d (old=%d) because the signatures are not neighbors.",
							signature->getWeight(), signature->id(), sB->id());
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
	else
	{
		if(stats) stats->addStatistic(Statistics::kMemoryRehearsal_merged(), 0);
		if(stats) stats->addStatistic(Statistics::kMemoryRehearsal_sim(), 0);
	}
}

bool Memory::rehearsalMerge(int oldId, int newId)
{
	ULOGGER_INFO("old=%d, new=%d", oldId, newId);
	Signature * oldS = _getSignature(oldId);
	Signature * newS = _getSignature(newId);
	if(oldS && newS && _incrementalMemory)
	{
		std::map<int, Link>::const_iterator iter = oldS->getLinks().find(newS->id());
		if(iter != oldS->getLinks().end() && iter->second.type() > Link::kNeighbor)
		{
			// do nothing, already merged
			UWARN("already merged, old=%d, new=%d", oldId, newId);
			return false;
		}
		UASSERT(!newS->isSaved());

		UINFO("Rehearsal merging %d and %d", oldS->id(), newS->id());

		//remove mutual links
		oldS->removeLink(newId);
		newS->removeLink(oldId);

		if(_idUpdatedToNewOneRehearsal)
		{
			// redirect neighbor links
			const std::map<int, Link> & links = oldS->getLinks();
			for(std::map<int, Link>::const_iterator iter = links.begin(); iter!=links.end(); ++iter)
			{
				Link link = iter->second;
				link.setFrom(newS->id());

				Signature * s = this->_getSignature(link.to());
				if(s)
				{
					// modify neighbor "from"
					s->changeLinkIds(oldS->id(), newS->id());

					newS->addLink(link);
				}
				else
				{
					UERROR("Didn't find neighbor %d of %d in RAM...", link.to(), oldS->id());
				}
			}
			newS->setLabel(oldS->getLabel());
			oldS->setLabel("");
			oldS->removeLinks(); // remove all links
			oldS->addLink(Link(oldS->id(), newS->id(), Link::kGlobalClosure, Transform(), 1, 1)); // to keep track of the merged location

			// Set old image to new signature
			this->copyData(oldS, newS);

			// update weight
			newS->setWeight(newS->getWeight() + 1 + oldS->getWeight());

			if(_lastGlobalLoopClosureId == oldS->id())
			{
				_lastGlobalLoopClosureId = newS->id();
			}
		}
		else
		{
			newS->addLink(Link(newS->id(), oldS->id(), Link::kGlobalClosure, Transform() , 1, 1)); // to keep track of the merged location

			// update weight
			oldS->setWeight(newS->getWeight() + 1 + oldS->getWeight());

			if(_lastSignature == newS)
			{
				_lastSignature = oldS;
			}
		}

		// remove location
		moveToTrash(_idUpdatedToNewOneRehearsal?oldS:newS, _notLinkedNodesKeptInDb);

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

Transform Memory::getOdomPose(int signatureId, bool lookInDatabase) const
{
	Transform pose;
	int mapId, weight;
	std::string label;
	double stamp;
	getNodeInfo(signatureId, pose, mapId, weight, label, stamp, lookInDatabase);
	return pose;
}

bool Memory::getNodeInfo(int signatureId,
		Transform & odomPose,
		int & mapId,
		int & weight,
		std::string & label,
		double & stamp,
		bool lookInDatabase) const
{
	const Signature * s = this->getSignature(signatureId);
	if(s)
	{
		odomPose = s->getPose();
		mapId = s->mapId();
		weight = s->getWeight();
		label = s->getLabel();
		stamp = s->getStamp();
		return true;
	}
	else if(lookInDatabase && _dbDriver)
	{
		return _dbDriver->getNodeInfo(signatureId, odomPose, mapId, weight, label, stamp);
	}
	return false;
}

cv::Mat Memory::getImageCompressed(int signatureId) const
{
	cv::Mat image;
	const Signature * s = this->getSignature(signatureId);
	if(s)
	{
		image = s->sensorData().imageCompressed();
	}
	if(image.empty() && this->isBinDataKept() && _dbDriver)
	{
		SensorData data;
		_dbDriver->getNodeData(signatureId, data);
		image = data.imageCompressed();
	}
	return image;
}

SensorData Memory::getNodeData(int nodeId, bool uncompressedData)
{
	UDEBUG("nodeId=%d", nodeId);
	SensorData r;
	Signature * s = this->_getSignature(nodeId);
	if(s && !s->sensorData().imageCompressed().empty())
	{
		if(uncompressedData)
		{
			s->sensorData().uncompressData();
		}
		r = s->sensorData();
	}
	else if(_dbDriver)
	{
		// load from database
		if(s)
		{
			std::list<Signature*> signatures;
			signatures.push_back(s);
			_dbDriver->loadNodeData(signatures);
			if(uncompressedData)
			{
				s->sensorData().uncompressData();
			}
			r = s->sensorData();
		}
		else
		{
			_dbDriver->getNodeData(nodeId, r);
			if(uncompressedData)
			{
				r.uncompressData();
			}
		}
	}

	return r;
}

void Memory::getNodeWords(int nodeId,
		std::multimap<int, cv::KeyPoint> & words,
		std::multimap<int, pcl::PointXYZ> & words3)
{
	UDEBUG("nodeId=%d", nodeId);
	Signature * s = this->_getSignature(nodeId);
	if(s)
	{
		words = s->getWords();
		words3 = s->getWords3();
	}
	else if(_dbDriver)
	{
		// load from database
		std::list<Signature*> signatures;
		std::list<int> ids;
		ids.push_back(nodeId);
		std::set<int> loadedFromTrash;
		_dbDriver->loadSignatures(ids, signatures, &loadedFromTrash);
		if(signatures.size())
		{
			words = signatures.front()->getWords();
			words3 = signatures.front()->getWords3();
			if(loadedFromTrash.size())
			{
				//put back
				_dbDriver->asyncSave(signatures.front());
			}
			else
			{
				delete signatures.front();
			}
		}
	}
}

SensorData Memory::getSignatureDataConst(int locationId) const
{
	UDEBUG("");
	SensorData r;
	const Signature * s = this->getSignature(locationId);
	if(s && !s->sensorData().imageCompressed().empty())
	{
		r = s->sensorData();
	}
	else if(_dbDriver)
	{
		// load from database
		if(s)
		{
			std::list<Signature*> signatures;
			Signature tmp = *s;
			signatures.push_back(&tmp);
			_dbDriver->loadNodeData(signatures);
			r = tmp.sensorData();
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
				if(sTmp->sensorData().imageCompressed().empty())
				{
					_dbDriver->loadNodeData(signatures);
				}
				r = sTmp->sensorData();
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
				 std::map<int, Link> links;
				 _dbDriver->loadLinks(id, links);
				 int weight = 0;
				 _dbDriver->getWeight(id, weight);
				 for(std::map<int, Link>::iterator iter = links.begin(); iter!=links.end(); ++iter)
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
					 if(iter->second.type() == Link::kNeighbor)
					 {
						 fprintf(fout, "   \"%d\\n%d\" -> \"%d\\n%d\"\n",
								 id,
								 weight,
								 iter->first,
								 weightNeighbor);
					 }
					 else if(iter->first > id)
					 {
						 //loop
						 fprintf(fout, "   \"%d\\n%d\" -> \"%d\\n%d\" [label=\"L\", fontcolor=%s, fontsize=8];\n",
								 id,
								 weight,
								 iter->first,
								 weightNeighbor,
								 colorG);
					 }
					 else
					 {
						 //child
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
		 for(std::map<int, Signature*>::iterator i=_signatures.begin(); i!=_signatures.end(); ++i)
		 {
			 if(ids.find(i->first) != ids.end())
			 {
				 int id = i->second->id();
				 const std::map<int, Link> & links = i->second->getLinks();
				 int weight = i->second->getWeight();
				 for(std::map<int, Link>::const_iterator iter = links.begin(); iter!=links.end(); ++iter)
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
					 if(iter->second.type() == Link::kNeighbor)
					 {
						 fprintf(fout, "   \"%d\\n%d\" -> \"%d\\n%d\"\n",
								 id,
								 weight,
								 iter->first,
								 weightNeighbor);
					 }
					 else if(iter->first > id)
					 {
						 //loop
						 fprintf(fout, "   \"%d\\n%d\" -> \"%d\\n%d\" [label=\"L\", fontcolor=%s, fontsize=8];\n",
								 id,
								 weight,
								 iter->first,
								 weightNeighbor,
								 colorG);
					 }
					 else
					 {
						 //child
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
			_dbDriver->getNodeData(from->id(), to->sensorData());
			UDEBUG("Loaded image data from database");
		}
		else
		{
			to->sensorData() = (SensorData)from->sensorData();
		}
		to->sensorData().setId(to->id());

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

Signature * Memory::createSignature(const SensorData & data, const Transform & pose, Statistics * stats)
{
	UDEBUG("");
	UASSERT(data.imageRaw().empty() ||
			data.imageRaw().type() == CV_8UC1 ||
			data.imageRaw().type() == CV_8UC3);
	UASSERT_MSG(data.depthOrRightRaw().empty() ||
			(  (data.depthOrRightRaw().type() == CV_16UC1 || data.depthOrRightRaw().type() == CV_32FC1 || data.depthOrRightRaw().type() == CV_8UC1) &&
				data.depthOrRightRaw().rows == data.imageRaw().rows &&
				data.depthOrRightRaw().cols == data.imageRaw().cols),
				uFormat("image=(%d/%d) depth=(%d/%d, type=%d [accepted=%d,%d,%d])",
						data.imageRaw().cols,
						data.imageRaw().rows,
						data.depthOrRightRaw().cols,
						data.depthOrRightRaw().rows,
						data.depthOrRightRaw().type(),
						CV_16UC1, CV_32FC1, CV_8UC1).c_str());
	UASSERT(data.laserScanRaw().empty() || data.laserScanRaw().type() == CV_32FC2);

	if(!data.depthOrRightRaw().empty() &&
		data.cameraModels().size() == 0 &&
		!data.stereoCameraModel().isValid())
	{
		UERROR("Rectified images required! Calibrate your camera.");
		return 0;
	}
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
			UERROR("Received image ID is null. "
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
			UERROR("Id of acquired image (%d) is smaller than the last in memory (%d). "
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
		if(_feature2D->getMaxFeatures() >= 0 && !data.imageRaw().empty())
		{
			// Extract features
			cv::Mat imageMono;
			// convert to grayscale
			if(data.imageRaw().channels() > 1)
			{
				cv::cvtColor(data.imageRaw(), imageMono, cv::COLOR_BGR2GRAY);
			}
			else
			{
				imageMono = data.imageRaw();
			}
			cv::Rect roi = Feature2D::computeRoi(imageMono, _roiRatios);

			if(!data.depthOrRightRaw().empty() && data.stereoCameraModel().isValid())
			{
				//stereo
				cv::Mat disparity;
				bool subPixelOn = false;
				if(_subPixWinSize > 0 && _subPixIterations > 0)
				{
					subPixelOn = true;
				}
				keypoints = _feature2D->generateKeypoints(imageMono, roi);
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
						if(stats) stats->addStatistic(Statistics::kTimingMemSubpixel(), t*1000.0f);
						UDEBUG("time subpix left kpts=%fs", t);
					}
					else
					{
						cv::KeyPoint::convert(keypoints, leftCorners);
					}

					//generate a disparity map
					disparity = util2d::disparityFromStereoImages(
							imageMono,
							data.depthOrRightRaw(),
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
						float minDisparity = data.stereoCameraModel().baseline() * data.stereoCameraModel().left().fx() / _wordsMaxDepth;
						Feature2D::filterKeypointsByDisparity(keypoints, descriptors, disparity, minDisparity);
						UDEBUG("filter keypoints by disparity (%d)", (int)keypoints.size());
					}

					if(keypoints.size())
					{
						if(!subPixelOn)
						{
							descriptors = _feature2D->generateDescriptors(imageMono, keypoints);
							t = timer.ticks();
							if(stats) stats->addStatistic(Statistics::kTimingMemDescriptors_extraction(), t*1000.0f);
							UDEBUG("time descriptors (%d) = %fs", descriptors.rows, t);
						}

						keypoints3D = util3d::generateKeypoints3DDisparity(
								keypoints,
								disparity,
								data.stereoCameraModel());
						t = timer.ticks();
						if(stats) stats->addStatistic(Statistics::kTimingMemKeypoints_3D(), t*1000.0f);
						UDEBUG("time keypoints 3D (%d) = %fs", (int)keypoints3D->size(), t);
					}
				}
			}
			else if(!data.depthOrRightRaw().empty() && data.cameraModels().size())
			{
				//depth
				bool subPixelOn = false;
				if(_subPixWinSize > 0 && _subPixIterations > 0)
				{
					subPixelOn = true;
				}
				keypoints = _feature2D->generateKeypoints(imageMono, roi);
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
						if(stats) stats->addStatistic(Statistics::kTimingMemSubpixel(), t*1000.0f);
						UDEBUG("time subpix left kpts=%fs", t);
					}

					if(_wordsMaxDepth > 0.0f)
					{
						Feature2D::filterKeypointsByDepth(keypoints, descriptors, data.depthOrRightRaw(), _wordsMaxDepth);
						UDEBUG("filter keypoints by depth (%d)", (int)keypoints.size());
					}

					if(keypoints.size())
					{
						if(!subPixelOn)
						{
							descriptors = _feature2D->generateDescriptors(imageMono, keypoints);
							t = timer.ticks();
							if(stats) stats->addStatistic(Statistics::kTimingMemDescriptors_extraction(), t*1000.0f);
							UDEBUG("time descriptors (%d) = %fs", descriptors.rows, t);
						}

						keypoints3D = util3d::generateKeypoints3DDepth(
								keypoints,
								data.depthOrRightRaw(),
								data.cameraModels());
						t = timer.ticks();
						if(stats) stats->addStatistic(Statistics::kTimingMemKeypoints_3D(), t*1000.0f);
						UDEBUG("time keypoints 3D (%d) = %fs", (int)keypoints3D->size(), t);
					}
				}
			}
			else
			{
				//RGB only
				keypoints = _feature2D->generateKeypoints(imageMono, roi);
				t = timer.ticks();
				if(stats) stats->addStatistic(Statistics::kTimingMemKeypoints_detection(), t*1000.0f);
				UDEBUG("time keypoints (%d) = %fs", (int)keypoints.size(), t);

				if(keypoints.size())
				{
					descriptors = _feature2D->generateDescriptors(imageMono, keypoints);
					t = timer.ticks();
					if(stats) stats->addStatistic(Statistics::kTimingMemDescriptors_extraction(), t*1000.0f);
					UDEBUG("time descriptors (%d) = %fs", descriptors.rows, t);

					if(_subPixWinSize > 0 && _subPixIterations > 0)
					{
						std::vector<cv::Point2f> corners;
						cv::KeyPoint::convert(keypoints, corners);
						cv::cornerSubPix( imageMono, corners,
								cv::Size( _subPixWinSize, _subPixWinSize ),
								cv::Size( -1, -1 ),
								cv::TermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, _subPixIterations, _subPixEps ) );

						for(unsigned int i=0;i<corners.size(); ++i)
						{
							keypoints[i].pt = corners[i];
						}

						t = timer.ticks();
						if(stats) stats->addStatistic(Statistics::kTimingMemSubpixel(), t*1000.0f);
						UDEBUG("time subpix kpts=%fs", t);
					}
				}
			}

			UDEBUG("ratio=%f, meanWordsPerLocation=%d", _badSignRatio, meanWordsPerLocation);
			if(descriptors.rows && descriptors.rows < _badSignRatio * float(meanWordsPerLocation))
			{
				descriptors = cv::Mat();
			}
		}
		else if(data.imageRaw().empty())
		{
			UDEBUG("Empty image, cannot extract features...");
		}
		else
		{
			UDEBUG("_feature2D->getMaxFeatures()(%d<0) so don't extract any features...", _feature2D->getMaxFeatures());
		}
	}
	else
	{
		keypoints = data.keypoints();
		descriptors = data.descriptors().clone();

		// filter by depth
		if(!data.depthOrRightRaw().empty() && data.stereoCameraModel().isValid())
		{
			//stereo
			cv::Mat imageMono;
			// convert to grayscale
			if(data.imageRaw().channels() > 1)
			{
				cv::cvtColor(data.imageRaw(), imageMono, cv::COLOR_BGR2GRAY);
			}
			else
			{
				imageMono = data.imageRaw();
			}
			//generate a disparity map
			std::vector<cv::Point2f> leftCorners;
			cv::KeyPoint::convert(keypoints, leftCorners);
			cv::Mat disparity = util2d::disparityFromStereoImages(
					imageMono,
					data.depthOrRightRaw(),
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
				float minDisparity = data.stereoCameraModel().baseline() * data.stereoCameraModel().left().fx() / _wordsMaxDepth;
				Feature2D::filterKeypointsByDisparity(keypoints, descriptors, disparity, minDisparity);
			}

			keypoints3D = util3d::generateKeypoints3DDisparity(
					keypoints,
					disparity,
					data.stereoCameraModel());
			t = timer.ticks();
			if(stats) stats->addStatistic(Statistics::kTimingMemKeypoints_3D(), t*1000.0f);
			UDEBUG("time keypoints 3D (%d) = %fs", (int)keypoints3D->size(), t);
		}
		else if(!data.depthOrRightRaw().empty() && data.cameraModels().size())
		{
			//depth
			if(_wordsMaxDepth)
			{
				Feature2D::filterKeypointsByDepth(keypoints, descriptors, _wordsMaxDepth);
				UDEBUG("filter keypoints by depth (%d)", (int)keypoints.size());
			}

			keypoints3D = util3d::generateKeypoints3DDepth(
					keypoints,
					data.depthOrRightRaw(),
					data.cameraModels());
			t = timer.ticks();
			if(stats) stats->addStatistic(Statistics::kTimingMemKeypoints_3D(), t*1000.0f);
			UDEBUG("time keypoints 3D (%d) = %fs", (int)keypoints3D->size(), t);
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
			if(_imageDecimation > 1)
			{
				cv::KeyPoint kpt = keypoints[i];
				kpt.pt.x /= float(_imageDecimation);
				kpt.pt.y /= float(_imageDecimation);
				kpt.size /= float(_imageDecimation);
				words.insert(std::pair<int, cv::KeyPoint>(*iter, kpt));
			}
			else
			{
				words.insert(std::pair<int, cv::KeyPoint>(*iter, keypoints[i]));
			}
			if(keypoints3D->size())
			{
				words3D.insert(std::pair<int, pcl::PointXYZ>(*iter, keypoints3D->at(i)));
			}
		}
	}

	if(words.size() > 8 &&
		words3D.size() == 0 &&
		!pose.isNull() &&
		data.cameraModels().size() == 1 &&
		_signatures.size())
	{
		UDEBUG("Generate 3D words using odometry");
		Signature * previousS = _signatures.rbegin()->second;
		if(previousS->getWords().size() > 8 && words.size() > 8 && !previousS->getPose().isNull())
		{
			Transform cameraTransform = pose.inverse() * previousS->getPose();
			// compute 3D words by epipolar geometry with the previous signature
			std::multimap<int, pcl::PointXYZ> inliers = util3d::generateWords3DMono(
					words,
					previousS->getWords(),
					data.cameraModels()[0],
					cameraTransform);

			// words3D should have the same size than words
			float bad_point = std::numeric_limits<float>::quiet_NaN ();
			for(std::multimap<int, cv::KeyPoint>::const_iterator iter=words.begin(); iter!=words.end(); ++iter)
			{
				std::multimap<int, pcl::PointXYZ>::iterator jter=inliers.find(iter->first);
				if(jter != inliers.end())
				{
					words3D.insert(std::make_pair(iter->first, jter->second));
				}
				else
				{
					words3D.insert(std::make_pair(iter->first, pcl::PointXYZ(bad_point,bad_point,bad_point)));
				}
			}

			t = timer.ticks();
			UASSERT(words3D.size() == words.size());
			if(stats) stats->addStatistic(Statistics::kTimingMemKeypoints_3D(), t*1000.0f);
			UDEBUG("time keypoints 3D (%d) = %fs", (int)keypoints3D->size(), t);
		}
	}

	cv::Mat image = data.imageRaw();
	cv::Mat depthOrRightImage = data.depthOrRightRaw();
	std::vector<CameraModel> cameraModels = data.cameraModels();
	StereoCameraModel stereoCameraModel = data.stereoCameraModel();

	// apply decimation?
	if((this->isBinDataKept() || this->isRawDataKept()) && _imageDecimation > 1)
	{
		image = util2d::decimate(image, _imageDecimation);
		depthOrRightImage = util2d::decimate(depthOrRightImage, _imageDecimation);
		for(unsigned int i=0; i<cameraModels.size(); ++i)
		{
			cameraModels[i].scale(1.0/double(_imageDecimation));
		}
		if(stereoCameraModel.isValid())
		{
			stereoCameraModel.scale(1.0/double(_imageDecimation));
		}
	}

	// apply icp2 voxel?
	cv::Mat laserScan = data.laserScanRaw();
	if(!laserScan.empty() && _laserScanVoxelSize > 0.0f)
	{
		laserScan = util3d::laserScanFromPointCloud(*util3d::voxelize(util3d::laserScanToPointCloud(laserScan), _laserScanVoxelSize));
	}

	Signature * s;
	if(this->isBinDataKept())
	{
		std::vector<unsigned char> imageBytes;
		std::vector<unsigned char> depthBytes;

		if(!depthOrRightImage.empty() && depthOrRightImage.type() == CV_32FC1)
		{
			UWARN("Keeping raw data in database: depth type is 32FC1, use 16UC1 depth format to avoid a conversion.");
			depthOrRightImage = util3d::cvtDepthFromFloat(depthOrRightImage);
		}

		rtabmap::CompressionThread ctImage(image, std::string(".jpg"));
		rtabmap::CompressionThread ctDepth(depthOrRightImage, std::string(".png"));
		rtabmap::CompressionThread ctDepth2d(laserScan);
		rtabmap::CompressionThread ctUserData(data.userDataRaw());
		ctImage.start();
		ctDepth.start();
		ctDepth2d.start();
		ctUserData.start();
		ctImage.join();
		ctDepth.join();
		ctDepth2d.join();
		ctUserData.join();

		s = new Signature(id,
			_idMapCount,
			(data.imageRaw().empty()&&data.laserScanRaw().empty()&&words.size()==0)?-1:0, // tag intermediate nodes as weight=-1
			data.stamp(),
			"",
			pose,
			stereoCameraModel.isValid()?
				SensorData(
						ctDepth2d.getCompressedData(),
						data.laserScanMaxPts(),
						ctImage.getCompressedData(),
						ctDepth.getCompressedData(),
						stereoCameraModel,
						id,
						0,
						ctUserData.getCompressedData()):
				SensorData(
						ctDepth2d.getCompressedData(),
						data.laserScanMaxPts(),
						ctImage.getCompressedData(),
						ctDepth.getCompressedData(),
						cameraModels,
						id,
						0,
						ctUserData.getCompressedData()));
	}
	else
	{
		rtabmap::CompressionThread ctDepth2d(laserScan);
		rtabmap::CompressionThread ctUserData(data.userDataRaw());
		ctDepth2d.start();
		ctUserData.start();
		ctDepth2d.join();
		ctUserData.join();

		s = new Signature(id,
			_idMapCount,
			(data.imageRaw().empty()&&data.laserScanRaw().empty()&&words.size()==0)?-1:0, // tag intermediate nodes as weight=-1
			data.stamp(),
			"",
			pose,
			stereoCameraModel.isValid()?
				SensorData(
						ctDepth2d.getCompressedData(),
						data.laserScanMaxPts(),
						cv::Mat(),
						cv::Mat(),
						stereoCameraModel,
						id,
						0,
						ctUserData.getCompressedData()):
				SensorData(
						ctDepth2d.getCompressedData(),
						data.laserScanMaxPts(),
						cv::Mat(),
						cv::Mat(),
						cameraModels,
						id,
						0,
						ctUserData.getCompressedData()));
	}
	s->setWords(words);
	s->setWords3(words3D);
	if(this->isRawDataKept())
	{
		s->sensorData().setImageRaw(image);
		s->sensorData().setDepthOrRightRaw(depthOrRightImage);
		s->sensorData().setLaserScanRaw(laserScan, data.laserScanMaxPts());
		s->sensorData().setUserDataRaw(data.userDataRaw());
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

// return all non-null poses
// return unique links between nodes (for neighbors: old->new, for loops: parent->child)
void Memory::getMetricConstraints(
		const std::set<int> & ids,
		std::map<int, Transform> & poses,
		std::multimap<int, Link> & links,
		bool lookInDatabase)
{
	UDEBUG("");
	for(std::set<int>::const_iterator iter=ids.begin(); iter!=ids.end(); ++iter)
	{
		Transform pose = getOdomPose(*iter, lookInDatabase);
		if(!pose.isNull())
		{
			poses.insert(std::make_pair(*iter, pose));
		}
	}

	for(std::set<int>::const_iterator iter=ids.begin(); iter!=ids.end(); ++iter)
	{
		if(uContains(poses, *iter))
		{
			std::map<int, Link> neighbors = this->getNeighborLinks(*iter, lookInDatabase); // only direct neighbors
			for(std::map<int, Link>::iterator jter=neighbors.begin(); jter!=neighbors.end(); ++jter)
			{
				if(	jter->second.isValid() &&
					uContains(poses, jter->first) &&
					graph::findLink(links, *iter, jter->first) == links.end())
				{
					if(!lookInDatabase)
					{
						Link link = jter->second;
						const Signature * s = this->getSignature(jter->first);
						UASSERT(s!=0);
						while(s && s->getWeight() == -1)
						{
							// skip to next neighbor, well we assume that bad signatures
							// are only linked by max 2 neighbor links.
							std::map<int, Link> n = this->getNeighborLinks(s->id(), false);
							UASSERT(n.size() <= 2);
							std::map<int, Link>::iterator uter = n.upper_bound(s->id());
							if(uter != n.end())
							{
								const Signature * s2 = this->getSignature(uter->first);
								if(s2)
								{
									link = link.merge(uter->second);
									poses.erase(s->id());
									s = s2;
								}

							}
							else
							{
								break;
							}
						}

						links.insert(std::make_pair(*iter, link));
					}
					else
					{
						links.insert(std::make_pair(*iter, jter->second));
					}
				}
			}

			std::map<int, Link> loops = this->getLoopClosureLinks(*iter, lookInDatabase);
			for(std::map<int, Link>::iterator jter=loops.begin(); jter!=loops.end(); ++jter)
			{
				if( jter->second.isValid() && // null transform means a rehearsed location
					jter->first < *iter && // Loop parent to child
					uContains(poses, jter->first))
				{
					links.insert(std::make_pair(*iter, jter->second));
				}
			}
		}
	}
}

} // namespace rtabmap
