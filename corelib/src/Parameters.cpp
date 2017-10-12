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

#include "rtabmap/core/Parameters.h"
#include <rtabmap/utilite/UDirectory.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UStl.h>
#include <cmath>
#include <stdlib.h>
#include <sstream>
#include <iostream>
#include <iomanip>
#include "SimpleIni.h"

namespace rtabmap
{

ParametersMap Parameters::parameters_;
ParametersMap Parameters::parametersType_;
ParametersMap Parameters::descriptions_;
Parameters Parameters::instance_;
std::map<std::string, std::pair<bool, std::string> > Parameters::removedParameters_;
ParametersMap Parameters::backwardCompatibilityMap_;

Parameters::Parameters()
{
}

Parameters::~Parameters()
{
}

std::string Parameters::createDefaultWorkingDirectory()
{
	std::string path = UDirectory::homeDir();
	if(!path.empty())
	{
		UDirectory::makeDir(path += UDirectory::separator() + "Documents");
		UDirectory::makeDir(path += UDirectory::separator() + "RTAB-Map");

	}
	else
	{
		UFATAL("Can't get the HOME variable environment!");
	}
	return path;
}

std::string Parameters::getVersion()
{
	return RTABMAP_VERSION;
	return ""; // Second return only to avoid compiler warning with RTABMAP_VERSION not yet set.
}

std::string Parameters::getDefaultDatabaseName()
{
	return "rtabmap.db";
}

std::string Parameters::serialize(const ParametersMap & parameters)
{
	std::stringstream output;
	for(ParametersMap::const_iterator iter=parameters.begin(); iter!=parameters.end(); ++iter)
	{
		if(iter != parameters.begin())
		{
			output << ";";
		}
		// make sure there are no commas instead of dots
		output << iter->first << ":" << uReplaceChar(iter->second, ',', '.');
	}
	UDEBUG("output=%s", output.str().c_str());
	return output.str();
}

ParametersMap Parameters::deserialize(const std::string & parameters)
{
	UDEBUG("parameters=%s", parameters.c_str());
	ParametersMap output;
	std::list<std::string> tuplets = uSplit(parameters, ';');
	for(std::list<std::string>::iterator iter=tuplets.begin(); iter!=tuplets.end(); ++iter)
	{
		std::list<std::string> p = uSplit(*iter, ':');
		if(p.size() == 2)
		{
			std::string key = p.front();
			std::string value = p.back();

			// look for old parameter name
			bool addParameter = true;
			std::map<std::string, std::pair<bool, std::string> >::const_iterator oldIter = Parameters::getRemovedParameters().find(key);
			if(oldIter!=Parameters::getRemovedParameters().end())
			{
				addParameter = oldIter->second.first;
				if(addParameter)
				{
					key = oldIter->second.second;
					UWARN("Parameter migration from \"%s\" to \"%s\" (value=%s).",
							oldIter->first.c_str(), oldIter->second.second.c_str(), value.c_str());
				}
				else if(oldIter->second.second.empty())
				{
					UWARN("Parameter \"%s\" doesn't exist anymore.",
								oldIter->first.c_str());
				}
				else
				{
					UWARN("Parameter \"%s\" doesn't exist anymore, you may want to use this similar parameter \"%s\":\"%s\".",
								oldIter->first.c_str(), oldIter->second.second.c_str(), Parameters::getDescription(oldIter->second.second).c_str());
				}

			}

			if(Parameters::getDefaultParameters().find(key) == Parameters::getDefaultParameters().end())
			{
				UWARN("Unknown parameter \"%s\"=\"%s\"! The parameter is still added to output map.", key.c_str(), value.c_str());
			}
			uInsert(output, ParametersPair(key, value));
		}
	}
	return output;
}

bool Parameters::isFeatureParameter(const std::string & parameter)
{
	std::string group = uSplit(parameter, '/').front();
	return  group.compare("SURF") == 0 ||
			group.compare("SIFT") == 0 ||
			group.compare("ORB") == 0 ||
			group.compare("FAST") == 0 ||
			group.compare("FREAK") == 0 ||
			group.compare("BRIEF") == 0 ||
			group.compare("GFTT") == 0 ||
			group.compare("BRISK") == 0 ||
			group.compare("KAZE") == 0;
}

rtabmap::ParametersMap Parameters::getDefaultOdometryParameters(bool stereo, bool vis, bool icp)
{
	rtabmap::ParametersMap odomParameters;
	rtabmap::ParametersMap defaultParameters = rtabmap::Parameters::getDefaultParameters();
	for(rtabmap::ParametersMap::iterator iter=defaultParameters.begin(); iter!=defaultParameters.end(); ++iter)
	{
		std::string group = uSplit(iter->first, '/').front();
		if(uStrContains(group, "Odom") ||
			(stereo && group.compare("Stereo") == 0) ||
			(icp && group.compare("Icp") == 0) ||
			(vis && Parameters::isFeatureParameter(iter->first)) ||
			group.compare("Reg") == 0 ||
			(vis && group.compare("Vis") == 0))
		{
			if(stereo)
			{
				if(iter->first.compare(Parameters::kVisEstimationType()) == 0)
				{
					iter->second = "1"; // 3D->2D (PNP)
				}
			}
			odomParameters.insert(*iter);
		}
	}
	return odomParameters;
}

ParametersMap Parameters::getDefaultParameters(const std::string & groupIn)
{
	rtabmap::ParametersMap parameters;
	const rtabmap::ParametersMap & defaultParameters = rtabmap::Parameters::getDefaultParameters();
	for(rtabmap::ParametersMap::const_iterator iter=defaultParameters.begin(); iter!=defaultParameters.end(); ++iter)
	{
		UASSERT(uSplit(iter->first, '/').size()  == 2);
		std::string group = uSplit(iter->first, '/').front();
		if(group.compare(groupIn) == 0)
		{
			parameters.insert(*iter);
		}
	}
	UASSERT_MSG(parameters.size(), uFormat("No parameters found for group %s!", groupIn.c_str()).c_str());
	return parameters;
}

ParametersMap Parameters::filterParameters(const ParametersMap & parameters, const std::string & groupIn)
{
	ParametersMap output;
	for(rtabmap::ParametersMap::const_iterator iter=parameters.begin(); iter!=parameters.end(); ++iter)
	{
		UASSERT(uSplit(iter->first, '/').size()  == 2);
		std::string group = uSplit(iter->first, '/').front();
		if(group.compare(groupIn) == 0)
		{
			output.insert(*iter);
		}
	}
	return output;
}

const std::map<std::string, std::pair<bool, std::string> > & Parameters::getRemovedParameters()
{
	if(removedParameters_.empty())
	{
		// removed parameters

		// 0.13.3
		removedParameters_.insert(std::make_pair("Icp/PointToPlaneNormalNeighbors", std::make_pair(true,  Parameters::kIcpPointToPlaneK())));


		// 0.13.1
		removedParameters_.insert(std::make_pair("Rtabmap/VhStrategy",            std::make_pair(true,  Parameters::kVhEpEnabled())));

		// 0.12.5
		removedParameters_.insert(std::make_pair("Grid/FullUpdate",               std::make_pair(true,  Parameters::kGridGlobalFullUpdate())));

		// 0.12.1
		removedParameters_.insert(std::make_pair("Grid/3DGroundIsObstacle",       std::make_pair(true,  Parameters::kGridGroundIsObstacle())));

		// 0.11.12
		removedParameters_.insert(std::make_pair("Optimizer/Slam2D",              std::make_pair(true,  Parameters::kRegForce3DoF())));
		removedParameters_.insert(std::make_pair("OdomF2M/FixedMapPath",          std::make_pair(false,  "")));

		// 0.11.10 typos
		removedParameters_.insert(std::make_pair("Grid/FlatObstaclesDetected",    std::make_pair(true,  Parameters::kGridFlatObstacleDetected())));

		// 0.11.8
		removedParameters_.insert(std::make_pair("Reg/Force2D",                   std::make_pair(true,  Parameters::kRegForce3DoF())));
		removedParameters_.insert(std::make_pair("OdomF2M/ScanSubstractRadius",   std::make_pair(true,  Parameters::kOdomF2MScanSubtractRadius())));

		// 0.11.6
		removedParameters_.insert(std::make_pair("RGBD/ProximityPathScansMerged", std::make_pair(false,  "")));

		// 0.11.3
		removedParameters_.insert(std::make_pair("Mem/ImageDecimation",           std::make_pair(true, Parameters::kMemImagePostDecimation())));

		// 0.11.2
		removedParameters_.insert(std::make_pair("OdomLocalMap/HistorySize",      std::make_pair(true, Parameters::kOdomF2MMaxSize())));
		removedParameters_.insert(std::make_pair("OdomLocalMap/FixedMapPath",     std::make_pair(false, "")));
		removedParameters_.insert(std::make_pair("OdomF2F/GuessMotion",           std::make_pair(true, Parameters::kOdomGuessMotion())));
		removedParameters_.insert(std::make_pair("OdomF2F/KeyFrameThr",           std::make_pair(false, Parameters::kOdomKeyFrameThr())));

		// 0.11.0
		removedParameters_.insert(std::make_pair("OdomBow/LocalHistorySize",      std::make_pair(true, Parameters::kOdomF2MMaxSize())));
		removedParameters_.insert(std::make_pair("OdomBow/FixedLocalMapPath",     std::make_pair(false, "")));
		removedParameters_.insert(std::make_pair("OdomFlow/KeyFrameThr",          std::make_pair(false, Parameters::kOdomKeyFrameThr())));
		removedParameters_.insert(std::make_pair("OdomFlow/GuessMotion",          std::make_pair(true, Parameters::kOdomGuessMotion())));

		removedParameters_.insert(std::make_pair("Kp/WordsPerImage",              std::make_pair(true, Parameters::kKpMaxFeatures())));

		removedParameters_.insert(std::make_pair("Mem/LocalSpaceLinksKeptInWM",   std::make_pair(false, "")));

		removedParameters_.insert(std::make_pair("RGBD/PoseScanMatching",         std::make_pair(true,  Parameters::kRGBDNeighborLinkRefining())));

		removedParameters_.insert(std::make_pair("Odom/ParticleFiltering",        std::make_pair(false, Parameters::kOdomFilteringStrategy())));
		removedParameters_.insert(std::make_pair("Odom/FeatureType",              std::make_pair(true,  Parameters::kVisFeatureType())));
		removedParameters_.insert(std::make_pair("Odom/EstimationType",           std::make_pair(true,  Parameters::kVisEstimationType())));
		removedParameters_.insert(std::make_pair("Odom/MaxFeatures",              std::make_pair(true,  Parameters::kVisMaxFeatures())));
		removedParameters_.insert(std::make_pair("Odom/InlierDistance",           std::make_pair(true,  Parameters::kVisInlierDistance())));
		removedParameters_.insert(std::make_pair("Odom/MinInliers",               std::make_pair(true,  Parameters::kVisMinInliers())));
		removedParameters_.insert(std::make_pair("Odom/Iterations",               std::make_pair(true,  Parameters::kVisIterations())));
		removedParameters_.insert(std::make_pair("Odom/RefineIterations",         std::make_pair(true,  Parameters::kVisRefineIterations())));
		removedParameters_.insert(std::make_pair("Odom/MaxDepth",                 std::make_pair(true,  Parameters::kVisMaxDepth())));
		removedParameters_.insert(std::make_pair("Odom/RoiRatios",                std::make_pair(true,  Parameters::kVisRoiRatios())));
		removedParameters_.insert(std::make_pair("Odom/Force2D",                  std::make_pair(true,  Parameters::kRegForce3DoF())));
		removedParameters_.insert(std::make_pair("Odom/VarianceFromInliersCount", std::make_pair(true,  Parameters::kRegVarianceFromInliersCount())));
		removedParameters_.insert(std::make_pair("Odom/PnPReprojError",           std::make_pair(true,  Parameters::kVisPnPReprojError())));
		removedParameters_.insert(std::make_pair("Odom/PnPFlags",                 std::make_pair(true,  Parameters::kVisPnPFlags())));

		removedParameters_.insert(std::make_pair("OdomBow/NNType",                std::make_pair(true,  Parameters::kVisCorNNType())));
		removedParameters_.insert(std::make_pair("OdomBow/NNDR",                  std::make_pair(true,  Parameters::kVisCorNNDR())));

		removedParameters_.insert(std::make_pair("OdomFlow/WinSize",              std::make_pair(true,  Parameters::kVisCorFlowWinSize())));
		removedParameters_.insert(std::make_pair("OdomFlow/Iterations",           std::make_pair(true,  Parameters::kVisCorFlowIterations())));
		removedParameters_.insert(std::make_pair("OdomFlow/Eps",                  std::make_pair(true,  Parameters::kVisCorFlowEps())));
		removedParameters_.insert(std::make_pair("OdomFlow/MaxLevel",             std::make_pair(true,  Parameters::kVisCorFlowMaxLevel())));

		removedParameters_.insert(std::make_pair("OdomSubPix/WinSize",            std::make_pair(true,  Parameters::kVisSubPixWinSize())));
		removedParameters_.insert(std::make_pair("OdomSubPix/Iterations",         std::make_pair(true,  Parameters::kVisSubPixIterations())));
		removedParameters_.insert(std::make_pair("OdomSubPix/Eps",                std::make_pair(true,  Parameters::kVisSubPixEps())));

		removedParameters_.insert(std::make_pair("LccReextract/Activated",         std::make_pair(true,   Parameters::kRGBDLoopClosureReextractFeatures())));
		removedParameters_.insert(std::make_pair("LccReextract/FeatureType",       std::make_pair(false,  Parameters::kVisFeatureType())));
		removedParameters_.insert(std::make_pair("LccReextract/MaxWords",          std::make_pair(false,  Parameters::kVisMaxFeatures())));
		removedParameters_.insert(std::make_pair("LccReextract/MaxDepth",          std::make_pair(false,  Parameters::kVisMaxDepth())));
		removedParameters_.insert(std::make_pair("LccReextract/RoiRatios",         std::make_pair(false,  Parameters::kVisRoiRatios())));
		removedParameters_.insert(std::make_pair("LccReextract/NNType",            std::make_pair(false,  Parameters::kVisCorNNType())));
		removedParameters_.insert(std::make_pair("LccReextract/NNDR",              std::make_pair(false,  Parameters::kVisCorNNDR())));

		removedParameters_.insert(std::make_pair("LccBow/EstimationType",           std::make_pair(false,  Parameters::kVisEstimationType())));
		removedParameters_.insert(std::make_pair("LccBow/InlierDistance",           std::make_pair(false,  Parameters::kVisInlierDistance())));
		removedParameters_.insert(std::make_pair("LccBow/MinInliers",               std::make_pair(false,  Parameters::kVisMinInliers())));
		removedParameters_.insert(std::make_pair("LccBow/Iterations",               std::make_pair(false,  Parameters::kVisIterations())));
		removedParameters_.insert(std::make_pair("LccBow/RefineIterations",         std::make_pair(false,  Parameters::kVisRefineIterations())));
		removedParameters_.insert(std::make_pair("LccBow/Force2D",                  std::make_pair(false,  Parameters::kRegForce3DoF())));
		removedParameters_.insert(std::make_pair("LccBow/VarianceFromInliersCount", std::make_pair(false,  Parameters::kRegVarianceFromInliersCount())));
		removedParameters_.insert(std::make_pair("LccBow/PnPReprojError",           std::make_pair(false,  Parameters::kVisPnPReprojError())));
		removedParameters_.insert(std::make_pair("LccBow/PnPFlags",                 std::make_pair(false,  Parameters::kVisPnPFlags())));
		removedParameters_.insert(std::make_pair("LccBow/EpipolarGeometryVar",      std::make_pair(true,   Parameters::kVisEpipolarGeometryVar())));

		removedParameters_.insert(std::make_pair("LccIcp/Type",                         std::make_pair(false,  Parameters::kRegStrategy())));

		removedParameters_.insert(std::make_pair("LccIcp3/Decimation",                  std::make_pair(false, "")));
		removedParameters_.insert(std::make_pair("LccIcp3/MaxDepth",                    std::make_pair(false, "")));
		removedParameters_.insert(std::make_pair("LccIcp3/VoxelSize",                   std::make_pair(false, Parameters::kIcpVoxelSize())));
		removedParameters_.insert(std::make_pair("LccIcp3/Samples",                     std::make_pair(false, Parameters::kIcpDownsamplingStep())));
		removedParameters_.insert(std::make_pair("LccIcp3/MaxCorrespondenceDistance",   std::make_pair(false, Parameters::kIcpMaxCorrespondenceDistance())));
		removedParameters_.insert(std::make_pair("LccIcp3/Iterations",                  std::make_pair(false, Parameters::kIcpIterations())));
		removedParameters_.insert(std::make_pair("LccIcp3/CorrespondenceRatio",         std::make_pair(false, Parameters::kIcpCorrespondenceRatio())));
		removedParameters_.insert(std::make_pair("LccIcp3/PointToPlane",                std::make_pair(true,  Parameters::kIcpPointToPlane())));
		removedParameters_.insert(std::make_pair("LccIcp3/PointToPlaneNormalNeighbors", std::make_pair(true,  Parameters::kIcpPointToPlaneK())));

		removedParameters_.insert(std::make_pair("LccIcp2/MaxCorrespondenceDistance",   std::make_pair(true,  Parameters::kIcpMaxCorrespondenceDistance())));
		removedParameters_.insert(std::make_pair("LccIcp2/Iterations",                  std::make_pair(true,  Parameters::kIcpIterations())));
		removedParameters_.insert(std::make_pair("LccIcp2/CorrespondenceRatio",         std::make_pair(true,  Parameters::kIcpCorrespondenceRatio())));
		removedParameters_.insert(std::make_pair("LccIcp2/VoxelSize",                   std::make_pair(true,  Parameters::kIcpVoxelSize())));

		removedParameters_.insert(std::make_pair("RGBD/LocalLoopDetectionByTime",              std::make_pair(true,  Parameters::kRGBDProximityByTime())));
		removedParameters_.insert(std::make_pair("RGBD/LocalLoopDetectionBySpace",             std::make_pair(true,  Parameters::kRGBDProximityBySpace())));
		removedParameters_.insert(std::make_pair("RGBD/LocalLoopDetectionTime",                std::make_pair(true,  Parameters::kRGBDProximityByTime())));
		removedParameters_.insert(std::make_pair("RGBD/LocalLoopDetectionSpace",               std::make_pair(true,  Parameters::kRGBDProximityBySpace())));
		removedParameters_.insert(std::make_pair("RGBD/LocalLoopDetectionPathScansMerged",     std::make_pair(false,  "")));
		removedParameters_.insert(std::make_pair("RGBD/LocalLoopDetectionMaxGraphDepth",       std::make_pair(true,  Parameters::kRGBDProximityMaxGraphDepth())));
		removedParameters_.insert(std::make_pair("RGBD/LocalLoopDetectionPathFilteringRadius", std::make_pair(true,  Parameters::kRGBDProximityPathFilteringRadius())));
		removedParameters_.insert(std::make_pair("RGBD/LocalLoopDetectionPathRawPosesUsed",    std::make_pair(true,  Parameters::kRGBDProximityPathRawPosesUsed())));

		removedParameters_.insert(std::make_pair("RGBD/OptimizeStrategy",             std::make_pair(true,  Parameters::kOptimizerStrategy())));
		removedParameters_.insert(std::make_pair("RGBD/OptimizeEpsilon",             std::make_pair(true,  Parameters::kOptimizerEpsilon())));
		removedParameters_.insert(std::make_pair("RGBD/OptimizeIterations",          std::make_pair(true,  Parameters::kOptimizerIterations())));
		removedParameters_.insert(std::make_pair("RGBD/OptimizeRobust",              std::make_pair(true,  Parameters::kOptimizerRobust())));
		removedParameters_.insert(std::make_pair("RGBD/OptimizeSlam2D",              std::make_pair(true,  Parameters::kRegForce3DoF())));
		removedParameters_.insert(std::make_pair("RGBD/OptimizeSlam2d",              std::make_pair(true,  Parameters::kRegForce3DoF())));
		removedParameters_.insert(std::make_pair("RGBD/OptimizeVarianceIgnored",     std::make_pair(true,  Parameters::kOptimizerVarianceIgnored())));

		removedParameters_.insert(std::make_pair("Stereo/WinSize",                   std::make_pair(true,  Parameters::kStereoWinWidth())));

		// before 0.11.0
		removedParameters_.insert(std::make_pair("GFTT/MaxCorners",                  std::make_pair(true, Parameters::kVisMaxFeatures())));
		removedParameters_.insert(std::make_pair("LccBow/MaxDepth",                  std::make_pair(true, Parameters::kVisMaxDepth())));
		removedParameters_.insert(std::make_pair("LccReextract/LoopClosureFeatures", std::make_pair(true, Parameters::kRGBDLoopClosureReextractFeatures())));
		removedParameters_.insert(std::make_pair("Rtabmap/DetectorStrategy",         std::make_pair(true, Parameters::kKpDetectorStrategy())));
		removedParameters_.insert(std::make_pair("RGBD/ScanMatchingSize",            std::make_pair(true, Parameters::kRGBDNeighborLinkRefining())));
		removedParameters_.insert(std::make_pair("RGBD/LocalLoopDetectionRadius",    std::make_pair(true, Parameters::kRGBDLocalRadius())));
		removedParameters_.insert(std::make_pair("RGBD/ToroIterations",              std::make_pair(true, Parameters::kOptimizerIterations())));
		removedParameters_.insert(std::make_pair("Mem/RehearsedNodesKept",           std::make_pair(true, Parameters::kMemNotLinkedNodesKept())));
		removedParameters_.insert(std::make_pair("RGBD/LocalLoopDetectionMaxDiffID", std::make_pair(true, Parameters::kRGBDProximityMaxGraphDepth())));
		removedParameters_.insert(std::make_pair("RGBD/PlanVirtualLinksMaxDiffID",   std::make_pair(false, "")));
		removedParameters_.insert(std::make_pair("RGBD/LocalLoopDetectionMaxDiffID", std::make_pair(false, "")));
		removedParameters_.insert(std::make_pair("Odom/Type",                        std::make_pair(true, Parameters::kVisFeatureType())));
		removedParameters_.insert(std::make_pair("Odom/MaxWords",                    std::make_pair(true, Parameters::kVisMaxFeatures())));
		removedParameters_.insert(std::make_pair("Odom/LocalHistory",                std::make_pair(true, Parameters::kOdomF2MMaxSize())));
		removedParameters_.insert(std::make_pair("Odom/NearestNeighbor",             std::make_pair(true, Parameters::kVisCorNNType())));
		removedParameters_.insert(std::make_pair("Odom/NNDR",                        std::make_pair(true, Parameters::kVisCorNNDR())));
	}
	return removedParameters_;
}

const ParametersMap & Parameters::getBackwardCompatibilityMap()
{
	if(backwardCompatibilityMap_.empty())
	{
		getRemovedParameters(); // make sure removedParameters is filled

		// compatibility
		for(std::map<std::string, std::pair<bool, std::string> >::iterator iter=removedParameters_.begin();
			iter!=removedParameters_.end();
			++iter)
		{
			if(iter->second.first)
			{
				backwardCompatibilityMap_.insert(ParametersPair(iter->second.second, iter->first));
			}
		}
	}
	return backwardCompatibilityMap_;
}

std::string Parameters::getType(const std::string & paramKey)
{
	std::string type;
	ParametersMap::iterator iter = parametersType_.find(paramKey);
	if(iter != parametersType_.end())
	{
		type = iter->second;
	}
	else
	{
		UERROR("Parameters \"%s\" doesn't exist!", paramKey.c_str());
	}
	return type;
}

std::string Parameters::getDescription(const std::string & paramKey)
{
	std::string description;
	ParametersMap::iterator iter = descriptions_.find(paramKey);
	if(iter != descriptions_.end())
	{
		description = iter->second;
	}
	else
	{
		UERROR("Parameters \"%s\" doesn't exist!", paramKey.c_str());
	}
	return description;
}

bool Parameters::parse(const ParametersMap & parameters, const std::string & key, bool & value)
{
	ParametersMap::const_iterator iter = parameters.find(key);
	if(iter != parameters.end())
	{
		value = uStr2Bool(iter->second.c_str());
		return true;
	}
	return false;
}
bool Parameters::parse(const ParametersMap & parameters, const std::string & key, int & value)
{
	ParametersMap::const_iterator iter = parameters.find(key);
	if(iter != parameters.end())
	{
		value = uStr2Int(iter->second.c_str());
		return true;
	}
	return false;
}
bool Parameters::parse(const ParametersMap & parameters, const std::string & key, unsigned int & value)
{
	ParametersMap::const_iterator iter = parameters.find(key);
	if(iter != parameters.end())
	{
		value = uStr2Int(iter->second.c_str());
		return true;
	}
	return false;
}
bool Parameters::parse(const ParametersMap & parameters, const std::string & key, float & value)
{
	ParametersMap::const_iterator iter = parameters.find(key);
	if(iter != parameters.end())
	{
		value = uStr2Float(iter->second);
		return true;
	}
	return false;
}
bool Parameters::parse(const ParametersMap & parameters, const std::string & key, double & value)
{
	ParametersMap::const_iterator iter = parameters.find(key);
	if(iter != parameters.end())
	{
		value = uStr2Double(iter->second);
		return true;
	}
	return false;
}
bool Parameters::parse(const ParametersMap & parameters, const std::string & key, std::string & value)
{
	ParametersMap::const_iterator iter = parameters.find(key);
	if(iter != parameters.end())
	{
		value = iter->second;
		return true;
	}
	return false;
}
void Parameters::parse(const ParametersMap & parameters, ParametersMap & parametersOut)
{
	for(ParametersMap::iterator iter=parametersOut.begin(); iter!=parametersOut.end(); ++iter)
	{
		ParametersMap::const_iterator jter = parameters.find(iter->first);
		if(jter != parameters.end())
		{
			iter->second = jter->second;
		}
	}
}

const char * Parameters::showUsage()
{
	return  "Logger options:\n"
			"   --nolog              Disable logger\n"
			"   --logconsole         Set logger console type\n"
			"   --logfile \"path\"     Set logger file type\n"
			"   --logfilea \"path\"    Set logger file type with appending mode if the file already exists\n"
			"   --udebug             Set logger level to debug\n"
			"   --uinfo              Set logger level to info\n"
			"   --uwarn              Set logger level to warn\n"
			"   --uerror             Set logger level to error\n"
			"   --logtime \"bool\"     Print time when logging\n"
			"   --logwhere \"bool\"    Print where when logging\n"
			"   --logthread \"bool\"   Print thread id when logging\n"
			"RTAB-Map options:\n"
			"   --params                       Show all parameters with their default value and description\n"
			"   --\"parameter name\" \"value\"     Overwrite a specific RTAB-Map's parameter :\n"
			"                                    --SURF/HessianThreshold 150\n"
			"                                   For parameters in table format, add ',' between values :\n"
			"                                    --Kp/RoiRatios 0,0,0.1,0\n"
			;
}

ParametersMap Parameters::parseArguments(int argc, char * argv[], bool onlyParameters)
{
	ParametersMap out;
	const ParametersMap & parameters = getDefaultParameters();
	const std::map<std::string, std::pair<bool, std::string> > & removedParams = getRemovedParameters();
	for(int i=0;i<argc;++i)
	{
		bool checkParameters = onlyParameters;
		if(!checkParameters)
		{
			if(strcmp(argv[i], "--nolog") == 0)
			{
				ULogger::setType(ULogger::kTypeNoLog);
			}
			else if(strcmp(argv[i], "--logconsole") == 0)
			{
				ULogger::setType(ULogger::kTypeConsole);
			}
			else if(strcmp(argv[i], "--logfile") == 0)
			{
				++i;
				if(i < argc)
				{
					ULogger::setType(ULogger::kTypeFile, argv[i], false);
				}
				else
				{
					UERROR("\"--logfile\" argument requires following file path");
				}
			}
			else if(strcmp(argv[i], "--logfilea") == 0)
			{
				++i;
				if(i < argc)
				{
					ULogger::setType(ULogger::kTypeFile, argv[i], true);
				}
				else
				{
					UERROR("\"--logfilea\" argument requires following file path");
				}
			}
			else if(strcmp(argv[i], "--udebug") == 0)
			{
				ULogger::setLevel(ULogger::kDebug);
			}
			else if(strcmp(argv[i], "--uinfo") == 0)
			{
				ULogger::setLevel(ULogger::kInfo);
			}
			else if(strcmp(argv[i], "--uwarn") == 0)
			{
				ULogger::setLevel(ULogger::kWarning);
			}
			else if(strcmp(argv[i], "--uerror") == 0)
			{
				ULogger::setLevel(ULogger::kError);
			}
			else if(strcmp(argv[i], "--ulogtime") == 0)
			{
				++i;
				if(i < argc)
				{
					ULogger::setPrintTime(uStr2Bool(argv[i]));
				}
				else
				{
					UERROR("\"--ulogtime\" argument requires a following boolean value");
				}
			}
			else if(strcmp(argv[i], "--ulogwhere") == 0)
			{
				++i;
				if(i < argc)
				{
					ULogger::setPrintWhere(uStr2Bool(argv[i]));
				}
				else
				{
					UERROR("\"--ulogwhere\" argument requires a following boolean value");
				}
			}
			else if(strcmp(argv[i], "--ulogthread") == 0)
			{
				++i;
				if(i < argc)
				{
					ULogger::setPrintThreadId(uStr2Bool(argv[i]));
				}
				else
				{
					UERROR("\"--ulogthread\" argument requires a following boolean value");
				}
			}
			else
			{
				checkParameters = true;
			}
		}

		if(checkParameters) // check for parameters
		{
			if(strcmp(argv[i], "--params") == 0)
			{
				for(rtabmap::ParametersMap::const_iterator iter=parameters.begin(); iter!=parameters.end(); ++iter)
				{
					bool ignore = false;
					UASSERT(uSplit(iter->first, '/').size()  == 2);
					std::string group = uSplit(iter->first, '/').front();
#ifndef RTABMAP_GTSAM
				   if(group.compare("GTSAM") == 0)
				   {
					   ignore = true;
				   }
#endif
#ifndef RTABMAP_G2O
					if(group.compare("g2o") == 0)
					{
						ignore = true;
					}
#endif
#ifndef RTABMAP_FOVIS
					if(group.compare("OdomFovis") == 0)
					{
						ignore = true;
					}
#endif
#ifndef RTABMAP_VISO2
					if(group.compare("OdomViso2") == 0)
					{
						ignore = true;
					}
#endif
#ifndef RTABMAP_VISO2
					if(group.compare("OdomORBSLAM2") == 0)
					{
						ignore = true;
					}
#endif
					if(!ignore)
					{
						std::string str = "Param: " + iter->first + " = \"" + iter->second + "\"";
						std::cout <<
								str <<
								std::setw(60 - str.size()) <<
								" [" <<
								rtabmap::Parameters::getDescription(iter->first).c_str() <<
								"]" <<
								std::endl;
					}
				}
				UWARN("App will now exit after showing default RTAB-Map parameters because "
						 "argument \"--params\" is detected!");
				exit(0);
			}
			else
			{
				std::string key = uReplaceChar(argv[i], '-', "");
				ParametersMap::const_iterator iter = parameters.find(key);
				if(iter != parameters.end())
				{
					++i;
					if(i < argc)
					{
						uInsert(out, ParametersPair(iter->first, argv[i]));
						UINFO("Parsed parameter \"%s\"=\"%s\"", iter->first.c_str(), argv[i]);
					}
				}
				else
				{
					// backward compatibility
					std::map<std::string, std::pair<bool, std::string> >::const_iterator jter = removedParams.find(key);
					if(jter!=removedParams.end())
					{
						if(jter->second.first)
						{
							++i;
							if(i < argc)
							{
								std::string value = argv[i];
								if(!value.empty())
								{
									value = uReplaceChar(value, ',', ' '); // for table
									key = jter->second.second;
									UWARN("Parameter migration from \"%s\" to \"%s\" (value=%s).",
											jter->first.c_str(), jter->second.second.c_str(), value.c_str());
									uInsert(out, ParametersPair(key, value));
								}
							}
							else
							{
								UERROR("Value missing for argument \"%s\"", argv[i-1]);
							}
						}
						else if(jter->second.second.empty())
						{
							UERROR("Parameter \"%s\" doesn't exist anymore.", jter->first.c_str());
						}
						else
						{
							UERROR("Parameter \"%s\" doesn't exist anymore, check this similar parameter \"%s\".", jter->first.c_str(), jter->second.second.c_str());
						}
					}
				}
			}
		}
	}
	return out;
}


void Parameters::readINI(const std::string & configFile, ParametersMap & parameters)
{
	CSimpleIniA ini;
	ini.LoadFile(configFile.c_str());
	const CSimpleIniA::TKeyVal * keyValMap = ini.GetSection("Core");
	if(keyValMap)
	{
		for(CSimpleIniA::TKeyVal::const_iterator iter=keyValMap->begin(); iter!=keyValMap->end(); ++iter)
		{
			std::string key = (*iter).first.pItem;
			if(key.compare("Version") == 0)
			{
				// Compare version in ini with the current RTAB-Map version
				std::vector<std::string> version = uListToVector(uSplit((*iter).second, '.'));
				if(version.size() == 3)
				{
					if(!RTABMAP_VERSION_COMPARE(std::atoi(version[0].c_str()), std::atoi(version[1].c_str()), std::atoi(version[2].c_str())))
					{
						if(configFile.find(".rtabmap") != std::string::npos)
						{
							UWARN("Version in the config file \"%s\" is more recent (\"%s\") than "
								   "current RTAB-Map version used (\"%s\"). The config file will be upgraded "
								   "to new version.",
								   configFile.c_str(),
								   (*iter).second,
								   RTABMAP_VERSION);
						}
						else
						{
							UERROR("Version in the config file \"%s\" is more recent (\"%s\") than "
								   "current RTAB-Map version used (\"%s\"). New parameters (if there are some) will "
								   "be ignored.",
								   configFile.c_str(),
								   (*iter).second,
								   RTABMAP_VERSION);
						}
					}
				}
			}
			else
			{
				key = uReplaceChar(key, '\\', '/'); // Ini files use \ by default for separators, so replace them

				// look for old parameter name
				bool addParameter = true;
				std::map<std::string, std::pair<bool, std::string> >::const_iterator oldIter = Parameters::getRemovedParameters().find(key);
				if(oldIter!=Parameters::getRemovedParameters().end())
				{
					addParameter = oldIter->second.first;
					if(addParameter)
					{
						if(parameters.find(oldIter->second.second) == parameters.end())
						{
							key = oldIter->second.second;
							UWARN("Parameter migration from \"%s\" to \"%s\" (value=%s, default=%s).",
									oldIter->first.c_str(), oldIter->second.second.c_str(), iter->second, Parameters::getDefaultParameters().at(oldIter->second.second).c_str());
						}
					}
					else if(oldIter->second.second.empty())
					{
						UWARN("Parameter \"%s\" doesn't exist anymore.",
									oldIter->first.c_str());
					}
					else if(parameters.find(oldIter->second.second) == parameters.end())
					{
						UWARN("Parameter \"%s\" (value=%s) doesn't exist anymore, you may want to use this similar parameter \"%s (default=%s): %s\".",
									oldIter->first.c_str(), iter->second, oldIter->second.second.c_str(), Parameters::getDefaultParameters().at(oldIter->second.second).c_str(), Parameters::getDescription(oldIter->second.second).c_str());
					}

				}

				if(Parameters::getDefaultParameters().find(key) != Parameters::getDefaultParameters().end())
				{
					uInsert(parameters, ParametersPair(key, iter->second));
				}
			}
		}
	}
	else
	{
		ULOGGER_WARN("Section \"Core\" in %s doesn't exist... "
				    "Ignore this warning if the ini file does not exist yet. "
				    "The ini file will be automatically created when rtabmap will close.", configFile.c_str());
	}
}

void Parameters::writeINI(const std::string & configFile, const ParametersMap & parameters)
{
	CSimpleIniA ini;
	ini.LoadFile(configFile.c_str());

	// Save current version
	ini.SetValue("Core", "Version", RTABMAP_VERSION, NULL, true);

	for(ParametersMap::const_iterator i=parameters.begin(); i!=parameters.end(); ++i)
	{
		std::string key = (*i).first;
		key = uReplaceChar(key, '/', '\\'); // Ini files use \ by default for separators, so replace the /
		
		std::string value = (*i).second.c_str();
		value = uReplaceChar(value, '\\', '/'); // use always slash for values

		ini.SetValue("Core", key.c_str(), value.c_str(), NULL, true);
	}

	ini.SaveFile(configFile.c_str());
}

}
