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

#include "rtabmap/core/Parameters.h"
#include <rtabmap/utilite/UDirectory.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UConversion.h>
#include <math.h>
#include <stdlib.h>
#include <sstream>

namespace rtabmap
{

ParametersMap Parameters::parameters_;
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

std::string Parameters::getDefaultWorkingDirectory()
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

std::string Parameters::getDefaultDatabaseName()
{
	return "rtabmap.db";
}

const std::map<std::string, std::pair<bool, std::string> > & Parameters::getRemovedParameters()
{
	if(removedParameters_.empty())
	{
		// removed parameters

		// 0.11.0
		removedParameters_.insert(std::make_pair("Mem/LaserScanVoxelSize",        std::make_pair(false, Parameters::kMemLaserScanDownsampleStepSize())));
		removedParameters_.insert(std::make_pair("Mem/LocalSpaceLinksKeptInWM",   std::make_pair(false, "")));

		removedParameters_.insert(std::make_pair("RGBD/PoseScanMatching",         std::make_pair(true,  Parameters::kRGBDNeighborLinkRefining())));

		removedParameters_.insert(std::make_pair("Odom/FeatureType",              std::make_pair(true,  Parameters::kVisFeatureType())));
		removedParameters_.insert(std::make_pair("Odom/EstimationType",           std::make_pair(true,  Parameters::kVisEstimationType())));
		removedParameters_.insert(std::make_pair("Odom/MaxFeatures",              std::make_pair(true,  Parameters::kVisMaxFeatures())));
		removedParameters_.insert(std::make_pair("Odom/InlierDistance",           std::make_pair(true,  Parameters::kVisInlierDistance())));
		removedParameters_.insert(std::make_pair("Odom/MinInliers",               std::make_pair(true,  Parameters::kVisMinInliers())));
		removedParameters_.insert(std::make_pair("Odom/Iterations",               std::make_pair(true,  Parameters::kVisIterations())));
		removedParameters_.insert(std::make_pair("Odom/RefineIterations",         std::make_pair(true,  Parameters::kVisRefineIterations())));
		removedParameters_.insert(std::make_pair("Odom/MaxDepth",                 std::make_pair(true,  Parameters::kVisMaxDepth())));
		removedParameters_.insert(std::make_pair("Odom/RoiRatios",                std::make_pair(true,  Parameters::kVisRoiRatios())));
		removedParameters_.insert(std::make_pair("Odom/Force2D",                  std::make_pair(true,  Parameters::kVisForce2D())));
		removedParameters_.insert(std::make_pair("Odom/VarianceFromInliersCount", std::make_pair(true,  Parameters::kRegVarianceFromInliersCount())));
		removedParameters_.insert(std::make_pair("Odom/PnPReprojError",           std::make_pair(true,  Parameters::kVisPnPReprojError())));
		removedParameters_.insert(std::make_pair("Odom/PnPFlags",                 std::make_pair(true,  Parameters::kVisPnPFlags())));

		removedParameters_.insert(std::make_pair("OdomBow/NNType",                std::make_pair(true,  Parameters::kVisNNType())));
		removedParameters_.insert(std::make_pair("OdomBow/NNDR",                  std::make_pair(true,  Parameters::kVisNNDR())));

		removedParameters_.insert(std::make_pair("OdomSubPix/WinSize",            std::make_pair(true,  Parameters::kVisSubPixWinSize())));
		removedParameters_.insert(std::make_pair("OdomSubPix/Iterations",         std::make_pair(true,  Parameters::kVisSubPixIterations())));
		removedParameters_.insert(std::make_pair("OdomSubPix/Eps",                std::make_pair(true,  Parameters::kVisSubPixEps())));

		removedParameters_.insert(std::make_pair("LccReextract/Activated",         std::make_pair(true,   Parameters::kRGBDLoopClosureReextractFeatures())));
		removedParameters_.insert(std::make_pair("LccReextract/FeatureType",       std::make_pair(false,  Parameters::kVisFeatureType())));
		removedParameters_.insert(std::make_pair("LccReextract/MaxWords",          std::make_pair(false,  Parameters::kVisMaxFeatures())));
		removedParameters_.insert(std::make_pair("LccReextract/MaxDepth",          std::make_pair(false,  Parameters::kVisMaxDepth())));
		removedParameters_.insert(std::make_pair("LccReextract/RoiRatios",         std::make_pair(false,  Parameters::kVisRoiRatios())));
		removedParameters_.insert(std::make_pair("LccReextract/NNType",            std::make_pair(false,  Parameters::kVisNNType())));
		removedParameters_.insert(std::make_pair("LccReextract/NNDR",              std::make_pair(false,  Parameters::kVisNNDR())));

		removedParameters_.insert(std::make_pair("LccBow/EstimationType",           std::make_pair(false,  Parameters::kVisEstimationType())));
		removedParameters_.insert(std::make_pair("LccBow/InlierDistance",           std::make_pair(false,  Parameters::kVisInlierDistance())));
		removedParameters_.insert(std::make_pair("LccBow/MinInliers",               std::make_pair(false,  Parameters::kVisMinInliers())));
		removedParameters_.insert(std::make_pair("LccBow/Iterations",               std::make_pair(false,  Parameters::kVisIterations())));
		removedParameters_.insert(std::make_pair("LccBow/RefineIterations",         std::make_pair(false,  Parameters::kVisRefineIterations())));
		removedParameters_.insert(std::make_pair("LccBow/Force2D",                  std::make_pair(false,  Parameters::kVisForce2D())));
		removedParameters_.insert(std::make_pair("LccBow/VarianceFromInliersCount", std::make_pair(false,  Parameters::kRegVarianceFromInliersCount())));
		removedParameters_.insert(std::make_pair("LccBow/PnPReprojError",           std::make_pair(false,  Parameters::kVisPnPReprojError())));
		removedParameters_.insert(std::make_pair("LccBow/PnPFlags",                 std::make_pair(false,  Parameters::kVisPnPFlags())));
		removedParameters_.insert(std::make_pair("LccBow/EpipolarGeometryVar",      std::make_pair(true,   Parameters::kVisEpipolarGeometryVar())));

		removedParameters_.insert(std::make_pair("LccIcp/Type",                         std::make_pair(true,  Parameters::kRGBDLoopClosureLinkRefining())));

		removedParameters_.insert(std::make_pair("LccIcp3/Decimation",                  std::make_pair(false, "")));
		removedParameters_.insert(std::make_pair("LccIcp3/MaxDepth",                    std::make_pair(false, "")));
		removedParameters_.insert(std::make_pair("LccIcp3/VoxelSize",                   std::make_pair(false, Parameters::kIcpVoxelSize())));
		removedParameters_.insert(std::make_pair("LccIcp3/Samples",                     std::make_pair(false, Parameters::kIcpDownsamplingStep())));
		removedParameters_.insert(std::make_pair("LccIcp3/MaxCorrespondenceDistance",   std::make_pair(false, Parameters::kIcpMaxCorrespondenceDistance())));
		removedParameters_.insert(std::make_pair("LccIcp3/Iterations",                  std::make_pair(false, Parameters::kIcpIterations())));
		removedParameters_.insert(std::make_pair("LccIcp3/CorrespondenceRatio",         std::make_pair(false, Parameters::kIcpCorrespondenceRatio())));
		removedParameters_.insert(std::make_pair("LccIcp3/PointToPlane",                std::make_pair(true,  Parameters::kIcpPointToPlane())));
		removedParameters_.insert(std::make_pair("LccIcp3/PointToPlaneNormalNeighbors", std::make_pair(true,  Parameters::kIcpPointToPlaneNormalNeighbors())));

		removedParameters_.insert(std::make_pair("LccIcp2/MaxCorrespondenceDistance",   std::make_pair(true,  Parameters::kIcpMaxCorrespondenceDistance())));
		removedParameters_.insert(std::make_pair("LccIcp2/Iterations",                  std::make_pair(true,  Parameters::kIcpIterations())));
		removedParameters_.insert(std::make_pair("LccIcp2/CorrespondenceRatio",         std::make_pair(true,  Parameters::kIcpCorrespondenceRatio())));
		removedParameters_.insert(std::make_pair("LccIcp2/VoxelSize",                   std::make_pair(true,  Parameters::kIcpVoxelSize())));

		removedParameters_.insert(std::make_pair("RGBD/LocalLoopDetectionByTime",              std::make_pair(true,  Parameters::kRGBDProximityByTime())));
		removedParameters_.insert(std::make_pair("RGBD/LocalLoopDetectionBySpace",             std::make_pair(true,  Parameters::kRGBDProximityBySpace())));
		removedParameters_.insert(std::make_pair("RGBD/LocalLoopDetectionPathScansMerged",     std::make_pair(true,  Parameters::kRGBDProximityPathScansMerged())));
		removedParameters_.insert(std::make_pair("RGBD/LocalLoopDetectionMaxGraphDepth",       std::make_pair(true,  Parameters::kRGBDProximityMaxGraphDepth())));
		removedParameters_.insert(std::make_pair("RGBD/LocalLoopDetectionPathFilteringRadius", std::make_pair(true,  Parameters::kRGBDProximityPathFilteringRadius())));
		removedParameters_.insert(std::make_pair("RGBD/LocalLoopDetectionPathRawPosesUsed",    std::make_pair(true,  Parameters::kRGBDProximityPathRawPosesUsed())));

		// before 0.11.0
		removedParameters_.insert(std::make_pair("GFTT/MaxCorners",                  std::make_pair(true, Parameters::kVisMaxFeatures())));
		removedParameters_.insert(std::make_pair("LccBow/MaxDepth",                  std::make_pair(true, Parameters::kVisMaxDepth())));
		removedParameters_.insert(std::make_pair("LccReextract/LoopClosureFeatures", std::make_pair(true, Parameters::kRGBDLoopClosureReextractFeatures())));
		removedParameters_.insert(std::make_pair("Rtabmap/DetectorStrategy",         std::make_pair(true, Parameters::kKpDetectorStrategy())));
		removedParameters_.insert(std::make_pair("RGBD/ScanMatchingSize",            std::make_pair(true, Parameters::kRGBDNeighborLinkRefining())));
		removedParameters_.insert(std::make_pair("RGBD/LocalLoopDetectionRadius",    std::make_pair(true, Parameters::kRGBDLocalRadius())));
		removedParameters_.insert(std::make_pair("RGBD/ToroIterations",              std::make_pair(true, Parameters::kRGBDOptimizeIterations())));
		removedParameters_.insert(std::make_pair("Mem/RehearsedNodesKept",           std::make_pair(true, Parameters::kMemNotLinkedNodesKept())));
		removedParameters_.insert(std::make_pair("RGBD/LocalLoopDetectionMaxDiffID", std::make_pair(true, Parameters::kRGBDProximityMaxGraphDepth())));
		removedParameters_.insert(std::make_pair("RGBD/PlanVirtualLinksMaxDiffID",   std::make_pair(false, "")));
		removedParameters_.insert(std::make_pair("RGBD/LocalLoopDetectionMaxDiffID", std::make_pair(false, "")));
		removedParameters_.insert(std::make_pair("Odom/Type",                        std::make_pair(true, Parameters::kVisFeatureType())));
		removedParameters_.insert(std::make_pair("Odom/MaxWords",                    std::make_pair(true, Parameters::kVisMaxFeatures())));
		removedParameters_.insert(std::make_pair("Odom/LocalHistory",                std::make_pair(true, Parameters::kOdomBowLocalHistorySize())));
		removedParameters_.insert(std::make_pair("Odom/NearestNeighbor",             std::make_pair(true, Parameters::kVisNNType())));
		removedParameters_.insert(std::make_pair("Odom/NNDR",                        std::make_pair(true, Parameters::kVisNNDR())));
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

void Parameters::parse(const ParametersMap & parameters, const std::string & key, bool & value)
{
	ParametersMap::const_iterator iter = parameters.find(key);
	if(iter != parameters.end())
	{
		value = uStr2Bool(iter->second.c_str());
	}
}
void Parameters::parse(const ParametersMap & parameters, const std::string & key, int & value)
{
	ParametersMap::const_iterator iter = parameters.find(key);
	if(iter != parameters.end())
	{
		value = atoi(iter->second.c_str());
	}
}
void Parameters::parse(const ParametersMap & parameters, const std::string & key, unsigned int & value)
{
	ParametersMap::const_iterator iter = parameters.find(key);
	if(iter != parameters.end())
	{
		value = atoi(iter->second.c_str());
	}
}
void Parameters::parse(const ParametersMap & parameters, const std::string & key, float & value)
{
	ParametersMap::const_iterator iter = parameters.find(key);
	if(iter != parameters.end())
	{
		value = uStr2Float(iter->second);
	}
}
void Parameters::parse(const ParametersMap & parameters, const std::string & key, double & value)
{
	ParametersMap::const_iterator iter = parameters.find(key);
	if(iter != parameters.end())
	{
		value = uStr2Double(iter->second);
	}
}
void Parameters::parse(const ParametersMap & parameters, const std::string & key, std::string & value)
{
	ParametersMap::const_iterator iter = parameters.find(key);
	if(iter != parameters.end())
	{
		value = iter->second;
	}
}

}
