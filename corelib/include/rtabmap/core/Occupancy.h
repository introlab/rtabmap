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

#ifndef CORELIB_SRC_OCCUPANCY_H_
#define CORELIB_SRC_OCCUPANCY_H_

#include "rtabmap/core/RtabmapExp.h" // DLL export/import defines

#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/Signature.h>

namespace rtabmap {

class RTABMAP_EXP Occupancy
{
public:
	Occupancy(const ParametersMap & parameters = ParametersMap());
	void parseParameters(const ParametersMap & parameters);
	float getCellSize() const {return cellSize_;}
	void segment(const Signature & node, cv::Mat & obstacles, cv::Mat & ground);

private:
	ParametersMap parameters_;
	int cloudDecimation_;
	float cloudMaxDepth_;
	float cloudMinDepth_;
	float cellSize_;
	bool occupancyFromCloud_;
	bool projMapFrame_;
	float maxObstacleHeight_;
	float maxGroundAngle_;
	int minClusterSize_;
	bool flatObstaclesDetected_;
	float maxGroundHeight_;
	bool grid3D_;
	bool groundIsObstacle_;
	float noiseFilteringRadius_;
	int noiseFilteringMinNeighbors_;

};

}

#endif /* CORELIB_SRC_OCCUPANCY_H_ */
