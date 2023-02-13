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

#ifndef ODOMETRYMONO_H_
#define ODOMETRYMONO_H_

#include <rtabmap/core/Odometry.h>
#include <rtabmap/core/Link.h>

namespace rtabmap {

class Memory;
class Feature2D;

class RTABMAP_CORE_EXPORT OdometryMono : public Odometry
{
public:
	OdometryMono(const rtabmap::ParametersMap & parameters = rtabmap::ParametersMap());
	virtual ~OdometryMono();
	virtual void reset(const Transform & initialPose);
	virtual Odometry::Type getType() {return kTypeUndef;}

private:
	virtual Transform computeTransform(SensorData & data, const Transform & guess = Transform(), OdometryInfo * info = 0);
private:
	//Parameters:
	int flowWinSize_;
	int flowIterations_;
	double flowEps_;
	int flowMaxLevel_;
	int minInliers_;
	int iterations_;
	double pnpReprojError_;
	int pnpFlags_;
	int pnpRefineIterations_;

	Feature2D * feature2D_;

	Memory * memory_;
	int localHistoryMaxSize_;
	float initMinFlow_;
	float initMinTranslation_;
	float minTranslation_;
	float fundMatrixReprojError_;
	float fundMatrixConfidence_;

	std::map<int, cv::Point2f> firstFrameGuessCorners_;
	std::map<int, cv::Point3f> localMap_;
	std::map<int, std::map<int, cv::Point3f> > keyFrameWords3D_;
	std::map<int, Transform> keyFramePoses_;
	std::multimap<int, Link> keyFrameLinks_;
	std::map<int, std::vector<CameraModel> > keyFrameModels_;
	float maxVariance_;
	float keyFrameThr_;
};

}

#endif /* ODOMETRYMONO_H_ */
