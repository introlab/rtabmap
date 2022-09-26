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

#ifndef ODOMETRYF2M_H_
#define ODOMETRYF2M_H_

#include <rtabmap/core/Odometry.h>
#include <rtabmap/core/Optimizer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/pcl_base.h>
#include <rtabmap/core/Link.h>

namespace rtabmap {

class Signature;
class Registration;
class Optimizer;

class RTABMAP_CORE_EXPORT OdometryF2M : public Odometry
{
public:
	OdometryF2M(const rtabmap::ParametersMap & parameters = rtabmap::ParametersMap());
	virtual ~OdometryF2M();

	virtual void reset(const Transform & initialPose = Transform::getIdentity());
	const Signature & getMap() const {return *map_;}
	const Signature & getLastFrame() const {return *lastFrame_;}

	virtual Odometry::Type getType() {return Odometry::kTypeF2M;}

private:
	virtual Transform computeTransform(SensorData & data, const Transform & guess = Transform(), OdometryInfo * info = 0);

private:
	//Parameters
	int maximumMapSize_;
	float keyFrameThr_;
	int visKeyFrameThr_;
	int maxNewFeatures_;
	float scanKeyFrameThr_;
	int scanMaximumMapSize_;
	float scanSubtractRadius_;
	float scanSubtractAngle_;
	float scanMapMaxRange_;
	int bundleAdjustment_;
	int bundleMaxFrames_;
	float validDepthRatio_;
	int pointToPlaneK_;
	float pointToPlaneRadius_;

	Registration * regPipeline_;
	Signature * map_;
	Signature * lastFrame_;
	int lastFrameOldestNewId_;
	std::vector<std::pair<pcl::PointCloud<pcl::PointXYZINormal>::Ptr, pcl::IndicesPtr> > scansBuffer_;

	std::map<int, std::map<int, FeatureBA> > bundleWordReferences_; //<WordId, <FrameId, pt2D+depth>>
	std::map<int, Transform> bundlePoses_;
	std::multimap<int, Link> bundleLinks_;
	std::multimap<int, Link> bundleIMUOrientations_;
	std::map<int, std::vector<CameraModel> > bundleModels_;
	std::map<int, int> bundlePoseReferences_;
	int bundleSeq_;
	Optimizer * sba_;
	ParametersMap parameters_;
};

}

#endif /* ODOMETRYF2M_H_ */
