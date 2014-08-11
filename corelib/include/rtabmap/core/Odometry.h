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

#ifndef ODOMETRY_H_
#define ODOMETRY_H_

#include <rtabmap/core/RtabmapExp.h>

#include <rtabmap/utilite/UThread.h>
#include <rtabmap/utilite/UEventsHandler.h>
#include <rtabmap/utilite/UEvent.h>
#include <rtabmap/utilite/UMutex.h>
#include <rtabmap/utilite/USemaphore.h>

#include <rtabmap/core/Parameters.h>

#include <rtabmap/core/Image.h>

#include <opencv2/opencv.hpp>

#include <pcl/common/eigen.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class UTimer;

namespace rtabmap {

class RTABMAP_EXP Odometry
{
public:
	virtual ~Odometry() {}
	Transform process(Image & image, int * quality = 0);
	virtual void reset();

	bool isLargeEnoughTransform(const Transform & transform);

	//getters
	const Transform & getPose() const {return _pose;}
	int getMaxFeatures() const  {return _maxFeatures;}
	int getMinInliers() const {return _minInliers;}
	float getInlierDistance() const {return _inlierDistance;}
	int getIterations() const {return _iterations;}
	float getWordsRatio() const {return _wordsRatio;}
	float getMaxDepth() const {return _maxDepth;}
	float geLinearUpdate() const {return _linearUpdate;}
	float getAngularUpdate() const {return _angularUpdate;}
	int getLocalHistory() const {return _localHistory;}

private:
	virtual Transform computeTransform(Image & image, int * quality = 0) = 0;

private:
	int _maxFeatures;
	int _minInliers;
	float _inlierDistance;
	int _iterations;
	float _wordsRatio;
	float _maxDepth;
	float _linearUpdate;
	float _angularUpdate;
	int _resetCountdown;
	int _localHistory;
	Transform _pose;
	int _resetCurrentCount;

protected:
	Odometry(const rtabmap::ParametersMap & parameters);
};

class Memory;

class RTABMAP_EXP OdometryBOW : public Odometry
{
public:
	OdometryBOW(const rtabmap::ParametersMap & parameters = rtabmap::ParametersMap());
	virtual ~OdometryBOW();

	virtual void reset();

private:
	virtual Transform computeTransform(Image & image, int * quality = 0);

private:
	Memory * _memory;
	std::multimap<int, pcl::PointXYZ> localMap_;
};

class RTABMAP_EXP OdometryICP : public Odometry
{
public:
	OdometryICP(int decimation = 4,
			float voxelSize = 0.005f,
			int samples = 0,
			float maxCorrespondenceDistance = 0.05f,
			int maxIterations = 30,
			float maxFitness = 0.01f,
			bool pointToPlane = true,
			const ParametersMap & odometryParameter = rtabmap::ParametersMap());
	void reset();

private:
	virtual Transform computeTransform(Image & image, int * quality = 0);

private:
	int _decimation;
	float _voxelSize;
	float _samples;
	float _maxCorrespondenceDistance;
	int	_maxIterations;
	float _maxFitness;
	bool _pointToPlane;

	pcl::PointCloud<pcl::PointNormal>::Ptr _previousCloudNormal; // for point ot plane
	pcl::PointCloud<pcl::PointXYZ>::Ptr _previousCloud; // for point to point
};

// return true if odometry is correctly computed
Transform computeTransform(Image & image);

class RTABMAP_EXP OdometryThread : public UThread, public UEventsHandler {
public:
	// take ownership of Odometry
	OdometryThread(Odometry * odometry);
	virtual ~OdometryThread();

protected:
	virtual void handleEvent(UEvent * event);

private:
	void mainLoopKill();

	//============================================================
	// MAIN LOOP
	//============================================================
	void mainLoop();
	void addImage(const Image & image);
	void getImage(Image & image);

private:
	USemaphore _imageAdded;
	UMutex _imageMutex;
	Image _imageBuffer;
	Odometry * _odometry;
	bool _resetOdometry;
};

} /* namespace rtabmap */
#endif /* ODOMETRY_H_ */
