/*
 * Odometry.h
 *
 *  Created on: 2013-08-23
 *      Author: Mathieu
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
	Transform process(Image & image);
	virtual void reset();

	bool isLargeEnoughTransform(const Transform & transform);

	//getters
	int getMaxFeatures() const  {return _maxFeatures;}
	int getMinInliers() const {return _minInliers;}
	float getInlierDistance() const {return _inlierDistance;}
	int getIterations() const {return _iterations;}
	float getMaxDepth() const {return _maxDepth;}
	float geLinearUpdate() const {return _linearUpdate;}
	float getAngularUpdate() const {return _angularUpdate;}

private:
	virtual Transform computeTransform(Image & image) = 0;

private:
	int _maxFeatures;
	int _minInliers;
	float _inlierDistance;
	int _iterations;
	float _maxDepth;
	float _linearUpdate;
	float _angularUpdate;
	int _resetCountdown;
	Transform _pose;
	int _resetCurrentCount;

protected:
	Odometry(float inlierDistance = Parameters::defaultOdomInlierDistance(),
			int maxWords = Parameters::defaultOdomMaxWords(),
			int minInliers = Parameters::defaultOdomMinInliers(),
			int iterations = Parameters::defaultOdomIterations(),
			float maxDepth = Parameters::defaultOdomMaxDepth(),
			float linearUpdate = Parameters::defaultOdomLinearUpdate(),
			float angularUpdate = Parameters::defaultOdomAngularUpdate(),
			int resetCountDown = Parameters::defaultOdomResetCountdown());
	Odometry(const rtabmap::ParametersMap & parameters);
};

class RTABMAP_EXP OdometryBinary : public Odometry
{
public:
	OdometryBinary(
			float inlierDistance = Parameters::defaultOdomInlierDistance(),
			int maxWords = Parameters::defaultOdomMaxWords(),
			int minInliers = Parameters::defaultOdomMinInliers(),
			int iterations = Parameters::defaultOdomIterations(),
			float maxDepth = Parameters::defaultOdomMaxDepth(),
			float linearUpdate = Parameters::defaultOdomLinearUpdate(),
			float angularUpdate = Parameters::defaultOdomAngularUpdate(),
			int resetCountdown = Parameters::defaultOdomResetCountdown(),
			int briefBytes = Parameters::defaultOdomBinBriefBytes(),
			int fastThreshold = Parameters::defaultOdomBinFastThreshold(),
			bool fastNonmaxSuppression = Parameters::defaultOdomBinFastNonmaxSuppression(),
			bool bruteForceMatching = Parameters::defaultOdomBinBruteForceMatching());
	OdometryBinary(const rtabmap::ParametersMap & parameters);
	virtual ~OdometryBinary() {}
	virtual void reset();

private:
	virtual Transform computeTransform(Image & image);

private:
	int _briefBytes;
	int _fastThreshold;
	bool _fastNonmaxSuppression;
	bool _bruteForceMatching;

	std::vector<cv::KeyPoint> _lastKeypoints;
	cv::Mat _lastDescriptors;
	cv::Mat _lastDepth;
};

class Memory;

class RTABMAP_EXP OdometryBOW : public Odometry
{
public:
	OdometryBOW(
			int detectorType = Parameters::defaultKpDetectorStrategy(), // 0=SURF or 1=SIFT
			float inlierDistance = Parameters::defaultOdomInlierDistance(),
			int maxWords = Parameters::defaultOdomMaxWords(),
			int minInliers = Parameters::defaultOdomMinInliers(),
			int iterations = Parameters::defaultOdomIterations(),
			float maxDepth = Parameters::defaultOdomMaxDepth(),
			float linearUpdate = Parameters::defaultOdomLinearUpdate(),
			float angularUpdate = Parameters::defaultOdomAngularUpdate(),
			int resetCoutdown = Parameters::defaultOdomResetCountdown(),
			float surfHessianThreshold = Parameters::defaultSURFHessianThreshold(),
			float nndr = Parameters::defaultKpNndrRatio()); // nearest neighbor distance ratio
	OdometryBOW(const rtabmap::ParametersMap & parameters);
	virtual ~OdometryBOW();

	virtual void reset();

private:
	virtual Transform computeTransform(Image & image);

private:
	Memory * _memory;
};

class RTABMAP_EXP OdometryICP : public Odometry
{
public:
	OdometryICP(
			int decimation = Parameters::defaultOdomICPDecimation(),
			float voxelSize = Parameters::defaultOdomICPVoxelSize(),
			float samples = Parameters::defaultOdomICPSamples(),
			float maxCorrespondenceDistance = Parameters::defaultOdomICPCorrespondencesDistance(),
			int	maxIterations = Parameters::defaultOdomICPIterations(),
			float maxFitness = Parameters::defaultOdomICPMaxFitness(),
			float maxDepth = Parameters::defaultOdomMaxDepth(),
			float linearUpdate = Parameters::defaultOdomLinearUpdate(),
			float angularUpdate = Parameters::defaultOdomAngularUpdate(),
			int resetCoutdown = Parameters::defaultOdomResetCountdown());
	OdometryICP(const ParametersMap & parameters);
	void reset();

private:
	virtual Transform computeTransform(Image & image);

private:
	int _decimation;
	float _voxelSize;
	float _samples;
	float _maxCorrespondenceDistance;
	int	_maxIterations;
	float _maxFitness;

	pcl::PointCloud<pcl::PointNormal>::Ptr _previousCloud;
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
