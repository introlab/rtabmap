/*
 * Odometry.cpp
 *
 *  Created on: 2013-08-23
 *      Author: Mathieu
 */

#include "rtabmap/core/Odometry.h"

#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UMath.h>

#include <rtabmap/core/OdometryEvent.h>
#include <rtabmap/core/CameraEvent.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/core/Features2d.h>

#include <rtabmap/core/Memory.h>
#include "rtabmap/core/Signature.h"

#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>

#if _MSC_VER
	#define ISFINITE(value) _finite(value)
#else
	#define ISFINITE(value) std::isfinite(value)
#endif

namespace rtabmap {

Odometry::Odometry(
		float inlierDistance,
		int maxWords,
		int minInliers,
		int iterations,
		float maxDepth,
		float linearUpdate,
		float angularUpdate,
		int resetCoutdown) :
			_maxFeatures(maxWords),
			_minInliers(minInliers),
			_inlierDistance(inlierDistance),
			_iterations(iterations),
			_maxDepth(maxDepth),
			_linearUpdate(linearUpdate),
			_angularUpdate(angularUpdate),
			_resetCountdown(resetCoutdown),
			_pose(Transform::getIdentity()),
			_resetCurrentCount(0)
{

}

Odometry::Odometry(const rtabmap::ParametersMap & parameters) :
		_maxFeatures(Parameters::defaultOdomMaxWords()),
		_minInliers(Parameters::defaultOdomMinInliers()),
		_inlierDistance(Parameters::defaultOdomInlierDistance()),
		_iterations(Parameters::defaultOdomIterations()),
		_maxDepth(Parameters::defaultOdomMaxDepth()),
		_linearUpdate(Parameters::defaultOdomLinearUpdate()),
		_angularUpdate(Parameters::defaultOdomAngularUpdate()),
		_resetCountdown(Parameters::defaultOdomResetCountdown()),
		_pose(Transform::getIdentity()),
		_resetCurrentCount(0)
{
	Parameters::parse(parameters, Parameters::kOdomLinearUpdate(), _linearUpdate);
	Parameters::parse(parameters, Parameters::kOdomAngularUpdate(), _angularUpdate);
	Parameters::parse(parameters, Parameters::kOdomResetCountdown(), _resetCountdown);
	Parameters::parse(parameters, Parameters::kOdomMinInliers(), _minInliers);
	Parameters::parse(parameters, Parameters::kOdomInlierDistance(), _inlierDistance);
	Parameters::parse(parameters, Parameters::kOdomIterations(), _iterations);
	Parameters::parse(parameters, Parameters::kOdomMaxDepth(), _maxDepth);
	Parameters::parse(parameters, Parameters::kOdomMaxWords(), _maxFeatures);
}

void Odometry::reset()
{
	_resetCurrentCount = 0;
	_pose = Transform::getIdentity();
}

bool Odometry::isLargeEnoughTransform(const Transform & transform)
{
	return fabs(transform.x()) > _linearUpdate ||
		   fabs(transform.y()) > _linearUpdate ||
	       fabs(transform.z()) > _linearUpdate;
}

Transform Odometry::process(Image & image)
{
	Transform t = this->computeTransform(image);
	if(!t.isNull())
	{
		_resetCurrentCount = _resetCountdown;

		_pose *= t;
		return _pose;
	}
	else if(_resetCurrentCount > 0)
	{
		UWARN("Odometry lost! Odometry will be reset after next %d consecutive unsuccessful odometry updates...", _resetCurrentCount);

		--_resetCurrentCount;
		if(_resetCurrentCount == 0)
		{
			UWARN("Odometry automatically reset!");
			this->reset();
		}
	}
	return Transform();
}
OdometryBinary::OdometryBinary(
		float inlierDistance,
		int maxWords,
		int minInliers,
		int iterations,
		float maxDepth,
		float linearUpdate,
		float angularUpdate,
		int resetCoutdown,
		int briefBytes,
		int fastThreshold,
		bool fastNonmaxSuppression,
		bool bruteForceMatching) :
	Odometry(inlierDistance, maxWords, minInliers, iterations, maxDepth, linearUpdate, angularUpdate, resetCoutdown),
 	_briefBytes(briefBytes),
	_fastThreshold(fastThreshold),
	_fastNonmaxSuppression(fastNonmaxSuppression),
	_bruteForceMatching(bruteForceMatching)
{

}

OdometryBinary::OdometryBinary(const ParametersMap & parameters) :
	Odometry(parameters),
 	_briefBytes(Parameters::defaultOdomBinBriefBytes()),
	_fastThreshold(Parameters::defaultOdomBinFastThreshold()),
	_fastNonmaxSuppression(Parameters::defaultOdomBinFastNonmaxSuppression()),
	_bruteForceMatching(Parameters::defaultOdomBinBruteForceMatching())
{
	Parameters::parse(parameters, Parameters::kOdomBinBriefBytes(), _briefBytes);
	Parameters::parse(parameters, Parameters::kOdomBinFastThreshold(), _fastThreshold);
	Parameters::parse(parameters, Parameters::kOdomBinFastNonmaxSuppression(), _fastNonmaxSuppression);
	Parameters::parse(parameters, Parameters::kOdomBinBruteForceMatching(), _bruteForceMatching);
}

void OdometryBinary::reset()
{
	Odometry::reset();
	_lastKeypoints.clear();
	_lastDescriptors = cv::Mat();
	_lastDepth = cv::Mat();
}


// return true if odometry is correctly computed
Transform OdometryBinary::computeTransform(Image & image)
{
	UTimer timer;
	cv::Mat imageMono;
	Transform output;
	// convert to grayscale
	if(image.image().channels() > 1)
	{
		cv::cvtColor(image.image(), imageMono, cv::COLOR_BGR2GRAY);
	}
	else
	{
		imageMono = image.image();
	}

	cv::FastFeatureDetector detector(_fastThreshold, _fastNonmaxSuppression);
	std::vector<cv::KeyPoint> newKeypoints;
	detector.detect(imageMono, newKeypoints);

	limitKeypoints(newKeypoints, this->getMaxFeatures());

	cv::BriefDescriptorExtractor extractor(_briefBytes);
	cv::Mat newDescriptors;
	extractor.compute(imageMono, newKeypoints, newDescriptors);

	int inliers = 0;
	int correspondences = 0;

	if(_lastKeypoints.size())
	{
		if(newDescriptors.rows && newDescriptors.rows > (int)_lastKeypoints.size()/2) // at least 50% keypoints
		{
			cv::Mat results;
			cv::Mat dists;
			int k=1; // find the 1 nearest neighbor
			std::vector<std::vector<cv::DMatch> > matches;

			if(_bruteForceMatching)
			{
				cv::BFMatcher matcher(cv::NORM_HAMMING);
				  matcher.knnMatch(newDescriptors, _lastDescriptors, matches, k);
			}
			else
			{
				 // Create Flann LSH index
				cv::flann::Index flannIndex(_lastDescriptors, cv::flann::LshIndexParams(12, 20, 2), cvflann::FLANN_DIST_HAMMING);
				results = cv::Mat(newDescriptors.rows, k, CV_32SC1);
				dists = cv::Mat(newDescriptors.rows, k, CV_32FC1);

				// search (nearest neighbor)
				flannIndex.knnSearch(newDescriptors, results, dists, k, cv::flann::SearchParams() );
			}

			pcl::PointCloud<pcl::PointXYZ>::Ptr mpts_1(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::PointCloud<pcl::PointXYZ>::Ptr mpts_2(new pcl::PointCloud<pcl::PointXYZ>);
			std::vector<int> indexes_1, indexes_2;
			std::vector<uchar> outlier_mask;
			// Check if this descriptor matches with those of the objects
			mpts_1->resize(newDescriptors.rows);
			mpts_2->resize(newDescriptors.rows);
			UDEBUG("newDescriptors=%d _lastKeypoints=%d time=%fs", newDescriptors.rows, _lastKeypoints.size(), timer.elapsed());
			int oi = 0;
			if(_bruteForceMatching)
			{
				for(unsigned int i=0; i<matches.size(); ++i)
				{
					pcl::PointXYZ pt1 = util3d::getDepth(image.depth(),
							int(newKeypoints.at(matches.at(i).at(0).queryIdx).pt.x+0.5f),
							int(newKeypoints.at(matches.at(i).at(0).queryIdx).pt.y+0.5f),
							(float)imageMono.cols/2,
							(float)imageMono.rows/2,
							1.0f/image.depthConstant(),
							1.0f/image.depthConstant());
					if(matches.at(i).at(0).trainIdx >=0)
					{
						pcl::PointXYZ pt2 = util3d::getDepth(_lastDepth,
								int(_lastKeypoints.at(matches.at(i).at(0).trainIdx).pt.x+0.5f),
								int(_lastKeypoints.at(matches.at(i).at(0).trainIdx).pt.y+0.5f),
								(float)imageMono.cols/2,
								(float)imageMono.rows/2,
								1.0f/image.depthConstant(),
								1.0f/image.depthConstant());

						if(uIsFinite(pt1.z) && uIsFinite(pt2.z) &&
						   (this->getMaxDepth() <= 0 || (pt1.z < this->getMaxDepth() && pt2.z < this->getMaxDepth())))
						{
							mpts_1->at(oi) = pt1;
							mpts_2->at(oi) = pt2;
							++oi;
						}
					}
					else
					{
						UWARN("Index = %d for i=%d ?!?", results.at<int>(i,0), i);
					}
				}
			}
			else
			{
				for(int i=0; i<newDescriptors.rows; ++i)
				{
					pcl::PointXYZ pt1 = util3d::getDepth(image.depth(),
							int(newKeypoints.at(i).pt.x+0.5f),
							int(newKeypoints.at(i).pt.y+0.5f),
							(float)imageMono.cols/2,
							(float)imageMono.rows/2,
							1.0f/image.depthConstant(),
							1.0f/image.depthConstant());
					if(results.at<int>(i,0) >=0)
					{
						pcl::PointXYZ pt2 = util3d::getDepth(_lastDepth,
								int(_lastKeypoints.at(results.at<int>(i,0)).pt.x+0.5f),
								int(_lastKeypoints.at(results.at<int>(i,0)).pt.y+0.5f),
								(float)imageMono.cols/2,
								(float)imageMono.rows/2,
								1.0f/image.depthConstant(),
								1.0f/image.depthConstant());

						if(uIsFinite(pt1.z) && uIsFinite(pt2.z) &&
						   (this->getMaxDepth() <= 0 || (pt1.z < this->getMaxDepth() && pt2.z < this->getMaxDepth())))
						{
							mpts_1->at(oi) = pt1;
							mpts_2->at(oi) = pt2;
							++oi;
						}
					}
					else
					{
						UWARN("Index = %d for i=%d ?!?", results.at<int>(i,0), i);
					}
				}
			}
			mpts_1->resize(oi);
			mpts_2->resize(oi);

			UDEBUG("Correspondences = %d", oi);

			if(oi >= this->getMinInliers())
			{
				mpts_1 = util3d::transformPointCloud(mpts_1, image.localTransform()); // new
				mpts_2 = util3d::transformPointCloud(mpts_2, image.localTransform()); // previous
				correspondences = mpts_2->size();
				Transform t = util3d::transformFromXYZCorrespondences(
						mpts_1,
						mpts_2,
						this->getInlierDistance(),
						this->getIterations(),
						&inliers);
				float x,y,z, roll,pitch,yaw;
				pcl::getTranslationAndEulerAngles(util3d::transformToEigen3f(t), x,y,z, roll,pitch,yaw);

				// Large transforms may be erroneous computed transforms, so keep under 1 m
				if(inliers >= this->getMinInliers())
				{
					if(isLargeEnoughTransform(t))
					{
						_lastKeypoints = newKeypoints;
						_lastDescriptors = newDescriptors;
						_lastDepth = image.depth().clone();
						output = t;
					}
					else
					{
						output.setIdentity();
					}
				}
				else
				{
					UWARN("Transform not valid (inliers = %d/%d)", inliers, correspondences);
				}
			}
			else
			{
				UWARN("Not enough inliers %d < %d", oi, this->getMinInliers());
			}
		}
		else if(newDescriptors.rows)
		{
			UWARN("At least 50%% keypoints of the last image required. New=%d last=%d",
					newDescriptors.rows, _lastKeypoints.size());
		}
		else
		{
			UWARN("No feature extracted!");
		}
	}
	else
	{
		_lastKeypoints = newKeypoints;
		_lastDescriptors = newDescriptors;
		_lastDepth = image.depth().clone();
		output.setIdentity();
	}

	UINFO("Odom update time = %fs features=%d inliers=%d/%d",
			timer.elapsed(),
			newDescriptors.rows,
			inliers,
			correspondences);

	return output;
}

//OdometryBOW

OdometryBOW::OdometryBOW(
		int detectorType, // SURF or SIFT
		float inlierDistance,
		int maxWords,
		int minInliers,
		int iterations,
		float maxDepth,
		float linearUpdate,
		float angularUpdate,
		int resetCoutdown,
		float surfHessianThreshold,
		float nndr) : // nearest neighbor distance ratio
	Odometry(inlierDistance, maxWords, minInliers, iterations, maxDepth, linearUpdate, angularUpdate, resetCoutdown),
	_memory(new Memory())
{
	ParametersMap customParameters;
	customParameters.insert(ParametersPair(Parameters::kKpWordsPerImage(), uNumber2Str(maxWords)));
	customParameters.insert(ParametersPair(Parameters::kKpMaxDepth(), uNumber2Str(maxDepth)));
	customParameters.insert(ParametersPair(Parameters::kKpDetectorStrategy(), uNumber2Str(detectorType)));
	customParameters.insert(ParametersPair(Parameters::kSURFHessianThreshold(), uNumber2Str(surfHessianThreshold)));
	customParameters.insert(ParametersPair(Parameters::kKpNndrRatio(), uNumber2Str(nndr)));
	customParameters.insert(ParametersPair(Parameters::kMemRehearsalSimilarity(), "1.0")); // desactivate rehearsal
	customParameters.insert(ParametersPair(Parameters::kMemImageKept(), "false"));
	if(!_memory->init("", false, customParameters, false))
	{
		UERROR("Error initializing the memory for BOW Odometry.");
	}
}

OdometryBOW::OdometryBOW(const ParametersMap & parameters) :
	Odometry(parameters),
	_memory(new Memory(parameters))
{
	ParametersMap customParameters;
	customParameters.insert(ParametersPair(Parameters::kKpWordsPerImage(), uNumber2Str(this->getMaxFeatures()))); // hack
	customParameters.insert(ParametersPair(Parameters::kKpMaxDepth(), uNumber2Str(this->getMaxDepth())));
	customParameters.insert(ParametersPair(Parameters::kMemRehearsalSimilarity(), "1.0")); // desactivate rehearsal
	customParameters.insert(ParametersPair(Parameters::kMemImageKept(), "false"));
	if(!_memory->init("", false, customParameters, false))
	{
		UERROR("Error initializing the memory for BOW Odometry.");
	}
}

OdometryBOW::~OdometryBOW()
{
	UDEBUG("");
	delete _memory;
	UDEBUG("");
}


void OdometryBOW::reset()
{
	Odometry::reset();
	_memory->init("", false, ParametersMap(), false);
}


// return true if odometry is correctly computed
Transform OdometryBOW::computeTransform(Image & image)
{
	UTimer timer;
	Transform output;

	std::vector<cv::KeyPoint> keypoints;
	cv::Mat descriptors;
	_memory->extractKeypointsAndDescriptors(image.image(), image.depth(), image.depthConstant(), keypoints, descriptors);

	image.setDescriptors(descriptors);
	image.setKeypoints(keypoints);

	int inliers = 0;
	int correspondences = 0;

	const Signature * previousSignature = _memory->getLastWorkingSignature();
	if(_memory->update(image))
	{
		const Signature * newSignature = _memory->getLastWorkingSignature();
		if(previousSignature && newSignature)
		{
			Transform transform;
			if(newSignature->getWords3().size() < previousSignature->getWords3().size()/2)
			{
				UWARN("At least 50%% keypoints of the last image required. New=%d last=%d",
						newSignature->getWords3().size(), previousSignature->getWords3().size());
			}
			else if(!previousSignature->getWords3().empty() && !newSignature->getWords3().empty())
			{
				pcl::PointCloud<pcl::PointXYZ>::Ptr inliers1(new pcl::PointCloud<pcl::PointXYZ>); // previous
				pcl::PointCloud<pcl::PointXYZ>::Ptr inliers2(new pcl::PointCloud<pcl::PointXYZ>); // new

				util3d::findCorrespondences(
						previousSignature->getWords3(),
						newSignature->getWords3(),
						*inliers1,
						*inliers2,
						this->getMaxDepth());

				if((int)inliers1->size() >= this->getMinInliers())
				{
					correspondences = inliers1->size();

					transform = util3d::transformFromXYZCorrespondences(
							inliers2,
							inliers1,
							this->getInlierDistance(),
							this->getIterations(),
							&inliers);

					if(inliers < this->getMinInliers())
					{
						transform.setNull();
						UWARN("Transform not valid (inliers = %d/%d)", inliers, correspondences);
					}
				}
				else
				{
					UWARN("Not enough inliers %d < %d", (int)inliers1->size(), this->getMinInliers());
				}
			}

			if(transform.isNull())
			{
				_memory->deleteLocation(newSignature->id());
			}
			else if(!isLargeEnoughTransform(transform))
			{
				output.setIdentity();
				_memory->deleteLocation(newSignature->id());
			}
			else
			{
				output = transform;
				_memory->deleteLocation(previousSignature->id());
			}
		}
		else if(!previousSignature && newSignature)
		{
			output.setIdentity();
		}

		_memory->emptyTrash();
	}

	UINFO("Odom update time = %fs features=%d inliers=%d/%d",
			timer.elapsed(),
			descriptors.rows,
			inliers,
			correspondences);
	return output;
}

// OdometryICP
OdometryICP::OdometryICP(
		int decimation,
		float voxelSize,
		float samples,
		float maxCorrespondenceDistance,
		int	maxIterations,
		float maxFitness,
		float maxDepth,
		float linearUpdate,
		float angularUpdate,
		int resetCoutdown) :
	Odometry(0, 0, 0, 0, maxDepth, linearUpdate, angularUpdate, resetCoutdown),
	_decimation(decimation),
	_voxelSize(voxelSize),
	_samples(samples),
	_maxCorrespondenceDistance(maxCorrespondenceDistance),
	_maxIterations(maxIterations),
	_maxFitness(maxFitness),
	_previousCloud(new pcl::PointCloud<pcl::PointNormal>)
{

}

OdometryICP::OdometryICP(const ParametersMap & parameters) :
	Odometry(parameters),
	_decimation(Parameters::defaultOdomICPDecimation()),
	_voxelSize(Parameters::defaultOdomICPVoxelSize()),
	_samples(Parameters::defaultOdomICPSamples()),
	_maxCorrespondenceDistance(Parameters::defaultOdomICPCorrespondencesDistance()),
	_maxIterations(Parameters::defaultOdomICPIterations()),
	_maxFitness(Parameters::defaultOdomICPMaxFitness()),
	_previousCloud(new pcl::PointCloud<pcl::PointNormal>)
{
	Parameters::parse(parameters, Parameters::kOdomICPDecimation(), _decimation);
	Parameters::parse(parameters, Parameters::kOdomICPVoxelSize(), _voxelSize);
	Parameters::parse(parameters, Parameters::kOdomICPSamples(), _samples);
	Parameters::parse(parameters, Parameters::kOdomICPCorrespondencesDistance(), _maxCorrespondenceDistance);
	Parameters::parse(parameters, Parameters::kOdomICPIterations(), _maxIterations);
	Parameters::parse(parameters, Parameters::kOdomICPMaxFitness(), _maxFitness);
}

void OdometryICP::reset()
{
	Odometry::reset();
	_previousCloud.reset(new pcl::PointCloud<pcl::PointNormal>);
}

// return not null if odometry is correctly computed
Transform OdometryICP::computeTransform(Image & image)
{
	UTimer timer;
	Transform output;

	bool hasConverged = false;
	double fitness = 0;
	unsigned int minPoints = 100;
	if(!image.depth().empty())
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr newCloudXYZ = util3d::getICPReadyCloud(
						image.depth(),
						image.depthConstant(),
						_decimation,
						this->getMaxDepth(),
						_voxelSize,
						_samples,
						image.localTransform());

		pcl::PointCloud<pcl::PointNormal>::Ptr newCloud = util3d::computeNormals(newCloudXYZ);

		std::vector<int> indices;
		newCloud = util3d::removeNaNNormalsFromPointCloud(newCloud);
		if(newCloudXYZ->size() != newCloud->size())
		{
			UWARN("removed nan normals...");
		}

		if(_previousCloud->size() > minPoints && newCloud->size() > minPoints)
		{
			Transform transform = util3d::icpPointToPlane(newCloud,
					_previousCloud,
					_maxCorrespondenceDistance,
					_maxIterations,
					hasConverged,
				    fitness);

			//pcl::io::savePCDFile("old.pcd", *_previousCloud);
			//pcl::io::savePCDFile("new.pcd", *newCloud);
			//pcl::PointCloud<pcl::PointXYZ>::Ptr newCloudTransformed = util3d::transformPointCloud(newCloud, transform);
			//pcl::io::savePCDFile("newicp.pcd", *newCloudTransformed);

			if(hasConverged && (_maxFitness == 0 || fitness < _maxFitness))
			{
				output = transform;
				_previousCloud = newCloud;
			}
			else
			{
				UWARN("Transform not valid (hasConverged=%s fitness = %f < %f)",
						hasConverged?"true":"false", fitness, _maxFitness);
			}
		}
		else if(newCloud->size() > minPoints)
		{
			output.setIdentity();
			_previousCloud = newCloud;
		}
	}
	else
	{
		UERROR("Depth is empty?!?");
	}

	UINFO("Odom update time = %fs hasConverged=%s fitness=%f cloud=%d",
			timer.elapsed(),
			hasConverged?"true":"false",
			fitness,
			(int)_previousCloud->size());

	return output;
}

// OdometryThread
OdometryThread::OdometryThread(Odometry * odometry) :
	_odometry(odometry),
	_resetOdometry(false)
{
	UASSERT(_odometry != 0);
}

OdometryThread::~OdometryThread()
{
	this->unregisterFromEventsManager();
	this->join(true);
	if(_odometry)
	{
		delete _odometry;
	}
}

void OdometryThread::handleEvent(UEvent * event)
{
	if(this->isRunning())
	{
		if(event->getClassName().compare("CameraEvent") == 0)
		{
			CameraEvent * cameraEvent = (CameraEvent*)event;
			if(cameraEvent->getCode() == CameraEvent::kCodeImageDepth)
			{
				this->addImage(cameraEvent->image());
			}
			else if(cameraEvent->getCode() == CameraEvent::kCodeNoMoreImages)
			{
				this->post(new CameraEvent()); // forward the event
			}
		}
		else if(event->getClassName().compare("OdometryResetEvent") == 0)
		{
			_resetOdometry = true;
		}
	}
}

void OdometryThread::mainLoopKill()
{
	_imageAdded.release();
}

//============================================================
// MAIN LOOP
//============================================================
void OdometryThread::mainLoop()
{
	if(_resetOdometry)
	{
		_odometry->reset();
		_resetOdometry = false;
	}

	Image image;
	getImage(image);
	if(!image.empty())
	{
		Transform pose = _odometry->process(image);
		image.setPose(pose); // a null pose notify that odometry could not be computed
		this->post(new OdometryEvent(image));
	}
}

void OdometryThread::addImage(const Image & image)
{
	if(image.empty() || image.depth().empty() || image.depthConstant() == 0.0f)
	{
		ULOGGER_ERROR("image empty !?");
		return;
	}

	bool notify = true;
	_imageMutex.lock();
	{
		notify = _imageBuffer.empty();
		_imageBuffer = image;
	}
	_imageMutex.unlock();

	if(notify)
	{
		_imageAdded.release();
	}
}

void OdometryThread::getImage(Image & image)
{
	_imageAdded.acquire();
	_imageMutex.lock();
	{
		if(!_imageBuffer.empty())
		{
			image = _imageBuffer;
			_imageBuffer = Image();
		}
	}
	_imageMutex.unlock();
}

} /* namespace rtabmap */
