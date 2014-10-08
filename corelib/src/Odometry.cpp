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
#include <rtabmap/core/VWDictionary.h>
#include "rtabmap/core/Signature.h"

#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>

#include <opencv2/gpu/gpu.hpp>

#if _MSC_VER
	#define ISFINITE(value) _finite(value)
#else
	#define ISFINITE(value) std::isfinite(value)
#endif

namespace rtabmap {

Odometry::Odometry(const rtabmap::ParametersMap & parameters) :
		_maxFeatures(Parameters::defaultOdomMaxWords()),
		_minInliers(Parameters::defaultOdomMinInliers()),
		_inlierDistance(Parameters::defaultOdomInlierDistance()),
		_iterations(Parameters::defaultOdomIterations()),
		_wordsRatio(Parameters::defaultOdomWordsRatio()),
		_maxDepth(Parameters::defaultOdomMaxDepth()),
		_linearUpdate(Parameters::defaultOdomLinearUpdate()),
		_angularUpdate(Parameters::defaultOdomAngularUpdate()),
		_resetCountdown(Parameters::defaultOdomResetCountdown()),
		_localHistoryMaxSize(Parameters::defaultOdomLocalHistory()),
		_pose(Transform::getIdentity()),
		_resetCurrentCount(0)
{
	Parameters::parse(parameters, Parameters::kOdomLinearUpdate(), _linearUpdate);
	Parameters::parse(parameters, Parameters::kOdomAngularUpdate(), _angularUpdate);
	Parameters::parse(parameters, Parameters::kOdomResetCountdown(), _resetCountdown);
	Parameters::parse(parameters, Parameters::kOdomMinInliers(), _minInliers);
	Parameters::parse(parameters, Parameters::kOdomInlierDistance(), _inlierDistance);
	Parameters::parse(parameters, Parameters::kOdomIterations(), _iterations);
	Parameters::parse(parameters, Parameters::kOdomWordsRatio(), _wordsRatio);
	Parameters::parse(parameters, Parameters::kOdomMaxDepth(), _maxDepth);
	Parameters::parse(parameters, Parameters::kOdomMaxWords(), _maxFeatures);
	Parameters::parse(parameters, Parameters::kOdomLocalHistory(), _localHistoryMaxSize);
}

void Odometry::reset()
{
	_resetCurrentCount = 0;
	_pose = Transform::getIdentity();
}

bool Odometry::isLargeEnoughTransform(const Transform & transform)
{
	float x,y,z, roll,pitch,yaw;
	transform.getTranslationAndEulerAngles(x,y,z, roll,pitch,yaw);
	return (_linearUpdate == 0.0f && _angularUpdate == 0.0f) ||
			fabs(x) > _linearUpdate ||
		   fabs(y) > _linearUpdate ||
	       fabs(z) > _linearUpdate ||
	       fabs(roll) > _angularUpdate ||
			fabs(pitch) > _angularUpdate ||
			fabs(yaw) > _angularUpdate;
}

Transform Odometry::process(SensorData & data, int * quality, int * features, int * localMapSize)
{
	Transform t = this->computeTransform(data, quality, features, localMapSize);
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

//OdometryBOW
OdometryBOW::OdometryBOW(const ParametersMap & parameters) :
	Odometry(parameters),
	_memory(0)
{
	ParametersMap customParameters;
	customParameters.insert(ParametersPair(Parameters::kKpWordsPerImage(), uNumber2Str(this->getMaxFeatures()))); // hack
	customParameters.insert(ParametersPair(Parameters::kKpMaxDepth(), uNumber2Str(this->getMaxDepth())));
	customParameters.insert(ParametersPair(Parameters::kMemRehearsalSimilarity(), "1.0")); // desactivate rehearsal
	customParameters.insert(ParametersPair(Parameters::kMemImageKept(), "false"));
	customParameters.insert(ParametersPair(Parameters::kMemSTMSize(), "0"));
	int nn = Parameters::defaultOdomNearestNeighbor();
	float nndr = Parameters::defaultOdomNNDR();
	int odomType = Parameters::defaultOdomType();
	Parameters::parse(parameters, Parameters::kOdomNearestNeighbor(), nn);
	Parameters::parse(parameters, Parameters::kOdomNNDR(), nndr);
	Parameters::parse(parameters, Parameters::kOdomType(), odomType);
	customParameters.insert(ParametersPair(Parameters::kKpNNStrategy(), uNumber2Str(nn)));
	customParameters.insert(ParametersPair(Parameters::kKpNndrRatio(), uNumber2Str(nndr)));
	customParameters.insert(ParametersPair(Parameters::kKpDetectorStrategy(), uNumber2Str(odomType)));

	// add only feature stuff
	for(ParametersMap::const_iterator iter=parameters.begin(); iter!=parameters.end(); ++iter)
	{
		std::string group = uSplit(iter->first, '/').front();
		if(group.compare("SURF") == 0 ||
			group.compare("SIFT") == 0 ||
			group.compare("BRIEF") == 0 ||
			group.compare("FAST") == 0 ||
			group.compare("ORB") == 0 ||
			group.compare("FREAK") == 0 ||
			group.compare("GFTT") == 0 ||
			group.compare("BRISK") == 0)
		{
			customParameters.insert(*iter);
		}
	}

	_memory = new Memory(customParameters);
	if(!_memory->init("", false, ParametersMap(), false))
	{
		UERROR("Error initializing the memory for BOW Odometry.");
	}
}

OdometryBOW::~OdometryBOW()
{
	delete _memory;
}


void OdometryBOW::reset()
{
	Odometry::reset();
	_memory->init("", false, ParametersMap(), false);
	localMap_.clear();
}

// return not null transform if odometry is correctly computed
Transform OdometryBOW::computeTransform(const SensorData & data, int * quality, int * features, int * localMapSize)
{
	UTimer timer;
	Transform output;

	int inliers = 0;
	int correspondences = 0;
	int nFeatures = 0;

	const Signature * previousSignature = _memory->getLastWorkingSignature();
	if(_memory->update(data))
	{
		const Signature * newSignature = _memory->getLastWorkingSignature();
		if(newSignature)
		{
			nFeatures = newSignature->getWords().size();
		}

		if(previousSignature && newSignature)
		{
			Transform transform;
			std::set<int> uniqueCorrespondences;
			if(newSignature->getWords3().size() < (unsigned int)(getWordsRatio() * float(previousSignature->getWords3().size())))
			{
				UWARN("At least %f%% keypoints of the last image required. New=%d last=%d",
						getWordsRatio()*100.0f, newSignature->getWords3().size(), previousSignature->getWords3().size());
			}
			else if(!localMap_.empty() && !newSignature->getWords3().empty())
			{
				pcl::PointCloud<pcl::PointXYZ>::Ptr inliers1(new pcl::PointCloud<pcl::PointXYZ>); // previous
				pcl::PointCloud<pcl::PointXYZ>::Ptr inliers2(new pcl::PointCloud<pcl::PointXYZ>); // new

				// No need to set max depth here, it is already applied in extractKeypointsAndDescriptors() above.
				// Also! the localMap_ have points not in camera frame anymore (in local map frame), so filtering
				// by depth here is wrong!
				util3d::findCorrespondences(
						localMap_,
						newSignature->getWords3(),
						*inliers1,
						*inliers2,
						0,
						&uniqueCorrespondences);

				UDEBUG("localMap=%d, new=%d, unique correspondences=%d", (int)localMap_.size(), (int)newSignature->getWords3().size(), (int)uniqueCorrespondences.size());

				if((int)inliers1->size() >= this->getMinInliers())
				{
					correspondences = inliers1->size();

					// the transform returned is global odometry pose, not incremental one
					transform = util3d::transformFromXYZCorrespondences(
							inliers2,
							inliers1,
							this->getInlierDistance(),
							this->getIterations(),
							&inliers);

					if(!transform.isNull())
					{
						// make it incremental
						transform = this->getPose().inverse() * transform;

						UDEBUG("Odom transform = %s", transform.prettyPrint().c_str());
					}
					else
					{
						UDEBUG("Odom transform null");
						//pcl::io::savePCDFile("from.pcd", *inliers1);
						//inliers2 = util3d::transformPointCloud(inliers2, this->getPose());
						//pcl::io::savePCDFile("to.pcd", *inliers2);
						//inliers2 = util3d::transformPointCloud(inliers2, transform);
						//pcl::io::savePCDFile("to_t.pcd", *inliers2);
					}
					/*pcl::io::savePCDFile("from.pcd", *inliers1);
					inliers2 = util3d::transformPointCloud(inliers2, this->getPose());
					pcl::io::savePCDFile("to.pcd", *inliers2);
					inliers2 = util3d::transformPointCloud(inliers2, transform);
					pcl::io::savePCDFile("to_t.pcd", *inliers2);*/

					/*
					//refine ICP test
					bool hasConverged;
					double fitness;
					inliers2 = util3d::transformPointCloud(inliers2, transform);
					Transform icpT = util3d::icp(inliers1, <-- must be all correspondences, not only unique
						inliers2,
						0.02,
						100,
						hasConverged,
						fitness);

					transform = transform * icpT;
					*/

					if(quality)
					{
						*quality = inliers;
					}

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
			else
			{
				output = transform;
				if(!isLargeEnoughTransform(transform))
				{
					// Transform not large enough, keep the old signature
					_memory->deleteLocation(newSignature->id());
				}
				else
				{
					// remove words if history max size is reached
					while(localMap_.size() && (int)localMap_.size() > this->getLocalHistoryMaxSize() && _memory->getStMem().size()>1)
					{
						int nodeId = *_memory->getStMem().begin();
						std::list<int> removedPts;
						_memory->deleteLocation(nodeId, &removedPts);
						for(std::list<int>::iterator iter = removedPts.begin(); iter!=removedPts.end(); ++iter)
						{
							localMap_.erase(*iter);
						}
					}

					if(this->getLocalHistoryMaxSize() == 0 && localMap_.size() > 0 && localMap_.size() > newSignature->getWords3().size())
					{
						UERROR("Local map should have only words of the last added signature here! (size=%d, max history size=%d, newWords=%d)",
								(int)localMap_.size(), this->getLocalHistoryMaxSize(), (int)newSignature->getWords3().size());
					}

					// update local map
					std::list<int> uniques = uUniqueKeys(newSignature->getWords3());
					Transform t = this->getPose()*output;
					for(std::list<int>::iterator iter = uniques.begin(); iter!=uniques.end(); ++iter)
					{
						// Only add unique words not in local map
						if(newSignature->getWords3().count(*iter) == 1)
						{
							// keep old word
							if(localMap_.find(*iter) == localMap_.end())
							{
								const pcl::PointXYZ & pt = newSignature->getWords3().find(*iter)->second;
								if(pcl::isFinite(pt))
								{
									pcl::PointXYZ pt2 = util3d::transformPoint(pt, t);
									localMap_.insert(std::make_pair(*iter, pt2));
								}
							}
						}
						else
						{
							localMap_.erase(*iter);
						}
					}
				}
			}
		}
		else if(!previousSignature && newSignature)
		{
			localMap_.clear();
			output.setIdentity();

			int count = 0;
			std::list<int> uniques = uUniqueKeys(newSignature->getWords3());
			for(std::list<int>::iterator iter = uniques.begin(); iter!=uniques.end(); ++iter)
			{
				// Only add unique words
				if(newSignature->getWords3().count(*iter) == 1)
				{
					const pcl::PointXYZ & pt = newSignature->getWords3().find(*iter)->second;
					if(pcl::isFinite(pt))
					{
						localMap_.insert(std::make_pair(*iter, pt));
					}
					else
					{
						++count;
					}
				}
			}
			UDEBUG("uniques=%d, pt not finite = %d", (int)uniques.size(),count);
		}

		_memory->emptyTrash();
	}

	if(features)
	{
		*features = nFeatures;
	}
	if(localMapSize)
	{
		*localMapSize = (int)localMap_.size();
	}

	UINFO("Odom update time = %fs features=%d inliers=%d/%d local_map=%d[%d] dict=%d nodes=%d",
			timer.elapsed(),
			nFeatures,
			inliers,
			correspondences,
			(int)uUniqueKeys(localMap_).size(),
			(int)localMap_.size(),
			(int)_memory->getVWDictionary()->getVisualWords().size(),
			(int)_memory->getStMem().size());
	return output;
}

// OdometryICP
OdometryICP::OdometryICP(int decimation,
		float voxelSize,
		int samples,
		float maxCorrespondenceDistance,
		int maxIterations,
		float maxFitness,
		bool pointToPlane,
		const ParametersMap & odometryParameter) :
	Odometry(odometryParameter),
	_decimation(decimation),
	_voxelSize(voxelSize),
	_samples(samples),
	_maxCorrespondenceDistance(maxCorrespondenceDistance),
	_maxIterations(maxIterations),
	_maxFitness(maxFitness),
	_pointToPlane(pointToPlane),
	_previousCloudNormal(new pcl::PointCloud<pcl::PointNormal>),
	_previousCloud(new pcl::PointCloud<pcl::PointXYZ>)
{
}

void OdometryICP::reset()
{
	Odometry::reset();
	_previousCloudNormal.reset(new pcl::PointCloud<pcl::PointNormal>);
	_previousCloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
}

// return not null transform if odometry is correctly computed
Transform OdometryICP::computeTransform(const SensorData & data, int * quality, int * features, int * localMapSize)
{
	UTimer timer;
	Transform output;

	bool hasConverged = false;
	double fitness = 0;
	unsigned int minPoints = 100;
	if(!data.depth().empty())
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr newCloudXYZ = util3d::getICPReadyCloud(
						data.depth(),
						data.depthFx(),
						data.depthFy(),
						data.depthCx(),
						data.depthCy(),
						_decimation,
						this->getMaxDepth(),
						_voxelSize,
						_samples,
						data.localTransform());

		if(_pointToPlane)
		{
			pcl::PointCloud<pcl::PointNormal>::Ptr newCloud = util3d::computeNormals(newCloudXYZ);

			std::vector<int> indices;
			newCloud = util3d::removeNaNNormalsFromPointCloud(newCloud);
			if(newCloudXYZ->size() != newCloud->size())
			{
				UWARN("removed nan normals...");
			}

			if(_previousCloudNormal->size() > minPoints && newCloud->size() > minPoints)
			{
				Transform transform = util3d::icpPointToPlane(newCloud,
						_previousCloudNormal,
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
					_previousCloudNormal = newCloud;
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
				_previousCloudNormal = newCloud;
			}
		}
		else
		{
			//point to point
			if(_previousCloud->size() > minPoints && newCloudXYZ->size() > minPoints)
			{
				Transform transform = util3d::icp(newCloudXYZ,
						_previousCloud,
						_maxCorrespondenceDistance,
						_maxIterations,
						hasConverged,
						fitness);

				//pcl::io::savePCDFile("old.pcd", *_previousCloudNormal);
				//pcl::io::savePCDFile("new.pcd", *newCloud);
				//pcl::PointCloud<pcl::PointXYZ>::Ptr newCloudTransformed = util3d::transformPointCloud(newCloud, transform);
				//pcl::io::savePCDFile("newicp.pcd", *newCloudTransformed);

				if(hasConverged && (_maxFitness == 0 || fitness < _maxFitness))
				{
					output = transform;
					_previousCloud = newCloudXYZ;
				}
				else
				{
					UWARN("Transform not valid (hasConverged=%s fitness = %f < %f)",
							hasConverged?"true":"false", fitness, _maxFitness);
				}
			}
			else if(newCloudXYZ->size() > minPoints)
			{
				output.setIdentity();
				_previousCloud = newCloudXYZ;
			}
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
			(int)(_pointToPlane?_previousCloudNormal->size():_previousCloud->size()));

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
				this->addData(cameraEvent->data());
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
	_dataAdded.release();
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

	SensorData data;
	getData(data);
	if(data.isValid())
	{
		int quality = -1;
		int features = -1;
		int localMapSize = -1;
		UTimer time;
		Transform pose = _odometry->process(data, &quality, &features, &localMapSize);
		data.setPose(pose); // a null pose notify that odometry could not be computed
		this->post(new OdometryEvent(data, quality, time.elapsed(), features, localMapSize));
	}
}

void OdometryThread::addData(const SensorData & data)
{
	if(data.image().empty() || data.depth().empty() || data.depthFx() == 0.0f || data.depthFy() == 0.0f)
	{
		ULOGGER_ERROR("image empty !?");
		return;
	}

	bool notify = true;
	_dataMutex.lock();
	{
		notify = !_dataBuffer.isValid();
		_dataBuffer = data;
	}
	_dataMutex.unlock();

	if(notify)
	{
		_dataAdded.release();
	}
}

void OdometryThread::getData(SensorData & data)
{
	_dataAdded.acquire();
	_dataMutex.lock();
	{
		if(_dataBuffer.isValid())
		{
			data = _dataBuffer;
			_dataBuffer = SensorData();
		}
	}
	_dataMutex.unlock();
}

} /* namespace rtabmap */
