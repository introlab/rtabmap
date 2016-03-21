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

#include <tango-gl/conversions.h>

#include "RTABMapApp.h"

#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/core/util3d_transforms.h>
#include <rtabmap/core/util3d_filtering.h>
#include <rtabmap/core/Graph.h>
#include <rtabmap/utilite/UEventsManager.h>
#include <rtabmap/utilite/UStl.h>
#include <opencv2/opencv_modules.hpp>
#include <rtabmap/core/util3d_surface.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/core/ParamEvent.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/ply_io.h>

const int kVersionStringLength = 128;
const int cameraTangoDecimation = 2;
const int renderingCloudDecimation = 8;
const float renderingCloudMaxDepth = 4.0f;
const int maxFeatures = 400;
const float meshAngleTolerance = 0.1745; // 10 degrees
const int meshTrianglePixels = 1;
const bool substractFiltering = false;
const float subtractRadius = 0.02;
const float subtractMaxAngle = M_PI/4.0f;
const bool textureMeshing = true;
const int minNeighborsInRadius = 5;
const float closeVerticesDistance = 0.02f;

static JavaVM *jvm;
static jobject RTABMapActivity = 0;

rtabmap::ParametersMap RTABMapApp::getRtabmapParameters()
{
	rtabmap::ParametersMap parameters;

	parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kRtabmapLoopThr(), "0.11"));
	parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kRGBDLoopClosureReextractFeatures(), std::string("false")));
	parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kKpDetectorStrategy(), std::string("6"))); // GFTT/BRIEF
	parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kGFTTQualityLevel(), std::string("0.0001")));
	parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kGFTTMinDistance(), std::string("10")));
	parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kFASTThreshold(), std::string("1")));
	parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kBRIEFBytes(), std::string("64")));
	parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kMemImageKept(), "false"));
	parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kMemBinDataKept(), uBool2Str(!trajectoryMode_)));
	parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kMemRawDescriptorsKept(), "true")); // for visual registration
	parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kMemNotLinkedNodesKept(), std::string("false")));
	parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kKpNNStrategy(), std::string("1"))); // Kd-tree
	parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kKpMaxFeatures(), std::string("200")));
	parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kKpNndrRatio(), std::string("0.8"))); // set the one for kd-tree
	parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kKpMaxFeatures(), !loopClosureDetection_?std::string("-1"):uNumber2Str(maxFeatures))); // Kd-tree
	parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kOptimizerIterations(), uNumber2Str(graphOptimization_?rtabmap::Parameters::defaultOptimizerIterations():0)));
	parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kMemIncrementalMemory(), uBool2Str(!localizationMode_)));
	parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kRtabmapMaxRetrieved(), uBool2Str(!localizationMode_)));
	parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kKpMaxDepth(), std::string("10"))); // to avoid extracting features in invalid depth (as we compute transformation directly from the words)
	//parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kMemImageDecimation(), std::string("2")));
	parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kRGBDOptimizeFromGraphEnd(), std::string("true")));
	parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kRGBDOptimizeMaxError(), std::string("1")));
	parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kOptimizerRobust(), std::string("false")));
	parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kOptimizerVarianceIgnored(), std::string("false")));
	parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kRtabmapTimeThr(), std::string("700")));
	parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kDbSqlite3InMemory(), std::string("true")));
	parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kVisMinInliers(), std::string("15")));
	parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kVisRefineIterations(), std::string("5")));

	return parameters;
}

RTABMapApp::RTABMapApp() :
		camera_(0),
		rtabmapThread_(0),
		logHandler_(0),
		mapCloudShown_(true),
		odomCloudShown_(true),
		loopClosureDetection_(true),
		graphOptimization_(true),
		localizationMode_(false),
		trajectoryMode_(false),
		autoExposure_(false),
		clearSceneOnNextRender_(false),
		totalPoints_(0),
		totalPolygons_(0),
		lastDrawnCloudsCount_(0)
{
}

RTABMapApp::~RTABMapApp() {
  if(camera_)
  {
	  delete camera_;
  }
  if(rtabmapThread_)
  {
	  rtabmapThread_->close(false);
	  delete rtabmapThread_;
  }
  if(logHandler_)
  {
	  delete logHandler_;
  }
}

int RTABMapApp::TangoInitialize(JNIEnv* env, jobject caller_activity)
{

	env->GetJavaVM(&jvm);
	RTABMapActivity = env->NewGlobalRef(caller_activity);

	LOGI("RTABMapApp::TangoInitialize()");
	createdMeshes_.clear();
	previousCloud_.first = 0;
	previousCloud_.second.first.reset();
	previousCloud_.second.second.reset();
	rawPoses_.clear();
	clearSceneOnNextRender_ = true;
	totalPoints_ = 0;
	totalPolygons_ = 0;
	lastDrawnCloudsCount_ = 0;

	if(camera_)
	{
	  delete camera_;
	}
	if(rtabmapThread_)
	{
		rtabmapThread_->close(false);
	    delete rtabmapThread_;
	    rtabmapThread_ = 0;
	}

	if(logHandler_ == 0)
	{
		logHandler_ = new LogHandler();
	}
	ULogger::setEventLevel(ULogger::kInfo);
	ULogger::setPrintThreadId(true);

	this->registerToEventsManager();

	camera_ = new rtabmap::CameraTango(cameraTangoDecimation, autoExposure_);


  // The first thing we need to do for any Tango enabled application is to
  // initialize the service. We'll do that here, passing on the JNI environment
  // and jobject corresponding to the Android activity that is calling us.
  return TangoService_initialize(env, caller_activity);
}

void RTABMapApp::openDatabase(const std::string & databasePath)
{
	this->unregisterFromEventsManager(); // to ignore published init events when closing rtabmap
	status_.first = rtabmap::RtabmapEventInit::kInitializing;
	rtabmapMutex_.lock();
	if(rtabmapThread_)
	{
		rtabmapThread_->close(false);
		delete rtabmapThread_;
		rtabmapThread_ = 0;
	}

	//Rtabmap
	rtabmap::Rtabmap * rtabmap = new rtabmap::Rtabmap();
	rtabmap::ParametersMap parameters = getRtabmapParameters();

	rtabmap->init(parameters, databasePath);
	rtabmapThread_ = new rtabmap::RtabmapThread(rtabmap);

	// Generate all meshes
	std::map<int, rtabmap::Signature> signatures;
	std::map<int, rtabmap::Transform> poses;
	std::multimap<int, rtabmap::Link> links;
	rtabmap->get3DMap(
			signatures,
			poses,
			links,
			true,
			true);

	clearSceneOnNextRender_ = true;
	rtabmap::Statistics stats;
	stats.setSignatures(signatures);
	stats.setPoses(poses);
	stats.setConstraints(links);
	rtabmapEvents_.push_back(stats);

	// Start threads
	LOGI("Start rtabmap thread");
	this->registerToEventsManager();
	rtabmapThread_->registerToEventsManager();
	rtabmapThread_->start();

	status_.first = rtabmap::RtabmapEventInit::kInitialized;
	status_.second = "";
	rtabmapMutex_.unlock();
}

int RTABMapApp::onResume()
{
	LOGW("onResume()");
	if(camera_)
	{
		camera_->join(true);

		if(camera_->init())
		{
			LOGI("Start camera thread");
			camera_->start();
			return TANGO_SUCCESS;
		}
		LOGE("Failed camera initialization!");
	}
	return TANGO_ERROR;
}

void RTABMapApp::onPause()
{
	LOGW("onPause()");
	if(camera_)
	{
		camera_->join(true);
		camera_->close();
	}
}


void RTABMapApp::TangoResetMotionTracking() {
  TangoService_resetMotionTracking();
}

// OpenGL thread
void RTABMapApp::InitializeGLContent()
{
	UINFO("");
	main_scene_.InitGLContent();
}

// OpenGL thread
void RTABMapApp::SetViewPort(int width, int height)
{
	UINFO("");
	main_scene_.SetupViewPort(width, height);
}

// OpenGL thread
int RTABMapApp::Render()
{
	// should be before clearSceneOnNextRender_ in case openDatabase is called
	std::list<rtabmap::Statistics> rtabmapEvents;
	{
		boost::mutex::scoped_lock  lock(rtabmapMutex_);
		rtabmapEvents = rtabmapEvents_;
		rtabmapEvents_.clear();
	}

	if(clearSceneOnNextRender_)
	{
		odomMutex_.lock();
		odomEvents_.clear();
		odomMutex_.unlock();

		poseMutex_.lock();
		poseEvents_.clear();
		poseMutex_.unlock();

		main_scene_.clear();
		clearSceneOnNextRender_ = false;
		createdMeshes_.clear();
		previousCloud_.first = 0;
		previousCloud_.second.first.reset();
		previousCloud_.second.second.reset();
		rawPoses_.clear();
		totalPoints_ = 0;
		totalPolygons_ = 0;
		lastDrawnCloudsCount_ = 0;
	}

	// Process events
	rtabmap::Transform pose;
	{
		boost::mutex::scoped_lock lock(poseMutex_);
		if(poseEvents_.size())
		{
			pose = poseEvents_.back();
			poseEvents_.clear();
		}
	}
	if(!pose.isNull())
	{
		// update camera pose?
		main_scene_.SetCameraPose(pose);
	}

	bool notifyDataLoaded = false;
	if(rtabmapEvents.size())
	{
		LOGI("Process rtabmap events");

		// update buffered signatures
		std::map<int, rtabmap::SensorData> bufferedSensorData;
		if(!trajectoryMode_)
		{
			for(std::list<rtabmap::Statistics>::iterator iter=rtabmapEvents.begin(); iter!=rtabmapEvents.end(); ++iter)
			{
				for(std::map<int, rtabmap::Signature>::const_iterator jter=iter->getSignatures().begin(); jter!=iter->getSignatures().end(); ++jter)
				{
					if(!jter->second.sensorData().imageRaw().empty() &&
					   !jter->second.sensorData().depthRaw().empty())
					{
						uInsert(bufferedSensorData, std::make_pair(jter->first, jter->second.sensorData()));
						uInsert(rawPoses_, std::make_pair(jter->first, jter->second.getPose()));
					}
					else if(!jter->second.sensorData().imageCompressed().empty() &&
							!jter->second.sensorData().depthOrRightCompressed().empty())
					{
						// uncompress
						rtabmap::SensorData data = jter->second.sensorData();
						cv::Mat tmpA,tmpB;
						data.uncompressData(&tmpA, &tmpB);
						uInsert(bufferedSensorData, std::make_pair(jter->first, data));
						uInsert(rawPoses_, std::make_pair(jter->first, jter->second.getPose()));
						notifyDataLoaded = true;
					}
				}
			}
		}

		std::map<int, rtabmap::Transform> poses = rtabmapEvents.back().poses();

		// Transform pose in OpenGL world
		for(std::map<int, rtabmap::Transform>::iterator iter=poses.begin(); iter!=poses.end(); ++iter)
		{
			if(!graphOptimization_)
			{
				std::map<int, rtabmap::Transform>::iterator jter = rawPoses_.find(iter->first);
				if(jter != rawPoses_.end())
				{
					iter->second = opengl_world_T_rtabmap_world*jter->second;
				}
			}
			else
			{
				iter->second = opengl_world_T_rtabmap_world*iter->second;
			}
		}

		if(poses.size())
		{
			const std::multimap<int, rtabmap::Link> & links = rtabmapEvents.back().constraints();

			//update graph
			main_scene_.updateGraph(poses, links);

			// update clouds

			//filter poses?

			// make sure the last pose is here though
			//poses.insert(*rtabmapEvents.back().poses().rbegin());

			std::set<std::string> strIds;
			for(std::map<int, rtabmap::Transform>::iterator iter=poses.begin(); iter!=poses.end(); ++iter)
			{
				int id = iter->first;
				if(!iter->second.isNull())
				{
					if(main_scene_.hasCloud(id))
					{
						//just update pose
						main_scene_.setCloudPose(id, iter->second);
						main_scene_.setCloudVisible(id, true);
					}
					else if(uContains(bufferedSensorData, id))
					{
						rtabmap::SensorData & data = bufferedSensorData.at(id);
						if(!data.imageRaw().empty() && !data.depthRaw().empty())
						{
							// Voxelize and filter depending on the previous cloud?
							pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudWithoutNormals;
							pcl::IndicesPtr indices(new std::vector<int>);
							LOGI("Creating node cloud %d (image size=%dx%d)", id, data.imageRaw().cols, data.imageRaw().rows);
							cloudWithoutNormals = rtabmap::util3d::cloudRGBFromSensorData(data, renderingCloudDecimation, renderingCloudMaxDepth, 0, 0, indices.get());

							//compute normals
							pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud = rtabmap::util3d::computeNormals(cloudWithoutNormals, 6);

							if(cloud->size() && indices->size())
							{
								UTimer time;

								// substract? set points to NaN which are over previous cloud
								pcl::IndicesPtr indicesKept = indices;
								if(substractFiltering &&
									subtractRadius > 0.0 &&
									indices->size() &&
									previousCloud_.first > 0 &&
									previousCloud_.second.first.get() != 0 &&
									previousCloud_.second.second.get() != 0 &&
									previousCloud_.second.second->size() &&
									poses.find(previousCloud_.first) != poses.end())
								{
									rtabmap::Transform t = iter->second.inverse() * poses.at(previousCloud_.first);
									pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr previousCloud = rtabmap::util3d::transformPointCloud(previousCloud_.second.first, t);
									indicesKept = rtabmap::util3d::subtractFiltering(
											cloud,
											indices,
											previousCloud,
											previousCloud_.second.second,
											subtractRadius,
											subtractMaxAngle,
											minNeighborsInRadius);
									UINFO("Subtraction %fs", time.ticks());
								}

								previousCloud_.first = id;
								previousCloud_.second.first = cloud;
								previousCloud_.second.second = indices;

								// pcl::organizedFastMesh doesn't take indices, so set to NaN points we don't need to mesh
								pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
								pcl::ExtractIndices<pcl::PointXYZRGBNormal> filter;
								filter.setIndices(indicesKept);
								filter.setKeepOrganized(true);
								filter.setInputCloud(cloud);
								filter.filter(*output);
								LOGE("Filtering %d from %d -> %d (%fs)", (int)indices->size(), (int)indices->size(), (int)indicesKept->size(), time.ticks());

								std::vector<pcl::Vertices> polygons = rtabmap::util3d::organizedFastMesh(output, meshAngleTolerance, false, meshTrianglePixels);
								pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr outputCloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
								std::vector<pcl::Vertices> outputPolygons;

								if(!textureMeshing)
								{
									rtabmap::util3d::filterNotUsedVerticesFromMesh(
											*output,
											polygons,
											*outputCloud,
											outputPolygons);
								}
								else
								{
									outputCloud = output;
									outputPolygons = polygons;
								}
								LOGI("Creating mesh, %d polygons (%fs)", (int)outputPolygons.size(), time.ticks());

								if(outputCloud->size() && (!textureMeshing || outputPolygons.size()))
								{
									totalPolygons_ += outputPolygons.size();
									if(textureMeshing)
									{
										main_scene_.addCloud(id, outputCloud, outputPolygons, iter->second, data.imageRaw());
									}
									else
									{
										main_scene_.addCloud(id, outputCloud, outputPolygons, iter->second);
									}


									// protect createdMeshes_ used also by exportMesh() method
									boost::mutex::scoped_lock  lock(meshesMutex_);
									createdMeshes_.insert(std::make_pair(id, std::make_pair(std::make_pair(outputCloud, outputPolygons), iter->second)));
								}
								else
								{
									LOGE("Not mesh could be created for node %d", id);
								}
							}
							totalPoints_+=indices->size();
						}
					}
				}
			}
		}

		//update cloud visibility
		std::set<int> addedClouds = main_scene_.getAddedClouds();
		for(std::set<int>::const_iterator iter=addedClouds.begin();
			iter!=addedClouds.end();
			++iter)
		{
			if(*iter > 0 && (!mapCloudShown_ || poses.find(*iter) == poses.end()))
			{
				main_scene_.setCloudVisible(*iter, false);
			}
		}
	}
	else
	{
		rtabmap::OdometryEvent event;
		bool set = false;
		{
			boost::mutex::scoped_lock  lock(odomMutex_);
			if(odomEvents_.size())
			{
				LOGI("Process odom events");
				event = odomEvents_.back();
				odomEvents_.clear();
				set = true;
			}
		}

		//just process the last one
		if(set && !event.pose().isNull())
		{
			main_scene_.setCloudVisible(-1, false);
			if(odomCloudShown_ && !trajectoryMode_)
			{
				if(!event.data().imageRaw().empty() && !event.data().depthRaw().empty())
				{
					LOGI("Creating Odom cloud (rgb=%dx%d depth=%dx%d)",
							event.data().imageRaw().cols, event.data().imageRaw().rows,
							event.data().depthRaw().cols, event.data().depthRaw().rows);
					pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
					cloud = rtabmap::util3d::cloudRGBFromSensorData(event.data(), renderingCloudDecimation, renderingCloudMaxDepth);
					if(cloud->size())
					{
						std::vector<pcl::Vertices> polygons = rtabmap::util3d::organizedFastMesh(cloud, meshAngleTolerance, false, meshTrianglePixels);
						main_scene_.addCloud(-1, cloud, polygons, opengl_world_T_rtabmap_world*event.pose(), textureMeshing?event.data().imageRaw():cv::Mat());
						main_scene_.setCloudVisible(-1, true);
					}
					else
					{
						LOGE("Generated cloud is empty!");
					}
				}
				else
				{
					LOGE("Odom data images are empty!");
				}
			}
		}
	}

    lastDrawnCloudsCount_ = main_scene_.Render();

    return notifyDataLoaded?1:0;
}

void RTABMapApp::SetCameraType(
    tango_gl::GestureCamera::CameraType camera_type) {
  main_scene_.SetCameraType(camera_type);
}

void RTABMapApp::OnTouchEvent(int touch_count,
                                      tango_gl::GestureCamera::TouchEvent event,
                                      float x0, float y0, float x1, float y1) {
  main_scene_.OnTouchEvent(touch_count, event, x0, y0, x1, y1);
}

void RTABMapApp::setPausedMapping(bool paused)
{
	if(camera_)
	{
		if(paused)
		{
			LOGW("Pause!");
			camera_->kill();
		}
		else
		{
			LOGW("Resume!");
			camera_->start();
		}
	}
}
void RTABMapApp::setMapCloudShown(bool shown)
{
	mapCloudShown_ = shown;
}
void RTABMapApp::setOdomCloudShown(bool shown)
{
	odomCloudShown_ = shown;
	main_scene_.setTraceVisible(shown);
}
void RTABMapApp::setMeshRendering(bool enabled)
{
	main_scene_.setMeshRendering(enabled);
}
void RTABMapApp::setLocalizationMode(bool enabled)
{
	localizationMode_ = enabled;
	this->post(new rtabmap::ParamEvent(rtabmap::Parameters::kMemIncrementalMemory(), uBool2Str(!localizationMode_)));
}
void RTABMapApp::setTrajectoryMode(bool enabled)
{
	if(trajectoryMode_ != enabled)
	{
		main_scene_.SetCameraType(enabled?tango_gl::GestureCamera::kTopDown:tango_gl::GestureCamera::kThirdPersonFollow);
	}
	trajectoryMode_ = enabled;
	this->post(new rtabmap::ParamEvent(rtabmap::Parameters::kMemBinDataKept(), uBool2Str(!trajectoryMode_)));
}

void RTABMapApp::setGraphOptimization(bool enabled)
{
	graphOptimization_ = enabled;
}
void RTABMapApp::setGraphVisible(bool visible)
{
	main_scene_.setGraphVisible(visible);
}

void RTABMapApp::setAutoExposure(bool enabled)
{
	if(autoExposure_ != enabled)
	{
		autoExposure_ = enabled;
		if(camera_)
		{
			camera_->join(true);
			camera_->close();
			camera_->setAutoExposure(autoExposure_);
			onResume();
		}
		resetMapping();
	}
}

void RTABMapApp::resetMapping()
{
	LOGW("Reset!");
	status_.first = rtabmap::RtabmapEventInit::kInitializing;
	status_.second = "";

	clearSceneOnNextRender_ = true;

	UEventsManager::post(new rtabmap::RtabmapEventCmd(rtabmap::RtabmapEventCmd::kCmdResetMemory));
}

void RTABMapApp::save()
{
	UEventsManager::post(new rtabmap::RtabmapEventCmd(rtabmap::RtabmapEventCmd::kCmdClose));
}

bool RTABMapApp::exportMesh(const std::string & filePath)
{
	bool success = false;

	//Assemble the meshes
	UINFO("Organized fast mesh... ");

	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr mergedClouds(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	std::vector<pcl::Vertices> mergedPolygons;

	{
		boost::mutex::scoped_lock  lock(meshesMutex_);

		for(std::map<int, std::pair<std::pair<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr, std::vector<pcl::Vertices> >, rtabmap::Transform> >::iterator iter=createdMeshes_.begin();
			iter!= createdMeshes_.end();
			++iter)
		{
			pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr transformedCloud = rtabmap::util3d::transformPointCloud(iter->second.first.first, iter->second.second);
			if(mergedClouds->size() == 0)
			{
				*mergedClouds = *transformedCloud;
				mergedPolygons = iter->second.first.second;
			}
			else
			{
				rtabmap::util3d::appendMesh(*mergedClouds, mergedPolygons, *transformedCloud, iter->second.first.second);
			}
		}
	}
	if(closeVerticesDistance)
	{
		UINFO("Filtering assembled mesh (points=%d, polygons=%d, close vertices=%fm)...",
				(int)mergedClouds->size(), (int)mergedPolygons.size(), closeVerticesDistance);

		mergedPolygons = rtabmap::util3d::filterCloseVerticesFromMesh(
				mergedClouds,
				mergedPolygons,
				closeVerticesDistance,
				M_PI/4,
				true);

		// filter invalid polygons
		unsigned int count = mergedPolygons.size();
		mergedPolygons = rtabmap::util3d::filterInvalidPolygons(mergedPolygons);
		UINFO("Filtered %d invalid polygons.", (int)count-mergedPolygons.size());

		// filter not used vertices
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
		std::vector<pcl::Vertices> filteredPolygons;
		rtabmap::util3d::filterNotUsedVerticesFromMesh(*mergedClouds, mergedPolygons, *filteredCloud, filteredPolygons);
		mergedClouds = filteredCloud;
		mergedPolygons = filteredPolygons;
	}

	if(mergedClouds->size() && mergedPolygons.size())
	{
		pcl::PolygonMesh mesh;
		pcl::toPCLPointCloud2(*mergedClouds, mesh.cloud);
		mesh.polygons = mergedPolygons;

		UINFO("Saving to %s.", filePath.c_str());
		success = pcl::io::savePLYFileBinary(filePath, mesh) == 0;
	}
	return success;
}

void RTABMapApp::handleEvent(UEvent * event)
{
	if(camera_ && camera_->isRunning())
	{
		// called from events manager thread, so protect the data
		if(event->getClassName().compare("OdometryEvent") == 0)
		{
			LOGI("GUI: Received OdometryEvent!");
			if(odomMutex_.try_lock())
			{
				odomEvents_.clear();
				if(camera_->isRunning())
				{
					odomEvents_.push_back(*((rtabmap::OdometryEvent*)(event)));
				}
				odomMutex_.unlock();
			}
		}
		if(status_.first == rtabmap::RtabmapEventInit::kInitialized &&
		   event->getClassName().compare("RtabmapEvent") == 0)
		{
			LOGI("GUI: Received RtabmapEvent!");
			int nodes =0;
			int words = 0;
			int loopClosureId = 0;
			float updateTime = 0.0f;
			int databaseMemoryUsed = 0;
			int inliers = 0;
			int featuresExtracted = 0;
			float hypothesis = 0.0f;
			{
				boost::mutex::scoped_lock  lock(rtabmapMutex_);
				if(camera_->isRunning())
				{
					rtabmapEvents_.push_back(((rtabmap::RtabmapEvent*)event)->getStats());
					nodes = (int)uValue(rtabmapEvents_.back().data(), rtabmap::Statistics::kMemoryWorking_memory_size(), 0.0f) +
							uValue(rtabmapEvents_.back().data(), rtabmap::Statistics::kMemoryShort_time_memory_size(), 0.0f);
					words = (int)uValue(rtabmapEvents_.back().data(), rtabmap::Statistics::kKeypointDictionary_size(), 0.0f);
					updateTime = uValue(rtabmapEvents_.back().data(), rtabmap::Statistics::kTimingTotal(), 0.0f);
					loopClosureId = rtabmapEvents_.back().loopClosureId()>0?rtabmapEvents_.back().loopClosureId():rtabmapEvents_.back().proximityDetectionId()>0?rtabmapEvents_.back().proximityDetectionId():0;
					databaseMemoryUsed = (int)uValue(rtabmapEvents_.back().data(), rtabmap::Statistics::kMemoryDatabase_memory_used(), 0.0f);
					inliers = (int)uValue(rtabmapEvents_.back().data(), rtabmap::Statistics::kLoopVisual_inliers(), 0.0f);
					featuresExtracted = rtabmapEvents_.back().getSignatures().size()?rtabmapEvents_.back().getSignatures().rbegin()->second.getWords().size():0;
					hypothesis = uValue(rtabmapEvents_.back().data(), rtabmap::Statistics::kLoopHighest_hypothesis_value(), 0.0f);
				}
			}

			// Call JAVA callback with some stats
			bool success = false;
			if(jvm && RTABMapActivity)
			{
				JNIEnv *env = 0;
				jint rs = jvm->AttachCurrentThread(&env, NULL);
				if(rs == JNI_OK && env)
				{
					jclass clazz = env->GetObjectClass(RTABMapActivity);
					if(clazz)
					{
						jmethodID methodID = env->GetMethodID(clazz, "updateStatsCallback", "(IIIIFIIIIFI)V" );
						if(methodID)
						{
							env->CallVoidMethod(RTABMapActivity, methodID,
									nodes,
									words,
									totalPoints_,
									totalPolygons_,
									updateTime,
									loopClosureId,
									databaseMemoryUsed,
									inliers,
									featuresExtracted,
									hypothesis,
									lastDrawnCloudsCount_);
							success = true;
						}
					}
				}
				jvm->DetachCurrentThread();
			}
			if(!success)
			{
				UERROR("Failed to call RTABMapActivity::updateStatsCallback");
			}
		}
	}

	if(event->getClassName().compare("PoseEvent") == 0)
	{
		if(poseMutex_.try_lock())
		{
			poseEvents_.clear();
			poseEvents_.push_back(((rtabmap::PoseEvent*)event)->pose());
			poseMutex_.unlock();
		}
	}

	if(event->getClassName().compare("CameraTangoEvent") == 0)
	{
		rtabmap::CameraTangoEvent * tangoEvent = (rtabmap::CameraTangoEvent*)event;

		// Call JAVA callback with tango event msg
		bool success = false;
		if(jvm && RTABMapActivity)
		{
			JNIEnv *env = 0;
			jint rs = jvm->AttachCurrentThread(&env, NULL);
			if(rs == JNI_OK && env)
			{
				jclass clazz = env->GetObjectClass(RTABMapActivity);
				if(clazz)
				{
					jmethodID methodID = env->GetMethodID(clazz, "tangoEventCallback", "(ILjava/lang/String;Ljava/lang/String;)V" );
					if(methodID)
					{
						env->CallVoidMethod(RTABMapActivity, methodID,
								tangoEvent->type(),
								env->NewStringUTF(tangoEvent->key().c_str()),
								env->NewStringUTF(tangoEvent->value().c_str()));
						success = true;
					}
				}
			}
			jvm->DetachCurrentThread();
		}
		if(!success)
		{
			UERROR("Failed to call RTABMapActivity::tangoEventCallback");
		}
	}

	if(event->getClassName().compare("RtabmapEventInit") == 0)
	{
		LOGI("GUI: Received RtabmapEventInit!");
		status_.first = ((rtabmap::RtabmapEventInit*)event)->getStatus();
		status_.second = ((rtabmap::RtabmapEventInit*)event)->getInfo();

		if(status_.first == rtabmap::RtabmapEventInit::kClosed)
		{
			clearSceneOnNextRender_ = true;
		}

		// Call JAVA callback with init msg
		bool success = false;
		if(jvm && RTABMapActivity)
		{
			JNIEnv *env = 0;
			jint rs = jvm->AttachCurrentThread(&env, NULL);
			if(rs == JNI_OK && env)
			{
				jclass clazz = env->GetObjectClass(RTABMapActivity);
				if(clazz)
				{
					jmethodID methodID = env->GetMethodID(clazz, "rtabmapInitEventCallback", "(ILjava/lang/String;)V" );
					if(methodID)
					{
						env->CallVoidMethod(RTABMapActivity, methodID,
								status_.first,
								env->NewStringUTF(status_.second.c_str()));
						success = true;
					}
				}
			}
			jvm->DetachCurrentThread();
		}
		if(!success)
		{
			UERROR("Failed to call RTABMapActivity::rtabmapInitEventsCallback");
		}
	}
}

