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
#include <rtabmap/core/util2d.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/core/util3d_transforms.h>
#include <rtabmap/core/util3d_filtering.h>
#include <rtabmap/core/Graph.h>
#include <rtabmap/utilite/UEventsManager.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/UDirectory.h>
#include <rtabmap/utilite/UFile.h>
#include <opencv2/opencv_modules.hpp>
#include <rtabmap/core/util3d_surface.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/core/ParamEvent.h>
#include <rtabmap/core/Compression.h>
#include <rtabmap/core/Optimizer.h>
#include <rtabmap/core/VWDictionary.h>
#include <rtabmap/core/Memory.h>
#include <rtabmap/core/GainCompensator.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>

const int kVersionStringLength = 128;
const int minPolygonClusterSize = 200;

static JavaVM *jvm;
static jobject RTABMapActivity = 0;

namespace {
constexpr int kTangoCoreMinimumVersion = 9377;
}  // anonymous namespace.

rtabmap::ParametersMap RTABMapApp::getRtabmapParameters()
{
	rtabmap::ParametersMap parameters;

	parameters.insert(mappingParameters_.begin(), mappingParameters_.end());

	parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kKpDetectorStrategy(), std::string("6"))); // GFTT/BRIEF
	parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kKpMaxFeatures(), std::string("200")));
	parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kGFTTQualityLevel(), std::string("0.0001")));
	parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kMemImagePreDecimation(), std::string(fullResolution_?"2":"1")));
	parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kFASTThreshold(), std::string("1")));
	parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kBRIEFBytes(), std::string("64")));
	parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kRtabmapTimeThr(), std::string("800")));
	parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kRtabmapPublishLikelihood(), std::string("false")));
	parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kRtabmapPublishPdf(), std::string("false")));
	parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kRtabmapStartNewMapOnLoopClosure(), uBool2Str(appendMode_)));
	parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kMemBinDataKept(), uBool2Str(!trajectoryMode_)));
	parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kMemNotLinkedNodesKept(), std::string("false")));
	parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kOptimizerIterations(), graphOptimization_?"10":"0"));
	parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kMemIncrementalMemory(), uBool2Str(!localizationMode_)));
	parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kRtabmapMaxRetrieved(), "1"));
	parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kRGBDMaxLocalRetrieved(), "0"));
	parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kKpMaxDepth(), std::string("10"))); // to avoid extracting features in invalid depth (as we compute transformation directly from the words)
	parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kRGBDOptimizeFromGraphEnd(), std::string("true")));
	parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kDbSqlite3InMemory(), std::string("true")));
	parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kVisMinInliers(), std::string("25")));
	parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kVisEstimationType(), std::string("0"))); // 0=3D-3D 1=PnP
	parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kRGBDOptimizeMaxError(), std::string("0.1")));
	parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kRGBDProximityPathMaxNeighbors(), std::string("0"))); // disable scan matching to merged nodes
	parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kRGBDProximityBySpace(), std::string("false"))); // just keep loop closure detection

	parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kRGBDNeighborLinkRefining(), uBool2Str(driftCorrection_)));
	parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kRegStrategy(), std::string(driftCorrection_?"2":"0")));
	parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kIcpPointToPlane(), std::string("true")));
	parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kMemLaserScanNormalK(), std::string("10")));
	parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kIcpIterations(), std::string("10")));
	parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kIcpEpsilon(), std::string("0.001")));
	parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kIcpMaxRotation(), std::string("0.17"))); // 10 degrees
	parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kIcpMaxTranslation(), std::string("0.05")));
	parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kIcpCorrespondenceRatio(), std::string("0.5")));
	parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kIcpMaxCorrespondenceDistance(), std::string("0.05")));

	parameters.insert(*rtabmap::Parameters::getDefaultParameters().find(rtabmap::Parameters::kKpMaxFeatures()));
	parameters.insert(*rtabmap::Parameters::getDefaultParameters().find(rtabmap::Parameters::kMemRehearsalSimilarity()));
	parameters.insert(*rtabmap::Parameters::getDefaultParameters().find(rtabmap::Parameters::kMemMapLabelsAdded()));
	if(dataRecorderMode_)
	{
		uInsert(parameters, rtabmap::ParametersPair(rtabmap::Parameters::kKpMaxFeatures(), std::string("-1")));
		uInsert(parameters, rtabmap::ParametersPair(rtabmap::Parameters::kMemRehearsalSimilarity(), std::string("1.0"))); // deactivate rehearsal
		uInsert(parameters, rtabmap::ParametersPair(rtabmap::Parameters::kMemMapLabelsAdded(), "false")); // don't create map labels
	}

	return parameters;
}

RTABMapApp::RTABMapApp() :
		camera_(0),
		rtabmapThread_(0),
		rtabmap_(0),
		logHandler_(0),
		odomCloudShown_(true),
		graphOptimization_(true),
		nodesFiltering_(false),
		driftCorrection_(false),
		localizationMode_(false),
		trajectoryMode_(false),
		autoExposure_(true),
		fullResolution_(false),
		appendMode_(true),
		maxCloudDepth_(0.0),
		meshDecimation_(1),
		meshTrianglePix_(1),
		meshAngleToleranceDeg_(15.0),
		paused_(false),
		dataRecorderMode_(false),
		clearSceneOnNextRender_(false),
		filterPolygonsOnNextRender_(false),
		gainCompensationOnNextRender_(0),
		bilateralFilteringOnNextRender_(false),
		cameraJustInitialized_(false),
		totalPoints_(0),
		totalPolygons_(0),
		lastDrawnCloudsCount_(0),
		renderingTime_(0.0f)

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

void RTABMapApp::onCreate(JNIEnv* env, jobject caller_activity)
{
	env->GetJavaVM(&jvm);
	RTABMapActivity = env->NewGlobalRef(caller_activity);

	LOGI("RTABMapApp::onCreate()");
	createdMeshes_.clear();
	rawPoses_.clear();
	clearSceneOnNextRender_ = true;
	totalPoints_ = 0;
	totalPolygons_ = 0;
	lastDrawnCloudsCount_ = 0;
	renderingTime_ = 0.0f;

	if(camera_)
	{
	  delete camera_;
	}
	if(rtabmapThread_)
	{
		rtabmapThread_->close(false);
	    delete rtabmapThread_;
	    rtabmapThread_ = 0;
	    rtabmap_ = 0;
	}

	if(logHandler_ == 0)
	{
		logHandler_ = new LogHandler();
	}

	this->registerToEventsManager();

	camera_ = new rtabmap::CameraTango(fullResolution_?1:2, autoExposure_);
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
		rtabmap_ = 0;
	}

	//Rtabmap
	rtabmap_ = new rtabmap::Rtabmap();
	rtabmap::ParametersMap parameters = getRtabmapParameters();

	rtabmap_->init(parameters, databasePath);
	rtabmapThread_ = new rtabmap::RtabmapThread(rtabmap_);
	if(parameters.find(rtabmap::Parameters::kRtabmapDetectionRate()) != parameters.end())
	{
		rtabmapThread_->setDetectorRate(uStr2Float(parameters.at(rtabmap::Parameters::kRtabmapDetectionRate())));
	}

	// Generate all meshes
	std::map<int, rtabmap::Signature> signatures;
	std::map<int, rtabmap::Transform> poses;
	std::multimap<int, rtabmap::Link> links;
	rtabmap_->get3DMap(
			signatures,
			poses,
			links,
			true,
			true);

	clearSceneOnNextRender_ = true;
	rtabmap::Statistics stats;
	stats.setSignatures(signatures);
	stats.addStatistic(rtabmap::Statistics::kMemoryWorking_memory_size(), (float)rtabmap_->getWMSize());
	stats.addStatistic(rtabmap::Statistics::kKeypointDictionary_size(), (float)rtabmap_->getMemory()->getVWDictionary()->getVisualWords().size());
	stats.addStatistic(rtabmap::Statistics::kMemoryDatabase_memory_used(), (float)rtabmap_->getMemory()->getDatabaseMemoryUsed());
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

bool RTABMapApp::onTangoServiceConnected(JNIEnv* env, jobject iBinder)
{
	LOGW("onTangoServiceConnected()");
	if(camera_)
	{
		camera_->join(true);

		if (TangoService_setBinder(env, iBinder) != TANGO_SUCCESS) {
		    LOGE("TangoHandler::ConnectTango, TangoService_setBinder error");
		    return false;
		}

		if(camera_->init())
		{
			LOGI("Start camera thread");
			if(!paused_)
			{
				camera_->start();
			}
			cameraJustInitialized_ = true;
			return true;
		}
		LOGE("Failed camera initialization!");
	}
	return false;
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

class PostRenderEvent : public UEvent
{
public:
	PostRenderEvent(const rtabmap::Statistics & stats) :
		stats_(stats)
	{

	}
	virtual std::string getClassName() const {return "PostRenderEvent";}
	const rtabmap::Statistics & getStats() const {return stats_;}
private:
	rtabmap::Statistics stats_;
};

// OpenGL thread
bool RTABMapApp::smoothMesh(int id, Mesh & mesh)
{
	UTimer t;
	// reconstruct depth image
	UASSERT(mesh.indices.get() && mesh.indices->size());
	cv::Mat depth = cv::Mat::zeros(mesh.cloud->height, mesh.cloud->width, CV_32FC1);
	rtabmap::Transform localTransformInv = mesh.cameraModel.localTransform().inverse();
	for(unsigned int i=0; i<mesh.indices->size(); ++i)
	{
		int index = mesh.indices->at(i);
		// FastBilateralFilter works in camera frame
		if(mesh.cloud->at(index).x > 0)
		{
			pcl::PointXYZRGB pt = rtabmap::util3d::transformPoint(mesh.cloud->at(index), localTransformInv);
			depth.at<float>(index) = pt.z;
		}
	}

	depth = rtabmap::util2d::fastBilateralFiltering(depth, 2.0f, 0.075f);
	LOGI("smoothMesh() Bilateral filtering of %d, time=%fs", id, t.ticks());

	if(!depth.empty() && mesh.indices->size())
	{
		pcl::IndicesPtr newIndices(new std::vector<int>(mesh.indices->size()));
		int oi = 0;
		for(unsigned int i=0; i<mesh.indices->size(); ++i)
		{
			int index = mesh.indices->at(i);

			 pcl::PointXYZRGB & pt = mesh.cloud->at(index);
			 pcl::PointXYZRGB newPt = rtabmap::util3d::transformPoint(mesh.cloud->at(index), localTransformInv);
			 if(depth.at<float>(index) > 0)
			 {
				 newPt.z = depth.at<float>(index);
				 newPt = rtabmap::util3d::transformPoint(newPt, mesh.cameraModel.localTransform());
				 newIndices->at(oi++) = index;
			 }
			 else
			 {
				 newPt.x = newPt.y = newPt.z = std::numeric_limits<float>::quiet_NaN();
			 }
			 pt.x = newPt.x;
			 pt.y = newPt.y;
			 pt.z = newPt.z;
		}
		newIndices->resize(oi);
		mesh.indices = newIndices;

		//reconstruct the mesh with smoothed surfaces
		std::vector<pcl::Vertices> polygons;
		if(main_scene_.isMeshRendering())
		{
			polygons = rtabmap::util3d::organizedFastMesh(mesh.cloud, meshAngleToleranceDeg_*M_PI/180.0, false, meshTrianglePix_);
		}
		LOGI("smoothMesh() Reconstructing the mesh of %d, time=%fs", id, t.ticks());
		mesh.polygons = polygons;
	}
	else
	{
		LOGE("smoothMesh() Failed to smooth surface %d", id);
		return false;
	}
	return true;
}

// OpenGL thread
int RTABMapApp::Render()
{
	UTimer fpsTime;
	bool notifyCameraStarted = false;
	bool notifyDataLoaded = false;
	boost::mutex::scoped_lock  lock(renderingMutex_);

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
		rawPoses_.clear();
		totalPoints_ = 0;
		totalPolygons_ = 0;
		lastDrawnCloudsCount_ = 0;
		renderingTime_ = 0.0f;
	}

	// Did we lose OpenGL context? If so, recreate the context;
	std::set<int> added = main_scene_.getAddedClouds();
	added.erase(-1);
	if(added.size() != createdMeshes_.size())
	{
		for(std::map<int, Mesh>::iterator iter=createdMeshes_.begin(); iter!=createdMeshes_.end(); ++iter)
		{
			if(!main_scene_.hasCloud(iter->first))
			{
				cv::Mat texture;
				if(main_scene_.isMeshTexturing())
				{
					texture = rtabmap::uncompressImage(rtabmap_->getMemory()->getImageCompressed(iter->first));
				}
				main_scene_.addMesh(iter->first, iter->second, texture, opengl_world_T_rtabmap_world*iter->second.pose);
				main_scene_.setCloudVisible(iter->first, iter->second.visible);
			}
		}
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
		main_scene_.SetCameraPose(opengl_world_T_tango_world*pose);
		if(!camera_->isRunning() && cameraJustInitialized_)
		{
			notifyCameraStarted = true;
			cameraJustInitialized_ = false;
		}
	}
	rtabmap::OdometryEvent odomEvent;
	{
		boost::mutex::scoped_lock  lock(odomMutex_);
		if(odomEvents_.size())
		{
			LOGI("Process odom events");
			odomEvent = odomEvents_.back();
			odomEvents_.clear();
			if(cameraJustInitialized_)
			{
				notifyCameraStarted = true;
				cameraJustInitialized_ = false;
			}
		}
	}

	if(rtabmapEvents.size())
	{
		LOGI("Process rtabmap events");

		// update buffered signatures
		std::map<int, rtabmap::SensorData> bufferedSensorData;
		if(!trajectoryMode_ && !dataRecorderMode_)
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
					else if(totalPoints_ == 0 &&
							!jter->second.sensorData().imageCompressed().empty() &&
							!jter->second.sensorData().depthOrRightCompressed().empty())
					{
						// uncompress
						rtabmap::SensorData data = jter->second.sensorData();
						cv::Mat tmpA,depth;
						data.uncompressData(&tmpA, &depth);

						// do post-processing bilateral filtering now
						UTimer t;
						depth = rtabmap::util2d::fastBilateralFiltering(depth, 2.0f, 0.075f);
						data.setDepthOrRightRaw(depth);
						LOGI("Bilateral filtering of %d, time=%fs", jter->first, t.ticks());

						uInsert(bufferedSensorData, std::make_pair(jter->first, data));
						uInsert(rawPoses_, std::make_pair(jter->first, jter->second.getPose()));

						LOGI("Detecting that we are loading a database, so do some post-processing...");
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

		const std::multimap<int, rtabmap::Link> & links = rtabmapEvents.back().constraints();
		if(poses.size())
		{
			//update graph
			main_scene_.updateGraph(poses, links);

			// update clouds
			boost::mutex::scoped_lock  lock(meshesMutex_);
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
						std::map<int, Mesh>::iterator meshIter = createdMeshes_.find(id);
						UASSERT(meshIter!=createdMeshes_.end());
						meshIter->second.pose = opengl_world_T_rtabmap_world.inverse()*iter->second;
						meshIter->second.visible = true;
					}
					else if(uContains(bufferedSensorData, id))
					{
						rtabmap::SensorData & data = bufferedSensorData.at(id);
						if(!data.imageRaw().empty() && !data.depthRaw().empty())
						{
							// Voxelize and filter depending on the previous cloud?
							pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
							pcl::IndicesPtr indices(new std::vector<int>);
							LOGI("Creating node cloud %d (depth=%dx%d rgb=%dx%d)", id, data.depthRaw().cols, data.depthRaw().rows, data.imageRaw().cols, data.imageRaw().rows);
							cloud = rtabmap::util3d::cloudRGBFromSensorData(data, meshDecimation_, maxCloudDepth_, 0, indices.get());

							if(cloud->size() && indices->size())
							{
								UTimer time;
								std::vector<pcl::Vertices> polygons;
								if(main_scene_.isMeshRendering())
								{
									polygons = rtabmap::util3d::organizedFastMesh(cloud, meshAngleToleranceDeg_*M_PI/180.0, false, meshTrianglePix_);
									LOGI("Creating mesh, %d polygons (%fs)", (int)polygons.size(), time.ticks());
								}

								if((main_scene_.isMeshRendering() && polygons.size()) || !main_scene_.isMeshRendering())
								{
									totalPolygons_ += polygons.size();

									std::pair<std::map<int, Mesh>::iterator, bool> inserted = createdMeshes_.insert(std::make_pair(id, Mesh()));
									UASSERT(inserted.second);
									inserted.first->second.cloud = cloud;
									inserted.first->second.indices = indices;
									inserted.first->second.polygons = polygons;
									inserted.first->second.pose = opengl_world_T_rtabmap_world.inverse()*iter->second;
									inserted.first->second.visible = true;
									inserted.first->second.cameraModel = data.cameraModels()[0];
									inserted.first->second.gain = 1.0f;

									main_scene_.addMesh(id, inserted.first->second, main_scene_.isMeshTexturing()?data.imageRaw():cv::Mat(), iter->second);
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

		//filter poses?
		if(poses.size() > 2)
		{
			if(nodesFiltering_)
			{
				for(std::multimap<int, rtabmap::Link>::const_iterator iter=links.begin(); iter!=links.end(); ++iter)
				{
					if(iter->second.type() != rtabmap::Link::kNeighbor)
					{
						int oldId = iter->second.to()>iter->second.from()?iter->second.from():iter->second.to();
						poses.erase(oldId);
					}
				}
			}
		}

		if(poses.size())
		{
			//update cloud visibility
			std::set<int> addedClouds = main_scene_.getAddedClouds();
			for(std::set<int>::const_iterator iter=addedClouds.begin();
				iter!=addedClouds.end();
				++iter)
			{
				if(*iter > 0 && poses.find(*iter) == poses.end())
				{
					main_scene_.setCloudVisible(*iter, false);
					std::map<int, Mesh>::iterator meshIter = createdMeshes_.find(*iter);
					UASSERT(meshIter!=createdMeshes_.end());
					meshIter->second.visible = false;
				}
			}
		}
	}
	else
	{
		main_scene_.setCloudVisible(-1, odomCloudShown_ && !trajectoryMode_ && !paused_);

		//just process the last one
		if(!odomEvent.pose().isNull())
		{
			if(odomCloudShown_ && !trajectoryMode_)
			{
				if(!odomEvent.data().imageRaw().empty() && !odomEvent.data().depthRaw().empty())
				{
					pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
					pcl::IndicesPtr indices(new std::vector<int>);
					cloud = rtabmap::util3d::cloudRGBFromSensorData(odomEvent.data(), meshDecimation_, maxCloudDepth_, 0.0f, indices.get());
					if(cloud->size() && indices->size())
					{
						LOGI("Created odom cloud (rgb=%dx%d depth=%dx%d cloud=%dx%d)",
								odomEvent.data().imageRaw().cols, odomEvent.data().imageRaw().rows,
								odomEvent.data().depthRaw().cols, odomEvent.data().depthRaw().rows,
							   (int)cloud->width, (int)cloud->height);
						main_scene_.addCloud(-1, cloud, indices, opengl_world_T_rtabmap_world*odomEvent.pose());
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

	if(notifyDataLoaded || gainCompensationOnNextRender_>0)
	{
		UTimer tGainCompensation;
		LOGI("Gain compensation...");
		boost::mutex::scoped_lock  lock(meshesMutex_);

		std::map<int, pcl::PointCloud<pcl::PointXYZRGB>::Ptr > clouds;
		std::map<int, pcl::IndicesPtr> indices;
		for(std::map<int, Mesh>::iterator iter = createdMeshes_.begin(); iter!=createdMeshes_.end(); ++iter)
		{
			clouds.insert(std::make_pair(iter->first, iter->second.cloud));
			indices.insert(std::make_pair(iter->first, iter->second.indices));
		}
		std::map<int, rtabmap::Transform> poses;
		std::multimap<int, rtabmap::Link> links;
		rtabmap_->getGraph(poses, links, true, true);
		if(gainCompensationOnNextRender_ == 2)
		{
			// full compensation
			links.clear();
			for(std::map<int, pcl::PointCloud<pcl::PointXYZRGB>::Ptr>::const_iterator iter=clouds.begin(); iter!=clouds.end(); ++iter)
			{
				int from = iter->first;
				std::map<int, pcl::PointCloud<pcl::PointXYZRGB>::Ptr>::const_iterator jter = iter;
				++jter;
				for(;jter!=clouds.end(); ++jter)
				{
					int to = jter->first;
					links.insert(std::make_pair(from, rtabmap::Link(from, to, rtabmap::Link::kUserClosure, poses.at(from).inverse()*poses.at(to))));
				}
			}
		}

		rtabmap::GainCompensator compensator;
		if(clouds.size() > 1 && links.size())
		{
			compensator.feed(clouds, indices, links);
			LOGI("Gain compensation... compute gain: links=%d, time=%fs", (int)links.size(), tGainCompensation.ticks());
		}

		for(std::map<int, Mesh>::iterator iter = createdMeshes_.begin(); iter!=createdMeshes_.end(); ++iter)
		{
			if(!iter->second.cloud->empty())
			{
				if(clouds.size() > 1 && links.size())
				{
					iter->second.gain = compensator.getGain(iter->first);
				}
			}

			main_scene_.updateMesh(iter->first, iter->second, cv::Mat());
		}
		LOGI("Gain compensation... applying gain: meshes=%d, time=%fs", (int)createdMeshes_.size(), tGainCompensation.ticks());

		gainCompensationOnNextRender_ = 0;
		notifyDataLoaded = true;
	}

	if(bilateralFilteringOnNextRender_)
	{
		LOGI("Bilateral filtering...");
		bilateralFilteringOnNextRender_ = false;
		boost::mutex::scoped_lock  lock(meshesMutex_);
		for(std::map<int, Mesh>::iterator iter = createdMeshes_.begin(); iter!=createdMeshes_.end(); ++iter)
		{
			if(iter->second.cloud->size() && iter->second.indices->size())
			{
				if(smoothMesh(iter->first, iter->second))
				{
					main_scene_.updateMesh(iter->first, iter->second, cv::Mat());
				}
			}
		}
		notifyDataLoaded = true;
	}

	if(filterPolygonsOnNextRender_)
	{
		LOGI("Polygon filtering...");
		filterPolygonsOnNextRender_ = false;
		boost::mutex::scoped_lock  lock(meshesMutex_);
		for(std::map<int, Mesh>::iterator iter = createdMeshes_.begin(); iter!=createdMeshes_.end(); ++iter)
		{
			if(iter->second.polygons.size())
			{
				// filter polygons
				std::vector<std::set<int> > neighbors;
				std::vector<std::set<int> > vertexToPolygons;
				rtabmap::util3d::createPolygonIndexes(
						iter->second.polygons,
						iter->second.cloud->size(),
						neighbors,
						vertexToPolygons);
				std::list<std::list<int> > clusters = rtabmap::util3d::clusterPolygons(
						neighbors,
						minPolygonClusterSize);
				std::vector<pcl::Vertices> filteredPolygons(iter->second.polygons.size());
				int oi=0;
				for(std::list<std::list<int> >::iterator jter=clusters.begin(); jter!=clusters.end(); ++jter)
				{
					for(std::list<int>::iterator kter=jter->begin(); kter!=jter->end(); ++kter)
					{
						filteredPolygons[oi++] = iter->second.polygons.at(*kter);
					}
				}
				filteredPolygons.resize(oi);
				iter->second.polygons = filteredPolygons;
				main_scene_.updateCloudPolygons(iter->first, iter->second.polygons);
			}
		}
		notifyDataLoaded = true;
	}

    lastDrawnCloudsCount_ = main_scene_.Render();
    if(renderingTime_ < fpsTime.elapsed())
    {
    	renderingTime_ = fpsTime.elapsed();
    }

    if(rtabmapEvents.size())
    {
    	// send statistics to GUI
		LOGI("Posting PostRenderEvent!");
		UEventsManager::post(new PostRenderEvent(rtabmapEvents.back()));
    }

    return notifyDataLoaded||notifyCameraStarted?1:0;
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
	paused_ = paused;
	if(camera_)
	{
		if(paused_)
		{
			LOGW("Pause!");
			camera_->kill();

		}
		else
		{
			LOGW("Resume!");
			UEventsManager::post(new rtabmap::RtabmapEventCmd(rtabmap::RtabmapEventCmd::kCmdTriggerNewMap));
			camera_->start();
		}
	}
}
void RTABMapApp::setMapCloudShown(bool shown)
{
	main_scene_.setMapRendering(shown);
}
void RTABMapApp::setOdomCloudShown(bool shown)
{
	odomCloudShown_ = shown;
	main_scene_.setTraceVisible(shown);
}
void RTABMapApp::setMeshRendering(bool enabled, bool withTexture)
{
	main_scene_.setMeshRendering(enabled, withTexture);
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
	if(!camera_->isRunning())
	{
		std::map<int, rtabmap::Transform> poses;
		std::multimap<int, rtabmap::Link> links;
		rtabmap_->getGraph(poses, links, true, true);
		if(poses.size())
		{
			boost::mutex::scoped_lock  lock(rtabmapMutex_);
			rtabmap::Statistics stats = rtabmap_->getStatistics();
			stats.setPoses(poses);
			stats.setConstraints(links);
			rtabmapEvents_.push_back(stats);

			rtabmap_->setOptimizedPoses(poses);
		}
	}
}
void RTABMapApp::setNodesFiltering(bool enabled)
{
	nodesFiltering_ = enabled;
	setGraphOptimization(graphOptimization_); // this will resend the graph if paused
}
void RTABMapApp::setDriftCorrection(bool enabled)
{
	driftCorrection_ = enabled;
	rtabmap::ParametersMap parameters;
	parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kRGBDNeighborLinkRefining(), uBool2Str(driftCorrection_)));
	parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kRegStrategy(), std::string(driftCorrection_?"1":"0")));
	this->post(new rtabmap::ParamEvent(parameters));
}
void RTABMapApp::setGraphVisible(bool visible)
{
	main_scene_.setGraphVisible(visible);
	main_scene_.setTraceVisible(visible);
}
void RTABMapApp::setGridVisible(bool visible)
{
	main_scene_.setGridVisible(visible);
}

void RTABMapApp::setAutoExposure(bool enabled)
{
	if(autoExposure_ != enabled)
	{
		autoExposure_ = enabled;
		if(camera_)
		{
			camera_->setAutoExposure(autoExposure_);
		}
	}
}

void RTABMapApp::setFullResolution(bool enabled)
{
	if(fullResolution_ != enabled)
	{
		fullResolution_ = enabled;
		if(camera_)
		{
			camera_->setDecimation(fullResolution_?1:2);
		}
		rtabmap::ParametersMap parameters;
		parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kMemImagePreDecimation(), std::string(fullResolution_?"2":"1")));
		this->post(new rtabmap::ParamEvent(parameters));
	}
}

void RTABMapApp::setAppendMode(bool enabled)
{
	if(appendMode_ != enabled)
	{
		appendMode_ = enabled;
		rtabmap::ParametersMap parameters;
		parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kRtabmapStartNewMapOnLoopClosure(), uBool2Str(appendMode_)));
		this->post(new rtabmap::ParamEvent(parameters));
	}
}

void RTABMapApp::setDataRecorderMode(bool enabled)
{
	if(dataRecorderMode_ != enabled)
	{
		dataRecorderMode_ = enabled; // parameters will be set when resuming (we assume we are paused)
	}
}

void RTABMapApp::setMaxCloudDepth(float value)
{
	maxCloudDepth_ = value;
}

void RTABMapApp::setMeshDecimation(int value)
{
	LOGE("Set decimation to level %d", value);
	meshDecimation_ = 1;
	if(camera_)
	{
		// Google Tango Tablet 160x90
		// Phab2Pro 240x135
		int width = camera_->getCameraModel().imageWidth()/8;
		if(value == 2) // high
		{
			if(width % 10 == 0)
			{
				meshDecimation_ = 10;
			}
			else if(width % 15 == 0)
			{
				meshDecimation_ = 15;
			}
			else
			{
				LOGE("Could not set decimation to high (width=%d)", width);
			}
		}
		else if(value == 1) // medium
		{
			if(width % 5 == 0)
			{
				meshDecimation_ = 5;
			}
			else
			{
				LOGE("Could not set decimation to medium (width=%d)", width);
			}
		}
	}
	LOGE("Set decimation to %d", meshDecimation_);
}

void RTABMapApp::setMeshAngleTolerance(float value)
{
	meshAngleToleranceDeg_ = value;
}

void RTABMapApp::setMeshTriangleSize(int value)
{
	 meshTrianglePix_ = value;
}

int RTABMapApp::setMappingParameter(const std::string & key, const std::string & value)
{
	std::string compatibleKey = key;

	// Backward compatibility
	std::map<std::string, std::pair<bool, std::string> >::const_iterator iter=rtabmap::Parameters::getRemovedParameters().find(key);
	if(iter != rtabmap::Parameters::getRemovedParameters().end())
	{
		if(iter->second.first)
		{
			// can be migrated
			compatibleKey = iter->second.second;
			LOGW("Parameter name changed: \"%s\" -> \"%s\". Please update the code accordingly. Value \"%s\" is still set to the new parameter name.",
					iter->first.c_str(), iter->second.second.c_str(), value.c_str());
		}
		else
		{
			if(iter->second.second.empty())
			{
				LOGE("Parameter \"%s\" doesn't exist anymore!",
						iter->first.c_str());
			}
			else
			{
				LOGE("Parameter \"%s\" doesn't exist anymore! You may look at this similar parameter: \"%s\"",
						iter->first.c_str(), iter->second.second.c_str());
			}
		}
	}

	if(rtabmap::Parameters::getDefaultParameters().find(compatibleKey) != rtabmap::Parameters::getDefaultParameters().end())
	{
		LOGI(uFormat("Setting param \"%s\"  to \"%s\"", compatibleKey.c_str(), value.c_str()).c_str());
		uInsert(mappingParameters_, rtabmap::ParametersPair(key, value));
		UEventsManager::post(new rtabmap::ParamEvent(mappingParameters_));
		return 0;
	}
	else
	{
		LOGE(uFormat("Key \"%s\" doesn't exist!", compatibleKey.c_str()).c_str());
		return -1;
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

void RTABMapApp::save(const std::string & databasePath)
{
	rtabmapThread_->join(true);

	// save mapping parameters in the database

	bool appendModeBackup = appendMode_;
	if(appendMode_)
	{
		appendMode_ = false;
	}

	bool dataRecorderModeBackup = dataRecorderMode_;
	if(dataRecorderMode_)
	{
		dataRecorderMode_ = false;
	}

	if(appendModeBackup || dataRecorderModeBackup)
	{
		rtabmap::ParametersMap parameters = getRtabmapParameters();
		rtabmap_->parseParameters(parameters);
		appendMode_ = appendModeBackup;
		dataRecorderMode_ = dataRecorderModeBackup;
	}

	rtabmap_->close(true, databasePath);
	rtabmap_->init(getRtabmapParameters(), dataRecorderMode_?"":databasePath);
	if(dataRecorderMode_)
	{
		clearSceneOnNextRender_ = true;
	}
	rtabmapThread_->start();

}

bool RTABMapApp::exportMesh(const std::string & filePath)
{
	bool success = false;

	//Assemble the meshes
	if(UFile::getExtension(filePath).compare("obj") == 0)
	{
		pcl::TextureMesh textureMesh;
		std::vector<int> textures;
		int totalPolygons = 0;
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr mergedClouds(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
		{
			boost::mutex::scoped_lock  lock(meshesMutex_);

			textureMesh.tex_materials.resize(createdMeshes_.size());
			textureMesh.tex_polygons.resize(createdMeshes_.size());
			textureMesh.tex_coordinates.resize(createdMeshes_.size());
			textures.resize(createdMeshes_.size());

			int polygonsStep = 0;
			int oi = 0;
			for(std::map<int, Mesh>::iterator iter=createdMeshes_.begin();
				iter!= createdMeshes_.end();
				++iter)
			{
				if(!rtabmap_->getMemory()->getImageCompressed(iter->first).empty() &&
					iter->second.cloud->size() &&
					iter->second.polygons.size())
				{
					// Convert organized to dense cloud
					pcl::PointCloud<pcl::PointXYZRGB>::Ptr outputCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
					std::vector<pcl::Vertices> outputPolygons;
					std::vector<int> denseToOrganizedIndices = rtabmap::util3d::filterNaNPointsFromMesh(*iter->second.cloud, iter->second.polygons, *outputCloud, outputPolygons);

					if(iter->second.gain != 1.0f)
					{
						for(unsigned int i=0; i<outputCloud->size(); ++i)
						{
							pcl::PointXYZRGB & pt = outputCloud->at(i);
							pt.r = uchar(std::max(0.0, std::min(255.0, double(pt.r) * iter->second.gain)));
							pt.g = uchar(std::max(0.0, std::min(255.0, double(pt.g) * iter->second.gain)));
							pt.b = uchar(std::max(0.0, std::min(255.0, double(pt.b) * iter->second.gain)));
						}
					}

					// OBJ format requires normals
					pcl::PointCloud<pcl::Normal>::Ptr normals = rtabmap::util3d::computeNormals(outputCloud, 6);

					pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudWithNormals(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
					pcl::concatenateFields(*outputCloud, *normals, *cloudWithNormals);

					// polygons
					UASSERT(outputPolygons.size());
					unsigned int polygonSize = outputPolygons.front().vertices.size();
					textureMesh.tex_polygons[oi].resize(outputPolygons.size());
					textureMesh.tex_coordinates[oi].resize(outputPolygons.size() * polygonSize);
					for(unsigned int j=0; j<outputPolygons.size(); ++j)
					{
						pcl::Vertices vertices = outputPolygons[j];
						UASSERT(polygonSize == vertices.vertices.size());
						for(unsigned int k=0; k<vertices.vertices.size(); ++k)
						{
							//uv
							UASSERT(vertices.vertices[k] < denseToOrganizedIndices.size());
							int originalVertex = denseToOrganizedIndices[vertices.vertices[k]];
							textureMesh.tex_coordinates[oi][j*vertices.vertices.size()+k] = Eigen::Vector2f(
									float(originalVertex % iter->second.cloud->width) / float(iter->second.cloud->width),   // u
									float(iter->second.cloud->height - originalVertex / iter->second.cloud->width) / float(iter->second.cloud->height)); // v

							vertices.vertices[k] += polygonsStep;
						}
						textureMesh.tex_polygons[oi][j] = vertices;

					}
					totalPolygons += outputPolygons.size();
					polygonsStep += outputCloud->size();

					pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr transformedCloud = rtabmap::util3d::transformPointCloud(cloudWithNormals, iter->second.pose);
					if(mergedClouds->size() == 0)
					{
						*mergedClouds = *transformedCloud;
					}
					else
					{
						*mergedClouds += *transformedCloud;
					}

					textures[oi] = iter->first;
					textureMesh.tex_materials[oi].tex_illum = 1;
					textureMesh.tex_materials[oi].tex_name = uFormat("material_%d", iter->first);
					++oi;
				}
				else
				{
					UERROR("Texture not set for mesh %d", iter->first);
				}
			}
			textureMesh.tex_materials.resize(oi);
			textureMesh.tex_polygons.resize(oi);
			textures.resize(oi);

			if(textures.size())
			{
				pcl::toPCLPointCloud2(*mergedClouds, textureMesh.cloud);

				std::string textureDirectory = uSplit(filePath, '.').front();
				UINFO("Saving %d textures to %s.", textures.size(), textureDirectory.c_str());
				UDirectory::makeDir(textureDirectory);
				for(unsigned int i=0;i<textures.size(); ++i)
				{
					cv::Mat rawImage = rtabmap::uncompressImage(rtabmap_->getMemory()->getImageCompressed(textures[i]));
					if(createdMeshes_.at(textures[i]).gain != 1.0f)
					{
						cv::multiply(rawImage, createdMeshes_.at(textures[i]).gain, rawImage);
					}
					std::string texFile = textureDirectory+"/"+textureMesh.tex_materials[i].tex_name+".png";
					cv::imwrite(texFile, rawImage);

					UINFO("Saved %s (%d bytes).", texFile.c_str(), rawImage.total()*rawImage.channels());

					// relative path
					textureMesh.tex_materials[i].tex_file = uSplit(UFile::getName(filePath), '.').front()+"/"+textureMesh.tex_materials[i].tex_name+".png";
				}

				UINFO("Saving obj (%d vertices, %d polygons) to %s.", (int)mergedClouds->size(), totalPolygons, filePath.c_str());
				success = pcl::io::saveOBJFile(filePath, textureMesh) == 0;
				if(success)
				{
					UINFO("Saved obj to %s!", filePath.c_str());
				}
				else
				{
					UERROR("Failed saving obj to %s!", filePath.c_str());
				}
			}
		}
	}
	else
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr mergedClouds(new pcl::PointCloud<pcl::PointXYZRGB>);
		std::vector<pcl::Vertices> mergedPolygons;

		{
			boost::mutex::scoped_lock  lock(meshesMutex_);

			for(std::map<int, Mesh>::iterator iter=createdMeshes_.begin();
				iter!= createdMeshes_.end();
				++iter)
			{
				// Convert organized to dense cloud
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr outputCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
				std::vector<pcl::Vertices> outputPolygons;
				rtabmap::util3d::filterNaNPointsFromMesh(*iter->second.cloud, iter->second.polygons, *outputCloud, outputPolygons);

				if(iter->second.gain != 1.0f)
				{
					for(unsigned int i=0; i<outputCloud->size(); ++i)
					{
						pcl::PointXYZRGB & pt = outputCloud->at(i);
						pt.r = uchar(std::max(0.0, std::min(255.0, double(pt.r) * iter->second.gain)));
						pt.g = uchar(std::max(0.0, std::min(255.0, double(pt.g) * iter->second.gain)));
						pt.b = uchar(std::max(0.0, std::min(255.0, double(pt.b) * iter->second.gain)));
					}
				}

				pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformedCloud = rtabmap::util3d::transformPointCloud(outputCloud, iter->second.pose);
				if(mergedClouds->size() == 0)
				{
					*mergedClouds = *transformedCloud;
					mergedPolygons = outputPolygons;
				}
				else
				{
					rtabmap::util3d::appendMesh(*mergedClouds, mergedPolygons, *transformedCloud, outputPolygons);
				}
			}
		}

		if(mergedClouds->size())
		{
			pcl::PolygonMesh mesh;
			pcl::toPCLPointCloud2(*mergedClouds, mesh.cloud);
			mesh.polygons = mergedPolygons;

			UINFO("Saving ply (%d vertices, %d polygons) to %s.", (int)mergedClouds->size(), (int)mergedPolygons.size(), filePath.c_str());
			success = pcl::io::savePLYFileBinary(filePath, mesh) == 0;
			if(success)
			{
				UINFO("Saved ply to %s!", filePath.c_str());
			}
			else
			{
				UERROR("Failed saving ply to %s!", filePath.c_str());
			}
		}
	}
	return success;
}

int RTABMapApp::postProcessing(int approach)
{
	LOGI("postProcessing(%d)", approach);
	int returnedValue = 0;
	if(rtabmap_)
	{
		std::map<int, rtabmap::Transform> poses;
		std::multimap<int, rtabmap::Link> links;

		// detect more loop closures
		if(approach == -1 || approach == 2)
		{
			// detect more loop closures
			returnedValue = rtabmap_->detectMoreLoopClosures(1.0f, M_PI/6.0f, approach == -1?5:1);
		}

		// ICP refining
		if(returnedValue >=0 && approach == 3)
		{
			rtabmap::ParametersMap parameters;
			parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kRegStrategy(), std::string("1"))); // ICP
			rtabmap_->parseParameters(parameters);
			int r = rtabmap_->refineLinks();
			if(approach == 3 )
			{
				returnedValue = r;
			}
			// reset back default registration (visual)
			uInsert(parameters, rtabmap::ParametersPair(rtabmap::Parameters::kRegStrategy(), std::string(driftCorrection_?"1":"0"))); // Visual
			rtabmap_->parseParameters(parameters);
		}

		// graph optimization
		if(returnedValue >=0)
		{
			if (approach == 1)
			{
				if(rtabmap::Optimizer::isAvailable(rtabmap::Optimizer::kTypeG2O))
				{
					std::map<int, rtabmap::Signature> signatures;
					rtabmap_->getGraph(poses, links, false, true, &signatures);

					rtabmap::ParametersMap param;
					param.insert(rtabmap::ParametersPair(rtabmap::Parameters::kOptimizerIterations(), "30"));
					param.insert(rtabmap::ParametersPair(rtabmap::Parameters::kOptimizerEpsilon(), "0"));
					rtabmap::Optimizer * sba = rtabmap::Optimizer::create(rtabmap::Optimizer::kTypeG2O, param);
					poses = sba->optimizeBA(poses.rbegin()->first, poses, links, signatures);
					delete sba;
				}
				else
				{
					LOGE("g2o not available!");
				}
			}
			else if(approach!=4 && approach!=5 && approach != 7)
			{
				// simple graph optmimization
				rtabmap_->getGraph(poses, links, true, true);
			}
		}

		if(poses.size())
		{
			boost::mutex::scoped_lock  lock(rtabmapMutex_);
			rtabmap::Statistics stats = rtabmap_->getStatistics();
			stats.setPoses(poses);
			stats.setConstraints(links);
			rtabmapEvents_.push_back(stats);

			rtabmap_->setOptimizedPoses(poses);
		}
		else if(approach!=4 && approach!=5 && approach != 7)
		{
			returnedValue = -1;
		}

		if(returnedValue >=0)
		{
			boost::mutex::scoped_lock  lock(renderingMutex_);
			// filter polygons
			if(approach == 4)
			{
				filterPolygonsOnNextRender_ = true;
			}

			// gain compensation
			if(approach == -1 || approach == 5 || approach == 6)
			{
				gainCompensationOnNextRender_ = approach == 6 ? 2 : 1; // 2 = full, 1 = fast
			}

			// bilateral filtering
			if(approach == -1 || approach == 7)
			{
				bilateralFilteringOnNextRender_ = true;
			}
		}
	}

	return returnedValue;
}

void RTABMapApp::handleEvent(UEvent * event)
{
	if(camera_ && camera_->isRunning())
	{
		// called from events manager thread, so protect the data
		if(event->getClassName().compare("OdometryEvent") == 0)
		{
			LOGI("Received OdometryEvent!");
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
			LOGI("Received RtabmapEvent!");
			if(camera_->isRunning())
			{
				boost::mutex::scoped_lock lock(rtabmapMutex_);
				rtabmapEvents_.push_back(((rtabmap::RtabmapEvent*)event)->getStats());
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
		LOGI("Received RtabmapEventInit!");
		status_.first = ((rtabmap::RtabmapEventInit*)event)->getStatus();
		status_.second = ((rtabmap::RtabmapEventInit*)event)->getInfo();

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

	if(event->getClassName().compare("PostRenderEvent") == 0)
	{
		LOGI("Received PostRenderEvent!");
		const rtabmap::Statistics & stats = ((PostRenderEvent*)event)->getStats();
		int nodes = (int)uValue(stats.data(), rtabmap::Statistics::kMemoryWorking_memory_size(), 0.0f) +
				uValue(stats.data(), rtabmap::Statistics::kMemoryShort_time_memory_size(), 0.0f);
		int words = (int)uValue(stats.data(), rtabmap::Statistics::kKeypointDictionary_size(), 0.0f);
		float updateTime = uValue(stats.data(), rtabmap::Statistics::kTimingTotal(), 0.0f);
		int loopClosureId = stats.loopClosureId()>0?stats.loopClosureId():stats.proximityDetectionId()>0?stats.proximityDetectionId():0;
		int highestHypId = (int)uValue(stats.data(), rtabmap::Statistics::kLoopHighest_hypothesis_id(), 0.0f);
		int databaseMemoryUsed = (int)uValue(stats.data(), rtabmap::Statistics::kMemoryDatabase_memory_used(), 0.0f);
		int inliers = (int)uValue(stats.data(), rtabmap::Statistics::kLoopVisual_inliers(), 0.0f);
		int rejected = (int)uValue(stats.data(), rtabmap::Statistics::kLoopRejectedHypothesis(), 0.0f);
		int featuresExtracted = stats.getSignatures().size()?stats.getSignatures().rbegin()->second.getWords().size():0;
		float hypothesis = uValue(stats.data(), rtabmap::Statistics::kLoopHighest_hypothesis_value(), 0.0f);

		// Call JAVA callback with some stats
		UINFO("Send statistics to GUI");
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
					jmethodID methodID = env->GetMethodID(clazz, "updateStatsCallback", "(IIIIFIIIIIFIFI)V" );
					if(methodID)
					{
						env->CallVoidMethod(RTABMapActivity, methodID,
								nodes,
								words,
								totalPoints_,
								totalPolygons_,
								updateTime,
								loopClosureId,
								highestHypId,
								databaseMemoryUsed,
								inliers,
								featuresExtracted,
								hypothesis,
								lastDrawnCloudsCount_,
								renderingTime_>0.0f?1.0f/renderingTime_:0.0f,
								rejected);
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
		renderingTime_ = 0.0f;
	}
}

