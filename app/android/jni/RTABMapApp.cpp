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
#include <rtabmap/core/util3d_surface.h>
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
#include <pcl/surface/poisson.h>
#include <pcl/surface/vtk_smoothing/vtk_mesh_quadric_decimation.h>


const int g_exportedMeshId = -100;

static JavaVM *jvm;
static jobject RTABMapActivity = 0;

namespace {
constexpr int kTangoCoreMinimumVersion = 9377;
}  // anonymous namespace.

rtabmap::ParametersMap RTABMapApp::getRtabmapParameters()
{
	rtabmap::ParametersMap parameters;

	parameters.insert(mappingParameters_.begin(), mappingParameters_.end());

	parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kKpMaxFeatures(), std::string("200")));
	parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kGFTTQualityLevel(), std::string("0.0001")));
	parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kMemImagePreDecimation(), std::string(fullResolution_?"2":"1")));
	parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kBRIEFBytes(), std::string("64")));
	parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kRtabmapTimeThr(), std::string("800")));
	parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kRtabmapPublishLikelihood(), std::string("false")));
	parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kRtabmapPublishPdf(), std::string("false")));
	parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kRtabmapStartNewMapOnLoopClosure(), uBool2Str(appendMode_)));
	parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kMemBinDataKept(), uBool2Str(!trajectoryMode_)));
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

	if(parameters.find(rtabmap::Parameters::kOptimizerStrategy()) != parameters.end())
	{
		if(parameters.at(rtabmap::Parameters::kOptimizerStrategy()).compare("2") == 0) // GTSAM
		{
			parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kOptimizerEpsilon(), "0.00001"));
			parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kOptimizerIterations(), graphOptimization_?"10":"0"));
		}
		else if(parameters.at(rtabmap::Parameters::kOptimizerStrategy()).compare("1") == 0) // g2o
		{
			parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kOptimizerEpsilon(), "0.0"));
			parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kOptimizerIterations(), graphOptimization_?"10":"0"));
		}
		else // TORO
		{
			parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kOptimizerEpsilon(), "0.00001"));
			parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kOptimizerIterations(), graphOptimization_?"100":"0"));
		}
	}

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
		uInsert(parameters, rtabmap::ParametersPair(rtabmap::Parameters::kMemNotLinkedNodesKept(), std::string("true")));
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
		localizationMode_(false),
		trajectoryMode_(false),
		autoExposure_(true),
		rawScanSaved_(false),
		fullResolution_(false),
		appendMode_(true),
		maxCloudDepth_(0.0),
		meshDecimation_(1),
		meshTrianglePix_(1),
		meshAngleToleranceDeg_(15.0),
		minClusterSize_(200),
		maxGainRadius_(0.02f),
		paused_(false),
		dataRecorderMode_(false),
		clearSceneOnNextRender_(false),
		optimizeOpenedDatabase_(true),
		filterPolygonsOnNextRender_(false),
		gainCompensationOnNextRender_(0),
		bilateralFilteringOnNextRender_(false),
		cameraJustInitialized_(false),
		totalPoints_(0),
		totalPolygons_(0),
		lastDrawnCloudsCount_(0),
		renderingTime_(0.0f),
		visualizingMesh_(false),
		exportedMeshUpdated_(false),
		exportedMesh_(new pcl::TextureMesh),
		mapToOdom_(rtabmap::Transform::getIdentity())

{
	mappingParameters_.insert(rtabmap::ParametersPair(rtabmap::Parameters::kKpDetectorStrategy(), "5")); // GFTT/FREAK
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
	  camera_ = 0;
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

	camera_ = new rtabmap::CameraTango(fullResolution_?1:2, autoExposure_, rawScanSaved_);
}

void RTABMapApp::setScreenRotation(int displayRotation, int cameraRotation)
{
	TangoSupportRotation rotation = tango_gl::util::GetAndroidRotationFromColorCameraToDisplay(
			displayRotation, cameraRotation);
	LOGI("Set orientation: display=%d camera=%d -> %d", displayRotation, cameraRotation, (int)rotation);
	main_scene_.setScreenRotation(rotation);
	camera_->setScreenRotation(rotation);
}

int RTABMapApp::openDatabase(const std::string & databasePath, bool databaseInMemory, bool optimize)
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
	mapToOdom_.setIdentity();
	rtabmap_ = new rtabmap::Rtabmap();
	rtabmap::ParametersMap parameters = getRtabmapParameters();

	parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kDbSqlite3InMemory(), uBool2Str(databaseInMemory)));
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

	int status = 0;
	if(signatures.size() && poses.empty())
	{
		LOGE("Failed to optimize the graph!");
		status = -1;
	}

	optimizeOpenedDatabase_ = optimize;
	clearSceneOnNextRender_ = true;
	rtabmap::Statistics stats;
	stats.setSignatures(signatures);
	stats.addStatistic(rtabmap::Statistics::kMemoryWorking_memory_size(), (float)rtabmap_->getWMSize());
	stats.addStatistic(rtabmap::Statistics::kKeypointDictionary_size(), (float)rtabmap_->getMemory()->getVWDictionary()->getVisualWords().size());
	stats.addStatistic(rtabmap::Statistics::kMemoryDatabase_memory_used(), (float)rtabmap_->getMemory()->getDatabaseMemoryUsed());
	stats.setPoses(poses);
	stats.setConstraints(links);

	LOGI("Open: add rtabmap event with loaded data");
	rtabmapEvents_.push_back(stats);

	// Start threads
	LOGI("Start rtabmap thread");
	this->registerToEventsManager();
	rtabmapThread_->registerToEventsManager();
	rtabmapThread_->start();

	status_.first = rtabmap::RtabmapEventInit::kInitialized;
	status_.second = "";
	rtabmapMutex_.unlock();

	return status;
}

bool RTABMapApp::onTangoServiceConnected(JNIEnv* env, jobject iBinder)
{
	LOGW("onTangoServiceConnected()");
	if(camera_)
	{
		camera_->join(true);

		if (TangoService_setBinder(env, iBinder) != TANGO_SUCCESS) {
		    UERROR("TangoHandler::ConnectTango, TangoService_setBinder error");
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
		UERROR("Failed camera initialization!");
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
		UERROR("smoothMesh() Failed to smooth surface %d", id);
		return false;
	}
	return true;
}

// OpenGL thread
int RTABMapApp::Render()
{
	try
	{
		UASSERT(camera_!=0 && rtabmap_!=0);

		UTimer fpsTime;
		boost::mutex::scoped_lock  lock(renderingMutex_);

		if(clearSceneOnNextRender_)
		{
			visualizingMesh_ = false;
		}

		bool notifyCameraStarted = false;

		// process only pose events in vsualization mode
		rtabmap::Transform pose;
		{
			boost::mutex::scoped_lock lock(poseMutex_);
			if(poseEvents_.size())
			{
				pose = poseEvents_.back();
				poseEvents_.clear();
			}
		}
		rtabmap::Transform mapOdom = rtabmap::Transform::getIdentity();
		if(!pose.isNull())
		{
			// update camera pose?
			if(graphOptimization_ && !visualizingMesh_ && !mapToOdom_.isIdentity())
			{
				mapOdom = mapToOdom_;
				main_scene_.SetCameraPose(opengl_world_T_rtabmap_world*mapOdom*rtabmap_world_T_tango_world*pose);
			}
			else
			{
				main_scene_.SetCameraPose(opengl_world_T_tango_world*pose);
			}
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

		if(visualizingMesh_)
		{
			if(exportedMeshUpdated_)
			{
				main_scene_.clear();
				exportedMeshUpdated_ = false;
			}
			if(!main_scene_.hasCloud(g_exportedMeshId))
			{
				if(exportedMesh_->tex_polygons.size() && exportedMesh_->tex_polygons[0].size())
				{
					Mesh mesh;
					mesh.gain = 1.0f;
					mesh.cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
					mesh.normals.reset(new pcl::PointCloud<pcl::Normal>);
					pcl::fromPCLPointCloud2(exportedMesh_->cloud, *mesh.cloud);
					pcl::fromPCLPointCloud2(exportedMesh_->cloud, *mesh.normals);
					mesh.polygons = exportedMesh_->tex_polygons[0];
					cv::Mat texture;
					if(exportedMesh_->tex_coordinates.size())
					{
						mesh.texCoords = exportedMesh_->tex_coordinates[0];
						texture = exportedTexture_;
					}

					main_scene_.addMesh(g_exportedMeshId, mesh, texture, opengl_world_T_rtabmap_world);
				}
				else
				{
					pcl::IndicesPtr indices(new std::vector<int>); // null
					pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
					pcl::fromPCLPointCloud2(exportedMesh_->cloud, *cloud);
					main_scene_.addCloud(g_exportedMeshId, cloud, indices, opengl_world_T_rtabmap_world);
				}
			}

			//backup state
			bool isMeshRendering = main_scene_.isMeshRendering();
			bool isTextureRendering = main_scene_.isMeshTexturing();
			bool isFrustumCulling = main_scene_.isFrustumCulling();

			main_scene_.setMeshRendering(main_scene_.hasMesh(g_exportedMeshId), main_scene_.hasTexture(g_exportedMeshId));
			main_scene_.setFrustumCulling(false);

			main_scene_.Render();

			// revert state
			main_scene_.setMeshRendering(isMeshRendering, isTextureRendering);
			main_scene_.setFrustumCulling(isFrustumCulling);

			if(renderingTime_ < fpsTime.elapsed())
			{
				renderingTime_ = fpsTime.elapsed();
			}

			return notifyCameraStarted;
		}
		else
		{
			if(main_scene_.hasCloud(g_exportedMeshId))
			{
				main_scene_.clear();
				exportedMesh_.reset(new pcl::TextureMesh);
				exportedTexture_ = cv::Mat();
			}

			bool notifyDataLoaded = false;
			// should be before clearSceneOnNextRender_ in case openDatabase is called
			std::list<rtabmap::Statistics> rtabmapEvents;
			{
				boost::mutex::scoped_lock  lock(rtabmapMutex_);
				rtabmapEvents = rtabmapEvents_;
				rtabmapEvents_.clear();

				boost::mutex::scoped_lock  lockMesh(meshesMutex_);
				if(!clearSceneOnNextRender_ && rtabmapEvents.size() && createdMeshes_.size())
				{
					if(rtabmapEvents.front().refImageId()>0 && rtabmapEvents.front().refImageId() < createdMeshes_.rbegin()->first)
					{
						LOGI("Detected new database! new=%d old=%d", rtabmapEvents.front().refImageId(), createdMeshes_.rbegin()->first);
						clearSceneOnNextRender_ = true;
					}
				}
			}

			if(clearSceneOnNextRender_)
			{
				boost::mutex::scoped_lock  lock(meshesMutex_);

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
			{
				boost::mutex::scoped_lock  lock(meshesMutex_);
				if(added.size() != createdMeshes_.size())
				{
					for(std::map<int, Mesh>::iterator iter=createdMeshes_.begin(); iter!=createdMeshes_.end(); ++iter)
					{
						if(!main_scene_.hasCloud(iter->first))
						{
							if(main_scene_.isMeshRendering() && iter->second.polygons.size() == 0)
							{
								iter->second.polygons = rtabmap::util3d::organizedFastMesh(iter->second.cloud, meshAngleToleranceDeg_*M_PI/180.0, false, meshTrianglePix_);
							}
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
						// Don't create mesh for the last node added if rehearsal happened or if discarded (small movement)
						int smallMovement = (int)uValue(iter->data(), rtabmap::Statistics::kMemorySmall_movement(), 0.0f);
						int rehearsalMerged = (int)uValue(iter->data(), rtabmap::Statistics::kMemoryRehearsal_merged(), 0.0f);
						if(smallMovement == 0 && rehearsalMerged == 0)
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
									uInsert(bufferedSensorData, std::make_pair(jter->first, jter->second.sensorData()));
									uInsert(rawPoses_, std::make_pair(jter->first, jter->second.getPose()));
									if(notifyDataLoaded == false)
									{
										LOGI("Detecting that we are loading a database");
									}
									notifyDataLoaded = true;
								}
							}
						}

						int loopClosure = (int)uValue(iter->data(), rtabmap::Statistics::kLoopAccepted_hypothesis_id(), 0.0f);
						int rejected = (int)uValue(iter->data(), rtabmap::Statistics::kLoopRejectedHypothesis(), 0.0f);
						if(!paused_ && loopClosure>0)
						{
							main_scene_.setBackgroundColor(0, 0.7f, 0); // green
						}
						else if(!paused_ && rejected>0)
						{
							main_scene_.setBackgroundColor(0, 0.1f, 0); // dark green
						}
						else if(!paused_ && rehearsalMerged>0)
						{
							main_scene_.setBackgroundColor(0, 0, 0.2f); // blue
						}
						else
						{
							main_scene_.setBackgroundColor(0, 0, 0);
						}
					}
				}

				std::map<int, rtabmap::Transform> poses = rtabmapEvents.back().poses();
				if(!rtabmapEvents.back().mapCorrection().isNull())
				{
					mapToOdom_ = rtabmapEvents.back().mapCorrection();
				}

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
								rtabmap::SensorData data = bufferedSensorData.at(id);

								cv::Mat tmpA, depth;
								data.uncompressData(&tmpA, &depth);

								if(notifyDataLoaded && optimizeOpenedDatabase_)
								{
									// do post-processing bilateral filtering now
									UTimer t;
									depth = rtabmap::util2d::fastBilateralFiltering(depth, 2.0f, 0.075f);
									data.setDepthOrRightRaw(depth);
									LOGI("Bilateral filtering of %d, time=%fs", id, t.ticks());
								}

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
											UERROR("No mesh could be created for node %d", id);
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
					boost::mutex::scoped_lock  lock(meshesMutex_);
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
								main_scene_.addCloud(-1, cloud, indices, opengl_world_T_rtabmap_world*mapOdom*odomEvent.pose());
								main_scene_.setCloudVisible(-1, true);
							}
							else
							{
								UERROR("Generated cloud is empty!");
							}
						}
						else
						{
							UERROR("Odom data images are empty!");
						}
					}
				}
			}

			if((notifyDataLoaded&&optimizeOpenedDatabase_) || gainCompensationOnNextRender_>0)
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

				UASSERT(maxGainRadius_>0.0f);
				rtabmap::GainCompensator compensator(maxGainRadius_);
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
							LOGI("%d mesh has gain %f", iter->first, iter->second.gain);
						}
					}

					main_scene_.updateGain(iter->first, iter->second.gain);
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

			if(filterPolygonsOnNextRender_ && minClusterSize_>0)
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
								minClusterSize_);
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
	}
	catch(const std::exception & e)
	{
		UERROR("Exception! msg=\"%s\"", e.what());
		return -1;
	}
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
	{
		boost::mutex::scoped_lock  lock(renderingMutex_);
		visualizingMesh_ = false;
		main_scene_.setBackgroundColor(0, 0, 0);
	}
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
void RTABMapApp::setPointSize(float value)
{
	main_scene_.setPointSize(value);
}
void RTABMapApp::setLighting(bool enabled)
{
	main_scene_.setLighting(enabled);
}
void RTABMapApp::setBackfaceCulling(bool enabled)
{
	main_scene_.setBackfaceCulling(enabled);
}

void RTABMapApp::setLocalizationMode(bool enabled)
{
	localizationMode_ = enabled;
	this->post(new rtabmap::ParamEvent(rtabmap::Parameters::kMemIncrementalMemory(), uBool2Str(!localizationMode_)));
}
void RTABMapApp::setTrajectoryMode(bool enabled)
{
	trajectoryMode_ = enabled;
	this->post(new rtabmap::ParamEvent(rtabmap::Parameters::kMemBinDataKept(), uBool2Str(!trajectoryMode_)));
}

void RTABMapApp::setGraphOptimization(bool enabled)
{
	graphOptimization_ = enabled;
	UASSERT(camera_ != 0 && rtabmap_!=0 && rtabmap_->getMemory()!=0);
	if(!camera_->isRunning() && rtabmap_->getMemory()->getLastWorkingSignature()!=0)
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

			LOGI("Send rtabmap event to update graph...");
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

void RTABMapApp::setRawScanSaved(bool enabled)
{
	if(rawScanSaved_ != enabled)
	{
		rawScanSaved_ = enabled;
		if(camera_)
		{
			camera_->setRawScanPublished(rawScanSaved_);
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
	LOGW("Set mesh decimation to level %d", value);
	meshDecimation_ = 1;
	if(camera_)
	{
		// Google Tango Tablet 160x90
		// Phab2Pro 240x135
		int width = camera_->getCameraModel().imageWidth()/8;
		int height = camera_->getCameraModel().imageHeight()/8;
		if(value == 3) // high
		{
			if(width % 10 == 0 && height % 10 == 0)
			{
				meshDecimation_ = 10;
			}
			else if(width % 15 == 0 && height % 15 == 0)
			{
				meshDecimation_ = 15;
			}
			else
			{
				UERROR("Could not set decimation to high (size=%dx%d)", width, height);
			}
		}
		else if(value == 2) // medium
		{
			if(width % 5 == 0 && height % 5 == 0)
			{
				meshDecimation_ = 5;
			}
			else
			{
				UERROR("Could not set decimation to medium (size=%dx%d)", width, height);
			}
		}
		else if(value == 1) // low
		{
			if(width % 3 == 0 && width % 3 == 0)
			{
				meshDecimation_ = 3;
			}
			else if(width % 2 == 0 && width % 2 == 0)
			{
				meshDecimation_ = 2;
			}
			else
			{
				UERROR("Could not set decimation to low (size=%dx%d)", width, height);
			}
		}
	}
	UINFO("Set decimation to %d", meshDecimation_);
}

void RTABMapApp::setMeshAngleTolerance(float value)
{
	meshAngleToleranceDeg_ = value;
}

void RTABMapApp::setMeshTriangleSize(int value)
{
	meshTrianglePix_ = value;
}

void RTABMapApp::setMinClusterSize(int value)
{
	minClusterSize_ = value;
}

void RTABMapApp::setMaxGainRadius(float value)
{
	maxGainRadius_ = value;
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
				UERROR("Parameter \"%s\" doesn't exist anymore!",
						iter->first.c_str());
			}
			else
			{
				UERROR("Parameter \"%s\" doesn't exist anymore! You may look at this similar parameter: \"%s\"",
						iter->first.c_str(), iter->second.second.c_str());
			}
		}
	}

	if(rtabmap::Parameters::getDefaultParameters().find(compatibleKey) != rtabmap::Parameters::getDefaultParameters().end())
	{
		LOGI(uFormat("Setting param \"%s\"  to \"%s\"", compatibleKey.c_str(), value.c_str()).c_str());
		if(compatibleKey.compare(rtabmap::Parameters::kKpDetectorStrategy()) == 0 &&
		   mappingParameters_.at(rtabmap::Parameters::kKpDetectorStrategy()).compare(value) != 0)
		{
			// Changing feature type should reset mapping!
			resetMapping();
		}
		uInsert(mappingParameters_, rtabmap::ParametersPair(compatibleKey, value));
		UEventsManager::post(new rtabmap::ParamEvent(mappingParameters_));
		return 0;
	}
	else
	{
		UERROR(uFormat("Key \"%s\" doesn't exist!", compatibleKey.c_str()).c_str());
		return -1;
	}
}

void RTABMapApp::resetMapping()
{
	LOGW("Reset!");
	status_.first = rtabmap::RtabmapEventInit::kInitializing;
	status_.second = "";

	mapToOdom_.setIdentity();
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

cv::Mat RTABMapApp::mergeTextures(pcl::TextureMesh & mesh, int textureSize) const
{
	UASSERT(textureSize> 0);
	LOGD("textureSize = %d materials=%d", textureSize, mesh.tex_materials.size());
	cv::Mat globalTexture;
	if(mesh.tex_materials.size() >= 1)
	{
		std::vector<int> textures(mesh.tex_materials.size(), -1);
		cv::Size imageSize;
		int imageType=CV_8UC3;
		for(unsigned int i=0; i<mesh.tex_materials.size(); ++i)
		{
			if(!mesh.tex_materials[i].tex_file.empty() &&
				mesh.tex_polygons[i].size() &&
			   uIsInteger(mesh.tex_materials[i].tex_file, false))
			{
				int textureId = uStr2Int(mesh.tex_materials[i].tex_file);
				textures[i] = textureId;

				if(imageSize.height == 0)
				{
					rtabmap::SensorData data = rtabmap_->getMemory()->getNodeData(textureId);

					UASSERT(!data.imageCompressed().empty() &&
						data.cameraModels().size()==1 &&
						data.cameraModels()[0].imageHeight()>0);
					imageSize = data.cameraModels()[0].imageSize();
				}
			}
			else
			{
				textures[i] = -1;
			}
		}
		if(textures.size() && imageSize.height>0 && imageSize.width>0)
		{
			float scale = 0.0f;
			std::vector<bool> materialsKept;
			rtabmap::util3d::concatenateTextureMaterials(mesh, imageSize, textureSize, scale, &materialsKept);
			LOGD("scale=%f materials=%d", scale, (int)mesh.tex_materials.size());
			if(scale && mesh.tex_materials.size()==1)
			{
				int cols = float(textureSize)/(scale*imageSize.width);

				globalTexture = cv::Mat(textureSize, textureSize, imageType, cv::Scalar::all(255));

				// make a blank texture
				cv::Mat emptyImage(int(imageSize.height*scale), int(imageSize.width*scale), imageType, cv::Scalar::all(255));
				int oi=0;
				for(int i=0; i<(int)textures.size(); ++i)
				{
					if(materialsKept.at(i))
					{
						int u = oi%cols * emptyImage.cols;
						int v = oi/cols * emptyImage.rows;
						UASSERT(u < textureSize-emptyImage.cols);
						UASSERT(v < textureSize-emptyImage.rows);
						if(textures[i]>=0)
						{
							rtabmap::SensorData data = rtabmap_->getMemory()->getNodeData(textures[i]);
							UASSERT_MSG(!data.imageCompressed().empty(), uFormat("id=%d", textures[i]).c_str());
							cv::Mat image;
							data.uncompressDataConst(&image, 0);
							UASSERT(!image.empty());
							cv::Mat resizedImage;
							cv::resize(image, resizedImage, emptyImage.size(), 0.0f, 0.0f, cv::INTER_AREA);
							if(createdMeshes_.find(textures[i]) != createdMeshes_.end() && createdMeshes_.at(textures[i]).gain != 1.0f)
							{
								cv::multiply(resizedImage, createdMeshes_.at(textures[i]).gain, resizedImage);
							}
							UASSERT(resizedImage.type() == globalTexture.type());
							resizedImage.copyTo(globalTexture(cv::Rect(u, v, resizedImage.cols, resizedImage.rows)));
						}
						else
						{
							emptyImage.copyTo(globalTexture(cv::Rect(u, v, emptyImage.cols, emptyImage.rows)));
						}
						++oi;
					}
				}
			}
			else
			{
				UERROR("Failed merging textures");
			}
		}
		else if(textures.size() == 0)
		{
			UERROR("No textures kept!");
		}
		else
		{
			UERROR("No image size set!");
		}
	}
	return globalTexture;
}

bool RTABMapApp::exportMesh(
		const std::string & filePath,
		float cloudVoxelSize,
		bool meshing,
		int textureSize,
		int normalK,
		float maxTextureDistance,
		bool optimized,
		float optimizedVoxelSize,
		int optimizedDepth,
		float optimizedDecimationFactor,
		float optimizedColorRadius,
		bool optimizedCleanWhitePolygons,
		bool optimizedColorWhitePolygons, // not yet used
		bool blockRendering)
{
	// make sure createdMeshes_ is not modified while exporting! We don't
	// lock the meshesMutex_ because we want to continue rendering.

	if(blockRendering)
	{
		renderingMutex_.lock();
		main_scene_.clear();
	}

	bool success = false;

	try
	{
		std::map<int, rtabmap::Transform> poses;
		std::multimap<int, rtabmap::Link> links;
		rtabmap_->getGraph(poses, links, true, true);

		//Assemble the meshes
		if(meshing) // Mesh or  Texture Mesh
		{
			pcl::PolygonMesh::Ptr polygonMesh(new pcl::PolygonMesh);
			pcl::TextureMesh::Ptr textureMesh(new pcl::TextureMesh);
			cv::Mat globalTexture;
			int totalPolygons = 0;
			{
				if(optimized)
				{
					std::map<int, rtabmap::Transform> cameraPoses;
					std::map<int, rtabmap::CameraModel> cameraModels;

					UTimer timer;
					LOGI("Assemble clouds (%d)...", (int)poses.size());
					int cloudCount=0;
					pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr mergedClouds(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
					for(std::map<int, rtabmap::Transform>::iterator iter=poses.begin();
						iter!= poses.end();
						++iter)
					{
						std::map<int, Mesh>::iterator jter = createdMeshes_.find(iter->first);
						pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
						pcl::IndicesPtr indices(new std::vector<int>);
						rtabmap::CameraModel model;
						float gain = 1.0f;
						if(jter != createdMeshes_.end())
						{
							cloud = jter->second.cloud;
							indices = jter->second.indices;
							model = jter->second.cameraModel;
							gain = jter->second.gain;
						}
						else
						{
							rtabmap::SensorData data = rtabmap_->getMemory()->getNodeData(iter->first, true);
							if(!data.imageRaw().empty() && !data.depthRaw().empty() && data.cameraModels().size() == 1)
							{
								cloud = rtabmap::util3d::cloudRGBFromSensorData(data, meshDecimation_, maxCloudDepth_, 0, indices.get());
								model = data.cameraModels()[0];
							}
						}
						if(cloud->size() && indices->size() && model.isValidForProjection())
						{
							pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformedCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
							if(optimizedVoxelSize > 0.0f)
							{
								transformedCloud = rtabmap::util3d::voxelize(cloud, indices, optimizedVoxelSize);
								transformedCloud = rtabmap::util3d::transformPointCloud(transformedCloud, iter->second);
							}
							else
							{
								// it looks like that using only transformPointCloud with indices
								// flushes the colors, so we should extract points before... maybe a too old PCL version
								pcl::copyPointCloud(*cloud, *indices, *transformedCloud);
								transformedCloud = rtabmap::util3d::transformPointCloud(transformedCloud, iter->second);
							}

							Eigen::Vector3f viewpoint( iter->second.x(),  iter->second.y(),  iter->second.z());
							pcl::PointCloud<pcl::Normal>::Ptr normals = rtabmap::util3d::computeNormals(transformedCloud, normalK, viewpoint);

							pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudWithNormals(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
							pcl::concatenateFields(*transformedCloud, *normals, *cloudWithNormals);

							if(textureSize == 0 && gain != 1.0f)
							{
								for(unsigned int i=0; i<cloudWithNormals->size(); ++i)
								{
									pcl::PointXYZRGBNormal & pt = cloudWithNormals->at(i);
									pt.r = uchar(std::max(0.0, std::min(255.0, double(pt.r) * gain)));
									pt.g = uchar(std::max(0.0, std::min(255.0, double(pt.g) * gain)));
									pt.b = uchar(std::max(0.0, std::min(255.0, double(pt.b) * gain)));
								}
							}


							if(mergedClouds->size() == 0)
							{
								*mergedClouds = *cloudWithNormals;
							}
							else
							{
								*mergedClouds += *cloudWithNormals;
							}

							cameraPoses.insert(std::make_pair(iter->first, iter->second));
							cameraModels.insert(std::make_pair(iter->first, model));

							LOGI("Assembled %d points (%d/%d total=%d)", (int)cloudWithNormals->size(), ++cloudCount, (int)poses.size(), (int)mergedClouds->size());
						}
						else
						{
							UERROR("Cloud %d not found or empty", iter->first);
						}
					}
					LOGI("Assembled clouds (%d)... done! %fs (total points=%d)", (int)cameraPoses.size(), timer.ticks(), (int)mergedClouds->size());

					if(mergedClouds->size())
					{
						// Mesh reconstruction
						LOGI("Mesh reconstruction...");
						pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh);
						pcl::Poisson<pcl::PointXYZRGBNormal> poisson;
						poisson.setDepth(optimizedDepth);
						poisson.setInputCloud(mergedClouds);
						poisson.reconstruct(*mesh);
						LOGI("Mesh reconstruction... done! %fs (%d polygons)", timer.ticks(), mesh->polygons.size());

						if(mesh->polygons.size())
						{
							if(textureSize > 0 && optimizedDecimationFactor > 0.0f)
							{
	#ifndef DISABLE_VTK
								unsigned int count = mesh->polygons.size();
								LOGI("Mesh decimation (factor=%f) from %d polygons...", optimizedDecimationFactor, (int)count);

								pcl::PolygonMesh::Ptr output(new pcl::PolygonMesh);
								pcl::MeshQuadricDecimationVTK mqd;
								mqd.setTargetReductionFactor(optimizedDecimationFactor);
								mqd.setInputMesh(mesh);
								mqd.process (*output);
								mesh = output;

								//mesh = rtabmap::util3d::meshDecimation(mesh, decimationFactor);
								// use direct instantiation above to this fix some linker errors on android like:
								// pcl::MeshQuadricDecimationVTK::performProcessing(pcl::PolygonMesh&): error: undefined reference to 'vtkQuadricDecimation::New()'
								// pcl::VTKUtils::mesh2vtk(pcl::PolygonMesh const&, vtkSmartPointer<vtkPolyData>&): error: undefined reference to 'vtkFloatArray::New()'

								LOGI("Mesh decimated (factor=%f) from %d to %d polygons (%fs)", optimizedDecimationFactor, count, (int)mesh->polygons.size(), timer.ticks());
								if(count < mesh->polygons.size())
								{
									UWARN("Decimated mesh has more polygons than before!");
								}
	#else
								UWARN("RTAB-Map is not built with PCL-VTK module so mesh decimation cannot be used!");
	#endif
							}

							if(textureSize == 0)
							{
								// colored polygon mesh
								if(optimizedColorRadius >= 0.0f)
								{
									LOGI("Transferring color from point cloud to mesh...");
									// transfer color from point cloud to mesh
									pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBNormal>(true));
									tree->setInputCloud(mergedClouds);
									pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr coloredCloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
									pcl::fromPCLPointCloud2(mesh->cloud, *coloredCloud);
									std::vector<bool> coloredPts(coloredCloud->size());
									for(unsigned int i=0; i<coloredCloud->size(); ++i)
									{
										std::vector<int> kIndices;
										std::vector<float> kDistances;
										pcl::PointXYZRGBNormal pt;
										pt.x = coloredCloud->at(i).x;
										pt.y = coloredCloud->at(i).y;
										pt.z = coloredCloud->at(i).z;
										if(optimizedColorRadius > 0.0f)
										{
											tree->radiusSearch(pt, optimizedColorRadius, kIndices, kDistances);
										}
										else
										{
											tree->nearestKSearch(pt, 1, kIndices, kDistances);
										}
										if(kIndices.size())
										{
											coloredCloud->at(i).r = mergedClouds->at(kIndices[0]).r;
											coloredCloud->at(i).g = mergedClouds->at(kIndices[0]).g;
											coloredCloud->at(i).b = mergedClouds->at(kIndices[0]).b;
											coloredCloud->at(i).a = mergedClouds->at(kIndices[0]).a;
											coloredPts.at(i) = true;
										}
										else
										{
											//white
											coloredCloud->at(i).r = coloredCloud->at(i).g = coloredCloud->at(i).b = 255;
											coloredPts.at(i) = false;
										}
									}

									// recompute normals and remove polygons with no color
									std::vector<pcl::Vertices> filteredPolygons(optimizedCleanWhitePolygons?mesh->polygons.size():0);
									int oi=0;
									for(unsigned int i=0; i<mesh->polygons.size(); ++i)
									{
										// recompute normals
										pcl::Vertices & v = mesh->polygons[i];
										UASSERT(v.vertices.size()>2);
										Eigen::Vector3f v0(
												coloredCloud->at(v.vertices[1]).x - coloredCloud->at(v.vertices[0]).x,
												coloredCloud->at(v.vertices[1]).y - coloredCloud->at(v.vertices[0]).y,
												coloredCloud->at(v.vertices[1]).z - coloredCloud->at(v.vertices[0]).z);
										int last = v.vertices.size()-1;
										Eigen::Vector3f v1(
												coloredCloud->at(v.vertices[last]).x - coloredCloud->at(v.vertices[0]).x,
												coloredCloud->at(v.vertices[last]).y - coloredCloud->at(v.vertices[0]).y,
												coloredCloud->at(v.vertices[last]).z - coloredCloud->at(v.vertices[0]).z);
										Eigen::Vector3f normal = v0.cross(v1);
										normal.normalize();
										// flat normal (per face)
										for(unsigned int j=0; j<v.vertices.size(); ++j)
										{
											coloredCloud->at(v.vertices[j]).normal_x = normal[0];
											coloredCloud->at(v.vertices[j]).normal_y = normal[1];
											coloredCloud->at(v.vertices[j]).normal_z = normal[2];
										}

										if(optimizedCleanWhitePolygons)
										{
											bool coloredPolygon = true;
											for(unsigned int j=0; j<mesh->polygons[i].vertices.size(); ++j)
											{
												if(!coloredPts.at(mesh->polygons[i].vertices[j]))
												{
													coloredPolygon = false;
													break;
												}
											}
											if(coloredPolygon)
											{
												filteredPolygons[oi++] = mesh->polygons[i];
											}
										}
									}
									if(optimizedCleanWhitePolygons)
									{
										filteredPolygons.resize(oi);
										mesh->polygons = filteredPolygons;
									}

									pcl::toPCLPointCloud2(*coloredCloud, mesh->cloud);
									LOGI("Transfering color from point cloud to mesh...done! %fs", timer.ticks());
								}
								else // recompute normals
								{
									pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
									pcl::fromPCLPointCloud2(mesh->cloud, *cloud);

									for(unsigned int i=0; i<mesh->polygons.size(); ++i)
									{
										pcl::Vertices & v = mesh->polygons[i];
										UASSERT(v.vertices.size()>2);
										Eigen::Vector3f v0(
												cloud->at(v.vertices[1]).x - cloud->at(v.vertices[0]).x,
												cloud->at(v.vertices[1]).y - cloud->at(v.vertices[0]).y,
												cloud->at(v.vertices[1]).z - cloud->at(v.vertices[0]).z);
										int last = v.vertices.size()-1;
										Eigen::Vector3f v1(
												cloud->at(v.vertices[last]).x - cloud->at(v.vertices[0]).x,
												cloud->at(v.vertices[last]).y - cloud->at(v.vertices[0]).y,
												cloud->at(v.vertices[last]).z - cloud->at(v.vertices[0]).z);
										Eigen::Vector3f normal = v0.cross(v1);
										normal.normalize();
										// flat normal (per face)
										for(unsigned int j=0; j<v.vertices.size(); ++j)
										{
											cloud->at(v.vertices[j]).normal_x = normal[0];
											cloud->at(v.vertices[j]).normal_y = normal[1];
											cloud->at(v.vertices[j]).normal_z = normal[2];
										}
									}
									pcl::toPCLPointCloud2 (*cloud, mesh->cloud);
								}
								polygonMesh = mesh;
								totalPolygons = mesh->polygons.size();
							}
							else
							{
								LOGI("Texturing...");
								textureMesh = rtabmap::util3d::createTextureMesh(
										mesh,
										cameraPoses,
										cameraModels,
										maxTextureDistance);
								LOGI("Texturing... done! %fs", timer.ticks());

								// Remove occluded polygons (polygons with no texture)
								if(textureMesh->tex_coordinates.size() && optimizedCleanWhitePolygons)
								{
									LOGI("Cleanup mesh...");

									// assume last texture is the occluded texture
									textureMesh->tex_coordinates.pop_back();
									textureMesh->tex_polygons.pop_back();
									textureMesh->tex_materials.pop_back();

									if(minClusterSize_>0)
									{
										LOGI("Filter small polygon clusters...");

										// concatenate all polygons
										int totalSize = 0;
										for(unsigned int t=0; t<textureMesh->tex_polygons.size(); ++t)
										{
											totalSize+=textureMesh->tex_polygons[t].size();
										}
										std::vector<pcl::Vertices> allPolygons(totalSize);
										int oi=0;
										for(unsigned int t=0; t<textureMesh->tex_polygons.size(); ++t)
										{
											for(unsigned int i=0; i<textureMesh->tex_polygons[t].size(); ++i)
											{
												allPolygons[oi++] =  textureMesh->tex_polygons[t][i];
											}
										}

										// filter polygons
										std::vector<std::set<int> > neighbors;
										std::vector<std::set<int> > vertexToPolygons;
										rtabmap::util3d::createPolygonIndexes(allPolygons,
												textureMesh->cloud.data.size()/textureMesh->cloud.point_step,
												neighbors,
												vertexToPolygons);
										std::list<std::list<int> > clusters = rtabmap::util3d::clusterPolygons(
												neighbors,
												minClusterSize_);

										std::set<int> validPolygons;
										for(std::list<std::list<int> >::iterator kter=clusters.begin(); kter!=clusters.end(); ++kter)
										{
											for(std::list<int>::iterator jter=kter->begin(); jter!=kter->end(); ++jter)
											{
												validPolygons.insert(*jter);
											}
										}

										// for each texture
										unsigned int allPolygonsIndex = 0;
										for(unsigned int t=0; t<textureMesh->tex_polygons.size(); ++t)
										{
											std::vector<pcl::Vertices> filteredPolygons(textureMesh->tex_polygons[t].size());
	#if PCL_VERSION_COMPARE(>=, 1, 8, 0)
											std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> > filteredCoordinates(textureMesh->tex_coordinates[t].size());
	#else
											std::vector<Eigen::Vector2f> filteredCoordinates(textureMesh->tex_coordinates[t].size());
	#endif
											int oi=0;
											unsigned int polygonSize = 0;
											if(textureMesh->tex_polygons[t].size())
											{
												UASSERT(allPolygonsIndex < allPolygons.size());

												polygonSize = textureMesh->tex_polygons[t][0].vertices.size();

												UASSERT(filteredCoordinates.size() == textureMesh->tex_polygons[t].size()*polygonSize);
												for(unsigned int i=0; i<textureMesh->tex_polygons[t].size(); ++i)
												{
													if(validPolygons.find(allPolygonsIndex) != validPolygons.end())
													{
														filteredPolygons[oi] = textureMesh->tex_polygons[t].at(i);
														for(unsigned int j=0; j<polygonSize; ++j)
														{
															filteredCoordinates[oi*polygonSize + j] = textureMesh->tex_coordinates[t][i*polygonSize + j];
														}
														++oi;
													}
													++allPolygonsIndex;
												}
												filteredPolygons.resize(oi);
												filteredCoordinates.resize(oi*polygonSize);
												textureMesh->tex_polygons[t] = filteredPolygons;
												textureMesh->tex_coordinates[t] = filteredCoordinates;
											}
										}

										LOGI("Filtered %d polygons.", (int)(allPolygons.size()-validPolygons.size()));
									}

									for(unsigned int t=0; t<textureMesh->tex_polygons.size(); ++t)
									{
										totalPolygons+=textureMesh->tex_polygons[t].size();
									}

									LOGI("Cleanup mesh... done! %fs (total polygons=%d)", timer.ticks(), totalPolygons);
								}
								else
								{
									for(unsigned int t=0; t<textureMesh->tex_polygons.size(); ++t)
									{
										totalPolygons+=textureMesh->tex_polygons[t].size();
									}
								}
							}
						}
					}
				}
				else // organized meshes
				{
					pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr mergedClouds(new pcl::PointCloud<pcl::PointXYZRGBNormal>);

					if(textureSize > 0)
					{
						textureMesh->tex_materials.resize(poses.size());
						textureMesh->tex_polygons.resize(poses.size());
						textureMesh->tex_coordinates.resize(poses.size());
					}

					int polygonsStep = 0;
					int oi = 0;
					for(std::map<int, rtabmap::Transform>::iterator iter=poses.begin();
						iter!= poses.end();
						++iter)
					{
						LOGI("Assembling cloud %d (total=%d)...", iter->first, (int)poses.size());

						std::map<int, Mesh>::iterator jter = createdMeshes_.find(iter->first);
						pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
						std::vector<pcl::Vertices> polygons;
						float gain = 1.0f;
						if(jter != createdMeshes_.end())
						{
							cloud = jter->second.cloud;
							polygons= jter->second.polygons;
							if(cloud->size() && polygons.size() == 0)
							{
								polygons = rtabmap::util3d::organizedFastMesh(cloud, meshAngleToleranceDeg_*M_PI/180.0, false, meshTrianglePix_);
							}
							gain = jter->second.gain;
						}
						else
						{
							rtabmap::SensorData data = rtabmap_->getMemory()->getNodeData(iter->first, true);
							if(!data.imageRaw().empty() && !data.depthRaw().empty() && data.cameraModels().size() == 1)
							{
								cloud = rtabmap::util3d::cloudRGBFromSensorData(data, meshDecimation_, maxCloudDepth_, 0);
								polygons = rtabmap::util3d::organizedFastMesh(cloud, meshAngleToleranceDeg_*M_PI/180.0, false, meshTrianglePix_);
							}
						}

						if(cloud->size() && polygons.size())
						{
							// Convert organized to dense cloud
							pcl::PointCloud<pcl::PointXYZRGB>::Ptr outputCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
							std::vector<pcl::Vertices> outputPolygons;
							std::vector<int> denseToOrganizedIndices = rtabmap::util3d::filterNaNPointsFromMesh(*cloud, polygons, *outputCloud, outputPolygons);

							pcl::PointCloud<pcl::Normal>::Ptr normals = rtabmap::util3d::computeNormals(outputCloud, normalK);

							pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudWithNormals(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
							pcl::concatenateFields(*outputCloud, *normals, *cloudWithNormals);

							UASSERT(outputPolygons.size());

							totalPolygons+=outputPolygons.size();

							if(textureSize == 0)
							{
								// colored mesh
								cloudWithNormals = rtabmap::util3d::transformPointCloud(cloudWithNormals, iter->second);

								if(gain != 1.0f)
								{
									for(unsigned int i=0; i<cloudWithNormals->size(); ++i)
									{
										pcl::PointXYZRGBNormal & pt = cloudWithNormals->at(i);
										pt.r = uchar(std::max(0.0, std::min(255.0, double(pt.r) * gain)));
										pt.g = uchar(std::max(0.0, std::min(255.0, double(pt.g) * gain)));
										pt.b = uchar(std::max(0.0, std::min(255.0, double(pt.b) * gain)));
									}
								}

								if(mergedClouds->size() == 0)
								{
									*mergedClouds = *cloudWithNormals;
									polygonMesh->polygons = outputPolygons;
								}
								else
								{
									rtabmap::util3d::appendMesh(*mergedClouds, polygonMesh->polygons, *cloudWithNormals, outputPolygons);
								}
							}
							else
							{
								// texture mesh
								unsigned int polygonSize = outputPolygons.front().vertices.size();
								textureMesh->tex_polygons[oi].resize(outputPolygons.size());
								textureMesh->tex_coordinates[oi].resize(outputPolygons.size() * polygonSize);
								for(unsigned int j=0; j<outputPolygons.size(); ++j)
								{
									pcl::Vertices vertices = outputPolygons[j];
									UASSERT(polygonSize == vertices.vertices.size());
									for(unsigned int k=0; k<vertices.vertices.size(); ++k)
									{
										//uv
										UASSERT(vertices.vertices[k] < denseToOrganizedIndices.size());
										int originalVertex = denseToOrganizedIndices[vertices.vertices[k]];
										textureMesh->tex_coordinates[oi][j*vertices.vertices.size()+k] = Eigen::Vector2f(
												float(originalVertex % cloud->width) / float(cloud->width),   // u
												float(cloud->height - originalVertex / cloud->width) / float(cloud->height)); // v

										vertices.vertices[k] += polygonsStep;
									}
									textureMesh->tex_polygons[oi][j] = vertices;

								}
								polygonsStep += outputCloud->size();

								pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr transformedCloud = rtabmap::util3d::transformPointCloud(cloudWithNormals, iter->second);
								if(mergedClouds->size() == 0)
								{
									*mergedClouds = *transformedCloud;
								}
								else
								{
									*mergedClouds += *transformedCloud;
								}

								textureMesh->tex_materials[oi].tex_illum = 1;
								textureMesh->tex_materials[oi].tex_name = uFormat("material_%d", iter->first);
								textureMesh->tex_materials[oi].tex_file = uNumber2Str(iter->first);
								++oi;
							}
						}
						else
						{
							UERROR("Mesh not found for mesh %d", iter->first);
						}
					}
					if(textureSize == 0)
					{
						if(mergedClouds->size())
						{
							pcl::toPCLPointCloud2(*mergedClouds, polygonMesh->cloud);
						}
						else
						{
							polygonMesh->polygons.clear();
						}
					}
					else
					{
						textureMesh->tex_materials.resize(oi);
						textureMesh->tex_polygons.resize(oi);

						if(mergedClouds->size())
						{
							pcl::toPCLPointCloud2(*mergedClouds, textureMesh->cloud);
						}
					}
				}

				// end optimized or organized

				if(textureSize>0 && totalPolygons && textureMesh->tex_materials.size())
				{
					LOGI("Merging %d textures...", (int)textureMesh->tex_materials.size());
					globalTexture = mergeTextures(*textureMesh, textureSize);

					std::string baseName = uSplit(UFile::getName(filePath), '.').front();
					std::string textureDirectory = UDirectory::getDir(filePath);
					std::string fullPath = textureDirectory+UDirectory::separator()+baseName+".jpg";
					textureMesh->tex_materials[0].tex_file = baseName+".jpg";
					LOGI("Saving texture to %s.", fullPath.c_str());
					if(!cv::imwrite(fullPath, globalTexture))
					{
						LOGI("Failed saving %s!", fullPath.c_str());
					}
					else
					{
						LOGI("Saved %s (%d bytes).", fullPath.c_str(), globalTexture.total()*globalTexture.channels());
					}
				}
			}
			if(totalPolygons)
			{
				if(textureSize == 0)
				{
					UASSERT((int)polygonMesh->polygons.size() == totalPolygons);
					if(polygonMesh->polygons.size())
					{
						LOGI("Saving ply (%d vertices, %d polygons) to %s.", (int)polygonMesh->cloud.data.size()/polygonMesh->cloud.point_step, totalPolygons, filePath.c_str());
						success = pcl::io::savePLYFile(filePath, *polygonMesh) == 0;
						if(success)
						{
							UINFO("Saved ply to %s!", filePath.c_str());
							exportedMesh_.reset(new pcl::TextureMesh);
							exportedMesh_->cloud = polygonMesh->cloud;
							exportedMesh_->tex_polygons.push_back(polygonMesh->polygons);
						}
						else
						{
							UERROR("Failed saving ply to %s!", filePath.c_str());
						}
					}
				}
				else if(textureMesh->tex_materials.size())
				{
					UASSERT(textureMesh->tex_polygons.size() && (int)textureMesh->tex_polygons[0].size() == totalPolygons);
					LOGI("Saving obj (%d vertices, %d polygons) to %s.", (int)textureMesh->cloud.data.size()/textureMesh->cloud.point_step, totalPolygons, filePath.c_str());
					success = pcl::io::saveOBJFile(filePath, *textureMesh) == 0;
					if(success)
					{
						LOGI("Saved obj to %s!", filePath.c_str());
						exportedMesh_ = textureMesh;
						exportedTexture_ = globalTexture;
					}
					else
					{
						UERROR("Failed saving obj to %s!", filePath.c_str());
					}
				}
				else
				{
					UERROR("Failed exporting obj to %s! There are no textures!", filePath.c_str());
				}
			}
			else
			{
				UERROR("Failed exporting to %s! There are no polygons!", filePath.c_str());
			}
		}
		else // Point cloud
		{
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr mergedClouds(new pcl::PointCloud<pcl::PointXYZRGB>);
			for(std::map<int, rtabmap::Transform>::iterator iter=poses.begin();
				iter!= poses.end();
				++iter)
			{
				std::map<int, Mesh>::iterator jter=createdMeshes_.find(iter->first);
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
				pcl::IndicesPtr indices(new std::vector<int>);
				float gain = 1.0f;
				if(jter != createdMeshes_.end())
				{
					cloud = jter->second.cloud;
					indices = jter->second.indices;
					gain = jter->second.gain;
				}
				else
				{
					rtabmap::SensorData data = rtabmap_->getMemory()->getNodeData(iter->first, true);
					if(!data.imageRaw().empty() && !data.depthRaw().empty())
					{
						cloud = rtabmap::util3d::cloudRGBFromSensorData(data, meshDecimation_, maxCloudDepth_, 0, indices.get());
					}
				}
				if(cloud->size() && indices->size())
				{
					// Convert organized to dense cloud
					pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformedCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
					if(cloudVoxelSize > 0.0f)
					{
						transformedCloud = rtabmap::util3d::voxelize(cloud, indices, cloudVoxelSize);
						transformedCloud = rtabmap::util3d::transformPointCloud(transformedCloud, iter->second);
					}
					else
					{
						// it looks like that using only transformPointCloud with indices
						// flushes the colors, so we should extract points before... maybe a too old PCL version
						pcl::copyPointCloud(*cloud, *indices, *transformedCloud);
						transformedCloud = rtabmap::util3d::transformPointCloud(transformedCloud, iter->second);
					}

					if(gain != 1.0f)
					{
						//LOGD("cloud %d, gain=%f", iter->first, gain);
						for(unsigned int i=0; i<transformedCloud->size(); ++i)
						{
							pcl::PointXYZRGB & pt = transformedCloud->at(i);
							//LOGI("color %d = %d %d %d", i, (int)pt.r, (int)pt.g, (int)pt.b);
							pt.r = uchar(std::max(0.0, std::min(255.0, double(pt.r) * gain)));
							pt.g = uchar(std::max(0.0, std::min(255.0, double(pt.g) * gain)));
							pt.b = uchar(std::max(0.0, std::min(255.0, double(pt.b) * gain)));

						}
					}

					if(mergedClouds->size() == 0)
					{
						*mergedClouds = *transformedCloud;
					}
					else
					{
						*mergedClouds += *transformedCloud;
					}
				}
			}

			if(mergedClouds->size())
			{
				if(cloudVoxelSize > 0.0f)
				{
					mergedClouds = rtabmap::util3d::voxelize(mergedClouds, cloudVoxelSize);
				}

				pcl::PolygonMesh mesh;
				pcl::toPCLPointCloud2(*mergedClouds, mesh.cloud);

				LOGI("Saving ply (%d points) to %s.", (int)mergedClouds->size(), filePath.c_str());
				success = pcl::io::savePLYFileBinary(filePath, mesh) == 0;
				if(success)
				{
					LOGI("Saved ply to %s!", filePath.c_str());
					mergedClouds->clear();
					exportedMesh_.reset(new pcl::TextureMesh);
					exportedMesh_->cloud = mesh.cloud;
				}
				else
				{
					UERROR("Failed saving ply to %s!", filePath.c_str());
				}
			}
		}

		if(blockRendering)
		{
			renderingMutex_.unlock();
		}
	}
	catch (std::exception & e)
	{
		UERROR("Out of memory! %s", e.what());

		if(blockRendering)
		{
			renderingMutex_.unlock();
		}

		success = false;
	}

	return success;
}

bool RTABMapApp::postExportation(bool visualize)
{
	LOGI("postExportation(visualize=%d)", visualize?1:0);
	if(visualize && exportedMesh_->cloud.data.size())
	{
		boost::mutex::scoped_lock  lock(renderingMutex_);
		visualizingMesh_ = true;
		exportedMeshUpdated_ = true;
	}
	else
	{
		exportedMesh_.reset(new pcl::TextureMesh);
		exportedTexture_ = cv::Mat();
		exportedMeshUpdated_ = false;
		visualizingMesh_ = false;
	}

	return visualizingMesh_;
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
			returnedValue = rtabmap_->detectMoreLoopClosures(1.0f, M_PI/6.0f, approach == -1?5:1);
		}

		// graph optimization
		if(returnedValue >=0)
		{
			if (approach == 1)
			{
				if(rtabmap::Optimizer::isAvailable(rtabmap::Optimizer::kTypeG2O))
				{
					std::map<int, rtabmap::Signature> signatures;
					rtabmap_->getGraph(poses, links, true, true, &signatures);

					rtabmap::ParametersMap param;
					param.insert(rtabmap::ParametersPair(rtabmap::Parameters::kOptimizerIterations(), "30"));
					param.insert(rtabmap::ParametersPair(rtabmap::Parameters::kOptimizerEpsilon(), "0"));
					rtabmap::Optimizer * sba = rtabmap::Optimizer::create(rtabmap::Optimizer::kTypeG2O, param);
					poses = sba->optimizeBA(poses.rbegin()->first, poses, links, signatures);
					delete sba;
				}
				else
				{
					UERROR("g2o not available!");
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

			LOGI("PostProcessing, sending rtabmap event to update graph...");
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
			LOGI("Received RtabmapEvent initialized event!");
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
		int matches = (int)uValue(stats.data(), rtabmap::Statistics::kLoopVisual_matches(), 0.0f);
		int rejected = (int)uValue(stats.data(), rtabmap::Statistics::kLoopRejectedHypothesis(), 0.0f);
		float optimizationMaxError = uValue(stats.data(), rtabmap::Statistics::kLoopOptimization_max_error(), 0.0f);
		float rehearsalValue = uValue(stats.data(), rtabmap::Statistics::kMemoryRehearsal_sim(), 0.0f);
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
					jmethodID methodID = env->GetMethodID(clazz, "updateStatsCallback", "(IIIIFIIIIIIFIFIFF)V" );
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
								matches,
								featuresExtracted,
								hypothesis,
								lastDrawnCloudsCount_,
								renderingTime_>0.0f?1.0f/renderingTime_:0.0f,
								rejected,
								rehearsalValue,
								optimizationMaxError);
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

