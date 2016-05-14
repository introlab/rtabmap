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

#ifndef RTABMAP_APP_H_
#define RTABMAP_APP_H_

#include <jni.h>
#include <memory>

#include <tango_client_api.h>  // NOLINT
#include <tango-gl/util.h>

#include <scene.h>
#include <CameraTango.h>
#include <util.h>

#include <rtabmap/core/RtabmapThread.h>
#include <rtabmap/utilite/UEventsHandler.h>
#include <boost/thread/mutex.hpp>
#include <pcl/pcl_base.h>

// RTABMapApp handles the application lifecycle and resources.
class RTABMapApp : public UEventsHandler {
 public:
  // Constructor and deconstructor.
  RTABMapApp();
  ~RTABMapApp();

  void onCreate(JNIEnv* env, jobject caller_activity);

  void openDatabase(const std::string & databasePath);

  bool onTangoServiceConnected(JNIEnv* env, jobject iBinder);

  // Explicitly reset motion tracking and restart the pipeline.
  // Note that this will cause motion tracking to re-initialize.
  void TangoResetMotionTracking();

  // Tango Service point cloud callback function for depth data. Called when new
  // new point cloud data is available from the Tango Service.
  //
  // @param pose: The current point cloud returned by the service,
  //              caller allocated.
  void onPointCloudAvailable(const TangoXYZij* xyz_ij);

  // Tango service pose callback function for pose data. Called when new
  // information about device pose is available from the Tango Service.
  //
  // @param pose: The current pose returned by the service, caller allocated.
  void onPoseAvailable(const TangoPoseData* pose);

  // Tango service event callback function for event data. Called when new events
  // are available from the Tango Service.
  //
  // @param event: Tango event, caller allocated.
  void onTangoEventAvailable(const TangoEvent* event);

  // Allocate OpenGL resources for rendering, mainly for initializing the Scene.
  void InitializeGLContent();

  // Setup the view port width and height.
  void SetViewPort(int width, int height);

  // Main render loop.
  int Render();

  // Release all non-OpenGL allocated resources.
  void onPause();

  // Set render camera's viewing angle, first person, third person or top down.
  //
  // @param: camera_type, camera type includes first person, third person and
  //         top down
  void SetCameraType(tango_gl::GestureCamera::CameraType camera_type);

  // Touch event passed from android activity. This function only supports two
  // touches.
  //
  // @param: touch_count, total count for touches.
  // @param: event, touch event of current touch.
  // @param: x0, normalized touch location for touch 0 on x axis.
  // @param: y0, normalized touch location for touch 0 on y axis.
  // @param: x1, normalized touch location for touch 1 on x axis.
  // @param: y1, normalized touch location for touch 1 on y axis.
  void OnTouchEvent(int touch_count, tango_gl::GestureCamera::TouchEvent event,
                    float x0, float y0, float x1, float y1);

  void setPausedMapping(bool paused);
  void setMapCloudShown(bool shown);
  void setOdomCloudShown(bool shown);
  void setMeshRendering(bool enabled);
  void setLocalizationMode(bool enabled);
  void setTrajectoryMode(bool enabled);
  void setGraphOptimization(bool enabled);
  void setNodesFiltering(bool enabled);
  void setGraphVisible(bool visible);
  void setAutoExposure(bool enabled);
  void setFullResolution(bool enabled);
  void setMaxCloudDepth(float value);
  void setMeshAngleTolerance(float value);
  void setMeshTriangleSize(int value);
  int setMappingParameter(const std::string & key, const std::string & value);

  void resetMapping();
  void save();
  bool exportMesh(const std::string & filePath);
  int postProcessing(int approach);

 protected:
  virtual void handleEvent(UEvent * event);

 private:
  rtabmap::ParametersMap getRtabmapParameters();

 private:
  rtabmap::CameraTango * camera_;
  rtabmap::RtabmapThread * rtabmapThread_;
  rtabmap::Rtabmap * rtabmap_;
  LogHandler * logHandler_;

  bool odomCloudShown_;
  bool graphOptimization_;
  bool nodesFiltering_;
  bool localizationMode_;
  bool trajectoryMode_;
  bool autoExposure_;
  bool fullResolution_;
  float maxCloudDepth_;
  int meshTrianglePix_;
  float meshAngleToleranceDeg_;

  rtabmap::ParametersMap mappingParameters_;


  bool clearSceneOnNextRender_;
  int totalPoints_;
  int totalPolygons_;
  int lastDrawnCloudsCount_;
  float renderingFPS_;

  // main_scene_ includes all drawable object for visualizing Tango device's
  // movement and point cloud.
  Scene main_scene_;

	std::list<rtabmap::Statistics> rtabmapEvents_;
	std::list<rtabmap::OdometryEvent> odomEvents_;
	std::list<rtabmap::Transform> poseEvents_;

	boost::mutex rtabmapMutex_;
	boost::mutex meshesMutex_;
	boost::mutex odomMutex_;
	boost::mutex poseMutex_;

	struct Mesh
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
		std::vector<pcl::Vertices> polygons;
		rtabmap::Transform pose;
		cv::Mat texture;
	};

	std::map<int, Mesh> createdMeshes_;
	std::map<int, rtabmap::Transform> rawPoses_;

	std::pair<rtabmap::RtabmapEventInit::Status, std::string> status_;
};

#endif  // TANGO_POINT_CLOUD_POINT_CLOUD_APP_H_
