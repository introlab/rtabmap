/*
 * Copyright 2014 Google Inc. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <tango-gl/conversions.h>
#include <tango-gl/gesture_camera.h>

#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/core/util3d_filtering.h>

#include "scene.h"
#include "util.h"

// We want to represent the device properly with respect to the ground so we'll
// add an offset in z to our origin. We'll set this offset to 1.3 meters based
// on the average height of a human standing with a Tango device. This allows us
// to place a grid roughly on the ground for most users.
const glm::vec3 kHeightOffset = glm::vec3(0.0f, 1.3f, 0.0f);

// Color of the motion tracking trajectory.
const tango_gl::Color kTraceColor(0.66f, 0.66f, 0.66f);

// Color of the ground grid.
const tango_gl::Color kGridColor(0.85f, 0.85f, 0.85f);

// Frustum scale.
const glm::vec3 kFrustumScale = glm::vec3(0.4f, 0.3f, 0.5f);

const std::string kPointCloudVertexShader =
    "precision mediump float;\n"
    "precision mediump int;\n"
    "attribute vec3 vertex;\n"
    "attribute vec3 color;\n"
    "uniform mat4 mvp;\n"
    "uniform float point_size;\n"
    "varying vec3 v_color;\n"
    "void main() {\n"
    "  gl_Position = mvp*vec4(vertex.x, vertex.y, vertex.z, 1.0);\n"
    "  gl_PointSize = point_size;\n"
    "  v_color = color;\n"
    "}\n";
const std::string kPointCloudFragmentShader =
    "precision mediump float;\n"
    "precision mediump int;\n"
    "varying vec3 v_color;\n"
    "void main() {\n"
    "  gl_FragColor = vec4(v_color.z, v_color.y, v_color.x, 1.0);\n"
    "}\n";

const std::string kTextureMeshVertexShader =
    "precision mediump float;\n"
    "precision mediump int;\n"
    "attribute vec3 vertex;\n"
    "attribute vec2 a_TexCoordinate;\n"
    "uniform mat4 mvp;\n"
    "varying vec2 v_TexCoordinate;\n"
    "void main() {\n"
    "  gl_Position = mvp*vec4(vertex.x, vertex.y, vertex.z, 1.0);\n"
    "  v_TexCoordinate = a_TexCoordinate;\n"
    "}\n";
const std::string kTextureMeshFragmentShader =
    "precision mediump float;\n"
    "precision mediump int;\n"
	"uniform sampler2D u_Texture;\n"
    "varying vec2 v_TexCoordinate;\n"
    "void main() {\n"
    "  gl_FragColor = texture2D(u_Texture, v_TexCoordinate);\n"
    "}\n";

const std::string kGraphVertexShader =
    "precision mediump float;\n"
    "precision mediump int;\n"
    "attribute vec3 vertex;\n"
    "uniform vec3 color;\n"
    "uniform mat4 mvp;\n"
    "varying vec3 v_color;\n"
    "void main() {\n"
    "  gl_Position = mvp*vec4(vertex.x, vertex.y, vertex.z, 1.0);\n"
    "  v_color = color;\n"
    "}\n";
const std::string kGraphFragmentShader =
    "precision mediump float;\n"
    "precision mediump int;\n"
    "varying vec3 v_color;\n"
    "void main() {\n"
    "  gl_FragColor = vec4(v_color.z, v_color.y, v_color.x, 1.0);\n"
    "}\n";



Scene::Scene() :
		gesture_camera_(0),
		axis_(0),
		frustum_(0),
		grid_(0),
		trace_(0),
		graph_(0),
		graphVisible_(true),
		traceVisible_(true),
		currentPose_(0),
		cloud_shader_program_(0),
		texture_mesh_shader_program_(0),
		graph_shader_program_(0),
		mapRendering_(true),
		meshRendering_(true),
		pointSize_(3.0f) {}

Scene::~Scene() {DeleteResources();}

//Should only be called in OpenGL thread!
void Scene::InitGLContent()
{
	if(gesture_camera_ != 0)
	{
		DeleteResources();
	}

	UASSERT(gesture_camera_ == 0);

  gesture_camera_ = new tango_gl::GestureCamera();
  axis_ = new tango_gl::Axis();
  frustum_ = new tango_gl::Frustum();
  trace_ = new tango_gl::Trace();
  grid_ = new tango_gl::Grid();
  currentPose_ = new rtabmap::Transform();


  axis_->SetScale(glm::vec3(0.5f,0.5f,0.5f));
  frustum_->SetColor(kTraceColor);
  trace_->ClearVertexArray();
  trace_->SetColor(kTraceColor);
  grid_->SetColor(kGridColor);
  grid_->SetPosition(-kHeightOffset);
  gesture_camera_->SetCameraType(
      tango_gl::GestureCamera::kFirstPerson);

  if(cloud_shader_program_ == 0)
  {
	  cloud_shader_program_ = tango_gl::util::CreateProgram(kPointCloudVertexShader.c_str(), kPointCloudFragmentShader.c_str());
	  UASSERT(cloud_shader_program_ != 0);
  }
  if(texture_mesh_shader_program_ == 0)
  {
	  texture_mesh_shader_program_ = tango_gl::util::CreateProgram(kTextureMeshVertexShader.c_str(), kTextureMeshFragmentShader.c_str());
	  UASSERT(texture_mesh_shader_program_ != 0);
  }
  if(graph_shader_program_ == 0)
  {
	  graph_shader_program_ = tango_gl::util::CreateProgram(kGraphVertexShader.c_str(), kGraphFragmentShader.c_str());
	  UASSERT(graph_shader_program_ != 0);
  }
}

//Should only be called in OpenGL thread!
void Scene::DeleteResources() {

	LOGI("Scene::DeleteResources()");
	if(gesture_camera_)
	{
	  delete gesture_camera_;
	  delete axis_;
	  delete frustum_;
	  delete trace_;
	  delete grid_;
	  delete currentPose_;
	  gesture_camera_ = 0;
	}

	if (cloud_shader_program_) {
		glDeleteShader(cloud_shader_program_);
		cloud_shader_program_ = 0;
	  }
	if (texture_mesh_shader_program_) {
		glDeleteShader(texture_mesh_shader_program_);
		texture_mesh_shader_program_ = 0;
	  }
	if (graph_shader_program_) {
		glDeleteShader(graph_shader_program_);
		graph_shader_program_ = 0;
	  }

	clear();
}

//Should only be called in OpenGL thread!
void Scene::clear()
{
	LOGI("Scene::clear()");
	for(std::map<int, PointCloudDrawable*>::iterator iter=pointClouds_.begin(); iter!=pointClouds_.end(); ++iter)
	{
		delete iter->second;
	}
	if(trace_)
	{
		trace_->ClearVertexArray();
	}
	if(graph_)
	{
		delete graph_;
		graph_ = 0;
	}
	pointClouds_.clear();
}

//Should only be called in OpenGL thread!
void Scene::SetupViewPort(int w, int h) {
  if (h == 0) {
    LOGE("Setup graphic height not valid");
  }
  UASSERT(gesture_camera_ != 0);
  gesture_camera_->SetAspectRatio(static_cast<float>(w) /
                                  static_cast<float>(h));
  glViewport(0, 0, w, h);
}

//Should only be called in OpenGL thread!
int Scene::Render() {
	UASSERT(gesture_camera_ != 0);

  glEnable(GL_DEPTH_TEST);
  glEnable(GL_CULL_FACE);

  glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
  glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);

  if(!currentPose_->isNull())
  {
	  glm::vec3 position(currentPose_->x(), currentPose_->y(), currentPose_->z());
	  Eigen::Quaternionf quat = currentPose_->getQuaternionf();
	  glm::quat rotation(quat.w(), quat.x(), quat.y(), quat.z());

	  if (gesture_camera_->GetCameraType() == tango_gl::GestureCamera::kFirstPerson)
	  {
		// In first person mode, we directly control camera's motion.
		gesture_camera_->SetPosition(position);
		gesture_camera_->SetRotation(rotation);
	  }
	  else
	  {
		// In third person or top down more, we follow the camera movement.
		gesture_camera_->SetAnchorPosition(position, rotation);

		frustum_->SetPosition(position);
		frustum_->SetRotation(rotation);
		// Set the frustum scale to 4:3, this doesn't necessarily match the physical
		// camera's aspect ratio, this is just for visualization purposes.
		frustum_->SetScale(kFrustumScale);
		frustum_->Render(gesture_camera_->GetProjectionMatrix(),
						 gesture_camera_->GetViewMatrix());

		axis_->SetPosition(position);
		axis_->SetRotation(rotation);
		axis_->Render(gesture_camera_->GetProjectionMatrix(),
					  gesture_camera_->GetViewMatrix());
	  }

	  trace_->UpdateVertexArray(position);
	  if(traceVisible_)
	  {
		  trace_->Render(gesture_camera_->GetProjectionMatrix(),
						 gesture_camera_->GetViewMatrix());
	  }
  }


	grid_->Render(gesture_camera_->GetProjectionMatrix(),
				  gesture_camera_->GetViewMatrix());

	bool frustumCulling = true;
	int cloudDrawn=0;
	if(mapRendering_ && frustumCulling)
	{
		//Use camera frustum to cull nodes that don't need to be drawn
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		std::vector<int> ids(pointClouds_.size());

		cloud->resize(pointClouds_.size());
		ids.resize(pointClouds_.size());
		int oi=0;
		for(std::map<int, PointCloudDrawable*>::const_iterator iter=pointClouds_.begin(); iter!=pointClouds_.end(); ++iter)
		{
			if(!iter->second->getPose().isNull() && iter->second->isVisible())
			{
				(*cloud)[oi] = pcl::PointXYZ(iter->second->getPose().x(), iter->second->getPose().y(), iter->second->getPose().z());
				ids[oi++] = iter->first;
			}
		}
		cloud->resize(oi);
		ids.resize(oi);

		if(oi)
		{
			float fov = 45.0f;
			rtabmap::Transform openglCamera = GetOpenGLCameraPose(&fov)*rtabmap::Transform(0.0f, 0.0f, 3.0f, 0.0f, 0.0f, 0.0f);
			// transform in same coordinate as frustum filtering
			openglCamera *= rtabmap::Transform(
				 0.0f,  0.0f,  1.0f, 0.0f,
				 0.0f,  1.0f,  0.0f, 0.0f,
				-1.0f,  0.0f,  0.0f, 0.0f);
			pcl::IndicesPtr indices = rtabmap::util3d::frustumFiltering(
					cloud,
					pcl::IndicesPtr(new std::vector<int>),
					openglCamera,
					fov*2.0f,
					fov*2.0f,
					0.1f,
					100.0f);

			//LOGI("Frustum poses filtered = %d (showing %d/%d)",
			//			(int)(pointClouds_.size()-indices->size()),
			//			(int)indices->size(),
			//			(int)pointClouds_.size());

			for(unsigned int i=0; i<indices->size(); ++i)
			{
				++cloudDrawn;
				pointClouds_.find(ids[indices->at(i)])->second->Render(gesture_camera_->GetProjectionMatrix(), gesture_camera_->GetViewMatrix(), meshRendering_, pointSize_);
			}
		}
	}
	else
	{
		for(std::map<int, PointCloudDrawable*>::const_iterator iter=pointClouds_.begin(); iter!=pointClouds_.end(); ++iter)
		{
			if((mapRendering_ || iter->first < 0) && iter->second->isVisible())
			{
				++cloudDrawn;
				iter->second->Render(gesture_camera_->GetProjectionMatrix(), gesture_camera_->GetViewMatrix(), meshRendering_, pointSize_);
			}
		}
	}

	if(graphVisible_ && graph_)
	{
		graph_->Render(gesture_camera_->GetProjectionMatrix(), gesture_camera_->GetViewMatrix());
	}

	return cloudDrawn;
}

void Scene::SetCameraType(tango_gl::GestureCamera::CameraType camera_type) {
  gesture_camera_->SetCameraType(camera_type);
}

void Scene::SetCameraPose(const rtabmap::Transform & pose)
{
	UASSERT(currentPose_ != 0);
	UASSERT(!pose.isNull());
	*currentPose_ = pose;
}

rtabmap::Transform Scene::GetOpenGLCameraPose(float * fov) const
{
	if(fov)
	{
		*fov = gesture_camera_->getFOV();
	}
	return glmToTransform(gesture_camera_->GetTransformationMatrix());

}

void Scene::OnTouchEvent(int touch_count,
                         tango_gl::GestureCamera::TouchEvent event, float x0,
                         float y0, float x1, float y1) {
	UASSERT(gesture_camera_ != 0);
  gesture_camera_->OnTouchEvent(touch_count, event, x0, y0, x1, y1);
}

void Scene::updateGraph(
		const std::map<int, rtabmap::Transform> & poses,
		const std::multimap<int, rtabmap::Link> & links)
{
	LOGI("updateGraph");
	if(graph_)
	{
		delete graph_;
		graph_ = 0;
	}

	//create
	UASSERT(graph_shader_program_ != 0);
	graph_ = new GraphDrawable(graph_shader_program_, poses, links);
}

void Scene::setGraphVisible(bool visible)
{
	graphVisible_ = visible;
}

void Scene::setTraceVisible(bool visible)
{
	traceVisible_ = visible;
}

//Should only be called in OpenGL thread!
void Scene::addCloud(
		int id,
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		const std::vector<pcl::Vertices> & polygons,
		const rtabmap::Transform & pose,
		const cv::Mat & image)
{
	LOGI("addOrUpdateCloud cloud %d", id);
	std::map<int, PointCloudDrawable*>::iterator iter=pointClouds_.find(id);
	if(iter != pointClouds_.end())
	{
		delete iter->second;
		pointClouds_.erase(iter);
	}

	//create
	UASSERT(cloud_shader_program_ != 0 && texture_mesh_shader_program_!=0);
	PointCloudDrawable * drawable = new PointCloudDrawable(
			cloud_shader_program_,
			texture_mesh_shader_program_,
			cloud,
			polygons,
			image);
	drawable->setPose(pose);
	pointClouds_.insert(std::make_pair(id, drawable));
}


void Scene::setCloudPose(int id, const rtabmap::Transform & pose)
{
	UASSERT(!pose.isNull());
	std::map<int, PointCloudDrawable*>::iterator iter=pointClouds_.find(id);
	if(iter != pointClouds_.end())
	{
		iter->second->setPose(pose);
	}
}

void Scene::setCloudVisible(int id, bool visible)
{
	std::map<int, PointCloudDrawable*>::iterator iter=pointClouds_.find(id);
	if(iter != pointClouds_.end())
	{
		iter->second->setVisible(visible);
	}
}

bool Scene::hasCloud(int id) const
{
	return pointClouds_.find(id) != pointClouds_.end();
}

std::set<int> Scene::getAddedClouds() const
{
	return uKeysSet(pointClouds_);
}
