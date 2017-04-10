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
#include <pcl/common/transforms.h>

#include <glm/gtx/transform.hpp>

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
    "attribute vec3 aVertex;\n"
	"attribute vec3 aNormal;\n"
    "attribute vec3 aColor;\n"

    "uniform mat4 uMVP;\n"
	"uniform mat3 uN;\n"
	"uniform vec3 uAmbientColor;\n"
	"uniform vec3 uLightingDirection;\n"
	"uniform bool uUseLighting;\n"

    "uniform float uPointSize;\n"
    "varying vec3 vColor;\n"
    "varying float vLightWeighting;\n"

    "void main() {\n"
    "  gl_Position = uMVP*vec4(aVertex.x, aVertex.y, aVertex.z, 1.0);\n"
    "  gl_PointSize = uPointSize;\n"
	"  if (!uUseLighting) {\n"
	"    vLightWeighting = 1.0;\n"
	"  } else {\n"
	"    vec3 transformedNormal = uN * aNormal;\n"
	"    vLightWeighting = max(dot(transformedNormal, uLightingDirection)*0.5+0.5, 0.0);\n"
	"    if(vLightWeighting<0.5) vLightWeighting=0.5;\n"
	"  }\n"
    "  vColor = aColor;\n"
    "}\n";
const std::string kPointCloudFragmentShader =
    "precision mediump float;\n"
    "precision mediump int;\n"
	"uniform float uGainR;\n"
	"uniform float uGainG;\n"
	"uniform float uGainB;\n"
    "varying vec3 vColor;\n"
	"varying float vLightWeighting;\n"
    "void main() {\n"
    "  vec4 textureColor = vec4(vColor.z, vColor.y, vColor.x, 1.0);\n"
	"  gl_FragColor = vec4(textureColor.r * uGainR * vLightWeighting, textureColor.g * uGainG * vLightWeighting, textureColor.b * uGainB * vLightWeighting, textureColor.a);\n"
    "}\n";

const std::string kTextureMeshVertexShader =
    "precision mediump float;\n"
    "precision mediump int;\n"
    "attribute vec3 aVertex;\n"
	"attribute vec3 aNormal;\n"
    "attribute vec2 aTexCoord;\n"

    "uniform mat4 uMVP;\n"
    "uniform mat3 uN;\n"
	"uniform vec3 uAmbientColor;\n"
    "uniform vec3 uLightingDirection;\n"
    "uniform bool uUseLighting;\n"

    "varying vec2 vTexCoord;\n"
    "varying float vLightWeighting;\n"

    "void main() {\n"
    "  gl_Position = uMVP*vec4(aVertex.x, aVertex.y, aVertex.z, 1.0);\n"

	"  if(aTexCoord.x < 0.0) {\n"
	"    vTexCoord.x = 1.0;\n"
	"    vTexCoord.y = 1.0;\n" // bottom right corner
	"  } else {\n"
    "    vTexCoord = aTexCoord;\n"
    "  }\n"

	"  if (!uUseLighting) {\n"
    "    vLightWeighting = 1.0;\n"
    "  } else {\n"
    "    vec3 transformedNormal = uN * aNormal;\n"
    "    vLightWeighting = max(dot(transformedNormal, uLightingDirection)*0.5+0.5, 0.0);\n"
    "    if(vLightWeighting<0.5) vLightWeighting=0.5;\n"
    "  }\n"
    "}\n";
const std::string kTextureMeshFragmentShader =
    "precision mediump float;\n"
    "precision mediump int;\n"
	"uniform sampler2D uTexture;\n"
	"uniform float uGainR;\n"
	"uniform float uGainG;\n"
	"uniform float uGainB;\n"
    "varying vec2 vTexCoord;\n"
	"varying float vLightWeighting;\n"
    "void main() {\n"
    "  vec4 textureColor = texture2D(uTexture, vTexCoord);\n"
	"  gl_FragColor = vec4(textureColor.r * uGainR * vLightWeighting, textureColor.g * uGainG * vLightWeighting, textureColor.b * uGainB * vLightWeighting, textureColor.a);\n"
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
		box_(0),
		trace_(0),
		graph_(0),
		graphVisible_(true),
		gridVisible_(true),
		traceVisible_(true),
		color_camera_to_display_rotation_(ROTATION_0),
		currentPose_(0),
		cloud_shader_program_(0),
		texture_mesh_shader_program_(0),
		graph_shader_program_(0),
		mapRendering_(true),
		meshRendering_(true),
		meshRenderingTexture_(true),
		pointSize_(5.0f),
		frustumCulling_(true),
		boundingBoxRendering_(false),
		lighting_(false),
		backfaceCulling_(true),
		r_(0.0f),
		g_(0.0f),
		b_(0.0f)
{
	gesture_camera_ = new tango_gl::GestureCamera();
	gesture_camera_->SetCameraType(
	      tango_gl::GestureCamera::kFirstPerson);
}

Scene::~Scene() {
	DeleteResources();
	delete gesture_camera_;
}

//Should only be called in OpenGL thread!
void Scene::InitGLContent()
{
	if(axis_ != 0)
	{
		DeleteResources();
	}

	UASSERT(axis_ == 0);


  axis_ = new tango_gl::Axis();
  frustum_ = new tango_gl::Frustum();
  trace_ = new tango_gl::Trace();
  grid_ = new tango_gl::Grid();
  box_ = new BoundingBoxDrawable();
  currentPose_ = new rtabmap::Transform();


  axis_->SetScale(glm::vec3(0.5f,0.5f,0.5f));
  frustum_->SetColor(kTraceColor);
  trace_->ClearVertexArray();
  trace_->SetColor(kTraceColor);
  grid_->SetColor(kGridColor);
  grid_->SetPosition(-kHeightOffset);
  box_->SetShader();
  box_->SetColor(1,0,0);

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
	if(axis_)
	{
	  delete axis_;
	  axis_ = 0;
	  delete frustum_;
	  delete trace_;
	  delete grid_;
	  delete currentPose_;
	  delete box_;
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

std::vector<glm::vec4> computeFrustumPlanes(const glm::mat4 & mat, bool normalize = true)
{
	// http://www.txutxi.com/?p=444
	std::vector<glm::vec4> planes(6);

	// Left Plane
	// col4 + col1
	planes[0].x = mat[0][3] + mat[0][0];
	planes[0].y = mat[1][3] + mat[1][0];
	planes[0].z = mat[2][3] + mat[2][0];
	planes[0].w = mat[3][3] + mat[3][0];

	// Right Plane
	// col4 - col1
	planes[1].x = mat[0][3] - mat[0][0];
	planes[1].y = mat[1][3] - mat[1][0];
	planes[1].z = mat[2][3] - mat[2][0];
	planes[1].w = mat[3][3] - mat[3][0];

	// Bottom Plane
	// col4 + col2
	planes[2].x = mat[0][3] + mat[0][1];
	planes[2].y = mat[1][3] + mat[1][1];
	planes[2].z = mat[2][3] + mat[2][1];
	planes[2].w = mat[3][3] + mat[3][1];

	// Top Plane
	// col4 - col2
	planes[3].x = mat[0][3] - mat[0][1];
	planes[3].y = mat[1][3] - mat[1][1];
	planes[3].z = mat[2][3] - mat[2][1];
	planes[3].w = mat[3][3] - mat[3][1];

	// Near Plane
	// col4 + col3
	planes[4].x = mat[0][3] + mat[0][2];
	planes[4].y = mat[1][3] + mat[1][2];
	planes[4].z = mat[2][3] + mat[2][2];
	planes[4].w = mat[3][3] + mat[3][2];

	// Far Plane
	// col4 - col3
	planes[5].x = mat[0][3] - mat[0][2];
	planes[5].y = mat[1][3] - mat[1][2];
	planes[5].z = mat[2][3] - mat[2][2];
	planes[5].w = mat[3][3] - mat[3][2];

	//if(normalize)
	{
		for(unsigned int i=0;i<planes.size(); ++i)
		{
			if(normalize)
			{
				float d = std::sqrt(planes[i].x * planes[i].x + planes[i].y * planes[i].y + planes[i].z * planes[i].z); // for normalizing the coordinates
				planes[i].x/=d;
				planes[i].y/=d;
				planes[i].z/=d;
				planes[i].w/=d;
			}
		}
	}

	return planes;
}

/**
 * Tells whether or not b is intersecting f.
 * http://www.txutxi.com/?p=584
 * @param f Viewing frustum.
 * @param b An axis aligned bounding box.
 * @return True if b intersects f, false otherwise.
 */
bool intersectFrustumAABB(
        const std::vector<glm::vec4> &planes,
        const pcl::PointXYZ &boxMin,
		const pcl::PointXYZ &boxMax)
{
  // Indexed for the 'index trick' later
	const pcl::PointXYZ * box[] = {&boxMin, &boxMax};

  // We only need to do 6 point-plane tests
  for (unsigned int i = 0; i < planes.size(); ++i)
  {
    // This is the current plane
    const glm::vec4 &p = planes[i];

    // p-vertex selection (with the index trick)
    // According to the plane normal we can know the
    // indices of the positive vertex
    const int px = p.x > 0.0f?1:0;
    const int py = p.y > 0.0f?1:0;
    const int pz = p.z > 0.0f?1:0;

    // Dot product
    // project p-vertex on plane normal
    // (How far is p-vertex from the origin)
    const float dp =
        (p.x*box[px]->x) +
        (p.y*box[py]->y) +
        (p.z*box[pz]->z) + p.w;

    // Doesn't intersect if it is behind the plane
    if (dp < 0) {return false; }
  }
  return true;
}

//Should only be called in OpenGL thread!
int Scene::Render() {
	UASSERT(gesture_camera_ != 0);

	glEnable(GL_DEPTH_TEST);
	if(backfaceCulling_)
	{
		glEnable(GL_CULL_FACE);
	}
	else
	{
		glDisable(GL_CULL_FACE);
	}

	glClearColor(r_, g_, b_, 1.0f);
	glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);

	if(!currentPose_->isNull())
	{
		glm::vec3 position(currentPose_->x(), currentPose_->y(), currentPose_->z());
		Eigen::Quaternionf quat = currentPose_->getQuaternionf();
		glm::quat rotation(quat.w(), quat.x(), quat.y(), quat.z());

		glm::mat4 rotateM;
		rotateM = glm::rotate<float>(float(color_camera_to_display_rotation_)*-1.57079632679489661923132169163975144, glm::vec3(0.0f, 0.0f, 1.0f));

		if (gesture_camera_->GetCameraType() == tango_gl::GestureCamera::kFirstPerson)
		{
			// In first person mode, we directly control camera's motion.
			gesture_camera_->SetPosition(position);
			gesture_camera_->SetRotation(rotation*glm::quat(rotateM));
		}
		else
		{
			// In third person or top down mode, we follow the camera movement.
			gesture_camera_->SetAnchorPosition(position, rotation*glm::quat(rotateM));

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

		if(gridVisible_)
		{
			grid_->Render(gesture_camera_->GetProjectionMatrix(),
					gesture_camera_->GetViewMatrix());
		}
	}

	float fov = 45.0f;
	rtabmap::Transform openglCamera = GetOpenGLCameraPose(&fov);//*rtabmap::Transform(0.0f, 0.0f, 3.0f, 0.0f, 0.0f, 0.0f);
	// transform in same coordinate as frustum filtering
	openglCamera *= rtabmap::Transform(
		 0.0f,  0.0f,  1.0f, 0.0f,
		 0.0f,  1.0f,  0.0f, 0.0f,
		-1.0f,  0.0f,  0.0f, 0.0f);

	int cloudDrawn=0;
	if(mapRendering_ && frustumCulling_)
	{
		std::vector<glm::vec4> planes = computeFrustumPlanes(gesture_camera_->GetProjectionMatrix()*gesture_camera_->GetViewMatrix(), true);
		for(std::map<int, PointCloudDrawable*>::const_iterator iter=pointClouds_.begin(); iter!=pointClouds_.end(); ++iter)
		{
			if(iter->second->isVisible())
			{
				if(intersectFrustumAABB(planes,
						iter->second->aabbMinWorld(),
						iter->second->aabbMaxWorld()))
				{
					if(boundingBoxRendering_)
					{
						box_->updateVertices(iter->second->aabbMinWorld(), iter->second->aabbMaxWorld());
						box_->Render(gesture_camera_->GetProjectionMatrix(),
											gesture_camera_->GetViewMatrix());
					}

					++cloudDrawn;
					Eigen::Vector3f cloudToCamera(
							iter->second->getPose().x() - openglCamera.x(),
							iter->second->getPose().y() - openglCamera.y(),
							iter->second->getPose().z() - openglCamera.z());
					float distanceToCameraSqr = cloudToCamera[0]*cloudToCamera[0] + cloudToCamera[1]*cloudToCamera[1] + cloudToCamera[2]*cloudToCamera[2];
					iter->second->Render(gesture_camera_->GetProjectionMatrix(), gesture_camera_->GetViewMatrix(), meshRendering_, pointSize_, meshRenderingTexture_, lighting_, distanceToCameraSqr);
				}
			}
		}
	}
	else
	{
		for(std::map<int, PointCloudDrawable*>::const_iterator iter=pointClouds_.begin(); iter!=pointClouds_.end(); ++iter)
		{
			if(!mapRendering_ && iter->first > 0)
			{
				break;
			}

			if(iter->second->isVisible())
			{
				if(boundingBoxRendering_)
				{
					box_->updateVertices(iter->second->aabbMinWorld(), iter->second->aabbMaxWorld());
					box_->Render(gesture_camera_->GetProjectionMatrix(),
										gesture_camera_->GetViewMatrix());
				}

				++cloudDrawn;
				Eigen::Vector3f cloudToCamera(
						iter->second->getPose().x() - openglCamera.x(),
						iter->second->getPose().y() - openglCamera.y(),
						iter->second->getPose().z() - openglCamera.z());
				float distanceToCameraSqr = cloudToCamera[0]*cloudToCamera[0] + cloudToCamera[1]*cloudToCamera[1] + cloudToCamera[2]*cloudToCamera[2];
				iter->second->Render(gesture_camera_->GetProjectionMatrix(), gesture_camera_->GetViewMatrix(), meshRendering_, pointSize_, meshRenderingTexture_, lighting_, distanceToCameraSqr);
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
	if(graphVisible_)
	{
		UASSERT(graph_shader_program_ != 0);
		graph_ = new GraphDrawable(graph_shader_program_, poses, links);
	}
}

void Scene::setGraphVisible(bool visible)
{
	graphVisible_ = visible;
}

void Scene::setGridVisible(bool visible)
{
	gridVisible_ = visible;
}

void Scene::setTraceVisible(bool visible)
{
	traceVisible_ = visible;
}

//Should only be called in OpenGL thread!
void Scene::addCloud(
		int id,
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		const rtabmap::Transform & pose)
{
	LOGI("add cloud %d (%d points %d indices)", id, (int)cloud->size(), indices.get()?(int)indices->size():0);
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
			indices);
	drawable->setPose(pose);
	pointClouds_.insert(std::make_pair(id, drawable));
}

void Scene::addMesh(
		int id,
		const Mesh & mesh,
		const rtabmap::Transform & pose)
{
	LOGI("add mesh %d", id);
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
			mesh);
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

bool Scene::hasMesh(int id) const
{
	return pointClouds_.find(id) != pointClouds_.end() && pointClouds_.at(id)->hasMesh();
}

bool Scene::hasTexture(int id) const
{
	return pointClouds_.find(id) != pointClouds_.end() && pointClouds_.at(id)->hasTexture();
}

std::set<int> Scene::getAddedClouds() const
{
	return uKeysSet(pointClouds_);
}

void Scene::updateCloudPolygons(int id, const std::vector<pcl::Vertices> & polygons)
{
	std::map<int, PointCloudDrawable*>::iterator iter=pointClouds_.find(id);
	if(iter != pointClouds_.end())
	{
		iter->second->updatePolygons(polygons);
	}
}

void Scene::updateMesh(int id, const Mesh & mesh)
{
	std::map<int, PointCloudDrawable*>::iterator iter=pointClouds_.find(id);
	if(iter != pointClouds_.end())
	{
		iter->second->updateMesh(mesh);
	}
}

void Scene::updateGains(int id, float gainR, float gainG, float gainB)
{
	std::map<int, PointCloudDrawable*>::iterator iter=pointClouds_.find(id);
	if(iter != pointClouds_.end())
	{
		iter->second->setGains(gainR, gainG, gainB);
	}
}
