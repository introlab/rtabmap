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
#include <tango-gl/util.h>

#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/core/util3d_filtering.h>
#include <rtabmap/core/util3d_transforms.h>
#include <rtabmap/core/util3d_surface.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>

#include <glm/gtx/transform.hpp>

#include "scene.h"
#include "util.h"

// We want to represent the device properly with respect to the ground so we'll
// add an offset in z to our origin. We'll set this offset to 1.3 meters based
// on the average height of a human standing with a Tango device. This allows us
// to place a grid roughly on the ground for most users.
const glm::vec3 kHeightOffset = glm::vec3(0.0f, -1.3f, 0.0f);

// Color of the motion tracking trajectory.
const tango_gl::Color kTraceColor(0.66f, 0.66f, 0.66f);

// Color of the ground grid.
const tango_gl::Color kGridColor(0.85f, 0.85f, 0.85f);

// Frustum scale.
const glm::vec3 kFrustumScale = glm::vec3(0.4f, 0.3f, 0.5f);

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
		background_renderer_(0),
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
		frustumVisible_(true),
		color_camera_to_display_rotation_(rtabmap::ROTATION_0),
		currentPose_(0),
		graph_shader_program_(0),
		blending_(true),
		mapRendering_(true),
		meshRendering_(true),
		meshRenderingTexture_(true),
		pointSize_(10.0f),
		boundingBoxRendering_(false),
		lighting_(false),
		backfaceCulling_(true),
		wireFrame_(false),
		r_(0.0f),
		g_(0.0f),
		b_(0.0f),
		fboId_(0),
		rboId_(0),
		screenWidth_(0),
		screenHeight_(0),
		doubleTapOn_(false)
{
    depthTexture_ = 0;
	gesture_camera_ = new tango_gl::GestureCamera();
	gesture_camera_->SetCameraType(
	      tango_gl::GestureCamera::kThirdPersonFollow);
}

Scene::~Scene() {
	DeleteResources();
	delete gesture_camera_;
	delete currentPose_;
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


	axis_->SetScale(glm::vec3(0.5f,0.5f,0.5f));
	frustum_->SetColor(kTraceColor);
	trace_->ClearVertexArray();
	trace_->SetColor(kTraceColor);
	grid_->SetColor(kGridColor);
	grid_->SetPosition(kHeightOffset);
	box_->SetShader();
	box_->SetColor(1,0,0);

	PointCloudDrawable::createShaderPrograms();

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
		delete box_;
		delete background_renderer_;
		background_renderer_ = 0;
	}

	PointCloudDrawable::releaseShaderPrograms();

	if (graph_shader_program_) {
		glDeleteShader(graph_shader_program_);
		graph_shader_program_ = 0;
	}

	if(fboId_>0)
	{
		glDeleteFramebuffers(1, &fboId_);
		fboId_ = 0;
		glDeleteRenderbuffers(1, &rboId_);
		rboId_ = 0;
        glDeleteTextures(1, &depthTexture_);
        depthTexture_ = 0;
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
	for(std::map<int, tango_gl::Axis*>::iterator iter=markers_.begin(); iter!=markers_.end(); ++iter)
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
	markers_.clear();
	if(grid_)
	{
		grid_->SetPosition(kHeightOffset);
	}
}

//Should only be called in OpenGL thread!
void Scene::SetupViewPort(int w, int h) {
	if (h == 0) {
		LOGE("Setup graphic height not valid");
	}
    
	UASSERT(gesture_camera_ != 0);
	gesture_camera_->SetWindowSize(static_cast<float>(w), static_cast<float>(h));
	glViewport(0, 0, w, h);
	if(screenWidth_ != w || screenHeight_ != h || fboId_ == 0)
	{
        UINFO("Setup viewport OpenGL: %dx%d", w, h);
        
		if(fboId_>0)
		{
			glDeleteFramebuffers(1, &fboId_);
			fboId_ = 0;
			glDeleteRenderbuffers(1, &rboId_);
			rboId_ = 0;
			glDeleteTextures(1, &depthTexture_);
            depthTexture_ = 0;
		}

        GLint originid = 0;
        glGetIntegerv(GL_FRAMEBUFFER_BINDING, &originid);
        
		// regenerate fbo texture
		// create a framebuffer object, you need to delete them when program exits.
		glGenFramebuffers(1, &fboId_);
		glBindFramebuffer(GL_FRAMEBUFFER, fboId_);

		// Create depth texture
		glGenTextures(1, &depthTexture_);
		glBindTexture(GL_TEXTURE_2D, depthTexture_);
		glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
		glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
		glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);
		glBindTexture(GL_TEXTURE_2D, 0);
        
		glGenRenderbuffers(1, &rboId_);
		glBindRenderbuffer(GL_RENDERBUFFER, rboId_);
		glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT16, w, h);
		glBindRenderbuffer(GL_RENDERBUFFER, 0);

		// Set the texture to be at the color attachment point of the FBO (we pack depth 32 bits in color)
		glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, depthTexture_, 0);
		glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, rboId_);

		GLuint status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
        UASSERT ( status == GL_FRAMEBUFFER_COMPLETE);
		glBindFramebuffer(GL_FRAMEBUFFER, originid);
    }
	screenWidth_ = w;
	screenHeight_ = h;
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
 * @param planes Viewing frustum.
 * @param boxMin The axis aligned bounding box min.
 * @param boxMax The axis aligned bounding box max.
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
int Scene::Render(const float * uvsTransformed, glm::mat4 arViewMatrix, glm::mat4 arProjectionMatrix, const rtabmap::Mesh & occlusionMesh, bool mapping)
{
	UASSERT(gesture_camera_ != 0);

	if(currentPose_ == 0)
	{
		currentPose_ = new rtabmap::Transform(0,0,0,0,0,-M_PI/2.0f);
	}
	glm::vec3 position(currentPose_->x(), currentPose_->y(), currentPose_->z());
	Eigen::Quaternionf quat = currentPose_->getQuaternionf();
	glm::quat rotation(quat.w(), quat.x(), quat.y(), quat.z());
	glm::mat4 rotateM;
	if(!currentPose_->isNull())
	{
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
		}
	}

	glm::mat4 projectionMatrix = gesture_camera_->GetProjectionMatrix();
	glm::mat4 viewMatrix = gesture_camera_->GetViewMatrix();

	bool renderBackgroundCamera =
			background_renderer_ &&
            gesture_camera_->GetCameraType() == tango_gl::GestureCamera::kFirstPerson &&
            !rtabmap::glmToTransform(arProjectionMatrix).isNull() &&
            uvsTransformed;

	if(renderBackgroundCamera)
	{
        if(projectionMatrix[0][0] > arProjectionMatrix[0][0]-0.3)
        {
            projectionMatrix = arProjectionMatrix;
            viewMatrix = arViewMatrix;
        }
        else
        {
            renderBackgroundCamera = false;
        }
	}

	rtabmap::Transform openglCamera = GetOpenGLCameraPose();//*rtabmap::Transform(0.0f, 0.0f, 3.0f, 0.0f, 0.0f, 0.0f);
	// transform in same coordinate as frustum filtering
	openglCamera *= rtabmap::Transform(
		 0.0f,  0.0f,  1.0f, 0.0f,
		 0.0f,  1.0f,  0.0f, 0.0f,
		-1.0f,  0.0f,  0.0f, 0.0f);

	//Culling
	std::vector<glm::vec4> planes = computeFrustumPlanes(projectionMatrix*viewMatrix, true);
	std::vector<PointCloudDrawable*> cloudsToDraw(pointClouds_.size());
	int oi=0;
	for(std::map<int, PointCloudDrawable*>::const_iterator iter=pointClouds_.begin(); iter!=pointClouds_.end(); ++iter)
	{
		if(!mapRendering_ && iter->first > 0)
		{
			break;
		}

		if(iter->second->isVisible())
		{
			if(intersectFrustumAABB(planes,
					iter->second->aabbMinWorld(),
					iter->second->aabbMaxWorld()))
			{
				cloudsToDraw[oi++] = iter->second;
			}
		}
	}
	cloudsToDraw.resize(oi);

	// First rendering to get depth texture
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LESS);
	glDepthMask(GL_TRUE);
	glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
	glDisable (GL_BLEND);
	glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	if(backfaceCulling_)
	{
		glEnable(GL_CULL_FACE);
	}
	else
	{
		glDisable(GL_CULL_FACE);
	}

	bool onlineBlending =
        (!meshRendering_ &&
         occlusionMesh.cloud.get() &&
         occlusionMesh.cloud->size()) ||
        (blending_ &&
         gesture_camera_->GetCameraType()!=tango_gl::GestureCamera::kTopOrtho &&
         mapRendering_ && meshRendering_ &&
         (cloudsToDraw.size() > 1 || (renderBackgroundCamera && wireFrame_)));

	if(onlineBlending && fboId_)
	{
        GLint originid = 0;
        glGetIntegerv(GL_FRAMEBUFFER_BINDING, &originid);
        
		// set the rendering destination to FBO
		glBindFramebuffer(GL_FRAMEBUFFER, fboId_);

		glClearColor(0, 0, 0, 0);
		glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
        
        // Draw scene
        for(std::vector<PointCloudDrawable*>::const_iterator iter=cloudsToDraw.begin(); iter!=cloudsToDraw.end(); ++iter)
        {
            Eigen::Vector3f cloudToCamera(
                      (*iter)->getPose().x() - openglCamera.x(),
                      (*iter)->getPose().y() - openglCamera.y(),
                      (*iter)->getPose().z() - openglCamera.z());
            float distanceToCameraSqr = cloudToCamera[0]*cloudToCamera[0] + cloudToCamera[1]*cloudToCamera[1] + cloudToCamera[2]*cloudToCamera[2];
            (*iter)->Render(projectionMatrix, viewMatrix, meshRendering_, pointSize_, false, false, distanceToCameraSqr, 0, 0, 0, 0, 0, true);
        }
        
        if(!meshRendering_ && occlusionMesh.cloud.get() && occlusionMesh.cloud->size())
        {
            PointCloudDrawable drawable(occlusionMesh);
            drawable.Render(projectionMatrix, viewMatrix, true, pointSize_, false, false, 0, 0, 0, 0, 0, 0, true);
        }
        
		// back to normal window-system-provided framebuffer
		glBindFramebuffer(GL_FRAMEBUFFER, originid); // unbind
	}

	if(doubleTapOn_ && gesture_camera_->GetCameraType() != tango_gl::GestureCamera::kFirstPerson)
	{
		glClearColor(0, 0, 0, 0);
		glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);

        // FIXME: we could use the depthTexture if already computed!
		for(std::vector<PointCloudDrawable*>::const_iterator iter=cloudsToDraw.begin(); iter!=cloudsToDraw.end(); ++iter)
		{
            Eigen::Vector3f cloudToCamera(
                      (*iter)->getPose().x() - openglCamera.x(),
                      (*iter)->getPose().y() - openglCamera.y(),
                      (*iter)->getPose().z() - openglCamera.z());
            float distanceToCameraSqr = cloudToCamera[0]*cloudToCamera[0] + cloudToCamera[1]*cloudToCamera[1] + cloudToCamera[2]*cloudToCamera[2];
            
            (*iter)->Render(projectionMatrix, viewMatrix, meshRendering_, pointSize_*10.0f, false, false, distanceToCameraSqr, 0, 0, 0, 0, 0, true);
		}

		GLubyte zValue[4];
		glReadPixels(doubleTapPos_.x*screenWidth_, screenHeight_-doubleTapPos_.y*screenHeight_, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE, zValue);
		float zValueF = float(zValue[0]/255.0f) + float(zValue[1]/255.0f)/255.0f + float(zValue[2]/255.0f)/65025.0f + float(zValue[3]/255.0f)/160581375.0f;

		if(zValueF != 0.0f)
		{
			zValueF = zValueF*2.0-1.0;//NDC
			glm::vec4 point = glm::inverse(projectionMatrix*viewMatrix)*glm::vec4(doubleTapPos_.x*2.0f-1.0f, (1.0f-doubleTapPos_.y)*2.0f-1.0f, zValueF, 1.0f);
			point /= point.w;
			gesture_camera_->SetAnchorOffset(glm::vec3(point.x, point.y, point.z) - position);
		}
	}
	doubleTapOn_ = false;

	glClearColor(r_, g_, b_, 1.0f);
	glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
    
    if(renderBackgroundCamera && (!onlineBlending || !meshRendering_))
    {
        background_renderer_->Draw(uvsTransformed, 0, screenWidth_, screenHeight_, false);

        //To debug occlusion image:
        //PointCloudDrawable drawable(occlusionMesh);
        //drawable.Render(projectionMatrix, viewMatrix, true, pointSize_, false, false, 999.0f);
    }

	if(!currentPose_->isNull())
	{
		if (frustumVisible_ && gesture_camera_->GetCameraType() != tango_gl::GestureCamera::kFirstPerson)
		{
			frustum_->SetPosition(position);
			frustum_->SetRotation(rotation);
			// Set the frustum scale to 4:3, this doesn't necessarily match the physical
			// camera's aspect ratio, this is just for visualization purposes.
			frustum_->SetScale(kFrustumScale);
			frustum_->Render(projectionMatrix, viewMatrix);

			rtabmap::Transform cameraFrame = *currentPose_*rtabmap::optical_T_opengl*rtabmap::CameraMobile::opticalRotationInv;
			glm::vec3 positionCamera(cameraFrame.x(), cameraFrame.y(), cameraFrame.z());
			Eigen::Quaternionf quatCamera = cameraFrame.getQuaternionf();
			glm::quat rotationCamera(quatCamera.w(), quatCamera.x(), quatCamera.y(), quatCamera.z());

			axis_->SetPosition(positionCamera);
			axis_->SetRotation(rotationCamera);
			axis_->Render(projectionMatrix, viewMatrix);
		}

		trace_->UpdateVertexArray(position);
		if(traceVisible_)
		{
			trace_->Render(projectionMatrix, viewMatrix);
		}
        else
        {
            trace_->ClearVertexArray();
        }
	}

	if(gridVisible_ && !renderBackgroundCamera)
	{
		grid_->Render(projectionMatrix, viewMatrix);
	}

	if(graphVisible_ && graph_)
	{
		graph_->Render(projectionMatrix, viewMatrix);
	}


	if(onlineBlending)
	{
		glEnable (GL_BLEND);
		glDepthMask(GL_FALSE);
	}

	for(std::vector<PointCloudDrawable*>::const_iterator iter=cloudsToDraw.begin(); iter!=cloudsToDraw.end(); ++iter)
	{
		PointCloudDrawable * cloud = *iter;

		if(boundingBoxRendering_)
		{
			box_->updateVertices(cloud->aabbMinWorld(), cloud->aabbMaxWorld());
			box_->Render(projectionMatrix, viewMatrix);
		}

		Eigen::Vector3f cloudToCamera(
				cloud->getPose().x() - openglCamera.x(),
				cloud->getPose().y() - openglCamera.y(),
				cloud->getPose().z() - openglCamera.z());
		float distanceToCameraSqr = cloudToCamera[0]*cloudToCamera[0] + cloudToCamera[1]*cloudToCamera[1] + cloudToCamera[2]*cloudToCamera[2];

		cloud->Render(projectionMatrix, viewMatrix, meshRendering_, pointSize_, meshRenderingTexture_, lighting_, distanceToCameraSqr, onlineBlending?depthTexture_:0, screenWidth_, screenHeight_, gesture_camera_->getNearClipPlane(), gesture_camera_->getFarClipPlane(), false, wireFrame_);
	}

	if(onlineBlending)
	{
        if(renderBackgroundCamera && meshRendering_)
        {
            background_renderer_->Draw(uvsTransformed, depthTexture_, screenWidth_, screenHeight_, mapping);
        }
        
		glDisable (GL_BLEND);
		glDepthMask(GL_TRUE);
	}
    
	//draw markers on foreground
	for(std::map<int, tango_gl::Axis*>::const_iterator iter=markers_.begin(); iter!=markers_.end(); ++iter)
	{
		iter->second->Render(projectionMatrix, viewMatrix);
	}

	return (int)cloudsToDraw.size();
}

void Scene::SetCameraType(tango_gl::GestureCamera::CameraType camera_type) {
  gesture_camera_->SetCameraType(camera_type);
}

void Scene::SetCameraPose(const rtabmap::Transform & pose)
{
	UASSERT(!pose.isNull());
	if(currentPose_ ==0)
	{
		currentPose_ = new rtabmap::Transform(0,0,0,0,0,-M_PI/2.0f);
	}
	*currentPose_ = pose;
}

void Scene::setFOV(float angle)
{
	gesture_camera_->SetFieldOfView(angle);
}
void Scene::setOrthoCropFactor(float value)
{
	gesture_camera_->SetOrthoCropFactor(value);
}
void Scene::setGridRotation(float angleDeg)
{
	float angleRad = angleDeg * DEGREE_2_RADIANS;
	if(grid_)
	{
		glm::quat rot = glm::rotate(glm::quat(1,0,0,0), angleRad, glm::vec3(0, 1, 0));
		grid_->SetRotation(rot);
	}
}

rtabmap::Transform Scene::GetOpenGLCameraPose(float * fov) const
{
	if(fov)
	{
		*fov = gesture_camera_->getFOV();
	}
	return rtabmap::glmToTransform(gesture_camera_->GetTransformationMatrix());
}

void Scene::OnTouchEvent(int touch_count,
                         tango_gl::GestureCamera::TouchEvent event, float x0,
                         float y0, float x1, float y1) {
	UASSERT(gesture_camera_ != 0);
	if(touch_count == 3)
	{
		//doubletap
		if(!doubleTapOn_)
		{
			doubleTapPos_.x = x0;
			doubleTapPos_.y = y0;
			doubleTapOn_ = true;
		}
	}
	else
	{
		// rotate/translate/zoom
		gesture_camera_->OnTouchEvent(touch_count, event, x0, y0, x1, y1);
	}
}

void Scene::updateGraph(
		const std::map<int, rtabmap::Transform> & poses,
		const std::multimap<int, rtabmap::Link> & links)
{
	LOGI("updateGraph");
	//create
    UASSERT(graph_shader_program_ != 0);
    delete graph_;
    graph_ = new GraphDrawable(graph_shader_program_, poses, links);
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

void Scene::setFrustumVisible(bool visible)
{
	frustumVisible_ = visible;
}

//Should only be called in OpenGL thread!
void Scene::addMarker(
		int id,
		const rtabmap::Transform & pose)
{
	LOGI("add marker %d", id);
	std::map<int, tango_gl::Axis*>::iterator iter=markers_.find(id);
	if(iter == markers_.end())
	{
		//create
		tango_gl::Axis * drawable = new tango_gl::Axis();
		drawable->SetScale(glm::vec3(0.05f,0.05f,0.05f));
		drawable->SetLineWidth(5);
		markers_.insert(std::make_pair(id, drawable));
	}
	setMarkerPose(id, pose);
}
void Scene::setMarkerPose(int id, const rtabmap::Transform & pose)
{
	UASSERT(!pose.isNull());
	std::map<int, tango_gl::Axis*>::iterator iter=markers_.find(id);
	if(iter != markers_.end())
	{
		glm::vec3 position(pose.x(), pose.y(), pose.z());
		Eigen::Quaternionf quat = pose.getQuaternionf();
		glm::quat rotation(quat.w(), quat.x(), quat.y(), quat.z());
		iter->second->SetPosition(position);
		iter->second->SetRotation(rotation);
	}
}
bool Scene::hasMarker(int id) const
{
	return markers_.find(id) != markers_.end();
}
void Scene::removeMarker(int id)
{
	std::map<int, tango_gl::Axis*>::iterator iter=markers_.find(id);
	if(iter != markers_.end())
	{
		delete iter->second;
		markers_.erase(iter);
	}
}
std::set<int> Scene::getAddedMarkers() const
{
	return uKeysSet(markers_);
}

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
	PointCloudDrawable * drawable = new PointCloudDrawable(cloud, indices);
	drawable->setPose(pose);
	pointClouds_.insert(std::make_pair(id, drawable));
}

void Scene::addMesh(
		int id,
		const rtabmap::Mesh & mesh,
		const rtabmap::Transform & pose,
		bool createWireframe)
{
	LOGI("add mesh %d", id);
	std::map<int, PointCloudDrawable*>::iterator iter=pointClouds_.find(id);
	if(iter != pointClouds_.end())
	{
		delete iter->second;
		pointClouds_.erase(iter);
	}

	//create
	PointCloudDrawable * drawable = new PointCloudDrawable(mesh, createWireframe);
	drawable->setPose(pose);
	pointClouds_.insert(std::make_pair(id, drawable));

	if(!mesh.pose.isNull() && mesh.cloud->size() && (!mesh.cloud->isOrganized() || mesh.indices->size()))
	{
		UTimer time;
		float height = 0.0f;
		Eigen::Affine3f affinePose = mesh.pose.toEigen3f();
		if(mesh.polygons.size())
		{
			for(unsigned int i=0; i<mesh.polygons.size(); ++i)
			{
				for(unsigned int j=0; j<mesh.polygons[i].vertices.size(); ++j)
				{
					pcl::PointXYZRGB pt = pcl::transformPoint(mesh.cloud->at(mesh.polygons[i].vertices[j]), affinePose);
					if(pt.z < height)
					{
						height = pt.z;
					}
				}
			}
		}
		else
		{
			if(mesh.cloud->isOrganized())
			{
				for(unsigned int i=0; i<mesh.indices->size(); ++i)
				{
					pcl::PointXYZRGB pt = pcl::transformPoint(mesh.cloud->at(mesh.indices->at(i)), affinePose);
					if(pt.z < height)
					{
						height = pt.z;
					}
				}
			}
			else
			{
				for(unsigned int i=0; i<mesh.cloud->size(); ++i)
				{
					pcl::PointXYZRGB pt = pcl::transformPoint(mesh.cloud->at(i), affinePose);
					if(pt.z < height)
					{
						height = pt.z;
					}
				}
			}
		}

		if(grid_->GetPosition().y == kHeightOffset.y || grid_->GetPosition().y > height)
		{
			grid_->SetPosition(glm::vec3(0,height,0));
		}
		LOGD("compute min height %f s", time.ticks());
	}
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

void Scene::updateMesh(int id, const rtabmap::Mesh & mesh)
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

void Scene::setGridColor(float r, float g, float b)
{
	if(grid_)
	{
		grid_->SetColor(r, g, b);
	}
}
