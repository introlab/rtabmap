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

#include "tango-gl/camera.h"
#include "tango-gl/util.h"

namespace tango_gl {

Camera::Camera() {
  field_of_view_ = 45.0f * DEGREE_2_RADIANS;
  aspect_ratio_ = 4.0f / 3.0f;
  width_ = 800.0f;
  height_ = 600.0f;
  near_clip_plane_ = 0.2f;
  far_clip_plane_ = 1000.0f;
  ortho_ = false;
  orthoScale_ = 2.0f;
  orthoCropFactor_ = -1.0f;
}

glm::mat4 Camera::GetViewMatrix() {
  return glm::inverse(GetTransformationMatrix());
}

glm::mat4 Camera::GetProjectionMatrix() {
	if(ortho_)
	{
		return glm::ortho(-orthoScale_*aspect_ratio_, orthoScale_*aspect_ratio_, -orthoScale_, orthoScale_, orthoScale_ + orthoCropFactor_, far_clip_plane_);
	}
  return glm::perspective(field_of_view_, aspect_ratio_, near_clip_plane_, far_clip_plane_);
}

void Camera::SetWindowSize(float width, float height) {
	width_ = width;
	height_ = height;
  aspect_ratio_ = width/height;
}

void Camera::SetFieldOfView(float fov) {
  field_of_view_ = fov * DEGREE_2_RADIANS;
}

void Camera::SetNearFarClipPlanes(const float near, const float far)
{
	near_clip_plane_ = near;
	far_clip_plane_ = far;
}

Camera::~Camera() {
}

glm::mat4 Camera::ProjectionMatrixForCameraIntrinsics(float width, float height,
                                                      float fx, float fy,
                                                      float cx, float cy,
                                                      float near, float far) {
  const float xscale = near / fx;
  const float yscale = near / fy;

  const float xoffset =  (cx - (width  / 2.0)) * xscale;
  // Color camera's coordinates has y pointing downwards so we negate this term.
  const float yoffset = -(cy - (height / 2.0)) * yscale;

  return  glm::frustum(xscale * -width  / 2.0f - xoffset,
                       xscale *  width  / 2.0f - xoffset,
                       yscale * -height / 2.0f - yoffset,
                       yscale *  height / 2.0f - yoffset,
                       near, far);
}

}  // namespace tango_gl
