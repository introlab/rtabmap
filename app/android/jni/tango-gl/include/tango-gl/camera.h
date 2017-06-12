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

#ifndef TANGO_GL_CAMERA_H_
#define TANGO_GL_CAMERA_H_

#include "tango-gl/transform.h"

namespace tango_gl {
class Camera : public Transform {
 public:
  Camera();
  Camera(const Camera& other) = delete;
  Camera& operator=(const Camera&) = delete;
  ~Camera();

  void SetWindowSize(const float width, const float height);
  void SetFieldOfView(const float fov);
  void SetOrthoMode(bool enabled) {ortho_ = enabled;}
  void SetOrthoScale(float scale) {orthoScale_ = scale;}
  void SetOrthoCropFactor(float value) {orthoCropFactor_ = value;}
  void SetNearFarClipPlanes(const float near, const float far);

  glm::mat4 GetViewMatrix();
  glm::mat4 GetProjectionMatrix();
  float getNearClipPlane() const {return near_clip_plane_;}
  float getFarClipPlane() const {return far_clip_plane_;}

  /**
   * Create an OpenGL perspective matrix from window size, camera intrinsics, and clip settings.
   *
   * @param width  - The width of the camera image.
   * @param height - The height of the camera image.
   * @param fx     - The x-axis focal length of the camera.
   * @param fy     - The y-axis focal length of the camera.
   * @param cx     - The x-coordinate principal point in pixels.
   * @param cy     - The y-coordinate principal point in pixels.
   * @param near   - The desired near z-clipping plane.
   * @param far    - The desired far z-clipping plane.
   */
  static glm::mat4 ProjectionMatrixForCameraIntrinsics(float width, float height,
                                                       float fx,    float fy,
                                                       float cx,    float cy,
                                                       float near,  float far);
 protected:
  float field_of_view_;
  float aspect_ratio_;
  float width_;
  float height_;
  float near_clip_plane_, far_clip_plane_;
  bool ortho_;
  float orthoScale_;
  float orthoCropFactor_;
};
}  // namespace tango_gl
#endif  // TANGO_GL_CAMERA_H_
