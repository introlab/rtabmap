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

#ifndef TANGO_GL_GESTURE_CAMERA_H_
#define TANGO_GL_GESTURE_CAMERA_H_

#include "tango-gl/camera.h"
#include "tango-gl/segment.h"
#include "tango-gl/transform.h"
#include "tango-gl/util.h"

namespace tango_gl {
class GestureCamera : public Camera {
 public:
  enum CameraType {
    kFirstPerson = 0,
    kThirdPersonFollow = 1,
    kTopDown = 2,
	kTopOrtho = 3,
    kThirdPerson = 4
  };

  enum TouchEvent {
    kTouch0Down = 0,
    kTouch0Up = 1,
    kTouchMove = 2,
    kTouch1Down = 5,
    kTouch1Up = 6,
    kTouchNone = -1
  };

  GestureCamera();
  ~GestureCamera();

  void OnTouchEvent(int touch_count, TouchEvent event, float x0, float y0,
                    float x1, float y1);

  // Get the ray in opengl world frame given the 2d touch position on screen,
  // normalized touch_x and normalized touch_y should be the same value get from
  // OnTouchEvent, x0 and y0, touch_range is the depth of the touch in
  // camera frame.
  Segment GetSegmentFromTouch(float normalized_x, float normalized_y,
                              float touch_range);

  void SetAnchorPosition(const glm::vec3& pos, const glm::quat & rotation);
  void SetAnchorOffset(const glm::vec3& pos) {anchor_offset_ = pos;}
  const glm::vec3& GetAnchorOffset() const {return anchor_offset_;}

  void SetCameraDistance(float cameraDistance) {cam_cur_dist_ = cameraDistance;}
  float GetCameraDistance() const {return cam_cur_dist_;}

  // Set camera type, set render camera's parent position and rotation.
  void SetCameraType(CameraType camera_index);

  CameraType GetCameraType() const { return camera_type_; }
  float getFOV() const {return field_of_view_ * RADIAN_2_DEGREE;}

 private:
  void StartCameraToCurrentTransform();

  // Render camera's parent transformation.
  Transform* cam_parent_transform_;

  CameraType camera_type_;

  glm::vec2 cam_start_angle_;
  glm::vec2 cam_cur_angle_;
  glm::quat cam_cur_target_rot_;

  float cam_start_dist_;
  float cam_start_fov_;
  float cam_cur_dist_;
  glm::vec3 anchor_offset_;

  float start_touch_dist_;
  float cur_touch_dist_;

  glm::vec2 touch0_start_position_;
};
}  // namespace tango_gl
#endif  // TANGO_GL_GESTURE_CAMERA_H_
