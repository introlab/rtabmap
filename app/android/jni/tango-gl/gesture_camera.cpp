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

#include "tango-gl/gesture_camera.h"
#include "tango-gl/util.h"
#include "glm/gtx/quaternion.hpp"

namespace {
// Render camera observation distance in third person camera mode.
const float kThirdPersonCameraDist = 7.0f;

// Render camera observation distance in third person camera mode.
const float kThirdPersonFollowCameraDist = 2.0f;

// Render camera observation distance in top down camera mode.
const float kTopDownCameraDist = 5.0f;

// Zoom in speed.
const float kZoomSpeed = 10.0f;

// Move speed
const float kMoveSpeed = 10.0f;

// Rotation speed
const float kRotationSpeed = 2.0f;

// Min/max clamp value of camera observation distance.
const float kCamViewMinDist = .1f;
const float kCamViewMaxDist = 100.f;

// FOV set up values.
// Third and top down camera's FOV is 65 degrees.
// First person camera's FOV is 85 degrees.
const float kHighestFov = 120.0f;
const float kHighFov = 85.0f;
const float kLowFov = 65.0f;
const float kLowestFov = 40.0f;
}

namespace tango_gl {

GestureCamera::GestureCamera() :
		cam_cur_target_rot_(1,0,0,0),
		start_touch_dist_(0.0f),
		cur_touch_dist_(0.0f)
{
  cam_parent_transform_ = new Transform();
  SetParent(cam_parent_transform_);
}

GestureCamera::~GestureCamera() { delete cam_parent_transform_; }

void GestureCamera::OnTouchEvent(int touch_count, TouchEvent event, float x0,
		float y0, float x1, float y1) {

	if (camera_type_!=kFirstPerson && touch_count == 1) {
		switch (event) {
		case kTouch0Down: {
			cam_start_angle_ = cam_cur_angle_;

			touch0_start_position_.x = x0;
			touch0_start_position_.y = y0;
			break;
		}
		case kTouchMove: {
			glm::vec2 offset;

			float rotation_x = (touch0_start_position_.y - y0) * kRotationSpeed;
			float rotation_y = (touch0_start_position_.x - x0) * kRotationSpeed;

			if(camera_type_!=kTopOrtho)
				cam_cur_angle_.x = cam_start_angle_.x + rotation_x;
			cam_cur_angle_.y = cam_start_angle_.y + rotation_y;

			StartCameraToCurrentTransform();

			break;
		}
		default: { break; }
		}
	}
	if (touch_count == 2) {
		switch (event) {
		case kTouch1Down: {
			float abs_x = x0 - x1;
			float abs_y = y0 - y1;
			start_touch_dist_ = std::sqrt(abs_x * abs_x + abs_y * abs_y);
			cam_start_dist_ = GetPosition().z;
			cam_start_fov_ = this->getFOV();

			// center touch
			touch0_start_position_.x = (x0+x1)/2.0f;
			touch0_start_position_.y = (y0+y1)/2.0f;
			break;
		}
		case kTouchMove: {
			float abs_x = x0 - x1;
			float abs_y = y0 - y1;
			float dist = start_touch_dist_ - std::sqrt(abs_x * abs_x + abs_y * abs_y);

			if(camera_type_ == kFirstPerson)
			{
				this->SetFieldOfView(tango_gl::util::Clamp(cam_start_fov_ + dist * kZoomSpeed*10.0f, kLowestFov, kHighestFov));
			}
			else
			{
				cam_cur_dist_ = tango_gl::util::Clamp(cam_start_dist_ + dist * kZoomSpeed,
								kCamViewMinDist, kCamViewMaxDist);

				this->SetOrthoMode(camera_type_ == kTopOrtho);
				if(camera_type_ == kTopOrtho)
				{
					this->SetOrthoScale(cam_cur_dist_);
				}

				glm::vec2 touch_center_position((x0+x1)/2.0f, (y0+y1)/2.0f);
				glm::vec2 offset;
				offset.x = (touch_center_position.x - touch0_start_position_.x) * kMoveSpeed;
				offset.y = (touch_center_position.y - touch0_start_position_.y) * kMoveSpeed;
				touch0_start_position_ = touch_center_position;

				StartCameraToCurrentTransform();

				anchor_offset_ += glm::rotate(cam_parent_transform_->GetRotation(), glm::vec3(-offset.x, offset.y, 0));
			}
			break;
		}
		default: { break; }
		}
	}
}

Segment GestureCamera::GetSegmentFromTouch(float normalized_x,
                                           float normalized_y,
                                           float touch_range) {
  float screen_height = touch_range * (2.0f * glm::tan(field_of_view_ * 0.5f));
  float screen_width = screen_height * aspect_ratio_;
  // normalized_x and normalized_x are from OnTouchEvent, top-left corner of the
  // screen
  // is [0, 0], transform it to opengl frame.
  normalized_x = normalized_x - 0.5f;
  normalized_y = 0.5f - normalized_y;
  glm::vec3 start =
      util::ApplyTransform(GetTransformationMatrix(), glm::vec3(0, 0, 0));
  glm::vec3 end = util::ApplyTransform(GetTransformationMatrix(), 
      glm::vec3(normalized_x * screen_width, normalized_y * screen_height,
                -touch_range));
  Segment segment(start, end);
  return segment;
}

void GestureCamera::SetAnchorPosition(const glm::vec3& pos, const glm::quat & rotation) {
	// Anchor position
	cam_parent_transform_->SetPosition(pos+anchor_offset_);

	// Anchor rotation
	if(camera_type_ == kThirdPersonFollow)
	{
		cam_cur_target_rot_ = rotation;
		cam_cur_target_rot_.x = 0;
		cam_cur_target_rot_.z = 0;
		cam_cur_target_rot_ = glm::normalize(cam_cur_target_rot_);
		StartCameraToCurrentTransform();
	}
}

void GestureCamera::SetCameraType(CameraType camera_index) {
  camera_type_ = camera_index;
  switch (camera_index) {
    case kFirstPerson:
      SetOrthoMode(false);
      SetFieldOfView(kLowestFov);
      SetNearFarClipPlanes(0.25, 25);
      SetPosition(glm::vec3(0.0f, 0.0f, 0.0f));
      SetRotation(glm::quat(1.0f, 0.0f, 0.0f, 0.0f));
      cam_cur_dist_ = 0.0f;
      anchor_offset_ = glm::vec3(0.0f,0.0f,0.0f);
      cam_cur_angle_.x = 0.0f;
      cam_cur_angle_.y = 0.0f;
      cam_cur_target_rot_ = glm::quat(1,0,0,0);
      cam_parent_transform_->SetPosition(glm::vec3(0.0f, 0.0f, 0.0f));
      StartCameraToCurrentTransform();
      break;
    case kThirdPerson:
    case kThirdPersonFollow:
      SetOrthoMode(false);
      SetFieldOfView(kLowFov);
      SetNearFarClipPlanes(1, 50);
      SetPosition(glm::vec3(0.0f, 0.0f, 0.0f));
      SetRotation(glm::quat(1.0f, 0.0f, 0.0f, 0.0f));
      cam_cur_dist_ = camera_index==kThirdPersonFollow?kThirdPersonFollowCameraDist:kThirdPersonCameraDist;
      anchor_offset_ = glm::vec3(0.0f,0.0f,0.0f);
      cam_cur_angle_.x = -M_PI / 12.0f;
      cam_cur_angle_.y = kThirdPersonFollow?0:M_PI / 2.0f;
      cam_cur_target_rot_ = glm::quat(1,0,0,0);
      StartCameraToCurrentTransform();
      break;
    case kTopDown:
      SetPosition(glm::vec3(0.0f, 0.0f, 0.0f));
      SetRotation(glm::quat(1.0f, 0.0f, 0.0f, 0.0f));
      SetOrthoMode(false);
      SetFieldOfView(kLowFov);
      SetNearFarClipPlanes(1, 50);
      cam_cur_dist_ = kTopDownCameraDist;
      anchor_offset_ = glm::vec3(0.0f,0.0f,0.0f);
      cam_cur_angle_.x = -M_PI / 2.0f;
      cam_cur_angle_.y = 0.0f;
      cam_cur_target_rot_ = glm::quat(1,0,0,0);
      StartCameraToCurrentTransform();
      break;
    case kTopOrtho:
	  SetPosition(glm::vec3(0.0f, 0.0f, 0.0f));
	  SetRotation(glm::quat(1.0f, 0.0f, 0.0f, 0.0f));
	  SetOrthoMode(true);
	  SetOrthoScale(kTopDownCameraDist);
	  SetOrthoCropFactor(-1.0f);
      SetFieldOfView(kLowFov);
      SetNearFarClipPlanes(1, 50);
	  cam_cur_dist_ = kTopDownCameraDist;
	  anchor_offset_ = glm::vec3(0.0f,0.0f,0.0f);
	  cam_cur_angle_.x = -M_PI / 2.0f;
	  cam_cur_angle_.y = 0.0f;
	  cam_cur_target_rot_ = glm::quat(1,0,0,0);
	  StartCameraToCurrentTransform();
	  break;
    default:
      break;
  }
}

void GestureCamera::StartCameraToCurrentTransform()
{
  //Anchor rotation
  glm::quat parent_cam_rot = glm::rotate(cam_cur_target_rot_, cam_cur_angle_.y, glm::vec3(0, 1, 0));
  parent_cam_rot = glm::rotate(parent_cam_rot, cam_cur_angle_.x, glm::vec3(1, 0, 0));
  cam_parent_transform_->SetRotation(parent_cam_rot);

  //Camera position
  SetPosition(glm::vec3(0, 0, cam_cur_dist_));
}
}  // namespace tango_gl
