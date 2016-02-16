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

#include "tango-gl/conversions.h"

namespace tango_gl {
namespace conversions {

glm::mat4 opengl_world_T_tango_world() {
  // Note glm is column-wise.
  return glm::mat4(1.0f, 0.0f,  0.0f, 0.0f,
                   0.0f, 0.0f, -1.0f, 0.0f,
                   0.0f, 1.0f,  0.0f, 0.0f,
                   0.0f, 0.0f,  0.0f, 1.0f);
}

glm::mat4 color_camera_T_opengl_camera() {
  // Note glm is column-wise.
  return glm::mat4(1.0f,  0.0f,  0.0f, 0.0f,
                   0.0f, -1.0f,  0.0f, 0.0f,
                   0.0f,  0.0f, -1.0f, 0.0f,
                   0.0f,  0.0f,  0.0f, 1.0f);
}

glm::mat4 depth_camera_T_opengl_camera() {
  // Note glm is column-wise.
  return glm::mat4(1.0f, 0.0f, 0.0f, 0.0f, 0.0f, -1.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                   -1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f);
}

glm::quat QuatTangoToGl(const glm::quat& tango_q_frame) {
  const float kSqrt2Over2 = std::sqrt(2.0) / 2.0f;
  // Tango frame is a -90 degree rotation about +X from the GL frame.
  glm::quat gl_q_tango = glm::quat(kSqrt2Over2, -kSqrt2Over2, 0.0f, 0.0f);
  return gl_q_tango * tango_q_frame;
}

}  // namespace gl_tango_conversions
}  // namespace tango_gl
