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

#ifndef TANGO_GL_BAND_H_
#define TANGO_GL_BAND_H_

#include <stdlib.h>
#include <vector>

#include "tango-gl/drawable_object.h"

namespace tango_gl {
class Band : public DrawableObject {
 public:
  enum BandMode {
    kNormal = 0,
    kKeepLeft = 1,
    kKeepRight = 2
  };

  Band(const unsigned int max_legnth);

  void SetWidth(const float width);
  // Render a Band with arrow head, pass in the mode for rendering,
  // kKeepLeft is left turn, kKeepRight is right turn,
  // when making a turn, vertices only get updated in one side to avoid overlapping.
  void UpdateVertexArray(const glm::mat4 m, BandMode mode);
  void UpdateVertexArray(const glm::mat4 m);
  void SetVertexArray(const std::vector<glm::vec3>& v, const glm::vec3& up);
  void ClearVertexArray();
  void Render(const glm::mat4& projection_mat, const glm::mat4& view_mat) const;

 private:
  float band_width_;
  unsigned int max_length_;
  std::vector<glm::vec3> vertices_v_;
  // Current band head's left and right position in world frame.
  glm::vec3 pivot_left;
  glm::vec3 pivot_right;
};
}  // namespace tango_gl
#endif  // TANGO_GL_BAND_H_
