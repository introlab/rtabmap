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

#ifndef TANGO_GL_LINE_H_
#define TANGO_GL_LINE_H_

#include "tango-gl/drawable_object.h"

namespace tango_gl {
class Line : public DrawableObject {
 public:
  Line(float line_width, GLenum render_mode);
  void SetLineWidth(const float pixels);
  void Render(const glm::mat4& projection_mat, const glm::mat4& view_mat) const;
  void UpdateLineVertices(const std::vector<glm::vec3>& vec_vertices) {
    vec_vertices_ = vec_vertices;
  }

 protected:
  float line_width_;
  std::vector<glm::vec3> vec_vertices_;
};
}  // namespace tango_gl
#endif  // TANGO_GL_LINE_H_