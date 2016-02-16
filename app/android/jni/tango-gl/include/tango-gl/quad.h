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

#ifndef TANGO_GL_QUAD_H_
#define TANGO_GL_QUAD_H_

#include "tango-gl/drawable_object.h"

namespace tango_gl {
class Quad : public DrawableObject {
 public:
  Quad();
  Quad(const Quad& other) = delete;
  Quad& operator=(const Quad&) = delete;
  ~Quad();

  void Render(const glm::mat4& projection_mat, const glm::mat4& view_mat) const;
  void SetTextureId(GLuint texture_id);

 private:
  GLuint vertex_buffer_;
  GLuint shader_program_;
  GLuint attrib_vertices_;
  GLuint texture_coords_;
  GLuint texture_handle;
  GLuint uniform_mvp_mat_;

  GLuint texture_id_;
};
}  // namespace tango_gl
#endif  // TANGO_GL_QUAD_H_
