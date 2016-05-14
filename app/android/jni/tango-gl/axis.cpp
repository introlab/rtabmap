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

#include "tango-gl/axis.h"
#include "tango-gl/shaders.h"

namespace tango_gl {

static const float float_vertices[] = {0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
                                       0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f,
                                       0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f};

static const float float_colors[] = {
    1.0f, 0.0f, 0.0f, 1.0f, 1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f, 1.0f,
    0.0f, 1.0f, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f, 1.0f, 0.0f, 0.0f, 1.0f, 1.0f};

Axis::Axis() : Line(3.0f, GL_LINES) {
  // Implement SetShader here, not using the dedault one.
  shader_program_ =
      util::CreateProgram(shaders::GetColorVertexShader().c_str(),
                          shaders::GetBasicFragmentShader().c_str());
  if (!shader_program_) {
    LOGE("Could not create program.");
  }
  uniform_mvp_mat_ = glGetUniformLocation(shader_program_, "mvp");
  attrib_colors_ = glGetAttribLocation(shader_program_, "color");
  attrib_vertices_ = glGetAttribLocation(shader_program_, "vertex");

  size_t size = sizeof(float_vertices) / (sizeof(float) * 3);
  for (size_t i = 0; i < size; i++) {
    vec_vertices_.push_back(glm::vec3(float_vertices[i * 3],
                                      float_vertices[i * 3 + 1],
                                      float_vertices[i * 3 + 2]));
    vec_colors_.push_back(
        glm::vec4(float_colors[i * 4], float_colors[i * 4 + 1],
                  float_colors[i * 4 + 2], float_colors[i * 4 + 3]));
  }
}

void Axis::Render(const glm::mat4& projection_mat,
                  const glm::mat4& view_mat) const {
  glUseProgram(shader_program_);
  glLineWidth(line_width_);
  glm::mat4 model_mat = GetTransformationMatrix();
  glm::mat4 mvp_mat = projection_mat * view_mat * model_mat;
  glUniformMatrix4fv(uniform_mvp_mat_, 1, GL_FALSE, glm::value_ptr(mvp_mat));

  glEnableVertexAttribArray(attrib_vertices_);
  glVertexAttribPointer(attrib_vertices_, 3, GL_FLOAT, GL_FALSE,
                        sizeof(glm::vec3), &vec_vertices_[0]);

  glEnableVertexAttribArray(attrib_colors_);
  glVertexAttribPointer(attrib_colors_, 4, GL_FLOAT, GL_FALSE,
                        sizeof(glm::vec4), &vec_colors_[0]);

  glDrawArrays(render_mode_, 0, vec_vertices_.size());

  glDisableVertexAttribArray(attrib_vertices_);
  glDisableVertexAttribArray(attrib_colors_);
  glUseProgram(0);
}
}  // namespace tango_gl
