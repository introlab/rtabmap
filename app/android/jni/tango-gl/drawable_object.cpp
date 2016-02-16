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

#include "tango-gl/drawable_object.h"
#include "tango-gl/shaders.h"

namespace tango_gl {

void DrawableObject::SetShader() {
  shader_program_ =
      util::CreateProgram(shaders::GetBasicVertexShader().c_str(),
                          shaders::GetBasicFragmentShader().c_str());
  if (!shader_program_) {
    LOGE("Could not create program.");
  }
  uniform_mvp_mat_ = glGetUniformLocation(shader_program_, "mvp");
  attrib_vertices_ = glGetAttribLocation(shader_program_, "vertex");
  uniform_color_ = glGetUniformLocation(shader_program_, "color");
}

void DrawableObject::DeleteGlResources() {
  if (shader_program_) {
    glDeleteShader(shader_program_);
  }
}

void DrawableObject::SetColor(float red, float green, float blue) {
  red_ = red;
  green_ = green;
  blue_ = blue;
}
void DrawableObject::SetColor(const Color& color) {
  SetColor(color.r, color.g, color.b);
}

void DrawableObject::SetAlpha(const float alpha) { alpha_ = alpha; }

void DrawableObject::SetVertices(const std::vector<GLfloat>& vertices) {
  vertices_ = vertices;
}

void DrawableObject::SetVertices(const std::vector<GLfloat>& vertices,
                                 const std::vector<GLushort>& indices) {
  vertices_ = vertices;
  indices_ = indices;
}

void DrawableObject::SetVertices(const std::vector<GLfloat>& vertices,
                                 const std::vector<GLfloat>& normals) {
  vertices_ = vertices;
  normals_ = normals;
}
}  // namespace tango_gl
