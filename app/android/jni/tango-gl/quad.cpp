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

#include "tango-gl/quad.h"
#include "tango-gl/util.h"

namespace tango_gl {

static const char kVertexShader[] =
    "attribute vec4 vertex;\n"
    "attribute vec2 inputTextureCoordinate;\n"
    "varying vec2 textureCoordinate;\n"
    "uniform mat4 mvp;\n"
    "void main() {\n"
    "  gl_Position = mvp*vertex;\n"
    "  textureCoordinate = inputTextureCoordinate.xy;\n"
    "}\n";

static const char kFragmentShader[] =
    "varying vec2 textureCoordinate;\n"
    "uniform sampler2D inputTexture;\n"
    "void main() {\n"
    "  gl_FragColor = texture2D(inputTexture, textureCoordinate);\n"
    "}\n";

static const float vertices[] = {-0.5f, -0.5f, 0.5f, -0.5f,
                                 -0.5f, 0.5f,  0.5f, 0.5f};

static const GLfloat texture_coords[] = {0.0f, 1.0f, 1.0f, 1.0f,
                                         0.0f, 0.0f, 1.0f, 0.0f, };

Quad::Quad() {
  shader_program_ = util::CreateProgram(kVertexShader, kFragmentShader);
  if (!shader_program_) {
    LOGE("Could not create program.");
  }
  uniform_mvp_mat_ = glGetUniformLocation(shader_program_, "mvp");
  attrib_vertices_ = glGetAttribLocation(shader_program_, "vertex");
  texture_coords_ =
      glGetAttribLocation(shader_program_, "inputTextureCoordinate");
  texture_handle = glGetUniformLocation(shader_program_, "inputTexture");
  glGenBuffers(1, &vertex_buffer_);
}

Quad::~Quad() { glDeleteShader(shader_program_); }

void Quat::SetTextureId(GLuint texture_id) { texture_id_ = texture_id; }

void Quad::Render(const glm::mat4& projection_mat,
                  const glm::mat4& view_mat) const {
  glEnable(GL_CULL_FACE);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glUseProgram(shader_program_);

  glBindTexture(GL_TEXTURE_2D, texture_id_);
  glUniform1i(texture_handle, 0);

  // Calculate MVP matrix and pass it to shader.
  glm::mat4 model_mat = GetTransformationMatrix();
  glm::mat4 mvp_mat = projection_mat * view_mat * model_mat;
  glUniformMatrix4fv(uniform_mvp_mat_, 1, GL_FALSE, glm::value_ptr(mvp_mat));

  // Vertice binding
  glEnableVertexAttribArray(attrib_vertices_);
  glVertexAttribPointer(attrib_vertices_, 2, GL_FLOAT, GL_FALSE, 0, vertices);
  glBindBuffer(GL_ARRAY_BUFFER, 0);

  glEnableVertexAttribArray(texture_coords_);
  glVertexAttribPointer(texture_coords_, 2, GL_FLOAT, GL_FALSE, 0,
                        texture_coords);

  glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
  glUseProgram(0);
  glBindTexture(GL_TEXTURE_2D, 0);
}

}  // namespace tango_gl
