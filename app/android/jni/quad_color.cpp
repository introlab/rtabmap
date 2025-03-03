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

#include "quad_color.h"
#include "tango-gl/util.h"

static const float vertices[] = {-1.0f, -1.0f, 1.0f, -1.0f,
                                 -1.0f, 1.0f,  1.0f, 1.0f};

QuadColor::QuadColor(float size) {
	SetShader();

	vertices_.resize(8);
	for(int i=0; i<8; ++i)
	{
		vertices_[i] = vertices[i]*size;
	}
}

QuadColor::QuadColor(
		float widthLeft,
		float widthRight,
		float heightBottom,
		float heightTop) {
	SetShader();

	vertices_.resize(8);
	vertices_[0] = vertices[0]*widthLeft;
	vertices_[1] = vertices[1]*heightBottom;
	vertices_[2] = vertices[2]*widthRight;
	vertices_[3] = vertices[3]*heightBottom;
	vertices_[4] = vertices[4]*widthLeft;
	vertices_[5] = vertices[5]*heightTop;
	vertices_[6] = vertices[6]*widthRight;
	vertices_[7] = vertices[7]*heightTop;
}

void QuadColor::Render(const glm::mat4& projection_mat,
                  const glm::mat4& view_mat) const {
  glUseProgram(shader_program_);

  // Calculate MVP matrix and pass it to shader.
  glm::mat4 model_mat = GetTransformationMatrix();
  glm::mat4 mvp_mat = projection_mat * view_mat * model_mat;
  glUniformMatrix4fv(uniform_mvp_mat_, 1, GL_FALSE, glm::value_ptr(mvp_mat));

  glUniform4f(uniform_color_, red_, green_, blue_, alpha_);

  // Vertice binding
  glEnableVertexAttribArray(attrib_vertices_);
  glVertexAttribPointer(attrib_vertices_, 2, GL_FLOAT, GL_FALSE, 0, &vertices_[0]);
  glBindBuffer(GL_ARRAY_BUFFER, 0);

  glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
  glUseProgram(0);
}
