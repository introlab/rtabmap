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

#include "tango-gl/mesh.h"
#include "tango-gl/shaders.h"

namespace tango_gl {
Mesh::Mesh() {
  render_mode_ = GL_TRIANGLES;
}
Mesh::Mesh(GLenum render_mode) {
  render_mode_ = render_mode;
}

void Mesh::SetShader() {
  DrawableObject::SetShader();
  // Default mode set to no lighting.
  is_lighting_on_ = false;
  // Default mode set to without bounding box detection.
  is_bounding_box_on_ = false;
}

void Mesh::SetShader(bool is_lighting_on) {
  if (is_lighting_on) {
    shader_program_ =
        util::CreateProgram(shaders::GetShadedVertexShader().c_str(),
                            shaders::GetBasicFragmentShader().c_str());
    if (!shader_program_) {
      LOGE("Could not create program.");
    }
    uniform_mvp_mat_ = glGetUniformLocation(shader_program_, "mvp");
    uniform_mv_mat_ = glGetUniformLocation(shader_program_, "mv");
    uniform_light_vec_ = glGetUniformLocation(shader_program_, "lightVec");
    uniform_color_ = glGetUniformLocation(shader_program_, "color");

    attrib_vertices_ = glGetAttribLocation(shader_program_, "vertex");
    attrib_normals_ = glGetAttribLocation(shader_program_, "normal");
    is_lighting_on_ = true;
    // Set a defualt direction for directional light.
    light_direction_ = glm::vec3(-1.0f, -3.0f, -1.0f);
    light_direction_ = glm::normalize(light_direction_);
  } else {
    SetShader();
  }
}

void Mesh::SetBoundingBox(){
  // Traverse all the vertices to define an axis-aligned
  // bounding box for this mesh, needs to be called after SetVertices().
  if(vertices_.size()==0){
    LOGE("Please set up vertices first!");
    return;
  }
  is_bounding_box_on_ = true;
  bounding_box_ = new BoundingBox(vertices_);
}

void Mesh::SetLightDirection(const glm::vec3& light_direction) {
  light_direction_ = light_direction;
}

bool Mesh::IsIntersecting(const Segment& segment) {
  // If there is no bounding box defined based on all vertices,
  // we can not calculate intersection.
  if (!is_bounding_box_on_) {
    LOGE("Mesh::IsIntersecting, bounding box is not available.");
    return false;
  }
  return bounding_box_->IsIntersecting(segment, GetRotation(),
                                       GetTransformationMatrix());
}

void Mesh::Render(const glm::mat4& projection_mat,
                  const glm::mat4& view_mat) const {
  glUseProgram(shader_program_);
  glm::mat4 model_mat = GetTransformationMatrix();
  glm::mat4 mv_mat = view_mat * model_mat;
  glm::mat4 mvp_mat = projection_mat * mv_mat;
  glUniformMatrix4fv(uniform_mvp_mat_, 1, GL_FALSE, glm::value_ptr(mvp_mat));
  glUniform4f(uniform_color_, red_, green_, blue_, alpha_);

  if (is_lighting_on_) {
    glUniformMatrix4fv(uniform_mv_mat_, 1, GL_FALSE, glm::value_ptr(mv_mat));

    glEnableVertexAttribArray(attrib_normals_);
    glVertexAttribPointer(attrib_normals_, 3, GL_FLOAT, GL_FALSE,
                          3 * sizeof(GLfloat), &normals_[0]);
    glm::vec3 light_direction = glm::mat3(view_mat) * light_direction_;
    glUniform3fv(uniform_light_vec_, 1, glm::value_ptr(light_direction));
  }

  glEnableVertexAttribArray(attrib_vertices_);

  if (!indices_.empty()) {
    glVertexAttribPointer(attrib_vertices_, 3, GL_FLOAT, GL_FALSE,
                          3 * sizeof(GLfloat), vertices_.data());
    glDrawElements(render_mode_, indices_.size(), GL_UNSIGNED_SHORT,
                   indices_.data());
  } else {
    glVertexAttribPointer(attrib_vertices_, 3, GL_FLOAT, GL_FALSE,
                          3 * sizeof(GLfloat), &vertices_[0]);
    glDrawArrays(render_mode_, 0, vertices_.size() / 3);
  }

  glDisableVertexAttribArray(attrib_vertices_);
  if (is_lighting_on_) {
    glDisableVertexAttribArray(attrib_normals_);
  }
  glUseProgram(0);
}
}  // namespace tango_gl
