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

#include "tango-gl/band.h"
#include "tango-gl/util.h"

namespace tango_gl {

// Set band resolution to 0.01m(1cm) when using UpdateVertexArray()
static const float kMinDistanceSquared = 0.0001f;

Band::Band(const unsigned int max_length)
    : band_width_(0.2), max_length_(max_length) {
  SetShader();
  vertices_v_.reserve(max_length);
  pivot_left = glm::vec3(0, 0, 0);
  pivot_right = glm::vec3(0, 0, 0);
}

void Band::SetWidth(const float width) {
  band_width_ = width;
}

void Band::UpdateVertexArray(const glm::mat4 m, BandMode mode) {
  // First 2 vertices of a band + 3 arrow head vertices.
  bool need_to_initialize = (vertices_v_.size() < 5);

  bool sufficient_delta = false;
  if (!need_to_initialize) {
    // Band head is the first two vertices after arrow head.
    glm::vec3 band_front = 0.5f * (vertices_v_[vertices_v_.size() - 4] +
                                   vertices_v_[vertices_v_.size() - 5]);
    sufficient_delta = kMinDistanceSquared <
        util::DistanceSquared(band_front, util::GetTranslationFromMatrix(m));
  }

  if (need_to_initialize || sufficient_delta) {
    glm::vec3 left  = glm::vec3(-band_width_ * 0.5f, 0, 0);
    glm::vec3 right = glm::vec3(band_width_ * 0.5f, 0, 0);
    glm::vec3 arrow_left = glm::vec3(-band_width_ * 0.75f, 0, 0);
    glm::vec3 arrow_right = glm::vec3(band_width_ * 0.75f, 0, 0);
    glm::vec3 arrow_front = glm::vec3(0, 0, -band_width_ * 0.75f);

    // If keep right pivot point, or normal mode,
    // then only update left pivot point.
    if (mode == BandMode::kNormal || mode == BandMode::kKeepRight) {
      pivot_left = util::ApplyTransform(m, left);
    }
    // If keep left pivot point, or normal mode,
    // then only update right pivot point.
    if (mode == BandMode::kNormal || mode == BandMode::kKeepLeft) {
      pivot_right = util::ApplyTransform(m, right);
    }

    glm::mat4 head_m = m;

    if (mode != BandMode::kNormal) {
      glm::vec3 up = glm::vec3(0, 1.0f, 0);
      glm::vec3 position = 0.5f * (pivot_left + pivot_right);
      glm::vec3 heading = glm::cross(up, pivot_right-pivot_left);
      head_m = glm::inverse(glm::lookAt(glm::vec3(0, 0, 0), heading, up));
      head_m[3][0] = position.x;
      head_m[3][1] = position.y;
      head_m[3][2] = position.z;
    }

    if (need_to_initialize) {
      vertices_v_.resize(5);
    } else {
      vertices_v_.resize(vertices_v_.size() + 2);
    }

    size_t insertion_start = vertices_v_.size() - 5;
    vertices_v_[insertion_start + 0] = pivot_left;
    vertices_v_[insertion_start + 1] = pivot_right;
    vertices_v_[insertion_start + 2] = util::ApplyTransform(head_m, arrow_left);
    vertices_v_[insertion_start + 3] = util::ApplyTransform(head_m, arrow_right);
    vertices_v_[insertion_start + 4] = util::ApplyTransform(head_m, arrow_front);

    if (vertices_v_.size() > max_length_) {
      vertices_v_.erase(vertices_v_.begin(), vertices_v_.begin() + 2);
    }
  }
}

void Band::UpdateVertexArray(const glm::mat4 m) {
  // Defualt to call update with normal mode.
  UpdateVertexArray(m, BandMode::kNormal);
}

void Band::SetVertexArray(const std::vector<glm::vec3>& v,
                          const glm::vec3& up) {
  vertices_v_.clear();
  vertices_v_.reserve(2 * v.size());
  if (v.size() < 2)
    return;

  for (size_t i = 0; i < v.size() - 1; ++i) {
    glm::vec3 gl_p_world_a = v[i];
    glm::vec3 gl_p_world_b = v[i + 1];
    glm::vec3 dir = glm::normalize(gl_p_world_b - gl_p_world_a);
    glm::vec3 left = glm::cross(up, dir);
    glm::normalize(left);

    vertices_v_.push_back(gl_p_world_a + (band_width_ / 2.0f * left));
    vertices_v_.push_back(gl_p_world_a - (band_width_ / 2.0f * left));

    // Cap the end of the path.
    if (i == v.size() - 2) {
      vertices_v_.push_back(gl_p_world_b + (band_width_ / 2.0f * left));
      vertices_v_.push_back(gl_p_world_b - (band_width_ / 2.0f * left));
    }
  }

}

void Band::ClearVertexArray() { vertices_v_.clear(); }

void Band::Render(const glm::mat4& projection_mat,
                  const glm::mat4& view_mat) const {
  glUseProgram(shader_program_);
  glm::mat4 model_mat = GetTransformationMatrix();
  glm::mat4 mvp_mat = projection_mat * view_mat * model_mat;
  glUniformMatrix4fv(uniform_mvp_mat_, 1, GL_FALSE, glm::value_ptr(mvp_mat));

  glUniform4f(uniform_color_, red_, green_, blue_, alpha_);

  glEnableVertexAttribArray(attrib_vertices_);
  glVertexAttribPointer(attrib_vertices_, 3, GL_FLOAT, GL_FALSE,
                        sizeof(glm::vec3), &vertices_v_[0]);
  glDrawArrays(GL_TRIANGLE_STRIP, 0, vertices_v_.size());
  glDisableVertexAttribArray(attrib_vertices_);
  glUseProgram(0);
}

}  // namespace tango_gl
