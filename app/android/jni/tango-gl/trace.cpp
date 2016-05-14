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

#include "tango-gl/trace.h"

namespace tango_gl {

static const int kMaxTraceLength = 5000;
static const float kDistanceCheck = 0.05f;

Trace::Trace() : Line(3.0f, GL_LINE_STRIP) { SetShader(); }

void Trace::UpdateVertexArray(const glm::vec3& v) {
  if (vec_vertices_.size() == 0) {
    vec_vertices_.push_back(v);
  } else {
    float dist = glm::distance(vec_vertices_[vec_vertices_.size() - 1], v);
    if (dist >= kDistanceCheck) {
      vec_vertices_.push_back(v);
    }
  }
}

void Trace::ClearVertexArray() { vec_vertices_.clear(); }
}  // namespace tango_gl
