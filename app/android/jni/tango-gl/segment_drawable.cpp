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

#include "tango-gl/segment_drawable.h"

namespace tango_gl {
SegmentDrawable::SegmentDrawable() : tango_gl::Line(5.0f, GL_LINES) {
  SetShader();
  vec_vertices_.push_back(glm::vec3(0, 0, 0));
  vec_vertices_.push_back(glm::vec3(1.f, 1.f, 1.f));
}
void SegmentDrawable::UpdateSegment(const Segment& segment) {
  vec_vertices_[0] = segment.start;
  vec_vertices_[1] = segment.end;
}
}  // namespace tango_gl
