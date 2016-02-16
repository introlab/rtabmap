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

#ifndef TANGO_GL_BOUNDING_BOX_H_
#define TANGO_GL_BOUNDING_BOX_H_

#include <vector>

#include "tango-gl/segment.h"
#include "tango-gl/util.h"

namespace tango_gl {
class BoundingBox {
 public:
  BoundingBox()
      : bounding_min_(glm::vec3(0, 0, 0)), bounding_max_(glm::vec3(0, 0, 0)) {}
  BoundingBox(const std::vector<float>& vertices);
  BoundingBox(const glm::vec3& min, const glm::vec3& max)
      : bounding_min_(min), bounding_max_(max) {}
  bool IsIntersecting(const Segment& segment, const glm::quat& rotation,
                      const glm::mat4& transformation);

 private:
  // Axis-aligned bounding box minimum and maximum point.
  glm::vec3 bounding_min_;
  glm::vec3 bounding_max_;
};
}  // namespace tango_gl
#endif  // TANGO_GL_BOUNDING_BOX_H_
