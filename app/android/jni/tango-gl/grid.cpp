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

#include "tango-gl/grid.h"

namespace tango_gl {

// Initialize Grid with x and y grid count,
// qx, quantity in x
// qy, quantity in y.
Grid::Grid(float density, int qx, int qy) : Line(1.0f, GL_LINES) {
  SetShader();

  // 3 float in 1 vertex, 2 vertices form a line.
  // Horizontal line and vertical line forms the grid.
  float width = density * qx / 2;
  float height = density * qy / 2;

  // Horizontal line.
  for (int i = 0; i < (qy + 1); i++) {
    vec_vertices_.push_back(glm::vec3(-width, 0.0f, -height + i * density));
    vec_vertices_.push_back(glm::vec3(width, 0.0f, -height + i * density));
  }

  for (int i = 0; i < (qx + 1); i++) {
    vec_vertices_.push_back(glm::vec3(-width + i * density, 0.0f, -height));
    vec_vertices_.push_back(glm::vec3(-width + i * density, 0.0f, height));
  }
}
}  // namespace tango_gl
