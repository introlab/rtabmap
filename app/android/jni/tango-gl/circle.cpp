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

#include "tango-gl/circle.h"

namespace tango_gl {
Circle::Circle(float radius, int resolution) : Mesh(GL_TRIANGLE_FAN){
  SetShader();
  std::vector<GLfloat> vertices;
  vertices.reserve(3 * (resolution + 2));
  vertices.push_back(0);
  vertices.push_back(0);
  vertices.push_back(0);
  float delta_theta = M_PI * 2.0f / static_cast<float>(resolution);
  for (int i = resolution; i >= 0; i--) {
    float theta = delta_theta * static_cast<float>(i);
    vertices.push_back(cos(theta) * radius);
    vertices.push_back(0);
    vertices.push_back(sin(theta) * radius);
  }
  SetVertices(vertices);
}
}  // namespace tango_gl
