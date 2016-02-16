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

#ifndef TANGO_GL_OBJ_LOADER_H
#define TANGO_GL_OBJ_LOADER_H

#include <vector>

#include "tango-gl/util.h"

namespace tango_gl {
namespace obj_loader {
//  Load standard .obj file into vertices, indices or normals vectors,
//  OBJ file can be exported from 3D tools like 3ds Max or Blender.
//  A readable file with only vertices should look like
//  "v 1.00 2.00 3.00
//   ...
//   f 1 2 3
//   f 1 2 3 4
//   ..."
//
//  If exported with normals, file should look like
//  "v 1.00 2.00 3.00
//   ...
//   f 1//1 2//3 3//4
//   f 1//1 2//3 3//4 4//6
//   ...
//   vn 1.00 2.00 3.00
//   ..."
//  this can be used with Mesh:
//
//  std::vector<GLfloat> vertices;
//  std::vector<GLushort> indices;
//  std::vector<GLfloat> normals;
//  tango_gl::obj_loader::LoadOBJData("/sdcard/model.obj", vertices, indices);
//  mesh->SetVertices(vertices, indices);
//  or
//  tango_gl::obj_loader::LoadOBJData("/sdcard/model.obj", vertices, normals);
//  mesh->SetVertices(vertices, normals);

bool LoadOBJData(const char* path, std::vector<GLfloat>& vertices,
                 std::vector<GLushort>& indices);

bool LoadOBJData(const char* path, std::vector<GLfloat>& vertices,
                 std::vector<GLfloat>& normals);
}  // namespace obj_loader
}  // namespace tango_gl
#endif  // TANGO_GL_OBJ_LOADER
