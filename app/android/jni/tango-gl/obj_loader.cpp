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
#include <string>

#include "tango-gl/obj_loader.h"

namespace tango_gl {
bool obj_loader::LoadOBJData(const char* path, std::vector<GLfloat>& vertices,
                             std::vector<GLushort>& indices) {
  FILE *file = fopen(path, "r");
  if (file == NULL) {
    LOGE("Failed to open file: %s", path);
    return false;
  }

  while (1) {
    char lineHeader[128];
    int res = fscanf(file, "%s", lineHeader);
    if (res == EOF) break;
    if (strcmp(lineHeader, "v") == 0) {
      GLfloat vertex[3];
      int matches =
          fscanf(file, "%f %f %f\n", &vertex[0], &vertex[1], &vertex[2]);
      if (matches != 3) {
        LOGE("Format of 'v float float float' required for each vertice line");
        return false;
      }
      vertices.push_back(vertex[0]);
      vertices.push_back(vertex[1]);
      vertices.push_back(vertex[2]);
    } else if (strcmp(lineHeader, "f") == 0) {
      GLushort vertexIndex[3];
      int matches = fscanf(file, "%hu %hu %hu\n", &vertexIndex[0],
                           &vertexIndex[1], &vertexIndex[2]);
      if (matches != 3) {
        LOGE("Format of 'f int int int' required for each face line");
        return false;
      }
      indices.push_back(vertexIndex[0] - 1);
      indices.push_back(vertexIndex[1] - 1);
      indices.push_back(vertexIndex[2] - 1);
    }
    else {
      char comments_buffer[1000];
      fgets(comments_buffer, 1000, file);
    }
  }
  fclose(file);
  return true;
}

bool obj_loader::LoadOBJData(const char* path, std::vector<GLfloat>& vertices,
                             std::vector<GLfloat>& normals) {
  std::vector<unsigned int> vertexIndices, normalIndices;
  std::vector<GLfloat> temp_vertices, temp_normals;

  FILE* file = fopen(path, "r");
  if (file == NULL) {
    LOGE("Failed to open file: %s", path);
    return false;
  }

  while (1) {
    char lineHeader[128];
    int res = fscanf(file, "%s", lineHeader);
    if (res == EOF) break;
    if (strcmp(lineHeader, "v") == 0) {
      GLfloat vertex[3];
      int matches =
          fscanf(file, "%f %f %f\n", &vertex[0], &vertex[1], &vertex[2]);
      if (matches != 3) {
        LOGE("Format of 'v float float float' required for each vertice line");
        return false;
      }
      temp_vertices.push_back(vertex[0]);
      temp_vertices.push_back(vertex[1]);
      temp_vertices.push_back(vertex[2]);
    } else if (strcmp(lineHeader, "vn") == 0) {
      GLfloat normal[3];
      int matches =
          fscanf(file, "%f %f %f\n", &normal[0], &normal[1], &normal[2]);
      if (matches != 3) {
        LOGE("Format of 'vn float float float' required for each normal line");
        return false;
      }
      temp_normals.push_back(normal[0]);
      temp_normals.push_back(normal[1]);
      temp_normals.push_back(normal[2]);
    } else if (strcmp(lineHeader, "f") == 0) {
      GLushort vertexIndex[4];
      GLushort normalIndex[4];
      int matches = fscanf(file, "%hu//%hu %hu//%hu %hu//%hu %hu//%hu\n",
                           &vertexIndex[0], &normalIndex[0], &vertexIndex[1], &normalIndex[1],
                           &vertexIndex[2], &normalIndex[2], &vertexIndex[3], &normalIndex[3]);

      // .obj file is 1-indexed, so subtract 1 from all indices.
      if (matches == 6) {
        // If triangles provided.
        vertexIndices.push_back(vertexIndex[0] - 1);
        vertexIndices.push_back(vertexIndex[1] - 1);
        vertexIndices.push_back(vertexIndex[2] - 1);
        normalIndices.push_back(normalIndex[0] - 1);
        normalIndices.push_back(normalIndex[1] - 1);
        normalIndices.push_back(normalIndex[2] - 1);
      } else if(matches == 8) {
        // If quads provided.
        vertexIndices.push_back(vertexIndex[0] - 1);
        vertexIndices.push_back(vertexIndex[1] - 1);
        vertexIndices.push_back(vertexIndex[2] - 1);
        vertexIndices.push_back(vertexIndex[0] - 1);
        vertexIndices.push_back(vertexIndex[2] - 1);
        vertexIndices.push_back(vertexIndex[3] - 1);
        normalIndices.push_back(normalIndex[0] - 1);
        normalIndices.push_back(normalIndex[1] - 1);
        normalIndices.push_back(normalIndex[2] - 1);
        normalIndices.push_back(normalIndex[0] - 1);
        normalIndices.push_back(normalIndex[2] - 1);
        normalIndices.push_back(normalIndex[3] - 1);
      } else {
        LOGE("Format of 'f int//int int//int int//int' required for each face");
        return false;
      }
    } else {
      char comments_buffer[1000];
      fgets(comments_buffer, 1000, file);
    }
  }

  for (unsigned int i = 0; i < vertexIndices.size(); i++) {
    unsigned int vertexIndex = vertexIndices[i];
    unsigned int normalIndex = normalIndices[i];

    vertices.push_back(temp_vertices[vertexIndex * 3]);
    vertices.push_back(temp_vertices[vertexIndex * 3 + 1]);
    vertices.push_back(temp_vertices[vertexIndex * 3 + 2]);
    normals.push_back(temp_normals[normalIndex * 3]);
    normals.push_back(temp_normals[normalIndex * 3 + 1]);
    normals.push_back(temp_normals[normalIndex * 3 + 2]);
  }
  fclose(file);
  return true;
}
}  // namespace tango_gl
