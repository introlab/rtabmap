
/*
 * Copyright 2014 Google Inc. All Rights Reserved.
 * Distributed under the Project Tango Preview Development Kit (PDK) Agreement.
 * CONFIDENTIAL. AUTHORIZED USE ONLY. DO NOT REDISTRIBUTE.
 */

#include "tango-gl/goal_marker.h"

namespace tango_gl {

static const GLfloat const_vertices[] = {
    -4.5298f,  -0.2676f,  0.0,       -4.3209f,  -1.3857f,  0.0,      -3.7242f,
    -11.1445f, 0.0,       -5.6799f,  -8.9811f,  0.0,       -4.4540f, 0.8673f,
    0.0,       -7.8376f,  -6.4508f,  0.0,       -9.5223f,  -3.0538f, 0.0,
    -9.9826f,  -0.5898f,  0.0,       -9.8157f,  1.9113f,   0.0,      -9.0320f,
    4.2923f,   0.0,       -7.6808f,  6.4036f,   0.0,       -5.8469f, 8.1125f,
    0.0,       -3.6457f,  9.3117f,   0.0,       -4.0984f,  1.9477f,  0.0,
    -1.2155f,  9.9259f,   0.0,       -3.4853f,  2.9057f,   0.0,      1.2912f,
    9.9163f,   0.0,       -2.6532f,  3.6812f,   0.0,       3.7167f,  9.2837f,
    0.0,       -1.6543f,  4.2254f,   0.0,       5.9087f,   8.0677f,  0.0,
    -0.5515f,  4.5040f,   0.0,       7.7294f,   6.3448f,   0.0,      0.5859f,
    4.4997f,   0.0,       9.0645f,   4.2232f,   0.0,       1.6865f,  4.2126f,
    0.0,       9.8300f,   1.8363f,   0.0,       2.6812f,   3.6609f,  0.0,
    9.9778f,   -0.6660f,  0.0,       3.5074f,   2.8791f,   0.0,      9.4987f,
    -3.1264f,  0.0,       4.1132f,   1.9164f,   0.0,       7.7964f,  -6.5204f,
    0.0,       4.4605f,   0.8333f,   0.0,       5.6245f,   -9.0444f, 0.0,
    4.5276f,   -0.3022f,  0.0,       3.6572f,   -11.1925f, 0.0,      4.3102f,
    -1.4187f,  0.0,       1.5165f,   -13.5960f, 0.0,       3.8220f,  -2.4460f,
    0.0,       -0.0382f,  -16.4245f, 0.0,       3.0936f,   -3.3197f, 0.0,
    -1.5902f,  -13.5657f, 0.0,       2.1709f,   -3.9847f,  0.0,      -3.8405f,
    -2.4168f,  0.0,       -3.1189f,  -3.2959f,  0.0,       -2.2012f, -3.9680f,
    0.0,       -1.1452f,  -4.3908f,  0.0,       -0.0173f,  -4.5377f, 0.0,
    1.1117f,   -4.3994f,  0.0};

static const GLushort const_indices[] = {
    1,  2,  3,  1,  3,  4,  5,  1,  4,  4,  6,  7,  5,  4,  7,  7,  8,  9,  9,
    10, 11, 7,  9,  11, 5,  7,  11, 5,  11, 12, 5,  12, 13, 14, 5,  13, 14, 13,
    15, 16, 14, 15, 16, 15, 17, 18, 16, 17, 18, 17, 19, 20, 18, 19, 20, 19, 21,
    22, 20, 21, 22, 21, 23, 24, 22, 23, 24, 23, 25, 26, 24, 25, 26, 25, 27, 28,
    26, 27, 28, 27, 29, 30, 28, 29, 30, 29, 31, 32, 30, 31, 32, 31, 33, 34, 32,
    33, 34, 33, 35, 36, 34, 35, 36, 35, 37, 38, 36, 37, 38, 37, 39, 40, 38, 39,
    40, 39, 41, 42, 40, 41, 42, 41, 43, 44, 42, 43, 44, 43, 3,  3,  2,  45, 3,
    45, 46, 3,  46, 47, 3,  47, 48, 3,  48, 49, 3,  49, 50, 44, 3,  50};

GoalMarker::GoalMarker() {
  SetShader();
  std::vector<GLfloat> vertices(
      const_vertices,
      const_vertices + sizeof(const_vertices) / sizeof(GLfloat));
  std::vector<GLushort> indices(
      const_indices, const_indices + sizeof(const_indices) / sizeof(GLushort));
  // Change indices so they are zero-indexed.
  for (size_t i = 0; i < indices.size(); ++i) {
    indices[i] = indices[i] - 1;
  }
  SetVertices(vertices, indices);
}
}  // namespace tango_gl
