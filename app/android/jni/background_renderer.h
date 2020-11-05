/*
 * Copyright 2018 Google Inc. All Rights Reserved.
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

#ifndef C_ARCORE_AUGMENTED_IMAGE_BACKGROUND_RENDERER_H_
#define C_ARCORE_AUGMENTED_IMAGE_BACKGROUND_RENDERER_H_

#include <GLES2/gl2.h>
#include <GLES2/gl2ext.h>
#include <cstdlib>

#include "util.h"

static const GLfloat BackgroundRenderer_kVertices[] = {
	-1.0f, -1.0f, +1.0f, -1.0f, -1.0f, +1.0f, +1.0f, +1.0f,
};

// This class renders the passthrough camera image into the OpenGL frame.
class BackgroundRenderer {
public:
	// Positions of the quad vertices in clip space (X, Y).

	static constexpr int kNumVertices = 4;

 public:
  BackgroundRenderer() = default;
  ~BackgroundRenderer() = default;

  // Sets up OpenGL state.  Must be called on the OpenGL thread and before any
  // other methods below.
  void InitializeGlContent(GLuint textureId);

  // Draws the background image.  This methods must be called for every ArFrame
  // returned by ArSession_update() to catch display geometry change events.
  void Draw(const float * transformed_uvs);

 private:

  GLuint shader_program_;
  GLuint texture_id_;

  GLuint attribute_vertices_;
  GLuint attribute_uvs_;
};

#endif  // C_ARCORE_AUGMENTED_IMAGE_BACKGROUND_RENDERER_H_
