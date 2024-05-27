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

#ifdef __ANDROID__
#include <GLES2/gl2.h>
#include <GLES2/gl2ext.h>
#else // __APPLE__
#include <OpenGLES/ES2/gl.h>
#include <OpenGLES/ES2/glext.h>
#endif
#include <cstdlib>

#include "util.h"

static const GLfloat BackgroundRenderer_kVerticesDevice[] = {
	-1.0f, -1.0f, +1.0f, -1.0f, -1.0f, +1.0f, +1.0f, +1.0f,
};
//static const GLfloat BackgroundRenderer_kVerticesView[] = {
//	0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f,
//};
static const GLfloat BackgroundRenderer_kVerticesView[] = {
	0.0f, 1.0f, 1.0f, 1.0f, 0.0f, 0.0f, 1.0f, 0.0f,
};
static const GLfloat BackgroundRenderer_kTexCoord[] = {
   1.0f, 1.0f, 1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
};

//android phone
//11 10 01 00 // portrait
//01 11 00 10 // left
//10 00 11 01 // right
//00 01 10 11 // down


// This class renders the passthrough camera image into the OpenGL frame.
class BackgroundRenderer {
public:
	// Positions of the quad vertices in clip space (X, Y).

	static constexpr int kNumVertices = 4;

 public:
  BackgroundRenderer() = default;
  ~BackgroundRenderer();

  // Sets up OpenGL state.  Must be called on the OpenGL thread and before any
  // other methods below.
  void InitializeGlContent(GLuint textureId, bool oes);

  // Draws the background image.  This methods must be called for every ArFrame
  // returned by ArSession_update() to catch display geometry change events.
  void Draw(const float * transformed_uvs, const GLuint & depthTexture, int screenWidth, int screenHeight, bool redUnknown);

private:
  static std::vector<GLuint> shaderPrograms_;
  GLuint texture_id_;
  bool oes_ = false;
};

#endif  // C_ARCORE_AUGMENTED_IMAGE_BACKGROUND_RENDERER_H_
