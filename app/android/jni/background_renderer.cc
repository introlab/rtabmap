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

// This modules handles drawing the passthrough camera image into the OpenGL
// scene.

#include "background_renderer.h"

#include <type_traits>

namespace {

const std::string kVertexShader =
		"attribute vec4 a_Position;\n"
		"attribute vec2 a_TexCoord;\n"

		"varying vec2 v_TexCoord;\n"

		"void main() {\n"
		"   gl_Position = a_Position;\n"
		"   v_TexCoord = a_TexCoord;\n"
		"}\n";

const std::string kFragmentShader =
	"#extension GL_OES_EGL_image_external : require\n"

	"precision mediump float;\n"
	"varying vec2 v_TexCoord;\n"
	"uniform samplerExternalOES sTexture;\n"

	"void main() {\n"
	"    vec4 sample = texture2D(sTexture, v_TexCoord);\n"
	"    float grey = 0.21 * sample.r + 0.71 * sample.g + 0.07 * sample.b;\n"
	"    gl_FragColor = vec4(grey, grey, grey, 0.5);\n"
	"}\n";

}  // namespace

void BackgroundRenderer::InitializeGlContent(GLuint textureId)
{
  texture_id_ = textureId;

  shader_program_ = tango_gl::util::CreateProgram(kVertexShader.c_str(), kFragmentShader.c_str());
  if (!shader_program_) {
    LOGE("Could not create program.");
  }
  glUseProgram(shader_program_);
  attribute_vertices_ = glGetAttribLocation(shader_program_, "a_Position");
  attribute_uvs_ = glGetAttribLocation(shader_program_, "a_TexCoord");
  glUseProgram(0);
}

void BackgroundRenderer::Draw(const float * transformed_uvs) {
  static_assert(std::extent<decltype(BackgroundRenderer_kVertices)>::value == kNumVertices * 2, "Incorrect kVertices length");

  glUseProgram(shader_program_);
  glDepthMask(GL_FALSE);
  glEnable (GL_BLEND);

  glActiveTexture(GL_TEXTURE0);
  glBindTexture(GL_TEXTURE_EXTERNAL_OES, texture_id_);

  glVertexAttribPointer(attribute_vertices_, 2, GL_FLOAT, GL_FALSE, 0, BackgroundRenderer_kVertices);
  glVertexAttribPointer(attribute_uvs_, 2, GL_FLOAT, GL_FALSE, 0, transformed_uvs);

  glEnableVertexAttribArray(attribute_vertices_);
  glEnableVertexAttribArray(attribute_uvs_);

  glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);

  glDisableVertexAttribArray(attribute_vertices_);
  glDisableVertexAttribArray(attribute_uvs_);

  glUseProgram(0);
  glDepthMask(GL_TRUE);
  glDisable (GL_BLEND);
  tango_gl::util::CheckGlError("BackgroundRenderer::Draw() error");
}

