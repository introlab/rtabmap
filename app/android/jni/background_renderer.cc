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

const std::string kFragmentShaderOES =
	"#extension GL_OES_EGL_image_external : require\n"
	"precision mediump float;\n"
	"varying vec2 v_TexCoord;\n"
	"uniform samplerExternalOES sTexture;\n"
    "uniform bool uRedUnknown;\n"
	"void main() {\n"
	"    vec4 sample = texture2D(sTexture, v_TexCoord);\n"
    "    float grey = 0.21 * sample.r + 0.71 * sample.g + 0.07 * sample.b;\n"
    "    gl_FragColor = vec4(grey, uRedUnknown?0.0:grey, uRedUnknown?0.0:grey, 0.5);\n"
	"}\n";

const std::string kFragmentShaderBlendingOES =
    "#extension GL_OES_EGL_image_external : require\n"
    "precision mediump float;\n"
    "varying vec2 v_TexCoord;\n"
    "uniform samplerExternalOES sTexture;\n"
	"uniform sampler2D uDepthTexture;\n"
	"uniform vec2 uScreenScale;\n"
    "uniform bool uRedUnknown;\n"
    "void main() {\n"
    "    vec4 sample = texture2D(sTexture, v_TexCoord);\n"
    "    vec2 coord = uScreenScale * gl_FragCoord.xy;\n;"
    "    vec4 depthPacked = texture2D(uDepthTexture, coord);\n"
    "    float depth = dot(depthPacked, 1./vec4(1.,255.,65025.,16581375.));\n"
    "    if(depth > 0.0)\n"
    "        gl_FragColor = vec4(sample.r, sample.g, sample.b, 0.5);\n"
    "    else {\n"
    "        float grey = 0.21 * sample.r + 0.71 * sample.g + 0.07 * sample.b;\n"
    "        gl_FragColor = vec4(grey, uRedUnknown?0.0:grey, uRedUnknown?0.0:grey, 0.5);\n"
    "    }\n"
    "}\n";

const std::string kFragmentShader =
	"precision mediump float;\n"
	"varying vec2 v_TexCoord;\n"
    "uniform sampler2D sTexture;\n"
    "uniform bool uRedUnknown;\n"
	"void main() {\n"
	"    vec4 sample = texture2D(sTexture, v_TexCoord);\n"
	"    float grey = 0.21 * sample.r + 0.71 * sample.g + 0.07 * sample.b;\n"
	"    gl_FragColor = vec4(grey, uRedUnknown?0.0:grey, uRedUnknown?0.0:grey, 0.5);\n"
	"}\n";

const std::string kFragmentShaderBlending =
    "precision mediump float;\n"
    "varying vec2 v_TexCoord;\n"
    "uniform sampler2D sTexture;\n"
    "uniform sampler2D uDepthTexture;\n"
    "uniform vec2 uScreenScale;\n"
    "uniform bool uRedUnknown;\n"
    "void main() {\n"
    "    vec4 sample = texture2D(sTexture, v_TexCoord);\n"
    "    vec2 coord = uScreenScale * gl_FragCoord.xy;\n;"
    "    vec4 depthPacked = texture2D(uDepthTexture, coord);\n"
    "    float depth = dot(depthPacked, 1./vec4(1.,255.,65025.,16581375.));\n"
    "    if(depth > 0.0)\n"
    "        gl_FragColor = vec4(sample.r, sample.g, sample.b, 0.5);\n"
    "    else {\n"
    "        float grey = 0.21 * sample.r + 0.71 * sample.g + 0.07 * sample.b;\n"
    "        gl_FragColor = vec4(grey, uRedUnknown?0.0:grey, uRedUnknown?0.0:grey, 0.5);\n"
    "    }\n"
    "}\n";

/* To debug depth texture
 const std::string kFragmentShader =
     "precision mediump float;\n"
     "varying vec2 v_TexCoord;\n"
     "uniform sampler2D sTexture;\n"

     "void main() {\n"
     "  float uNearZ = 0.2;\n"
     "  float uFarZ = 1000.0;\n"
     "  float depth = texture2D(sTexture, v_TexCoord).r;\n"
     "  float num =  (2.0 * uNearZ * uFarZ);\n"
     "  float diff = (uFarZ - uNearZ);\n"
     "  float add = (uFarZ + uNearZ);\n"
     "  float ndcDepth = depth * 2.0 - 1.0;\n" // Back to NDC
     "  float linearDepth = num / (add - ndcDepth * diff);\n" // inverse projection matrix
     "  float grey = linearDepth/3.0;\n"
     "  gl_FragColor = vec4(grey, grey, grey, 0.5);\n"
     "}\n";
 */

}  // namespace

std::vector<GLuint> BackgroundRenderer::shaderPrograms_;

BackgroundRenderer::~BackgroundRenderer()
{
	for(unsigned int i=0; i<shaderPrograms_.size(); ++i)
	{
		glDeleteShader(shaderPrograms_[i]);
	}
	shaderPrograms_.clear();
}

void BackgroundRenderer::InitializeGlContent(GLuint textureId, bool oes)
{
	LOGI("textureId=%d", textureId);

  texture_id_ = textureId;
#ifdef __ANDROID__
  oes_ = oes;
#endif

    if(shaderPrograms_.empty())
    {
        shaderPrograms_.resize(2,0);
        shaderPrograms_[0] = tango_gl::util::CreateProgram(
              kVertexShader.c_str(),
              oes_?kFragmentShaderOES.c_str():kFragmentShader.c_str());
        UASSERT(shaderPrograms_[0]!=0);
        shaderPrograms_[1] = tango_gl::util::CreateProgram(
              kVertexShader.c_str(),
              oes_?kFragmentShaderBlendingOES.c_str():kFragmentShaderBlending.c_str());
        UASSERT(shaderPrograms_[1]!=0);
    }
}

void BackgroundRenderer::Draw(const float * transformed_uvs, const GLuint & depthTexture, int screenWidth, int screenHeight, bool redUnknown) {
  static_assert(std::extent<decltype(BackgroundRenderer_kVerticesDevice)>::value == kNumVertices * 2, "Incorrect kVertices length");

  GLuint program = shaderPrograms_[depthTexture>0?1:0];
    
  glUseProgram(program);
  glDepthMask(GL_FALSE);
  glEnable (GL_BLEND);

  glActiveTexture(GL_TEXTURE0);
#ifdef __ANDROID__
  if(oes_)
    glBindTexture(GL_TEXTURE_EXTERNAL_OES, texture_id_);
  else
#endif
    glBindTexture(GL_TEXTURE_2D, texture_id_);

  if(depthTexture>0)
  {
    // Texture activate unit 1
    glActiveTexture(GL_TEXTURE1);
    // Bind the texture to this unit.
    glBindTexture(GL_TEXTURE_2D, depthTexture);
    // Tell the texture uniform sampler to use this texture in the shader by binding to texture unit 1.
    GLuint depth_texture_handle = glGetUniformLocation(program, "uDepthTexture");
    glUniform1i(depth_texture_handle, 1);
      
    GLuint screenScale_handle = glGetUniformLocation(program, "uScreenScale");
    glUniform2f(screenScale_handle, 1.0f/(float)screenWidth, 1.0f/(float)screenHeight);
  }
    
  GLuint screenScale_handle = glGetUniformLocation(program, "uRedUnknown");
  glUniform1i(screenScale_handle, redUnknown);

  GLuint attributeVertices = glGetAttribLocation(program, "a_Position");
  GLuint attributeUvs = glGetAttribLocation(program, "a_TexCoord");

  glVertexAttribPointer(attributeVertices, 2, GL_FLOAT, GL_FALSE, 0, BackgroundRenderer_kVerticesDevice);
  glVertexAttribPointer(attributeUvs, 2, GL_FLOAT, GL_FALSE, 0, transformed_uvs?transformed_uvs:BackgroundRenderer_kTexCoord);

  glEnableVertexAttribArray(attributeVertices);
  glEnableVertexAttribArray(attributeUvs);

  glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);

  glDisableVertexAttribArray(attributeVertices);
  glDisableVertexAttribArray(attributeUvs);

  glUseProgram(0);
  glDepthMask(GL_TRUE);
  glDisable (GL_BLEND);
  tango_gl::util::CheckGlError("BackgroundRenderer::Draw() error");
}

