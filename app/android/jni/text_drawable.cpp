/*
Copyright (c) 2010-2025, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Universite de Sherbrooke nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifdef __ANDROID__
#include <jni.h>
#endif

#include <tango-gl/util.h>
#include <vector>
#include <rtabmap/core/Transform.h>
#include "util.h"
#include "text_drawable.h"
#include "text_atlas_png.h"
#include <rtabmap/utilite/UConversion.h>

const std::string kTextVertexShader =
	"uniform mat4 uMVPMatrix;"
	"attribute vec4 vPosition;"
	"attribute vec2 a_texCoord;"
	"varying vec2 v_texCoord;"
	"void main() {"
	"  gl_Position = uMVPMatrix * vPosition;"
	"  v_texCoord = a_texCoord;"
	"}";
const std::string kTextFragmentShader =
	"precision mediump float;"
	"uniform vec3 uColor;"
	"varying vec2 v_texCoord;"
	"uniform sampler2D s_texture;"
	"void main() {"
	"  gl_FragColor = texture2D( s_texture, v_texCoord );"
	"  gl_FragColor.rgb = uColor;"
	"}";

const int RI_TEXT_TEXTURE_SIZE = 512; // 512
const float RI_TEXT_HEIGHT_BASE = 32.0f;
const char RI_TEXT_START = ' ';
const char RI_TEXT_STOP = '~'+1;

GLuint TextDrawable::textProgram_ = 0;
GLuint TextDrawable::textTextureId_ = 0;
float TextDrawable::textUVWidth_ = 0;
float TextDrawable::textUVHeight_ = 0;
float TextDrawable::textHeight_ = 0;
std::vector<float> TextDrawable::textCharacterWidths_;

void TextDrawable::createShaderProgram()
{
	releaseShaderProgram();

	// hard-coded with text_atlas.png resource
	textCharacterWidths_.resize(95);
	textCharacterWidths_[0]=8;
	textCharacterWidths_[1]=9.000000;
	textCharacterWidths_[2]=10.000000;
	textCharacterWidths_[3]=19.000000;
	textCharacterWidths_[4]=18.000000;
	textCharacterWidths_[5]=24.000000;
	textCharacterWidths_[6]=21.000000;
	textCharacterWidths_[7]=5.000000;
	textCharacterWidths_[8]=11.000000;
	textCharacterWidths_[9]=11.000000;
	textCharacterWidths_[10]=15.000000;
	textCharacterWidths_[11]=17.000000;
	textCharacterWidths_[12]=8.000000;
	textCharacterWidths_[13]=12.000000;
	textCharacterWidths_[14]=9.000000;
	textCharacterWidths_[15]=12.000000;
	textCharacterWidths_[16]=18.000000;
	textCharacterWidths_[17]=18.000000;
	textCharacterWidths_[18]=18.000000;
	textCharacterWidths_[19]=18.000000;
	textCharacterWidths_[20]=18.000000;
	textCharacterWidths_[21]=18.000000;
	textCharacterWidths_[22]=18.000000;
	textCharacterWidths_[23]=18.000000;
	textCharacterWidths_[24]=18.000000;
	textCharacterWidths_[25]=18.000000;
	textCharacterWidths_[26]=9.000000;
	textCharacterWidths_[27]=8.000000;
	textCharacterWidths_[28]=16.000000;
	textCharacterWidths_[29]=18.000000;
	textCharacterWidths_[30]=17.000000;
	textCharacterWidths_[31]=16.000000;
	textCharacterWidths_[32]=29.000000;
	textCharacterWidths_[33]=22.000000;
	textCharacterWidths_[34]=20.000000;
	textCharacterWidths_[35]=21.000000;
	textCharacterWidths_[36]=21.000000;
	textCharacterWidths_[37]=18.000000;
	textCharacterWidths_[38]=18.000000;
	textCharacterWidths_[39]=22.000000;
	textCharacterWidths_[40]=23.000000;
	textCharacterWidths_[41]=9.000000;
	textCharacterWidths_[42]=18.000000;
	textCharacterWidths_[43]=20.000000;
	textCharacterWidths_[44]=17.000000;
	textCharacterWidths_[45]=28.000000;
	textCharacterWidths_[46]=23.000000;
	textCharacterWidths_[47]=22.000000;
	textCharacterWidths_[48]=21.000000;
	textCharacterWidths_[49]=22.000000;
	textCharacterWidths_[50]=20.000000;
	textCharacterWidths_[51]=20.000000;
	textCharacterWidths_[52]=20.000000;
	textCharacterWidths_[53]=21.000000;
	textCharacterWidths_[54]=21.000000;
	textCharacterWidths_[55]=28.000000;
	textCharacterWidths_[56]=20.000000;
	textCharacterWidths_[57]=20.000000;
	textCharacterWidths_[58]=19.000000;
	textCharacterWidths_[59]=9.000000;
	textCharacterWidths_[60]=14.000000;
	textCharacterWidths_[61]=9.000000;
	textCharacterWidths_[62]=14.000000;
	textCharacterWidths_[63]=14.000000;
	textCharacterWidths_[64]=11.000000;
	textCharacterWidths_[65]=17.000000;
	textCharacterWidths_[66]=18.000000;
	textCharacterWidths_[67]=17.000000;
	textCharacterWidths_[68]=18.000000;
	textCharacterWidths_[69]=17.000000;
	textCharacterWidths_[70]=11.000000;
	textCharacterWidths_[71]=18.000000;
	textCharacterWidths_[72]=18.000000;
	textCharacterWidths_[73]=8.000000;
	textCharacterWidths_[74]=8.000000;
	textCharacterWidths_[75]=17.000000;
	textCharacterWidths_[76]=8.000000;
	textCharacterWidths_[77]=28.000000;
	textCharacterWidths_[78]=18.000000;
	textCharacterWidths_[79]=18.000000;
	textCharacterWidths_[80]=18.000000;
	textCharacterWidths_[81]=18.000000;
	textCharacterWidths_[82]=12.000000;
	textCharacterWidths_[83]=16.000000;
	textCharacterWidths_[84]=11.000000;
	textCharacterWidths_[85]=18.000000;
	textCharacterWidths_[86]=16.000000;
	textCharacterWidths_[87]=24.000000;
	textCharacterWidths_[88]=16.000000;
	textCharacterWidths_[89]=16.000000;
	textCharacterWidths_[90]=16.000000;
	textCharacterWidths_[91]=11.000000;
	textCharacterWidths_[92]=8.000000;
	textCharacterWidths_[93]=11.000000;
	textCharacterWidths_[94]=21.000000;
	textUVWidth_=0.062500;
	textUVHeight_=0.082031;
	textHeight_=42.000000;

	textProgram_ = tango_gl::util::CreateProgram(kTextVertexShader.c_str(), kTextFragmentShader.c_str());
	UASSERT(textProgram_ != 0);

	glGenTextures(1, &textTextureId_);
	UASSERT(textTextureId_);

	// gen texture from image
	glBindTexture(GL_TEXTURE_2D, textTextureId_);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

	std::vector<char> data = uHex2Bytes(rtabmap::TEXT_ATLAS_PNG);

	cv::Mat rgbImage = cv::imdecode(data, cv::IMREAD_UNCHANGED);

	glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, rgbImage.cols, rgbImage.rows, 0, GL_RGBA, GL_UNSIGNED_BYTE, rgbImage.data);

	GLint error = glGetError();
	UASSERT(error == GL_NO_ERROR);
}

void TextDrawable::releaseShaderProgram()
{
	if(textProgram_)
	{
		glDeleteShader(textProgram_);
		textProgram_ = 0;
	}
}

TextDrawable::TextDrawable(
		const std::string & text,
		const rtabmap::Transform & pose,
		float textSize, // meters
		const tango_gl::Color & color) :
		poseGl_(glmFromTransform(pose)),
		color_(color)
{
	if(textProgram_ > 0)
	{
		vertexBuffer_ = std::vector<float>(text.size() * 12);
		textureBuffer_ = std::vector<float>(text.size() * 8);
		drawListBuffer_ = std::vector<unsigned short>(text.size() * 6);

		int index_vecs = 0;
		int index_indices = 0;
		int index_uvs = 0;
		float x=0.0f;
		float y=0.0f;
		float uniformscale = textSize/textHeight_;
		for(unsigned int i=0; i<text.size(); ++i)
		{
			// get ascii value
			char c = text[i];
			int c_val = (int)c;

			int indx = c_val-RI_TEXT_START;

			if(indx<0 || indx>=textCharacterWidths_.size()) {
				// unknown character, we will add a space for it to be save.
				indx = 0;
			}

			int colCount = RI_TEXT_TEXTURE_SIZE/(int)RI_TEXT_HEIGHT_BASE;

			// Calculate the uv parts
			int row = indx / colCount;
			int col = indx % colCount;

			float v = row * textUVHeight_;
			float v2 = v + textUVHeight_;
			float u = col * textUVWidth_;
			float u2 = u + textCharacterWidths_[indx]/(float)RI_TEXT_TEXTURE_SIZE;

			// Creating the triangle information
			std::vector<float> vec(12);
			std::vector<float> uv(8);

			vec[0] = x;
			vec[1] = y + (textHeight_ * uniformscale);
			vec[2] = 0;
			vec[3] = x;
			vec[4] = y;
			vec[5] = 0;
			vec[6] = x + (textCharacterWidths_[indx] * uniformscale);
			vec[7] = y;
			vec[8] = 0;
			vec[9] = x + (textCharacterWidths_[indx] * uniformscale);
			vec[10] = y + (textHeight_ * uniformscale);
			vec[11] = 0;

			// 0.001f = texture bleeding hack/fix
			uv[0] = u+0.001f;
			uv[1] = v+0.001f;
			uv[2] = u+0.001f;
			uv[3] = v2-0.001f;
			uv[4] = u2-0.001f;
			uv[5] = v2-0.001f;
			uv[6] = u2-0.001f;
			uv[7] = v+0.001f;

			unsigned short inds[6] = {0, 1, 2, 0, 2, 3};

			// We need a base value because the object has indices related to
			// that object and not to this collection so basicly we need to
			// translate the indices to align with the vertexlocation in ou
			// vecs array of vectors.
			short base = (short) (index_vecs / 3);

			// We should add the vec, translating the indices to our saved vector
			for(int i=0;i<vec.size();i++)
			{
				vertexBuffer_[index_vecs] = vec[i];
				index_vecs++;
			}

			// We should add the uvs
			for(int i=0;i<uv.size();i++)
			{
				textureBuffer_[index_uvs] = uv[i];
				index_uvs++;
			}

			// We handle the indices
			for(int j=0;j<6;j++)
			{
				drawListBuffer_[index_indices] = (unsigned short) (base + inds[j]);
				index_indices++;
			}

			// Calculate the new position
			x += (textCharacterWidths_[indx]  * uniformscale);
		}
	}
}

void TextDrawable::Render(
		const glm::mat4 & projectionMatrix,
		const glm::mat4 & viewMatrix,
		const glm::mat4 & viewMatrixRotInv) const
{
	glUseProgram(textProgram_);

	// get handle to vertex shader's vPosition member
	int mPositionHandle = glGetAttribLocation(textProgram_, "vPosition");

	// Enable a handle to the triangle vertices
	glEnableVertexAttribArray(mPositionHandle);

	// Prepare the background coordinate data
	glVertexAttribPointer(mPositionHandle, 3, GL_FLOAT, false, 0, &vertexBuffer_[0]);

	int mTexCoordLoc = glGetAttribLocation(textProgram_, "a_texCoord" );

	// Prepare the texturecoordinates
	glVertexAttribPointer ( mTexCoordLoc, 2, GL_FLOAT, false, 0, &textureBuffer_[0]);

	glEnableVertexAttribArray ( mPositionHandle );
	glEnableVertexAttribArray ( mTexCoordLoc );

	// get handle to shape's transformation matrix
	int mtrxhandle = glGetUniformLocation(textProgram_, "uMVPMatrix");

	// Apply the projection and view transformation
	//glUniformMatrix4fv(mtrxhandle, 1, false, m, 0);
	glm::mat4 mvp_mat = projectionMatrix * viewMatrix * poseGl_ * viewMatrixRotInv;//glm::toMat4(gesture_camera_->GetParent()->GetRotation());
	glUniformMatrix4fv(mtrxhandle, 1, GL_FALSE, glm::value_ptr(mvp_mat));

	// get handle to color value
	int colorhandle = glGetUniformLocation(textProgram_, "uColor");
	glUniform3f(colorhandle, color_.r, color_.g, color_.b);

	int mSamplerLoc = glGetUniformLocation (textProgram_, "s_texture" );

	// Texture activate unit 0
	glActiveTexture(GL_TEXTURE0);
	// Bind the texture to this unit.
	glBindTexture(GL_TEXTURE_2D, textTextureId_);
	// Set the sampler texture unit to our selected id
	glUniform1i ( mSamplerLoc, 0);

	// Draw the triangle
	glDrawElements(GL_TRIANGLES, drawListBuffer_.size(), GL_UNSIGNED_SHORT, &drawListBuffer_[0]);

	// Disable vertex array
	glDisableVertexAttribArray(mPositionHandle);
	glDisableVertexAttribArray(mTexCoordLoc);
}
