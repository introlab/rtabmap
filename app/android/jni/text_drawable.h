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

#ifndef TANGO_TEXT_DRAWABLE_H_
#define TANGO_TEXT_DRAWABLE_H_

#include <vector>
#include <rtabmap/core/Transform.h>
#include <tango-gl/color.h>

// PointCloudDrawable is responsible for the point cloud rendering.
class TextDrawable {
private:
	static GLuint textProgram_;
	static GLuint textTextureId_;
	static float textUVWidth_;
	static float textUVHeight_;
	static float textHeight_;
	static std::vector<float> textCharacterWidths_;

public:
	static void createShaderProgram();
	static void releaseShaderProgram();

public:
	TextDrawable(
			const std::string & text,
			const rtabmap::Transform & pose,
			float textSize = 0.1f, // meters
			const tango_gl::Color & color = tango_gl::Color(1,0,0));

  virtual ~TextDrawable() {}

  void Render(
		  const glm::mat4 & projectionMatrix,
		  const glm::mat4 & viewMatrix,
		  const glm::mat4 & viewMatrixRotInv) const;

 private:
	std::vector<float> vertexBuffer_;
	std::vector<float> textureBuffer_;
	std::vector<unsigned short> drawListBuffer_;
	glm::mat4 poseGl_;
	tango_gl::Color color_;
};

#endif  // TANGO_POINT_CLOUD_POINT_CLOUD_DRAWABLE_H_
