/*
Copyright (c) 2010-2016, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
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

#include <sstream>

#include "graph_drawable.h"
#include "rtabmap/utilite/ULogger.h"
#include "util.h"

#include <GLES2/gl2.h>

GraphDrawable::GraphDrawable(
		GLuint shaderProgram,
		const std::map<int, rtabmap::Transform> & poses,
		const std::multimap<int, rtabmap::Link> & links) :
		vertex_buffers_(0),
		pose_(1.0f),
		visible_(true),
		lineWidth_(3.0f),
		shader_program_(shaderProgram)
{
	UASSERT(!poses.empty());

	glGenBuffers(1, &vertex_buffers_);

	if(vertex_buffers_)
	{
		LOGI("Creating vertex buffer %d", vertex_buffers_);
		std::vector<float> vertices = std::vector<float>(poses.size()*3);
		int i=0;
		std::map<int, int> idsToIndices;
		for(std::map<int, rtabmap::Transform>::const_iterator iter=poses.begin(); iter!=poses.end(); ++iter)
		{
			vertices[i*3] = iter->second.x();
			vertices[i*3+1] = iter->second.y();
			vertices[i*3+2] = iter->second.z();
			idsToIndices.insert(std::make_pair(iter->first, i));
			++i;
		}

		glBindBuffer(GL_ARRAY_BUFFER, vertex_buffers_);
		glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * (int)vertices.size(), (const void *)vertices.data(), GL_STATIC_DRAW);
		glBindBuffer(GL_ARRAY_BUFFER, 0);

		GLint error = glGetError();
		if(error != GL_NO_ERROR)
		{
			LOGI("OpenGL: Could not allocate point cloud (0x%x)\n", error);
			vertex_buffers_ = 0;
		}
		else if(links.size())
		{
			neighborIndices_.resize(links.size() * 2);
			loopClosureIndices_.resize(links.size() * 2);
			int oiNeighbors = 0;
			int oiLoopClosures = 0;
			for(std::multimap<int, rtabmap::Link>::const_iterator iter=links.begin(); iter!=links.end(); ++iter)
			{
				std::map<int, int>::const_iterator jterFrom = idsToIndices.find(iter->second.from());
				std::map<int, int>::const_iterator jterTo = idsToIndices.find(iter->second.to());
				if(jterFrom != idsToIndices.end() && jterTo != idsToIndices.end())
				{
					if(iter->second.type() == rtabmap::Link::kNeighbor)
					{
						neighborIndices_[oiNeighbors++] = (unsigned short)jterFrom->second;
						neighborIndices_[oiNeighbors++] = (unsigned short)jterTo->second;
					}
					else
					{
						loopClosureIndices_[oiLoopClosures++] = (unsigned short)jterFrom->second;
						loopClosureIndices_[oiLoopClosures++] = (unsigned short)jterTo->second;
					}
				}
			}
			neighborIndices_.resize(oiNeighbors);
			loopClosureIndices_.resize(oiLoopClosures);
		}
	}
}

GraphDrawable::~GraphDrawable()
{
	LOGI("Freeing cloud buffer %d", vertex_buffers_);
	if (vertex_buffers_)
	{
		glDeleteBuffers(1, &vertex_buffers_);
		tango_gl::util::CheckGlError("GraphDrawable::~GraphDrawable()");
		vertex_buffers_ = 0;
	}
}

void GraphDrawable::setPose(const rtabmap::Transform & pose)
{
	UASSERT(!pose.isNull());

	pose_ = glmFromTransform(pose);
}

void GraphDrawable::Render(const glm::mat4 & projectionMatrix, const glm::mat4 & viewMatrix) {

	if(vertex_buffers_ && (neighborIndices_.size() || loopClosureIndices_.size()) && visible_)
	{
		glUseProgram(shader_program_);
		glLineWidth(lineWidth_);

		GLuint mvp_handle_ = glGetUniformLocation(shader_program_, "mvp");
		glm::mat4 mvp_mat = projectionMatrix * viewMatrix * pose_;
		glUniformMatrix4fv(mvp_handle_, 1, GL_FALSE, glm::value_ptr(mvp_mat));

		GLuint color_handle = glGetUniformLocation(shader_program_, "color");

		GLint attribute_vertex = glGetAttribLocation(shader_program_, "vertex");

		glEnableVertexAttribArray(attribute_vertex);
		glBindBuffer(GL_ARRAY_BUFFER, vertex_buffers_);
		glVertexAttribPointer(attribute_vertex, 3, GL_FLOAT, GL_FALSE, 3*sizeof(GLfloat), 0);

		if(neighborIndices_.size())
		{
			glUniform3f(color_handle, 1.0f, 0.0f, 0.0f); // blue for neighbors
			glDrawElements(GL_LINES, neighborIndices_.size(), GL_UNSIGNED_SHORT, neighborIndices_.data());
		}
		if(loopClosureIndices_.size())
		{
			glUniform3f(color_handle, 0.0f, 0.0f, 1.0f); // red for loop closures
			glDrawElements(GL_LINES, loopClosureIndices_.size(), GL_UNSIGNED_SHORT, loopClosureIndices_.data());
		}

		glDisableVertexAttribArray(0);
		glBindBuffer(GL_ARRAY_BUFFER, 0);

		glUseProgram(0);
		tango_gl::util::CheckGlError("GraphDrawable::Render()");
	}
}

