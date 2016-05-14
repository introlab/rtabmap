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

#include "point_cloud_drawable.h"
#include "rtabmap/utilite/ULogger.h"
#include "rtabmap/utilite/UConversion.h"
#include <opencv2/imgproc/imgproc.hpp>
#include "util.h"

#include <GLES2/gl2.h>

PointCloudDrawable::PointCloudDrawable(
		GLuint cloudShaderProgram,
		GLuint textureShaderProgram,
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		const std::vector<pcl::Vertices> & polygons,
		const cv::Mat & image) :
		vertex_buffers_(0),
		textures_(0),
		nPoints_(0),
		pose_(1.0f),
		visible_(true),
		cloud_shader_program_(cloudShaderProgram),
		texture_shader_program_(textureShaderProgram)
{
	UASSERT(!cloud->empty());

	glGenBuffers(1, &vertex_buffers_);
	if(!vertex_buffers_)
	{
		LOGE("OpenGL: could not generate vertex buffers\n");
		return;
	}

	if(!cloud->is_dense && !image.empty())
	{
		LOGI("cloud=%dx%d image=%dx%d\n", (int)cloud->width, (int)cloud->height, image.cols, image.rows);
		UASSERT(polygons.size() && !cloud->is_dense && !image.empty() && image.type() == CV_8UC3);
		glGenTextures(1, &textures_);
		if(!textures_)
		{
			vertex_buffers_ = 0;
			LOGE("OpenGL: could not generate vertex buffers\n");
			return;
		}
	}

	LOGI("Creating cloud buffer %d", vertex_buffers_);
	std::vector<float> vertices;
	if(textures_)
	{
		vertices = std::vector<float>(cloud->size()*6);
		for(unsigned int i=0; i<cloud->size(); ++i)
		{
			vertices[i*6] = cloud->at(i).x;
			vertices[i*6+1] = cloud->at(i).y;
			vertices[i*6+2] = cloud->at(i).z;

			// rgb
			vertices[i*6+3] = cloud->at(i).rgb;

			// texture uv
			vertices[i*6+4] = float(i % cloud->width)/float(cloud->width); //u
			vertices[i*6+5] = float(i/cloud->width)/float(cloud->height);  //v
		}
	}
	else
	{
		vertices = std::vector<float>(cloud->size()*4);
		for(unsigned int i=0; i<cloud->size(); ++i)
		{
			vertices[i*4] = cloud->at(i).x;
			vertices[i*4+1] = cloud->at(i).y;
			vertices[i*4+2] = cloud->at(i).z;
			vertices[i*4+3] = cloud->at(i).rgb;
		}
	}

	glBindBuffer(GL_ARRAY_BUFFER, vertex_buffers_);
	glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * (int)vertices.size(), (const void *)vertices.data(), GL_STATIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	GLint error = glGetError();
	if(error != GL_NO_ERROR)
	{
		LOGE("OpenGL: Could not allocate point cloud (0x%x)\n", error);
		vertex_buffers_ = 0;
		return;
	}

	if(textures_)
	{
		// gen texture from image
		glBindTexture(GL_TEXTURE_2D, textures_);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
		cv::Mat rgbImage;
		cv::cvtColor(image, rgbImage, CV_BGR2RGB);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, rgbImage.cols, rgbImage.rows, 0, GL_RGB, GL_UNSIGNED_BYTE, rgbImage.data);

		GLint error = glGetError();
		if(error != GL_NO_ERROR)
		{
			LOGE("OpenGL: Could not allocate texture (0x%x)\n", error);
			textures_ = 0;

			glDeleteBuffers(1, &vertex_buffers_);
			vertex_buffers_ = 0;
			return;
		}
	}

	nPoints_ = cloud->size();

	if(polygons.size())
	{
		int polygonSize = polygons[0].vertices.size();
		UASSERT(polygonSize == 3);
		polygons_.resize(polygons.size() * polygonSize);
		int oi = 0;
		for(unsigned int i=0; i<polygons.size(); ++i)
		{
			UASSERT((int)polygons[i].vertices.size() == polygonSize);
			for(int j=0; j<polygonSize; ++j)
			{
				polygons_[oi++] = (unsigned short)polygons[i].vertices[j];
			}
		}
	}
}

PointCloudDrawable::~PointCloudDrawable()
{
	LOGI("Freeing cloud buffer %d", vertex_buffers_);
	if (vertex_buffers_)
	{
		glDeleteBuffers(1, &vertex_buffers_);
		tango_gl::util::CheckGlError("PointCloudDrawable::~PointCloudDrawable()");
		vertex_buffers_ = 0;
	}

	if (textures_)
	{
		glDeleteTextures(1, &textures_);
		tango_gl::util::CheckGlError("PointCloudDrawable::~PointCloudDrawable()");
		textures_ = 0;
	}
}

void PointCloudDrawable::setPose(const rtabmap::Transform & pose)
{
	UASSERT(!pose.isNull());

	pose_ = glmFromTransform(pose);
}

void PointCloudDrawable::Render(const glm::mat4 & projectionMatrix, const glm::mat4 & viewMatrix, bool meshRendering, float pointSize) {

	if(vertex_buffers_ && nPoints_ && visible_)
	{
		if(meshRendering && textures_)
		{
			glUseProgram(texture_shader_program_);

			GLuint mvp_handle_ = glGetUniformLocation(texture_shader_program_, "mvp");
			glm::mat4 mvp_mat = projectionMatrix * viewMatrix * pose_;
			glUniformMatrix4fv(mvp_handle_, 1, GL_FALSE, glm::value_ptr(mvp_mat));

			// Texture activate unit 0
			glActiveTexture(GL_TEXTURE0);
			// Bind the texture to this unit.
			glBindTexture(GL_TEXTURE_2D, textures_);
			// Tell the texture uniform sampler to use this texture in the shader by binding to texture unit 0.
			GLuint texture_handle = glGetUniformLocation(texture_shader_program_, "u_Texture");
			glUniform1i(texture_handle, 0);

			GLint attribute_vertex = glGetAttribLocation(texture_shader_program_, "vertex");
			GLint attribute_texture = glGetAttribLocation(texture_shader_program_, "a_TexCoordinate");

			glEnableVertexAttribArray(attribute_vertex);
			glEnableVertexAttribArray(attribute_texture);
			glBindBuffer(GL_ARRAY_BUFFER, vertex_buffers_);
			glVertexAttribPointer(attribute_vertex, 3, GL_FLOAT, GL_FALSE, 6*sizeof(GLfloat), 0);
			glVertexAttribPointer(attribute_texture, 2, GL_FLOAT, GL_FALSE, 6*sizeof(GLfloat), (GLvoid*) (4 * sizeof(GLfloat)));

			glDrawElements(GL_TRIANGLES, polygons_.size(), GL_UNSIGNED_SHORT, polygons_.data());
		}
		else // point cloud or colored mesh
		{
			glUseProgram(cloud_shader_program_);

			GLuint mvp_handle_ = glGetUniformLocation(cloud_shader_program_, "mvp");
			glm::mat4 mvp_mat = projectionMatrix * viewMatrix * pose_;
			glUniformMatrix4fv(mvp_handle_, 1, GL_FALSE, glm::value_ptr(mvp_mat));

			GLuint point_size_handle_ = glGetUniformLocation(cloud_shader_program_, "point_size");
			glUniform1f(point_size_handle_, pointSize);

			GLint attribute_vertex = glGetAttribLocation(cloud_shader_program_, "vertex");
			GLint attribute_color = glGetAttribLocation(cloud_shader_program_, "color");

			glEnableVertexAttribArray(attribute_vertex);
			glEnableVertexAttribArray(attribute_color);
			glBindBuffer(GL_ARRAY_BUFFER, vertex_buffers_);
			if(textures_)
			{
				glVertexAttribPointer(attribute_vertex, 3, GL_FLOAT, GL_FALSE, 6*sizeof(GLfloat), 0);
				glVertexAttribPointer(attribute_color, 3, GL_UNSIGNED_BYTE, GL_TRUE, 6*sizeof(GLfloat), (GLvoid*) (3 * sizeof(GLfloat)));
			}
			else
			{
				glVertexAttribPointer(attribute_vertex, 3, GL_FLOAT, GL_FALSE, 4*sizeof(GLfloat), 0);
				glVertexAttribPointer(attribute_color, 3, GL_UNSIGNED_BYTE, GL_TRUE, 4*sizeof(GLfloat), (GLvoid*) (3 * sizeof(GLfloat)));
			}
			if(meshRendering && polygons_.size())
			{
				glDrawElements(GL_TRIANGLES, polygons_.size(), GL_UNSIGNED_SHORT, polygons_.data());
			}
			else
			{
				glDrawArrays(GL_POINTS, 0, nPoints_);
			}
		}
		glDisableVertexAttribArray(0);
		glBindBuffer(GL_ARRAY_BUFFER, 0);

		glUseProgram(0);
		tango_gl::util::CheckGlError("Pointcloud::Render()");
	}
}

