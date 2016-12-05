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
		const pcl::IndicesPtr & indices,
		float gain) :
		vertex_buffers_(0),
		textures_(0),
		nPoints_(0),
		pose_(1.0f),
		visible_(true),
		cloud_shader_program_(cloudShaderProgram),
		texture_shader_program_(textureShaderProgram),
		gain_(1.0f)
{
	updateCloud(cloud, indices, gain);
}

PointCloudDrawable::PointCloudDrawable(
		GLuint cloudShaderProgram,
		GLuint textureShaderProgram,
		const Mesh & mesh,
		const cv::Mat & texture) :
		vertex_buffers_(0),
		textures_(0),
		nPoints_(0),
		pose_(1.0f),
		visible_(true),
		cloud_shader_program_(cloudShaderProgram),
		texture_shader_program_(textureShaderProgram),
		gain_(1.0f)
{
	updateMesh(mesh, texture);
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

void PointCloudDrawable::updatePolygons(const std::vector<pcl::Vertices> & polygons)
{
	polygons_.clear();
	if(polygons.size() && organizedToDenseIndices_.size())
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
				polygons_[oi++] = organizedToDenseIndices_.at((unsigned short)polygons[i].vertices[j]);
			}
		}
	}
}

void PointCloudDrawable::updateCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud, const pcl::IndicesPtr & indices, float gain)
{
	UASSERT(cloud.get() && !cloud->empty() && indices.get() && !indices->empty());
	nPoints_ = 0;
	polygons_.clear();
	gain_ = gain;

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

	glGenBuffers(1, &vertex_buffers_);
	if(!vertex_buffers_)
	{
		LOGE("OpenGL: could not generate vertex buffers\n");
		return;
	}

	LOGI("Creating cloud buffer %d", vertex_buffers_);
	std::vector<float> vertices(indices->size()*4);
	for(unsigned int i=0; i<indices->size(); ++i)
	{
		vertices[i*4] = cloud->at(indices->at(i)).x;
		vertices[i*4+1] = cloud->at(indices->at(i)).y;
		vertices[i*4+2] = cloud->at(indices->at(i)).z;
		vertices[i*4+3] = cloud->at(indices->at(i)).rgb;
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

	nPoints_ = indices->size();
}

void PointCloudDrawable::updateMesh(const Mesh & mesh, const cv::Mat & texture)
{
	UASSERT(mesh.cloud.get() && !mesh.cloud->empty() && mesh.indices.get() && !mesh.indices->empty());
	nPoints_ = 0;

	if (vertex_buffers_)
	{
		glDeleteBuffers(1, &vertex_buffers_);
		tango_gl::util::CheckGlError("PointCloudDrawable::~PointCloudDrawable()");
		vertex_buffers_ = 0;
	}

	gain_ = mesh.gain;

	bool textureUpdate = false;
	if(!texture.empty() && texture.type() == CV_8UC3)
	{
		if (textures_)
		{
			glDeleteTextures(1, &textures_);
			tango_gl::util::CheckGlError("PointCloudDrawable::~PointCloudDrawable()");
			textures_ = 0;
		}
		textureUpdate = true;
	}

	glGenBuffers(1, &vertex_buffers_);
	if(!vertex_buffers_)
	{
		LOGE("OpenGL: could not generate vertex buffers\n");
		return;
	}

	if(textureUpdate)
	{
		UASSERT(!mesh.cloud->is_dense);
		glGenTextures(1, &textures_);
		if(!textures_)
		{
			vertex_buffers_ = 0;
			LOGE("OpenGL: could not generate texture buffers\n");
			return;
		}
	}

	LOGI("Creating cloud buffer %d", vertex_buffers_);
	std::vector<float> vertices;
	organizedToDenseIndices_ = std::vector<int>(mesh.cloud->width*mesh.cloud->height, -1);
	if(textures_)
	{
		vertices = std::vector<float>(mesh.indices->size()*6);
		for(unsigned int i=0; i<mesh.indices->size(); ++i)
		{
			vertices[i*6] = mesh.cloud->at(mesh.indices->at(i)).x;
			vertices[i*6+1] = mesh.cloud->at(mesh.indices->at(i)).y;
			vertices[i*6+2] = mesh.cloud->at(mesh.indices->at(i)).z;

			// rgb
			vertices[i*6+3] = mesh.cloud->at(mesh.indices->at(i)).rgb;

			// texture uv
			int index = mesh.indices->at(i);
			vertices[i*6+4] = float(index % mesh.cloud->width)/float(mesh.cloud->width); //u
			vertices[i*6+5] = float(index / mesh.cloud->width)/float(mesh.cloud->height);  //v

			organizedToDenseIndices_[mesh.indices->at(i)] = i;
		}
	}
	else
	{
		vertices = std::vector<float>(mesh.indices->size()*4);
		for(unsigned int i=0; i<mesh.indices->size(); ++i)
		{
			vertices[i*4] = mesh.cloud->at(mesh.indices->at(i)).x;
			vertices[i*4+1] = mesh.cloud->at(mesh.indices->at(i)).y;
			vertices[i*4+2] = mesh.cloud->at(mesh.indices->at(i)).z;
			vertices[i*4+3] = mesh.cloud->at(mesh.indices->at(i)).rgb;
			organizedToDenseIndices_[mesh.indices->at(i)] = i;
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

	if(textures_ && textureUpdate)
	{
		// gen texture from image
		glBindTexture(GL_TEXTURE_2D, textures_);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		cv::Mat rgbImage;
		cv::cvtColor(texture, rgbImage, CV_BGR2RGB);
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

	nPoints_ = mesh.indices->size();

	if(polygons_.size() != mesh.polygons.size())
	{
		updatePolygons(mesh.polygons);
	}
}

void PointCloudDrawable::setPose(const rtabmap::Transform & pose)
{
	UASSERT(!pose.isNull());

	pose_ = glmFromTransform(pose);
}

void PointCloudDrawable::Render(const glm::mat4 & projectionMatrix, const glm::mat4 & viewMatrix, bool meshRendering, float pointSize, bool textureRendering) {

	if(vertex_buffers_ && nPoints_ && visible_)
	{
		if(meshRendering && textureRendering && textures_)
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

			GLuint gain_handle = glGetUniformLocation(texture_shader_program_, "u_gain");
			glUniform1f(gain_handle, gain_);

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

			GLuint gain_handle = glGetUniformLocation(cloud_shader_program_, "u_gain");
			glUniform1f(gain_handle, gain_);

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

