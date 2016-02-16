
/*
 * Copyright 2014 Google Inc. All Rights Reserved.
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

#include <sstream>

#include "point_cloud_drawable.h"
#include "rtabmap/utilite/ULogger.h"
#include "util.h"

#include <GLES2/gl2.h>

PointCloudDrawable::PointCloudDrawable(
		GLuint shaderProgram,
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud,
		const std::vector<pcl::Vertices> & indices) :
		vertex_buffers_(0),
		nPoints_(0),
		pose_(1.0f),
		visible_(true),
		shader_program_(shaderProgram)
{
	UASSERT(!cloud->empty());

	glGenBuffers(1, &vertex_buffers_);

	if(vertex_buffers_)
	{
		LOGI("Creating cloud buffer %d", vertex_buffers_);
		std::vector<float> vertices = std::vector<float>(cloud->size()*4);
		for(unsigned int i=0; i<cloud->size(); ++i)
		{
			vertices[i*4] = cloud->at(i).x;
			vertices[i*4+1] = cloud->at(i).y;
			vertices[i*4+2] = cloud->at(i).z;
			vertices[i*4+3] = cloud->at(i).rgb;
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
		else
		{
			nPoints_ = cloud->size();

			if(indices.size())
			{
				int polygonSize = indices[0].vertices.size();
				UASSERT(polygonSize == 3);
				indices_.resize(indices.size() * polygonSize);
				int oi = 0;
				for(unsigned int i=0; i<indices.size(); ++i)
				{
					UASSERT((int)indices[i].vertices.size() == polygonSize);
					for(int j=0; j<polygonSize; ++j)
					{
						indices_[oi++] = (unsigned short)indices[i].vertices[j];
					}
				}
			}
		}
	}
}
PointCloudDrawable::PointCloudDrawable(
		GLuint shaderProgram,
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		const std::vector<pcl::Vertices> & indices) :
		vertex_buffers_(0),
		nPoints_(0),
		pose_(1.0f),
		visible_(true),
		shader_program_(shaderProgram)
{
	UASSERT(!cloud->empty());

	glGenBuffers(1, &vertex_buffers_);

	if(vertex_buffers_)
	{
		LOGI("Creating cloud buffer %d", vertex_buffers_);
		std::vector<float> vertices = std::vector<float>(cloud->size()*4);
		for(unsigned int i=0; i<cloud->size(); ++i)
		{
			vertices[i*4] = cloud->at(i).x;
			vertices[i*4+1] = cloud->at(i).y;
			vertices[i*4+2] = cloud->at(i).z;
			vertices[i*4+3] = cloud->at(i).rgb;
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
		else
		{
			nPoints_ = cloud->size();

			if(indices.size())
			{
				int polygonSize = indices[0].vertices.size();
				UASSERT(polygonSize == 3);
				indices_.resize(indices.size() * polygonSize);
				int oi = 0;
				for(unsigned int i=0; i<indices.size(); ++i)
				{
					UASSERT((int)indices[i].vertices.size() == polygonSize);
					for(int j=0; j<polygonSize; ++j)
					{
						indices_[oi++] = (unsigned short)indices[i].vertices[j];
					}
				}
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
}

void PointCloudDrawable::setPose(const rtabmap::Transform & pose)
{
	UASSERT(!pose.isNull());

	pose_ = glmFromTransform(pose);
}

void PointCloudDrawable::Render(const glm::mat4 & projectionMatrix, const glm::mat4 & viewMatrix, bool meshRendering, float pointSize) {

	if(vertex_buffers_ && nPoints_ && visible_)
	{
		glUseProgram(shader_program_);

		GLuint mvp_handle_ = glGetUniformLocation(shader_program_, "mvp");
		glm::mat4 mvp_mat = projectionMatrix * viewMatrix * pose_;
		glUniformMatrix4fv(mvp_handle_, 1, GL_FALSE, glm::value_ptr(mvp_mat));

		GLuint point_size_handle_ = glGetUniformLocation(shader_program_, "point_size");
		glUniform1f(point_size_handle_, pointSize);

		GLint attribute_vertex = glGetAttribLocation(shader_program_, "vertex");
		GLint attribute_color = glGetAttribLocation(shader_program_, "color");

		glEnableVertexAttribArray(attribute_vertex);
		glEnableVertexAttribArray(attribute_color);
		glBindBuffer(GL_ARRAY_BUFFER, vertex_buffers_);
		glVertexAttribPointer(attribute_vertex, 3, GL_FLOAT, GL_FALSE, 4*sizeof(GLfloat), 0);
		glVertexAttribPointer(attribute_color, 3, GL_UNSIGNED_BYTE, GL_TRUE, 4*sizeof(GLfloat), (GLvoid*) (3 * sizeof(GLfloat)));

		if(meshRendering && indices_.size())
		{
			glDrawElements(GL_TRIANGLES, indices_.size(), GL_UNSIGNED_SHORT, indices_.data());
		}
		else
		{
			glDrawArrays(GL_POINTS, 0, nPoints_);
		}

		glDisableVertexAttribArray(0);
		glBindBuffer(GL_ARRAY_BUFFER, 0);

		glUseProgram(0);
		tango_gl::util::CheckGlError("Pointcloud::Render()");
	}
}

