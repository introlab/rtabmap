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

#define LOW_DEC 2
#define LOWLOW_DEC 4

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
		hasNormals_(false),
		cloud_shader_program_(cloudShaderProgram),
		texture_shader_program_(textureShaderProgram),
		gain_(1.0f)
{
	updateCloud(cloud, indices, gain);
}

PointCloudDrawable::PointCloudDrawable(
		GLuint cloudShaderProgram,
		GLuint textureShaderProgram,
		const Mesh & mesh) :
		vertex_buffers_(0),
		textures_(0),
		nPoints_(0),
		pose_(1.0f),
		visible_(true),
		hasNormals_(false),
		cloud_shader_program_(cloudShaderProgram),
		texture_shader_program_(textureShaderProgram),
		gain_(1.0f)
{
	updateMesh(mesh);
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

void PointCloudDrawable::updatePolygons(const std::vector<pcl::Vertices> & polygons, const std::vector<pcl::Vertices> & polygonsLowRes)
{
	LOGD("Update polygons");
	polygons_.clear();
	if(polygons.size() && organizedToDenseIndices_.size())
	{
		unsigned int polygonSize = polygons[0].vertices.size();
		UASSERT(polygonSize == 3);
		polygons_.resize(polygons.size() * polygonSize);
		int oi = 0;
		for(unsigned int i=0; i<polygons.size(); ++i)
		{
			UASSERT(polygons[i].vertices.size() == polygonSize);
			for(unsigned int j=0; j<polygonSize; ++j)
			{
				polygons_[oi++] = organizedToDenseIndices_.at(polygons[i].vertices[j]);
			}
		}

		if(polygonsLowRes.size())
		{
			unsigned int polygonSize = polygonsLowRes[0].vertices.size();
			UASSERT(polygonSize == 3);
			polygonsLowRes_.resize(polygonsLowRes.size() * polygonSize);
			int oi = 0;
			for(unsigned int i=0; i<polygonsLowRes.size(); ++i)
			{
				UASSERT(polygonsLowRes[i].vertices.size() == polygonSize);
				for(unsigned int j=0; j<polygonSize; ++j)
				{
					polygonsLowRes_[oi++] = organizedToDenseIndices_.at(polygonsLowRes[i].vertices[j]);
				}
			}
		}
	}
}

void PointCloudDrawable::updateCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud, const pcl::IndicesPtr & indices, float gain)
{
	UASSERT(cloud.get() && !cloud->empty());
	nPoints_ = 0;
	polygons_.clear();
	polygonsLowRes_.clear();
	gain_ = gain;
	verticesLowRes_.clear();
	verticesLowLowRes_.clear();

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
	std::vector<float> vertices;
	int totalPoints = 0;
	if(indices.get() && indices->size())
	{
		totalPoints = indices->size();
		vertices.resize(indices->size()*4);
		verticesLowRes_.resize(cloud->isOrganized()?totalPoints:0);
		verticesLowLowRes_.resize(cloud->isOrganized()?totalPoints:0);
		int oi_low = 0;
		int oi_lowlow = 0;
		for(unsigned int i=0; i<indices->size(); ++i)
		{
			vertices[i*4] = cloud->at(indices->at(i)).x;
			vertices[i*4+1] = cloud->at(indices->at(i)).y;
			vertices[i*4+2] = cloud->at(indices->at(i)).z;
			vertices[i*4+3] = cloud->at(indices->at(i)).rgb;

			if(cloud->isOrganized())
			{
				if(indices->at(i)%LOW_DEC == 0 && (indices->at(i)/cloud->width) % LOW_DEC == 0)
				{
					verticesLowRes_[oi_low++] = i;
				}
				if(indices->at(i)%LOWLOW_DEC == 0 && (indices->at(i)/cloud->width) % LOWLOW_DEC == 0)
				{
					verticesLowLowRes_[oi_lowlow++] = i;
				}
			}
		}
		verticesLowRes_.resize(oi_low);
		verticesLowLowRes_.resize(oi_lowlow);
	}
	else
	{
		totalPoints = cloud->size();
		vertices.resize(cloud->size()*4);
		verticesLowRes_.resize(cloud->isOrganized()?totalPoints:0);
		verticesLowLowRes_.resize(cloud->isOrganized()?totalPoints:0);
		int oi_low = 0;
		int oi_lowlow = 0;
		for(unsigned int i=0; i<cloud->size(); ++i)
		{
			vertices[i*4] = cloud->at(i).x;
			vertices[i*4+1] = cloud->at(i).y;
			vertices[i*4+2] = cloud->at(i).z;
			vertices[i*4+3] = cloud->at(i).rgb;

			if(cloud->isOrganized())
			{
				if(i%LOW_DEC == 0 && (i/cloud->width) % LOW_DEC == 0)
				{
					verticesLowRes_[oi_low++] = i;
				}
				if(i%LOWLOW_DEC == 0 && (i/cloud->width) % LOWLOW_DEC == 0)
				{
					verticesLowLowRes_[oi_lowlow++] = i;
				}
			}
		}
		verticesLowRes_.resize(oi_low);
		verticesLowLowRes_.resize(oi_lowlow);
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

	nPoints_ = totalPoints;
}

void PointCloudDrawable::updateMesh(const Mesh & mesh)
{
	UASSERT(mesh.cloud.get() && !mesh.cloud->empty());
	nPoints_ = 0;

	if (vertex_buffers_)
	{
		glDeleteBuffers(1, &vertex_buffers_);
		tango_gl::util::CheckGlError("PointCloudDrawable::~PointCloudDrawable()");
		vertex_buffers_ = 0;
	}

	gain_ = mesh.gain;

	bool textureUpdate = false;
	if(!mesh.texture.empty() && mesh.texture.type() == CV_8UC3)
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
		glGenTextures(1, &textures_);
		if(!textures_)
		{
			vertex_buffers_ = 0;
			LOGE("OpenGL: could not generate texture buffers\n");
			return;
		}
	}

	//LOGD("Creating cloud buffer %d", vertex_buffers_);
	std::vector<float> vertices;
	int totalPoints = 0;
	std::vector<pcl::Vertices> polygons = mesh.polygons;
	std::vector<pcl::Vertices> polygonsLowRes;
	hasNormals_ = mesh.normals.get() && mesh.normals->size() == mesh.cloud->size();
	UASSERT(!hasNormals_ || mesh.cloud->size() == mesh.normals->size());
	if(mesh.cloud->isOrganized()) // assume organized mesh
	{
		polygonsLowRes = mesh.polygonsLowRes; // only in organized we keep the low res
		organizedToDenseIndices_ = std::vector<unsigned int>(mesh.cloud->width*mesh.cloud->height, -1);
		totalPoints = mesh.indices->size();
		verticesLowRes_.resize(totalPoints);
		verticesLowLowRes_.resize(totalPoints);
		int oi_low = 0;
		int oi_lowlow = 0;
		if(textures_ && polygons.size())
		{
			//LOGD("Organized mesh with texture");
			int items = hasNormals_?9:6;
			vertices = std::vector<float>(mesh.indices->size()*9);
			for(unsigned int i=0; i<mesh.indices->size(); ++i)
			{
				vertices[i*items] = mesh.cloud->at(mesh.indices->at(i)).x;
				vertices[i*items+1] = mesh.cloud->at(mesh.indices->at(i)).y;
				vertices[i*items+2] = mesh.cloud->at(mesh.indices->at(i)).z;

				// rgb
				vertices[i*items+3] = mesh.cloud->at(mesh.indices->at(i)).rgb;

				// texture uv
				int index = mesh.indices->at(i);
				vertices[i*items+4] = float(index % mesh.cloud->width)/float(mesh.cloud->width); //u
				vertices[i*items+5] = float(index / mesh.cloud->width)/float(mesh.cloud->height);  //v

				if(hasNormals_)
				{
					// normal
					vertices[i*items+6] = mesh.normals->at(mesh.indices->at(i)).normal_x;
					vertices[i*items+7] = mesh.normals->at(mesh.indices->at(i)).normal_y;
					vertices[i*items+8] = mesh.normals->at(mesh.indices->at(i)).normal_z;
				}

				organizedToDenseIndices_[mesh.indices->at(i)] = i;

				if(mesh.indices->at(i)%LOW_DEC == 0 && (mesh.indices->at(i)/mesh.cloud->width) % LOW_DEC == 0)
				{
					verticesLowRes_[oi_low++] = i;
				}
				if(mesh.indices->at(i)%LOWLOW_DEC == 0 && (mesh.indices->at(i)/mesh.cloud->width) % LOWLOW_DEC == 0)
				{
					verticesLowLowRes_[oi_lowlow++] = i;
				}
			}
		}
		else
		{
			//LOGD("Organized mesh");
			int items = hasNormals_?7:4;
			vertices = std::vector<float>(mesh.indices->size()*items);
			for(unsigned int i=0; i<mesh.indices->size(); ++i)
			{
				vertices[i*items] = mesh.cloud->at(mesh.indices->at(i)).x;
				vertices[i*items+1] = mesh.cloud->at(mesh.indices->at(i)).y;
				vertices[i*items+2] = mesh.cloud->at(mesh.indices->at(i)).z;
				vertices[i*items+3] = mesh.cloud->at(mesh.indices->at(i)).rgb;

				if(hasNormals_)
				{
					// normal
					vertices[i*items+4] = mesh.normals->at(mesh.indices->at(i)).normal_x;
					vertices[i*items+5] = mesh.normals->at(mesh.indices->at(i)).normal_y;
					vertices[i*items+6] = mesh.normals->at(mesh.indices->at(i)).normal_z;
				}

				organizedToDenseIndices_[mesh.indices->at(i)] = i;

				if(mesh.indices->at(i)%LOW_DEC == 0 && (mesh.indices->at(i)/mesh.cloud->width) % LOW_DEC == 0)
				{
					verticesLowRes_[oi_low++] = i;
				}
				if(mesh.indices->at(i)%LOWLOW_DEC == 0 && (mesh.indices->at(i)/mesh.cloud->width) % LOWLOW_DEC == 0)
				{
					verticesLowLowRes_[oi_lowlow++] = i;
				}
			}
		}
		verticesLowRes_.resize(oi_low);
		verticesLowLowRes_.resize(oi_lowlow);
	}
	else // assume dense mesh with texCoords set to polygons
	{
		totalPoints = mesh.cloud->size();
		if(textures_ && polygons.size() && mesh.normals->size())
		{
			//LOGD("Dense mesh with texture (%d texCoords %d points %d polygons %dx%d)",
			//		(int)mesh.texCoords.size(), (int)mesh.cloud->size(), (int)mesh.polygons.size(), texture.cols, texture.rows);

			// Texturing issue:
			//  tex_coordinates should be linked to points, not
			//  polygon vertices. Points linked to multiple different texCoords (different textures) should
			//  be duplicated.
			vertices = std::vector<float>(mesh.texCoords.size()*9);
			organizedToDenseIndices_ = std::vector<unsigned int>(mesh.texCoords.size(), -1);

			UASSERT_MSG(mesh.texCoords.size() == polygons[0].vertices.size()*polygons.size(),
					uFormat("%d vs %d x %d", (int)mesh.texCoords.size(), (int)polygons[0].vertices.size(), (int)polygons.size()).c_str());

			int items = hasNormals_?9:6;
			unsigned int oi=0;
			for(unsigned int i=0; i<polygons.size(); ++i)
			{
				pcl::Vertices & v = polygons[i];
				for(unsigned int j=0; j<v.vertices.size(); ++j)
				{
					UASSERT(oi < mesh.texCoords.size());
					UASSERT(v.vertices[j] < mesh.cloud->size());

					vertices[oi*items] = mesh.cloud->at(v.vertices[j]).x;
					vertices[oi*items+1] = mesh.cloud->at(v.vertices[j]).y;
					vertices[oi*items+2] = mesh.cloud->at(v.vertices[j]).z;

					// rgb
					vertices[oi*items+3] = mesh.cloud->at(v.vertices[j]).rgb;

					// texture uv
					if(mesh.texCoords[oi][0]>=0.0f)
					{
						vertices[oi*items+4] = mesh.texCoords[oi][0]; //u
						vertices[oi*items+5] = 1.0f-mesh.texCoords[oi][1];  //v
					}
					else
					{
						vertices[oi*items+4] = vertices[oi*items+5] = -1.0f;
					}

					if(hasNormals_)
					{
						// normal
						vertices[oi*items+6] = mesh.normals->at(v.vertices[j]).normal_x;
						vertices[oi*items+7] = mesh.normals->at(v.vertices[j]).normal_y;
						vertices[oi*items+8] = mesh.normals->at(v.vertices[j]).normal_z;
					}

					v.vertices[j] = (int)oi; // new vertex index

					UASSERT(oi < organizedToDenseIndices_.size());
					organizedToDenseIndices_[oi] = oi;

					++oi;
				}
			}
		}
		else
		{
			//LOGD("Dense mesh");
			int items = hasNormals_?7:4;
			organizedToDenseIndices_ = std::vector<unsigned int>(mesh.cloud->size(), -1);
			vertices = std::vector<float>(mesh.cloud->size()*items);
			for(unsigned int i=0; i<mesh.cloud->size(); ++i)
			{
				vertices[i*items] = mesh.cloud->at(i).x;
				vertices[i*items+1] = mesh.cloud->at(i).y;
				vertices[i*items+2] = mesh.cloud->at(i).z;
				vertices[i*items+3] = mesh.cloud->at(i).rgb;

				if(hasNormals_)
				{
					vertices[i*items+4] = mesh.normals->at(i).normal_x;
					vertices[i*items+5] = mesh.normals->at(i).normal_y;
					vertices[i*items+6] = mesh.normals->at(i).normal_z;
				}

				organizedToDenseIndices_[i] = i;
			}
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
		//GLint maxTextureSize = 0;
		//glGetIntegerv(GL_MAX_TEXTURE_SIZE, &maxTextureSize);
		//LOGI("maxTextureSize=%d", maxTextureSize);
		//GLint maxTextureUnits = 0;
		//glGetIntegerv(GL_MAX_TEXTURE_IMAGE_UNITS, &maxTextureUnits);
		//LOGW("maxTextureUnits=%d", maxTextureUnits);

		// gen texture from image
		glBindTexture(GL_TEXTURE_2D, textures_);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		cv::Mat rgbImage;
		cv::cvtColor(mesh.texture, rgbImage, CV_BGR2RGB);

		glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
		//glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
		//glPixelStorei(GL_UNPACK_SKIP_PIXELS, 0);
		//glPixelStorei(GL_UNPACK_SKIP_ROWS, 0);
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

	nPoints_ = totalPoints;

	if(polygons_.size() != polygons.size())
	{
		updatePolygons(polygons, polygonsLowRes);
	}
}

void PointCloudDrawable::setPose(const rtabmap::Transform & pose)
{
	UASSERT(!pose.isNull());

	pose_ = glmFromTransform(pose);
}

void PointCloudDrawable::Render(const glm::mat4 & projectionMatrix,
		const glm::mat4 & viewMatrix,
		bool meshRendering,
		float pointSize,
		bool textureRendering,
		bool lighting,
		float distanceToCameraSqr) {

	if(vertex_buffers_ && nPoints_ && visible_)
	{
		if(meshRendering && textureRendering && textures_ && (verticesLowRes_.empty() || distanceToCameraSqr<50.0f))
		{
			glUseProgram(texture_shader_program_);

			GLuint mvp_handle = glGetUniformLocation(texture_shader_program_, "uMVP");
			glm::mat4 mv_mat = viewMatrix * pose_;
			glm::mat4 mvp_mat = projectionMatrix * mv_mat;
			glUniformMatrix4fv(mvp_handle, 1, GL_FALSE, glm::value_ptr(mvp_mat));

			GLuint n_handle = glGetUniformLocation(texture_shader_program_, "uN");
			glm::mat3 normalMatrix(mv_mat);
			normalMatrix = glm::inverse(normalMatrix);
			normalMatrix = glm::transpose(normalMatrix);
			glUniformMatrix3fv(n_handle, 1, GL_FALSE, glm::value_ptr(normalMatrix));

			if(!hasNormals_)
			{
				lighting = false;
			}

			//lighting
			GLuint lighting_handle = glGetUniformLocation(texture_shader_program_, "uUseLighting");
			glUniform1i(lighting_handle, lighting?1:0);

			if(lighting)
			{
				GLuint ambiant_handle = glGetUniformLocation(texture_shader_program_, "uAmbientColor");
				glUniform3f(ambiant_handle,0.6,0.6,0.6);

				GLuint lightingDirection_handle = glGetUniformLocation(texture_shader_program_, "uLightingDirection");
				glUniform3f(lightingDirection_handle, 0.0, 0.0, 1.0); // from the camera
			}

			// Texture activate unit 0
			glActiveTexture(GL_TEXTURE0);
			// Bind the texture to this unit.
			glBindTexture(GL_TEXTURE_2D, textures_);
			// Tell the texture uniform sampler to use this texture in the shader by binding to texture unit 0.
			GLuint texture_handle = glGetUniformLocation(texture_shader_program_, "uTexture");
			glUniform1i(texture_handle, 0);

			GLuint gain_handle = glGetUniformLocation(texture_shader_program_, "uGain");
			glUniform1f(gain_handle, gain_);

			GLint attribute_vertex = glGetAttribLocation(texture_shader_program_, "aVertex");
			GLint attribute_texture = glGetAttribLocation(texture_shader_program_, "aTexCoord");
			GLint attribute_normal=0;
			if(hasNormals_)
			{
				attribute_normal = glGetAttribLocation(texture_shader_program_, "aNormal");
			}

			glEnableVertexAttribArray(attribute_vertex);
			glEnableVertexAttribArray(attribute_texture);
			if(hasNormals_)
			{
				glEnableVertexAttribArray(attribute_normal);
			}
			glBindBuffer(GL_ARRAY_BUFFER, vertex_buffers_);
			glVertexAttribPointer(attribute_vertex, 3, GL_FLOAT, GL_FALSE, (hasNormals_?9:6)*sizeof(GLfloat), 0);
			glVertexAttribPointer(attribute_texture, 2, GL_FLOAT, GL_FALSE, (hasNormals_?9:6)*sizeof(GLfloat), (GLvoid*) (4 * sizeof(GLfloat)));
			if(hasNormals_)
			{
				glVertexAttribPointer(attribute_normal, 3, GL_FLOAT, GL_FALSE, 9*sizeof(GLfloat), (GLvoid*) (6 * sizeof(GLfloat)));
			}
			if(distanceToCameraSqr<150.0f || polygonsLowRes_.empty())
			{
				glDrawElements(GL_TRIANGLES, polygons_.size(), GL_UNSIGNED_INT, polygons_.data());
			}
			else
			{
				glDrawElements(GL_TRIANGLES, polygonsLowRes_.size(), GL_UNSIGNED_INT, polygonsLowRes_.data());
			}
		}
		else // point cloud or colored mesh
		{
			glUseProgram(cloud_shader_program_);

			GLuint mvp_handle_ = glGetUniformLocation(cloud_shader_program_, "uMVP");
			glm::mat4 mv_mat = viewMatrix * pose_;
			glm::mat4 mvp_mat = projectionMatrix * mv_mat;
			glUniformMatrix4fv(mvp_handle_, 1, GL_FALSE, glm::value_ptr(mvp_mat));

			GLuint n_handle = glGetUniformLocation(texture_shader_program_, "uN");
			glm::mat3 normalMatrix(mv_mat);
			normalMatrix = glm::inverse(normalMatrix);
			normalMatrix = glm::transpose(normalMatrix);
			glUniformMatrix3fv(n_handle, 1, GL_FALSE, glm::value_ptr(normalMatrix));

			if(!hasNormals_)
			{
				lighting = false;
			}

			//lighting
			GLuint lighting_handle = glGetUniformLocation(texture_shader_program_, "uUseLighting");
			glUniform1i(lighting_handle, lighting?1:0);

			if(lighting)
			{
				GLuint ambiant_handle = glGetUniformLocation(texture_shader_program_, "uAmbientColor");
				glUniform3f(ambiant_handle,0.6,0.6,0.6);

				GLuint lightingDirection_handle = glGetUniformLocation(texture_shader_program_, "uLightingDirection");
				glUniform3f(lightingDirection_handle, 0.0, 0.0, 1.0); // from the camera
			}

			GLuint point_size_handle_ = glGetUniformLocation(cloud_shader_program_, "uPointSize");
			glUniform1f(point_size_handle_, pointSize);

			GLuint gain_handle = glGetUniformLocation(cloud_shader_program_, "uGain");
			glUniform1f(gain_handle, gain_);

			GLint attribute_vertex = glGetAttribLocation(cloud_shader_program_, "aVertex");
			GLint attribute_color = glGetAttribLocation(cloud_shader_program_, "aColor");
			GLint attribute_normal=0;
			if(hasNormals_)
			{
				attribute_normal = glGetAttribLocation(cloud_shader_program_, "aNormal");
			}

			glEnableVertexAttribArray(attribute_vertex);
			glEnableVertexAttribArray(attribute_color);
			if(hasNormals_)
			{
				glEnableVertexAttribArray(attribute_normal);
			}
			glBindBuffer(GL_ARRAY_BUFFER, vertex_buffers_);
			if(textures_)
			{
				glVertexAttribPointer(attribute_vertex, 3, GL_FLOAT, GL_FALSE, (hasNormals_?9:6)*sizeof(GLfloat), 0);
				glVertexAttribPointer(attribute_color, 3, GL_UNSIGNED_BYTE, GL_TRUE,  (hasNormals_?9:6)*sizeof(GLfloat), (GLvoid*) (3 * sizeof(GLfloat)));
				if(hasNormals_)
				{
					glVertexAttribPointer(attribute_normal, 3, GL_FLOAT, GL_FALSE, 9*sizeof(GLfloat), (GLvoid*) (6 * sizeof(GLfloat)));
				}
			}
			else
			{
				glVertexAttribPointer(attribute_vertex, 3, GL_FLOAT, GL_FALSE, (hasNormals_?7:4)*sizeof(GLfloat), 0);
				glVertexAttribPointer(attribute_color, 3, GL_UNSIGNED_BYTE, GL_TRUE, (hasNormals_?7:4)*sizeof(GLfloat), (GLvoid*) (3 * sizeof(GLfloat)));
				if(hasNormals_)
				{
					glVertexAttribPointer(attribute_normal, 3, GL_FLOAT, GL_FALSE, 7*sizeof(GLfloat), (GLvoid*) (4 * sizeof(GLfloat)));
				}
			}
			if(meshRendering && polygons_.size())
			{
				if(distanceToCameraSqr<150.0f || polygonsLowRes_.empty())
				{
					glDrawElements(GL_TRIANGLES, polygons_.size(), GL_UNSIGNED_INT, polygons_.data());
				}
				else
				{
					glDrawElements(GL_TRIANGLES, polygonsLowRes_.size(), GL_UNSIGNED_INT, polygonsLowRes_.data());
				}
			}
			else if(!verticesLowRes_.empty())
			{
				if(distanceToCameraSqr>600.0f)
				{
					glDrawElements(GL_POINTS, verticesLowLowRes_.size(), GL_UNSIGNED_INT, verticesLowLowRes_.data());
				}
				else if(distanceToCameraSqr>150.0f)
				{
					glDrawElements(GL_POINTS, verticesLowRes_.size(), GL_UNSIGNED_INT, verticesLowRes_.data());
				}
				else
				{
					glDrawArrays(GL_POINTS, 0, nPoints_);
				}
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

