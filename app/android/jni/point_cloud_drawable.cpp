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
#include "rtabmap/utilite/UTimer.h"
#include "rtabmap/utilite/UConversion.h"
#include <opencv2/imgproc/imgproc.hpp>
#include "util.h"
#include "pcl/common/transforms.h"

#ifdef __ANDROID__
#include <GLES2/gl2.h>
#else // __APPLE__
#include <OpenGLES/ES2/gl.h>
#endif

#define LOW_DEC 2
#define LOWLOW_DEC 4

enum PointCloudShaders
{
	kPointCloud = 0,
	kPointCloudBlending = 1,
	kPointCloudLighting = 2,
	kPointCloudLightingBlending = 3,

	kTexture = 4,
	kTextureBlending = 5,
	kTextureLighting = 6,
	kTextureLightingBlending = 7,

	kDepthPacking = 8
};

// PointCloud shaders
const std::string kPointCloudVertexShader =
    "precision highp float;\n"
    "precision mediump int;\n"
    "attribute vec3 aVertex;\n"
    "attribute vec3 aColor;\n"

    "uniform mat4 uMVP;\n"
    "uniform float uPointSize;\n"

    "varying vec3 vColor;\n"
    "varying float vLightWeighting;\n"

    "void main() {\n"
    "  gl_Position = uMVP*vec4(aVertex.x, aVertex.y, aVertex.z, 1.0);\n"
    "  gl_PointSize = uPointSize;\n"
	"  vLightWeighting = 1.0;\n"
    "  vColor = aColor;\n"
    "}\n";
const std::string kPointCloudLightingVertexShader =
    "precision highp float;\n"
    "precision mediump int;\n"
    "attribute vec3 aVertex;\n"
	"attribute vec3 aNormal;\n"
    "attribute vec3 aColor;\n"

    "uniform mat4 uMVP;\n"
	"uniform mat3 uN;\n"
	"uniform vec3 uLightingDirection;\n"
    "uniform float uPointSize;\n"

    "varying vec3 vColor;\n"
    "varying float vLightWeighting;\n"

    "void main() {\n"
    "  gl_Position = uMVP*vec4(aVertex.x, aVertex.y, aVertex.z, 1.0);\n"
    "  gl_PointSize = uPointSize;\n"
	"  vec3 transformedNormal = uN * aNormal;\n"
	"  vLightWeighting = max(dot(transformedNormal, uLightingDirection)*0.5+0.5, 0.0);\n"
	"  if(vLightWeighting<0.5)"
	"    vLightWeighting=0.5;\n"
    "  vColor = aColor;\n"
    "}\n";

const std::string kPointCloudFragmentShader =
    "precision highp float;\n"
    "precision mediump int;\n"
	"uniform float uGainR;\n"
	"uniform float uGainG;\n"
	"uniform float uGainB;\n"
    "varying vec3 vColor;\n"
	"varying float vLightWeighting;\n"
    "void main() {\n"
    "  vec4 textureColor = vec4(vColor.z, vColor.y, vColor.x, 1.0);\n"
	"  gl_FragColor = vec4(textureColor.r * uGainR * vLightWeighting, textureColor.g * uGainG * vLightWeighting, textureColor.b * uGainB * vLightWeighting, textureColor.a);\n"
    "}\n";
const std::string kPointCloudBlendingFragmentShader =
    "precision highp float;\n"
    "precision mediump int;\n"
	"uniform float uGainR;\n"
	"uniform float uGainG;\n"
	"uniform float uGainB;\n"
	"uniform float uNearZ;\n"
	"uniform float uFarZ;\n"
	"uniform sampler2D uDepthTexture;\n"
	"uniform vec2 uScreenScale;\n"
    "varying vec3 vColor;\n"
	"varying float vLightWeighting;\n"
    "void main() {\n"
    "  vec4 textureColor = vec4(vColor.z, vColor.y, vColor.x, 1.0);\n"
	"  float alpha = 1.0;\n"
	"  vec2 coord = uScreenScale * gl_FragCoord.xy;\n;"
	"  vec4 depthPacked = texture2D(uDepthTexture, coord);\n"
	"  float depth = dot(depthPacked, 1./vec4(1.,255.,65025.,16581375.));\n"
	"  float num =  (2.0 * uNearZ * uFarZ);\n"
	"  float diff = (uFarZ - uNearZ);\n"
	"  float add = (uFarZ + uNearZ);\n"
	"  float ndcDepth = depth * 2.0 - 1.0;\n" // Back to NDC
	"  float linearDepth = num / (add - ndcDepth * diff);\n" // inverse projection matrix
	"  float ndcFragz = gl_FragCoord.z * 2.0 - 1.0;\n" // Back to NDC
	"  float linearFragz = num / (add - ndcFragz * diff);\n" // inverse projection matrix
	"  if(linearFragz > linearDepth + 0.05)\n"
	"    alpha=0.0;\n"
	"  gl_FragColor = vec4(textureColor.r * uGainR * vLightWeighting, textureColor.g * uGainG * vLightWeighting, textureColor.b * uGainB * vLightWeighting, alpha);\n"
    "}\n";

const std::string kPointCloudDepthPackingVertexShader =
    "precision highp float;\n"
    "precision mediump int;\n"
    "attribute vec3 aVertex;\n"
    "uniform mat4 uMVP;\n"
    "uniform float uPointSize;\n"
    "void main() {\n"
    "  gl_Position = uMVP*vec4(aVertex.x, aVertex.y, aVertex.z, 1.0);\n"
    "  gl_PointSize = uPointSize;\n"
    "}\n";
const std::string kPointCloudDepthPackingFragmentShader =
    "precision highp float;\n"
    "precision mediump int;\n"
    "void main() {\n"
	"  vec4 enc = vec4(1.,255.,65025.,16581375.) * gl_FragCoord.z;\n"
	"  enc = fract(enc);\n"
	"  enc -= enc.yzww * vec2(1./255., 0.).xxxy;\n"
	"  gl_FragColor = enc;\n"
    "}\n";

// Texture shaders
const std::string kTextureMeshVertexShader =
    "precision highp float;\n"
    "precision mediump int;\n"
    "attribute vec3 aVertex;\n"
    "attribute vec2 aTexCoord;\n"

    "uniform mat4 uMVP;\n"

    "varying vec2 vTexCoord;\n"
    "varying float vLightWeighting;\n"

    "void main() {\n"
    "  gl_Position = uMVP*vec4(aVertex.x, aVertex.y, aVertex.z, 1.0);\n"

	"  if(aTexCoord.x < 0.0) {\n"
	"    vTexCoord.x = 1.0;\n"
	"    vTexCoord.y = 1.0;\n" // bottom right corner
	"  } else {\n"
    "    vTexCoord = aTexCoord;\n"
    "  }\n"

    "  vLightWeighting = 1.0;\n"
    "}\n";
const std::string kTextureMeshLightingVertexShader =
    "precision highp float;\n"
    "precision mediump int;\n"
    "attribute vec3 aVertex;\n"
	"attribute vec3 aNormal;\n"
    "attribute vec2 aTexCoord;\n"

    "uniform mat4 uMVP;\n"
    "uniform mat3 uN;\n"
    "uniform vec3 uLightingDirection;\n"

    "varying vec2 vTexCoord;\n"
    "varying float vLightWeighting;\n"

    "void main() {\n"
    "  gl_Position = uMVP*vec4(aVertex.x, aVertex.y, aVertex.z, 1.0);\n"

	"  if(aTexCoord.x < 0.0) {\n"
	"    vTexCoord.x = 1.0;\n"
	"    vTexCoord.y = 1.0;\n" // bottom right corner
	"  } else {\n"
    "    vTexCoord = aTexCoord;\n"
    "  }\n"

    "  vec3 transformedNormal = uN * aNormal;\n"
    "  vLightWeighting = max(dot(transformedNormal, uLightingDirection)*0.5+0.5, 0.0);\n"
    "  if(vLightWeighting<0.5) \n"
    "    vLightWeighting=0.5;\n"
    "}\n";
const std::string kTextureMeshFragmentShader =
    "precision highp float;\n"
    "precision mediump int;\n"
	"uniform sampler2D uTexture;\n"
	"uniform float uGainR;\n"
	"uniform float uGainG;\n"
	"uniform float uGainB;\n"
    "varying vec2 vTexCoord;\n"
	"varying float vLightWeighting;\n"
	""
    "void main() {\n"
    "  vec4 textureColor = texture2D(uTexture, vTexCoord);\n"
	"  gl_FragColor = vec4(textureColor.r * uGainR * vLightWeighting, textureColor.g * uGainG * vLightWeighting, textureColor.b * uGainB * vLightWeighting, textureColor.a);\n"
    "}\n";
const std::string kTextureMeshBlendingFragmentShader =
    "precision highp float;\n"
    "precision mediump int;\n"
	"uniform sampler2D uTexture;\n"
    "uniform sampler2D uDepthTexture;\n"
	"uniform float uGainR;\n"
	"uniform float uGainG;\n"
	"uniform float uGainB;\n"
	"uniform vec2 uScreenScale;\n"
	"uniform float uNearZ;\n"
	"uniform float uFarZ;\n"
    "varying vec2 vTexCoord;\n"
	"varying float vLightWeighting;\n"
	""
    "void main() {\n"
    "  vec4 textureColor = texture2D(uTexture, vTexCoord);\n"
    "  float alpha = 1.0;\n"
	"  vec2 coord = uScreenScale * gl_FragCoord.xy;\n;"
	"  vec4 depthPacked = texture2D(uDepthTexture, coord);\n"
	"  float depth = dot(depthPacked, 1./vec4(1.,255.,65025.,16581375.));\n"
	"  float num =  (2.0 * uNearZ * uFarZ);\n"
	"  float diff = (uFarZ - uNearZ);\n"
	"  float add = (uFarZ + uNearZ);\n"
	"  float ndcDepth = depth * 2.0 - 1.0;\n" // Back to NDC
	"  float linearDepth = num / (add - ndcDepth * diff);\n" // inverse projection matrix
	"  float ndcFragz = gl_FragCoord.z * 2.0 - 1.0;\n" // Back to NDC
	"  float linearFragz = num / (add - ndcFragz * diff);\n" // inverse projection matrix
	"  if(linearFragz > linearDepth + 0.05)\n"
	"    alpha=0.0;\n"
    "  gl_FragColor = vec4(textureColor.r * uGainR * vLightWeighting, textureColor.g * uGainG * vLightWeighting, textureColor.b * uGainB * vLightWeighting, alpha);\n"
    "}\n";

std::vector<GLuint> PointCloudDrawable::shaderPrograms_;

void PointCloudDrawable::createShaderPrograms()
{
	if(shaderPrograms_.empty())
	{
		shaderPrograms_.resize(9);

		shaderPrograms_[kPointCloud] = tango_gl::util::CreateProgram(kPointCloudVertexShader.c_str(), kPointCloudFragmentShader.c_str());
		UASSERT(shaderPrograms_[kPointCloud] != 0);
		shaderPrograms_[kPointCloudBlending] = tango_gl::util::CreateProgram(kPointCloudVertexShader.c_str(), kPointCloudBlendingFragmentShader.c_str());
		UASSERT(shaderPrograms_[kPointCloudBlending] != 0);
		shaderPrograms_[kPointCloudLighting] = tango_gl::util::CreateProgram(kPointCloudLightingVertexShader.c_str(), kPointCloudFragmentShader.c_str());
		UASSERT(shaderPrograms_[kPointCloudLighting] != 0);
		shaderPrograms_[kPointCloudLightingBlending] = tango_gl::util::CreateProgram(kPointCloudLightingVertexShader.c_str(), kPointCloudBlendingFragmentShader.c_str());
		UASSERT(shaderPrograms_[kPointCloudLightingBlending] != 0);

		shaderPrograms_[kTexture] = tango_gl::util::CreateProgram(kTextureMeshVertexShader.c_str(), kTextureMeshFragmentShader.c_str());
		UASSERT(shaderPrograms_[kTexture] != 0);
		shaderPrograms_[kTextureBlending] = tango_gl::util::CreateProgram(kTextureMeshVertexShader.c_str(), kTextureMeshBlendingFragmentShader.c_str());
		UASSERT(shaderPrograms_[kTextureBlending] != 0);
		shaderPrograms_[kTextureLighting] = tango_gl::util::CreateProgram(kTextureMeshLightingVertexShader.c_str(), kTextureMeshFragmentShader.c_str());
		UASSERT(shaderPrograms_[kTextureLighting] != 0);
		shaderPrograms_[kTextureLightingBlending] = tango_gl::util::CreateProgram(kTextureMeshLightingVertexShader.c_str(), kTextureMeshBlendingFragmentShader.c_str());
		UASSERT(shaderPrograms_[kTextureLightingBlending] != 0);

		shaderPrograms_[kDepthPacking] = tango_gl::util::CreateProgram(kPointCloudDepthPackingVertexShader.c_str(), kPointCloudDepthPackingFragmentShader.c_str());
		UASSERT(shaderPrograms_[kDepthPacking] != 0);
	}
}
void PointCloudDrawable::releaseShaderPrograms()
{
	for(unsigned int i=0; i<shaderPrograms_.size(); ++i)
	{
		glDeleteShader(shaderPrograms_[i]);
	}
	shaderPrograms_.clear();
}

PointCloudDrawable::PointCloudDrawable(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		const pcl::IndicesPtr & indices,
		float gainR,
		float gainG,
		float gainB) :
				vertex_buffer_(0),
				texture_(0),
				nPoints_(0),
				pose_(rtabmap::Transform::getIdentity()),
				poseGl_(1.0f),
				visible_(true),
				hasNormals_(false),
				gainR_(gainR),
				gainG_(gainG),
				gainB_(gainB)
{
    index_buffers_.resize(6, 0);
    index_buffers_count_.resize(6, 0);
	updateCloud(cloud, indices);
}

PointCloudDrawable::PointCloudDrawable(
		const rtabmap::Mesh & mesh,
		bool createWireframe) :
				vertex_buffer_(0),
				texture_(0),
				nPoints_(0),
				pose_(rtabmap::Transform::getIdentity()),
				poseGl_(1.0f),
				visible_(true),
				hasNormals_(false),
				gainR_(1.0f),
				gainG_(1.0f),
				gainB_(1.0f)
{
    index_buffers_.resize(6, 0);
    index_buffers_count_.resize(6, 0);
	updateMesh(mesh, createWireframe);
}

PointCloudDrawable::~PointCloudDrawable()
{
	LOGI("Freeing cloud buffer %d", vertex_buffer_);
	if (vertex_buffer_)
	{
		glDeleteBuffers(1, &vertex_buffer_);
		tango_gl::util::CheckGlError("PointCloudDrawable::~PointCloudDrawable()");
		vertex_buffer_ = 0;
	}

	if (texture_)
	{
		glDeleteTextures(1, &texture_);
		tango_gl::util::CheckGlError("PointCloudDrawable::~PointCloudDrawable()");
		texture_ = 0;
	}
    
    for(size_t i=0; i<index_buffers_.size(); ++i)
    {
        if(index_buffers_[i])
        {
            glDeleteBuffers(1, &index_buffers_[i]);
            index_buffers_[i] = 0;
            tango_gl::util::CheckGlError("PointCloudDrawable::~PointCloudDrawable()");
        }
    }
}

void PointCloudDrawable::updatePolygons(const std::vector<pcl::Vertices> & polygons, const std::vector<pcl::Vertices> & polygonsLowRes, bool createWireframe)
{
    for(int i=0; i<4; ++i)
    {
        if(index_buffers_[i])
        {
            glDeleteBuffers(1, &index_buffers_[i]);
            index_buffers_[i] = 0;
            tango_gl::util::CheckGlError("PointCloudDrawable::updatePolygons() clearing polygon buffers");
        }
    }
    
	//LOGD("Update polygons");
	if(polygons.size() && organizedToDenseIndices_.size())
	{
		size_t polygonSize = polygons[0].vertices.size();
		UASSERT(polygonSize == 3);
        std::vector<std::vector<GLuint> > indexes(4);
        indexes[0].resize(polygons.size() * polygonSize);
		if(createWireframe)
            indexes[2].resize(indexes[0].size()*2);
		int oi = 0;
		int li = 0;
        for(size_t i=0; i<polygons.size(); ++i)
		{
			UASSERT(polygons[i].vertices.size() == polygonSize);
			for(unsigned int j=0; j<polygonSize; ++j)
			{
                indexes[0][oi++] = organizedToDenseIndices_.at(polygons[i].vertices[j]);
				if(createWireframe)
				{
                    indexes[2][li++] = organizedToDenseIndices_.at(polygons[i].vertices[j]);
                    indexes[2][li++] = organizedToDenseIndices_.at(polygons[i].vertices[(j+1) % polygonSize]);
				}
			}
		}

		if(polygonsLowRes.size())
		{
			size_t polygonSize = polygonsLowRes[0].vertices.size();
			UASSERT(polygonSize == 3);
            indexes[1].resize(polygonsLowRes.size() * polygonSize);
			if(createWireframe)
                indexes[3].resize(indexes[1].size()*2);
			int oi = 0;
			int li = 0;
			for(unsigned int i=0; i<polygonsLowRes.size(); ++i)
			{
				UASSERT(polygonsLowRes[i].vertices.size() == polygonSize);
				for(unsigned int j=0; j<polygonSize; ++j)
				{
                    indexes[1][oi++] = organizedToDenseIndices_.at(polygonsLowRes[i].vertices[j]);
					if(createWireframe)
					{
                        indexes[3][li++] = organizedToDenseIndices_.at(polygonsLowRes[i].vertices[j]);
                        indexes[3][li++] = organizedToDenseIndices_.at(polygonsLowRes[i].vertices[(j+1)%polygonSize]);
					}
				}
			}
		}
        
        // Generate index buffers
        for(size_t i=0; i<indexes.size(); ++i)
        {
            if(!indexes[i].empty())
            {
                glGenBuffers(1, &index_buffers_[i]);
                if(!index_buffers_[i])
                {
                    LOGE("OpenGL: could not generate index buffer %ld\n", i);
                    return;
                }
                
                LOGD("Adding polygon index %ld size=%ld", i, indexes[i].size());
                
                glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, index_buffers_[i]);
                glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(uint32_t) * indexes[i].size(), indexes[i].data(), GL_STATIC_DRAW);
                glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
                index_buffers_count_[i] = (int)indexes[i].size();

                GLint error = glGetError();
                if(error != GL_NO_ERROR)
                {
                    LOGE("OpenGL: Could not allocate indexes (0x%x)\n", error);
                    index_buffers_[i] = 0;
                    return;
                }
            }
        }
	}
}

void PointCloudDrawable::updateCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud, const pcl::IndicesPtr & indices)
{
	UASSERT(cloud.get() && !cloud->empty());
	nPoints_ = 0;
	aabbMinModel_ = aabbMinWorld_ = pcl::PointXYZ(1000,1000,1000);
	aabbMaxModel_ = aabbMaxWorld_ = pcl::PointXYZ(-1000,-1000,-1000);

	if (vertex_buffer_)
	{
		glDeleteBuffers(1, &vertex_buffer_);
		tango_gl::util::CheckGlError("PointCloudDrawable::updateCloud() clear vertex buffer");
		vertex_buffer_ = 0;
	}

	if (texture_)
	{
		glDeleteTextures(1, &texture_);
		tango_gl::util::CheckGlError("PointCloudDrawable::updateCloud() clear texture buffer");
		texture_ = 0;
	}
    
    for(size_t i=0; i<index_buffers_.size(); ++i)
    {
        if(index_buffers_[i])
        {
            glDeleteBuffers(1, &index_buffers_[i]);
            index_buffers_[i] = 0;
            tango_gl::util::CheckGlError("PointCloudDrawable::updateCloud() clear index buffer");
        }
    }

	glGenBuffers(1, &vertex_buffer_);
	if(!vertex_buffer_)
	{
		LOGE("OpenGL: could not generate vertex buffers\n");
		return;
	}

	LOGI("Creating cloud buffer %d", vertex_buffer_);
	std::vector<float> vertices;
	size_t totalPoints = 0;
    std::vector<GLuint> verticesLowRes;
    std::vector<GLuint> verticesLowLowRes;
	if(indices.get() && indices->size())
	{
		totalPoints = indices->size();
		vertices.resize(indices->size()*4);
		verticesLowRes.resize(cloud->isOrganized()?totalPoints:0);
		verticesLowLowRes.resize(cloud->isOrganized()?totalPoints:0);
		int oi_low = 0;
		int oi_lowlow = 0;
		for(unsigned int i=0; i<indices->size(); ++i)
		{
			const pcl::PointXYZRGB & pt = cloud->at(indices->at(i));
			vertices[i*4] = pt.x;
			vertices[i*4+1] = pt.y;
			vertices[i*4+2] = pt.z;
			vertices[i*4+3] = pt.rgb;

			updateAABBMinMax(pt, aabbMinModel_, aabbMaxModel_);

			if(cloud->isOrganized())
			{
				if(indices->at(i)%LOW_DEC == 0 && (indices->at(i)/cloud->width) % LOW_DEC == 0)
				{
					verticesLowRes[oi_low++] = i;
				}
				if(indices->at(i)%LOWLOW_DEC == 0 && (indices->at(i)/cloud->width) % LOWLOW_DEC == 0)
				{
					verticesLowLowRes[oi_lowlow++] = i;
				}
			}
		}
		verticesLowRes.resize(oi_low);
		verticesLowLowRes.resize(oi_lowlow);
	}
	else
	{
		totalPoints = cloud->size();
		vertices.resize(cloud->size()*4);
		verticesLowRes.resize(cloud->isOrganized()?totalPoints:0);
		verticesLowLowRes.resize(cloud->isOrganized()?totalPoints:0);
		int oi_low = 0;
		int oi_lowlow = 0;
		for(unsigned int i=0; i<cloud->size(); ++i)
		{
			const pcl::PointXYZRGB & pt = cloud->at(i);
			vertices[i*4] = pt.x;
			vertices[i*4+1] = pt.y;
			vertices[i*4+2] = pt.z;
			vertices[i*4+3] = pt.rgb;

			updateAABBMinMax(pt, aabbMinModel_, aabbMaxModel_);

			if(cloud->isOrganized())
			{
				if(i%LOW_DEC == 0 && (i/cloud->width) % LOW_DEC == 0)
				{
					verticesLowRes[oi_low++] = i;
				}
				if(i%LOWLOW_DEC == 0 && (i/cloud->width) % LOWLOW_DEC == 0)
				{
					verticesLowLowRes[oi_lowlow++] = i;
				}
			}
		}
		verticesLowRes.resize(oi_low);
		verticesLowLowRes.resize(oi_lowlow);
	}

	glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer_);
	glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * (int)vertices.size(), (const void *)vertices.data(), GL_STATIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	GLint error = glGetError();
	if(error != GL_NO_ERROR)
	{
		LOGE("OpenGL: Could not allocate point cloud (0x%x)\n", error);
		vertex_buffer_ = 0;
		return;
	}
    
    // vertex index buffers
    for(size_t i=4; i<5; ++i)
    {
        if((i==4 && !verticesLowRes.empty()) ||
           (i==5 && !verticesLowLowRes.empty()))
        {
            glGenBuffers(1, &index_buffers_[i]);
            if(!index_buffers_[i])
            {
                LOGE("OpenGL: could not generate index buffer %ld\n", i);
                return;
            }
            
            index_buffers_count_[i] = i==4?(int)verticesLowRes.size():(int)verticesLowLowRes.size();
            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, index_buffers_[i]);
            glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(uint32_t) * index_buffers_count_[i], i==4?verticesLowRes.data():verticesLowLowRes.data(), GL_STATIC_DRAW);
            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
            

            GLint error = glGetError();
            if(error != GL_NO_ERROR)
            {
                LOGE("OpenGL: Could not allocate indexes (0x%x)\n", error);
                index_buffers_[i] = 0;
                return;
            }
        }
    }

	nPoints_ = (int)totalPoints;
}

void PointCloudDrawable::updateMesh(const rtabmap::Mesh & mesh, bool createWireframe)
{
	UASSERT(mesh.cloud.get() && !mesh.cloud->empty());
	nPoints_ = 0;
	aabbMinModel_ = aabbMinWorld_ = pcl::PointXYZ(1000,1000,1000);
	aabbMaxModel_ = aabbMaxWorld_ = pcl::PointXYZ(-1000,-1000,-1000);

	if (vertex_buffer_)
	{
		glDeleteBuffers(1, &vertex_buffer_);
		tango_gl::util::CheckGlError("PointCloudDrawable::updateMesh() clear vertex buffer");
		vertex_buffer_ = 0;
	}
    
    for(size_t i=0; i<index_buffers_.size(); ++i)
    {
        if(index_buffers_[i])
        {
            glDeleteBuffers(1, &index_buffers_[i]);
            index_buffers_[i] = 0;
            tango_gl::util::CheckGlError("PointCloudDrawable::updateMesh() clear index buffer");
        }
    }

	gainR_ = mesh.gains[0];
	gainG_ = mesh.gains[1];
	gainB_ = mesh.gains[2];

	bool textureUpdate = false;
	if(!mesh.texture.empty() && mesh.texture.type() == CV_8UC3)
	{
		if (texture_)
		{
			glDeleteTextures(1, &texture_);
			tango_gl::util::CheckGlError("PointCloudDrawable::updateMesh() clear texture buffer");
			texture_ = 0;
		}
		textureUpdate = true;
	}

	glGenBuffers(1, &vertex_buffer_);
	if(!vertex_buffer_)
	{
		LOGE("OpenGL: could not generate vertex buffers\n");
		return;
	}

	if(textureUpdate)
	{
		glGenTextures(1, &texture_);
		if(!texture_)
		{
			vertex_buffer_ = 0;
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
		totalPoints = (int)mesh.indices->size();
        std::vector<GLuint> verticesLowRes;
        std::vector<GLuint> verticesLowLowRes;
        verticesLowRes.resize(totalPoints);
        verticesLowLowRes.resize(totalPoints);
		int oi_low = 0;
		int oi_lowlow = 0;
		if(texture_ && polygons.size())
		{
			int items = hasNormals_?9:6;
			vertices = std::vector<float>(mesh.indices->size()*items);
			for(unsigned int i=0; i<mesh.indices->size(); ++i)
			{
				const pcl::PointXYZRGB & pt = mesh.cloud->at(mesh.indices->at(i));
				vertices[i*items] = pt.x;
				vertices[i*items+1] = pt.y;
				vertices[i*items+2] = pt.z;

				// rgb
				vertices[i*items+3] = pt.rgb;

				updateAABBMinMax(pt, aabbMinModel_, aabbMaxModel_);

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
                    verticesLowRes[oi_low++] = i;
				}
				if(mesh.indices->at(i)%LOWLOW_DEC == 0 && (mesh.indices->at(i)/mesh.cloud->width) % LOWLOW_DEC == 0)
				{
                    verticesLowLowRes[oi_lowlow++] = i;
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
				const pcl::PointXYZRGB & pt = mesh.cloud->at(mesh.indices->at(i));
				vertices[i*items] = pt.x;
				vertices[i*items+1] = pt.y;
				vertices[i*items+2] = pt.z;
				vertices[i*items+3] = pt.rgb;

				updateAABBMinMax(pt, aabbMinModel_, aabbMaxModel_);

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
                    verticesLowRes[oi_low++] = i;
				}
				if(mesh.indices->at(i)%LOWLOW_DEC == 0 && (mesh.indices->at(i)/mesh.cloud->width) % LOWLOW_DEC == 0)
				{
                    verticesLowLowRes[oi_lowlow++] = i;
				}
			}
		}
        verticesLowRes.resize(oi_low);
        verticesLowLowRes.resize(oi_lowlow);
        
        // vertex index buffers
        for(size_t i=4; i<5; ++i)
        {
            if((i==4 && !verticesLowRes.empty()) ||
               (i==5 && !verticesLowLowRes.empty()))
            {
                glGenBuffers(1, &index_buffers_[i]);
                if(!index_buffers_[i])
                {
                    LOGE("OpenGL: could not generate index buffer %ld\n", i);
                    return;
                }
                
                index_buffers_count_[i] = i==4?(int)verticesLowRes.size():(int)verticesLowLowRes.size();
                glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, index_buffers_[i]);
                glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(uint32_t) * index_buffers_count_[i], i==4?verticesLowRes.data():verticesLowLowRes.data(), GL_STATIC_DRAW);
                glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

                GLint error = glGetError();
                if(error != GL_NO_ERROR)
                {
                    LOGE("OpenGL: Could not allocate indexes (0x%x)\n", error);
                    index_buffers_[i] = 0;
                    return;
                }
            }
        }
	}
	else // assume dense mesh with texCoords set to polygons
	{
		if(texture_ && polygons.size())
		{
			//LOGD("Dense mesh with texture (%d texCoords %d points %d polygons %dx%d)",
			//		(int)mesh.texCoords.size(), (int)mesh.cloud->size(), (int)mesh.polygons.size(), texture.cols, texture.rows);

			// Texturing issue:
			//  tex_coordinates should be linked to points, not
			//  polygon vertices. Points linked to multiple different texCoords (different textures) should
			//  be duplicated.
			totalPoints = (int)mesh.texCoords.size();
            int items = hasNormals_?9:6;
			vertices = std::vector<float>(mesh.texCoords.size()*items);
			organizedToDenseIndices_ = std::vector<unsigned int>(totalPoints, -1);

			UASSERT_MSG(mesh.texCoords.size() == polygons[0].vertices.size()*polygons.size(),
					uFormat("%d vs %d x %d", (int)mesh.texCoords.size(), (int)polygons[0].vertices.size(), (int)polygons.size()).c_str());

			unsigned int oi=0;
			for(unsigned int i=0; i<polygons.size(); ++i)
			{
				pcl::Vertices & v = polygons[i];
				for(unsigned int j=0; j<v.vertices.size(); ++j)
				{
					UASSERT(oi < mesh.texCoords.size());
					UASSERT(v.vertices[j] < mesh.cloud->size());

					const pcl::PointXYZRGB & pt =  mesh.cloud->at(v.vertices[j]);

					vertices[oi*items] = pt.x;
					vertices[oi*items+1] = pt.y;
					vertices[oi*items+2] = pt.z;

					// rgb
					vertices[oi*items+3] = pt.rgb;

					updateAABBMinMax(pt, aabbMinModel_, aabbMaxModel_);

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
			totalPoints = (int)mesh.cloud->size();
			//LOGD("Dense mesh");
			int items = hasNormals_?7:4;
			organizedToDenseIndices_ = std::vector<unsigned int>(totalPoints, -1);
			vertices = std::vector<float>(mesh.cloud->size()*items);
			for(unsigned int i=0; i<mesh.cloud->size(); ++i)
			{
				const pcl::PointXYZRGB & pt =  mesh.cloud->at(i);

				vertices[i*items] =pt.x;
				vertices[i*items+1] = pt.y;
				vertices[i*items+2] = pt.z;
				vertices[i*items+3] = pt.rgb;

				updateAABBMinMax(pt, aabbMinModel_, aabbMaxModel_);

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

	glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer_);
	glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * (int)vertices.size(), (const void *)vertices.data(), GL_STATIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	GLint error = glGetError();
	if(error != GL_NO_ERROR)
	{
		LOGE("OpenGL: Could not allocate point cloud (0x%x)\n", error);
		vertex_buffer_ = 0;
		return;
	}

	if(texture_ && textureUpdate)
	{
		//GLint maxTextureSize = 0;
		//glGetIntegerv(GL_MAX_TEXTURE_SIZE, &maxTextureSize);
		//LOGI("maxTextureSize=%d", maxTextureSize);
		//GLint maxTextureUnits = 0;
		//glGetIntegerv(GL_MAX_TEXTURE_IMAGE_UNITS, &maxTextureUnits);
		//LOGW("maxTextureUnits=%d", maxTextureUnits);

		// gen texture from image
		glBindTexture(GL_TEXTURE_2D, texture_);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
		cv::Mat rgbImage;
		cv::cvtColor(mesh.texture, rgbImage, cv::COLOR_BGR2RGBA);

		glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
		//glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
		//glPixelStorei(GL_UNPACK_SKIP_PIXELS, 0);
		//glPixelStorei(GL_UNPACK_SKIP_ROWS, 0);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, rgbImage.cols, rgbImage.rows, 0, GL_RGBA, GL_UNSIGNED_BYTE, rgbImage.data);

		GLint error = glGetError();
		if(error != GL_NO_ERROR)
		{
			LOGE("OpenGL: Could not allocate texture (0x%x)\n", error);
			texture_ = 0;

			glDeleteBuffers(1, &vertex_buffer_);
			vertex_buffer_ = 0;
			return;
		}
	}

	nPoints_ = totalPoints;

    updatePolygons(polygons, polygonsLowRes, createWireframe);

	if(!pose_.isNull())
	{
		updateAABBWorld(pose_);
	}
}

void PointCloudDrawable::setPose(const rtabmap::Transform & pose)
{
	UASSERT(!pose.isNull());

	if(pose_ != pose)
	{
		updateAABBWorld(pose);
	}

	pose_ = pose;
	poseGl_ = glmFromTransform(pose);
}

void PointCloudDrawable::updateAABBWorld(const rtabmap::Transform & pose)
{
	pcl::PointCloud<pcl::PointXYZ> corners;
	corners.resize(8);
	corners.at(0) = pcl::PointXYZ(aabbMinModel_.x, aabbMinModel_.y, aabbMinModel_.z);
	corners.at(1) = pcl::PointXYZ(aabbMinModel_.x, aabbMinModel_.y, aabbMaxModel_.z);
	corners.at(2) = pcl::PointXYZ(aabbMinModel_.x, aabbMaxModel_.y, aabbMinModel_.z);
	corners.at(3) = pcl::PointXYZ(aabbMaxModel_.x, aabbMinModel_.y, aabbMinModel_.z);
	corners.at(4) = pcl::PointXYZ(aabbMaxModel_.x, aabbMaxModel_.y, aabbMaxModel_.z);
	corners.at(5) = pcl::PointXYZ(aabbMaxModel_.x, aabbMaxModel_.y, aabbMinModel_.z);
	corners.at(6) = pcl::PointXYZ(aabbMaxModel_.x, aabbMinModel_.y, aabbMaxModel_.z);
	corners.at(7) = pcl::PointXYZ(aabbMinModel_.x, aabbMaxModel_.y, aabbMaxModel_.z);

	pcl::PointCloud<pcl::PointXYZ> cornersTransformed;
	pcl::transformPointCloud(corners, cornersTransformed, pose.toEigen3f());

	aabbMinWorld_ = pcl::PointXYZ(1000,1000,1000);
	aabbMaxWorld_ = pcl::PointXYZ(-1000,-1000,-1000);
	for(unsigned int i=0; i<cornersTransformed.size(); ++i)
	{
		updateAABBMinMax(cornersTransformed.at(i), aabbMinWorld_, aabbMaxWorld_);
	}
}


void PointCloudDrawable::Render(
		const glm::mat4 & projectionMatrix,
		const glm::mat4 & viewMatrix,
		bool meshRendering,
		float pointSize,
		bool textureRendering,
		bool lighting,
		float distanceToCameraSqr,
		const GLuint & depthTexture,
		int screenWidth,
		int screenHeight,
		float nearClipPlane,
	    float farClipPlane,
		bool packDepthToColorChannel,
		bool wireFrame) const
{
	if(vertex_buffer_ && nPoints_ && visible_ && !shaderPrograms_.empty())
	{
		if(packDepthToColorChannel || !hasNormals_)
		{
			lighting = false;
		}

		if(packDepthToColorChannel || !(meshRendering && textureRendering && texture_))
		{
			textureRendering = false;
		}

		GLuint program;
		if(packDepthToColorChannel)
		{
			program = shaderPrograms_[kDepthPacking];
		}
		else if(textureRendering)
		{
			if(lighting)
			{
				program = shaderPrograms_[depthTexture>0?kTextureLightingBlending:kTextureLighting];
			}
			else
			{
				program = shaderPrograms_[depthTexture>0?kTextureBlending:kTexture];
			}
		}
		else
		{
			if(lighting)
			{
				program = shaderPrograms_[depthTexture>0?kPointCloudLightingBlending:kPointCloudLighting];
			}
			else
			{
				program = shaderPrograms_[depthTexture>0?kPointCloudBlending:kPointCloud];
			}
		}

		glUseProgram(program);
		tango_gl::util::CheckGlError("Pointcloud::Render() set program");

		GLuint mvp_handle = glGetUniformLocation(program, "uMVP");
		glm::mat4 mv_mat = viewMatrix * poseGl_;
		glm::mat4 mvp_mat = projectionMatrix * mv_mat;
		glUniformMatrix4fv(mvp_handle, 1, GL_FALSE, glm::value_ptr(mvp_mat));

		GLint attribute_vertex = glGetAttribLocation(program, "aVertex");
		glEnableVertexAttribArray(attribute_vertex);
		GLint attribute_color = 0;
		GLint attribute_texture = 0;
		GLint attribute_normal = 0;

		if(packDepthToColorChannel || !textureRendering)
		{
			GLuint point_size_handle_ = glGetUniformLocation(program, "uPointSize");
			glUniform1f(point_size_handle_, pointSize);
		}
		tango_gl::util::CheckGlError("Pointcloud::Render() vertex");

		if(!packDepthToColorChannel)
		{
			GLuint gainR_handle = glGetUniformLocation(program, "uGainR");
			GLuint gainG_handle = glGetUniformLocation(program, "uGainG");
			GLuint gainB_handle = glGetUniformLocation(program, "uGainB");
			glUniform1f(gainR_handle, gainR_);
			glUniform1f(gainG_handle, gainG_);
			glUniform1f(gainB_handle, gainB_);

			// blending
			if(depthTexture > 0)
			{
				// Texture activate unit 1
				glActiveTexture(GL_TEXTURE1);
				// Bind the texture to this unit.
				glBindTexture(GL_TEXTURE_2D, depthTexture);
				// Tell the texture uniform sampler to use this texture in the shader by binding to texture unit 1.
				GLuint depth_texture_handle = glGetUniformLocation(program, "uDepthTexture");
				glUniform1i(depth_texture_handle, 1);

				GLuint zNear_handle = glGetUniformLocation(program, "uNearZ");
				GLuint zFar_handle = glGetUniformLocation(program, "uFarZ");
				glUniform1f(zNear_handle, nearClipPlane);
				glUniform1f(zFar_handle, farClipPlane);

				GLuint screenScale_handle = glGetUniformLocation(program, "uScreenScale");
				glUniform2f(screenScale_handle, 1.0f/(float)screenWidth, 1.0f/(float)screenHeight);
			}

			if(lighting)
			{
				GLuint n_handle = glGetUniformLocation(program, "uN");
				glm::mat3 normalMatrix(mv_mat);
				normalMatrix = glm::inverse(normalMatrix);
				normalMatrix = glm::transpose(normalMatrix);
				glUniformMatrix3fv(n_handle, 1, GL_FALSE, glm::value_ptr(normalMatrix));

				GLuint lightingDirection_handle = glGetUniformLocation(program, "uLightingDirection");
				glUniform3f(lightingDirection_handle, 0.0, 0.0, 1.0); // from the camera

				attribute_normal = glGetAttribLocation(program, "aNormal");
				glEnableVertexAttribArray(attribute_normal);
			}

			if(textureRendering)
			{
				// Texture activate unit 0
				glActiveTexture(GL_TEXTURE0);
				// Bind the texture to this unit.
				glBindTexture(GL_TEXTURE_2D, texture_);
				// Tell the texture uniform sampler to use this texture in the shader by binding to texture unit 0.
				GLuint texture_handle = glGetUniformLocation(program, "uTexture");
				glUniform1i(texture_handle, 0);

				attribute_texture = glGetAttribLocation(program, "aTexCoord");
				glEnableVertexAttribArray(attribute_texture);
			}
			else
			{
				attribute_color = glGetAttribLocation(program, "aColor");
				glEnableVertexAttribArray(attribute_color);
			}
		}
		tango_gl::util::CheckGlError("Pointcloud::Render() common");

		glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer_);
		if(texture_)
		{
			glVertexAttribPointer(attribute_vertex, 3, GL_FLOAT, GL_FALSE, (hasNormals_?9:6)*sizeof(GLfloat), 0);
			if(textureRendering)
			{
				glVertexAttribPointer(attribute_texture, 2, GL_FLOAT, GL_FALSE, (hasNormals_?9:6)*sizeof(GLfloat), (GLvoid*) (4 * sizeof(GLfloat)));
			}
			else if(!packDepthToColorChannel)
			{
				glVertexAttribPointer(attribute_color, 3, GL_UNSIGNED_BYTE, GL_TRUE,  (hasNormals_?9:6)*sizeof(GLfloat), (GLvoid*) (3 * sizeof(GLfloat)));
			}
			if(lighting && hasNormals_)
			{
				glVertexAttribPointer(attribute_normal, 3, GL_FLOAT, GL_FALSE, 9*sizeof(GLfloat), (GLvoid*) (6 * sizeof(GLfloat)));
			}
		}
		else
		{
			glVertexAttribPointer(attribute_vertex, 3, GL_FLOAT, GL_FALSE, (hasNormals_?7:4)*sizeof(GLfloat), 0);
			if(!packDepthToColorChannel)
			{
				glVertexAttribPointer(attribute_color, 3, GL_UNSIGNED_BYTE, GL_TRUE, (hasNormals_?7:4)*sizeof(GLfloat), (GLvoid*) (3 * sizeof(GLfloat)));
			}
			if(lighting && hasNormals_)
			{
				glVertexAttribPointer(attribute_normal, 3, GL_FLOAT, GL_FALSE, 7*sizeof(GLfloat), (GLvoid*) (4 * sizeof(GLfloat)));
			}
		}
		tango_gl::util::CheckGlError("Pointcloud::Render() set attribute pointer");

		UTimer drawTime;
		if((textureRendering || meshRendering) && index_buffers_[0])
		{
            float dist = meshRendering?50.0f:16.0f;
			if(distanceToCameraSqr<dist || index_buffers_[1]==0)
			{
				wireFrame = wireFrame && index_buffers_[2];
				if(wireFrame)
                {
                    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, index_buffers_[2]);
                    glDrawElements(GL_LINES, index_buffers_count_[2], GL_UNSIGNED_INT, 0);
                }
				else
                {
                    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, index_buffers_[0]);
                    glDrawElements(GL_TRIANGLES, index_buffers_count_[0], GL_UNSIGNED_INT, 0);
                }
			}
			else
			{
				wireFrame = wireFrame && index_buffers_[3];
				if(wireFrame)
                {
                    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, index_buffers_[3]);
                    glDrawElements(GL_LINES, index_buffers_count_[3], GL_UNSIGNED_INT, 0);
                }
				else
                {
                    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, index_buffers_[1]);
                    glDrawElements(GL_TRIANGLES, index_buffers_count_[1], GL_UNSIGNED_INT, 0);
                }
			}
		}
		else if(index_buffers_[4])
		{
			if(distanceToCameraSqr>600.0f && index_buffers_[5])
			{
                glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, index_buffers_[5]);
                glDrawElements(GL_POINTS, index_buffers_count_[5], GL_UNSIGNED_INT, 0);
			}
			else if(distanceToCameraSqr>150.0f)
			{
                glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, index_buffers_[4]);
                glDrawElements(GL_POINTS, index_buffers_count_[4], GL_UNSIGNED_INT, 0);
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
		//UERROR("drawTime=%fs", drawTime.ticks());
		tango_gl::util::CheckGlError("Pointcloud::Render() draw");

		glDisableVertexAttribArray(0);
		glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

		glUseProgram(0);
		tango_gl::util::CheckGlError("Pointcloud::Render() cleaning");
	}
}

