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

#ifndef TANGO_POINT_CLOUD_POINT_CLOUD_DRAWABLE_H_
#define TANGO_POINT_CLOUD_POINT_CLOUD_DRAWABLE_H_

#ifdef __ANDROID__
#include <jni.h>
#endif

#include <tango-gl/util.h>
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rtabmap/core/Transform.h>
#include <pcl/Vertices.h>
#include "util.h"

// PointCloudDrawable is responsible for the point cloud rendering.
class PointCloudDrawable {
public:
	static void createShaderPrograms();
	static void releaseShaderPrograms();

private:
	static std::vector<GLuint> shaderPrograms_;

 public:
  PointCloudDrawable(
  		  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		  const pcl::IndicesPtr & indices,
		  float gainR = 1.0f,
		  float gainG = 1.0f,
		  float gainB = 1.0f);
  PointCloudDrawable(
    	  const rtabmap::Mesh & mesh,
		  bool createWireframe = false);
  virtual ~PointCloudDrawable();

  void updatePolygons(const std::vector<pcl::Vertices> & polygons, const std::vector<pcl::Vertices> & polygonsLowRes = std::vector<pcl::Vertices>(), bool createWireframe = false);
  void updateCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud, const pcl::IndicesPtr & indices);
  void updateMesh(const rtabmap::Mesh & mesh, bool createWireframe = false);
  void setPose(const rtabmap::Transform & pose);
  void setVisible(bool visible) {visible_=visible;}
  void setGains(float gainR, float gainG, float gainB) {gainR_ = gainR; gainG_ = gainG; gainB_ = gainB;}
  rtabmap::Transform getPose() const {return pose_;}
  const glm::mat4 & getPoseGl() const {return poseGl_;}
  bool isVisible() const {return visible_;}
  bool hasMesh() const {return index_buffers_[0] != 0;}
  bool hasTexture() const {return texture_ != 0;}
  float getMinHeight() const {return minHeight_;}
  const pcl::PointXYZ & aabbMinModel() const {return aabbMinModel_;}
  const pcl::PointXYZ & aabbMaxModel() const {return aabbMaxModel_;}
  const pcl::PointXYZ & aabbMinWorld() const {return aabbMinWorld_;}
  const pcl::PointXYZ & aabbMaxWorld() const {return aabbMaxWorld_;}

  // Update current point cloud data.
  //
  // @param projection_mat: projection matrix from current render camera.
  // @param view_mat: view matrix from current render camera.
  // @param model_mat: model matrix for this point cloud frame.
  // @param vertices: all vertices in this point cloud frame.
  void Render(
		  const glm::mat4 & projectionMatrix,
		  const glm::mat4 & viewMatrix,
		  bool meshRendering = true,
		  float pointSize = 3.0f,
		  bool textureRendering = false,
		  bool lighting = true,
		  float distanceToCamSqr = 0.0f,
		  const GLuint & depthTexture = 0,
		  int screenWidth = 0,     // nonnull if depthTexture>0
		  int screenHeight = 0,    // nonnull if depthTexture>0
		  float nearClipPlane = 0, // nonnull if depthTexture>0
		  float farClipPlane = 0,  // nonnull if depthTexture>0
		  bool packDepthToColorChannel = false,
		  bool wireFrame = false) const;

 private:
  template<class PointT>
  void updateAABBMinMax(const PointT & pt, pcl::PointXYZ & min, pcl::PointXYZ & max)
  {
	  if(pt.x<min.x) min.x = pt.x;
	  if(pt.y<min.y) min.y = pt.y;
	  if(pt.z<min.z) min.z = pt.z;
	  if(pt.x>max.x) max.x = pt.x;
	  if(pt.y>max.y) max.y = pt.y;
	  if(pt.z>max.z) max.z = pt.z;
  }
  void updateAABBWorld(const rtabmap::Transform & pose);

 private:
  // Vertex buffer of the point cloud geometry.
  GLuint vertex_buffer_;
  GLuint texture_;
  std::vector<GLuint> index_buffers_;
  std::vector<int> index_buffers_count_;
  int nPoints_;
  rtabmap::Transform pose_;
  glm::mat4 poseGl_;
  bool visible_;
  bool hasNormals_;
  std::vector<unsigned int> organizedToDenseIndices_;
  float minHeight_; // odom frame

  float gainR_;
  float gainG_;
  float gainB_;

  pcl::PointXYZ aabbMinModel_;
  pcl::PointXYZ aabbMaxModel_;
  pcl::PointXYZ aabbMinWorld_;
  pcl::PointXYZ aabbMaxWorld_;
};

#endif  // TANGO_POINT_CLOUD_POINT_CLOUD_DRAWABLE_H_
