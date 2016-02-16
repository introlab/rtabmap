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

#ifndef TANGO_POINT_CLOUD_POINT_CLOUD_DRAWABLE_H_
#define TANGO_POINT_CLOUD_POINT_CLOUD_DRAWABLE_H_

#include <jni.h>

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
  PointCloudDrawable(
		  GLuint shaderProgram,
		  const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud,
		  const std::vector<pcl::Vertices> & indices = std::vector<pcl::Vertices>());
  PointCloudDrawable(
  		  GLuint shaderProgram,
  		  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
  		  const std::vector<pcl::Vertices> & indices = std::vector<pcl::Vertices>());
  virtual ~PointCloudDrawable();

  void setPose(const rtabmap::Transform & pose);
  void setVisible(bool visible) {visible_=visible;}
  rtabmap::Transform getPose() const {return glmToTransform(pose_);}
  bool isVisible() const {return visible_;}

  // Update current point cloud data.
  //
  // @param projection_mat: projection matrix from current render camera.
  // @param view_mat: view matrix from current render camera.
  // @param model_mat: model matrix for this point cloud frame.
  // @param vertices: all vertices in this point cloud frame.
  void Render(const glm::mat4 & projectionMatrix, const glm::mat4 & viewMatrix, bool meshRendering = true, float pointSize = 3.0f);

 private:
  // Vertex buffer of the point cloud geometry.
  GLuint vertex_buffers_;
  std::vector<GLushort> indices_;
  int nPoints_;
  glm::mat4 pose_;
  bool visible_;

  GLuint shader_program_;
};

#endif  // TANGO_POINT_CLOUD_POINT_CLOUD_DRAWABLE_H_
