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

#ifndef GRAPH_DRAWABLE_H_
#define GRAPH_DRAWABLE_H_

#include <jni.h>

#include <tango-gl/util.h>
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rtabmap/core/Transform.h>
#include <rtabmap/core/Link.h>
#include <pcl/Vertices.h>
#include "util.h"

class GraphDrawable {
 public:
	GraphDrawable(
		  GLuint shaderProgram,
		  const std::map<int, rtabmap::Transform> & poses,
		  const std::multimap<int, rtabmap::Link> & links);
  virtual ~GraphDrawable();

  void setPose(const rtabmap::Transform & mapToOdom);
  void setVisible(bool visible) {visible_=visible;}

  void Render(const glm::mat4 & projectionMatrix, const glm::mat4 & viewMatrix);

 private:
  // Vertex buffer of the point cloud geometry.
  GLuint vertex_buffers_;
  std::vector<GLushort> neighborIndices_;
  std::vector<GLushort> loopClosureIndices_;
  glm::mat4 pose_;
  bool visible_;
  float lineWidth_;

  GLuint shader_program_;
};

#endif  // GRAPH_DRAWABLE_H_
