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

#ifndef TANGO_GL_RENDERER_VIDEO_OVERLAY_H_
#define TANGO_GL_RENDERER_VIDEO_OVERLAY_H_

#include "tango-gl/drawable_object.h"

namespace tango_gl {
class VideoOverlay : public DrawableObject {
 public:
  VideoOverlay();
  void Render(const glm::mat4& projection_mat, const glm::mat4& view_mat) const;
  GLuint GetTextureId() const { return texture_id_; }
  void SetTextureId(GLuint texture_id) { texture_id_ = texture_id; }

 private:
  // This id is populated on construction, and is passed to the tango service.
  GLuint texture_id_;

  GLuint attrib_texture_coords_;
  GLuint uniform_texture_;
  GLuint vertex_buffers_[3];
};
}  // namespace tango_gl
#endif  // TANGO_GL_RENDERER_VIDEO_OVERLAY_H_
