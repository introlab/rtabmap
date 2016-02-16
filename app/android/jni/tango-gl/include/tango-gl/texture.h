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

#ifndef TANGO_GL_TEXTURE_H_
#define TANGO_GL_TEXTURE_H_

#include <errno.h>
#include <png.h>

#include "tango-gl/util.h"

namespace tango_gl {
class Texture {
 public:
  Texture(const char* file_path);
  Texture(const Texture& other) = delete;
  Texture& operator=(const Texture&) = delete;
  ~Texture();

  bool LoadFromPNG(const char* file_path);
  GLuint GetTextureID() const;

 private:
  png_uint_32 width_, height_;
  int bit_depth_, color_type_;
  char* byte_data_;
  GLuint texture_id_;
};
}  // namespace tango_gl
#endif  // TANGO_GL_TEXTURE_H_
