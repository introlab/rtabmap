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
#include "tango-gl/texture.h"
#include "tango-gl/util.h"

namespace tango_gl {

static const int kMaxExponentiation = 12;

static int RoundUpPowerOfTwo(int w) {
  int start = 2;
  for (int i = 0; i <= kMaxExponentiation; ++i) {
    if (w < start) {
      w = start;
      break;
    } else {
      start = start << 1;
    }
  }
  return w;
}

Texture::Texture(const char* file_path) {
  if (!LoadFromPNG(file_path)) {
    LOGE("Texture initialing error");
  }
}

bool Texture::LoadFromPNG(const char* file_path) {
  FILE* file = fopen(file_path, "rb");

  if (file == NULL) {
    LOGE("fp not loaded: %s", strerror(errno));
    return false;
  }

  fseek(file, 8, SEEK_CUR);

  png_structp png_ptr =
      png_create_read_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
  png_infop info_ptr = png_create_info_struct(png_ptr);

  png_init_io(png_ptr, file);
  png_set_sig_bytes(png_ptr, 8);
  png_read_info(png_ptr, info_ptr);
  png_get_IHDR(png_ptr, info_ptr, &width_, &height_, &bit_depth_, &color_type_,
               NULL, NULL, NULL);

  width_ = RoundUpPowerOfTwo(width_);
  height_ = RoundUpPowerOfTwo(height_);
  int row = width_ * (color_type_ == PNG_COLOR_TYPE_RGBA ? 4 : 3);
  byte_data_ = new char[row * height_];

  png_bytep* row_pointers = new png_bytep[height_];
  for (uint i = 0; i < height_; ++i) {
    row_pointers[i] = (png_bytep)(byte_data_ + i * row);
  }
  png_read_image(png_ptr, row_pointers);
  png_destroy_read_struct(&png_ptr, &info_ptr, 0);

  glGenTextures(1, &texture_id_);
  glBindTexture(GL_TEXTURE_2D, texture_id_);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  util::CheckGlError("glBindTexture");
  if (color_type_ == PNG_COLOR_TYPE_RGBA) {
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width_, height_, 0, GL_RGBA,
                 GL_UNSIGNED_BYTE, byte_data_);
  } else {
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width_, height_, 0, GL_RGB,
                 GL_UNSIGNED_BYTE, byte_data_);
  }
  util::CheckGlError("glTexImage2D");
  glBindTexture(GL_TEXTURE_2D, 0);

  fclose(file);
  delete[] row_pointers;
  delete[] byte_data_;

  return true;
}

GLuint Texture::GetTextureID() const { return texture_id_; }

Texture::~Texture() {
  if (byte_data_ != NULL) {
    delete[] byte_data_;
  }
  byte_data_ = NULL;
}

}  // namespace tango_gl
