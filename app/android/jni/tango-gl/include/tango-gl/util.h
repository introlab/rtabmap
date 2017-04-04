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

#ifndef TANGO_GL_GL_UTIL_H_
#define TANGO_GL_GL_UTIL_H_
#define GLM_FORCE_RADIANS

#define GL_VERTEX_PROGRAM_POINT_SIZE 0x8642

#include <stdlib.h>
#include <jni.h>
#include <android/log.h>
#include <GLES2/gl2.h>
#include <GLES2/gl2ext.h>
#include <tango_support_api.h>

#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtc/quaternion.hpp"
#include "glm/gtc/type_ptr.hpp"
#include "glm/gtx/matrix_decompose.hpp"

#define LOG_TAG "rtabmap"
#ifdef DISABLE_LOG
#define LOGD(...) ;
#define LOGI(...) ;
#define LOGW(...) ;
#else
#define LOGD(...) __android_log_print(ANDROID_LOG_DEBUG,LOG_TAG,__VA_ARGS__)
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO,LOG_TAG,__VA_ARGS__)
#define LOGW(...) __android_log_print(ANDROID_LOG_WARN,LOG_TAG,__VA_ARGS__)
#endif
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR,LOG_TAG,__VA_ARGS__)

#ifndef M_PI
#define M_PI 3.1415926f
#endif

#define RADIAN_2_DEGREE 57.2957795f
#define DEGREE_2_RADIANS 0.0174532925f

namespace tango_gl {
namespace util {
  void CheckGlError(const char* operation);

  GLuint CreateProgram(const char* vertex_source,
                       const char* fragment_source);

  void DecomposeMatrix(const glm::mat4& transform_mat,
                       glm::vec3& translation,
                       glm::quat& rotation,
                       glm::vec3& scale);

  // Get a 3x1 column from the upper 3x4 of a transformation matrix. Columns
  // 0, 1, 2 are the rotation/scale portion, and column 3 is the translation.
  glm::vec3 GetColumnFromMatrix(const glm::mat4& mat, const int col);

  // Get the translation component of a transformation matrix.
  glm::vec3 GetTranslationFromMatrix(const glm::mat4& mat);

  float Clamp(float value, float min, float max);
  void PrintMatrix(const glm::mat4& matrix);
  void PrintVector(const glm::vec3& vector);
  void PrintQuaternion(const glm::quat& quat);

  glm::vec3 LerpVector(const glm::vec3& x, const glm::vec3& y, float a);
  float DistanceSquared(const glm::vec3& v1, const glm::vec3& v2);

  bool SegmentAABBIntersect(const glm::vec3& aabb_min, const glm::vec3& aabb_max,
                        const glm::vec3& start, const glm::vec3& end);

  glm::vec3 ApplyTransform(const glm::mat4& mat, const glm::vec3& vec);

  // Get the Android rotation integer value from color camera to display.
  // This function is used to compute the orientation difference to handle
  // the portrait and landscape mode for color camera display.
  //
  // @param display: integer value of display orientation, values available
  // are 0, 1, 2 ,3. Followed by Android display orientation standard:
  // https://developer.android.com/reference/android/view/Display.html#getRotation()
  // @param color_camera: integer value of color camera oreintation, values
  // available are 0, 90, 180, 270. Followed by Android camera orientation
  // standard:
  // https://developer.android.com/reference/android/hardware/Camera.CameraInfo.html#orientation
  TangoSupportRotation GetAndroidRotationFromColorCameraToDisplay(
      int display_rotation, int color_camera_rotation);

  // Get the Android rotation integer value from color camera to display.
  // This function is used to compute the orientation difference to handle
  // the portrait and landscape mode for color camera display.
  //
  // @param display: the device display orientation.
  // @param color_camera: integer value of color camera oreintation, values
  // available are 0, 90, 180, 270. Followed by Android camera orientation
  // standard:
  // https://developer.android.com/reference/android/hardware/Camera.CameraInfo.html#orientation
  TangoSupportRotation GetAndroidRotationFromColorCameraToDisplay(
      TangoSupportRotation display_rotation, int color_camera_rotation);

}  // namespace util
}  // namespace tango_gl
#endif  // TANGO_GL_RENDERER_GL_UTIL
