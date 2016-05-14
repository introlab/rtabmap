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

#ifndef TANGO_GL_GL_TANGO_CONVERSIONS_H_
#define TANGO_GL_GL_TANGO_CONVERSIONS_H_

#define GLM_FORCE_RADIANS

#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtc/quaternion.hpp"

namespace tango_gl {
namespace conversions {

/**
 * @brief Creates a glm::vec3 from double[3] = {x, y, z}. This is designed to
 * work with the TangoPoseData.translation field.
 */
inline glm::vec3 Vec3FromArray(const double* array) {
  return glm::vec3(array[0], array[1], array[2]);
}

/**
 * @brief Creates a glm::quat from double[4] = {x, y, z, w}. This is designed to
 * work with the TangoPoseData.orientation field.
 */
inline glm::quat QuatFromArray(const double* array) {
  // Note GLM expects arguments in order {w, x, y, z}.
  return glm::quat(array[3], array[0], array[1], array[2]);
}

/**
 * @brief Creates a glm::mat4 rigid-frame transformation matrix from two arrays.
 * This is designed for the TangoPoseData translation and orientation fields.
 * @param A_p_B Position [x, y, z] of B_origin from A_origin, expressed in A.
 * @param A_q_B The quaternion representation [x, y, z, w] of the rotation
 * matrix A_R_B.
 * @return The transformation matrix A_T_B.
 */
inline glm::mat4 TransformFromArrays(const double* A_p_B, const double* A_q_B) {
  glm::vec3 glm_A_p_B = Vec3FromArray(A_p_B);
  glm::quat glm_A_q_B = QuatFromArray(A_q_B);
  return glm::translate(glm::mat4(1.0f), glm_A_p_B) * glm::mat4_cast(glm_A_q_B);
}

/**
 * @brief Creates a glm::mat4 rigid-frame transformation matrix from glm::vec3 and glm::quat.
 * This is designed for the TangoPoseData translation and orientation fields.
 * @param A_p_B A position vector of B_origin from A_origin, expressed in A.
 * @param A_q_B A quaternion representation of the rotation.
 * matrix A_R_B.
 * @return The transformation matrix A_T_B.
 */
inline glm::mat4 TransformFromVecAndQuat(const glm::vec3& A_p_B, const glm::quat& A_q_B) {
  return glm::translate(glm::mat4(1.0f), A_p_B) * glm::mat4_cast(A_q_B);
}

/**
 * @brief Convert (re-express, or rotate) a vector from the Tango ADF (or start-
 * of-service) frame convention [right, forward, up] to the typical OpenGl world
 * frame convention [right, up, backward]. Note this assumes the two frames are
 * coincident, and it doesn't know about any additional offsets between a
 * particular OpenGl scene and the Tango service frames.
 * @param tango_vec A vector expressed in the Tango ADF frame convention.
 * @return The same vector expressed using the Opengl frame convention.
 */
inline glm::vec3 Vec3TangoToGl(const glm::vec3& tango_vec) {
  return glm::vec3(tango_vec.x, tango_vec.z, -tango_vec.y);
}

/**
 * @brief Convert (re-express, or rotate) a vector from the typical OpenGl world
 * frame convention [right, up, backward] to the Tango ADF (or start-of-service)
 * frame convention [right, forward, up]. Note this assumes the two frames are
 * coincident, and it doesn't know about any additional offsets between a
 * particular OpenGl scene and the Tango service frames.
 * @param gl_vec A vector expressed in the Opengl world frame convention.
 * @return The same vector expressed using the Tango ADF frame convention.
 */
inline glm::vec3 Vec3GlToTango(const glm::vec3& gl_vec) {
  return glm::vec3(gl_vec.x, -gl_vec.z, gl_vec.y);
}

/**
 * @brief Given a quaternion representing the rotation matrix tango_R_any,
 * returns the quaternion representing gl_R_any, where "any" is an arbitrary
 * frame. Note the gl base frame is rotated by 90-degrees about +X from the
 * Tango ADF/start-of-service frame, so this is equivalent to applying such a
 * rotation to the quaternion.
 * @param tango_q_any A quaternion representing rotation matrix tango_R_any.
 * @return The quaternion representing gl_R_any.
 */
glm::quat QuatTangoToGl(const glm::quat& tango_q_any);

/**
 * Get the fixed transformation matrix relating the opengl frame convention
 * (with Y-up, X-right) and the tango frame convention for the start-of-service
 * and ADF frames (with Z-up, X-right), termed "world" here.
 */
glm::mat4 opengl_world_T_tango_world();

/**
 * Get the fixed transformation matrix relating the frame convention of the
 * device's color camera frame (with Z-forward, X-right) and the opengl camera
 * frame (with Z-backward, X-right).
 */
glm::mat4 color_camera_T_opengl_camera();

/**
 * Get the fixed transformation matrix relating the frame convention of the
 * device's depth camera frame (with Z-forward, X-right) and the opengl camera
 * frame (with Z-backward, X-right).
 */
glm::mat4 depth_camera_T_opengl_camera();

}  // namespace conversions
}  // namespace tango_gl
#endif  // TANGO_GL_GL_TANGO_CONVERSIONS_H_
