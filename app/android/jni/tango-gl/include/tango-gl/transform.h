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

#ifndef TANGO_GL_TRANSFORM_H_
#define TANGO_GL_TRANSFORM_H_

#include "glm/glm.hpp"
#include "glm/gtc/quaternion.hpp"

namespace tango_gl {
class Transform {
 public:
  Transform();
  virtual ~Transform();

  Transform(const Transform& other) = delete;
  const Transform& operator=(const Transform& rhs) = delete;

  void SetPosition(const glm::vec3& position);
  glm::vec3 GetPosition() const;

  void SetRotation(const glm::quat& rotation);
  glm::quat GetRotation() const;

  void SetScale(const glm::vec3& scale);
  glm::vec3 GetScale() const;

  void Translate(const glm::vec3& translation);

  void SetTransformationMatrix(const glm::mat4& transform_mat);
  glm::mat4 GetTransformationMatrix() const;

  void SetParent(Transform* transform);

  const Transform* GetParent() const ;
  Transform* GetParent() ;

 private:
  Transform* parent_;

  glm::vec3 position_;
  glm::quat rotation_;
  glm::vec3 scale_;
};
}  // namespace tango_gl
#endif  // TANGO_GL_TRANSFORM_H_
