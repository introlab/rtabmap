#include "tango-gl/bounding_box.h"

namespace tango_gl {
BoundingBox::BoundingBox(const std::vector<float>& vertices) {
  // Set min and max to the first vertice.
  bounding_min_ = glm::vec3(vertices[0], vertices[1], vertices[2]);
  bounding_max_ = bounding_min_;
  size_t vertices_count = vertices.size() / 3;
  for (size_t i = 1; i < vertices_count; i += 3) {
    bounding_min_.x = std::min(vertices[i * 3], bounding_min_.x);
    bounding_min_.y = std::min(vertices[i * 3 + 1], bounding_min_.y);
    bounding_min_.z = std::min(vertices[i * 3 + 2], bounding_min_.z);

    bounding_max_.x = std::max(vertices[i * 3], bounding_max_.x);
    bounding_max_.y = std::max(vertices[i * 3 + 1], bounding_max_.y);
    bounding_max_.z = std::max(vertices[i * 3 + 2], bounding_max_.z);
  }
}

bool BoundingBox::IsIntersecting(const Segment& segment,
                                 const glm::quat& rotation,
                                 const glm::mat4& transformation) {
  // The current bounding box.
  glm::vec3 min, max;

  // If the mesh has been rotated, we need to derive a new bounding box
  // based on the original one, if it just been translated or scaled,
  // we can still use the original one with current model matrix applied.
  if (rotation == glm::quat(1.0f, 0.0f, 0.0f, 0.0f)) {
    min = util::ApplyTransform(transformation, bounding_min_);
    max = util::ApplyTransform(transformation, bounding_max_);
  } else {
    std::vector<glm::vec3> box;
    // Derive 8 vertices of the new bounding box from original min and max.
    box.push_back(bounding_min_);
    box.push_back(bounding_max_);

    box.push_back(glm::vec3(bounding_min_.x, bounding_max_.y, bounding_max_.z));
    box.push_back(glm::vec3(bounding_max_.x, bounding_min_.y, bounding_min_.z));

    box.push_back(glm::vec3(bounding_min_.x, bounding_min_.y, bounding_max_.z));
    box.push_back(glm::vec3(bounding_max_.x, bounding_max_.y, bounding_min_.z));

    box.push_back(glm::vec3(bounding_max_.x, bounding_min_.y, bounding_max_.z));
    box.push_back(glm::vec3(bounding_min_.x, bounding_max_.y, bounding_min_.z));

    min = util::ApplyTransform(transformation, bounding_min_);
    max = min;
    for (size_t i = 1; i < box.size(); i++) {
      glm::vec3 temp = util::ApplyTransform(transformation, box[i]);
      min.x = std::min(temp.x, min.x);
      min.y = std::min(temp.y, min.y);
      min.z = std::min(temp.z, min.z);

      max.x = std::max(temp.x, max.x);
      max.y = std::max(temp.y, max.y);
      max.z = std::max(temp.z, max.z);
    }
  }
  return util::SegmentAABBIntersect(min, max, segment.start, segment.end);
}
}  // namespace tango_gl
