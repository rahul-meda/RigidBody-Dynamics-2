
#pragma once

#include <glm/glm.hpp>

class Collider;

// true if point is inside
bool QueryPoint(Collider* collider, const glm::vec3& point);