
#pragma once

#include<glm/glm.hpp>

struct HalfSpace
{
	glm::vec3 normal;
	float distance;

	HalfSpace();
	HalfSpace(const glm::vec3& normal, const float distance);
	HalfSpace(const glm::vec3& normal, const glm::vec3& point);

	glm::vec3 Origin() const;
	float Distance(const glm::vec3& point) const;
	glm::vec3 Projection(const glm::vec3& point) const;

	bool Infront(const glm::vec3& point) const;
	bool Behind(const glm::vec3& point) const;
	bool On(const glm::vec3& point) const;
};

// Compute a consistent orthonormal basis with the given vector
void ComputeBasis(const glm::vec3& i, glm::vec3& j, glm::vec3& k);