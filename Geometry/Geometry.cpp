
#include "Geometry.h"

#define EPSILON 0.005f

HalfSpace::HalfSpace()
{}

HalfSpace::HalfSpace(const glm::vec3& normal, float distance)
	: normal(normal), distance(distance)
{}

HalfSpace::HalfSpace(const glm::vec3& normal, const glm::vec3& point)
	: normal(normal), distance(glm::dot(point, normal))
{}

glm::vec3 HalfSpace::Origin() const
{
	return distance * normal;
}

float HalfSpace::Distance(const glm::vec3& point) const
{
	return glm::dot(point, normal) - distance;
}

glm::vec3 HalfSpace::Projection(const glm::vec3& point) const
{
	return point - Distance(point) * normal;
}

bool HalfSpace::Infront(const glm::vec3& point) const
{
	return Distance(point) > 0.0f;
}

bool HalfSpace::Behind(const glm::vec3& point) const
{
	return Distance(point) < 0.0f;
}

bool HalfSpace::On(const glm::vec3& point) const
{
	float dist = Distance(point);

	return (dist < EPSILON && dist > -EPSILON);
}

// Compute a consistent orthonormal basis with the given vector
void ComputeBasis(const glm::vec3& i, glm::vec3& j, glm::vec3& k)
{
	// Ref: http://box2d.org/2014/02/computing-a-basis/
	// At least one component of a unit vector should be >= 0.57735f
	if (std::abs(i.x) >= 0.57735f)
		j = glm::vec3(i.y, -j.x, 0);
	else
		j = glm::vec3(0, i.z, -i.y);

	j = glm::normalize(j);
	k = glm::cross(i, j);
	k = glm::normalize(k);
}