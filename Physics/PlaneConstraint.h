#pragma once

#include <glm/glm.hpp>

struct Jacobian;
class Body;

// Constrains a point on the body to move on the plane
class PlaneConstraint
{
private:
	Body* A;

	glm::vec3 localAnchor;
	glm::vec3 axis;

	void CalculateJacobian(Jacobian& J, const glm::vec3& r);

	float CalculateBias(float correction);

public:
	PlaneConstraint(Body* A, const glm::vec3& anchor, const glm::vec3& axis);

	void Solve();
};