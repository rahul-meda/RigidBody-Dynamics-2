#pragma once

#include <glm/glm.hpp>

struct Jacobian;
class Body;

// For picking objects using mouse
// Drives a point on the body to the mouse position
// Removes 3 degrees of freedom - 3 constraints required
// The 2 anchor points for this joint are the picked point on the body
// and the mouse point
// The 2 points are constrained to move together, with no restriction on rotation
// Similar to revolute joint, but mouse is considered static (zero velocity, zero mass)
class MouseJoint
{
private:
	// The picked body
	Body* A;

	// The anchor point on A in A's space
	glm::vec3 localAnchorA;

	// The mouse point in world space
	glm::vec3 wMouse;

	glm::vec3 rA;

	// Calculates the Jacobian for this joint
	// @param axis - movement restricted along this world axis
	// @param anchorA, anchorB - world position of anchors relative to each body
	void CalculateJacobian(Jacobian& J, const glm::vec3& axis);

	// Bias is the "work" term in the velocity constraint equation
	// Accounts for position drift
	float CalculateBias(float correction);

public:
	MouseJoint() {};

	MouseJoint(Body* A, const glm::vec3& anchor);

	glm::vec3 GetMouseAnchor() const;
	void SetMouseAnchor(const glm::vec3& anchor);

	// Solves this contact by applying impulses
	void Solve();
};