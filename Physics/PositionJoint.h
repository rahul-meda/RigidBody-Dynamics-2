
#pragma once

#include <glm/glm.hpp>

// Position Joint simulates "ball in a socket"
// The anchor point is in the world space
// The 2 local points relative to each body are constrained to move together
// Restricts 3 linear degrees of freedom. No restriction on rotation

struct Jacobian;
class Body;

class PositionJoint
{
private:
	// The bodies invovled in this constraint
	Body* A;

	Body* B;

	// The anchor point on A in A's space
	glm::vec3 localAnchorA;

	// The anchor point on B in B's space
	glm::vec3 localAnchorB;

	glm::vec3 rA;
	glm::vec3 rB;

	float lambdaSum;

	// Calculates the Jacobian for this joint
	// @param axis - movement restricted along this world axis
	// @param anchorA, anchorB - world position of anchors relative to each body
	void CalculateJacobian(Jacobian& J, const glm::vec3& axis);

	// Bias is the "work" term in the velocity constraint equation
	// Accounts for position drift
	float CalculateBias(float correction);

public:
	PositionJoint(Body* A, Body* B, const glm::vec3& anchor);

	glm::vec3 GetReactionForce() const;

	glm::vec3 GetAnchorA() const;
	glm::vec3 GetAnchorB() const;

	Body* GetBodyA() const;
	Body* GetBodyB() const;

	// Solves this contact by applying impulses
	void Solve();
};