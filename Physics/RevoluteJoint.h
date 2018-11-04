
#pragma once

#include <glm/glm.hpp>

// Revolute Joint simulates "wheel and axle"
// Restricts 5 degrees of freedom, allowing only rotation about 1 axis

struct Jacobian;
class Body;
class Line;

class RevoluteJoint
{
private:
	// The bodies invovled in this constraint
	Body* A;
	Body* B;

	glm::vec3 localAnchorA;
	glm::vec3 localAnchorB;
	glm::vec3 axis;
	glm::vec3 localAxisA;
	glm::vec3 localAxisB;

	float CalculateBias(float correction);

public:
	RevoluteJoint(Body* A, Body* B,const glm::vec3& anchor, const glm::vec3& axis);

	Body* GetBodyA() const;
	Body* GetBodyB() const;

	glm::vec3 GetAnchor() const;
	glm::vec3 GetAxis() const;

	// Solves this contact by applying impulses
	void Solve();

	void SolvePositions();
};