
#include "RevoluteJoint.h"
#include "ConstraintCommon.h"
#include "Geometry.h"

RevoluteJoint::RevoluteJoint(Body* A, Body* B, const glm::vec3& anchor, const glm::vec3& axis)
	: A(A), B(B), axis(axis)
{
	localAnchorA = A->GlobalToLocalPoint(anchor);
	localAnchorB = B->GlobalToLocalPoint(anchor);
	localAxisA = A->GlobalToLocalVec(axis);
	localAxisB = B->GlobalToLocalVec(axis);
}

float RevoluteJoint::CalculateBias(float correction)
{
	const static float B = 0.1f;//0.179f;
	const static float inv_dt = 60.0f;

	return (B * inv_dt * correction);
}

Body* RevoluteJoint::GetBodyA() const
{
	return A;
}

Body* RevoluteJoint::GetBodyB() const
{
	return B;
}

glm::vec3 RevoluteJoint::GetAnchor() const
{
	return A->LocalToGlobalPoint(localAnchorA);
}

glm::vec3 RevoluteJoint::GetAxis() const
{
	return axis;
}

void RevoluteJoint::Solve()
{
	glm::vec3 anchorA = A->LocalToGlobalPoint(localAnchorA);
	glm::vec3 anchorB = B->LocalToGlobalPoint(localAnchorB);
	glm::vec3 rA = anchorA - A->GetCentroid();
	glm::vec3 rB = anchorB - B->GetCentroid();

	// 3 position constraints
	static const glm::vec3 X(1, 0, 0);
	static const glm::vec3 Y(0, 1, 0);
	static const glm::vec3 Z(0, 0, 1);
	Jacobian J1(-X, -glm::cross(rA, X), X, glm::cross(rB, X));
	Jacobian J2(-Y, -glm::cross(rA, Y), Y, glm::cross(rB, Y));
	Jacobian J3(-Z, -glm::cross(rA, Z), Z, glm::cross(rB, Z));

	float bias1 = CalculateBias(anchorB.x - anchorA.x);
	float bias2 = CalculateBias(anchorB.y - anchorA.y);
	float bias3 = CalculateBias(anchorB.z - anchorA.z);

	float effMass1 = CalculateEffectiveMass(J1, A, B);
	float effMass2 = CalculateEffectiveMass(J2, A, B);
	float effMass3 = CalculateEffectiveMass(J3, A, B);

	float lambda1 = CalculateLagrangian(J1, A, B, effMass1, bias1);
	float lambda2 = CalculateLagrangian(J2, A, B, effMass2, bias2);
	float lambda3 = CalculateLagrangian(J3, A, B, effMass3, bias3);

	// 2 angular constraints;
	glm::vec3 t1, t2;	// locked axes
	ComputeBasis(axis, t1, t2);
	Jacobian J4(glm::vec3(0), -t1, glm::vec3(0), t1);
	Jacobian J5(glm::vec3(0), -t2, glm::vec3(0), t2);
	
	glm::vec3 wAxisA = A->LocalToGlobalVec(localAxisA);
	glm::vec3 wAxisB = B->LocalToGlobalVec(localAxisB);
	glm::vec3 correction = glm::cross(wAxisA, wAxisB);
	float bias4 = CalculateBias(glm::dot(correction, t1));
	float bias5 = CalculateBias(glm::dot(correction, t2));

	float effMass4 = CalculateEffectiveMass(J4, A, B);
	float effMass5 = CalculateEffectiveMass(J5, A, B);

	float lambda4 = CalculateLagrangian(J4, A, B, effMass4, bias4);
	float lambda5 = CalculateLagrangian(J5, A, B, effMass5, bias5);

	// To Do : applying impulses immediately might invalidate jacobians for future iterations
	// instead save the velocities to be applied, and apply them finally after all constraints are solved?
	ApplyImpulse(J1, A, B, lambda1);
	ApplyImpulse(J2, A, B, lambda2);
	ApplyImpulse(J3, A, B, lambda3);
	ApplyImpulse(J4, A, B, lambda4);
	ApplyImpulse(J5, A, B, lambda5);
}

void RevoluteJoint::SolvePositions()
{

}