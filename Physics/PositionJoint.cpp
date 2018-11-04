
#include "PositionJoint.h"
#include "ConstraintCommon.h"

PositionJoint::PositionJoint(Body* A, Body* B, const glm::vec3& anchor)
	: A(A), B(B), lambdaSum(0)
{
	localAnchorA = A->GlobalToLocalPoint(anchor);
	localAnchorB = B->GlobalToLocalPoint(anchor);
}

void PositionJoint::CalculateJacobian(Jacobian& J, const glm::vec3& axis)
{
	J = Jacobian(-axis, -glm::cross(rA, axis), axis, glm::cross(rB, axis));
}

float PositionJoint::CalculateBias(float correction)
{
	const static float B = 0.179f;
	const static float inv_dt = 60.0f;

	return (B * inv_dt * correction);
}

glm::vec3 PositionJoint::GetReactionForce() const
{
	glm::vec3 anchorA = A->LocalToGlobalPoint(localAnchorA);
	glm::vec3 anchorB = B->LocalToGlobalPoint(localAnchorB);
	glm::vec3 dir = anchorB - anchorA;
	float l2 = glm::length2(dir);
	if (l2 == 0)
		dir = glm::vec3(0);
	else
		dir /= std::sqrtf(l2);
	glm::vec3 reactionForce = (lambdaSum) * dir * 60.0f;
	return reactionForce;
}

glm::vec3 PositionJoint::GetAnchorA() const
{
	return A->LocalToGlobalPoint(localAnchorA);
}

glm::vec3 PositionJoint::GetAnchorB() const
{
	return B->LocalToGlobalPoint(localAnchorB);
}

Body* PositionJoint::GetBodyA() const
{
	return A;
}

Body* PositionJoint::GetBodyB() const
{
	return B;
}

void PositionJoint::Solve()
{
	glm::vec3 anchorA = A->LocalToGlobalPoint(localAnchorA);
	glm::vec3 anchorB = B->LocalToGlobalPoint(localAnchorB);
	rA = anchorA - A->GetCentroid();
	rB = anchorB - B->GetCentroid();

	Jacobian J1, J2, J3;	// 3 position constraints
	static const glm::vec3 X(1, 0, 0);
	static const glm::vec3 Y(0, 1, 0);
	static const glm::vec3 Z(0, 0, 1);
	CalculateJacobian(J1, X);
	CalculateJacobian(J2, Y);
	CalculateJacobian(J3, Z);

	float bias1 = CalculateBias(anchorB.x - anchorA.x);
	float bias2 = CalculateBias(anchorB.y - anchorA.y);
	float bias3 = CalculateBias(anchorB.z - anchorA.z);

	float effMass1 = CalculateEffectiveMass(J1, A, B);
	float effMass2 = CalculateEffectiveMass(J2, A, B);
	float effMass3 = CalculateEffectiveMass(J3, A, B);

	float lambda1 = CalculateLagrangian(J1, A, B, effMass1, bias1);
	float lambda2 = CalculateLagrangian(J2, A, B, effMass2, bias2);
	float lambda3 = CalculateLagrangian(J3, A, B, effMass3, bias3);

	lambdaSum += lambda1 + lambda2 + lambda3;

	ApplyImpulse(J1, A, B, lambda1);
	ApplyImpulse(J2, A, B, lambda2);
	ApplyImpulse(J3, A, B, lambda3);
}