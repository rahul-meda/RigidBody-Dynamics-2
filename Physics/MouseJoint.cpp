
#include "MouseJoint.h"
#include "ConstraintCommon.h"

MouseJoint::MouseJoint(Body* A, const glm::vec3& anchor)
	: A(A)
{
	localAnchorA = A->GlobalToLocalPoint(anchor);
}

glm::vec3 MouseJoint::GetMouseAnchor() const
{
	return wMouse;
}

void MouseJoint::SetMouseAnchor(const glm::vec3& anchor)
{
	wMouse = anchor;
}

void MouseJoint::CalculateJacobian(Jacobian& J, const glm::vec3& axis)
{
	J = Jacobian(-axis, -glm::cross(rA, axis), glm::vec3(0.0f), glm::vec3(0.0f));
}

float MouseJoint::CalculateBias(float correction)
{
	const static float B = 0.179f;
	const static float inv_dt = 60.0f;

	if (std::abs(correction > 5.0f))
		int x = 1;

	return (B * inv_dt * correction);
}

void MouseJoint::Solve()
{
	glm::vec3 anchorA = A->LocalToGlobalPoint(localAnchorA);
	rA = anchorA - A->GetCentroid();

	Jacobian J1, J2, J3;	// 3 position constraints
	static const glm::vec3 X(1, 0, 0);
	static const glm::vec3 Y(0, 1, 0);
	static const glm::vec3 Z(0, 0, 1);
	CalculateJacobian(J1, X);
	CalculateJacobian(J2, Y);
	CalculateJacobian(J3, Z);

	float bias1 = CalculateBias(wMouse.x - anchorA.x);
	float bias2 = CalculateBias(wMouse.y - anchorA.y);
	float bias3 = CalculateBias(wMouse.z - anchorA.z);

	float effMass1 = glm::dot(J1.L1, J1.L1) * A->GetInvMass() + glm::dot(J1.A1, J1.A1 * A->GetInvInertia());
	float effMass2 = glm::dot(J2.L1, J2.L1) * A->GetInvMass() + glm::dot(J2.A1, J2.A1 * A->GetInvInertia());
	float effMass3 = glm::dot(J3.L1, J3.L1) * A->GetInvMass() + glm::dot(J3.A1, J3.A1 * A->GetInvInertia());

	float JV1 = glm::dot(J1.L1, A->GetVelocity()) + glm::dot(J1.A1, A->GetAngularVelocity());
	float lambda1 = (-(JV1 + bias1) / effMass1);
	float JV2 = glm::dot(J2.L1, A->GetVelocity()) + glm::dot(J2.A1, A->GetAngularVelocity());
	float lambda2 = (-(JV2 + bias2) / effMass2);
	float JV3 = glm::dot(J3.L1, A->GetVelocity()) + glm::dot(J3.A1, A->GetAngularVelocity());
	float lambda3 = (-(JV3 + bias3) / effMass3);

	glm::vec3 v = A->GetVelocity();
	glm::vec3 w = A->GetAngularVelocity();
	v += lambda1 * J1.L1 * A->GetInvMass() + lambda2 * J2.L1 * A->GetInvMass() + lambda3 * J3.L1 * A->GetInvMass();
	w += lambda1 * J1.A1 * A->GetInvInertia() + lambda2 * J2.A1 * A->GetInvInertia() + lambda3 * J3.A1 * A->GetInvInertia();
	A->SetVelocity(v);
	A->SetAngularVelocity(w);
}